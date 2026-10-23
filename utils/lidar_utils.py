"""LiDAR utility functions and state management"""

import asyncio
import json
import math
import random
import time
import threading
import signal
import atexit

import bisect

from collections import deque

from config import (
    LIDAR_PORT,
    LIDAR_PORT_CANDIDATES,
    LIDAR_BAUD,
    LIDAR_SECTORS,
    LIDAR_SECTOR_ORDER,
    LIDAR_ANGLE_OFFSET,
    LIDAR_RPM,
    LIDAR_MAX_RPM,
    LIDAR_MIRROR,
    GUIDANCE_CONE_DEG,
    GUIDANCE_CONE_CENTRES,
    GUIDANCE_CORRIDOR_HALF_WIDTH_MM,
    GUIDANCE_CORRIDOR_MAX_MM,
    LIDAR_CLUSTER_MIN_POINTS,
    LIDAR_CLUSTER_WINDOW_MM,
    LIDAR_PERSIST_DEG,
    LIDAR_PERSIST_MAX_AGE_S,
    LIDAR_SELF_PERSIST_S,
    LIDAR_SELF_RANGE_MM,
    LIDAR_MAX_BACKLOG_S,
    LIDAR_KEEP_BACKLOG_S,
    LIDAR_MEAS_RATE_HZ,
    LIDAR_SCAN_MAX_GAP_DEG,
    LIDAR_RETRY_BACKOFF_S,
    LIDAR_SIMULATION_ALLOWED,
    HEADING_ENABLED,
    DRIFT_ENABLED,
    DRIFT_ANALYZE_HZ,
    BEV_ENABLED,
    LIDAR_MOVERS_ENABLED,
    MOTION_TRACKER_ENABLED,
)
from utils.shared_state import (
    update_lidar_state,
    update_drift_state,
    get_radar_state,
    get_camera_state,
)
from utils.heading_tracker import get_tracker
from utils import drift_guidance


def lidar_port() -> str:
    """The LiDAR's serial port NOW: the first of config.LIDAR_PORT_CANDIDATES that exists — a SHAPPE_LIDAR_PORT override, then the rig's /dev/serial/by-id name, then /dev/ttyUSB0 — resolved at every (re)connect, so a re-enumerated device is followed instead of its old node being retried forever (F13)."""
    import os

    for candidate in LIDAR_PORT_CANDIDATES:
        if os.path.exists(candidate):
            return candidate
    return LIDAR_PORT


def correct_angle(raw_angle: float) -> float:
    """Apply mirroring (left<->right) and the configured offset to a raw angle."""
    if LIDAR_MIRROR:
        return (LIDAR_ANGLE_OFFSET - raw_angle) % 360.0
    return (raw_angle + LIDAR_ANGLE_OFFSET) % 360.0


# ============== GLOBAL STATE ==============
clients = set()
stop_scanner = threading.Event()
lidar_instance = None
lidar_lock = threading.Lock()

MIN_SCAN_INTERVAL = (
    0.066  # cap ~15 Hz (matches 900 RPM motor; bounds broadcast/UI load)
)

_pending_scan = None
_scan_cv = threading.Condition()

_reader_health = {
    "backlog_meas": 0,
    "dropped_meas": 0,
    "resyncs": 0,
    "partial_scans": 0,
    "meas_rate_hz": None,
    "source": "starting",
    "requested_rpm": LIDAR_RPM,
    "pwm_command": None,
    "points_per_scan": None,
    "packets_per_scan": None,
    "connected": False,
}


def reader_health():
    """A copy of the reader's health counters (GET /lidar/reader)."""
    return dict(_reader_health)


_previous_scan = None

# ============== MOTOR SPEED CONTROL ==============
MAX_MOTOR_PWM = 1023  # RPLIDAR PWM range is 0..1023
MOTOR_REASSERT_INTERVAL = 1.0  # seconds; re-send PWM to resist transient resets


def rpm_to_pwm(rpm: float) -> int:
    """Map a target RPM onto the 0..MAX_MOTOR_PWM PWM range (clamped)."""
    pwm = round(rpm / LIDAR_MAX_RPM * MAX_MOTOR_PWM)
    return max(0, min(MAX_MOTOR_PWM, pwm))


def _open_without_motor(RPLidar, port):
    """RPLidar(port) WITHOUT the motor kick its constructor gives."""

    class _NoConstructorKick(RPLidar):
        _opened = False

        def start_motor(self):
            if self._opened:
                return super().start_motor()
            self.motor_running = False  # left disabled until the ramp

    lidar = _NoConstructorKick(port, baudrate=LIDAR_BAUD, timeout=3)
    lidar._opened = True
    return lidar


def set_motor_speed(lidar, rpm: float = LIDAR_RPM) -> int:
    """Start the motor and soft-start it up to the requested RPM."""
    target_pwm = rpm_to_pwm(rpm)
    lidar._serial_port.setDTR(False)
    lidar.motor_running = True

    ramp_steps = 10
    start_pwm = min(150, target_pwm)
    for step in range(ramp_steps + 1):
        pwm = round(start_pwm + (target_pwm - start_pwm) * step / ramp_steps)
        try:
            lidar.set_pwm(max(0, min(MAX_MOTOR_PWM, pwm)))
        except Exception as e:
            print(f"[WARN] set_pwm failed during ramp: {e}")
            break
        time.sleep(0.1)

    try:
        lidar.set_pwm(target_pwm)
    except Exception:
        pass
    print(f"[OK] LiDAR motor at {rpm:.0f} RPM (PWM {target_pwm}/{MAX_MOTOR_PWM})")
    return target_pwm


def cleanup_lidar():
    """Stop LiDAR motor and disconnect."""
    global lidar_instance
    with lidar_lock:
        lidar, lidar_instance = lidar_instance, None
    if lidar is None:
        return
    errors = []
    for step in (lidar.stop, lidar.stop_motor, lidar.disconnect):
        try:
            step()
        except Exception as e:
            errors.append(f"[WARN] Error during LiDAR cleanup ({step.__name__}): {e}")
    for line in errors + ["[OK] LiDAR motor stopped and disconnected"]:
        try:
            print(line)
        except Exception:
            pass


_previous_handlers = {}


def setup_signal_handlers():
    """Setup signal handlers for graceful shutdown"""

    def say(line):
        try:
            print(line)
        except Exception:
            pass

    def signal_handler(signum, frame):
        say(f"\n[INFO] Received signal {signum}, shutting down...")

        try:
            from utils import service

            service.arm_shutdown_backstop()
        except Exception as exc:
            say(
                f"[WARN] could not arm the shutdown backstop: "
                f"{type(exc).__name__}: {exc}"
            )

        try:
            from utils.pi_camera_utils import begin_shutdown

            begin_shutdown()
        except Exception as exc:
            say(f"[WARN] could not stop camera streams: {type(exc).__name__}: {exc}")

        stop_scanner.set()
        try:
            cleanup_lidar()
        except Exception as exc:  # never before the hand-over below
            say(f"[WARN] LiDAR cleanup failed: {type(exc).__name__}: {exc}")
        previous = _previous_handlers.get(signum)
        if callable(previous):
            previous(signum, frame)
        else:
            raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM):
        _previous_handlers[sig] = signal.getsignal(sig)
        signal.signal(sig, signal_handler)
    atexit.register(cleanup_lidar)


def get_sector(angle: float) -> str:
    """Map an angle (already offset/mirror-corrected) to one of the 8 x 45° sectors."""
    idx = int(((angle % 360.0) + 22.5) % 360.0 // 45.0)
    return LIDAR_SECTOR_ORDER[idx]


def analyze_sectors(points, previous=None):
    """Analyze points and return per-sector statistics."""
    buckets = {sector: [] for sector in LIDAR_SECTORS.keys()}
    for angle, dist in points:
        if dist <= 0:
            continue
        sector = get_sector(angle)
        if sector in buckets:
            buckets[sector].append((dist, angle))

    prev = _persistence_index(previous)
    return {name: _cluster_min(pts, previous=prev) for name, pts in buckets.items()}


def _persistence_index(previous):
    """The previous scan's returns sorted by range, for _cluster_min: (ranges, [(dist, angle)])."""
    if not previous:
        return None
    pairs = sorted((float(d), float(a)) for a, d in previous if d and d > 0)
    return [d for d, _ in pairs], pairs


def _cluster_min(pts, support=None, previous=None):
    """Nearest CONFIRMED return from a list of (dist, angle) points."""
    min_dist, min_angle = None, None
    if pts:
        pts.sort()  # by distance ascending (then angle)
        dists = sorted(d for d, _ in (pts if support is None else support))
        for d, a in pts:
            lo = bisect.bisect_left(dists, d - LIDAR_CLUSTER_WINDOW_MM)
            hi = bisect.bisect_right(dists, d + LIDAR_CLUSTER_WINDOW_MM)
            if (hi - lo) >= LIDAR_CLUSTER_MIN_POINTS or _persists(d, a, previous):
                min_dist, min_angle = d, a
                break
    return {"min_dist": min_dist, "min_angle": min_angle, "count": len(pts)}


def _persists(dist, angle, previous) -> bool:
    """Did the previous scan have a return where this one is?"""
    if not previous:
        return False
    ranges, pairs = previous
    lo = bisect.bisect_left(ranges, dist - LIDAR_CLUSTER_WINDOW_MM)
    hi = bisect.bisect_right(ranges, dist + LIDAR_CLUSTER_WINDOW_MM)
    return any(
        _angular_gap(pairs[i][1], angle) <= LIDAR_PERSIST_DEG for i in range(lo, hi)
    )


def _angular_gap(a: float, centre: float) -> float:
    """Smallest absolute angular difference between two bearings (degrees)."""
    return abs((a - centre + 180.0) % 360.0 - 180.0)


def analyze_cones(points, previous=None):
    """Narrow STEERING cones — one per cardinal, GUIDANCE_CONE_DEG wide."""
    half = GUIDANCE_CONE_DEG / 2.0
    buckets = {name: [] for name in GUIDANCE_CONE_CENTRES}
    line = []
    support = {name: [] for name in GUIDANCE_CONE_CENTRES}
    for angle, dist in points:
        if dist <= 0:
            continue
        for name, centre in GUIDANCE_CONE_CENTRES.items():
            gap = _angular_gap(angle, centre)
            in_cone = gap <= half
            in_corridor = (
                name == "front"
                and GUIDANCE_CORRIDOR_HALF_WIDTH_MM > 0
                and gap < 90.0
                and dist <= GUIDANCE_CORRIDOR_MAX_MM
                and abs(dist * math.sin(math.radians(gap)))
                <= GUIDANCE_CORRIDOR_HALF_WIDTH_MM
            )
            if in_cone or in_corridor:
                buckets[name].append((dist, angle))
            if in_cone or in_corridor or gap <= 2.0 * half:
                support[name].append((dist, angle))
            if name == "front" and in_cone:
                line.append((dist, angle))
    prev = _persistence_index(previous)
    out = {
        name: _cluster_min(pts, support[name], prev) for name, pts in buckets.items()
    }
    out["front_line"] = _cluster_min(line, support["front"], prev)
    return out


async def broadcast(msg: str):
    """Broadcast message to all connected WebSocket clients"""
    if not clients:
        return
    targets = list(clients)
    results = await asyncio.gather(
        *[asyncio.wait_for(ws.send_text(msg), timeout=0.5) for ws in targets],
        return_exceptions=True,
    )
    dead = [ws for ws, r in zip(targets, results) if isinstance(r, Exception)]
    for ws in dead:
        clients.discard(ws)
    if dead:
        await asyncio.gather(*[_close_quietly(ws) for ws in dead])


async def _close_quietly(ws):
    """Close a client we have given up on. A close that fails is already closed."""
    try:
        await asyncio.wait_for(ws.close(code=1011), timeout=0.5)
    except Exception:
        pass


def add_client(websocket):
    """Add a WebSocket client"""
    clients.add(websocket)
    print(f"[INFO] Client connected. Total: {len(clients)}")


def remove_client(websocket):
    """Remove a WebSocket client"""
    clients.discard(websocket)
    print(f"[INFO] Client disconnected. Total: {len(clients)}")


_MEAS_LEN = 5  # bytes per measurement in the standard scan response
_READ_CHUNK = 65536  # cap on a single bulk read
_SYNC_RUN = 4  # consecutive plausible packets required to re-lock

_RATE_WARMUP_S = 2.0

_SILENCE_TIMEOUT_S = 2.0
_NO_SCAN_TIMEOUT_S = 5.0  # bytes arriving but no complete scan this long: reconnect
_MAX_PACKETS_PER_SCAN = 10000  # ~2.5 s of packets: far more than one revolution holds


def _plausible(buf, i: int) -> bool:
    """True if buf[i:i+5] could be the start of a measurement packet."""
    if i + _MEAS_LEN > len(buf):
        return False
    b0 = buf[i]
    if (b0 & 0b1) == ((b0 >> 1) & 0b1):
        return False
    b1 = buf[i + 1]
    if not b1 & 0b1:
        return False
    return ((b1 >> 1) | (buf[i + 2] << 7)) < 360 * 64


def _find_phase(buf, start: int) -> int:
    """Index of the next byte where _SYNC_RUN plausible packets begin in a row."""
    limit = len(buf) - _SYNC_RUN * _MEAS_LEN
    i = start
    while i <= limit:
        if _in_phase(buf, i):
            return i
        i += 1
    return -1


def _in_phase(buf, i: int) -> bool:
    """True if _SYNC_RUN plausible packets begin at buf[i]."""
    return all(_plausible(buf, i + k * _MEAS_LEN) for k in range(_SYNC_RUN))


class _ScanStream:
    """Assembles scans from the LiDAR's serial port with bulk reads."""

    def __init__(self, lidar):
        self._lidar = lidar
        self._port = lidar._serial_port
        self._buf = bytearray()
        self._pos = 0  # read cursor; the buffer is compacted on refill
        self._locked = False  # byte phase known good
        self._synced = False  # sitting on a scan boundary
        self._last_data = 0.0  # when bytes last arrived — silence detection
        self.dropped_meas = 0  # cumulative — reported as reader health
        self.resyncs = 0
        self.partial_scans = 0  # fragments discarded by the coverage check
        self.backlog_meas = 0  # measurements still waiting at the last refill
        self.captured_at = None  # when the device finished the last yielded scan
        self.swept = []
        self._rx_bytes = 0  # bytes received since the first arrived...
        self._rx_since = 0.0  # ...at this time — the measured rate

    def start(self):
        """Put the device into continuous scan mode."""
        from rplidar import SCAN_BYTE, SCAN_TYPE, RPLidarException

        status, error_code = self._lidar.get_health()
        if status == "Error":
            raise RPLidarException(
                f"RPLidar hardware failure. Error code: {error_code}"
            )
        if status == "Warning":
            print(f"[WARN] LiDAR health warning, error code {error_code}")

        self._lidar._send_cmd(SCAN_BYTE)
        dsize, is_single, dtype = self._lidar._read_descriptor()
        if dsize != _MEAS_LEN or is_single or dtype != SCAN_TYPE:
            raise RPLidarException(
                f"Unexpected scan descriptor: size={dsize} single={is_single} "
                f"type={dtype}"
            )

    def _refill(self) -> bool:
        """Pull everything the port holds into our buffer in as few reads as possible."""
        if self._pos:
            del self._buf[: self._pos]  # one memmove per refill, not per measurement
            self._pos = 0

        chunk = self._port.read(1)
        now = time.monotonic()
        if not chunk:
            if self._last_data and now - self._last_data > _SILENCE_TIMEOUT_S:
                from rplidar import RPLidarException

                raise RPLidarException(
                    f"No data from LiDAR for {now - self._last_data:.1f}s"
                )
            self._last_data = self._last_data or now
            return False

        self._last_data = time.monotonic()
        self._rx_since = self._rx_since or self._last_data
        before = len(self._buf)
        self._buf += chunk
        waiting = self._port.in_waiting
        if waiting:
            self._buf += self._port.read(min(waiting, _READ_CHUNK))
        self._rx_bytes += len(self._buf) - before

        self.backlog_meas = len(self._buf) // _MEAS_LEN
        if self.backlog_meas > self.meas_rate() * LIDAR_MAX_BACKLOG_S:
            self._drop_stale()
        return True

    def meas_rate(self) -> float:
        """Measurements a second the device is sending, as measured here."""
        span = self._last_data - self._rx_since
        if self._rx_since and span >= _RATE_WARMUP_S:
            return max(1.0, self._rx_bytes / _MEAS_LEN / span)
        return LIDAR_MEAS_RATE_HZ

    def _drop_stale(self):
        """Discard the OLDEST buffered data, keeping the newest ~1.5 revolutions."""
        keep = int(self.meas_rate() * LIDAR_KEEP_BACKLOG_S) * _MEAS_LEN
        excess = len(self._buf) - keep
        self.dropped_meas += excess // _MEAS_LEN
        del self._buf[:excess]
        self._pos = 0
        self._locked = False
        self._synced = False

    def iter_scans(self, min_len=5):
        """Yield complete scans as lists of (quality, angle, distance)."""
        scan = []
        angles = []  # every packet's bearing, valid or not — coverage
        last_scan = time.monotonic()
        while not stop_scanner.is_set():
            if time.monotonic() - last_scan > _NO_SCAN_TIMEOUT_S:
                # Bytes still arrive (silence is _refill's), but nothing that makes a scan:
                # end the session so it reconnects (fuzz test 2026-10-22).
                from rplidar import RPLidarException

                raise RPLidarException(
                    f"LiDAR data but no complete scan for "
                    f"{time.monotonic() - last_scan:.1f}s"
                )
            if len(self._buf) - self._pos < _SYNC_RUN * _MEAS_LEN:
                if not self._refill():
                    continue
                if len(self._buf) - self._pos < _SYNC_RUN * _MEAS_LEN:
                    continue

            if not self._locked:
                i = _find_phase(self._buf, self._pos)
                if i < 0:
                    # Keep only what could still start a run and read more.
                    self._pos = max(self._pos, len(self._buf) - _SYNC_RUN * _MEAS_LEN)
                    self._refill()
                    continue
                self._pos = i
                self._locked = True
                self.resyncs += 1

            if not _in_phase(self._buf, self._pos):
                self._locked = False
                self._synced = False
                self._pos += 1
                continue

            i = self._pos
            b0 = self._buf[i]
            self._pos += _MEAS_LEN

            if b0 & 0b1:  # start-of-revolution flag
                if self._synced and len(scan) > min_len:
                    if _max_gap(angles) <= LIDAR_SCAN_MAX_GAP_DEG:
                        self.captured_at = time.time() - self._arrived_since()
                        self.swept = angles
                        last_scan = time.monotonic()
                        yield scan
                    else:
                        self.partial_scans += 1
                self._synced = True
                scan = []
                angles = []
            if not self._synced:
                continue  # mid-scan after a drop; wait it out
            if len(angles) >= _MAX_PACKETS_PER_SCAN:
                # No start flag for far longer than a revolution: a lost flag, not a scan.
                # Drop it rather than let the lists grow without end.
                self.partial_scans += 1
                self._synced = False
                scan, angles = [], []
                continue

            angle = ((self._buf[i + 1] >> 1) | (self._buf[i + 2] << 7)) / 64.0
            angles.append(angle)
            quality = b0 >> 2
            distance = (self._buf[i + 3] | (self._buf[i + 4] << 8)) / 4.0
            if quality > 0 and distance > 0:
                scan.append((quality, angle, distance))

    def _arrived_since(self) -> float:
        """Seconds of data that arrived after the packet just parsed."""
        try:
            waiting = self._port.in_waiting
        except Exception:
            waiting = 0
        unread = len(self._buf) - self._pos + waiting
        return unread / _MEAS_LEN / self.meas_rate()


def _max_gap(angles) -> float:
    """Widest angular gap, degrees, left between a revolution's packets."""
    if not angles:
        return 360.0
    ordered = sorted(angles)
    gap = ordered[0] + 360.0 - ordered[-1]
    for a, b in zip(ordered, ordered[1:]):
        if b - a > gap:
            gap = b - a
    return gap


def _mark_unswept(cones, swept):
    """Flag each steering cone that holds NO return and that no packet of this revolution swept (audit follow-up, NEW-044)."""
    if swept is None:
        return cones
    bearings = [correct_angle(a) for a in swept]
    half = GUIDANCE_CONE_DEG / 2.0
    out = dict(cones)
    for name, centre in GUIDANCE_CONE_CENTRES.items():
        cone = out.get(name)
        if not isinstance(cone, dict) or cone.get("min_dist") is not None:
            continue
        if not any(_angular_gap(a, centre) <= half for a in bearings):
            out[name] = {**cone, "unswept": True}
            if (
                name == "front"
                and isinstance(out.get("front_line"), dict)
                and out["front_line"].get("min_dist") is None
            ):
                out["front_line"] = {**out["front_line"], "unswept": True}
    return out


_SELF_BIN_DEG = 10.0
_self_since = {}  # 10-degree slice -> first scan with a near return there, while it lasts


def _drop_transient_self(points, captured_at):
    """Returns nearer than LIDAR_SELF_RANGE_MM, dropped until they have held in one slice for LIDAR_SELF_PERSIST_S: the wearer's arm or cane handle swinging through the cones raised false "Turn" warnings (2026-10-21)."""
    if LIDAR_SELF_RANGE_MM <= 0:
        return points
    near = {
        int((a % 360.0) // _SELF_BIN_DEG) for a, d in points if d < LIDAR_SELF_RANGE_MM
    }
    for b in list(_self_since):
        if b not in near or captured_at < _self_since[b]:  # gone, or a clock step
            del _self_since[b]
    for b in near:
        _self_since.setdefault(b, captured_at)
    held = {
        b for b in near if captured_at - _self_since[b] >= LIDAR_SELF_PERSIST_S - 1e-6
    }
    if held == near:
        return points
    return [
        (a, d)
        for a, d in points
        if d >= LIDAR_SELF_RANGE_MM or int((a % 360.0) // _SELF_BIN_DEG) in held
    ]


def _publish_scan(loop, scan, scan_index, hz, health, captured_at=None, swept=None):
    """Turn one raw scan into sector/cone stats and push it to state + clients."""
    global _previous_scan
    captured_at = time.time() if captured_at is None else captured_at
    points = [(correct_angle(angle), dist) for _q, angle, dist in scan if dist > 0]
    if not points:
        return
    valid_fraction = round(len(points) / len(swept), 3) if swept else None
    points = _drop_transient_self(points, captured_at)
    if not points:
        return

    previous = None
    if _previous_scan is not None:
        age = captured_at - _previous_scan[1]
        if 0 < age <= LIDAR_PERSIST_MAX_AGE_S:
            previous = _previous_scan[0]
    _previous_scan = (points, captured_at)

    sector_stats = analyze_sectors(points, previous)
    cone_stats = _mark_unswept(analyze_cones(points, previous), swept)
    points_data = {
        "angles": [round(a, 1) for a, d in points],
        "distances": [int(d) for a, d in points],
    }

    update_lidar_state(
        points_data,
        sector_stats,
        hz * 60.0,
        scan_index,
        cone_stats,
        captured_at=captured_at,
        valid_fraction=valid_fraction,
    )

    heading = _step_heading(points, captured_at)
    _update_movers(points, heading, captured_at)
    _update_drift(heading)
    _update_bev(points, heading, captured_at)

    if clients:
        msg_data = {
            "scan_index": scan_index,
            "rpm": round(hz * 60.0, 1),
            "hz": round(hz, 1),
            "points": points_data,
            "sectors": sector_stats,
            "total_points": len(points),
            "reader": health,
        }
        asyncio.run_coroutine_threadsafe(broadcast(json.dumps(msg_data)), loop)


_last_drift_analysis = float("-inf")  # monotonic


def _step_heading(points, captured_at, trusted=True):
    """Advance the LiDAR odometry by one scan."""
    if not HEADING_ENABLED:
        return None
    try:
        return get_tracker().update(points, captured_at, trusted=trusted)
    except Exception as e:
        print(f"[WARN] heading tracker failed: {e}")
        return None


def _update_movers(points, heading, captured_at):
    """Follow the moving things in this scan (utils/lidar_movers, then utils/motion_tracker)."""
    ego = None
    if LIDAR_MOVERS_ENABLED:
        try:
            from utils.lidar_movers import get_movers

            movers = get_movers()
            movers.update(points, captured_at, heading)
            ego = movers.snapshot().get("ego")
        except Exception as e:
            print(f"[WARN] LiDAR movers failed: {e}")
    if MOTION_TRACKER_ENABLED:
        try:
            from utils.motion_tracker import get_motion_tracker

            get_motion_tracker().update(
                points, captured_at, heading, camera_state=get_camera_state(), ego=ego
            )
        except Exception as e:
            print(f"[WARN] motion tracker failed: {e}")


def _update_drift(heading):
    """Refresh the drift assessment from the heading the tracker just produced."""
    global _last_drift_analysis
    if heading is None or not DRIFT_ENABLED:
        return
    try:
        mono = time.monotonic()
        if (mono - _last_drift_analysis) < (1.0 / max(1e-3, DRIFT_ANALYZE_HZ)):
            return
        _last_drift_analysis = mono
        now = time.time()
        update_drift_state(drift_guidance.analyze(heading=heading, now=now))
    except Exception as e:
        print(f"[WARN] drift guidance failed: {e}")


def _update_bev(points, heading, captured_at=None):
    """Fold this scan into the BEV occupancy grid and re-extract its shapes."""
    if not BEV_ENABLED:
        return
    try:
        from utils.bev_grid import get_grid

        grid = get_grid()
        # Same clock as the tracker, so the grid's dt and the heading's agree.
        grid.update(points, heading=heading, radar=get_radar_state(), now=captured_at)
        grid.maybe_extract(camera=get_camera_state())
    except Exception as e:
        print(f"[WARN] BEV grid failed: {e}")


def _analysis_worker(loop):
    """Drains the single-slot mailbox filled by the reader thread."""
    global _pending_scan
    last = float("-inf")
    while not stop_scanner.is_set():
        wait = MIN_SCAN_INTERVAL - (time.monotonic() - last)
        if wait > 0 and stop_scanner.wait(wait):
            return
        with _scan_cv:
            while _pending_scan is None and not stop_scanner.is_set():
                _scan_cv.wait(timeout=0.5)
            if stop_scanner.is_set():
                return
            job, _pending_scan = _pending_scan, None
        last = time.monotonic()
        try:
            _publish_scan(loop, *job)
        except Exception as e:
            print(f"[WARN] LiDAR analysis error: {e}")


def _submit_scan(scan, scan_index, hz, health, captured_at, swept=None):
    """Hand a scan to the analysis thread without ever blocking the reader."""
    global _pending_scan
    with _scan_cv:
        _pending_scan = (scan, scan_index, hz, health, captured_at, swept)
        _scan_cv.notify()


def scanner_thread(loop):
    """LiDAR scanner thread: one device session at a time, reconnecting on faults."""
    try:
        from rplidar import RPLidar
    except ImportError as exc:
        if not LIDAR_SIMULATION_ALLOWED:
            _reader_health["source"] = f"unavailable: rplidar not importable ({exc})"
            print(
                f"[ERROR] rplidar is not importable ({exc}) — NO LiDAR. Publishing "
                f"nothing; health reports it down. (Development simulation "
                f"needs SHAPPE_SIMULATE_LIDAR=1.)"
            )
            return
        print(
            "[WARN] rplidar not installed and SHAPPE_SIMULATE_LIDAR=1: SIMULATED "
            "LiDAR — fabricated scans, flagged, never evidence of a clear path"
        )
        _reader_health["source"] = "simulated"
        run_simulation(loop)
        return
    _reader_health["source"] = "hardware"

    analysis = threading.Thread(
        target=_analysis_worker, args=(loop,), name="lidar-analysis", daemon=True
    )
    analysis.start()

    progress = {"scan_index": 0}
    failures = 0
    while not stop_scanner.is_set():
        before = progress["scan_index"]
        try:
            _scan_session(RPLidar, progress)
        except Exception as e:
            print(f"[ERROR] LiDAR error: {e}")
        finally:
            cleanup_lidar()
            _reader_health["pwm_command"] = None
            _reader_health["connected"] = False
        if stop_scanner.is_set():
            break
        if progress["scan_index"] > before:
            failures = 0
        delay = LIDAR_RETRY_BACKOFF_S[min(failures, len(LIDAR_RETRY_BACKOFF_S) - 1)]
        failures += 1
        print(
            f"[WARN] LiDAR offline — publishing NO scans (state goes stale, "
            f"health reports it down); reconnecting in {delay:g}s"
        )
        stop_scanner.wait(delay)


def _scan_session(RPLidar, progress):
    """Connect, spin up and stream until the device fails or we are stopped."""
    global lidar_instance

    scan_times = deque(maxlen=30)

    port = lidar_port()
    print(f"[INFO] Connecting to LiDAR on {port}...")

    with lidar_lock:
        lidar_instance = _open_without_motor(RPLidar, port)
        lidar = lidar_instance

    info = None
    for attempt in range(3):
        if stop_scanner.is_set():
            return
        try:
            lidar.stop()
            lidar.stop_motor()
            time.sleep(0.1)
            lidar.clear_input()
            time.sleep(0.1)
            info = lidar.get_info()
            print(f"[OK] LiDAR Info: {info}")
            break
        except Exception as e:
            print(f"[WARN] LiDAR info attempt {attempt + 1}/3: {e}; resetting…")
            try:
                lidar.reset()  # full device reset -> back to idle
                time.sleep(1.5)
                lidar.clear_input()
            except Exception:
                pass
    if info is None:
        print(
            "[WARN] No LiDAR info after 3 tries — continuing; "
            "scanning may still work once the buffer settles."
        )

    # Start motor at the configured speed (soft-started)
    target_pwm = rpm_to_pwm(LIDAR_RPM)
    ramp_ok = False
    try:
        target_pwm = set_motor_speed(lidar, LIDAR_RPM)
        ramp_ok = True
        time.sleep(2)
    except Exception as e:
        print(f"[WARN] Could not start motor: {e}")
    last_reassert = 0
    _reader_health["pwm_command"] = target_pwm if ramp_ok else None
    _reader_health["connected"] = True

    for _ in range(3):
        try:
            lidar.clear_input()
        except:
            pass
        time.sleep(0.1)
    print("[OK] Initial buffer cleared")

    stream = _ScanStream(lidar)
    stream.start()
    print(
        "[INFO] Starting LiDAR scanning (bulk serial reads, "
        f"{LIDAR_MAX_BACKLOG_S * 1000:.0f} ms latency budget)..."
    )

    last_health_log = 0.0
    reported_drops = 0

    for scan in stream.iter_scans(min_len=5):
        if stop_scanner.is_set():
            break

        current_time = time.time()  # a STAMP: compared with other stamps
        mono = time.monotonic()
        progress["scan_index"] += 1
        scan_index = progress["scan_index"]
        scan_times.append(mono)

        if mono - last_reassert >= MOTOR_REASSERT_INTERVAL:
            try:
                lidar.set_pwm(target_pwm)
                _reader_health["pwm_command"] = target_pwm
            except Exception:
                pass
            last_reassert = mono

        if len(scan_times) > 1:
            span = scan_times[-1] - scan_times[0]
            hz = (len(scan_times) - 1) / span if span > 0 else 0.0
        else:
            hz = 0.0

        _reader_health["backlog_meas"] = stream.backlog_meas
        _reader_health["dropped_meas"] = stream.dropped_meas
        _reader_health["resyncs"] = stream.resyncs
        _reader_health["partial_scans"] = stream.partial_scans
        _reader_health["meas_rate_hz"] = round(stream.meas_rate())
        _reader_health["points_per_scan"] = len(scan)
        _reader_health["packets_per_scan"] = len(stream.swept)
        _submit_scan(
            scan,
            scan_index,
            hz,
            dict(_reader_health),
            stream.captured_at or current_time,
            stream.swept,
        )

        if stream.dropped_meas > reported_drops and mono - last_health_log >= 1.0:
            print(
                f"[WARN] LiDAR reader behind: dropped "
                f"{stream.dropped_meas - reported_drops} stale measurements "
                f"(backlog {stream.backlog_meas}, {hz:.1f} Hz)"
            )
            reported_drops = stream.dropped_meas
            last_health_log = mono


def run_simulation(loop):
    """Simulated scans, for development on a machine without rplidar installed."""
    t0 = time.time()
    scan_index = 0

    print("[INFO] Running in simulation mode")
    print(
        "[INFO] Sectors: 8 x 45 deg, each centred on its direction (front = 337.5-22.5)"
    )

    while not stop_scanner.is_set():
        scan_index += 1
        points = []

        for angle in range(0, 360, 2):
            adjusted_angle = (angle + LIDAR_ANGLE_OFFSET) % 360.0

            # FRONT sector (315-360 and 0-45)
            if adjusted_angle >= 315 or adjusted_angle < 45:
                obstacle_center = (scan_index * 5) % 90 - 45 + 360
                obstacle_center = obstacle_center % 360
                if (adjusted_angle >= 315 and obstacle_center >= 315) or (
                    adjusted_angle < 45 and obstacle_center < 45
                ):
                    dist_from_center = min(
                        abs(adjusted_angle - obstacle_center),
                        abs(adjusted_angle - obstacle_center + 360),
                        abs(adjusted_angle - obstacle_center - 360),
                    )
                    if dist_from_center < 20:
                        d = 600 + 200 * math.sin(scan_index * 0.3)
                    else:
                        d = 3000 + random.uniform(-200, 200)
                else:
                    d = 3000 + random.uniform(-200, 200)
            elif 45 <= adjusted_angle < 135:
                d = 6000 + random.uniform(-300, 300)
            elif 135 <= adjusted_angle < 225:
                d = 2500 + 200 * math.sin(math.radians(angle))
            else:
                d = 1200 + 100 * math.cos(math.radians(angle))

            if d > 0:
                d += random.uniform(-20, 20)
                points.append((adjusted_angle, max(100, d)))

        elapsed = max(1e-6, time.time() - t0)
        hz = scan_index / elapsed
        rpm_val = hz * 60.0

        sector_stats = analyze_sectors(points)

        cone_stats = analyze_cones(points)

        points_data = {
            "angles": [round(a, 1) for a, d in points],
            "distances": [int(d) for a, d in points],
        }

        update_lidar_state(
            points_data, sector_stats, rpm_val, scan_index, cone_stats, simulated=True
        )

        _update_drift(_step_heading(points, time.time(), trusted=False))

        msg_data = {
            "scan_index": scan_index,
            "rpm": round(rpm_val, 1),
            "hz": round(hz, 1),
            "points": points_data,
            "sectors": sector_stats,
            "total_points": len(points),
            "simulated": True,
        }

        asyncio.run_coroutine_threadsafe(broadcast(json.dumps(msg_data)), loop)
        time.sleep(0.1)  # 10Hz simulation


def start_scanner(loop):
    """Start the LiDAR scanner thread"""
    stop_scanner.clear()
    if BEV_ENABLED:
        try:
            from utils.bev_grid import reset_grid

            reset_grid()
        except Exception as e:
            print(f"[WARN] BEV grid reset failed: {e}")
    thread = threading.Thread(target=scanner_thread, args=(loop,), daemon=True)
    thread.start()
    print("[INFO] Scanner thread started")


def stop_scanner_thread():
    """Stop the scanner thread"""
    stop_scanner.set()
    cleanup_lidar()
