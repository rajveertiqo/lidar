"""A121 radar array — acquisition for the 6-board, 4-direction radar."""

import glob
import json
import os
import re
import sys
import threading
import time
import types
import warnings
from collections import deque
from typing import Any, Dict, List, Optional

from config import (
    RADAR_MIN_RATE_FRACTION,
    RADAR_BAUDRATE, RADAR_CALIBRATE_STALL_S, RADAR_END_MM, RADAR_ENABLED,
    RADAR_LATE_JOIN_S, RADAR_MAX_LAG_S, RADAR_PORT_EXCLUDE,
    RADAR_PROFILE, RADAR_READY_TIMEOUT_S, RADAR_RESTART_WAIT_S, RADAR_SECTORS,
    RADAR_STALL_S,
    RADAR_REFLECTOR_SHAPE, RADAR_SIGNAL_QUALITY, RADAR_THRESHOLD_SENSITIVITY,
    RADAR_START_MM,
    RADAR_UPDATE_RATE, radar_board_key,
)
from utils.radar_calibration import (
    cardinal_sources, get_bearings, is_board_serial,
)
from utils.shared_state import get_radar_state, update_radar_sector, set_radar_boards

PROBE_CMD = b'{"cmd":"get_system_info"}\n'
STOP_CMD = b'{"cmd":"stop_streaming"}\n'

_SYSFS_TTY = "/sys/class/tty"
_BY_ID_DIR = "/dev/serial/by-id"

warnings.filterwarnings("ignore", message=r"Server timestamp was from .*")

_threads: List[threading.Thread] = []
_stop = threading.Event()
_JOIN_TIMEOUT_S = 3.0       # per thread, in stop_radar()

_discovery: Dict[str, Any] = {"ran": False, "found": [], "error": None}

_key_lock = threading.RLock()

_live: Dict[str, Any] = {"entries": [], "complete": False, "stop": None,
                         "allow_provisional": True}

_restart_state: Dict[str, Any] = {"since": None, "reason": None}


def discovery_info() -> Dict[str, Any]:
    """What the last discovery pass saw, for the diagnostics view."""
    return {"ran": _discovery["ran"],
            "found": [dict(e) for e in _discovery["found"]],
            "error": _discovery["error"]}


def read_system_info(port: str, timeout: float = 1.0) -> Optional[Dict[str, Any]]:
    """Query the exploration server over a raw 115200 link."""
    import serial

    try:
        ser = serial.Serial(port, 115200, timeout=0.15)
    except Exception:
        return None
    try:
        with ser:
            time.sleep(0.1)
            if ser.read(4096):
                _quiesce(ser)

            for _ in range(2):
                ser.reset_input_buffer()
                ser.write(PROBE_CMD)
                ser.flush()
                info = _parse_system_info(_read_for(ser, timeout))
                if info is not None:
                    return info
    except Exception:
        return None
    return None


def _read_for(ser, seconds: float) -> bytes:
    """Accumulate bytes for up to `seconds`, stopping as soon as the reply lands."""
    end = time.monotonic() + seconds
    buf = bytearray()
    while time.monotonic() < end:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
            if b'"system_info"' in buf:
                break
    return bytes(buf)


def _parse_system_info(raw: bytes) -> Optional[Dict[str, Any]]:
    for line in raw.split(b"\n"):
        try:
            msg = json.loads(line)
        except Exception:
            continue
        # A line that is JSON but not an object, or an info that is not one, is not a reply:
        # skip it rather than raise and lose the real reply on the next line (2026-10-22).
        if (isinstance(msg, dict) and msg.get("status") == "ok"
                and isinstance(msg.get("system_info"), dict)):
            return msg["system_info"]
    return None


def _quiesce(ser, max_wait: float = 5.0, silence: float = 0.3) -> bool:
    """Stop a streaming session and wait for the board to actually go quiet."""
    deadline = time.monotonic() + max_wait
    last_data = time.monotonic()
    last_cmd = float("-inf")
    while time.monotonic() < deadline:
        if time.monotonic() - last_cmd > 1.0:
            last_cmd = time.monotonic()
            try:
                ser.write(STOP_CMD)
                ser.flush()
            except Exception:
                return False
        try:
            chunk = ser.read(4096)
        except Exception:
            return False
        if chunk:
            last_data = time.monotonic()
        elif time.monotonic() - last_data > silence:
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
            return True
    return False


def _usb_info(dev: str) -> Dict[str, str]:
    """Walk sysfs from a tty node up to the USB device that owns it."""
    path = os.path.realpath(os.path.join(_SYSFS_TTY, os.path.basename(dev), "device"))
    device = None
    for _ in range(6):
        if os.path.exists(os.path.join(path, "idVendor")):
            device = path
            break
        path = os.path.dirname(path)
    if device is None:
        return {"usb_path": "?", "serial": "?", "hub_port": "-"}

    def read(name: str) -> str:
        try:
            with open(os.path.join(device, name)) as fh:
                return fh.read().strip() or "?"
        except OSError:
            return "?"

    usb_path = os.path.basename(device)
    return {
        "usb_path": usb_path,
        "serial": read("serial"),
        "hub_port": usb_path.split(".")[-1] if "." in usb_path else "-",
    }


def _interface_number(dev: str) -> Optional[int]:
    """The USB interface a tty node sits on (0 for -if00), or None if unknown."""
    try:
        with open(os.path.join(_SYSFS_TTY, os.path.basename(dev), "device",
                               "bInterfaceNumber")) as fh:
            return int(fh.read().strip(), 16)
    except (OSError, ValueError):
        return None


def _lidar_serials() -> set:
    """The LiDAR adapter's USB serial(s): read from the by-id names in RADAR_PORT_EXCLUDE (udev writes it into them), and from whatever those paths resolve to now."""
    found = set()
    for path in RADAR_PORT_EXCLUDE:
        m = re.search(r"_([0-9A-Za-z]+)-if\d+", os.path.basename(path))
        if m:
            found.add(m.group(1))
        if os.path.exists(path):
            try:
                found.add(_usb_info(os.path.realpath(path)).get("serial"))
            except Exception:
                pass
    found.discard("?")
    found.discard(None)
    return found


def discover(verbose: bool = False) -> List[Dict[str, Any]]:
    """Every serial port that answers as an A121 exploration server."""
    candidates = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))

    excluded = {os.path.realpath(p) for p in RADAR_PORT_EXCLUDE}
    candidates = [c for c in candidates if os.path.realpath(c) not in excluded]
    lidar_serials = _lidar_serials()
    if lidar_serials:
        candidates = [c for c in candidates
                      if _usb_info(c).get("serial") not in lidar_serials]
    if not candidates:
        return []

    from concurrent.futures import ThreadPoolExecutor

    with ThreadPoolExecutor(max_workers=len(candidates)) as pool:
        infos = list(pool.map(lambda d: read_system_info(d, timeout=1.0), candidates))

    found = []
    for dev, info in zip(candidates, infos):
        if info is None:
            if verbose:
                print(f"  {dev}: -")
            continue
        entry = {"device": dev, **_usb_info(dev), "rss": info.get("rss_version")}
        found.append(entry)
        if verbose:
            print(f"  {dev}: A121 serial={entry['serial']} hub_port={entry['hub_port']}")
    return found


def _dedupe_serials(found: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """One entry per board, and no usable entry for a board of unknown identity."""
    groups: Dict[Any, List[Dict[str, Any]]] = {}
    for entry in found:
        groups.setdefault(entry.get("serial"), []).append(entry)

    out = []
    for serial, group in groups.items():
        if not is_board_serial(serial):
            for e in group:
                e["refused"] = (f"board reports no usable USB serial ({serial!r}) — it "
                                f"cannot be told apart from another board, so it is not used")
                print(f"[RADAR] {e.get('device')}: {e['refused']}")
            out.extend(group)
            continue
        if len(group) == 1:
            out.extend(group)
            continue
        paths = {e.get("usb_path") for e in group}
        if len(paths) == 1 and not paths & {None, "", "?"}:
            def order(e):
                iface = _interface_number(e.get("device") or "")
                name = os.path.basename(e.get("device") or "")
                return (iface is None, iface or 0, len(name), name)
            keep = min(group, key=order)
            dropped = [e.get("device") for e in group if e is not keep]
            print(f"[RADAR] {serial}: answered on {len(group)} interfaces of one USB "
                  f"device ({next(iter(paths))}) — using {keep.get('device')}, "
                  f"ignoring {', '.join(map(str, dropped))}")
            out.append(keep)
            continue
        reason = (f"serial {serial} answered on {len(group)} different USB devices "
                  f"({', '.join(sorted(str(p) for p in paths))}) — they cannot be told "
                  f"apart, so neither is used")
        for e in group:
            e["refused"] = reason
        print(f"[RADAR] WARNING: {reason}")
        out.extend(group)
    return out


def assign_boards(found: List[Dict[str, Any]],
                  allow_provisional: bool = False) -> List[Dict[str, Any]]:
    """Attach a measured bearing, and its state key, to each discovered board."""
    found = _dedupe_serials(found)
    bearings = get_bearings()
    for entry in found:
        bearing = bearings.get(entry["serial"])
        entry["bearing"] = bearing
        if entry.get("refused"):
            entry["position"] = None
            entry["provisional"] = False
        elif bearing is not None:
            entry["position"] = radar_board_key(bearing)
            entry["provisional"] = False
        elif allow_provisional:
            entry["position"] = f"port{entry['hub_port']}"
            entry["provisional"] = True
        else:
            entry["position"] = None
            entry["provisional"] = False
        entry["assumed"] = False            # retained for /radar/boards' shape

    # Bearing order, so logs read round the compass rather than by USB socket.
    found.sort(key=lambda e: (e["bearing"] is None, e["bearing"] or 0.0))
    return found


def _publish(board_key: str, payload: Dict[str, Any]) -> None:
    """Publish one board's reading, then recompute the cardinals it feeds."""
    with _key_lock:
        update_radar_sector(board_key, payload)

        secs = get_radar_state().get("sectors", {})
        for cardinal, sources in cardinal_sources().items():
            if board_key not in sources:
                continue
            update_radar_sector(cardinal, _derive_cardinal(sources, secs))


def _derive_cardinal(sources, secs: Dict[str, Any]) -> Dict[str, Any]:
    """One cardinal's row from its source boards' rows (see _publish)."""
    rows = [secs.get(k) or {} for k in sources]
    live = [r for r in rows if r.get("online")]
    dists = [r["min_dist"] for r in live
             if isinstance(r.get("min_dist"), (int, float)) and r["min_dist"]]
    return {
        "min_dist": round(min(dists), 1) if dists else None,
        "peaks_mm": sorted(d for r in live for d in (r.get("peaks_mm") or [])),
        "online": bool(live),
        "lag_s": max((r.get("lag_s") or 0.0) for r in live) if live else None,
        "degraded": any(r.get("degraded") for r in live),
        "sources": list(sources),
        "sources_live": [k for k, r in zip(sources, rows) if r.get("online")],
        "error": None if live else "no source board online",
        "last_update": min((r.get("last_update") or 0.0) for r in live) if live else None,
    }


def _recompute_cardinals() -> None:
    """Re-derive EVERY cardinal from the current rows and the current table."""
    with _key_lock:
        secs = get_radar_state().get("sectors", {})
        for cardinal, sources in cardinal_sources().items():
            update_radar_sector(cardinal, _derive_cardinal(sources, secs))


def _publish_for(entry: Dict[str, Any], stop: Optional[threading.Event],
                 payload: Dict[str, Any]) -> bool:
    """Publish for one board thread, under the key its entry holds NOW."""
    with _key_lock:
        if stop is not None and stop.is_set():
            return False
        key = entry.get("position")
        if not key:
            return False
        _publish(key, payload)
        return True


def _tombstone(phase: str, error: str) -> Dict[str, Any]:
    """An offline row: what a key reads once no board is streaming it."""
    return {"min_dist": None, "peaks_mm": [], "online": False, "degraded": False,
            "lag_s": None, "measured_hz": None, "phase": phase, "error": error}


def _preload_exptool() -> None:
    """Import acconeer-exptool once, without its start-up banner."""
    name = "acconeer.exptool._qt_bindings_canary"
    if name not in sys.modules:
        sys.modules[name] = types.ModuleType(name)
    import acconeer.exptool  # noqa: F401


def _detector_config():
    from acconeer.exptool import a121
    from acconeer.exptool.a121.algo.distance import (
        DetectorConfig, PeakSortingMethod, ReflectorShape, ThresholdMethod,
    )

    return DetectorConfig(
        start_m=RADAR_START_MM / 1000.0,
        end_m=RADAR_END_MM / 1000.0,
        max_profile=getattr(a121.Profile, f"PROFILE_{RADAR_PROFILE}"),
        threshold_method=ThresholdMethod.CFAR,
        peaksorting_method=PeakSortingMethod.CLOSEST,   # see note below
        reflector_shape=getattr(ReflectorShape, RADAR_REFLECTOR_SHAPE),
        signal_quality=RADAR_SIGNAL_QUALITY,
        threshold_sensitivity=RADAR_THRESHOLD_SENSITIVITY,
        update_rate=RADAR_UPDATE_RATE,
    )


def _reset_link(device: str) -> None:
    """Stop any session left running on a board and drain what is in flight."""
    import serial

    try:
        ser = serial.Serial(device, RADAR_BAUDRATE, timeout=0.4)
    except Exception:
        return
    try:
        with ser:
            _quiesce(ser)
    except Exception:
        pass                    # best effort; the open below reports real faults


class BoardIdentityError(RuntimeError):
    """The node a board thread was about to open is not provably its board."""


def _node_for(entry: Dict[str, Any], stop: Optional[threading.Event] = None) -> str:
    """The tty node that belongs to entry["serial"] right now, verified."""
    serial = entry.get("serial")
    node = entry.get("device") or ""
    if not is_board_serial(serial):
        raise BoardIdentityError(
            f"board has no usable USB serial ({serial!r}), so no node can be proven "
            f"to be it — nothing opened")
    owner = _usb_info(node)["serial"] if node else "?"
    if owner == serial and _interface_number(node) in (0, None):
        return node

    if owner == serial:
        lost = f"{node} is this board's second interface, not its server"
    elif owner != "?":
        lost = f"device node now belongs to another board ({node} reports serial {owner})"
    elif node and os.path.exists(os.path.join(_SYSFS_TTY, os.path.basename(node))):
        lost = f"device node now belongs to another board ({node} reports no USB serial)"
    else:
        lost = f"its device node {node or '?'} is gone"
    links = sorted(glob.glob(os.path.join(_BY_ID_DIR, f"*_{glob.escape(serial)}-if00*")))
    if len(links) != 1:
        why = ("no /dev/serial/by-id link names it" if not links else
               f"{len(links)} /dev/serial/by-id links name it, so which is it cannot be told")
        raise BoardIdentityError(
            f"{lost}; board not found by serial {serial} ({why}) — nothing opened")
    resolved = os.path.realpath(links[0])
    got = _usb_info(resolved)["serial"]
    if got != serial:
        raise BoardIdentityError(
            f"{lost}; board not found by serial {serial} (its by-id link leads to "
            f"{resolved}, which reports serial {got}) — nothing opened")

    with _key_lock:
        current = stop is None or not stop.is_set()
        if current:
            entry["device"] = resolved
            _record_node(serial, resolved)
    if current:
        print(f"[RADAR] {entry.get('position')}: board {serial} re-enumerated — now on "
              f"{resolved} (was {node or '?'}); followed by serial")
    return resolved


def _record_node(serial: str, node: str) -> None:
    """Point the board lists the pages read at a board's new node, so the diagnostics name the node actually being streamed."""
    snap = get_radar_state()
    set_radar_boards([dict(b, device=node) if b.get("serial") == serial else b
                      for b in snap.get("boards") or []])
    _discovery["found"] = [dict(e, device=node) if e.get("serial") == serial else e
                           for e in _discovery.get("found") or []]


def _run_board(entry: Dict[str, Any], stop: threading.Event) -> None:
    """Supervise one board, reconnecting for as long as the app runs."""
    attempt = 0
    failures = 0            # in a row, without a session that streamed — the backoff
    while not stop.is_set():
        entry["streamed"] = False           # set by _stream_board on its first frame
        try:
            _stream_board(entry, stop)
            return                          # clean stop
        except Exception as exc:
            if stop.is_set():
                return
            attempt += 1
            failures = 1 if entry.get("streamed") else failures + 1
            entry["reconnects"] = attempt
            _publish_for(entry, stop, {
                "min_dist": None, "peaks_mm": [], "online": False,
                "phase": "reconnecting", "measured_hz": None,
                "reconnects": attempt,
                "error": f"{type(exc).__name__}: {exc}",
            })
            backoff = min(2.0 * failures, 15.0)
            detail = str(exc) if isinstance(exc, BoardIdentityError) else str(exc)[:60]
            print(f"[RADAR] {entry.get('position')}: lost the link ({type(exc).__name__}: "
                  f"{detail}) — reconnecting in {backoff:.0f}s "
                  f"(attempt {attempt})")
            if stop.wait(backoff):
                return


class _StallGuard:
    """Ends a board session that stops making progress, so _run_board reconnects it."""

    def __init__(self, entry: Dict[str, Any], client: Any, limit_s: float):
        self._entry, self._client = entry, client
        self._done = threading.Event()
        self.fired = ""                 # why the session was ended, once it was
        self.progress(limit_s)
        threading.Thread(target=self._watch, daemon=True,
                         name=f"radar-{entry.get('position')}-guard").start()

    def progress(self, limit_s: Optional[float] = None) -> None:
        """The session moved on; the next step may take `limit_s` (default: as the last)."""
        limit = self._window[0] if limit_s is None else limit_s
        self._window = (limit, time.monotonic() + limit)    # one assignment: read whole

    def close(self) -> None:
        self._done.set()

    def _watch(self) -> None:
        while not self._done.wait(1.0):
            limit, deadline = self._window
            if time.monotonic() <= deadline:
                continue
            # The client's link raises "recv timeout" on its next read, closes the
            # port and reraises. Re-applied each second: starting a session resets it.
            link = getattr(self._client, "_link", None)
            try:
                link.timeout = float("-inf")
            except Exception:
                continue
            if not self.fired:
                self.fired = f"no progress for {limit:.0f} s"
                print(f"[RADAR] {self._entry.get('position')}: {self.fired} — "
                      f"ending the session")


def _stream_board(entry: Dict[str, Any], stop: threading.Event) -> None:
    """One connect-and-stream cycle. Raises if the link fails."""
    from acconeer.exptool import a121
    from acconeer.exptool.a121.algo.distance import Detector

    client = detector = guard = None
    recent = deque(maxlen=60)
    try:
        device = _node_for(entry, stop)
        if stop.is_set():       # retired meanwhile: the port may be the next generation's
            return
        _reset_link(device)
        info = read_system_info(device)
        if info is None:
            raise RuntimeError(
                "board not responding — if this repeats, unplug and replug its USB-C cable"
            )

        max_baud = info.get("max_baudrate")
        baud = min(RADAR_BAUDRATE, max_baud) if max_baud else RADAR_BAUDRATE

        client = a121.Client.open(serial_port=device, override_baudrate=baud)
        guard = _StallGuard(entry, client, RADAR_CALIBRATE_STALL_S)
        detector = Detector(client=client, sensor_ids=[1], detector_config=_detector_config())

        _publish_for(entry, stop, {"phase": "calibrating", "online": False, "error": None,
                                   "reconnects": entry.get("reconnects", 0)})
        detector.calibrate_detector()
        if stop.is_set():
            return
        detector.start()
        guard.progress(RADAR_STALL_S)
        print(f"[RADAR] {entry.get('position')}: streaming on {device} @ {baud} baud")

        started = time.monotonic()
        frames = 0
        last_lag_log = float("-inf")
        period = 1.0 / RADAR_UPDATE_RATE

        while not stop.is_set():
            result = detector.get_next()[1]
            guard.progress()
            frames += 1
            entry["streamed"] = True
            recent.append(time.monotonic())

            lag = (time.monotonic() - started) - frames * period

            if lag > RADAR_MAX_LAG_S:
                budget = time.monotonic() + 0.25
                drained = False
                while lag > RADAR_MAX_LAG_S and time.monotonic() < budget:
                    call_started = time.monotonic()
                    result = detector.get_next()[1]
                    guard.progress()
                    frames += 1
                    recent.append(time.monotonic())
                    lag = (time.monotonic() - started) - frames * period
                    if time.monotonic() - call_started > period / 2:
                        drained = True
                        break

                if drained:
                    if lag > RADAR_MAX_LAG_S and time.monotonic() - last_lag_log > 10.0:
                        last_lag_log = time.monotonic()
                        print(f"[RADAR] {entry.get('position')}: was {lag:.2f}s behind, "
                              f"queue now empty — baseline reset, readings are current")
                    started, frames, lag = time.monotonic(), 0, 0.0
                elif time.monotonic() - last_lag_log > 10.0:
                    last_lag_log = time.monotonic()
                    print(f"[RADAR] {entry.get('position')}: running {lag:.2f}s behind — "
                          f"host cannot keep up at {RADAR_UPDATE_RATE:g} Hz")

            # Temperature drift eventually invalidates the recorded threshold.
            if result.calibration_needed:
                guard.progress(RADAR_CALIBRATE_STALL_S)
                detector.stop()
                _publish_for(entry, stop, {"phase": "recalibrating", "online": False,
                                           "reconnects": entry.get("reconnects", 0)})
                detector.calibrate_detector()
                if stop.is_set():                    # retired meanwhile: see above
                    return
                detector.start()
                guard.progress(RADAR_STALL_S)
                started, frames = time.monotonic(), 0    # restart the lag baseline
                recent.clear()                           # and the rate window
                continue

            distances = [float(d) * 1000.0 for d in result.distances]
            strengths = [float(s) for s in result.strengths]

            hz = None
            if len(recent) > 5:
                span = recent[-1] - recent[0]
                hz = round((len(recent) - 1) / span, 2) if span > 0 else None

            _publish_for(entry, stop, {
                "min_dist": round(min(distances), 1) if distances else None,
                "strength_db": round(strengths[distances.index(min(distances))], 2)
                               if distances else None,
                "peaks_mm": [round(d, 1) for d in distances],
                "temperature": int(result.temperature),
                "near_edge": bool(result.near_edge_status),
                "lag_s": round(max(lag, 0.0), 3),
                "degraded": (lag > RADAR_MAX_LAG_S
                             or (hz is not None
                                 and hz < RADAR_UPDATE_RATE * RADAR_MIN_RATE_FRACTION)),
                "degraded_reason": ("backlog" if lag > RADAR_MAX_LAG_S else
                                    "slow" if (hz is not None
                                               and hz < RADAR_UPDATE_RATE
                                               * RADAR_MIN_RATE_FRACTION)
                                    else None),
                "measured_hz": hz,
                "frames": frames,
                "reconnects": entry.get("reconnects", 0),
                "phase": "streaming",
                "online": True,
                "error": None,
            })
    except Exception as exc:
        if guard is not None and guard.fired:
            raise RuntimeError(f"{guard.fired} — session ended, reconnecting") from exc
        raise
    finally:
        for closer in (getattr(detector, "stop", None), getattr(client, "close", None)):
            if closer is None:
                continue
            try:
                closer()
            except Exception:
                pass
        if guard is not None:
            guard.close()           # after the closers, so they are bounded too


def _log_inventory(found: List[Dict[str, Any]]) -> List[Dict[str, str]]:
    """Print exactly which radars answered and which did not."""
    expected = get_bearings()                   # serial -> bearing (or None)
    seen = {e["serial"] for e in found}

    print(f"[RADAR] {len(found)}/{len(expected)} boards answered "
          f"(LiDAR port {', '.join(RADAR_PORT_EXCLUDE)} excluded from the scan)")
    for e in found:
        bearing = ("UNMEASURED" if e["bearing"] is None
                   else f"{e['bearing']:>5.1f} deg")
        print(f"[RADAR]   {str(e['position'] or '------'):<7} {bearing:<11} "
              f"{e['device']:<14} serial {e['serial']}  hub port {e['hub_port']}")

    missing = [{"serial": s, "bearing": b,
                "position": radar_board_key(b) if b is not None else None}
               for s, b in expected.items() if s not in seen]
    for m in missing:
        where = ("bearing unmeasured" if m["bearing"] is None
                 else f"bearing {m['bearing']:.1f} deg")
        print(f"[RADAR]   MISSING — configured board {m['serial']} ({where}) "
              f"did not answer on any USB port")

    unexpected = [e for e in found if e["serial"] not in expected and not e.get("refused")]
    for e in unexpected:
        print(f"[RADAR]   note: board {e['serial']} is on no roster — calibrate it "
              f"from /radar/test or it stays unused")

    if missing:
        print(f"[RADAR] WARNING: {len(missing)} configured board(s) absent — "
              f"check the USB cable and hub socket for those serials")

    unmeasured = [e for e in found if e["bearing"] is None and not e.get("refused")]
    if unmeasured:
        print(f"[RADAR] WARNING: {len(unmeasured)} board(s) have NO MEASURED BEARING "
              f"and will not be used: {', '.join(e['serial'] for e in unmeasured)}")
        print(f"[RADAR]   fix: open /radar/test and use the CALIBRATE panel — put a "
              f"hand in front of one board at a time and set its bearing")

    starved = [c for c, srcs in cardinal_sources().items() if not srcs]
    if starved:
        print(f"[RADAR] WARNING: no board covers {', '.join(sorted(starved))} — "
              f"GLASS WILL NOT BE DETECTED there")
    return missing


def _new_generation() -> threading.Event:
    """Retire every earlier board thread and return a fresh stop flag."""
    global _stop
    with _key_lock:
        _stop.set()
        _live.update(entries=[], complete=False, stop=None)
    deadline = time.monotonic() + RADAR_RESTART_WAIT_S      # an interval: monotonic
    for t in list(_threads):
        t.join(timeout=max(0.0, deadline - time.monotonic()))
    stuck = [t.name for t in _threads if t.is_alive()]
    if stuck:
        print(f"[RADAR] WARNING: {', '.join(stuck)} still inside a blocking call "
              f"after {RADAR_RESTART_WAIT_S:.0f}s — restarting anyway; they are "
              f"stopped and will exit when it returns")
    _threads.clear()
    fresh = threading.Event()
    with _key_lock:
        _stop = fresh
        _live["stop"] = fresh
    return fresh


def _startup(allow_provisional: bool = False) -> Dict[str, Any]:
    """Discover, log the inventory, and start one acquisition thread per board."""
    global _cycle_again
    stop = _new_generation()
    with _cycle_lock:
        if not stop.is_set():
            _cycle_again = False
    with _key_lock:
        _live["allow_provisional"] = allow_provisional
    try:
        _preload_exptool()
        print("[RADAR] discovering boards ...")
        found = assign_boards(discover(), allow_provisional=allow_provisional)
    except Exception as exc:
        _discovery.update({"ran": True, "found": [], "error": f"{type(exc).__name__}: {exc}"})
        print(f"[RADAR] discovery failed: {type(exc).__name__}: {exc}")
        return {"enabled": True, "boards": [], "error": str(exc)}

    _discovery.update({"ran": True, "found": [dict(e) for e in found], "error": None})

    if not found:
        print("[RADAR] no A121 boards answered — running LiDAR-ONLY. "
              "Glass will NOT be detected in any direction.")
        set_radar_boards([], [
            {"serial": s, "bearing": b,
             "position": radar_board_key(b) if b is not None else None}
            for s, b in get_bearings().items()
        ])
        return {"enabled": True, "boards": []}

    missing = _log_inventory(found)

    boards = []
    for entry in found:
        if entry["position"] is None:
            if entry.get("refused"):
                print(f"[RADAR] {entry['serial']} on {entry['device']}: REFUSING to use "
                      f"this board — {entry['refused']}")
            else:
                print(f"[RADAR] {entry['serial']}: bearing not measured — REFUSING to use "
                      f"this board (a guessed direction is worse than none)")
            continue
        if entry.get("provisional"):
            print(f"[RADAR] {entry['serial']}: bearing not measured — streaming for "
                  f"CALIBRATION ONLY (feeds no direction; open /radar/test to set it)")
        boards.append({k: entry.get(k) for k in
                       ("device", "serial", "hub_port", "usb_path", "rss",
                        "position", "bearing", "provisional", "assumed")})
        _publish(entry["position"], {
            "phase": "starting", "online": False, "min_dist": None,
            "peaks_mm": [], "reconnects": 0, "error": None,
        })
        t = threading.Thread(target=_run_board, args=(entry, stop),
                             name=f"radar-{entry['position']}", daemon=True)
        with _key_lock:
            if _live["stop"] is stop:
                _live["entries"].append((entry, t))
        t.start()
        _threads.append(t)
        time.sleep(0.4)     # stagger so failures stay attributable to a board

    set_radar_boards(boards, missing)

    with _key_lock:
        if _live["stop"] is stop and not stop.is_set() and boards:
            _live["complete"] = True

    if not boards:
        print("[RADAR] no usable board — running LiDAR-ONLY. Glass will NOT be "
              "detected in any direction.")
        set_radar_boards([], missing)
        return {"enabled": True, "boards": [], "missing": missing}

    expected = _live_positions(stop) or {b["position"] for b in boards}
    deadline = time.monotonic() + RADAR_READY_TIMEOUT_S     # an interval: monotonic
    while time.monotonic() < deadline and not stop.is_set():
        expected = _live_positions(stop) or expected
        live = {p for p, s in get_radar_state()["sectors"].items() if s.get("online")}
        if expected <= live:
            break
        time.sleep(0.5)
    expected = _live_positions(stop) or expected

    secs = get_radar_state()["sectors"]
    live_boards = sorted(k for k in expected if (secs.get(k) or {}).get("online"))
    dead_boards = sorted(expected - set(live_boards))
    live_cards = sorted(c for c in RADAR_SECTORS if (secs.get(c) or {}).get("online"))
    dead_cards = sorted(set(RADAR_SECTORS) - set(live_cards))

    print(f"[RADAR] READY {len(live_boards)}/{len(expected)} boards live: "
          f"{', '.join(live_boards) or 'none'}"
          + (f"; NO DATA from: {', '.join(dead_boards)}" if dead_boards else ""))

    if not live_cards:
        print(f"[RADAR] *** NO GLASS DETECTION IN ANY DIRECTION *** "
              f"0/{len(RADAR_SECTORS)} cardinals have a live board")
        if live_boards:
            print(f"[RADAR]     {len(live_boards)} board(s) ARE streaming — this is a "
                  f"CALIBRATION gap, not a hardware fault. Boards with no measured "
                  f"bearing feed no direction.")
            print(f"[RADAR]     fix: open /radar/test and set each board's bearing "
                  f"(hold a hand in front of one board at a time)")
    elif dead_cards:
        print(f"[RADAR] READY {len(live_cards)}/{len(RADAR_SECTORS)} cardinals: "
              f"{', '.join(live_cards)} — NO GLASS DETECTION for "
              f"{', '.join(dead_cards)}")
    else:
        print(f"[RADAR] READY all {len(RADAR_SECTORS)} cardinals live: "
              f"{', '.join(live_cards)}")
    return {"enabled": True, "boards": boards, "missing": missing}


def start_radar(background: bool = True,
                allow_provisional: bool = True) -> Dict[str, Any]:
    """Discover the boards and start one acquisition thread each."""
    if not RADAR_ENABLED:
        print("[RADAR] disabled in config (RADAR_ENABLED=False) — LiDAR-only, "
              "glass will not be detected")
        return {"enabled": False, "boards": []}

    if not background:
        return _startup(allow_provisional=allow_provisional)

    _request_cycle(restart=False, allow_provisional=allow_provisional)
    return {"enabled": True, "starting": True}


_cycle_lock = threading.Lock()
_cycle_running = False
_cycle_again = False


def _request_cycle(restart: bool, allow_provisional: bool,
                   reason: Optional[str] = None) -> None:
    global _cycle_running, _cycle_again
    with _cycle_lock:
        if not _cycle_running or _restart_state["since"] is None:
            _restart_state["since"] = time.time()
        _restart_state["reason"] = reason or ("restart" if restart else "startup")
        if _cycle_running:
            _cycle_again = True
            _stop.set()
            return
        _cycle_running = True
    threading.Thread(target=_run_cycles, args=(restart, allow_provisional),
                     name="radar-restart" if restart else "radar-startup",
                     daemon=True).start()


def _run_cycles(restart: bool, allow_provisional: bool) -> None:
    global _cycle_running, _cycle_again
    while True:
        try:
            if restart:
                stop_radar()
            _startup(allow_provisional=allow_provisional)
        except Exception as exc:
            print(f"[RADAR] {'restart' if restart else 'startup'} failed: "
                  f"{type(exc).__name__}: {exc}")
        with _cycle_lock:
            if not _cycle_again:
                _cycle_running = False
                _restart_state.update(since=None, reason=None)
                with _key_lock:
                    stop = _live["stop"]
                if stop is not None and not stop.is_set() and RADAR_LATE_JOIN_S > 0:
                    t = threading.Thread(target=_late_join_loop, args=(stop,),
                                         name="radar-late-join", daemon=True)
                    t.start()
                    _threads.append(t)      # joined like a board's on restart or stop
                return
            _cycle_again = False
        restart = True


def _late_join_loop(stop: threading.Event) -> None:
    """While this generation lasts, start any board with a measured bearing that is not part of it once it answers — plugged back in, or its firmware recovered (2026-10-21)."""
    while not stop.wait(RADAR_LATE_JOIN_S):
        try:
            _late_join_once(stop)
        except Exception as exc:
            print(f"[RADAR] late-join search failed: {type(exc).__name__}: {exc}")


def _late_join_once(stop: threading.Event) -> List[str]:
    """One search; the positions started. Opens only ports whose USB serial is a missing board's, and none a running board holds."""
    with _key_lock:
        if _live["stop"] is not stop:
            return []
        entries = [e for e, _ in _live["entries"]]
    have = {e.get("serial") for e in entries}
    wanted = {s for s, b in get_bearings().items() if b is not None and s not in have}
    if not wanted:
        return []
    busy = {os.path.realpath(e.get("device") or "") for e in entries}
    candidates = [c for c in sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
                  if os.path.realpath(c) not in busy and _usb_info(c).get("serial") in wanted]
    if not candidates:
        return []
    found = []
    for dev in candidates:
        if stop.is_set():
            return []
        info = read_system_info(dev, timeout=1.0)
        if info is not None:
            found.append({"device": dev, **_usb_info(dev), "rss": info.get("rss_version")})
    started = []
    for entry in assign_boards(found, allow_provisional=False):
        if entry.get("position") is None or entry.get("serial") in have:
            continue
        print(f"[RADAR] {entry['serial']} ({entry['position']}, bearing "
              f"{entry['bearing']:.1f} deg) answered on {entry['device']} — it was missing; "
              f"starting it")
        t = threading.Thread(target=_run_board, args=(entry, stop),
                             name=f"radar-{entry['position']}", daemon=True)
        with _key_lock:
            if _live["stop"] is not stop or stop.is_set():
                return started
            _publish(entry["position"], {
                "phase": "starting", "online": False, "min_dist": None,
                "peaks_mm": [], "reconnects": 0, "error": None,
            })
            _live["entries"].append((entry, t))
            _live["complete"] = True           # start-up has finished; every board is in
            _discovery["found"] = list(_discovery.get("found") or []) + [dict(entry)]
        t.start()
        _threads.append(t)
        have.add(entry["serial"])
        started.append(entry["position"])
    if started:
        snap = get_radar_state()
        joined = {e["serial"] for e in found if e.get("position") in started}
        boards = list(snap.get("boards") or []) + [
            {k: e.get(k) for k in ("device", "serial", "hub_port", "usb_path", "rss",
                                   "position", "bearing", "provisional", "assumed")}
            for e in found if e.get("position") in started]
        set_radar_boards(boards, [m for m in snap.get("missing") or []
                                  if m.get("serial") not in joined])
    return started


def restart_status() -> Dict[str, Any]:
    """Whether a discovery pass is running now, since when, and why."""
    with _cycle_lock:
        return {"in_progress": bool(_cycle_running),
                "since": _restart_state["since"],
                "reason": _restart_state["reason"]}


def restart_radar(reason: str = "restart") -> None:
    """Stop every board and rediscover, off-thread."""
    if not RADAR_ENABLED:
        return
    _request_cycle(restart=True, allow_provisional=True, reason=reason)


def _live_positions(stop: Optional[threading.Event] = None) -> set:
    """Keys the current generation's boards publish under right now."""
    with _key_lock:
        if stop is not None and _live["stop"] is not stop:
            return set()
        return {e.get("position") for e, _ in _live["entries"] if e.get("position")}


def _retire_generation(phase: str, error: str) -> List[str]:
    """Stop the current generation NOW and blank every row it was feeding."""
    with _key_lock:
        _stop.set()
        _live["complete"] = False
        keys = sorted(_live_positions())
        for key in keys:
            _publish(key, _tombstone(phase, error))
    return keys


def _plan_rekey(table: Dict[str, Optional[float]], requested: set):
    """How the running boards' keys must change for `table`, or why they can't."""
    if not (_live["complete"] and _live["stop"] is _stop and not _stop.is_set()):
        return None, "the radar is not running a complete set of boards"
    entries = list(_live["entries"])
    running = {e.get("serial") for e, _ in entries}
    discovered = {e.get("serial") for e in _discovery.get("found") or []}
    idle = sorted((requested & discovered) - running)
    if idle:
        return None, f"board(s) {', '.join(idle)} answered discovery but are not running"

    changes, final = [], {}
    for entry, thread in entries:
        serial = entry.get("serial")
        bearing = table.get(serial)
        if bearing is not None:
            key, provisional = radar_board_key(bearing), False
        elif _live["allow_provisional"]:
            key, provisional = f"port{entry.get('hub_port')}", True
        else:
            return None, f"{serial} would be refused without a bearing"
        final.setdefault(key, []).append(serial)
        if key == entry.get("position") and bearing == entry.get("bearing"):
            continue
        if serial not in requested:
            return None, (f"running board {serial} disagrees with the saved table "
                          f"although it was not in this request")
        changes.append((entry, thread, key, bearing, provisional))
    shared = sorted(k for k, owners in final.items() if len(owners) > 1)
    if shared:
        return None, f"two running boards would share key(s) {', '.join(shared)}"
    return changes, None


def _apply_rekey(changes) -> List[Dict[str, Any]]:
    """Move running boards to their new keys."""
    moved = [(e, t, k, b, p) for e, t, k, b, p in changes if k != e.get("position")]
    dirty = ({e.get("position") for e, *_ in moved} | {k for _, _, k, _, _ in moved}) - {None}

    for cardinal in cardinal_sources():
        update_radar_sector(cardinal, {
            **_tombstone("rekeying", "board mapping changed by calibration"),
            "sources": [], "sources_live": []})
    for key in sorted(dirty):
        update_radar_sector(key, _tombstone("rekeyed", "board re-mapped by calibration"))

    done, by_serial = [], {}
    for entry, thread, key, bearing, provisional in changes:
        done.append({"serial": entry.get("serial"), "from": entry.get("position"),
                     "to": key, "bearing": bearing})
        entry["position"], entry["bearing"], entry["provisional"] = key, bearing, provisional
        by_serial[entry.get("serial")] = (key, bearing, provisional)
        try:
            thread.name = f"radar-{key}"
        except Exception:
            pass

    def rewrite(rows):
        out = []
        for row in rows:
            row = dict(row)
            if row.get("serial") in by_serial:
                row["position"], row["bearing"], row["provisional"] = by_serial[row["serial"]]
            out.append(row)
        return out

    snap = get_radar_state()
    table = get_bearings()
    missing = [{**m, "bearing": table.get(m.get("serial")),
                "position": (radar_board_key(table[m["serial"]])
                             if table.get(m.get("serial")) is not None else None)}
               for m in snap.get("missing") or []]
    set_radar_boards(rewrite(snap.get("boards") or []), missing)
    _discovery["found"] = rewrite(_discovery.get("found") or [])

    _recompute_cardinals()
    return done


def apply_calibration(mapping: Dict[str, Optional[float]]) -> Dict[str, Any]:
    """Record measured bearings and make them take effect."""
    from utils import radar_calibration as rc
    requested = {str(s) for s in mapping}
    with rc._save_lock:
        staged, text = rc.prepare_bearings(mapping)      # ValueError: nothing changed
        rc.persist_bearings(text)                       # CalibrationSaveError: ditto
        with _key_lock:
            table = rc.commit_bearings(staged)
            result = _apply_committed(table, requested)
    if result["applied"] == "live" and result["rekeyed"]:
        print("[RADAR] calibration applied LIVE (no restart): "
              + ", ".join(f"{d['serial']} {d['from']} -> {d['to']}"
                          for d in result["rekeyed"]))
    elif result["applied"] == "restart":
        print(f"[RADAR] calibration saved; restarting the array to apply it "
              f"({result['reason']})")
    return result


def _apply_committed(table, requested) -> Dict[str, Any]:
    """The re-key-or-restart half of apply_calibration."""
    if not RADAR_ENABLED:
        return {"bearings": table, "applied": "recorded", "restarting": False,
                "rekeyed": [], "reason": "radar disabled in config"}
    changes, why = _plan_rekey(table, requested)
    if changes is not None:
        done = _apply_rekey(changes)
        return {"bearings": table, "applied": "live", "restarting": False,
                "rekeyed": done, "reason": None}
    _retire_generation("restarting", "restarting to apply a calibration")
    _recompute_cardinals()
    restart_radar(reason="calibration")
    return {"bearings": table, "applied": "restart", "restarting": True,
            "rekeyed": [], "reason": why}


def stop_radar() -> None:
    _retire_generation("stopped", "radar stopped or restarting")
    for t in list(_threads):
        t.join(timeout=_JOIN_TIMEOUT_S)
    _threads[:] = [t for t in _threads if t.is_alive()]


CHECK_SOFTWARE_ERROR = 4


def _run_check() -> int:
    """The --check pre-flight, as an exit code. Never raises."""
    try:
        return _check()
    except Exception:
        import traceback
        traceback.print_exc()
        return CHECK_SOFTWARE_ERROR


def _check() -> int:
    """Report which boards are present; the bitmask described at the end."""
    expected = get_bearings()
    found = assign_boards(discover())
    seen = {e["serial"] for e in found}

    rows = [((e["bearing"] is None, e["bearing"] or 0.0),
             str(e["position"] or "------"),
             ("UNMEASURED" if e["bearing"] is None else f"{e['bearing']:.0f} deg"),
             e["device"], e["serial"], str(e["hub_port"])) for e in found]
    missing = [(s, b) for s, b in expected.items() if s not in seen]
    rows += [((b is None, b or 0.0),
              str(radar_board_key(b) if b is not None else "------"),
              ("UNMEASURED" if b is None else f"{b:.0f} deg"),
              "MISSING", s, "-") for s, b in missing]

    for _, key, bearing, device, serial, hub in sorted(rows):
        tail = ("— no glass detection on that arc" if device == "MISSING"
                else f"hub port {hub}")
        print(f"  {key:<7} {bearing:<11} {device:<14} serial {serial}  {tail}")

    unmeasured = [e for e in found if e["bearing"] is None]
    if unmeasured:
        print(f"  {len(unmeasured)} board(s) have no measured bearing and will "
              f"NOT steer — calibrate them at /radar/test")
    for e in found:
        if e.get("refused"):                # identity, see _dedupe_serials
            print(f"  REFUSED {e['device']}: {e['refused']}")
    for cardinal in RADAR_SECTORS:
        srcs = cardinal_sources().get(cardinal) or ()
        live = [k for k in srcs
                if k in {e["position"] for e in found if e["position"]}]
        state = ("NO SOURCE" if not srcs else
                 f"{len(live)}/{len(srcs)} boards present ({', '.join(srcs)})")
        print(f"  cardinal {cardinal:<6} {state}")

    print(f"  {len(found)}/{len(expected)} radars detected, "
          f"{sum(1 for e in found if e['position'])} usable")

    return (1 if missing else 0) | (2 if unmeasured else 0)


# ==========================================================================
if __name__ == "__main__":
    if "--check" in sys.argv:
        sys.exit(_run_check())

    if "--identify" in sys.argv:
        print("discovering A121 boards (LiDAR port excluded) ...")
        entries = assign_boards(discover(verbose=True))
        if not entries:
            print("\nnothing found.")
            sys.exit(1)
        print(f"\n{'key':<9}{'bearing':<12}{'device':<15}{'serial':<13}{'hub port':<10}source")
        for e in entries:
            origin = ("NOT MEASURED — calibrate at /radar/test" if e["bearing"] is None
                      else "measured")
            bearing = ("-" if e["bearing"] is None else f"{e['bearing']:.0f} deg")
            print(f"{str(e['position'] or '------'):<9}{bearing:<12}{e['device']:<15}"
                  f"{e['serial']:<13}{e['hub_port']:<10}{origin}")
        print("\nThis only reports what is already recorded. To MEASURE which board "
              "faces which way, open /radar/test and use the CALIBRATE panel (or run "
              "--watch for the terminal equivalent).")
        sys.exit(0)

    if "--watch" in sys.argv:
        print("Starting all boards — this takes ~30 s (each calibrates first).\n")
        from utils.walk_policy import _mm_to_ft

        info = start_radar(background=False, allow_provisional=True)
        boards = info.get("boards", [])
        if not boards:
            print("no boards to watch")
            sys.exit(1)

        order = sorted(boards, key=lambda b: b["hub_port"])
        print(f"\nPut your hand ~1.5 ft in front of ONE sensor at a time.")
        print("The column whose distance drops is that sensor.")
        print(f"Closer than {RADAR_START_MM / 304.8:.1f} ft the board is blind, "
              f"and it reaches {RADAR_END_MM / 304.8:.1f} ft — '---' means "
              f"nothing in that range.\n")
        print("  " + "".join(f"port {b['hub_port']}: {b['serial']:<14}" for b in order))
        print("  " + "".join(
            (f"(cfg {b['position']}@{b['bearing']:.0f}deg){'':<4}"
             if b.get("bearing") is not None
             else f"(UNMEASURED {b['position']}){'':<3}") for b in order))
        print("  " + "-" * (28 * len(order)))
        try:
            while True:
                secs = get_radar_state().get("sectors", {})
                cells = []
                for b in order:
                    sec = secs.get(b["position"]) or {}
                    ft = _mm_to_ft(sec.get("min_dist"))
                    cells.append(f"{(f'{ft:.1f} ft' if ft else '---'):<28}")
                print("  " + "".join(cells), flush=True)
                time.sleep(0.4)
        except KeyboardInterrupt:
            stop_radar()
        sys.exit(0)

    start_radar()
    try:
        while True:
            time.sleep(0.5)
            secs = get_radar_state().get("sectors", {})
            print("  ".join(
                f"{s}={(secs.get(s) or {}).get('min_dist') or '---'}" for s in RADAR_SECTORS
            ))
    except KeyboardInterrupt:
        stop_radar()
