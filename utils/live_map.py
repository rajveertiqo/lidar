"""Live map for the /map test page: what the LiDAR, radar and cameras see, placed around the user."""

import base64
import math
import threading
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover - only on a broken install
    cv2 = None

from config import (
    AUDIO_ENABLED,
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    FRAME_GRID_COLS,
    FRAME_GRID_ROWS,
    GUIDANCE_CORRIDOR_HALF_WIDTH_MM,
    GYRO_MOUNTING_CONFIRMED,
    LIDAR_MAX_DISTANCE,
    RADAR_AGREE_MM,
    RADAR_END_MM,
    RADAR_GLASS_GAP_MM,
    RADAR_START_MM,
    SIMPLE_CAMERA_FRESH_S,
    SIMPLE_CAMERA_HFOV_DEG,
    SIMPLE_OBJECT_HEIGHT_M,
    SIMPLE_VEHICLE_HEIGHT_M,
    SIMPLE_VEHICLE_MISS_M,
    SIMPLE_VEHICLE_RANGE_MATCH,
    WALK_BLOCK_FT,
    radar_beam_deg,
    radar_ring,
)
from utils.gyro_reader import get_gyro
from utils.heading_tracker import get_tracker
from utils.shared_state import get_camera_state, get_lidar_state, get_radar_state

# Nothing here is read by the guide; the page shows (and, if asked, speaks) its alerts.

CELL_M = 0.10  # memory map cell
GRID_HALF_M = 25.0  # the grid spans 50 m and is re-centred under the user
RECENTRE_M = 8.0  # re-centre once the user is this far from the grid centre
KEEP_M = 20.0  # the rolling map: walls farther than this are not shown
L_HIT, L_FREE, L_MAX, L_MIN = 0.9, 0.35, 3.5, -2.0
OCC_L = 1.5  # two hits before a cell counts as a wall
FREE_L = -0.6  # below this a cell has been seen empty
DECAY = 0.996  # per scan: a wall not seen again fades in ~8-14 s at 15 Hz (weak-strong)
SCAN_GAP_DEG = 5.0  # a wider gap between returns is unknown, not free floor
MAX_STEP_M = 0.4  # one scan's walking step; more is a failed match
WALL_CELLS_MAX = 9000
FLOOR_EVERY_S = 1.0

POLL_S = 0.02
GYRO_LAG_S = 0.025  # integrate the gyro this far behind now, so its samples are in
SCAN_WAIT_S = 0.08  # wait this long for the heading tracker's step for a scan
SCAN_FRESH_S = 1.0  # an older scan is not drawn: the page is live
IDLE_STOP_S = 30.0  # the worker stops when nobody has looked for this long
# One GET /map/state keeps it only this long: each kept the worker 30 s, CPU on a hot Pi
# (scan 2026-10-23). A page streams over the WebSocket, which holds it while open.
STATE_HOLD_S = 5.0
HISTORY_S = 3.0

TRACK_HIST_S = 2.0
TRACK_MIN_SPAN_S = 0.8
TRACK_MIN_SAMPLES = 6
TRACK_STALE_S = 0.6  # no speed from a history whose newest point is older than this
TRACK_MIN_AGE_S = (
    1.0  # a new track's first second can slide onto a neighbour: no speed yet
)
# Vehicles are big, lone outlines and the most dangerous: their speed is wanted sooner.
_FAST_START = {
    "vehicle": (0.5, 0.5),
    "bike": (0.5, 0.5),
}  # group -> (min age, min span), s
STILL_MPS = 0.3
TURNING_DPS = 45.0  # movement is not judged while the user turns faster than this
MATCH_GATE_M = 1.2
MATCH_GATE_MPS = 2.0
LASER_MIN_POINTS = 2
LASER_RECENT_S = 0.3
CONTINUITY_M = (
    0.6  # a camera box keeps a tracked thing on the outline it already stands on
)
LASER_KEEP_S = 1.0  # a laser fix this fresh outranks the camera's size-based range
SINGLE_POINT_M = 0.25  # a tracked thing may be followed on one return this close to where it should be
VEL_MIN_PAIR_S = 0.25
CAMERA_RANGE_BLEND = 0.35  # camera-only ranges are smoothed; their bearings are not
CAMERA_STILL_FRAC = (
    0.12  # a camera-only thing must move this share of its range per second to count
)
LASER_STILL_FRAC = (
    0.07  # laser outlines coarsen with range too (~0.16 m between returns at 6.6 m)
)
CAMERA_BEARING_TOL_DEG = 3.0
CAMERA_MISSES_WITH_LASER = (
    3  # camera frames a thing may be missed in while the laser still follows it
)
LENS_SAMPLES = 60

ALERT_HORIZON_S = 10.0  # reaching the user later than this is not an approach yet
ALERT_URGENT_S = 3.0
ALERT_SOON_S = 6.0
ALERT_CONFIRM_S = 0.3  # on course this long before it is shown
ALERT_SMOOTH_S = 0.5
ALERT_MIN_CLOSING_MPS = 0.3
ALERT_KEEP = (
    1.3  # a standing alert holds until the pass is this much wider (no flicker)
)
USER_VEL_WINDOW_S = 0.6
# The thing itself must move this fast (walking up to a parked car is not an approach) ...
ALERT_MIN_SPEED_MPS = {"person": 0.4, "animal": 0.5, "bike": 0.8, "vehicle": 0.8}
# ... and be on course to pass this close to the user.
ALERT_MISS_M = {
    "person": 1.0,
    "animal": 1.0,
    "bike": SIMPLE_VEHICLE_MISS_M,
    "vehicle": SIMPLE_VEHICLE_MISS_M,
}
ZONE_TEXT = {
    "front": "in front",
    "front-left": "in front, left side",
    "front-right": "in front, right side",
    "left": "on your left",
    "right": "on your right",
    "behind-left": "behind you, left side",
    "behind-right": "behind you, right side",
    "behind": "behind you",
}
_PLURAL = {
    "person": "people",
    "vehicle": "vehicles",
    "bike": "bikes",
    "animal": "animals",
}
_ALERT_ORDER = {"vehicle": 0, "bike": 1, "person": 2, "animal": 3}
_LEVEL_ORDER = {"urgent": 0, "soon": 1, "info": 2}

# Threat levels (Shappe spatial threat concept): 1 path conflict, 2 high potential threat, 3 monitor.
PATH_HALF_M = (
    GUIDANCE_CORRIDOR_HALF_WIDTH_MM / 1000.0 + 0.3
)  # the body's corridor plus a margin
PATH_MIN_M = 3.0  # the projected path when standing
PATH_MAX_M = 8.0
WALKING_MPS = 0.3
FAST_PERSON_MPS = 2.5  # a runner is a level-2 threat even off the path
_RADIUS_M = {
    "person": 0.25,
    "bike": 0.4,
    "vehicle": 1.0,
    "animal": 0.3,
    "pole": 0.1,
    "other": 0.3,
}
NEAR_URGENT_M = (
    WALK_BLOCK_FT * 0.3048
)  # in the path and this close: the guide's "blocked" distance
NEAR_SOON_M = 2.5
STEP_DT_MAX_S = 0.2  # longest gap one step-pace update may cover
OCCL_JUMP_M = (
    1.2  # a return this much nearer than its neighbours stands in front of them
)
OCCL_NEAR_M = 8.0  # only things this close cast a hidden area worth showing
OCCL_FAR_M = 10.0
OCCL_MAX_WIDTH_DEG = 60.0  # wider than this is a wall, not something to hide behind
OCCL_MAX_DEPTH_M = 2.5  # an object's own depth; more is two things
OCCL_MAX = 12

MM_PER_FT = 304.8

_QW = CAMERA_WIDTH / FRAME_GRID_COLS  # one camera's frame in the mosaic
_QH = CAMERA_HEIGHT / FRAME_GRID_ROWS
_FOCAL_PX = (_QW / 2.0) / math.tan(math.radians(SIMPLE_CAMERA_HFOV_DEG) / 2.0)
CAMERA_CENTRES = {"front": 0.0, "right": 90.0, "back": 180.0, "left": 270.0}

_PERSON = {"person", "persona", "pessoa"}
_BIKE = {"bicycle", "motorcycle", "motorbike", "cyclist"}
_VEHICLE = set(SIMPLE_VEHICLE_HEIGHT_M) - _BIKE
_ANIMAL = {
    "dog",
    "cat",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "bird",
    "animal",
    "bufflow",
}
_POLE = {
    "traffic light",
    "traffic_light",
    "stop sign",
    "parking meter",
    "fire hydrant",
    "fire_hydrant",
    "simple_pole",
    "street_pole",
    "double_pole",
    "pole_special",
    "traffic_sign",
    "street_sign",
    "delimitation_stake",
    "stopper",
}
_MOVERS = ("person", "bike", "vehicle", "animal")

# Typical heights for classes the guide's tables leave out, so the camera can put them at a range.
_EXTRA_HEIGHT_M = {
    "chair": 0.9,
    "couch": 0.85,
    "sofa": 0.85,
    "bed": 0.6,
    "dining table": 0.75,
    "diningtable": 0.75,
    "toilet": 0.75,
    "tv": 0.6,
    "tvmonitor": 0.6,
    "laptop": 0.25,
    "refrigerator": 1.75,
    "backpack": 0.5,
    "suitcase": 0.65,
    "handbag": 0.3,
    "umbrella": 1.0,
    "cat": 0.3,
    "bird": 0.25,
    "oven": 0.9,
    "microwave": 0.3,
    "sink": 0.25,
    "bottle": 0.25,
    "cup": 0.1,
    "book": 0.25,
    "clock": 0.3,
    "vase": 0.3,
    "teddy bear": 0.4,
    "sports ball": 0.22,
    "skateboard": 0.12,
    "surfboard": 1.8,
    "zebra": 1.4,
    "giraffe": 4.5,
    "kite": 0.8,
}
_DEFAULT_HEIGHT_M = 0.6

# How far one scan may move a tracked thing, and how deep its laser outline is.
_FOLLOW_M = {
    "person": 0.45,
    "bike": 0.7,
    "vehicle": 1.6,
    "animal": 0.45,
    "pole": 0.3,
    "other": 0.4,
}
_DEPTH_M = {
    "person": 0.4,
    "bike": 0.8,
    "vehicle": 2.5,
    "animal": 0.4,
    "pole": 0.3,
    "other": 0.5,
}


def _wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def group_of(cls: str) -> str:
    """Which figure the page draws for a detector class."""
    if cls in _PERSON:
        return "person"
    if cls in _BIKE:
        return "bike"
    if cls in _VEHICLE:
        return "vehicle"
    if cls in _ANIMAL:
        return "animal"
    if cls in _POLE:
        return "pole"
    return "other"


def class_height_m(cls: str) -> Optional[float]:
    h = (
        SIMPLE_VEHICLE_HEIGHT_M.get(cls)
        or SIMPLE_OBJECT_HEIGHT_M.get(cls)
        or _EXTRA_HEIGHT_M.get(cls)
    )
    return float(h) if h else None


def _pixel_angle(px: float) -> float:
    """Degrees right of the camera's axis for a column in its frame."""
    return math.degrees(math.atan((px - _QW / 2.0) / _FOCAL_PX))


def box_bearings(sector: str, box) -> Tuple[float, float, float, str, float]:
    """(centre, left edge, right edge bearing, which half of the frame, pixels right of centre)."""
    x1, _y1, x2, _y2 = (float(v) for v in box)
    centre = CAMERA_CENTRES[sector]
    cx = (x1 + x2) / 2.0
    half = "left" if cx < _QW / 2.0 else "right"
    return (
        centre + _pixel_angle(cx),
        centre + _pixel_angle(x1),
        centre + _pixel_angle(x2),
        half,
        cx - _QW / 2.0,
    )


def camera_range_m(cls: str, box) -> Tuple[Optional[float], bool]:
    """(range from the box height and the class's typical height, whether the frame cuts the box)."""
    _x1, y1, _x2, y2 = (float(v) for v in box)
    cut = y1 <= 2.0 or y2 >= _QH - 2.0
    h = class_height_m(cls)
    if not h:
        return None, cut
    return _FOCAL_PX * h / max(1.0, y2 - y1), cut


def camera_covering(bearing: float) -> Optional[str]:
    """The camera whose view holds this bearing, or None in the gap between two."""
    for sector, centre in CAMERA_CENTRES.items():
        if abs(_wrap180(bearing - centre)) <= SIMPLE_CAMERA_HFOV_DEG / 2.0:
            return sector
    return None


def describe_motion(
    vx: float,
    vy: float,
    obj: Tuple[float, float],
    user: Tuple[float, float],
    yaw_deg: float,
) -> str:
    """How a thing moves as the user would put it."""
    speed = math.hypot(vx, vy)
    if speed < STILL_MPS:
        return "still"
    dx, dy = obj[0] - user[0], obj[1] - user[1]
    rng = math.hypot(dx, dy) or 1.0
    radial = (vx * dx + vy * dy) / rng
    psi = math.radians(yaw_deg)
    lateral = vx * math.cos(psi) - vy * math.sin(psi)  # + = toward the user's right
    if abs(radial) >= abs(lateral):
        return "coming toward you" if radial < 0 else "moving away"
    return "crossing to your right" if lateral > 0 else "crossing to your left"


def zone_of(bearing: float) -> str:
    """One of the user's eight directions: front, front-left, left, behind-left, behind and the mirrors."""
    b = _wrap180(bearing)
    a, side = abs(b), ("right" if b > 0 else "left")
    if a <= 22.5:
        return "front"
    if a <= 67.5:
        return "front-" + side
    if a <= 112.5:
        return side
    if a <= 157.5:
        return "behind-" + side
    return "behind"


def alert_level(ttc_s: float) -> str:
    return (
        "urgent"
        if ttc_s <= ALERT_URGENT_S
        else "soon"
        if ttc_s <= ALERT_SOON_S
        else "info"
    )


def alert_sentence(group: str, cls: str, count: int, zone: str, seconds: int) -> str:
    """ "Vehicle approaching you in 5 seconds on your left" / "2 people in front, left side, approaching in 4 seconds"."""
    unit = "second" if seconds == 1 else "seconds"
    where = ZONE_TEXT[zone]
    if count == 1:
        name = {"person": "Person", "vehicle": "Vehicle"}.get(group) or cls.replace(
            "_", " "
        ).capitalize()
        return f"{name} approaching you in {seconds} {unit} {where}"
    return f"{count} {_PLURAL.get(group, 'things')} {where}, approaching in {seconds} {unit}"


def path_sentence(name: str, seconds: int) -> str:
    """ "Bottle in your path, 3 seconds ahead"."""
    return f"{name} in your path, {seconds} {'second' if seconds == 1 else 'seconds'} ahead"


def near_sentence(name: str, gap_m: float) -> str:
    """ "Chair in front of you, 1.6 meters" — for when the user is standing."""
    if gap_m < 0.5:
        return f"{name} right in front of you"
    return f"{name} in front of you, {gap_m:.1f} meters"


def near_level(gap_m: float) -> str:
    return (
        "urgent"
        if gap_m <= NEAR_URGENT_M
        else "soon"
        if gap_m <= NEAR_SOON_M
        else "info"
    )


def group_alerts(objects: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """One line per kind of thing per direction, most urgent first."""
    groups: Dict[Tuple[str, str], List[Dict[str, Any]]] = {}
    for o in objects:
        if o.get("approach"):
            groups.setdefault((o["group"], o["approach"]["zone"]), []).append(o)
    out = []
    for (group, zone), members in groups.items():
        first = min(members, key=lambda o: o["approach"]["ttc_s"])
        ttc = first["approach"]["ttc_s"]
        seconds = max(1, int(round(ttc)))
        out.append(
            {
                "key": f"{group}:{zone}",
                "group": group,
                "zone": zone,
                "count": len(members),
                "ttc_s": ttc,
                "seconds": seconds,
                "level": alert_level(ttc),
                "ids": [o["id"] for o in members],
                "text": alert_sentence(
                    group, first["cls"], len(members), zone, seconds
                ),
            }
        )
    for o in objects:
        th = o.get("threat") or {}
        if th.get("kind") != "path":
            continue
        if th.get("ttc_s") is not None:
            out.append(
                {
                    "key": f"path:{o['id']}",
                    "group": o["group"],
                    "zone": "front",
                    "count": 1,
                    "ttc_s": th["ttc_s"],
                    "seconds": th["seconds"],
                    "level": alert_level(th["ttc_s"]),
                    "ids": [o["id"]],
                    "text": path_sentence(o["name"], th["seconds"]),
                }
            )
        else:
            gap = th.get("gap_m") or 0.0
            out.append(
                {
                    "key": f"near:{o['id']}",
                    "group": o["group"],
                    "zone": "front",
                    "count": 1,
                    "ttc_s": None,
                    "seconds": None,
                    "dist_m": round(gap, 1),
                    "level": near_level(gap),
                    "ids": [o["id"]],
                    "text": near_sentence(o["name"], gap),
                }
            )
    out.sort(
        key=lambda a: (
            _LEVEL_ORDER[a["level"]],
            a["ttc_s"] if a["ttc_s"] is not None else 99.0,
            _ALERT_ORDER.get(a["group"], 9),
            a.get("dist_m", 0.0),
        )
    )
    return out


def _interp(hist: Deque[tuple], t: float, current: tuple) -> tuple:
    """Values at time t from a (t, *values) history, newest last."""
    if not hist or t >= hist[-1][0]:
        return current
    newer = None
    for i in range(len(hist) - 1, -1, -1):
        entry = hist[i]
        if entry[0] <= t:
            if newer is None:
                return entry[1:]
            span = newer[0] - entry[0]
            f = 0.0 if span <= 1e-9 else (t - entry[0]) / span
            return tuple(a + (b - a) * f for a, b in zip(entry[1:], newer[1:]))
        newer = entry
    return hist[0][1:]


def _push(hist: Deque[tuple], entry: tuple) -> None:
    if hist and entry[0] <= hist[-1][0]:
        hist[-1] = (hist[-1][0],) + tuple(entry[1:])
    else:
        hist.append(entry)
    while hist and hist[-1][0] - hist[0][0] > HISTORY_S:
        hist.popleft()


def _i16_b64(values: np.ndarray) -> str:
    """Centimetre offsets as little-endian int16, base64 — what the page decodes."""
    cm = np.clip(np.rint(values * 100.0), -32000, 32000).astype("<i2")
    return base64.b64encode(cm.tobytes()).decode("ascii")


class _Track:
    """One thing the cameras named, followed by the LiDAR between camera frames."""

    def __init__(self, tid: int, m: Dict[str, Any], t: float):
        self.id = tid
        self.born = t
        self.group = m["group"]
        self.hist: Deque[Tuple[float, float, float, float]] = (
            deque()
        )  # (t, x, y, 1 if a laser fix)
        self.x, self.y, self.t_pos = m["x"], m["y"], t
        self.t_laser: Optional[float] = None
        self.between = False
        self.dead = False
        self.misses = 0
        self.alert: Optional[Dict[str, Any]] = None
        self.alert_since: Optional[float] = None
        self.alert_t: Optional[float] = None
        self.label(m, t)
        self.move(t, m["x"], m["y"], laser=m["source"] == "laser")

    def label(self, m: Dict[str, Any], t: float) -> None:
        self.cls = m["cls"]
        self.conf = m["conf"]
        self.sector = m["sector"]
        self.half = m["half"]
        self.rough = m["rough"]
        self.cut = m["cut"]
        self.t_cam = t
        self.between = False
        self.misses = 0

    def move(self, t: float, x: float, y: float, laser: bool) -> None:
        self.x, self.y = x, y
        self.t_pos = max(t, self.t_pos)
        if laser:
            self.t_laser = t
        self.hist.append((t, x, y, 1.0 if laser else 0.0))
        while self.hist and t - self.hist[0][0] > TRACK_HIST_S:
            self.hist.popleft()

    def laser_recent(self, t: float, within: float = LASER_RECENT_S) -> bool:
        return self.t_laser is not None and (t - self.t_laser) <= within

    def laser_share(self) -> float:
        return float(np.mean([h[3] for h in self.hist])) if self.hist else 0.0

    def still_limit(self, rng: float) -> float:
        """Speed below which it counts as still; a camera-only range is rough, so far ones need more."""
        frac = LASER_STILL_FRAC if self.laser_share() >= 0.5 else CAMERA_STILL_FRAC
        return max(STILL_MPS, frac * rng)

    def velocity(self, now: Optional[float] = None) -> Optional[Tuple[float, float]]:
        """Median of pairwise slopes (Theil-Sen): one jump onto the wrong outline cannot fake a walk."""
        if len(self.hist) < TRACK_MIN_SAMPLES:
            return None
        h = np.asarray(self.hist, dtype=np.float64)
        min_age, min_span = _FAST_START.get(
            self.group, (TRACK_MIN_AGE_S, TRACK_MIN_SPAN_S)
        )
        if h[:, 0].max() - h[:, 0].min() < min_span:
            return None
        if now is not None and (
            now - h[:, 0].max() > TRACK_STALE_S or now - self.born < min_age
        ):
            return None
        dt = h[None, :, 0] - h[:, None, 0]
        ok = dt >= VEL_MIN_PAIR_S
        if np.count_nonzero(ok) < 10:
            return None
        return (
            float(np.median((h[None, :, 1] - h[:, None, 1])[ok] / dt[ok])),
            float(np.median((h[None, :, 2] - h[:, None, 2])[ok] / dt[ok])),
        )

    def predict(self, t: float) -> Tuple[float, float]:
        v = self.velocity()
        if v is None or math.hypot(v[0], v[1]) < STILL_MPS:
            return self.x, self.y
        dt = min(max(t - self.t_pos, 0.0), 0.5)
        return self.x + v[0] * dt, self.y + v[1] * dt


class LiveMap:
    """A rolling, floor-fixed map around the user, fed from the shared sensor state."""

    def __init__(self):
        self._lock = threading.RLock()
        self._thread: Optional[threading.Thread] = None
        self._clients = 0
        self._stop_after = 0.0  # monotonic: with no client, the worker stops after this
        # The gyro only once its axis and sign are commissioned; the page can still choose
        # it, to commission it (scan 2026-10-23).
        self.yaw_setting = "gyro" if GYRO_MOUNTING_CONFIRMED else "lidar"
        self.gyro_flip = False
        self._n = int(round(2 * GRID_HALF_M / CELL_M)) + 1
        self._h = self._n // 2
        self._error: Optional[str] = None
        self._error_count = 0
        self.reset()

    # ---- lifecycle --------------------------------------------------------

    def reset(self) -> None:
        """Start the map again: the user back at the origin, facing 0°."""
        with self._lock:
            self.x = self.y = self.yaw = 0.0
            self._yaw_hist: Deque[tuple] = deque()
            self._xy_hist: Deque[tuple] = deque()
            self._grid = np.zeros((self._n, self._n), dtype=np.float32)
            self._gc = [0.0, 0.0]
            self._tracks: Dict[int, _Track] = {}
            self._next_id = 1
            self._drops: Dict[str, int] = {}
            self._scan_key = None
            self._pending = None
            self._scan: Optional[Dict[str, Any]] = None
            self._heading_t = None
            self._gyro_t: Optional[float] = None
            self._gyro_live = False
            self._yaw_src = "none"
            self._measured: Deque[bool] = deque(maxlen=30)
            self._stepped: Deque[bool] = deque(maxlen=30)
            self._cam_seen: Dict[str, Any] = {}
            self._lens: Dict[str, Dict[str, Any]] = {
                s: {
                    "hfov": deque(maxlen=LENS_SAMPLES),
                    "offset": deque(maxlen=LENS_SAMPLES),
                    "mirror": 0,
                }
                for s in CAMERA_CENTRES
            }
            self._floor: Optional[Dict[str, Any]] = None
            self._floor_t = 0.0
            self._scans = 0
            self._steps = 0
            self._reset_at = time.time()

    def settings(self) -> Dict[str, Any]:
        with self._lock:
            return {"yaw_source": self.yaw_setting, "gyro_flip": self.gyro_flip}

    def configure(
        self, yaw_source: Optional[str] = None, gyro_flip: Optional[bool] = None
    ) -> Dict[str, Any]:
        """Change where the direction comes from; the map restarts, as it was drawn with the old one."""
        with self._lock:
            changed = False
            if yaw_source in ("gyro", "lidar") and yaw_source != self.yaw_setting:
                self.yaw_setting, changed = yaw_source, True
            if gyro_flip is not None and bool(gyro_flip) != self.gyro_flip:
                self.gyro_flip, changed = bool(gyro_flip), True
            if changed:
                self.reset()
            return self.settings()

    def _hold(self, seconds: float) -> None:
        self._stop_after = max(self._stop_after, time.monotonic() + seconds)

    def touch(self, hold_s: float = IDLE_STOP_S) -> None:
        with self._lock:
            self._hold(hold_s)
            self._ensure_worker()

    def acquire(self) -> None:
        with self._lock:
            self._clients += 1
            self._hold(IDLE_STOP_S)
            self._ensure_worker()

    def release(self) -> None:
        with self._lock:
            self._clients = max(0, self._clients - 1)
            self._hold(IDLE_STOP_S)

    def _ensure_worker(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(
                target=self._run, name="live-map", daemon=True
            )
            self._thread.start()

    def _run(self) -> None:
        while True:
            try:
                self.tick()
            except Exception as exc:
                with self._lock:
                    self._error = f"{type(exc).__name__}: {exc}"
                    self._error_count += 1
                    if (
                        self._error_count in (1, 10, 100)
                        or self._error_count % 1000 == 0
                    ):
                        print(f"[WARN] live map: {self._error} (x{self._error_count})")
            with self._lock:
                if self._clients <= 0 and time.monotonic() > self._stop_after:
                    self._thread = None
                    return
            time.sleep(POLL_S)

    # ---- one poll of the shared state -------------------------------------

    def tick(self, now: Optional[float] = None) -> None:
        now = time.time() if now is None else now
        heading = get_tracker().state()
        lidar = get_lidar_state()
        camera = get_camera_state()
        with self._lock:
            self._step_gyro(now)
            self._step_heading(heading)
            stamp = float(lidar.get("last_update") or 0.0)
            key = (lidar.get("scan_index"), stamp)
            if stamp and key != self._scan_key:
                self._scan_key = key
                self._pending = (stamp, lidar, time.monotonic())
            if self._pending is not None:
                h_at = heading.get("updated_at")
                ready = h_at is not None and h_at >= self._pending[0] - 1e-6
                if ready or time.monotonic() - self._pending[2] > SCAN_WAIT_S:
                    self._place_scan(self._pending[0], self._pending[1])
                    self._pending = None
            self._step_cameras(camera, now)

    def _step_gyro(self, now: float) -> None:
        """Turn the user by what the MPU6500 measured since the last poll."""
        if self.yaw_setting != "gyro":
            self._gyro_live, self._gyro_t = False, None
            return
        gyro = get_gyro()
        if not gyro.healthy(now):
            self._gyro_live, self._gyro_t = False, None
            return
        t1 = now - GYRO_LAG_S
        if self._gyro_t is None:
            self._gyro_t = t1
            return
        if t1 <= self._gyro_t:
            return
        turned = gyro.yaw_between(self._gyro_t, t1)
        if turned is None:
            if t1 - self._gyro_t > 0.5:  # a real gap: stop leaning on the gyro
                self._gyro_live, self._gyro_t = False, t1
            return  # else retry over the longer window next poll
        self._gyro_t = t1
        self._gyro_live = True
        self._yaw_src = "gyro"
        self.yaw += -turned if self.gyro_flip else turned
        _push(self._yaw_hist, (t1, self.yaw))

    def _step_heading(self, heading: Dict[str, Any]) -> None:
        """Walk the user by one scan of LiDAR odometry (and turn them by it when the gyro is not in use)."""
        t = heading.get("updated_at")
        if t is None or t == self._heading_t:
            return
        prev_t, self._heading_t = self._heading_t, t
        self._steps += 1
        if not self._gyro_live:
            step = heading.get("scan_yaw_deg")
            if step is not None and abs(step) <= 25.0:
                self.yaw += float(step)
                _push(self._yaw_hist, (t, self.yaw))
                self._yaw_src = "lidar"
            else:
                self._yaw_src = "none"
        fwd, right = heading.get("scan_fwd_mm"), heading.get("scan_right_mm")
        measured = fwd is not None or right is not None
        f, r = (fwd or 0.0) / 1000.0, (right or 0.0) / 1000.0
        if math.hypot(f, r) > MAX_STEP_M:
            f = r = 0.0
            measured = False
        stepped = False
        if not measured and heading.get("walking_confirmed") and prev_t is not None:
            # The LiDAR cannot see the walk here, but the accelerometer counts steps: walk on at that pace.
            f = float(heading.get("speed_mps") or 0.0) * min(
                max(t - prev_t, 0.0), STEP_DT_MAX_S
            )
            stepped = f > 0.0
        self._measured.append(measured)
        self._stepped.append(stepped)
        psi = math.radians(self._yaw_at(t))
        self.x += f * math.sin(psi) + r * math.cos(psi)
        self.y += f * math.cos(psi) - r * math.sin(psi)
        _push(self._xy_hist, (t, self.x, self.y))

    def _yaw_at(self, t: float) -> float:
        return _interp(self._yaw_hist, t, (self.yaw,))[0]

    def _pose_at(self, t: float) -> Tuple[float, float, float]:
        x, y = _interp(self._xy_hist, t, (self.x, self.y))
        return x, y, self._yaw_at(t)

    def _turn_rate(self, now: float) -> float:
        if not self._yaw_hist:
            return 0.0
        then = self._yaw_at(now - 0.3)
        return abs(self.yaw - then) / 0.3

    # ---- LiDAR ------------------------------------------------------------

    def _place_scan(self, t: float, lidar: Dict[str, Any]) -> None:
        pts = lidar.get("points") or {}
        ang = np.asarray(pts.get("angles") or [], dtype=np.float64)
        dist = np.asarray(pts.get("distances") or [], dtype=np.float64)
        n = min(len(ang), len(dist))
        ang, dist = ang[:n], dist[:n]
        ok = (dist > 0) & (dist <= LIDAR_MAX_DISTANCE)
        ang, d_m = ang[ok], dist[ok] / 1000.0
        if len(ang) == 0:
            return
        x, y, yaw = self._pose_at(t)
        th = np.radians(yaw + ang)
        wx = x + d_m * np.sin(th)
        wy = y + d_m * np.cos(th)
        self._scan = {
            "t": t,
            "ang": ang,
            "d": d_m,
            "wx": wx,
            "wy": wy,
            "x": x,
            "y": y,
            "yaw": yaw,
            "simulated": bool(lidar.get("simulated")),
        }
        movers = self._follow_tracks(t, wx, wy, d_m)
        self._scan["mover"] = movers
        self._scan["occl"] = self._occlusions(ang, d_m, x, y, yaw)
        self._update_grid(x, y, ang, wx, wy, ~movers)
        self._scans += 1

    def _follow_tracks(
        self, t: float, wx: np.ndarray, wy: np.ndarray, d_m: np.ndarray
    ) -> np.ndarray:
        """Move every tracked thing onto the laser outline nearest where it should be now."""
        free = np.ones(len(wx), dtype=bool)
        movers = np.zeros(len(wx), dtype=bool)
        sx, sy = self._scan["x"], self._scan["y"]
        order = sorted(
            self._tracks.values(), key=lambda k: math.hypot(k.x - sx, k.y - sy)
        )
        for tr in order:
            px, py = tr.predict(t)
            near = free & (np.hypot(wx - px, wy - py) <= _FOLLOW_M[tr.group])
            idx = np.nonzero(near)[0]
            if len(idx) >= 1:
                dn = d_m[idx]
                idx = idx[dn <= dn.min() + _DEPTH_M[tr.group]]
            if (
                len(idx) == 1
                and math.hypot(wx[idx[0]] - px, wy[idx[0]] - py) > SINGLE_POINT_M
            ):
                idx = idx[:0]
            if len(idx) < 1:
                if tr.between:
                    tr.dead = True  # between cameras, only the laser had it
                continue
            tr.move(t, float(wx[idx].mean()), float(wy[idx].mean()), laser=True)
            free[idx] = False
            if tr.group in _MOVERS:
                movers[idx] = True
        for tid in [k for k, tr in self._tracks.items() if tr.dead]:
            del self._tracks[tid]
            self._drops["laser lost it between cameras"] = (
                self._drops.get("laser lost it between cameras", 0) + 1
            )
        return movers

    @staticmethod
    def _occlusions(
        ang: np.ndarray, d: np.ndarray, x: float, y: float, yaw: float
    ) -> List[Dict[str, Any]]:
        """Hidden areas: the shadow behind each thing standing in front of the background."""
        n = len(ang)
        if n < 8:
            return []
        order = np.argsort(ang)
        a, r = ang[order], d[order]
        gap_after = np.diff(np.append(a, a[0] + 360.0))  # degrees to the next return

        def far_before(i):
            return gap_after[i - 1] > SCAN_GAP_DEG or r[i - 1] - r[i] > OCCL_JUMP_M

        def far_after(i):
            return gap_after[i] > SCAN_GAP_DEG or r[(i + 1) % n] - r[i] > OCCL_JUMP_M

        out = []
        for s in range(n):
            if not (0.3 <= r[s] <= OCCL_NEAR_M and far_before(s)):
                continue
            seg, width, e = [s], 0.0, None
            for k in range(1, n):
                j = (s + k - 1) % n
                if far_after(j):
                    e = j
                    break
                width += gap_after[j]
                nxt = (j + 1) % n
                if width > OCCL_MAX_WIDTH_DEG or far_before(nxt):
                    break
                seg.append(nxt)
            if e is None:
                continue
            rs = r[seg]
            if rs.max() - rs.min() > OCCL_MAX_DEPTH_M or rs.min() < 0.3:
                continue
            left = r[s - 1] if gap_after[s - 1] <= SCAN_GAP_DEG else OCCL_FAR_M
            right = r[(e + 1) % n] if gap_after[e] <= SCAN_GAP_DEG else OCCL_FAR_M
            r_far = min(OCCL_FAR_M, left, right)
            if r_far < rs.max() + 0.5:
                continue
            a0 = a[s]
            angs = [a0 + ((a[j] - a0) % 360.0) for j in seg]
            arc = np.arange(angs[-1], angs[0] - 1e-6, -4.0)
            if len(arc) == 0 or abs(arc[-1] - angs[0]) > 1e-6:
                arc = np.append(arc, angs[0])
            th = np.radians(yaw + np.concatenate([angs, arc]))
            rr = np.concatenate([rs, np.full(len(arc), r_far)])
            out.append(
                {
                    "wx": x + rr * np.sin(th),
                    "wy": y + rr * np.cos(th),
                    "near_m": float(rs.min()),
                    "far_m": float(r_far),
                    "bearing_deg": float(_wrap180((angs[0] + angs[-1]) / 2.0)),
                    "width_deg": float(angs[-1] - angs[0]),
                }
            )
        out.sort(key=lambda o: o["near_m"])
        return out[:OCCL_MAX]

    def _cells(self, wx: np.ndarray, wy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        cols = np.rint((wx - self._gc[0]) / CELL_M).astype(np.int64) + self._h
        rows = np.rint((wy - self._gc[1]) / CELL_M).astype(np.int64) + self._h
        return rows, cols

    def _recentre(self, x: float, y: float) -> None:
        dc = int(round((x - self._gc[0]) / CELL_M))
        dr = int(round((y - self._gc[1]) / CELL_M))
        n = self._n
        new = np.zeros_like(self._grid)
        sr0, sr1 = max(0, dr), min(n, n + dr)
        sc0, sc1 = max(0, dc), min(n, n + dc)
        if sr1 > sr0 and sc1 > sc0:
            dr0, dc0 = max(0, -dr), max(0, -dc)
            new[dr0 : dr0 + (sr1 - sr0), dc0 : dc0 + (sc1 - sc0)] = self._grid[
                sr0:sr1, sc0:sc1
            ]
        self._grid = new
        self._gc[0] += dc * CELL_M
        self._gc[1] += dr * CELL_M

    def _update_grid(
        self,
        x: float,
        y: float,
        ang: np.ndarray,
        wx: np.ndarray,
        wy: np.ndarray,
        stamp: np.ndarray,
    ) -> None:
        """Clear what the laser saw through, mark what it hit (people and vehicles are not walls)."""
        self._grid *= DECAY
        if math.hypot(x - self._gc[0], y - self._gc[1]) > RECENTRE_M:
            self._recentre(x, y)
        g, n = self._grid, self._n
        rows, cols = self._cells(wx, wy)
        if cv2 is not None and len(ang) >= 3:
            ur, uc = self._cells(np.array([x]), np.array([y]))
            order = np.argsort(ang)
            a, pr, pc = ang[order], rows[order], cols[order]
            gaps = np.diff(a) > SCAN_GAP_DEG
            verts = []
            for i in range(len(a)):
                if i > 0 and gaps[i - 1]:
                    verts.append((int(uc[0]), int(ur[0])))
                verts.append((int(pc[i]), int(pr[i])))
            if a[0] + 360.0 - a[-1] > SCAN_GAP_DEG:
                verts.append((int(uc[0]), int(ur[0])))
            poly = np.asarray(verts, dtype=np.int32)
            c0, c1 = max(0, int(poly[:, 0].min())), min(n - 1, int(poly[:, 0].max()))
            r0, r1 = max(0, int(poly[:, 1].min())), min(n - 1, int(poly[:, 1].max()))
            if c1 > c0 and r1 > r0:
                mask = np.zeros((r1 - r0 + 1, c1 - c0 + 1), dtype=np.uint8)
                cv2.fillPoly(mask, [poly - np.array([c0, r0], dtype=np.int32)], 1)
                g[r0 : r1 + 1, c0 : c1 + 1][mask.astype(bool)] -= L_FREE
        inside = stamp & (rows >= 0) & (rows < n) & (cols >= 0) & (cols < n)
        if inside.any():
            g.flat[np.unique(rows[inside] * n + cols[inside])] += L_HIT + L_FREE
        np.clip(g, L_MIN, L_MAX, out=g)

    # ---- cameras ----------------------------------------------------------

    def _step_cameras(self, camera: Dict[str, Any], now: float) -> None:
        for sector in CAMERA_CENTRES:
            entry = camera.get(sector) or {}
            ft = entry.get("frame_time")
            if (
                not ft
                or (now - float(ft)) > SIMPLE_CAMERA_FRESH_S
                or entry.get("simulated")
            ):
                self._drop_sector(
                    sector
                )  # a dead or simulated camera vouches for nothing
                continue
            if ft == self._cam_seen.get(sector):
                continue
            self._cam_seen[sector] = ft
            self._ingest(sector, entry.get("detections") or [], float(ft), now)

    def _drop_sector(self, sector: str) -> None:
        for tid in [
            k
            for k, tr in self._tracks.items()
            if tr.sector == sector and not tr.between
        ]:
            del self._tracks[tid]
            self._drops["camera stale or simulated"] = (
                self._drops.get("camera stale or simulated", 0) + 1
            )

    def _ingest(
        self, sector: str, detections: List[Dict[str, Any]], ft: float, now: float
    ) -> None:
        """Place one camera frame's detections and match them to what is already tracked."""
        fx, fy, fyaw = self._pose_at(ft)
        scan = self._scan if (self._scan and abs(self._scan["t"] - ft) <= 0.5) else None
        meas = []
        for det in detections:
            cls = det.get("class_name")
            box = det.get("bbox")
            if not cls or not isinstance(box, (list, tuple)) or len(box) != 4:
                continue
            centre, b_lo, b_hi, half, rel_px = box_bearings(sector, box)
            est, cut = camera_range_m(cls, box)
            group = group_of(cls)
            m = {
                "cls": cls,
                "group": group,
                "conf": float(det.get("confidence") or 0.0),
                "sector": sector,
                "half": half,
                "cut": cut,
                "rough": est is None or cut,
                "heading": fyaw + centre,
                "half_deg": abs(b_hi - b_lo) / 2.0,
            }
            hit = None
            if scan is not None:
                hit = self._pick_cluster(
                    self._laser_clusters(scan, b_lo, b_hi, fyaw, est, cut, group),
                    group,
                    est,
                    cut,
                    now,
                )
            if hit is not None:
                m.update(x=hit[0], y=hit[1], source="laser")
                self._lens_sample(sector, rel_px, centre, hit[2] - (fyaw - scan["yaw"]))
            else:
                rng = (
                    est
                    if est is not None
                    else _FOCAL_PX
                    * _DEFAULT_HEIGHT_M
                    / max(1.0, float(box[3]) - float(box[1]))
                )
                th = math.radians(fyaw + centre)
                m.update(
                    x=fx + rng * math.sin(th),
                    y=fy + rng * math.cos(th),
                    source="camera",
                )
            m["range"] = math.hypot(m["x"] - fx, m["y"] - fy)
            meas.append(m)

        pairs = []
        for mi, m in enumerate(meas):
            for tid, tr in self._tracks.items():
                if tr.group != m["group"]:
                    continue
                px, py = tr.predict(ft)
                if m["source"] == "laser":
                    gap = math.hypot(m["x"] - px, m["y"] - py)
                    gate = MATCH_GATE_M + MATCH_GATE_MPS * min(
                        max(ft - tr.t_pos, 0.0), 1.0
                    )
                    if gap <= gate:
                        pairs.append((gap, mi, tid))
                    continue
                # A camera-only range is rough but its bearing is good: match on bearing, loosely on range.
                t_rng = math.hypot(px - fx, py - fy)
                if t_rng < 1e-3:
                    continue
                off = abs(
                    _wrap180(m["heading"] - math.degrees(math.atan2(px - fx, py - fy)))
                )
                ratio = m["range"] / t_rng
                if (
                    off <= max(CAMERA_BEARING_TOL_DEG, m["half_deg"])
                    and 0.5 <= ratio <= 2.0
                ):
                    pairs.append(
                        (
                            math.radians(off) * t_rng + 0.2 * abs(math.log(ratio)),
                            mi,
                            tid,
                        )
                    )
        pairs.sort()
        used_m, used_t = set(), set()
        for _gap, mi, tid in pairs:
            if mi in used_m or tid in used_t:
                continue
            used_m.add(mi)
            used_t.add(tid)
            tr, m = self._tracks[tid], meas[mi]
            tr.label(m, ft)
            if m["source"] == "laser":
                if ft >= tr.t_pos - 0.05:
                    tr.move(ft, m["x"], m["y"], laser=True)
            elif not tr.laser_recent(now, LASER_KEEP_S) and ft >= tr.t_pos - 0.05:
                # Camera only: take its bearing, but ease toward its rough range.
                old = math.hypot(tr.x - fx, tr.y - fy)
                rng = old + CAMERA_RANGE_BLEND * (m["range"] - old)
                th = math.radians(m["heading"])
                tr.move(
                    ft, fx + rng * math.sin(th), fy + rng * math.cos(th), laser=False
                )
        for mi, m in enumerate(meas):
            if mi not in used_m:
                self._tracks[self._next_id] = _Track(self._next_id, m, ft)
                used_t.add(self._next_id)
                self._next_id += 1

        ux, uy = self.x, self.y
        for tid, tr in list(self._tracks.items()):
            if tid in used_t:
                continue
            bearing = math.degrees(math.atan2(tr.x - ux, tr.y - uy)) - self.yaw
            covered_by = camera_covering(bearing)
            mine = tr.sector == sector or (tr.between and covered_by == sector)
            if not mine:
                continue
            if covered_by != sector and tr.laser_recent(now):
                tr.between = True  # it left this camera's view; the laser still has it
            elif tr.misses < (CAMERA_MISSES_WITH_LASER if tr.laser_recent(now) else 1):
                tr.misses += 1  # the detector missed it; the laser (if it still has it) vouches for it
            else:
                del self._tracks[tid]  # this camera looked and it is not there
                why = "camera no longer sees it" + (
                    "" if tr.laser_recent(now) else " (laser lost it too)"
                )
                self._drops[why] = self._drops.get(why, 0) + 1

    def _laser_clusters(
        self,
        scan: Dict[str, Any],
        b_lo: float,
        b_hi: float,
        fyaw: float,
        est: Optional[float],
        cut: bool,
        group: str,
    ) -> List[Tuple[float, float, float, float, int]]:
        """Every laser outline inside a camera box whose range fits its size: (world x, y, scan bearing, range, returns)."""
        shift = fyaw - scan["yaw"]
        centre = (b_lo + b_hi) / 2.0 + shift
        half = max(1.0, (b_hi - b_lo) * 0.35)
        rel = (scan["ang"] - centre + 180.0) % 360.0 - 180.0
        d = scan["d"]
        lo = 0.2 if (cut or est is None) else est / SIMPLE_VEHICLE_RANGE_MATCH
        hi = (
            LIDAR_MAX_DISTANCE / 1000.0
            if est is None
            else est * SIMPLE_VEHICLE_RANGE_MATCH
        )
        idx = np.nonzero((np.abs(rel) <= half) & (d >= lo) & (d <= hi))[0]
        if len(idx) == 0:
            return []
        idx = idx[np.argsort(d[idx])]
        depth = _DEPTH_M[group]
        out, first = [], 0
        for i in range(1, len(idx) + 1):
            if i == len(idx) or d[idx[i]] - d[idx[first]] > depth:
                sel = idx[first:i]
                a = np.radians(scan["ang"][sel])
                bearing = math.degrees(math.atan2(np.sin(a).mean(), np.cos(a).mean()))
                out.append(
                    (
                        float(scan["wx"][sel].mean()),
                        float(scan["wy"][sel].mean()),
                        bearing,
                        float(np.median(d[sel])),
                        len(sel),
                    )
                )
                first = i
        return out

    def _pick_cluster(
        self,
        clusters: List[Tuple[float, float, float, float, int]],
        group: str,
        est: Optional[float],
        cut: bool,
        now: float,
    ) -> Optional[Tuple[float, float, float, float, int]]:
        """The outline that is this thing: the one a tracked thing already stands on, else the one its size fits."""
        if not clusters:
            return None
        best, best_gap = None, CONTINUITY_M
        for tr in self._tracks.values():
            if tr.group != group or not tr.laser_recent(now, LASER_KEEP_S):
                continue
            px, py = tr.predict(now)
            for c in clusters:
                gap = math.hypot(c[0] - px, c[1] - py)
                if gap < best_gap:
                    best, best_gap = c, gap
        if best is not None:
            return best
        clusters = [
            c for c in clusters if c[4] >= LASER_MIN_POINTS
        ]  # a lone return starts nothing
        if not clusters:
            return None
        if est is None or cut:
            return min(clusters, key=lambda c: c[3])  # size unknown: the nearest
        return min(clusters, key=lambda c: abs(math.log(c[3] / est)))

    def _lens_sample(
        self, sector: str, rel_px: float, cam_bearing: float, laser_bearing: float
    ) -> None:
        """Compare where the camera and the laser put the same thing, to check the assumed lens."""
        lens = self._lens[sector]
        centre = CAMERA_CENTRES[sector]
        laser_rel = _wrap180(laser_bearing - centre)
        if abs(rel_px) > _QW / 6.0 and abs(laser_rel) > 2.0:
            focal = rel_px / math.tan(math.radians(laser_rel))
            if focal > 0:
                lens["hfov"].append(2.0 * math.degrees(math.atan((_QW / 2.0) / focal)))
            else:
                lens["mirror"] += 1
        if abs(rel_px) < _QW / 12.0:
            lens["offset"].append(_wrap180(laser_bearing - cam_bearing))

    # ---- radar ------------------------------------------------------------

    def _radar_boards(self, radar: Dict[str, Any]) -> List[Dict[str, Any]]:
        sectors = radar.get("sectors") or {}
        out, seen = [], []
        for b in radar.get("boards") or []:
            if b.get("bearing") is None:
                continue
            bearing = float(b["bearing"]) % 360.0
            sec = sectors.get(b.get("position")) or {}
            online = bool(sec.get("online")) and not sec.get("stale")
            mn = sec.get("min_dist") if online else None
            peaks = (
                [
                    float(p) / 1000.0
                    for p in (sec.get("peaks_mm") or [])
                    if isinstance(p, (int, float)) and p > 0
                ]
                if online
                else []
            )
            out.append(
                {
                    "bearing": bearing,
                    "beam_deg": radar_beam_deg(bearing),
                    "online": online,
                    "degraded": bool(sec.get("degraded")),
                    "min_m": round(float(mn) / 1000.0, 3) if mn else None,
                    "peaks_m": [round(p, 3) for p in peaks],
                    "missing": False,
                }
            )
            seen.append(bearing)
        for bearing in radar_ring():
            if not any(abs(_wrap180(bearing - s)) < 0.5 for s in seen):
                out.append(
                    {
                        "bearing": bearing % 360.0,
                        "beam_deg": radar_beam_deg(bearing),
                        "online": False,
                        "degraded": False,
                        "min_m": None,
                        "peaks_m": [],
                        "missing": True,
                    }
                )
        scan = (
            self._scan
            if (self._scan and time.time() - self._scan["t"] <= SCAN_FRESH_S)
            else None
        )
        for b in out:
            b["laser_min_m"] = None
            b["glass_hint"] = False
            if scan is None:
                continue
            rel = (scan["ang"] - b["bearing"] + 180.0) % 360.0 - 180.0
            inbeam = scan["d"][np.abs(rel) <= b["beam_deg"] / 2.0]
            lmin = float(inbeam.min()) if len(inbeam) else None
            b["laser_min_m"] = round(lmin, 3) if lmin is not None else None
            if b["min_m"] is not None:
                b["glass_hint"] = (
                    lmin is None or b["min_m"] < lmin - RADAR_GLASS_GAP_MM / 1000.0
                )
        return out

    @staticmethod
    def _radar_agrees(
        boards: List[Dict[str, Any]], bearing: float, dist_m: float
    ) -> Optional[bool]:
        """Does a radar board facing this thing hear an echo at its distance? None when none can."""
        mm = dist_m * 1000.0
        if not (RADAR_START_MM <= mm <= RADAR_END_MM):
            return None
        covering = [
            b
            for b in boards
            if b["online"]
            and abs(_wrap180(bearing - b["bearing"])) <= b["beam_deg"] / 2.0
        ]
        if not covering:
            return None
        for b in covering:
            echoes = list(b["peaks_m"]) + ([b["min_m"]] if b["min_m"] else [])
            if any(abs(e * 1000.0 - mm) <= RADAR_AGREE_MM for e in echoes):
                return True
        return False

    # ---- approach times ---------------------------------------------------

    def _user_velocity(self, now: float) -> Tuple[float, float]:
        """The user's own walking velocity over the last second, from the odometry steps."""
        if not self._xy_hist or now - self._xy_hist[-1][0] > 1.0:
            return 0.0, 0.0
        h = [
            e for e in self._xy_hist if self._xy_hist[-1][0] - e[0] <= USER_VEL_WINDOW_S
        ]
        if len(h) < 3:
            return 0.0, 0.0
        a = np.asarray(h, dtype=np.float64)
        ts = a[:, 0] - a[:, 0].mean()
        var = float((ts * ts).sum())
        if var <= 1e-9:
            return 0.0, 0.0
        return (
            float((ts * (a[:, 1] - a[:, 1].mean())).sum() / var),
            float((ts * (a[:, 2] - a[:, 2].mean())).sum() / var),
        )

    def _approach(
        self,
        tr: _Track,
        v: Optional[Tuple[float, float]],
        ox: float,
        oy: float,
        uv: Tuple[float, float],
        now: float,
        turning: bool,
    ) -> Optional[Dict[str, Any]]:
        """When a moving thing reaches the user, if it is on course to pass within reach (else None)."""
        floor = ALERT_MIN_SPEED_MPS.get(tr.group)
        if floor is None:
            return None
        floor = max(floor, tr.still_limit(math.hypot(tr.x - ox, tr.y - oy)))
        if turning:
            # The world swings while the user turns: carry a standing alert on, counting down.
            if tr.alert is not None:
                tr.alert["ttc_s"] = max(0.0, tr.alert["ttc_s"] - (now - tr.alert_t))
                tr.alert_t = now
            return tr.alert
        hit = None
        if v is not None and math.hypot(v[0], v[1]) >= floor:
            px, py = tr.x - ox, tr.y - oy
            rvx, rvy = v[0] - uv[0], v[1] - uv[1]
            rng, rv2 = math.hypot(px, py), rvx * rvx + rvy * rvy
            if rng > 1e-6 and rv2 > 1e-6:
                dot = px * rvx + py * rvy
                closing, t_cpa = -dot / rng, -dot / rv2
                miss = math.hypot(px + rvx * t_cpa, py + rvy * t_cpa)
                keep = ALERT_KEEP if tr.alert is not None else 1.0
                if (
                    closing >= ALERT_MIN_CLOSING_MPS / keep
                    and 0.0 < t_cpa <= ALERT_HORIZON_S * keep
                    and miss <= ALERT_MISS_M[tr.group] * keep
                ):
                    hit = (t_cpa, closing, miss)
        if hit is None:
            tr.alert = tr.alert_since = tr.alert_t = None
            return None
        t_cpa, closing, miss = hit
        if tr.alert is None:
            tr.alert_since, ttc = now, t_cpa
        else:
            # Count the last estimate down, then lean it toward the new one: no lag on a steady approach.
            dt = max(0.0, now - tr.alert_t)
            pred = tr.alert["ttc_s"] - dt
            ttc = pred + (1.0 - math.exp(-dt / ALERT_SMOOTH_S)) * (t_cpa - pred)
        tr.alert_t = now
        tr.alert = {
            "ttc_s": max(0.0, ttc),
            "closing_mps": closing,
            "miss_m": miss,
            "confirmed": now - tr.alert_since >= ALERT_CONFIRM_S,
        }
        return tr.alert

    @staticmethod
    def _projected_path(uv: Tuple[float, float], yaw: float) -> Dict[str, Any]:
        """Where the user is heading: along their walking velocity, or the way they face when standing."""
        speed = math.hypot(uv[0], uv[1])
        walking = speed >= WALKING_MPS
        heading = math.degrees(math.atan2(uv[0], uv[1])) if walking else yaw
        return {
            "heading_deg": round(heading, 1),
            "half_m": round(PATH_HALF_M, 2),
            "length_m": round(
                min(PATH_MAX_M, max(PATH_MIN_M, speed * ALERT_HORIZON_S)), 2
            ),
            "speed_mps": round(speed, 2),
            "walking": walking,
        }

    @staticmethod
    def _threat(
        tr: _Track,
        v: Optional[Tuple[float, float]],
        appr: Optional[Dict[str, Any]],
        px: float,
        py: float,
        path: Dict[str, Any],
        laser: bool,
    ) -> Dict[str, Any]:
        """Level 1 path conflict, 2 high potential threat, 3 monitor."""
        if appr is not None:
            return {
                "level": 1,
                "kind": "approach",
                "reason": "on course to reach you",
                "ttc_s": appr["ttc_s"],
                "seconds": appr["seconds"],
            }
        h = math.radians(path["heading_deg"])
        dx, dy = math.sin(h), math.cos(h)
        along, lateral = px * dx + py * dy, px * dy - py * dx
        rad = _RADIUS_M.get(tr.group, 0.3)
        speed = math.hypot(*v) if v else 0.0
        if (
            0.0 < along <= path["length_m"] + rad
            and abs(lateral) <= path["half_m"] + rad
        ):
            gap = max(
                0.0, along - (0.0 if laser else rad)
            )  # a laser fix is already the near surface
            closing = path["speed_mps"] - ((v[0] * dx + v[1] * dy) if v else 0.0)
            if path["walking"] and closing >= WALKING_MPS:
                t = gap / closing
                if t <= ALERT_HORIZON_S:
                    return {
                        "level": 1,
                        "kind": "path",
                        "reason": "in your path",
                        "gap_m": round(gap, 2),
                        "ttc_s": round(t, 1),
                        "seconds": max(1, int(round(t))),
                    }
            return {
                "level": 1,
                "kind": "path",
                "reason": "in your path",
                "gap_m": round(gap, 2),
                "ttc_s": None,
                "seconds": None,
            }
        if tr.group in ("vehicle", "bike"):
            why = (
                "moving vehicle, could turn into your path"
                if speed >= ALERT_MIN_SPEED_MPS[tr.group]
                else "vehicle that could move off"
            )
            return {
                "level": 2,
                "kind": "watch",
                "reason": why,
                "ttc_s": None,
                "seconds": None,
            }
        if (tr.group == "animal" and speed >= ALERT_MIN_SPEED_MPS["animal"]) or (
            tr.group == "person" and speed >= FAST_PERSON_MPS
        ):
            return {
                "level": 2,
                "kind": "watch",
                "reason": "moving fast",
                "ttc_s": None,
                "seconds": None,
            }
        return {
            "level": 3,
            "kind": "monitor",
            "reason": "outside your path",
            "ttc_s": None,
            "seconds": None,
        }

    # ---- what the page gets -----------------------------------------------

    def snapshot(self, include_map: bool = False) -> Dict[str, Any]:
        now = time.time()
        radar = get_radar_state()
        lidar = get_lidar_state()
        camera = get_camera_state()
        try:
            gyro = get_gyro().state()
        except Exception as exc:
            gyro = {"healthy": False, "last_error": str(exc)}
        try:
            from utils.sensor_fusion import lidar_is_fresh

            lidar_fresh = bool(lidar_is_fresh(lidar, now))
        except Exception:
            lidar_fresh = False
        lidar_stamp = float(lidar.get("last_update") or 0.0)

        with self._lock:
            ox, oy, yaw = self.x, self.y, self.yaw
            boards = self._radar_boards(radar)
            turning = self._turn_rate(now) > TURNING_DPS

            scan = (
                self._scan
                if (self._scan and now - self._scan["t"] <= SCAN_FRESH_S)
                else None
            )
            if scan is not None:
                rel = np.empty(2 * len(scan["wx"]))
                rel[0::2], rel[1::2] = scan["wx"] - ox, scan["wy"] - oy
                live = {
                    "points": _i16_b64(rel),
                    "mover": [int(i) for i in np.nonzero(scan["mover"])[0]],
                    "n": int(len(scan["wx"])),
                }
            else:
                live = {"points": "", "mover": [], "n": 0}

            objects = []
            uv = self._user_velocity(now)
            path = self._projected_path(uv, yaw)
            for tr in self._tracks.values():
                dx, dy = tr.x - ox, tr.y - oy
                dist = math.hypot(dx, dy)
                bearing = _wrap180(math.degrees(math.atan2(dx, dy)) - yaw)
                v = tr.velocity(now)
                motion = None
                limit = tr.still_limit(dist)
                if v is not None and not turning:
                    motion = (
                        "still"
                        if math.hypot(v[0], v[1]) < limit
                        else describe_motion(v[0], v[1], (tr.x, tr.y), (ox, oy), yaw)
                    )
                appr = self._approach(tr, v, ox, oy, uv, now, turning)
                if appr is not None and appr["confirmed"]:
                    appr = {
                        "ttc_s": round(appr["ttc_s"], 1),
                        "seconds": max(1, int(round(appr["ttc_s"]))),
                        "closing_mps": round(appr["closing_mps"], 2),
                        "miss_m": round(appr["miss_m"], 2),
                        "zone": zone_of(bearing),
                        "level": alert_level(appr["ttc_s"]),
                    }
                else:
                    appr = None
                objects.append(
                    {
                        "id": tr.id,
                        "cls": tr.cls,
                        "name": tr.cls.replace("_", " ").capitalize(),
                        "group": tr.group,
                        "x": round(dx, 3),
                        "y": round(dy, 3),
                        "dist_m": round(dist, 2),
                        "dist_ft": round(dist * 1000.0 / MM_PER_FT, 1),
                        "bearing_deg": round(bearing, 1),
                        "clock": int(round((bearing % 360.0) / 30.0)) % 12 or 12,
                        "source": "laser" if tr.laser_recent(now) else "camera",
                        "rough": tr.rough and not tr.laser_recent(now),
                        "cut": tr.cut,
                        "camera": tr.sector,
                        "half": tr.half,
                        "between": tr.between,
                        "conf": round(tr.conf, 2),
                        "vx": round(v[0], 2) if v else None,
                        "vy": round(v[1], 2) if v else None,
                        "speed_mps": round(math.hypot(*v), 2) if v else None,
                        "motion": motion,
                        "radar": self._radar_agrees(boards, bearing, dist),
                        "seen_s": round(now - tr.t_cam, 2),
                        "approach": appr,
                        "threat": self._threat(
                            tr, v, appr, dx, dy, path, tr.laser_recent(now)
                        ),
                    }
                )
            objects.sort(key=lambda o: o["dist_m"])
            alerts = group_alerts(objects)
            levels = {
                str(k): sum(1 for o in objects if o["threat"]["level"] == k)
                for k in (1, 2, 3)
            }
            occluded = []
            for oc in (scan or {}).get("occl") or []:
                rx, ry = oc["wx"] - ox, oc["wy"] - oy
                h = math.radians(path["heading_deg"])
                along = rx * math.sin(h) + ry * math.cos(h)
                lateral = rx * math.cos(h) - ry * math.sin(h)
                beside = (
                    (along > -1.0)
                    & (along <= path["length_m"] + 3.0)
                    & (np.abs(lateral) <= path["half_m"] + 1.5)
                )
                flat = np.empty(2 * len(rx))
                flat[0::2], flat[1::2] = rx, ry
                occluded.append(
                    {
                        "poly": [round(float(c), 2) for c in flat],
                        "near_m": round(oc["near_m"], 2),
                        "far_m": round(oc["far_m"], 2),
                        "bearing_deg": round(
                            _wrap180(oc["bearing_deg"] + scan["yaw"] - yaw), 1
                        ),
                        "borders_path": bool(beside.any()),
                    }
                )

            measured = (
                (sum(self._measured) / len(self._measured)) if self._measured else None
            )
            cams = {}
            for sector, centre in CAMERA_CENTRES.items():
                entry = camera.get(sector) or {}
                ft = entry.get("frame_time")
                lens = self._lens[sector]
                cams[sector] = {
                    "centre_deg": centre,
                    "age_s": round(now - float(ft), 2) if ft else None,
                    "fresh": bool(ft) and (now - float(ft)) <= SIMPLE_CAMERA_FRESH_S,
                    "simulated": bool(entry.get("simulated")),
                    "detections": len(entry.get("detections") or []),
                    "lens_hfov_deg": round(float(np.median(lens["hfov"])), 1)
                    if lens["hfov"]
                    else None,
                    "lens_samples": len(lens["hfov"]),
                    "aim_offset_deg": round(float(np.median(lens["offset"])), 1)
                    if lens["offset"]
                    else None,
                    "aim_samples": len(lens["offset"]),
                    "mirror_votes": lens["mirror"],
                }

            payload = {
                "t": now,
                "origin": {"x": round(ox, 3), "y": round(oy, 3)},
                "pose": {
                    "x": round(ox, 3),
                    "y": round(oy, 3),
                    "yaw_deg": round(yaw, 2),
                    "facing_deg": round(_wrap180(yaw), 1),
                    "yaw_source": self._yaw_src,
                    "turning": turning,
                    "position_measured": round(measured, 2)
                    if measured is not None
                    else None,
                    "position_source": (
                        "measured"
                        if measured is not None and measured >= 0.5
                        else "steps"
                        if self._stepped
                        and sum(self._stepped) >= 0.3 * len(self._stepped)
                        else "none"
                    ),
                    "settings": self.settings(),
                },
                "lidar": {
                    "fresh": lidar_fresh,
                    "age_s": round(now - lidar_stamp, 2) if lidar_stamp else None,
                    "hz": round(float(lidar.get("rpm") or 0.0) / 60.0, 1),
                    "simulated": bool(lidar.get("simulated")),
                    **live,
                },
                "radar": {
                    "boards": boards,
                    "start_m": RADAR_START_MM / 1000.0,
                    "reach_m": RADAR_END_MM / 1000.0,
                },
                "cameras": {
                    "hfov_deg": SIMPLE_CAMERA_HFOV_DEG,
                    "hfov_assumed": True,
                    "sectors": cams,
                },
                "gyro": {
                    "healthy": bool(gyro.get("healthy")),
                    "chip": gyro.get("chip"),
                    "rate_hz": gyro.get("sample_rate_hz"),
                    "bias_trusted": bool(gyro.get("bias_trusted")),
                    "mounting_confirmed": bool(GYRO_MOUNTING_CONFIRMED),
                    "last_error": gyro.get("last_error"),
                },
                "objects": objects,
                "alerts": alerts,
                "levels": levels,
                "path": path,
                "occluded": occluded,
                "audio_enabled": bool(AUDIO_ENABLED),
                "user_velocity": {"vx": round(uv[0], 2), "vy": round(uv[1], 2)},
                "geometry": {
                    "block_m": round(WALK_BLOCK_FT * MM_PER_FT / 1000.0, 2),
                    "keep_m": KEEP_M,
                    "lidar_max_m": LIDAR_MAX_DISTANCE / 1000.0,
                },
                "stats": {
                    "scans": self._scans,
                    "steps": self._steps,
                    "tracks": len(self._tracks),
                    "dropped": dict(self._drops),
                    "created": self._next_id - 1,
                    "error": self._error,
                    "errors": self._error_count,
                    "since_reset_s": round(now - self._reset_at, 1),
                },
            }
            if include_map:
                payload["map"] = self._map_payload(ox, oy, now)
        return payload

    def _map_payload(self, ox: float, oy: float, now: float) -> Dict[str, Any]:
        """The remembered walls (and, once a second, the seen-empty floor) within KEEP_M of the user."""
        g, h = self._grid, self._h
        rows, cols = np.nonzero(g > OCC_L)
        wx = self._gc[0] + (cols - h) * CELL_M - ox
        wy = self._gc[1] + (rows - h) * CELL_M - oy
        rng = np.hypot(wx, wy)
        keep = rng <= KEEP_M
        wx, wy, rng = wx[keep], wy[keep], rng[keep]
        if len(wx) > WALL_CELLS_MAX:
            nearest = np.argsort(rng)[:WALL_CELLS_MAX]
            wx, wy = wx[nearest], wy[nearest]
        flat = np.empty(2 * len(wx))
        flat[0::2], flat[1::2] = wx, wy
        out = {"cell_m": CELL_M, "walls": _i16_b64(flat), "n": int(len(wx))}
        if cv2 is not None and (
            self._floor is None or now - self._floor_t >= FLOOR_EVERY_S
        ):
            self._floor = self._floor_png(ox, oy)
            self._floor_t = now
        if self._floor is not None:
            out["floor"] = {
                **self._floor,
                "cx": round(self._floor["gx"] - ox, 3),
                "cy": round(self._floor["gy"] - oy, 3),
            }
        return out

    def _floor_png(self, ox: float, oy: float) -> Optional[Dict[str, Any]]:
        n2 = (self._n // 2) * 2
        g = self._grid[:n2, :n2].reshape(n2 // 2, 2, n2 // 2, 2).min(axis=(1, 3))
        m = g.shape[0]
        cell = 2 * CELL_M
        # World position of the coarse grid's centre (cell edges lie half a fine cell off).
        gx = self._gc[0] + (m / 2.0) * cell - self._h * CELL_M - CELL_M / 2.0
        gy = self._gc[1] + (m / 2.0) * cell - self._h * CELL_M - CELL_M / 2.0
        yy, xx = np.mgrid[0:m, 0:m]
        cxw = gx + (xx + 0.5 - m / 2.0) * cell - ox
        cyw = gy + (yy + 0.5 - m / 2.0) * cell - oy
        near = np.hypot(cxw, cyw) <= KEEP_M
        img = np.zeros((m, m, 4), dtype=np.uint8)  # BGRA
        free = (g < FREE_L) & near
        img[free] = (70, 92, 40, 150)
        ok, buf = cv2.imencode(
            ".png", np.flipud(img)
        )  # image top = forward of the start
        if not ok:
            return None
        return {
            "png": base64.b64encode(buf.tobytes()).decode("ascii"),
            "id": round(time.time(), 3),
            "size_m": round(m * cell, 3),
            "gx": gx,
            "gy": gy,
        }


_map: Optional[LiveMap] = None
_map_lock = threading.Lock()


def get_map() -> LiveMap:
    global _map
    with _map_lock:
        if _map is None:
            _map = LiveMap()
        return _map
