"""Simplified walking guidance — a short, closed vocabulary, warned fast and cleared slowly."""

import math
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from config import (
    GUIDANCE_CONE_CENTRES, GUIDANCE_CONE_DEG, GUIDANCE_CORRIDOR_HALF_WIDTH_MM,
    GUIDANCE_CORRIDOR_MAX_MM, LIDAR_CLUSTER_WINDOW_MM, LIDAR_STALE_S,
    NOTIFY_WORDS_PER_SEC, SIMPLE_GLASS_LOSS_CONFIRM_S,
    SIMPLE_GLASS_RECOVER_CONFIRM_S, SIMPLE_GLASS_RECOVER_MAX_S,
    SIMPLE_GLASS_STARTUP_GRACE_S,
    SIMPLE_ALERT_CONFIRM_S, SIMPLE_ALERT_GAP_S, SIMPLE_BAND_MARGIN_FT,
    SIMPLE_CLEAR_CONFIRM_S, SIMPLE_CROSSING_HOLD_S, SIMPLE_GLASS_RELEASE_S,
    SIMPLE_CONFIRM_S, SIMPLE_LIDAR_FRESH_S, SIMPLE_MIN_GAP_S, SIMPLE_ON_WALL_MM,
    SIMPLE_REPEAT_MAX_S, SIMPLE_REPEAT_S, SIMPLE_WALL_APPROACH_FT,
    SIMPLE_WALL_MATCH_FT, WALK_BLOCK_FT, WALK_GLASS_GAP_FT, WALK_MIN_TURN_FT,
    WALK_TURN_HYSTERESIS_FT,
    CAMERA_HEIGHT, CAMERA_WIDTH, DRIFT_WALL_SIDE_MAX_DEG,
    DRIFT_WALL_SIDE_MIN_DEG, FRAME_GRID_COLS, FRAME_GRID_ROWS,
    SIMPLE_CAMERA_FRESH_S, SIMPLE_DRIFT_AHEAD_FT, SIMPLE_DRIFT_MIN_DEG,
    SIMPLE_DRIFT_NEAR_FT, SIMPLE_DRIFT_CLOSING_FT_S, SIMPLE_DRIFT_JUMP_MM,
    SIMPLE_DRIFT_MIN_SPAN_S, SIMPLE_DRIFT_WINDOW_S, SIMPLE_SIDE_WALL_EXTEND_MM,
    SIMPLE_SIDE_WALL_HOLD_SPAN_MM, SIMPLE_WALL_NEAR_FT, SIMPLE_WALL_NEAR_MARGIN_FT, SIMPLE_MOVE_ROOM_FT, SIMPLE_GLASS_BESIDE_FT, SIMPLE_OBJECT_CENTRE_BAND,
    SIMPLE_OBJECT_MIN_CONF, SIMPLE_OBJECT_MIN_HEIGHT_FRAC,
    SIMPLE_OBJECT_RELEASE_S, SIMPLE_OFFLINE_S, SIMPLE_SETTLE_S,
    SIMPLE_CAMERA_HFOV_DEG, SIMPLE_VEHICLE_APPROACH_RATE, SIMPLE_VEHICLE_CLASSES,
    SIMPLE_VEHICLE_CONFIRM_FRAMES,
    SIMPLE_VEHICLE_EGO_MARGIN, SIMPLE_VEHICLE_HEIGHT_M, SIMPLE_VEHICLE_IDLE_RATE,
    SIMPLE_VEHICLE_IDLE_SLIDE, SIMPLE_VEHICLE_LOST_S, SIMPLE_VEHICLE_MIN_CONF,
    SIMPLE_VEHICLE_MIN_HEIGHT_FRAC, SIMPLE_VEHICLE_MIN_SPAN_S, SIMPLE_VEHICLE_MISS_M,
    SIMPLE_VEHICLE_SIDE_MARGIN, SIMPLE_VEHICLE_WINDOW_S,
    SIMPLE_VEHICLE_CAREFUL_HEIGHT_FRAC, SIMPLE_VEHICLE_CAREFUL_MISS_M,
    SIMPLE_VEHICLE_RANGE_MATCH, SIMPLE_NAMED_OBJECTS,
    SIMPLE_OBJECT_HEIGHT_M, SIMPLE_STEER_DEAD_BAND, SIMPLE_STEER_GAP_M,
    SIMPLE_STEER_RANGE_M, SIMPLE_STEER_SIDE_MIN_FT, SIMPLE_STEER_STICKY,
    HEADING_STATE_STALE_S, RADAR_CONE_TRUST_MM,
)

MM_PER_FT = 304.8

WALK_STRAIGHT = "Walk straight"
WALL_AHEAD = "Wall ahead"
GLASS_AHEAD = "Glass ahead"
BLOCKED_WALL = "Blocked by wall"
BLOCKED_GLASS = "Blocked by glass"
TURN_LEFT = "Turn left"
TURN_RIGHT = "Turn right"

DRIFT_LEFT = "Drifting toward left wall, turn slightly right"
DRIFT_RIGHT = "Drifting toward right wall, turn slightly left"

NEAR_LEFT = "Near left wall, move right"
NEAR_RIGHT = "Near right wall, move left"
_SIDE_WALL = (DRIFT_LEFT, DRIFT_RIGHT, NEAR_LEFT, NEAR_RIGHT)

# Glass only one front board hears, inside SIMPLE_GLASS_BESIDE_FT (decided 2026-10-01).
GLASS_LEFT = "Glass on left, move right"
GLASS_RIGHT = "Glass on right, move left"
_GLASS_BESIDE = (GLASS_LEFT, GLASS_RIGHT)

VEHICLE_FROM_LEFT = "Vehicle approaching, move right"
VEHICLE_FROM_RIGHT = "Vehicle approaching, move left"
VEHICLE_APPROACHING = "Vehicle approaching"
VEHICLE_AHEAD = "Vehicle in front, be careful"
_VEHICLE_APPROACH = (VEHICLE_FROM_LEFT, VEHICLE_FROM_RIGHT, VEHICLE_APPROACHING)
_VEHICLE_SENTENCES = _VEHICLE_APPROACH + (VEHICLE_AHEAD,)

SENSORS_OFFLINE = "Sensors offline"

GLASS_SENSOR_OFF = "Glass sensor off"
GLASS_SENSOR_ON = "Glass sensor on"
_GLASS_NOTICES = (GLASS_SENSOR_OFF, GLASS_SENSOR_ON)

_OBJECT_WORDS = {c.lower(): w for c, w in SIMPLE_NAMED_OBJECTS.items()}
_WORDS = sorted(set(_OBJECT_WORDS.values()))
OBJECT_AHEAD = {w: f"{w} ahead" for w in _WORDS}
OBJECT_IN_FRONT = {w: f"{w} in front" for w in _WORDS}
_OBJECT_OF = {t: w for table in (OBJECT_AHEAD, OBJECT_IN_FRONT) for w, t in table.items()}
_PATH_AHEAD = (WALL_AHEAD, GLASS_AHEAD) + tuple(OBJECT_AHEAD.values())

STEER_PAST: Dict[Tuple[str, str, str], str] = {}
for _w in _WORDS:
    STEER_PAST[(_w, "left", "right")] = f"{_w} on left, turn slightly right"
    STEER_PAST[(_w, "right", "left")] = f"{_w} on right, turn slightly left"
    STEER_PAST[(_w, "ahead", "left")] = f"{_w} ahead, turn slightly left"
    STEER_PAST[(_w, "ahead", "right")] = f"{_w} ahead, turn slightly right"
STEER_IN_FRONT = {(_w, _t): f"{_w} in front, turn {_t}"
                  for _w in _WORDS for _t in ("left", "right")}
BETWEEN_LEFT = "Walk between them, slightly left"
BETWEEN_RIGHT = "Walk between them, slightly right"
BETWEEN_STRAIGHT = "Walk straight between them"
_BETWEEN = (BETWEEN_LEFT, BETWEEN_RIGHT, BETWEEN_STRAIGHT)
_WALK_ON = (WALK_STRAIGHT,) + _BETWEEN
_STEER_INFO: Dict[str, Dict[str, Any]] = {
    **{t: {"turn": k[2], "word": k[0], "where": k[1]} for k, t in STEER_PAST.items()},
    **{t: {"turn": k[1], "word": k[0], "where": "in front"} for k, t in STEER_IN_FRONT.items()},
    BETWEEN_LEFT: {"turn": "left", "word": None, "where": "between"},
    BETWEEN_RIGHT: {"turn": "right", "word": None, "where": "between"},
    BETWEEN_STRAIGHT: {"turn": None, "word": None, "where": "between"},
}

ALLOWED = ((WALK_STRAIGHT, WALL_AHEAD, GLASS_AHEAD,
            BLOCKED_WALL, BLOCKED_GLASS, TURN_LEFT, TURN_RIGHT,
            DRIFT_LEFT, DRIFT_RIGHT, NEAR_LEFT, NEAR_RIGHT, GLASS_LEFT, GLASS_RIGHT, SENSORS_OFFLINE,
            VEHICLE_FROM_LEFT, VEHICLE_FROM_RIGHT, VEHICLE_APPROACHING, VEHICLE_AHEAD)
           + _GLASS_NOTICES
           + tuple(OBJECT_AHEAD.values()) + tuple(OBJECT_IN_FRONT.values())
           + tuple(STEER_PAST.values()) + tuple(STEER_IN_FRONT.values()) + _BETWEEN)

_ACTION = {
    WALK_STRAIGHT: "walk_straight",
    WALL_AHEAD: "wall_ahead",
    GLASS_AHEAD: "glass_ahead",
    BLOCKED_WALL: "blocked_wall",
    BLOCKED_GLASS: "blocked_glass",
    TURN_LEFT: "turn_left",
    TURN_RIGHT: "turn_right",
    DRIFT_LEFT: "drift_left",
    DRIFT_RIGHT: "drift_right",
    NEAR_LEFT: "near_wall_left",
    NEAR_RIGHT: "near_wall_right",
    GLASS_LEFT: "glass_beside_left",
    GLASS_RIGHT: "glass_beside_right",
    VEHICLE_FROM_LEFT: "vehicle_approaching_left",
    VEHICLE_FROM_RIGHT: "vehicle_approaching_right",
    VEHICLE_APPROACHING: "vehicle_approaching",
    VEHICLE_AHEAD: "vehicle_ahead",
    SENSORS_OFFLINE: "sensors_offline",
    GLASS_SENSOR_OFF: "glass_sensor_off",
    GLASS_SENSOR_ON: "glass_sensor_on",
    **{t: "object_ahead" for t in OBJECT_AHEAD.values()},
    **{t: "object_in_front" for t in OBJECT_IN_FRONT.values()},
    **{t: "steer_" + i["turn"] for t, i in _STEER_INFO.items() if i["turn"]},
    BETWEEN_STRAIGHT: "walk_between",
}

_LEVEL = {
    WALK_STRAIGHT: 0,
    WALL_AHEAD: 1, GLASS_AHEAD: 1,
    DRIFT_LEFT: 1, DRIFT_RIGHT: 1, NEAR_LEFT: 1, NEAR_RIGHT: 1,
    GLASS_LEFT: 1, GLASS_RIGHT: 1,
    BLOCKED_WALL: 2, BLOCKED_GLASS: 2, TURN_LEFT: 2, TURN_RIGHT: 2,
    SENSORS_OFFLINE: 2,
    GLASS_SENSOR_OFF: 1, GLASS_SENSOR_ON: 1,
    **{t: 1 for t in OBJECT_AHEAD.values()},
    **{t: 2 for t in OBJECT_IN_FRONT.values()},
    # A stopped vehicle, the user standing still: an advisory, said once.
    VEHICLE_AHEAD: 1,
    VEHICLE_FROM_LEFT: 3, VEHICLE_FROM_RIGHT: 3, VEHICLE_APPROACHING: 3,
    **{t: 1 for t in STEER_PAST.values()},
    **{t: 1 for t in _BETWEEN},
    **{t: 2 for t in STEER_IN_FRONT.values()},
}
_BLOCKED = 2
_VEHICLE = 3


def _front(walk_decision: Dict[str, Any]) -> Dict[str, Any]:
    return (walk_decision.get("directions") or {}).get("front") or {}


def _clearance(walk_decision: Dict[str, Any], direction: str) -> float:
    """Measured room in one direction, in feet."""
    view = (walk_decision.get("directions") or {}).get(direction) or {}
    if view.get("unknown"):
        return 0.0                      # device silence is not room
    distance = view.get("distance_ft")
    if distance is None:
        return float("inf")             # no return = nothing out there = open
    try:
        return float(distance)
    except (TypeError, ValueError):
        return 0.0


def safer_side(walk_decision: Dict[str, Any],
               previous: Optional[str] = None,
               lidar_state: Optional[Dict[str, Any]] = None) -> str:
    """Which of left/right to turn toward when the front is blocked."""
    chosen = walk_decision.get("direction")
    if chosen in ("left", "right"):
        return chosen
    if previous in ("left", "right"):
        other = "right" if previous == "left" else "left"
        if (_clearance(walk_decision, other)
                <= _clearance(walk_decision, previous) + WALK_TURN_HYSTERESIS_FT):
            return previous
    left = _clearance(walk_decision, "left")
    right = _clearance(walk_decision, "right")
    try:
        from utils import spatial_scan
        side = spatial_scan.preferred_side(spatial_scan.live_snapshot(lidar_state or {}))
        if side in ("left", "right"):
            room, other = (left, right) if side == "left" else (right, left)
            if room >= WALK_MIN_TURN_FT and room + WALK_TURN_HYSTERESIS_FT >= other:
                return side
    except Exception:
        pass          # advisory layer: a fault here costs nothing but the hint

    return "right" if right > left else "left"


def _side_room(walk_decision: Dict[str, Any], lidar_state: Optional[Dict[str, Any]],
               fresh: bool, side: str) -> float:
    """Room to step to one side, in feet: the walk policy's clearance that way and, on a fresh scan, the laser's whole 45 deg sector — a sideways step needs the sector, not just the narrow steering cone."""
    room = _clearance(walk_decision, side)
    if fresh:
        mm = (((lidar_state or {}).get("sectors") or {}).get(side) or {}).get("min_dist")
        if isinstance(mm, (int, float)) and mm > 0:
            room = min(room, mm / MM_PER_FT)
    return room


def _fresh(lidar_state: Optional[Dict[str, Any]], now: float) -> bool:
    """Is this scan EVIDENCE — of a wall, a pane, a side wall, room to step?"""
    from utils.freshness import reading_is_current
    return (bool(lidar_state) and not lidar_state.get("simulated")
            and reading_is_current(lidar_state, LIDAR_STALE_S, now))


def _not_updating(lidar_state: Dict[str, Any], now: float) -> Optional[str]:
    """Why "Sensors offline" must be said now, or None."""
    from utils.freshness import (age_s, describe_reading, reading_is_current,
                                 received_within)
    if lidar_state.get("simulated"):
        return "the LiDAR is simulated — not a measurement"
    if not reading_is_current(lidar_state, SIMPLE_LIDAR_FRESH_S, now):
        age = age_s(lidar_state.get("last_update"), now)
        if (age is not None and age < 0.0
                and received_within(lidar_state, SIMPLE_LIDAR_FRESH_S)):
            return None
        return (f"the LiDAR is not updating "
                f"(newest scan {describe_reading(lidar_state, now)})")
    return None


def _front_cluster(lidar_state: Dict[str, Any], cone_only: bool = False
                   ) -> Tuple[Optional[float], List[Tuple[float, float]]]:
    """The laser returns walk_policy's FRONT reading came from."""
    from utils.lidar_utils import _angular_gap, _cluster_min

    pts = (lidar_state or {}).get("points") or {}
    centre = GUIDANCE_CONE_CENTRES["front"]
    half = GUIDANCE_CONE_DEG / 2.0
    bucket = []
    for angle, dist in zip(pts.get("angles") or [], pts.get("distances") or []):
        if not dist or dist <= 0:
            continue
        gap = _angular_gap(angle, centre)
        in_cone = gap <= half
        in_corridor = (
            not cone_only
            and GUIDANCE_CORRIDOR_HALF_WIDTH_MM > 0
            and gap < 90.0
            and dist <= GUIDANCE_CORRIDOR_MAX_MM
            and abs(dist * math.sin(math.radians(gap))) <= GUIDANCE_CORRIDOR_HALF_WIDTH_MM
        )
        if in_cone or in_corridor:
            bucket.append((float(dist), float(angle)))
    nearest = _cluster_min(bucket)["min_dist"]
    if nearest is None:
        return None, []
    cluster = [(a, d) for d, a in bucket
               if nearest <= d <= nearest + LIDAR_CLUSTER_WINDOW_MM]
    return float(nearest), cluster


def _on_line_mm(lidar_state: Optional[Dict[str, Any]]) -> Optional[float]:
    """Nearest confirmed return on the narrow straight-ahead beam alone — is anything on the walking line itself?"""
    line = ((lidar_state or {}).get("cones") or {}).get("front_line")
    if isinstance(line, dict) and "min_dist" in line:
        d = line["min_dist"]
        return None if d is None else float(d)
    return _front_cluster(lidar_state, cone_only=True)[0]


def _on_surface(angle: float, dist: float, surface: Dict[str, Any]) -> bool:
    """Does one laser return lie on a fitted surface's segment?"""
    from utils.drift_guidance import signed_bearing

    b = math.radians(signed_bearing(angle))
    x, y = dist * math.cos(b), dist * math.sin(b)
    cx, cy = surface["centroid_mm"]
    dx, dy = surface["direction"]
    lo, hi = surface["extent_mm"]
    along = (x - cx) * dx + (y - cy) * dy
    off = abs((y - cy) * dx - (x - cx) * dy)
    tol = max(SIMPLE_ON_WALL_MM, 3.0 * float(surface.get("rms_mm") or 0.0))
    return (lo - SIMPLE_ON_WALL_MM <= along <= hi + SIMPLE_ON_WALL_MM
            and off <= tol)


def _front_wall(walk_decision: Dict[str, Any],
                lidar_state: Optional[Dict[str, Any]],
                now: float, radius_ft: float,
                surfaces: Optional[List[Dict[str, Any]]] = None
                ) -> Optional[Dict[str, Any]]:
    """The wall walk_policy's front reading is looking at, or None when the thing ahead is not (known to be) a wall."""
    if not _fresh(lidar_state, now):
        return None
    front = _front(walk_decision)
    if not front.get("lidar_measured") or front.get("lidar_ft") is None:
        return None
    lidar_ft = float(front["lidar_ft"])
    fused = front.get("distance_ft")
    if fused is not None and abs(float(fused) - lidar_ft) > WALK_GLASS_GAP_FT:
        return None
    try:
        from utils.drift_guidance import signed_bearing
        nearest_mm, cluster = _front_cluster(lidar_state)
        if nearest_mm is None:
            return None
        if abs(nearest_mm / MM_PER_FT - lidar_ft) > SIMPLE_WALL_MATCH_FT:
            return None                 # a different scan from walk_policy's
        if surfaces is None:
            surfaces = _surfaces(lidar_state)
        on_line_mm = _on_line_mm(lidar_state)
    except Exception:
        return None                     # no wall named beats a crashed voice
    walls = [w for w in surfaces if w.get("kind") == "wall"]
    on = sum(1 for a, d in cluster if any(_on_surface(a, d, w) for w in walls))
    if on * 2 <= len(cluster):
        return None
    lateral = sum(d * math.sin(math.radians(signed_bearing(a)))
                  for a, d in cluster) / len(cluster)
    ft = nearest_mm / MM_PER_FT
    clear_to = max(radius_ft, ft + SIMPLE_WALL_MATCH_FT)
    return {
        "ft": ft,
        "alongside": on_line_mm is None or on_line_mm / MM_PER_FT > clear_to,
        "side": "right" if lateral >= 0 else "left",
    }


def _brushing(lidar_state: Optional[Dict[str, Any]], surface: Dict[str, Any],
              radius_ft: float) -> bool:
    """Is a block explained by this side wall, straight ahead being clear?"""
    try:
        from utils.drift_guidance import signed_bearing
        nearest_mm, cluster = _front_cluster(lidar_state)
        on_line_mm = _on_line_mm(lidar_state)
    except Exception:
        return False
    if nearest_mm is None or not cluster:
        return False
    clear_to = max(radius_ft, nearest_mm / MM_PER_FT + SIMPLE_WALL_MATCH_FT)
    if on_line_mm is not None and on_line_mm / MM_PER_FT <= clear_to:
        return False
    cx, cy = surface["centroid_mm"]
    dx, dy = surface["direction"]
    tol = max(SIMPLE_ON_WALL_MM, 3.0 * float(surface.get("rms_mm") or 0.0))
    on = 0
    for a, d in cluster:
        b = math.radians(signed_bearing(a))
        x, y = d * math.cos(b), d * math.sin(b)
        on += abs((y - cy) * dx - (x - cx) * dy) <= tol
    return on * 2 > len(cluster)


def _surfaces(lidar_state: Optional[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """The drift layer's fitted surfaces for the current scan, once per poll."""
    try:
        from utils import drift_guidance
        return drift_guidance.fit_surfaces(drift_guidance._forward_points(lidar_state))
    except Exception:
        return []


def side_walls(surfaces: List[Dict[str, Any]],
               holding: bool = False) -> Dict[str, Dict[str, Any]]:
    """The wall BESIDE the user on each side: {"left"|"right": {"ft", "line_ft", "converging", "surface"}}."""
    try:
        from utils.drift_guidance import DRIFT_SURFACE_MAX_RESIDUAL_MM as flat_rms
    except Exception:
        flat_rms = 0.0
    out: Dict[str, Dict[str, Any]] = {}
    edge = GUIDANCE_CORRIDOR_HALF_WIDTH_MM
    for surface in surfaces:
        if surface.get("kind") != "wall" and not (
                holding
                and float(surface.get("rms_mm", float("inf"))) <= flat_rms
                and float(surface.get("span_mm", 0.0)) >= SIMPLE_SIDE_WALL_HOLD_SPAN_MM):
            continue
        cx, cy = surface["centroid_mm"]
        dx, dy = surface["direction"]
        lo, hi = surface["extent_mm"]
        if dx < 0:
            dx, dy, lo, hi = -dx, -dy, -hi, -lo
        if dx < math.cos(math.radians(60.0)):
            continue                           # across the path, not beside it
        t = min(max(-(cx * dx + cy * dy), lo - SIMPLE_SIDE_WALL_EXTEND_MM), hi)
        px, py = cx + t * dx, cy + t * dy
        bearing = abs(math.degrees(math.atan2(py, px)))
        if not (DRIFT_WALL_SIDE_MIN_DEG <= bearing <= DRIFT_WALL_SIDE_MAX_DEG):
            continue
        side = "right" if py > 0 else "left"
        dist_mm = math.hypot(px, py)
        entry = out.get(side)
        if entry is None or dist_mm / MM_PER_FT < entry["ft"]:
            out[side] = entry = {
                "ft": dist_mm / MM_PER_FT,
                "line_ft": abs(cx * dy - cy * dx) / MM_PER_FT,
                "converging": bool(entry and entry["converging"]),
                "surface": surface,
            }
        if dist_mm > SIMPLE_DRIFT_NEAR_FT * MM_PER_FT:
            continue
        slope = dy / dx                        # lateral change per mm forward
        closing = -slope if side == "right" else slope
        if math.degrees(math.atan(max(closing, 0.0))) < SIMPLE_DRIFT_MIN_DEG:
            continue                           # parallel, or opening away
        reach_mm = max(0.0, (abs(py) - edge) / closing)
        if px + reach_mm > SIMPLE_DRIFT_AHEAD_FT * MM_PER_FT:
            continue
        entry["converging"] = True
    return out


def _theil_sen(samples: List[Tuple[float, float]]) -> Optional[float]:
    """Median of the pairwise slopes of (t, value) samples: a trend that one bad sample cannot make on its own."""
    slopes = sorted((v2 - v1) / (t2 - t1)
                    for i, (t1, v1) in enumerate(samples)
                    for t2, v2 in samples[i + 1:] if t2 - t1 > 1e-6)
    if not slopes:
        return None
    mid = len(slopes) // 2
    return slopes[mid] if len(slopes) % 2 else 0.5 * (slopes[mid - 1] + slopes[mid])


_CLOCK_STEP_S = 2.0


class _Approach:
    """Is the user actually MOVING towards the wall beside them?"""

    def __init__(self):
        self._samples: Dict[str, List[Tuple[float, float]]] = {"left": [], "right": []}

    def update(self, walls: Dict[str, Dict[str, Any]], scan_t: float) -> None:
        latest = max((p[-1][0] for p in self._samples.values() if p), default=None)
        if latest is not None and latest - _CLOCK_STEP_S < scan_t < latest - 1e-6:
            return                             # an older snapshot (_CLOCK_STEP_S)
        for side, samples in self._samples.items():
            if samples and scan_t < samples[-1][0] - 1e-6:
                samples.clear()                # the clock went backwards
            samples[:] = [p for p in samples if scan_t - p[0] <= SIMPLE_DRIFT_WINDOW_S]
            wall = walls.get(side)
            if wall is None:
                continue
            mm = float(wall.get("line_ft", wall["ft"])) * MM_PER_FT
            if samples and scan_t <= samples[-1][0] + 1e-6:
                continue                       # the same scan, polled again
            if samples and abs(mm - samples[-1][1]) > SIMPLE_DRIFT_JUMP_MM:
                samples.clear()                # a different wall
            samples.append((scan_t, mm))

    def rate_ft_s(self, side: str) -> Optional[float]:
        """Closing speed on that side's wall in ft/s (positive = getting closer), or None while the history is too short to say."""
        samples = self._samples.get(side) or []
        if len(samples) < 4 or samples[-1][0] - samples[0][0] < SIMPLE_DRIFT_MIN_SPAN_S - 1e-6:
            return None
        slope = _theil_sen(samples)                                   # mm/s
        return None if slope is None else -slope / MM_PER_FT

    def closing(self, side: str) -> bool:
        rate = self.rate_ft_s(side)
        return rate is not None and rate >= SIMPLE_DRIFT_CLOSING_FT_S - 1e-9

    def reset(self) -> None:
        self.__init__()


def glass_beside(radar_state: Optional[Dict[str, Any]]) -> Optional[str]:
    """"left" / "right" when the front radar echo comes from ONE board only."""
    try:
        from utils.radar_calibration import cardinal_sources
        sources = cardinal_sources().get("front") or ()
    except Exception:
        return None
    secs = (radar_state or {}).get("sectors") or {}
    reads = {}
    for key in sources:
        row = secs.get(key) or {}
        if not row.get("online") or row.get("degraded"):
            return None
        try:
            bearing = float(str(key).replace("deg", "")) % 360.0
        except ValueError:
            return None
        side = "right" if 0.0 < bearing < 180.0 else "left"
        dist = row.get("min_dist")
        reads[side] = (float(dist) / MM_PER_FT
                       if isinstance(dist, (int, float)) and dist else None)
    if set(reads) != {"left", "right"}:
        return None
    left, right = reads["left"], reads["right"]
    if (left is None) == (right is None):
        return None
    # Nearer than RADAR_CONE_TRUST_MM the pane is inside the walking corridor, whichever board heard it.
    if (left if left is not None else right) * MM_PER_FT <= RADAR_CONE_TRUST_MM:
        return None
    return "left" if left is not None else "right"


_QUAD_W = CAMERA_WIDTH / FRAME_GRID_COLS      # one camera's frame in the mosaic
_QUAD_H = CAMERA_HEIGHT / FRAME_GRID_ROWS


def _band() -> Tuple[float, float]:
    """The front camera's box, in pixels: the middle SIMPLE_OBJECT_CENTRE_BAND."""
    return (_QUAD_W * (0.5 - SIMPLE_OBJECT_CENTRE_BAND / 2.0),
            _QUAD_W * (0.5 + SIMPLE_OBJECT_CENTRE_BAND / 2.0))


def _front_stamp(cam: Dict[str, Any]) -> float:
    """When the front camera's current detections were captured."""
    front = cam.get("front") or {}
    return float(front.get("frame_time") or cam.get("last_update") or 0.0)


def _camera_sees(cam: Dict[str, Any], now: float) -> bool:
    """Front detections fresh, and the lens able to see (sensor_fusion's visibility gate — a fogged lens still emits labels)."""
    from utils.freshness import is_fresh
    if not is_fresh(_front_stamp(cam), SIMPLE_CAMERA_FRESH_S, now):
        return False
    if ((cam or {}).get("front") or {}).get("simulated"):
        return False
    try:
        from utils.sensor_fusion import _camera_identity_trusted
        return bool(_camera_identity_trusted()[0])
    except Exception:
        return True


def _object_height_m(cls: str) -> float:
    return float(SIMPLE_VEHICLE_HEIGHT_M.get(cls) or SIMPLE_OBJECT_HEIGHT_M.get(cls) or 1.5)


def _size_range(cls: str, box: Tuple[float, float, float, float]) -> Tuple[float, bool]:
    """(range in metres from the box's height and the class's typical height through the assumed lens, whether the frame cuts the box off)."""
    x1, y1, x2, y2 = box
    focal_px = (_QUAD_W / 2.0) / math.tan(math.radians(SIMPLE_CAMERA_HFOV_DEG) / 2.0)
    cut = y1 <= 2.0 or y2 >= _QUAD_H - 2.0
    return focal_px * _object_height_m(cls) / max(1.0, y2 - y1), cut


def _range_matches(est_m: float, front_m: Optional[float], cut: bool) -> bool:
    """Is a camera thing at `est_m` (by its size) plausibly what the rangefinders range at `front_m`?"""
    if front_m is None or front_m <= 0:
        return False
    ratio = est_m / front_m
    return ratio >= 1.0 / SIMPLE_VEHICLE_RANGE_MATCH and (cut or ratio <= SIMPLE_VEHICLE_RANGE_MATCH)


def front_object(camera_state: Optional[Dict[str, Any]],
                 now: float, front_ft: Optional[float] = None) -> Optional[str]:
    """The spoken word for the object on the user's path, or None."""
    cam = camera_state or {}
    if not _camera_sees(cam, now):
        return None
    height = _QUAD_H
    lo, hi = _band()
    best = None
    for det in (cam.get("front") or {}).get("detections") or []:
        word = _OBJECT_WORDS.get(str(det.get("class_name", "")).lower())
        if word is None or float(det.get("confidence") or 0.0) < SIMPLE_OBJECT_MIN_CONF:
            continue
        try:
            x1, y1, x2, y2 = (float(v) for v in det.get("bbox") or ())
        except (TypeError, ValueError):
            continue
        if x2 < lo or x1 > hi:
            continue                           # off to one side
        if (y2 - y1) < SIMPLE_OBJECT_MIN_HEIGHT_FRAC * height:
            continue                           # far away
        if front_ft is not None:
            est, cut = _size_range(str(det.get("class_name", "")).lower(), (x1, y1, x2, y2))
            if not _range_matches(est, front_ft * MM_PER_FT / 1000.0, cut):
                continue                       # not the thing being ranged
        rank = (str(det.get("class_name", "")).lower() in SIMPLE_VEHICLE_CLASSES,
                (x2 - x1) * (y2 - y1))
        if best is None or rank > best[0]:
            best = (rank, word)
    return best[1] if best else None


def ranged_vehicle(camera_state: Optional[Dict[str, Any]], now: float,
                   front_ft: Optional[float]) -> Optional[str]:
    """The spoken word for a vehicle that IS the thing ranged ahead, or None."""
    cam = camera_state or {}
    if front_ft is None or front_ft <= 0 or not _camera_sees(cam, now):
        return None
    lo, hi = _band()
    front_m = front_ft * MM_PER_FT / 1000.0
    best = None
    for det in (cam.get("front") or {}).get("detections") or []:
        cls = str(det.get("class_name", "")).lower()
        if cls not in SIMPLE_VEHICLE_CLASSES or cls not in _OBJECT_WORDS:
            continue
        if float(det.get("confidence") or 0.0) < SIMPLE_VEHICLE_MIN_CONF:
            continue
        try:
            x1, y1, x2, y2 = (float(v) for v in det.get("bbox") or ())
        except (TypeError, ValueError):
            continue
        h = y2 - y1
        if x2 < lo or x1 > hi or h < SIMPLE_OBJECT_MIN_HEIGHT_FRAC * _QUAD_H:
            continue
        range_m, cut = _size_range(cls, (x1, y1, x2, y2))
        if not _range_matches(range_m, front_m, cut):
            continue
        if best is None or h > best[0]:
            best = (h, _OBJECT_WORDS[cls])
    return best[1] if best else None


def current_motion(motion: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """`motion`, or None once it has stopped updating (audit pass 2, NEW-026)."""
    if not motion:
        return None
    from utils.freshness import is_fresh
    mono = motion.get("updated_mono")
    if isinstance(mono, (int, float)) and not isinstance(mono, bool):
        return motion if time.monotonic() - float(mono) <= HEADING_STATE_STALE_S else None
    if "updated_at" in motion:
        return motion if is_fresh(motion.get("updated_at"), HEADING_STATE_STALE_S) else None
    return motion


def user_motion(motion: Optional[Dict[str, Any]]) -> Tuple[Optional[bool], float]:
    """(standing still?, walking pace m/s) from heading_tracker's state."""
    motion = current_motion(motion)          # a stale state is none (NEW-026)
    if not motion:
        return None, 0.0
    walking = bool(motion.get("moving_confirmed") or motion.get("walking_confirmed"))
    if not walking:
        return True, 0.0
    return False, max(0.0, float(motion.get("speed_mps") or 0.0))


_NO_VEHICLE: Dict[str, Any] = {
    "approaching": False, "side": None, "stopped": False, "moving": False,
    "class": None,
    "ttc_s": None, "miss_m": None, "growth": None, "own_growth": None,
    "box": None, "tracks": 0,
}


class _Vehicles:
    """Vehicles in the front camera, followed from one detector frame to the next."""

    def __init__(self):
        self._tracks: List[Dict[str, Any]] = []
        self._frame: Optional[float] = None
        self._seen: Dict[str, Any] = dict(_NO_VEHICLE)

    def update(self, camera_state: Optional[Dict[str, Any]], now: float,
               pace_mps: float, yaw_dps: float = 0.0) -> Dict[str, Any]:
        cam = camera_state or {}
        if not _camera_sees(cam, now):
            self._tracks, self._frame, self._seen = [], None, dict(_NO_VEHICLE)
            return self._seen
        stamp = _front_stamp(cam)
        if self._frame is not None and stamp < self._frame - _CLOCK_STEP_S:
            self._tracks = []                  # the clock went backwards
        elif self._frame is not None and stamp <= self._frame + 1e-6:
            # The same front frame again — or an older one (_CLOCK_STEP_S).
            return self._seen
        self._frame = stamp
        self._follow(self._detections(cam), stamp)
        self._seen = self._judge(stamp, pace_mps, yaw_dps)
        return self._seen

    def reset(self) -> None:
        self.__init__()

    @staticmethod
    def _detections(cam: Dict[str, Any]) -> List[Dict[str, Any]]:
        out = []
        for det in (cam.get("front") or {}).get("detections") or []:
            cls = str(det.get("class_name", "")).lower()
            if cls not in SIMPLE_VEHICLE_CLASSES:
                continue
            if float(det.get("confidence") or 0.0) < SIMPLE_VEHICLE_MIN_CONF:
                continue
            try:
                x1, y1, x2, y2 = (float(v) for v in det.get("bbox") or ())
            except (TypeError, ValueError):
                continue
            if x2 <= x1 or y2 <= y1 or (y2 - y1) < SIMPLE_VEHICLE_MIN_HEIGHT_FRAC * _QUAD_H:
                continue
            out.append({"cls": cls, "box": (x1, y1, x2, y2)})
        return out

    def _follow(self, dets: List[Dict[str, Any]], t: float) -> None:
        """Match this frame's vehicles to the tracks by box overlap."""
        def iou(a, b):
            ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
            iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
            inter = ix * iy
            union = ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter)
            return inter / union if union > 0 else 0.0
        pairs = sorted(((iou(tr["samples"][-1][1], d["box"]), ti, di)
                        for ti, tr in enumerate(self._tracks)
                        for di, d in enumerate(dets)), reverse=True)
        used_t, used_d = set(), set()
        for score, ti, di in pairs:
            if score < 0.2:
                break
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            tr = self._tracks[ti]
            tr["samples"].append((t, dets[di]["box"]))
            tr["cls"] = dets[di]["cls"]
        for di, d in enumerate(dets):
            if di not in used_d:
                self._tracks.append({"samples": [(t, d["box"])], "cls": d["cls"],
                                     "side": None, "approach_frames": 0,
                                     "moving_frames": 0})
        kept = []
        for tr in self._tracks:
            if t - tr["samples"][-1][0] > SIMPLE_VEHICLE_LOST_S:
                continue
            tr["samples"] = [p for p in tr["samples"] if t - p[0] <= SIMPLE_VEHICLE_WINDOW_S]
            kept.append(tr)
        self._tracks = kept

    def _judge(self, t: float, pace_mps: float, yaw_dps: float = 0.0) -> Dict[str, Any]:
        lo, hi = _band()
        mid = _QUAD_W / 2.0
        focal_px = mid / math.tan(math.radians(SIMPLE_CAMERA_HFOV_DEG) / 2.0)
        turn_slide = -yaw_dps / SIMPLE_CAMERA_HFOV_DEG
        approaching, moving, stopped = [], [], []
        for tr in self._tracks:
            samples = tr["samples"]
            if samples[-1][0] != t:
                tr["approach_frames"] = tr["moving_frames"] = 0
                continue                       # not seen on this frame
            x1, y1, x2, y2 = samples[-1][1]
            if x2 < lo or x1 > hi:
                tr["approach_frames"] = tr["moving_frames"] = 0
                continue                       # outside the box
            cx = (x1 + x2) / 2.0
            side = tr["side"]
            if side == "left" and cx > mid + SIMPLE_VEHICLE_SIDE_MARGIN * _QUAD_W:
                side = None
            elif side == "right" and cx < mid - SIMPLE_VEHICLE_SIDE_MARGIN * _QUAD_W:
                side = None
            tr["side"] = side = side or ("left" if cx < mid else "right")
            if len(samples) < 3 or t - samples[0][0] < SIMPLE_VEHICLE_MIN_SPAN_S - 1e-6:
                continue
            edge = 2.0
            cut_lr = any(b[0] <= edge or b[2] >= _QUAD_W - edge for _, b in samples)
            cut_tb = any(b[1] <= edge or b[3] >= _QUAD_H - edge for _, b in samples)
            if cut_lr and cut_tb:
                continue
            size = ((lambda b: b[3] - b[1]) if cut_lr else
                    (lambda b: b[2] - b[0]) if cut_tb else
                    (lambda b: math.sqrt((b[2] - b[0]) * (b[3] - b[1]))))
            growth = _theil_sen([(ts, math.log(size(b))) for ts, b in samples])
            slide = _theil_sen([(ts, (b[0] + b[2]) / 2.0 / _QUAD_W) for ts, b in samples])
            if growth is None or slide is None:
                continue
            height_m = SIMPLE_VEHICLE_HEIGHT_M.get(tr["cls"], 1.5)
            heights = sorted(b[3] - b[1] for _, b in samples)
            h_px = heights[len(heights) // 2]
            range_m = focal_px * height_m / h_px
            own = pace_mps / range_m
            # Nearer edge's offset from the walking line, in metres.
            misses = sorted(
                0.0 if b[0] <= mid <= b[2] else
                height_m * min(abs(b[0] - mid), abs(b[2] - mid)) / (b[3] - b[1])
                for _, b in samples)
            miss_m = misses[len(misses) // 2]
            coming = (growth - SIMPLE_VEHICLE_EGO_MARGIN * own >= SIMPLE_VEHICLE_APPROACH_RATE
                      and miss_m <= SIMPLE_VEHICLE_MISS_M)
            tr["approach_frames"] = tr["approach_frames"] + 1 if coming else 0
            approach = tr["approach_frames"] >= SIMPLE_VEHICLE_CONFIRM_FRAMES
            across = slide - (cx / _QUAD_W - 0.5) * growth - turn_slide
            crossing = (abs(across) > SIMPLE_VEHICLE_IDLE_SLIDE
                        and abs(turn_slide) <= 2.0 * SIMPLE_VEHICLE_IDLE_SLIDE)
            closing = (growth - SIMPLE_VEHICLE_EGO_MARGIN * own >= SIMPLE_VEHICLE_IDLE_RATE
                       and miss_m <= SIMPLE_VEHICLE_CAREFUL_MISS_M)
            tr["moving_frames"] = (tr["moving_frames"] + 1
                                   if (crossing or closing) else 0)
            seen = {"approaching": approach,
                    "side": side,
                    "stopped": (abs(growth) <= SIMPLE_VEHICLE_IDLE_RATE
                                and abs(slide) <= SIMPLE_VEHICLE_IDLE_SLIDE),
                    "moving": (not approach
                               and tr["moving_frames"] >= SIMPLE_VEHICLE_CONFIRM_FRAMES
                               and h_px >= SIMPLE_VEHICLE_CAREFUL_HEIGHT_FRAC * _QUAD_H),
                    "class": tr["cls"],
                    "ttc_s": round(1.0 / growth, 1) if growth > 1e-3 else None,
                    "miss_m": round(miss_m, 2), "growth": round(growth, 3),
                    "own_growth": round(own, 3),
                    "box": [round(v) for v in samples[-1][1]]}
            if seen["approaching"]:
                approaching.append(seen)
            elif seen["moving"]:
                moving.append((h_px, seen))
            elif seen["stopped"]:
                stopped.append((h_px, seen))
        n = len(self._tracks)
        if approaching:
            # The soonest to arrive.
            return {**max(approaching, key=lambda v: v["growth"]), "tracks": n}
        pick = max(moving or stopped, key=lambda p: p[0])[1] if (moving or stopped) else None
        if pick is None:
            return {**_NO_VEHICLE, "tracks": n}
        return {**pick, "moving": bool(moving), "stopped": bool(stopped), "tracks": n}


def camera_path(camera_state: Optional[Dict[str, Any]], now: float,
                previous_turn: Optional[str] = None) -> Optional[Dict[str, Any]]:
    """How to get past the things that can hurt the user in the front camera's view, or None when none is in the way (field request 2026-09-25)."""
    cam = camera_state or {}
    if not _camera_sees(cam, now):
        return None
    things = []
    for det in (cam.get("front") or {}).get("detections") or []:
        cls = str(det.get("class_name", "")).lower()
        word = _OBJECT_WORDS.get(cls)
        if word is None or float(det.get("confidence") or 0.0) < SIMPLE_OBJECT_MIN_CONF:
            continue
        try:
            x1, y1, x2, y2 = (float(v) for v in det.get("bbox") or ())
        except (TypeError, ValueError):
            continue
        if x2 <= x1 or y2 <= y1:
            continue
        ppm_px = (y2 - y1) / _object_height_m(cls)          # pixels per metre
        range_m, cut = _size_range(cls, (x1, y1, x2, y2))
        if range_m > SIMPLE_STEER_RANGE_M and not cut:
            continue
        things.append({"word": word, "x1": x1 / _QUAD_W, "x2": x2 / _QUAD_W,
                       "need": SIMPLE_STEER_GAP_M * ppm_px / _QUAD_W,
                       "range_m": range_m, "cut": cut,
                       "vehicle": cls in SIMPLE_VEHICLE_CLASSES})
    if not things:
        return None
    things.sort(key=lambda t: t["x1"])
    in_way = [t for t in things
              if t["x2"] > 0.5 - t["need"] / 2.0 and t["x1"] < 0.5 + t["need"] / 2.0]
    lo, hi = SIMPLE_OBJECT_CENTRE_BAND / 2.0, 1.0 - SIMPLE_OBJECT_CENTRE_BAND / 2.0
    if not in_way:
        left = [t for t in things if t["x2"] <= 0.5 and t["x2"] > 0.5 - (0.5 - lo)]
        right = [t for t in things if t["x1"] >= 0.5 and t["x1"] < hi]
        if not (left and right):
            return None
        near = min(left + right, key=lambda t: t["range_m"])
        return {"kind": "between", "turn": None, "where": "both", "word": near["word"],
                "target": 0.5, "range_m": round(near["range_m"], 2),
                "cut": near["cut"], "in_way": 0}

    # Groups of overlapping boxes, left to right.
    groups: List[List[Dict[str, Any]]] = []
    for t in things:
        if groups and t["x1"] <= max(m["x2"] for m in groups[-1]):
            groups[-1].append(t)
        else:
            groups.append([t])
    candidates = []
    for a, b in zip(groups, groups[1:]):
        edge_a = max(a, key=lambda m: m["x2"])
        edge_b = min(b, key=lambda m: m["x1"])
        if edge_a["cut"] or edge_b["cut"]:
            continue
        if edge_b["x1"] - edge_a["x2"] >= max(edge_a["need"], edge_b["need"]):
            candidates.append({"kind": "between",
                               "target": (edge_a["x2"] + edge_b["x1"]) / 2.0})
    first = min(groups[0], key=lambda m: m["x1"])
    last = max(groups[-1], key=lambda m: m["x2"])
    candidates.append({"kind": "around", "target": first["x1"] - first["need"] / 2.0})
    candidates.append({"kind": "around", "target": last["x2"] + last["need"] / 2.0})

    def open_frame(left_side: bool) -> float:
        span = (0.0, 0.5) if left_side else (0.5, 1.0)
        covered = sum(max(0.0, min(t["x2"], span[1]) - max(t["x1"], span[0])) for t in things)
        return (span[1] - span[0]) - covered

    def rank(c):
        on_left = c["target"] < 0.5
        return (abs(c["target"] - 0.5), -open_frame(on_left))

    best = min(candidates, key=rank)
    if previous_turn in ("left", "right"):
        same = [c for c in candidates if (c["target"] < 0.5) == (previous_turn == "left")]
        if same:
            keep = min(same, key=rank)
            if abs(keep["target"] - 0.5) <= abs(best["target"] - 0.5) + SIMPLE_STEER_STICKY:
                best = keep
    off = best["target"] - 0.5
    dead = SIMPLE_STEER_DEAD_BAND if best["kind"] == "between" else 0.0
    turn = "left" if off < -dead else "right" if off > dead else None
    nearest = min(in_way, key=lambda t: (not t["vehicle"], t["range_m"]))
    if best["kind"] == "between":
        where = "both"
    else:
        def side_of(t):
            c = (t["x1"] + t["x2"]) / 2.0
            return ("ahead" if abs(c - 0.5) <= t["need"] / 4.0
                    else "left" if c < 0.5 else "right")
        away = "left" if turn == "right" else "right" if turn == "left" else None
        where = away if (away and all(side_of(t) == away for t in in_way)) else "ahead"
    return {"kind": best["kind"], "turn": turn, "where": where, "word": nearest["word"],
            "target": round(best["target"], 3), "range_m": round(nearest["range_m"], 2),
            "cut": nearest["cut"], "in_way": len(in_way)}


def vehicle_move_blocked(walk_decision: Optional[Dict[str, Any]],
                         lidar_state: Optional[Dict[str, Any]], fresh: bool,
                         vehicle: Optional[str]) -> bool:
    """True when the move a vehicle warning would give — AWAY from the vehicle's side — has under SIMPLE_STEER_SIDE_MIN_FT of room, or no reading at all (decided 2026-09-29, audit NEW-049)."""
    if vehicle not in ("left", "right"):
        return False
    move = "right" if vehicle == "left" else "left"
    if not ((walk_decision or {}).get("directions") or {}).get(move):
        return True                                   # nothing reports that side
    return _side_room(walk_decision, lidar_state, fresh, move) < SIMPLE_STEER_SIDE_MIN_FT


def _steer_text(steer: Optional[Dict[str, Any]], blocked: bool) -> Optional[str]:
    """The sentence for a camera_path plan, or None when it has none."""
    if not steer:
        return None
    turn, word = steer.get("turn"), steer.get("word")
    if blocked:
        return STEER_IN_FRONT.get((word, turn)) if turn else None
    if steer.get("kind") == "between":
        return (BETWEEN_LEFT if turn == "left" else BETWEEN_RIGHT if turn == "right"
                else BETWEEN_STRAIGHT)
    return STEER_PAST.get((word, steer.get("where"), turn))


def _steer_allowed(steer: Dict[str, Any], side_room: Dict[str, Any],
                   blocked: bool, front_ft: Optional[float],
                   turn_side: Optional[str] = None) -> bool:
    """May this way past be said now?"""
    turn = steer.get("turn")
    room = side_room.get(turn) if turn else None
    if isinstance(room, (int, float)) and room < SIMPLE_STEER_SIDE_MIN_FT:
        return False
    if blocked:
        front_m = None if front_ft is None else front_ft * MM_PER_FT / 1000.0
        if not turn or not _range_matches(steer["range_m"], front_m, bool(steer.get("cut"))):
            return False
        if turn_side in ("left", "right") and turn != turn_side:
            def ft(v: Any) -> float:
                return float("inf") if v is None else float(v)   # None: open
            mine, chosen = ft(side_room.get(turn)), ft(side_room.get(turn_side))
            if mine < WALK_MIN_TURN_FT or mine + WALK_TURN_HYSTERESIS_FT < chosen:
                return False
    return True


def _sentence(blocked: bool, near: bool, glass: bool, wall: bool,
              side: Optional[str], drift: Optional[str] = None,
              obj: Optional[str] = None, close: Optional[str] = None,
              vehicle: Optional[str] = None, vehicle_ahead: bool = False,
              steer: Optional[Dict[str, Any]] = None,
              vehicle_no_room: bool = False,
              glass_side: Optional[str] = None) -> str:
    """A vehicle coming at the user, above everything."""
    if vehicle:
        if vehicle_no_room:
            return VEHICLE_APPROACHING
        return VEHICLE_FROM_LEFT if vehicle == "left" else VEHICLE_FROM_RIGHT
    if blocked:
        if glass:
            return BLOCKED_GLASS
        if wall:
            return BLOCKED_WALL
        if vehicle_ahead:
            return VEHICLE_AHEAD
        if close:
            return NEAR_LEFT if close == "left" else NEAR_RIGHT
        if drift:
            return DRIFT_LEFT if drift == "left" else DRIFT_RIGHT
        past = _steer_text(steer, True)
        if past:
            return past
        if obj in OBJECT_IN_FRONT:
            return OBJECT_IN_FRONT[obj]
        return TURN_LEFT if side != "right" else TURN_RIGHT
    if vehicle_ahead:
        return VEHICLE_AHEAD
    # A pane on the path outranks a wall beside you, which a 4 ft near-wall band makes common (2026-10-01).
    if near and glass:
        return GLASS_AHEAD
    if close:
        return NEAR_LEFT if close == "left" else NEAR_RIGHT
    if glass_side:
        return GLASS_LEFT if glass_side == "left" else GLASS_RIGHT
    if drift:
        return DRIFT_LEFT if drift == "left" else DRIFT_RIGHT
    if near and wall:
        return WALL_AHEAD
    past = _steer_text(steer, False)
    if past:
        return past
    if near and obj in OBJECT_AHEAD:
        return OBJECT_AHEAD[obj]
    return WALK_STRAIGHT


def decide(walk_decision: Optional[Dict[str, Any]] = None,
           lidar_state: Optional[Dict[str, Any]] = None,
           now: Optional[float] = None,
           previous_side: Optional[str] = None,
           radar_state: Optional[Dict[str, Any]] = None,
           camera_state: Optional[Dict[str, Any]] = None,
           approach: Optional[_Approach] = None,
           motion: Optional[Dict[str, Any]] = None,
           vehicles: Optional[_Vehicles] = None,
           previous_steer: Optional[str] = None) -> Dict[str, Any]:
    """What this one poll's readings say, and the sentence they map to."""
    walk_decision = walk_decision or {}
    now = time.time() if now is None else now
    motion = current_motion(motion)          # stale motion is no motion (NEW-026)
    still, pace = user_motion(motion)
    yaw = float((motion or {}).get("yaw_rate_dps") or 0.0)
    seen = (vehicles.update(camera_state, now, pace, yaw) if vehicles is not None
            else dict(_NO_VEHICLE))
    vehicle = seen["side"] if seen["approaching"] else None
    vehicle_ahead = vehicle is None and (bool(seen.get("moving"))
                                         or (bool(seen["stopped"]) and still is True))

    if walk_decision.get("action") in (None, "", "unknown"):
        return {"instruction": "", "action": "unknown", "rule": "waiting",
                "side": None, "obstacle": None, "blocked": False,
                "near": False, "glass": False, "wall": False, "drift": None,
                "close": None, "close_band": [], "beside": {}, "closing": {},
                "vehicle": None, "vehicle_ahead": False, "vehicles": seen,
                "steer": None, "object": None, "front_ft": None,
                "reason": "front not measured yet"}

    blocked = bool(walk_decision.get("blocked"))
    front = _front(walk_decision)
    front_ft = front.get("distance_ft")
    front_ft = None if front_ft is None else float(front_ft)

    glass = bool(walk_decision.get("glass_ahead")) and _fresh(lidar_state, now)
    beside = glass_beside(radar_state) if glass else None
    beside_ft = None
    if beside:
        beside_ft = front.get("radar_ft")          # the one board's echo
        glass = False
        laser = front.get("lidar_ft") if front.get("lidar_measured") else None
        front_ft = None if laser is None else float(laser)
        blocked = front_ft is not None and front_ft < WALK_BLOCK_FT
        walk_decision = {**walk_decision, "blocked": blocked, "glass_ahead": False,
                         "directions": {**(walk_decision.get("directions") or {}),
                                        "front": {**front, "distance_ft": front_ft,
                                                  "glass": False}}}
    near = front_ft is not None and front_ft <= SIMPLE_WALL_APPROACH_FT
    in_band = front_ft is not None and front_ft <= (SIMPLE_WALL_APPROACH_FT
                                                    + SIMPLE_BAND_MARGIN_FT)

    # One surface fit per poll, shared by the front-wall and side-wall tests.
    fresh = _fresh(lidar_state, now)
    surfaces = _surfaces(lidar_state) if fresh else []
    on_line_ft = None
    if fresh:
        try:
            line_mm = _on_line_mm(lidar_state)
            on_line_ft = None if line_mm is None else line_mm / MM_PER_FT
        except Exception:
            on_line_ft = None
    walls = side_walls(surfaces)
    held = side_walls(surfaces, holding=True)
    if approach is not None and fresh:
        approach.update(walls, float(lidar_state.get("last_update") or now))
    rates = {s: (approach.rate_ft_s(s) if approach is not None else None)
             for s in ("left", "right")}
    moving_to = {s for s in ("left", "right")
                 if approach is not None and approach.closing(s)}
    if (motion or {}).get("confidently_still"):
        moving_to = set()

    found = None
    if (blocked or in_band) and not glass:
        found = _front_wall(walk_decision, lidar_state, now,
                            WALK_BLOCK_FT if blocked else SIMPLE_WALL_APPROACH_FT,
                            surfaces)
    wall = found is not None and not found["alongside"]
    alongside = found is not None and found["alongside"]

    near_sides = {s for s, w in walls.items() if w["ft"] <= SIMPLE_WALL_NEAR_FT}
    band_sides = {s for s, w in held.items()
                  if w["ft"] <= SIMPLE_WALL_NEAR_FT + SIMPLE_WALL_NEAR_MARGIN_FT}
    boxed = {s for s in ("left", "right")
             if _side_room(walk_decision, lidar_state, fresh,
                           "right" if s == "left" else "left") < SIMPLE_MOVE_ROOM_FT}
    # Drifting: pointed at it (converging) AND moving towards it (closing).
    drift_sides = {s for s, w in walls.items() if w["converging"] and s in moving_to}
    one = lambda sides: next(iter(sides)) if len(sides) == 1 else None
    brushed = None
    if blocked and not (glass or wall or alongside):
        s = one(near_sides)
        if s and _brushing(lidar_state, walls[s]["surface"], WALK_BLOCK_FT):
            alongside, found = True, {"side": s, "alongside": True,
                                      "ft": walls[s]["ft"]}
        elif one(band_sides) and _brushing(lidar_state, held[one(band_sides)]["surface"],
                                           WALK_BLOCK_FT):
            brushed = one(band_sides)
    close = drift = tight_turn = glass_side = None
    close_band: List[str] = []
    if blocked:
        if alongside:
            s = found["side"]
            tight = len(near_sides) == 2 or s in boxed
            if s in moving_to and (tight or s not in near_sides):
                drift = s
            elif tight:
                tight_turn = "left" if s == "right" else "right"
            else:
                close, close_band = s, [s]
        elif brushed:
            close_band = [brushed]
    else:
        close = one(near_sides - boxed)
        close_band = (sorted(band_sides) if len(band_sides) == 1
                      and not band_sides & boxed else [])
        drift = one(drift_sides)
        if (beside and isinstance(beside_ft, (int, float))
                and beside_ft <= SIMPLE_GLASS_BESIDE_FT and beside not in boxed):
            glass_side = beside

    veh_word = (ranged_vehicle(camera_state, now, front_ft)
                if (blocked or near) and not glass and not (alongside or brushed)
                else None)
    if veh_word:
        wall = False

    obj = None
    if (blocked or near) and not (glass or wall or alongside or brushed):
        obj = veh_word or front_object(camera_state, now, front_ft)

    side = (safer_side(walk_decision, previous_side, lidar_state if fresh else None)
            if blocked else None)
    if tight_turn:
        side = tight_turn

    views = walk_decision.get("directions") or {}
    side_room: Dict[str, Optional[float]] = {}
    for s in ("left", "right"):
        view = views.get(s)
        # No view at all is as unknown as one marked unknown.
        room = 0.0 if (not view or view.get("unknown")) else view.get("distance_ft")
        if fresh and room != 0.0:
            mm = (((lidar_state or {}).get("sectors") or {}).get(f"front_{s}") or {}).get("min_dist")
            if isinstance(mm, (int, float)) and mm > 0:
                room = mm / MM_PER_FT if room is None else min(room, mm / MM_PER_FT)
        side_room[s] = room
    steer = camera_path(camera_state, now, previous_steer) if not (glass or wall) else None
    if steer and not _steer_allowed(steer, side_room, blocked, front_ft, side):
        steer = None
    no_room_by_side = {s: vehicle_move_blocked(walk_decision, lidar_state, fresh, s)
                       for s in ("left", "right")}
    vehicle_no_room = bool(vehicle and no_room_by_side.get(vehicle))
    text = _sentence(blocked, near, glass, wall, side, drift, obj, close,
                     vehicle, vehicle_ahead, steer, vehicle_no_room, glass_side)
    if text in _VEHICLE_APPROACH:
        rule = "vehicle_approaching"
        why = f"{seen['class']} approaching from your {vehicle}" + (
            f", contact in ~{seen['ttc_s']:.0f} s" if seen.get("ttc_s") else "") + (
            f" — no room to step {'right' if vehicle == 'left' else 'left'}"
            if vehicle_no_room else "")
    elif text == VEHICLE_AHEAD:
        rule = "vehicle_ahead"
        why = (f"{seen['class']} moving ahead" if seen.get("moving")
               else f"{seen['class']} stopped ahead, you are standing still")
    elif text in _STEER_INFO:
        rule = "blocked" if blocked else "steering"
        why = _steer_reason(text, steer)
    elif text in _GLASS_BESIDE:
        rule = "glass_beside"
        why = (f"glass on your {glass_side} ({beside_ft:.1f} ft) — only the "
               f"{glass_side}-front radar hears it, the laser sees through it")
    elif text in (NEAR_LEFT, NEAR_RIGHT):
        rule = "near_wall"
        ft = (walls.get(close) or {}).get("ft")
        why = f"near the wall on your {close}" + (f" ({ft:.1f} ft)" if ft else "")
    elif text in (DRIFT_LEFT, DRIFT_RIGHT):
        rule = "drifting"
        why = (f"moving toward the wall on your {drift} "
               f"({rates[drift] or 0:.1f} ft/s, {walls[drift]['ft']:.1f} ft away)"
               if drift in walls else f"moving toward the wall on your {drift}")
    elif blocked and tight_turn:
        rule = "blocked"
        wall_side = "left" if tight_turn == "right" else "right"
        why = (f"standing close to the wall on your {wall_side}"
               + (f" ({walls[wall_side]['ft']:.1f} ft)" if wall_side in walls else "")
               + f", no room to step {tight_turn}: turn away from it")
    elif blocked:
        rule = "blocked"
        why = f"front blocked by {'glass' if glass else 'wall' if wall else (obj or 'an obstacle').lower()}"
    elif near and (glass or wall or obj):
        rule = "approaching"
        why = f"{'glass' if glass else 'wall' if wall else obj.lower()} {front_ft or 0:.1f} ft ahead"
    else:
        rule, why = "clear", "path ahead is clear"
    if beside and text not in _GLASS_BESIDE:
        why += f" (glass on your {beside} ignored)"
    return {"instruction": text, "action": _ACTION[text], "rule": rule,
            "side": side, "obstacle": ("glass" if glass else "wall" if wall
                                       else obj.lower() if obj else None),
            "blocked": blocked, "near": near, "glass": glass, "wall": wall,
            "drift": drift, "close": close, "close_band": close_band,
            "beside": {s: round(w["ft"], 2) for s, w in walls.items()},
            "closing": {s: (None if r is None else round(r, 2))
                        for s, r in rates.items()},
            "converging": sorted(s for s, w in walls.items() if w["converging"]),
            "vehicle": vehicle, "vehicle_ahead": vehicle_ahead, "vehicles": seen,
            "steer": steer, "side_room": side_room, "user_still": still,
            "vehicle_no_room": vehicle_no_room,
            "vehicle_no_room_by_side": no_room_by_side,
            "glass_side": glass_side, "glass_side_ft": beside_ft,
            "object": obj, "front_ft": front_ft, "on_line_ft": on_line_ft,
            "reason": why}


class _Hold:
    """A boolean that turns on fast and off slowly, and tolerates dropouts."""

    def __init__(self, attack: float, release: float, gap: float):
        self.attack, self.release, self.gap = attack, release, gap
        self.state = False
        self._first: Optional[float] = None       # start of the evidence run
        self._miss_since: Optional[float] = None  # first poll of the miss run

    @property
    def active(self) -> bool:
        """Held, or evidence for it is still being gathered."""
        return self.state or self._first is not None

    def update(self, raw: bool, now: float) -> bool:
        if self._first is not None and now < self._first:
            self._first = now
        if self._miss_since is not None and now < self._miss_since:
            self._miss_since = now
        if raw:
            if self._first is None:
                self._first = now
            self._miss_since = None
            if not self.state and now - self._first >= self.attack - 1e-6:
                self.state = True
            return self.state
        if self._miss_since is None:
            self._miss_since = now
        missed = now - self._miss_since
        if self.state and missed >= self.release - 1e-6:
            self.state = False
        if not self.state and missed >= self.gap - 1e-6:
            self._first = None
        return self.state

    def clear(self) -> None:
        self.state, self._first, self._miss_since = False, None, None


class _World:
    """The raw readings of successive polls, smoothed into one steady picture."""

    def __init__(self):
        timing = (SIMPLE_ALERT_CONFIRM_S, SIMPLE_CLEAR_CONFIRM_S, SIMPLE_ALERT_GAP_S)
        self.blocked = _Hold(*timing)
        self.near = _Hold(*timing)
        self.glass = _Hold(*timing)
        self.wall = _Hold(*timing)
        self.drift_left = _Hold(*timing)
        self.drift_right = _Hold(*timing)
        self.close_left = _Hold(*timing)
        self.close_right = _Hold(*timing)
        self.glass_left = _Hold(*timing)
        self.glass_right = _Hold(*timing)
        # Side-wall ranges over time — what tells drifting from standing still.
        self.approach = _Approach()
        # Vehicle boxes over time — what tells approaching from parked.
        self.vehicles = _Vehicles()
        self.vehicle = _Hold(*timing)
        self.vehicle_side: Optional[str] = None
        self.vehicle_careful = _Hold(SIMPLE_ALERT_CONFIRM_S, SIMPLE_OBJECT_RELEASE_S,
                                     SIMPLE_ALERT_GAP_S)
        self.steer = _Hold(SIMPLE_ALERT_CONFIRM_S, SIMPLE_OBJECT_RELEASE_S,
                           SIMPLE_ALERT_GAP_S)
        self.steer_plan: Optional[Dict[str, Any]] = None
        self.obj = _Hold(SIMPLE_ALERT_CONFIRM_S, SIMPLE_OBJECT_RELEASE_S,
                         SIMPLE_ALERT_GAP_S)
        self.obj_word: Optional[str] = None
        self.episode = 0
        self._in_episode = False
        self._unknown_since: Optional[float] = None
        self._last_text = ""                 # the sentence the last update mapped to
        self.side: Optional[str] = None
        self._blocked_since: Optional[float] = None
        self._crossing_until: float = float("-inf")

    def at_crossing(self, detected: bool, now: float) -> bool:
        """The crossing detector's verdict, held SIMPLE_CROSSING_HOLD_S past its last sighting."""
        if detected:
            self._crossing_until = now + SIMPLE_CROSSING_HOLD_S
        return now < self._crossing_until

    def update(self, raw: Dict[str, Any], now: float) -> Tuple[str, bool]:
        """(the sentence the smoothed picture maps to, whether a block's NAME is still being confirmed and the sentence should wait for it)."""
        front = raw.get("front_ft")
        # A backwards clock step: see _Hold.update.
        if self._blocked_since is not None and now < self._blocked_since:
            self._blocked_since = now
        self._crossing_until = min(self._crossing_until, now + SIMPLE_CROSSING_HOLD_S)
        in_block = raw["blocked"] or (
            self.blocked.state and front is not None
            and front < WALK_BLOCK_FT + SIMPLE_BAND_MARGIN_FT)
        in_band = raw["near"] or (
            self.near.state and front is not None
            and front < SIMPLE_WALL_APPROACH_FT + SIMPLE_BAND_MARGIN_FT)
        release = (SIMPLE_GLASS_RELEASE_S
                   if self.glass.state and (self.blocked.state or self.near.state)
                   else SIMPLE_CLEAR_CONFIRM_S)
        for hold in (self.blocked, self.near, self.glass):
            hold.release = release
        blocked = self.blocked.update(in_block, now)
        near = self.near.update(in_band, now)
        glass = self.glass.update(raw["glass"], now)
        wall = self.wall.update(raw["wall"], now)
        left = self.drift_left.update(raw.get("drift") == "left", now)
        right = self.drift_right.update(raw.get("drift") == "right", now)
        drift = ("left" if left and not right else
                 "right" if right and not left else None)
        band = raw.get("close_band") or []
        c_left = self.close_left.update(
            raw.get("close") == "left" or (self.close_left.state and "left" in band), now)
        c_right = self.close_right.update(
            raw.get("close") == "right" or (self.close_right.state and "right" in band), now)
        close = ("left" if c_left and not c_right else
                 "right" if c_right and not c_left else None)
        g_left = self.glass_left.update(raw.get("glass_side") == "left", now)
        g_right = self.glass_right.update(raw.get("glass_side") == "right", now)
        glass_side = ("left" if g_left and not g_right else
                      "right" if g_right and not g_left else None)
        if raw.get("vehicle"):
            self.vehicle_side = raw["vehicle"]
        vehicle = self.vehicle_side if self.vehicle.update(bool(raw.get("vehicle")), now) else None
        if vehicle is None:
            self.vehicle_side = None if not self.vehicle.active else self.vehicle_side
        vehicle_ahead = (self.vehicle_careful.update(bool(raw.get("vehicle_ahead")), now)
                         and vehicle is None)
        side_seen = bool(raw.get("close") or raw.get("drift") or raw.get("close_band"))
        on_line = raw.get("on_line_ft")
        line_blocked = on_line is not None and on_line < WALK_BLOCK_FT + SIMPLE_BAND_MARGIN_FT
        if raw["blocked"] and line_blocked and not side_seen:
            close = drift = None
        if raw["blocked"] and not raw.get("vehicle_ahead"):
            vehicle_ahead = False
        if not raw.get("vehicle_ahead") and self._last_text != VEHICLE_AHEAD:
            vehicle_ahead = False
        if raw.get("steer"):
            self.steer_plan = raw["steer"]
        steer = self.steer_plan if self.steer.update(bool(raw.get("steer")), now) else None
        if not self.steer.active:
            self.steer_plan = None
        if self.steer_plan and not _steer_allowed(
                self.steer_plan, raw.get("side_room") or {}, blocked, front,
                raw.get("side") or self.side):
            self.steer.clear()
            self.steer_plan = steer = None
        if raw.get("object"):
            self.obj_word = raw["object"]
        obj = self.obj.update(bool(raw.get("object")), now)

        if not (self.blocked.active or self.near.active):
            self.glass.clear()
            self.wall.clear()
            self.obj.clear()
            self.obj_word = None
            glass = wall = obj = False
        if blocked:
            if self._blocked_since is None:
                self._blocked_since = now
            if raw["blocked"] and raw.get("side") in ("left", "right"):
                self.side = raw["side"]
        else:
            self.side = None
            self._blocked_since = None
        side_active = ((side_seen or not line_blocked)
                       and (self.drift_left.active or self.drift_right.active
                            or self.close_left.active or self.close_right.active))
        naming = (blocked and not (glass or wall or drift or close or obj)
                  and (self.glass.active or self.wall.active or self.obj.active
                       or side_active)
                  and now - self._blocked_since < SIMPLE_ALERT_CONFIRM_S + 1e-6)
        text = _sentence(blocked, near, glass, wall, self.side, drift,
                         self.obj_word if obj else None, close,
                         vehicle, vehicle_ahead, steer,
                         vehicle_no_room=bool((raw.get("vehicle_no_room_by_side")
                                               or {}).get(vehicle)),
                         glass_side=glass_side)
        self._last_text = text
        hazard = _LEVEL.get(text, 0) >= 1
        if hazard and not self._in_episode:
            self.episode += 1
        self._in_episode = hazard
        return text, naming

    @property
    def steer_turn(self) -> Optional[str]:
        """The side the user is being steered to, kept by camera_path."""
        return (self.steer_plan or {}).get("turn")

    def offline(self, raw: Dict[str, Any], lidar_state: Optional[Dict[str, Any]],
                now: float) -> Optional[str]:
        """Why the device cannot currently see ahead, or None."""
        if raw.get("rule") == "waiting":
            if self._unknown_since is None or now < self._unknown_since:
                self._unknown_since = now
        else:
            self._unknown_since = None
        if lidar_state is not None:
            why = _not_updating(lidar_state, now)
            if why:
                return why
        if self._unknown_since is not None and now - self._unknown_since >= SIMPLE_OFFLINE_S:
            return "no range reading ahead"
        return None

    def reset(self) -> None:
        self.__init__()


class _GlassNotice:
    """Whether the wearer has been told the front radar is lost, or back."""

    def __init__(self):
        self.announced_ok = True
        self._since: Optional[float] = None
        self._first: Optional[float] = None     # first poll judged
        self._ever_ok = False                   # full coverage seen at least once
        self._losses = 0                        # announced losses, for the backoff
        self._ok_since: Optional[float] = None  # coverage unbroken since

    def _recover_need(self) -> float:
        return min(SIMPLE_GLASS_RECOVER_CONFIRM_S * 2 ** max(self._losses - 1, 0),
                   SIMPLE_GLASS_RECOVER_MAX_S)

    def pending(self, ok: bool, now: float, settling: bool = True) -> Optional[str]:
        """`settling`: the radar is still coming up (front_glass_coverage) — discovery has not reported, or an owed front board is starting or calibrating."""
        if self._first is None or now < self._first:
            self._first = now
        if ok:
            self._ever_ok = True
            if self._ok_since is None or now < self._ok_since:
                self._ok_since = now
            if (self.announced_ok and self._losses
                    and now - self._ok_since >= SIMPLE_GLASS_RECOVER_MAX_S):
                self._losses = 0                # a long quiet run: start afresh
        else:
            self._ok_since = None
        if ok == self.announced_ok:
            self._since = None
            return None
        if self._since is None or now < self._since:     # new run, or clock step
            self._since = now
        if ok:
            need = self._recover_need()
        else:
            need = SIMPLE_GLASS_LOSS_CONFIRM_S
            if (not self._ever_ok and settling
                    and now - self._first < SIMPLE_GLASS_STARTUP_GRACE_S - 1e-6):
                return None                     # the boards are still coming up
        if now - self._since >= need - 1e-6:
            return GLASS_SENSOR_ON if ok else GLASS_SENSOR_OFF
        return None

    def said(self, text: str) -> None:
        self.announced_ok = text == GLASS_SENSOR_ON
        if text == GLASS_SENSOR_OFF:
            self._losses += 1
        self._since = None

    def state(self) -> Dict[str, Any]:
        return {"announced": "on" if self.announced_ok else "off",
                "changing_since": self._since,
                "recover_after_s": self._recover_need()}

    def reset(self) -> None:
        self.__init__()


def _airtime(text: str) -> float:
    return len(text.split()) / NOTIFY_WORDS_PER_SEC


class _Gate:
    """Decides whether a sentence is worth SAYING on this poll."""

    def __init__(self):
        self._spoken: str = ""          # last sentence actually announced
        self._level: int = -1
        self._spoken_at: float = 0.0
        self._busy_until: float = 0.0   # when that sentence finishes playing
        self._has_spoken: bool = False
        self._repeats: int = 0          # consecutive re-statements of _spoken
        self._pending: str = ""         # candidate waiting out the debounce
        self._pending_since: float = 0.0
        self._episode: Optional[int] = None   # the obstacle _spoken was about
        self._seq: int = 0              # never goes backwards, even on reset
        # The sentence the latest cue_seq counted: guidance OR a status notice.
        self._counted: str = ""
        self._status_busy_until: float = 0.0

    def admit(self, text: str, now: float,
              episode: Optional[int] = None) -> Tuple[bool, int, str]:
        """(say it, sequence number, why not)."""
        if not text:
            self._pending, self._pending_since = "", 0.0
            return False, self._seq, "nothing to say"
        level = _LEVEL[text]
        if self._has_spoken and now < self._spoken_at:
            step = now - self._spoken_at
            self._spoken_at += step
            self._busy_until += step
            self._pending_since = min(self._pending_since + step, now)
        self._unstep_status(now)
        if text != self._pending:
            self._pending, self._pending_since = text, now

        if text == self._spoken and (episode is None or episode == self._episode):
            if level < _BLOCKED:
                return False, self._seq, "already said — not repeated"
            window = min(SIMPLE_REPEAT_S * (2 ** self._repeats), SIMPLE_REPEAT_MAX_S)
            since = now - self._spoken_at
            if since < window:
                return (False, self._seq, f"unchanged — said {since:.0f}s ago "
                                          f"(repeats after {window:.0f}s)")
            return self._commit(text, level, now, episode)

        arrived = (self._spoken in (DRIFT_LEFT, DRIFT_RIGHT)
                   and text in (NEAR_LEFT, NEAR_RIGHT))
        vehicle_news = (text in _VEHICLE_SENTENCES
                        and self._spoken not in _VEHICLE_SENTENCES)
        after_vehicle = (self._spoken in _VEHICLE_SENTENCES
                         and text not in _VEHICLE_SENTENCES and level >= _BLOCKED)
        offline_news = text == SENSORS_OFFLINE and self._spoken != SENSORS_OFFLINE
        # Something on the path after a wall beside you is news, said once that sentence ends (2026-10-01).
        path_news = self._spoken in _SIDE_WALL + _GLASS_BESIDE and text in _PATH_AHEAD
        busy_until = max(self._busy_until if self._has_spoken else 0.0,
                         self._status_busy_until)
        if level >= 1 and (level > self._level or arrived or vehicle_news
                           or after_vehicle or offline_news or path_news):
            if now < busy_until and level < _VEHICLE:
                return (False, self._seq, f"more urgent — waiting "
                        f"{busy_until - now:.1f}s for the last sentence to end")
            return self._commit(text, level, now, episode)

        held = now - self._pending_since
        need = SIMPLE_CONFIRM_S
        flipped = (self._spoken in _VEHICLE_APPROACH and text in _VEHICLE_APPROACH)
        if (level >= 1 and self._has_spoken and episode is not None
                and episode == self._episode):
            upgrade = (self._spoken in (TURN_LEFT, TURN_RIGHT)
                       and text not in (TURN_LEFT, TURN_RIGHT)
                       and level == self._level)
            upgrade = upgrade or flipped or (self._spoken in _VEHICLE_SENTENCES
                                             and text not in _VEHICLE_SENTENCES)
            upgrade = upgrade or self._spoken == SENSORS_OFFLINE
            if not upgrade:
                need = SIMPLE_SETTLE_S
        if held < need:
            return (False, self._seq,
                    f"same obstacle — held {held:.1f}s of {need:.0f}s"
                    if need > SIMPLE_CONFIRM_S else
                    f"unconfirmed — held {held:.1f}s of {need:.1f}s")
        if (self._has_spoken and now - self._spoken_at < SIMPLE_MIN_GAP_S
                and not flipped):
            return (False, self._seq, f"too soon — "
                    f"{SIMPLE_MIN_GAP_S - (now - self._spoken_at):.1f}s until "
                    f"the voice is free")
        if now < self._status_busy_until:
            return (False, self._seq, f"too soon — "
                    f"{self._status_busy_until - now:.1f}s until the status ends")
        return self._commit(text, level, now, episode)

    def _unstep_status(self, now: float) -> None:
        if self._status_busy_until - now > 2.0 * _airtime(GLASS_SENSOR_OFF):
            self._status_busy_until = now

    def admit_status(self, text: str, now: float) -> Tuple[bool, int, str]:
        """Admit a STATUS notice ("Glass sensor off/on"): its own cue_seq, and nothing else about the gate changes (audit pass 2, NEW-028)."""
        if self._has_spoken and now < self._spoken_at:
            self._busy_until = min(self._busy_until, now)      # clock stepped back
        self._unstep_status(now)
        busy_until = max(self._busy_until if self._has_spoken else 0.0,
                         self._status_busy_until)
        if now < busy_until:
            return (False, self._seq, f"status — waiting "
                    f"{busy_until - now:.1f}s for the last sentence to end")
        self._status_busy_until = now + _airtime(text)
        self._counted = text
        self._seq += 1
        return True, self._seq, ""

    def _commit(self, text: str, level: int, now: float,
                episode: Optional[int] = None) -> Tuple[bool, int, str]:
        same = text == self._spoken and (episode is None or episode == self._episode)
        self._repeats = self._repeats + 1 if same else 0
        self._spoken, self._level, self._spoken_at = text, level, now
        self._episode = episode
        self._busy_until = now + _airtime(text)
        self._has_spoken = True
        self._counted = text
        self._seq += 1
        return True, self._seq, ""

    def settle(self, level: int) -> None:
        """The situation the last sentence described has eased to `level`."""
        if 0 <= level < self._level:
            self._level = level

    def forget_pending(self) -> None:
        """Drop the candidate waiting out the debounce, as an empty poll does."""
        self._pending, self._pending_since = "", 0.0

    @property
    def last(self) -> str:
        """The last GUIDANCE sentence announced (a status notice is not one)."""
        return self._spoken

    @property
    def counted(self) -> str:
        """The sentence the sequence number last counted — guidance or status."""
        return self._counted

    @property
    def level(self) -> int:
        """Urgency of the last sentence spoken, -1 before the first."""
        return self._level

    def state(self) -> Dict[str, Any]:
        return {"last": self._spoken, "cue_seq": self._seq,
                "pending": self._pending}

    def reset(self) -> None:
        seq = self._seq
        self.__init__()
        self._seq = seq


_lock = threading.Lock()
_world = _World()
_gate = _Gate()
_glass = _GlassNotice()


def _steer_reason(text: str, steer: Optional[Dict[str, Any]]) -> str:
    info = _STEER_INFO.get(text) or {}
    turn = info.get("turn")
    if info.get("where") == "between":
        return ("a gap straight ahead between them" if not turn
                else f"a gap between them, slightly {turn}")
    word = (info.get("word") or "something").lower()
    rng = (steer or {}).get("range_m")
    at = f" (~{rng:.0f} m)" if isinstance(rng, (int, float)) else ""
    return f"{word} {info.get('where')}{at}: passing it on the {turn}"


def _reason(text: str, raw: Dict[str, Any]) -> str:
    """The reason for the sentence actually chosen."""
    if (not text or text == raw.get("instruction") or text == SENSORS_OFFLINE
            or text in _GLASS_NOTICES):
        return raw.get("reason", "")
    if text in _STEER_INFO:
        return _steer_reason(text, raw.get("steer"))
    if text == VEHICLE_APPROACHING:
        return "vehicle approaching — no room to step aside"
    if text in _VEHICLE_APPROACH:
        return f"vehicle approaching from your {'left' if text == VEHICLE_FROM_LEFT else 'right'}"
    if text == VEHICLE_AHEAD:
        return "vehicle ahead"
    if text in (DRIFT_LEFT, DRIFT_RIGHT):
        return f"moving toward the wall on your {'left' if text == DRIFT_LEFT else 'right'}"
    if text in _GLASS_BESIDE:
        side = "left" if text == GLASS_LEFT else "right"
        ft = raw.get("glass_side_ft")
        return f"glass on your {side}" + (f" ({ft:.1f} ft)" if isinstance(ft, (int, float)) else "")
    if text in (NEAR_LEFT, NEAR_RIGHT):
        side = "left" if text == NEAR_LEFT else "right"
        ft = (raw.get("beside") or {}).get(side)
        return f"near the wall on your {side}" + (f" ({ft:.1f} ft)" if ft else "")
    if text == WALK_STRAIGHT:
        return "path ahead is clear"
    if text in (TURN_LEFT, TURN_RIGHT):
        return "front blocked by an obstacle"
    thing = ("glass" if "glass" in text.lower() and text in (BLOCKED_GLASS, GLASS_AHEAD)
             else "wall" if text in (BLOCKED_WALL, WALL_AHEAD)
             else text.rsplit(" ", 2 if text.endswith("in front") else 1)[0].lower())
    return (f"front blocked by {thing}" if _LEVEL.get(text) == _BLOCKED
            else f"{thing} ahead")


def guidance(walk_decision: Optional[Dict[str, Any]] = None,
             lidar_state: Optional[Dict[str, Any]] = None,
             now: Optional[float] = None,
             hold: Optional[str] = None,
             at_crossing: bool = False,
             radar_state: Optional[Dict[str, Any]] = None,
             camera_state: Optional[Dict[str, Any]] = None,
             motion: Optional[Dict[str, Any]] = None,
             hold_lidar_absent: bool = False) -> Dict[str, Any]:
    """The sentence AND whether to announce it."""
    now = time.time() if now is None else now
    with _lock:
        raw = decide(walk_decision, lidar_state, now, previous_side=_world.side,
                     radar_state=radar_state, camera_state=camera_state,
                     approach=_world.approach, motion=motion,
                     vehicles=_world.vehicles, previous_steer=_world.steer_turn)
        naming = False
        offline = _world.offline(raw, lidar_state, now)
        stale = (not offline and lidar_state is not None
                 and not _fresh(lidar_state, now))
        seen = raw.get("vehicles") or {}
        if ((offline or stale) and seen.get("approaching")
                and seen.get("side") in ("left", "right")):
            text = (VEHICLE_APPROACHING
                    if vehicle_move_blocked(walk_decision, lidar_state, False, seen["side"])
                    else VEHICLE_FROM_LEFT if seen["side"] == "left" else VEHICLE_FROM_RIGHT)
            raw = {**raw, "rule": "vehicle_approaching",
                   "reason": f"{seen.get('class') or 'vehicle'} approaching from your "
                             f"{seen['side']} (LiDAR {'offline: ' + offline if offline else 'stale'})"}
        elif offline:
            text = SENSORS_OFFLINE
            raw = {**raw, "rule": "offline", "reason": offline}
        elif stale:
            from utils.freshness import describe_reading
            text = ""
            raw = {**raw, "rule": "stale",
                   "reason": f"LiDAR scan {describe_reading(lidar_state, now)}"
                             f" — holding, nothing new said"}
        elif raw["rule"] == "waiting":
            text = ""
        else:
            text, naming = _world.update(raw, now)
        notice, cover = None, {}
        if radar_state is not None:
            try:
                from utils.sensor_fusion import front_glass_coverage
                cover = front_glass_coverage(radar_state)
                notice = _glass.pending(bool(cover["ok"]), now,
                                        settling=bool(cover.get("settling", True)))
            except Exception:
                notice, cover = None, {}
        if text and text not in ALLOWED:
            text = ""               # never hand the delivery layer a stray line
        if text:
            _gate.settle(_LEVEL[text])
        # A start-up hold waiting on a LiDAR that is not coming (NEW-029).
        through_hold = bool(hold and hold_lidar_absent and offline
                            and text in (SENSORS_OFFLINE,) + _VEHICLE_APPROACH)
        status = (notice if notice and not hold and text == WALK_STRAIGHT
                  and raw.get("instruction") == WALK_STRAIGHT else None)
        if hold and not through_hold:
            speak, cue_seq, quiet = False, _gate.state()["cue_seq"], hold
        elif status:
            speak, cue_seq, quiet = _gate.admit_status(status, now)
            if speak:
                _glass.said(status)
                text = status
                raw = {**raw, "rule": "status",
                       "reason": (cover.get("reason") or "front radar coverage restored")}
        elif raw["rule"] == "stale":
            speak, cue_seq, quiet = False, _gate.state()["cue_seq"], raw["reason"]
        elif text in _WALK_ON and (
                raw.get("blocked")
                or (raw.get("near") and raw.get("instruction") not in _WALK_ON)
                or raw.get("instruction") in _VEHICLE_SENTENCES):
            speak, cue_seq = False, _gate.state()["cue_seq"]
            quiet = "confirming — the newest reading is not clear"
        elif _world.at_crossing(at_crossing, now) and text in _WALK_ON:
            _gate.forget_pending()
            speak, cue_seq = False, _gate.state()["cue_seq"]
            quiet = "at a road crossing — not saying walk straight"
        elif naming and _gate.level < _BLOCKED:
            speak, cue_seq = False, _gate.state()["cue_seq"]
            quiet = "blocked — confirming what it is"
        else:
            speak, cue_seq, quiet = _gate.admit(text, now, _world.episode)
        if raw["rule"] in ("waiting", "stale"):
            quiet = quiet if (hold and not through_hold) else raw["reason"]
        announced = _gate.counted

    level = _LEVEL.get(text, -1)
    return {
        "instruction": text,
        "action": _ACTION.get(text, "unknown"),
        "rule": ("offline" if text == SENSORS_OFFLINE else
                 "status" if text in _GLASS_NOTICES else
                 "vehicle_approaching" if text in _VEHICLE_APPROACH else
                 "vehicle_ahead" if text == VEHICLE_AHEAD else
                 "steering" if _LEVEL.get(text) == 1 and text in _STEER_INFO else
                 "near_wall" if text in (NEAR_LEFT, NEAR_RIGHT) else
                 "glass_beside" if text in _GLASS_BESIDE else
                 "drifting" if text in (DRIFT_LEFT, DRIFT_RIGHT) else
                 "blocked" if level == _BLOCKED else
                 "approaching" if level == 1 else
                 "clear" if text else raw["rule"]),
        "side": (_world.side if text in (TURN_LEFT, TURN_RIGHT) else
                 _STEER_INFO[text]["turn"] if text in _STEER_INFO else
                 "left" if text in (DRIFT_LEFT, NEAR_LEFT, VEHICLE_FROM_LEFT, GLASS_LEFT) else
                 "right" if text in (DRIFT_RIGHT, NEAR_RIGHT, VEHICLE_FROM_RIGHT, GLASS_RIGHT)
                 else None),
        "obstacle": ("vehicle" if text in _VEHICLE_APPROACH + (VEHICLE_AHEAD,) else
                     (_STEER_INFO[text]["word"] or "").lower() or None
                     if text in _STEER_INFO else
                     "glass" if text in (BLOCKED_GLASS, GLASS_AHEAD) + _GLASS_BESIDE else
                     "wall" if text in (BLOCKED_WALL, WALL_AHEAD) + _SIDE_WALL else
                     _OBJECT_OF[text].lower() if text in _OBJECT_OF else None),
        "reason": _reason(text, raw),
        "raw": {k: raw.get(k) for k in ("instruction", "blocked", "near", "glass",
                                    "wall", "drift", "close", "beside",
                                    "closing", "converging", "vehicle",
                                    "vehicle_ahead", "vehicles", "steer", "user_still", "glass_side",
                                    "object", "front_ft")},
        "speak": speak, "cue_seq": cue_seq, "quiet_reason": quiet,
        "announced": announced,
    }


def reset() -> None:
    """Forget what was said and what was seen — a new journey."""
    with _lock:
        _gate.reset()
        _world.reset()
        _glass.reset()


def state() -> Dict[str, Any]:
    with _lock:
        return {**_gate.state(), "counted": _gate.counted,
                "glass_sensor": _glass.state()}
