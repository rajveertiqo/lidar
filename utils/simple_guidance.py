"""Simplified walking guidance — a short, closed vocabulary, warned fast and cleared slowly."""

import math
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from config import (
    GUIDANCE_CONE_CENTRES,
    GUIDANCE_CONE_DEG,
    GUIDANCE_CORRIDOR_HALF_WIDTH_MM,
    GUIDANCE_CORRIDOR_MAX_MM,
    LIDAR_CLUSTER_WINDOW_MM,
    LIDAR_STALE_S,
    NOTIFY_AIRTIME_BASE_S,
    NOTIFY_AIRTIME_PER_CHAR_S,
    NOTIFY_AIRTIME_PER_COMMA_S,
    SIMPLE_GLASS_LOSS_CONFIRM_S,
    SIMPLE_GLASS_RECOVER_CONFIRM_S,
    SIMPLE_GLASS_RECOVER_MAX_S,
    SIMPLE_GLASS_STARTUP_GRACE_S,
    SIMPLE_CAMERA_LOSS_CONFIRM_S,
    SIMPLE_CAMERA_RECOVER_CONFIRM_S,
    SIMPLE_CAMERA_RECOVER_MAX_S,
    SIMPLE_CAMERA_STARTUP_GRACE_S,
    SIMPLE_ALERT_CONFIRM_S,
    SIMPLE_ALERT_GAP_S,
    SIMPLE_BAND_MARGIN_FT,
    SIMPLE_BLOCKED_REPEAT_S,
    SIMPLE_CLEAR_CONFIRM_S,
    SIMPLE_CROSSING_HOLD_S,
    SIMPLE_CROSSING_MAX_HOLD_S,
    SIMPLE_CROSSING_WALKED_S,
    SIMPLE_GLASS_RELEASE_S,
    SIMPLE_CONFIRM_S,
    SIMPLE_LIDAR_FRESH_S,
    SIMPLE_MIN_GAP_S,
    SIMPLE_OBSTACLE_AHEAD_FT,
    SIMPLE_OFFLINE_REPEAT_S,
    SIMPLE_ON_WALL_MM,
    SIMPLE_REPEAT_MAX_S,
    SIMPLE_REPEAT_S,
    SIMPLE_WALL_APPROACH_FT,
    SIMPLE_WALL_MATCH_FT,
    WALK_BLOCK_FT,
    WALK_GLASS_GAP_FT,
    WALK_MIN_TURN_FT,
    WALK_TURN_HYSTERESIS_FT,
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    DRIFT_WALL_SIDE_MAX_DEG,
    DRIFT_WALL_SIDE_MIN_DEG,
    FRAME_GRID_COLS,
    FRAME_GRID_ROWS,
    SIMPLE_CAMERA_FRESH_S,
    SIMPLE_DRIFT_AHEAD_FT,
    SIMPLE_DRIFT_MIN_DEG,
    SIMPLE_DRIFT_NEAR_FT,
    SIMPLE_DRIFT_CLOSING_FT_S,
    SIMPLE_DRIFT_JUMP_MM,
    SIMPLE_DRIFT_MIN_SPAN_S,
    SIMPLE_DRIFT_WINDOW_S,
    SIMPLE_SIDE_WALL_EXTEND_MM,
    SIMPLE_SIDE_WALL_HOLD_SPAN_MM,
    SIMPLE_WALL_NEAR_FT,
    SIMPLE_WALL_NEAR_MARGIN_FT,
    SIMPLE_MOVE_ROOM_FT,
    SIMPLE_GLASS_BESIDE_FT,
    SIMPLE_OBJECT_CENTRE_BAND,
    SIMPLE_OBJECT_MIN_CONF,
    SIMPLE_OBJECT_MIN_HEIGHT_FRAC,
    SIMPLE_OBJECT_RELEASE_S,
    SIMPLE_OFFLINE_S,
    SIMPLE_SETTLE_S,
    SIMPLE_SYSTEM_LOSS_CONFIRM_S,
    SIMPLE_SYSTEM_RECOVER_CONFIRM_S,
    SIMPLE_SYSTEM_RECOVER_MAX_S,
    SIMPLE_SYSTEM_STARTUP_GRACE_S,
    SIMPLE_CAMERA_HFOV_DEG,
    SIMPLE_VEHICLE_APPROACH_RATE,
    SIMPLE_VEHICLE_CLASSES,
    SIMPLE_VEHICLE_CONFIRM_FRAMES,
    SIMPLE_VEHICLE_EGO_MARGIN,
    SIMPLE_VEHICLE_HEIGHT_M,
    SIMPLE_VEHICLE_IDLE_RATE,
    SIMPLE_VEHICLE_IDLE_SLIDE,
    SIMPLE_VEHICLE_LOST_S,
    SIMPLE_VEHICLE_MIN_CONF,
    SIMPLE_VEHICLE_MIN_HEIGHT_FRAC,
    SIMPLE_VEHICLE_MIN_SPAN_S,
    SIMPLE_VEHICLE_MISS_M,
    SIMPLE_VEHICLE_SIDE_MARGIN,
    SIMPLE_VEHICLE_WINDOW_S,
    SIMPLE_VEHICLE_CAREFUL_HEIGHT_FRAC,
    SIMPLE_VEHICLE_CAREFUL_MISS_M,
    SIMPLE_VEHICLE_RANGE_MATCH,
    SIMPLE_VEHICLE_PACE_UNKNOWN_MPS,
    SIMPLE_NAMED_OBJECTS,
    SIMPLE_OBJECT_HEIGHT_M,
    SIMPLE_STEER_DEAD_BAND,
    SIMPLE_STEER_GAP_M,
    SIMPLE_STEER_RANGE_M,
    SIMPLE_STEER_SIDE_MIN_FT,
    SIMPLE_STEER_STICKY,
    HEADING_STATE_STALE_S,
    RADAR_CONE_TRUST_MM,
    LIDAR_MOVERS_CAMERA_MATCH_DEG,
    LIDAR_MOVERS_REPEAT_S,
    LIDAR_MOVERS_SPEAK,
    MOTION_TRACKER_WARN,
    ROUTE_NEED_M,
    ROUTE_PLANNER_ENABLED,
    ROUTE_PLANNER_SPEAK,
    ROUTE_STEER_MAX_DEG,
    TOF_USE_DISTANCE,
)

MM_PER_FT = 304.8

WALK_STRAIGHT = "Walk straight"
WALL_AHEAD = "Wall ahead"
GLASS_AHEAD = "Glass ahead"
BLOCKED_WALL = "Blocked by wall"
BLOCKED_GLASS = "Blocked by glass"
TURN_LEFT = "Turn left"
TURN_RIGHT = "Turn right"
# The route planner's (utils/route_planner, 2026-10-13): past something no camera names, and
# no way ahead at all. Said only with ROUTE_PLANNER_SPEAK.
TURN_SLIGHT_LEFT = "Turn slightly left"
TURN_SLIGHT_RIGHT = "Turn slightly right"
TURN_AROUND = "Turn around"

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

# Something the LiDAR sees moving by itself, coming at the user (utils/lidar_movers,
# 2026-10-07), or that only the radar hears closing in where the LiDAR cannot see it
# (utils/radar_movers: "Something"). Crossing: stop and let it pass. Head-on or from
# behind: step aside, where the laser shows room. Spoken only with LIDAR_MOVERS_SPEAK.
_MOVER_WORDS = ("Person", "Vehicle", "Something")
MOVER_CROSSING = {
    (w, s): f"{w} approaching from {s}, stop"
    for w in _MOVER_WORDS
    for s in ("left", "right")
}
MOVER_TOWARD = {
    (w, m): f"{w} coming toward you, move {m}"
    for w in _MOVER_WORDS
    for m in ("left", "right")
}
MOVER_TOWARD_STOP = {w: f"{w} coming toward you, stop" for w in _MOVER_WORDS}
MOVER_BEHIND = {
    (w, m): f"{w} approaching from behind, move {m}"
    for w in _MOVER_WORDS
    for m in ("left", "right")
}
MOVER_BEHIND_ONLY = {w: f"{w} approaching from behind" for w in _MOVER_WORDS}
_MOVER_INFO: Dict[str, Dict[str, Any]] = {
    **{t: {"word": k[0], "move": None} for k, t in MOVER_CROSSING.items()},
    **{t: {"word": k[0], "move": k[1]} for k, t in MOVER_TOWARD.items()},
    **{t: {"word": w, "move": None} for w, t in MOVER_TOWARD_STOP.items()},
    **{t: {"word": k[0], "move": k[1]} for k, t in MOVER_BEHIND.items()},
    **{t: {"word": w, "move": None} for w, t in MOVER_BEHIND_ONLY.items()},
}
_MOVER_SENTENCES = tuple(_MOVER_INFO)
_APPROACHING = _VEHICLE_SENTENCES + _MOVER_SENTENCES

# Was "Sensors offline" until 2026-10-21: it sounded like the "Glass sensor off" advisory,
# and did not say what to do.
SENSORS_OFFLINE = "Guidance stopped, use your cane"

# On the walking line inside SIMPLE_OBSTACLE_AHEAD_FT, not a wall, glass or anything a
# camera names: said before the WALK_BLOCK_FT turn (2026-10-21).
OBSTACLE_AHEAD = "Obstacle ahead"
# The walk policy's "stop": blocked, and no direction has room to turn into (2026-10-21).
STOP_NO_WAY = "Stop, no clear way"
# A traffic light in the front camera: said once per crossing, in place of the "Walk
# straight" that is withheld there. Never "safe" (2026-10-21).
TRAFFIC_LIGHT_AHEAD = "Traffic light ahead"
# The start-up hold (utils/startup_gate): said when it begins, and when guidance does.
STARTING = "Starting, please wait"
READY = "Ready"
_STARTUP_NOTICES = (STARTING, READY)

GLASS_SENSOR_OFF = "Glass sensor off"
GLASS_SENSOR_ON = "Glass sensor on"
_GLASS_NOTICES = (GLASS_SENSOR_OFF, GLASS_SENSOR_ON)

CAMERA_OFF = "Camera off"
CAMERA_ON = "Camera on"
_CAMERA_NOTICES = (CAMERA_OFF, CAMERA_ON)

# The Pi itself (utils/pi_health, 2026-10-21): under-voltage means the battery or supply is
# failing and the device may switch off; heat means it is slowing down.
POWER_LOW = "Power low"
POWER_NORMAL = "Power normal"
DEVICE_HOT = "Device overheating"
DEVICE_COOL = "Device cooled down"
_SYSTEM_NOTICES = (POWER_LOW, POWER_NORMAL, DEVICE_HOT, DEVICE_COOL)
_STATUS_NOTICES = _GLASS_NOTICES + _CAMERA_NOTICES + _SYSTEM_NOTICES

_OBJECT_WORDS = {c.lower(): w for c, w in SIMPLE_NAMED_OBJECTS.items()}
_WORDS = sorted(set(_OBJECT_WORDS.values()))
OBJECT_AHEAD = {w: f"{w} ahead" for w in _WORDS}
OBJECT_IN_FRONT = {w: f"{w} in front" for w in _WORDS}
_OBJECT_OF = {
    t: w for table in (OBJECT_AHEAD, OBJECT_IN_FRONT) for w, t in table.items()
}
_PATH_AHEAD = (WALL_AHEAD, GLASS_AHEAD, OBSTACLE_AHEAD) + tuple(OBJECT_AHEAD.values())

STEER_PAST: Dict[Tuple[str, str, str], str] = {}
for _w in _WORDS:
    STEER_PAST[(_w, "left", "right")] = f"{_w} on left, turn slightly right"
    STEER_PAST[(_w, "right", "left")] = f"{_w} on right, turn slightly left"
    STEER_PAST[(_w, "ahead", "left")] = f"{_w} ahead, turn slightly left"
    STEER_PAST[(_w, "ahead", "right")] = f"{_w} ahead, turn slightly right"
STEER_IN_FRONT = {
    (_w, _t): f"{_w} in front, turn {_t}" for _w in _WORDS for _t in ("left", "right")
}
BETWEEN_LEFT = "Walk between them, slightly left"
BETWEEN_RIGHT = "Walk between them, slightly right"
BETWEEN_STRAIGHT = "Walk straight between them"
_BETWEEN = (BETWEEN_LEFT, BETWEEN_RIGHT, BETWEEN_STRAIGHT)
_WALK_ON = (WALK_STRAIGHT,) + _BETWEEN
_STEER_INFO: Dict[str, Dict[str, Any]] = {
    **{t: {"turn": k[2], "word": k[0], "where": k[1]} for k, t in STEER_PAST.items()},
    **{
        t: {"turn": k[1], "word": k[0], "where": "in front"}
        for k, t in STEER_IN_FRONT.items()
    },
    BETWEEN_LEFT: {"turn": "left", "word": None, "where": "between"},
    BETWEEN_RIGHT: {"turn": "right", "word": None, "where": "between"},
    BETWEEN_STRAIGHT: {"turn": None, "word": None, "where": "between"},
    TURN_SLIGHT_LEFT: {"turn": "left", "word": None, "where": "ahead"},
    TURN_SLIGHT_RIGHT: {"turn": "right", "word": None, "where": "ahead"},
}

ALLOWED = (
    (
        WALK_STRAIGHT,
        WALL_AHEAD,
        GLASS_AHEAD,
        BLOCKED_WALL,
        BLOCKED_GLASS,
        TURN_LEFT,
        TURN_RIGHT,
        DRIFT_LEFT,
        DRIFT_RIGHT,
        NEAR_LEFT,
        NEAR_RIGHT,
        GLASS_LEFT,
        GLASS_RIGHT,
        SENSORS_OFFLINE,
        VEHICLE_FROM_LEFT,
        VEHICLE_FROM_RIGHT,
        VEHICLE_APPROACHING,
        VEHICLE_AHEAD,
    )
    + _GLASS_NOTICES
    + _MOVER_SENTENCES
    + tuple(OBJECT_AHEAD.values())
    + tuple(OBJECT_IN_FRONT.values())
    + tuple(STEER_PAST.values())
    + tuple(STEER_IN_FRONT.values())
    + _BETWEEN
    + _CAMERA_NOTICES
    + (TURN_SLIGHT_LEFT, TURN_SLIGHT_RIGHT, TURN_AROUND)
    + (OBSTACLE_AHEAD, STOP_NO_WAY, TRAFFIC_LIGHT_AHEAD)
    + _STARTUP_NOTICES
    + _SYSTEM_NOTICES
)

_ACTION = {
    WALK_STRAIGHT: "walk_straight",
    WALL_AHEAD: "wall_ahead",
    GLASS_AHEAD: "glass_ahead",
    BLOCKED_WALL: "blocked_wall",
    BLOCKED_GLASS: "blocked_glass",
    TURN_LEFT: "turn_left",
    TURN_RIGHT: "turn_right",
    TURN_AROUND: "turn_around",
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
    CAMERA_OFF: "camera_off",
    CAMERA_ON: "camera_on",
    OBSTACLE_AHEAD: "obstacle_ahead",
    STOP_NO_WAY: "stop",
    TRAFFIC_LIGHT_AHEAD: "traffic_light_ahead",
    STARTING: "starting",
    READY: "ready",
    POWER_LOW: "power_low",
    POWER_NORMAL: "power_normal",
    DEVICE_HOT: "device_hot",
    DEVICE_COOL: "device_cool",
    **{t: "object_ahead" for t in OBJECT_AHEAD.values()},
    **{t: "object_in_front" for t in OBJECT_IN_FRONT.values()},
    **{t: "steer_" + i["turn"] for t, i in _STEER_INFO.items() if i["turn"]},
    BETWEEN_STRAIGHT: "walk_between",
    **{
        t: (
            "mover_move_" + i["move"]
            if i["move"]
            else "mover_warn"
            if t in MOVER_BEHIND_ONLY.values()
            else "mover_stop"
        )
        for t, i in _MOVER_INFO.items()
    },
}

_LEVEL = {
    WALK_STRAIGHT: 0,
    WALL_AHEAD: 1,
    GLASS_AHEAD: 1,
    DRIFT_LEFT: 1,
    DRIFT_RIGHT: 1,
    NEAR_LEFT: 1,
    NEAR_RIGHT: 1,
    GLASS_LEFT: 1,
    GLASS_RIGHT: 1,
    BLOCKED_WALL: 2,
    BLOCKED_GLASS: 2,
    TURN_LEFT: 2,
    TURN_RIGHT: 2,
    TURN_AROUND: 2,
    STOP_NO_WAY: 2,
    SENSORS_OFFLINE: 2,
    OBSTACLE_AHEAD: 1,
    GLASS_SENSOR_OFF: 1,
    GLASS_SENSOR_ON: 1,
    CAMERA_OFF: 1,
    CAMERA_ON: 1,
    TRAFFIC_LIGHT_AHEAD: 1,
    STARTING: 1,
    READY: 1,
    **{t: 1 for t in _SYSTEM_NOTICES},
    **{t: 1 for t in OBJECT_AHEAD.values()},
    **{t: 2 for t in OBJECT_IN_FRONT.values()},
    # A stopped vehicle, the user standing still: an advisory, said once.
    VEHICLE_AHEAD: 1,
    VEHICLE_FROM_LEFT: 3,
    VEHICLE_FROM_RIGHT: 3,
    VEHICLE_APPROACHING: 3,
    # Said the moment the sentence playing ends; the camera's vehicle cuts in.
    **{t: 2 for t in _MOVER_SENTENCES},
    **{t: 1 for t in STEER_PAST.values()},
    **{t: 1 for t in _BETWEEN},
    TURN_SLIGHT_LEFT: 1,
    TURN_SLIGHT_RIGHT: 1,
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
        return 0.0  # device silence is not room
    distance = view.get("distance_ft")
    if distance is None:
        return float("inf")  # no return = nothing out there = open
    try:
        return float(distance)
    except (TypeError, ValueError):
        return 0.0


def safer_side(
    walk_decision: Dict[str, Any],
    previous: Optional[str] = None,
    lidar_state: Optional[Dict[str, Any]] = None,
) -> str:
    """Which of left/right to turn toward when the front is blocked."""
    chosen = walk_decision.get("direction")
    if chosen in ("left", "right"):
        return chosen
    # A side inside the LiDAR's blind arc has only the radar's word for its room (it misses
    # poles): turn the way the laser can see when that has room (2026-10-21).
    views = walk_decision.get("directions") or {}
    blind = {s for s in ("left", "right") if (views.get(s) or {}).get("lidar_blind")}
    if len(blind) == 1:
        seen = "right" if "left" in blind else "left"
        # The same hysteresis as below: easier to stay, harder to switch away.
        need = (
            WALK_MIN_TURN_FT
            - (WALK_TURN_HYSTERESIS_FT if previous == seen else 0.0)
            + (WALK_TURN_HYSTERESIS_FT if previous in blind else 0.0)
        )
        if _clearance(walk_decision, seen) >= need:
            return seen
    if previous in ("left", "right"):
        other = "right" if previous == "left" else "left"
        if (
            _clearance(walk_decision, other)
            <= _clearance(walk_decision, previous) + WALK_TURN_HYSTERESIS_FT
        ):
            return previous
    left = _clearance(walk_decision, "left")
    right = _clearance(walk_decision, "right")
    try:
        from utils import spatial_scan

        side = spatial_scan.preferred_side(
            spatial_scan.live_snapshot(lidar_state or {})
        )
        if side in ("left", "right"):
            room, other = (left, right) if side == "left" else (right, left)
            if room >= WALK_MIN_TURN_FT and room + WALK_TURN_HYSTERESIS_FT >= other:
                return side
    except Exception:
        pass  # advisory layer: a fault here costs nothing but the hint

    return "right" if right > left else "left"


def _side_room(
    walk_decision: Dict[str, Any],
    lidar_state: Optional[Dict[str, Any]],
    fresh: bool,
    side: str,
) -> float:
    """Room to step to one side, in feet: the walk policy's clearance that way and, on a fresh scan, the laser's whole 45 deg sector — a sideways step needs the sector, not just the narrow steering cone."""
    room = _clearance(walk_decision, side)
    if fresh:
        mm = (((lidar_state or {}).get("sectors") or {}).get(side) or {}).get(
            "min_dist"
        )
        if isinstance(mm, (int, float)) and mm > 0:
            room = min(room, mm / MM_PER_FT)
    return room


def _fresh(lidar_state: Optional[Dict[str, Any]], now: float) -> bool:
    """Is this scan EVIDENCE — of a wall, a pane, a side wall, room to step?"""
    from utils.freshness import reading_is_current
    from utils.sensor_fusion import lidar_sparse

    return (
        bool(lidar_state)
        and not lidar_state.get("simulated")
        and reading_is_current(lidar_state, LIDAR_STALE_S, now)
        and not lidar_sparse(lidar_state)
    )


def _not_updating(lidar_state: Dict[str, Any], now: float) -> Optional[str]:
    """Why "Sensors offline" must be said now, or None."""
    from utils.freshness import (
        age_s,
        describe_reading,
        reading_is_current,
        received_within,
    )

    if lidar_state.get("simulated"):
        return "the LiDAR is simulated — not a measurement"
    if not reading_is_current(lidar_state, SIMPLE_LIDAR_FRESH_S, now):
        age = age_s(lidar_state.get("last_update"), now)
        if (
            age is not None
            and age < 0.0
            and received_within(lidar_state, SIMPLE_LIDAR_FRESH_S)
        ):
            return None
        return (
            f"the LiDAR is not updating "
            f"(newest scan {describe_reading(lidar_state, now)})"
        )
    from utils.sensor_fusion import lidar_sparse

    return lidar_sparse(lidar_state)  # no returns is not open space (NEW-043)


def _front_cluster(
    lidar_state: Dict[str, Any], cone_only: bool = False
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
            and abs(dist * math.sin(math.radians(gap)))
            <= GUIDANCE_CORRIDOR_HALF_WIDTH_MM
        )
        if in_cone or in_corridor:
            bucket.append((float(dist), float(angle)))
    nearest = _cluster_min(bucket)["min_dist"]
    if nearest is None:
        return None, []
    cluster = [
        (a, d) for d, a in bucket if nearest <= d <= nearest + LIDAR_CLUSTER_WINDOW_MM
    ]
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
    return lo - SIMPLE_ON_WALL_MM <= along <= hi + SIMPLE_ON_WALL_MM and off <= tol


def _front_wall(
    walk_decision: Dict[str, Any],
    lidar_state: Optional[Dict[str, Any]],
    now: float,
    radius_ft: float,
    surfaces: Optional[List[Dict[str, Any]]] = None,
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
            return None  # a different scan from walk_policy's
        if surfaces is None:
            surfaces = _surfaces(lidar_state)
        on_line_mm = _on_line_mm(lidar_state)
    except Exception:
        return None  # no wall named beats a crashed voice
    walls = [w for w in surfaces if w.get("kind") == "wall"]
    on = sum(1 for a, d in cluster if any(_on_surface(a, d, w) for w in walls))
    if on * 2 <= len(cluster):
        return None
    lateral = sum(
        d * math.sin(math.radians(signed_bearing(a))) for a, d in cluster
    ) / len(cluster)
    ft = nearest_mm / MM_PER_FT
    clear_to = max(radius_ft, ft + SIMPLE_WALL_MATCH_FT)
    return {
        "ft": ft,
        "alongside": on_line_mm is None or on_line_mm / MM_PER_FT > clear_to,
        "side": "right" if lateral >= 0 else "left",
    }


def _brushing(
    lidar_state: Optional[Dict[str, Any]], surface: Dict[str, Any], radius_ft: float
) -> bool:
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


def side_walls(
    surfaces: List[Dict[str, Any]], holding: bool = False
) -> Dict[str, Dict[str, Any]]:
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
            and float(surface.get("span_mm", 0.0)) >= SIMPLE_SIDE_WALL_HOLD_SPAN_MM
        ):
            continue
        cx, cy = surface["centroid_mm"]
        dx, dy = surface["direction"]
        lo, hi = surface["extent_mm"]
        if dx < 0:
            dx, dy, lo, hi = -dx, -dy, -hi, -lo
        if dx < math.cos(math.radians(60.0)):
            continue  # across the path, not beside it
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
        slope = dy / dx  # lateral change per mm forward
        closing = -slope if side == "right" else slope
        if math.degrees(math.atan(max(closing, 0.0))) < SIMPLE_DRIFT_MIN_DEG:
            continue  # parallel, or opening away
        reach_mm = max(0.0, (abs(py) - edge) / closing)
        if px + reach_mm > SIMPLE_DRIFT_AHEAD_FT * MM_PER_FT:
            continue
        entry["converging"] = True
    return out


def _theil_sen(samples: List[Tuple[float, float]]) -> Optional[float]:
    """Median of the pairwise slopes of (t, value) samples: a trend that one bad sample cannot make on its own."""
    slopes = sorted(
        (v2 - v1) / (t2 - t1)
        for i, (t1, v1) in enumerate(samples)
        for t2, v2 in samples[i + 1 :]
        if t2 - t1 > 1e-6
    )
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
            return  # an older snapshot (_CLOCK_STEP_S)
        for side, samples in self._samples.items():
            if samples and scan_t < samples[-1][0] - 1e-6:
                samples.clear()  # the clock went backwards
            samples[:] = [p for p in samples if scan_t - p[0] <= SIMPLE_DRIFT_WINDOW_S]
            wall = walls.get(side)
            if wall is None:
                continue
            mm = float(wall.get("line_ft", wall["ft"])) * MM_PER_FT
            if samples and scan_t <= samples[-1][0] + 1e-6:
                continue  # the same scan, polled again
            if samples and abs(mm - samples[-1][1]) > SIMPLE_DRIFT_JUMP_MM:
                samples.clear()  # a different wall
            samples.append((scan_t, mm))

    def rate_ft_s(self, side: str) -> Optional[float]:
        """Closing speed on that side's wall in ft/s (positive = getting closer), or None while the history is too short to say."""
        samples = self._samples.get(side) or []
        if (
            len(samples) < 4
            or samples[-1][0] - samples[0][0] < SIMPLE_DRIFT_MIN_SPAN_S - 1e-6
        ):
            return None
        slope = _theil_sen(samples)  # mm/s
        return None if slope is None else -slope / MM_PER_FT

    def closing(self, side: str) -> bool:
        rate = self.rate_ft_s(side)
        return rate is not None and rate >= SIMPLE_DRIFT_CLOSING_FT_S - 1e-9

    def reset(self) -> None:
        self.__init__()


def glass_beside(radar_state: Optional[Dict[str, Any]]) -> Optional[str]:
    """ "left" / "right" when the front radar echo comes from ONE board only."""
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
        reads[side] = (
            float(dist) / MM_PER_FT if isinstance(dist, (int, float)) and dist else None
        )
    if set(reads) != {"left", "right"}:
        return None
    left, right = reads["left"], reads["right"]
    if (left is None) == (right is None):
        return None
    # Nearer than RADAR_CONE_TRUST_MM the pane is inside the walking corridor, whichever board heard it.
    if (left if left is not None else right) * MM_PER_FT <= RADAR_CONE_TRUST_MM:
        return None
    return "left" if left is not None else "right"


_QUAD_W = CAMERA_WIDTH / FRAME_GRID_COLS  # one camera's frame in the mosaic
_QUAD_H = CAMERA_HEIGHT / FRAME_GRID_ROWS


def _band() -> Tuple[float, float]:
    """The front camera's box, in pixels: the middle SIMPLE_OBJECT_CENTRE_BAND."""
    return (
        _QUAD_W * (0.5 - SIMPLE_OBJECT_CENTRE_BAND / 2.0),
        _QUAD_W * (0.5 + SIMPLE_OBJECT_CENTRE_BAND / 2.0),
    )


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
    return float(
        SIMPLE_VEHICLE_HEIGHT_M.get(cls) or SIMPLE_OBJECT_HEIGHT_M.get(cls) or 1.5
    )


def _size_range(cls: str, box: Tuple[float, float, float, float]) -> Tuple[float, bool]:
    """(range in metres from the box's height and the class's typical height through the assumed lens, whether the frame cuts the box off)."""
    x1, y1, x2, y2 = box
    focal_px = (_QUAD_W / 2.0) / math.tan(math.radians(SIMPLE_CAMERA_HFOV_DEG) / 2.0)
    cut = y1 <= 2.0 or y2 >= _QUAD_H - 2.0
    return focal_px * _object_height_m(cls) / max(1.0, y2 - y1), cut


def _box_range(
    det: Dict[str, Any], cls: str, box: Tuple[float, float, float, float]
) -> Tuple[float, bool]:
    """(range in metres, cut off?) of a front detection: the ToF's measured distance of the
    nearest surface in its box (utils/depth_fusion.py) with TOF_USE_DISTANCE, else the
    box-size estimate. A measured distance is never "cut off"."""
    depth = det.get("depth") if TOF_USE_DISTANCE else None
    if isinstance(depth, dict) and isinstance(depth.get("distance_m"), (int, float)):
        return float(depth["distance_m"]), False
    return _size_range(cls, box)


def _range_matches(est_m: float, front_m: Optional[float], cut: bool) -> bool:
    """Is a camera thing at `est_m` (by its size) plausibly what the rangefinders range at `front_m`?"""
    if front_m is None or front_m <= 0:
        return False
    ratio = est_m / front_m
    return ratio >= 1.0 / SIMPLE_VEHICLE_RANGE_MATCH and (
        cut or ratio <= SIMPLE_VEHICLE_RANGE_MATCH
    )


def front_object(
    camera_state: Optional[Dict[str, Any]], now: float, front_ft: Optional[float] = None
) -> Optional[str]:
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
            continue  # off to one side
        if (y2 - y1) < SIMPLE_OBJECT_MIN_HEIGHT_FRAC * height:
            continue  # far away
        if front_ft is not None:
            est, cut = _box_range(
                det, str(det.get("class_name", "")).lower(), (x1, y1, x2, y2)
            )
            if not _range_matches(est, front_ft * MM_PER_FT / 1000.0, cut):
                continue  # not the thing being ranged
        rank = (
            str(det.get("class_name", "")).lower() in SIMPLE_VEHICLE_CLASSES,
            (x2 - x1) * (y2 - y1),
        )
        if best is None or rank > best[0]:
            best = (rank, word)
    return best[1] if best else None


def ranged_vehicle(
    camera_state: Optional[Dict[str, Any]], now: float, front_ft: Optional[float]
) -> Optional[str]:
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
        range_m, cut = _box_range(det, cls, (x1, y1, x2, y2))
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
        return (
            motion if time.monotonic() - float(mono) <= HEADING_STATE_STALE_S else None
        )
    if "updated_at" in motion:
        return (
            motion
            if is_fresh(motion.get("updated_at"), HEADING_STATE_STALE_S)
            else None
        )
    return motion


def user_motion(motion: Optional[Dict[str, Any]]) -> Tuple[Optional[bool], float]:
    """(standing still?, walking pace m/s) from heading_tracker's state. Still is True only when MEASURED still; when nothing can tell (open space, no gait reading) it is None with SIMPLE_VEHICLE_PACE_UNKNOWN_MPS — until 2026-10-21 that was "still, pace 0", and a parked car walked toward read as one approaching."""
    motion = current_motion(motion)  # a stale state is none (NEW-026)
    if not motion:
        return None, SIMPLE_VEHICLE_PACE_UNKNOWN_MPS
    walking = bool(motion.get("moving_confirmed") or motion.get("walking_confirmed"))
    if walking:
        return False, max(0.0, float(motion.get("speed_mps") or 0.0))
    if motion.get("confidently_still"):
        return True, 0.0
    return None, SIMPLE_VEHICLE_PACE_UNKNOWN_MPS


_NO_VEHICLE: Dict[str, Any] = {
    "approaching": False,
    "side": None,
    "stopped": False,
    "moving": False,
    "class": None,
    "ttc_s": None,
    "miss_m": None,
    "growth": None,
    "own_growth": None,
    "box": None,
    "tracks": 0,
}


class _Vehicles:
    """Vehicles in the front camera, followed from one detector frame to the next."""

    def __init__(self):
        self._tracks: List[Dict[str, Any]] = []
        self._frame: Optional[float] = None
        self._seen: Dict[str, Any] = dict(_NO_VEHICLE)

    def update(
        self,
        camera_state: Optional[Dict[str, Any]],
        now: float,
        pace_mps: float,
        yaw_dps: float = 0.0,
    ) -> Dict[str, Any]:
        cam = camera_state or {}
        if not _camera_sees(cam, now):
            self._tracks, self._frame, self._seen = [], None, dict(_NO_VEHICLE)
            return self._seen
        stamp = _front_stamp(cam)
        if self._frame is not None and stamp < self._frame - _CLOCK_STEP_S:
            self._tracks = []  # the clock went backwards
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
            if (
                x2 <= x1
                or y2 <= y1
                or (y2 - y1) < SIMPLE_VEHICLE_MIN_HEIGHT_FRAC * _QUAD_H
            ):
                continue
            out.append({"cls": cls, "box": (x1, y1, x2, y2)})
        return out

    def _follow(self, dets: List[Dict[str, Any]], t: float) -> None:
        """Match this frame's vehicles to the tracks by box overlap."""

        def iou(a, b):
            ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
            iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
            inter = ix * iy
            union = (
                (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
            )
            return inter / union if union > 0 else 0.0

        pairs = sorted(
            (
                (iou(tr["samples"][-1][1], d["box"]), ti, di)
                for ti, tr in enumerate(self._tracks)
                for di, d in enumerate(dets)
            ),
            reverse=True,
        )
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
                self._tracks.append(
                    {
                        "samples": [(t, d["box"])],
                        "cls": d["cls"],
                        "side": None,
                        "approach_frames": 0,
                        "moving_frames": 0,
                    }
                )
        kept = []
        for tr in self._tracks:
            if t - tr["samples"][-1][0] > SIMPLE_VEHICLE_LOST_S:
                continue
            tr["samples"] = [
                p for p in tr["samples"] if t - p[0] <= SIMPLE_VEHICLE_WINDOW_S
            ]
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
                continue  # not seen on this frame
            x1, y1, x2, y2 = samples[-1][1]
            if x2 < lo or x1 > hi:
                tr["approach_frames"] = tr["moving_frames"] = 0
                continue  # outside the box
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
            size = (
                (lambda b: b[3] - b[1])
                if cut_lr
                else (lambda b: b[2] - b[0])
                if cut_tb
                else (lambda b: math.sqrt((b[2] - b[0]) * (b[3] - b[1])))
            )
            growth = _theil_sen([(ts, math.log(size(b))) for ts, b in samples])
            slide = _theil_sen(
                [(ts, (b[0] + b[2]) / 2.0 / _QUAD_W) for ts, b in samples]
            )
            if growth is None or slide is None:
                continue
            height_m = SIMPLE_VEHICLE_HEIGHT_M.get(tr["cls"], 1.5)
            heights = sorted(b[3] - b[1] for _, b in samples)
            h_px = heights[len(heights) // 2]
            range_m = focal_px * height_m / h_px
            own = pace_mps / range_m
            # Nearer edge's offset from the walking line, in metres.
            misses = sorted(
                0.0
                if b[0] <= mid <= b[2]
                else height_m * min(abs(b[0] - mid), abs(b[2] - mid)) / (b[3] - b[1])
                for _, b in samples
            )
            miss_m = misses[len(misses) // 2]
            coming = (
                growth - SIMPLE_VEHICLE_EGO_MARGIN * own >= SIMPLE_VEHICLE_APPROACH_RATE
                and miss_m <= SIMPLE_VEHICLE_MISS_M
            )
            tr["approach_frames"] = tr["approach_frames"] + 1 if coming else 0
            approach = tr["approach_frames"] >= SIMPLE_VEHICLE_CONFIRM_FRAMES
            across = slide - (cx / _QUAD_W - 0.5) * growth - turn_slide
            crossing = (
                abs(across) > SIMPLE_VEHICLE_IDLE_SLIDE
                and abs(turn_slide) <= 2.0 * SIMPLE_VEHICLE_IDLE_SLIDE
            )
            closing = (
                growth - SIMPLE_VEHICLE_EGO_MARGIN * own >= SIMPLE_VEHICLE_IDLE_RATE
                and miss_m <= SIMPLE_VEHICLE_CAREFUL_MISS_M
            )
            tr["moving_frames"] = (
                tr["moving_frames"] + 1 if (crossing or closing) else 0
            )
            seen = {
                "approaching": approach,
                "side": side,
                "stopped": (
                    abs(growth) <= SIMPLE_VEHICLE_IDLE_RATE
                    and abs(slide) <= SIMPLE_VEHICLE_IDLE_SLIDE
                ),
                "moving": (
                    not approach
                    and tr["moving_frames"] >= SIMPLE_VEHICLE_CONFIRM_FRAMES
                    and h_px >= SIMPLE_VEHICLE_CAREFUL_HEIGHT_FRAC * _QUAD_H
                ),
                "class": tr["cls"],
                "ttc_s": round(1.0 / growth, 1) if growth > 1e-3 else None,
                "miss_m": round(miss_m, 2),
                "growth": round(growth, 3),
                "own_growth": round(own, 3),
                "box": [round(v) for v in samples[-1][1]],
            }
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
        pick = (
            max(moving or stopped, key=lambda p: p[0])[1]
            if (moving or stopped)
            else None
        )
        if pick is None:
            return {**_NO_VEHICLE, "tracks": n}
        return {**pick, "moving": bool(moving), "stopped": bool(stopped), "tracks": n}


def camera_path(
    camera_state: Optional[Dict[str, Any]],
    now: float,
    previous_turn: Optional[str] = None,
) -> Optional[Dict[str, Any]]:
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
        ppm_px = (y2 - y1) / _object_height_m(cls)  # pixels per metre
        range_m, cut = _box_range(det, cls, (x1, y1, x2, y2))
        if range_m > SIMPLE_STEER_RANGE_M and not cut:
            continue
        things.append(
            {
                "word": word,
                "x1": x1 / _QUAD_W,
                "x2": x2 / _QUAD_W,
                "need": SIMPLE_STEER_GAP_M * ppm_px / _QUAD_W,
                "range_m": range_m,
                "cut": cut,
                "vehicle": cls in SIMPLE_VEHICLE_CLASSES,
            }
        )
    if not things:
        return None
    things.sort(key=lambda t: t["x1"])
    in_way = [
        t
        for t in things
        if t["x2"] > 0.5 - t["need"] / 2.0 and t["x1"] < 0.5 + t["need"] / 2.0
    ]
    lo, hi = SIMPLE_OBJECT_CENTRE_BAND / 2.0, 1.0 - SIMPLE_OBJECT_CENTRE_BAND / 2.0
    if not in_way:
        left = [t for t in things if t["x2"] <= 0.5 and t["x2"] > 0.5 - (0.5 - lo)]
        right = [t for t in things if t["x1"] >= 0.5 and t["x1"] < hi]
        if not (left and right):
            return None
        near = min(left + right, key=lambda t: t["range_m"])
        return {
            "kind": "between",
            "turn": None,
            "where": "both",
            "word": near["word"],
            "target": 0.5,
            "range_m": round(near["range_m"], 2),
            "cut": near["cut"],
            "in_way": 0,
        }

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
            candidates.append(
                {"kind": "between", "target": (edge_a["x2"] + edge_b["x1"]) / 2.0}
            )
    first = min(groups[0], key=lambda m: m["x1"])
    last = max(groups[-1], key=lambda m: m["x2"])
    candidates.append({"kind": "around", "target": first["x1"] - first["need"] / 2.0})
    candidates.append({"kind": "around", "target": last["x2"] + last["need"] / 2.0})

    def open_frame(left_side: bool) -> float:
        span = (0.0, 0.5) if left_side else (0.5, 1.0)
        covered = sum(
            max(0.0, min(t["x2"], span[1]) - max(t["x1"], span[0])) for t in things
        )
        return (span[1] - span[0]) - covered

    def rank(c):
        on_left = c["target"] < 0.5
        return (abs(c["target"] - 0.5), -open_frame(on_left))

    best = min(candidates, key=rank)
    if previous_turn in ("left", "right"):
        same = [
            c for c in candidates if (c["target"] < 0.5) == (previous_turn == "left")
        ]
        if same:
            keep = min(same, key=rank)
            if (
                abs(keep["target"] - 0.5)
                <= abs(best["target"] - 0.5) + SIMPLE_STEER_STICKY
            ):
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
            return (
                "ahead"
                if abs(c - 0.5) <= t["need"] / 4.0
                else "left"
                if c < 0.5
                else "right"
            )

        away = "left" if turn == "right" else "right" if turn == "left" else None
        where = away if (away and all(side_of(t) == away for t in in_way)) else "ahead"
    return {
        "kind": best["kind"],
        "turn": turn,
        "where": where,
        "word": nearest["word"],
        "target": round(best["target"], 3),
        "range_m": round(nearest["range_m"], 2),
        "cut": nearest["cut"],
        "in_way": len(in_way),
    }


def vehicle_move_blocked(
    walk_decision: Optional[Dict[str, Any]],
    lidar_state: Optional[Dict[str, Any]],
    fresh: bool,
    vehicle: Optional[str],
) -> bool:
    """True when the move a vehicle warning would give — AWAY from the vehicle's side — has under SIMPLE_STEER_SIDE_MIN_FT of room, or no reading at all (decided 2026-09-29, audit NEW-049)."""
    if vehicle not in ("left", "right"):
        return False
    return _step_blocked(
        walk_decision, lidar_state, fresh, "right" if vehicle == "left" else "left"
    )


def _step_blocked(
    walk_decision: Optional[Dict[str, Any]],
    lidar_state: Optional[Dict[str, Any]],
    fresh: bool,
    move: str,
) -> bool:
    """True when a step `move` (left/right) has under SIMPLE_STEER_SIDE_MIN_FT of room, or no reading."""
    if not ((walk_decision or {}).get("directions") or {}).get(move):
        return True  # nothing reports that side
    return (
        _side_room(walk_decision, lidar_state, fresh, move) < SIMPLE_STEER_SIDE_MIN_FT
    )


_FT_PER_M = 1000.0 / MM_PER_FT


def _in_ft(metres: Any) -> Optional[float]:
    """Metres (or m/s) as feet (or ft/s), to 0.1; None stays None."""
    return (
        round(float(metres) * _FT_PER_M, 1)
        if isinstance(metres, (int, float))
        else None
    )


_PERSON_CLASSES = frozenset(
    c.lower() for c, w in SIMPLE_NAMED_OBJECTS.items() if w == "Person"
)


def _camera_word(
    camera_state: Optional[Dict[str, Any]], now: float, bearing_deg: float
) -> Optional[str]:
    """ "Person" / "Vehicle" when the front camera has one at this bearing (LIDAR_MOVERS_CAMERA_MATCH_DEG), else None."""
    cam = camera_state or {}
    if abs(bearing_deg) > SIMPLE_CAMERA_HFOV_DEG / 2.0 or not _camera_sees(cam, now):
        return None
    focal_px = (_QUAD_W / 2.0) / math.tan(math.radians(SIMPLE_CAMERA_HFOV_DEG) / 2.0)
    best = None
    for det in (cam.get("front") or {}).get("detections") or []:
        cls = str(det.get("class_name", "")).lower()
        word = (
            "Vehicle"
            if cls in SIMPLE_VEHICLE_CLASSES
            else "Person"
            if cls in _PERSON_CLASSES
            else None
        )
        if word is None or float(det.get("confidence") or 0.0) < SIMPLE_OBJECT_MIN_CONF:
            continue
        try:
            x1, _y1, x2, _y2 = (float(v) for v in det.get("bbox") or ())
        except (TypeError, ValueError):
            continue
        off = abs(
            math.degrees(math.atan(((x1 + x2) / 2.0 - _QUAD_W / 2.0) / focal_px))
            - bearing_deg
        )
        if off <= LIDAR_MOVERS_CAMERA_MATCH_DEG and (best is None or off < best[0]):
            best = (off, word)
    return best[1] if best else None


def mover_sentence(
    threat: Optional[Dict[str, Any]],
    walk_decision: Optional[Dict[str, Any]],
    lidar_state: Optional[Dict[str, Any]],
    fresh: bool,
    camera_state: Optional[Dict[str, Any]],
    now: float,
) -> Tuple[Optional[str], str]:
    """(the sentence for the LiDAR's most urgent mover, why), or (None, "")."""
    if not threat or threat.get("mode") not in ("crossing", "toward", "behind"):
        return None, ""
    seen = _camera_word(camera_state, now, float(threat.get("bearing_deg") or 0.0))
    word = seen or threat.get("word")
    if word not in _MOVER_WORDS:
        return None, ""
    mode, side = threat["mode"], threat.get("side")
    ft = lambda key: float(threat.get(key) or 0.0) * _FT_PER_M  # metres -> feet
    if threat.get("source") == "radar":
        why = (
            f"{word.lower()}{' (camera)' if seen else ''} only the radar hears, closing "
            f"{ft('closing_mps'):.1f} ft/s, {ft('distance_m'):.1f} ft away "
            f"where the LiDAR sees nothing — reaches you in {threat.get('ttc_s', 0):.1f} s"
        )
    else:
        why = (
            f"{word.lower()}{' (camera)' if seen else ''} moving {ft('own_speed_mps'):.1f} ft/s, "
            f"{ft('distance_m'):.1f} ft away, passes within {ft('miss_m'):.1f} ft "
            f"in {threat.get('ttc_s', 0):.1f} s"
        )
    if mode == "crossing":
        return MOVER_CROSSING.get(
            (word, side)
        ), f"{why} — crossing from your {side}: stop, let it pass"
    # Step away from the side it will pass on; when it comes dead centre, to the roomier side.
    away = {"left": "right", "right": "left"}.get(threat.get("pass_side"))
    options = (
        [away]
        if away
        else sorted(
            ("left", "right"),
            key=lambda s: -_side_room(walk_decision or {}, lidar_state, fresh, s),
        )
    )
    move = next(
        (s for s in options if not _step_blocked(walk_decision, lidar_state, fresh, s)),
        None,
    )
    where = "ahead, coming at you" if mode == "toward" else "behind you"
    room = f": step {move}" if move else ": no room to step aside"
    if mode == "toward":
        text = MOVER_TOWARD.get((word, move)) if move else MOVER_TOWARD_STOP.get(word)
    else:
        text = MOVER_BEHIND.get((word, move)) if move else MOVER_BEHIND_ONLY.get(word)
    return text, f"{why} — {where}{room}"


def _steer_text(steer: Optional[Dict[str, Any]], blocked: bool) -> Optional[str]:
    """The sentence for a camera_path plan, or None when it has none."""
    if not steer:
        return None
    turn, word = steer.get("turn"), steer.get("word")
    if blocked:
        return STEER_IN_FRONT.get((word, turn)) if turn else None
    if steer.get("kind") == "between":
        return (
            BETWEEN_LEFT
            if turn == "left"
            else BETWEEN_RIGHT
            if turn == "right"
            else BETWEEN_STRAIGHT
        )
    if steer.get("kind") == "route":
        named = STEER_PAST.get((word, steer.get("where"), turn)) if word else None
        return named or (
            TURN_SLIGHT_LEFT
            if turn == "left"
            else TURN_SLIGHT_RIGHT
            if turn == "right"
            else None
        )
    return STEER_PAST.get((word, steer.get("where"), turn))


def _steer_allowed(
    steer: Dict[str, Any],
    side_room: Dict[str, Any],
    blocked: bool,
    front_ft: Optional[float],
    turn_side: Optional[str] = None,
) -> bool:
    """May this way past be said now?"""
    turn = steer.get("turn")
    room = side_room.get(turn) if turn else None
    if isinstance(room, (int, float)) and room < SIMPLE_STEER_SIDE_MIN_FT:
        return False
    if blocked:
        front_m = None if front_ft is None else front_ft * MM_PER_FT / 1000.0
        if not turn or not _range_matches(
            steer["range_m"], front_m, bool(steer.get("cut"))
        ):
            return False
        if turn_side in ("left", "right") and turn != turn_side:

            def ft(v: Any) -> float:
                return float("inf") if v is None else float(v)  # None: open

            mine, chosen = ft(side_room.get(turn)), ft(side_room.get(turn_side))
            if mine < WALK_MIN_TURN_FT or mine + WALK_TURN_HYSTERESIS_FT < chosen:
                return False
    return True


def _sentence(
    blocked: bool,
    near: bool,
    glass: bool,
    wall: bool,
    side: Optional[str],
    drift: Optional[str] = None,
    obj: Optional[str] = None,
    close: Optional[str] = None,
    vehicle: Optional[str] = None,
    vehicle_ahead: bool = False,
    steer: Optional[Dict[str, Any]] = None,
    vehicle_no_room: bool = False,
    glass_side: Optional[str] = None,
    mover: Optional[str] = None,
    obstacle: bool = False,
) -> str:
    """A vehicle coming at the user, above everything."""
    if vehicle:
        if vehicle_no_room:
            return VEHICLE_APPROACHING
        return VEHICLE_FROM_LEFT if vehicle == "left" else VEHICLE_FROM_RIGHT
    if mover:  # already a sentence (mover_sentence)
        return mover
    if blocked:
        # No way out is said before naming what is in the way (2026-10-21).
        if side == "stop":
            return STOP_NO_WAY
        if glass:
            return BLOCKED_GLASS
        if wall:
            return BLOCKED_WALL
        # A vehicle further off no longer replaces the block sentence (2026-10-21): the
        # thing in the way is what the user must act on; a vehicle IN the way is named by
        # ranged_vehicle as "Vehicle in front".
        if close:
            return NEAR_LEFT if close == "left" else NEAR_RIGHT
        if drift:
            return DRIFT_LEFT if drift == "left" else DRIFT_RIGHT
        past = _steer_text(steer, True)
        if past:
            return past
        if obj in OBJECT_IN_FRONT:
            return OBJECT_IN_FRONT[obj]
        if side == "around":
            return TURN_AROUND
        return TURN_LEFT if side != "right" else TURN_RIGHT
    if vehicle_ahead:
        return VEHICLE_AHEAD
    # A pane on the path outranks a wall beside you, which a 4 ft near-wall band makes common (2026-10-01).
    if near and glass:
        return GLASS_AHEAD
    # Never "move" toward glass beside you (2026-10-22): with a wall near on the other side
    # the glass is the one to keep clear of — glass_side is only set when that side has room.
    if glass_side and close and close != glass_side:
        return GLASS_LEFT if glass_side == "left" else GLASS_RIGHT
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
    if obstacle:
        return OBSTACLE_AHEAD
    return WALK_STRAIGHT


def decide(
    walk_decision: Optional[Dict[str, Any]] = None,
    lidar_state: Optional[Dict[str, Any]] = None,
    now: Optional[float] = None,
    previous_side: Optional[str] = None,
    radar_state: Optional[Dict[str, Any]] = None,
    camera_state: Optional[Dict[str, Any]] = None,
    approach: Optional[_Approach] = None,
    motion: Optional[Dict[str, Any]] = None,
    vehicles: Optional[_Vehicles] = None,
    previous_steer: Optional[str] = None,
    movers: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """What this one poll's readings say, and the sentence they map to."""
    walk_decision = walk_decision or {}
    now = time.time() if now is None else now
    motion = current_motion(motion)  # stale motion is no motion (NEW-026)
    still, pace = user_motion(motion)
    yaw = float((motion or {}).get("yaw_rate_dps") or 0.0)
    seen = (
        vehicles.update(camera_state, now, pace, yaw)
        if vehicles is not None
        else dict(_NO_VEHICLE)
    )
    vehicle = seen["side"] if seen["approaching"] else None
    vehicle_ahead = vehicle is None and (
        bool(seen.get("moving")) or (bool(seen["stopped"]) and still is True)
    )

    if walk_decision.get("action") in (None, "", "unknown"):
        return {
            "instruction": "",
            "action": "unknown",
            "rule": "waiting",
            "side": None,
            "obstacle": None,
            "blocked": False,
            "near": False,
            "glass": False,
            "wall": False,
            "drift": None,
            "close": None,
            "close_band": [],
            "beside": {},
            "closing": {},
            "vehicle": None,
            "vehicle_ahead": False,
            "vehicles": seen,
            "steer": None,
            "object": None,
            "front_ft": None,
            "obstacle_ahead": False,
            "obstacle_band": False,
            "reason": "front not measured yet",
        }

    blocked = bool(walk_decision.get("blocked"))
    front = _front(walk_decision)
    front_ft = front.get("distance_ft")
    front_ft = None if front_ft is None else float(front_ft)

    glass = bool(walk_decision.get("glass_ahead")) and _fresh(lidar_state, now)
    beside = glass_beside(radar_state) if glass else None
    beside_ft = None
    if beside:
        # Read as beside only where the user can step away from it: "Glass on left, move
        # right" needs room on the right. Without it a near pane stays glass AHEAD, the
        # walk policy's own verdict — until 2026-10-22 it was dropped, and with nothing to
        # say instead the wearer heard "Walk straight" with glass ~1 m away (fuzz test).
        echo_ft = front.get("radar_ft")
        away = "right" if beside == "left" else "left"
        if (
            isinstance(echo_ft, (int, float))
            and echo_ft <= SIMPLE_GLASS_BESIDE_FT
            and _side_room(walk_decision, lidar_state, _fresh(lidar_state, now), away)
            < SIMPLE_MOVE_ROOM_FT
        ):
            beside = None
    if beside:
        beside_ft = front.get("radar_ft")  # the one board's echo
        glass = False
        laser = front.get("lidar_ft") if front.get("lidar_measured") else None
        front_ft = None if laser is None else float(laser)
        blocked = front_ft is not None and front_ft < WALK_BLOCK_FT
        walk_decision = {
            **walk_decision,
            "blocked": blocked,
            "glass_ahead": False,
            "directions": {
                **(walk_decision.get("directions") or {}),
                "front": {**front, "distance_ft": front_ft, "glass": False},
            },
        }
    near = front_ft is not None and front_ft <= SIMPLE_WALL_APPROACH_FT
    in_band = front_ft is not None and front_ft <= (
        SIMPLE_WALL_APPROACH_FT + SIMPLE_BAND_MARGIN_FT
    )

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
    rates = {
        s: (approach.rate_ft_s(s) if approach is not None else None)
        for s in ("left", "right")
    }
    moving_to = {
        s for s in ("left", "right") if approach is not None and approach.closing(s)
    }
    if (motion or {}).get("confidently_still"):
        moving_to = set()

    found = None
    if (blocked or in_band) and not glass:
        found = _front_wall(
            walk_decision,
            lidar_state,
            now,
            WALK_BLOCK_FT if blocked else SIMPLE_WALL_APPROACH_FT,
            surfaces,
        )
    wall = found is not None and not found["alongside"]
    alongside = found is not None and found["alongside"]

    near_sides = {s for s, w in walls.items() if w["ft"] <= SIMPLE_WALL_NEAR_FT}
    band_sides = {
        s
        for s, w in held.items()
        if w["ft"] <= SIMPLE_WALL_NEAR_FT + SIMPLE_WALL_NEAR_MARGIN_FT
    }
    boxed = {
        s
        for s in ("left", "right")
        if _side_room(
            walk_decision, lidar_state, fresh, "right" if s == "left" else "left"
        )
        < SIMPLE_MOVE_ROOM_FT
    }
    # Drifting: pointed at it (converging) AND moving towards it (closing).
    drift_sides = {s for s, w in walls.items() if w["converging"] and s in moving_to}
    one = lambda sides: next(iter(sides)) if len(sides) == 1 else None
    brushed = None
    if blocked and not (glass or wall or alongside):
        s = one(near_sides)
        if s and _brushing(lidar_state, walls[s]["surface"], WALK_BLOCK_FT):
            alongside, found = (
                True,
                {"side": s, "alongside": True, "ft": walls[s]["ft"]},
            )
        elif one(band_sides) and _brushing(
            lidar_state, held[one(band_sides)]["surface"], WALK_BLOCK_FT
        ):
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
        close_band = (
            sorted(band_sides)
            if len(band_sides) == 1 and not band_sides & boxed
            else []
        )
        drift = one(drift_sides)
        if (
            beside
            and isinstance(beside_ft, (int, float))
            and beside_ft <= SIMPLE_GLASS_BESIDE_FT
            and beside not in boxed
        ):
            glass_side = beside

    veh_word = (
        ranged_vehicle(camera_state, now, front_ft)
        if (blocked or near) and not glass and not (alongside or brushed)
        else None
    )
    if veh_word:
        wall = False

    obj = None
    if (blocked or near) and not (glass or wall or alongside or brushed):
        obj = veh_word or front_object(camera_state, now, front_ft)

    # Something the laser measures on the walking line, short of a block, that is not a
    # wall, glass or named: until 2026-10-21 that was "Walk straight" right up to 4 ft.
    laser_ft = front.get("lidar_ft") if front.get("lidar_measured") else None
    # Held out to SIMPLE_BAND_MARGIN_FT beyond, once said, like "near" (no chatter at 8 ft).
    obstacle_band = bool(
        fresh
        and not blocked
        and isinstance(laser_ft, (int, float))
        and laser_ft <= SIMPLE_OBSTACLE_AHEAD_FT + SIMPLE_BAND_MARGIN_FT
        and not (glass or wall or alongside or brushed or obj)
    )
    obstacle_ahead = obstacle_band and laser_ft <= SIMPLE_OBSTACLE_AHEAD_FT

    side = (
        safer_side(walk_decision, previous_side, lidar_state if fresh else None)
        if blocked
        else None
    )
    if tight_turn:
        side = tight_turn
    # The walk policy's way out when there is no side to turn into (2026-10-21): until
    # then both were display-only and the user heard "Turn left/right" anyway.
    if blocked and walk_decision.get("action") == "stop":
        side = "stop"
    elif blocked and walk_decision.get("action") == "turn_back" and not tight_turn:
        side = "around"
    elif (blocked and not tight_turn and side in ("left", "right")
          and walk_decision.get("direction") not in ("left", "right")
          and _clearance(walk_decision, side) < WALK_MIN_TURN_FT):
        # The walk policy chose no turn ("Slow down. Something ahead.": nothing has room)
        # and the bigger side has less than a turn needs either: a turn there is into the
        # wall (2026-10-23: "Turn left" toward 1.5 ft, with the back unknown).
        side = "stop"

    # The route planner (advisory): the way through everything sensed, now and as things move.
    kalman = (movers or {}).get("kalman") or {}
    route = None
    if ROUTE_PLANNER_ENABLED and fresh:
        try:
            from utils.route_planner import get_planner

            g: Dict[str, Any] = {}
            if glass and isinstance(front.get("radar_ft"), (int, float)):
                g["ahead_m"] = float(front["radar_ft"]) * MM_PER_FT / 1000.0
            elif beside and isinstance(beside_ft, (int, float)):
                g = {"side": beside, "side_m": float(beside_ft) * MM_PER_FT / 1000.0}
            route = get_planner().plan(
                lidar_state, radar_state, camera_state, kalman, motion, g, now
            )
        except Exception as e:
            route = {"ok": False, "error": f"{type(e).__name__}: {e}"}
    route_ok = bool(route and route.get("ok"))
    route_turn = (
        ("around" if route["around"] else route["side"])
        if route_ok and blocked
        else None
    )
    device_side = side
    if ROUTE_PLANNER_SPEAK and route_turn and not tight_turn:
        side = route_turn

    views = walk_decision.get("directions") or {}
    side_room: Dict[str, Optional[float]] = {}
    for s in ("left", "right"):
        view = views.get(s)
        # No view at all is as unknown as one marked unknown.
        room = 0.0 if (not view or view.get("unknown")) else view.get("distance_ft")
        if fresh and room != 0.0:
            mm = (
                ((lidar_state or {}).get("sectors") or {}).get(f"front_{s}") or {}
            ).get("min_dist")
            if isinstance(mm, (int, float)) and mm > 0:
                room = mm / MM_PER_FT if room is None else min(room, mm / MM_PER_FT)
        side_room[s] = room
    steer = (
        camera_path(camera_state, now, previous_steer) if not (glass or wall) else None
    )
    if steer and not _steer_allowed(steer, side_room, blocked, front_ft, side):
        steer = None
    # Past something on the way that a small turn clears (or a moving thing that will be there).
    route_steer = None
    if (
        route_ok
        and not blocked
        and not glass
        and route["side"]
        and route["speed"] != "wait"
        and abs(route["head_deg"]) <= ROUTE_STEER_MAX_DEG
        and (route["straight_m"] < ROUTE_NEED_M or route["by_mover"])
    ):
        word, where = None, "ahead"
        blk = str(route.get("blocker") or "")
        cause = route.get("cause") or {}
        if route["by_mover"] and cause:
            word = cause.get("word") if cause.get("word") in _WORDS else None
            y = float(cause.get("y_m") or 0.0)
            where = "right" if y > 0.2 else "left" if y < -0.2 else "ahead"
        elif blk.startswith("camera:"):
            word = blk.split(":", 1)[1]
        if (where, route["side"]) in (("left", "left"), ("right", "right")):
            where = "ahead"  # passing it on its own side: "X ahead, turn slightly"
        route_steer = {
            "kind": "route",
            "turn": route["side"],
            "word": word,
            "where": where,
            "range_m": route["straight_m"],
            "cut": False,
        }
        if not _steer_allowed(route_steer, side_room, blocked, front_ft, side):
            route_steer = None
    if ROUTE_PLANNER_SPEAK and steer is None and route_steer is not None:
        steer = route_steer
    no_room_by_side = {
        s: vehicle_move_blocked(walk_decision, lidar_state, fresh, s)
        for s in ("left", "right")
    }
    vehicle_no_room = bool(vehicle and no_room_by_side.get(vehicle))
    # The soonest of the LiDAR's mover and the radar's (where the LiDAR cannot see it). The
    # Kalman tracker (utils/motion_tracker), when it runs, stands in for lidar_movers.
    radar = (movers or {}).get("radar") or {}
    kalman_live = bool(
        MOTION_TRACKER_WARN
        and kalman
        and not kalman.get("stale")
        and kalman.get("t") is not None
    )
    lidar_threat = (
        kalman.get("threat")
        if kalman_live
        else (movers or {}).get("threat")
        if not (movers or {}).get("stale")
        else None
    )
    threats = [
        t
        for t in (lidar_threat, radar.get("threat") if not radar.get("stale") else None)
        if t
    ]
    # The planner says wait (something will cross the way), step aside (something is coming
    # at the spot the user stands on) or, for a vehicle, slow down: the mover sentence for it
    # ("... approaching from ..., stop"), if no closest approach already gives one.
    cause_word = ((route or {}).get("cause") or {}).get("word")
    if (
        route_ok
        and not threats
        and route.get("cause")
        and (
            route["speed"] == "wait"
            or route["step_aside"]
            or (route["speed"] == "slow" and cause_word == "Vehicle")
        )
    ):
        c = route["cause"]
        brg = float(c.get("bearing_deg") or 0.0)
        speed = max(0.1, float(c.get("own_speed_mps") or 0.0))
        if route["step_aside"]:
            mode = "behind" if abs(brg) > 135.0 else "toward"
            away = route["side"]
            passing = "left" if away == "right" else "right" if away == "left" else None
        else:
            mode, passing = "crossing", None
        threats = [
            {
                **c,
                "source": "route",
                "mode": mode,
                "side": "left" if brg < 0 else "right",
                "pass_side": passing,
                "ttc_s": round(float(c.get("distance_m") or 0.0) / speed, 1),
                "miss_m": 0.0,
            }
        ]
    threat = (
        min(threats, key=lambda t: float(t.get("ttc_s") or 1e9))
        if fresh and threats
        else None
    )
    would_say, mover_why = mover_sentence(
        threat, walk_decision, lidar_state, fresh, camera_state, now
    )
    mover = would_say if LIDAR_MOVERS_SPEAK else None
    route_would, route_why = None, ""
    if route_ok and not ROUTE_PLANNER_SPEAK:
        if blocked and route_turn and route_turn != device_side and not tight_turn:
            route_would = (
                TURN_AROUND
                if route_turn == "around"
                else (TURN_LEFT if route_turn == "left" else TURN_RIGHT)
            )
            route_why = (
                f"route: the way {route['side'] or 'ahead'} is clear "
                f"{route['dist_m']:.1f} m (the device turns {device_side})"
                if route_turn != "around"
                else "route: no way 1 m clear anywhere ahead"
            )
        elif route_steer is not None and steer is not route_steer:
            route_would = _steer_text(route_steer, False)
            route_why = (
                f"route: {route.get('blocker') or 'something'} "
                f"{route['straight_m']:.1f} m ahead in the way; "
                f"{abs(route['head_deg'])} deg {route['side']} is clear "
                f"{route['dist_m']:.1f} m"
            )
    text = _sentence(
        blocked,
        near,
        glass,
        wall,
        side,
        drift,
        obj,
        close,
        vehicle,
        vehicle_ahead,
        steer,
        vehicle_no_room,
        glass_side,
        mover,
        obstacle_ahead,
    )
    if text in _MOVER_SENTENCES:
        rule, why = "mover_approaching", mover_why
    elif text in _VEHICLE_APPROACH:
        rule = "vehicle_approaching"
        why = (
            f"{seen['class']} approaching from your {vehicle}"
            + (f", contact in ~{seen['ttc_s']:.0f} s" if seen.get("ttc_s") else "")
            + (
                f" — no room to step {'right' if vehicle == 'left' else 'left'}"
                if vehicle_no_room
                else ""
            )
        )
    elif text == VEHICLE_AHEAD:
        rule = "vehicle_ahead"
        why = (
            f"{seen['class']} moving ahead"
            if seen.get("moving")
            else f"{seen['class']} stopped ahead, you are standing still"
        )
    elif text in _STEER_INFO:
        rule = "blocked" if blocked else "steering"
        why = _steer_reason(text, steer)
    elif text in _GLASS_BESIDE:
        rule = "glass_beside"
        why = (
            f"glass on your {glass_side} ({beside_ft:.1f} ft) — only the "
            f"{glass_side}-front radar hears it, the laser sees through it"
        )
    elif text in (NEAR_LEFT, NEAR_RIGHT):
        rule = "near_wall"
        ft = (walls.get(close) or {}).get("ft")
        why = f"near the wall on your {close}" + (f" ({ft:.1f} ft)" if ft else "")
    elif text in (DRIFT_LEFT, DRIFT_RIGHT):
        rule = "drifting"
        why = (
            f"moving toward the wall on your {drift} "
            f"({rates[drift] or 0:.1f} ft/s, {walls[drift]['ft']:.1f} ft away)"
            if drift in walls
            else f"moving toward the wall on your {drift}"
        )
    elif text == STOP_NO_WAY:
        rule = "blocked"
        why = (
            f"front blocked {front_ft or 0:.1f} ft ahead, and no direction has "
            f"{WALK_MIN_TURN_FT:.0f} ft of room to turn into"
        )
    elif blocked and tight_turn:
        rule = "blocked"
        wall_side = "left" if tight_turn == "right" else "right"
        why = (
            f"standing close to the wall on your {wall_side}"
            + (f" ({walls[wall_side]['ft']:.1f} ft)" if wall_side in walls else "")
            + f", no room to step {tight_turn}: turn away from it"
        )
    elif blocked:
        rule = "blocked"
        why = f"front blocked by {'glass' if glass else 'wall' if wall else (obj or 'an obstacle').lower()}"
    elif near and (glass or wall or obj):
        rule = "approaching"
        why = f"{'glass' if glass else 'wall' if wall else obj.lower()} {front_ft or 0:.1f} ft ahead"
    elif text == OBSTACLE_AHEAD:
        rule = "approaching"
        why = f"something unnamed on your path {laser_ft:.1f} ft ahead"
    else:
        rule, why = "clear", "path ahead is clear"
    if beside and text not in _GLASS_BESIDE:
        why += f" (glass on your {beside} ignored)"
    return {
        "instruction": text,
        "action": _ACTION[text],
        "rule": rule,
        "side": side,
        "obstacle": (
            "glass" if glass else "wall" if wall else obj.lower() if obj else None
        ),
        "blocked": blocked,
        "near": near,
        "glass": glass,
        "wall": wall,
        "drift": drift,
        "close": close,
        "close_band": close_band,
        "beside": {s: round(w["ft"], 2) for s, w in walls.items()},
        "closing": {s: (None if r is None else round(r, 2)) for s, r in rates.items()},
        "converging": sorted(s for s, w in walls.items() if w["converging"]),
        "vehicle": vehicle,
        "vehicle_ahead": vehicle_ahead,
        "vehicles": seen,
        "steer": steer,
        "side_room": side_room,
        "user_still": still,
        "vehicle_no_room": vehicle_no_room,
        "vehicle_no_room_by_side": no_room_by_side,
        "glass_side": glass_side,
        "glass_side_ft": beside_ft,
        "object": obj,
        "front_ft": front_ft,
        "on_line_ft": on_line_ft,
        "obstacle_ahead": obstacle_ahead,
        "obstacle_band": obstacle_band,
        "mover": (
            {
                **threat,
                "distance_ft": _in_ft(threat.get("distance_m")),
                "would_say": would_say,
                "spoken": bool(LIDAR_MOVERS_SPEAK),
            }
            if threat
            else None
        ),
        "mover_text": mover,
        "mover_reason": mover_why,
        # For the dashboard, in FEET like every figure a person reads there.
        "movers": [
            {
                **{
                    k: m.get(k)
                    for k in ("word", "x_m", "y_m", "vx_mps", "vy_mps", "alert")
                },
                "distance_ft": _in_ft(m.get("distance_m")),
                "speed_ft_s": _in_ft(m.get("own_speed_mps")),
            }
            for m in (movers or {}).get("tracks") or []
        ]
        if fresh
        else [],
        "radar_movers": [
            {
                **{
                    k: m.get(k)
                    for k in ("bearing_deg", "distance_m", "lidar_sees", "alert")
                },
                "distance_ft": _in_ft(m.get("distance_m")),
                "closing_ft_s": _in_ft(m.get("closing_mps")),
            }
            for m in radar.get("tracks") or []
            if m.get("confirmed")
        ]
        if fresh
        else [],
        "route": route,
        "route_would_say": route_would,
        "route_reason": route_why,
        # The Kalman tracks (utils/motion_tracker): own speed and how sure, in FEET.
        "tracks": [
            {
                "id": m.get("id"),
                "word": m.get("word"),
                "x_m": m.get("x_m"),
                "y_m": m.get("y_m"),
                "distance_ft": _in_ft(m.get("distance_m")),
                "speed_ft_s": _in_ft(m.get("speed")),
                "speed_sd_ft_s": _in_ft(m.get("speed_sd")),
                "sources": m.get("sources"),
                "alert": m.get("alert"),
            }
            for m in kalman.get("tracks") or []
        ]
        if fresh and kalman_live
        else [],
        "reason": why,
    }


class _Hold:
    """A boolean that turns on fast and off slowly, and tolerates dropouts."""

    def __init__(self, attack: float, release: float, gap: float):
        self.attack, self.release, self.gap = attack, release, gap
        self.state = False
        self._first: Optional[float] = None  # start of the evidence run
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
        self.vehicle_careful = _Hold(
            SIMPLE_ALERT_CONFIRM_S, SIMPLE_OBJECT_RELEASE_S, SIMPLE_ALERT_GAP_S
        )
        # A LiDAR mover's sentence, kept while its hold lasts (LIDAR_MOVERS_SPEAK).
        self.mover = _Hold(*timing)
        self.mover_text: Optional[str] = None
        self.steer = _Hold(
            SIMPLE_ALERT_CONFIRM_S, SIMPLE_OBJECT_RELEASE_S, SIMPLE_ALERT_GAP_S
        )
        self.steer_plan: Optional[Dict[str, Any]] = None
        self.obj = _Hold(
            SIMPLE_ALERT_CONFIRM_S, SIMPLE_OBJECT_RELEASE_S, SIMPLE_ALERT_GAP_S
        )
        self.obj_word: Optional[str] = None
        self.obstacle = _Hold(*timing)
        self.episode = 0
        self._in_episode = False
        self._unknown_since: Optional[float] = None
        self._last_text = ""  # the sentence the last update mapped to
        self.side: Optional[str] = None
        self._blocked_since: Optional[float] = None
        self._crossing_seen: Optional[float] = None  # last traffic light, this crossing
        self._walked_since: Optional[float] = None  # measured walking since, after it
        self.crossing_told = False  # "Traffic light ahead" said for it

    def at_crossing(
        self, detected: bool, now: float, still: Optional[bool] = None
    ) -> bool:
        """The crossing detector's verdict, held past its last sighting: SIMPLE_CROSSING_HOLD_S, then until the user has been measured walking SIMPLE_CROSSING_WALKED_S. Until 2026-10-21 it ended on the timer alone, at the kerb if a bus hid the light. Standing still holds it; with no motion reading at all it ends SIMPLE_CROSSING_MAX_HOLD_S after the last sighting."""
        if self._crossing_seen is not None and now < self._crossing_seen:
            self._crossing_seen = now  # clock stepped back
        if detected:
            if self._crossing_seen is None:
                self.crossing_told = False  # a new crossing
            self._crossing_seen = now
            self._walked_since = None
            return True
        if self._crossing_seen is None:
            return False
        out = now - self._crossing_seen
        if out < SIMPLE_CROSSING_HOLD_S:
            return True
        if still is False:
            if self._walked_since is None or now < self._walked_since:
                self._walked_since = now
            done = now - self._walked_since >= SIMPLE_CROSSING_WALKED_S
        else:
            self._walked_since = None
            done = still is None and out >= SIMPLE_CROSSING_MAX_HOLD_S
        if done:
            self._crossing_seen = self._walked_since = None
        return not done

    def update(self, raw: Dict[str, Any], now: float) -> Tuple[str, bool]:
        """(the sentence the smoothed picture maps to, whether a block's NAME is still being confirmed and the sentence should wait for it)."""
        front = raw.get("front_ft")
        # A backwards clock step: see _Hold.update.
        if self._blocked_since is not None and now < self._blocked_since:
            self._blocked_since = now
        in_block = raw["blocked"] or (
            self.blocked.state
            and front is not None
            and front < WALK_BLOCK_FT + SIMPLE_BAND_MARGIN_FT
        )
        in_band = raw["near"] or (
            self.near.state
            and front is not None
            and front < SIMPLE_WALL_APPROACH_FT + SIMPLE_BAND_MARGIN_FT
        )
        release = (
            SIMPLE_GLASS_RELEASE_S
            if self.glass.state and (self.blocked.state or self.near.state)
            else SIMPLE_CLEAR_CONFIRM_S
        )
        for hold in (self.blocked, self.near, self.glass):
            hold.release = release
        blocked = self.blocked.update(in_block, now)
        near = self.near.update(in_band, now)
        glass = self.glass.update(raw["glass"], now)
        wall = self.wall.update(raw["wall"], now)
        left = self.drift_left.update(raw.get("drift") == "left", now)
        right = self.drift_right.update(raw.get("drift") == "right", now)
        drift = (
            "left" if left and not right else "right" if right and not left else None
        )
        band = raw.get("close_band") or []
        c_left = self.close_left.update(
            raw.get("close") == "left" or (self.close_left.state and "left" in band),
            now,
        )
        c_right = self.close_right.update(
            raw.get("close") == "right" or (self.close_right.state and "right" in band),
            now,
        )
        close = (
            "left"
            if c_left and not c_right
            else "right"
            if c_right and not c_left
            else None
        )
        g_left = self.glass_left.update(raw.get("glass_side") == "left", now)
        g_right = self.glass_right.update(raw.get("glass_side") == "right", now)
        glass_side = (
            "left"
            if g_left and not g_right
            else "right"
            if g_right and not g_left
            else None
        )
        if raw.get("vehicle"):
            self.vehicle_side = raw["vehicle"]
        vehicle = (
            self.vehicle_side
            if self.vehicle.update(bool(raw.get("vehicle")), now)
            else None
        )
        if vehicle is None:
            self.vehicle_side = None if not self.vehicle.active else self.vehicle_side
        vehicle_ahead = (
            self.vehicle_careful.update(bool(raw.get("vehicle_ahead")), now)
            and vehicle is None
        )
        if raw.get("mover_text"):
            self.mover_text = raw["mover_text"]
        mover = (
            self.mover_text
            if self.mover.update(bool(raw.get("mover_text")), now)
            else None
        )
        if not self.mover.active:
            self.mover_text = None
        side_seen = bool(raw.get("close") or raw.get("drift") or raw.get("close_band"))
        on_line = raw.get("on_line_ft")
        line_blocked = (
            on_line is not None and on_line < WALK_BLOCK_FT + SIMPLE_BAND_MARGIN_FT
        )
        if raw["blocked"] and line_blocked and not side_seen:
            close = drift = None
        if raw["blocked"] and not raw.get("vehicle_ahead"):
            vehicle_ahead = False
        if not raw.get("vehicle_ahead") and self._last_text != VEHICLE_AHEAD:
            vehicle_ahead = False
        if raw.get("steer"):
            self.steer_plan = raw["steer"]
        steer = (
            self.steer_plan if self.steer.update(bool(raw.get("steer")), now) else None
        )
        if not self.steer.active:
            self.steer_plan = None
        if self.steer_plan and not _steer_allowed(
            self.steer_plan,
            raw.get("side_room") or {},
            blocked,
            front,
            raw.get("side") or self.side,
        ):
            self.steer.clear()
            self.steer_plan = steer = None
        if raw.get("object"):
            self.obj_word = raw["object"]
        obj = self.obj.update(bool(raw.get("object")), now)
        obstacle = self.obstacle.update(
            bool(
                raw.get("obstacle_ahead")
                or (self.obstacle.state and raw.get("obstacle_band"))
            ),
            now,
        )

        if not (self.blocked.active or self.near.active):
            self.glass.clear()
            self.wall.clear()
            self.obj.clear()
            self.obj_word = None
            self.obstacle.clear()
            glass = wall = obj = obstacle = False
        if blocked:
            if self._blocked_since is None:
                self._blocked_since = now
            if raw["blocked"] and raw.get("side") in (
                "left",
                "right",
                "around",
                "stop",
            ):
                self.side = raw["side"]
        else:
            self.side = None
            self._blocked_since = None
        side_active = (side_seen or not line_blocked) and (
            self.drift_left.active
            or self.drift_right.active
            or self.close_left.active
            or self.close_right.active
        )
        naming = (
            blocked
            and not (glass or wall or drift or close or obj)
            and (
                self.glass.active or self.wall.active or self.obj.active or side_active
            )
            and now - self._blocked_since < SIMPLE_ALERT_CONFIRM_S + 1e-6
        )
        text = _sentence(
            blocked,
            near,
            glass,
            wall,
            self.side,
            drift,
            self.obj_word if obj else None,
            close,
            vehicle,
            vehicle_ahead,
            steer,
            vehicle_no_room=bool(
                (raw.get("vehicle_no_room_by_side") or {}).get(vehicle)
            ),
            glass_side=glass_side,
            mover=mover,
            obstacle=obstacle and not (wall or obj),
        )
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

    def offline(
        self, raw: Dict[str, Any], lidar_state: Optional[Dict[str, Any]], now: float
    ) -> Optional[str]:
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
        if (
            self._unknown_since is not None
            and now - self._unknown_since >= SIMPLE_OFFLINE_S
        ):
            return "no range reading ahead"
        return None

    def reset(self) -> None:
        self.__init__()


class _SensorNotice:
    """Whether the wearer has been told a sensor is lost, or back ("Glass sensor off",
    "Camera off", ...): a loss is told once it has lasted `loss_s`, a recovery once it has
    held `recover_s` (doubling with each loss told, up to `recover_max_s`), and nothing is
    told inside `grace_s` of start-up while the sensor is still coming up."""

    def __init__(
        self,
        off: str,
        on: str,
        loss_s: float,
        recover_s: float,
        recover_max_s: float,
        grace_s: float,
    ):
        self._off, self._on = off, on
        self._loss_s, self._recover_s = loss_s, recover_s
        self._recover_max_s, self._grace_s = recover_max_s, grace_s
        self._clear()

    def _clear(self) -> None:
        self.announced_ok = True
        self._since: Optional[float] = None
        self._first: Optional[float] = None  # first poll judged
        self._ever_ok = False  # full coverage seen at least once
        self._losses = 0  # announced losses, for the backoff
        self._ok_since: Optional[float] = None  # coverage unbroken since

    def _recover_need(self) -> float:
        return min(self._recover_s * 2 ** max(self._losses - 1, 0), self._recover_max_s)

    def pending(self, ok: bool, now: float, settling: bool = True) -> Optional[str]:
        """`settling`: the sensor is still coming up (e.g. front_glass_coverage: discovery has not reported, or an owed front board is starting or calibrating)."""
        if self._first is None or now < self._first:
            self._first = now
        if ok:
            self._ever_ok = True
            if self._ok_since is None or now < self._ok_since:
                self._ok_since = now
            if (
                self.announced_ok
                and self._losses
                and now - self._ok_since >= self._recover_max_s
            ):
                self._losses = 0  # a long quiet run: start afresh
        else:
            self._ok_since = None
        if ok == self.announced_ok:
            self._since = None
            return None
        if self._since is None or now < self._since:  # new run, or clock step
            self._since = now
        if ok:
            need = self._recover_need()
        else:
            need = self._loss_s
            if (
                not self._ever_ok
                and settling
                and now - self._first < self._grace_s - 1e-6
            ):
                return None  # still coming up
        if now - self._since >= need - 1e-6:
            return self._on if ok else self._off
        return None

    def said(self, text: str) -> None:
        self.announced_ok = text == self._on
        if text == self._off:
            self._losses += 1
        self._since = None

    def state(self) -> Dict[str, Any]:
        return {
            "announced": "on" if self.announced_ok else "off",
            "changing_since": self._since,
            "recover_after_s": self._recover_need(),
        }

    def reset(self) -> None:
        self._clear()


class _MoverShadow:
    """What the LiDAR mover warning WOULD say while it is not spoken (LIDAR_MOVERS_SPEAK False): held as a spoken one is, logged once when an approach is confirmed, then at most every LIDAR_MOVERS_REPEAT_S while it lasts."""

    def __init__(self):
        self.hold = _Hold(
            SIMPLE_ALERT_CONFIRM_S, SIMPLE_CLEAR_CONFIRM_S, SIMPLE_ALERT_GAP_S
        )
        self.text: Optional[str] = None
        self.logged_at: Optional[float] = None

    def update(self, text: Optional[str], now: float) -> Optional[str]:
        """The sentence to log as "would say" on this poll, or None."""
        if text:
            self.text = text
        if not self.hold.update(bool(text), now):
            if not self.hold.active:
                self.text, self.logged_at = None, None  # that approach is over
            return None
        if (
            self.logged_at is not None
            and 0.0 <= now - self.logged_at < LIDAR_MOVERS_REPEAT_S
        ):
            return None
        self.logged_at = now
        return self.text

    def reset(self) -> None:
        self.__init__()


# Each sentence's airtime as the Pi voice renders it, trailing silence included: how long
# the voice is busy. A words/s guess ran short, so a block warning cut "Vehicle
# approaching, move right" before "move right" (2026-10-23).
_measured_airtime: Dict[str, float] = {}


def measure_airtimes(cmd: List[str]) -> int:
    """Time every sentence in ALLOWED with the voice command (espeak-ng --stdout, no sound); how many were timed. pi_voice runs it off-thread at start-up."""
    import subprocess

    timed = 0
    for text in sorted(ALLOWED):
        try:
            wav = subprocess.run(
                list(cmd) + ["--stdout", text], capture_output=True, timeout=5.0
            ).stdout
            if wav[:4] != b"RIFF" or wav[8:12] != b"WAVE":
                continue  # not espeak's WAV: keep the estimate
            byte_rate = int.from_bytes(wav[28:32], "little")
            if len(wav) > 44 and byte_rate > 0:
                _measured_airtime[text] = (len(wav) - 44) / byte_rate
                timed += 1
        except Exception:
            pass
    return timed


def _airtime(text: str) -> float:
    measured = _measured_airtime.get(text)
    if measured is not None:
        return measured
    return (
        NOTIFY_AIRTIME_BASE_S
        + NOTIFY_AIRTIME_PER_CHAR_S * len(text)
        + NOTIFY_AIRTIME_PER_COMMA_S * text.count(",")
    )


class _Gate:
    """Decides whether a sentence is worth SAYING on this poll."""

    def __init__(self):
        self._spoken: str = ""  # last sentence actually announced
        self._level: int = -1
        self._spoken_at: float = 0.0
        self._busy_until: float = 0.0  # when that sentence finishes playing
        self._has_spoken: bool = False
        self._repeats: int = 0  # consecutive re-statements of _spoken
        self._pending: str = ""  # candidate waiting out the debounce
        self._pending_since: float = 0.0
        self._episode: Optional[int] = None  # the obstacle _spoken was about
        self._seq: int = 0  # never goes backwards, even on reset
        # The sentence the latest cue_seq counted: guidance OR a status notice.
        self._counted: str = ""
        self._status_busy_until: float = 0.0

    def admit(
        self, text: str, now: float, episode: Optional[int] = None
    ) -> Tuple[bool, int, str]:
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
            # Still blocked (perhaps after turning into something else) is said again
            # soon; "Guidance stopped" for as long as it lasts (2026-10-21).
            if text == SENSORS_OFFLINE:
                window = SIMPLE_OFFLINE_REPEAT_S
            else:
                base = (
                    SIMPLE_BLOCKED_REPEAT_S
                    if level == _BLOCKED and text not in _MOVER_SENTENCES
                    else SIMPLE_REPEAT_S
                )
                window = min(base * (2**self._repeats), SIMPLE_REPEAT_MAX_S)
            since = now - self._spoken_at
            if since < window:
                return (
                    False,
                    self._seq,
                    f"unchanged — said {since:.0f}s ago (repeats after {window:.0f}s)",
                )
            return self._commit(text, level, now, episode)

        arrived = self._spoken in (DRIFT_LEFT, DRIFT_RIGHT) and text in (
            NEAR_LEFT,
            NEAR_RIGHT,
        )
        # A camera vehicle or a LiDAR mover coming at the user is news, and so is its passing.
        vehicle_news = text in _APPROACHING and self._spoken not in _APPROACHING
        after_vehicle = (
            self._spoken in _APPROACHING
            and text not in _APPROACHING
            and level >= _BLOCKED
        )
        offline_news = text == SENSORS_OFFLINE and self._spoken != SENSORS_OFFLINE
        # Something on the path after a wall beside you is news, said once that sentence ends (2026-10-01).
        path_news = self._spoken in _SIDE_WALL + _GLASS_BESIDE and text in _PATH_AHEAD
        # And a wall beside you, or a way past, after "Obstacle ahead" (2026-10-21): until
        # then that obstacle was "Walk straight", which opened no episode to settle in.
        side_news = (
            self._spoken == OBSTACLE_AHEAD
            and text in _SIDE_WALL + _GLASS_BESIDE + tuple(_STEER_INFO)
        )
        busy_until = max(
            self._busy_until if self._has_spoken else 0.0, self._status_busy_until
        )
        if level >= 1 and (
            level > self._level
            or arrived
            or vehicle_news
            or after_vehicle
            or offline_news
            or path_news
            or side_news
        ):
            # A block cuts into a guidance sentence like a vehicle does (2026-10-21):
            # waiting out a 2.5 s side-wall sentence cost ~3 m at walking pace, inside the
            # 4 ft block distance. Not into a vehicle or mover warning, which must be heard
            # whole, nor into a status notice (~1 s, said once, on a clear path only).
            guidance_playing = self._has_spoken and now < self._busy_until
            if (
                level < _VEHICLE
                and now < busy_until
                and (
                    level < _BLOCKED
                    or now < self._status_busy_until
                    or (guidance_playing and self._spoken in _APPROACHING)
                )
            ):
                return (
                    False,
                    self._seq,
                    f"more urgent — waiting "
                    f"{busy_until - now:.1f}s for the last sentence to end",
                )
            return self._commit(text, level, now, episode)

        held = now - self._pending_since
        need = SIMPLE_CONFIRM_S
        flipped = self._spoken in _VEHICLE_APPROACH and text in _VEHICLE_APPROACH
        if (
            level >= 1
            and self._has_spoken
            and episode is not None
            and episode == self._episode
        ):
            upgrade = (
                self._spoken in (TURN_LEFT, TURN_RIGHT)
                and text not in (TURN_LEFT, TURN_RIGHT)
                and level == self._level
            )
            upgrade = (
                upgrade
                or flipped
                or (self._spoken in _APPROACHING and text not in _APPROACHING)
            )
            upgrade = upgrade or self._spoken in (SENSORS_OFFLINE, STOP_NO_WAY)
            if not upgrade:
                need = SIMPLE_SETTLE_S
        if held < need:
            return (
                False,
                self._seq,
                f"same obstacle — held {held:.1f}s of {need:.0f}s"
                if need > SIMPLE_CONFIRM_S
                else f"unconfirmed — held {held:.1f}s of {need:.1f}s",
            )
        if (
            self._has_spoken
            and now - self._spoken_at < SIMPLE_MIN_GAP_S
            and not flipped
        ):
            return (
                False,
                self._seq,
                f"too soon — "
                f"{SIMPLE_MIN_GAP_S - (now - self._spoken_at):.1f}s until "
                f"the voice is free",
            )
        if now < self._status_busy_until:
            return (
                False,
                self._seq,
                f"too soon — "
                f"{self._status_busy_until - now:.1f}s until the status ends",
            )
        return self._commit(text, level, now, episode)

    def _unstep_status(self, now: float) -> None:
        if self._status_busy_until - now > 2.0 * _airtime(GLASS_SENSOR_OFF):
            self._status_busy_until = now

    def admit_status(
        self, text: str, now: float, cut_in: bool = False
    ) -> Tuple[bool, int, str]:
        """Admit a STATUS notice ("Glass sensor off/on", "Camera off/on"): its own cue_seq, and nothing else about the gate changes (audit pass 2, NEW-028). `cut_in`: do not wait for the sentence playing — "Guidance stopped" (2026-10-23)."""
        if self._has_spoken and now < self._spoken_at:
            self._busy_until = min(self._busy_until, now)  # clock stepped back
        self._unstep_status(now)
        busy_until = max(
            self._busy_until if self._has_spoken else 0.0, self._status_busy_until
        )
        if now < busy_until and not cut_in:
            return (
                False,
                self._seq,
                f"status — waiting "
                f"{busy_until - now:.1f}s for the last sentence to end",
            )
        self._status_busy_until = now + _airtime(text)
        self._counted = text
        self._seq += 1
        return True, self._seq, ""

    def _commit(
        self, text: str, level: int, now: float, episode: Optional[int] = None
    ) -> Tuple[bool, int, str]:
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

    def forget_said(self) -> None:
        """Forget the last guidance sentence, so the current one is said again: after "Guidance stopped" (a fault) or "Traffic light ahead", what was said before no longer stands (2026-10-21)."""
        self._spoken, self._level, self._episode, self._repeats = "", -1, None, 0

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
        return {"last": self._spoken, "cue_seq": self._seq, "pending": self._pending}

    def reset(self) -> None:
        seq = self._seq
        self.__init__()
        self._seq = seq


_lock = threading.Lock()
_world = _World()
_gate = _Gate()
_glass = _SensorNotice(
    GLASS_SENSOR_OFF,
    GLASS_SENSOR_ON,
    SIMPLE_GLASS_LOSS_CONFIRM_S,
    SIMPLE_GLASS_RECOVER_CONFIRM_S,
    SIMPLE_GLASS_RECOVER_MAX_S,
    SIMPLE_GLASS_STARTUP_GRACE_S,
)
_camera = _SensorNotice(
    CAMERA_OFF,
    CAMERA_ON,
    SIMPLE_CAMERA_LOSS_CONFIRM_S,
    SIMPLE_CAMERA_RECOVER_CONFIRM_S,
    SIMPLE_CAMERA_RECOVER_MAX_S,
    SIMPLE_CAMERA_STARTUP_GRACE_S,
)
_power = _SensorNotice(
    POWER_LOW,
    POWER_NORMAL,
    SIMPLE_SYSTEM_LOSS_CONFIRM_S,
    SIMPLE_SYSTEM_RECOVER_CONFIRM_S,
    SIMPLE_SYSTEM_RECOVER_MAX_S,
    SIMPLE_SYSTEM_STARTUP_GRACE_S,
)
_heat = _SensorNotice(
    DEVICE_HOT,
    DEVICE_COOL,
    SIMPLE_SYSTEM_LOSS_CONFIRM_S,
    SIMPLE_SYSTEM_RECOVER_CONFIRM_S,
    SIMPLE_SYSTEM_RECOVER_MAX_S,
    SIMPLE_SYSTEM_STARTUP_GRACE_S,
)
_shadow = _MoverShadow()
_route_shadow = (
    _MoverShadow()
)  # the route planner's would-say (ROUTE_PLANNER_SPEAK False)
# The start-up hold as the wearer hears it: "Starting, please wait" when it begins, "Ready"
# when guidance does (2026-10-21; until then the hold was silent, like a dead device).
_startup = {"held": False, "starting_said": False, "ready_said": False}
# The guidance step itself failing (fault_notice): when "Guidance stopped" was last said.
_fault = {"said_at": None}


def _steer_reason(text: str, steer: Optional[Dict[str, Any]]) -> str:
    info = _STEER_INFO.get(text) or {}
    turn = info.get("turn")
    if info.get("where") == "between":
        return (
            "a gap straight ahead between them"
            if not turn
            else f"a gap between them, slightly {turn}"
        )
    word = (info.get("word") or "something").lower()
    rng = (steer or {}).get("range_m")
    at = f" (~{rng:.0f} m)" if isinstance(rng, (int, float)) else ""
    return f"{word} {info.get('where')}{at}: passing it on the {turn}"


def _reason(text: str, raw: Dict[str, Any]) -> str:
    """The reason for the sentence actually chosen."""
    if (
        not text
        or text == raw.get("instruction")
        or text == SENSORS_OFFLINE
        or text in _STATUS_NOTICES + _STARTUP_NOTICES
        or text == TRAFFIC_LIGHT_AHEAD
    ):
        return raw.get("reason", "")
    if text == STOP_NO_WAY:
        return "front blocked, and no direction has room to turn into"
    if text == OBSTACLE_AHEAD:
        return "something unnamed on your path ahead"
    if text in _STEER_INFO:
        return _steer_reason(text, raw.get("steer"))
    if text in _MOVER_SENTENCES:
        return raw.get("mover_reason") or "something moving is coming at you"
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
        return f"glass on your {side}" + (
            f" ({ft:.1f} ft)" if isinstance(ft, (int, float)) else ""
        )
    if text in (NEAR_LEFT, NEAR_RIGHT):
        side = "left" if text == NEAR_LEFT else "right"
        ft = (raw.get("beside") or {}).get(side)
        return f"near the wall on your {side}" + (f" ({ft:.1f} ft)" if ft else "")
    if text == WALK_STRAIGHT:
        return "path ahead is clear"
    if text in (TURN_LEFT, TURN_RIGHT):
        return "front blocked by an obstacle"
    if text == TURN_AROUND:
        return "front blocked, and no way 1 m clear anywhere ahead"
    thing = (
        "glass"
        if "glass" in text.lower() and text in (BLOCKED_GLASS, GLASS_AHEAD)
        else "wall"
        if text in (BLOCKED_WALL, WALL_AHEAD)
        else text.rsplit(" ", 2 if text.endswith("in front") else 1)[0].lower()
    )
    return (
        f"front blocked by {thing}"
        if _LEVEL.get(text) == _BLOCKED
        else f"{thing} ahead"
    )


def guidance(
    walk_decision: Optional[Dict[str, Any]] = None,
    lidar_state: Optional[Dict[str, Any]] = None,
    now: Optional[float] = None,
    hold: Optional[str] = None,
    at_crossing: bool = False,
    radar_state: Optional[Dict[str, Any]] = None,
    camera_state: Optional[Dict[str, Any]] = None,
    motion: Optional[Dict[str, Any]] = None,
    hold_lidar_absent: bool = False,
    movers: Optional[Dict[str, Any]] = None,
    system: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """The sentence AND whether to announce it. `system`: utils/pi_health.read()."""
    now = time.time() if now is None else now
    with _lock:
        raw = decide(
            walk_decision,
            lidar_state,
            now,
            previous_side=_world.side,
            radar_state=radar_state,
            camera_state=camera_state,
            approach=_world.approach,
            motion=motion,
            vehicles=_world.vehicles,
            previous_steer=_world.steer_turn,
            movers=movers,
        )
        naming = False
        # Judged on every poll, whatever is said (it used to be judged only when nothing
        # earlier in the chain below had matched, so a sighting could be missed). A motion
        # reading that cannot tell counts as standing there: only measured walking, or no
        # reading at all for SIMPLE_CROSSING_MAX_HOLD_S, ends the hold at a kerb.
        still = user_motion(motion)[0]
        if still is None and current_motion(motion):
            still = True
        crossing = _world.at_crossing(at_crossing, now, still)
        offline = _world.offline(raw, lidar_state, now)
        stale = not offline and lidar_state is not None and not _fresh(lidar_state, now)
        seen = raw.get("vehicles") or {}
        if (
            (offline or stale)
            and seen.get("approaching")
            and seen.get("side") in ("left", "right")
        ):
            text = (
                VEHICLE_APPROACHING
                if vehicle_move_blocked(walk_decision, lidar_state, False, seen["side"])
                else VEHICLE_FROM_LEFT
                if seen["side"] == "left"
                else VEHICLE_FROM_RIGHT
            )
            raw = {
                **raw,
                "rule": "vehicle_approaching",
                "reason": f"{seen.get('class') or 'vehicle'} approaching from your "
                f"{seen['side']} (LiDAR {'offline: ' + offline if offline else 'stale'})",
            }
        elif offline:
            text = SENSORS_OFFLINE
            raw = {**raw, "rule": "offline", "reason": offline}
        elif stale:
            from utils.freshness import describe_reading

            text = ""
            raw = {
                **raw,
                "rule": "stale",
                "reason": f"LiDAR scan {describe_reading(lidar_state, now)}"
                f" — holding, nothing new said",
            }
        elif raw["rule"] == "waiting":
            text = ""
        else:
            text, naming = _world.update(raw, now)
        notice, cover, notifier = None, {}, None
        if radar_state is not None:
            try:
                from utils.sensor_fusion import front_glass_coverage

                cover = front_glass_coverage(radar_state)
                notice = _glass.pending(
                    bool(cover["ok"]), now, settling=bool(cover.get("settling", True))
                )
                notifier = _glass if notice else None
            except Exception:
                notice, cover = None, {}
        if camera_state is not None:
            try:
                from utils.sensor_fusion import front_camera_coverage

                cam_cover = front_camera_coverage(camera_state, now)
                cam_notice = _camera.pending(
                    bool(cam_cover["ok"]),
                    now,
                    settling=bool(cam_cover.get("settling", True)),
                )
                if cam_notice and not notice:  # the glass notice goes first
                    notice, cover, notifier = cam_notice, cam_cover, _camera
            except Exception:
                pass
        if system is not None and system.get("readable"):
            try:
                power = _power.pending(not system.get("power_low"), now)
                heat = _heat.pending(not system.get("hot"), now)
                soc = system.get("soc_c")
                if power:  # it may switch off: before any sensor notice
                    notice, notifier = power, _power
                    cover = {
                        "reason": (
                            "the Pi reports under-voltage — its battery or "
                            "power supply is failing"
                            if power == POWER_LOW
                            else "the Pi's power supply is back to normal"
                        )
                    }
                elif heat and not notice:
                    notice, notifier = heat, _heat
                    cover = {
                        "reason": (
                            f"the Pi is at {soc:.0f} C or throttling, and slowing down"
                            if heat == DEVICE_HOT and isinstance(soc, (int, float))
                            else "the Pi is throttling"
                            if heat == DEVICE_HOT
                            else "the Pi's temperature is back to normal"
                        )
                    }
            except Exception:
                pass
        if text and text not in ALLOWED:
            text = ""  # never hand the delivery layer a stray line
        if text:
            _gate.settle(_LEVEL[text])
        # A start-up hold waiting on a LiDAR that is not coming (NEW-029).
        through_hold = bool(
            hold
            and hold_lidar_absent
            and offline
            and text in (SENSORS_OFFLINE,) + _VEHICLE_APPROACH
        )
        status = (
            notice
            if notice
            and not hold
            and text == WALK_STRAIGHT
            and raw.get("instruction") == WALK_STRAIGHT
            else None
        )
        if hold:
            _startup["held"] = True
        ready_due = _startup["held"] and not _startup["ready_said"] and not hold
        if ready_due and (offline or _LEVEL.get(text, -1) >= _BLOCKED):
            _startup["ready_said"] = True  # the warning itself says guidance began
            ready_due = False
        if hold and not through_hold:
            speak, cue_seq, quiet = False, _gate.state()["cue_seq"], hold
            if not _startup["starting_said"]:
                speak, cue_seq, _ = _gate.admit_status(STARTING, now)
                if speak:
                    _startup["starting_said"] = True
                    text, quiet = STARTING, ""
                    raw = {**raw, "rule": "startup", "reason": f"start-up hold: {hold}"}
        elif ready_due and raw["rule"] not in ("stale", "waiting"):
            speak, cue_seq, quiet = _gate.admit_status(READY, now)
            if speak:
                _startup["ready_said"] = True
                text = READY
                raw = {
                    **raw,
                    "rule": "startup",
                    "reason": "start-up scan finished — guidance begins",
                }
        elif status:
            speak, cue_seq, quiet = _gate.admit_status(status, now)
            if speak:
                notifier.said(status)
                text = status
                raw = {
                    **raw,
                    "rule": "status",
                    "reason": (
                        cover.get("reason")
                        or (
                            "front camera can see again"
                            if notifier is _camera
                            else "front radar coverage restored"
                        )
                    ),
                }
        elif raw["rule"] == "stale":
            speak, cue_seq, quiet = False, _gate.state()["cue_seq"], raw["reason"]
        elif text in _WALK_ON and (
            raw.get("blocked")
            or (raw.get("near") and raw.get("instruction") not in _WALK_ON)
            or raw.get("instruction") in _APPROACHING
        ):
            speak, cue_seq = False, _gate.state()["cue_seq"]
            quiet = "confirming — the newest reading is not clear"
        elif crossing and text in _WALK_ON:
            _gate.forget_pending()
            speak, cue_seq = False, _gate.state()["cue_seq"]
            quiet = "at a road crossing — not saying walk straight"
            if not _world.crossing_told:
                # Once per crossing, so an earlier "Walk straight" is not left standing.
                speak, cue_seq, why = _gate.admit_status(TRAFFIC_LIGHT_AHEAD, now)
                if speak:
                    _world.crossing_told = True
                    if _gate.last in _WALK_ON:
                        _gate.forget_said()  # said again once the hold ends
                    text = TRAFFIC_LIGHT_AHEAD
                    raw = {
                        **raw,
                        "rule": "crossing",
                        "reason": "traffic light in the front camera — not saying "
                        "walk straight, and never that it is safe to cross",
                    }
                else:
                    quiet = why
        elif naming and _gate.level < _BLOCKED:
            speak, cue_seq = False, _gate.state()["cue_seq"]
            quiet = "blocked — confirming what it is"
        else:
            speak, cue_seq, quiet = _gate.admit(text, now, _world.episode)
        if raw["rule"] in ("waiting", "stale"):
            quiet = quiet if (hold and not through_hold) else raw["reason"]
        announced = _gate.counted
        shadow = (
            _shadow.update((raw.get("mover") or {}).get("would_say"), now)
            if not LIDAR_MOVERS_SPEAK and not hold
            else None
        )
        route_shadow = (
            _route_shadow.update(raw.get("route_would_say"), now)
            if not ROUTE_PLANNER_SPEAK and not hold
            else None
        )
        # A whole tick after "Guidance stopped" (fault_notice): guidance is back, so what
        # stands now is said again rather than taken as already said, and the next fault
        # is told at once. Here, not at the top: a tick that raises must not get this far.
        if _fault["said_at"] is not None and not speak:
            _fault["said_at"] = None
            _gate.forget_said()

    level = _LEVEL.get(text, -1)
    return {
        "instruction": text,
        "action": _ACTION.get(text, "unknown"),
        "rule": (
            "offline"
            if text == SENSORS_OFFLINE
            else "status"
            if text in _STATUS_NOTICES
            else "startup"
            if text in _STARTUP_NOTICES
            else "crossing"
            if text == TRAFFIC_LIGHT_AHEAD
            else "mover_approaching"
            if text in _MOVER_SENTENCES
            else "vehicle_approaching"
            if text in _VEHICLE_APPROACH
            else "vehicle_ahead"
            if text == VEHICLE_AHEAD
            else "steering"
            if _LEVEL.get(text) == 1 and text in _STEER_INFO
            else "near_wall"
            if text in (NEAR_LEFT, NEAR_RIGHT)
            else "glass_beside"
            if text in _GLASS_BESIDE
            else "drifting"
            if text in (DRIFT_LEFT, DRIFT_RIGHT)
            else "blocked"
            if level == _BLOCKED
            else "approaching"
            if level == 1
            else "clear"
            if text
            else raw["rule"]
        ),
        "side": (
            _world.side
            if text in (TURN_LEFT, TURN_RIGHT)
            else _STEER_INFO[text]["turn"]
            if text in _STEER_INFO
            else _MOVER_INFO[text]["move"]
            if text in _MOVER_INFO
            else "left"
            if text in (DRIFT_LEFT, NEAR_LEFT, VEHICLE_FROM_LEFT, GLASS_LEFT)
            else "right"
            if text in (DRIFT_RIGHT, NEAR_RIGHT, VEHICLE_FROM_RIGHT, GLASS_RIGHT)
            else None
        ),
        "obstacle": (
            "vehicle"
            if text in _VEHICLE_APPROACH + (VEHICLE_AHEAD,)
            else _MOVER_INFO[text]["word"].lower()
            if text in _MOVER_INFO
            else (_STEER_INFO[text]["word"] or "").lower() or None
            if text in _STEER_INFO
            else "glass"
            if text in (BLOCKED_GLASS, GLASS_AHEAD) + _GLASS_BESIDE
            else "wall"
            if text in (BLOCKED_WALL, WALL_AHEAD) + _SIDE_WALL
            else _OBJECT_OF[text].lower()
            if text in _OBJECT_OF
            else None
        ),
        "reason": _reason(text, raw),
        "raw": {
            k: raw.get(k)
            for k in (
                "instruction",
                "blocked",
                "near",
                "glass",
                "wall",
                "drift",
                "close",
                "beside",
                "closing",
                "converging",
                "vehicle",
                "vehicle_ahead",
                "vehicles",
                "steer",
                "user_still",
                "glass_side",
                "object",
                "front_ft",
                "obstacle_ahead",
                "mover",
                "movers",
                "radar_movers",
                "route",
                "route_would_say",
                "tracks",
            )
        },
        "speak": speak,
        "cue_seq": cue_seq,
        "quiet_reason": quiet,
        "announced": announced,
        # A LiDAR mover warning held and new on this poll, not spoken (LIDAR_MOVERS_SPEAK).
        "mover_shadow": shadow,
        "mover_shadow_reason": raw.get("mover_reason", "") if shadow else "",
        # The route planner's turn or steer, held and new on this poll, not spoken (ROUTE_PLANNER_SPEAK).
        "route_shadow": route_shadow,
        "route_shadow_reason": raw.get("route_reason", "") if route_shadow else "",
    }


def fault_notice(why: str, now: Optional[float] = None) -> Dict[str, Any]:
    """The guidance step itself keeps failing (unified_guidance, GUIDANCE_FAIL_TICKS): "Guidance stopped, use your cane", every SIMPLE_OFFLINE_REPEAT_S, through the gate's status path so the phone gets a cue for it too (2026-10-21)."""
    now = time.time() if now is None else now
    with _lock:
        last = _fault["said_at"]
        if last is None or not (0.0 <= now - last < SIMPLE_OFFLINE_REPEAT_S):
            speak, cue_seq, quiet = _gate.admit_status(SENSORS_OFFLINE, now, cut_in=True)
        else:
            speak, cue_seq = False, _gate.state()["cue_seq"]
            quiet = f"guidance fault — said {now - last:.0f}s ago"
        if speak:
            _fault["said_at"] = now
        announced = _gate.counted
    return {
        "instruction": SENSORS_OFFLINE,
        "action": _ACTION[SENSORS_OFFLINE],
        "rule": "offline",
        "side": None,
        "obstacle": None,
        "reason": f"the guidance step is failing: {why}",
        "raw": {},
        "speak": speak,
        "cue_seq": cue_seq,
        "quiet_reason": quiet,
        "announced": announced,
    }


def voice_failed(seq: int, text: str) -> bool:
    """The Pi voice could not play cue `seq`, which the gate counted as said when it was queued: unless something newer was said since, forget it so it is said again (pi_voice, 2026-10-23). True if forgotten."""
    with _lock:
        if _gate.state()["cue_seq"] != seq or _gate.last != text:
            return False
        _gate.forget_said()
        return True


def reset() -> None:
    """Forget what was said and what was seen — a new journey."""
    with _lock:
        _gate.reset()
        _world.reset()
        _glass.reset()
        _camera.reset()
        _power.reset()
        _heat.reset()
        _shadow.reset()
        _route_shadow.reset()
        _startup.update(held=False, starting_said=False, ready_said=False)
        _fault["said_at"] = None
    try:
        from utils.route_planner import get_planner

        get_planner().reset()
    except Exception:
        pass


def state() -> Dict[str, Any]:
    with _lock:
        return {
            **_gate.state(),
            "counted": _gate.counted,
            "glass_sensor": _glass.state(),
            "camera": _camera.state(),
            "power": _power.state(),
            "heat": _heat.state(),
        }
