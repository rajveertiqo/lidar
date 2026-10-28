"""Drift guidance — catch the wall BEFORE it is a wall in front of you."""

from typing import Any, Dict, List, Optional, Sequence, Tuple

import math
import re
import threading
import time

import numpy as np

from config import (
    CRITICAL_OBJECTS,
    DRIFT_ALERT_TTC_S,
    DRIFT_BEAR_DEG,
    DRIFT_BODY_RADIUS_MM,
    DRIFT_CLEAR_TTC_S,
    DRIFT_CONFIRM_SCANS,
    DRIFT_ENABLED,
    DRIFT_FAN_DEG,
    DRIFT_FAN_STEP_DEG,
    DRIFT_IMPROVE_S,
    DRIFT_LOOKAHEAD_S,
    DRIFT_MAX_RANGE_MM,
    DRIFT_RELEASE_TTC_S,
    DRIFT_REPEAT_S,
    DRIFT_SLIGHT_DEG,
    DRIFT_STEP_S,
    DRIFT_SURFACE_GAP_MM,
    DRIFT_SURFACE_MAX_RESIDUAL_MM,
    DRIFT_SURFACE_MIN_POINTS,
    DRIFT_SURFACE_MIN_SPAN_MM,
    DRIFT_URGENT_TTC_S,
    DRIFT_VEER_DEG,
    DRIFT_VEER_RATE_DPS,
    DRIFT_VEER_WINDOW_S,
    DRIFT_WALL_CLOSING_FT,
    DRIFT_WALL_HORIZON_S,
    DRIFT_WALL_NEAR_FT,
    DRIFT_WALL_RELEASE_FT,
    DRIFT_WALL_ROOM_FT,
    DRIFT_WALL_SIDE_MAX_DEG,
    DRIFT_WALL_SIDE_MIN_DEG,
    GUIDANCE_AWARE_MM,
    RADAR_AGREE_MM,
    RADAR_BEARING_OFFSET_DEG,
    RADAR_CARDINAL_ARC_SECTORS,
    RADAR_CARDINAL_CENTRES,
    RADAR_SECTORS,
    RADAR_BEAM_H_DEG,
)

# Urgency ladder. "clear" is not spoken at all; the rest escalate.
LEVEL_CLEAR = "clear"
LEVEL_NOTICE = "notice"  # visible on the dashboard, not yet worth speaking
LEVEL_ADVISORY = "advisory"  # a gentle nudge — the whole point of this module
LEVEL_URGENT = "urgent"  # correct now, or walk_policy is about to stop you
_LEVEL_RANK = {LEVEL_CLEAR: 0, LEVEL_NOTICE: 1, LEVEL_ADVISORY: 2, LEVEL_URGENT: 3}
_LABEL_DEG = 15.0  # a camera detection names a contact point within this bearing

MM_PER_FT = 304.8


def signed_bearing(angle_deg: float) -> float:
    """LiDAR bearing (0-360, clockwise) -> signed degrees, + = right, - = left."""
    return ((float(angle_deg) + 180.0) % 360.0) - 180.0


def _side(signed_deg: float) -> str:
    return "right" if signed_deg >= 0 else "left"


def _side_of(bearing_deg: float) -> Optional[str]:
    """The side a THING sits on, or None when it is dead ahead."""
    if abs(bearing_deg) < DRIFT_FAN_STEP_DEG:
        return None
    return _side(bearing_deg)


def _opposite(side: str) -> str:
    return "left" if side == "right" else "right"


def project_path(
    speed_mps: float,
    yaw_rate_dps: float = 0.0,
    heading_deg: float = 0.0,
    lookahead_s: float = DRIFT_LOOKAHEAD_S,
    step_s: float = DRIFT_STEP_S,
    lateral_mps: float = 0.0,
) -> np.ndarray:
    """Where the user will be, in mm, at each future time step."""
    n = max(1, int(round(lookahead_s / step_s)))
    heading = math.radians(heading_deg)
    yaw_step = math.radians(yaw_rate_dps) * step_s
    dist_step = speed_mps * step_s * 1000.0  # signed forward mm per step
    side_step = lateral_mps * step_s * 1000.0

    positions = np.empty((n, 2), dtype=np.float64)
    x = y = 0.0
    for i in range(n):
        heading += yaw_step
        x += dist_step * math.cos(heading) - side_step * math.sin(heading)
        y += dist_step * math.sin(heading) + side_step * math.cos(heading)
        positions[i] = (x, y)
    return positions


def sweep_collision(
    positions: np.ndarray,
    points_xy: np.ndarray,
    radius_mm: float,
    step_s: float = DRIFT_STEP_S,
) -> Tuple[Optional[float], Optional[int]]:
    """First time the swept body disc touches a point."""
    if points_xy.size == 0 or positions.size == 0:
        return None, None

    r2 = float(radius_mm) ** 2

    outside = (points_xy**2).sum(axis=1) > r2
    if not outside.any():
        return None, None
    candidates = points_xy[outside]
    index_of = np.flatnonzero(outside)

    for k in range(positions.shape[0]):
        d2 = ((candidates - positions[k]) ** 2).sum(axis=1)
        hit = int(np.argmin(d2))
        if d2[hit] <= r2:
            return (k + 1) * step_s, int(index_of[hit])
    return None, None


def _reachable(
    points_xy: np.ndarray, speed_mps: float, lookahead_s: float = DRIFT_LOOKAHEAD_S
) -> np.ndarray:
    """Drop everything the user could not reach inside the horizon."""
    if points_xy.size == 0:
        return points_xy
    reach = max(abs(speed_mps), 0.1) * lookahead_s * 1000.0 + DRIFT_BODY_RADIUS_MM * 2.0
    within = (points_xy**2).sum(axis=1) <= reach**2
    return points_xy[within]


def fit_surfaces(points: Sequence[Tuple[float, float]]) -> List[Dict[str, Any]]:
    """Group returns into flat surfaces and fit each with a total-least-squares line."""
    ordered = sorted(
        ((signed_bearing(a), float(d)) for a, d in points if d and d > 0),
        key=lambda p: p[0],
    )
    if len(ordered) < DRIFT_SURFACE_MIN_POINTS:
        return []

    runs: List[List[Tuple[float, float]]] = []
    current = [ordered[0]]
    for angle, dist in ordered[1:]:
        prev_angle, prev_dist = current[-1]
        gap_limit = max(DRIFT_SURFACE_GAP_MM, 0.25 * prev_dist)
        if (angle - prev_angle) > 3.0 or abs(dist - prev_dist) > gap_limit:
            runs.append(current)
            current = [(angle, dist)]
        else:
            current.append((angle, dist))
    runs.append(current)

    surfaces = []
    for run in runs:
        surfaces.extend(_split_and_fit(run))
    return surfaces


def _split_and_fit(
    run: Sequence[Tuple[float, float]], depth: int = 0
) -> List[Dict[str, Any]]:
    """Fit a run, splitting it wherever one line cannot describe it."""
    if len(run) < DRIFT_SURFACE_MIN_POINTS:
        return []

    surface, _residual = _fit_line(run)
    if surface is None:
        return []

    if (
        surface["rms_mm"] <= DRIFT_SURFACE_MAX_RESIDUAL_MM
        or depth >= 8  # backstop: noise can never be fitted
        or len(run) < 2 * DRIFT_SURFACE_MIN_POINTS
    ):
        return [surface]

    ang = np.radians(np.array([a for a, _ in run], dtype=np.float64))
    dist = np.array([d for _, d in run], dtype=np.float64)
    xy = np.column_stack((dist * np.cos(ang), dist * np.sin(ang)))

    chord = xy[-1] - xy[0]
    length = float(np.hypot(chord[0], chord[1]))
    if length < 1e-6:
        return [surface]
    normal = np.array([-chord[1], chord[0]]) / length
    deviation = np.abs((xy - xy[0]) @ normal)

    edge = DRIFT_SURFACE_MIN_POINTS
    interior = deviation[edge : len(run) - edge]
    if interior.size == 0:
        return [surface]

    worst = edge + int(np.argmax(interior))
    return _split_and_fit(run[: worst + 1], depth + 1) + _split_and_fit(
        run[worst:], depth + 1
    )


def _fit_line(
    run: Sequence[Tuple[float, float]],
) -> Tuple[Optional[Dict[str, Any]], Optional[np.ndarray]]:
    """Total-least-squares (PCA) line through one run of returns."""
    ang = np.radians(np.array([a for a, _ in run], dtype=np.float64))
    dist = np.array([d for _, d in run], dtype=np.float64)
    xy = np.column_stack((dist * np.cos(ang), dist * np.sin(ang)))

    centroid = xy.mean(axis=0)
    centred = xy - centroid
    try:
        _u, _sv, vt = np.linalg.svd(centred, full_matrices=False)
    except np.linalg.LinAlgError:
        return None, None

    direction = vt[0]
    normal = np.array([-direction[1], direction[0]])
    residual = centred @ normal
    rms = float(np.sqrt(np.mean(residual**2)))
    extent = centred @ direction
    span = float(extent.max() - extent.min())

    is_flat = rms <= DRIFT_SURFACE_MAX_RESIDUAL_MM
    return {
        "kind": "wall" if (is_flat and span >= DRIFT_SURFACE_MIN_SPAN_MM) else "object",
        "centroid_mm": [float(centroid[0]), float(centroid[1])],
        "direction": [float(direction[0]), float(direction[1])],
        "extent_mm": [float(extent.min()), float(extent.max())],
        "bearing_deg": round(math.degrees(math.atan2(direction[1], direction[0])), 1),
        "span_mm": round(span, 1),
        "rms_mm": round(rms, 1),
        "points": len(run),
        "nearest_mm": round(float(np.min(np.hypot(xy[:, 0], xy[:, 1]))), 1),
        "centre_bearing_deg": round(
            math.degrees(math.atan2(centroid[1], centroid[0])), 1
        ),
    }, residual


def _describe_hazard(
    contact_xy: np.ndarray, surfaces: List[Dict[str, Any]], camera_state: Dict[str, Any]
) -> Dict[str, Any]:
    """Name what the user is converging on."""
    cx, cy = float(contact_xy[0]), float(contact_xy[1])
    bearing = math.degrees(math.atan2(cy, cx))

    nearest_surface, best = None, float("inf")
    for surface in surfaces:
        sx, sy = surface["centroid_mm"]
        dx, dy = surface["direction"]
        along = (cx - sx) * dx + (cy - sy) * dy
        off = abs((cx - sx) * -dy + (cy - sy) * dx)
        lo, hi = surface["extent_mm"]
        margin = DRIFT_BODY_RADIUS_MM
        if not (lo - margin <= along <= hi + margin):
            continue
        if off > max(300.0, 3.0 * surface["rms_mm"]) + margin:
            continue
        if off < best:
            nearest_surface, best = surface, off

    name = "obstacle"
    if nearest_surface is not None:
        name = "wall" if nearest_surface["kind"] == "wall" else "obstacle"

    from utils.lidar_utils import get_sector

    sector = get_sector(bearing % 360.0)
    # Named only by a CURRENT picture's detection at the contact's own bearing: the top
    # detection of the mapped camera's last frame, however old, named the wrong thing
    # (scan 2026-10-23).
    from utils.sensor_fusion import _detections_by_sector, fresh_camera_state

    seen = _detections_by_sector(fresh_camera_state(camera_state), {}, time.time())
    critical = [
        d
        for dets in seen.values()
        for d in dets
        if str(d.get("class_name", "")).lower() in CRITICAL_OBJECTS
        and abs((d["bearing_deg"] - bearing + 180.0) % 360.0 - 180.0) <= _LABEL_DEG
    ]
    if critical:
        top = max(critical, key=lambda d: float(d.get("confidence", 0.0)))
        name = str(top.get("class_name"))

    return {
        "object": name,
        "bearing_deg": round(bearing, 1),
        "side": _side_of(bearing),
        "distance_mm": round(math.hypot(cx, cy), 1),
        "distance_ft": round(math.hypot(cx, cy) / MM_PER_FT, 1),
        "sector": sector,
        "surface": nearest_surface,
    }


def _closest_on_surface(
    surface: Dict[str, Any], origin: np.ndarray
) -> Tuple[np.ndarray, float]:
    """Closest point of a fitted wall SEGMENT to `origin`, and its distance."""
    centroid = np.asarray(surface["centroid_mm"], dtype=np.float64)
    direction = np.asarray(surface["direction"], dtype=np.float64)
    low, high = surface["extent_mm"]
    along = float(np.dot(origin - centroid, direction))
    closest = centroid + min(max(along, low), high) * direction
    return closest, float(np.hypot(*(closest - origin)))


def wall_proximity(
    surfaces: List[Dict[str, Any]],
    speed_mps: float,
    yaw_rate_dps: float = 0.0,
    lateral_mps: float = 0.0,
) -> Dict[str, Optional[Dict[str, Any]]]:
    """Nearest wall on the LEFT and on the RIGHT, and whether it is closing."""
    result: Dict[str, Optional[Dict[str, Any]]] = {"left": None, "right": None}
    if not surfaces:
        return result

    origin = np.zeros(2)
    future = project_path(
        speed_mps,
        yaw_rate_dps=yaw_rate_dps,
        lookahead_s=DRIFT_WALL_HORIZON_S,
        step_s=DRIFT_WALL_HORIZON_S,
        lateral_mps=lateral_mps,
    )[-1]

    for surface in surfaces:
        if surface.get("kind") != "wall":
            continue

        closest, distance_mm = _closest_on_surface(surface, origin)
        bearing = math.degrees(math.atan2(closest[1], closest[0]))
        if not (DRIFT_WALL_SIDE_MIN_DEG <= abs(bearing) <= DRIFT_WALL_SIDE_MAX_DEG):
            continue  # ahead or behind — not this rule's job

        side = _side(bearing)
        _later, later_mm = _closest_on_surface(surface, future)

        entry = {
            "distance_ft": round(distance_mm / MM_PER_FT, 2),
            "later_ft": round(later_mm / MM_PER_FT, 2),
            "closing": (distance_mm - later_mm) >= DRIFT_WALL_CLOSING_FT * MM_PER_FT,
            "bearing_deg": round(bearing, 1),
            "span_ft": round(surface["span_mm"] / MM_PER_FT, 1),
        }
        # Keep the NEAREST wall per side — a lobby can show several.
        if result[side] is None or entry["distance_ft"] < result[side]["distance_ft"]:
            result[side] = entry

    return result


_wall_latch: Dict[str, bool] = {"left": False, "right": False}
_wall_lock = threading.Lock()


def wall_alert(walls: Dict[str, Optional[Dict[str, Any]]]) -> Optional[Dict[str, Any]]:
    """Which side, if any, the user should move away from."""
    with _wall_lock:
        near = {}
        for side in ("left", "right"):
            wall = walls.get(side)
            if wall is None:
                _wall_latch[side] = False
                continue
            distance = wall["distance_ft"]
            if distance <= DRIFT_WALL_NEAR_FT:
                _wall_latch[side] = True
            elif distance >= DRIFT_WALL_RELEASE_FT:
                _wall_latch[side] = False
            if _wall_latch[side]:
                near[side] = wall

        if not near:
            return None

        if len(near) == 2:
            tighter = min(near, key=lambda s: near[s]["distance_ft"])
            return {
                "side": None,
                "narrow": True,
                "distance_ft": near[tighter]["distance_ft"],
                "closing": near[tighter]["closing"],
                "room_ft": None,
            }

        side = next(iter(near))
        other = walls.get(_opposite(side))
        room = other["distance_ft"] if other else None
        if room is not None and room < DRIFT_WALL_ROOM_FT:
            return None  # not enough space to send them across

        return {
            "side": side,
            "narrow": False,
            "distance_ft": near[side]["distance_ft"],
            "closing": near[side]["closing"],
            "room_ft": room,
        }


def reset_wall_latch() -> None:
    with _wall_lock:
        _wall_latch["left"] = _wall_latch["right"] = False


_BOARD_KEY = re.compile(r"^deg(\d{3})$")  # radar_board_key's format


def glass_warning(
    radar_state: Dict[str, Any],
    lidar_state: Dict[str, Any],
    path_bearing_deg: float,
    speed_mps: float,
) -> Optional[Dict[str, Any]]:
    """A transparent surface on the projected heading, seen only by the radar."""
    sectors = (radar_state or {}).get("sectors", {}) or {}
    lidar_sectors = (lidar_state or {}).get("sectors", {}) or {}
    centres = dict(RADAR_CARDINAL_CENTRES)

    def _arc(direction: str) -> Tuple[float, float]:
        """(centre, half-width) of a cardinal's ARC, from the boards that are LIVE."""
        sec = sectors.get(direction) or {}
        live = sec.get("sources_live")
        keys = live if live is not None else (sec.get("sources") or ())
        bearings = [float(m.group(1)) for k in keys if (m := _BOARD_KEY.match(str(k)))]
        if not bearings:
            n = len(keys)
            return (
                centres[direction],
                (RADAR_BEARING_OFFSET_DEG if n > 1 else 0.0) + RADAR_BEAM_H_DEG / 2.0,
            )
        rad = [math.radians(b) for b in bearings]
        centre = math.degrees(
            math.atan2(sum(map(math.sin, rad)), sum(map(math.cos, rad)))
        )
        spread = max(abs(signed_bearing(b - centre)) for b in bearings)
        return centre, spread + RADAR_BEAM_H_DEG / 2.0

    from utils.sensor_fusion import lidar_is_fresh

    lidar_fresh = "last_update" in (lidar_state or {}) and lidar_is_fresh(lidar_state)

    def _arc_corroborates(direction: str, radar_mm: float) -> bool:
        """Did the laser see the SAME OBJECT the radar echoed off?"""
        if not lidar_fresh:
            return False
        vals = []
        for name in RADAR_CARDINAL_ARC_SECTORS.get(direction, (direction,)):
            v = (lidar_sectors.get(name) or {}).get("min_dist")
            # At any range: cut at GUIDANCE_AWARE_MM, a wall 3-4 m ahead the radar also
            # hears was "possible glass" (scan 2026-10-23).
            if isinstance(v, (int, float)) and v > 0:
                vals.append(float(v))
        if not vals:
            return False
        return abs(min(vals) - radar_mm) <= RADAR_AGREE_MM

    best = None
    for direction in RADAR_SECTORS:
        sector = sectors.get(direction) or {}
        if not sector.get("online"):
            continue
        radar_mm = sector.get("min_dist")
        if not isinstance(radar_mm, (int, float)) or radar_mm <= 0:
            continue

        centre, half = _arc(direction)
        offset = signed_bearing(path_bearing_deg - centre)
        if abs(offset) > half:
            continue

        if _arc_corroborates(direction, radar_mm):
            continue

        ttc = (radar_mm / 1000.0) / max(speed_mps, 0.1)
        if best is None or ttc < best["ttc_s"]:
            best = {
                "direction": direction,
                "distance_mm": round(float(radar_mm), 1),
                "distance_ft": round(float(radar_mm) / MM_PER_FT, 1),
                "ttc_s": round(ttc, 2),
                "beam_offset_deg": round(offset, 1),
            }
    return best


def find_correction(
    points_xy: np.ndarray,
    speed_mps: float,
    radius_mm: float = DRIFT_BODY_RADIUS_MM,
    avoid_side: Optional[str] = None,
    current_ttc_s: Optional[float] = None,
) -> Optional[Dict[str, Any]]:
    """The least disruptive heading change that improves the way ahead."""

    def evaluate(heading: float) -> Dict[str, Any]:
        path = project_path(speed_mps, yaw_rate_dps=0.0, heading_deg=heading)
        ttc, _ = sweep_collision(path, points_xy, radius_mm)
        return {
            "heading_deg": heading,
            "side": _side(heading),
            "magnitude_deg": abs(heading),
            "ttc_s": ttc,
        }

    def is_clear(option: Dict[str, Any]) -> bool:
        return option["ttc_s"] is None or option["ttc_s"] >= DRIFT_CLEAR_TTC_S

    straight = evaluate(0.0)
    if is_clear(straight):
        straight["side"] = None  # no side to it — just stop turning
        return straight

    steps = int(DRIFT_FAN_DEG / DRIFT_FAN_STEP_DEG)
    candidates = [
        evaluate(sign * i * DRIFT_FAN_STEP_DEG)
        for i in range(1, steps + 1)
        for sign in (-1.0, 1.0)
    ]

    def pick(options: List[Dict[str, Any]]) -> Dict[str, Any]:
        if avoid_side is not None:
            preferred = [o for o in options if o["side"] != avoid_side]
            if preferred:
                options = preferred
        return options[0]

    clear = [o for o in candidates if is_clear(o)]
    if clear:
        smallest = min(o["magnitude_deg"] for o in clear)
        return pick(
            sorted(
                [o for o in clear if o["magnitude_deg"] == smallest],
                key=lambda o: (o["ttc_s"] is not None, -(o["ttc_s"] or 0.0)),
            )
        )

    if current_ttc_s is not None:
        better = [
            o
            for o in candidates
            if o["ttc_s"] is not None and o["ttc_s"] >= current_ttc_s + DRIFT_IMPROVE_S
        ]
        if better:
            best = max(o["ttc_s"] for o in better)
            return pick(
                sorted(
                    [o for o in better if o["ttc_s"] == best],
                    key=lambda o: o["magnitude_deg"],
                )
            )
    return None


def steer_phrase(
    what: str, hazard_side: Optional[str], escape_side: Optional[str], wording: str
) -> str:
    """The spoken line for "something is in your path, steer around it"."""
    lead = f"{_article(what).capitalize()} {what}"
    if escape_side and hazard_side and hazard_side != escape_side:
        return f"{lead} on your {hazard_side} — {wording}."
    return f"{lead} coming up — {wording}."


def _phrase_correction(magnitude_deg: float, side: Optional[str]) -> Tuple[str, str]:
    """(action, spoken phrase) for a required heading change."""
    if side is None or magnitude_deg < DRIFT_FAN_STEP_DEG:
        return "straighten", "straighten up"
    if magnitude_deg <= DRIFT_SLIGHT_DEG:
        return f"bear_{side}", f"bear slightly {side}"
    if magnitude_deg <= DRIFT_BEAR_DEG:
        return f"bear_{side}", f"bear {side}"
    return f"turn_{side}", f"turn {side}"


def _article(noun: str) -> str:
    return "an" if noun[:1].lower() in "aeiou" else "a"


def _band(value: float, bands: Sequence[Tuple[float, str]]) -> str:
    """First phrase whose upper bound the value falls under."""
    for limit, phrase in bands:
        if value < limit:
            return phrase
    return bands[-1][1]


_PACE_BANDS = (
    (0.25, "standing still"),
    (0.8, "walking slowly"),
    (1.6, "walking"),
    (float("inf"), "walking quickly"),
)

_TURN_BANDS = (
    (1.0, ""),
    (4.0, "turning slightly"),
    (10.0, "turning"),
    (float("inf"), "turning sharply"),
)

_LINE_BANDS = ((5.0, ""), (15.0, "slightly"), (40.0, ""), (float("inf"), "well"))

_SENSING_BANDS = (
    (0.15, "surroundings too plain to track heading"),
    (0.40, "heading tracking is weak here"),
    (0.70, "heading tracking is fair"),
    (float("inf"), "heading tracking is good"),
)


def describe_heading(heading: Dict[str, Any]) -> Dict[str, str]:
    """Turn the heading numbers into phrases a person can act on."""
    speed = float(heading.get("speed_mps") or 0.0)
    rate = float(heading.get("yaw_rate_dps") or 0.0)
    line = float(heading.get("heading_deg") or 0.0)
    quality = float(heading.get("yaw_quality") or 0.0)
    estimated = heading.get("speed_source") != "odometry"

    pace = _band(abs(speed), _PACE_BANDS)
    if not heading.get("moving", True):
        pace = "standing still"
    elif estimated:
        pace += " (pace estimated)"

    turn_word = _band(abs(rate), _TURN_BANDS)
    turning = "going straight" if not turn_word else f"{turn_word} {_side(rate)}"

    line_word = _band(abs(line), _LINE_BANDS)
    if abs(line) < 5.0:
        on_line = "still on your original line"
    else:
        qualifier = f"{line_word} " if line_word else ""
        on_line = f"{qualifier}{_side(line)} of your original line".strip()

    summary = f"{pace}, {turning}"
    if abs(line) >= 5.0:
        summary += f", {on_line}"

    return {
        "pace": pace,
        "turning": turning,
        "line": on_line,
        "sensing": _band(quality, _SENSING_BANDS),
        "summary": summary[0].upper() + summary[1:] if summary else "",
    }


def _describe_veer(veer_deg: float) -> str:
    """How far off a drift has taken the user, without saying a number."""
    magnitude = abs(veer_deg)
    if magnitude < DRIFT_VEER_DEG:
        return "starting to drift"
    if magnitude < DRIFT_VEER_DEG * 2:
        return "drifting steadily"
    return "curving well off your line"


def _describe_turn_size(magnitude_deg: float, side: Optional[str]) -> str:
    """The correction, as prose — used in the explanation, not the instruction."""
    if side is None or magnitude_deg < DRIFT_FAN_STEP_DEG:
        return "simply straightening up"
    if magnitude_deg <= DRIFT_SLIGHT_DEG:
        return f"a slight turn to the {side}"
    if magnitude_deg <= DRIFT_BEAR_DEG:
        return f"a turn to the {side}"
    return f"a firm turn to the {side}"


def _describe_where(hazard: Optional[Dict[str, Any]]) -> str:
    """Where a hazard sits, in words, from its bearing."""
    if not hazard:
        return "ahead"
    offset = abs(float(hazard.get("bearing_deg") or 0.0))
    side = hazard.get("side") or "right"
    if offset <= 15.0:
        return "straight ahead"
    if offset <= 50.0:
        return f"ahead on your {side}"
    if offset <= 110.0:
        return f"beside you on your {side}"
    return f"behind you on your {side}"


def _describe_seconds(seconds: Optional[float]) -> str:
    """Time-to-contact in words."""
    if seconds is None:
        return "clear ahead"
    if seconds < 1.0:
        return "right in front of you"
    if seconds < 2.0:
        return "about a second away"
    return f"about {round(seconds)} seconds away"


class _CueGate:
    """Decides whether a computed cue should actually be SPOKEN."""

    def __init__(self):
        self._lock = threading.Lock()
        self._pending_key: Optional[str] = None
        self._pending_count = 0
        self._spoken_key: Optional[str] = None
        self._spoken_level = LEVEL_CLEAR
        self._spoken_at = 0.0
        self._sequence = 0

    def admit(self, key: Optional[str], level: str, now: float) -> Tuple[bool, int]:
        """(should this be spoken, cue sequence number)."""
        with self._lock:
            if key is None or level in (LEVEL_CLEAR, LEVEL_NOTICE):
                self._pending_key, self._pending_count = None, 0
                self._spoken_key, self._spoken_level = None, LEVEL_CLEAR
                return False, self._sequence

            if key == self._pending_key:
                self._pending_count += 1
            else:
                self._pending_key, self._pending_count = key, 1

            if self._pending_count < DRIFT_CONFIRM_SCANS:
                return False, self._sequence

            escalated = (
                key == self._spoken_key
                and _LEVEL_RANK[level] > _LEVEL_RANK[self._spoken_level]
            )
            if (
                key == self._spoken_key
                and not escalated
                and (now - self._spoken_at) < DRIFT_REPEAT_S
            ):
                return False, self._sequence

            self._spoken_key, self._spoken_level, self._spoken_at = key, level, now
            self._sequence += 1
            return True, self._sequence

    def sequence(self) -> int:
        with self._lock:
            return self._sequence

    def reset(self) -> None:
        with self._lock:
            self._pending_key, self._pending_count = None, 0
            self._spoken_key, self._spoken_level = None, LEVEL_CLEAR


_gate = _CueGate()
_active_lock = threading.Lock()
_active_level = LEVEL_CLEAR


def _grade(ttc_s: Optional[float]) -> str:
    """Time-to-contact -> urgency, with hysteresis on the way back to clear."""
    global _active_level
    with _active_lock:
        if ttc_s is None:
            level = LEVEL_CLEAR
        elif ttc_s <= DRIFT_URGENT_TTC_S:
            level = LEVEL_URGENT
        elif ttc_s <= DRIFT_ALERT_TTC_S:
            level = LEVEL_ADVISORY
        elif ttc_s <= DRIFT_RELEASE_TTC_S:
            level = (
                LEVEL_ADVISORY
                if _active_level in (LEVEL_ADVISORY, LEVEL_URGENT)
                else LEVEL_NOTICE
            )
        else:
            level = LEVEL_CLEAR
        _active_level = level
        return level


def reset_state() -> None:
    """Clear latched cue state — call on a deliberate turn, with the tracker."""
    global _active_level
    _gate.reset()
    reset_wall_latch()
    with _active_lock:
        _active_level = LEVEL_CLEAR


def analyze(
    lidar_state: Optional[Dict[str, Any]] = None,
    radar_state: Optional[Dict[str, Any]] = None,
    camera_state: Optional[Dict[str, Any]] = None,
    heading: Optional[Dict[str, Any]] = None,
    now: Optional[float] = None,
) -> Dict[str, Any]:
    """One drift assessment from the current sensor state."""
    now = time.time() if now is None else now

    if lidar_state is None or radar_state is None or camera_state is None:
        from utils.shared_state import (
            get_camera_state,
            get_lidar_state,
            get_radar_state,
        )

        lidar_state = get_lidar_state() if lidar_state is None else lidar_state
        radar_state = get_radar_state() if radar_state is None else radar_state
        camera_state = get_camera_state() if camera_state is None else camera_state

    if heading is None:
        from utils.heading_tracker import get_tracker

        heading = get_tracker().state()

    if not DRIFT_ENABLED:
        return _clear_result(heading, "drift guidance disabled")

    speed = float(heading.get("speed_mps") or 0.0)
    lateral = (
        float(heading.get("right_mps") or 0.0)
        if heading.get("velocity_trusted")
        else 0.0
    )
    if heading.get("confidently_still"):
        speed = lateral = 0.0
    yaw_rate = float(heading.get("yaw_rate_dps") or 0.0)

    points = _cartesian(lidar_state)
    points = _reachable(points, math.hypot(speed, lateral))

    # --- A. collision course on the user's ACTUAL arc ----------------------
    path = project_path(
        speed, yaw_rate_dps=yaw_rate, heading_deg=0.0, lateral_mps=lateral
    )
    ttc, hit_index = sweep_collision(path, points, DRIFT_BODY_RADIUS_MM)

    surfaces = fit_surfaces(_side_and_forward_points(lidar_state))

    hazard = None
    if hit_index is not None:
        hazard = _describe_hazard(points[hit_index], surfaces, camera_state)

    # How close the walls beside the user are, and whether they are closing.
    walls = wall_proximity(surfaces, speed, yaw_rate, lateral_mps=lateral)
    near_wall = wall_alert(walls)

    end = path[-1]
    path_bearing = math.degrees(math.atan2(end[1], end[0])) if np.any(end) else 0.0
    glass = glass_warning(
        radar_state, lidar_state, path_bearing, math.hypot(speed, lateral)
    )

    effective_ttc = ttc
    if glass is not None and (ttc is None or glass["ttc_s"] < ttc):
        effective_ttc = glass["ttc_s"]

    level = _grade(effective_ttc)

    # --- B. open-space veer ------------------------------------------------
    veer = _veer_check(heading, now)

    # --- the instruction ---------------------------------------------------
    action, phrase, correction, reason = "hold_course", "", None, ""

    glass_leads = glass is not None and effective_ttc == glass["ttc_s"]

    # The near-wall cue gives way to an urgent collision with anything but that wall: it
    # overrode "Stop. Obstacle ahead" (scan 2026-10-23).
    wall_is_it = (
        near_wall is not None
        and hazard is not None
        and (hazard.get("side") == near_wall["side"])
    )
    if near_wall is not None and not (
        level == LEVEL_URGENT and (glass_leads or not wall_is_it)
    ):
        if near_wall["narrow"]:
            action = "caution_narrow"
            phrase = (
                f"Narrow gap, about {near_wall['distance_ft']:.1f} feet "
                f"either side — walk slowly and straight."
            )
            reason = "walls close on both sides; no room to move across"
            level = LEVEL_ADVISORY
        else:
            side = near_wall["side"]
            away = _opposite(side)
            action = f"move_away_{away}"
            if near_wall["closing"]:
                phrase = (
                    f"You are going toward the wall on your {side} — "
                    f"move {away} and walk straight."
                )
            else:
                phrase = (
                    f"You are walking close to the wall on your {side} — "
                    f"ease {away} and walk straight."
                )
            room = (
                "open ground"
                if near_wall["room_ft"] is None
                else f"{near_wall['room_ft']:.1f} ft"
            )
            reason = (
                f"wall {near_wall['distance_ft']:.1f} ft to your {side}"
                + (" and closing" if near_wall["closing"] else "")
                + f"; {room} on your {away}"
            )
            level = LEVEL_URGENT if near_wall["closing"] else LEVEL_ADVISORY

    elif level in (LEVEL_ADVISORY, LEVEL_URGENT) and glass_leads:
        where = (
            "ahead"
            if glass["direction"] == "front"
            else f"on your {glass['direction']}"
        )
        if level == LEVEL_URGENT:
            action, phrase = "stop", f"Stop. Glass door or window {where}."
        else:
            action = "caution_glass"
            phrase = f"Slow down — possible glass door or window {where}."
        reason = (
            f"radar sees something {glass['distance_ft']:.1f} ft {where} "
            f"that the laser looks straight through, "
            f"{_describe_seconds(glass['ttc_s'])} — that pattern means glass"
        )

    elif level in (LEVEL_ADVISORY, LEVEL_URGENT):
        # The steering fan describes forward walking. For other movement,
        # warn/stop at the collision rather than recommend an untested turn.
        correction = None
        if speed > 0 and abs(lateral) < abs(speed):
            correction = find_correction(
                points,
                speed,
                avoid_side=hazard["side"] if hazard else None,
                current_ttc_s=effective_ttc,
            )
        if correction is None:
            what = hazard["object"] if hazard else "obstacle"
            where = _describe_where(hazard)
            feet = (
                (hazard["distance_mm"] / MM_PER_FT)
                if hazard
                else speed * effective_ttc / 0.3048
            )
            if level == LEVEL_URGENT:
                action = "stop"
                phrase = f"Stop. {what.capitalize()} {where}."
            else:
                action = "caution_ahead"
                phrase = (
                    f"{what.capitalize()} {where}, {feet:.0f} feet away, "
                    f"{_describe_seconds(effective_ttc)}."
                )
            reason = (
                f"it is {_describe_seconds(effective_ttc)} and neither "
                f"side is any better — nothing to steer around"
            )
        else:
            action, wording = _phrase_correction(
                correction["magnitude_deg"], correction["side"]
            )
            what = hazard["object"] if hazard else "obstacle"
            if action == "straighten":
                phrase = (
                    f"You are veering {_side(yaw_rate)} toward "
                    f"{_article(what)} {what} — {wording}."
                )
            else:
                phrase = steer_phrase(
                    what,
                    hazard["side"] if hazard else _side_of(path_bearing),
                    correction["side"],
                    wording,
                )
            reason = (
                f"on your current course that is "
                f"{_describe_seconds(effective_ttc)}; "
                f"{_describe_turn_size(correction['magnitude_deg'], correction['side'])}"
                f" clears it"
                + (
                    f"; you are veering {_side(yaw_rate)}"
                    if abs(yaw_rate) >= DRIFT_VEER_RATE_DPS
                    else ""
                )
                + (
                    f"; it is on your {hazard['side']}"
                    if hazard and hazard["side"]
                    else ""
                )
            )

    elif veer["veering"]:
        # Nothing to hit, but the user is not going where they think they are.
        side = veer["side"]
        action, wording = _phrase_correction(abs(veer["veer_deg"]), _opposite(side))
        phrase = f"You are drifting {side} — {wording}."
        reason = (
            f"you have been {_describe_veer(veer['veer_deg'])} to the "
            f"{side} over the last {DRIFT_VEER_WINDOW_S:.0f} seconds, with "
            f"nothing ahead to explain it"
        )
        level = LEVEL_ADVISORY

    # The action alone: with the level in it, the same action going urgent was a new cue
    # to confirm, and _CueGate's escalation check could never fire (scan 2026-10-23).
    key = None if action == "hold_course" else action
    speak, cue_seq = _gate.admit(key, level, now)

    return {
        "enabled": True,
        "action": action,
        "level": level,
        "speak": speak,  # has passed confirmation + anti-repeat
        "cue_seq": cue_seq,
        "instruction": phrase,
        "reason": reason,
        "plain": describe_heading(heading),
        "ttc_s": None if effective_ttc is None else round(float(effective_ttc), 2),
        "ttc_phrase": _describe_seconds(effective_ttc),
        "lidar_ttc_s": None if ttc is None else round(float(ttc), 2),
        "hazard": hazard,
        "glass": glass,
        "correction": correction,
        "walls": walls,
        "near_wall": near_wall,
        "veer": veer,
        "heading": heading,
        "path_bearing_deg": round(path_bearing, 1),
        "points_considered": int(points.shape[0]),
        "updated": now,
    }


def _veer_check(heading: Dict[str, Any], now: float) -> Dict[str, Any]:
    """Case B — a sustained turn with nothing to blame it on."""
    from utils.heading_tracker import get_tracker

    if not heading.get("moving"):
        return {
            "veering": False,
            "veer_deg": 0.0,
            "side": None,
            "plain": "standing still",
            "reason": "not moving",
        }

    accumulated = get_tracker().veer_over(DRIFT_VEER_WINDOW_S, now)
    rate = float(heading.get("yaw_rate_dps") or 0.0)
    if accumulated is None:
        return {
            "veering": False,
            "veer_deg": 0.0,
            "side": None,
            "plain": "still working out your line",
            "reason": "not enough heading history yet",
        }

    by_total = abs(accumulated) >= DRIFT_VEER_DEG
    by_rate = abs(rate) >= DRIFT_VEER_RATE_DPS and (rate * accumulated) > 0
    veering = by_total or by_rate

    return {
        "veering": bool(veering),
        "veer_deg": round(accumulated, 1),
        "rate_dps": round(rate, 2),
        "side": _side(accumulated) if veering else None,
        "window_s": DRIFT_VEER_WINDOW_S,
        # Same fact in words, for anything that shows or speaks it.
        "plain": (
            f"{_describe_veer(accumulated)} to the {_side(accumulated)}"
            if veering
            else "holding your line"
        ),
        "reason": (
            "accumulated heading change"
            if by_total
            else "sustained turn rate"
            if by_rate
            else "on line"
        ),
    }


def _cartesian(lidar_state: Dict[str, Any]) -> np.ndarray:
    """LiDAR points as an (N, 2) array of mm in the body frame (x fwd, y right)."""
    pts = (lidar_state or {}).get("points", {}) or {}
    angles = pts.get("angles") or []
    dists = pts.get("distances") or []
    if not angles or not dists:
        return np.empty((0, 2), dtype=np.float64)

    n = min(len(angles), len(dists))
    a = np.asarray(angles[:n], dtype=np.float64)
    d = np.asarray(dists[:n], dtype=np.float64)
    ok = np.isfinite(a) & np.isfinite(d) & (d > 0) & (d <= DRIFT_MAX_RANGE_MM)
    if not ok.any():
        return np.empty((0, 2), dtype=np.float64)

    rad = np.radians(a[ok])
    return np.column_stack((d[ok] * np.cos(rad), d[ok] * np.sin(rad)))


def _side_and_forward_points(lidar_state: Dict[str, Any]) -> List[Tuple[float, float]]:
    """Returns from ahead AND from both sides, for surface fitting."""
    return _points_within(lidar_state, 150.0)


def _forward_points(lidar_state: Dict[str, Any]) -> List[Tuple[float, float]]:
    """(angle, distance) pairs in the forward half, for surface fitting."""
    return _points_within(lidar_state, 100.0)


def _points_within(
    lidar_state: Dict[str, Any], max_bearing_deg: float
) -> List[Tuple[float, float]]:
    """(angle, distance) pairs within +/- `max_bearing_deg` of straight ahead."""
    pts = (lidar_state or {}).get("points", {}) or {}
    angles = pts.get("angles") or []
    dists = pts.get("distances") or []
    out = []
    for angle, dist in zip(angles, dists):
        if not dist or dist <= 0 or dist > DRIFT_MAX_RANGE_MM:
            continue
        if abs(signed_bearing(angle)) <= max_bearing_deg:
            out.append((float(angle), float(dist)))
    return out


def _clear_result(heading: Dict[str, Any], reason: str) -> Dict[str, Any]:
    return {
        "enabled": DRIFT_ENABLED,
        "action": "hold_course",
        "level": LEVEL_CLEAR,
        "speak": False,
        "cue_seq": _gate.sequence(),
        "instruction": "",
        "reason": reason,
        "plain": describe_heading(heading),
        "ttc_s": None,
        "ttc_phrase": _describe_seconds(None),
        "lidar_ttc_s": None,
        "hazard": None,
        "glass": None,
        "correction": None,
        "walls": {"left": None, "right": None},
        "near_wall": None,
        "veer": {
            "veering": False,
            "veer_deg": 0.0,
            "side": None,
            "plain": "holding your line",
        },
        "heading": heading,
        "path_bearing_deg": 0.0,
        "points_considered": 0,
        "updated": time.time(),
    }
