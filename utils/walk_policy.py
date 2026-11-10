"""Straight-walk policy — one decision, in FEET."""

import collections
import math
import re
import threading
import time

from typing import Any, Dict, List, Optional, Tuple

from config import (
    GLASS_BOARD_VIEW_MARGIN_DEG,
    GLASS_DOOR_MAX_BEARING_DEG,
    GLASS_DOOR_WALL_DEPTH_MM,
    GLASS_GHOST_TOL_MM,
    GLASS_GRAZING_DEG,
    GLASS_LASER_BEYOND_MM,
    GLASS_SAME_ECHO_MM,
    GUIDANCE_CONE_CENTRES,
    GUIDANCE_SECTORS,
    LIDAR_BLIND_ARCS_DEG,
    LIDAR_NO_RETURN_FT,
    RADAR_AGREE_MM,
    GUIDANCE_CORRIDOR_HALF_WIDTH_MM,
    RADAR_BEAM_H_DEG,
    RADAR_BEARING_OFFSET_DEG,
    RADAR_CARDINAL_ARC_SECTORS,
    RADAR_CONE_TRUST_MM,
    RADAR_NO_RETURN_FT,
    RADAR_SECTORS,
    WALK_BLOCK_FT,
    WALK_FUSION_GLASS_OVERRIDE,
    WALK_GLASS_ALL_ECHOES,
    GLASS_FACING_DEG,
    GLASS_TOF_SAME_MM,
    GLASS_NEAR_STEADY_FT,
    GLASS_NEAR_STEADY_S,
    GLASS_NEAR_STEADY_FRAC,
    WALK_GLASS_GAP_FT,
    WALK_MIN_TURN_FT,
    WALK_TURN_MARGIN_FT,
    WALK_TURN_ORDER,
    WALK_TURN_HYSTERESIS_FT,
    WALK_TURN_HOLD_S,
    WALK_TURN_AWAY_FROM_OBSTACLE,
    WALK_TURN_AMPLE_FT,
    WALK_TURN_AWAY_MIN_DEG,
    WALK_MEASURED_MEMORY_S,
)

MM_PER_FT = 304.8

_TURN_PHRASE = {
    "left": "Path blocked. Turn left.",
    "right": "Path blocked. Turn right.",
    "back": "Path blocked. Turn around.",
}


def _mm_to_ft(value: Any) -> Optional[float]:
    """Sensor millimetres -> policy feet. The single conversion boundary."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    # NaN or inf is no reading (fuzz test 2026-10-22): NaN slipped through every
    # comparison, "nan < 4 ft" is False, and the policy said "Walk straight".
    if not math.isfinite(value) or value <= 0:
        return None
    return round(float(value) / MM_PER_FT, 2)


def lidar_blind(direction: str) -> bool:
    """Is this direction's steering cone inside an arc the LiDAR cannot see from where it is worn (LIDAR_BLIND_ARCS_DEG, clockwise from the first bearing to the second)?"""
    centre = GUIDANCE_CONE_CENTRES.get(direction)
    if centre is None:
        return False
    return any(
        (centre - lo) % 360.0 <= (hi - lo) % 360.0 for lo, hi in LIDAR_BLIND_ARCS_DEG
    )


def _lidar_reading(
    lidar_state: Dict[str, Any], direction: str
) -> Tuple[Optional[float], bool]:
    """(feet, measured) for the LiDAR in one direction."""
    if lidar_state.get("simulated"):
        return None, False
    if lidar_blind(direction):
        return None, False  # the wearer's body, not open space (2026-10-21)
    cones = lidar_state.get("cones") or {}
    sectors = lidar_state.get("sectors") or {}
    if not cones and not sectors:
        return None, False  # not scanning — no information
    source = cones if cones else sectors
    view = source.get(direction) or {}
    value = _mm_to_ft(view.get("min_dist"))
    if value is not None:
        return value, True
    if view.get("unswept"):
        return None, False
    return LIDAR_NO_RETURN_FT, False


# RADAR_AGREE_MM in feet — the project's definition of "the same object".
_AGREE_FT = RADAR_AGREE_MM / 304.8


def _lidar_arc_ft(lidar_state: Dict[str, Any], direction: str) -> Optional[float]:
    """Nearest laser return anywhere across a radar cardinal's ARC, in feet."""
    sectors = lidar_state.get("sectors") or {}
    vals = []
    for name in RADAR_CARDINAL_ARC_SECTORS.get(direction, (direction,)):
        v = _mm_to_ft((sectors.get(name) or {}).get("min_dist"))
        if v is not None:
            vals.append(v)
    return min(vals) if vals else None


def _arc_corroborates(
    lidar_state: Dict[str, Any], direction: str, radar_ft: Optional[float]
) -> bool:
    """Did the laser see the SAME OBJECT the radar echoed off?"""
    if radar_ft is None:
        return False
    arc = _lidar_arc_ft(lidar_state, direction)
    return arc is not None and abs(arc - radar_ft) <= _AGREE_FT


def _straddled(radar_state: Dict[str, Any], direction: str) -> bool:
    """True when this cardinal is fed by more than one board."""
    sec = (radar_state.get("sectors") or {}).get(direction) or {}
    return len(sec.get("sources") or ()) > 1


def _radar_drives_block(
    radar_state: Dict[str, Any],
    direction: str,
    radar_ft: Optional[float],
    corroborated: bool,
) -> bool:
    """May this radar echo decide that the direction is BLOCKED?"""
    if radar_ft is None or not _straddled(radar_state, direction):
        return True
    if radar_ft * 304.8 <= RADAR_CONE_TRUST_MM:
        return True
    return not corroborated


def _radar_reading(
    radar_state: Dict[str, Any], direction: str
) -> Tuple[Optional[float], bool]:
    """(feet, measured) for the radar in one direction."""
    if direction not in RADAR_SECTORS:
        return None, False
    sec = (radar_state.get("sectors") or {}).get(direction) or {}
    if not sec.get("online"):
        return None, False  # dead or stale — no information
    value = _mm_to_ft(sec.get("min_dist"))

    if sec.get("degraded"):
        return None, False

    sources = sec.get("sources") or ()
    live = sec.get("sources_live") or ()
    owed = 0
    if "sources_live" in sec:
        try:
            from utils.sensor_fusion import _expected_cardinal_sources

            owed = len(_expected_cardinal_sources().get(direction) or ())
        except Exception:
            owed = 0
    if (
        value is None
        and max(len(sources), owed)
        and len(live) < max(len(sources), owed)
    ):
        return None, False

    if value is not None:
        return value, True
    return RADAR_NO_RETURN_FT, False


def _is_glass(
    lidar_ft: Optional[float],
    radar_ft: Optional[float],
    radar_measured: bool = True,
    lidar_measured: bool = True,
    arc_corroborates: bool = False,
) -> bool:
    """The signature of a transparent surface: the radar echoes off something the laser did not see, or saw much further away."""
    if radar_ft is None or not radar_measured:
        return False  # no echo, no glass claim

    if arc_corroborates:
        return False  # the laser found that same object
    if lidar_ft is None or not lidar_measured:
        return True  # radar sees it, the laser does not
    return (lidar_ft - radar_ft) >= WALK_GLASS_GAP_FT


def fuse(
    lidar_ft: Optional[float],
    radar_ft: Optional[float],
    lidar_measured: bool = True,
    radar_measured: bool = True,
) -> Tuple[Optional[float], str]:
    """One clearance from two readings: the NEARER when both measured, else the one that did."""
    if lidar_ft is None and radar_ft is None:
        return None, "no sensor data"
    if radar_ft is None:
        return lidar_ft, "lidar only (radar not reporting)"
    if lidar_ft is None:
        return radar_ft, "radar only (LiDAR not reporting)"

    disagree = abs(lidar_ft - radar_ft) >= WALK_GLASS_GAP_FT
    nearer_is_measured = lidar_measured if lidar_ft <= radar_ft else radar_measured
    if disagree and nearer_is_measured and WALK_FUSION_GLASS_OVERRIDE:
        # "glass" only when the radar is the nearer: the laser passing through what the
        # radar hears. A nearer laser return is not glass, least of all against a radar
        # that saw nothing (an off-axis echo dismissed included) (scan 2026-10-23).
        if lidar_ft <= radar_ft:
            why = "the laser's" if radar_measured else "radar saw nothing there"
        else:
            why = "glass"
        return min(lidar_ft, radar_ft), f"nearer (sensors disagree — {why})"

    # Not the average (2026-10-30): a radar reading 1.5 ft past the laser, and up to a frame
    # older, put a 3.5 ft wall at 4.25 ft and held "Blocked" back to ~3 ft. Not labelled
    # "nearer": _front_detail reads that as the sensors disagreeing across a glass gap.
    if lidar_measured and radar_measured:
        return min(lidar_ft, radar_ft), "closest of both readings"
    if lidar_measured:
        return lidar_ft, (
            f"LiDAR (radar saw nothing within its {RADAR_NO_RETURN_FT:g} ft range)"
        )
    if radar_measured:
        return radar_ft, "radar (LiDAR saw nothing)"
    return (
        round((lidar_ft + radar_ft) / 2.0, 2),
        "average of both max ranges (nothing detected)",
    )


def _centred_echo(radar_state: Dict[str, Any], direction: str) -> bool:
    """Every board of a straddled cardinal is live and echoes at the same range (RADAR_AGREE_MM): the object is between them, ahead."""
    if not _straddled(radar_state, direction):
        return False
    secs = radar_state.get("sectors") or {}
    rows = [secs.get(k) or {} for k in (secs.get(direction) or {}).get("sources") or ()]
    dists = [r.get("min_dist") for r in rows if r.get("online")]
    if len(dists) < 2 or len(dists) != len(rows):
        return False
    if not all(isinstance(d, (int, float)) and d > 0 for d in dists):
        return False
    return max(dists) - min(dists) <= RADAR_AGREE_MM


def _laser_points(lidar_state: Dict[str, Any]) -> Tuple[List[float], List[float]]:
    """The scan's (angles, distances), only the pairs that are both finite numbers, so the two
    lists stay the same length: one None angle, or a shorter list, raised out of the glass
    checks and made the whole walk decision "unknown" (fuzz, 2026-10-30)."""
    pts = lidar_state.get("points") or {}
    good = [
        (float(a), float(d))
        for a, d in zip(pts.get("angles") or [], pts.get("distances") or [])
        if isinstance(a, (int, float)) and not isinstance(a, bool) and math.isfinite(a)
        and isinstance(d, (int, float)) and not isinstance(d, bool) and math.isfinite(d)
    ]
    return [a for a, _ in good], [d for _, d in good]


_near_frames: "collections.deque[Tuple[float, Tuple[float, ...]]]" = collections.deque(maxlen=96)
_near_lock = threading.Lock()


def _near_echo_steady(radar_state: Dict[str, Any], echo_ft: Optional[float] = None) -> bool:
    """Is an echo at `echo_ft` (the front's) heard in at least GLASS_NEAR_STEADY_FRAC of the
    front radar's frames over the last GLASS_NEAR_STEADY_S? A frame counts when it lists an
    echo within RADAR_AGREE_MM of that range, plus walking pace times the frame's age, so a
    pane being approached matches every frame. Records every frame (by its stamp, not per
    caller); too few frames to judge, or no range given: yes, as before."""
    sec = ((radar_state or {}).get("sectors") or {}).get("front") or {}
    stamp = sec.get("last_update")
    if not isinstance(stamp, (int, float)):
        return True
    peaks = tuple(float(e) for e in (sec.get("peaks_mm") or []) if isinstance(e, (int, float)))
    with _near_lock:
        if not _near_frames or stamp > _near_frames[-1][0]:
            _near_frames.append((float(stamp), peaks))
        elif stamp < _near_frames[-1][0] - 5.0:  # a clock step back: start again
            _near_frames.clear()
            _near_frames.append((float(stamp), peaks))
        recent = [(stamp - t, p) for t, p in _near_frames if stamp - t <= GLASS_NEAR_STEADY_S]
    if echo_ft is None or len(recent) < 4:
        return True  # no history to judge by (start-up, a desk scene): as before
    echo = echo_ft * MM_PER_FT
    hit = [
        any(abs(e - echo) <= RADAR_AGREE_MM + _WALK_MM_S * age for e in p) for age, p in recent
    ]
    # Judged from its first appearance in the window, so frames from before the wearer
    # turned to face it do not count against a pane. Under 4 frames since: not yet glass
    # (still an obstacle, ~0.7 s at 6 frames a second).
    since = hit[hit.index(True):] if True in hit else []
    if len(since) < 4:
        return False
    return sum(since) / len(since) >= GLASS_NEAR_STEADY_FRAC


_WALK_MM_S = 1500.0  # a brisk walking pace: how far a pane's echo can move per second


def _echo_on_line(
    lidar_state: Dict[str, Any], direction: str, radar_ft: Optional[float]
) -> bool:
    """Does the laser see something at the radar echo's range (RADAR_AGREE_MM) ON this
    direction's walking line — within GUIDANCE_CORRIDOR_HALF_WIDTH_MM of it, in front of the
    wearer that way — rather than beside it?"""
    centre = GUIDANCE_CONE_CENTRES.get(direction)
    if centre is None or radar_ft is None:
        return False
    echo = radar_ft * MM_PER_FT
    for a, d in zip(*_laser_points(lidar_state)):
        g = abs((a - centre + 180.0) % 360.0 - 180.0)
        if g >= 90.0 or abs(d - echo) > RADAR_AGREE_MM:
            continue
        if d * math.sin(math.radians(g)) <= GUIDANCE_CORRIDOR_HALF_WIDTH_MM:
            return True
    return False


def _matches_ahead(lidar_state: Dict[str, Any], radar_ft: float) -> bool:
    """Laser returns at the echo's range lie within GLASS_DOOR_MAX_BEARING_DEG of ahead, and none beyond it (side walls)."""
    angles, dists = _laser_points(lidar_state)
    echo_mm = radar_ft * MM_PER_FT
    ahead = beside = False
    for a, d in zip(angles, dists):
        if not isinstance(d, (int, float)) or abs(d - echo_mm) > RADAR_AGREE_MM:
            continue
        off = abs((float(a) + 180.0) % 360.0 - 180.0)
        if off <= GLASS_DOOR_MAX_BEARING_DEG:
            ahead = True
        # Close to a door its own wall lies past 45 deg too (2026-10-01): beside means near enough to have echoed and not out in front at the echo's distance.
        elif (
            off <= 90.0
            and d <= echo_mm + GLASS_LASER_BEYOND_MM
            and d * math.cos(math.radians(off)) < echo_mm - GLASS_DOOR_WALL_DEPTH_MM
        ):
            beside = True
    return ahead and not beside


_BOARD_KEY = re.compile(r"^deg(\d{3})$")


def _echo_mm(d: Any) -> Optional[float]:
    if isinstance(d, (int, float)) and not isinstance(d, bool) and math.isfinite(d) and d > 0:
        return float(d)
    return None


def _front_view_blocked(lidar_state: Dict[str, Any]) -> bool:
    """Something right at the LiDAR hides the way ahead from the laser in this scan
    (lidar_utils._front_blocked, 2026-11-09)."""
    front = ((lidar_state or {}).get("cones") or {}).get("front") or {}
    return front.get("blocked_near_mm") is not None


def _board_echoes(
    radar_state: Dict[str, Any], direction: str, all_peaks: bool = False
) -> List[Tuple[float, float]]:
    """(bearing, echo mm) for each live board of this cardinal that hears something: its
    nearest echo, or with `all_peaks` every echo it lists (peaks_mm)."""
    secs = radar_state.get("sectors") or {}
    out = []
    for key in (secs.get(direction) or {}).get("sources") or ():
        row = secs.get(key) or {}
        m = _BOARD_KEY.match(str(key))
        if not (m and row.get("online") and not row.get("degraded")):
            continue
        echoes = {e for e in [_echo_mm(row.get("min_dist"))] if e is not None}
        if all_peaks:
            echoes |= {e for e in map(_echo_mm, row.get("peaks_mm") or ()) if e is not None}
        out.extend((float(m.group(1)), e) for e in sorted(echoes))
    return out


_GRAZING_TAN = math.tan(math.radians(GLASS_GRAZING_DEG))
_JUMP_TAN = math.tan(
    math.radians(87.0)
)  # steeper than this between returns is a gap, not a surface
_GRAZING_MIN_STEP_MM = 50.0  # range noise cannot make a surface look edge-on


def _grazing(angles: List[Any], dists: List[Any], i: int) -> bool:
    """Is return i on a surface the laser sees nearly edge-on (past GLASS_GRAZING_DEG), judged from its neighbours?"""
    n, r = len(dists), float(dists[i])
    if r <= 0.0:
        return False  # no range: nothing to judge (and no / 0)
    slopes = []
    for j in (i - 1, (i + 1) % n):
        d = dists[j]
        if j == i or not isinstance(d, (int, float)) or d <= 0:
            continue
        dth = math.radians(
            abs((float(angles[j]) - float(angles[i]) + 180.0) % 360.0 - 180.0)
        )
        if not 0.0 < dth <= math.radians(4.0):
            continue
        dr = abs(float(d) - r)
        t = dr / (r * dth)
        if t > _JUMP_TAN:
            continue  # a depth jump: the next return is another surface
        slopes.append(t if dr >= _GRAZING_MIN_STEP_MM else 0.0)
    return bool(slopes) and min(slopes) >= _GRAZING_TAN


_FACING_TAN = math.tan(math.radians(GLASS_FACING_DEG))
# How far round the front boards hear between them (each 30 deg off ahead, its beam half-width
# and the margin a board hears past it): a facing surface there may be what they echoed off.
_FRONT_VIEW_DEG = RADAR_BEARING_OFFSET_DEG + RADAR_BEAM_H_DEG / 2.0 + GLASS_BOARD_VIEW_MARGIN_DEG


def _facing(angles: List[Any], dists: List[Any], i: int) -> bool:
    """Is return i on a surface turned toward the sensor (seen within GLASS_FACING_DEG of
    square-on), one that sends a radar echo straight back? A lone return (a thin pole) is."""
    n, r = len(dists), float(dists[i])
    if r <= 0.0:
        return False
    slopes = []
    for j in (i - 1, (i + 1) % n):
        d = dists[j]
        if j == i or not isinstance(d, (int, float)) or d <= 0:
            continue
        dth = math.radians(abs((float(angles[j]) - float(angles[i]) + 180.0) % 360.0 - 180.0))
        if not 0.0 < dth <= math.radians(4.0):
            continue
        t = abs(float(d) - r) / (r * dth)
        if t <= _JUMP_TAN:  # a depth jump is another surface, not this one's slope
            slopes.append(t)
    return not slopes or min(slopes) <= _FACING_TAN


def _laser_explains(
    lidar_state: Dict[str, Any], bearing: float, echo_mm: float, widen_deg: float = 0.0
) -> bool:
    """Does the laser see, inside this board's view (widened by `widen_deg`), something the echo could have come off — at its range, or nearer, and not seen edge-on?"""
    angles, dists = _laser_points(lidar_state)
    half = RADAR_BEAM_H_DEG / 2.0 + GLASS_BOARD_VIEW_MARGIN_DEG + widen_deg
    for i, (a, d) in enumerate(zip(angles, dists)):
        if (
            isinstance(d, (int, float))
            and echo_mm - RADAR_AGREE_MM <= d <= echo_mm + GLASS_LASER_BEYOND_MM
            and abs((float(a) - bearing + 180.0) % 360.0 - 180.0) <= half
            and not _grazing(angles, dists, i)
        ):
            return True
    return False


def _single_board_glass_ft(
    lidar_state: Dict[str, Any], radar_state: Dict[str, Any], direction: str
) -> Optional[float]:
    """The nearest echo one board heard off something the laser sees nothing of in that board's view, in feet, or None. Every echo a board lists, with WALK_GLASS_ALL_ECHOES — not only its nearest, which in a narrow corridor is the side wall (2026-11-02)."""
    judged = [
        (b, e, _laser_explains(lidar_state, b, e))
        for b, e in _board_echoes(radar_state, direction, all_peaks=WALK_GLASS_ALL_ECHOES)
    ]
    explained = [(b, e) for b, e, ok in judged if ok]
    unexplained = []
    for b, e, ok in judged:
        if ok:
            continue
        # The same thing another board explained, at the same range — only if the laser sees
        # something there this board could hear too, its beam spilling over (until 2026-11-02
        # any range match: a post beside one board hid the pane the other heard).
        if any(abs(e - x) <= GLASS_SAME_ECHO_MM for _bx, x in explained) and _laser_explains(
            lidar_state, b, e, widen_deg=GLASS_BOARD_VIEW_MARGIN_DEG
        ):
            continue
        # A double bounce off a surface this board ranges: its ghost, not a pane.
        if any(abs(e - 2.0 * x) <= GLASS_GHOST_TOL_MM for bx, x in explained if bx == b):
            continue
        unexplained.append(e)
    return _mm_to_ft(min(unexplained)) if unexplained else None


_SIDE_WALL_TOL_MM = 80.0  # a return this close to a side wall's lateral offset lies on it
_SIDE_WALL_MIN_RETURNS = 6
_SIDE_WALL_MIN_SPAN_MM = 400.0


def _side_walls_mm(angles: List[float], dists: List[float]) -> Dict[bool, float]:
    """The lateral offset (mm) of a wall running ALONG the walking line, each side (True:
    right), from the laser: at least _SIDE_WALL_MIN_RETURNS returns 45-135 deg off ahead whose
    lateral offsets line up within 2 x _SIDE_WALL_TOL_MM, spread _SIDE_WALL_MIN_SPAN_MM along
    the path — the nearest such line. A corridor's own walls."""
    out: Dict[bool, float] = {}
    for right in (False, True):
        pts = []
        for a, d in zip(angles, dists):
            signed = (float(a) + 180.0) % 360.0 - 180.0
            if (signed > 0) != right or not 45.0 <= abs(signed) <= 135.0 or not 0.0 < d < 4000.0:
                continue
            r = math.radians(abs(signed))
            pts.append((d * math.sin(r), d * math.cos(r)))  # (lateral, forward)
        pts.sort()
        j = 0
        for i in range(len(pts)):
            j = max(j, i)
            while j + 1 < len(pts) and pts[j + 1][0] - pts[i][0] <= 2.0 * _SIDE_WALL_TOL_MM:
                j += 1
            group = pts[i : j + 1]
            if len(group) >= _SIDE_WALL_MIN_RETURNS and (
                max(g[1] for g in group) - min(g[1] for g in group) >= _SIDE_WALL_MIN_SPAN_MM
            ):
                out[right] = group[len(group) // 2][0]
                break
    return out


def _on_side_wall(a: float, d: float, side_lat: Dict[bool, float]) -> bool:
    """Is the laser return (bearing a, range d) on a side wall _side_walls_mm found?"""
    signed = (float(a) + 180.0) % 360.0 - 180.0
    lat = side_lat.get(signed > 0)
    return (
        lat is not None
        and 30.0 <= abs(signed) <= 150.0
        and abs(d * math.sin(math.radians(abs(signed))) - lat) <= _SIDE_WALL_TOL_MM
    )


def _side_wall_echo(
    angles: List[float],
    dists: List[float],
    side_lat: Dict[bool, float],
    e: float,
    bearings: List[float],
) -> bool:
    """Could a board at one of `bearings` have heard a side wall at `e` mm: the laser sees
    that wall at the echo's range inside the board's view (as _laser_explains)?"""
    half = RADAR_BEAM_H_DEG / 2.0 + GLASS_BOARD_VIEW_MARGIN_DEG
    for i, (a, d) in enumerate(zip(angles, dists)):
        if not (e - RADAR_AGREE_MM <= d <= e + GLASS_LASER_BEYOND_MM):
            continue
        if not any(abs((float(a) - b + 180.0) % 360.0 - 180.0) <= half for b in bearings):
            continue
        if _on_side_wall(a, d, side_lat) and not _grazing(angles, dists, i):
            return True
    return False


def _echo_is_side_wall(
    lidar_state: Dict[str, Any], radar_state: Dict[str, Any], e: float
) -> bool:
    """Is a front echo at `e` mm the wall beside the wearer: EVERY front board that hears it
    (GLASS_SAME_ECHO_MM) has a side wall on its own side the laser sees at that range? A pane
    ahead is heard by the board with no wall beside it too."""
    angles, dists = _laser_points(lidar_state)
    side_lat = _side_walls_mm(angles, dists)
    if not side_lat:
        return False
    hearing = [b for b, x in _board_echoes(radar_state, "front", all_peaks=True)
               if abs(x - e) <= GLASS_SAME_ECHO_MM]
    if not hearing:
        return False
    for b in set(hearing):
        right = 0.0 < (b + 180.0) % 360.0 - 180.0
        if right not in side_lat or not _side_wall_echo(
            angles, dists, {right: side_lat[right]}, e, [b]
        ):
            return False
    return True


def _door_in_wall_ft(
    lidar_state: Dict[str, Any],
    radar_state: Dict[str, Any],
    lidar_ft: float,
    lidar_measured: bool,
    behind_ok: bool = False,
) -> Optional[float]:
    """A glass door at the end of a corridor, in feet, or None (2026-11-06). `behind_ok`: the
    laser on the walking line need only see past it at all, not WALK_GLASS_GAP_FT — the door
    as the front's distance, with someone just behind it (2026-11-09), not named glass.

    The glass-door-in-a-wall rule judged only each board's NEAREST echo, which in a corridor
    is the side wall at the edge of its beam: the door's own echo, behind it, never reached
    the rule, and the per-board check let the corridor's walls "explain" it. A 1.6-2.5 m
    corridor ending in a glass door said "Walk straight" until 2-3 ft from it, or never
    (review, 2026-11-06). Here every echo the front boards list is judged: one that EVERY
    live front board hears alike (RADAR_AGREE_MM), that the laser on the walking line looks
    past (WALK_GLASS_GAP_FT), with the laser seeing wall at that range on BOTH sides of
    the laser sees no surface FACING it at that range on just one side of ahead: what faces
    the sensor is what echoes, so a person or post off the walking line explains the echo,
    while the corridor's own walls, seen obliquely there, do not (they echo nearer, square
    on). Facing surfaces on both sides are the wall the pane is set in; on neither, a pane
    across the corridor. A double bounce off a nearer surface is not a pane."""
    if not WALK_GLASS_ALL_ECHOES or not _straddled(radar_state, "front"):
        return None
    secs = radar_state.get("sectors") or {}
    boards = [k for k in (secs.get("front") or {}).get("sources") or ()]
    echoes = _board_echoes(radar_state, "front", all_peaks=True)
    heard: Dict[float, List[float]] = {}
    for b, e in echoes:
        heard.setdefault(b, []).append(e)
    if len(heard) < 2 or len(heard) != len(boards):  # every board live and hearing
        return None
    angles, dists = _laser_points(lidar_state)
    every = sorted({e for _, e in echoes})
    # The corridor's own side walls, from the laser (2026-11-09). Square-on at the edge of
    # the front view, they read as "something beside" that echoed, and nearer than ~3.8 ft a
    # door's echo IS each board's nearest: it was set aside as the corridor's wall. A 2 m
    # corridor ending in a glass door said "Walk straight" from 3.8 to 2.2 ft (glass review).
    side_lat = _side_walls_mm(angles, dists)
    board_bearings = [
        float(m.group(1)) for m in (_BOARD_KEY.match(str(k)) for k in boards) if m
    ]

    def on_side_wall(a: float, d: float) -> bool:
        return _on_side_wall(a, d, side_lat)

    def side_wall_echo(e: float) -> bool:
        return _side_wall_echo(angles, dists, side_lat, e, board_bearings)

    # Each board's nearest echo, when the laser explains it, is the corridor's wall at the
    # edge of its beam: the rules above judge that one. This one is for what lies behind it.
    walls = set()
    for key in boards:
        row, m = secs.get(key) or {}, _BOARD_KEY.match(str(key))
        near = _echo_mm(row.get("min_dist"))
        if m and near is not None and _laser_explains(lidar_state, float(m.group(1)), near):
            walls.add(near)
    for e in every:
        if not all(any(abs(x - e) <= RADAR_AGREE_MM for x in xs) for xs in heard.values()):
            continue
        e_ft = e / MM_PER_FT
        if lidar_measured and lidar_ft - e_ft < (0.0 if behind_ok else WALK_GLASS_GAP_FT) + 1e-9:
            continue  # the laser stops there too: something solid, not a pane
        # A double bounce off a nearer surface, at twice its range. Any nearer echo: one that
        # had to be RADAR_AGREE_MM nearer never matched a 0.35 m wall's ghost at 0.7 m, and an
        # empty 0.8 m corridor said "Blocked by glass" (scene hunt, 2026-11-06).
        if any(abs(e - 2.0 * x) <= GLASS_GHOST_TOL_MM for x in every if x < e):
            continue
        depth: Dict[bool, List[float]] = {}  # side (right?) -> how far ahead each facing return lies
        beside = False
        for i, (a, d) in enumerate(zip(angles, dists)):
            if abs(d - e) > RADAR_AGREE_MM:
                continue
            signed = (float(a) + 180.0) % 360.0 - 180.0
            off = abs(signed)
            if off < 1.0 or off > _FRONT_VIEW_DEG or not _facing(angles, dists, i):
                continue
            if on_side_wall(a, d):
                continue  # the corridor's own wall: neither beside nor across
            if off > GLASS_DOOR_MAX_BEARING_DEG:
                # Something off to the side, square-on, echoed — unless it is the corridor's
                # own wall, whose echo is the nearest one set aside above (near a door in a
                # 1.2 m corridor its wall lies within the match of the door's range).
                if not any(abs(d - w) <= RADAR_AGREE_MM for w in walls):
                    beside = True
            else:
                depth.setdefault(signed > 0, []).append(d * math.cos(math.radians(signed)))
        if beside:
            continue
        # Facing on both sides: the wall the pane is set in is ONE surface at the pane's
        # depth. Two posts either side at other depths are not (the scene hunt's false
        # glass went 13 -> 31 in 1500 scenes without this; docs/known-limits.md).
        in_wall = False
        if len(depth) == 2:
            zl, zr = (sorted(depth[s])[len(depth[s]) // 2] for s in (False, True))
            in_wall = (
                abs(zl - zr) <= GLASS_DOOR_WALL_DEPTH_MM
                and abs((zl + zr) / 2.0 - e) <= GLASS_DOOR_WALL_DEPTH_MM
            )
        # A board's nearest echo the laser explains is the corridor's wall at the edge of its
        # beam — unless the laser sees the wall a door is set in at that very depth: then it
        # is the door's (2026-11-09).
        if not in_wall and (
            any(abs(e - w) <= GLASS_SAME_ECHO_MM for w in walls) or side_wall_echo(e)
        ):
            continue
        if not depth:
            return e_ft  # nothing facing at that range: a pane across the corridor
        if in_wall:
            return e_ft
    return None


def read_direction(
    lidar_state: Dict[str, Any],
    radar_state: Dict[str, Any],
    direction: str,
    tof_front: Optional[Tuple[float, Dict[str, Any]]] = None,
    tof_solids: Optional[List[Tuple[float, str]]] = None,
    tof_near: Optional[List[int]] = None,
    tof_rest: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Both sensors' view of one direction, fused into a single figure in feet. `tof_rest`:
    the surface the rig is set down on, by the ToF (_tof_rest). `tof_near`:
    the ToF's points on the walking path per TOF_NEAR_BIN_M of distance (_tof_near). `tof_front`:
    (feet, hazard) of the ToF's nearest confirmed solid thing on the path (_tof_front).
    `tof_solids`: (feet, kind) of every solid thing it has confirmed there (_tof_solids)."""
    lidar_ft, lidar_measured = _lidar_reading(lidar_state, direction)
    radar_ft, radar_measured = _radar_reading(radar_state, direction)
    # Set down on a desk (ground_hazards.rest_surface, 2026-11-10): the front boards hear the
    # desk top ~0.3 m ahead, and with the ToF seeing it solid there the echo was "Obstacle in
    # front, turn left" — front 0.9 ft, the laser seeing the window blind at 4 ft. An echo no
    # further than the surface the ToF rests on reaches (REST_ECHO_TOL_M), which the laser sees
    # past, is that surface: dropped, as an off-axis echo is. Worn, there is no such surface.
    rest_echo = bool(
        direction == "front"
        and tof_rest
        and radar_measured
        and isinstance(radar_ft, (int, float))
        and radar_ft * MM_PER_FT / 1000.0
        <= float(tof_rest.get("to_m") or 0.0) + REST_ECHO_TOL_M
        and (
            not lidar_measured
            or (lidar_ft is not None and lidar_ft - radar_ft >= WALK_GLASS_GAP_FT)
        )
    )
    if rest_echo:
        radar_measured = False
    lidar_arc_ft = _lidar_arc_ft(lidar_state, direction)
    arc_corroborates = _arc_corroborates(lidar_state, direction, radar_ft)
    # Glass door in a wall (decided 2026-09-29): both boards agree and the laser ahead sees past it, so the arc's returns are the wall beside the pane.
    # Past it by ANY amount, not only WALK_GLASS_GAP_FT (2026-11-09): a person standing just
    # behind a glass door "explained" its echo, the echo was dropped as off-axis, and the
    # block came at the person, up to 1.9 ft late, or not at all (glass review). The echo is
    # then the nearer reading; naming it glass still needs the gap (_is_glass).
    if (
        arc_corroborates
        and radar_measured
        and lidar_ft is not None
        and (not lidar_measured or lidar_ft > radar_ft)
        and _centred_echo(radar_state, direction)
        and direction == "front"
        and _matches_ahead(lidar_state, radar_ft)
    ):
        arc_corroborates = False
    # Glass in the walking corridor (decided 2026-09-30, NEW-045): an echo nearer than RADAR_CONE_TRUST_MM is inside the corridor the front cone watches and that cone sees past it, so the arc's return is a wall beside the pane.
    if (
        arc_corroborates
        and radar_measured
        and lidar_ft is not None
        and (not lidar_measured or lidar_ft - radar_ft >= WALK_GLASS_GAP_FT)
        and direction == "front"
        and lidar_state.get("cones")
        and radar_ft * MM_PER_FT <= RADAR_CONE_TRUST_MM
        # Not the wall beside the wearer (2026-11-09): in a 0.9 m nook the right board heard
        # its wall at ~1.1-1.2 ft, inside RADAR_CONE_TRUST_MM — "Blocked by glass, turn left"
        # facing a window blind 4.4 ft off, the laser seeing it.
        and not _echo_is_side_wall(lidar_state, radar_state, radar_ft * MM_PER_FT)
    ):
        arc_corroborates = False
    # Each front board on its own (2026-10-01, NEW-045): the pair's nearer echo can be a wall beside one board while the other hears a pane the laser looks through.
    board_glass = False
    if (
        arc_corroborates
        and radar_measured
        and direction == "front"
        and lidar_ft is not None
        and _straddled(radar_state, direction)
    ):
        single = _single_board_glass_ft(lidar_state, radar_state, direction)
        # Nearer than what the laser ranges on the walking line by any amount: the echo is
        # the front's distance (a person just behind a pane, 2026-11-09); glass only past
        # WALK_GLASS_GAP_FT, as before.
        if single is not None and (not lidar_measured or lidar_ft > single):
            radar_ft, arc_corroborates = single, False
            board_glass = not lidar_measured or lidar_ft - single >= WALK_GLASS_GAP_FT
    # A glass door at the end of a corridor: its echo is behind the side walls' (2026-11-06).
    door_glass = False
    if (
        arc_corroborates
        and radar_measured
        and direction == "front"
        and lidar_ft is not None
    ):
        door = _door_in_wall_ft(lidar_state, radar_state, lidar_ft, lidar_measured)
        if door is not None:
            radar_ft, arc_corroborates, door_glass = door, False, True
        elif lidar_measured:
            # Someone just behind the door: the laser stops at them, under WALK_GLASS_GAP_FT
            # past it. The door is still the front's distance (not named glass): the block
            # came at the person, up to 1.9 ft late (glass review, 2026-11-09).
            door = _door_in_wall_ft(lidar_state, radar_state, lidar_ft, True, behind_ok=True)
            if door is not None:
                radar_ft, arc_corroborates = door, False

    # The laser saw nothing ahead because something right at the LiDAR hid it (a hand, the
    # wearer's body, a person pressed close: lidar_utils._front_blocked): that is no
    # evidence of glass. The radar's echo still blocks, as an obstacle (2026-11-09).
    laser_hidden = (
        direction == "front"
        and not lidar_measured
        and _front_view_blocked(lidar_state)
    )

    # Where the laser cannot see down the line itself (LIDAR_BLIND_ARCS_DEG: behind, the
    # wearer's body), an echo the laser explains ON that line is not off-axis: a person 2.3 ft
    # directly behind, seen only by their edge at the blind arc's border, was dismissed and
    # "…, turn around" said into them (turn review, 2026-11-09). One the laser explains
    # beside the line (a corridor's walls at the back boards' beam edges) still is.
    radar_off_axis = (
        radar_measured
        and not _radar_drives_block(radar_state, direction, radar_ft, arc_corroborates)
        and not (lidar_blind(direction) and _echo_on_line(lidar_state, direction, radar_ft))
    )
    if radar_off_axis:
        radar_measured = False

    distance_ft, how = fuse(
        lidar_ft,
        RADAR_NO_RETURN_FT if (radar_off_axis or rest_echo) else radar_ft,
        lidar_measured,
        radar_measured,
    )
    if rest_echo:
        how += (
            f" (the radar's echo {radar_ft:.1f} ft ahead is the surface the device is set down"
            " on, by the ToF: not something in the way)"
        )
    if radar_off_axis:
        how += " (radar echo off-axis — the laser sees that object to one side)"
    if board_glass:
        how += (
            " (one front board heard something the laser sees nothing of on its side)"
        )
    if door_glass:
        how += " (both front boards hear a pane set in the wall ahead, behind the corridor's walls)"
    glass = (
        not lidar_blind(direction)
        and not laser_hidden
        and _is_glass(lidar_ft, radar_ft, radar_measured, lidar_measured, arc_corroborates)
    )
    # The ToF sees something solid on the path the laser does not: a blind or a door it
    # passes through, anything above or below its one plane (2026-11-09: a window blind
    # 2.6 ft ahead, the laser through it, the radar silent — "Walk straight"). It can only
    # bring the front NEARER. It never stands in for the laser in the glass tests nor drops
    # the radar's echo: the first version did, and a table or a frame just behind a pane
    # unnamed the pane and moved the block past it (glass review, 2026-11-09). An echo at
    # the ToF's solid thing, or past it, is that thing, not glass; one in front of it may be.
    tof_ft = None
    # Only where the laser has NO return ahead: where it ranges something, it rules — the
    # ToF's misreads (a side wall seen at an angle, the cane) cannot stop someone there, and
    # a pane the laser looks through keeps its name past the ToF seeing its frame.
    if direction == "front" and tof_front is not None and not lidar_measured:
        tof_ft = float(tof_front[0])
        if distance_ft is None or tof_ft < distance_ft - 1e-6:
            distance_ft = tof_ft
            how += (
                f" (the ToF's {tof_front[1].get('kind')} {tof_ft:.1f} ft ahead, which the"
                " laser does not see, is nearer)"
            )
        if glass and radar_ft is not None and tof_ft <= radar_ft + GLASS_TOF_SAME_MM / 304.8:
            glass = False
            how += " (the ToF sees the echo's surface solid: not glass)"
    # The radar heard what the ToF sees SOLID at that range (RADAR_AGREE_MM): not a pane, which
    # the ToF looks through (2026-11-09). On the rig, facing a desk with a monitor: the laser
    # got nothing back ahead, the radar heard the desk and what stands on it at 0.26 and
    # 0.52 m, steady, and the ToF had a low obstacle 0.55 m ahead, camera-confirmed — "Blocked
    # by glass, turn left" for 14 minutes. A low obstacle or an overhang is what the laser's one
    # plane passes over or under: a table, a counter, a shelf, wherever the laser looks. A
    # taller thing only where the laser has no return ahead: a glass door's frame is one at the
    # pane's range, and the laser looking through the pane keeps it glass (ToF review).
    # And the radar hears that thing itself: one of the front boards' echoes lies on it
    # (GLASS_TOF_SAME_MM; at the desk, 0.526 m on the left board against the ToF's 0.547 m).
    # A low thing just behind a real pane, seen through it, is not heard: the pane stays glass.
    if glass and direction == "front" and tof_solids and isinstance(radar_ft, (int, float)):
        agree_ft = RADAR_AGREE_MM / MM_PER_FT
        same_ft = GLASS_TOF_SAME_MM / MM_PER_FT
        heard_ft = [e / MM_PER_FT for _b, e in _board_echoes(radar_state, "front", all_peaks=True)]
        hit = sorted(
            (t, k)
            for t, k in tof_solids
            if abs(t - radar_ft) <= agree_ft
            and (k in _TOF_OFF_PLANE or not lidar_measured)
            and any(abs(t - h) <= same_ft for h in heard_ft)
        )
        if hit:
            glass = False
            how += (
                f" (the ToF sees a {hit[0][1].replace('_', ' ')} {hit[0][0]:.1f} ft ahead where"
                " the radar heard it: solid, not glass)"
            )
    # The ToF sees a solid surface on the walking path at the echo's distance (GLASS_TOF_SAME_MM),
    # within reach, with no ground needed (ground_hazards.near_counts): the echo is that thing,
    # not a pane, which the ToF looks through (2026-11-09: the rig on a desk, its ToF finding
    # no floor, said "Blocked by glass" at the desk and what stood on it). A glass door's push
    # bar there makes it "Obstacle in front": still the block.
    if glass and direction == "front" and tof_near and isinstance(radar_ft, (int, float)):
        from utils.ground_hazards import TOF_NEAR_BIN_M, TOF_NEAR_MAX_M

        e_m = radar_ft * MM_PER_FT / 1000.0
        tol_m = GLASS_TOF_SAME_MM / 1000.0
        if e_m + tol_m <= TOF_NEAR_MAX_M:
            lo = max(0, int((e_m - tol_m) / TOF_NEAR_BIN_M))
            hi = min(len(tof_near), int((e_m + tol_m) / TOF_NEAR_BIN_M) + 1)
            seen = sum(tof_near[lo:hi])
            if seen >= _TOF_NEAR_MIN_POINTS:
                glass = False
                how += (
                    f" (the ToF sees {seen} points of something solid on the path at the"
                    " echo's distance: not glass)"
                )
    # A pane within GLASS_NEAR_STEADY_FT echoes on nearly every radar frame. An echo there in
    # under GLASS_NEAR_STEADY_FRAC of the last GLASS_NEAR_STEADY_S of frames is clutter (on the
    # rig, 2026-11-09: -17 dB at 1.0 ft on 1 frame in 3-5, in front of a window blind, made
    # "Blocked by glass" — the 3 s glass hold joined the blips up). It still blocks, as an
    # obstacle, not as glass.
    steady = (  # every front frame is recorded, glass or not
        _near_echo_steady(radar_state, radar_ft if radar_measured else None)
        if direction == "front"
        else True
    )
    if (
        glass
        and direction == "front"
        and isinstance(radar_ft, (int, float))
        and radar_ft <= GLASS_NEAR_STEADY_FT
        and not steady
    ):
        glass = False
        how += " (a near echo in under half the radar's frames: clutter, not glass)"
    if laser_hidden and radar_measured:
        how += (
            " (something right at the LiDAR hides the way ahead from the laser: the"
            " radar's echo is an obstacle, not glass)"
        )

    return {
        "direction": direction,
        "lidar_ft": lidar_ft,
        "radar_ft": radar_ft,
        "lidar_measured": lidar_measured,
        "radar_measured": radar_measured,
        "distance_ft": distance_ft,
        "fusion": how,
        "open": (lidar_ft is not None or radar_ft is not None)
        and not lidar_measured
        and not radar_measured
        and tof_ft is None,
        "unknown": distance_ft is None,
        # Glass is the radar hearing what the laser looks through, so none where the laser
        # cannot look (LIDAR_BLIND_ARCS_DEG, the wearer's body): there every echo read as
        # glass, and the dashboard said "left · glass" all the time on the desk (2026-11-05).
        # The echo still counts as an obstacle at its distance.
        "glass": glass,
        "laser_hidden": laser_hidden,
        "rest_echo": rest_echo,
        "tof_ft": None if tof_ft is None else round(tof_ft, 2),
        "lidar_arc_ft": lidar_arc_ft,
        "arc_corroborates": arc_corroborates,
        "radar_off_axis": radar_off_axis,
        "radar_board_glass": board_glass,
        "radar_blind": radar_ft is None,
        "lidar_blind": lidar_blind(direction),
        "units": "ft",
    }


def read_all(
    lidar_state: Dict[str, Any],
    radar_state: Dict[str, Any],
    tof_front: Optional[Tuple[float, Dict[str, Any]]] = None,
    tof_solids: Optional[List[Tuple[float, str]]] = None,
    tof_near: Optional[List[int]] = None,
    tof_rest: Optional[Dict[str, Any]] = None,
) -> Dict[str, Dict[str, Any]]:
    return {
        d: read_direction(
            lidar_state,
            radar_state,
            d,
            tof_front if d == "front" else None,
            tof_solids if d == "front" else None,
            tof_near if d == "front" else None,
            tof_rest if d == "front" else None,
        )
        for d in GUIDANCE_SECTORS
    }


# The ToF's kinds that stand on the path from the ground up, as a wall or a person does for the
# laser. Overhangs and low obstacles wait on TOF_HAZARDS_SPEAK: a wall alongside has read as an
# overhang, and range noise as a low obstacle (review, 2026-11-06).
_TOF_SOLID = ("obstacle", "pole")
# What the LiDAR's one plane passes under or over, wherever it looks (_tof_solids).
_TOF_OFF_PLANE = ("low_obstacle", "overhang")
# This many of the ToF's points at an echo's distance on the walking path are a solid thing
# there (a patch ~3 x 3 deg at 1 m; flying pixels and noise come nowhere near).
_TOF_NEAR_MIN_POINTS = 200


# A front echo no further than this past where the desk the rig rests on ends, by the ToF, is
# the desk (read_direction): the boards sit beside the ToF, a few cm off.
REST_ECHO_TOL_M = 0.15


def _tof_rest() -> Optional[Dict[str, Any]]:
    """The surface the rig is set down on, by the ToF (ground_hazards.rest_surface), from a
    fresh frame while its worker runs; else None."""
    try:
        from config import TOF_ENABLED, TOF_HAZARDS_ENABLED, TOF_HAZARD_FRESH_S
        from utils import ground_hazards as gh

        if not (TOF_ENABLED and TOF_HAZARDS_ENABLED):
            return None
        if gh._thread is None or not gh._thread.is_alive():
            return None
        snap = gh.snapshot()
    except Exception:
        return None
    age = snap.get("age_s")
    if not isinstance(age, (int, float)) or not -1.0 <= age <= TOF_HAZARD_FRESH_S:
        return None
    rest = snap.get("rest")
    return dict(rest) if isinstance(rest, dict) else None


def _tof_near() -> Optional[List[int]]:
    """The ToF's points on the walking path per ground_hazards.TOF_NEAR_BIN_M of distance,
    from a fresh frame while its worker runs — with or without a ground found; else None."""
    try:
        from config import TOF_ENABLED, TOF_HAZARDS_ENABLED, TOF_HAZARD_FRESH_S
        from utils import ground_hazards as gh

        if not (TOF_ENABLED and TOF_HAZARDS_ENABLED):
            return None
        if gh._thread is None or not gh._thread.is_alive():
            return None
        snap = gh.snapshot()
    except Exception:
        return None
    age = snap.get("age_s")
    near = snap.get("near")
    if not isinstance(age, (int, float)) or not -1.0 <= age <= TOF_HAZARD_FRESH_S:
        return None
    return list(near) if isinstance(near, list) else None


def _tof_confirmed() -> List[Dict[str, Any]]:
    """The ToF's hazards on the walking path that are measured, held (3 of 5 frames) and
    confirmed by the camera itself, from a fresh analysis while its worker runs; else []."""
    try:
        from config import TOF_ENABLED, TOF_HAZARDS_ENABLED, TOF_HAZARD_FRESH_S
        from utils import ground_hazards as gh

        if not (TOF_ENABLED and TOF_HAZARDS_ENABLED):
            return []
        if gh._thread is None or not gh._thread.is_alive():
            return []  # not running: tests, desk tools
        snap = gh.snapshot()
    except Exception:
        return []
    age = snap.get("age_s")
    if not snap.get("ok") or not isinstance(age, (int, float)) or age > TOF_HAZARD_FRESH_S:
        return []
    return [
        h
        for h in snap.get("hazards") or []
        if h.get("in_path")
        and h.get("measured", True)
        and h.get("speak")
        and (h.get("camera") or {}).get("status") == "confirmed"
    ]


def _tof_solids() -> List[Tuple[float, str]]:
    """(feet, kind) of every solid thing the ToF has confirmed on the walking path — where it
    stands in the corridor (path_m), else its nearest point: what may explain a radar echo
    the laser does not see (read_direction)."""
    out = []
    for h in _tof_confirmed():
        if h.get("kind") not in _TOF_SOLID + _TOF_OFF_PLANE:
            continue
        m = h.get("path_m") if isinstance(h.get("path_m"), (int, float)) else h.get("ahead_m")
        if isinstance(m, (int, float)) and m > 0:
            out.append((float(m) / 0.3048, str(h["kind"])))
    return out


def _tof_front() -> Optional[Tuple[float, Dict[str, Any]]]:
    """(feet, hazard) of the nearest solid thing the ToF has confirmed on the walking path
    (utils/ground_hazards: 3 of 5 frames, and the camera's check), or None. Only while the
    ToF worker runs and its analysis is under TOF_HAZARD_FRESH_S old."""
    best = None
    # Measured where it stands IN the corridor (path_m), and seen by the camera itself:
    # "unseen" (below the picture: the wearer's own cane) is not enough to stop someone
    # (ToF review, 2026-11-09).
    for h in _tof_confirmed():
        if h.get("kind") in _TOF_SOLID and isinstance(h.get("path_m"), (int, float)):
            ft = float(h["path_m"]) / 0.3048
            if best is None or ft < best[0]:
                best = (ft, h)
    return best


_measured_lock = threading.Lock()
_last_measured: Dict[str, Tuple[float, float]] = {}  # direction -> (ft, at)


def _carry_measured(views: Dict[str, Dict[str, Any]], now: float) -> None:
    """Let a recent MEASUREMENT outvote a missing return."""
    with _measured_lock:
        for direction, view in views.items():
            if direction == "front" or view.get("unknown"):
                continue
            if not view.get("open"):
                # A real return: this is what gets remembered.
                ft = view.get("distance_ft")
                if isinstance(ft, (int, float)):
                    _last_measured[direction] = (float(ft), now)
                continue

            prev = _last_measured.get(direction)
            if not prev:
                continue
            ft, at = prev
            if not (0.0 <= now - at <= WALK_MEASURED_MEMORY_S):
                continue  # expired, or from before a backward clock step
            current = view.get("distance_ft")
            if isinstance(current, (int, float)) and ft >= current:
                continue  # not worse — leave the scan alone
            view["distance_ft"] = ft
            view["open"] = False
            view["carried_ft"] = ft  # so the reason string can say so
            view["carried_age_s"] = round(now - at, 1)


def _room_ft(view: Dict[str, Any]) -> float:
    """Room in a direction for RANKING."""
    if view["unknown"]:
        return float("-inf")
    room = view["distance_ft"]
    if view.get("radar_blind") and room > RADAR_NO_RETURN_FT:
        return RADAR_NO_RETURN_FT
    return room


def _describe_room(view: Dict[str, Any]) -> str:
    """A direction's room, in feet, saying when the figure is a max range standing in for 'saw nothing' rather than something actually ranged."""
    if view["unknown"]:
        return "no data"
    if view["open"]:
        return f"{view['distance_ft']:.1f} ft (nothing detected)"
    return f"{view['distance_ft']:.1f} ft"


def _nothing_detected(front: Dict[str, Any]) -> str:
    """Reason text when both sensors are live and neither saw anything."""
    parts = []
    if front["lidar_ft"] is not None:
        parts.append(f"LiDAR at its {front['lidar_ft']:.0f} ft range")
    if front["radar_ft"] is not None:
        parts.append(f"radar at its {front['radar_ft']:.1f} ft range")
    detail = ", ".join(parts)
    return (
        f"nothing detected ahead — {detail}, so {front['distance_ft']:.1f} ft"
        if detail
        else "nothing detected ahead"
    )


def _front_detail(front: Dict[str, Any]) -> str:
    """The forward clearance, naming BOTH sensors when they disagreed."""
    base = f"{front['distance_ft']:.1f} ft ahead"
    lidar_ft, radar_ft = front["lidar_ft"], front["radar_ft"]
    if lidar_ft is None or radar_ft is None:
        return base
    if not front["fusion"].startswith("nearer"):
        return base

    readings = f"radar {radar_ft:.1f} ft vs LiDAR {lidar_ft:.1f} ft"
    if radar_ft < lidar_ft:
        return (
            f"{base} ({readings} — the laser looks through glass, the radar does not)"
        )
    return (
        f"{base} ({readings} — the radar missed what the laser ranged; "
        f"too low or too thin for its beam)"
    )


def _alternatives(views: Dict[str, Dict[str, Any]], chosen: Optional[str]) -> str:
    """The other directions and their room, so a choice can be checked."""
    return ", ".join(
        f"{d} {_describe_room(views[d])}"
        for d in WALK_TURN_ORDER
        if d in views and d != chosen
    )


_turn_lock = threading.Lock()
_last_turn: Dict[str, Any] = {"direction": None, "at": 0.0}


def _incumbent(now: float) -> Optional[str]:
    """The direction we are still holding, or None if it has expired."""
    with _turn_lock:
        if not _last_turn["direction"]:
            return None
        if not (0.0 <= now - _last_turn["at"] <= WALK_TURN_HOLD_S):
            _last_turn["direction"] = None  # expired, or the clock stepped back
            return None
        return _last_turn["direction"]


def _remember_turn(direction: Optional[str], now: float) -> None:
    """Record what was given."""
    with _turn_lock:
        _last_turn["direction"] = direction
        _last_turn["at"] = now if direction else 0.0


def _front_obstacle_deg(
    lidar_state: Dict[str, Any], front: Dict[str, Any]
) -> Optional[float]:
    """Signed bearing (degrees, + right) of the laser return the front block was measured on,
    or None when the laser did not measure it (a pane: the radar gives no bearing)."""
    cone = ((lidar_state or {}).get("cones") or {}).get("front") or {}
    dist, angle = cone.get("min_dist"), cone.get("min_angle")
    if not (
        front.get("lidar_measured")
        and isinstance(front.get("distance_ft"), (int, float))
        and isinstance(dist, (int, float))
        and isinstance(angle, (int, float))
    ):
        return None
    if abs(dist / MM_PER_FT - front["distance_ft"]) > 0.5:
        return None  # the block is another reading's (the radar's), not this return
    return (float(angle) + 180.0) % 360.0 - 180.0


def choose_turn(
    views: Dict[str, Dict[str, Any]],
    front_ft: Optional[float] = None,
    incumbent: Optional[str] = None,
    obstacle_deg: Optional[float] = None,
) -> Optional[Dict[str, Any]]:
    """Pick a way out when straight ahead is blocked (rules 4 and 5). `obstacle_deg`: where
    the laser measured the block (_front_obstacle_deg), for WALK_TURN_AWAY_FROM_OBSTACLE."""
    if front_ft is None:
        return None  # open ahead — never turn away from it

    floor = max(WALK_MIN_TURN_FT, front_ft + WALK_TURN_MARGIN_FT)

    held_floor = max(WALK_MIN_TURN_FT, floor - WALK_TURN_HYSTERESIS_FT)

    def bar(direction: str) -> float:
        return held_floor if direction == incumbent else floor

    candidates = [
        views[d] for d in WALK_TURN_ORDER if d in views and _room_ft(views[d]) >= bar(d)
    ]
    if not candidates:
        return None

    # Room enough both ways: where the obstacle is decides, not a few more feet (2026-11-02).
    sides = {v["direction"]: v for v in candidates if v["direction"] in ("left", "right")}
    if (
        WALK_TURN_AWAY_FROM_OBSTACLE
        and len(sides) == 2
        and all(_room_ft(v) >= WALK_TURN_AMPLE_FT for v in sides.values())
    ):
        if incumbent in sides:
            return sides[incumbent]  # held: a bearing jittering about ahead cannot flip it
        if obstacle_deg is not None and abs(obstacle_deg) >= WALK_TURN_AWAY_MIN_DEG:
            return sides["left" if obstacle_deg > 0 else "right"]

    # Room only one sensor can vouch for ranks behind room both can: a silent radar misses
    # thin poles, and the LiDAR's blind arc is the wearer's body.
    # Equal room by the fused reading (both side radars hearing the same wall ahead at their
    # beams' edge): the laser's own side reading decides, not WALK_TURN_ORDER — that sent
    # "Blocked by wall, turn left" toward a wall 9 ft off with the right open (2026-11-09).
    def rank(view):
        laser = view.get("lidar_ft")
        return (
            view["direction"] == "back",
            bool(view.get("radar_blind") or view.get("lidar_blind")),
            -_room_ft(view),
            -(laser if isinstance(laser, (int, float)) else 0.0),
        )

    return sorted(candidates, key=rank)[0]


def decide(
    lidar_state: Optional[Dict[str, Any]] = None,
    radar_state: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """The walking decision. All distances in FEET."""
    if lidar_state is None or radar_state is None:
        from utils.shared_state import get_lidar_state, get_radar_state

        lidar_state = get_lidar_state() if lidar_state is None else lidar_state
        radar_state = get_radar_state() if radar_state is None else radar_state

    from utils.sensor_fusion import lidar_is_fresh, lidar_sparse

    if lidar_state and (not lidar_is_fresh(lidar_state) or lidar_sparse(lidar_state)):
        lidar_state = {}
    views = read_all(
        lidar_state,
        radar_state,
        _tof_front() if lidar_state else None,
        _tof_solids() if lidar_state else None,
        _tof_near() if lidar_state else None,
        _tof_rest() if lidar_state else None,
    )
    _carry_measured(views, time.time())
    front = views["front"]

    if front["unknown"]:
        return {
            "action": "unknown",
            "direction": None,
            "blocked": False,
            "instruction": "Waiting for sensors.",
            "reason": "neither the LiDAR nor the radar is reporting ahead",
            "front_ft": None,
            "glass_ahead": False,
            "directions": views,
            "units": "ft",
        }

    # Rules 1-3: walk straight unless something is inside the block radius.
    blocked = front["distance_ft"] < WALK_BLOCK_FT

    if not blocked:
        _remember_turn(None, time.time())
        return {
            "action": "walk_straight",
            "direction": "front",
            "blocked": False,
            "instruction": "Walk straight",
            "reason": (
                _nothing_detected(front)
                if front["open"]
                else f"{_front_detail(front)}, clear to walk ({front['fusion']})"
            ),
            "front_ft": front["distance_ft"],
            "glass_ahead": front["glass"],
            "directions": views,
            "units": "ft",
        }

    # Rules 4-5: blocked ahead, turn toward the most room.
    obstacle = "Glass wall" if front["glass"] else "Obstacle"
    detail = f"{obstacle.lower()} {_front_detail(front)}"
    now = time.time()
    turn = choose_turn(
        views,
        front["distance_ft"],
        incumbent=_incumbent(now),
        obstacle_deg=_front_obstacle_deg(lidar_state, front),
    )

    if turn is None:
        # Nothing beats the way ahead. Two very different situations:
        if front["distance_ft"] >= WALK_MIN_TURN_FT:
            _remember_turn(None, now)
            return {
                "action": "caution_forward",
                "direction": "front",
                "blocked": True,
                "instruction": "Slow down. Something ahead.",
                "reason": f"{detail}; still more room ahead than any turn "
                f"({_alternatives(views, None)})",
                "front_ft": front["distance_ft"],
                "glass_ahead": front["glass"],
                "directions": views,
                "units": "ft",
            }
        _remember_turn(None, now)
        return {
            "action": "stop",
            "direction": None,
            "blocked": True,
            "instruction": "Stop. No clear way.",
            "reason": f"{detail}; no direction has more than {WALK_MIN_TURN_FT:.1f} ft",
            "front_ft": front["distance_ft"],
            "glass_ahead": front["glass"],
            "directions": views,
            "units": "ft",
        }

    _remember_turn(turn["direction"], now)

    room = _describe_room(turn)
    others = _alternatives(views, turn["direction"])
    return {
        "action": f"turn_{turn['direction']}",
        "direction": turn["direction"],
        "blocked": True,
        "instruction": _TURN_PHRASE[turn["direction"]],
        "reason": f"{detail}; {turn['direction']} has the most room at {room}"
        + (f" (vs {others})" if others else ""),
        "front_ft": front["distance_ft"],
        "glass_ahead": front["glass"],
        "directions": views,
        "units": "ft",
    }
