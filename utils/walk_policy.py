"""Straight-walk policy — one decision, in FEET."""

import math
import re
import threading
import time

from typing import Any, Dict, List, Optional, Tuple

from config import (
    GLASS_BOARD_VIEW_MARGIN_DEG,
    GLASS_DOOR_MAX_BEARING_DEG,
    GLASS_DOOR_WALL_DEPTH_MM,
    GLASS_GRAZING_DEG,
    GLASS_LASER_BEYOND_MM,
    GLASS_SAME_ECHO_MM,
    GUIDANCE_CONE_CENTRES,
    GUIDANCE_SECTORS,
    LIDAR_BLIND_ARCS_DEG,
    LIDAR_NO_RETURN_FT,
    RADAR_AGREE_MM,
    RADAR_BEAM_H_DEG,
    RADAR_CARDINAL_ARC_SECTORS,
    RADAR_CONE_TRUST_MM,
    RADAR_NO_RETURN_FT,
    RADAR_SECTORS,
    WALK_BLOCK_FT,
    WALK_FUSION_GLASS_OVERRIDE,
    WALK_GLASS_GAP_FT,
    WALK_MIN_TURN_FT,
    WALK_TURN_MARGIN_FT,
    WALK_TURN_ORDER,
    WALK_TURN_HYSTERESIS_FT,
    WALK_TURN_HOLD_S,
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


def _matches_ahead(lidar_state: Dict[str, Any], radar_ft: float) -> bool:
    """Laser returns at the echo's range lie within GLASS_DOOR_MAX_BEARING_DEG of ahead, and none beyond it (side walls)."""
    pts = lidar_state.get("points") or {}
    angles, dists = pts.get("angles") or [], pts.get("distances") or []
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


def _board_echoes(
    radar_state: Dict[str, Any], direction: str
) -> List[Tuple[float, float]]:
    """(bearing, echo mm) for each live board of this cardinal that hears something."""
    secs = radar_state.get("sectors") or {}
    out = []
    for key in (secs.get(direction) or {}).get("sources") or ():
        row = secs.get(key) or {}
        m = _BOARD_KEY.match(str(key))
        d = row.get("min_dist")
        if (
            m
            and row.get("online")
            and not row.get("degraded")
            and isinstance(d, (int, float))
            and not isinstance(d, bool)
            and d > 0
        ):
            out.append((float(m.group(1)), float(d)))
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


def _laser_explains(
    lidar_state: Dict[str, Any], bearing: float, echo_mm: float
) -> bool:
    """Does the laser see, inside this board's view, something the echo could have come off — at its range, or nearer, and not seen edge-on?"""
    pts = lidar_state.get("points") or {}
    angles, dists = pts.get("angles") or [], pts.get("distances") or []
    half = RADAR_BEAM_H_DEG / 2.0 + GLASS_BOARD_VIEW_MARGIN_DEG
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
    """The nearest echo one board heard off something the laser sees nothing of in that board's view, in feet, or None."""
    echoes = [
        (e, _laser_explains(lidar_state, b, e))
        for b, e in _board_echoes(radar_state, direction)
    ]
    explained = [e for e, ok in echoes if ok]
    unexplained = [
        e
        for e, ok in echoes
        if not ok and not any(abs(e - x) <= GLASS_SAME_ECHO_MM for x in explained)
    ]
    return _mm_to_ft(min(unexplained)) if unexplained else None


def read_direction(
    lidar_state: Dict[str, Any], radar_state: Dict[str, Any], direction: str
) -> Dict[str, Any]:
    """Both sensors' view of one direction, fused into a single figure in feet."""
    lidar_ft, lidar_measured = _lidar_reading(lidar_state, direction)
    radar_ft, radar_measured = _radar_reading(radar_state, direction)
    lidar_arc_ft = _lidar_arc_ft(lidar_state, direction)
    arc_corroborates = _arc_corroborates(lidar_state, direction, radar_ft)
    # Glass door in a wall (decided 2026-09-29): both boards agree and the laser ahead sees past it, so the arc's returns are the wall beside the pane.
    if (
        arc_corroborates
        and radar_measured
        and lidar_ft is not None
        and (not lidar_measured or lidar_ft - radar_ft >= WALK_GLASS_GAP_FT)
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
        if single is not None and (
            not lidar_measured or lidar_ft - single >= WALK_GLASS_GAP_FT
        ):
            radar_ft, arc_corroborates, board_glass = single, False, True

    radar_off_axis = radar_measured and not _radar_drives_block(
        radar_state, direction, radar_ft, arc_corroborates
    )
    if radar_off_axis:
        radar_measured = False

    distance_ft, how = fuse(
        lidar_ft,
        RADAR_NO_RETURN_FT if radar_off_axis else radar_ft,
        lidar_measured,
        radar_measured,
    )
    if radar_off_axis:
        how += " (radar echo off-axis — the laser sees that object to one side)"
    if board_glass:
        how += (
            " (one front board heard something the laser sees nothing of on its side)"
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
        and not radar_measured,
        "unknown": distance_ft is None,
        "glass": _is_glass(
            lidar_ft, radar_ft, radar_measured, lidar_measured, arc_corroborates
        ),
        "lidar_arc_ft": lidar_arc_ft,
        "arc_corroborates": arc_corroborates,
        "radar_off_axis": radar_off_axis,
        "radar_board_glass": board_glass,
        "radar_blind": radar_ft is None,
        "lidar_blind": lidar_blind(direction),
        "units": "ft",
    }


def read_all(
    lidar_state: Dict[str, Any], radar_state: Dict[str, Any]
) -> Dict[str, Dict[str, Any]]:
    return {d: read_direction(lidar_state, radar_state, d) for d in GUIDANCE_SECTORS}


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


def choose_turn(
    views: Dict[str, Dict[str, Any]],
    front_ft: Optional[float] = None,
    incumbent: Optional[str] = None,
) -> Optional[Dict[str, Any]]:
    """Pick a way out when straight ahead is blocked (rules 4 and 5)."""
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

    # Room only one sensor can vouch for ranks behind room both can: a silent radar misses
    # thin poles, and the LiDAR's blind arc is the wearer's body.
    def rank(view):
        return (
            view["direction"] == "back",
            bool(view.get("radar_blind") or view.get("lidar_blind")),
            -_room_ft(view),
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
    views = read_all(lidar_state, radar_state)
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
    turn = choose_turn(views, front["distance_ft"], incumbent=_incumbent(now))

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
