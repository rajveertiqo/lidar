"""Sensor fusion for collaborative blind-navigation guidance."""

import math
import time
from typing import Any, Dict, List, Optional, Tuple

from config import (
    CAMERA_SECTORS,
    CAMERA_WIDTH,
    DETECTION_DISTANCE_AGREE,
    DETECTION_DISTANCE_HFOV_DEG,
    FRAME_GRID_COLS,
    GUIDANCE_CONE_CENTRES,
    BEV_SHAPE_IN_HAZARDS,
    BEV_SHAPE_MATCH_DEG,
    BEV_SHAPE_MATCH_MM,
    ENV_IDENTITY_MIN_VISIBILITY,
    ENV_STALE_S,
    ENV_TRUST_ENABLED,
    GUIDANCE_DANGER_MM,
    GUIDANCE_WARNING_MM,
    GUIDANCE_SAFE_MM,
    GUIDANCE_AWARE_MM,
    LIDAR_MIN_VALID_FRACTION,
    LIDAR_SECTOR_ORDER,
    LIDAR_SECTORS,
    LIDAR_STALE_S,
    SIMPLE_CAMERA_FRESH_S,
    RADAR_AGREE_MM,
    RADAR_CARDINAL_ARC_SECTORS,
    RADAR_CONE_TRUST_MM,
    RADAR_GLASS_GAP_MM,
    RADAR_PRIORITY,
    RADAR_SECTORS,
    RADAR_START_MM,
)
from utils.shared_state import (
    get_lidar_state,
    get_radar_state,
    get_camera_state,
    get_fused_sensor_state,
)

# Distance thresholds (mm) — single source of truth in config.py.
DANGER_MM = GUIDANCE_DANGER_MM  # imminent — stop
WARNING_MM = GUIDANCE_WARNING_MM  # close — caution
SAFE_MM = GUIDANCE_SAFE_MM  # be aware
AWARE_MM = GUIDANCE_AWARE_MM  # furthest distance we bother mentioning

# 8 x 45° LiDAR sectors (clockwise from straight ahead).
SECTORS = list(LIDAR_SECTOR_ORDER)

# Object identity → how much it raises urgency (moving things matter most).
_CRITICAL = {
    "person": 5,
    "pedestrian": 5,
    "car": 4,
    "truck": 4,
    "bus": 4,
    "motorcycle": 4,
    "bicycle": 4,
    "scooter": 4,
    "train": 4,
    "dog": 3,
    "cat": 3,
}

_SECTOR_CLOCK = {
    "front": 12,
    "front_right": 2,
    "right": 3,
    "back_right": 5,
    "back": 6,
    "back_left": 8,
    "left": 9,
    "front_left": 11,
}

_URGENCY_RANK = {"danger": 0, "warning": 1, "caution": 2, "clear": 3}


def angle_to_clock(angle: Optional[float]) -> int:
    """Map a corrected LiDAR angle to a clock hour (front=0°=12, right=90°=3)."""
    if angle is None:
        return 12
    hour = int(round((angle % 360.0) / 30.0)) % 12
    return 12 if hour == 0 else hour


def mm_to_ft(mm: Optional[float]) -> Optional[float]:
    if mm is None or mm <= 0:
        return None
    return round(mm / 304.8, 1)


def _criticality(obj: Optional[str]) -> int:
    if not obj:
        return 1
    o = obj.lower()
    for key, w in _CRITICAL.items():
        if key in o:
            return w
    return 2  # a generic obstacle still matters


def _urgency(distance_mm: Optional[float], obj: Optional[str]) -> str:
    """Distance drives urgency; a person/vehicle bumps it up one level."""
    if distance_mm is None:
        return "caution" if _criticality(obj) >= 4 else "clear"
    if distance_mm <= DANGER_MM:
        return "danger"
    if distance_mm <= WARNING_MM:
        return "warning"
    if distance_mm <= SAFE_MM:
        # A close-ish person / vehicle is escalated to a warning.
        return "warning" if _criticality(obj) >= 4 else "caution"
    return "clear"


def _radar_distance(sector: str, radar_state: Dict[str, Any]) -> Optional[float]:
    """The radar's reading for a LiDAR sector, or None where it cannot speak."""
    if sector not in RADAR_SECTORS:
        return None
    sec = (radar_state.get("sectors") or {}).get(sector) or {}
    if not sec.get("online"):
        return None
    dist = sec.get("min_dist")
    if not isinstance(dist, (int, float)) or not dist or dist > AWARE_MM:
        return None
    return float(dist)


def _expected_cardinal_sources() -> Dict[str, tuple]:
    """How many boards each cardinal SHOULD have, from the mount geometry."""
    from config import RADAR_BEAM_H_DEG, RADAR_CARDINAL_CENTRES, _radar_ang_diff
    from utils.radar_calibration import ring_slots

    half = RADAR_BEAM_H_DEG / 2.0 + 22.5
    return {
        name: tuple(b for b in ring_slots() if _radar_ang_diff(b, centre) < half)
        for name, centre in RADAR_CARDINAL_CENTRES.items()
    }


def _lidar_arc_values(cardinal: str, sectors: Dict[str, Any]) -> List[float]:
    """Every LiDAR return across a radar cardinal's ARC, one per sector."""
    vals = []
    for name in RADAR_CARDINAL_ARC_SECTORS.get(cardinal, (cardinal,)):
        v = (sectors.get(name) or {}).get("min_dist")
        # At any range: a wall the laser puts at 3.1 m corroborates a 2.95 m echo; cut at
        # AWARE_MM, the pair read as "glass or clear obstacle" (scan 2026-10-23).
        if isinstance(v, (int, float)) and v > 0:
            vals.append(float(v))
    return vals


def _lidar_arc_corroborates(
    cardinal: str, sectors: Dict[str, Any], radar_mm: float
) -> bool:
    """Did the laser see something at the SAME RANGE as the radar echo?"""
    vals = _lidar_arc_values(cardinal, sectors)
    if not vals:
        return False
    return abs(min(vals) - radar_mm) <= RADAR_AGREE_MM


def fuse_range(
    lidar_mm: Optional[float],
    radar_mm: Optional[float],
    arc_corroborates: Optional[bool] = None,
) -> tuple[Optional[float], str, bool]:
    """Combine the two rangefinders for one direction."""
    if lidar_mm is None and radar_mm is None:
        return None, "none", False
    if radar_mm is None:
        return lidar_mm, "lidar", False

    can_be_glass = radar_mm >= RADAR_START_MM and not arc_corroborates

    if lidar_mm is None:
        return radar_mm, "radar", can_be_glass

    if abs(lidar_mm - radar_mm) <= RADAR_AGREE_MM:
        return (radar_mm if RADAR_PRIORITY else lidar_mm), "both", False

    if radar_mm < lidar_mm:
        return (
            radar_mm,
            "radar",
            (can_be_glass and (lidar_mm - radar_mm) >= RADAR_GLASS_GAP_MM),
        )
    return lidar_mm, "lidar", False


def merge_navigation_inputs(
    lidar_sectors: Optional[Dict[str, Any]],
    lidar_cones: Optional[Dict[str, Any]],
    radar_state: Optional[Dict[str, Any]] = None,
) -> tuple[Dict[str, Any], Dict[str, Any]]:
    """Fold the radar into the inputs the WALKING DECISION actually uses."""
    if radar_state is None:
        radar_state = get_radar_state()

    sectors = {k: dict(v) for k, v in (lidar_sectors or {}).items()}
    cones = {k: dict(v) for k, v in (lidar_cones or {}).items()}

    cones_usable = bool(cones)

    for sector in RADAR_SECTORS:
        radar_mm = _radar_distance(sector, radar_state)
        if radar_mm is None:
            continue

        sec = sectors.setdefault(
            sector, {"min_dist": None, "min_angle": None, "count": 0}
        )
        lidar_mm = sec.get("min_dist")
        lidar_mm = (
            float(lidar_mm) if isinstance(lidar_mm, (int, float)) and lidar_mm else None
        )

        fused, _source, _glass = fuse_range(
            lidar_mm, radar_mm, _lidar_arc_corroborates(sector, sectors, radar_mm)
        )
        if fused is not None and (lidar_mm is None or fused < lidar_mm):
            sec["min_dist"] = fused
            sec["radar_backed"] = True

        trusted = radar_mm <= RADAR_CONE_TRUST_MM if sector == "front" else True
        if cones_usable and trusted:
            cone = cones.setdefault(sector, {"min_dist": None})
            cone_mm = cone.get("min_dist")
            cone_mm = (
                float(cone_mm)
                if isinstance(cone_mm, (int, float)) and cone_mm
                else None
            )
            if cone_mm is None or radar_mm < cone_mm:
                cone["min_dist"] = radar_mm
                cone["radar_backed"] = True

    return sectors, cones


def _top_detection(detections: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Pick the most safety-relevant camera detection in a sector."""
    if not detections:
        return None
    return max(
        detections,
        key=lambda d: (
            _criticality(d.get("class_name")),
            float(d.get("confidence", 0.0)),
        ),
    )


_SHAPE_WORD = {"wall": "wall", "pole": "post"}


def _camera_identity_trusted() -> Tuple[bool, str]:
    """Can the camera's LABELS be believed right now?"""
    if not ENV_TRUST_ENABLED:
        return True, ""
    try:
        from utils.environment_detection import get_environment_state

        env = get_environment_state() or {}
    except Exception:
        return True, ""

    from utils.freshness import is_fresh

    last = env.get("last_update") or 0
    if not is_fresh(last, ENV_STALE_S):
        return True, ""

    vis = float(env.get("visibility") or 1.0)
    if vis >= ENV_IDENTITY_MIN_VISIBILITY:
        return True, ""
    return False, f"{env.get('condition', 'poor visibility')} (visibility {vis:.2f})"


def _bev_shapes() -> List[Dict[str, Any]]:
    """The current BEV clusters, or [] if the grid has nothing to say."""
    if not BEV_SHAPE_IN_HAZARDS:
        return []
    try:
        from utils.bev_grid import get_grid

        snap = get_grid().snapshot()
        if not snap.get("available") or not snap.get("last_update"):
            return []
        return snap.get("objects") or []
    except Exception:
        return []


def _shape_for(
    shapes: List[Dict[str, Any]], angle: Optional[float], dist_mm: Optional[float]
) -> Optional[str]:
    """Which grid cluster, if any, is the thing this sector is reporting."""
    if not shapes or angle is None or dist_mm is None:
        return None

    hits = [
        o
        for o in shapes
        if _angular_gap(o.get("bearing_deg", 999.0), angle) <= BEV_SHAPE_MATCH_DEG
        and abs((o.get("range_mm") or 1e9) - dist_mm) <= BEV_SHAPE_MATCH_MM
    ]
    if len(hits) != 1:
        return None
    return _SHAPE_WORD.get(hits[0].get("kind"))


def _angular_gap(a: float, b: float) -> float:
    """Smallest absolute angular difference between two bearings (degrees)."""
    return abs((a - b + 180.0) % 360.0 - 180.0)


def _phrase(hazard: Dict[str, Any]) -> str:
    """Human phrasing, e.g. \'person, 12 o\'clock, 5 feet\'."""
    obj = hazard["object"]
    parts = [obj, f"{hazard['clock']} o'clock"]
    if hazard.get("distance_ft") is not None:
        parts.append(f"{hazard['distance_ft']:.1f} feet")
    return ", ".join(parts)


def lidar_is_fresh(
    lidar_state: Optional[Dict[str, Any]], now: Optional[float] = None
) -> bool:
    """Is this LiDAR state a current reading?"""
    from utils.freshness import reading_is_current

    if not lidar_state:
        return False
    if lidar_state.get("simulated"):
        return False
    if "last_update" not in lidar_state:
        return bool(
            lidar_state.get("sectors")
            or lidar_state.get("cones")
            or lidar_state.get("points")
        )
    return reading_is_current(lidar_state, LIDAR_STALE_S, now)


def lidar_sparse(lidar_state: Optional[Dict[str, Any]]) -> Optional[str]:
    """Why this scan is too sparse to be a reading (LIDAR_MIN_VALID_FRACTION, NEW-043), or None."""
    frac = (lidar_state or {}).get("valid_fraction")
    if not isinstance(frac, (int, float)) or isinstance(frac, bool):
        return None  # not reported (simulated, older reader)
    if frac >= LIDAR_MIN_VALID_FRACTION:
        return None
    return (
        f"the LiDAR is getting almost no returns ({frac:.0%} of the scan, "
        f"under {LIDAR_MIN_VALID_FRACTION:.0%}) — window blinded, dirty or failing"
    )


def front_glass_coverage(
    radar_state: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Can the radar currently see glass AHEAD? {"ok", "reason", "live", "expected"}"""
    from config import RADAR_ENABLED

    if not RADAR_ENABLED:
        return {
            "ok": False,
            "reason": "radar disabled in config",
            "live": [],
            "expected": [],
            "settling": False,
        }
    if radar_state is None:
        radar_state = get_radar_state()
    try:
        expected = list(_expected_cardinal_sources().get("front") or ())
    except Exception:
        expected = []
    row = ((radar_state or {}).get("sectors") or {}).get("front") or {}
    live = list(row.get("sources_live") or [])
    sources = list(row.get("sources") or [])
    from config import radar_board_key

    owed = {radar_board_key(b) for b in expected}
    dark = sorted(owed - set(live))
    if not row.get("online") or row.get("stale"):
        why = "front radar offline"
    elif row.get("degraded"):
        why = "front radar lagging"
    elif dark or len(live) < max(len(sources), len(expected), 1):
        why = (
            f"front radar half-covered ({len(set(live) & owed) if owed else len(live)} of "
            f"{max(len(sources), len(expected))} boards"
            + (f"; {', '.join(dark)} dark" if dark else "")
            + ")"
        )
    else:
        why = ""
    sectors = (radar_state or {}).get("sectors") or {}
    discovered = bool(
        (radar_state or {}).get("boards") or (radar_state or {}).get("missing")
    )
    settling = bool(why) and (
        not discovered
        or any(
            (sectors.get(k) or {}).get("phase") in ("starting", "calibrating")
            for k in owed
        )
    )
    return {
        "ok": not why,
        "reason": why,
        "live": live,
        "expected": [f"{b:g}" for b in expected],
        "settling": settling,
    }


def front_camera_coverage(
    camera_state: Optional[Dict[str, Any]], now: Optional[float] = None
) -> Dict[str, Any]:
    """Can the front camera name what is ahead right now? {"ok", "reason", "settling"} —
    the definition the voice's "Camera off" notice uses. Judged from the camera state
    alone: fresh front detections (so a lost feed, a stopped detector or pictures all
    skipped as corrupted all count), from a real picture that is not black."""
    from utils.freshness import is_fresh

    now = time.time() if now is None else now
    front = (camera_state or {}).get("front")
    if not isinstance(front, dict) or not front.get("frame_time"):
        return {"ok": False, "reason": "no front camera pictures yet", "settling": True}
    if front.get("simulated"):
        return {
            "ok": False,
            "reason": "the front camera is simulated",
            "settling": False,
        }
    if not is_fresh(front.get("frame_time"), SIMPLE_CAMERA_FRESH_S, now):
        return {
            "ok": False,
            "settling": False,
            "reason": "no new front camera pictures (feed lost, detector stopped or "
            "every picture corrupted)",
        }
    if (front.get("view") or {}).get("dark"):
        return {
            "ok": False,
            "settling": False,
            "reason": "the front camera sees only black (lens covered, unplugged "
            "or no light)",
        }
    return {"ok": True, "reason": "", "settling": False}


_QUAD_W = CAMERA_WIDTH / FRAME_GRID_COLS  # one camera's frame in the mosaic


def _box_bearings(sector: str, x1: float, x2: float) -> Optional[Tuple[float, float]]:
    """The bearings a box's left and right edges could be at, for any field of view in
    DETECTION_DISTANCE_HFOV_DEG (each camera faces its cardinal; image right is
    clockwise for every one)."""
    centre = GUIDANCE_CONE_CENTRES.get(sector)
    if centre is None:
        return None
    edges = [
        centre
        + math.degrees(
            math.atan(
                (x - _QUAD_W / 2.0) * math.tan(math.radians(h) / 2.0) / (_QUAD_W / 2.0)
            )
        )
        for x in (x1, x2)
        for h in DETECTION_DISTANCE_HFOV_DEG
    ]
    return min(edges[:2]) - 1.0, max(edges[2:]) + 1.0


def _laser_ranges_mm(
    lidar_state: Optional[Dict[str, Any]], lo: float, hi: float
) -> List[float]:
    """The distinct things the laser ranges between two bearings: each the nearest
    return of a run of at least LIDAR_CLUSTER_MIN_POINTS returns no more than
    LIDAR_CLUSTER_WINDOW_MM apart in range (a lone reflection is not a thing)."""
    from config import LIDAR_CLUSTER_MIN_POINTS, LIDAR_CLUSTER_WINDOW_MM

    mid, half = (lo + hi) / 2.0, (hi - lo) / 2.0
    pts = (lidar_state or {}).get("points") or {}
    ds = sorted(
        float(d)
        for a, d in zip(pts.get("angles") or [], pts.get("distances") or [])
        if isinstance(d, (int, float)) and d > 0 and _angular_gap(float(a), mid) <= half
    )
    groups: List[List[float]] = []
    for d in ds:
        if groups and d - groups[-1][-1] <= LIDAR_CLUSTER_WINDOW_MM:
            groups[-1].append(d)
        else:
            groups.append([d])
    return [g[0] for g in groups if len(g) >= LIDAR_CLUSTER_MIN_POINTS]


def detection_distance(
    sector: str,
    det: Dict[str, Any],
    lidar_state: Optional[Dict[str, Any]] = None,
    now: Optional[float] = None,
    explain: bool = False,
):
    """How far away one camera detection is, in feet, and how that is known:
    (feet, "lidar" | "camera" | "camera_cut" | ""), or with `explain`, a dict saying why.

    "camera": the box's height against its class's typical height (a person whose box
    is wide for its height is sitting: DETECTION_SEATED_HEIGHT_M) through the assumed
    lens; "camera_cut" when the frame cuts the box off, which makes that an upper bound.
    "lidar": of the things the laser ranges inside the box's bearings, the one nearest
    that estimate, if within DETECTION_DISTANCE_AGREE of it — never simply the nearest
    (a desk in front of someone) or whatever is there (the wall behind someone the
    laser's plane passes over).
    "tof": with TOF_USE_DISTANCE, the ToF's distance of the nearest surface in the box
    (utils/depth_fusion.py), ahead of all of these; without it, that is only shown
    (`tof_ft`) beside them."""
    from config import (
        DETECTION_DISTANCE_AGREE,
        DETECTION_SEATED_ASPECT,
        DETECTION_SEATED_HEIGHT_M,
        SIMPLE_NAMED_OBJECTS,
        TOF_USE_DISTANCE,
    )
    from utils.simple_guidance import _object_height_m, _size_range

    def result(ft, source, **why):
        if not explain:
            return ft, source
        return {
            "sector": sector,
            "class": det.get("class_name"),
            "bbox": det.get("bbox"),
            "distance_ft": ft,
            "source": source,
            **why,
        }

    try:
        x1, y1, x2, y2 = (float(v) for v in det.get("bbox") or ())
    except (TypeError, ValueError):
        return result(None, "", why="no box")
    bearings = _box_bearings(sector, x1, x2)
    if bearings is None or x2 <= x1 or y2 <= y1:
        return result(None, "", why="no box")
    cls = str(det.get("class_name", "")).lower()
    metres, cut = _size_range(cls, (x1, y1, x2, y2))
    seated = (
        SIMPLE_NAMED_OBJECTS.get(cls) == "Person"
        and not cut
        and (x2 - x1) >= DETECTION_SEATED_ASPECT * (y2 - y1)
    )
    if seated:
        metres *= DETECTION_SEATED_HEIGHT_M / _object_height_m(
            cls
        )  # it assumed standing
    est_mm = metres * 1000.0
    estimate = {
        "estimate_ft": round(est_mm / 304.8, 1),
        "estimate_basis": "sitting person" if seated else "class height",
        "cut": cut,
        "bearings_deg": [round(bearings[0], 1), round(bearings[1], 1)],
    }
    depth = det.get("depth")
    if isinstance(depth, dict) and isinstance(depth.get("distance_m"), (int, float)):
        estimate["tof_ft"] = round(float(depth["distance_m"]) / 0.3048, 1)
        if TOF_USE_DISTANCE:
            return result(
                estimate["tof_ft"],
                "tof",
                why="measured by the ToF: the nearest surface in the box",
                tof=depth,
                **estimate,
            )
    if not lidar_is_fresh(lidar_state, now):
        return result(
            estimate["estimate_ft"],
            "camera_cut" if cut else "camera",
            laser_ft=[],
            why="the LiDAR is not current",
            **estimate,
        )
    lo_k, hi_k = DETECTION_DISTANCE_AGREE
    seen = _laser_ranges_mm(lidar_state, *bearings)
    fits = [mm for mm in seen if (cut or lo_k * est_mm <= mm) and mm <= hi_k * est_mm]
    laser_ft = [round(mm / 304.8, 1) for mm in seen]
    if fits:
        best = min(fits, key=lambda mm: abs(math.log(mm / est_mm)))
        # The lens angle this box implies, if the laser is right and it stands at its
        # class height: a few of these from a real walk measure SIMPLE_CAMERA_HFOV_DEG.
        implied = (
            None
            if cut or seated
            else round(
                math.degrees(
                    2.0
                    * math.atan(
                        (_QUAD_W / 2.0)
                        / ((y2 - y1) * best / 1000.0 / _object_height_m(cls))
                    )
                ),
                1,
            )
        )
        return result(
            round(best / 304.8, 1),
            "lidar",
            laser_ft=laser_ft,
            why="the laser range that fits the box's size",
            implied_hfov_deg=implied,
            **estimate,
        )
    return result(
        estimate["estimate_ft"],
        "camera_cut" if cut else "camera",
        laser_ft=laser_ft,
        why=(
            "nothing the laser ranges there fits the box's size"
            if seen
            else "the laser ranges nothing there"
        ),
        **estimate,
    )


def with_distances(
    stats: Dict[str, Any],
    lidar_state: Optional[Dict[str, Any]] = None,
    explain: bool = False,
) -> Dict[str, Any]:
    """A copy of camera stats/state with `distance_ft` and `distance_source` on every
    detection (with `explain`: each detection's whole reasoning, for /camera/distances)."""
    if lidar_state is None:
        lidar_state = get_lidar_state()
    now = time.time()
    out = dict(stats or {})
    for sector in CAMERA_SECTORS:
        entry = out.get(sector)
        if not isinstance(entry, dict):
            continue
        dets = []
        for det in entry.get("detections") or []:
            if explain:
                dets.append(
                    detection_distance(sector, det, lidar_state, now, explain=True)
                )
                continue
            ft, source = detection_distance(sector, det, lidar_state, now)
            dets.append({**det, "distance_ft": ft, "distance_source": source})
        out[sector] = {**entry, "detections": dets}
    return out


def fresh_camera_state(
    camera_state: Optional[Dict[str, Any]], now: Optional[float] = None
) -> Dict[str, Any]:
    """A copy of the camera state with any quadrant's labels removed that were not captured within SIMPLE_CAMERA_FRESH_S."""
    from utils.freshness import is_fresh

    cam = dict(camera_state or {})
    now = time.time() if now is None else now
    shared = cam.get("last_update")
    for name in ("front", "right", "left", "back"):
        sec = cam.get(name)
        if not isinstance(sec, dict):
            continue
        if sec.get("simulated"):
            # Detections from a FABRICATED frame name nothing (NEW-018).
            cam[name] = {**sec, "detections": [], "num_objects": 0, "stale": True}
            continue
        stamp = sec.get("frame_time", shared)
        if stamp is None:
            continue  # no clock supplied
        if not is_fresh(stamp, SIMPLE_CAMERA_FRESH_S, now):
            cam[name] = {**sec, "detections": [], "num_objects": 0, "stale": True}
    return cam


def _sector_of(bearing: float) -> str:
    """The LiDAR sector a bearing (degrees clockwise from ahead) falls in."""
    b = bearing % 360.0
    for name, (lo, hi) in LIDAR_SECTORS.items():
        if (lo <= b < hi) if lo < hi else (b >= lo or b < hi):  # front wraps 0
            return name
    return "front"


def _detections_by_sector(
    camera_state: Dict[str, Any], lidar_state: Dict[str, Any], now: float
) -> Dict[str, List[Dict[str, Any]]]:
    """Every camera detection filed under the LiDAR sector its box centre faces, whichever
    camera saw it, with its bearing and own distance (detection_distance)."""
    out: Dict[str, List[Dict[str, Any]]] = {s: [] for s in SECTORS}
    for cam in CAMERA_SECTORS:
        for det in (camera_state.get(cam) or {}).get("detections") or []:
            try:
                x1, _y1, x2, _y2 = (float(v) for v in det.get("bbox") or ())
            except (TypeError, ValueError):
                continue
            span = _box_bearings(cam, x1, x2)
            if span is None or not x2 > x1:
                continue
            bearing = ((span[0] + span[1]) / 2.0) % 360.0
            ft, source = detection_distance(cam, det, lidar_state, now)
            out[_sector_of(bearing)].append(
                {**det, "bearing_deg": bearing, "distance_ft": ft, "distance_source": source}
            )
    return out


def _same_thing(det: Dict[str, Any], range_mm: float) -> bool:
    """Is this detection what is ranged at `range_mm`? detection_distance's own rule:
    within DETECTION_DISTANCE_AGREE of its distance (a cut-off box's is an upper bound)."""
    ft = det.get("distance_ft")
    if not isinstance(ft, (int, float)) or not ft > 0:
        return False
    est_mm = ft * 304.8
    lo_k, hi_k = DETECTION_DISTANCE_AGREE
    cut = det.get("distance_source") == "camera_cut"
    return (cut or lo_k * est_mm <= range_mm) and range_mm <= hi_k * est_mm


def build_hazards(
    lidar_state: Dict[str, Any],
    camera_state: Dict[str, Any],
    radar_state: Optional[Dict[str, Any]] = None,
) -> List[Dict[str, Any]]:
    """Combine per-sector RANGE (LiDAR + radar, nearest wins) with per-sector camera identity into one ranked hazard list."""
    if radar_state is None:
        radar_state = get_radar_state()

    now = time.time()
    lidar_live = lidar_is_fresh(lidar_state, now)
    camera_state = fresh_camera_state(camera_state, now)

    sectors = (lidar_state.get("sectors", {}) or {}) if lidar_live else {}
    hazards: List[Dict[str, Any]] = []

    shapes = _bev_shapes()
    cam_trusted, cam_untrusted_why = _camera_identity_trusted()

    ranges: Dict[str, tuple] = {}
    for _s in SECTORS:
        _l = (sectors.get(_s) or {}).get("min_dist")
        _l = float(_l) if isinstance(_l, (int, float)) and _l > 0 else None
        _r = _radar_distance(_s, radar_state)
        _arc = _lidar_arc_corroborates(_s, sectors, _r) if _r is not None else None
        # The two compared at their real ranges, THEN cut to what is worth mentioning.
        _d, _src, _glass = fuse_range(_l, _r, _arc)
        if _d is not None and _d > AWARE_MM:
            _d, _src, _glass = None, "none", False
        ranges[_s] = (_d, _src, _glass and lidar_live)

    seen = _detections_by_sector(camera_state, lidar_state, now)

    for sector in SECTORS:
        lidar_sec = sectors.get(sector) or {}
        dist_mm, range_source, glass_suspected = ranges[sector]
        min_angle = lidar_sec.get("min_angle")
        has_lidar = dist_mm is not None  # "has a fused range", LiDAR and/or radar

        # Only a detection in THIS sector, and as far away as the thing ranged here, names
        # it: the top detection of a whole camera called a wall "person, 11 o'clock, 2 ft"
        # (scan 2026-10-23).
        candidates = [
            d for d in seen[sector] if not has_lidar or _same_thing(d, dist_mm)
        ]
        top = _top_detection(candidates) if cam_trusted else None
        obj_name = top.get("class_name") if top else None
        obj_conf = float(top.get("confidence", 0.0)) if top else None
        has_camera = obj_name is not None

        if not has_lidar and not has_camera:
            continue

        contributors = []
        if has_lidar:
            contributors += (
                ["lidar", "radar"] if range_source == "both" else [range_source]
            )
        if has_camera:
            contributors.append("camera")

        if contributors == ["lidar", "camera"]:
            source = "both"
        elif contributors in (["lidar"], ["camera"]):
            source = contributors[0]
        else:
            source = "+".join(contributors)

        use_angle = min_angle is not None and range_source in ("lidar", "both")
        if use_angle:
            clock = angle_to_clock(min_angle)
        elif not has_lidar:
            clock = angle_to_clock(top["bearing_deg"])  # where the camera sees it
        else:
            clock = _SECTOR_CLOCK[sector]

        shape = (
            _shape_for(shapes, min_angle, dist_mm)
            if range_source in ("lidar", "both")
            else None
        )

        if has_camera:
            obj = obj_name
        elif glass_suspected:
            obj = "glass or clear obstacle"
        elif shape:
            obj = shape
        else:
            obj = "obstacle"

        urgency = _urgency(dist_mm if has_lidar else None, obj)

        hazard = {
            "sector": sector,
            "clock": clock,
            "object": obj,
            "confidence": round(obj_conf, 2) if obj_conf is not None else None,
            "distance_mm": int(dist_mm) if has_lidar else None,
            # FEET only — one unit everywhere a person reads a distance.
            "distance_ft": mm_to_ft(dist_mm) if has_lidar else None,
            "urgency": urgency,
            "source": source,
            "range_source": range_source,
            "glass_suspected": glass_suspected,
            "shape": shape,
            "identity_withheld": "" if cam_trusted else cam_untrusted_why,
        }
        hazard["message"] = _phrase(hazard)
        hazards.append(hazard)

    # Rank: most urgent first, then nearest, then most-critical identity.
    hazards.sort(
        key=lambda h: (
            _URGENCY_RANK.get(h["urgency"], 3),
            h["distance_mm"] if h["distance_mm"] is not None else 10**9,
            -_criticality(h["object"]),
        )
    )
    return hazards


def _action_for(urgency: Optional[str]) -> str:
    return {"danger": "stop", "warning": "caution", "caution": "proceed_slow"}.get(
        urgency, "clear"
    )


def fuse_guidance(
    lidar_state: Optional[Dict] = None,
    camera_state: Optional[Dict] = None,
    radar_state: Optional[Dict] = None,
) -> Dict[str, Any]:
    """Produce one prioritized guidance packet from all three sensors."""
    if lidar_state is None:
        lidar_state = get_lidar_state()
    if camera_state is None:
        camera_state = get_camera_state()
    if radar_state is None:
        radar_state = get_radar_state()

    hazards = build_hazards(lidar_state, camera_state, radar_state)
    lidar_live = lidar_is_fresh(lidar_state)

    if not hazards:
        # Nothing found is only "clear" if the LiDAR was looking.
        return {
            "action": "clear" if lidar_live else "unknown",
            "primary": "Path clear" if lidar_live else "LiDAR not reporting",
            "detail": "",
            "hazards": [],
            "count": 0,
            "lidar_stale": not lidar_live,
            "timestamp": time.time(),
        }

    top = hazards[0]
    action = _action_for(top["urgency"])
    verb = {"stop": "Stop.", "caution": "Caution.", "proceed_slow": "Careful."}.get(
        action, ""
    )
    primary = f"{verb} {top['message']}".strip()

    # Detail = the next couple of hazards, so the user has situational context.
    detail = "; ".join(h["message"] for h in hazards[1:3])
    if not lidar_live:
        detail = "; ".join(x for x in ("LiDAR not reporting — radar only", detail) if x)

    return {
        "action": action,
        "primary": primary,
        "detail": detail,
        "hazards": hazards,
        "count": len(hazards),
        "lidar_stale": not lidar_live,
        "timestamp": time.time(),
    }


def describe_scene(
    lidar_state: Optional[Dict] = None,
    camera_state: Optional[Dict] = None,
    radar_state: Optional[Dict] = None,
) -> Dict[str, Any]:
    """On-demand 'what's around me?' — a full readout of every direction, so the user can query their surroundings instead of relying on continuous chatter."""
    if lidar_state is None:
        lidar_state = get_lidar_state()
    if camera_state is None:
        camera_state = get_camera_state()
    if radar_state is None:
        radar_state = get_radar_state()

    hazards = build_hazards(lidar_state, camera_state, radar_state)
    lidar_live = lidar_is_fresh(lidar_state)
    if not hazards:
        spoken = (
            "All directions clear."
            if lidar_live
            else "The LiDAR is not reporting, so I cannot tell what is around you."
        )
    else:
        spoken = ". ".join(h["message"] for h in hazards) + "."
        if not lidar_live:
            spoken = "The LiDAR is not reporting; radar only. " + spoken
    return {
        "spoken": spoken,
        "hazards": hazards,
        "count": len(hazards),
        "lidar_stale": not lidar_live,
        "timestamp": time.time(),
    }


def get_system_health() -> Dict[str, Any]:
    """Sensor liveness, for /health, the startup hold and the dashboard."""
    from utils.freshness import is_fresh

    fused = get_fused_sensor_state()
    active = fused.get("sensors_active", {})

    try:
        from utils import pi_camera_utils

        hailo_ok = bool(getattr(pi_camera_utils, "hailo_ready", False))
        frame_t = float(getattr(pi_camera_utils, "current_frame_t", 0.0) or 0.0)
        camera_simulated = bool(
            getattr(pi_camera_utils, "current_frame_simulated", False)
            or fused.get("camera_simulated")
        )
    except Exception:
        hailo_ok, frame_t, camera_simulated = False, 0.0, False

    sparse = lidar_sparse(fused.get("lidar"))
    lidar_ok = bool(active.get("lidar")) and not sparse
    lidar_simulated = bool(fused.get("lidar_simulated"))
    camera_ok = bool(active.get("camera")) and not camera_simulated
    frames_ok = is_fresh(frame_t, SIMPLE_CAMERA_FRESH_S) and not camera_simulated
    radar_ok = bool(active.get("radar"))

    radar_sectors = (fused.get("radar") or {}).get("sectors") or {}
    radar_down = sorted(
        p for p in RADAR_SECTORS if not (radar_sectors.get(p) or {}).get("online")
    )

    expected = {c: len(v) for c, v in _expected_cardinal_sources().items()}
    radar_partial = sorted(
        p
        for p in RADAR_SECTORS
        if (radar_sectors.get(p) or {}).get("online")
        and len((radar_sectors.get(p) or {}).get("sources_live") or [])
        < max(
            len((radar_sectors.get(p) or {}).get("sources") or []), expected.get(p, 0)
        )
    )

    warnings: List[str] = []
    if lidar_simulated:
        warnings.append("LiDAR is SIMULATED — no real ranging; do not navigate")
    elif sparse and active.get("lidar"):
        warnings.append(
            "LiDAR getting almost no returns (blinded, dirty or failing) — "
            "rely on your cane"
        )
    elif not lidar_ok:
        warnings.append("LiDAR signal lost — rely on your cane")
    if camera_simulated:
        warnings.append(
            "Camera is SIMULATED — no real images; object identification off"
        )
    elif not camera_ok:
        if not frames_ok:
            warnings.append("Camera feed lost — object identification off")
        elif hailo_ok:
            warnings.append("Object detection stalled — object identification off")
        # else: the camera is delivering and the NPU is the cause, said below.
    if not hailo_ok:
        warnings.append(
            "Detection engine not ready"
            + (" — object identification off" if frames_ok else "")
        )
    detection_reduced = ""
    try:
        from utils import pi_camera_utils as _pc

        cover = _pc.detection_coverage()
        if cover.get("reason") == "seg_view_priority":
            detection_reduced = cover["reason"]
            warnings.append(
                "Side and back object detection paused — a "
                "segmentation debug view is open"
            )
        bad = sorted(
            s
            for s, c in (_pc.corruption() or {}).items()
            if c.get("pictures", 0) >= 5 and c.get("share", 0.0) >= 0.2
        )
        if bad:
            warnings.append(
                f"Camera picture corrupted ({', '.join(bad)}) — those frames "
                f"are skipped; the camera board's 4-in-1 merge is faulty"
            )
        views = _pc.camera_views() or {}
        now = time.time()
        looked = {
            c: v for c, v in views.items() if now - float(v.get("at") or 0.0) <= 10.0
        }
        dark = [
            c
            for c in ("front", "right", "left", "back")
            if (looked.get(c) or {}).get("dark")
        ]
        if dark:
            warnings.append(
                f"Camera sees only black ({', '.join(dark)}) — lens covered, "
                f"unplugged or no light; nothing is named "
                f"{'there' if len(dark) == 1 else 'in those directions'}"
            )
        blurred = [
            c
            for c in ("front", "right", "left", "back")
            if (looked.get(c) or {}).get("blurred")
        ]
        if blurred:
            warnings.append(
                f"Camera picture blurred ({', '.join(blurred)}) — lens dirty, "
                f"covered or out of focus; objects may not be named"
            )
        if cover.get("reason") == "cameras_missing":
            detection_reduced = cover["reason"]
            missing = [
                c
                for c in ("front", "right", "left", "back")
                if c not in (cover.get("sectors_detected") or [])
            ]
            warnings.append(
                f"No camera connected: {', '.join(missing)} — no object "
                f"detection {'there' if len(missing) == 1 else 'in those directions'}"
            )
        elif cover.get("reason") == "camera_fallback":
            detection_reduced = cover["reason"]
            warnings.append(
                "The 4-camera picture stayed corrupted — only the front camera "
                "is in use (automatic fallback until the app restarts)"
            )
        elif cover.get("reason") == "single_camera":
            detection_reduced = cover["reason"]
            only = ", ".join(cover.get("sectors_detected") or []) or "one"
            warnings.append(
                f"Only the {only} camera is in use (CAMERA_SINGLE_CHANNEL) — "
                f"no object detection anywhere else"
            )
    except Exception:
        pass
    tof = None
    try:
        from config import TOF_ENABLED

        if TOF_ENABLED:
            from utils.tof_camera import tof_status

            tof = tof_status()
            if not tof.get("ok") or tof.get("age_s") is None or tof["age_s"] > 1.0:
                warnings.append(
                    "Depth camera (ToF) not delivering — distances come from "
                    "box sizes and the LiDAR"
                    + (f" ({tof['error']})" if tof.get("error") else "")
                )
            elif (tof.get("calibration") or {}).get("source") != "bench":
                warnings.append(
                    "Depth camera not calibrated — its dots are placed by the "
                    "design, not measured (tools/tof_calibrate.py)"
                )
    except Exception:
        pass
    lagging = sorted(
        p for p in RADAR_SECTORS if (radar_sectors.get(p) or {}).get("degraded")
    )
    if lagging:
        warnings.append(
            f"Radar delayed {', '.join(lagging)} — distances may be out of date"
        )

    if not radar_ok:
        warnings.append("Radar offline — glass may not be detected")
    elif radar_down:
        warnings.append(
            f"Radar down {', '.join(radar_down)} — glass may not be "
            f"detected {'there' if len(radar_down) == 1 else 'in those directions'}"
        )

    if radar_partial:
        warnings.append(
            f"Radar half-covered {', '.join(radar_partial)} — glass may "
            f"be missed to one side"
        )

    # The same definition the voice's "Glass sensor off" notice uses.
    front_glass = front_glass_coverage(fused.get("radar"))

    all_ok = (
        lidar_ok
        and camera_ok
        and hailo_ok
        and radar_ok
        and not radar_down
        and not radar_partial
        and not lagging
        and not warnings
    )
    return {
        "all_ok": all_ok,
        "radar_partial": radar_partial,
        "lidar_ok": lidar_ok,
        "lidar_simulated": lidar_simulated,
        "detection_reduced": detection_reduced,
        "front_glass_ok": front_glass["ok"],
        "front_glass_reason": front_glass["reason"],
        "camera_ok": camera_ok,
        "camera_simulated": camera_simulated,
        "radar_ok": radar_ok,
        "radar_down": radar_down,
        "radar_lagging": lagging,
        "hailo_ok": hailo_ok,
        "tof": tof,
        "warnings": warnings,
        "spoken": "All systems normal" if all_ok else ". ".join(warnings),
        "timestamp": time.time(),
    }
