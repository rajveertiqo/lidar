"""Sensor fusion for collaborative blind-navigation guidance."""

import time
from typing import Any, Dict, List, Optional, Tuple

from config import (
    BEV_SHAPE_IN_HAZARDS, BEV_SHAPE_MATCH_DEG, BEV_SHAPE_MATCH_MM,
    ENV_IDENTITY_MIN_VISIBILITY, ENV_STALE_S, ENV_TRUST_ENABLED,
    GUIDANCE_DANGER_MM, GUIDANCE_WARNING_MM, GUIDANCE_SAFE_MM, GUIDANCE_AWARE_MM,
    LIDAR_SECTOR_ORDER, LIDAR_STALE_S, LIDAR_TO_CAMERA, SIMPLE_CAMERA_FRESH_S,
    RADAR_AGREE_MM, RADAR_CARDINAL_ARC_SECTORS, RADAR_CONE_TRUST_MM,
    RADAR_GLASS_GAP_MM, RADAR_PRIORITY,
    RADAR_SECTORS, RADAR_START_MM,
)
from utils.shared_state import (
    get_lidar_state, get_radar_state, get_camera_state, get_fused_sensor_state,
)

# Distance thresholds (mm) — single source of truth in config.py.
DANGER_MM = GUIDANCE_DANGER_MM     # imminent — stop
WARNING_MM = GUIDANCE_WARNING_MM   # close — caution
SAFE_MM = GUIDANCE_SAFE_MM         # be aware
AWARE_MM = GUIDANCE_AWARE_MM       # furthest distance we bother mentioning

# 8 x 45° LiDAR sectors (clockwise from straight ahead).
SECTORS = list(LIDAR_SECTOR_ORDER)

# Object identity → how much it raises urgency (moving things matter most).
_CRITICAL = {
    "person": 5, "pedestrian": 5,
    "car": 4, "truck": 4, "bus": 4, "motorcycle": 4, "bicycle": 4,
    "scooter": 4, "train": 4,
    "dog": 3, "cat": 3,
}

_SECTOR_CLOCK = {
    "front": 12, "front_right": 2, "right": 3, "back_right": 5,
    "back": 6, "back_left": 8, "left": 9, "front_left": 11,
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
    return {name: tuple(b for b in ring_slots() if _radar_ang_diff(b, centre) < half)
            for name, centre in RADAR_CARDINAL_CENTRES.items()}


def _lidar_arc_values(cardinal: str, sectors: Dict[str, Any]) -> List[float]:
    """Every LiDAR return across a radar cardinal's ARC, one per sector."""
    vals = []
    for name in RADAR_CARDINAL_ARC_SECTORS.get(cardinal, (cardinal,)):
        v = (sectors.get(name) or {}).get("min_dist")
        if isinstance(v, (int, float)) and v and v <= AWARE_MM:
            vals.append(float(v))
    return vals


def _lidar_arc_corroborates(cardinal: str, sectors: Dict[str, Any],
                            radar_mm: float) -> bool:
    """Did the laser see something at the SAME RANGE as the radar echo?"""
    vals = _lidar_arc_values(cardinal, sectors)
    if not vals:
        return False
    return abs(min(vals) - radar_mm) <= RADAR_AGREE_MM


def fuse_range(lidar_mm: Optional[float],
               radar_mm: Optional[float],
               arc_corroborates: Optional[bool] = None) -> tuple[Optional[float], str, bool]:
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
        return radar_mm, "radar", (can_be_glass
                                   and (lidar_mm - radar_mm) >= RADAR_GLASS_GAP_MM)
    return lidar_mm, "lidar", False


def merge_navigation_inputs(lidar_sectors: Optional[Dict[str, Any]],
                            lidar_cones: Optional[Dict[str, Any]],
                            radar_state: Optional[Dict[str, Any]] = None
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

        sec = sectors.setdefault(sector, {"min_dist": None, "min_angle": None, "count": 0})
        lidar_mm = sec.get("min_dist")
        lidar_mm = float(lidar_mm) if isinstance(lidar_mm, (int, float)) and lidar_mm else None

        fused, _source, _glass = fuse_range(
            lidar_mm, radar_mm,
            _lidar_arc_corroborates(sector, sectors, radar_mm))
        if fused is not None and (lidar_mm is None or fused < lidar_mm):
            sec["min_dist"] = fused
            sec["radar_backed"] = True

        trusted = radar_mm <= RADAR_CONE_TRUST_MM if sector == "front" else True
        if cones_usable and trusted:
            cone = cones.setdefault(sector, {"min_dist": None})
            cone_mm = cone.get("min_dist")
            cone_mm = float(cone_mm) if isinstance(cone_mm, (int, float)) and cone_mm else None
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
        key=lambda d: (_criticality(d.get("class_name")), float(d.get("confidence", 0.0))),
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


def _shape_for(shapes: List[Dict[str, Any]], angle: Optional[float],
               dist_mm: Optional[float]) -> Optional[str]:
    """Which grid cluster, if any, is the thing this sector is reporting."""
    if not shapes or angle is None or dist_mm is None:
        return None

    hits = [
        o for o in shapes
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


def lidar_is_fresh(lidar_state: Optional[Dict[str, Any]],
                   now: Optional[float] = None) -> bool:
    """Is this LiDAR state a current reading?"""
    from utils.freshness import reading_is_current
    if not lidar_state:
        return False
    if lidar_state.get("simulated"):
        return False
    if "last_update" not in lidar_state:
        return bool(lidar_state.get("sectors") or lidar_state.get("cones")
                    or lidar_state.get("points"))
    return reading_is_current(lidar_state, LIDAR_STALE_S, now)


def front_glass_coverage(radar_state: Optional[Dict[str, Any]] = None
                         ) -> Dict[str, Any]:
    """Can the radar currently see glass AHEAD? {"ok", "reason", "live", "expected"}"""
    from config import RADAR_ENABLED
    if not RADAR_ENABLED:
        return {"ok": False, "reason": "radar disabled in config",
                "live": [], "expected": [], "settling": False}
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
        why = (f"front radar half-covered ({len(set(live) & owed) if owed else len(live)} of "
               f"{max(len(sources), len(expected))} boards"
               + (f"; {', '.join(dark)} dark" if dark else "") + ")")
    else:
        why = ""
    sectors = (radar_state or {}).get("sectors") or {}
    discovered = bool((radar_state or {}).get("boards") or (radar_state or {}).get("missing"))
    settling = bool(why) and (not discovered or any(
        (sectors.get(k) or {}).get("phase") in ("starting", "calibrating") for k in owed))
    return {"ok": not why, "reason": why, "live": live,
            "expected": [f"{b:g}" for b in expected], "settling": settling}


def fresh_camera_state(camera_state: Optional[Dict[str, Any]],
                       now: Optional[float] = None) -> Dict[str, Any]:
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
            continue                          # no clock supplied
        if not is_fresh(stamp, SIMPLE_CAMERA_FRESH_S, now):
            cam[name] = {**sec, "detections": [], "num_objects": 0, "stale": True}
    return cam


def build_hazards(lidar_state: Dict[str, Any], camera_state: Dict[str, Any],
                  radar_state: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
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
        _l = float(_l) if isinstance(_l, (int, float)) and _l and _l <= AWARE_MM else None
        _r = _radar_distance(_s, radar_state)
        _arc = _lidar_arc_corroborates(_s, sectors, _r) if _r is not None else None
        _d, _src, _glass = fuse_range(_l, _r, _arc)
        ranges[_s] = (_d, _src, _glass and lidar_live)

    lidar_backed_cams = set()
    for _s in SECTORS:
        if ranges[_s][0] is not None:
            lidar_backed_cams.add(LIDAR_TO_CAMERA.get(_s, _s))

    for sector in SECTORS:
        lidar_sec = sectors.get(sector) or {}
        dist_mm, range_source, glass_suspected = ranges[sector]
        min_angle = lidar_sec.get("min_angle")

        cam_sec = camera_state.get(LIDAR_TO_CAMERA.get(sector, sector)) or {}
        top = _top_detection(cam_sec.get("detections", []))
        obj_name = top.get("class_name") if top else None
        obj_conf = float(top.get("confidence", 0.0)) if top else None

        has_lidar = dist_mm is not None       # "has a fused range", LiDAR and/or radar
        if obj_name is not None and not cam_trusted:
            obj_name, obj_conf = None, None
        has_camera = obj_name is not None

        if not has_lidar and not has_camera:
            continue

        if not has_lidar:
            cam_name = LIDAR_TO_CAMERA.get(sector, sector)
            if cam_name != sector or cam_name in lidar_backed_cams:
                continue

        contributors = []
        if has_lidar:
            contributors += ["lidar", "radar"] if range_source == "both" else [range_source]
        if has_camera:
            contributors.append("camera")

        if contributors == ["lidar", "camera"]:
            source = "both"
        elif contributors in (["lidar"], ["camera"]):
            source = contributors[0]
        else:
            source = "+".join(contributors)

        use_angle = min_angle is not None and range_source in ("lidar", "both")
        clock = angle_to_clock(min_angle) if use_angle else _SECTOR_CLOCK[sector]

        shape = _shape_for(shapes, min_angle, dist_mm) if range_source in ("lidar", "both") else None

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
    hazards.sort(key=lambda h: (
        _URGENCY_RANK.get(h["urgency"], 3),
        h["distance_mm"] if h["distance_mm"] is not None else 10 ** 9,
        -_criticality(h["object"]),
    ))
    return hazards


def _action_for(urgency: Optional[str]) -> str:
    return {"danger": "stop", "warning": "caution", "caution": "proceed_slow"}.get(urgency, "clear")


def fuse_guidance(lidar_state: Optional[Dict] = None,
                  camera_state: Optional[Dict] = None,
                  radar_state: Optional[Dict] = None) -> Dict[str, Any]:
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
    verb = {"stop": "Stop.", "caution": "Caution.", "proceed_slow": "Careful."}.get(action, "")
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


def describe_scene(lidar_state: Optional[Dict] = None,
                   camera_state: Optional[Dict] = None,
                   radar_state: Optional[Dict] = None) -> Dict[str, Any]:
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
        spoken = ("All directions clear." if lidar_live else
                  "The LiDAR is not reporting, so I cannot tell what is around you.")
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
        camera_simulated = bool(getattr(pi_camera_utils, "current_frame_simulated", False)
                                or fused.get("camera_simulated"))
    except Exception:
        hailo_ok, frame_t, camera_simulated = False, 0.0, False

    lidar_ok = bool(active.get("lidar"))
    lidar_simulated = bool(fused.get("lidar_simulated"))
    camera_ok = bool(active.get("camera")) and not camera_simulated
    frames_ok = is_fresh(frame_t, SIMPLE_CAMERA_FRESH_S) and not camera_simulated
    radar_ok = bool(active.get("radar"))

    radar_sectors = (fused.get("radar") or {}).get("sectors") or {}
    radar_down = sorted(p for p in RADAR_SECTORS
                        if not (radar_sectors.get(p) or {}).get("online"))

    expected = {c: len(v) for c, v in _expected_cardinal_sources().items()}
    radar_partial = sorted(
        p for p in RADAR_SECTORS
        if (radar_sectors.get(p) or {}).get("online")
        and len((radar_sectors.get(p) or {}).get("sources_live") or [])
            < max(len((radar_sectors.get(p) or {}).get("sources") or []),
                  expected.get(p, 0)))

    warnings: List[str] = []
    if lidar_simulated:
        warnings.append("LiDAR is SIMULATED — no real ranging; do not navigate")
    elif not lidar_ok:
        warnings.append("LiDAR signal lost — rely on your cane")
    if camera_simulated:
        warnings.append("Camera is SIMULATED — no real images; object identification off")
    elif not camera_ok:
        if not frames_ok:
            warnings.append("Camera feed lost — object identification off")
        elif hailo_ok:
            warnings.append("Object detection stalled — object identification off")
        # else: the camera is delivering and the NPU is the cause, said below.
    if not hailo_ok:
        warnings.append("Detection engine not ready"
                        + (" — object identification off" if frames_ok else ""))
    detection_reduced = ""
    try:
        from utils import pi_camera_utils as _pc
        cover = _pc.detection_coverage()
        if cover.get("reason") == "seg_view_priority":
            detection_reduced = cover["reason"]
            warnings.append("Side and back object detection paused — a "
                            "segmentation debug view is open")
    except Exception:
        pass
    lagging = sorted(p for p in RADAR_SECTORS
                     if (radar_sectors.get(p) or {}).get("degraded"))
    if lagging:
        warnings.append(f"Radar delayed {', '.join(lagging)} — distances may be out of date")

    if not radar_ok:
        warnings.append("Radar offline — glass may not be detected")
    elif radar_down:
        warnings.append(f"Radar down {', '.join(radar_down)} — glass may not be "
                        f"detected {'there' if len(radar_down) == 1 else 'in those directions'}")

    if radar_partial:
        warnings.append(f"Radar half-covered {', '.join(radar_partial)} — glass may "
                        f"be missed to one side")

    # The same definition the voice's "Glass sensor off" notice uses.
    front_glass = front_glass_coverage(fused.get("radar"))

    all_ok = (lidar_ok and camera_ok and hailo_ok and radar_ok
              and not radar_down and not radar_partial and not lagging
              and not warnings)
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
        "warnings": warnings,
        "spoken": "All systems normal" if all_ok else ". ".join(warnings),
        "timestamp": time.time(),
    }
