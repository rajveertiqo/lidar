"""Shared state management for sensor fusion and navigation"""

import threading
from typing import Dict, Any, List, Optional
import time

# ============== THREAD LOCKS ==============
lidar_lock = threading.Lock()
radar_lock = threading.Lock()
camera_lock = threading.Lock()
navigation_lock = threading.Lock()
path_guidance_lock = threading.Lock()


def _default_lidar_state():
    return {
        "points": {
            "angles": [],
            "distances": []
        },
        "sectors": {},      # 8 x 45° — threat detection
        "cones": {},        # narrow per-cardinal cones — steering guidance
        "rpm": 0,
        "last_update": 0,
        "scan_index": 0,
        "simulated": False,
    }


def _default_radar_state():
    return {
        "sectors": {},      # position -> {min_dist, strength_db, peaks_mm, online, last_update}
        "boards": [],       # discovery result: device / serial / hub_port / position / assumed
        "missing": [],      # configured boards that did not answer discovery
        "last_update": 0,
    }


def _default_camera_state():
    return {
        "front": {"detections": [], "num_objects": 0},
        "right": {"detections": [], "num_objects": 0},
        "left": {"detections": [], "num_objects": 0},
        "back": {"detections": [], "num_objects": 0},
        "total_objects": 0,
        "inference_time": 0,
        "last_update": 0,
    }


def _default_navigation_state():
    return {
        "current_action": "idle",
        "message": "",
        "dangerous_sectors": [],
        "safe_sectors": [],
        "warning_sectors": [],
        "path": None,
        "landmarks": [],
        "wall_detection": {},
        "last_decision_time": 0,
    }


def _default_drift_state():
    return {
        "action": "hold_course", "level": "clear", "instruction": "",
        "ttc_s": None, "last_update": 0,
    }


lidar_state = _default_lidar_state()
radar_state = _default_radar_state()
camera_state = _default_camera_state()
navigation_state = _default_navigation_state()


# ============== STATE UPDATE FUNCTIONS ==============

def update_lidar_state(points: Dict, sectors: Dict, rpm: float, scan_index: int,
                       cones: Dict = None, captured_at: Optional[float] = None,
                       simulated: bool = False, valid_fraction: Optional[float] = None):
    """Thread-safe update of LiDAR state. `valid_fraction`: valid returns / packets swept (NEW-043), None when not known."""
    from utils.freshness import RECEIVED_KEY, received_mono
    stamp = time.time() if captured_at is None else captured_at
    with lidar_lock:
        lidar_state["points"] = points
        lidar_state["sectors"] = sectors
        if cones is not None:
            lidar_state["cones"] = cones
        lidar_state["rpm"] = rpm
        lidar_state["scan_index"] = scan_index
        lidar_state["last_update"] = stamp
        lidar_state[RECEIVED_KEY] = received_mono(stamp)
        lidar_state["simulated"] = bool(simulated)
        lidar_state["valid_fraction"] = valid_fraction


def get_lidar_state() -> Dict[str, Any]:
    """Thread-safe retrieval of LiDAR state"""
    with lidar_lock:
        return lidar_state.copy()


def update_radar_sector(position: str, data: Dict[str, Any]):
    """Thread-safe update of ONE radar direction (called by that board's thread)."""
    from utils.freshness import RECEIVED_KEY, received_mono
    with radar_lock:
        entry = dict(data)
        if entry.get("last_update") is None:
            entry["last_update"] = time.time()
        if entry.get(RECEIVED_KEY) is None:
            entry[RECEIVED_KEY] = received_mono(entry["last_update"])
        radar_state["sectors"][position] = entry
        radar_state["last_update"] = max(radar_state.get("last_update") or 0.0,
                                         entry["last_update"])


def set_radar_boards(boards: List[Dict[str, Any]], missing: Optional[List[Dict[str, Any]]] = None):
    """Record the discovery result for diagnostics."""
    with radar_lock:
        radar_state["boards"] = list(boards)
        if missing is not None:
            radar_state["missing"] = list(missing)


def get_radar_state() -> Dict[str, Any]:
    """Thread-safe retrieval of radar state."""
    from config import RADAR_STALE_S
    from utils.freshness import reading_is_current

    with radar_lock:
        now = time.time()
        sectors = {}
        for pos, sec in radar_state["sectors"].items():
            sec = dict(sec)
            if not reading_is_current(sec, RADAR_STALE_S, now):
                sec["online"] = False
                sec["min_dist"] = None
                sec["stale"] = True
            else:
                sec["stale"] = False
            sectors[pos] = sec
        return {
            "sectors": sectors,
            "boards": list(radar_state["boards"]),
            "missing": list(radar_state.get("missing", [])),
            "last_update": radar_state["last_update"],
        }


def update_camera_state(detections: Dict[str, Any]):
    """Thread-safe update of camera detection state"""
    with camera_lock:
        for key in ["front", "right", "left", "back"]:
            if key in detections:
                camera_state[key] = detections[key]
        if "total_objects" in detections:
            camera_state["total_objects"] = detections["total_objects"]
        if "inference_time" in detections:
            camera_state["inference_time"] = detections["inference_time"]
        camera_state["last_update"] = time.time()


def get_camera_state() -> Dict[str, Any]:
    """Thread-safe retrieval of camera state"""
    with camera_lock:
        return camera_state.copy()


drift_state: Dict[str, Any] = _default_drift_state()


def update_drift_state(result: Dict[str, Any]):
    """Thread-safe update of the latest drift assessment."""
    global drift_state
    with path_guidance_lock:
        entry = dict(result)
        entry["last_update"] = time.time()
        drift_state = entry


def get_drift_state() -> Dict[str, Any]:
    """Thread-safe retrieval of the latest drift assessment."""
    from config import DRIFT_LOOKAHEAD_S
    from utils.freshness import is_fresh

    with path_guidance_lock:
        state = dict(drift_state)
    now = time.time()
    age = now - (state.get("last_update") or 0)
    state["age_s"] = round(age, 2)
    if not is_fresh(state.get("last_update"), max(1.0, DRIFT_LOOKAHEAD_S / 6.0), now):
        state["stale"] = True
        state["speak"] = False
    else:
        state["stale"] = False
    return state


def update_navigation_state(decision: Dict[str, Any]):
    """Thread-safe update of navigation state"""
    with navigation_lock:
        navigation_state["current_action"] = decision.get("action", "idle")
        navigation_state["message"] = decision.get("urgent_message", "")
        navigation_state["dangerous_sectors"] = decision.get("dangerous_sectors", [])
        navigation_state["safe_sectors"] = decision.get("safe_sectors", [])
        navigation_state["warning_sectors"] = decision.get("warning_sectors", [])
        navigation_state["path"] = decision.get("path")
        navigation_state["landmarks"] = decision.get("landmarks", [])
        navigation_state["wall_detection"] = decision.get("wall_detection", {})
        navigation_state["last_decision_time"] = time.time()


def get_navigation_state() -> Dict[str, Any]:
    """Thread-safe retrieval of navigation state"""
    with navigation_lock:
        return navigation_state.copy()


# ============== FUSED STATE ==============

def get_fused_sensor_state() -> Dict[str, Any]:
    """Get combined state from all sensors"""
    lidar = get_lidar_state()
    radar = get_radar_state()
    camera = get_camera_state()
    nav = get_navigation_state()

    radar_active = any(s.get("online") for s in radar["sectors"].values())

    from config import LIDAR_STALE_S, SIMPLE_CAMERA_FRESH_S
    from utils.freshness import is_fresh, reading_is_current
    now = time.time()

    return {
        "lidar": lidar,
        "radar": radar,
        "camera": camera,
        "navigation": nav,
        "timestamp": now,
        "sensors_active": {
            "lidar": (reading_is_current(lidar, LIDAR_STALE_S, now)
                      and not lidar.get("simulated")),
            "radar": radar_active,
            "camera": (is_fresh((camera.get("front") or {}).get(
                           "frame_time", camera.get("last_update")),
                           SIMPLE_CAMERA_FRESH_S, now)
                       and not (camera.get("front") or {}).get("simulated")),
        },
        "lidar_simulated": bool(lidar.get("simulated")),
        "camera_simulated": bool((camera.get("front") or {}).get("simulated")),
    }


# ============== UTILITY FUNCTIONS ==============

def reset_all_states():
    """Reset every shared store to its default."""
    global lidar_state, radar_state, camera_state, navigation_state
    global drift_state

    with lidar_lock:
        lidar_state = _default_lidar_state()

    with radar_lock:
        radar_state = _default_radar_state()

    with camera_lock:
        camera_state = _default_camera_state()

    with navigation_lock:
        navigation_state = _default_navigation_state()

    with path_guidance_lock:
        drift_state = _default_drift_state()
