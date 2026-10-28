"""Navigation API routes"""

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from starlette.concurrency import run_in_threadpool
from pydantic import BaseModel
from typing import Dict, Any, List, Optional

from utils.unified_navigation_system import (
    decisionMaker,
    detect_walls_by_sector,
    valid_scan_points,
)
from utils.shared_state import (
    get_fused_sensor_state,
    get_navigation_state,
    update_navigation_state,
    get_lidar_state,
    get_camera_state,
)
from config import (
    DANGER_THRESHOLD_MM,
    SAFE_THRESHOLD_MM,
    WALL_WARNING_DISTANCE,
    CRITICAL_OBJECTS,
)

router = APIRouter(prefix="/navigation", tags=["Navigation"])


class NavigationRequest(BaseModel):
    """Request model for navigation decision"""

    scan_data: Optional[List[Dict[str, Any]]] = []
    front: Optional[List[Dict[str, Any]]] = []
    right: Optional[List[Dict[str, Any]]] = []
    left: Optional[List[Dict[str, Any]]] = []
    back: Optional[List[Dict[str, Any]]] = []
    context: Optional[Dict[str, Any]] = {}


@router.post("/decide")
async def make_navigation_decision(request: NavigationRequest):
    """Make a navigation decision based on sensor data"""
    sample = {
        "scan_data": request.scan_data or [],
        "front": request.front or [],
        "right": request.right or [],
        "left": request.left or [],
        "back": request.back or [],
        "context": request.context or {},
    }

    decision = await run_in_threadpool(decisionMaker, sample)
    if not _has_scan(sample):
        decision = _no_lidar_decision(decision, "no usable scan in the request")
    update_navigation_state(decision)

    return decision


def _has_scan(sample: Any) -> bool:
    """A nonempty collection of rejected measurements is still no scan."""
    if not isinstance(sample, dict):
        return False
    if next(valid_scan_points(sample.get("scan_data")), None) is not None:
        return True
    sectors = sample.get("precomputed_sectors")
    if not isinstance(sectors, dict):
        return False
    measurements = [
        {"angle_deg": 0, "distance_mm": sec.get("min_dist")}
        for sec in sectors.values()
        if isinstance(sec, dict)
    ]
    return next(valid_scan_points(measurements), None) is not None


def _lidar_unavailable(lidar: Dict[str, Any]) -> str:
    """Why this LiDAR state is unusable, or "" for a current, real scan of sufficient quality."""
    from utils.freshness import describe
    from utils.sensor_fusion import lidar_is_fresh, lidar_sparse

    if lidar.get("simulated"):
        return "the scan is simulated"
    sparse = lidar_sparse(lidar)
    if sparse:
        return sparse
    if lidar_is_fresh(lidar):
        return ""
    stamp = lidar.get("last_update")
    if not stamp:  # 0 = none since start-up
        return "no scan ever reported"
    return f"last scan {describe(stamp)}"


def _no_lidar_decision(decision: Dict[str, Any], reason: str) -> Dict[str, Any]:
    """`decision` with everything the LiDAR would have vouched for withdrawn."""
    msg = f"No LiDAR data ({reason}) — the way ahead is unknown."
    lights = dict(decision.get("traffic_lights") or {})
    d = dict(decision)
    d.update(
        action="no_lidar_data",
        urgent_message=msg,
        primary_direction=None,
        speed={"speed": "stop", "multiplier": 0.0},
        dangerous_sectors=[],
        warning_sectors=[],
        safe_sectors=[],
        sector_distances={
            s: {"distance_mm": None, "distance_ft": None, "count": 0, "is_safe": False}
            for s in (decision.get("sector_distances") or {})
        },
        cone_distances={
            c: {"distance_mm": None, "distance_ft": None}
            for c in (decision.get("cone_distances") or {})
        },
        wall_detection={
            s: {"wall_detected": False, "distance": None, "warning": False}
            for s in (decision.get("wall_detection") or {})
        },
        path={"direction": None, "clear_distance_ft": None, "recommended": msg},
        cues={"primary": "No LiDAR data", "detail": msg, "warning": msg},
        traffic_lights=lights,
        sensor_health={
            **(decision.get("sensor_health") or {}),
            "lidar_active": False,
            "lidar_points": 0,
            "lidar_fresh": False,
            "lidar_reason": reason,
        },
    )
    return d


@router.get("/decide")
async def make_navigation_decision_from_state():
    """Make a navigation decision using current sensor state."""
    lidar = get_lidar_state()
    from utils.sensor_fusion import fresh_camera_state

    camera = fresh_camera_state(get_camera_state())

    sample = {
        "precomputed_sectors": lidar.get("sectors", {}),
        "precomputed_cones": lidar.get("cones", {}),
        "front": camera.get("front", {}).get("detections", []),
        "right": camera.get("right", {}).get("detections", []),
        "left": camera.get("left", {}).get("detections", []),
        "back": camera.get("back", {}).get("detections", []),
        "context": {},
    }

    decision = await run_in_threadpool(decisionMaker, sample)
    unavailable = _lidar_unavailable(lidar)
    if unavailable:
        decision = _no_lidar_decision(decision, unavailable)
    else:
        decision["sensor_health"] = {
            **(decision.get("sensor_health") or {}),
            "lidar_fresh": True,
            "lidar_reason": "",
        }
    update_navigation_state(decision)

    return decision


@router.get("/status")
async def get_navigation_status():
    """Get current navigation status"""
    nav = get_navigation_state()

    return {
        "current_action": nav.get("current_action", "idle"),
        "message": nav.get("message", ""),
        "dangerous_sectors": nav.get("dangerous_sectors", []),
        "warning_sectors": nav.get("warning_sectors", []),
        "safe_sectors": nav.get("safe_sectors", []),
        "wall_detection": nav.get("wall_detection", {}),
        "path": nav.get("path", {}),
        "landmarks": nav.get("landmarks", []),
        "last_update": nav.get("last_decision_time", 0),
    }


@router.get("/sensors")
async def get_sensor_status():
    """Get fused sensor state"""
    return get_fused_sensor_state()


# Plain `def` (threadpool): wall detection walks every point of the scan.
@router.post("/walls")
def detect_walls(scan_data: List[Dict[str, Any]]):
    """Detect walls from LiDAR scan data"""
    return detect_walls_by_sector(scan_data)


@router.get("/walls")
def detect_walls_from_state():
    """Detect walls using current LiDAR state"""
    lidar = get_lidar_state()
    points = lidar.get("points", {})

    scan_data = []
    if isinstance(points, dict):
        angles = points.get("angles", [])
        distances = points.get("distances", [])
        for angle, dist in zip(angles, distances):
            scan_data.append({"angle_deg": angle, "distance_mm": dist})

    return detect_walls_by_sector(scan_data)


@router.get("/path")
async def get_current_path():
    """Get current recommended path"""
    nav = get_navigation_state()

    return {
        "path": nav.get("path", {}),
        "action": nav.get("current_action", "idle"),
        "message": nav.get("message", ""),
        "landmarks": nav.get("landmarks", []),
    }


@router.get("/config")
async def get_navigation_config():
    """Get navigation configuration parameters"""
    return {
        "danger_threshold_mm": DANGER_THRESHOLD_MM,
        "safe_threshold_mm": SAFE_THRESHOLD_MM,
        "wall_warning_distance_mm": WALL_WARNING_DISTANCE,
        "critical_objects": CRITICAL_OBJECTS,
    }


@router.websocket("/ws")
async def navigation_websocket(websocket: WebSocket):
    """WebSocket endpoint for real-time navigation updates"""
    await websocket.accept()

    try:
        while True:
            data = await websocket.receive_json()

            decision = await run_in_threadpool(decisionMaker, data)
            if not _has_scan(data):
                decision = _no_lidar_decision(decision, "no usable scan in the message")
            update_navigation_state(decision)

            await websocket.send_json(decision)

    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"[ERROR] Navigation WebSocket error: {e}")
