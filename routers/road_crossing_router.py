"""Road Crossing API Routes Provides endpoints for traffic light detection and crossing assistance"""

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from pydantic import BaseModel
import asyncio
import json
import logging

from utils.traffic_light_detection import get_traffic_detector
from utils.shared_state import get_camera_state, get_lidar_state

router = APIRouter(prefix="/crossing", tags=["Road Crossing"])


class CrossingActionRequest(BaseModel):
    """Request model for crossing actions"""
    action: str  # "start_crossing", "abort", "crossing_complete", "reset"


@router.get("/status")
async def get_crossing_status():
    """Get current traffic light and crossing status"""
    detector = get_traffic_detector()
    
    camera_data = get_camera_state()
    lidar_data = get_lidar_state()
    
    if logging.getLogger(__name__).isEnabledFor(logging.DEBUG):
        for cam in ["front", "right", "left", "back"]:
            cam_info = camera_data.get(cam, {})
            dets = cam_info.get("detections", []) if isinstance(cam_info, dict) else []
            if dets:
                logging.getLogger(__name__).debug(
                    "[CROSSING] %s camera detections: %s",
                    cam, [d.get("class_name") for d in dets]
                )

    detector.process_camera_detections(camera_data)
    
    if lidar_data.get("sectors"):
        detector.process_lidar_for_vehicles(lidar_data["sectors"])
    
    detector.update_crossing_state()
    
    return detector.get_full_status()


@router.get("/safety")
async def get_crossing_safety():
    """Quick safety check - is it safe to cross?"""
    detector = get_traffic_detector()
    
    camera_data = get_camera_state()
    lidar_data = get_lidar_state()
    
    detector.process_camera_detections(camera_data)
    if lidar_data.get("sectors"):
        detector.process_lidar_for_vehicles(lidar_data["sectors"])
    
    safety = detector.get_crossing_safety()
    
    return {
        "is_safe": safety["is_safe"],
        "action": safety["action"],
        "message": safety["message"],
        "safety_level": safety["safety_level"],
        "front_light": safety["front_light"]
    }


@router.post("/action")
async def crossing_action(request: CrossingActionRequest):
    """Perform crossing action"""
    detector = get_traffic_detector()
    
    if request.action == "reset":
        detector.reset()
        return {"status": "reset", "message": "Crossing state reset"}
    
    new_state = detector.update_crossing_state(request.action)
    
    safety = detector.get_crossing_safety()
    
    response = {
        "status": "ok",
        "action": request.action,
        "new_state": new_state.value,
        "safety": safety,
        "guidance": detector.crossing_message()
    }
    
    if request.action == "start_crossing":
        if not safety["is_safe"]:
            response["status"] = "denied"
            response["message"] = "Cannot start crossing - not safe!"
            response["guidance"]["primary"] = "STOP"
            response["guidance"]["detail"] = safety["message"]
    
    return response


@router.get("/lights")
async def get_traffic_lights():
    """Get traffic light states for all directions"""
    detector = get_traffic_detector()
    
    camera_data = get_camera_state()
    detector.process_camera_detections(camera_data)
    
    return {
        direction: {
            "state": status.traffic_light.value,
            "pedestrian": status.pedestrian_signal.value,
            "confidence": status.confidence,
            "icon": _get_light_icon(status.traffic_light.value)
        }
        for direction, status in detector.directions.items()
    }


@router.get("/intersection")
async def get_intersection_info():
    """Get intersection type and overview"""
    detector = get_traffic_detector()
    
    camera_data = get_camera_state()
    detector.process_camera_detections(camera_data)
    
    intersection_type = detector.detect_intersection_type()
    
    light_counts = {"red": 0, "green": 0, "yellow": 0, "unknown": 0}
    for status in detector.directions.values():
        light_counts[status.traffic_light.value] += 1
    
    return {
        "type": intersection_type,
        "description": _get_intersection_description(intersection_type),
        "light_counts": light_counts,
        "directions_with_signals": [
            d for d, s in detector.directions.items() 
            if s.traffic_light.value != "unknown"
        ]
    }


@router.websocket("/ws")
async def crossing_websocket(websocket: WebSocket):
    """WebSocket for real-time crossing updates"""
    await websocket.accept()
    detector = get_traffic_detector()
    
    try:
        while True:
            camera_data = get_camera_state()
            lidar_data = get_lidar_state()
            
            detector.process_camera_detections(camera_data)
            if lidar_data.get("sectors"):
                detector.process_lidar_for_vehicles(lidar_data["sectors"])
            
            # Check for client messages
            try:
                msg = await asyncio.wait_for(
                    websocket.receive_text(), 
                    timeout=0.1
                )
                data = json.loads(msg)
                if "action" in data:
                    detector.update_crossing_state(data["action"])
            except asyncio.TimeoutError:
                pass
            
            detector.update_crossing_state()

            status = detector.get_full_status()
            await websocket.send_json(status)

            await asyncio.sleep(0.2)  # 5 Hz
                
    except WebSocketDisconnect:
        pass
    except Exception as e:
        logging.getLogger(__name__).error("Crossing WebSocket error: %s", e)


def _get_light_icon(state: str) -> str:
    """Get emoji icon for traffic light state"""
    icons = {
        "red": "🔴",
        "yellow": "🟡",
        "green": "🟢",
        "unknown": "⚪"
    }
    return icons.get(state, "⚪")


def _get_intersection_description(intersection_type: str) -> str:
    """Get description for intersection type"""
    descriptions = {
        "2-way": "Two-way intersection - traffic from two directions",
        "3-way": "T-intersection - traffic from three directions",
        "4-way": "Four-way intersection - traffic from all directions",
        "unknown": "Intersection type not determined"
    }
    return descriptions.get(intersection_type, "Unknown intersection")