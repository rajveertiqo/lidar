"""Environment Detection API routes"""

from fastapi import APIRouter
from starlette.concurrency import run_in_threadpool

from utils.environment_detection import (
    detect_environment_conditions,
    get_environment_state,
    get_visibility_description,
    get_condition_icon,
    clear_detection_history,
)
from utils.shared_state import get_lidar_state
from utils import pi_camera_utils

router = APIRouter(prefix="/environment", tags=["Environment"])


@router.get("/status")
async def get_environment_status():
    """Get current environment conditions (fog, rain, lighting, etc.)"""
    lidar_state = get_lidar_state()
    lidar_points = []

    if lidar_state.get("points"):
        angles = lidar_state["points"].get("angles", [])
        distances = lidar_state["points"].get("distances", [])
        lidar_points = list(zip(angles, distances))

    result = await run_in_threadpool(
        lambda: detect_environment_conditions(
            image=pi_camera_utils.get_current_frame(),
            lidar_points=lidar_points,
        )
    )

    result["visibility_text"] = get_visibility_description(result["visibility"])
    result["condition_icon"] = get_condition_icon(result["condition"])

    return result


@router.get("/quick")
async def get_quick_status():
    """Get simplified environment status (faster, cached)"""
    state = get_environment_state()
    return {
        "condition": state["condition"],
        "condition_icon": get_condition_icon(state["condition"]),
        "visibility": state["visibility"],
        "visibility_text": get_visibility_description(state["visibility"]),
        "warnings": state["warnings"],
    }


@router.get("/details")
async def get_environment_details():
    """Get detailed environment analysis"""
    state = get_environment_state()

    return {
        "condition": state["condition"],
        "condition_icon": get_condition_icon(state["condition"]),
        "visibility": {
            "value": state["visibility"],
            "text": get_visibility_description(state["visibility"]),
            "percentage": int(state["visibility"] * 100),
        },
        "fog": {
            "level": state["details"]["fog_level"],
            "percentage": int(state["details"]["fog_level"] * 100),
            "severity": "heavy"
            if state["details"]["fog_level"] > 0.7
            else "moderate"
            if state["details"]["fog_level"] > 0.4
            else "light",
        },
        "rain": {
            "intensity": state["details"]["rain_intensity"],
            "percentage": int(state["details"]["rain_intensity"] * 100),
            "severity": "heavy"
            if state["details"]["rain_intensity"] > 0.6
            else "moderate"
            if state["details"]["rain_intensity"] > 0.3
            else "light",
        },
        "lighting": {
            "brightness": state["details"]["brightness"],
            "percentage": int(state["details"]["brightness"] * 100),
            "status": "low"
            if state["details"]["brightness"] < 0.3
            else "normal"
            if state["details"]["brightness"] < 0.8
            else "bright",
        },
        "lidar_quality": {
            "value": state["details"]["lidar_quality"],
            "percentage": int(state["details"]["lidar_quality"] * 100),
            "status": "good"
            if state["details"]["lidar_quality"] > 0.7
            else "degraded"
            if state["details"]["lidar_quality"] > 0.4
            else "poor",
        },
        "warnings": state["warnings"],
        "confidence": state["confidence"],
        "last_update": state["last_update"],
    }


@router.get("/warnings")
async def get_environment_warnings():
    """Get active environment warnings"""
    state = get_environment_state()

    warnings = list(state["warnings"])

    advice = []
    if state["condition"] == "fog":
        advice.append("Stay close to walls for guidance")
        advice.append("Use audio cues more than visual")
    elif state["condition"] == "rain":
        advice.append("Surfaces may be slippery")
        advice.append("Sound detection may be affected")
    elif state["condition"] == "low_light":
        advice.append("Camera detection less reliable")
        advice.append("Rely more on LiDAR for navigation")
    elif state["condition"] == "storm":
        advice.append("Seek shelter if possible")
        advice.append("All sensors may be affected")

    return {
        "condition": state["condition"],
        "warnings": warnings,
        "advice": advice,
        "should_alert": len(warnings) > 0,
        "severity": "high"
        if state["condition"] in ["storm", "fog"] and state["visibility"] < 0.4
        else "medium"
        if state["visibility"] < 0.6
        else "low",
    }


@router.post("/reset")
async def reset_detection_history():
    """Clear detection history for immediate reset."""
    clear_detection_history()
    return {
        "success": True,
        "message": "Detection history cleared - predictions will now respond immediately",
    }
