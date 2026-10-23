"""Pi Camera API routes - Multi-camera support"""

from fastapi import APIRouter
from fastapi.responses import StreamingResponse

from config import CAMERA_WIDTH, CAMERA_HEIGHT, JPEG_QUALITY, TARGET_FPS, HEF_PATH
from utils import hailo_stats
from utils.pi_camera_utils import (
    gen_frames,
    gen_seg_frames,
    gen_seg_grid,
    get_camera_stats,
    fps_stats,
    get_frame_payload,
)

router = APIRouter(prefix="/camera", tags=["Camera"])

VALID_SECTORS = ["front", "right", "left", "back"]


@router.get("/video_feed")
async def video_feed():
    """MJPEG video stream for front camera (default)"""
    return StreamingResponse(
        gen_frames("front"), media_type="multipart/x-mixed-replace; boundary=frame"
    )


@router.get("/stream/{sector}")
async def sector_video_feed(sector: str):
    """MJPEG video stream for a specific camera sector"""
    if sector not in VALID_SECTORS:
        return {"error": f"Invalid sector. Valid: {VALID_SECTORS}"}

    return StreamingResponse(
        gen_frames(sector), media_type="multipart/x-mixed-replace; boundary=frame"
    )


@router.get("/seg/{sector}")
async def sector_seg_feed(sector: str):
    """MJPEG stream of the COLORIZED segmentation for a sector (debug view)."""
    if sector not in VALID_SECTORS:
        return {"error": f"Invalid sector. Valid: {VALID_SECTORS}"}

    return StreamingResponse(
        gen_seg_frames(sector), media_type="multipart/x-mixed-replace; boundary=frame"
    )


@router.get("/seg_grid")
async def seg_grid_feed():
    """Single MJPEG stream: 2x2 colorized segmentation of all four sectors."""
    return StreamingResponse(
        gen_seg_grid(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


@router.get("/stats")
async def stats():
    """Camera and detection statistics; every detection carries how far away it is (`distance_ft`, `distance_source`: lidar / camera / camera_cut / tof; and `depth` when the ToF measured it)."""
    from utils.sensor_fusion import with_distances

    return with_distances(get_camera_stats())


@router.get("/distances")
async def distances():
    """Why each detection's distance is what it is: the bearings searched, every thing the laser ranges there, the box-size estimate, and which was chosen."""
    from utils.sensor_fusion import with_distances

    stats = with_distances(get_camera_stats(), explain=True)
    return {
        s: (stats.get(s) or {}).get("detections", [])
        for s in ("front", "right", "left", "back")
    }


@router.get("/fps")
async def fps():
    """Get current detection FPS"""
    return {
        "capture_fps": fps_stats["capture_fps"],
        "detection_fps": fps_stats["detection_fps"],
    }


@router.get("/hailo")
async def hailo():
    """Per-model NPU utilization and FPS."""
    return hailo_stats.snapshot()


@router.get("/config")
async def config():
    """Get camera configuration"""
    return {
        "width": CAMERA_WIDTH,
        "height": CAMERA_HEIGHT,
        "quality": JPEG_QUALITY,
        "target_fps": TARGET_FPS,
        "hef_path": HEF_PATH,
        "sectors": VALID_SECTORS,
    }


@router.get("/frame")
def get_frame():
    """Get current frame as base64 (front camera)"""
    p = get_frame_payload("front")
    return {
        "frame": p["frame"],
        "sector": "front",
        "success": bool(p["frame"]),
        "simulated": p["simulated"],
    }


@router.get("/frame/{sector}")
def get_sector_frame(sector: str):
    """Get current frame for a specific sector as base64"""
    if sector not in VALID_SECTORS:
        return {
            "error": "Invalid sector",
            "valid_sectors": VALID_SECTORS,
            "success": False,
        }

    p = get_frame_payload(sector)
    return {
        "frame": p["frame"],
        "sector": sector,
        "success": bool(p["frame"]),
        "simulated": p["simulated"],
    }


@router.get("/frames/all")
def get_all_frames():
    """Get current frames from all cameras as base64"""
    frames = {}
    for sector in VALID_SECTORS:
        p = get_frame_payload(sector)
        frames[sector] = {
            "frame": p["frame"],
            "available": p["frame"] is not None,
            "simulated": p["simulated"],
        }

    return {"frames": frames, "success": True}
