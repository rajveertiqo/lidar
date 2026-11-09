"""Pi Camera API routes - Multi-camera support"""

import json
import time

from fastapi import APIRouter, Response
from fastapi.responses import StreamingResponse

from config import (
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    FRAME_SECTOR_LAYOUT,
    HEF_PATH,
    JPEG_QUALITY,
    TARGET_FPS,
)
from utils import hailo_stats
from utils.pi_camera_utils import (
    gen_frames,
    gen_seg_frames,
    gen_seg_grid,
    get_camera_stats,
    fps_stats,
    get_frame_payload,
    live_detections,
    live_jpeg,
)

router = APIRouter(prefix="/camera", tags=["Camera"])

VALID_SECTORS = ["front", "right", "left", "back"]

# Boxes sent with a live picture, per camera (the most confident): enough to show what the
# detector sees, and a header the size of a few kB at most.
LIVE_BOXES_PER_CAMERA = 15
# A camera's boxes are dropped once the picture they were found in is this much older than
# the picture shown (a side camera not detected for a while, e.g. while a seg view is open).
LIVE_BOXES_MAX_AGE_S = 2.5


def _live_boxes(view: str, fid: int, shot: float) -> dict:
    """{camera: {"frame_id", "age_ms", "detections"}} for picture `fid`: what the detector
    found in it — the front's, waited for — or, for a side camera detected every ~1.2 s, in
    its last picture, `age_ms` older than this one (drawn as old)."""
    from utils.sensor_fusion import with_distances

    found = with_distances(live_detections(view, fid))
    out = {}
    for cam, entry in found.items():
        if not isinstance(entry, dict) or entry.get("frame_id") is None:
            continue
        age_s = max(0.0, shot - (entry.get("frame_time") or shot))
        if age_s > LIVE_BOXES_MAX_AGE_S:
            continue
        dets = []
        for d in (entry.get("detections") or [])[:LIVE_BOXES_PER_CAMERA]:
            box = {
                "class_name": d.get("class_name"),
                "confidence": round(float(d.get("confidence") or 0), 2),
                "bbox": [round(float(v)) for v in (d.get("bbox") or [0, 0, 0, 0])],
                "distance_ft": None
                if d.get("distance_ft") is None
                else round(float(d["distance_ft"]), 1),
                "distance_source": d.get("distance_source"),
            }
            depth = d.get("depth") or {}
            if depth.get("distance_m") is not None:
                box["depth"] = {"distance_m": round(float(depth["distance_m"]), 2)}
            dets.append(box)
        out[cam] = {
            "frame_id": entry["frame_id"],
            "age_ms": round(age_s * 1000),
            "detections": dets,
        }
    return out


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


@router.get("/live/{view}")
def live_picture(view: str, after: int = -1, width: int = 480, boxes: int = 0):
    """The newest picture of one camera ("front", "right", "left", "back") or of all four
    ("grid", in the camera's 2x2), as one small JPEG — waiting up to 1 s for one newer than
    frame `after`; 204 if none came. The dashboard asks for the next one only once this one
    is shown, so nothing queues and the picture cannot fall behind, as the MJPEG stream did
    over Wi-Fi (2026-11-02). A def, not async: the wait runs off the event loop.
    With `boxes=1`, X-Detections carries what the detector found in the picture's camera(s)
    (_live_boxes) — the boxes drawn on THIS picture, not on one polled 0.7 s apart."""
    if view != "grid" and view not in VALID_SECTORS:
        view = "front"
    got = live_jpeg(view, after=after, wait_s=1.0, width=width)
    if got is None:
        return Response(status_code=204, headers={"Cache-Control": "no-store"})
    jpeg, fid, shot, simulated = got
    headers = {
        "Cache-Control": "no-store",
        "X-Frame-Id": str(fid),
        "X-Simulated": "1" if simulated else "0",
    }
    if view == "grid":
        # Where each camera sits in THIS picture, [row, column] (2026-11-09): the dashboard
        # cut its tiles by a layout it fetched once — or, before that came, by the one built
        # into the page, the cameras' order before they were moved — and showed one camera's
        # picture under another's name, with the right camera's boxes on it.
        headers["X-Camera-Layout"] = json.dumps(
            {s: list(rc) for s, rc in FRAME_SECTOR_LAYOUT.items()}, separators=(",", ":")
        )
    if boxes:
        headers["X-Detections"] = json.dumps(
            _live_boxes(view, fid, shot), separators=(",", ":")
        )
    # Last: the wait for the front's boxes is part of the picture's age.
    headers["X-Frame-Age-Ms"] = str(max(0, round((time.time() - shot) * 1000)))
    return Response(content=jpeg, media_type="image/jpeg", headers=headers)


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
