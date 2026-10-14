"""ToF depth camera routes (2026-10-13): its status, a depth preview, the depth dots placed on
the front camera's picture (the calibration check), and the bench calibration's captures."""

import json
import os
import time

import numpy as np
from fastapi import APIRouter, Depends, HTTPException
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import Response

from config import TOF_CALIBRATION_PAIRS_DIR, TOF_ENABLED, TOF_MAX_MM
from utils.access import require_admin

router = APIRouter(prefix="/tof", tags=["ToF depth"])


def _need_tof():
    if not TOF_ENABLED:
        raise HTTPException(status_code=404, detail="the ToF is off (config TOF_ENABLED)")


def _front_picture():
    """(BGR copy of the front camera's newest picture, when it was captured), or (None, 0)."""
    from utils import pi_camera_utils as pc
    with pc.frame_lock:
        mosaic, t, meta = pc.current_frame, pc.current_frame_t, pc.current_quad_meta
        if mosaic is None:
            return None, 0.0
        quad = pc._sector_slice(mosaic, "front").copy()
        t = float((meta.get("front") or {}).get("t") or t) if meta else float(t)
    return pc._to_bgr(quad), t


def _jpeg(img) -> Response:
    import cv2
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    if not ok:
        raise HTTPException(status_code=500, detail="could not encode the picture")
    return Response(content=buf.tobytes(), media_type="image/jpeg",
                    headers={"Cache-Control": "no-store"})


@router.get("/status")
def status():
    """Is the ToF delivering, how fast, and which calibration places its dots."""
    if not TOF_ENABLED:
        return {"enabled": False}
    from utils.tof_camera import tof_status
    return tof_status()


@router.get("/depth.jpg")
def depth_preview():
    """The newest depth frame, coloured red (near) to blue (TOF_MAX_MM), 4x enlarged."""
    _need_tof()
    import cv2
    from utils.tof_camera import latest_raw
    raw = latest_raw()
    if raw is None:
        raise HTTPException(status_code=503, detail="no ToF frame yet")
    depth = raw[1]
    hue = (np.clip(depth / TOF_MAX_MM, 0.0, 1.0) * 120.0).astype(np.uint8)
    hsv = np.dstack([hue, np.full_like(hue, 255), np.where(depth > 0, 255, 0).astype(np.uint8)])
    img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    return _jpeg(cv2.resize(img, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))


@router.get("/overlay.jpg")
def overlay():
    """The front camera's picture with the ToF frame captured nearest it drawn on, each dot
    where its own distance places it: on a correct calibration a table's dots sit on the
    table and the floor's on the floor, near and far."""
    _need_tof()
    from config import TOF_PAIR_MAX_S
    from utils.depth_fusion import draw_overlay
    from utils.tof_camera import nearest
    picture, t = _front_picture()
    if picture is None:
        raise HTTPException(status_code=503, detail="no camera picture yet")
    frame = nearest(t, max(TOF_PAIR_MAX_S, 0.2))
    return _jpeg(draw_overlay(picture, frame))


@router.post("/calibrate/capture", dependencies=[Depends(require_admin)])
async def capture_pair():
    """Save one calibration pair to TOF_CALIBRATION_PAIRS_DIR: the front camera's picture,
    the ToF's brightness picture and its depth, captured within 50 ms of each other. Hold
    the checkerboard still where both see it; take 15-25 at different places and tilts,
    then run tools/tof_calibrate.py solve."""
    _need_tof()

    def save():
        import cv2
        from utils.tof_camera import latest_raw
        deadline = time.time() + 2.0
        while time.time() < deadline:
            picture, t_cam = _front_picture()
            raw = latest_raw()
            if picture is not None and raw is not None and abs(raw[0] - t_cam) <= 0.05:
                break
            time.sleep(0.02)
        else:
            raise HTTPException(status_code=503, detail="no camera picture and ToF frame "
                                                        "within 50 ms of each other in 2 s")
        t_tof, depth, ir = raw
        if ir is None:
            raise HTTPException(status_code=503, detail="this SDK gives no brightness "
                                                        "picture to find the board in")
        os.makedirs(TOF_CALIBRATION_PAIRS_DIR, exist_ok=True)
        n = 1 + sum(1 for f in os.listdir(TOF_CALIBRATION_PAIRS_DIR) if f.endswith("_cam.png"))
        stem = os.path.join(TOF_CALIBRATION_PAIRS_DIR, f"{n:03d}")
        lo, hi = np.percentile(ir[ir > 0], [1, 99]) if np.any(ir > 0) else (0.0, 1.0)
        ir8 = (np.clip((ir - lo) / max(hi - lo, 1e-6), 0.0, 1.0) * 255.0).astype(np.uint8)
        cv2.imwrite(stem + "_cam.png", picture)
        cv2.imwrite(stem + "_tof.png", ir8)
        np.save(stem + "_depth.npy", depth.astype(np.float32))
        with open(stem + "_meta.json", "w") as f:
            json.dump({"camera_t": t_cam, "tof_t": t_tof}, f)
        return {"saved": os.path.basename(stem), "pairs": n,
                "gap_ms": round(abs(t_tof - t_cam) * 1000.0, 1)}

    return await run_in_threadpool(save)


@router.post("/calibrate/reload", dependencies=[Depends(require_admin)])
def reload():
    """Re-read the calibration file after tools/tof_calibrate.py solve."""
    _need_tof()
    from utils.depth_fusion import reload_calibration
    return reload_calibration()
