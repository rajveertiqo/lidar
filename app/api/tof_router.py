"""ToF depth camera routes (2026-10-13): its status, a depth preview, the depth dots placed on
the front camera's picture (the calibration check), and the bench calibration's captures.
Since 2026-11-03 also the live view, /tof/live (from ~/tof/tof_live.py), fed by /tof/ws.
Since 2026-11-06 the front camera as a depth camera: /tof/rgbd.jpg and /tof/aligned.npy."""

import logging
import asyncio
import json
import os
import time
from pathlib import Path

import numpy as np
from fastapi import APIRouter, Depends, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import HTMLResponse, Response

from app.config import TOF_CALIBRATION_PAIRS_DIR, TOF_ENABLED, TOF_LIVE_MAX_FPS, TOF_MAX_MM
from app.core.access import require_admin

_log = logging.getLogger(__name__)

router = APIRouter(prefix="/tof", tags=["ToF depth"])

_TEMPLATES_DIR = Path(__file__).resolve().parents[2] / "templates"
_STATUS_S = 0.5  # /tof/ws: the status this often, between the frames
_UNASKED_S = 2.0  # /tof/ws: a page that never asks for frames gets one this often
_SEND_LIMIT_S = 3.0  # /tof/ws: a send the page has not taken in this long ends the stream
# /tof/rgbd.jpg: the front's boxes are drawn only if found in a picture this near the one
# shown (the front is detected on every picture, so they are normally one picture old).
_RGBD_BOXES_MAX_AGE_S = 0.5


def _need_tof():
    if not TOF_ENABLED:
        raise HTTPException(
            status_code=404, detail="the ToF is off (config TOF_ENABLED)"
        )


def _front_picture():
    """(BGR copy of the front camera's newest picture, when it was captured), or (None, 0)."""
    from app.sensors import pi_camera_utils as pc

    with pc.frame_lock:
        mosaic, t = pc.current_frame, pc.current_frame_t
        if mosaic is None:
            return None, 0.0
        quad = pc._sector_slice(mosaic, "front").copy()
    return pc._to_bgr(quad), float(t)


def _jpeg(img, quality: int = 85) -> Response:
    import cv2

    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise HTTPException(status_code=500, detail="could not encode the picture")
    return Response(
        content=buf.tobytes(),
        media_type="image/jpeg",
        headers={"Cache-Control": "no-store"},
    )


@router.get("/status")
def status():
    """Is the ToF delivering, how fast, and which calibration places its dots."""
    if not TOF_ENABLED:
        return {"enabled": False}
    from app.sensors.tof_camera import tof_status

    return tof_status()


@router.get("/live", response_class=HTMLResponse)
async def live_page():
    """The live depth picture (templates/tof_live.html): tap it to read a distance; below
    it, the depth dots on the front camera's picture. Read fresh from disk like the others."""
    f = _TEMPLATES_DIR / "tof_live.html"
    if not f.exists():
        return HTMLResponse("<h1>tof_live.html missing</h1>", status_code=404)
    return HTMLResponse(f.read_text(encoding="utf-8"))


@router.websocket("/ws")
async def live_websocket(websocket: WebSocket):
    """Stream /tof/live each new frame as it came up from the capture process (at most
    TOF_LIVE_MAX_FPS; layout in app/sensors/tof_camera._HEADER), and /tof/status between them.

    The page asks for each frame ("next") once it has drawn the one before (2026-11-06).
    Pushed as fast as the socket took them, ~170 KB at 15 a second (2.6 MB/s) queued in the
    network's buffers on Wi-Fi: the page showed seconds-old frames, the camera picture below
    it waited behind them, and a shutdown hung on a send. Now at most one frame is in flight.
    A page that never asks (one loaded before this) still gets one every _UNASKED_S."""
    await websocket.accept()
    reader = None
    try:
        if not TOF_ENABLED:
            await websocket.send_json({"enabled": False})
            await websocket.close()
            return
        from app.sensors.tof_camera import latest_packet, tof_status

        asked = asyncio.Event()
        asked.set()  # the first frame is not waited for

        async def listen():
            while True:
                if await websocket.receive_text() == "next":
                    asked.set()

        reader = asyncio.create_task(listen())
        seq, status_at, sent_at = 0, 0.0, 0.0
        gap = 1.0 / TOF_LIVE_MAX_FPS
        while not reader.done():  # done: the page went away
            now = time.monotonic()
            if now - status_at >= _STATUS_S:  # first, or the frames would starve it
                status_at = now
                status = await run_in_threadpool(tof_status)
                await asyncio.wait_for(websocket.send_json(status), _SEND_LIMIT_S)
                continue
            n, packet = latest_packet()
            wanted = asked.is_set() or now - sent_at >= _UNASKED_S
            if n != seq and packet is not None and wanted and now - sent_at >= gap:
                seq, sent_at = n, now
                asked.clear()
                await asyncio.wait_for(websocket.send_bytes(packet), _SEND_LIMIT_S)
            else:
                await asyncio.sleep(0.01)
    except (WebSocketDisconnect, asyncio.TimeoutError):
        pass
    except Exception as e:
        _log.warning(f"ToF WebSocket error: {e}")
    finally:
        if reader is not None:
            reader.cancel()


@router.get("/depth.jpg")
def depth_preview():
    """The newest depth frame, coloured red (near) to blue (TOF_MAX_MM), 4x enlarged."""
    _need_tof()
    import cv2
    from app.sensors.tof_camera import latest_raw

    raw = latest_raw()
    if raw is None:
        raise HTTPException(status_code=503, detail="no ToF frame yet")
    depth = raw[1]
    hue = (np.clip(depth / TOF_MAX_MM, 0.0, 1.0) * 120.0).astype(np.uint8)
    hsv = np.dstack(
        [hue, np.full_like(hue, 255), np.where(depth > 0, 255, 0).astype(np.uint8)]
    )
    img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    return _jpeg(cv2.resize(img, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))


@router.get("/overlay.jpg")
def overlay():
    """The front camera's picture with the ToF frame captured nearest it drawn on, each dot
    where its own distance places it: on a correct calibration a table's dots sit on the
    table and the floor's on the floor, near and far."""
    _need_tof()
    from app.config import TOF_PAIR_MAX_S
    from app.perception.depth_fusion import draw_overlay
    from app.sensors.tof_camera import nearest

    picture, t = _front_picture()
    if picture is None:
        raise HTTPException(status_code=503, detail="no camera picture yet")
    frame = nearest(t, max(TOF_PAIR_MAX_S, 0.2))
    return _jpeg(draw_overlay(picture, frame))


@router.get("/rgbd.jpg")
def rgbd(w: int = 0):
    """The front camera as a depth camera (2026-11-06): its picture with the depth aligned to
    it laid over (red near -> blue far; its own colours where the ToF has nothing) and each
    of its newest detections boxed with the distance the ToF measured inside the box. `w`:
    drawn this wide (the page asks for the width it shows; 0: the camera's 1164)."""
    _need_tof()
    w = max(0, min(int(w), 4096))
    from app.config import TOF_PAIR_MAX_S
    from app.perception.depth_fusion import draw_rgbd
    from app.sensors.pi_camera_utils import live_detections
    from app.sensors.tof_camera import nearest

    picture, t = _front_picture()
    if picture is None:
        raise HTTPException(status_code=503, detail="no camera picture yet")
    frame = nearest(t, max(TOF_PAIR_MAX_S, 0.2))
    found = live_detections("front", 0, 0.0).get("front") or {}
    dets = found.get("detections") or []
    if abs(float(found.get("frame_time") or 0.0) - t) > _RGBD_BOXES_MAX_AGE_S:
        dets = []  # from an older picture: they would mark where things were
    img = draw_rgbd(picture, frame, dets, width=w or None)
    from app.config import TOF_HAZARDS_ENABLED

    if TOF_HAZARDS_ENABLED:  # and the hazards on the ground, where they land
        from app.perception.ground_hazards import draw, snapshot

        snap = snapshot()
        if snap.get("t") is not None and abs(float(snap["t"]) - t) <= 1.0:
            scale = (img.shape[1] / picture.shape[1], img.shape[0] / picture.shape[0])
            img = draw(img, snap, scale)
    return _jpeg(img, 80)


@router.get("/hazards")
def hazards():
    """The ToF's hazards on the ground ahead (app/perception/ground_hazards): the ground it fitted,
    each confirmed hazard (kind, distance in feet, the camera's word on it, whether it would
    be said and why), the one the guidance would say ("top"), every unconfirmed sighting
    of the newest frame ("seen"), and the wall check ("wall": is the nearest tall thing on
    the path a wall, and why)."""
    _need_tof()
    from app.config import TOF_HAZARDS_ENABLED

    if not TOF_HAZARDS_ENABLED:
        return {"enabled": False}
    import time

    from app.config import TOF_WALL_MIN_WIDTH_M, TOF_WALL_SPEAK
    from app.perception.ground_hazards import confirmed_wall, snapshot

    snap = snapshot()
    # The wall check (2026-11-10): what the guidance would call the nearest tall thing on the
    # path, for /tof/live.
    found, why = confirmed_wall(snap, time.time())
    surfaces = snap.get("front_surfaces") or []
    snap["wall"] = {
        "found": found is not None,
        "why": why,
        "surface": surfaces[-1] if surfaces else None,
        "min_width_m": TOF_WALL_MIN_WIDTH_M,
        "speak": bool(TOF_WALL_SPEAK),
    }
    return {"enabled": True, **snap}


@router.get("/aligned.npy")
def aligned_npy(what: str = "range"):
    """The newest depth picture aligned to the front camera's, for scripts (np.load it):
    float32 metres, NaN where the ToF has nothing, one value per TOF_ALIGNED_CELL_PX x
    TOF_ALIGNED_CELL_PX block of camera pixels. `what`: "range" from the lens, or "ground"
    along the ground. The headers say the block size, the capture time and the calibration."""
    _need_tof()
    if what not in ("range", "ground"):
        raise HTTPException(status_code=400, detail='what: "range" or "ground"')
    import io

    from app.config import TOF_ALIGNED_CELL_PX
    from app.sensors.tof_camera import nearest

    frame = nearest(time.time(), 1.0)
    if frame is None:
        raise HTTPException(status_code=503, detail="no ToF frame in the last second")
    buf = io.BytesIO()
    np.save(buf, frame.depth_map(TOF_ALIGNED_CELL_PX, what))
    return Response(
        content=buf.getvalue(),
        media_type="application/octet-stream",
        headers={
            "Cache-Control": "no-store",
            "X-Cell-Px": str(TOF_ALIGNED_CELL_PX),
            "X-Picture-Size": f"{frame.size[0]}x{frame.size[1]}",
            "X-Captured-At": f"{frame.t:.3f}",
            "X-Calibration": frame.source,
        },
    )


@router.post("/calibrate/capture", dependencies=[Depends(require_admin)])
async def capture_pair():
    """Save one calibration pair to TOF_CALIBRATION_PAIRS_DIR: the front camera's picture,
    the ToF's brightness picture and its depth, captured within 50 ms of each other. Hold
    the checkerboard still where both see it; take 15-25 at different places and tilts,
    then run tools/device/tof_calibrate.py solve."""
    _need_tof()

    def save():
        import cv2
        from app.sensors.tof_camera import latest_raw

        deadline = time.time() + 2.0
        while time.time() < deadline:
            picture, t_cam = _front_picture()
            raw = latest_raw()
            if picture is not None and raw is not None and abs(raw[0] - t_cam) <= 0.05:
                break
            time.sleep(0.02)
        else:
            raise HTTPException(
                status_code=503,
                detail="no camera picture and ToF frame "
                "within 50 ms of each other in 2 s",
            )
        t_tof, depth, ir = raw
        if ir is None:
            raise HTTPException(
                status_code=503,
                detail="this SDK gives no brightness picture to find the board in",
            )
        os.makedirs(TOF_CALIBRATION_PAIRS_DIR, exist_ok=True)
        # After the highest number, not the count: with one deleted, the count named an
        # existing pair and the capture overwrote it (scan 2026-10-23).
        n = 1 + max(
            (
                int(f[:-8])
                for f in os.listdir(TOF_CALIBRATION_PAIRS_DIR)
                if f.endswith("_cam.png") and f[:-8].isdigit()
            ),
            default=0,
        )
        pairs = 1 + sum(
            1 for f in os.listdir(TOF_CALIBRATION_PAIRS_DIR) if f.endswith("_cam.png")
        )
        stem = os.path.join(TOF_CALIBRATION_PAIRS_DIR, f"{n:03d}")
        lo, hi = np.percentile(ir[ir > 0], [1, 99]) if np.any(ir > 0) else (0.0, 1.0)
        ir8 = (np.clip((ir - lo) / max(hi - lo, 1e-6), 0.0, 1.0) * 255.0).astype(
            np.uint8
        )
        cv2.imwrite(stem + "_cam.png", picture)
        cv2.imwrite(stem + "_tof.png", ir8)
        np.save(stem + "_depth.npy", depth.astype(np.float32))
        with open(stem + "_meta.json", "w") as f:
            json.dump({"camera_t": t_cam, "tof_t": t_tof}, f)
        return {
            "saved": os.path.basename(stem),
            "pairs": pairs,
            "gap_ms": round(abs(t_tof - t_cam) * 1000.0, 1),
        }

    return await run_in_threadpool(save)


@router.post("/calibrate/reload", dependencies=[Depends(require_admin)])
def reload():
    """Re-read the calibration file after tools/device/tof_calibrate.py solve."""
    _need_tof()
    from app.perception.depth_fusion import reload_calibration

    return reload_calibration()
