"""Live map API — the /map test page: LiDAR, radar and cameras placed around the user, turned by the gyro."""

import asyncio
from pathlib import Path

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from starlette.concurrency import run_in_threadpool

from utils.live_map import get_map

router = APIRouter(prefix="/map", tags=["Map"])

_TEMPLATES_DIR = Path(__file__).resolve().parent.parent / "templates"

_PUSH_S = 0.1  # ten updates a second
_MAP_EVERY = 5  # the remembered walls go with every 5th (twice a second)


@router.get("", response_class=HTMLResponse)
async def map_page():
    """The live 3D map page, read fresh from disk like the other pages."""
    f = _TEMPLATES_DIR / "map_test.html"
    if not f.exists():
        return HTMLResponse("<h1>map_test.html missing</h1>", status_code=404)
    return HTMLResponse(f.read_text(encoding="utf-8"))


# Plain `def`: the snapshot reads every sensor's state.
@router.get("/state")
def map_state(walls: bool = False):
    """One snapshot, as the page receives it (starts the map if nobody is watching)."""
    live_map = get_map()
    live_map.touch()
    return live_map.snapshot(include_map=walls)


@router.post("/reset")
def map_reset():
    """Start the map again from where the user stands."""
    get_map().reset()
    return {"ok": True}


@router.post("/settings")
def map_settings(payload: dict):
    """Where the user's direction comes from: {"yaw_source": "gyro"|"lidar", "gyro_flip": bool}."""
    payload = payload or {}
    flip = payload.get("gyro_flip")
    settings = get_map().configure(
        yaw_source=payload.get("yaw_source"),
        gyro_flip=None if flip is None else bool(flip),
    )
    return {"ok": True, "settings": settings}


@router.websocket("/ws")
async def map_websocket(websocket: WebSocket):
    """Stream the live map to the page while it is open."""
    await websocket.accept()
    live_map = get_map()
    live_map.acquire()
    try:
        n = 0
        while True:
            snap = await run_in_threadpool(live_map.snapshot, n % _MAP_EVERY == 0)
            await websocket.send_json(snap)
            n += 1
            await asyncio.sleep(_PUSH_S)
    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"[WARN] Map WebSocket error: {e}")
    finally:
        live_map.release()
