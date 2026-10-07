"""LiDAR API routes"""

import time

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from fastapi.responses import Response
from starlette.websockets import WebSocketState

from config import (
    LIDAR_SECTORS, LIDAR_WARNING_DISTANCE, LIDAR_MAX_DISTANCE, BEV_ENABLED,
)
from utils.lidar_utils import add_client, reader_health, remove_client
from utils.shared_state import get_lidar_state

router = APIRouter(prefix="/lidar", tags=["LiDAR"])


@router.websocket("/ws")
async def lidar_websocket(websocket: WebSocket):
    """WebSocket endpoint for LiDAR data streaming"""
    await websocket.accept()
    add_client(websocket)
    try:
        while True:
            # Keep connection alive, data is broadcast from scanner thread
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    except Exception as e:
        if websocket.application_state != WebSocketState.DISCONNECTED:
            print(f"[WARN] LiDAR WebSocket error: {e}")
    finally:
        remove_client(websocket)


@router.get("/config")
async def config():
    """Get LiDAR configuration"""
    return {
        "sectors": LIDAR_SECTORS,
        "warning_distance": LIDAR_WARNING_DISTANCE,
        "max_distance": LIDAR_MAX_DISTANCE
    }


@router.get("/reader")
def reader():
    """Is the serial reader keeping up?"""
    from utils.sensor_fusion import lidar_is_fresh
    state = get_lidar_state()
    last = state.get("last_update") or 0.0
    return {
        **reader_health(),
        "scan_index": state.get("scan_index"),
        "rpm": round(float(state.get("rpm") or 0.0), 1),
        "scan_age_s": round(time.time() - last, 3) if last else None,
        # The same verdict the guide and /health reach (utils/freshness).
        "fresh": lidar_is_fresh(state),
        "simulated": bool(state.get("simulated")),
    }


@router.get("/sectors")
async def get_sectors():
    """Get sector definitions"""
    return {
        "sectors": LIDAR_SECTORS,
        "sector_names": list(LIDAR_SECTORS.keys())
    }


@router.get("/bev")
async def bev_state():
    """Latest BEV extraction: obstacle shapes, openings, blind wedges."""
    if not BEV_ENABLED:
        return {"available": False, "error": "BEV_ENABLED is False"}
    from utils.bev_grid import get_grid
    return get_grid().snapshot()


@router.get("/bev/summary")
async def bev_summary():
    """The same extraction reduced to plain sentences, in feet."""
    if not BEV_ENABLED:
        return {"available": False, "error": "BEV_ENABLED is False"}

    from utils.bev_grid import get_grid
    snap = get_grid().snapshot()
    if not snap.get("available"):
        return snap

    lines = []
    for o in snap.get("objects", [])[:8]:
        glass = " (GLASS — radar only, laser saw through it)" if o["radar_confirmed"] else ""
        seen = f" [camera sees: {', '.join(o['camera_candidates'])}]" if o["camera_candidates"] else ""
        lines.append(
            f"{o['kind']} {o['length_mm'] / 1000:.1f} m long, "
            f"{o['clock']} o'clock, {o['range_ft']:.1f} ft{glass}{seen}"
        )
    for g in snap.get("gaps", []):
        lines.append(
            f"opening {g['width_ft']:.1f} ft wide, "
            f"{g['clock']} o'clock, {g['range_ft']:.1f} ft ahead"
        )
    for w in snap.get("unknown_wedges", [])[:4]:
        lines.append(
            f"blind arc {w['width_deg']:.0f}° wide centred {w['centre_deg']:.0f}° "
            f"— no return, could be glass"
        )

    return {
        "available": True,
        "lines": lines or ["nothing resolved yet"],
        "registered": snap.get("registered"),
        "reset_reason": snap.get("reset_reason"),
        "frames": snap.get("frames"),
        "compute_ms": snap.get("compute_ms"),
        "last_update": snap.get("last_update"),
    }


@router.get("/bev/image")
def bev_image(scale: int = 2):
    """Top-down PNG of the grid."""
    if not BEV_ENABLED:
        return Response(content=b"", status_code=404)

    from utils.bev_grid import get_grid
    png = get_grid().render_png(scale=max(1, min(4, scale)))
    if png is None:
        return Response(content=b"", status_code=503)
    return Response(content=png, media_type="image/png",
                    headers={"Cache-Control": "no-store"})
