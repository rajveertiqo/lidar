"""Fused guidance API routes."""

import asyncio
from pathlib import Path

from fastapi import APIRouter, Depends, HTTPException, Request, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, StrictBool
from fastapi.responses import HTMLResponse
from starlette.concurrency import run_in_threadpool

from utils.access import require_admin
from utils.sensor_fusion import fuse_guidance, describe_scene, get_system_health
from utils.unified_guidance import latest_guidance

router = APIRouter(prefix="/guidance", tags=["Guidance"])

_TEMPLATES_DIR = Path(__file__).resolve().parent.parent / "templates"


class MobileDeliveryReport(BaseModel):
    connected: StrictBool
    audio_ready: StrictBool


@router.post("/mobile-delivery")
def mobile_delivery_report(payload: MobileDeliveryReport, request: Request):
    """The local BLE bridge's lease; a connection alone does not establish audio readiness."""
    if request.client is None or request.client.host not in ("127.0.0.1", "::1"):
        raise HTTPException(status_code=403, detail="local BLE bridge only")
    from utils import mobile_delivery

    mobile_delivery.report(payload.connected, payload.audio_ready)
    return {"ok": True}


@router.get("/fused")
def get_fused():
    """Current prioritized guidance (nearest / most-dangerous hazard first)."""
    return fuse_guidance()


@router.get("/walk")
def get_walk():
    """ONE merged walking instruction, as the guidance loop last made it — a request never runs the pipeline, so it cannot decide or commit what is said."""
    return latest_guidance()


@router.get("/policy")
def get_policy():
    """The straight-walk decision on its own, in FEET, as the guidance loop last made it — running walk_policy.decide() here would rewrite the loop's turn memory and could change what is spoken (2026-10-23)."""
    latest = latest_guidance()
    walk = latest.get("walk") or {
        "action": "unknown",
        "direction": None,
        "instruction": "",
        "reason": (latest.get("path") or {}).get("reason", ""),
        "units": "ft",
    }
    return {**walk, "loop": latest["loop"]}


@router.get("/drift")
def get_drift():
    """Predictive drift / collision-course assessment."""
    from utils.shared_state import get_drift_state

    state = dict(get_drift_state() or {})
    state["drift_would_speak"] = bool(state.get("speak"))
    state["speak"] = False
    state["note"] = "diagnostic only — nothing from this endpoint is spoken"
    return state


@router.get("/heading")
def get_heading():
    """Raw LiDAR odometry: turn rate, walking speed, accumulated heading."""
    from utils.heading_tracker import get_tracker

    return get_tracker().state()


@router.get("/gyro")
async def get_gyro_state():
    """IMU health, yaw bias, tilt and gait — the fusion's other half."""
    from utils.gyro_reader import get_gyro

    return get_gyro().state()


@router.get("/gait")
def get_gait():
    """Walking speed, cadence and the stride behind them — one readout."""
    import config
    from utils.gait_calibration import status as stride_status
    from utils.gyro_reader import get_gyro
    from utils.heading_tracker import get_tracker

    gyro = get_gyro()
    gs = gyro.state()
    hs = get_tracker().state()

    return {
        "walking": gs.get("walking"),
        "cadence_spm": gs.get("cadence_spm"),
        "steps": gs.get("steps"),
        "speed_mps": hs.get("speed_mps"),
        "speed_source": hs.get("speed_source"),
        "speed_cadence_mps": hs.get("speed_cadence_mps"),
        "stride": stride_status(),
        "gyro": {
            "healthy": gs.get("healthy"),
            "chip": gs.get("chip"),
            "transport": gs.get("transport"),
            "sample_rate_hz": gs.get("sample_rate_hz"),
            "accel_std_g": gs.get("accel_std_g"),
            "tilt_deg": gs.get("tilt_deg"),
            "tilt_warn": gs.get("tilt_warn"),
            "last_error": gs.get("last_error"),
        },
        "tuning": gyro.gait_tuning(),
        "thresholds": {
            "peak_g": config.GYRO_STEP_PEAK_G,
            "std_g": config.GYRO_STEP_STD_G,
            "refractory_s": config.GYRO_STEP_REFRACTORY_S,
            "cadence_window_s": config.GYRO_CADENCE_WINDOW_S,
            "upward_only": config.GYRO_STEP_UPWARD_ONLY,
        },
    }


@router.post("/gait/stride", dependencies=[Depends(require_admin)])
async def set_gait_stride(payload: dict):
    """Record a measured stride, and persist it across restarts."""
    from utils.gait_calibration import (
        clear_stride,
        set_stride,
        status,
        stride_from_walk,
    )

    if payload.get("clear"):
        clear_stride()
        return {"ok": True, "stride": status(), "restart_required": True}

    distance = payload.get("distance_m")
    steps = payload.get("steps")
    stride = payload.get("stride_m")

    try:
        if distance is not None and steps is not None:
            implied = stride_from_walk(float(distance), int(steps))
            saved = set_stride(
                implied,
                source="measured walk",
                distance_m=float(distance),
                steps=int(steps),
            )
        elif stride is not None:
            saved = set_stride(float(stride), source="set directly")
        else:
            return {
                "ok": False,
                "error": "expected {'distance_m', 'steps'}, "
                "{'stride_m'} or {'clear': true}",
            }
    except (TypeError, ValueError, OverflowError) as exc:  # {"steps": Infinity}: a 500
        return {"ok": False, "error": str(exc)}

    return {"ok": True, "saved": saved, "stride": status(), "restart_required": True}


@router.get("/gait/test", response_class=HTMLResponse)
async def gait_test():
    """The walking-speed test page — the old walking_speed.py, in a browser."""
    f = _TEMPLATES_DIR / "gait_test.html"
    if not f.exists():
        return HTMLResponse("<h1>gait_test.html missing</h1>", status_code=404)
    return HTMLResponse(f.read_text(encoding="utf-8"))


@router.get("/scene")
def get_scene():  # plain `def`, as /fused
    """On-demand 'what's around me?' — full readout of every direction."""
    return describe_scene()


# Plain `def`: the scan fits surfaces to the whole sweep.
@router.get("/spatial")
def get_spatial():
    """What KIND of place this is, and the ways out of it."""
    from utils.spatial_scan import scan

    try:
        clearances = ((latest_guidance() or {}).get("path") or {}).get("clearance_ft")
    except Exception:
        clearances = None
    return scan(clearances)


@router.get("/status")
async def get_status():
    """Sensor health so the user can be told audibly if the device is degraded."""
    return get_system_health()


@router.websocket("/ws")
async def guidance_websocket(websocket: WebSocket):
    """Stream fused guidance ~5x/sec for a continuous delivery layer."""
    await websocket.accept()
    try:
        while True:
            # Off the event loop, as /fused: five times a second per client.
            await websocket.send_json(await run_in_threadpool(fuse_guidance))
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"[WARN] Guidance WebSocket error: {e}")


@router.get("/log")
async def speech_log_recent(limit: int = 50):
    """Everything the device has ADMITTED for speech this run, newest last, with timestamps, where each was handed (`via`) and what became of it (`outcome`: queued, started, completed, replaced, interrupted, failed, unavailable…)."""
    from utils import speech_log

    return speech_log.recent(limit=max(1, min(400, limit)))


# The longest tester note kept (config.MARK_NOTE_MAX_CHARS, audit pass 2).
from config import MARK_NOTE_MAX_CHARS  # noqa: E402


def _one_line_note(raw) -> str:
    """A tester note as one plain line of at most MARK_NOTE_MAX_CHARS."""
    text = str(raw)[: MARK_NOTE_MAX_CHARS * 4]  # bound the work before the scan
    text = "".join(c if c.isprintable() else " " for c in text)
    return " ".join(text.split())[:MARK_NOTE_MAX_CHARS]


@router.post("/mark")
async def tester_mark(payload: dict):
    """A field tester's verdict, stamped into the speech log."""
    from utils import speech_log, simple_guidance, test_console

    kind = str((payload or {}).get("kind", "note")).lower()
    if kind not in ("right", "wrong", "missed", "note"):
        kind = "note"
    note = _one_line_note((payload or {}).get("note", ""))
    st = simple_guidance.state()
    last = st.get("counted") or st.get("last") or "nothing yet"
    speech_log.say(
        "tester",
        f"MARK {kind.upper()}",
        level="mark",
        detail=f"{note + ' | ' if note else ''}last said: {last}",
    )
    test_console.mark(kind, f"{note + ' | ' if note else ''}last said: {last}")
    try:
        from utils.walk_recorder import get_recorder

        get_recorder().mark(kind, f"{note + ' | ' if note else ''}last said: {last}")
    except Exception:
        pass
    return {"ok": True, "kind": kind, "last_said": last}


@router.get("/record")
def walk_record_status():
    """Is the walk being recorded for replay (tools/walk_replay.py)?"""
    from utils.walk_recorder import get_recorder

    return get_recorder().status()


@router.post("/record", dependencies=[Depends(require_admin)])
async def walk_record(payload: dict):
    """{"on": true, "label": "..."} starts recording every guidance tick to
    logs/walks/; {"on": false} stops. Nothing about guidance changes. Admin token: from
    any device on the Wi-Fi it filled the card (scan 2026-10-23)."""
    from utils.walk_recorder import get_recorder

    rec = get_recorder()
    if (payload or {}).get("on"):
        return rec.start(_one_line_note((payload or {}).get("label", "")))
    return await run_in_threadpool(rec.stop)


@router.get("/test")
def field_test_snapshot(limit: int = 12):
    """Everything the dashboard's Field test panel shows, in one poll: sensor health, startup, which voice is speaking, how much was said in the last minute, and the latest lines of the speech log (marks included)."""
    import time as _time
    from utils import pi_voice, speech_log, startup_gate

    log = speech_log.recent(limit=400)
    lines = log.get("lines") or []
    now = _time.time()
    # Tester marks and the LiDAR movers' silent "would say" lines were not said.
    unsaid = ("tester", "lidar-movers")
    spoken = [r for r in lines if r.get("source") not in unsaid]
    totals = log.get("totals") or {}
    marks = int(totals.get("tester", 0))
    return {
        "health": get_system_health(),
        "startup": startup_gate.state(),
        "voice": pi_voice.status(),
        "said_last_minute": sum(1 for r in spoken if now - r.get("epoch", 0) <= 60),
        "said_total": sum(n for s, n in totals.items() if s not in unsaid),
        "marks_total": marks,
        "would_say_total": int(totals.get("lidar-movers", 0)),
        "log_path": log.get("path"),
        "lines": lines[-max(1, min(50, limit)) :],
        "geometry": _geometry(),
    }


def _geometry():
    import config as c

    return {
        "block_ft": c.WALK_BLOCK_FT,
        "approach_ft": c.SIMPLE_WALL_APPROACH_FT,
        "corridor_half_mm": c.GUIDANCE_CORRIDOR_HALF_WIDTH_MM,
        "corridor_max_mm": c.GUIDANCE_CORRIDOR_MAX_MM,
        "cone_deg": c.GUIDANCE_CONE_DEG,
        # The front camera's box: objects and vehicles only count inside it.
        "camera_band": c.SIMPLE_OBJECT_CENTRE_BAND,
        "named_objects": c.SIMPLE_NAMED_OBJECTS,
        "vehicle_classes": list(c.SIMPLE_VEHICLE_CLASSES),
    }


@router.get("/log/text")
async def speech_log_text(limit: int = 50):
    """The same lines as plain text, for `curl` from a phone mid-walk."""
    from fastapi.responses import PlainTextResponse
    from utils import speech_log

    snap = speech_log.recent(limit=max(1, min(400, limit)))
    if not snap["lines"]:
        return PlainTextResponse("(nothing said yet this run)\n")

    body = "\n".join(
        f"{r['time']}  {r['elapsed']}  {r['source']:<17} {r['text']}"
        + (f"   ({r['detail']})" if r["detail"] else "")
        for r in snap["lines"]
    )
    return PlainTextResponse(body + "\n")
