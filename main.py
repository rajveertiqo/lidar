"""Main FastAPI Application Combines Pi Camera, LiDAR, Navigation, Environment Detection, and Road Crossing systems"""

import asyncio
from pathlib import Path
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.exceptions import RequestValidationError
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from starlette.datastructures import Headers

from config import (
    ADMIN_TOKEN,
    CORS_ORIGINS,
    SERVER_GRACEFUL_SHUTDOWN_S,
    SERVER_HOST,
    SERVER_PORT,
)
from utils.pi_camera_utils import start_camera, stop_camera
from utils.lidar_utils import start_scanner, stop_scanner_thread, setup_signal_handlers
from utils.radar_utils import start_radar, stop_radar
from utils.gyro_reader import start_gyro, stop_gyro
from utils.shared_state import reset_all_states
from routers import radar_router
from routers import (
    environment_router,
    lidar_router,
    navigation_router,
    pi_camera_router,
)
from routers import guidance_router
from routers import tof_router
from routers import road_crossing_router
from routers import map_router

BASE_DIR = Path(__file__).resolve().parent
TEMPLATES_DIR = BASE_DIR / "templates"


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan manager"""
    setup_signal_handlers()
    reset_all_states()
    start_camera()

    start_gyro()

    loop = asyncio.get_running_loop()
    start_scanner(loop)
    start_radar()
    from utils.tof_camera import start_tof

    start_tof()  # only with TOF_ENABLED (the depth camera)

    from utils import pi_voice, service, test_console

    pi_voice.start()
    test_console.start()

    heartbeat = asyncio.create_task(_event_loop_heartbeat())
    service.start_watchdog(required=("guidance", "event_loop"))

    import socket

    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("10.255.255.255", 1))
        lan_ip = probe.getsockname()[0]
        probe.close()
    except Exception:
        lan_ip = SERVER_HOST
    print("🚀 Application started")
    print(f"🔌 Dashboard:  http://{lan_ip}:{SERVER_PORT}   (open this on the phone)")
    print(
        f"📝 Speech log: http://{lan_ip}:{SERVER_PORT}/guidance/log/text   (and logs/speech_*.log)"
    )
    print(
        f"🗺  Live map:   http://{lan_ip}:{SERVER_PORT}/map   (3D test page, display only)"
    )
    print("⏹  Ctrl+C to stop")
    if not ADMIN_TOKEN:
        print(
            "⚠  No admin token: radar calibration, radar restart and the stride "
            "are REFUSED until SHAPPE_ADMIN_TOKEN is set (./start_lidar.sh and "
            "the service set one)"
        )
    from config import AUDIO_ENABLED

    if not AUDIO_ENABLED:
        print(
            "⚠  AUDIO_ENABLED is False in config.py: NOTHING will be spoken, on the Pi "
            "or the phone. Bench use only — never with a wearer."
        )

    yield

    service.stopping()
    heartbeat.cancel()
    test_console.stop()
    pi_voice.stop()
    stop_camera()
    stop_scanner_thread()
    stop_radar()
    from utils.tof_camera import stop_tof

    stop_tof()
    stop_gyro()
    print("👋 Application shutdown complete")


app = FastAPI(
    title="Blind Assistance System - LiDAR, Vision & Road Crossing",
    description="Combined camera detection, LiDAR visualization, navigation,  environment detection, and road crossing assistance",
    version="3.0.0",
    lifespan=lifespan,
)


@app.exception_handler(RequestValidationError)
async def _invalid_request(request, exc):
    """422 without echoing the input back: FastAPI's own reply copies the whole body into
    the error, and a deeply nested one overflowed the encoder (a 500 and a log line the size
    of the body, fuzz test 2026-10-22)."""
    detail = [
        {
            "type": e.get("type"),
            "loc": [str(p) for p in e.get("loc", ())][:8],
            "msg": str(e.get("msg", ""))[:200],
        }
        for e in exc.errors()[:20]
    ]
    return JSONResponse({"detail": detail}, status_code=422)


_NO_FRAMING = {
    "X-Frame-Options": "DENY",
    "Content-Security-Policy": "frame-ancestors 'none'",
}


@app.middleware("http")
async def _refuse_cross_site_writes(request, call_next):
    """Refuse a state-changing request made by another site's page (F9)."""
    from utils.access import cross_site_refusal

    why = cross_site_refusal(
        request.method,
        request.headers.get("origin", ""),
        request.headers.get("host", ""),
        CORS_ORIGINS,
    )
    if why:
        response = JSONResponse({"detail": why}, status_code=403)
    else:
        response = await call_next(request)
    for name, value in _NO_FRAMING.items():
        response.headers.setdefault(name, value)
    return response


class _RefuseCrossSiteWebSockets:
    """Refuse a WebSocket handshake from another site's page (audit pass 2, 2026-09-29, NEW-021)."""

    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] == "websocket":
            from utils.access import websocket_refusal

            headers = Headers(scope=scope)  # read as the HTTP rule reads them
            if websocket_refusal(
                headers.get("origin", ""), headers.get("host", ""), CORS_ORIGINS
            ):
                await send({"type": "websocket.close", "code": 1008})
                return
        await self.app(scope, receive, send)


app.add_middleware(_RefuseCrossSiteWebSockets)


if CORS_ORIGINS:
    app.add_middleware(
        CORSMiddleware,
        allow_origins=CORS_ORIGINS,
        allow_methods=["*"],
        allow_headers=["*"],
    )


async def _event_loop_heartbeat():
    """The web server's heartbeat for the systemd watchdog (utils/service)."""
    from utils import service

    while True:
        service.beat("event_loop")
        await asyncio.sleep(1.0)


app.include_router(pi_camera_router.router)
app.include_router(lidar_router.router)
app.include_router(radar_router.router)
app.include_router(navigation_router.router)
app.include_router(environment_router.router)
app.include_router(road_crossing_router.router)
app.include_router(guidance_router.router)
app.include_router(tof_router.router)
app.include_router(map_router.router)


def _page(name: str) -> HTMLResponse:
    """A template, read fresh from disk on every request so an edit shows on a browser refresh without restarting the app."""
    html_file = TEMPLATES_DIR / name
    if html_file.exists():
        return HTMLResponse(content=html_file.read_text(), status_code=200)
    return HTMLResponse(content=_get_fallback_html(), status_code=200)


@app.get("/", response_class=HTMLResponse)
async def root():
    """The dashboard — rebuilt for field testing (templates/dashboard.html)."""
    return _page("dashboard.html")


@app.get("/test", response_class=HTMLResponse)
async def field_test_page():
    """Same page as /: it grew out of the field-test page and replaces it."""
    return _page("dashboard.html")


@app.get("/classic", response_class=HTMLResponse)
async def classic_dashboard():
    """The previous dashboard, kept for comparison (templates/index.html)."""
    return _page("index.html")


@app.get("/health")
def health_check():
    """Can this device guide someone right now?"""
    from utils import pi_health, pi_voice, service, startup_gate
    from utils.sensor_fusion import get_system_health

    sensors = get_system_health()
    loop = pi_voice.loop_status()
    age = loop.get("last_ok_age_s")
    guiding = age is not None and age < 5.0
    voice = pi_voice.status()
    ready = bool(startup_gate.state().get("ready"))
    warnings = list(sensors.get("warnings", []))
    system = pi_health.read()
    if system.get("power_low"):
        warnings.append(
            "Under-voltage: the battery or power supply is failing — the device "
            "may switch off"
        )
    if system.get("hot"):
        soc = system.get("soc_c")
        warnings.append(
            "The Pi is overheating"
            + (f" ({soc:.0f} C)" if soc is not None else "")
            + " or throttling — guidance may slow down"
        )
    if not voice.get("audio_enabled"):
        warnings.append(
            "Audio is switched off in config (AUDIO_ENABLED) — nothing is "
            "spoken, on the Pi or the phone"
        )
    elif not voice.get("usable"):
        warnings.append(
            f"The Pi voice cannot be heard ({voice.get('error') or 'no wearable audio output'})"
            " — only the phone can speak"
        )
    # "starting": the start-up hold is still holding every warning back (2026-10-21);
    # "down" with audio off: nothing can tell the wearer anything.
    if not (guiding and sensors.get("lidar_ok")) or not voice.get("audio_enabled"):
        status = "down"
    elif not ready:
        status = "starting"
    else:
        status = (
            "ok"
            if sensors.get("all_ok")
            and not system.get("power_low")
            and not system.get("hot")
            else "degraded"
        )
    body = {
        "status": status,
        "guiding": guiding,
        "ready": ready,
        "sensors": {
            k: bool(sensors.get(k))
            for k in ("lidar_ok", "radar_ok", "camera_ok", "hailo_ok")
        },
        "radar_down": sensors.get("radar_down", []),
        "warnings": warnings,
        "voice": {
            k: voice.get(k)
            for k in ("audio_enabled", "running", "usable", "output", "error")
        },
        "system": {
            k: system.get(k)
            for k in ("soc_c", "throttled", "power_low", "hot", "readable")
        },
        "guidance_loop": loop,
        "service": service.status(),
    }
    return JSONResponse(body, status_code=200 if status in ("ok", "degraded") else 503)


@app.get("/api/status")
async def api_status():
    """Get overall system statphus"""
    from utils.shared_state import get_fused_sensor_state
    from utils.environment_detection import get_environment_state
    from utils.traffic_light_detection import get_traffic_detector

    state = get_fused_sensor_state()
    env_state = get_environment_state()
    crossing = get_traffic_detector()

    return {
        "status": "running",
        "sensors": state.get("sensors_active", {}),
        "navigation": state.get("navigation", {}).get("current_action", "idle"),
        "environment": {
            "condition": env_state.get("condition", "unknown"),
            "visibility": env_state.get("visibility", 1.0),
        },
        "crossing": {
            "state": crossing.crossing_state.value,
            "intersection_type": crossing.intersection_type,
        },
    }


def _get_fallback_html():
    """Fallback HTML if template not found"""
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Blind Assistance System</title>
        <style>
            body { font-family: system-ui; background: #0a0a0a; color: #fff;
                display: flex; justify-content: center; align-items: center;
                height: 100vh; margin: 0; }
            .container { text-align: center; max-width: 600px; }
            h1 { color: #0f8; }
            .api-links { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 20px; }
            a { color: #4d96ff; padding: 10px; background: #1a1a2e; border-radius: 8px;
                text-decoration: none; display: block; }
            a:hover { background: #2a2a4e; }
            .new { border: 1px solid #22c55e; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🦯 Blind Assistance System</h1>
            <p>Template not found. API endpoints available:</p>
            <div class="api-links">
                <a href="/camera/stats">📷 Camera Stats</a>
                <a href="/lidar/config">📡 LiDAR Config</a>
                <a href="/navigation/status">🧭 Navigation</a>
                <a href="/environment/status">🌤️ Environment</a>
                <a href="/crossing/status" class="new">🚦 Road Crossing</a>
                <a href="/crossing/safety" class="new">✅ Crossing Safety</a>
                <a href="/crossing/lights" class="new">🚥 Traffic Lights</a>
                <a href="/health">❤️ Health Check</a>
            </div>
        </div>
        
    </body>
    </html>
"""


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        app,
        host=SERVER_HOST,
        port=SERVER_PORT,
        access_log=False,  # silence per-request access logs (GET ... 200 OK)
        log_level="warning",  # only surface warnings/errors from uvicorn itself
        # One open stream must not hold the teardown (config, NEW-052).
        timeout_graceful_shutdown=SERVER_GRACEFUL_SHUTDOWN_S,
    )
