"""Radar API routes — the 6 x A121 glass-detection array."""

import json
import time
from pathlib import Path

from fastapi import APIRouter, Depends, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import HTMLResponse, JSONResponse

import config as _config
from config import (
    GUIDANCE_AWARE_MM,
    GUIDANCE_DANGER_MM,
    GUIDANCE_SAFE_MM,
    GUIDANCE_WARNING_MM,
    RADAR_AGREE_MM,
    RADAR_BAUDRATE,
    RADAR_ENABLED,
    RADAR_END_MM,
    RADAR_GLASS_GAP_MM,
    RADAR_MAX_LAG_S,
    RADAR_BEAM_H_DEG,
    RADAR_BEARING_OFFSET_DEG,
    RADAR_BEARING_STEP_DEG,
    RADAR_PORT_EXCLUDE,
    RADAR_PRIORITY,
    RADAR_PROFILE,
    RADAR_SECTORS,
    RADAR_STALE_S,
    RADAR_START_MM,
    RADAR_UPDATE_RATE,
    radar_board_key,
    radar_sector_label,
    radar_sector_start,
)
from utils.radar_calibration import (
    CalibrationRejected,
    CalibrationSaveError,
    board_keys,
    cardinal_sources,
    get_bearings,
    ring_slots,
    status,
    validate_request,
)
from utils.access import require_admin
from utils.radar_utils import (
    apply_calibration,
    discovery_info,
    restart_radar,
    restart_status,
)
from utils.walk_policy import _mm_to_ft
from utils.sensor_fusion import build_hazards
from utils.shared_state import get_camera_state, get_lidar_state, get_radar_state

router = APIRouter(prefix="/radar", tags=["Radar"])


_TEMPLATES_DIR = Path(__file__).resolve().parent.parent / "templates"

# Largest POST /radar/calibrate body read, in bytes (config).
_CALIBRATE_MAX_BODY_BYTES = int(_config.RADAR_CALIBRATE_MAX_BODY_BYTES)


@router.get("/map")
async def radar_map():
    """Everything the live map page needs, in FEET, in one round trip."""
    snap = get_radar_state()
    sectors = snap.get("sectors", {}) or {}
    now = time.time()

    placed, unplaced = {}, []
    for b in snap.get("boards", []) or []:
        sec = sectors.get(b["position"]) or {}
        last = sec.get("last_update")
        age = (now - last) if last else None
        row = {
            "serial": b["serial"],
            "hub_port": b.get("hub_port"),
            "key": b["position"],
            "bearing": b.get("bearing"),
            "online": bool(sec.get("online")),
            "degraded": bool(sec.get("degraded")),
            "stale": bool(sec.get("stale")),
            "lag_s": sec.get("lag_s"),
            "age_s": round(age, 2) if age is not None else None,
            "ft": _mm_to_ft(sec.get("min_dist")),
            "peaks_ft": [
                f for f in (_mm_to_ft(x) for x in (sec.get("peaks_mm") or [])) if f
            ],
            "error": sec.get("error"),
        }
        if b.get("bearing") is None:
            row["reason"] = "bearing not measured"
            unplaced.append(row)
        else:
            placed[float(b["bearing"])] = row

    slots = [
        {
            "bearing": b,
            "key": radar_board_key(b),
            "sector": radar_sector_start(b),
            "sector_label": radar_sector_label(b),
            "board": placed.get(b),
        }
        for b in ring_slots()
    ]
    known = {s["bearing"] for s in slots}
    # A board measured OFF the intended grid still gets its own wedge.
    slots += [
        {
            "bearing": bg,
            "key": r["key"],
            "sector": radar_sector_start(bg),
            "sector_label": radar_sector_label(bg),
            "board": r,
        }
        for bg, r in placed.items()
        if bg not in known
    ]
    slots.sort(key=lambda s: s["bearing"])

    srcs = cardinal_sources()
    cardinals = {
        c: {
            "ft": _mm_to_ft((sectors.get(c) or {}).get("min_dist")),
            "online": bool((sectors.get(c) or {}).get("online")),
            "degraded": bool((sectors.get(c) or {}).get("degraded")),
            "sources": list(srcs.get(c) or ()),
        }
        for c in RADAR_SECTORS
    }

    restart = restart_status()
    blind = [c for c in RADAR_SECTORS if not cardinals[c]["online"]]
    restart["directions_without_radar"] = blind if restart["in_progress"] else []
    restart["age_s"] = (
        round(now - restart["since"], 1)
        if restart["in_progress"] and restart["since"]
        else None
    )

    return {
        "enabled": RADAR_ENABLED,
        "now": now,
        "restart": restart,
        "geometry": {
            "beam_deg": RADAR_BEAM_H_DEG,
            "step_deg": RADAR_BEARING_STEP_DEG,
            "offset_deg": RADAR_BEARING_OFFSET_DEG,
            # Positive = blind slit between neighbours; negative = overlap.
            "gap_deg": round(RADAR_BEARING_STEP_DEG - RADAR_BEAM_H_DEG, 1),
            "start_ft": round(RADAR_START_MM / 304.8, 2),
            "end_ft": round(RADAR_END_MM / 304.8, 2),
        },
        "zones_ft": {
            "danger": round(GUIDANCE_DANGER_MM / 304.8, 2),
            "warning": round(GUIDANCE_WARNING_MM / 304.8, 2),
            "safe": round(GUIDANCE_SAFE_MM / 304.8, 2),
            "aware": round(GUIDANCE_AWARE_MM / 304.8, 2),
        },
        "slots": slots,
        "unplaced": unplaced,
        "cardinals": cardinals,
        "summary": {
            **status(),
            "connected": len(snap.get("boards", []) or []),
            "live": sum(1 for s in slots if (s["board"] or {}).get("online")),
            "restarting": restart["in_progress"],
            "directions_without_radar": blind,
        },
        # Offered to the calibrate UI as the legal bearings for this mount.
        "ring": list(ring_slots()),
    }


def _refuse(error: str, status_code: int = 422) -> JSONResponse:
    """A refused calibration: nothing was recorded."""
    return JSONResponse({"ok": False, "error": error}, status_code=status_code)


class _DuplicateKey(ValueError):
    """A JSON object named the same key twice."""


def _refuse_duplicate_keys(pairs):
    """object_pairs_hook for json.loads: a repeated key is an error."""
    out = {}
    for key, value in pairs:
        if key in out:
            raise _DuplicateKey(key)
        out[key] = value
    return out


async def _read_body(request: Request, limit: int):
    """The raw body, or None if it is larger than `limit` bytes."""
    declared = request.headers.get("content-length")
    if declared is not None:
        try:
            if int(declared) > limit:
                return None
        except ValueError:
            return None
    buf = bytearray()
    async for chunk in request.stream():
        buf += chunk
        if len(buf) > limit:
            return None
    return bytes(buf)


@router.post("/calibrate", dependencies=[Depends(require_admin)])
async def calibrate(request: Request):
    """Record measured bearings and make them take effect -- live if it can."""
    raw = await _read_body(request, _CALIBRATE_MAX_BODY_BYTES)
    if raw is None:
        return _refuse(
            f"request body larger than {_CALIBRATE_MAX_BODY_BYTES} bytes", 413
        )
    try:
        payload = (
            json.loads(raw, object_pairs_hook=_refuse_duplicate_keys) if raw else None
        )
    except _DuplicateKey as exc:
        return _refuse(
            f"duplicate key {str(exc.args[0])[:64]!r} in the request — "
            f"send each board once; nothing was recorded"
        )
    except (ValueError, RecursionError):
        return _refuse("request body is not valid JSON")
    if not isinstance(payload, dict):
        return _refuse("expected {'bearings': {serial: degrees}}")

    discovered = [e.get("serial") for e in discovery_info().get("found") or []]
    connected = [b.get("serial") for b in get_radar_state().get("boards") or []]
    try:
        clean = validate_request(
            payload.get("bearings"), discovered=discovered, connected=connected
        )
    except CalibrationRejected as exc:
        return _refuse(str(exc))

    try:
        result = await run_in_threadpool(apply_calibration, clean)
    except CalibrationSaveError as exc:
        return _refuse(str(exc), 500)
    except ValueError as exc:  # two boards on one slot
        return _refuse(str(exc))

    bearings = result["bearings"]
    out = {
        "ok": True,
        "bearings": bearings,
        "measured": sum(v is not None for v in bearings.values()),
        "cardinal_sources": {k: list(v) for k, v in cardinal_sources().items()},
        "restarting": bool(result["restarting"]),
        "applied": result["applied"],
        "rekeyed": result["rekeyed"],
    }
    if result["restarting"]:
        rs = restart_status()
        out["restart"] = {
            "since": rs["since"],
            "reason": result.get("reason"),
            "directions_without_radar": list(RADAR_SECTORS),
        }
    return out


@router.get("/movers")
async def radar_movers():
    """Echoes closing in by themselves, and the one only the radar hears coming at the user (utils/radar_movers)."""
    from utils.radar_movers import get_radar_movers

    return get_radar_movers().snapshot()


@router.get("/view", response_class=HTMLResponse)
async def radar_view():
    """The live map page itself."""
    f = _TEMPLATES_DIR / "radar_map.html"
    if not f.exists():
        return HTMLResponse("<h1>radar_map.html missing</h1>", status_code=404)
    return HTMLResponse(f.read_text(encoding="utf-8"))


@router.get("/state")
async def state():
    """Live per-direction radar readings."""
    return get_radar_state()


@router.get("/config")
async def config():
    return {
        "enabled": RADAR_ENABLED,
        "sectors": list(RADAR_SECTORS),
        "range_mm": {"start": RADAR_START_MM, "end": RADAR_END_MM},
        "baudrate": RADAR_BAUDRATE,
        "bearings": get_bearings(),
        "board_keys": list(board_keys()),
        "cardinal_sources": {k: list(v) for k, v in cardinal_sources().items()},
        "zones_mm": {
            "danger": GUIDANCE_DANGER_MM,
            "warning": GUIDANCE_WARNING_MM,
            "safe": GUIDANCE_SAFE_MM,
            "aware": GUIDANCE_AWARE_MM,
        },
        # A client that prints "offline" should know the rule it is rendering.
        "timing": {
            "update_rate_hz": RADAR_UPDATE_RATE,
            "stale_s": RADAR_STALE_S,
            "max_lag_s": RADAR_MAX_LAG_S,
        },
        "fusion": {
            "radar_priority": RADAR_PRIORITY,
            "agree_mm": RADAR_AGREE_MM,
            "glass_gap_mm": RADAR_GLASS_GAP_MM,
        },
    }


@router.get("/boards")
async def boards():
    """Which board is on which port, and its measured bearing."""
    snap = get_radar_state()
    found = snap.get("boards", [])
    sectors = snap.get("sectors", {})
    live = {p for p, s in sectors.items() if s.get("online")}
    return {
        "boards": [
            {
                **b,
                "live": b["position"] in live,
                "degraded": bool((sectors.get(b["position"]) or {}).get("degraded")),
                "lag_s": (sectors.get(b["position"]) or {}).get("lag_s"),
            }
            for b in found
        ],
        "connected": len(found),
        "expected": len(get_bearings()),
        "missing": snap.get("missing", []),
        "unverified": [b["serial"] for b in found if b.get("provisional")],
        "refused": _refused_boards(
            discovery_info().get("found") or [],
            streaming={b.get("serial") for b in found},
        ),
    }


@router.get("/comparison")
def comparison():
    """Side-by-side LiDAR vs radar per direction — the glass-detection view."""
    lidar = get_lidar_state()
    radar = get_radar_state()
    hazards = {h["sector"]: h for h in build_hazards(lidar, get_camera_state(), radar)}

    from utils.sensor_fusion import lidar_is_fresh

    lidar_age = time.time() - (lidar.get("last_update") or 0)
    lidar_stale = not lidar_is_fresh(lidar)

    rows = []
    for sector in RADAR_SECTORS:
        lidar_sec = lidar.get("sectors", {}).get(sector) or {}
        radar_sec = radar.get("sectors", {}).get(sector) or {}
        hazard = hazards.get(sector)
        online = bool(radar_sec.get("online"))
        rows.append(
            {
                "sector": sector,
                "lidar_mm": None if lidar_stale else lidar_sec.get("min_dist"),
                "radar_mm": radar_sec.get("min_dist"),
                "radar_online": online,
                "radar_stale": bool(radar_sec.get("stale")),
                "radar_degraded": bool(radar_sec.get("degraded")),
                "radar_lag_s": radar_sec.get("lag_s"),
                "fused_mm": hazard["distance_mm"] if hazard else None,
                "range_source": hazard["range_source"] if hazard else None,
                "glass_suspected": (
                    bool(hazard["glass_suspected"])
                    if hazard and not lidar_stale
                    else False
                ),
            }
        )
    return {
        "comparison": rows,
        "enabled": RADAR_ENABLED,
        "lidar_stale": lidar_stale,
        "lidar_age_s": round(lidar_age, 2),
        "lidar_updated": lidar.get("last_update", 0),
        "radar_updated": radar.get("last_update", 0),
    }


def _refused_boards(discovered, streaming):
    """Discovered boards that are not streaming, each with the reason."""
    return [
        {
            "serial": e.get("serial"),
            "device": e.get("device"),
            "hub_port": e.get("hub_port"),
            "usb_path": e.get("usb_path"),
            "rss_version": e.get("rss"),
            "online": False,
            "phase": "refused",
            "reason": (
                e.get("refused")
                or "bearing not measured — calibrate it before it can be used"
            ),
            "reconnects": 0,
            "peaks_mm": [],
        }
        for e in discovered
        if e.get("serial") not in streaming
    ]


def _board_rows():
    """Every board keyed by SERIAL, with its hardware telemetry."""
    snap = get_radar_state()
    sectors = snap.get("sectors", {}) or {}
    now = time.time()

    rows = {}
    for b in snap.get("boards", []) or []:
        sec = sectors.get(b["position"]) or {}
        last = sec.get("last_update")
        age = (now - last) if last else None
        rows[b["serial"]] = {
            "serial": b["serial"],
            "device": b.get("device"),
            "hub_port": b.get("hub_port"),
            "usb_path": b.get("usb_path"),
            "rss_version": b.get("rss"),
            "key": b.get("position"),
            "bearing": b.get("bearing"),
            "provisional": bool(b.get("provisional")),
            "phase": sec.get("phase"),
            "online": bool(sec.get("online")),
            "degraded": bool(sec.get("degraded")),
            "stale": bool(sec.get("stale")),  # the row's own verdict: see /map
            "mm": sec.get("min_dist"),
            "ft": _mm_to_ft(sec.get("min_dist")),
            "peaks_mm": list(sec.get("peaks_mm") or []),
            "strength_db": sec.get("strength_db"),
            "temperature_c": sec.get("temperature"),
            "near_edge": sec.get("near_edge"),
            "lag_s": sec.get("lag_s"),
            "measured_hz": sec.get("measured_hz"),
            "frames": sec.get("frames"),
            "reconnects": sec.get("reconnects") or 0,
            "age_s": round(age, 2) if age is not None else None,
            "error": sec.get("error"),
        }
    return rows, snap, now


@router.get("/diagnostics")
async def diagnostics():
    """Per-board hardware telemetry, discovery result, and the ring — one trip."""
    rows, snap, now = _board_rows()
    disco = discovery_info()

    refused = _refused_boards(disco["found"], streaming=set(rows))

    bearings = get_bearings()
    placed = {s: b for s, b in bearings.items() if b is not None}
    return {
        "enabled": RADAR_ENABLED,
        "boards": rows,
        "refused": refused,
        "discovery": disco,
        "missing": snap.get("missing", []),
        "config": {
            "baudrate": RADAR_BAUDRATE,
            "update_rate_hz": RADAR_UPDATE_RATE,
            "start_mm": RADAR_START_MM,
            "end_mm": RADAR_END_MM,
            "start_ft": round(RADAR_START_MM / 304.8, 2),
            "end_ft": round(RADAR_END_MM / 304.8, 2),
            "profile": RADAR_PROFILE,
            "max_lag_s": RADAR_MAX_LAG_S,
            "stale_s": RADAR_STALE_S,
            "beam_deg": RADAR_BEAM_H_DEG,
            "step_deg": RADAR_BEARING_STEP_DEG,
            "offset_deg": RADAR_BEARING_OFFSET_DEG,
            "excluded_ports": list(RADAR_PORT_EXCLUDE),
        },
        "layout": {
            "positions": placed,  # serial -> measured bearing
            "ring": list(ring_slots()),  # the intended mount
            # `centre` equals `start` here — see the convention note above.
            "sectors": [
                {
                    "start": b,
                    "centre": b,
                    "key": radar_board_key(b),
                    "label": f"{b:.0f}\u00b0",
                }
                for b in ring_slots()
            ],
            "unmeasured": [s for s, b in bearings.items() if b is None],
        },
        "cardinal_sources": {k: list(v) for k, v in cardinal_sources().items()},
        "now": now,
    }


@router.get("/health")
async def radar_health():
    """One-line verdict on the array, for a shell check or a glance."""
    rows, snap, _ = _board_rows()
    disco = discovery_info()
    found = [e["serial"] for e in disco["found"]]

    live = sorted(k for k, b in rows.items() if b["online"])
    lagging = sorted(k for k, b in rows.items() if b["degraded"])
    silent = sorted(set(found) - set(live))
    unmeasured = [s for s, b in get_bearings().items() if b is None]

    sectors = snap.get("sectors", {}) or {}
    blind = sorted(c for c in RADAR_SECTORS if not (sectors.get(c) or {}).get("online"))

    return {
        "ok": bool(
            RADAR_ENABLED
            and found
            and len(live) == len(found)
            and not lagging
            and not blind
        ),
        "enabled": RADAR_ENABLED,
        "discovered": len(found),
        "streaming": len(live),
        "lagging": lagging,
        "not_streaming": silent,
        "unmeasured": unmeasured,
        "blind_cardinals": blind,
        "discovery_error": disco["error"],
    }


@router.post("/rediscover", dependencies=[Depends(require_admin)])
async def rediscover():
    """Stop every reader and rediscover — for after replugging a cable."""
    restart_radar(reason="rediscover")
    return {"ok": True, "restarting": True}


@router.get("/test", response_class=HTMLResponse)
async def radar_test():
    """The hardware test dashboard — one card per board."""
    f = _TEMPLATES_DIR / "radar_test.html"
    if not f.exists():
        return HTMLResponse("<h1>radar_test.html missing</h1>", status_code=404)
    return HTMLResponse(f.read_text(encoding="utf-8"))
