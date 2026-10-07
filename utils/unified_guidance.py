"""Unified walking guidance — ONE voice for the user, and a closed vocabulary."""

import threading
import time
from typing import Dict, Any, Tuple

from config import LIDAR_SECTOR_ORDER

_warn_lock = threading.Lock()
_warned: Dict[str, Tuple[str, float]] = {}
_WARN_REPEAT_S = 60.0


def _warn(layer: str, exc: Exception) -> None:
    msg = f"{type(exc).__name__}: {exc}"
    now = time.time()
    with _warn_lock:
        last = _warned.get(layer)
        if last and last[0] == msg and now - last[1] < _WARN_REPEAT_S:
            return
        _warned[layer] = (msg, now)
    print(f"[WARN] unified guidance: {layer} failed: {msg}")

_turn_lock = threading.Lock()
_last_turn = None

def _startup_ready() -> Dict[str, Any]:
    """Whether the sensors have settled enough to speak. Never raises."""
    try:
        from utils import startup_gate
        return startup_gate.check()
    except Exception:
        return {"ready": True, "cue": "", "quiet_reason": ""}


def get_unified_guidance() -> Dict[str, Any]:
    """Compute one merged guidance instruction from live sensor state."""
    from utils.unified_navigation_system import decisionMaker
    from utils.sensor_fusion import fresh_camera_state, merge_navigation_inputs
    from utils.shared_state import (
        get_lidar_state, get_radar_state, get_camera_state,
    )

    lidar, camera, seen = {}, {}, {}
    try:
        lidar = get_lidar_state()
        camera = get_camera_state()
        sectors, cones = merge_navigation_inputs(
            lidar.get("sectors", {}), lidar.get("cones", {}), get_radar_state()
        )
        seen = fresh_camera_state(camera)
        sample = {
            "precomputed_sectors": sectors,
            "precomputed_cones": cones,
            "front": seen.get("front", {}).get("detections", []),
            "right": seen.get("right", {}).get("detections", []),
            "left":  seen.get("left", {}).get("detections", []),
            "back":  seen.get("back", {}).get("detections", []),
            "context": {},
        }
        from utils.sensor_fusion import lidar_is_fresh
        if lidar_is_fresh(lidar):
            nav = decisionMaker(sample)
        else:
            nav = {"action": "no_lidar_data", "crossing_mode": False,
                   "urgent_message": "No LiDAR data — the way ahead is unknown."}
    except Exception as e:
        nav = {}
        _warn("nav decision", e)

    at_crossing = False
    try:
        from utils.unified_navigation_system import detect_traffic_lights_from_detections
        front_lights = detect_traffic_lights_from_detections(
            {"front": (seen.get("front") or {}).get("detections", [])})
        at_crossing = bool(front_lights["directions"]["front"]["has_light"])
    except Exception as e:
        _warn("crossing (front camera)", e)

    path = {
        "clear_ft": (nav.get("cone_distances", {}) or {})
                    .get("front", {}).get("distance_ft"),
    }

    try:
        from utils import walk_policy
        walk_decision = walk_policy.decide(lidar, get_radar_state())
    except Exception as e:
        walk_decision = {"action": "unknown", "instruction": "—", "units": "ft"}
        _warn("walk policy", e)

    if not nav.get("crossing_mode") and walk_decision.get("action") != "unknown":
        path["front_ft"] = walk_decision.get("front_ft")
        path["glass_ahead"] = bool(walk_decision.get("glass_ahead"))

    dirs = walk_decision.get("directions") or {}
    if dirs:
        path["clearance_ft"] = {d: v.get("distance_ft") for d, v in dirs.items()}
        path["fusion"] = {d: v.get("fusion") for d, v in dirs.items()}
        path["readings"] = {
            d: {
                "lidar_ft": v.get("lidar_ft"),
                "radar_ft": v.get("radar_ft"),
                "lidar_measured": v.get("lidar_measured"),
                "radar_measured": v.get("radar_measured"),
                "fused_ft": v.get("distance_ft"),
                "glass": v.get("glass"),
                "unknown": v.get("unknown"),
                "chosen": d == walk_decision.get("direction"),
            }
            for d, v in dirs.items()
        }

    drift = {}
    try:
        from utils.shared_state import get_drift_state
        drift = get_drift_state()
    except Exception as e:
        _warn("drift state", e)

    simple = {"instruction": "", "action": "unknown", "speak": False,
              "cue_seq": 0, "quiet_reason": "", "rule": "unavailable",
              "reason": "", "side": None}
    try:
        from utils import simple_guidance
        ready = _startup_ready()
        motion = None
        try:
            from utils.heading_tracker import get_tracker
            motion = get_tracker().state()
        except Exception as e:
            _warn("heading state", e)
        simple = simple_guidance.guidance(
            walk_decision, lidar,
            hold=None if ready["ready"] else (ready["quiet_reason"] or "scanning"),
            hold_lidar_absent=bool(ready.get("lidar_absent")),
            at_crossing=at_crossing,
            radar_state=get_radar_state(),
            camera_state=camera,
            motion=motion)
    except Exception as e:
        _warn("simple guidance", e)

    path["action"] = simple.get("action", "unknown")
    path["instruction"] = simple.get("instruction", "")
    path["source"] = "simple-guidance"
    path["reason"] = simple.get("reason", "")
    path["rule"] = simple.get("rule", "")
    path["turn_side"] = simple.get("side")
    path["obstacle"] = simple.get("obstacle")
    path["raw"] = simple.get("raw") or {}

    hazard = drift.get("hazard") or {}
    veer = drift.get("veer") or {}
    plain = drift.get("plain") or {}
    drift_stale = bool(drift.get("stale", True))

    if drift_stale:
        note = "waiting for enough scans to judge your heading"
    elif drift.get("glass") and drift.get("ttc_s") == (drift["glass"] or {}).get("ttc_s"):
        g = drift["glass"]
        where = "ahead" if g["direction"] == "front" else f"on your {g['direction']}"
        note = f"radar sees glass {where} — the laser looks straight through it"
    elif veer.get("veering"):
        note = veer.get("plain", "")
    elif hazard and drift.get("ttc_s") is not None:
        note = (f"{hazard.get('object', 'obstacle')} "
                f"{drift.get('ttc_phrase', '')}").strip()
    else:
        note = plain.get("line", "")

    path.update({
        "ttc_s": drift.get("ttc_s"),
        "ttc_phrase": drift.get("ttc_phrase", ""),
        # How the user is actually moving, in plain English.
        "status": plain.get("summary", ""),
        "sensing": plain.get("sensing", ""),
        "note": note,
        # What drift is converging on, if anything — named and placed.
        "hazard": hazard or None,
        "drift_level": drift.get("level", "clear"),
        "drift_instruction": drift.get("instruction", ""),
        "stale": drift_stale,
    })

    path["drift"] = {
        "action": drift.get("action", "hold_course"),
        "cue_seq": drift.get("cue_seq", 0),
        "level": drift.get("level", "clear"),
        "instruction": drift.get("instruction", ""),
        "reason": drift.get("reason", ""),
        "ttc_s": drift.get("ttc_s"),
        "hazard": hazard or None,
        "veer": veer or None,
        "glass": drift.get("glass"),
        "heading": drift.get("heading"),
        "plain": plain or None,
        "stale": drift_stale,
    }

    global _last_turn
    turning = str(walk_decision.get("action", "")).startswith(("turn_", "stop"))
    key = (walk_decision.get("action"), walk_decision.get("direction")) if turning else None
    with _turn_lock:
        new_turn = key is not None and key != _last_turn
        _last_turn = key
    if new_turn:
        try:
            from utils.heading_tracker import get_tracker
            from utils import drift_guidance
            get_tracker().reset_reference()
            drift_guidance.reset_state()
        except Exception as e:
            _warn("heading reset", e)

    obstacles = {"level": "clear", "summary": "No obstacles nearby",
                 "count": 0, "items": []}
    try:
        from utils.sensor_fusion import fuse_guidance
        fused = fuse_guidance()
        hz = fused.get("hazards", []) or []
        obstacles = {
            "level": fused.get("action", "clear"),   # clear|proceed_slow|caution|stop
            "summary": fused.get("primary", ""),
            "count": len(hz),
            "items": [
                {
                    "object": h.get("object"),
                    "sector": h.get("sector"),
                    "clock": h.get("clock"),
                    "distance_ft": h.get("distance_ft"),
                    "urgency": h.get("urgency"),
                    "source": h.get("source"),
                    "shape": h.get("shape"),
                }
                for h in hz[:6]
            ],
        }
    except Exception as e:
        _warn("obstacle list", e)

    instruction = simple.get("instruction") or ""
    urgent = simple.get("rule") in ("blocked", "vehicle_approaching")

    say = bool(simple.get("speak"))
    notify_seq = simple.get("cue_seq", 0)
    notify_quiet = simple.get("quiet_reason", "")

    try:
        from utils import pi_voice
        voice = pi_voice.status()
    except Exception:
        voice = {"enabled": False, "running": False, "usable": False,
                 "sink_usable": False}

    if say:
        handed = "none"
        try:
            from utils import pi_voice
            handed = pi_voice.speak(instruction, urgent=urgent, seq=notify_seq,
                                    sink_usable=bool(voice.get("sink_usable")))
        except Exception:
            pass          # the voice must never cost the instruction
        try:
            from utils import test_console
            test_console.spoken(instruction, str(path.get("reason") or ""),
                                urgent=urgent, raw=path.get("raw"), voice=voice)
        except Exception:
            pass
        try:
            from utils import speech_log
            speech_log.say(
                source=str(path.get("source") or "guidance"),
                text=instruction,
                level="urgent" if urgent else "advisory",
                detail=str(path.get("reason") or path.get("note") or ""),
                cue_seq=notify_seq,
                via={"queued": "pi", "unavailable": "phone"}.get(handed, "none"),
            )
        except Exception:
            pass          # a log must never cost the instruction it records

    try:
        from utils import pi_voice
        voice = {**voice, "accepted_seq": pi_voice.accepted_seq()}
    except Exception:
        voice = {**voice, "accepted_seq": None}

    # ONE decision about whether to talk, mirrored where consumers already look.
    path["speak"] = say
    path["cue_seq"] = notify_seq
    notify = {
        "speak": say,
        "cue_seq": notify_seq,
        "pi_voice": voice,
        "instruction": simple.get("announced") or (instruction if say else ""),
        "level": "urgent" if urgent else "advisory",
        "source": path.get("source"),
        "reason": path.get("reason") or path.get("note") or "",
        "quiet_reason": notify_quiet,
    }

    # A little context for the UI.
    sectors = nav.get("sector_distances", {}) or {}
    return {
        "notify": notify,
        "path": path,
        "walk": walk_decision,
        "obstacles": obstacles,
        # Legacy: any camera's traffic light (decisionMaker). Display only.
        "crossing": bool(nav.get("crossing_mode", False)),
        "at_crossing": at_crossing,
        # All 8 sectors — threat awareness is reported at the finer resolution.
        "sector_ft": {s: sectors.get(s, {}).get("distance_ft")
                      for s in LIDAR_SECTOR_ORDER},
    }
