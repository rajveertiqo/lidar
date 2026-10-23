"""Unified walking guidance — ONE voice for the user, and a closed vocabulary."""

import threading
import time
from typing import Dict, Any, Optional, Tuple

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

# The simple-guidance step failing tick after tick (2026-10-21): until then each failure
# was a log line, the wearer heard nothing, and /health and the watchdog saw a live loop.
_fail_lock = threading.Lock()
_fail_count = 0
_fail_error = ""
_fallback_said_at: Optional[float] = None


def _current_seq() -> int:
    """The newest cue_seq, unchanged, for a tick that announces nothing. Not 0: the phone
    bridge reads a cue_seq that goes backwards as a restart of this service, and re-alerts
    the last sentence (2026-10-22, fuzz test)."""
    try:
        from utils import simple_guidance

        return int(simple_guidance.state()["cue_seq"])
    except Exception:
        return 0


def _note_guidance(exc: Optional[Exception]) -> str:
    """Count one tick of the guidance step; the fault once GUIDANCE_FAIL_TICKS fail in a row, else ""."""
    from config import GUIDANCE_FAIL_TICKS

    global _fail_count, _fail_error
    with _fail_lock:
        if exc is None:
            _fail_count, _fail_error = 0, ""
            return ""
        _fail_count += 1
        _fail_error = f"{type(exc).__name__}: {exc}"
        return _fail_error if _fail_count >= GUIDANCE_FAIL_TICKS else ""


def guidance_fault() -> str:
    """Why the guidance step is failing, or "" — for the loop's heartbeat (pi_voice) and /health."""
    from config import GUIDANCE_FAIL_TICKS

    with _fail_lock:
        return _fail_error if _fail_count >= GUIDANCE_FAIL_TICKS else ""


def _fault_result(why: str) -> Dict[str, Any]:
    """What to say while the guidance step is failing: "Guidance stopped, use your cane"."""
    global _fallback_said_at
    try:
        from utils import simple_guidance

        return simple_guidance.fault_notice(why)
    except Exception as e:
        _warn("fault notice", e)
    # The gate itself is broken: straight to the Pi voice, without a cue for the phone.
    from config import SIMPLE_OFFLINE_REPEAT_S

    text = "Guidance stopped, use your cane"
    try:
        from utils.simple_guidance import SENSORS_OFFLINE as text
    except Exception:
        pass
    mono = time.monotonic()
    if _fallback_said_at is None or mono - _fallback_said_at >= SIMPLE_OFFLINE_REPEAT_S:
        _fallback_said_at = mono
        try:
            from utils import pi_voice

            pi_voice.speak(text, urgent=True, seq=None)
        except Exception:
            pass
    return {
        "instruction": text,
        "action": "sensors_offline",
        "speak": False,
        "cue_seq": _current_seq(),
        "quiet_reason": "said directly by the Pi voice",
        "rule": "offline",
        "reason": f"the guidance step is failing: {why}",
        "side": None,
        "raw": {},
    }


# The guidance loop's newest result: (result, time.monotonic() when made).
_latest_lock = threading.Lock()
_latest: Optional[Tuple[Dict[str, Any], float]] = None


def latest_guidance() -> Dict[str, Any]:
    """The newest result of the guidance loop (pi_voice), for the web pages."""
    from config import GUIDANCE_RESULT_STALE_S

    with _latest_lock:
        latest = _latest
    age = None if latest is None else time.monotonic() - latest[1]
    if latest is not None and age <= GUIDANCE_RESULT_STALE_S:
        return {**latest[0], "loop": {"age_s": round(age, 2), "stale": False}}

    from utils import pi_voice, simple_guidance

    why = (
        "the guidance loop has not produced a result yet"
        if latest is None
        else f"the guidance loop is not updating (newest result {age:.1f} s old)"
    )
    error = pi_voice.loop_status().get("error")
    if error:
        why += f": {error}"
    seq = simple_guidance.state()["cue_seq"]  # unchanged: nothing to announce
    return {
        "notify": {
            "speak": False,
            "cue_seq": seq,
            "instruction": "",
            "level": "advisory",
            "source": None,
            "reason": why,
            "quiet_reason": why,
        },
        "path": {
            "instruction": "",
            "action": "unknown",
            "rule": "unavailable",
            "reason": why,
            "speak": False,
            "cue_seq": seq,
            "raw": {},
        },
        "walk": {},
        "obstacles": {"level": "unknown", "summary": "", "count": 0, "items": []},
        "crossing": False,
        "at_crossing": False,
        "sector_ft": {},
        "loop": {"age_s": None if age is None else round(age, 2), "stale": True},
    }


def _startup_ready() -> Dict[str, Any]:
    """Whether the sensors have settled enough to speak. Never raises."""
    try:
        from utils import startup_gate

        return startup_gate.check()
    except Exception:
        return {"ready": True, "cue": "", "quiet_reason": ""}


def _record(
    now: float, mono: float, inputs: Dict[str, Any], simple: Dict[str, Any]
) -> None:
    """Hand one tick to the walk recorder, if it is recording (utils/walk_recorder)."""
    try:
        from utils.walk_recorder import get_recorder

        rec = get_recorder()
        if rec.active:
            rec.tick(
                now,
                mono,
                inputs,
                {
                    k: simple.get(k)
                    for k in (
                        "instruction",
                        "speak",
                        "cue_seq",
                        "quiet_reason",
                        "rule",
                        "reason",
                        "mover_shadow",
                        "route_shadow",
                    )
                },
            )
    except Exception as e:
        _warn("walk recorder", e)


def get_unified_guidance() -> Dict[str, Any]:
    """Compute one merged guidance instruction from live sensor state, and commit it to speech — the guidance loop's tick; readers use latest_guidance()."""
    from utils.unified_navigation_system import decisionMaker
    from utils.sensor_fusion import fresh_camera_state, merge_navigation_inputs
    from utils.shared_state import (
        get_lidar_state,
        get_radar_state,
        get_camera_state,
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
            "left": seen.get("left", {}).get("detections", []),
            "back": seen.get("back", {}).get("detections", []),
            "context": {},
        }
        from utils.sensor_fusion import lidar_is_fresh

        if lidar_is_fresh(lidar):
            nav = decisionMaker(sample)
        else:
            nav = {
                "action": "no_lidar_data",
                "crossing_mode": False,
                "urgent_message": "No LiDAR data — the way ahead is unknown.",
            }
    except Exception as e:
        nav = {}
        _warn("nav decision", e)

    at_crossing = False
    try:
        from utils.unified_navigation_system import (
            detect_traffic_lights_from_detections,
        )

        front_lights = detect_traffic_lights_from_detections(
            {"front": (seen.get("front") or {}).get("detections", [])}
        )
        at_crossing = bool(front_lights["directions"]["front"]["has_light"])
    except Exception as e:
        _warn("crossing (front camera)", e)

    path = {
        "clear_ft": (nav.get("cone_distances", {}) or {})
        .get("front", {})
        .get("distance_ft"),
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

    simple = {
        "instruction": "",
        "action": "unknown",
        "speak": False,
        "cue_seq": _current_seq(),
        "quiet_reason": "",
        "rule": "unavailable",
        "reason": "",
        "side": None,
    }
    try:
        from utils import simple_guidance

        ready = _startup_ready()
        motion = None
        try:
            from utils.heading_tracker import get_tracker

            motion = get_tracker().state()
        except Exception as e:
            _warn("heading state", e)
        movers = None
        try:
            from utils.lidar_movers import get_movers

            movers = get_movers().snapshot()
        except Exception as e:
            _warn("lidar movers", e)
        try:
            from config import MOTION_TRACKER_ENABLED

            if MOTION_TRACKER_ENABLED:
                # Kalman tracks (utils/motion_tracker): read here so the walk recorder keeps them.
                from utils.motion_tracker import get_motion_tracker

                movers = {**(movers or {}), "kalman": get_motion_tracker().snapshot()}
        except Exception as e:
            _warn("motion tracker", e)
        try:
            from config import RADAR_MOVERS_ENABLED

            if RADAR_MOVERS_ENABLED:
                # Fed here, once a tick: this loop is the only caller (latest_guidance).
                from utils.radar_movers import get_radar_movers

                radar_movers = get_radar_movers()
                radar_movers.update(
                    get_radar_state(),
                    (movers or {}).get("blobs"),
                    (movers or {}).get("ego"),
                    turn_dps=(motion or {}).get("yaw_rate_dps"),
                )
                movers = {**(movers or {}), "radar": radar_movers.snapshot()}
        except Exception as e:
            _warn("radar movers", e)
        radar_now = get_radar_state()
        system = None
        try:
            from utils import pi_health

            system = pi_health.read()
        except Exception as e:
            _warn("pi health", e)
        hold = None if ready["ready"] else (ready["quiet_reason"] or "scanning")
        tick_t, tick_mono = time.time(), time.monotonic()
        simple = simple_guidance.guidance(
            walk_decision,
            lidar,
            now=tick_t,
            hold=hold,
            hold_lidar_absent=bool(ready.get("lidar_absent")),
            at_crossing=at_crossing,
            radar_state=radar_now,
            camera_state=camera,
            motion=motion,
            movers=movers,
            system=system,
        )
        _note_guidance(None)
        _record(
            tick_t,
            tick_mono,
            {
                "walk_decision": walk_decision,
                "lidar": lidar,
                "radar": radar_now,
                "camera": camera,
                "motion": motion,
                "movers": movers,
                "hold": hold,
                "hold_lidar_absent": bool(ready.get("lidar_absent")),
                "at_crossing": at_crossing,
            },
            simple,
        )
    except Exception as e:
        _warn("simple guidance", e)
        fault = _note_guidance(e)
        if fault:
            simple = _fault_result(fault)

    if simple.get("mover_shadow"):
        # Not spoken (LIDAR_MOVERS_SPEAK False): logged so a test walk can judge it.
        try:
            from utils import speech_log, test_console

            speech_log.say(
                source="lidar-movers",
                text=f"would say: {simple['mover_shadow']}",
                level="silent",
                detail=simple.get("mover_shadow_reason", ""),
                via="none",
            )
            test_console.would_say(
                simple["mover_shadow"], simple.get("mover_shadow_reason", "")
            )
        except Exception:
            pass

    if simple.get("route_shadow"):
        # Not spoken (ROUTE_PLANNER_SPEAK False): logged so a test walk can judge it.
        try:
            from utils import speech_log, test_console

            speech_log.say(
                source="route-planner",
                text=f"would say: {simple['route_shadow']}",
                level="silent",
                detail=simple.get("route_shadow_reason", ""),
                via="none",
            )
            test_console.would_say(
                simple["route_shadow"], simple.get("route_shadow_reason", "")
            )
        except Exception:
            pass

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
    elif drift.get("glass") and drift.get("ttc_s") == (drift["glass"] or {}).get(
        "ttc_s"
    ):
        g = drift["glass"]
        where = "ahead" if g["direction"] == "front" else f"on your {g['direction']}"
        note = f"radar sees glass {where} — the laser looks straight through it"
    elif veer.get("veering"):
        note = veer.get("plain", "")
    elif hazard and drift.get("ttc_s") is not None:
        note = (
            f"{hazard.get('object', 'obstacle')} {drift.get('ttc_phrase', '')}"
        ).strip()
    else:
        note = plain.get("line", "")

    path.update(
        {
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
        }
    )

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
    key = (
        (walk_decision.get("action"), walk_decision.get("direction"))
        if turning
        else None
    )
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

    obstacles = {
        "level": "clear",
        "summary": "No obstacles nearby",
        "count": 0,
        "items": [],
    }
    try:
        from utils.sensor_fusion import fuse_guidance

        fused = fuse_guidance()
        hz = fused.get("hazards", []) or []
        obstacles = {
            "level": fused.get("action", "clear"),  # clear|proceed_slow|caution|stop
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
    urgent = simple.get("rule") in (
        "blocked",
        "vehicle_approaching",
        "mover_approaching",
    )

    say = bool(simple.get("speak"))
    notify_seq = simple.get("cue_seq", 0)
    notify_quiet = simple.get("quiet_reason", "")

    try:
        from utils import pi_voice

        voice = pi_voice.status()
    except Exception:
        voice = {
            "enabled": False,
            "running": False,
            "usable": False,
            "sink_usable": False,
        }

    if say:
        handed = "none"
        try:
            from utils import pi_voice

            handed = pi_voice.speak(
                instruction,
                urgent=urgent,
                seq=notify_seq,
                sink_usable=bool(voice.get("sink_usable")),
            )
        except Exception:
            pass  # the voice must never cost the instruction
        try:
            from utils import test_console

            test_console.spoken(
                instruction,
                str(path.get("reason") or ""),
                urgent=urgent,
                raw=path.get("raw"),
                voice=voice,
            )
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
            pass  # a log must never cost the instruction it records

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
    result = {
        "notify": notify,
        "path": path,
        "walk": walk_decision,
        "obstacles": obstacles,
        # Legacy: any camera's traffic light (decisionMaker). Display only.
        "crossing": bool(nav.get("crossing_mode", False)),
        "at_crossing": at_crossing,
        # All 8 sectors — threat awareness is reported at the finer resolution.
        "sector_ft": {
            s: sectors.get(s, {}).get("distance_ft") for s in LIDAR_SECTOR_ORDER
        },
    }
    global _latest
    with _latest_lock:
        _latest = (result, time.monotonic())
    return result
