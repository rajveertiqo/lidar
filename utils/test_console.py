"""The terminal, made for field testing."""

import sys
import threading
import time
from collections import deque
from typing import Any, Dict, Optional

from config import CONSOLE_ENABLED, CONSOLE_HEARTBEAT_S, CONSOLE_STATUS_S

_TTY = sys.stdout.isatty()
_RESET, _DIM, _BOLD = ("\033[0m", "\033[2m", "\033[1m") if _TTY else ("", "", "")
_RED, _YEL, _GRN, _CYN = (
    ("\033[31m", "\033[33m", "\033[32m", "\033[36m") if _TTY else ("", "", "", "")
)

_lock = threading.Lock()
_said = deque(maxlen=500)  # timestamps of spoken sentences
_last_status: Optional[str] = None
_last_heartbeat = 0.0
_thread: Optional[threading.Thread] = None
_stop = threading.Event()


def _now() -> str:
    t = time.time()
    return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int(t % 1 * 10)}"


def _emit(line: str) -> None:
    if not CONSOLE_ENABLED:
        return
    try:
        print(line, flush=True)
    except Exception:
        pass


def _evidence(raw: Dict[str, Any]) -> str:
    raw = raw or {}
    bits = []
    ft = raw.get("front_ft")
    bits.append(
        f"front {ft:4.1f} ft" if isinstance(ft, (int, float)) else "front  open  "
    )
    bits.append("wall ✓" if raw.get("wall") else "wall ·")
    bits.append("glass ✓" if raw.get("glass") else "glass ·")
    if raw.get("object"):
        bits.append(f"cam {raw['object']}")
    steer = raw.get("steer") or {}
    if steer:
        if steer.get("kind") == "between":
            bits.append("gap " + (steer.get("turn") or "straight"))
        else:
            bits.append(
                f"past {steer.get('word')} ({steer.get('where')}) -> {steer.get('turn')}"
            )
    veh = raw.get("vehicles") or {}
    if veh.get("class"):
        if veh.get("approaching"):
            ttc = veh.get("ttc_s")
            bits.append(
                f"{veh['class']} APPROACHING {veh.get('side')}"
                + (f" ~{ttc:.0f}s" if isinstance(ttc, (int, float)) else "")
            )
        elif veh.get("stopped"):
            bits.append(
                f"{veh['class']} stopped"
                + (" (you still)" if raw.get("user_still") else "")
            )
    if raw.get("drift"):
        bits.append(f"drift {raw['drift']}")
    if raw.get("close"):
        bits.append(f"near {raw['close']}")
    for side in ("left", "right"):
        ft = (raw.get("beside") or {}).get(side)
        if isinstance(ft, (int, float)):
            rate = (raw.get("closing") or {}).get(side)
            move = (
                f" ↘{rate:.1f}ft/s"
                if isinstance(rate, (int, float)) and rate > 0.05
                else ""
            )
            bits.append(f"{side[0].upper()}-wall {ft:.1f}ft{move}")
    return "  ".join(bits)


def spoken(
    text: str,
    reason: str = "",
    urgent: bool = False,
    raw: Optional[Dict[str, Any]] = None,
    voice: Optional[Dict[str, Any]] = None,
    timing: Optional[Dict[str, Any]] = None,
) -> None:
    """One line per sentence actually committed. Called at the commit point. `timing`: how
    late it is (unified_guidance._timing), shown as "⏱ scan 45 ms"."""
    with _lock:
        _said.append(time.time())
    colour = _RED if urgent else _YEL if text not in ("Walk straight",) else _GRN
    muted = bool(voice) and voice.get("audio_enabled") is False
    who = (
        "🔇 audio off"
        if muted
        else f"🎧 {voice.get('output')}"
        if voice and voice.get("usable")
        else "📱 phone"
    )
    age = (timing or {}).get("scan_age_ms")
    late = f"  ⏱ scan {age:.0f} ms" if isinstance(age, (int, float)) else ""
    _emit(
        f"{_DIM}{_now()}{_RESET}  {'🔇' if muted else '🔊'} {colour}{_BOLD}{text:<46}{_RESET} "
        f"{_DIM}{_evidence(raw)}  ({reason}){late}  {who}{_RESET}"
    )


def would_say(text: str, reason: str = "", source: str = "LiDAR movers") -> None:
    """A silent trial's sentence, held but not spoken (LIDAR_MOVERS_SPEAK, ROUTE_PLANNER_SPEAK)."""
    _emit(
        f"{_DIM}{_now()}{_RESET}  👁   {_CYN}would say: {text:<35}{_RESET} "
        f"{_DIM}{source}, silent  ({reason}){_RESET}"
    )


def mark(kind: str, note: str = "") -> None:
    """A tester's verdict from the dashboard."""
    from utils.speech_log import _one_line

    kind, note = _one_line(kind), _one_line(note)
    icon = {"right": "✓", "wrong": "✗", "missed": "⚠", "note": "📝"}.get(kind, "•")
    _emit(
        f"{_DIM}{_now()}{_RESET}  {_CYN}{_BOLD}{icon} TESTER: {kind.upper()}"
        f"{_RESET}{_CYN}{('  — ' + note) if note else ''}{_RESET}"
    )


def said_last_minute() -> int:
    cutoff = time.time() - 60.0
    with _lock:
        return sum(1 for t in _said if t >= cutoff)


def _status_line() -> str:
    """Sensors, startup and voice, in one line."""
    try:
        from utils.sensor_fusion import get_system_health

        h = get_system_health() or {}
    except Exception:
        h = {}
    try:
        from utils import startup_gate

        ready = startup_gate.state().get("ready")
    except Exception:
        ready = None
    try:
        from utils import pi_voice

        v = pi_voice.status()
    except Exception:
        v = {}
    mark_ok = lambda ok: f"{_GRN}✓{_RESET}" if ok else f"{_RED}✗{_RESET}"
    down = h.get("radar_down") or []
    radar = (
        f"{_GRN}✓{_RESET}"
        if h.get("radar_ok") and not down
        else f"{_YEL}partial ({', '.join(map(str, down))} down){_RESET}"
        if h.get("radar_ok")
        else f"{_RED}✗{_RESET}"
    )
    voice = (
        "🔇 audio off"
        if v.get("audio_enabled") is False
        else f"🎧 {v.get('output')}"
        if v.get("usable")
        else f"📱 phone ({v.get('output') or 'no Pi output'})"
        if v.get("running")
        else "📱 phone"
    )
    start = "ready" if ready else "scanning" if ready is False else "?"
    return (
        f"LiDAR {mark_ok(h.get('lidar_ok'))}  radar {radar}  "
        f"camera {mark_ok(h.get('camera_ok'))}  NPU {mark_ok(h.get('hailo_ok'))}"
        f"  │ startup {start}  │ voice {voice}"
    )


def _watch() -> None:
    global _last_status, _last_heartbeat
    while not _stop.wait(CONSOLE_STATUS_S):
        try:
            line = _status_line()
            now = time.time()
            if line != _last_status:
                tag = "●" if _last_status is None else f"{_YEL}⚑ CHANGED{_RESET}"
                _emit(f"{_DIM}{_now()}{_RESET}  {tag} {line}")
                _last_status, _last_heartbeat = line, now
            elif now - _last_heartbeat >= CONSOLE_HEARTBEAT_S:
                _emit(
                    f"{_DIM}{_now()}  · {line}  │ said {said_last_minute()} "
                    f"in the last minute{_RESET}"
                )
                _last_heartbeat = now
        except Exception:
            pass


def start() -> None:
    global _thread
    if not CONSOLE_ENABLED or (_thread and _thread.is_alive()):
        return
    _stop.clear()
    _emit(
        f"{_DIM}{_now()}{_RESET}  {_BOLD}Guidance console{_RESET} — one line per "
        f"sentence spoken; health on every change"
    )
    _thread = threading.Thread(target=_watch, name="test-console", daemon=True)
    _thread.start()


def stop() -> None:
    _stop.set()
