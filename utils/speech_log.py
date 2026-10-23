"""What the device ADMITTED for speech, with a timestamp, and what the Pi voice then did with it."""

import os
import threading
import time
from typing import Any, Dict, List, Optional

from config import (
    SPEECH_LOG_TO_FILE,
    SPEECH_LOG_DIR,
    SPEECH_LOG_ENABLED,
    SPEECH_LOG_KEEP_LINES,
    SPEECH_LOG_KEEP_RUNS,
    SPEECH_LOG_MAX_BYTES,
)

_HEADER = (
    "# Shappe speech log — one line per sentence the speech gate ADMITTED and\n"
    "# handed to delivery, written at that moment (not by polling).\n"
    "# ADMITTED IS NOT HEARD: `via pi` = handed to the Pi voice, `via phone` =\n"
    "# left to the dashboard's screen reader (not observable here). What the Pi\n"
    "# voice then did follows as '#   cue N  <outcome>' lines; 'completed' means\n"
    "# espeak-ng finished, not that the wearer heard it.\n"
    "#\n"
    "# wall time                elapsed   source            level    cue / via / what was admitted\n"
)

_lock = threading.Lock()
_path: Optional[str] = None
_started: Optional[float] = None
_bytes = 0
_full = False
_recent: List[Dict[str, Any]] = []
_counts: Dict[str, int] = {}  # lines this run, by source
_by_seq: Dict[int, Dict[str, Any]] = {}  # cue_seq -> its record in _recent
_early: Dict[int, List[tuple]] = {}
_EARLY_MAX = 16


def _elapsed(now: float) -> str:
    """+MM:SS.s since the first line."""
    secs = max(0.0, now - (_started or now))
    return f"+{int(secs // 60):02d}:{secs % 60:04.1f}"


def _open(now: float) -> Optional[str]:
    """Create this run's file on the FIRST utterance, not at import."""
    global _path, _started, _bytes
    if _path is not None:
        return _path
    if not SPEECH_LOG_TO_FILE:
        return None  # in memory only (config.SPEECH_LOG_TO_FILE)
    try:
        os.makedirs(SPEECH_LOG_DIR, exist_ok=True)
        _prune_old_runs()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        _path = os.path.join(SPEECH_LOG_DIR, f"speech_{stamp}.log")
        _started = now
        with open(_path, "w", encoding="utf-8") as f:
            f.write(_HEADER)
        _bytes = len(_HEADER)
        return _path
    except Exception:
        _path = None
        return None


def _prune_old_runs() -> None:
    """Keep the newest SPEECH_LOG_KEEP_RUNS - 1 logs, making room for this run's."""
    try:
        runs = sorted(
            n
            for n in os.listdir(SPEECH_LOG_DIR)
            if n.startswith("speech_") and n.endswith(".log")
        )
        for name in runs[: max(0, len(runs) - (SPEECH_LOG_KEEP_RUNS - 1))]:
            os.remove(os.path.join(SPEECH_LOG_DIR, name))
    except Exception:
        pass


def _one_line(value: Any) -> str:
    """`value` as ONE printable line (audit pass 2, NEW-023)."""
    text = "" if value is None else str(value)
    return " ".join("".join(c if c.isprintable() else " " for c in text).split())


def say(
    source: str,
    text: str,
    level: str = "",
    detail: str = "",
    now: Optional[float] = None,
    cue_seq: Optional[int] = None,
    via: str = "",
) -> None:
    """Record one utterance."""
    global _bytes, _full
    source, level, text, detail = (
        _one_line(source),
        _one_line(level),
        _one_line(text),
        _one_line(detail),
    )
    via = _one_line(via)
    if not SPEECH_LOG_ENABLED or not text:
        return

    now = time.time() if now is None else now
    try:
        with _lock:
            rec = {
                "t": now,
                "source": source,
                "level": level,
                "text": text,
                "detail": detail,
                "cue_seq": cue_seq,
                "via": via,
                "outcome": "admitted",
                "outcomes": [],
            }
            _recent.append(rec)
            early = []
            if cue_seq is not None:
                _by_seq[cue_seq] = rec
                early = _early.pop(cue_seq, [])
                for event, t_ev, det in early:
                    rec["outcome"] = event
                    rec["outcomes"].append(
                        {"event": event, "t": round(t_ev, 3), "detail": det}
                    )
            for old in _recent[:-SPEECH_LOG_KEEP_LINES]:
                _by_seq.pop(old.get("cue_seq"), None)
            del _recent[:-SPEECH_LOG_KEEP_LINES]
            _counts[source] = _counts.get(source, 0) + 1

            if _full or _open(now) is None:
                return

            stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
            tag = ""
            if cue_seq is not None:
                tag = f"cue {cue_seq}" + (f" via {via}" if via else "") + "  "
            line = (
                f"{stamp}.{int(now % 1 * 1000):03d}  {_elapsed(now)}  "
                f"{source:<17} {level or '-':<8} {tag}{text}"
            )
            if detail:
                line += f"   ({detail})"
            line += "\n"

            if _bytes + len(line) > SPEECH_LOG_MAX_BYTES:
                note = (
                    "# SPEECH_LOG_MAX_BYTES reached — logging stopped. "
                    "The walk continued; only this file ended.\n"
                )
                with open(_path, "a", encoding="utf-8") as f:
                    f.write(note)
                _full = True
                return

            for event, t_ev, det in early:
                line += (
                    f"#   cue {cue_seq}  {event:<11} {_elapsed(t_ev)}"
                    + (f"  ({det})" if det else "")
                    + "\n"
                )
            with open(_path, "a", encoding="utf-8") as f:
                f.write(line)
            _bytes += len(line)
    except Exception:
        pass


def delivery(
    cue_seq: int,
    event: str,
    text: str = "",
    detail: str = "",
    now: Optional[float] = None,
) -> None:
    """Record what became of cue `cue_seq` after admission (utils/pi_voice)."""
    global _bytes
    if not SPEECH_LOG_ENABLED:
        return
    event, detail = _one_line(event), _one_line(detail)
    now = time.time() if now is None else now
    try:
        with _lock:
            rec = _by_seq.get(cue_seq)
            if rec is None:
                _early.setdefault(cue_seq, []).append((event, now, detail))
                while len(_early) > _EARLY_MAX:
                    _early.pop(next(iter(_early)))
                return
            rec["outcome"] = event
            rec["outcomes"].append(
                {"event": event, "t": round(now, 3), "detail": detail}
            )
            del rec["outcomes"][:-8]
            if _full or _path is None:
                return
            line = (
                f"#   cue {cue_seq}  {event:<11} {_elapsed(now)}"
                + (f"  ({detail})" if detail else "")
                + "\n"
            )
            if _bytes + len(line) > SPEECH_LOG_MAX_BYTES:
                return
            with open(_path, "a", encoding="utf-8") as f:
                f.write(line)
            _bytes += len(line)
    except Exception:
        pass


def recent(limit: int = 50) -> Dict[str, Any]:
    """The last utterances, newest last. For /guidance/log and the dashboard."""
    with _lock:
        rows = _recent[-max(1, limit) :]
        return {
            "enabled": bool(SPEECH_LOG_ENABLED),
            "path": _path,
            "full": _full,
            "count": len(_recent),
            # Every line this run, by source — not just the ones in `lines`.
            "totals": dict(_counts),
            "lines": [
                {
                    "time": time.strftime("%H:%M:%S", time.localtime(r["t"])),
                    "elapsed": _elapsed(r["t"]),
                    "epoch": round(r["t"], 3),
                    "source": r["source"],
                    "level": r["level"],
                    "text": r["text"],
                    "detail": r["detail"],
                    "cue_seq": r.get("cue_seq"),
                    "via": r.get("via", ""),
                    "outcome": r.get("outcome", "admitted"),
                }
                for r in rows
            ],
        }


def reset() -> None:
    """Start a new file. Used by the tests; a running rig never calls it."""
    global _path, _started, _bytes, _full
    with _lock:
        _path, _started, _bytes, _full = None, None, 0, False
        _recent.clear()
        _counts.clear()
        _by_seq.clear()
        _early.clear()
