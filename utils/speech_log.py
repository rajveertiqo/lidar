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
_open_failed_at: Optional[float] = None  # monotonic: the last file that could not be made
_OPEN_RETRY_S = 60.0


def _elapsed(now: float) -> str:
    """+MM:SS.s since the first line."""
    secs = max(0.0, now - (_started or now))
    return f"+{int(secs // 60):02d}:{secs % 60:04.1f}"


def _open(now: float) -> Optional[str]:
    """Create this run's file on the FIRST utterance, not at import. Made and written
    before any old one is pruned, and after a failure not tried again for _OPEN_RETRY_S: on
    a full card every sentence made an empty file and deleted an old log (scan 2026-10-23)."""
    global _path, _bytes, _open_failed_at
    if _path is not None:
        return _path
    if not SPEECH_LOG_TO_FILE:
        return None  # in memory only (config.SPEECH_LOG_TO_FILE)
    mono = time.monotonic()
    if _open_failed_at is not None and mono - _open_failed_at < _OPEN_RETRY_S:
        return None
    path = None
    try:
        os.makedirs(SPEECH_LOG_DIR, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(SPEECH_LOG_DIR, f"speech_{stamp}.log")
        with open(path, "w", encoding="utf-8") as f:
            f.write(_HEADER)
    except Exception:
        _open_failed_at = mono
        if path is not None:
            try:
                os.remove(path)  # no empty file left behind
            except OSError:
                pass
        return None
    _path, _bytes, _open_failed_at = path, len(_HEADER), None
    _prune_old_runs()
    return _path


def _prune_old_runs() -> None:
    """Keep the newest SPEECH_LOG_KEEP_RUNS logs, this run's included."""
    try:
        runs = sorted(
            n
            for n in os.listdir(SPEECH_LOG_DIR)
            if n.startswith("speech_") and n.endswith(".log")
        )
        for name in runs[: max(0, len(runs) - SPEECH_LOG_KEEP_RUNS)]:
            os.remove(os.path.join(SPEECH_LOG_DIR, name))
    except Exception:
        pass


def _one_line(value: Any) -> str:
    """`value` as ONE printable line (audit pass 2, NEW-023)."""
    text = "" if value is None else str(value)
    return " ".join("".join(c if c.isprintable() else " " for c in text).split())


def _timing_text(timing: Dict[str, Any]) -> str:
    """ "scan 45 ms old · decided in 4 ms · voice +80 ms · front 3.6 ft" — what is known."""
    parts = []
    for key, fmt in (
        ("scan_age_ms", "scan {:.0f} ms old"),
        ("camera_age_ms", "camera {:.0f} ms old"),
        ("decide_ms", "decided in {:.0f} ms"),
        ("voice_start_ms", "voice +{:.0f} ms"),
        ("front_ft", "front {:.1f} ft"),
    ):
        value = timing.get(key)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            parts.append(fmt.format(value))
    return " · ".join(parts)


def _note_start(rec: Dict[str, Any], event: str, t_ev: float) -> None:
    """The Pi voice started this sentence: its delay after admission, into its timing."""
    if event == "started" and t_ev >= rec["t"]:
        rec.setdefault("timing", {})["voice_start_ms"] = round((t_ev - rec["t"]) * 1000)


def say(
    source: str,
    text: str,
    level: str = "",
    detail: str = "",
    now: Optional[float] = None,
    cue_seq: Optional[int] = None,
    via: str = "",
    timing: Optional[Dict[str, Any]] = None,
) -> None:
    """Record one utterance. `timing`: how late it was — scan_age_ms, camera_age_ms,
    decide_ms, front_ft (unified_guidance); voice_start_ms is added when the Pi voice starts
    it (2026-10-30)."""
    global _bytes, _full, _started
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
            if _started is None:
                _started = now  # the first line, file or not: else every one is +00:00.0
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
                "timing": dict(timing or {}),
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
                    _note_start(rec, event, t_ev)
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
            if rec["timing"]:
                line += f"   [{_timing_text(rec['timing'])}]"
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
            _note_start(rec, event, now)
            if event == "started" and "voice_start_ms" in rec.get("timing", {}):
                after = f"+{rec['timing']['voice_start_ms']} ms after admitted"
                detail = f"{detail}; {after}" if detail else after
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
                    "timing": dict(r.get("timing") or {}),
                }
                for r in rows
            ],
        }


def latency_summary() -> Dict[str, Any]:
    """How late this run's sentences were (say(timing=...)): median and worst of each part,
    for urgent sentences and for all, and how far ahead a block was when its warning went.
    For /guidance/latency, to check a field walk against the bench (2026-10-30)."""
    with _lock:
        rows = [(r["level"], dict(r.get("timing") or {})) for r in _recent]

    def stats(values):
        v = sorted(x for x in values if isinstance(x, (int, float)))
        if not v:
            return None
        return {"median": v[len(v) // 2], "worst": v[-1], "n": len(v)}

    out: Dict[str, Any] = {}
    for group, keep in (("urgent", lambda lvl: lvl == "urgent"), ("all", lambda lvl: True)):
        timings = [t for lvl, t in rows if t and keep(lvl)]
        summary: Dict[str, Any] = {"sentences": len(timings)}
        for key in ("scan_age_ms", "camera_age_ms", "decide_ms", "voice_start_ms"):
            s = stats(t.get(key) for t in timings)
            if s:
                summary[key] = s
        # Nearer is later: the worst is the nearest (WALK_BLOCK_FT is where it should come).
        ft = sorted(t["front_ft"] for t in timings if isinstance(t.get("front_ft"), (int, float)))
        if ft:
            summary["front_ft"] = {"median": ft[len(ft) // 2], "nearest": ft[0], "n": len(ft)}
        out[group] = summary
    return out


def reset() -> None:
    """Start a new file. Used by the tests; a running rig never calls it."""
    global _path, _started, _bytes, _full, _open_failed_at
    with _lock:
        _path, _started, _bytes, _full, _open_failed_at = None, None, 0, False, None
        _recent.clear()
        _counts.clear()
        _by_seq.clear()
        _early.clear()
