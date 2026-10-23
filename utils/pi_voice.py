"""The Pi's own voice — guidance that does not depend on a browser tab."""

import shutil
import subprocess
import threading
import time
from collections import deque
from typing import Any, Dict, Optional

from config import (
    AUDIO_ENABLED,
    PI_VOICE_ALLOW_HDMI,
    PI_VOICE_CMD,
    PI_VOICE_ENABLED,
    PI_VOICE_REAP_WAIT_S,
    PI_VOICE_REPORT_QUEUE_MAX,
    PI_VOICE_TICK_HZ,
)


_OUTPUT_CACHE_S = 5.0
_UNHEARD_SINKS = ("hdmi", "dummy", "null")
_OUTPUT_STALE_S = 3.0 * _OUTPUT_CACHE_S
_output_lock = threading.Lock()
_output: Dict[str, Any] = {"name": None, "usable": False, "checked": 0.0}
_prober: Optional[threading.Thread] = None
_prober_stop = threading.Event()


def _prober_running() -> bool:
    return bool(_prober and _prober.is_alive())


def audio_output() -> Dict[str, Any]:
    """The default audio sink and whether a person can hear it."""
    now = time.monotonic()
    with _output_lock:
        cached = {"name": _output["name"], "usable": _output["usable"]}
        age = now - _output["checked"]
    if _prober_running() or _running:
        if not (-1.0 <= age <= _OUTPUT_STALE_S):  # stale, or a clock step
            return {"name": cached["name"], "usable": False, "stale": True}
        return cached
    if 0.0 <= age < _OUTPUT_CACHE_S:
        return cached
    return probe_audio_output()


def probe_audio_output() -> Dict[str, Any]:
    """Ask WirePlumber (`wpctl inspect @DEFAULT_AUDIO_SINK@`) now, and cache it."""
    name = None
    try:
        out = subprocess.run(
            ["wpctl", "inspect", "@DEFAULT_AUDIO_SINK@"],
            capture_output=True,
            text=True,
            timeout=2,
        ).stdout
        for key in ("node.description", "node.nick", "node.name"):
            for line in out.splitlines():
                if key in line and "=" in line:
                    name = line.split("=", 1)[1].strip().strip('"')
                    break
            if name:
                break
    except Exception:
        name = None
    usable = bool(name) and _wearable(name)
    with _output_lock:
        _output.update(name=name, usable=usable, checked=time.monotonic())
    return {"name": name, "usable": usable}


def _wearable(name: str) -> bool:
    low = name.lower()
    return not any(
        k in low for k in _UNHEARD_SINKS if not (k == "hdmi" and PI_VOICE_ALLOW_HDMI)
    )


def _probe_loop() -> None:
    """Keep the sink answer fresh, off the speaking path."""
    while not _prober_stop.is_set():
        try:
            probe_audio_output()
        except Exception:
            pass
        _prober_stop.wait(_OUTPUT_CACHE_S)


def _start_prober() -> None:
    global _prober
    if _prober_running():
        return
    _prober_stop.clear()
    _prober = threading.Thread(target=_probe_loop, name="audio-sink-probe", daemon=True)
    _prober.start()


def _stop_prober() -> None:
    _prober_stop.set()


_REPORT_QUEUE_MAX = PI_VOICE_REPORT_QUEUE_MAX
_reports: "deque" = deque(maxlen=_REPORT_QUEUE_MAX)
_reports_lock = threading.Lock()
_reports_wake = threading.Event()
_reports_idle = threading.Event()
_reports_idle.set()
_reporter: Optional[threading.Thread] = None


def _report(seq: Optional[int], event: str, text: str, detail: str = "") -> None:
    """Hand one delivery event to the speech log's writer thread."""
    global _reporter
    if seq is None:
        return
    try:
        with _reports_lock:
            _reports.append((seq, event, text, detail))
            _reports_idle.clear()
            if not (_reporter and _reporter.is_alive()):
                _reporter = threading.Thread(
                    target=_drain_reports, name="pi-voice-log", daemon=True
                )
                _reporter.start()
        _reports_wake.set()
    except Exception:
        pass


def _drain_reports() -> None:
    from utils import speech_log

    while True:
        _reports_wake.wait(timeout=5.0)
        _reports_wake.clear()
        while True:
            with _reports_lock:
                if not _reports:
                    _reports_idle.set()
                    break
                seq, event, text, detail = _reports.popleft()
            try:
                speech_log.delivery(seq, event, text=text, detail=detail)
            except Exception:
                pass


def flush_reports(timeout: float = 2.0) -> bool:
    """Wait (bounded) until every queued delivery event is in the log."""
    _reports_wake.set()
    return _reports_idle.wait(timeout)


_SAY_TIMEOUT_S = 8.0


class _Voice:
    """Speaks one sentence at a time; the newest wins, urgent cuts in."""

    def __init__(self):
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._next: Optional[str] = None
        self._next_seq: Optional[int] = None
        self._last_seq: Optional[int] = None
        self._playing_seq: Optional[int] = None
        self._proc: Optional[subprocess.Popen] = None
        self._killed: Optional[subprocess.Popen] = None  # stopped by us, not failed
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.last: str = ""
        self.last_at: float = 0.0
        self.error: str = ""
        self.failing: bool = False
        # Monotonic time an utterance first started with none completing since.
        self._unfinished_since: Optional[float] = None

    @property
    def accepted_seq(self) -> Optional[int]:
        """The newest cue_seq this voice accepted (NEW-031)."""
        return self._last_seq

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="pi-voice", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        with self._lock:
            self._kill()

    def say(self, text: str, urgent: bool = False, seq: Optional[int] = None) -> bool:
        """Queue `text`; True if accepted."""
        if not text:
            return False
        events = []
        with self._lock:
            if seq is not None and self._last_seq is not None and seq <= self._last_seq:
                events.append(
                    (
                        seq,
                        "dropped",
                        text,
                        f"older than cue {self._last_seq}, already accepted",
                    )
                )
                accepted = False
            else:
                if self._next is not None:
                    events.append(
                        (
                            self._next_seq,
                            "replaced",
                            self._next,
                            f"by cue {seq}"
                            if seq is not None
                            else "by a newer sentence",
                        )
                    )
                self._next, self._next_seq = text, seq
                if seq is not None:
                    self._last_seq = seq
                events.append((seq, "queued", text, "urgent" if urgent else ""))
                if urgent:
                    self._kill()
                accepted = True
        if accepted:
            self._wake.set()  # the voice first; the log after (NEW-032)
        for ev in events:
            _report(*ev)
        return accepted

    def _kill(self) -> None:
        proc = self._proc
        if proc is not None and proc.poll() is None:
            self._killed = proc
            try:
                proc.terminate()
            except Exception:
                pass

    def _fail(self, error: str) -> None:
        self.error, self.failing = error, True

    def _run(self) -> None:
        while not self._stop.is_set():
            self._wake.wait(timeout=1.0)
            self._wake.clear()
            with self._lock:
                text, self._next = self._next, None
                seq, self._next_seq = self._next_seq, None
                if not text or self._stop.is_set():
                    continue
                try:
                    proc = subprocess.Popen(
                        list(PI_VOICE_CMD) + [text],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                except Exception as exc:
                    self._fail(f"{type(exc).__name__}: {exc}")
                    started = None
                else:
                    self._proc, self._playing_seq = proc, seq
                    started = time.monotonic()
                    if self._unfinished_since is None:
                        self._unfinished_since = started
            if started is None:
                _report(seq, "failed", text, self.error)
                continue
            _report(seq, "started", text)
            try:
                try:
                    rc = proc.wait(timeout=_SAY_TIMEOUT_S)
                except subprocess.TimeoutExpired:
                    self._fail(f"{PI_VOICE_CMD[0]} hung for {_SAY_TIMEOUT_S:.0f} s")
                    proc.kill()
                    try:
                        proc.wait(timeout=PI_VOICE_REAP_WAIT_S)
                    except Exception:
                        pass
                    _report(seq, "hung", text, self.error)
                    continue
                took = f"{time.monotonic() - started:.1f}s"
                if self._killed is proc:
                    if (
                        self._unfinished_since is not None
                        and time.monotonic() - self._unfinished_since > _SAY_TIMEOUT_S
                    ):
                        self._fail(
                            f"no sentence has finished for "
                            f"{time.monotonic() - self._unfinished_since:.0f} s"
                        )
                    _report(seq, "interrupted", text, f"after {took}")
                    continue
                if rc != 0:
                    self._fail(f"{PI_VOICE_CMD[0]} exited with status {rc}")
                    _report(seq, "failed", text, self.error)
                    continue
                self.last, self.last_at = text, time.time()
                self.error, self.failing = "", False
                self._unfinished_since = None
                _report(seq, "completed", text, took)
            except Exception as exc:
                self._fail(f"{type(exc).__name__}: {exc}")
                _report(seq, "failed", text, self.error)
            finally:
                with self._lock:
                    self._proc, self._playing_seq = None, None


_voice = _Voice()


_loop_thread: Optional[threading.Thread] = None
_loop_stop = threading.Event()
_running = False
_tick_ok_at = 0.0
_tick_error = ""


def _loop() -> None:
    global _tick_ok_at, _tick_error
    from utils import service
    from utils.unified_guidance import get_unified_guidance, guidance_fault

    period = 1.0 / max(PI_VOICE_TICK_HZ, 0.5)
    while not _loop_stop.is_set():
        started = time.monotonic()
        try:
            get_unified_guidance()
            fault = guidance_fault()
            if fault:
                raise RuntimeError(f"guidance step failing: {fault}")
            _tick_ok_at, _tick_error = time.monotonic(), ""  # local: F20
            # Only a tick that guided feeds the watchdog (2026-10-21): a loop that keeps
            # failing is restarted by systemd, not left running silent.
            service.beat("guidance")
        except Exception as exc:
            if str(exc) != _tick_error:
                print(f"[WARN] pi_voice: guidance tick failed: {exc}")
            _tick_error = str(exc)
        _loop_stop.wait(max(0.0, period - (time.monotonic() - started)))


def loop_status() -> Dict[str, Any]:
    """The guidance loop's own health, for /health."""
    age = (time.monotonic() - _tick_ok_at) if _tick_ok_at else None
    return {
        "running": bool(_loop_thread and _loop_thread.is_alive()),
        "last_ok_age_s": None if age is None else round(age, 2),
        "error": _tick_error,
    }


def start() -> None:
    """Start the guidance loop, and the voice if enabled and espeak-ng exists."""
    global _loop_thread, _running
    if _loop_thread and _loop_thread.is_alive():
        return
    # The voice before the loop, with a fresh sink answer, so the first tick's "Starting,
    # please wait" is heard rather than marked unavailable (2026-10-21).
    if AUDIO_ENABLED and PI_VOICE_ENABLED and shutil.which(str(PI_VOICE_CMD[0])):
        probe_audio_output()  # bounded by wpctl's 2 s timeout
        _start_prober()
        _voice.start()
        _running = True
    else:
        _running = False
        _voice.error = (
            "audio switched off in config (AUDIO_ENABLED)"
            if not AUDIO_ENABLED
            else "disabled in config"
            if not PI_VOICE_ENABLED
            else f"{PI_VOICE_CMD[0]} not found"
        )
    _loop_stop.clear()
    _loop_thread = threading.Thread(target=_loop, name="guidance-loop", daemon=True)
    _loop_thread.start()


def stop() -> None:
    global _running
    _loop_stop.set()
    _voice.stop()
    _stop_prober()
    _running = False
    flush_reports(timeout=1.0)


def speak(
    text: str,
    urgent: bool = False,
    seq: Optional[int] = None,
    sink_usable: Optional[bool] = None,
) -> str:
    """Called at the one commit point in unified_guidance."""
    if not AUDIO_ENABLED:
        _report(seq, "muted", text, "audio switched off in config (AUDIO_ENABLED)")
        return "muted"
    if sink_usable is None:
        sink_usable = bool(_running and audio_output()["usable"])
    if not (_running and sink_usable):
        why = (
            "Pi voice not running"
            if not _running
            else "no wearable audio sink — the phone is expected to speak"
        )
        _report(seq, "unavailable", text, why)
        return "unavailable"
    return "queued" if _voice.say(text, urgent, seq) else "dropped"


def accepted_seq() -> Optional[int]:
    """The newest cue_seq the Pi voice accepted, or None (NEW-031)."""
    return _voice.accepted_seq if _running else None


def status() -> Dict[str, Any]:
    out = audio_output() if _running else {"name": None, "usable": False}
    return {
        "enabled": bool(PI_VOICE_ENABLED),
        "audio_enabled": bool(
            AUDIO_ENABLED
        ),  # False: no device, Pi or phone, may speak
        "running": _running,
        "output": out["name"],
        "sink_usable": bool(_running and out["usable"]),
        "sink_stale": bool(out.get("stale")),
        "usable": bool(_running and out["usable"] and not _voice.failing),
        "accepted_seq": _voice.accepted_seq if _running else None,
        "last": _voice.last,
        "error": _voice.error,
    }
