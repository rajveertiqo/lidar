"""Expiring phone audio readiness, reported by the local BLE bridge."""

import threading
import time

REPORT_TTL_S = 3.0
_lock = threading.Lock()
_last = None
_connected = False
_audio_ready = False


def report(connected: bool, audio_ready: bool) -> None:
    global _last, _connected, _audio_ready
    with _lock:
        _last = time.monotonic()
        _connected = bool(connected)
        _audio_ready = bool(connected and audio_ready)


def status() -> dict:
    with _lock:
        age = None if _last is None else time.monotonic() - _last
        current = age is not None and 0 <= age <= REPORT_TTL_S
        return {
            "connected": bool(current and _connected),
            "usable": bool(current and _audio_ready),
            "stale": not current,
        }
