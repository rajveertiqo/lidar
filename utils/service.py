"""Running as a service — systemd notify, heartbeats, and a shutdown backstop."""

import os
import socket
import sys
import threading
import time
from typing import Dict, Iterable, Optional

from config import SERVICE_HEARTBEAT_MAX_AGE_S, SHUTDOWN_TIMEOUT_S

_started = time.monotonic()
_lock = threading.RLock()
_beats: Dict[str, float] = {}
_watchdog_stop = threading.Event()
_watchdog_thread: Optional[threading.Thread] = None
_withheld: Optional[str] = None  # why pings are being withheld, if they are
_shutdown_armed = False


def notify(state: str) -> bool:
    """Send one sd_notify message. False outside systemd or on any error."""
    addr = os.environ.get("NOTIFY_SOCKET")
    if not addr:
        return False
    if addr[0] == "@":  # abstract namespace
        addr = "\0" + addr[1:]
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
            sock.connect(addr)
            sock.sendall(state.encode("utf-8"))
        return True
    except OSError:
        return False


def under_systemd() -> bool:
    return bool(os.environ.get("NOTIFY_SOCKET"))


def _watchdog_interval() -> Optional[float]:
    """Seconds between pings, or None when systemd is not watching this PID."""
    usec = os.environ.get("WATCHDOG_USEC")
    pid = os.environ.get("WATCHDOG_PID")
    if not usec or (pid and pid != str(os.getpid())):
        return None
    try:
        # A third of the timeout, the usual margin: one late ping is survivable.
        return max(0.5, int(usec) / 1e6 / 3.0)
    except ValueError:
        return None


def beat(name: str) -> None:
    """Record that `name` is alive. Cheap; call it on every tick."""
    with _lock:
        _beats[name] = time.monotonic()


def heartbeats(now: Optional[float] = None) -> Dict[str, Optional[float]]:
    """Age in seconds of every heartbeat seen so far (monotonic clock)."""
    now = time.monotonic() if now is None else now
    with _lock:
        return {name: round(now - t, 2) for name, t in _beats.items()}


def _stale(required: Iterable[str], now: float) -> Optional[str]:
    """The first required heartbeat that is missing or too old, as a reason."""
    ages = heartbeats(now)
    for name in required:
        age = ages.get(name)
        if age is None:
            return f"{name} has not reported"
        if age > SERVICE_HEARTBEAT_MAX_AGE_S:
            return f"{name} silent for {age:.0f}s"
    return None


def start_watchdog(required: Iterable[str]) -> None:
    """Tell systemd the app is up, and ping its watchdog while `required` beat."""
    global _watchdog_thread
    notify("READY=1")
    interval = _watchdog_interval()
    if interval is None or (_watchdog_thread and _watchdog_thread.is_alive()):
        return
    required = tuple(required)
    _watchdog_stop.clear()

    def run():
        global _withheld
        while not _watchdog_stop.wait(interval):
            why = _stale(required, time.monotonic())
            if why is None:
                notify("WATCHDOG=1")
                if _withheld:
                    print(
                        f"[SERVICE] watchdog pings resumed ({_withheld} has recovered)"
                    )
                _withheld = None
            elif why != _withheld:
                # Said once per cause, not once per interval.
                print(
                    f"[SERVICE] withholding the watchdog ping — {why}; systemd "
                    f"restarts the app if this lasts"
                )
                _withheld = why

    _watchdog_thread = threading.Thread(
        target=run, name="systemd-watchdog", daemon=True
    )
    _watchdog_thread.start()


def stopping() -> None:
    """Shutdown has begun: tell systemd, stop pinging, arm the backstop."""
    notify("STOPPING=1")
    _watchdog_stop.set()
    arm_shutdown_backstop()


def arm_shutdown_backstop(timeout_s: float = SHUTDOWN_TIMEOUT_S) -> None:
    """End the process if shutdown has not finished in timeout_s."""
    global _shutdown_armed
    with _lock:
        if _shutdown_armed:
            return
        _shutdown_armed = True

    def bail():
        time.sleep(timeout_s)
        print(
            f"[SERVICE] shutdown still wedged after {timeout_s:.0f}s — forcing exit "
            f"so the camera and serial ports are released",
            file=sys.stderr,
            flush=True,
        )
        os._exit(1)

    threading.Thread(target=bail, name="shutdown-backstop", daemon=True).start()


def status() -> Dict[str, object]:
    return {
        "uptime_s": round(time.monotonic() - _started, 1),
        "under_systemd": under_systemd(),
        "watchdog": _watchdog_interval() is not None,
        "watchdog_withheld": _withheld,
        "heartbeats_age_s": heartbeats(),
    }
