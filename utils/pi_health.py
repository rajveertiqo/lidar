"""The Pi itself: SoC temperature and the firmware's under-voltage / throttle flags.

Read on a background thread every PI_HEALTH_POLL_S, so a slow `vcgencmd` never holds up
the guidance loop or /health (2026-10-21; until then only the BLE prototype read them).
"""

import subprocess
import threading
import time
from typing import Any, Dict, Optional

from config import PI_HEALTH_POLL_S, PI_HOT_C, PI_HOT_CLEAR_C, PI_UNDERVOLTAGE_HOLD_S

_THERMAL_ZONE = "/sys/class/thermal/thermal_zone0/temp"
# `vcgencmd get_throttled`, the "now" bits (the sticky "since boot" ones are 16-19).
UV_NOW, CAPPED_NOW, THROTTLED_NOW, SOFT_TEMP_NOW = 1 << 0, 1 << 1, 1 << 2, 1 << 3

_lock = threading.Lock()
_state: Dict[str, Any] = {"soc_c": None, "throttled": None, "power_low": False,
                          "hot": False, "readable": False, "checked_mono": None}
_uv_at: Optional[float] = None
_thread: Optional[threading.Thread] = None


def _read_soc() -> Optional[float]:
    try:
        with open(_THERMAL_ZONE) as fh:
            return round(int(fh.read().strip()) / 1000.0, 1)
    except (OSError, ValueError):
        return None


def _read_throttled() -> Optional[int]:
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                             text=True, timeout=2).stdout
        return int(out.strip().split("=", 1)[1], 16)
    except Exception:
        return None


def poll_once(mono: Optional[float] = None, soc: Any = False,
              throttled: Any = False) -> Dict[str, Any]:
    """Read both sources once (or take `soc` / `throttled` as given, for tests) and update the state."""
    global _uv_at
    mono = time.monotonic() if mono is None else mono
    soc = _read_soc() if soc is False else soc
    throttled = _read_throttled() if throttled is False else throttled
    with _lock:
        if throttled is not None and throttled & UV_NOW:
            _uv_at = mono
        power_low = _uv_at is not None and 0.0 <= mono - _uv_at <= PI_UNDERVOLTAGE_HOLD_S
        # Capped / throttled are set by under-voltage as well as by heat (0x50005 is the
        # classic weak supply): they mean heat only while the supply is fine. The soft
        # temperature limit is heat alone.
        thr = throttled or 0
        thermal = bool(thr & SOFT_TEMP_NOW) or (
            bool(thr & (CAPPED_NOW | THROTTLED_NOW)) and not power_low)
        hot = _state["hot"]
        if thermal or (soc is not None and soc >= PI_HOT_C):
            hot = True
        elif soc is None or soc < PI_HOT_CLEAR_C:
            hot = False
        _state.update(soc_c=soc, throttled=throttled, power_low=power_low, hot=hot,
                      readable=soc is not None or throttled is not None,
                      checked_mono=mono)
        return dict(_state)


def _loop() -> None:
    while True:
        try:
            poll_once()
        except Exception:
            pass
        time.sleep(PI_HEALTH_POLL_S)


def read() -> Dict[str, Any]:
    """The newest reading (the poller starts on first use). `readable` is False before the first poll, when the reading has stopped coming, or on a machine with neither source."""
    global _thread
    with _lock:
        if _thread is None:
            _thread = threading.Thread(target=_loop, name="pi-health", daemon=True)
            _thread.start()
        out = dict(_state)
    at = out.get("checked_mono")
    if at is None or time.monotonic() - at > 5.0 * PI_HEALTH_POLL_S:
        out["readable"] = False
    return out
