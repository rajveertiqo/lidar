"""Shared Hailo VDevice."""

import os
import threading

os.environ.setdefault("HAILORT_LOGGER_PATH", "NONE")

_lock = threading.Lock()
_vdevice = None


def get_shared_vdevice():
    """Return the process-wide singleton VDevice, creating it on first use."""
    global _vdevice
    with _lock:
        if _vdevice is None:
            from hailo_platform import VDevice
            vdevice = VDevice()          # raises -> _vdevice stays None
            _vdevice = vdevice
        return _vdevice


def release_shared_vdevice() -> bool:
    """Forget the shared VDevice and release it, so the next get_shared_vdevice() opens a fresh one (HAILO_VDEVICE_RESET). Only once no engine on it can still be running (pi_camera_utils._restart_npu): a native call into a released device can crash the process."""
    global _vdevice
    with _lock:
        vdevice, _vdevice = _vdevice, None
    if vdevice is None:
        return True
    try:
        vdevice.release()
        return True
    except Exception as e:
        print(f"[WARN] Hailo VDevice release failed: {type(e).__name__}: {e}")
        return False
