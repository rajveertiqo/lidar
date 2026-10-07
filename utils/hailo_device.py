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
