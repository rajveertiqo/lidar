"""The ToF depth camera (Arducam, 240 x 180 distances on CSI): a capture thread keeping its
last TOF_BUFFER_FRAMES frames (2026-10-13). Started only with TOF_ENABLED.

Frames are kept raw and placed on the front camera's picture (utils/depth_fusion.py) only
when a detection pass asks for the one nearest its picture, so a frame nobody uses costs
nothing. The SDK (pip install ArducamDepthCamera) is imported only here; both of its API
generations are handled (ArducamCamera with Connection/FrameType/Control, and the older
TOFConnect/TOFOutput/TOFControl one, which reads metres instead of millimetres).
"""

import collections
import threading
import time
from typing import Any, Dict, Optional, Tuple

import numpy as np

from config import (
    TOF_BUFFER_FRAMES, TOF_CSI_PORT, TOF_DEPTH_IS_RADIAL, TOF_ENABLED,
    TOF_FRAME_TIMEOUT_MS, TOF_RANGE_MM, TOF_RETRY_BACKOFF_S,
)

_NO_FRAMES_S = 3.0          # this long without a frame: the sensor is reopened


class _Frame:
    __slots__ = ("t", "depth", "ir", "placed")

    def __init__(self, t: float, depth: np.ndarray, ir: Optional[np.ndarray]):
        self.t, self.depth, self.ir, self.placed = t, depth, ir, None


_lock = threading.Lock()
_frames: "collections.deque[_Frame]" = collections.deque(maxlen=TOF_BUFFER_FRAMES)
_stop = threading.Event()
_thread: Optional[threading.Thread] = None
_status: Dict[str, Any] = {"running": False, "ok": False, "error": None, "frames": 0,
                           "opened_at": None, "sdk": None}
_arrivals: "collections.deque[float]" = collections.deque(maxlen=30)


class _Sdk:
    """The sensor, through whichever ArducamDepthCamera API is installed."""

    def __init__(self):
        import ArducamDepthCamera as ac
        self.cam = ac.ArducamCamera()
        self.new_api = hasattr(ac, "Connection")
        conn = ac.Connection.CSI if self.new_api else ac.TOFConnect.CSI
        ret = self.cam.open(conn, TOF_CSI_PORT)
        if ret not in (0, None):
            raise RuntimeError(f"cannot open the ToF on CSI port {TOF_CSI_PORT} (error {ret})")
        ret = self.cam.start(ac.FrameType.DEPTH if self.new_api else ac.TOFOutput.DEPTH)
        if ret not in (0, None):
            self.cam.close()
            raise RuntimeError(f"the ToF would not start (error {ret})")
        try:                             # the new API takes millimetres, the old one metres
            if self.new_api:
                self.cam.setControl(ac.Control.RANGE, int(TOF_RANGE_MM))
            else:
                self.cam.setControl(ac.TOFControl.RANGE, int(round(TOF_RANGE_MM / 1000.0)))
        except Exception:
            pass                         # it keeps its own default range
        self.name = "ArducamDepthCamera " + ("(Connection API)" if self.new_api else "(TOFConnect API)")

    @staticmethod
    def _field(frame, attr: str, getter: str):
        val = getattr(frame, attr, None)
        if val is None and hasattr(frame, getter):
            val = getattr(frame, getter)()
        return None if val is None else np.array(val, dtype=np.float32, copy=True)

    def read(self) -> Optional[Tuple[np.ndarray, Optional[np.ndarray]]]:
        """(depth in mm, brightness or confidence) of the next frame, or None on a timeout."""
        frame = self.cam.requestFrame(TOF_FRAME_TIMEOUT_MS)
        if frame is None:
            return None
        try:
            depth = self._field(frame, "depth_data", "getDepthData")
            ir = self._field(frame, "amplitude_data", "getAmplitudeData")
            if ir is None:
                ir = self._field(frame, "confidence_data", "getConfidenceData")
        finally:
            self.cam.releaseFrame(frame)
        if depth is None:
            return None
        if not self.new_api:
            depth *= 1000.0              # the old API reads metres
        return depth, ir

    def close(self):
        for step in ("stop", "close"):
            try:
                getattr(self.cam, step)()
            except Exception:
                pass


def _run():
    failures = 0
    while not _stop.is_set():
        try:
            sdk = _Sdk()
        except Exception as e:
            _note(ok=False, error=f"{type(e).__name__}: {e}")
            failures += 1
            _stop.wait(TOF_RETRY_BACKOFF_S[min(failures, len(TOF_RETRY_BACKOFF_S)) - 1])
            continue
        _note(ok=True, error=None, opened_at=time.time(), sdk=sdk.name)
        last = time.monotonic()
        try:
            while not _stop.is_set():
                got = sdk.read()
                if got is None:
                    if time.monotonic() - last > _NO_FRAMES_S:
                        raise RuntimeError(f"no ToF frame for {_NO_FRAMES_S:.0f} s")
                    continue
                now = time.time()
                last = time.monotonic()
                failures = 0
                with _lock:
                    _frames.append(_Frame(now, got[0], got[1]))
                    _arrivals.append(now)
                    _status["frames"] += 1
        except Exception as e:
            _note(ok=False, error=f"{type(e).__name__}: {e}")
            failures += 1
        finally:
            sdk.close()
        _stop.wait(TOF_RETRY_BACKOFF_S[min(max(failures, 1), len(TOF_RETRY_BACKOFF_S)) - 1])
    _note(running=False, ok=False)


def _note(**kw):
    with _lock:
        _status.update(kw)


def start_tof() -> bool:
    """Start the capture thread (only with TOF_ENABLED)."""
    global _thread
    if not TOF_ENABLED or (_thread is not None and _thread.is_alive()):
        return False
    _stop.clear()
    _note(running=True, ok=False, error=None)
    _thread = threading.Thread(target=_run, name="tof-capture", daemon=True)
    _thread.start()
    return True


def stop_tof(timeout_s: float = 3.0) -> None:
    _stop.set()
    if _thread is not None:
        _thread.join(timeout_s)


def nearest(frame_time: float, max_gap_s: float):
    """The ToF frame captured nearest `frame_time` (within max_gap_s), placed on the
    camera's picture (a depth_fusion.DepthFrame), or None."""
    with _lock:
        best = min(_frames, key=lambda f: abs(f.t - frame_time), default=None)
    if best is None or abs(best.t - float(frame_time)) > max_gap_s:
        return None
    if best.placed is None:              # placed once, on first use
        from utils.depth_fusion import get_projector
        best.placed = get_projector().project(best.depth, best.ir, best.t)
    return best.placed


def latest_raw() -> Optional[Tuple[float, np.ndarray, Optional[np.ndarray]]]:
    """(capture time, depth mm, brightness) of the newest frame: the calibration capture and the preview."""
    with _lock:
        f = _frames[-1] if _frames else None
    return None if f is None else (f.t, f.depth, f.ir)


def tof_status() -> Dict[str, Any]:
    """For /tof/status and /health."""
    from utils.depth_fusion import get_projector
    with _lock:
        st = dict(_status)
        last = _frames[-1].t if _frames else None
        times = list(_arrivals)
    now = time.time()
    st.update({"enabled": TOF_ENABLED,
               "fps": (round((len(times) - 1) / (times[-1] - times[0]), 1)
                       if len(times) >= 2 and times[-1] > times[0] else None),
               "age_s": None if last is None else round(now - last, 2),
               "depth_is_radial": TOF_DEPTH_IS_RADIAL})
    try:
        st["calibration"] = get_projector().calib.summary()
    except Exception as e:
        st["calibration"] = {"source": "error", "note": f"{type(e).__name__}: {e}"}
    return st
