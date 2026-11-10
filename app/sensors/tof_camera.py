"""The ToF depth camera (Arducam, 240 x 180 distances on CSI): frames captured in a child
process, the last TOF_BUFFER_FRAMES kept here (2026-10-13; own process since 2026-11-03).
Started only with TOF_ENABLED.

Its own process because the SDK holds the GIL while it waits for a frame: a 1 ms ticker
thread ran once every 32 ms beside it (measured 2026-11-03, 30 fps), so in this process it
would stall every other thread, the LiDAR reader and the guidance loop included. The child
(as ~/tof/tof_live.py does) sends each frame up a pipe: depth and confidence as uint16,
~170 KB. It finds the sensor's /dev/videoN itself (media-ctl, as ~/tof/test_tof.py), and
reopens it when it stalls; this side restarts the child if it dies or goes silent.

Frames are kept raw and placed on the front camera's picture (app/perception/depth_fusion.py) only
when a detection pass asks for the one nearest its picture, so a frame nobody uses costs
nothing. The SDK (pip install --no-deps ArducamDepthCamera) is imported only in the child;
both of its API generations are handled (ArducamCamera with Connection/FrameType/Control,
and the older TOFConnect/TOFOutput/TOFControl one, which reads metres, not millimetres).
"""

import collections
import glob
import multiprocessing as mp
import re
import signal
import struct
import subprocess
import threading
import time
from typing import Any, Dict, Optional, Tuple

import numpy as np

from app.config import (
    TOF_BUFFER_FRAMES,
    TOF_DEPTH_IS_RADIAL,
    TOF_ENABLED,
    TOF_FRAME_TIMEOUT_MS,
    TOF_MAX_MM,
    TOF_MIN_CONFIDENCE,
    TOF_MIN_MM,
    TOF_RANGE_MM,
    TOF_RETRY_BACKOFF_S,
    TOF_VIDEO_INDEX,
)

DRIVER = "arducam-pivariety"  # the ToF's kernel driver
# The CSI receiver node the sensor streams into: Pi 5 (rp1-cfe), Pi 4 (unicam).
CSI_NODE_NAMES = ("rp1-cfe-csi2_ch0", "unicam-image")

_NO_FRAMES_S = 3.0  # this long without a frame: the child reopens the sensor
_SILENT_S = 30.0  # this long without a word from the child (it is stuck): it is restarted
# A frame up the pipe: this header, then depth uint16[h*w] (mm), then, if it has one,
# confidence uint16[h*w]. /tof/ws sends it to the page as it is.
_HEADER = struct.Struct("<dHHHH")  # capture time.time(), width, height, range mm, has conf


class _Frame:
    __slots__ = ("t", "depth", "ir", "placed", "packet")

    def __init__(
        self,
        t: float,
        depth: np.ndarray,
        ir: Optional[np.ndarray],
        packet: Optional[bytes] = None,
    ):
        self.t, self.depth, self.ir, self.placed = t, depth, ir, None
        self.packet = packet  # as it came up the pipe (None for a simulated frame)


_lock = threading.Lock()
_frames: "collections.deque[_Frame]" = collections.deque(maxlen=TOF_BUFFER_FRAMES)
_stop = threading.Event()
_thread: Optional[threading.Thread] = None
_child_stop = None  # the capture process's stop event (made in start_tof)
_proc = None
_status: Dict[str, Any] = {
    "running": False,
    "ok": False,
    "error": None,
    "frames": 0,
    "opened_at": None,
    "sdk": None,
    "device": None,
    "sensor": None,
    "range_mm": None,
    "pid": None,
    "restarts": 0,
}
_arrivals: "collections.deque[float]" = collections.deque(maxlen=30)


def _backoff(failures: int) -> float:
    return TOF_RETRY_BACKOFF_S[min(max(failures, 1), len(TOF_RETRY_BACKOFF_S)) - 1]


def node_via_media_ctl() -> Optional[int]:
    """N for /dev/videoN on the media graph that has the ToF sensor, or None.
    Raises FileNotFoundError when media-ctl isn't installed (sudo apt install v4l-utils)."""
    for media in sorted(glob.glob("/dev/media*")):
        out = subprocess.run(
            ["media-ctl", "-d", media, "-p"], capture_output=True, text=True, timeout=5
        ).stdout
        if DRIVER not in out:
            continue
        entity = None
        for line in out.splitlines():
            m = re.match(r"- entity \d+: (\S+)", line.strip())
            if m:
                entity = m.group(1)
            m = re.search(r"device node name /dev/video(\d+)", line)
            if m and entity in CSI_NODE_NAMES:
                return int(m.group(1))
    return None


def find_video_index() -> Optional[int]:
    """The ToF's /dev/videoN: TOF_VIDEO_INDEX if set, else the node media-ctl links to the
    sensor, else (no media-ctl) the only CSI node there is. None when it can't tell: with
    the camera array on the other port there are two CSI nodes, and the number each gets
    depends on which probed first."""
    if TOF_VIDEO_INDEX is not None:
        return int(TOF_VIDEO_INDEX)
    try:
        return node_via_media_ctl()
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    nodes = []
    for path in glob.glob("/sys/class/video4linux/video*/name"):
        try:
            with open(path) as f:
                name = f.read().strip()
        except OSError:
            continue
        if name in CSI_NODE_NAMES:
            nodes.append(int(re.search(r"video(\d+)", path).group(1)))
    return nodes[0] if len(nodes) == 1 else None


def _failed(ret) -> bool:
    return ret is not None and int(ret) != 0


class _Sdk:
    """The sensor, through whichever ArducamDepthCamera API is installed (capture process)."""

    def __init__(self, index: int):
        import ArducamDepthCamera as ac

        self.cam = ac.ArducamCamera()
        self.new_api = hasattr(ac, "Connection")
        conn = ac.Connection.CSI if self.new_api else ac.TOFConnect.CSI
        ret = self.cam.open(conn, index)
        if _failed(ret):
            raise RuntimeError(
                f"cannot open the ToF at /dev/video{index} (error {ret}) - "
                "held by another program (~/tof/tof_live.py, tools/device/tof_check.py)?"
            )
        try:
            ret = self.cam.start(
                ac.FrameType.DEPTH if self.new_api else ac.TOFOutput.DEPTH
            )
        except BaseException:
            self.cam.close()  # opened: a start that RAISES must not leave it held
            raise
        if _failed(ret):
            self.cam.close()
            raise RuntimeError(f"the ToF would not start (error {ret})")
        self.range_mm = int(TOF_RANGE_MM)
        try:  # the new API takes millimetres, the old one metres
            if self.new_api:
                self.cam.setControl(ac.Control.RANGE, int(TOF_RANGE_MM))
                self.range_mm = int(self.cam.getControl(ac.Control.RANGE) or TOF_RANGE_MM)
            else:
                self.cam.setControl(
                    ac.TOFControl.RANGE, int(round(TOF_RANGE_MM / 1000.0))
                )
        except Exception:
            pass  # it keeps its own default range
        try:
            info = self.cam.getCameraInfo()
            kind = getattr(info.device_type, "name", info.device_type)
            self.sensor = f"{kind} {info.width}x{info.height}"
        except Exception:
            self.sensor = None
        self.name = "ArducamDepthCamera " + (
            "(Connection API)" if self.new_api else "(TOFConnect API)"
        )

    @staticmethod
    def _field(frame, attr: str, getter: str):
        val = getattr(frame, attr, None)
        if val is None and hasattr(frame, getter):
            val = getattr(frame, getter)()
        return None if val is None else np.array(val, dtype=np.float32, copy=True)

    def read(self) -> Optional[Tuple[np.ndarray, Optional[np.ndarray]]]:
        """(depth in mm, confidence) of the next frame, or None on a timeout. The confidence
        is also the brightness picture the calibration finds the board in (0.1.24 gives
        the same numbers as confidence_data and amplitude_data)."""
        frame = self.cam.requestFrame(TOF_FRAME_TIMEOUT_MS)
        if frame is None:
            return None
        try:
            depth = self._field(frame, "depth_data", "getDepthData")
            ir = self._field(frame, "confidence_data", "getConfidenceData")
            if ir is None:
                ir = self._field(frame, "amplitude_data", "getAmplitudeData")
        finally:
            self.cam.releaseFrame(frame)
        if depth is None:
            return None
        if not self.new_api:
            depth *= 1000.0  # the old API reads metres
        return depth, ir

    def close(self):
        for step in ("stop", "close"):
            try:
                getattr(self.cam, step)()
            except Exception:
                pass


def _u16(a: np.ndarray) -> bytes:
    a = np.nan_to_num(a, nan=0.0, posinf=65535.0, neginf=0.0)
    return np.clip(np.rint(a), 0, 65535).astype("<u2").tobytes()


def _pack(t: float, depth: np.ndarray, ir: Optional[np.ndarray], range_mm: int) -> bytes:
    h, w = depth.shape
    head = _HEADER.pack(t, w, h, int(range_mm), ir is not None)
    return head + _u16(depth) + (b"" if ir is None else _u16(ir))


def _unpack(packet: bytes) -> _Frame:
    t, w, h, _, has_ir = _HEADER.unpack_from(packet)
    n = w * h
    depth = np.frombuffer(packet, "<u2", n, _HEADER.size).reshape(h, w)
    ir = (
        np.frombuffer(packet, "<u2", n, _HEADER.size + 2 * n).reshape(h, w)
        if has_ir
        else None
    )
    return _Frame(t, depth, ir, packet)


def _capture(conn, stop) -> None:
    """The capture process: finds and opens the sensor, sends each frame up `conn`, and
    reopens it when it stalls or will not open (backing off TOF_RETRY_BACKOFF_S)."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # Ctrl-C goes to the app, which stops this
    failures = 0
    try:
        while not stop.is_set():
            sdk = None
            try:
                index = find_video_index()
                if index is None:
                    raise RuntimeError(
                        f"no ToF sensor ({DRIVER}) on any camera port - "
                        "run tools/device/tof_check.py"
                    )
                sdk = _Sdk(index)
                conn.send(
                    (
                        "state",
                        {
                            "ok": True,
                            "error": None,
                            "opened_at": time.time(),
                            "sdk": sdk.name,
                            "device": f"/dev/video{index}",
                            "sensor": sdk.sensor,
                            "range_mm": sdk.range_mm,
                        },
                    )
                )
                last = time.monotonic()
                while not stop.is_set():
                    got = sdk.read()
                    t = time.time()
                    if got is None:
                        if time.monotonic() - last > _NO_FRAMES_S:
                            raise RuntimeError(f"no ToF frame for {_NO_FRAMES_S:.0f} s")
                        continue
                    last = time.monotonic()
                    failures = 0
                    conn.send(("frame", _pack(t, got[0], got[1], sdk.range_mm)))
            except (BrokenPipeError, EOFError):
                raise
            except Exception as e:
                failures += 1
                conn.send(("state", {"ok": False, "error": f"{type(e).__name__}: {e}"}))
            finally:
                if sdk is not None:
                    sdk.close()
            stop.wait(_backoff(failures))
    except (BrokenPipeError, EOFError, OSError):
        pass  # the app is gone


def _run():
    """Keeps the capture process running and files its frames."""
    global _proc
    ctx = mp.get_context("spawn")  # never fork: this process runs many threads
    while not _stop.is_set():
        recv, send = ctx.Pipe(duplex=False)
        proc = ctx.Process(
            target=_capture, args=(send, _child_stop), name="tof-capture", daemon=True
        )
        try:
            proc.start()
        except Exception as e:
            send.close()
            recv.close()
            _note(ok=False, error=f"capture process would not start: {type(e).__name__}: {e}")
            _stop.wait(_backoff(len(TOF_RETRY_BACKOFF_S)))
            continue
        _proc = proc
        send.close()  # only the child's end left: its exit reads as EOF here
        _note(pid=proc.pid)
        try:
            while True:
                if not recv.poll(_SILENT_S):
                    _note(ok=False, error=f"capture process silent for {_SILENT_S:.0f} s")
                    break
                kind, data = recv.recv()
                if kind == "frame":
                    f = _unpack(data)
                    with _lock:
                        _frames.append(f)
                        _arrivals.append(f.t)
                        _status["frames"] += 1
                else:
                    _note(**data)
        except (EOFError, OSError):
            pass
        finally:
            recv.close()
        if proc.is_alive():
            proc.terminate()
        proc.join(2.0)
        if not _stop.is_set():
            with _lock:
                _status["ok"] = False
                _status["restarts"] += 1
                if proc.exitcode not in (0, None):
                    _status["error"] = f"capture process exited ({proc.exitcode})"
            _stop.wait(_backoff(1))
    _note(running=False, ok=False, pid=None)


def _note(**kw):
    with _lock:
        _status.update(kw)


def start_tof() -> bool:
    """Start the capture process and its supervisor thread (only with TOF_ENABLED)."""
    global _thread, _child_stop
    if not TOF_ENABLED or (_thread is not None and _thread.is_alive()):
        return False
    _stop.clear()
    _child_stop = mp.get_context("spawn").Event()
    _note(running=True, ok=False, error=None)
    _thread = threading.Thread(target=_run, name="tof-capture", daemon=True)
    _thread.start()
    return True


def stop_tof(timeout_s: float = 3.0) -> None:
    _stop.set()
    if _child_stop is not None:
        _child_stop.set()
    if _thread is not None:
        _thread.join(timeout_s)
    if _proc is not None and _proc.is_alive():
        _proc.terminate()  # stuck in the SDK: the sensor is freed when it goes


def nearest(frame_time: float, max_gap_s: float):
    """The ToF frame captured nearest `frame_time` (within max_gap_s), placed on the
    camera's picture (a depth_fusion.DepthFrame), or None."""
    with _lock:
        best = min(_frames, key=lambda f: abs(f.t - frame_time), default=None)
    if best is None or abs(best.t - float(frame_time)) > max_gap_s:
        return None
    if best.placed is None:  # placed once, on first use
        from app.perception.depth_fusion import get_projector

        best.placed = get_projector().project(best.depth, best.ir, best.t)
    return best.placed


def latest_raw() -> Optional[Tuple[float, np.ndarray, Optional[np.ndarray]]]:
    """(capture time, depth mm, brightness) of the newest frame: the calibration capture and the preview."""
    with _lock:
        f = _frames[-1] if _frames else None
    return None if f is None else (f.t, f.depth, f.ir)


def latest_packet() -> Tuple[int, Optional[bytes]]:
    """(frames so far, the newest frame as it came up the pipe): what /tof/ws sends."""
    with _lock:
        f = _frames[-1] if _frames else None
        return _status["frames"], (None if f is None else f.packet)


def tof_status() -> Dict[str, Any]:
    """For /tof/status and /health."""
    from app.perception.depth_fusion import get_projector

    with _lock:
        st = dict(_status)
        last = _frames[-1].t if _frames else None
        times = list(_arrivals)
    now = time.time()
    st.update(
        {
            "enabled": TOF_ENABLED,
            "fps": (
                round((len(times) - 1) / (times[-1] - times[0]), 1)
                if len(times) >= 2 and times[-1] > times[0]
                else None
            ),
            "age_s": None if last is None else round(now - last, 2),
            "depth_is_radial": TOF_DEPTH_IS_RADIAL,
            # A depth pixel counts with at least this confidence, inside these distances
            # (app/perception/depth_fusion.py): /tof/live starts from the same cut.
            "min_confidence": TOF_MIN_CONFIDENCE,
            "min_mm": TOF_MIN_MM,
            "max_mm": TOF_MAX_MM,
        }
    )
    try:
        st["calibration"] = get_projector().calib.summary()
    except Exception as e:
        st["calibration"] = {"source": "error", "note": f"{type(e).__name__}: {e}"}
    return st
