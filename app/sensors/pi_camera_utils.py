"""Pi Camera utility functions and state management."""

import logging
import atexit
import contextlib
import fcntl
import glob
import io
import os
import struct
import sys
import tempfile
import threading
import time
import base64
import weakref
from collections import deque

import numpy as np
from PIL import Image

_log = logging.getLogger(__name__)

try:
    import cv2 as _cv2
except Exception:
    _cv2 = None
try:
    import simplejpeg as _simplejpeg
except Exception:
    _simplejpeg = None

from app.config import (
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    JPEG_QUALITY,
    TARGET_FPS,
    FRAME_INTERVAL,
    HEF_PATH,
    DETECTION_CONF,
    DETECTION_IOU,
    FRAME_GRID_ROWS,
    FRAME_GRID_COLS,
    FRAME_SECTOR_LAYOUT,
    SEG_ENABLED,
    SEG_HEF_PATH,
    SEG_TARGET_FPS,
    STREAM_MAX_WIDTH,
    STREAM_QUALITY,
    STREAM_STALL_TIMEOUT_S,
    CAMERA_CAPTURE_FPS,
    SEG_VIEW_FPS,
    SEG_VIEW_PRIORITY,
    SEG_ALWAYS,
    DETECTION_FRONT_EVERY_PASS,
    DETECTION_SIDE_MIN_INTERVAL_S,
    CAMERA_FRAME_TIMEOUT_S,
    CAMERA_STOP_TIMEOUT_S,
    CAMERA_FOCUS_CODE,
    CAMERA_LENS_DRIVER,
    CAMERA_RETRY_BACKOFF_S,
    HAILO_RETRY_BACKOFF_S,
    HAILO_INFER_FAIL_RESET_S,
    HAILO_SEG_RUN_TIMEOUT_MS,
    HAILO_VDEVICE_RESET,
    HAILO_VDEVICE_RESET_AFTER,
    HAILO_VDEVICE_RESET_WAIT_S,
    HAILO_PROBE_TIMEOUT_S,
    SIMPLE_CAMERA_FRESH_S,
    CAMERA_HAT_ADDRESS,
    CAMERA_HAT_I2C_BUS,
    CAMERA_VIEW_CHECK_S,
    CAMERA_DARK_LEVEL,
    CAMERA_DARK_CONFIRM_S,
    CAMERA_BLUR_VAR,
    CAMERA_BLUR_CONFIRM_S,
    CAMERA_DIM_LUX,
    CAMERA_DIM_CONFIRM_S,
    CAMERA_SHUTTER_CAP_MS,
    CAMERA_TUNING_FILE,
    CAMERA_STALL_REOPEN_S,
    CAMERA_STALL_REOPEN_MAX_S,
    TOF_ENABLED,
)

from app.core.freshness import is_fresh
from app.core.shared_state import update_camera_state

SWAP_RB = True

from app.config import CAMERA_SIMULATION_ALLOWED  # noqa: E402

# ============== GLOBAL STATE ==============
picam2 = None
current_frame = None  # latest full mosaic as a numpy array (H, W, 3)
current_frame_id = 0  # increments every new capture
current_frame_t = 0.0  # time.time() when current_frame was captured
current_frame_simulated = False  # current_frame was FABRICATED (_simulate_thread);
# written with the frame, under frame_lock
capture_stalled = False  # no frame for CAMERA_FRAME_TIMEOUT_S (see _capture_thread)
_stall_reopens = 0  # times a stuck stream was closed and reopened
_camera_source = "not started"  # "hardware", "simulated" or "unavailable: <why>"
frame_lock = threading.Lock()
# Notified each time a new picture is stored: /camera/live waits on it (live_jpeg).
_new_frame = threading.Condition()
# Notified each time a camera's detections are published: the live picture waits on it,
# so the front camera's boxes go out with the very picture they were found in.
_new_detection = threading.Condition()
running = True

_stream_shutdown = threading.Event()


def begin_shutdown() -> None:
    """End every MJPEG stream now, before the server drains connections."""
    _stream_shutdown.set()


def _streams_live() -> bool:
    """Whether MJPEG generators should keep going."""
    return running and not _stream_shutdown.is_set()


hailo_ready = False
hailo_error = None
hailo_init_attempts = 0  # detector init attempts since start_camera()
seg_engine = None
seg_error = None  # same as hailo_error, for the segmenter
seg_init_attempts = 0  # same as hailo_init_attempts, for the segmenter
seg_view_active_ts = 0.0
seg_front_view_ts = 0.0

seg_latest = {"class_map": None, "frame_id": -1, "infer_ms": 0.0}
seg_latest_lock = threading.Lock()

fps_stats = {
    "capture_fps": 0,
    "detection_fps": 0,
    "seg_fps": 0,
    "capture_times": deque(maxlen=30),
    "detection_times": deque(maxlen=30),
    "seg_times": deque(maxlen=30),
}

detection_stats = {
    "front": {"detections": [], "num_objects": 0},
    "right": {"detections": [], "num_objects": 0},
    "left": {"detections": [], "num_objects": 0},
    "back": {"detections": [], "num_objects": 0},
    "total_objects": 0,
    "inference_time": 0,
}

_VALID = ("front", "right", "left", "back")
_SIDES = tuple(s for s in _VALID if s != "front")


# ============== HELPERS ==============


def _to_rgb(arr):
    """Return a C-contiguous RGB array from the captured (BGR-order) frame."""
    if SWAP_RB:
        arr = arr[:, :, ::-1]
    return np.ascontiguousarray(arr)


def _to_bgr(arr):
    """The captured frame in OpenCV's BGR order — as captured, when SWAP_RB."""
    return arr if SWAP_RB else np.ascontiguousarray(arr[:, :, ::-1])


def _to_seg_input(arr):
    """Return a quadrant in the byte order the SEGMENTATION model expects."""
    return _to_rgb(arr)


def _sector_slice(mosaic, sector):
    """Return a numpy VIEW of one quadrant of the mosaic."""
    h, w = mosaic.shape[:2]
    quad_h = h // FRAME_GRID_ROWS
    quad_w = w // FRAME_GRID_COLS
    row, col = FRAME_SECTOR_LAYOUT[sector]
    y1, x1 = row * quad_h, col * quad_w
    return mosaic[y1 : y1 + quad_h, x1 : x1 + quad_w]


def _encode_jpeg(rgb_arr):
    """Encode an RGB numpy array to JPEG bytes."""
    buf = io.BytesIO()
    Image.fromarray(rgb_arr).save(buf, format="JPEG", quality=JPEG_QUALITY)
    return buf.getvalue()


def _encode_jpeg_stream(arr, colorspace="RGB", max_width=STREAM_MAX_WIDTH):
    """Encode a DOWNSCALED, lower-quality JPEG for the dashboard's live picture."""
    h, w = arr.shape[:2]
    size = (
        (max_width, max(1, int(h * max_width / w)))
        if w > max_width
        else None
    )
    if _simplejpeg is not None and _cv2 is not None:
        if size:
            arr = _cv2.resize(arr, size, interpolation=_cv2.INTER_AREA)
        return _simplejpeg.encode_jpeg(
            np.ascontiguousarray(arr),
            quality=STREAM_QUALITY,
            colorspace=colorspace,
            colorsubsampling="420",
        )
    rgb = arr if colorspace == "RGB" else arr[:, :, ::-1]
    img = Image.fromarray(np.ascontiguousarray(rgb))
    if size:
        img = img.resize(size)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=STREAM_QUALITY)
    return buf.getvalue()


# ============== CAMERA START / STOP ==============

_threads = []


def _start_thread(target):
    t = threading.Thread(target=target, name=target.__name__.strip("_"), daemon=True)
    t.camera_generation = _generation
    t.start()
    _threads.append(t)


def start_camera():
    """Start capture + Hailo inference threads."""
    global running, _camera_source
    _new_generation(reset_status=True)
    running = True
    _stream_shutdown.clear()  # a restart must not inherit the last stop's flag

    try:
        import picamera2  # noqa: F401 — only asking whether it is installed
    except ImportError as e:
        if not CAMERA_SIMULATION_ALLOWED:
            _camera_source = f"unavailable: picamera2 not importable ({e})"
            _log.error(
                f"picamera2 is not importable ({e}) — NO CAMERA. Publishing "
                f"nothing; health reports the camera lost and object "
                f"identification is off. (Development simulation needs "
                f"SHAPPE_SIMULATE_CAMERA=1.)"
            )
        else:
            _camera_source = "simulated"
            _log.warning(
                f"picamera2 not installed ({e}) and SHAPPE_SIMULATE_CAMERA=1: "
                f"SIMULATED camera frames — flagged `simulated`, never evidence "
                f"of anything"
            )
            _start_thread(_simulate_thread)
    else:
        _camera_source = "hardware"
        _start_thread(_capture_supervised)

    _start_thread(_inference_thread)

    _start_thread(_seg_thread)
    _log.info(f"🧠 Hailo local inference: {HEF_PATH}")
    _log.info(f"🎯 Target: {TARGET_FPS} FPS")


def stop_camera():
    """Stop camera capture and inference threads. Bounded: never hangs."""
    global running, picam2
    running = False
    _new_generation()  # hailo_ready off now; nothing publishes after
    begin_shutdown()  # in case we got here without a signal
    for t in _threads:
        t.join(timeout=3.0)
    _threads.clear()

    with _camera_lock:
        cam, picam2 = picam2, None
    if cam is not None:
        _close_camera(cam)


def _close_camera(cam):
    """Stop and close a camera, bounded."""
    try:
        atexit.unregister(cam.close)
    except Exception:
        pass
    _bounded(lambda: (cam.stop(), cam.close()), CAMERA_STOP_TIMEOUT_S, "camera stop")


def _bounded(fn, timeout_s: float, what: str) -> bool:
    """Run fn on a daemon thread and give it timeout_s."""
    done = threading.Event()

    def run():
        try:
            fn()
        except Exception as exc:
            _log.warning(f"{what} failed: {type(exc).__name__}: {exc}")
        finally:
            done.set()

    threading.Thread(target=run, name=what.replace(" ", "-"), daemon=True).start()
    if done.wait(timeout_s):
        return True
    _log.warning(
        f"{what} still blocked after {timeout_s:g}s — abandoning it "
        f"so shutdown can finish"
    )
    return False


# ============== FRAME CAPTURE ==============

_camera_lock = threading.Lock()
_QUAD = (CAMERA_WIDTH // FRAME_GRID_COLS, CAMERA_HEIGHT // FRAME_GRID_ROWS)


def _set_board_mode() -> bool:
    """Put the camera board in 4-in-1 mode — all four cameras in the 2x2 picture (0x00) —
    which it forgets on a power cycle. False if the write failed."""
    try:
        from smbus2 import SMBus

        with SMBus(CAMERA_HAT_I2C_BUS) as bus:
            bus.write_byte_data(CAMERA_HAT_ADDRESS, 0x24, 0x00)
        time.sleep(0.3)  # let it switch before streaming
        _log.info("[CAMERA] board set to all four cameras (2x2, 0x00)")
        return True
    except Exception as e:
        _log.warning(f"Camera board mode not set ({type(e).__name__}: {e})")
        return False


_view_lock = threading.Lock()
_views = {}  # sector -> what _view_check last measured

# The sensor's light reading (_note_light): libcamera's lux for the whole 2x2 picture and the
# exposure that gave it. Replaced whole, never changed in place, so a reader needs no lock.
_light = {"lux": None, "exposure_ms": None, "gain": None, "mono": float("-inf")}
_LIGHT_EVERY_S = 0.5  # read at most this often (~0.1 ms each)
_LIGHT_FRESH_S = 2.0  # older than this: no light reading


def _note_light(request):
    """Picamera2's post_callback, on its own thread with each picture: keep the sensor's light
    reading. Never raises: a light reading is not worth a lost picture."""
    global _light
    try:
        now = time.monotonic()
        if now - _light["mono"] < _LIGHT_EVERY_S:
            return
        md = request.get_metadata() or {}
        lux = md.get("Lux")
        _light = {
            "lux": float(lux) if lux is not None else None,
            "exposure_ms": float(md.get("ExposureTime") or 0) / 1000.0,
            "gain": float(md.get("AnalogueGain") or 0) * float(md.get("DigitalGain") or 1.0),
            "mono": now,
        }
    except Exception:
        pass


def light_reading() -> dict:
    """The sensor's latest light reading, or Nones when there is none (simulated, stalled)."""
    light = _light
    if time.monotonic() - light["mono"] > _LIGHT_FRESH_S:
        return {"lux": None, "exposure_ms": None, "gain": None}
    return {
        "lux": None if light["lux"] is None else round(light["lux"], 1),
        "exposure_ms": round(light["exposure_ms"], 1),
        "gain": round(light["gain"], 2),
    }


def _camera_lux(level, mosaic):
    """How much light one camera sees: the sensor's lux for the whole picture, scaled by how
    bright this camera's part is against the whole (one exposure serves all four)."""
    lux = light_reading()["lux"]
    if lux is None or mosaic is None:
        return None
    whole = float(mosaic[::16, ::16].mean())
    return lux * level / whole if whole > 1.0 else lux


def _view_check(sector, quad, mosaic=None) -> dict:
    """Can this camera see? Picture level, sharpness and light (lux), measured at most every
    CAMERA_VIEW_CHECK_S; "dark" / "blurred" / "dim" only once they have lasted their
    CAMERA_*_CONFIRM_S. ~2 ms on a camera's picture (every 4th pixel)."""
    now = time.monotonic()
    with _view_lock:
        v = _views.get(sector)
        if v is not None and 0.0 <= now - v["mono"] < CAMERA_VIEW_CHECK_S:
            return dict(v)
    s = quad[::4, ::4].astype(np.float32)
    level = float(s.mean())
    g = s.mean(axis=2) if s.ndim == 3 else s
    lap = 4.0 * g[1:-1, 1:-1] - g[:-2, 1:-1] - g[2:, 1:-1] - g[1:-1, :-2] - g[1:-1, 2:]
    sharpness = float(lap.var())
    lux = _camera_lux(level, mosaic)
    with _view_lock:
        v = _views.get(sector) or {}
        dark_now = level < CAMERA_DARK_LEVEL
        blur_now = not dark_now and sharpness < CAMERA_BLUR_VAR
        dim_now = not dark_now and lux is not None and lux < CAMERA_DIM_LUX
        dark_since = (v.get("dark_since") or now) if dark_now else None
        blur_since = (v.get("blur_since") or now) if blur_now else None
        dim_since = (v.get("dim_since") or now) if dim_now else None
        v = {
            "mono": now,
            "at": time.time(),
            "brightness": round(level, 1),
            "sharpness": round(sharpness, 1),
            "lux": None if lux is None else round(lux, 1),
            "dark_since": dark_since,
            "blur_since": blur_since,
            "dim_since": dim_since,
            "dark": dark_since is not None
            and now - dark_since >= CAMERA_DARK_CONFIRM_S,
            "blurred": blur_since is not None
            and now - blur_since >= CAMERA_BLUR_CONFIRM_S,
            "dim": dim_since is not None and now - dim_since >= CAMERA_DIM_CONFIRM_S,
        }
        _views[sector] = v
        return dict(v)


def camera_views() -> dict:
    """What each camera's picture looked like at its last check (see _view_check)."""
    with _view_lock:
        return {
            s: {
                k: v.get(k)
                for k in ("at", "brightness", "sharpness", "lux", "dark", "blurred", "dim")
            }
            for s, v in _views.items()
        }


# set: the capture thread closes and reopens the camera (a stream that stalled)
_reopen = threading.Event()


# VIDIOC_S_CTRL (_IOWR('V', 28, struct v4l2_control {u32 id; s32 value})) of
# V4L2_CID_FOCUS_ABSOLUTE: what `v4l2-ctl -c focus_absolute=` does.
_VIDIOC_S_CTRL = 0xC008561C
_V4L2_CID_FOCUS_ABSOLUTE = 0x009A090A
_focus_code = None  # the lens position set at the last camera start, None if not set


def _lens_device(sysfs="/sys/class/video4linux"):
    """/dev/v4l-subdevN of the lens driver (CAMERA_LENS_DRIVER), or None."""
    for d in sorted(glob.glob(os.path.join(sysfs, "v4l-subdev*"))):
        try:
            with open(os.path.join(d, "name"), encoding="utf-8") as f:
                if f.read().strip().startswith(CAMERA_LENS_DRIVER):
                    return "/dev/" + os.path.basename(d)
        except OSError:
            continue
    return None


def _set_focus() -> bool:
    """Put all four lenses at CAMERA_FOCUS_CODE — after every camera start, as reopening the
    camera powers the lens driver down and back to rest. False if it could not be set."""
    global _focus_code
    _focus_code = None
    if CAMERA_FOCUS_CODE is None:
        return False
    dev = _lens_device()
    if dev is None:
        _log.warning(f"Lens driver {CAMERA_LENS_DRIVER!r} not found — focus not set, pictures may blur")
        return False
    code = int(max(0, min(4095, CAMERA_FOCUS_CODE)))
    try:
        fd = os.open(dev, os.O_RDWR)
        try:
            fcntl.ioctl(fd, _VIDIOC_S_CTRL, struct.pack("Ii", _V4L2_CID_FOCUS_ABSOLUTE, code))
        finally:
            os.close(fd)
    except OSError as e:
        _log.warning(f"Lens focus not set ({dev}: {e}) — pictures may blur")
        return False
    _focus_code = code
    _log.info(f"[CAMERA] lens focus set to {code} ({dev})")
    return True


# libcamera looks at every camera port when the camera opens, and in CAM0 finds the ToF,
# which it cannot run: no tuning file, no helper (the ToF's own SDK reads it). That was five
# ERROR lines at every start that read as a failure (2026-11-05); one plain line now says so.
_TOF_PORT_DRIVER = b"arducam-pivariety"
_TOF_PORT_FOLLOW_ON = (  # libcamera's lines about that port that don't name it
    b"Failed to load a suitable IPA library",
    b"Please consider updating the camera sensor properties database",
)


@contextlib.contextmanager
def _quiet_tof_port():
    """Hold back stderr while libcamera looks over the camera ports, then pass all of it on
    but its lines about the ToF's port. A file, not a pipe: no writer can block on it."""
    try:
        if sys.stderr:
            sys.stderr.flush()
        console = os.dup(2)
    except (OSError, ValueError):  # no stderr to hold back
        yield
        return
    with tempfile.TemporaryFile() as held:
        os.dup2(held.fileno(), 2)
        try:
            yield
        finally:
            if sys.stderr:
                sys.stderr.flush()
            os.dup2(console, 2)
            os.close(console)
            held.seek(0)
            lines = held.read().splitlines(keepends=True)
            tof = any(_TOF_PORT_DRIVER in line for line in lines)
            kept = b"".join(
                line for line in lines
                if _TOF_PORT_DRIVER not in line
                and not (tof and any(s in line for s in _TOF_PORT_FOLLOW_ON))
            )
            if tof:
                _log.info("[CAMERA] libcamera passed over the ToF's port (arducam-pivariety), "
                      "as expected: the ToF's own SDK reads it")
            while kept:
                kept = kept[os.write(2, kept):]


def _short_exposure():
    """(tuning, controls) that keep the exposure at CAMERA_SHUTTER_CAP_MS while gain can make
    up the light: the tuning's "short" profile rewritten from the "normal" one (its last
    stage, the most light, kept), and AeExposureMode Short. (None, {}) when off or the tuning
    cannot be read: the sensor's own profile, as before."""
    if not CAMERA_SHUTTER_CAP_MS:
        return None, {}
    try:
        from libcamera import controls
        from picamera2 import Picamera2

        tuning = Picamera2.load_tuning_file(CAMERA_TUNING_FILE)
        agc = Picamera2.find_tuning_algo(tuning, "rpi.agc")
        cap_us = float(CAMERA_SHUTTER_CAP_MS) * 1000.0
        for ch in agc.get("channels", [agc]):
            normal = ch["exposure_modes"]["normal"]
            most_us, most_gain = float(normal["shutter"][-1]), float(normal["gain"][-1])
            ch["exposure_modes"]["short"] = {
                "shutter": [100.0, cap_us, cap_us, cap_us, max(most_us, cap_us)],
                "gain": [1.0, 2.0, 4.0, most_gain, most_gain],
            }
        return tuning, {"AeExposureMode": controls.AeExposureModeEnum.Short}
    except Exception as e:
        _log.warning(
            f"Camera exposure cap not set ({type(e).__name__}: {e}) — the sensor's own "
            f"exposure, which can blur while walking"
        )
        return None, {}


def _open_camera():
    """Open, configure and start the camera, retrying until it works or we are stopped."""
    global picam2
    from picamera2 import Picamera2

    # In an unknown mode no picture can be said to come from any one camera: filed under
    # the wrong direction, a person behind is "ahead". So no picture until it is set, as
    # for a camera that will not open (scan 2026-10-23).
    tries = 0
    while not _set_board_mode():
        delay = CAMERA_RETRY_BACKOFF_S[min(tries, len(CAMERA_RETRY_BACKOFF_S) - 1)]
        tries += 1
        _log.warning(
            f"No camera pictures until the board mode is set; retrying in {delay:g}s"
        )
        deadline = time.monotonic() + delay
        while running and time.monotonic() < deadline:
            time.sleep(0.25)
        if not running:
            return False
    failures = 0
    while running:
        cam = None
        try:
            tuning, exposure = _short_exposure()
            with _quiet_tof_port():
                cam = Picamera2(tuning=tuning)
            cam.configure(
                cam.create_video_configuration(
                    main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"},
                    raw={"size": (CAMERA_WIDTH, CAMERA_HEIGHT)},
                    buffer_count=4,  # small buffer -> low latency, always-newest
                    controls={"FrameRate": float(CAMERA_CAPTURE_FPS), **exposure},
                )
            )
            cam.post_callback = _note_light  # the sensor's light reading, with each picture
            # Let the sensor run as fast as the mode allows; capture loop paces us.
            cam.start()
            _set_focus()  # once streaming: the lens driver is powered while it streams
        except Exception as e:
            if cam is not None:
                _close_camera(cam)  # half-opened: release the nodes
            delay = CAMERA_RETRY_BACKOFF_S[
                min(failures, len(CAMERA_RETRY_BACKOFF_S) - 1)
            ]
            failures += 1
            _log.warning(
                f"Camera would not open ({type(e).__name__}: {e}) — no frames "
                f"(health reports the camera lost); retrying in {delay:g}s"
            )
            deadline = time.monotonic() + delay
            while running and time.monotonic() < deadline:
                time.sleep(0.25)
            continue
        with _camera_lock:
            if running:
                picam2 = cam
                _log.info(
                    f"✅ Camera (Picamera2): {CAMERA_WIDTH}x{CAMERA_HEIGHT}, all four "
                    f"cameras at {CAMERA_WIDTH // FRAME_GRID_COLS}x"
                    f"{CAMERA_HEIGHT // FRAME_GRID_ROWS}, raw numpy capture"
                )
                return True
        _close_camera(cam)  # stopped while it opened
        return False
    return False


def _capture_supervised():
    """_capture_thread, started again whenever an error it did not expect ends it (2026-10-21): until then the thread died — e.g. on a failed get_metadata()/make_array() of a cycling request — and no picture came until the app restarted, while "Camera off" was all the wearer heard."""
    # `running`, not the generation: an NPU restart (HAILO_VDEVICE_RESET) starts a new
    # generation of NPU workers and must not end capture. stop_camera() joins this thread
    # after clearing `running`, so a restart in progress sees it within 0.25 s.
    global picam2
    failures = 0
    while running:
        began = time.monotonic()
        try:
            _capture_thread()
            return  # it returns only when stopped
        except Exception as e:
            if time.monotonic() - began > 60.0:
                failures = 0  # it had been running well: no long backoff
            delay = CAMERA_RETRY_BACKOFF_S[
                min(failures, len(CAMERA_RETRY_BACKOFF_S) - 1)
            ]
            failures += 1
            _log.warning(
                f"Camera capture thread failed ({type(e).__name__}: {e}) — "
                f"restarting it in {delay:g}s (restart {failures})"
            )
            # Opened afresh, as at start: the board and the cycle start on the same camera,
            # so no side camera's picture is filed as the front's.
            with _camera_lock:
                cam, picam2 = picam2, None
            if cam is not None:
                _close_camera(cam)
            deadline = time.monotonic() + delay
            while running and time.monotonic() < deadline:
                time.sleep(0.25)


def _capture_thread():
    """Pull the 2x2 picture of all four cameras from Picamera2 into the shared buffer."""
    global current_frame, current_frame_id, current_frame_t, capture_stalled
    global current_frame_simulated, picam2, _stall_reopens

    if picam2 is None and not _open_camera():
        return
    job = None
    asked_at = 0.0
    last_error_at = None
    stall_wait = CAMERA_STALL_REOPEN_S  # no frame this long: reopen (doubles each time)
    while running:
        if _reopen.is_set():  # the stream stalled
            _reopen.clear()
            job = None
            with _camera_lock:
                cam, picam2 = picam2, None
            if cam is not None:
                _close_camera(cam)
            if not _open_camera():
                return
            continue
        try:
            if job is None:
                job = picam2.capture_array("main", wait=False)
                asked_at = time.monotonic()
            arr = picam2.wait(job, timeout=0.25)
        except TimeoutError:
            waited = time.monotonic() - asked_at
            if not capture_stalled and waited > CAMERA_FRAME_TIMEOUT_S:
                capture_stalled = True
                _log.warning(
                    f"Camera: no frame for {waited:.0f}s — the camera is not "
                    f"delivering (still waiting on it; health reports it lost)"
                )
            if waited > stall_wait:
                # A stuck stream does not come back by itself (2026-10-09): start it again.
                _stall_reopens += 1
                _log.warning(
                    f"[CAMERA] no frame for {waited:.0f}s — closing and reopening the "
                    f"camera (reopen {_stall_reopens}; next after "
                    f"{min(stall_wait * 2.0, CAMERA_STALL_REOPEN_MAX_S):.0f}s)"
                )
                stall_wait = min(stall_wait * 2.0, CAMERA_STALL_REOPEN_MAX_S)
                _reopen.set()
            continue
        except Exception as e:
            job = None
            if running and (
                last_error_at is None
                or time.monotonic() - last_error_at > CAMERA_FRAME_TIMEOUT_S
            ):
                last_error_at = time.monotonic()
                _log.warning(f"Capture error: {e}")
            time.sleep(0.05)
            continue

        job = None
        stall_wait = CAMERA_STALL_REOPEN_S
        if capture_stalled:
            capture_stalled = False
            _log.info(
                f"[OK] Camera delivering again after {time.monotonic() - asked_at:.1f}s"
            )

        with frame_lock:
            current_frame = arr
            current_frame_id += 1
            current_frame_t = time.time()
            current_frame_simulated = False  # a real frame, in the same write
        with _new_frame:
            _new_frame.notify_all()

        fps_stats["capture_times"].append(time.monotonic())
        if len(fps_stats["capture_times"]) >= 2:
            t = list(fps_stats["capture_times"])
            fps_stats["capture_fps"] = (len(t) - 1) / (t[-1] - t[0])


def _simulate_thread():
    """Generate a synthetic 2x2 mosaic when no camera is present (dev mode)."""
    global current_frame, current_frame_id, current_frame_t
    global current_frame_simulated

    base = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
    hh, hw = CAMERA_HEIGHT // 2, CAMERA_WIDTH // 2
    base[:hh, :hw] = (40, 40, 40)  # top-left
    base[:hh, hw:] = (80, 80, 80)  # top-right
    base[hh:, :hw] = (120, 120, 120)  # bottom-left
    base[hh:, hw:] = (160, 160, 160)  # bottom-right

    while running:
        with frame_lock:
            current_frame = base
            current_frame_id += 1
            current_frame_t = time.time()
            current_frame_simulated = True
        with _new_frame:
            _new_frame.notify_all()

        fps_stats["capture_times"].append(time.monotonic())
        if len(fps_stats["capture_times"]) >= 2:
            t = list(fps_stats["capture_times"])
            fps_stats["capture_fps"] = (len(t) - 1) / (t[-1] - t[0])

        time.sleep(1.0 / TARGET_FPS)


_generation = 0
_gen_lock = threading.RLock()  # guards _generation and every NPU publish

_WARN_REPEAT_S = 5.0

# detection_coverage() reasons
_COVERAGE_FULL = ""
_COVERAGE_SEG_VIEW = "seg_view_priority"
_COVERAGE_NOT_READY = "detector_not_ready"
_coverage = {"reason": _COVERAGE_NOT_READY, "sectors": (), "since": time.time()}


def _live(gen) -> bool:
    """Whether a worker of generation `gen` may still run and publish."""
    return running and gen == _generation


def _my_generation():
    """The generation the calling worker thread was started in."""
    gen = getattr(threading.current_thread(), "camera_generation", None)
    return _generation if gen is None else gen


def _new_generation(reset_status=False):
    """Retire every NPU worker started so far (start_camera / stop_camera)."""
    global _generation, hailo_ready, seg_engine
    global hailo_error, hailo_init_attempts, seg_error, seg_init_attempts
    with _gen_lock:
        _generation += 1
        hailo_ready = False
        seg_engine = None
        _streaks.update(detector=None, seg=None)  # those engines are retired too
        if reset_status:
            hailo_error, hailo_init_attempts = None, 0
            seg_error, seg_init_attempts = None, 0
        _note_coverage(_COVERAGE_NOT_READY, ())
        return _generation


def _all_coverage():
    """(reason, sectors) when detection runs on every camera: all four are in every picture
    (a camera unplugged or covered shows as black: _view_check, "Camera sees only black")."""
    return (_COVERAGE_FULL, _VALID)


def _note_coverage(reason, sectors):
    """Record which cameras detection currently covers (see detection_coverage)."""
    with _gen_lock:
        sectors = tuple(sectors)
        if _coverage["reason"] != reason or _coverage["sectors"] != sectors:
            _coverage.update(reason=reason, sectors=sectors, since=time.time())


def detection_coverage() -> dict:
    """Which cameras object detection is covering right now."""
    with _gen_lock:
        c = dict(_coverage)
    return {
        "reduced": c["reason"] != _COVERAGE_FULL,
        "reason": c["reason"],
        "sectors_detected": list(c["sectors"]),
        "since": c["since"],
    }


def _backoff(failures: int) -> float:
    """The wait after `failures` consecutive failures (0-based), last repeating."""
    return HAILO_RETRY_BACKOFF_S[min(failures, len(HAILO_RETRY_BACKOFF_S) - 1)]


def _retry_wait(gen, delay: float) -> bool:
    """Wait out an init-retry backoff, seeing a stop or a restart within 0.25 s (the same slices _open_camera waits in)."""
    deadline = time.monotonic() + delay
    while _live(gen):
        left = deadline - time.monotonic()
        if left <= 0:
            return True
        time.sleep(min(0.25, left))
    return False


def _npu_answers() -> bool:
    """Does the NPU answer a separate process (`hailortcli fw-control identify`)? A dead chip can block an in-process HailoRT call with the GIL held, freezing every thread (HAILO_PROBE_TIMEOUT_S); a probe that hangs costs only its own process."""
    import shutil
    import subprocess

    cli = shutil.which("hailortcli")
    if cli is None:
        return True  # nothing to ask with: rebuild as before
    try:
        return (
            subprocess.run(
                [cli, "fw-control", "identify"],
                capture_output=True,
                timeout=HAILO_PROBE_TIMEOUT_S,
            ).returncode
            == 0
        )
    except Exception:  # timed out (the child is killed), or could not run
        return False


def _wait_npu_answers(gen, what: str) -> bool:
    """After `what`'s engine broke mid-run: wait, probing with the init backoff, until the NPU answers (True) or this worker is stopped or replaced (False). Detection stays off meanwhile; guidance goes on with the LiDAR and radar."""
    tries = 0
    while _live(gen):
        if _npu_answers():
            if tries:
                _log.info(f"[{what}] the NPU answers again — rebuilding the engine")
            return True
        delay = _backoff(tries)
        tries += 1
        _log.error(
            f"{what}: the NPU does not answer (hailortcli fw-control identify, "
            f"try {tries}) — not rebuilding the engine on it, which can freeze the whole "
            f"app; detection off, guidance goes on; asking again in {delay:g}s"
        )
        if not _retry_wait(gen, delay):
            return False
    return False


def _close_engine(engine, what: str):
    """Close an engine, reporting rather than raising a failure to."""
    try:
        close = getattr(engine, "close", None)
        if close is not None:
            close()
        else:
            engine.__exit__(None, None, None)
    except Exception as e:
        _log.warning(f"{what} close failed: {type(e).__name__}: {e}")


def _attempt_init(gen, what, make, on_up, on_fail):
    """One try at building an NPU engine for a worker of generation `gen`."""
    try:
        engine = make()
    except Exception as e:
        error = f"{type(e).__name__}: {e}"
        with _gen_lock:
            live = _live(gen)
            if live:
                on_fail(error)
        return None, error, live
    with _gen_lock:
        live = _live(gen)
        if live:
            on_up(engine)
    if not live:
        _log.info(
            f"[{what}] came up after its worker was stopped or replaced — "
            f"closing it unused"
        )
        _close_engine(engine, what)
        return None, None, False
    return engine, None, True


class _WarnThrottle:
    """print() a warning at most once per _WARN_REPEAT_S per distinct message; the repeats in between are counted into the next print of it."""

    def __init__(self, interval_s=None):
        self.interval_s = _WARN_REPEAT_S if interval_s is None else interval_s
        self._seen = {}  # message -> [last printed at, repeats since]

    def __call__(self, msg: str) -> bool:
        now = time.monotonic()  # an interval only (F20)
        seen = self._seen.get(msg)
        if seen is not None and now - seen[0] < self.interval_s:
            seen[1] += 1
            return False
        if seen is None and len(self._seen) >= 16:
            # A message carrying a varying detail must not grow this for ever.
            self._seen = {
                m: s for m, s in self._seen.items() if now - s[0] < self.interval_s
            }
        extra = (
            f" (+{seen[1]} more in the last {now - seen[0]:.0f}s)"
            if seen is not None and seen[1]
            else ""
        )
        _log.info(msg + extra)
        self._seen[msg] = [now, 0]
        return True


class _FailStreak:
    """Consecutive failed calls into one engine: since when, how many, last."""

    def __init__(self, limit_s: float):
        self.limit_s = limit_s
        self.reset()

    def reset(self):
        self.since, self.count, self.last, self._last_end = None, 0, "", None

    def fail(self, error: str, started=None):
        """Record a failed call."""
        now = time.monotonic()
        started = now if started is None else started
        if self.since is None or (
            self._last_end is not None and started - self._last_end > self.limit_s
        ):
            self.since, self.count = started, 0
        self.count += 1
        self.last = error
        self._last_end = now

    def failing_s(self):
        """Seconds since the first failing call of this streak began, or None."""
        since = self.since  # once: get_camera_stats reads from another thread
        return None if since is None else time.monotonic() - since

    def broken(self) -> bool:
        return self.count >= 1 and self.failing_s() >= self.limit_s

    def describe(self) -> str:
        return (
            f"every inference failed for {self.failing_s():.1f}s "
            f"({self.count} call{'' if self.count == 1 else 's'}): {self.last}"
        )


_streaks = {"detector": None, "seg": None}


def _track_streak(gen, key, streak):
    """Publish (or, with None, withdraw) a worker's streak, if still current."""
    with _gen_lock:
        if gen == _generation:
            _streaks[key] = streak


# ============== LOCAL HAILO INFERENCE THREAD ==============


def _make_detector():
    from app.sensors.hailo_inference import HailoInferenceEngine
    from app.sensors.hailo_device import get_shared_vdevice

    return HailoInferenceEngine(
        hef_path=HEF_PATH,
        conf_thresh=DETECTION_CONF,
        iou_thresh=DETECTION_IOU,
        vdevice=get_shared_vdevice(),
    )


def _detector_up(engine):  # under _gen_lock, generation current
    global hailo_ready, hailo_error, hailo_init_attempts
    hailo_init_attempts += 1
    hailo_error = None
    hailo_ready = True
    _note_coverage(*_all_coverage())


def _detector_failed(error):  # under _gen_lock, generation current
    global hailo_ready, hailo_error, hailo_init_attempts
    hailo_init_attempts += 1
    hailo_error = error
    hailo_ready = False


def _inference_thread():
    """Run Hailo inference on the 4 quadrants."""
    global hailo_ready

    gen = _my_generation()
    failures = 0  # consecutive failures: the backoff index
    was_up = False  # came up at least once in this generation
    rebuild_fails = 0  # init failures in a row since it last broke
    broke = False  # broke mid-run and not up again since
    try:
        while True:
            if broke and not _wait_npu_answers(gen, "Hailo"):
                return
            engine, error, live = _attempt_init(
                gen, "Hailo", _make_detector, _detector_up, _detector_failed
            )
            if not live:
                return
            if engine is None:
                rebuild_fails += 1
                if (
                    HAILO_VDEVICE_RESET
                    and was_up
                    and rebuild_fails >= HAILO_VDEVICE_RESET_AFTER
                ):
                    # It worked, broke, and cannot be rebuilt: the device under it is gone.
                    _log.error(
                        f"Hailo detector would not come back ({rebuild_fails} tries: "
                        f"{error}) — restarting the NPU on a fresh device"
                    )
                    threading.Thread(
                        target=_restart_npu,
                        args=(gen, error),
                        name="npu-restart",
                        daemon=True,
                    ).start()
                    return
                delay = _backoff(failures)
                failures += 1
                _log.error(
                    f"Hailo engine init failed (attempt {hailo_init_attempts}): {error} "
                    f"— detection off, hailo_ready False; retrying in {delay:g}s"
                )
                if not _retry_wait(gen, delay):
                    return
                continue

            failures = 0  # it came up: a later failure retries quickly
            was_up, rebuild_fails, broke = True, 0, False
            reason = _detect(engine, gen)  # closes the engine on exit
            engine = None
            if reason is None:
                return  # stopped, or superseded by a newer start
            broke = True
            delay = _backoff(failures)
            failures += 1
            _log.error(
                f"Hailo detector reset — {reason}; detection off, "
                f"re-initialising in {delay:g}s"
            )
            if not _retry_wait(gen, delay):
                return
    finally:
        with _gen_lock:
            if gen == _generation:  # a newer worker's flag is not ours
                hailo_ready = False
                _note_coverage(_COVERAGE_NOT_READY, ())


_npu_restart_lock = threading.Lock()


def _restart_npu(gen, why):
    """Stop every NPU worker of generation `gen`, release the shared VDevice once none of them can still be inside it, and start fresh workers, which open a new one (HAILO_VDEVICE_RESET, 2026-10-21). Capture carries on throughout."""
    from app.sensors.hailo_device import release_shared_vdevice

    with _npu_restart_lock:
        with _gen_lock:
            if not _live(gen):
                return  # stopped, or already restarted
            fresh = _new_generation()  # the old detector and segmenter wind down
        # Every NPU worker still alive, of any generation: one left stuck by an earlier
        # restart is inside the same device too.
        old = [
            t
            for t in list(_threads)
            if t.name in ("inference_thread", "seg_thread")
            and t.is_alive()
            and t is not threading.current_thread()
        ]
        deadline = time.monotonic() + HAILO_VDEVICE_RESET_WAIT_S
        for t in old:
            t.join(timeout=max(0.0, deadline - time.monotonic()))
        stuck = [t.name for t in old if t.is_alive()]
        if stuck:
            _log.error(
                f"NPU restart: {', '.join(stuck)} still inside a call after "
                f"{HAILO_VDEVICE_RESET_WAIT_S:g}s — NOT releasing the device (that could "
                f"crash the app); starting the workers on it again"
            )
        else:
            # A segmentation run that outlived close() releases its model when it returns.
            time.sleep(HAILO_SEG_RUN_TIMEOUT_MS / 1000.0)
            ok = release_shared_vdevice()
            _log.info(
                f"[Hailo] NPU restart ({why}): device "
                f"{'released' if ok else 'release FAILED'} — opening a new one"
            )
        with _gen_lock:
            if not running or _generation != fresh:
                return
            _threads[:] = [t for t in _threads if t.is_alive()]
            _start_thread(_inference_thread)
            _start_thread(_seg_thread)


def _detect(engine, gen):
    """Detection passes on one engine until stopped, superseded or broken."""
    global hailo_ready, hailo_error

    last_id = -1
    last_side = float("-inf")  # monotonic: the last side-camera detection
    side_done = {}  # side camera -> monotonic of its last detection
    streak = _FailStreak(HAILO_INFER_FAIL_RESET_S)
    warn = _WarnThrottle()
    _track_streak(gen, "detector", streak)

    try:
        while _live(gen):
            loop_start = time.monotonic()

            with frame_lock:
                mosaic = current_frame
                fid = current_frame_id
                frame_t = current_frame_t
                simulated = current_frame_simulated

            if mosaic is None or fid == last_id:
                time.sleep(0.002)
                continue
            last_id = fid
            if not is_fresh(frame_t, SIMPLE_CAMERA_FRESH_S):
                continue

            seg_viewing = SEG_VIEW_PRIORITY and _seg_view_active()

            npu_ok = False  # an inference succeeded this pass
            npu_error = None  # ...or the one that raised
            npu_started = None  # time.monotonic() that call began
            try:
                total_inference_ms = 0.0
                if seg_viewing:
                    sectors = ["front"]
                elif not DETECTION_FRONT_EVERY_PASS:
                    sectors = list(_VALID)
                else:
                    # Front first (2026-10-30): only the front's detections reach the voice,
                    # and a side's inference ahead of it costs the front ~40 ms. Every picture
                    # holds all four cameras: the front on each, and a side camera at most
                    # every DETECTION_SIDE_MIN_INTERVAL_S, the one longest undetected first,
                    # so each is detected again within 3 x that (labels expire at 2 s).
                    sectors = ["front"]
                    now_m = time.monotonic()
                    if now_m - last_side >= DETECTION_SIDE_MIN_INTERVAL_S:
                        side = min(_SIDES, key=lambda s: side_done.get(s, float("-inf")))
                        sectors.append(side)
                        last_side = side_done[side] = now_m
                with _gen_lock:
                    if not _live(gen):
                        return None
                    if seg_viewing:
                        _note_coverage(_COVERAGE_SEG_VIEW, ("front",))
                    else:
                        _note_coverage(*_all_coverage())

                for sector in sectors:
                    quad = _sector_slice(mosaic, sector)  # view, free
                    view = _view_check(sector, quad, mosaic)
                    npu_started = time.monotonic()
                    try:
                        detections, inference_ms = engine.infer_array(quad, bgr=SWAP_RB)
                    except Exception as e:
                        npu_error = f"{type(e).__name__}: {e}"
                        raise
                    npu_ok = True
                    shot_t = frame_t  # every camera is in every picture
                    if TOF_ENABLED and sector == "front" and not simulated:
                        try:  # each box's measured distance (app/perception/depth_fusion.py)
                            from app.perception.depth_fusion import annotate

                            annotate(detections, shot_t)
                        except Exception as e:
                            warn(f"[WARN] ToF depth: {type(e).__name__}: {e}")
                    entry = {
                        "detections": detections,
                        "num_objects": len(detections),
                        "frame_time": shot_t,
                        "frame_id": fid,  # the picture these were found in (/camera/live)
                        "simulated": simulated,
                        # Can this camera see at all (a black or blurred picture still
                        # gets a detection pass — it just names nothing)?
                        "view": {
                            "brightness": view["brightness"],
                            "sharpness": view["sharpness"],
                            "lux": view["lux"],
                            "dark": view["dark"],
                            "blurred": view["blurred"],
                            "dim": view["dim"],
                        },
                    }
                    total_inference_ms += inference_ms
                    with _gen_lock:
                        if not _live(gen):
                            return None
                        detection_stats[sector] = entry
                        detection_stats["total_objects"] = sum(
                            detection_stats[s]["num_objects"] for s in _VALID
                        )
                        update_camera_state(
                            {
                                sector: detection_stats[sector],
                                "total_objects": detection_stats["total_objects"],
                            }
                        )
                    with _new_detection:
                        _new_detection.notify_all()

                with _gen_lock:
                    if not _live(gen):
                        return None
                    if npu_ok:
                        detection_stats["inference_time"] = total_inference_ms

                        fps_stats["detection_times"].append(time.monotonic())
                        if len(fps_stats["detection_times"]) >= 2:
                            t = list(fps_stats["detection_times"])
                            fps_stats["detection_fps"] = (len(t) - 1) / (t[-1] - t[0])

                        update_camera_state({"inference_time": total_inference_ms})

            except Exception as e:
                warn(f"[WARN] Inference error: {e}")

            if npu_ok:
                streak.reset()
            elif npu_error is not None:
                streak.fail(npu_error, npu_started)
                if streak.broken():
                    reason = streak.describe()
                    with _gen_lock:
                        if gen == _generation:
                            hailo_ready = False
                            hailo_error = reason
                            _note_coverage(_COVERAGE_NOT_READY, ())
                    return reason

            elapsed = time.monotonic() - loop_start
            sleep_time = FRAME_INTERVAL - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        return None
    finally:
        _track_streak(gen, "detector", None)
        _close_engine(engine, "Hailo engine")




def _make_segmenter():
    from app.sensors.seg_inference import SegmentationEngine
    from app.sensors.hailo_device import get_shared_vdevice

    return SegmentationEngine(SEG_HEF_PATH, vdevice=get_shared_vdevice())


def _segmenter_up(engine):  # under _gen_lock, generation current
    global seg_engine, seg_error, seg_init_attempts
    seg_init_attempts += 1
    seg_error = None
    seg_engine = engine


def _segmenter_failed(error):  # under _gen_lock, generation current
    global seg_error, seg_init_attempts
    seg_init_attempts += 1
    seg_error = error


def _retire_seg_engine(eng):
    """Unpublish a segmenter (only if it is still the published one — a newer generation's must not be taken down with it) and close it."""
    global seg_engine
    with _gen_lock:
        if seg_engine is eng:
            seg_engine = None
    _close_engine(eng, "Seg engine")
    for sector, entry in list(_seg_view_cache.items()):
        if entry.get("engine") is not None and entry["engine"]() is eng:
            _seg_view_cache.pop(sector, None)


def _seg_released(eng) -> bool:
    """Whether a closed segmenter has actually let go of the NPU."""
    return bool(getattr(eng, "released", True))


def _wait_seg_released(gen, eng) -> bool:
    """Wait, in the same 0.25 s slices as _retry_wait, until `eng` is released."""
    global seg_error
    if _seg_released(eng):
        return True
    with _gen_lock:
        if gen == _generation:
            seg_error = ((seg_error + "; ") if seg_error else "") + (
                "waiting for the previous segmenter to be released (a debug "
                "view's inference is still running on it)"
            )
    _log.warning(
        "Seg engine: the old engine is still inside an inference — "
        "not re-initialising until it is released"
    )
    while _live(gen):
        if _seg_released(eng):
            return True
        time.sleep(0.25)
    return False


def _seg_thread():
    """Segment the FRONT quadrant into walkable surface while its debug view is open (or always, with SEG_ALWAYS): a display only — nothing spoken uses it."""
    if not SEG_ENABLED:
        return

    gen = _my_generation()
    failures = 0
    broke = False  # broke mid-run and not up again since
    while True:
        if broke and not _wait_npu_answers(gen, "Seg"):
            return
        eng, error, live = _attempt_init(
            gen, "Seg", _make_segmenter, _segmenter_up, _segmenter_failed
        )
        if not live:
            return
        if eng is None:
            delay = _backoff(failures)
            failures += 1
            _log.warning(
                f"Seg engine init failed (attempt {seg_init_attempts}; walking "
                f"guide off): {error} — retrying in {delay:g}s"
            )
            if not _retry_wait(gen, delay):
                return
            continue

        _log.info(f"[Seg] Walkable-surface guide ready: {SEG_HEF_PATH}")
        failures, broke = 0, False
        try:
            reason = _segment(eng, gen)
        finally:
            _track_streak(gen, "seg", None)
            _retire_seg_engine(eng)
        if reason is None:
            return
        broke = True
        if not _wait_seg_released(gen, eng):
            return
        eng = None  # closed and released: not kept through the backoff
        delay = _backoff(failures)
        failures += 1
        _log.warning(
            f"Seg engine reset — {reason}; walking guide off, "
            f"re-initialising in {delay:g}s"
        )
        if not _retry_wait(gen, delay):
            return


def _segment(eng, gen):
    """The segmentation loop on one engine, until stopped, superseded or broken."""
    global seg_latest, seg_engine, seg_error

    min_period = 1.0 / SEG_TARGET_FPS if SEG_TARGET_FPS > 0 else 0.0
    last_id = -1
    streak = _FailStreak(HAILO_INFER_FAIL_RESET_S)
    warn = _WarnThrottle()
    _track_streak(gen, "seg", streak)  # withdrawn by _seg_thread

    while _live(gen):
        # Monotonic pacing, as in _detect (F20).
        loop_start = time.monotonic()

        if not SEG_ALWAYS and not _seg_front_view_active():
            fps_stats["seg_times"].clear()
            fps_stats["seg_fps"] = 0
            time.sleep(0.1)
            continue

        with frame_lock:
            mosaic = current_frame
            fid = current_frame_id
            frame_t = current_frame_t

        if mosaic is None or fid == last_id:
            time.sleep(0.002)
            continue
        last_id = fid
        if not is_fresh(frame_t, SIMPLE_CAMERA_FRESH_S):
            continue

        npu_ok = False
        npu_error = None
        npu_started = None
        try:
            front_pil = Image.fromarray(_to_seg_input(_sector_slice(mosaic, "front")))
            npu_started = time.monotonic()
            try:
                class_map, infer_ms = eng.infer_class_map(front_pil)
            except Exception as e:
                npu_error = f"{type(e).__name__}: {e}"
                raise
            npu_ok = True

            with _gen_lock:
                if not _live(gen):
                    return None
                with seg_latest_lock:
                    seg_latest = {
                        "class_map": class_map,
                        "frame_id": fid,
                        "infer_ms": infer_ms,
                    }
                fps_stats["seg_times"].append(time.monotonic())
                if len(fps_stats["seg_times"]) >= 2:
                    t = list(fps_stats["seg_times"])
                    fps_stats["seg_fps"] = (len(t) - 1) / (t[-1] - t[0])
        except Exception as e:
            warn(f"[WARN] Segmentation error: {e}")

        if npu_ok:
            streak.reset()
        elif npu_error is not None:
            streak.fail(npu_error, npu_started)
            if streak.broken():
                reason = streak.describe()
                with _gen_lock:
                    if gen == _generation:
                        seg_error = reason
                        if seg_engine is eng:
                            seg_engine = None
                return reason

        # Pace so segmentation cannot monopolise the shared NPU.
        spare = min_period - (time.monotonic() - loop_start)
        if spare > 0:
            time.sleep(spare)
    return None


# ============== FRAME / STATS ACCESSORS ==============


_live_lock = threading.Lock()
_live_cache = {}  # (view, width) -> (frame id, jpeg, picture time, simulated)


# The front camera is detected on every picture: the live picture waits this long, at most,
# for that picture's detections, so its boxes are its own (~50 ms after capture, measured).
_LIVE_DETECTION_WAIT_S = 0.15


def live_detections(view: str, fid: int, wait_s: float = _LIVE_DETECTION_WAIT_S) -> dict:
    """{camera: {"frame_id", "frame_time", "detections"}} for the camera(s) in picture `fid`
    of `view` (one camera, or all four for "grid"). The front's waits up to `wait_s` for this
    picture's own; the side cameras', detected every ~1.2 s, are their latest — an older
    picture's, which frame_id says (the dashboard draws those as old)."""
    cameras = _VALID if view == "grid" else (view,)
    deadline = time.monotonic() + max(0.0, wait_s)
    if "front" in cameras and hailo_ready:  # no detector: nothing to wait for
        with _new_detection:
            while (detection_stats["front"].get("frame_id") or 0) < fid and hailo_ready:
                left = deadline - time.monotonic()
                if left <= 0 or not _streams_live():
                    break
                _new_detection.wait(left)
    with _gen_lock:
        return {
            c: {
                "frame_id": detection_stats[c].get("frame_id"),
                "frame_time": detection_stats[c].get("frame_time"),
                "detections": list(detection_stats[c].get("detections") or []),
            }
            for c in cameras
        }


def live_jpeg(view: str, after: int = -1, wait_s: float = 1.0, width: int = STREAM_MAX_WIDTH):
    """The newest picture of one camera — or "grid", all four in their 2x2 — as a small
    JPEG: (jpeg, frame id, picture time, simulated), or None when no picture newer than
    `after` came within `wait_s`. Pulled one at a time (/camera/live), so unlike the MJPEG
    stream nothing queues between here and the screen, and it cannot fall behind
    (2026-11-02: the stream did, over Wi-Fi). Each picture is encoded once for every viewer."""
    if view != "grid" and view not in _VALID:
        view = "front"
    width = int(max(64, min(width, CAMERA_WIDTH)))
    deadline = time.monotonic() + max(0.0, wait_s)
    with _new_frame:
        while True:
            with frame_lock:
                mosaic, fid = current_frame, current_frame_id
                shot, simulated = current_frame_t, current_frame_simulated
            if mosaic is not None and fid != after:
                break
            left = deadline - time.monotonic()
            if left <= 0 or not _streams_live():
                return None
            _new_frame.wait(left)
    key = (view, width)
    with _live_lock:
        hit = _live_cache.get(key)
    if hit is not None and hit[0] == fid:
        return hit[1], fid, hit[2], hit[3]
    picture = mosaic if view == "grid" else _sector_slice(mosaic, view)
    jpeg = _encode_jpeg_stream(picture, "BGR" if SWAP_RB else "RGB", width)
    with _live_lock:
        _live_cache[key] = (fid, jpeg, shot, simulated)
    return jpeg, fid, shot, simulated




def _seg_view_active():
    """True if a seg-view stream asked for a frame within the last ~1.5 s (drives the auto detection-throttle in _inference_thread)."""
    return is_fresh(seg_view_active_ts, 1.5)


def _seg_front_view_active():
    """True if the FRONT seg view was open within the last ~1.5 s — the only thing that needs _seg_thread running when SEG_ALWAYS is off."""
    return is_fresh(seg_front_view_ts, 1.5)


_seg_view_cache = {}  # sector -> {"engine", "frame_id", "class_map", "t"}
_seg_view_sector_locks = {s: threading.Lock() for s in _VALID}


def _seg_view_map(eng, sector, mosaic, fid):
    """The class map of `sector` for the debug views: at most ONE inference per sector per frame however many viewers ask, and no more often than SEG_VIEW_FPS per sector in all."""
    period = 1.0 / SEG_VIEW_FPS if SEG_VIEW_FPS > 0 else 0.0
    lock = _seg_view_sector_locks[sector]
    while not lock.acquire(timeout=0.25):
        if not _streams_live():
            return None, None
    try:
        c = _seg_view_cache.get(sector)
        if (
            c is not None
            and c["engine"]() is eng
            and (c["frame_id"] >= fid or time.monotonic() - c["t"] < period)
        ):
            return c["class_map"], c["frame_id"]
        pil = Image.fromarray(_to_seg_input(_sector_slice(mosaic, sector)))
        class_map, _ = eng.infer_class_map(pil)
        _seg_view_cache[sector] = {
            "engine": weakref.ref(eng),
            "frame_id": fid,
            "class_map": class_map,
            "t": time.monotonic(),
        }
        return class_map, fid
    finally:
        lock.release()


def gen_seg_frames(sector: str = "front"):
    """MJPEG stream of the COLORIZED segmentation for one sector — a separate debug view, does not touch the normal camera feed."""
    global seg_view_active_ts, seg_front_view_ts
    from app.sensors.seg_inference import colorize_class_map

    if sector not in _VALID:
        sector = "front"

    last_id = -1
    last_yield_at = time.monotonic()  # an interval only (F20)
    while _streams_live():
        eng = seg_engine  # snapshot the shared engine

        class_map = None
        if sector == "front":
            seg_front_view_ts = time.time()
            with seg_latest_lock:
                if seg_latest["frame_id"] != last_id:
                    class_map = seg_latest["class_map"]
                    last_id = seg_latest["frame_id"]
        else:
            if eng is not None:
                seg_view_active_ts = time.time()
            with frame_lock:
                mosaic = current_frame
                fid = current_frame_id
            if eng is not None and mosaic is not None and fid != last_id:
                try:
                    class_map, map_id = _seg_view_map(eng, sector, mosaic, fid)
                    if map_id is None or map_id == last_id:
                        class_map = None  # nothing this stream has not shown
                    else:
                        last_id = map_id
                except Exception as e:
                    last_id = fid  # not retried until a new frame
                    _log.warning(f"Seg-view error ({sector}): {e}")

        if class_map is not None and eng is not None:
            try:
                jpeg = _encode_jpeg_stream(
                    colorize_class_map(class_map, eng.walkable_ids)
                )
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
                last_yield_at = time.monotonic()
            except Exception as e:
                _log.warning(f"Seg-view encode error ({sector}): {e}")

        if time.monotonic() - last_yield_at > STREAM_STALL_TIMEOUT_S:
            _log.warning(
                f"Seg stream ({sector}) produced nothing for "
                f"{STREAM_STALL_TIMEOUT_S:g}s — ending the stream"
            )
            return

        time.sleep(0.01 if sector == "front" else max(0.05, 1.0 / SEG_VIEW_FPS))




def get_camera_stats():
    """Get current camera and detection statistics."""
    with _gen_lock:
        npu = {
            "hailo_ready": hailo_ready,
            "hailo_error": hailo_error,
            "hailo_init_attempts": hailo_init_attempts,
            "hailo_failing_s": _streak_seconds(_streaks["detector"]),
            "seg_ready": seg_engine is not None,
            "seg_error": seg_error,
            "seg_init_attempts": seg_init_attempts,
            "seg_failing_s": _streak_seconds(_streaks["seg"]),
        }
    with frame_lock:
        simulated = current_frame_simulated
    return {
        "capture_fps": fps_stats["capture_fps"],
        "detection_fps": fps_stats["detection_fps"],
        "seg_fps": fps_stats["seg_fps"],
        "total_objects": detection_stats["total_objects"],
        "inference_time": detection_stats["inference_time"],
        **npu,
        "camera_simulated": simulated,
        "camera_source": _camera_source,
        "capture_stalled": capture_stalled,
        "detection_coverage": detection_coverage(),
        "views": camera_views(),
        "light": light_reading(),  # the sensor's, for the whole 2x2 picture
        "stall_reopens": _stall_reopens,
        "camera_px": list(_QUAD),  # one camera's picture, width x height
        "focus_code": _focus_code,  # lens position set at start (None: not set)
        "front": detection_stats["front"],
        "right": detection_stats["right"],
        "left": detection_stats["left"],
        "back": detection_stats["back"],
    }


def _streak_seconds(streak):
    s = streak.failing_s() if streak is not None else None
    return None if s is None else round(s, 1)


def get_frame_payload(sector: str) -> dict:
    """{"frame": base64 JPEG or None, "simulated": bool} for one sector."""
    if sector not in _VALID:
        return {"frame": None, "simulated": False}

    with frame_lock:
        mosaic = current_frame
        simulated = current_frame_simulated

    if mosaic is None:
        return {"frame": None, "simulated": False}

    try:
        quad = _sector_slice(mosaic, sector)
        jpeg = _encode_jpeg(_to_seg_input(quad))
        return {"frame": base64.b64encode(jpeg).decode("utf-8"), "simulated": simulated}
    except Exception as e:
        _log.warning(f"Frame encode error: {e}")
        return {"frame": None, "simulated": simulated}
