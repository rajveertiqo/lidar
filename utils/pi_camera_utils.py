"""Pi Camera utility functions and state management."""

import atexit
import io
import threading
import time
import base64
import weakref
from collections import deque

import numpy as np
from PIL import Image

try:
    import cv2 as _cv2
except Exception:
    _cv2 = None
try:
    import simplejpeg as _simplejpeg
except Exception:
    _simplejpeg = None

from config import (
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
    STREAM_FPS,
    STREAM_STALL_TIMEOUT_S,
    CAMERA_CAPTURE_FPS,
    SEG_VIEW_FPS,
    SEG_VIEW_PRIORITY,
    ENV_TRUST_ENABLED,
    ENV_ANALYZE_HZ,
    SEG_ALWAYS,
    DETECTION_FRONT_EVERY_PASS,
    CAMERA_FRAME_TIMEOUT_S,
    CAMERA_STOP_TIMEOUT_S,
    CAMERA_RETRY_BACKOFF_S,
    HAILO_RETRY_BACKOFF_S,
    HAILO_INFER_FAIL_RESET_S,
    HAILO_SEG_RUN_TIMEOUT_MS,
    HAILO_VDEVICE_RESET,
    HAILO_VDEVICE_RESET_AFTER,
    HAILO_VDEVICE_RESET_WAIT_S,
    HAILO_PROBE_TIMEOUT_S,
    SIMPLE_CAMERA_FRESH_S,
    CAMERA_SINGLE_CHANNEL,
    CAMERA_HAT_ADDRESS,
    CAMERA_HAT_CHANNEL,
    CAMERA_HAT_I2C_BUS,
    CAMERA_CORRUPT_PCT,
    CAMERA_CORRUPT_RESTART_S,
    CAMERA_CORRUPT_RESTART_SHARE,
    CAMERA_CORRUPT_WINDOW_S,
    CAMERA_FALLBACK_RESTARTS,
    CAMERA_FALLBACK_WINDOW_S,
    CAMERA_FRONT_RESTART_S,
    CAMERA_CYCLE,
    CAMERA_CYCLE_FRONT_FRAMES,
    CAMERA_VIEW_CHECK_S,
    CAMERA_DARK_LEVEL,
    CAMERA_DARK_CONFIRM_S,
    CAMERA_BLUR_VAR,
    CAMERA_BLUR_CONFIRM_S,
    CAMERA_STALL_REOPEN_S,
    CAMERA_STALL_REOPEN_MAX_S,
    TOF_ENABLED,
)

from utils.freshness import is_fresh
from utils.shared_state import update_camera_state

SWAP_RB = True

from config import CAMERA_SIMULATION_ALLOWED  # noqa: E402

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

# The camera in use alone, or None for all four: CAMERA_SINGLE_CHANNEL, until the
# 4-in-1 picture proves unusable and the front camera is fallen back to (_is_corrupt).
_single = CAMERA_SINGLE_CHANNEL
_fell_back = False
# Cycling one camera at a time (CAMERA_CYCLE): which cameras the board has (found
# once, at the first camera start) and the order they are visited in.
_cycle = bool(CAMERA_CYCLE) and not CAMERA_SINGLE_CHANNEL
_cycle_present = None
_cycle_order = []  # [(sector, frames), ...]
current_quad_meta = {}  # cycling: sector -> {"t", "id"} of its newest frame
_quad_times = {s: deque(maxlen=60) for s in ("front", "right", "left", "back")}


def _pass_sectors(pass_no: int) -> tuple:
    """The cameras one detection pass runs, in order."""
    if _single:
        return (_single,)  # the other quadrants are black
    if not DETECTION_FRONT_EVERY_PASS:
        return _VALID
    others = tuple(s for s in _VALID if s != "front")
    return ("front", others[pass_no % len(others)])


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


def _encode_jpeg_stream(arr, colorspace="RGB"):
    """Encode a DOWNSCALED, lower-quality JPEG for the dashboard MJPEG stream."""
    h, w = arr.shape[:2]
    size = (
        (STREAM_MAX_WIDTH, max(1, int(h * STREAM_MAX_WIDTH / w)))
        if w > STREAM_MAX_WIDTH
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
            print(
                f"[ERROR] picamera2 is not importable ({e}) — NO CAMERA. Publishing "
                f"nothing; health reports the camera lost and object "
                f"identification is off. (Development simulation needs "
                f"SHAPPE_SIMULATE_CAMERA=1.)"
            )
        else:
            _camera_source = "simulated"
            print(
                f"[WARN] picamera2 not installed ({e}) and SHAPPE_SIMULATE_CAMERA=1: "
                f"SIMULATED camera frames — flagged `simulated`, never evidence "
                f"of anything"
            )
            _start_thread(_simulate_thread)
    else:
        _camera_source = "hardware"
        _start_thread(_capture_supervised)

    _start_thread(_inference_thread)

    _start_thread(_seg_thread)
    print(f"🧠 Hailo local inference: {HEF_PATH}")
    print(f"🎯 Target: {TARGET_FPS} FPS")


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
    # Leave the board showing all four cameras, as other tools (rpicam-hello,
    # camera_web.py) expect; cycling or single mode left it on one.
    try:
        _write_board(0x00)
        print("[CAMERA] board left showing all four cameras (2x2)")
    except Exception as e:
        print(
            f"[WARN] Camera board not reset to 2x2 ({type(e).__name__}: {e}) — "
            f"run: i2cset -y -f {CAMERA_HAT_I2C_BUS} 0x24 0x24 0x00"
        )


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
            print(f"[WARN] {what} failed: {type(exc).__name__}: {exc}")
        finally:
            done.set()

    threading.Thread(target=run, name=what.replace(" ", "-"), daemon=True).start()
    if done.wait(timeout_s):
        return True
    print(
        f"[WARN] {what} still blocked after {timeout_s:g}s — abandoning it "
        f"so shutdown can finish"
    )
    return False


# ============== FRAME CAPTURE ==============

_camera_lock = threading.Lock()
_QUAD = (CAMERA_WIDTH // FRAME_GRID_COLS, CAMERA_HEIGHT // FRAME_GRID_ROWS)


_hat = None  # smbus2.SMBus to the board, kept open while cycling


def _board_value(sector) -> int:
    """The board register value that shows this camera alone."""
    return 0x02 | CAMERA_HAT_CHANNEL[sector] << 4


def _write_board(value) -> None:
    """One write to the board's mode register (capture thread only)."""
    global _hat
    if _hat is None:
        from smbus2 import SMBus

        _hat = SMBus(CAMERA_HAT_I2C_BUS)
    try:
        _hat.write_byte_data(CAMERA_HAT_ADDRESS, 0x24, value)
    except Exception:
        _hat.close()
        _hat = None
        raise


def _set_board_mode() -> bool:
    """Put the camera board in the mode the camera opens in (it forgets it on a power cycle). False if the write failed."""
    if _single:
        value, what = _board_value(_single), f"only the {_single} camera"
    elif _cycle:
        value = _board_value(_cycle_order[0][0])
        what = "one camera at a time: " + ", ".join(s for s, _ in _cycle_order)
    else:
        value, what = 0x00, "all four cameras (2x2)"
    try:
        _write_board(value)
        time.sleep(0.3)  # let it switch before streaming
        print(f"[CAMERA] board set to {what} (0x{value:02x})")
        return True
    except Exception as e:
        print(
            f"[WARN] Camera board mode not set ({type(e).__name__}: {e}) — wanted {what}"
        )
        return False


class _NoPicture(RuntimeError):
    """The camera gave no picture at all while the board was being searched."""


def _find_cameras(Picamera2) -> tuple:
    """Which channels have a camera, from the 2x2 picture: a tile that is not black in a
    clean picture, or bright on average (scrambled data spills ~25 into an empty tile).
    Unsure means absent: visiting an empty channel stops the stream for good."""
    _write_board(0x00)
    time.sleep(0.3)
    clean = {s: [] for s in _VALID}
    seen = {s: [] for s in _VALID}
    cam = Picamera2()
    try:
        cam.configure(
            cam.create_video_configuration(
                main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"},
                raw={"size": (CAMERA_WIDTH, CAMERA_HEIGHT)},
                buffer_count=4,
            )
        )
        cam.start()
        time.sleep(1.0)
        for _ in range(20):
            # Bounded (2026-10-23): a plain capture_array() waits for ever, and a camera
            # that had stalled hung the capture thread here for the rest of the session.
            try:
                frame = cam.wait(cam.capture_array("main", wait=False),
                                 timeout=CAMERA_FRAME_TIMEOUT_S)
            except TimeoutError:
                raise _NoPicture(f"no picture within {CAMERA_FRAME_TIMEOUT_S:g}s") from None
            for s in _VALID:
                quad = _sector_slice(frame, s)
                level = float(quad[::4, ::4].mean())
                seen[s].append(level)
                if _corrupt_pct(quad) <= CAMERA_CORRUPT_PCT:
                    clean[s].append(level)
    finally:
        _close_camera(cam)
    present = tuple(
        s
        for s in _VALID
        if (clean[s] and max(clean[s]) > 3.0)
        or (seen[s] and float(np.median(seen[s])) > 40.0)
    )
    print(
        f"[CAMERA] cameras on the board: {', '.join(present) or 'none'}"
        + (
            f" (no camera: {', '.join(s for s in _VALID if s not in present)})"
            if len(present) < len(_VALID)
            else ""
        )
    )
    return present


def _schedule(present) -> list:
    """The cycle: the front's CAMERA_CYCLE_FRONT_FRAMES frames before each other camera's one."""
    others = [s for s in ("right", "left", "back") if s in present]
    if "front" not in present:
        return [(s, 1) for s in others]
    if not others:
        return [("front", 1)]
    return [
        slot for s in others for slot in (("front", CAMERA_CYCLE_FRONT_FRAMES), (s, 1))
    ]


def _store_quad(sector, arr) -> None:
    """Put one camera's new frame in its quadrant of a copy of the mosaic (readers may
    still hold the old one), with its own capture time for the detector's freshness."""
    global current_frame, current_frame_id, current_frame_t, current_frame_simulated
    global current_quad_meta
    now = time.time()
    prev = current_frame  # the capture thread is the only writer
    if prev is None or prev.shape != (CAMERA_HEIGHT, CAMERA_WIDTH, 3):
        mosaic = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=arr.dtype)
    else:
        mosaic = prev.copy()
    row, col = FRAME_SECTOR_LAYOUT[sector]
    qw, qh = _QUAD
    mosaic[row * qh : (row + 1) * qh, col * qw : (col + 1) * qw] = arr[:qh, :qw]
    meta = dict(current_quad_meta)
    meta[sector] = {"t": now, "id": (meta.get(sector) or {}).get("id", 0) + 1}
    with frame_lock:
        current_frame = mosaic
        current_frame_id += 1
        current_frame_t = now
        current_frame_simulated = False
        current_quad_meta = meta
    _quad_times[sector].append(time.monotonic())


def _in_mosaic(arr):
    """A single camera's frame, in its own quadrant of an otherwise black mosaic."""
    mosaic = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=arr.dtype)
    row, col = FRAME_SECTOR_LAYOUT[_single]
    qw, qh = _QUAD
    mosaic[row * qh : (row + 1) * qh, col * qw : (col + 1) * qw] = arr[:qh, :qw]
    return mosaic


_corrupt_lock = threading.Lock()
_corrupt_hist = {s: deque() for s in ("front", "right", "left", "back")}  # (mono, bad)
_reopen = threading.Event()  # set: the capture thread restarts the camera
_want_single = None  # ...as this camera alone ("front"), if set
_front_restarts = deque()  # monotonic times the front camera forced a restart
_last_reopen = float("-inf")  # monotonic


def _corrupt_pct(quad) -> float:
    """Speckle in one camera's picture, %: pixels (every 2nd) differing by more than 70
    from BOTH neighbours, which a real scene, even blurred, almost never has and the
    board's scrambled 4-in-1 data is full of. ~12 ms on a 960x540 quadrant."""
    s = quad[::2, ::2]
    if _cv2 is not None:
        dx = _cv2.absdiff(s[:, 1:], s[:, :-1]).max(axis=2)
        dy = _cv2.absdiff(s[1:, :], s[:-1, :]).max(axis=2)
    else:
        a = s.astype(np.int16)
        dx = np.abs(np.diff(a, axis=1)).max(axis=2)
        dy = np.abs(np.diff(a, axis=0)).max(axis=2)
    return float(((dx[:-1, :] > 70) & (dy[:, :-1] > 70)).mean() * 100.0)


def _is_corrupt(sector, quad) -> bool:
    """Is this picture scrambled board data? Kept per camera; one corrupted most of the
    time asks the capture thread to restart the camera (CAMERA_CORRUPT_RESTART_*)."""
    global _last_reopen, _want_single
    bad = _corrupt_pct(quad) > CAMERA_CORRUPT_PCT
    now = time.monotonic()
    with _corrupt_lock:
        hist = _corrupt_hist[sector]
        hist.append((now, bad))
        while hist and now - hist[0][0] > CAMERA_CORRUPT_WINDOW_S:
            hist.popleft()
        share = sum(b for _, b in hist) / len(hist)
        if (
            len(hist) < 10
            or share < CAMERA_CORRUPT_RESTART_SHARE
            or now - _last_reopen
            < (
                CAMERA_FRONT_RESTART_S
                if sector == "front"
                else CAMERA_CORRUPT_RESTART_S
            )
        ):
            return bad
        _last_reopen = now
        for h in _corrupt_hist.values():
            h.clear()
        if sector == "front":
            _front_restarts.append(now)
            while (
                _front_restarts and now - _front_restarts[0] > CAMERA_FALLBACK_WINDOW_S
            ):
                _front_restarts.popleft()
        if _single is None and len(_front_restarts) >= CAMERA_FALLBACK_RESTARTS:
            _front_restarts.clear()
            _want_single = "front"
            print(
                f"[CAMERA] the front camera needed {CAMERA_FALLBACK_RESTARTS} restarts in "
                f"{CAMERA_FALLBACK_WINDOW_S:.0f} s and is still corrupted — falling back to "
                f"the front camera alone until the app restarts"
            )
        else:
            print(
                f"[CAMERA] {sector} camera corrupted in {share:.0%} of its pictures — "
                f"restarting the camera (the board's 4-in-1 merge)"
            )
        _reopen.set()
    return bad


_view_lock = threading.Lock()
_views = {}  # sector -> what _view_check last measured


def _view_check(sector, quad) -> dict:
    """Can this camera see? Picture level and sharpness, measured at most every
    CAMERA_VIEW_CHECK_S; "dark" / "blurred" only once they have lasted their
    CAMERA_*_CONFIRM_S. ~2 ms on a 960x540 quadrant."""
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
    with _view_lock:
        v = _views.get(sector) or {"dark_since": None, "blur_since": None}
        dark_now = level < CAMERA_DARK_LEVEL
        blur_now = not dark_now and sharpness < CAMERA_BLUR_VAR
        dark_since = (v["dark_since"] or now) if dark_now else None
        blur_since = (v["blur_since"] or now) if blur_now else None
        v = {
            "mono": now,
            "at": time.time(),
            "brightness": round(level, 1),
            "sharpness": round(sharpness, 1),
            "dark_since": dark_since,
            "blur_since": blur_since,
            "dark": dark_since is not None
            and now - dark_since >= CAMERA_DARK_CONFIRM_S,
            "blurred": blur_since is not None
            and now - blur_since >= CAMERA_BLUR_CONFIRM_S,
        }
        _views[sector] = v
        return dict(v)


def camera_views() -> dict:
    """What each camera's picture looked like at its last check (see _view_check)."""
    with _view_lock:
        return {
            s: {k: v[k] for k in ("at", "brightness", "sharpness", "dark", "blurred")}
            for s, v in _views.items()
        }


def corruption() -> dict:
    """Each camera's share of corrupted pictures over the last CAMERA_CORRUPT_WINDOW_S."""
    now = time.monotonic()
    with _corrupt_lock:
        return {
            s: {
                "share": round(
                    sum(b for t, b in h if now - t <= CAMERA_CORRUPT_WINDOW_S)
                    / max(
                        1, sum(1 for t, _ in h if now - t <= CAMERA_CORRUPT_WINDOW_S)
                    ),
                    2,
                ),
                "pictures": sum(1 for t, _ in h if now - t <= CAMERA_CORRUPT_WINDOW_S),
            }
            for s, h in _corrupt_hist.items()
        }


def _open_camera():
    """Open, configure and start the camera, retrying until it works or we are stopped."""
    global picam2, _cycle, _cycle_present, _cycle_order
    from picamera2 import Picamera2

    if _cycle and not _single and _cycle_present is None:
        tries = 0
        while running:
            try:
                _cycle_present = _find_cameras(Picamera2)
                _cycle_order = _schedule(_cycle_present)
                break
            except _NoPicture as e:
                # A stalled camera is not a board without cameras: search again, as a
                # camera that will not open is retried, rather than give up on cycling.
                delay = CAMERA_RETRY_BACKOFF_S[min(tries, len(CAMERA_RETRY_BACKOFF_S) - 1)]
                tries += 1
                print(f"[WARN] Camera search: {e} — the camera is not delivering; "
                      f"searching again in {delay:g}s")
                deadline = time.monotonic() + delay
                while running and time.monotonic() < deadline:
                    time.sleep(0.25)
            except Exception as e:
                print(
                    f"[WARN] Could not tell which cameras the board has ({type(e).__name__}: "
                    f"{e}) — using the 2x2 picture"
                )
                _cycle_order = []
                break
        if not running:
            return False
        if not _cycle_order:
            _cycle = False
    # In an unknown mode no picture can be said to come from any one camera: filed under
    # the wrong direction, a person behind is "ahead". So no picture until it is set, as
    # for a camera that will not open (scan 2026-10-23).
    tries = 0
    while not _set_board_mode():
        delay = CAMERA_RETRY_BACKOFF_S[min(tries, len(CAMERA_RETRY_BACKOFF_S) - 1)]
        tries += 1
        print(f"[WARN] No camera pictures until the board mode is set; retrying in {delay:g}s")
        deadline = time.monotonic() + delay
        while running and time.monotonic() < deadline:
            time.sleep(0.25)
        if not running:
            return False
    # One camera at a time: the ISP halves it to a quadrant's size, to drop into the mosaic.
    main_size = _QUAD if (_single or _cycle) else (CAMERA_WIDTH, CAMERA_HEIGHT)
    failures = 0
    while running:
        cam = None
        try:
            cam = Picamera2()
            cam.configure(
                cam.create_video_configuration(
                    main={"size": main_size, "format": "RGB888"},
                    raw={"size": (CAMERA_WIDTH, CAMERA_HEIGHT)},
                    buffer_count=4,  # small buffer -> low latency, always-newest
                    controls={"FrameRate": float(CAMERA_CAPTURE_FPS)},
                )
            )
            # Let the sensor run as fast as the mode allows; capture loop paces us.
            cam.start()
        except Exception as e:
            if cam is not None:
                _close_camera(cam)  # half-opened: release the nodes
            delay = CAMERA_RETRY_BACKOFF_S[
                min(failures, len(CAMERA_RETRY_BACKOFF_S) - 1)
            ]
            failures += 1
            print(
                f"[WARN] Camera would not open ({type(e).__name__}: {e}) — no frames "
                f"(health reports the camera lost); retrying in {delay:g}s"
            )
            deadline = time.monotonic() + delay
            while running and time.monotonic() < deadline:
                time.sleep(0.25)
            continue
        with _camera_lock:
            if running:
                picam2 = cam
                print(
                    f"✅ Camera (Picamera2): {CAMERA_WIDTH}x{CAMERA_HEIGHT}, "
                    f"raw numpy capture"
                    + (
                        f" — {_single} camera only"
                        if _single
                        else " — cycling the cameras"
                        if _cycle
                        else ""
                    )
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
            print(
                f"[WARN] Camera capture thread failed ({type(e).__name__}: {e}) — "
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
    """Pull raw mosaic frames from Picamera2 into the shared numpy buffer."""
    global current_frame, current_frame_id, current_frame_t, capture_stalled
    global current_frame_simulated, picam2, _single, _fell_back, _want_single
    global _cycle, current_quad_meta, _cycle_present, _stall_reopens

    if picam2 is None and not _open_camera():
        return
    job = None
    asked_at = 0.0
    last_error_at = None
    unreadable_since = None  # cycling pictures unreadable since, in a row
    stall_wait = CAMERA_STALL_REOPEN_S  # no frame this long: reopen (doubles each time)

    def cycle_start():
        """(slot, camera, frames still wanted from it, when it was switched to)."""
        return 0, _cycle_order[0][0], _cycle_order[0][1], time.monotonic_ns()

    slot, sector, wanted, switched = cycle_start() if _cycle else (0, None, 0, 0)
    while running:
        if _reopen.is_set():  # its pictures kept arriving corrupted
            _reopen.clear()
            job = None
            with _corrupt_lock:
                if _want_single:
                    _single, _fell_back, _want_single = _want_single, True, None
                    for h in _corrupt_hist.values():
                        h.clear()
                    with frame_lock:
                        current_quad_meta = {}
            with _camera_lock:
                cam, picam2 = picam2, None
            if cam is not None:
                _close_camera(cam)
            if not _open_camera():
                return
            if _cycle and not _single:
                slot, sector, wanted, switched = cycle_start()
            continue
        cycling = _cycle and not _single
        try:
            if job is None:
                job = (
                    picam2.capture_request(wait=False)
                    if cycling
                    else picam2.capture_array("main", wait=False)
                )
                asked_at = time.monotonic()
            arr = picam2.wait(job, timeout=0.25)
        except TimeoutError:
            waited = time.monotonic() - asked_at
            if not capture_stalled and waited > CAMERA_FRAME_TIMEOUT_S:
                capture_stalled = True
                print(
                    f"[WARN] Camera: no frame for {waited:.0f}s — the camera is not "
                    f"delivering (still waiting on it; health reports it lost)"
                )
            if waited > stall_wait:
                # A stuck stream does not come back by itself (2026-10-09): start it again.
                _stall_reopens += 1
                print(
                    f"[CAMERA] no frame for {waited:.0f}s — closing and reopening the "
                    f"camera (reopen {_stall_reopens}; next after "
                    f"{min(stall_wait * 2.0, CAMERA_STALL_REOPEN_MAX_S):.0f}s)"
                )
                stall_wait = min(stall_wait * 2.0, CAMERA_STALL_REOPEN_MAX_S)
                if _cycle and not _single:
                    _cycle_present = None  # look again for which cameras answer
                _reopen.set()
            continue
        except Exception as e:
            job = None
            if running and (
                last_error_at is None
                or time.monotonic() - last_error_at > CAMERA_FRAME_TIMEOUT_S
            ):
                last_error_at = time.monotonic()
                print(f"[WARN] Capture error: {e}")
            time.sleep(0.05)
            continue

        job = None
        stall_wait = CAMERA_STALL_REOPEN_S
        if capture_stalled:
            capture_stalled = False
            print(
                f"[OK] Camera delivering again after {time.monotonic() - asked_at:.1f}s"
            )
        if cycling:
            request, arr = arr, None
            try:
                stamp = request.get_metadata().get("SensorTimestamp", 0)
                arr = request.make_array("main")
            except Exception as e:
                # One bad request is one lost picture, not the end of capture (2026-10-21).
                if (
                    last_error_at is None
                    or time.monotonic() - last_error_at > CAMERA_FRAME_TIMEOUT_S
                ):
                    last_error_at = time.monotonic()
                    print(
                        f"[WARN] Camera picture unreadable ({type(e).__name__}: {e}) — skipped"
                    )
            finally:
                request.release()
            if arr is None:
                # It still counts as that camera's turn, so the board moves on and the other
                # cameras keep coming; unreadable for as long as a stall, it is reopened.
                if unreadable_since is None:
                    unreadable_since = time.monotonic()
                elif time.monotonic() - unreadable_since > stall_wait:
                    _stall_reopens += 1
                    print(
                        f"[CAMERA] pictures unreadable for "
                        f"{time.monotonic() - unreadable_since:.0f}s — closing and "
                        f"reopening the camera (reopen {_stall_reopens})"
                    )
                    unreadable_since = None
                    _cycle_present = None
                    _reopen.set()
                    continue
            else:
                unreadable_since = None
                if stamp <= switched:
                    continue  # exposed before the switch: the last camera's
                _store_quad(sector, arr)
            wanted -= 1
            if wanted <= 0:
                slot = (slot + 1) % len(_cycle_order)
                nxt, wanted = _cycle_order[slot]
                if nxt != sector:
                    try:
                        _write_board(_board_value(nxt))
                    except Exception as e:
                        print(
                            f"[WARN] Camera switch failed ({type(e).__name__}: {e}) — "
                            f"using the 2x2 picture instead"
                        )
                        _cycle = False
                        with frame_lock:
                            current_quad_meta = {}
                        _reopen.set()
                        continue
                    switched, sector = time.monotonic_ns(), nxt
            fps_stats["capture_times"].append(time.monotonic())
            if len(fps_stats["capture_times"]) >= 2:
                t = list(fps_stats["capture_times"])
                fps_stats["capture_fps"] = (len(t) - 1) / (t[-1] - t[0])
            continue
        if _single:
            arr = _in_mosaic(arr)

        with frame_lock:
            current_frame = arr
            current_frame_id += 1
            current_frame_t = time.time()
            current_frame_simulated = False  # a real frame, in the same write

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
_COVERAGE_SINGLE = "single_camera"  # CAMERA_SINGLE_CHANNEL set
_COVERAGE_MISSING = "cameras_missing"  # cycling, some channels empty
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
    """(reason, sectors) when detection runs on every camera there is."""
    if _single:
        return ("camera_fallback" if _fell_back else _COVERAGE_SINGLE, (_single,))
    if _cycle and _cycle_present is not None:
        return (
            (
                _COVERAGE_FULL
                if set(_cycle_present) == set(_VALID)
                else _COVERAGE_MISSING
            ),
            tuple(_cycle_present),
        )
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
                print(f"[{what}] the NPU answers again — rebuilding the engine")
            return True
        delay = _backoff(tries)
        tries += 1
        print(
            f"[ERROR] {what}: the NPU does not answer (hailortcli fw-control identify, "
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
        print(f"[WARN] {what} close failed: {type(e).__name__}: {e}")


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
        print(
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
        print(msg + extra)
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
    from utils.hailo_inference import HailoInferenceEngine
    from utils.hailo_device import get_shared_vdevice

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
                    print(
                        f"[ERROR] Hailo detector would not come back ({rebuild_fails} tries: "
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
                print(
                    f"[ERROR] Hailo engine init failed (attempt {hailo_init_attempts}): {error} "
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
            print(
                f"[ERROR] Hailo detector reset — {reason}; detection off, "
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
    from utils.hailo_device import release_shared_vdevice

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
            print(
                f"[ERROR] NPU restart: {', '.join(stuck)} still inside a call after "
                f"{HAILO_VDEVICE_RESET_WAIT_S:g}s — NOT releasing the device (that could "
                f"crash the app); starting the workers on it again"
            )
        else:
            # A segmentation run that outlived close() releases its model when it returns.
            time.sleep(HAILO_SEG_RUN_TIMEOUT_MS / 1000.0)
            ok = release_shared_vdevice()
            print(
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
    pass_no = 0
    detected_id = {}  # cycling: the quad_meta id each camera was last detected at
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
                quad_meta = current_quad_meta

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
                sectors = ("front",) if seg_viewing else _pass_sectors(pass_no)
                if quad_meta:  # cycling: each camera's new frame, once
                    sectors = [
                        s
                        for s in _VALID
                        if s in quad_meta
                        and quad_meta[s]["id"] != detected_id.get(s)
                        and (s == "front" or not seg_viewing)
                    ][:2]
                pass_no += 1
                with _gen_lock:
                    if not _live(gen):
                        return None
                    if seg_viewing:
                        _note_coverage(_COVERAGE_SEG_VIEW, ("front",))
                    else:
                        _note_coverage(*_all_coverage())

                for sector in sectors:
                    quad = _sector_slice(mosaic, sector)  # view, free
                    # Scrambled board data (2026-10-08) never reaches the detector; this
                    # camera's last detections just age out.
                    if not simulated and _is_corrupt(sector, quad):
                        # Handled too (2026-10-23): left unmarked, the same flagged picture
                        # was judged again every pass — counted many times toward the
                        # restart share (16% read as 63%), and re-chosen by the [:2] pick
                        # ahead of the other cameras' new pictures, which starved them.
                        if quad_meta:
                            detected_id[sector] = quad_meta[sector]["id"]
                        continue
                    view = _view_check(sector, quad)
                    npu_started = time.monotonic()
                    try:
                        detections, inference_ms = engine.infer_array(quad, bgr=SWAP_RB)
                    except Exception as e:
                        npu_error = f"{type(e).__name__}: {e}"
                        raise
                    npu_ok = True

                    if quad_meta:
                        detected_id[sector] = quad_meta[sector]["id"]
                    shot_t = quad_meta[sector]["t"] if quad_meta else frame_t
                    if TOF_ENABLED and sector == "front" and not simulated:
                        try:  # each box's measured distance (utils/depth_fusion.py)
                            from utils.depth_fusion import annotate

                            annotate(detections, shot_t)
                        except Exception as e:
                            warn(f"[WARN] ToF depth: {type(e).__name__}: {e}")
                    entry = {
                        "detections": detections,
                        "num_objects": len(detections),
                        "frame_time": shot_t,
                        "simulated": simulated,
                        # Can this camera see at all (a black or blurred picture still
                        # gets a detection pass — it just names nothing)?
                        "view": {
                            "brightness": view["brightness"],
                            "sharpness": view["sharpness"],
                            "dark": view["dark"],
                            "blurred": view["blurred"],
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

                with _gen_lock:
                    if not _live(gen):
                        return None
                    if npu_ok:  # every picture corrupted: not a pass
                        detection_stats["inference_time"] = total_inference_ms

                        fps_stats["detection_times"].append(time.monotonic())
                        if len(fps_stats["detection_times"]) >= 2:
                            t = list(fps_stats["detection_times"])
                            fps_stats["detection_fps"] = (len(t) - 1) / (t[-1] - t[0])

                        update_camera_state({"inference_time": total_inference_ms})
                _refresh_environment(mosaic)

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


_last_env_analysis = 0.0


def _refresh_environment(mosaic):
    """Keep the fog/rain/darkness reading current, on the detection thread."""
    global _last_env_analysis
    if not ENV_TRUST_ENABLED:
        return
    now = time.time()
    if (now - _last_env_analysis) < (1.0 / max(1e-3, ENV_ANALYZE_HZ)):
        return
    _last_env_analysis = now
    try:
        from utils.environment_detection import detect_environment_conditions

        detect_environment_conditions(image=_to_bgr(mosaic))
    except Exception as e:
        print(f"[WARN] environment analysis failed: {e}")


def _make_segmenter():
    from utils.seg_inference import SegmentationEngine
    from utils.hailo_device import get_shared_vdevice

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
    print(
        "[WARN] Seg engine: the old engine is still inside an inference — "
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
            print(
                f"[WARN] Seg engine init failed (attempt {seg_init_attempts}; walking "
                f"guide off): {error} — retrying in {delay:g}s"
            )
            if not _retry_wait(gen, delay):
                return
            continue

        print(f"[Seg] Walkable-surface guide ready: {SEG_HEF_PATH}")
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
        print(
            f"[WARN] Seg engine reset — {reason}; walking guide off, "
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


def gen_frames(sector: str = "front"):
    """MJPEG generator for one sector."""
    if sector not in _VALID:
        sector = "front"

    last_id = -1
    last_frame_at = time.monotonic()  # an interval only (F20)
    while _streams_live():
        with frame_lock:
            mosaic = current_frame
            fid = current_frame_id

        if mosaic is not None and fid != last_id:
            last_id = fid
            last_frame_at = time.monotonic()
            try:
                quad = _sector_slice(mosaic, sector)
                # The quadrant as captured: BGR when SWAP_RB, with no copy.
                jpeg = _encode_jpeg_stream(quad, "BGR" if SWAP_RB else "RGB")
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
            except Exception as e:
                print(f"[WARN] Stream encode error: {e}")
        elif time.monotonic() - last_frame_at > STREAM_STALL_TIMEOUT_S:
            print(
                f"[WARN] Camera stream ({sector}) had no new frame for "
                f"{STREAM_STALL_TIMEOUT_S:g}s — ending the stream"
            )
            return

        # Pace to the (lower) display framerate — saves encode CPU and WiFi.
        time.sleep(max(0.005, 1.0 / STREAM_FPS))


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
    from utils.seg_inference import colorize_class_map

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
                    print(f"[WARN] Seg-view error ({sector}): {e}")

        if class_map is not None and eng is not None:
            try:
                jpeg = _encode_jpeg_stream(
                    colorize_class_map(class_map, eng.walkable_ids)
                )
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
                last_yield_at = time.monotonic()
            except Exception as e:
                print(f"[WARN] Seg-view encode error ({sector}): {e}")

        if time.monotonic() - last_yield_at > STREAM_STALL_TIMEOUT_S:
            print(
                f"[WARN] Seg stream ({sector}) produced nothing for "
                f"{STREAM_STALL_TIMEOUT_S:g}s — ending the stream"
            )
            return

        time.sleep(0.01 if sector == "front" else max(0.05, 1.0 / SEG_VIEW_FPS))


def gen_seg_grid(cell=(240, 135)):
    """MJPEG stream of a 2x2 grid (front/right/left/back) of the colorized segmentation — all four sectors in ONE stream."""
    from utils.seg_inference import colorize_class_map
    from PIL import ImageDraw

    global seg_view_active_ts, seg_front_view_ts

    cw, ch = cell
    last_id = -1
    last_yield_at = time.monotonic()  # an interval only (F20)
    while _streams_live():
        eng = seg_engine
        if eng is not None:
            seg_view_active_ts = time.time()
        seg_front_view_ts = time.time()
        with frame_lock:
            mosaic = current_frame
            fid = current_frame_id

        if eng is not None and mosaic is not None and fid != last_id:
            last_id = fid
            try:
                cells = {}
                for sector in _VALID:
                    class_map = None
                    if sector == "front":
                        with seg_latest_lock:
                            class_map = seg_latest["class_map"]
                    else:
                        class_map, _ = _seg_view_map(eng, sector, mosaic, fid)

                    if class_map is None:
                        tile = Image.new("RGB", (cw, ch))
                    else:
                        tile = Image.fromarray(
                            colorize_class_map(class_map, eng.walkable_ids)
                        ).resize((cw, ch))
                    ImageDraw.Draw(tile).text(
                        (5, 4), sector.upper(), fill=(255, 255, 255)
                    )
                    cells[sector] = np.asarray(tile)

                top = np.hstack([cells["front"], cells["right"]])
                bot = np.hstack([cells["left"], cells["back"]])
                grid = np.vstack([top, bot])

                buf = io.BytesIO()
                Image.fromarray(grid).save(buf, format="JPEG", quality=STREAM_QUALITY)
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                    + buf.getvalue()
                    + b"\r\n"
                )
                last_yield_at = time.monotonic()
            except Exception as e:
                print(f"[WARN] Seg-grid error: {e}")

        # Same rule as the other two streams — see STREAM_STALL_TIMEOUT_S.
        if time.monotonic() - last_yield_at > STREAM_STALL_TIMEOUT_S:
            print(
                f"[WARN] Seg-grid stream produced nothing for "
                f"{STREAM_STALL_TIMEOUT_S:g}s — ending the stream"
            )
            return

        time.sleep(max(0.05, 1.0 / SEG_VIEW_FPS))


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
        "corrupt": corruption(),
        "views": camera_views(),
        "stall_reopens": _stall_reopens,
        "camera_mode": ("single" if _single else "cycle" if _cycle else "2x2"),
        "cycle_cameras": list(_cycle_present or []) if _cycle else [],
        "camera_fps": {
            s: (
                round((len(t) - 1) / (t[-1] - t[0]), 1)
                if len(t) >= 2 and t[-1] > t[0]
                else 0.0
            )
            for s, t in ((k, list(v)) for k, v in _quad_times.items())
        },
        "front": detection_stats["front"],
        "right": detection_stats["right"],
        "left": detection_stats["left"],
        "back": detection_stats["back"],
    }


def _streak_seconds(streak):
    s = streak.failing_s() if streak is not None else None
    return None if s is None else round(s, 1)


def get_current_frame():
    """The latest full mosaic in BGR order, or None before the first frame."""
    with frame_lock:
        mosaic = current_frame
    if mosaic is None:
        return None
    return _to_bgr(mosaic)


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
        print(f"[WARN] Frame encode error: {e}")
        return {"frame": None, "simulated": simulated}
