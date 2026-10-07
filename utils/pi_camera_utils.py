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
    CAMERA_WIDTH, CAMERA_HEIGHT, JPEG_QUALITY, TARGET_FPS,
    FRAME_INTERVAL, HEF_PATH, DETECTION_CONF, DETECTION_IOU,
    FRAME_GRID_ROWS, FRAME_GRID_COLS, FRAME_SECTOR_LAYOUT,
    SEG_ENABLED, SEG_HEF_PATH, SEG_TARGET_FPS,
    STREAM_MAX_WIDTH, STREAM_QUALITY, STREAM_FPS, STREAM_STALL_TIMEOUT_S,
    CAMERA_CAPTURE_FPS,
    SEG_VIEW_FPS, SEG_VIEW_PRIORITY,
    ENV_TRUST_ENABLED, ENV_ANALYZE_HZ,
    SEG_ALWAYS, DETECTION_FRONT_EVERY_PASS,
    CAMERA_FRAME_TIMEOUT_S, CAMERA_STOP_TIMEOUT_S, CAMERA_RETRY_BACKOFF_S,
    HAILO_RETRY_BACKOFF_S, HAILO_INFER_FAIL_RESET_S,
    SIMPLE_CAMERA_FRESH_S,
)

from utils.freshness import is_fresh
from utils.shared_state import update_camera_state

SWAP_RB = True

from config import CAMERA_SIMULATION_ALLOWED  # noqa: E402

# ============== GLOBAL STATE ==============
picam2 = None
current_frame = None          # latest full mosaic as a numpy array (H, W, 3)
current_frame_id = 0          # increments every new capture
current_frame_t = 0.0         # time.time() when current_frame was captured
current_frame_simulated = False  # current_frame was FABRICATED (_simulate_thread);
                              # written with the frame, under frame_lock
capture_stalled = False       # no frame for CAMERA_FRAME_TIMEOUT_S (see _capture_thread)
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
hailo_init_attempts = 0       # detector init attempts since start_camera()
seg_engine = None
seg_error = None              # same as hailo_error, for the segmenter
seg_init_attempts = 0         # same as hailo_init_attempts, for the segmenter
seg_view_active_ts = 0.0
seg_front_view_ts = 0.0

seg_latest = {"class_map": None, "frame_id": -1, "infer_ms": 0.0}
seg_latest_lock = threading.Lock()

fps_stats = {
    "capture_fps":     0,
    "detection_fps":   0,
    "seg_fps":         0,
    "capture_times":   deque(maxlen=30),
    "detection_times": deque(maxlen=30),
    "seg_times":       deque(maxlen=30),
}

detection_stats = {
    "front":         {"detections": [], "num_objects": 0},
    "right":         {"detections": [], "num_objects": 0},
    "left":          {"detections": [], "num_objects": 0},
    "back":          {"detections": [], "num_objects": 0},
    "total_objects": 0,
    "inference_time": 0,
}

_VALID = ("front", "right", "left", "back")


def _pass_sectors(pass_no: int) -> tuple:
    """The cameras one detection pass runs, in order."""
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
    return mosaic[y1:y1 + quad_h, x1:x1 + quad_w]


def _encode_jpeg(rgb_arr):
    """Encode an RGB numpy array to JPEG bytes."""
    buf = io.BytesIO()
    Image.fromarray(rgb_arr).save(buf, format="JPEG", quality=JPEG_QUALITY)
    return buf.getvalue()


def _encode_jpeg_stream(arr, colorspace="RGB"):
    """Encode a DOWNSCALED, lower-quality JPEG for the dashboard MJPEG stream."""
    h, w = arr.shape[:2]
    size = ((STREAM_MAX_WIDTH, max(1, int(h * STREAM_MAX_WIDTH / w)))
            if w > STREAM_MAX_WIDTH else None)
    if _simplejpeg is not None and _cv2 is not None:
        if size:
            arr = _cv2.resize(arr, size, interpolation=_cv2.INTER_AREA)
        return _simplejpeg.encode_jpeg(np.ascontiguousarray(arr), quality=STREAM_QUALITY,
                                       colorspace=colorspace, colorsubsampling="420")
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
    _stream_shutdown.clear()      # a restart must not inherit the last stop's flag

    try:
        import picamera2  # noqa: F401 — only asking whether it is installed
    except ImportError as e:
        if not CAMERA_SIMULATION_ALLOWED:
            _camera_source = f"unavailable: picamera2 not importable ({e})"
            print(f"[ERROR] picamera2 is not importable ({e}) — NO CAMERA. Publishing "
                  f"nothing; health reports the camera lost and object "
                  f"identification is off. (Development simulation needs "
                  f"SHAPPE_SIMULATE_CAMERA=1.)")
        else:
            _camera_source = "simulated"
            print(f"[WARN] picamera2 not installed ({e}) and SHAPPE_SIMULATE_CAMERA=1: "
                  f"SIMULATED camera frames — flagged `simulated`, never evidence "
                  f"of anything")
            _start_thread(_simulate_thread)
    else:
        _camera_source = "hardware"
        _start_thread(_capture_thread)

    _start_thread(_inference_thread)

    _start_thread(_seg_thread)
    print(f"🧠 Hailo local inference: {HEF_PATH}")
    print(f"🎯 Target: {TARGET_FPS} FPS")


def stop_camera():
    """Stop camera capture and inference threads. Bounded: never hangs."""
    global running, picam2
    running = False
    _new_generation()             # hailo_ready off now; nothing publishes after
    begin_shutdown()              # in case we got here without a signal
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
            print(f"[WARN] {what} failed: {type(exc).__name__}: {exc}")
        finally:
            done.set()

    threading.Thread(target=run, name=what.replace(" ", "-"), daemon=True).start()
    if done.wait(timeout_s):
        return True
    print(f"[WARN] {what} still blocked after {timeout_s:g}s — abandoning it "
          f"so shutdown can finish")
    return False


# ============== FRAME CAPTURE ==============

_camera_lock = threading.Lock()


def _open_camera():
    """Open, configure and start the camera, retrying until it works or we are stopped."""
    global picam2
    from picamera2 import Picamera2

    failures = 0
    while running:
        cam = None
        try:
            cam = Picamera2()
            cam.configure(cam.create_video_configuration(
                main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"},
                raw={"size": (CAMERA_WIDTH, CAMERA_HEIGHT)},
                buffer_count=4,          # small buffer -> low latency, always-newest
                controls={"FrameRate": float(CAMERA_CAPTURE_FPS)},
            ))
            # Let the sensor run as fast as the mode allows; capture loop paces us.
            cam.start()
        except Exception as e:
            if cam is not None:
                _close_camera(cam)          # half-opened: release the nodes
            delay = CAMERA_RETRY_BACKOFF_S[min(failures, len(CAMERA_RETRY_BACKOFF_S) - 1)]
            failures += 1
            print(f"[WARN] Camera would not open ({type(e).__name__}: {e}) — no frames "
                  f"(health reports the camera lost); retrying in {delay:g}s")
            deadline = time.monotonic() + delay
            while running and time.monotonic() < deadline:
                time.sleep(0.25)
            continue
        with _camera_lock:
            if running:
                picam2 = cam
                print(f"✅ Camera (Picamera2): {CAMERA_WIDTH}x{CAMERA_HEIGHT}, "
                      f"raw numpy capture")
                return True
        _close_camera(cam)                  # stopped while it opened
        return False
    return False


def _capture_thread():
    """Pull raw mosaic frames from Picamera2 into the shared numpy buffer."""
    global current_frame, current_frame_id, current_frame_t, capture_stalled
    global current_frame_simulated

    if picam2 is None and not _open_camera():
        return
    job = None
    asked_at = 0.0
    last_error_at = None
    while running:
        try:
            if job is None:
                job = picam2.capture_array("main", wait=False)
                asked_at = time.monotonic()
            arr = picam2.wait(job, timeout=0.25)
        except TimeoutError:
            waited = time.monotonic() - asked_at
            if not capture_stalled and waited > CAMERA_FRAME_TIMEOUT_S:
                capture_stalled = True
                print(f"[WARN] Camera: no frame for {waited:.0f}s — the camera is not "
                      f"delivering (still waiting on it; health reports it lost)")
            continue
        except Exception as e:
            job = None
            if running and (last_error_at is None
                            or time.monotonic() - last_error_at > CAMERA_FRAME_TIMEOUT_S):
                last_error_at = time.monotonic()
                print(f"[WARN] Capture error: {e}")
            time.sleep(0.05)
            continue

        job = None
        if capture_stalled:
            capture_stalled = False
            print(f"[OK] Camera delivering again after {time.monotonic() - asked_at:.1f}s")

        with frame_lock:
            current_frame = arr
            current_frame_id += 1
            current_frame_t = time.time()
            current_frame_simulated = False       # a real frame, in the same write

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
    base[:hh, :hw] = (40, 40, 40)      # top-left
    base[:hh, hw:] = (80, 80, 80)      # top-right
    base[hh:, :hw] = (120, 120, 120)   # bottom-left
    base[hh:, hw:] = (160, 160, 160)   # bottom-right

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
_gen_lock = threading.RLock()     # guards _generation and every NPU publish

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
        _streaks.update(detector=None, seg=None)     # those engines are retired too
        if reset_status:
            hailo_error, hailo_init_attempts = None, 0
            seg_error, seg_init_attempts = None, 0
        _note_coverage(_COVERAGE_NOT_READY, ())
        return _generation


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
    return {"reduced": c["reason"] != _COVERAGE_FULL, "reason": c["reason"],
            "sectors_detected": list(c["sectors"]), "since": c["since"]}


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
        print(f"[{what}] came up after its worker was stopped or replaced — "
              f"closing it unused")
        _close_engine(engine, what)
        return None, None, False
    return engine, None, True


class _WarnThrottle:
    """print() a warning at most once per _WARN_REPEAT_S per distinct message; the repeats in between are counted into the next print of it."""

    def __init__(self, interval_s=None):
        self.interval_s = _WARN_REPEAT_S if interval_s is None else interval_s
        self._seen = {}             # message -> [last printed at, repeats since]

    def __call__(self, msg: str) -> bool:
        now = time.monotonic()        # an interval only (F20)
        seen = self._seen.get(msg)
        if seen is not None and now - seen[0] < self.interval_s:
            seen[1] += 1
            return False
        if seen is None and len(self._seen) >= 16:
            # A message carrying a varying detail must not grow this for ever.
            self._seen = {m: s for m, s in self._seen.items()
                          if now - s[0] < self.interval_s}
        extra = (f" (+{seen[1]} more in the last {now - seen[0]:.0f}s)"
                 if seen is not None and seen[1] else "")
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
        if (self.since is None
                or (self._last_end is not None
                    and started - self._last_end > self.limit_s)):
            self.since, self.count = started, 0
        self.count += 1
        self.last = error
        self._last_end = now

    def failing_s(self):
        """Seconds since the first failing call of this streak began, or None."""
        since = self.since            # once: get_camera_stats reads from another thread
        return None if since is None else time.monotonic() - since

    def broken(self) -> bool:
        return self.count >= 1 and self.failing_s() >= self.limit_s

    def describe(self) -> str:
        return (f"every inference failed for {self.failing_s():.1f}s "
                f"({self.count} call{'' if self.count == 1 else 's'}): {self.last}")


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


def _detector_up(engine):             # under _gen_lock, generation current
    global hailo_ready, hailo_error, hailo_init_attempts
    hailo_init_attempts += 1
    hailo_error = None
    hailo_ready = True
    _note_coverage(_COVERAGE_FULL, _VALID)


def _detector_failed(error):          # under _gen_lock, generation current
    global hailo_ready, hailo_error, hailo_init_attempts
    hailo_init_attempts += 1
    hailo_error = error
    hailo_ready = False


def _inference_thread():
    """Run Hailo inference on the 4 quadrants."""
    global hailo_ready

    gen = _my_generation()
    failures = 0                  # consecutive failures: the backoff index
    try:
        while True:
            engine, error, live = _attempt_init(gen, "Hailo", _make_detector,
                                                _detector_up, _detector_failed)
            if not live:
                return
            if engine is None:
                delay = _backoff(failures)
                failures += 1
                print(f"[ERROR] Hailo engine init failed (attempt {hailo_init_attempts}): {error} "
                      f"— detection off, hailo_ready False; retrying in {delay:g}s")
                if not _retry_wait(gen, delay):
                    return
                continue

            failures = 0          # it came up: a later failure retries quickly
            reason = _detect(engine, gen)          # closes the engine on exit
            engine = None
            if reason is None:
                return            # stopped, or superseded by a newer start
            delay = _backoff(failures)
            failures += 1
            print(f"[ERROR] Hailo detector reset — {reason}; detection off, "
                  f"re-initialising in {delay:g}s")
            if not _retry_wait(gen, delay):
                return
    finally:
        with _gen_lock:
            if gen == _generation:      # a newer worker's flag is not ours
                hailo_ready = False
                _note_coverage(_COVERAGE_NOT_READY, ())


def _detect(engine, gen):
    """Detection passes on one engine until stopped, superseded or broken."""
    global hailo_ready, hailo_error

    last_id = -1
    pass_no = 0
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

            npu_ok = False            # an inference succeeded this pass
            npu_error = None          # ...or the one that raised
            npu_started = None        # time.monotonic() that call began
            try:
                total_inference_ms = 0.0
                sectors = ("front",) if seg_viewing else _pass_sectors(pass_no)
                pass_no += 1
                with _gen_lock:
                    if not _live(gen):
                        return None
                    if seg_viewing:
                        _note_coverage(_COVERAGE_SEG_VIEW, ("front",))
                    else:
                        _note_coverage(_COVERAGE_FULL, _VALID)

                for sector in sectors:
                    quad = _sector_slice(mosaic, sector)      # view, free
                    npu_started = time.monotonic()
                    try:
                        detections, inference_ms = engine.infer_array(quad, bgr=SWAP_RB)
                    except Exception as e:
                        npu_error = f"{type(e).__name__}: {e}"
                        raise
                    npu_ok = True

                    entry = {
                        "detections":  detections,
                        "num_objects": len(detections),
                        "frame_time":  frame_t,
                        "simulated":   simulated,
                    }
                    total_inference_ms += inference_ms
                    with _gen_lock:
                        if not _live(gen):
                            return None
                        detection_stats[sector] = entry
                        detection_stats["total_objects"] = sum(
                            detection_stats[s]["num_objects"] for s in _VALID)
                        update_camera_state({
                            sector: detection_stats[sector],
                            "total_objects": detection_stats["total_objects"],
                        })

                with _gen_lock:
                    if not _live(gen):
                        return None
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


def _segmenter_up(engine):            # under _gen_lock, generation current
    global seg_engine, seg_error, seg_init_attempts
    seg_init_attempts += 1
    seg_error = None
    seg_engine = engine


def _segmenter_failed(error):         # under _gen_lock, generation current
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
                "view's inference is still running on it)")
    print("[WARN] Seg engine: the old engine is still inside an inference — "
          "not re-initialising until it is released")
    while _live(gen):
        if _seg_released(eng):
            return True
        time.sleep(0.25)
    return False


def _seg_thread():
    """Continuously segment the FRONT quadrant and publish the steering cue."""
    if not SEG_ENABLED:
        return

    gen = _my_generation()
    failures = 0
    while True:
        eng, error, live = _attempt_init(gen, "Seg", _make_segmenter,
                                         _segmenter_up, _segmenter_failed)
        if not live:
            return
        if eng is None:
            delay = _backoff(failures)
            failures += 1
            print(f"[WARN] Seg engine init failed (attempt {seg_init_attempts}; walking "
                  f"guide off): {error} — retrying in {delay:g}s")
            if not _retry_wait(gen, delay):
                return
            continue

        print(f"[Seg] Walkable-surface guide ready: {SEG_HEF_PATH}")
        failures = 0
        try:
            reason = _segment(eng, gen)
        finally:
            _track_streak(gen, "seg", None)
            _retire_seg_engine(eng)
        if reason is None:
            return
        if not _wait_seg_released(gen, eng):
            return
        eng = None                # closed and released: not kept through the backoff
        delay = _backoff(failures)
        failures += 1
        print(f"[WARN] Seg engine reset — {reason}; walking guide off, "
              f"re-initialising in {delay:g}s")
        if not _retry_wait(gen, delay):
            return


def _segment(eng, gen):
    """The segmentation loop on one engine, until stopped, superseded or broken."""
    global seg_latest, seg_engine, seg_error

    min_period = 1.0 / SEG_TARGET_FPS if SEG_TARGET_FPS > 0 else 0.0
    last_id = -1
    streak = _FailStreak(HAILO_INFER_FAIL_RESET_S)
    warn = _WarnThrottle()
    _track_streak(gen, "seg", streak)            # withdrawn by _seg_thread

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
                    seg_latest = {"class_map": class_map, "frame_id": fid,
                                  "infer_ms": infer_ms}
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
    last_frame_at = time.monotonic()      # an interval only (F20)
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
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + jpeg + b'\r\n')
            except Exception as e:
                print(f"[WARN] Stream encode error: {e}")
        elif time.monotonic() - last_frame_at > STREAM_STALL_TIMEOUT_S:
            print(f"[WARN] Camera stream ({sector}) had no new frame for "
                  f"{STREAM_STALL_TIMEOUT_S:g}s — ending the stream")
            return

        # Pace to the (lower) display framerate — saves encode CPU and WiFi.
        time.sleep(max(0.005, 1.0 / STREAM_FPS))


def _seg_view_active():
    """True if a seg-view stream asked for a frame within the last ~1.5 s (drives the auto detection-throttle in _inference_thread)."""
    return is_fresh(seg_view_active_ts, 1.5)


def _seg_front_view_active():
    """True if the FRONT seg view was open within the last ~1.5 s — the only thing that needs _seg_thread running when SEG_ALWAYS is off."""
    return is_fresh(seg_front_view_ts, 1.5)


_seg_view_cache = {}                  # sector -> {"engine", "frame_id", "class_map", "t"}
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
        if (c is not None and c["engine"]() is eng
                and (c["frame_id"] >= fid or time.monotonic() - c["t"] < period)):
            return c["class_map"], c["frame_id"]
        pil = Image.fromarray(_to_seg_input(_sector_slice(mosaic, sector)))
        class_map, _ = eng.infer_class_map(pil)
        _seg_view_cache[sector] = {"engine": weakref.ref(eng), "frame_id": fid,
                                   "class_map": class_map, "t": time.monotonic()}
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
    last_yield_at = time.monotonic()      # an interval only (F20)
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
                        class_map = None        # nothing this stream has not shown
                    else:
                        last_id = map_id
                except Exception as e:
                    last_id = fid               # not retried until a new frame
                    print(f"[WARN] Seg-view error ({sector}): {e}")

        if class_map is not None and eng is not None:
            try:
                jpeg = _encode_jpeg_stream(
                    colorize_class_map(class_map, eng.walkable_ids))
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + jpeg + b'\r\n')
                last_yield_at = time.monotonic()
            except Exception as e:
                print(f"[WARN] Seg-view encode error ({sector}): {e}")

        if time.monotonic() - last_yield_at > STREAM_STALL_TIMEOUT_S:
            print(f"[WARN] Seg stream ({sector}) produced nothing for "
                  f"{STREAM_STALL_TIMEOUT_S:g}s — ending the stream")
            return

        time.sleep(0.01 if sector == "front" else max(0.05, 1.0 / SEG_VIEW_FPS))


def gen_seg_grid(cell=(240, 135)):
    """MJPEG stream of a 2x2 grid (front/right/left/back) of the colorized segmentation — all four sectors in ONE stream."""
    from utils.seg_inference import colorize_class_map
    from PIL import ImageDraw

    global seg_view_active_ts, seg_front_view_ts

    cw, ch = cell
    last_id = -1
    last_yield_at = time.monotonic()      # an interval only (F20)
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
                            colorize_class_map(class_map, eng.walkable_ids)).resize((cw, ch))
                    ImageDraw.Draw(tile).text((5, 4), sector.upper(), fill=(255, 255, 255))
                    cells[sector] = np.asarray(tile)

                top = np.hstack([cells["front"], cells["right"]])
                bot = np.hstack([cells["left"], cells["back"]])
                grid = np.vstack([top, bot])

                buf = io.BytesIO()
                Image.fromarray(grid).save(buf, format="JPEG", quality=STREAM_QUALITY)
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + buf.getvalue() + b'\r\n')
                last_yield_at = time.monotonic()
            except Exception as e:
                print(f"[WARN] Seg-grid error: {e}")

        # Same rule as the other two streams — see STREAM_STALL_TIMEOUT_S.
        if time.monotonic() - last_yield_at > STREAM_STALL_TIMEOUT_S:
            print(f"[WARN] Seg-grid stream produced nothing for "
                  f"{STREAM_STALL_TIMEOUT_S:g}s — ending the stream")
            return

        time.sleep(max(0.05, 1.0 / SEG_VIEW_FPS))


def get_camera_stats():
    """Get current camera and detection statistics."""
    with _gen_lock:
        npu = {
            "hailo_ready":         hailo_ready,
            "hailo_error":         hailo_error,
            "hailo_init_attempts": hailo_init_attempts,
            "hailo_failing_s":     _streak_seconds(_streaks["detector"]),
            "seg_ready":           seg_engine is not None,
            "seg_error":           seg_error,
            "seg_init_attempts":   seg_init_attempts,
            "seg_failing_s":       _streak_seconds(_streaks["seg"]),
        }
    with frame_lock:
        simulated = current_frame_simulated
    return {
        "capture_fps":    fps_stats["capture_fps"],
        "detection_fps":  fps_stats["detection_fps"],
        "seg_fps":        fps_stats["seg_fps"],
        "total_objects":  detection_stats["total_objects"],
        "inference_time": detection_stats["inference_time"],
        **npu,
        "camera_simulated": simulated,
        "camera_source":  _camera_source,
        "capture_stalled": capture_stalled,
        "detection_coverage": detection_coverage(),
        "front":          detection_stats["front"],
        "right":          detection_stats["right"],
        "left":           detection_stats["left"],
        "back":           detection_stats["back"],
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
        return {"frame": base64.b64encode(jpeg).decode("utf-8"),
                "simulated": simulated}
    except Exception as e:
        print(f"[WARN] Frame encode error: {e}")
        return {"frame": None, "simulated": simulated}
