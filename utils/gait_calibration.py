"""Runtime home for the walking stride."""

import json
import os
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

from config import (
    GYRO_STRIDE_DEFAULT_M,
    GYRO_STRIDE_MAX_M,
    GYRO_STRIDE_MIN_M,
)

STRIDE_FILE = Path(__file__).resolve().parent.parent / "stride.json"

_SAVE_MIN_INTERVAL_S = 30.0
_SAVE_MIN_DELTA_M = 0.005

_lock = threading.Lock()
_state: Optional[Dict[str, Any]] = None  # what is on disk, or {} if nothing
_last_save = 0.0
# The file itself (taken after _lock, never before it), and which write is the newest: a
# learned stride is written off the LiDAR thread, and must not land over a newer one.
_file_lock = threading.Lock()
_write_seq = 0

_NEW_FILE_MODE = 0o644

_OWNER = "_owner"

_operator_override = False


def _load() -> Dict[str, Any]:
    """Whatever is on disk, or {} if nothing has been measured yet."""
    global _state
    if _state is not None:
        return _state
    data: Dict[str, Any] = {_OWNER: "operator"}
    try:
        raw = json.loads(STRIDE_FILE.read_text(encoding="utf-8"))
        if isinstance(raw, dict) and raw.get("source") == "learned":
            data[_OWNER] = "learned"
        value = raw.get("stride_m")
        if value is not None:
            stride = float(value)
            if GYRO_STRIDE_MIN_M <= stride <= GYRO_STRIDE_MAX_M:
                data["stride_m"] = stride
                data["source"] = str(raw.get("source") or "file")
                data["measured_at"] = raw.get("measured_at")
                data["distance_m"] = raw.get("distance_m")
                data["steps"] = raw.get("steps")
            else:
                print(
                    f"[GAIT] stride.json holds {stride} m, outside the "
                    f"{GYRO_STRIDE_MIN_M}-{GYRO_STRIDE_MAX_M} m clamp — "
                    f"ignoring it and using the config seed"
                )
    except FileNotFoundError:
        data = {}  # nothing on disk: nobody to protect
    except Exception as exc:  # corrupt file must not brick boot
        print(
            f"[GAIT] stride file unreadable ({type(exc).__name__}: {exc}) — "
            f"using the config seed"
        )
    _state = data
    return _state


def get_stride() -> float:
    """The stride to multiply cadence by, in metres."""
    with _lock:
        return float(_load().get("stride_m") or GYRO_STRIDE_DEFAULT_M)


def is_measured() -> bool:
    """True when the figure came from this device rather than the config seed."""
    with _lock:
        return _load().get("stride_m") is not None


def _atomic_write_text(path: Path, text: str) -> None:
    """Write `text` to `path` so a crash or power cut leaves either the old file or the new one, never half of each: a temp file in the same directory, fsynced, then renamed over the original."""
    fd, tmp = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        try:
            f = os.fdopen(fd, "w", encoding="utf-8")
        except BaseException:
            os.close(fd)  # fdopen failed: the fd is still ours
            raise
        with f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        try:
            mode = os.stat(path).st_mode & 0o777
        except FileNotFoundError:
            mode = _NEW_FILE_MODE
        try:
            os.chmod(tmp, mode)
        except OSError:
            pass
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:  # never mask the original failure
            pass
        raise
    _fsync_dir(path.parent)


def _fsync_dir(directory: Path) -> None:
    """Make a rename (or unlink) in `directory` durable."""
    try:
        dfd = os.open(str(directory), os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    except OSError:
        return
    try:
        os.fsync(dfd)
    except OSError:
        pass
    finally:
        os.close(dfd)


def _payload(stride: float, source: str, **extra) -> Dict[str, Any]:
    data: Dict[str, Any] = {
        "stride_m": round(float(stride), 4),
        "source": source,
        "measured_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    data.update({k: v for k, v in extra.items() if v is not None})
    return data


def _adopt(data: Dict[str, Any]) -> None:
    """What is now on disk. Caller holds _lock."""
    globals()["_state"] = dict(
        data, **{_OWNER: "learned" if data["source"] == "learned" else "operator"}
    )


def _write(stride: float, source: str, **extra) -> Dict[str, Any]:
    """Caller holds _lock: written now, and newer than any learned write in flight."""
    global _last_save, _write_seq
    data = _payload(stride, source, **extra)
    _write_seq += 1
    with _file_lock:
        _atomic_write_text(
            STRIDE_FILE, json.dumps(data, indent=2, sort_keys=True) + "\n"
        )
    _adopt(data)
    _last_save = time.time()
    return dict(data)


def _save_learned(seq: int, stride: float) -> None:
    """note_learned's write, on its own thread: a write and two fsyncs on the LiDAR thread,
    under the lock every scan takes, could stall a scan on a busy card (scan 2026-10-23)."""
    data = _payload(stride, "learned")
    with _file_lock:
        if seq != _write_seq:
            return  # an operator's stride, a clear or a newer learned one came since
        try:
            _atomic_write_text(
                STRIDE_FILE, json.dumps(data, indent=2, sort_keys=True) + "\n"
            )
        except Exception as exc:
            print(
                f"[GAIT] could not persist learned stride ({type(exc).__name__}: {exc})"
            )
            return
    with _lock:
        if seq == _write_seq:
            _adopt(data)


def set_stride(
    stride_m: float,
    source: str = "measured",
    distance_m: Optional[float] = None,
    steps: Optional[int] = None,
) -> Dict[str, Any]:
    """Record a stride and persist it."""
    stride = float(stride_m)
    if not (GYRO_STRIDE_MIN_M <= stride <= GYRO_STRIDE_MAX_M):
        raise ValueError(
            f"stride {stride:.3f} m is outside {GYRO_STRIDE_MIN_M}-"
            f"{GYRO_STRIDE_MAX_M} m — suspect a miscounted footfall, not a "
            f"long pace"
        )
    global _operator_override
    with _lock:
        saved = _write(stride, source, distance_m=distance_m, steps=steps)
        _operator_override = True
        return saved


def note_learned(stride_m: float) -> None:
    """Offer a scan-match-learned stride for persistence."""
    global _last_save, _write_seq
    stride = float(stride_m)
    if not (GYRO_STRIDE_MIN_M <= stride <= GYRO_STRIDE_MAX_M):
        return
    now = time.time()
    with _lock:
        if _operator_override:
            return
        loaded = _load()
        if loaded.get(_OWNER) == "operator":
            return
        current = loaded.get("stride_m")
        if now - _last_save < _SAVE_MIN_INTERVAL_S:
            return
        if current is not None and abs(current - stride) < _SAVE_MIN_DELTA_M:
            return
        _last_save = now
        _write_seq += 1
        seq = _write_seq
    threading.Thread(
        target=_save_learned, args=(seq, stride), name="gait-save", daemon=True
    ).start()


def clear_stride() -> Dict[str, Any]:
    """Drop the overlay and fall back to the config seed."""
    global _state, _last_save, _operator_override, _write_seq
    with _lock:
        _write_seq += 1  # a learned write in flight must not bring the file back
        with _file_lock:
            try:
                STRIDE_FILE.unlink()
            except FileNotFoundError:
                pass
            else:
                _fsync_dir(STRIDE_FILE.parent)  # or a power cut can undo the clear
        _state = {}
        _last_save = 0.0
        _operator_override = True
        return {"stride_m": GYRO_STRIDE_DEFAULT_M, "source": "config seed"}


def stride_from_walk(distance_m: float, steps: int) -> float:
    """Stride implied by walking a measured distance. Raises on nonsense input."""
    if distance_m <= 0:
        raise ValueError("distance must be positive")
    if steps < 2:
        raise ValueError(f"{steps} steps is too few to imply a stride")
    return distance_m / float(steps)


def status() -> Dict[str, Any]:
    with _lock:
        data = _load()
        stride = data.get("stride_m")
        return {
            "stride_m": round(float(stride or GYRO_STRIDE_DEFAULT_M), 4),
            "measured": stride is not None,
            "source": data.get("source", "config seed"),
            "measured_at": data.get("measured_at"),
            "distance_m": data.get("distance_m"),
            "steps": data.get("steps"),
            "seed_m": GYRO_STRIDE_DEFAULT_M,
            "min_m": GYRO_STRIDE_MIN_M,
            "max_m": GYRO_STRIDE_MAX_M,
            "file": str(STRIDE_FILE),
            "file_exists": STRIDE_FILE.exists(),
        }
