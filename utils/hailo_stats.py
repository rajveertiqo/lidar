"""In-process replacement for `hailortcli monitor`."""

import os
import threading
import time
from collections import deque

DEBUG = os.environ.get("HAILO_DEBUG") == "1"

_WINDOW_SEC = 5.0

_lock = threading.Lock()
_models = {}  # model name -> deque[(completed_at, infer_ms, n_results)]


def _bucket(model: str):
    d = _models.get(model)
    if d is None:
        d = _models[model] = deque()
    return d


def record(model: str, infer_ms: float, n_results: int = 0):
    """Report one completed inference. Called from inside the engines."""
    now = time.time()
    with _lock:
        d = _bucket(model)
        d.append((now, infer_ms, n_results))
        cutoff = now - _WINDOW_SEC
        while d and d[0][0] < cutoff:
            d.popleft()
        n = len(d)
        span = d[-1][0] - d[0][0] if n >= 2 else 0.0
        busy_ms = sum(e[1] for e in d)

    if DEBUG:
        fps = (n - 1) / span if span > 0 else 0.0
        util = (busy_ms / (span * 1000.0) * 100.0) if span > 0 else 0.0
        print(
            f"[Hailo/{model}] infer done {infer_ms:6.1f} ms | "
            f"{fps:5.1f} FPS | util {util:5.1f}% | results {n_results}",
            flush=True,
        )


def snapshot():
    """Current per-model FPS/utilization plus the NPU total."""
    now = time.time()
    cutoff = now - _WINDOW_SEC
    out = {}
    total_util = 0.0

    with _lock:
        for model, d in _models.items():
            entries = [e for e in d if e[0] >= cutoff]
            if len(entries) < 2:
                out[model] = {
                    "fps": 0.0,
                    "utilization": 0.0,
                    "avg_infer_ms": 0.0,
                    "inferences": len(entries),
                    "last_results": 0,
                }
                continue
            span = entries[-1][0] - entries[0][0]
            busy_ms = sum(e[1] for e in entries)
            fps = (len(entries) - 1) / span if span > 0 else 0.0
            util = (busy_ms / (span * 1000.0) * 100.0) if span > 0 else 0.0
            total_util += util
            out[model] = {
                "fps": round(fps, 1),
                "utilization": round(util, 1),
                "avg_infer_ms": round(busy_ms / len(entries), 1),
                "inferences": len(entries),
                "last_results": entries[-1][2],
            }

    return {
        "models": out,
        "total_utilization": round(total_util, 1),
        "window_sec": _WINDOW_SEC,
    }
