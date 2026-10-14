"""Walk recorder — every guidance tick's inputs and what was said, for replay at the desk.

A recording is gzipped JSON lines in WALK_RECORD_DIR, one line per guidance tick:
    {"t": wall clock, "mono": monotonic clock, "in": {walk_decision, lidar, radar, camera,
     motion, movers, hold, hold_lidar_absent, at_crossing}, "out": {instruction, speak, ...}}
plus {"mark": ...} lines from a tester (/guidance/mark). tools/walk_replay.py feeds the
inputs back through the CURRENT simple_guidance (and, by default, walk_policy) and lists
every sentence that would now be said differently — so a change is tried on real walks
before a blind user hears it.

Off unless SHAPPE_RECORD_WALK=1 or POST /guidance/record. Encoding happens on the
guidance thread (a snapshot, not a reference another thread may change); compression
and the disk on a writer thread, so a slow card never holds up the guidance loop.
"""

import gzip
import json
import os
import queue
import threading
import time
from typing import Any, Dict, Optional

from config import WALK_RECORD, WALK_RECORD_DIR, WALK_RECORD_MAX_MB

_FLUSH_S = 2.0


def _plain(obj: Any) -> Any:
    """json.dumps fallback: numpy values and arrays, sets, anything else as text."""
    tolist = getattr(obj, "tolist", None)
    if callable(tolist):
        return tolist()
    if isinstance(obj, (set, frozenset)):
        return sorted(obj, key=str)
    return str(obj)


class WalkRecorder:
    def __init__(self):
        self._lock = threading.Lock()
        self._queue: "queue.Queue[Optional[str]]" = queue.Queue(maxsize=200)
        self._thread: Optional[threading.Thread] = None
        self._path: Optional[str] = None
        self._started: Optional[float] = None
        self._ticks = 0
        self._dropped = 0
        self._bytes = 0
        self._error = ""

    # ---- control ---------------------------------------------------------

    def start(self, label: str = "") -> Dict[str, Any]:
        with self._lock:
            if self._path is not None:
                return self._status()
            os.makedirs(WALK_RECORD_DIR, exist_ok=True)
            safe = "".join(c if c.isalnum() or c in "-_" else "-" for c in label.strip())[:40]
            name = time.strftime("walk-%Y%m%d-%H%M%S") + (f"-{safe}" if safe else "")
            self._path = os.path.join(WALK_RECORD_DIR, name + ".jsonl.gz")
            self._started = time.time()
            self._ticks = self._dropped = self._bytes = 0
            self._error = ""
            self._queue = queue.Queue(maxsize=200)
            self._thread = threading.Thread(target=self._write, args=(self._path, self._queue),
                                            name="walk-recorder", daemon=True)
            self._thread.start()
            print(f"[REC] recording the walk to {self._path}")
            return self._status()

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            if self._path is None:
                return self._status()
            q, thread, path = self._queue, self._thread, self._path
            self._path = None
        try:
            q.put(None, timeout=2.0)
        except queue.Full:
            pass
        if thread is not None:
            thread.join(timeout=5.0)
        print(f"[REC] stopped: {path} ({self._ticks} ticks, {self._dropped} dropped)")
        with self._lock:
            st = self._status()
            st["last_file"] = path
            return st

    @property
    def active(self) -> bool:
        return self._path is not None

    def status(self) -> Dict[str, Any]:
        with self._lock:
            return self._status()

    def _status(self) -> Dict[str, Any]:
        return {"recording": self._path is not None, "file": self._path,
                "started": self._started, "ticks": self._ticks, "dropped": self._dropped,
                "megabytes": round(self._bytes / 1e6, 2), "error": self._error}

    # ---- feed (guidance thread) -----------------------------------------

    def tick(self, now: float, mono: float, inputs: Dict[str, Any],
             output: Dict[str, Any]) -> None:
        """One guidance tick. Never raises — a recording must never stop guidance."""
        if self._path is None:
            return
        try:
            line = json.dumps({"t": now, "mono": mono, "in": inputs, "out": output},
                              default=_plain, separators=(",", ":"))
        except Exception as e:
            self._error = f"encode: {type(e).__name__}: {e}"
            return
        self._put(line, counts=True)

    def mark(self, kind: str, text: str = "") -> None:
        if self._path is None:
            return
        self._put(json.dumps({"t": time.time(), "mono": time.monotonic(),
                              "mark": kind, "text": text}), counts=False)

    def _put(self, line: str, counts: bool) -> None:
        try:
            self._queue.put_nowait(line)
            if counts:
                self._ticks += 1
        except queue.Full:
            self._dropped += 1

    # ---- writer thread ---------------------------------------------------

    def _write(self, path: str, q: "queue.Queue[Optional[str]]") -> None:
        limit = WALK_RECORD_MAX_MB * 1e6
        last_flush = time.monotonic()
        try:
            with gzip.open(path, "wt", encoding="utf-8", compresslevel=5) as fh:
                while True:
                    line = q.get()
                    if line is None:
                        break
                    fh.write(line + "\n")
                    self._bytes += len(line) + 1
                    if time.monotonic() - last_flush >= _FLUSH_S:
                        fh.flush()
                        last_flush = time.monotonic()
                    if self._bytes >= limit:
                        self._error = f"stopped at WALK_RECORD_MAX_MB ({WALK_RECORD_MAX_MB} MB)"
                        print(f"[REC] {self._error}")
                        with self._lock:
                            if self._path == path:
                                self._path = None
                        break
        except Exception as e:
            self._error = f"write: {type(e).__name__}: {e}"
            print(f"[REC] recording failed — {self._error}")
            with self._lock:
                if self._path == path:
                    self._path = None


_recorder = WalkRecorder()
if WALK_RECORD:
    _recorder.start("auto")


def get_recorder() -> WalkRecorder:
    return _recorder
