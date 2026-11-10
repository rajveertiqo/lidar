"""Walk recorder — every guidance tick's inputs and what was said, for replay at the desk.

A recording is gzipped JSON lines in WALK_RECORD_DIR, one line per guidance tick:
    {"t": wall clock, "mono": monotonic clock, "in": {walk_decision, lidar, radar, camera,
     motion, movers, ground, hold, hold_lidar_absent, at_crossing}, "out": {instruction,
     speak, ...}}   (ground: the ToF's hazards on the ground, since 2026-11-06)
plus {"mark": ...} lines from a tester (/guidance/mark). tools/walk_replay.py feeds the
inputs back through the CURRENT simple_guidance (and, by default, walk_policy) and lists
every sentence that would now be said differently — so a change is tried on real walks
before a blind user hears it.

Off unless SHAPPE_RECORD_WALK=1 or POST /guidance/record. Encoding happens on the
guidance thread (a snapshot, not a reference another thread may change); compression
and the disk on a writer thread, so a slow card never holds up the guidance loop.

Recording or not, the last few seconds of ticks are kept in memory (WALK_CLIP_ON_MARK): a
tester's "Wrong" or "Missed" saves them as a clip (clip-*.jsonl.gz, the same format), from
WALK_CLIP_BEFORE_S before the mark to WALK_CLIP_AFTER_S after (2026-11-10). A clip starts
mid-walk, so its first seconds replay without the device's held state.
"""

import gzip
import json
import os
import queue
import threading
import time
from collections import deque
from typing import Any, Deque, Dict, Optional, Tuple

from config import (
    WALK_CLIP_AFTER_S,
    WALK_CLIP_BEFORE_S,
    WALK_CLIP_KEEP_FILES,
    WALK_CLIP_ON_MARK,
    WALK_RECORD,
    WALK_RECORD_DIR,
    WALK_RECORD_KEEP_FILES,
    WALK_RECORD_KEEP_MB,
    WALK_RECORD_MAX_MB,
)

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
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._path: Optional[str] = None
        self._started: Optional[float] = None
        self._ticks = 0
        self._dropped = 0
        self._bytes = 0
        self._error = ""
        # (monotonic, encoded tick) for the last WALK_CLIP_BEFORE_S + WALK_CLIP_AFTER_S + 1 s.
        self._recent: Deque[Tuple[float, str]] = deque(maxlen=2000)
        self._recent_lock = threading.Lock()
        self._clips = 0

    # ---- control ---------------------------------------------------------

    def start(self, label: str = "") -> Dict[str, Any]:
        with self._lock:
            if self._path is not None:
                return self._status()
            os.makedirs(WALK_RECORD_DIR, exist_ok=True)
            _prune()
            safe = "".join(
                c if c.isalnum() or c in "-_" else "-" for c in label.strip()
            )[:40]
            name = time.strftime("walk-%Y%m%d-%H%M%S") + (f"-{safe}" if safe else "")
            self._path = os.path.join(WALK_RECORD_DIR, name + ".jsonl.gz")
            self._started = time.time()
            self._ticks = self._dropped = self._bytes = 0
            self._error = ""
            self._queue = queue.Queue(maxsize=200)
            self._stop = threading.Event()
            self._thread = threading.Thread(
                target=self._write,
                args=(self._path, self._queue, self._stop),
                name="walk-recorder",
                daemon=True,
            )
            self._thread.start()
            print(f"[REC] recording the walk to {self._path}")
            return self._status()

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            if self._path is None:
                return self._status()
            thread, path = self._thread, self._path
            self._path = None
            # An event, not a marker in the queue: a full queue dropped the marker, and the
            # writer then never closed the file (scan 2026-10-23).
            self._stop.set()
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

    @property
    def wants_ticks(self) -> bool:
        """Recording, or keeping the last seconds for a tester's mark (WALK_CLIP_ON_MARK)."""
        return self._path is not None or WALK_CLIP_ON_MARK

    def status(self) -> Dict[str, Any]:
        with self._lock:
            return self._status()

    def _status(self) -> Dict[str, Any]:
        return {
            "recording": self._path is not None,
            "file": self._path,
            "started": self._started,
            "ticks": self._ticks,
            "dropped": self._dropped,
            "megabytes": round(self._bytes / 1e6, 2),
            "error": self._error,
            "clips_on_mark": bool(WALK_CLIP_ON_MARK),
            "clips_saved": self._clips,
        }

    # ---- feed (guidance thread) -----------------------------------------

    def tick(
        self, now: float, mono: float, inputs: Dict[str, Any], output: Dict[str, Any]
    ) -> None:
        """One guidance tick. Never raises — a recording must never stop guidance."""
        if not self.wants_ticks:
            return
        try:
            line = json.dumps(
                {"t": now, "mono": mono, "in": inputs, "out": output},
                default=_plain,
                separators=(",", ":"),
            )
        except Exception as e:
            self._error = f"encode: {type(e).__name__}: {e}"
            return
        if WALK_CLIP_ON_MARK:
            keep = WALK_CLIP_BEFORE_S + WALK_CLIP_AFTER_S + 1.0
            with self._recent_lock:
                if self._recent and mono < self._recent[-1][0]:
                    self._recent.clear()  # a new monotonic clock: a restart in tests
                self._recent.append((mono, line))
                while self._recent and mono - self._recent[0][0] > keep:
                    self._recent.popleft()
        if self._path is not None:
            self._put(line, counts=True)

    def clip(self, kind: str, text: str = "") -> Optional[str]:
        """Save the ticks around a tester's mark as WALK_RECORD_DIR/clip-*.jsonl.gz, written
        WALK_CLIP_AFTER_S from now on its own thread; returns that path, or None when off."""
        if not WALK_CLIP_ON_MARK:
            return None
        t, mono = time.time(), time.monotonic()
        safe = "".join(c if c.isalnum() else "-" for c in str(kind))[:12] or "mark"
        try:
            os.makedirs(WALK_RECORD_DIR, exist_ok=True)
        except OSError as e:
            self._error = f"clip: {e}"
            return None
        path = os.path.join(
            WALK_RECORD_DIR,
            time.strftime("clip-%Y%m%d-%H%M%S", time.localtime(t)) + f"-{safe}.jsonl.gz",
        )
        mark = json.dumps({"t": t, "mono": mono, "mark": kind, "text": text})

        def write() -> None:
            time.sleep(WALK_CLIP_AFTER_S)  # what happened just after the mark too
            with self._recent_lock:
                ticks = [
                    (m, line)
                    for m, line in self._recent
                    if mono - WALK_CLIP_BEFORE_S <= m <= mono + WALK_CLIP_AFTER_S
                ]
            part = path + ".part"
            try:
                with gzip.open(part, "wt", encoding="utf-8", compresslevel=5) as fh:
                    marked = False  # the mark goes where it fell among the ticks
                    for m, line in ticks:
                        if not marked and m > mono:
                            fh.write(mark + "\n")
                            marked = True
                        fh.write(line + "\n")
                    if not marked:
                        fh.write(mark + "\n")
                os.replace(part, path)
                self._clips += 1
                _prune_clips()
                print(f"[REC] {kind} mark: {len(ticks)} ticks saved to {path}")
            except Exception as e:
                self._error = f"clip: {type(e).__name__}: {e}"
                print(f"[REC] could not save the {kind} clip — {self._error}")
                try:
                    os.remove(part)
                except OSError:
                    pass

        threading.Thread(target=write, name="walk-clip", daemon=True).start()
        return path

    def mark(self, kind: str, text: str = "") -> None:
        if self._path is None:
            return
        self._put(
            json.dumps(
                {"t": time.time(), "mono": time.monotonic(), "mark": kind, "text": text}
            ),
            counts=False,
        )

    def _put(self, line: str, counts: bool) -> None:
        try:
            self._queue.put_nowait(line)
            if counts:
                self._ticks += 1
        except queue.Full:
            self._dropped += 1

    # ---- writer thread ---------------------------------------------------

    def _write(
        self, path: str, q: "queue.Queue[Optional[str]]", stop: threading.Event
    ) -> None:
        limit = WALK_RECORD_MAX_MB * 1e6
        last_flush = time.monotonic()
        try:
            with gzip.open(path, "wt", encoding="utf-8", compresslevel=5) as fh:
                while True:
                    try:
                        line = q.get(timeout=0.5)
                    except queue.Empty:
                        if stop.is_set():
                            break  # stopped, and everything queued is written
                        if time.monotonic() - last_flush >= _FLUSH_S:
                            fh.flush()  # idle: what is written reaches the card
                            last_flush = time.monotonic()
                        continue
                    if line is None:
                        break
                    fh.write(line + "\n")
                    self._bytes += len(line) + 1
                    if time.monotonic() - last_flush >= _FLUSH_S:
                        fh.flush()
                        last_flush = time.monotonic()
                    if self._bytes >= limit:
                        self._error = (
                            f"stopped at WALK_RECORD_MAX_MB ({WALK_RECORD_MAX_MB} MB)"
                        )
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


def _prune() -> None:
    """Keep the newest WALK_RECORD_KEEP_FILES recordings, WALK_RECORD_KEEP_MB in all: they
    were never deleted. Called before a new one starts, so never the one being written."""
    try:
        files = sorted(
            (
                os.path.join(WALK_RECORD_DIR, n)
                for n in os.listdir(WALK_RECORD_DIR)
                if n.startswith("walk-") and n.endswith(".jsonl.gz")
            ),
            key=os.path.getmtime,
            reverse=True,
        )
        total = 0.0
        for i, f in enumerate(files):
            total += os.path.getsize(f) / 1e6
            if i >= WALK_RECORD_KEEP_FILES or total > WALK_RECORD_KEEP_MB:
                os.remove(f)
                print(f"[REC] removed an old recording: {os.path.basename(f)}")
    except OSError as e:
        print(f"[REC] could not tidy {WALK_RECORD_DIR}: {e}")


def _prune_clips() -> None:
    """Keep the newest WALK_CLIP_KEEP_FILES clips."""
    try:
        clips = sorted(
            (
                os.path.join(WALK_RECORD_DIR, n)
                for n in os.listdir(WALK_RECORD_DIR)
                if n.startswith("clip-") and n.endswith(".jsonl.gz")
            ),
            key=os.path.getmtime,
            reverse=True,
        )
        for f in clips[WALK_CLIP_KEEP_FILES:]:
            os.remove(f)
    except OSError as e:
        print(f"[REC] could not tidy the clips in {WALK_RECORD_DIR}: {e}")


_recorder = WalkRecorder()
if WALK_RECORD:
    _recorder.start("auto")


def get_recorder() -> WalkRecorder:
    return _recorder
