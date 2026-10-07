"""Walkable-surface segmentation engine for the Hailo-10H."""

import os
import threading
import time
from contextlib import ExitStack

import numpy as np
import yaml
from PIL import Image

from config import HAILO_SEG_RUN_TIMEOUT_MS
from utils import hailo_device  # noqa: F401  (disables the hailort.log file)
from utils import hailo_stats
from utils.walkable_guidance import resolve_walkable_ids
from utils.yoloseg_decode import (
    decode_detections, build_class_map, BACKGROUND_ID, _PROTO_SIZE,
)

_METADATA_PATH = os.path.join(os.path.dirname(__file__), "..", "model", "metadata.yaml")


def load_metadata(path: str = _METADATA_PATH) -> dict:
    """Read class names + input size from the Ultralytics export metadata."""
    try:
        with open(path) as fh:
            meta = yaml.safe_load(fh) or {}
        names = {int(k): str(v) for k, v in (meta.get("names") or {}).items()}
        imgsz = meta.get("imgsz") or [640, 640]
        if not names:
            print(f"[Seg] WARNING: {path} lists no class names; "
                  "surface steering disabled (LiDAR steering unaffected)")
        return {"names": names, "imgsz": int(imgsz[0])}
    except Exception as e:
        print(f"[Seg] WARNING: metadata.yaml unreadable ({e}); "
              "surface steering disabled (LiDAR steering unaffected)")
        return {"names": {}, "imgsz": 640}


_NAMED_COLORS = {
    "path": (144, 238, 144),       # light green
    "stairs": (135, 206, 250),     # light blue
    "stair": (135, 206, 250),
    "staircase": (135, 206, 250),
    "steps": (135, 206, 250),
    "tactile": (255, 182, 193),    # light pink
    "tactiles": (255, 182, 193),
    "tactile_paving": (255, 182, 193),
    "road": (60, 180, 220),        # cyan
    "hallway": (250, 170, 30),     # orange
    "corridor": (250, 170, 30),
    "floor": (250, 170, 30),
    "sidewalk": (60, 180, 220),
    "pavement": (60, 180, 220),
    "person": (230, 40, 40),       # red — never walkable, must not look like it
}

_PALETTE_SIZE = 1 << 16


def _build_palette(names=None):
    """Deterministic mid-tone colour per class ID; background black."""
    idx = np.arange(_PALETTE_SIZE, dtype=np.uint64)
    h = (idx * np.uint64(2654435761)) & np.uint64(0xFFFFFFFF)
    pal = np.stack([(h >> np.uint64(16)) & 0xFF,
                    (h >> np.uint64(8)) & 0xFF,
                    h & 0xFF], axis=1).astype(np.uint8)
    pal = (pal // 2 + 80).astype(np.uint8)
    named_ids = set()
    for cid, name in (names or {}).items():
        colour = _NAMED_COLORS.get(str(name).strip().lower())
        if colour is not None and 0 <= int(cid) < _PALETTE_SIZE:
            pal[int(cid)] = colour
            named_ids.add(int(cid))
    pal[BACKGROUND_ID] = (0, 0, 0)
    return pal, named_ids


_PALETTE, _NAMED_IDS = _build_palette(load_metadata()["names"])


def colorize_class_map(class_map: np.ndarray, walkable_ids=None) -> np.ndarray:
    """Map a HxW class-ID array to an HxWx3 RGB image (background -> black)."""
    rgb = _PALETTE[class_map]
    if walkable_ids:
        bright = set(walkable_ids) | _NAMED_IDS
        keep = np.isin(class_map, list(bright))
        rgb = np.where(keep[..., None], rgb, rgb // 3)
    return rgb


class SegmentationEngine:
    """Persistent, thread-safe Hailo segmentation engine for the walkable surface."""

    _CLOSE_WAIT_S = 2.0
    _released = False
    _close_pending = False

    def __init__(self, hef_path: str, walkable_ids=None, vdevice=None,
                 conf_thres: float = 0.25, iou_thres: float = 0.7):
        self._hef_path = hef_path
        self._conf_thres = conf_thres
        self._iou_thres = iou_thres
        self._lock = threading.Lock()
        self._close_lock = threading.Lock()
        self._closed = False
        self._stack = ExitStack()

        try:
            self._load(hef_path, walkable_ids, vdevice)
        except BaseException:
            self._stack.close()
            raise

    def _load(self, hef_path, walkable_ids, vdevice):
        """The body of __init__: everything that can fail once resources exist."""
        from hailo_platform import FormatType

        meta = load_metadata()
        self.class_names = meta["names"]

        self._walkable_ids = (tuple(walkable_ids) if walkable_ids is not None
                              else resolve_walkable_ids(self.class_names))
        print(f"[Seg] {len(self.class_names)} classes; walkable = "
              + ", ".join(f"{i}:{self.class_names.get(i, '?')}"
                          for i in self._walkable_ids))

        if vdevice is None:
            from utils.hailo_device import get_shared_vdevice
            vdevice = get_shared_vdevice()

        infer_model = vdevice.create_infer_model(hef_path)
        infer_model.set_batch_size(1)

        self._input_name = infer_model.input_names[0]
        in_shape = list(infer_model.input(self._input_name).shape)   # (H, W, 3)
        self._in_h, self._in_w = int(in_shape[0]), int(in_shape[1])

        # Ask HailoRT for dequantized FLOAT32 so we never touch scale/zero-point.
        self._out_shapes = {}
        for name in infer_model.output_names:
            out = infer_model.output(name)
            out.set_format_type(FormatType.FLOAT32)
            self._out_shapes[name] = tuple(int(d) for d in out.shape)

        self._proto_name = None
        by_stride = {}   # grid size -> {"box"|"cls"|"coeff": layer name}
        for name, shape in self._out_shapes.items():
            h, ch = shape[0], shape[-1]
            if h == _PROTO_SIZE and ch == 32:
                self._proto_name = name
            elif ch in (4, 64):
                by_stride.setdefault(h, {})["box"] = name
            elif ch == 32:
                by_stride.setdefault(h, {})["coeff"] = name
            else:
                by_stride.setdefault(h, {})["cls"] = name

        if self._proto_name is None or not by_stride:
            raise RuntimeError(
                f"{hef_path}: unexpected output layout {self._out_shapes} — "
                "expected a 160x160x32 prototype tensor plus box (4- or "
                "64-channel), mask-coeff (32-channel) and N-class heads"
            )
        for grid, group in by_stride.items():
            missing = {"box", "cls", "coeff"} - set(group)
            if missing:
                raise RuntimeError(f"{hef_path}: scale {grid} is missing {missing} head(s)")

        # Largest grid first (stride 8 -> 16 -> 32).
        self._scales = [(g, by_stride[g], self._in_h // g) for g in sorted(by_stride, reverse=True)]

        self._model = self._stack.enter_context(infer_model.configure())
        # Name this engine reports under in utils.hailo_stats.
        self._stat_name = os.path.basename(hef_path)

    @property
    def walkable_ids(self):
        """Class IDs this engine treats as walkable ground (resolved from metadata)."""
        return self._walkable_ids

    # ── context-manager support ──────────────────────────────────────────────
    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    @property
    def released(self) -> bool:
        """True once close() has actually released the configured model — which can be after close() returned (see close())."""
        return self._released

    def _release_locked(self):
        """Release the configured model. Caller holds _lock. Idempotent."""
        self._closed = True
        self._close_pending = False
        try:
            self._stack.close()
        finally:
            self._released = True

    def close(self):
        if not self._lock.acquire(timeout=self._CLOSE_WAIT_S):
            with self._close_lock:
                if not self._lock.acquire(blocking=False):
                    self._closed = True
                    self._close_pending = True
                    print(f"[Seg] close: an inference is still running on "
                          f"{self._hef_path} after {self._CLOSE_WAIT_S:g}s — it "
                          f"releases the model when it returns")
                    return
        try:
            self._release_locked()
        finally:
            self._lock.release()

    # ── inference ────────────────────────────────────────────────────────────
    def infer_class_map(self, img: Image.Image):
        """Run segmentation on a PIL image."""
        if hailo_stats.DEBUG:
            print(f"[Hailo/{self._stat_name}] preprocess in  {img.size[0]}x{img.size[1]}",
                  flush=True)
        img = img.convert("RGB").resize((self._in_w, self._in_h))
        arr = np.ascontiguousarray(np.array(img, dtype=np.uint8))[np.newaxis, ...]

        out_bufs = {n: np.empty(s, dtype=np.float32) for n, s in self._out_shapes.items()}

        self._lock.acquire()
        try:
            if self._closed:
                raise RuntimeError("segmentation engine is closed")
            bindings = self._model.create_bindings()
            bindings.input(self._input_name).set_buffer(arr)
            for name, buf in out_bufs.items():
                bindings.output(name).set_buffer(buf)

            if hailo_stats.DEBUG:
                print(f"[Hailo/{self._stat_name}] run() start", flush=True)
            t0 = time.perf_counter()
            self._model.run([bindings], timeout=HAILO_SEG_RUN_TIMEOUT_MS)
            infer_ms = (time.perf_counter() - t0) * 1000
        finally:
            with self._close_lock:
                try:
                    if self._close_pending:
                        self._release_locked()
                finally:
                    self._lock.release()

        heads = [
            (out_bufs[g["box"]], out_bufs[g["cls"]], out_bufs[g["coeff"]], stride)
            for _grid, g, stride in self._scales
        ]
        det = decode_detections(heads, conf_thres=self._conf_thres,
                                iou_thres=self._iou_thres)
        class_map = build_class_map(det, out_bufs[self._proto_name],
                                    input_size=self._in_w)
        hailo_stats.record(self._stat_name, infer_ms, len(det))
        return class_map, infer_ms
