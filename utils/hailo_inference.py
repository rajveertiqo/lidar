import math
import os
import threading
import time
from contextlib import ExitStack

import numpy as np
import yaml

from utils import hailo_device  # noqa: F401  (disables the hailort.log file)
from utils import hailo_stats
from utils.walkable_guidance import WALKABLE_NAMES
from utils.yoloseg_decode import cls_is_probability

# ── Constants ──────────────────────────────────────────────────────────────────
IMG_SIZE = 640

from config import DETECTION_CLASSES, HAILO_DETECT_RUN_TIMEOUT_MS  # noqa: E402

_N_PROTO = 32  # mask prototypes / channels in a coeff head
_PROTO_FEAT = IMG_SIZE // 4  # 160 — prototype spatial resolution

_METADATA_PATH = os.path.join(os.path.dirname(__file__), "..", "model", "metadata.yaml")

_SURFACE_NAMES = frozenset(
    WALKABLE_NAMES
    | {
        "tactile",
        "tactile paving",
        "tactile_paving",
        "tactile path",
    }
)

_REPORTED_NAMES = frozenset(str(c).strip().lower() for c in DETECTION_CLASSES)


def _unreported(names) -> frozenset:
    """Class ids never reported as objects: ground surfaces, and every class outside config.DETECTION_CLASSES (chairs, tables, laptops ...)."""
    low = [str(n).strip().lower() for n in names]
    return frozenset(
        i for i, n in enumerate(low) if n in _SURFACE_NAMES or n not in _REPORTED_NAMES
    )


def _metadata_names(num_classes: int):
    """Class names from the export's own model/metadata.yaml, or None."""
    try:
        with open(_METADATA_PATH) as fh:
            names = (yaml.safe_load(fh) or {}).get("names") or {}
        ordered = [str(names[k]) for k in sorted(names, key=int)]
        return ordered if len(ordered) == num_classes else None
    except Exception:
        return None


COCO_NAMES = [
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush",
]

# Merged custom dataset used to train yolo26n (nc=129) — order matches data.yaml.
CUSTOM_129_NAMES = [
    "person",
    "car",
    "bus",
    "motorcycle",
    "bicycle",
    "traffic_light",
    "traffic_sign",
    "hazard_sign",
    "crosswalk",
    "stairs",
    "pothole",
    "tree",
    "branch",
    "stone",
    "bench",
    "chair",
    "table",
    "sofa",
    "dog",
    "cat",
    "animal",
    "advertising_column",
    "bus_stop",
    "delimitation_stake",
    "double_pole",
    "flower_bucket",
    "pole_special",
    "simple_pole",
    "street_pole",
    "traffic_arrow",
    "alligator_cracking",
    "lateral_cracking",
    "longitudinal_cracking",
    "brick",
    "line",
    "waste_container",
    "0",
    "1",
    "2",
    "cyclist",
    "persona",
    "pessoa",
    "signboard",
    "stopper",
    "aeroplane",
    "bag",
    "berdiri",
    "bird",
    "boat",
    "bottle",
    "cow",
    "dianzhuan",
    "diningtable",
    "face",
    "forklift",
    "handbag",
    "head",
    "helmet",
    "high",
    "horse",
    "jatuh",
    "laptop",
    "low",
    "medium",
    "pottedplant",
    "refrigerator",
    "sheep",
    "teddy_bear",
    "train",
    "tv",
    "tvmonitor",
    "vase",
    "vehicle",
    "fire_hydrant",
    "d00",
    "d00rotation",
    "d10",
    "d10rotation",
    "d20",
    "d20rotation",
    "d40",
    "d40rotation",
    "address_plaque",
    "license_plate",
    "street_advertisement",
    "street_sign",
    "window",
    "windshield",
    "g",
    "ga",
    "gf",
    "n",
    "nf",
    "r",
    "ra",
    "rf",
    "y",
    "ya",
    "yf",
    "sdsd",
    "big_bu",
    "big_truck",
    "bus_l",
    "bus_",
    "mid_truck",
    "small_bu",
    "small_truck",
    "truck_l",
    "truck_m",
    "truck_",
    "truck_xl",
    "bufflow",
    "elephant",
    "z100",
    "z30",
    "z40",
    "z50",
    "z60",
    "z70",
    "z80",
    "z90",
    "danger",
    "mandatory",
    "other",
    "prohibitory",
    "",
    "pickup",
    "sedan",
    "suv",
]


# ── Pre / post processing helpers ─────────────────────────────────────────────


def _letterbox_array(arr: np.ndarray, bgr: bool = False):
    """Letterbox a numpy frame into 640×640 via OpenCV (preserve aspect ratio, pad with grey) instead of stretching it — a 960×540 sector squished to 640×640 distorts objects and hurts detection."""
    import cv2

    oh, ow = arr.shape[:2]
    scale = min(IMG_SIZE / ow, IMG_SIZE / oh)
    nw, nh = max(1, int(round(ow * scale))), max(1, int(round(oh * scale)))
    pad_x = (IMG_SIZE - nw) // 2
    pad_y = (IMG_SIZE - nh) // 2
    small = cv2.resize(arr, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas = np.full((IMG_SIZE, IMG_SIZE, 3), 114, dtype=np.uint8)
    canvas[pad_y : pad_y + nh, pad_x : pad_x + nw] = small[:, :, ::-1] if bgr else small
    return canvas[np.newaxis, ...], scale, float(pad_x), float(pad_y)


def _make_anchors(feat: int):
    """Grid-cell centres for an feat×feat feature map, flattened h-major (w inner)."""
    coords = np.arange(feat, dtype=np.float32) + 0.5
    cx = np.tile(coords, feat)  # x varies fastest (w inner)
    cy = np.repeat(coords, feat)  # y repeated per row (h outer)
    return cx, cy


def _dfl_decode(box_raw: np.ndarray, reg_max: int) -> np.ndarray:
    """Collapse raw DFL bin logits to ltrb distances."""
    n = box_raw.shape[0]
    x = box_raw.reshape(n, 4, reg_max)
    x = x - x.max(axis=2, keepdims=True)
    probs = np.exp(x)
    probs /= probs.sum(axis=2, keepdims=True)
    bins = np.arange(reg_max, dtype=np.float32)
    return probs @ bins


def _nms(boxes: np.ndarray, scores: np.ndarray, iou_thr: float):
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
        order = order[1:][iou <= iou_thr]
    return keep


# ── Persistent inference engine ────────────────────────────────────────────────


class HailoInferenceEngine:
    """Thread-safe, persistent Hailo-10H inference engine."""

    def __init__(
        self,
        hef_path: str,
        conf_thresh: float = 0.25,
        iou_thresh: float = 0.45,
        max_candidates: int = 300,
        vdevice=None,
    ):
        self._hef_path = hef_path
        self._conf = conf_thresh
        self._iou = iou_thresh
        self._max_candidates = max_candidates
        self._lock = threading.Lock()
        self._stack = ExitStack()
        self._logit_thresh = (
            math.log(conf_thresh / (1.0 - conf_thresh)) if 0 < conf_thresh < 1 else -1e9
        )

        try:
            self._load(hef_path, vdevice)
        except BaseException:
            self._stack.close()
            raise

    def _load(self, hef_path, vdevice):
        """The body of __init__: everything that can fail once resources exist."""
        from hailo_platform import FormatType

        print(f"[Hailo] Loading HEF: {hef_path}")

        if vdevice is None:
            from utils.hailo_device import get_shared_vdevice

            vdevice = get_shared_vdevice()
        infer_model = vdevice.create_infer_model(hef_path)
        infer_model.set_batch_size(1)

        self._input_name = infer_model.input_names[0]

        self._out_shapes = {}
        for name in infer_model.output_names:
            out = infer_model.output(name)
            out.set_format_type(FormatType.FLOAT32)
            self._out_shapes[name] = tuple(int(d) for d in out.shape)  # (F, F, C)

        is_seg = any(
            sh[0] == _PROTO_FEAT and sh[-1] == _N_PROTO
            for sh in self._out_shapes.values()
        )

        box_by_feat = {}
        cls_by_feat = {}
        self._num_classes = 0
        for name, shape in self._out_shapes.items():
            feat, ch = shape[0], shape[-1]
            if ch == 4:
                box_by_feat[feat] = (name, int(np.prod(shape)), 1)
            elif ch == 64:
                box_by_feat[feat] = (name, int(np.prod(shape)), 16)
            elif is_seg and ch == _N_PROTO:
                continue  # mask coeffs / prototypes: not ours
            else:
                cls_by_feat[feat] = (name, int(np.prod(shape)))
                self._num_classes = max(self._num_classes, ch)

        # Build per-stride level metadata (sorted small-stride first).
        self._levels = []
        for feat in sorted(box_by_feat.keys(), reverse=True):  # 80 -> 40 -> 20
            if feat not in cls_by_feat:
                continue
            stride = IMG_SIZE // feat
            cx, cy = _make_anchors(feat)
            self._levels.append(
                {
                    "feat": feat,
                    "stride": float(stride),
                    "box_name": box_by_feat[feat][0],
                    "box_size": box_by_feat[feat][1],
                    "reg_max": box_by_feat[feat][2],
                    "cls_name": cls_by_feat[feat][0],
                    "cls_size": cls_by_feat[feat][1],
                    "cx": cx,
                    "cy": cy,
                }
            )

        if not self._levels:
            raise RuntimeError(
                f"Could not find box(4-ch or 64-ch DFL)+class head pairs in {hef_path}; "
                f"outputs were {list(infer_model.output_names)}"
            )

        if self._num_classes == len(COCO_NAMES):
            self._names = COCO_NAMES
        elif self._num_classes == len(CUSTOM_129_NAMES):
            self._names = CUSTOM_129_NAMES
        else:
            self._names = _metadata_names(self._num_classes) or [
                f"class_{i}" for i in range(self._num_classes)
            ]

        self._suppressed = _unreported(self._names)
        self._reported = np.array(
            [i not in self._suppressed for i in range(self._num_classes)], dtype=bool
        )

        self._cls_is_prob = None

        decoded = {lvl["box_name"] for lvl in self._levels} | {
            lvl["cls_name"] for lvl in self._levels
        }
        self._scratch = {
            n: np.empty(int(np.prod(sh)), dtype=np.float32)
            for n, sh in self._out_shapes.items()
            if n not in decoded
        }

        self._model = self._stack.enter_context(infer_model.configure())
        self._stat_name = os.path.basename(hef_path)
        print(
            f"[Hailo] Engine ready: {len(self._levels)} scales, "
            f"{self._num_classes} classes, input '{self._input_name}'."
        )
        if self._scratch:
            print(
                f"[Hailo] Segmentation hef: {len(self._scratch)} mask/prototype "
                f"output(s) bound but unused ({', '.join(sorted(self._scratch))})"
            )
        surfaces = sorted(
            n for n in self._names if str(n).strip().lower() in _SURFACE_NAMES
        )
        if surfaces:
            print(
                "[Hailo] Not reported as objects (ground surfaces): "
                + ", ".join(surfaces)
            )
        reported = [n for i, n in enumerate(self._names) if self._reported[i]]
        print(
            f"[Hailo] Reporting {len(reported)} of {self._num_classes} classes "
            f"(config.DETECTION_CLASSES): {', '.join(reported) or 'NONE'}"
        )

    # ── context-manager support ────────────────────────────────────────────────

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def close(self):
        self._stack.close()

    # ── inference ─────────────────────────────────────────────────────────────

    def infer_array(self, arr: np.ndarray, bgr: bool = False) -> tuple:
        """Run inference on a numpy frame (H, W, 3). Returns (detections, inference_ms)."""
        orig_h, orig_w = arr.shape[:2]
        input_data, scale, pad_x, pad_y = _letterbox_array(arr, bgr)
        return self._infer(input_data, orig_w, orig_h, scale, pad_x, pad_y)

    def _infer(self, input_data, orig_w, orig_h, scale, pad_x, pad_y) -> tuple:
        """Run the network on a letterboxed input and decode its boxes back into the original image's pixels."""
        buffers = {
            lvl["box_name"]: np.empty(lvl["box_size"], dtype=np.float32)
            for lvl in self._levels
        }
        buffers.update(
            {
                lvl["cls_name"]: np.empty(lvl["cls_size"], dtype=np.float32)
                for lvl in self._levels
            }
        )

        with self._lock:
            bindings = self._model.create_bindings()
            bindings.input(self._input_name).set_buffer(input_data)
            for name, buf in buffers.items():
                bindings.output(name).set_buffer(buf)
            for name, buf in self._scratch.items():
                bindings.output(name).set_buffer(buf)

            if hailo_stats.DEBUG:
                print(f"[Hailo/{self._stat_name}] run() start", flush=True)
            t0 = time.perf_counter()
            self._model.run([bindings], timeout=HAILO_DETECT_RUN_TIMEOUT_MS)
            inference_ms = (time.perf_counter() - t0) * 1000

        detections = self._postprocess(buffers, orig_w, orig_h, scale, pad_x, pad_y)
        hailo_stats.record(self._stat_name, inference_ms, len(detections))
        return detections, inference_ms

    def _postprocess(self, buffers, orig_w, orig_h, scale, pad_x, pad_y):
        nc = self._num_classes
        cand_boxes = []
        cand_logits = []

        for lvl in self._levels:
            n = lvl["feat"] * lvl["feat"]
            box = buffers[lvl["box_name"]].reshape(n, 4 * lvl["reg_max"])
            cls = buffers[lvl["cls_name"]].reshape(n, nc)

            if self._cls_is_prob is None:
                self._cls_is_prob = cls_is_probability(cls)
                print(
                    f"[Hailo] Class head reads as "
                    f"{'probabilities (sigmoid baked on-chip)' if self._cls_is_prob else 'logits'}"
                    f"; scoring accordingly."
                )

            best = cls.max(axis=1)
            keep = best >= (self._conf if self._cls_is_prob else self._logit_thresh)
            if not keep.any():
                continue

            box_k = box[keep]
            if lvl["reg_max"] > 1:
                box_k = _dfl_decode(box_k, lvl["reg_max"])
            cx = lvl["cx"][keep]
            cy = lvl["cy"][keep]
            s = lvl["stride"]
            x1 = (cx - box_k[:, 0]) * s
            y1 = (cy - box_k[:, 1]) * s
            x2 = (cx + box_k[:, 2]) * s
            y2 = (cy + box_k[:, 3]) * s
            cand_boxes.append(np.stack([x1, y1, x2, y2], axis=1))
            cand_logits.append(cls[keep])

        if not cand_boxes:
            return []

        boxes = np.concatenate(cand_boxes, axis=0)  # (M, 4) in 640 letterbox space
        scores = np.concatenate(cand_logits, axis=0)  # (M, nc)
        if not self._cls_is_prob:
            scores = 1.0 / (1.0 + np.exp(-scores))

        boxes[:, [0, 2]] = np.clip((boxes[:, [0, 2]] - pad_x) / scale, 0, orig_w)
        boxes[:, [1, 3]] = np.clip((boxes[:, [1, 3]] - pad_y) / scale, 0, orig_h)

        row_idx, cls_idx = np.where(scores >= self._conf)

        if self._suppressed and row_idx.size:
            wanted = self._reported[cls_idx]
            row_idx, cls_idx = row_idx[wanted], cls_idx[wanted]

        if row_idx.size == 0:
            return []

        sel_boxes = boxes[row_idx]
        sel_scores = scores[row_idx, cls_idx]

        if sel_scores.size > self._max_candidates:
            top = np.argpartition(sel_scores, -self._max_candidates)[
                -self._max_candidates :
            ]
            row_idx, cls_idx = row_idx[top], cls_idx[top]
            sel_boxes, sel_scores = sel_boxes[top], sel_scores[top]

        detections = []
        for cls_id in np.unique(cls_idx):
            mask = cls_idx == cls_id
            b = sel_boxes[mask]
            s = sel_scores[mask]
            for k in _nms(b, s, self._iou):
                detections.append(
                    {
                        "class_name": self._names[cls_id],
                        "confidence": float(s[k]),
                        "bbox": b[k].tolist(),
                    }
                )

        detections.sort(key=lambda d: -d["confidence"])
        return detections
