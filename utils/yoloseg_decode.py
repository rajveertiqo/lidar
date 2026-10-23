"""Pure-NumPy decoder for the Ultralytics YOLO-seg HEF (model/best.hef)."""

from typing import Dict, List, Tuple

import numpy as np

BACKGROUND_ID = 65535
CLASS_MAP_DTYPE = np.uint16

_N_PROTO = 32  # mask prototypes
_PROTO_SIZE = 160  # prototype spatial resolution (640 / 4)


def _softmax(x: np.ndarray, axis: int = -1) -> np.ndarray:
    x = x - x.max(axis=axis, keepdims=True)
    e = np.exp(x)
    return e / e.sum(axis=axis, keepdims=True)


def _sigmoid(x: np.ndarray) -> np.ndarray:
    # Branch on sign so exp() never sees a large positive argument and overflows.
    out = np.empty_like(x, dtype=np.float32)
    pos = x >= 0
    out[pos] = 1.0 / (1.0 + np.exp(-x[pos]))
    ex = np.exp(x[~pos])
    out[~pos] = ex / (1.0 + ex)
    return out


def cls_is_probability(cls_head: np.ndarray, tol: float = 0.02) -> bool:
    """True if this class head already had its sigmoid applied on-chip."""
    return float(cls_head.min()) >= -tol and float(cls_head.max()) <= 1.0 + tol


def _decode_box(box_head: np.ndarray, stride: int) -> np.ndarray:
    """(S,S,C) box head -> (S*S,4) xyxy boxes in 640-space input pixels."""
    s, ch = box_head.shape[0], box_head.shape[-1]
    if ch == 4:
        ltrb = box_head.reshape(-1, 4).astype(np.float32)  # (S*S, 4)
    elif ch % 4 == 0:
        reg_max = ch // 4
        dist = _softmax(box_head.reshape(-1, 4, reg_max).astype(np.float32), axis=-1)
        ltrb = (dist * np.arange(reg_max, dtype=np.float32)).sum(-1)  # (S*S, 4)
    else:
        raise ValueError(
            f"box head has {ch} channels; expected 4 (raw ltrb) or 4*reg_max (DFL)"
        )

    # Anchor points are grid-cell CENTRES, in grid units.
    gy, gx = np.mgrid[0:s, 0:s]
    ax = (gx.reshape(-1) + 0.5).astype(np.float32)
    ay = (gy.reshape(-1) + 0.5).astype(np.float32)

    x1 = (ax - ltrb[:, 0]) * stride
    y1 = (ay - ltrb[:, 1]) * stride
    x2 = (ax + ltrb[:, 2]) * stride
    y2 = (ay + ltrb[:, 3]) * stride
    return np.stack([x1, y1, x2, y2], axis=1)


def _nms(boxes: np.ndarray, scores: np.ndarray, iou_thres: float) -> List[int]:
    """Plain greedy NMS. Returns kept indices, highest score first."""
    if len(boxes) == 0:
        return []
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(x1[i], x1[rest])
        yy1 = np.maximum(y1[i], y1[rest])
        xx2 = np.minimum(x2[i], x2[rest])
        yy2 = np.minimum(y2[i], y2[rest])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        iou = inter / np.maximum(1e-9, areas[i] + areas[rest] - inter)
        order = rest[iou <= iou_thres]
    return keep


def decode_detections(
    heads: List[Tuple[np.ndarray, np.ndarray, np.ndarray, int]],
    conf_thres: float = 0.25,
    iou_thres: float = 0.7,
    max_det: int = 32,
) -> Dict[str, np.ndarray]:
    """Decode the three detection scales into boxes/scores/classes/mask-coeffs."""
    all_boxes, all_scores, all_cls, all_coef = [], [], [], []

    for box_head, cls_head, coeff_head, stride in heads:
        cls = cls_head.reshape(-1, cls_head.shape[-1]).astype(np.float32)
        is_prob = cls_is_probability(cls)

        best = cls.max(axis=1)
        if is_prob:
            thres = conf_thres
        elif 0.0 < conf_thres < 1.0:
            thres = float(np.log(conf_thres / (1.0 - conf_thres)))
        else:
            thres = -np.inf
        hit = np.nonzero(best >= thres)[0]
        if hit.size == 0:
            continue

        boxes = _decode_box(box_head, stride)[hit]
        all_boxes.append(boxes)
        all_scores.append(best[hit] if is_prob else _sigmoid(best[hit]))
        all_cls.append(cls[hit].argmax(axis=1))
        all_coef.append(
            coeff_head.reshape(-1, coeff_head.shape[-1])[hit].astype(np.float32)
        )

    if not all_boxes:
        return {
            "boxes": np.zeros((0, 4), np.float32),
            "scores": np.zeros((0,), np.float32),
            "classes": np.zeros((0,), np.int32),
            "coeffs": np.zeros((0, _N_PROTO), np.float32),
        }

    boxes = np.concatenate(all_boxes).astype(np.float32)
    scores = np.concatenate(all_scores).astype(np.float32)
    classes = np.concatenate(all_cls).astype(np.int32)
    coeffs = np.concatenate(all_coef).astype(np.float32)

    offset = classes.astype(np.float64) * 8192.0
    keep = _nms(boxes.astype(np.float64) + offset[:, None], scores, iou_thres)[:max_det]

    return {
        "boxes": boxes[keep],
        "scores": scores[keep],
        "classes": classes[keep],
        "coeffs": coeffs[keep],
    }


def build_class_map(
    det: Dict[str, np.ndarray],
    protos: np.ndarray,
    mask_thres: float = 0.5,
    input_size: int = 640,
) -> np.ndarray:
    """Paint decoded instances into one semantic class map at prototype resolution."""
    ph, pw = protos.shape[0], protos.shape[1]
    class_map = np.full((ph, pw), BACKGROUND_ID, dtype=CLASS_MAP_DTYPE)

    n = len(det["scores"])
    if n == 0:
        return class_map

    # (P, 32) x (32, H*W) -> (P, H*W): every instance mask in one matmul.
    proto_flat = protos.reshape(-1, protos.shape[-1]).astype(np.float32).T
    masks = _sigmoid(det["coeffs"] @ proto_flat).reshape(n, ph, pw)

    scale_x, scale_y = pw / input_size, ph / input_size
    xs = np.arange(pw, dtype=np.float32)[None, :]
    ys = np.arange(ph, dtype=np.float32)[:, None]

    for i in np.argsort(det["scores"]):
        x1, y1, x2, y2 = det["boxes"][i]
        inside = (
            (xs >= x1 * scale_x)
            & (xs <= x2 * scale_x)
            & (ys >= y1 * scale_y)
            & (ys <= y2 * scale_y)
        )
        hit = (masks[i] >= mask_thres) & inside
        class_map[hit] = det["classes"][i]

    return class_map
