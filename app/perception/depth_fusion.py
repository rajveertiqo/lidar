"""Camera + ToF depth: every ToF depth pixel placed on the front camera's picture (2026-10-13).

The ToF (240 x 180 distances) and the front camera's detector picture (960 x 540) are both
pinhole cameras, with different numbers: focal length in pixels, image centre, lens
distortion, where each sits and where it points. So their pictures do not line up by
stretching one over the other, nor by angle alone (the lenses are a few cm apart: about
2 deg at 1 m, 0.5 deg at 4 m). Each depth pixel is placed with its own distance:

    1. P  = d * K_tof^-1 [u, v, 1]   the point the ToF pixel (u, v) measured at distance d
    2. P' = R * P + t                that point in the camera's frame
    3. [u', v'] = K_cam * P' / Z'    the camera pixel it lands on

K_tof, K_cam (with each lens's distortion), R and t come from the bench calibration
(tools/device/tof_calibrate.py -> TOF_CALIBRATION_FILE), or until then from the design (config
TOF_* / FUSION_*). Each detection box then takes the distance of the nearest surface inside
it (DepthFrame.box_depth): a person's, not the wall's behind them.

The placed points also make a depth picture aligned to the camera's, as an RGB-D camera
gives (DepthFrame.depth_map, 2026-11-06): a distance per TOF_ALIGNED_CELL_PX block of camera
pixels, the nearest point wins where several land in one block (the camera sees the near
thing, not what it hides). Only where the two views overlap: the ToF looks 20 deg lower, so
the top of the camera's picture has no depth, and nothing past TOF_MAX_MM does.

Frames: OpenCV's, x right, y down, z ahead. "Level" is the camera's frame turned up by
FUSION_CAMERA_DOWN_DEG, so its z runs along the ground.
"""

import json
import math
import os
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from app.config import (
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    FRAME_GRID_COLS,
    FRAME_GRID_ROWS,
    FUSION_CAMERA_CROP_PX,
    FUSION_CAMERA_DOWN_DEG,
    FUSION_CAMERA_LENS_HFOV_DEG,
    FUSION_CAMERA_SENSOR_PX,
    FUSION_TOF_DOWN_DEG,
    FUSION_TOF_OFFSET_M,
    TOF_ALIGNED_CELL_PX,
    TOF_BOX_GROUP_M,
    TOF_BOX_MIN_POINTS,
    TOF_BOX_MIN_SHARE,
    TOF_BOX_TRIM,
    TOF_CALIBRATION_FILE,
    TOF_DEPTH_IS_RADIAL,
    TOF_DIAGONAL_FOV_DEG,
    TOF_ENABLED,
    TOF_MAX_MM,
    TOF_MIN_CONFIDENCE,
    TOF_MIN_MM,
    TOF_PAIR_MAX_S,
    TOF_SIZE_PX,
)

PICTURE_PX = (
    int(CAMERA_WIDTH // FRAME_GRID_COLS),
    int(CAMERA_HEIGHT // FRAME_GRID_ROWS),
)


def pitch_down(deg: float) -> np.ndarray:
    """The rotation taking a frame aimed `deg` further down into the frame it hangs from:
    its z axis (ahead) becomes (0, sin, cos), i.e. pointing below."""
    s, c = math.sin(math.radians(deg)), math.cos(math.radians(deg))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, s], [0.0, -s, c]])


@dataclass
class Lens:
    """A pinhole camera's numbers, in the pixels of one picture (OpenCV's convention:
    pixel centres at whole numbers; dist = k1, k2, p1, p2, k3)."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    dist: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)

    @classmethod
    def from_focal(cls, f: float, width: int, height: int) -> "Lens":
        return cls(f, f, (width - 1) / 2.0, (height - 1) / 2.0, int(width), int(height))

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Lens":
        dist = tuple(float(v) for v in (d.get("dist") or ()))
        return cls(
            float(d["fx"]),
            float(d["fy"]),
            float(d["cx"]),
            float(d["cy"]),
            int(d["width"]),
            int(d["height"]),
            (dist + (0.0,) * 5)[:5],
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy,
            "width": self.width,
            "height": self.height,
            "dist": list(self.dist),
        }

    def matrix(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]]
        )

    def resized(self, width: int, height: int) -> "Lens":
        """The same lens for the picture scaled to width x height (distortion unchanged)."""
        sx, sy = width / self.width, height / self.height
        return Lens(
            self.fx * sx,
            self.fy * sy,
            (self.cx + 0.5) * sx - 0.5,
            (self.cy + 0.5) * sy - 0.5,
            int(width),
            int(height),
            self.dist,
        )

    def fov_deg(self) -> Tuple[float, float]:
        return (
            math.degrees(2.0 * math.atan(self.width / 2.0 / self.fx)),
            math.degrees(2.0 * math.atan(self.height / 2.0 / self.fy)),
        )

    def _distorted(self) -> bool:
        return any(abs(k) > 1e-12 for k in self.dist)

    def normalised(self, u: np.ndarray, v: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Pixels -> (x/z, y/z) on the ideal (undistorted) image plane."""
        if not self._distorted():
            return (u - self.cx) / self.fx, (v - self.cy) / self.fy
        import cv2

        pts = np.stack([u, v], axis=-1).reshape(-1, 1, 2).astype(np.float64)
        und = cv2.undistortPoints(
            pts, self.matrix(), np.array(self.dist, dtype=np.float64)
        )
        und = und.reshape(-1, 2)
        return und[:, 0].reshape(np.shape(u)), und[:, 1].reshape(np.shape(u))

    def project(self, pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Points (N, 3) in this lens's frame -> pixels (u, v); z must be > 0."""
        x, y = pts[:, 0] / pts[:, 2], pts[:, 1] / pts[:, 2]
        if self._distorted():
            k1, k2, p1, p2, k3 = self.dist
            r2 = x * x + y * y
            radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
            x, y = (
                x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
                y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y,
            )
        return self.fx * x + self.cx, self.fy * y + self.cy


@dataclass
class Calibration:
    """The two lenses and how the ToF sits against the camera: P_cam = R P_tof + t (metres)."""

    tof: Lens
    cam: Lens
    R: np.ndarray
    t: np.ndarray
    source: str = "design"  # "design" until the bench calibration exists, then "bench"
    note: str = ""

    def summary(self) -> Dict[str, Any]:
        """For /tof/status: how far apart the lenses are and how differently they point."""
        fwd = self.R @ np.array([0.0, 0.0, 1.0])
        return {
            "source": self.source,
            "note": self.note,
            "baseline_cm": round(float(np.linalg.norm(self.t)) * 100.0, 1),
            "tof_down_deg": round(math.degrees(math.atan2(fwd[1], fwd[2])), 1),
            "tof_right_deg": round(math.degrees(math.atan2(fwd[0], fwd[2])), 1),
            "tof_fov_deg": [round(a, 1) for a in self.tof.fov_deg()],
            "camera_fov_deg": [round(a, 1) for a in self.cam.fov_deg()],
        }


def design_calibration(note: str = "") -> Calibration:
    """Ideal lenses placed as in the 3D design, until the bench calibration exists."""
    tw, th = TOF_SIZE_PX
    f_tof = (
        math.hypot(tw, th) / 2.0 / math.tan(math.radians(TOF_DIAGONAL_FOV_DEG) / 2.0)
    )
    sensor_w = FUSION_CAMERA_SENSOR_PX[0]
    f_sensor = (
        sensor_w / 2.0 / math.tan(math.radians(FUSION_CAMERA_LENS_HFOV_DEG) / 2.0)
    )
    f_cam = f_sensor * PICTURE_PX[0] / FUSION_CAMERA_CROP_PX[0]  # crop, then scale down
    return Calibration(
        Lens.from_focal(f_tof, tw, th),
        Lens.from_focal(f_cam, *PICTURE_PX),
        pitch_down(FUSION_TOF_DOWN_DEG),
        np.array(FUSION_TOF_OFFSET_M, dtype=float),
        "design",
        note or "no bench calibration yet: ideal lenses, design position",
    )


def load_calibration(path: Optional[str] = None) -> Calibration:
    """The bench calibration (tools/device/tof_calibrate.py), or the design if there is none or it is unusable."""
    path = path or TOF_CALIBRATION_FILE
    if not os.path.exists(path):
        return design_calibration()
    try:
        with open(path) as f:
            d = json.load(f)
        tof, cam = Lens.from_dict(d["tof"]), Lens.from_dict(d["camera"])
        R = np.array(d["R"], dtype=float).reshape(3, 3)
        t = np.array(d["t_m"], dtype=float).reshape(3)
        if abs(np.linalg.det(R) - 1.0) > 1e-3 or np.linalg.norm(t) > 0.5:
            raise ValueError("R is not a rotation, or the lenses are over 50 cm apart")
        if (cam.width, cam.height) != PICTURE_PX:
            if abs(cam.width / cam.height - PICTURE_PX[0] / PICTURE_PX[1]) > 0.01:
                raise ValueError(
                    f"camera calibrated at {cam.width}x{cam.height}, "
                    f"not the shape of the {PICTURE_PX[0]}x{PICTURE_PX[1]} picture"
                )
            cam = cam.resized(*PICTURE_PX)
        return Calibration(
            tof,
            cam,
            R,
            t,
            "bench",
            f"{d.get('date', '?')}: {d.get('pairs', '?')} pairs, "
            f"stereo error {d.get('rms', {}).get('stereo', '?')} px",
        )
    except Exception as e:
        return design_calibration(
            f"{os.path.basename(path)} unusable ({e}); design used"
        )


class DepthFrame:
    """One ToF frame, its depth points placed on the camera's picture."""

    def __init__(
        self,
        t: float,
        u: np.ndarray,
        v: np.ndarray,
        rng: np.ndarray,
        ground: np.ndarray,
        side: np.ndarray,
        ahead: np.ndarray,
        source: str,
        pixels: int,
        size: Tuple[int, int] = PICTURE_PX,
    ):
        self.t = t  # time.time() of capture
        self.u, self.v = u, v  # camera pixel of each point (only those in the picture)
        self.rng = rng  # metres from the camera
        self.ground = ground  # metres along the ground from the camera
        self.side = side  # metres right of the camera's line (level)
        self.ahead = ahead  # metres ahead (level)
        self.source = source  # calibration: "design" | "bench"
        self.pixels = pixels  # depth pixels with a usable reading
        self.size = (int(size[0]), int(size[1]))  # the camera picture: width, height
        self._aligned: Dict[int, np.ndarray] = {}  # cell px -> aligned(), made once

    def __len__(self) -> int:
        return int(self.u.size)

    def aligned(self, cell: int = TOF_ALIGNED_CELL_PX) -> np.ndarray:
        """The depth picture aligned to the camera's, as point numbers: for each `cell` x
        `cell` block of camera pixels (rows, columns), the point nearest the camera of
        those that land in it, -1 where none does. A block with none, beside one that has,
        takes the nearest of its eight neighbours' (a ToF pixel lands ~4.3 camera pixels
        from the next, so 4-px blocks leave such gaps); wider gaps stay empty. Made once
        per frame: every view and lookup of it shares the one array (do not write to it)."""
        cell = max(1, int(cell))
        got = self._aligned.get(cell)
        if got is not None:
            return got
        W, H = self.size
        gw, gh = -(-W // cell), -(-H // cell)
        idx = np.full(gh * gw, -1, dtype=np.int64)
        if len(self):
            # Pixel centres sit at whole numbers, so the picture runs -0.5 .. W - 0.5.
            cu = np.clip(((self.u + 0.5) // cell).astype(np.int64), 0, gw - 1)
            cv = np.clip(((self.v + 0.5) // cell).astype(np.int64), 0, gh - 1)
            block = cv * gw + cu
            order = np.lexsort((self.rng, block))  # by block, nearest first in each
            b = block[order]
            first = np.ones(b.size, dtype=bool)
            first[1:] = b[1:] != b[:-1]
            idx[b[first]] = order[first]
        idx = idx.reshape(gh, gw)
        rng = np.append(self.rng, np.inf)  # index -1 reads as infinitely far
        hole = idx < 0
        if hole.any() and len(self):
            best, best_r = idx.copy(), rng[idx]
            pad = np.pad(idx, 1, constant_values=-1)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dy == 0 and dx == 0:
                        continue
                    nb = pad[1 + dy : 1 + dy + gh, 1 + dx : 1 + dx + gw]
                    nr = rng[nb]
                    take = hole & (nr < best_r)
                    best[take], best_r[take] = nb[take], nr[take]
            idx = best
        idx.setflags(write=False)
        self._aligned[cell] = idx
        return idx

    def depth_map(
        self, cell: int = TOF_ALIGNED_CELL_PX, what: str = "range"
    ) -> np.ndarray:
        """The aligned depth picture in metres, float32 (rows, columns of `cell`-px blocks;
        NaN where the ToF has nothing): "range" from the camera's lens, or "ground" along
        the ground (box_depth's distance_m)."""
        vals = {"range": self.rng, "ground": self.ground}[what]
        return np.append(vals, np.nan).astype(np.float32)[self.aligned(cell)]

    def box_depth(
        self, box: Sequence[float], trim: Sequence[float] = TOF_BOX_TRIM
    ) -> Optional[Dict[str, Any]]:
        """The nearest surface inside a detection box (camera pixels x1, y1, x2, y2), or None.

        The box is trimmed (sides, top, bottom: the ground at its feet), the depth points
        inside are grouped where they are no more than TOF_BOX_GROUP_M apart along the
        ground, and the nearest group with at least TOF_BOX_MIN_SHARE of them (and
        TOF_BOX_MIN_POINTS) is the thing: a person, not the wall seen past them."""
        try:
            x1, y1, x2, y2 = (float(b) for b in box)
        except (TypeError, ValueError):
            return None
        w, h = x2 - x1, y2 - y1
        if w <= 0 or h <= 0 or not len(self):
            return None
        side_k, top_k, bottom_k = trim
        inside = (
            (self.u >= x1 + side_k * w)
            & (self.u <= x2 - side_k * w)
            & (self.v >= y1 + top_k * h)
            & (self.v <= y2 - bottom_k * h)
        )
        idx = np.flatnonzero(inside)
        n = idx.size
        if n < TOF_BOX_MIN_POINTS:
            return None
        order = idx[np.argsort(self.ground[idx])]
        g = self.ground[order]
        cuts = np.flatnonzero(np.diff(g) > TOF_BOX_GROUP_M) + 1
        need = max(TOF_BOX_MIN_POINTS, int(math.ceil(TOF_BOX_MIN_SHARE * n)))
        start = 0
        for end in list(cuts) + [n]:
            if end - start >= need:
                sel = order[start:end]
                gd = self.ground[sel]
                q1, q3 = np.percentile(gd, [25, 75])
                side, ahead = (
                    float(np.median(self.side[sel])),
                    float(np.median(self.ahead[sel])),
                )
                return {
                    "distance_m": round(float(np.median(gd)), 3),
                    "range_m": round(float(np.median(self.rng[sel])), 3),
                    "bearing_deg": round(math.degrees(math.atan2(side, ahead)), 1),
                    "points": int(sel.size),
                    "share": round(sel.size / n, 2),
                    "spread_m": round(float(q3 - q1), 3),
                    "calibration": self.source,
                }
            start = end
        return None


class DepthProjector:
    """Places ToF frames on the camera's picture. The ToF's rays are worked out once."""

    def __init__(
        self, calib: Optional[Calibration] = None, radial: bool = TOF_DEPTH_IS_RADIAL
    ):
        self.calib = calib or load_calibration()
        tof = self.calib.tof
        vv, uu = np.mgrid[0 : tof.height, 0 : tof.width].astype(np.float64)
        xn, yn = tof.normalised(uu.ravel(), vv.ravel())
        rays = np.stack([xn, yn, np.ones_like(xn)], axis=1)
        if radial:  # the reading is along the ray: a unit ray per metre
            rays /= np.linalg.norm(rays, axis=1, keepdims=True)
        self.rays = rays  # (H*W, 3): metres in the ToF's frame per metre read
        self.radial = radial
        self.level = pitch_down(FUSION_CAMERA_DOWN_DEG)  # camera frame -> level frame

    def project(
        self,
        depth_mm: np.ndarray,
        confidence: Optional[np.ndarray] = None,
        t: float = 0.0,
    ) -> DepthFrame:
        """One ToF frame (H x W distances in mm, and the sensor's confidence) -> a DepthFrame."""
        c = self.calib
        d = np.asarray(depth_mm, dtype=np.float64).reshape(-1)
        keep = np.isfinite(d) & (d >= TOF_MIN_MM) & (d <= TOF_MAX_MM)
        if confidence is not None:
            keep &= (
                np.asarray(confidence, dtype=np.float64).reshape(-1)
                >= TOF_MIN_CONFIDENCE
            )
        pixels = int(keep.sum())
        P = self.rays[keep] * (d[keep, None] / 1000.0)  # 1. the measured points
        Pc = P @ c.R.T + c.t  # 2. in the camera's frame
        ahead_of_lens = Pc[:, 2] > 0.05
        Pc = Pc[ahead_of_lens]
        u, v = c.cam.project(Pc)  # 3. onto the camera's picture
        seen = (
            (u >= -0.5)
            & (u < c.cam.width - 0.5)
            & (v >= -0.5)
            & (v < c.cam.height - 0.5)
        )
        Pc, u, v = Pc[seen], u[seen], v[seen]
        L = Pc @ self.level.T
        return DepthFrame(
            t,
            u,
            v,
            np.linalg.norm(Pc, axis=1),
            np.hypot(L[:, 0], L[:, 2]),
            L[:, 0],
            L[:, 2],
            c.source,
            pixels,
            (c.cam.width, c.cam.height),
        )


_projector: Optional[DepthProjector] = None


def get_projector() -> DepthProjector:
    global _projector
    if _projector is None:
        _projector = DepthProjector()
    return _projector


def reload_calibration() -> Dict[str, Any]:
    """Re-read TOF_CALIBRATION_FILE (after tools/device/tof_calibrate.py solve)."""
    global _projector
    _projector = DepthProjector()
    return _projector.calib.summary()


def annotate(
    detections: List[Dict[str, Any]], frame_time: float, sector: str = "front"
) -> int:
    """Give each front detection a "depth" (box_depth) from the ToF frame captured nearest
    the picture, within TOF_PAIR_MAX_S. Returns how many got one."""
    if not TOF_ENABLED or sector != "front" or not detections:
        return 0
    from app.sensors import tof_camera

    frame = tof_camera.nearest(frame_time, TOF_PAIR_MAX_S)
    if frame is None:
        return 0
    got = 0
    for det in detections:
        depth = frame.box_depth(det.get("bbox") or ())
        if depth is not None:
            depth["age_s"] = round(float(frame_time) - frame.t, 3)
            det["depth"] = depth
            got += 1
    return got


def draw_overlay(
    picture_bgr: np.ndarray, frame: Optional[DepthFrame], step: int = 1
) -> np.ndarray:
    """The camera's picture with the depth points drawn where they land, coloured by
    distance (red near -> blue at TOF_MAX_MM): the check that the calibration is right
    (a table's dots on the table, the floor's on the floor)."""
    import cv2

    out = picture_bgr.copy()
    if frame is None or not len(frame):
        return out
    k = max(1, int(step))
    hue = np.clip(frame.rng[::k] / (TOF_MAX_MM / 1000.0), 0.0, 1.0) * 120.0
    hsv = np.stack([hue, np.full_like(hue, 255.0), np.full_like(hue, 255.0)], axis=1)
    bgr = cv2.cvtColor(
        hsv.astype(np.uint8).reshape(-1, 1, 3), cv2.COLOR_HSV2BGR
    ).reshape(-1, 3)
    us = np.round(frame.u[::k]).astype(int)
    vs = np.round(frame.v[::k]).astype(int)
    for x, y, col in zip(us, vs, bgr):
        cv2.circle(
            out, (int(x), int(y)), 1, (int(col[0]), int(col[1]), int(col[2])), -1
        )
    return out


def _label_spot(placed, x, y, w, h, limit):
    """Where a label (top-left x, baseline y) goes so it covers no label already placed:
    moved down a line at a time, at most a few (two boxes side by side hid each other's
    label, 2026-11-06)."""
    for _ in range(6):
        box = (x, y - h - 4, x + w + 6, y + 4)
        if not any(box[0] < b[2] and b[0] < box[2] and box[1] < b[3] and b[1] < box[3] for b in placed):
            break
        y += h + 8
    y = min(y, limit)
    placed.append((x, y - h - 4, x + w + 6, y + 4))
    return y


def draw_rgbd(
    picture_bgr: np.ndarray,
    frame: Optional[DepthFrame],
    detections: Sequence[Dict[str, Any]] = (),
    cell: int = TOF_ALIGNED_CELL_PX,
    alpha: float = 0.45,
    width: Optional[int] = None,
) -> np.ndarray:
    """The front camera as a depth camera: its picture with the aligned depth laid over it
    (red near -> blue at TOF_MAX_MM; the picture's own colours where the ToF has nothing)
    and each detection boxed with the distance the ToF measured in it (its "depth", along
    the ground, from annotate), or "no depth" where it has none: past TOF_MAX_MM, or above
    the ToF's view. `width`: drawn this wide (the page asks for the width it shows) — at
    1164 px the blend took 65 ms and the JPEG was 231 KB; at 582, ~5 ms and ~31 KB."""
    import cv2

    H0, W0 = picture_bgr.shape[:2]
    if width and 0 < int(width) < W0:
        W = int(width)
        H = max(1, int(round(H0 * W / W0)))
        out = cv2.resize(picture_bgr, (W, H), interpolation=cv2.INTER_AREA)
    else:
        W, H, out = W0, H0, picture_bgr.copy()
    sx, sy = W / W0, H / H0
    if frame is not None and len(frame):
        rng = frame.depth_map(cell)
        fw, fh = frame.size
        hue = np.clip(np.nan_to_num(rng, nan=0.0) / (TOF_MAX_MM / 1000.0), 0.0, 1.0)
        hue = (hue * 120.0).astype(np.uint8)
        full = np.full_like(hue, 255)
        col = cv2.cvtColor(np.dstack([hue, full, full]), cv2.COLOR_HSV2BGR)
        have = np.isfinite(rng).astype(np.uint8)
        # Blocks straight onto this picture: each is `cell` camera pixels of the fw x fh
        # picture the calibration places points on.
        size = (
            max(1, int(round(rng.shape[1] * cell * W / fw))),
            max(1, int(round(rng.shape[0] * cell * H / fh))),
        )
        col = cv2.resize(col, size, interpolation=cv2.INTER_NEAREST)[:H, :W]
        have = cv2.resize(have, size, interpolation=cv2.INTER_NEAREST)[:H, :W]
        if col.shape[:2] != (H, W):  # a pixel short by rounding
            col = cv2.resize(col, (W, H), interpolation=cv2.INTER_NEAREST)
            have = cv2.resize(have, (W, H), interpolation=cv2.INTER_NEAREST)
        blend = cv2.addWeighted(out, 1.0 - alpha, col, alpha, 0.0)
        out = np.where(have[..., None].astype(bool), blend, out)
    font = cv2.FONT_HERSHEY_SIMPLEX
    big = W >= 900
    fs, thick = (0.6, 2) if big else (0.45, 1)
    placed: List[Tuple[int, int, int, int]] = []
    labels = []
    for det in detections:
        try:
            x1, y1, x2, y2 = (float(b) for b in det.get("bbox") or ())
        except (TypeError, ValueError):
            continue
        x1, x2 = int(round(x1 * sx)), int(round(x2 * sx))
        y1, y2 = int(round(y1 * sy)), int(round(y2 * sy))
        name = str(det.get("class_name") or "object")
        m = (det.get("depth") or {}).get("distance_m")
        if isinstance(m, (int, float)):
            text, colour = f"{name} {m:.1f} m ({m / 0.3048:.1f} ft)", (255, 255, 255)
        else:
            text, colour = f"{name}: no depth", (170, 170, 170)
        cv2.rectangle(out, (x1, y1), (x2, y2), colour, thick)
        labels.append((x1, y1, text, colour))
    for x1, y1, text, colour in labels:  # after every box, so no box is drawn over a label
        (tw, th), base = cv2.getTextSize(text, font, fs, thick)
        x1 = max(0, min(x1, W - tw - 7))  # inside the picture, a box at its right edge too
        ty = y1 - 6 if y1 - th - 10 >= 0 else y1 + th + 6
        ty = _label_spot(placed, x1, ty, tw, th, H - base - 1)
        cv2.rectangle(out, (x1, ty - th - 4), (x1 + tw + 6, ty + base), (0, 0, 0), -1)
        cv2.putText(out, text, (x1 + 3, ty), font, fs, colour, thick, cv2.LINE_AA)
    return out
