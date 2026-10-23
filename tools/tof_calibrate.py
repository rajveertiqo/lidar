#!/usr/bin/env python3
"""Bench calibration of the ToF depth camera against the front camera (2026-10-13).

Gives utils/depth_fusion.py its numbers: each lens's K (focal length in pixels, image
centre) and distortion, and R, t — where the ToF sits and how it points, seen from the
camera. Until this has run, the dots are placed by the design and /health says so.

On the bench (the app running with TOF_ENABLED, the shoulder unit rigid, the camera's
focus locked — a focus change moves its focal length):
  1. Print a checkerboard on matte paper (gloss blinds the ToF) and fix it to something
     flat: 9 x 6 inner corners (10 x 7 squares — an odd and an even count, so it does not
     look the same turned round), 35 mm squares on A3 (or 25 mm on A4, held nearer).
  2. Hold it 0.5-1.5 m away and capture (/tof/overlay.jpg shows the camera's picture,
     /tof/depth.jpg the ToF's):
        curl -X POST -H "X-Admin-Token: $SHAPPE_ADMIN_TOKEN" http://localhost:8000/tof/calibrate/capture
     15-25 times where BOTH pictures see all of it (they overlap only in the camera's
     lower part), moving and tilting the board, near the overlap's edges too. Optional:
     more across the rest of either picture — each lens learns its own numbers from every
     view it sees the board in (R and t come from the shared ones); the desk test places
     the dots within ~0.5 px either way (tools/tof_fusion_sim.py).
  3. python3 tools/tof_calibrate.py solve --board 9x6 --square-mm 35
     then POST /tof/calibrate/reload (or restart) and look at /tof/overlay.jpg: a table's
     dots on the table and the floor's on the floor, near and far.
Also:
  python3 tools/tof_calibrate.py wall calibration/tof_pairs/001_depth.npy
     the ToF pointed square-on at a flat wall: is its distance along each pixel's ray
     (TOF_DEPTH_IS_RADIAL = True) or along the lens's axis (False)?
  python3 tools/tof_calibrate.py check    overlay pictures of every pair, to look at
"""

import argparse
import datetime
import glob
import json
import math
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import config  # noqa: E402
from utils import depth_fusion as df  # noqa: E402

TOF_UPSCALE = 4  # the 240 x 180 picture is enlarged this much to find the corners


def board_points(cols: int, rows: int, square_mm: float) -> np.ndarray:
    """The board's inner corners in its own plane (mm), in OpenCV's order."""
    grid = np.zeros((rows * cols, 3), np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm
    return grid


def find_corners(
    gray: np.ndarray, cols: int, rows: int, upscale: int = 1
) -> Optional[np.ndarray]:
    """The board's inner corners in a grey picture (N, 1, 2), or None."""
    import cv2

    img = gray
    if upscale > 1:
        img = cv2.resize(
            gray, None, fx=upscale, fy=upscale, interpolation=cv2.INTER_CUBIC
        )
        img = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(img)
    found, corners = False, None
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(
            img, (cols, rows), cv2.CALIB_CB_NORMALIZE_IMAGE
        )
    if not found:
        found, corners = cv2.findChessboardCorners(
            img,
            (cols, rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if found:
            corners = cv2.cornerSubPix(
                img,
                corners,
                (5, 5),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-3),
            )
    if not found:
        return None
    corners = corners.reshape(-1, 1, 2).astype(np.float32)
    if upscale > 1:  # back to the small picture's pixels (centres at whole numbers)
        corners = (corners + 0.5) / upscale - 0.5
    return corners


def same_order(cam: np.ndarray, tof: np.ndarray) -> np.ndarray:
    """The ToF's corners in the camera's order. A board that looks the same turned round
    may be numbered from opposite ends in the two pictures; the two lenses are not turned
    against each other, so the first-to-last corner runs the same way in both."""
    a = cam.reshape(-1, 2)[-1] - cam.reshape(-1, 2)[0]
    b = tof.reshape(-1, 2)[-1] - tof.reshape(-1, 2)[0]
    return tof[::-1].copy() if float(a @ b) < 0.0 else tof


def calibrate(
    views: Sequence[Tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]],
    tof_size: Tuple[int, int],
    cam_size: Tuple[int, int],
) -> Dict:
    """Each lens from every view it saw the board in, then the pair from the views both
    saw it in: P_cam = R P_tof + t. Views are (board points, ToF corners or None, camera
    corners or None); board units in, metres out. The pair's views only cover where the
    two pictures overlap (the camera's lower part), so boards seen by one lens alone —
    across its whole picture — are what pin its own numbers down."""
    import cv2

    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    tof_v = [(o, t) for o, t, _ in views if t is not None]
    cam_v = [(o, c) for o, _, c in views if c is not None]
    both = [(o, t, c) for o, t, c in views if t is not None and c is not None]
    if len(tof_v) < 6 or len(cam_v) < 6 or len(both) < 4:
        raise ValueError(
            f"too few views: ToF {len(tof_v)}, camera {len(cam_v)}, both "
            f"{len(both)} (need 6, 6 and 4)"
        )
    # The small ToF picture cannot pin a 3rd radial term down; a fixed k3 keeps it honest.
    rms_t, Kt, Dt, _, _ = cv2.calibrateCamera(
        [o for o, _ in tof_v],
        [t for _, t in tof_v],
        tof_size,
        None,
        None,
        flags=cv2.CALIB_FIX_K3,
        criteria=crit,
    )
    rms_c, Kc, Dc, _, _ = cv2.calibrateCamera(
        [o for o, _ in cam_v],
        [c for _, c in cam_v],
        cam_size,
        None,
        None,
        criteria=crit,
    )
    out = cv2.stereoCalibrate(
        [o for o, _, _ in both],
        [t for _, t, _ in both],
        [c for _, _, c in both],
        Kt,
        Dt,
        Kc,
        Dc,
        cam_size,
        flags=cv2.CALIB_FIX_INTRINSIC,
        criteria=crit,
    )
    rms_s, R, T = out[0], out[5], out[6]

    def lens(K, D, size):
        d = [float(v) for v in np.ravel(D)[:5]] + [0.0] * max(0, 5 - np.size(D))
        return {
            "fx": float(K[0, 0]),
            "fy": float(K[1, 1]),
            "cx": float(K[0, 2]),
            "cy": float(K[1, 2]),
            "width": int(size[0]),
            "height": int(size[1]),
            "dist": d,
        }

    return {
        "tof": lens(Kt, Dt, tof_size),
        "camera": lens(Kc, Dc, cam_size),
        "R": np.asarray(R).tolist(),
        "t_m": (np.ravel(T) / 1000.0).tolist(),
        "rms": {
            "tof": round(float(rms_t), 3),
            "camera": round(float(rms_c), 3),
            "stereo": round(float(rms_s), 3),
        },
        "pairs": len(both),
        "views": {"tof": len(tof_v), "camera": len(cam_v)},
    }


def _pairs(folder: str) -> List[str]:
    return sorted(
        p[: -len("_cam.png")] for p in glob.glob(os.path.join(folder, "*_cam.png"))
    )


def solve(args) -> int:
    import cv2

    cols, rows = (int(v) for v in args.board.lower().split("x"))
    grid = board_points(cols, rows, args.square_mm)
    views, used = [], []
    cam_size = tof_size = None
    for stem in _pairs(args.pairs):
        cam = cv2.imread(stem + "_cam.png", cv2.IMREAD_GRAYSCALE)
        tof = cv2.imread(stem + "_tof.png", cv2.IMREAD_GRAYSCALE)
        if cam is None or tof is None:
            continue
        cam_size, tof_size = (cam.shape[1], cam.shape[0]), (tof.shape[1], tof.shape[0])
        c = find_corners(cam, cols, rows)
        t = find_corners(tof, cols, rows, TOF_UPSCALE)
        name = os.path.basename(stem)
        if c is None and t is None:
            print(f"  {name}: board not found in either picture — skipped")
            continue
        if c is None or t is None:
            print(
                f"  {name}: board only in the {'ToF' if c is None else 'camera'}'s picture "
                f"— used for that lens alone"
            )
        if c is not None and t is not None:
            t = same_order(c, t)
        views.append((grid, t, c))
        used.append(name)
    print(
        f"{len(used)} usable of {len(_pairs(args.pairs))}: "
        f"{sum(1 for _, t, c in views if t is not None and c is not None)} seen by both"
    )
    try:
        res = calibrate(views, tof_size, cam_size)
    except ValueError as e:
        print(
            f"{e}. Capture more: 15-25 with the board where both pictures see all of it, "
            f"moved and tilted."
        )
        return 1
    res.update(
        {
            "date": datetime.date.today().isoformat(),
            "board": args.board,
            "square_mm": args.square_mm,
            "used": used,
        }
    )
    cal = df.Calibration(
        df.Lens.from_dict(res["tof"]),
        df.Lens.from_dict(res["camera"]),
        np.array(res["R"]),
        np.array(res["t_m"]),
        "bench",
    )
    s = cal.summary()
    print(
        f"error: ToF {res['rms']['tof']} px, camera {res['rms']['camera']} px, "
        f"pair {res['rms']['stereo']} px"
    )
    print(
        f"ToF view {s['tof_fov_deg'][0]} x {s['tof_fov_deg'][1]} deg; camera picture "
        f"{s['camera_fov_deg'][0]} x {s['camera_fov_deg'][1]} deg"
    )
    print(
        f"lenses {s['baseline_cm']} cm apart; ToF aimed {s['tof_down_deg']} deg further "
        f"down and {s['tof_right_deg']} deg right of the camera "
        f"(design: {math.hypot(*config.FUSION_TOF_OFFSET_M) * 100:.1f} cm, "
        f"{config.FUSION_TOF_DOWN_DEG:.0f} deg down)"
    )
    if res["rms"]["stereo"] > 1.0:
        print(
            "WARNING: the pair's error is over 1 px — a pair was blurred or the board "
            "moved between the two pictures; look at `check` and drop bad pairs."
        )
    out = args.out or config.TOF_CALIBRATION_FILE
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        json.dump(res, f, indent=2)
    print(f"written {out}: POST /tof/calibrate/reload (or restart) to use it")
    return 0


def check(args) -> int:
    """Each pair's camera picture with its own depth drawn where the calibration places it."""
    import cv2

    proj = df.DepthProjector(df.load_calibration(args.calib))
    print(f"calibration: {proj.calib.source} — {proj.calib.note}")
    out_dir = os.path.join(args.pairs, "check")
    os.makedirs(out_dir, exist_ok=True)
    for stem in _pairs(args.pairs):
        cam = cv2.imread(stem + "_cam.png")
        if cam is None or not os.path.exists(stem + "_depth.npy"):
            continue
        frame = proj.project(np.load(stem + "_depth.npy"), None)
        path = os.path.join(out_dir, os.path.basename(stem) + "_overlay.jpg")
        cv2.imwrite(path, df.draw_overlay(cam, frame))
        print(f"  {path}: {len(frame)} of {frame.pixels} depth points in the picture")
    return 0


def wall_fit(
    depth_mm: np.ndarray, lens: df.Lens, middle: float = 0.6
) -> Dict[str, float]:
    """How flat a wall comes out (RMS mm off the best plane) read each way, from the
    middle of the picture (the floor or ceiling may be at its edges)."""
    out = {}
    depth = np.asarray(depth_mm, dtype=np.float64)
    h, w = depth.shape
    vv, uu = np.mgrid[0:h, 0:w]
    centre = (
        (np.abs(uu - (w - 1) / 2.0) <= middle * w / 2.0)
        & (np.abs(vv - (h - 1) / 2.0) <= middle * h / 2.0)
    ).reshape(-1)
    for radial in (True, False):
        p = df.DepthProjector(
            df.Calibration(lens, lens, np.eye(3), np.zeros(3)), radial
        )
        d = depth.reshape(-1)
        ok = centre & np.isfinite(d) & (d > config.TOF_MIN_MM) & (d < config.TOF_MAX_MM)
        pts = p.rays[ok] * d[ok, None]
        c = pts - pts.mean(axis=0)
        normal = np.linalg.svd(c, full_matrices=False)[2][2]
        out["radial" if radial else "axis"] = float(np.sqrt(np.mean((c @ normal) ** 2)))
    return out


def wall(args) -> int:
    lens = df.load_calibration(args.calib).tof
    fit = wall_fit(np.load(args.depth), lens)
    best = "radial" if fit["radial"] < fit["axis"] else "axis"
    print(
        f"off a flat plane: read along each ray {fit['radial']:.1f} mm, along the lens "
        f"axis {fit['axis']:.1f} mm"
    )
    print(
        f"-> TOF_DEPTH_IS_RADIAL = {best == 'radial'} "
        f"(config now {config.TOF_DEPTH_IS_RADIAL})"
    )
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("solve", help="calibrate from the captured pairs")
    s.add_argument("--pairs", default=config.TOF_CALIBRATION_PAIRS_DIR)
    s.add_argument("--board", default="9x6", help="inner corners, columns x rows")
    s.add_argument("--square-mm", type=float, default=35.0)
    s.add_argument("--out", default=None)
    c = sub.add_parser("check", help="overlay pictures of every pair")
    c.add_argument("--pairs", default=config.TOF_CALIBRATION_PAIRS_DIR)
    c.add_argument("--calib", default=None)
    w = sub.add_parser("wall", help="is the ToF's distance along the ray or the axis?")
    w.add_argument("depth", help="a *_depth.npy of a flat wall, square-on")
    w.add_argument("--calib", default=None)
    args = ap.parse_args(argv)
    return {"solve": solve, "check": check, "wall": wall}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
