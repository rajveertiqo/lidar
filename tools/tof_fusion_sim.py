#!/usr/bin/env python3
"""The camera + ToF merge (utils/depth_fusion.py) and its bench calibration
(tools/tof_calibrate.py) without the sensor, which is not bought yet (2026-10-13).

A synthetic street seen from the shoulder: ground, a person, a pole, a table and a wall,
rendered through a ToF whose lens, distortion, position and aim are known exactly (and
differ a little from the design, as a real one will). Checks:
  - each depth pixel lands on the camera pixel where its true 3D point is (with distortion);
  - each detection box gets the distance of the thing, not the wall behind it, cut off by
    the picture or not; nothing when the ToF cannot see it (too far, above its view);
  - the two wrong ways (stretching the depth picture, angles only) and the design numbers
    are measurably off, which is why the bench calibration exists;
  - the calibration solver gets the lenses and the pose back from checkerboard corners;
  - the wall check tells a ray reading from an axis reading;
  - the detection pass, detection_distance, the guidance and the tracker use the ToF's
    distance only with TOF_USE_DISTANCE;
  - the cost per frame on this machine.
"""

import math
import sys
import tempfile
import time
import json
import os
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import config  # noqa: E402
from utils import depth_fusion as df  # noqa: E402
from tools import tof_calibrate as tc  # noqa: E402

CAM_HEIGHT_M = 1.45  # the camera above the ground
RNG = np.random.default_rng(7)

# The "true" rig: lenses with distortion and off-centre, the ToF a little off the design.
TRUE_TOF = df.Lens(
    214.0, 213.2, 121.3, 88.6, 240, 180, (-0.12, 0.03, 0.001, -0.0005, 0.0)
)
TRUE_CAM = df.Lens(
    896.0, 895.1, 482.1, 268.4, 960, 540, (0.05, -0.08, 0.0004, 0.0002, 0.0)
)


def _rot(axis: str, deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return {
        "x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
        "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
        "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]),
    }[axis]


TRUE_R = df.pitch_down(21.0) @ _rot("y", 1.2) @ _rot("z", 0.8)
TRUE_T = np.array([0.004, 0.039, 0.007])
TRUE = df.Calibration(TRUE_TOF, TRUE_CAM, TRUE_R, TRUE_T, "bench", "the simulated rig")
LEVEL = df.pitch_down(config.FUSION_CAMERA_DOWN_DEG)  # camera frame -> level frame


# ---------------------------------------------------------------- the scene, level frame
# (x right, y down, z ahead, origin at the camera's lens)


def box(x0, x1, top_h, bottom_h, z0, z1):
    """An upright box between heights above the ground (m)."""
    return (
        np.array([x0, CAM_HEIGHT_M - top_h, z0]),
        np.array([x1, CAM_HEIGHT_M - bottom_h, z1]),
    )


def render(things, wall_z=None, radial=True, noise=0.01, calib=TRUE):
    """The ToF's depth picture (mm) and confidence of a scene, and each pixel's true 3D
    point (level frame, NaN where nothing is hit)."""
    tof = calib.tof
    vv, uu = np.mgrid[0 : tof.height, 0 : tof.width].astype(np.float64)
    xn, yn = tof.normalised(uu.ravel(), vv.ravel())
    rays_tof = np.stack([xn, yn, np.ones_like(xn)], axis=1)
    unit = rays_tof / np.linalg.norm(rays_tof, axis=1, keepdims=True)
    D = unit @ (LEVEL @ calib.R).T  # ray directions, level frame
    O = LEVEL @ calib.t  # the ToF's lens, level frame
    s = np.full(len(D), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        g = (CAM_HEIGHT_M - O[1]) / D[:, 1]  # the ground
        s = np.where((D[:, 1] > 0) & (g > 0), np.minimum(s, g), s)
        if wall_z is not None:
            w = (wall_z - O[2]) / D[:, 2]
            s = np.where((D[:, 2] > 0) & (w > 0), np.minimum(s, w), s)
        for lo, hi in things:
            t1, t2 = (lo - O) / D, (hi - O) / D
            tmin = np.nanmax(np.minimum(t1, t2), axis=1)
            tmax = np.nanmin(np.maximum(t1, t2), axis=1)
            hit = (tmax >= np.maximum(tmin, 0)) & (tmin > 0)
            s = np.where(hit, np.minimum(s, tmin), s)
    pts = np.where(np.isfinite(s)[:, None], O + D * s[:, None], np.nan)
    reading = s if radial else s * unit[:, 2]  # along the ray, or the axis
    depth = reading * 1000.0 * (1.0 + noise * RNG.standard_normal(len(s)))
    conf = np.where(np.isfinite(s), 200.0, 0.0)
    depth = np.where(np.isfinite(depth), depth, 0.0)
    return (
        depth.reshape(tof.height, tof.width),
        conf.reshape(tof.height, tof.width),
        pts,
    )


def cam_pixel(p_level, lens=TRUE_CAM):
    pc = (LEVEL.T @ np.asarray(p_level, dtype=float).T).T
    u, v = lens.project(pc.reshape(-1, 3))
    return u, v


def detection_box(lo, hi):
    """The box a detector would draw: the 3D box's corners in the true camera picture."""
    xs = [lo[0], hi[0]]
    ys = [lo[1], hi[1]]
    zs = [lo[2], hi[2]]
    corners = np.array([[x, y, z] for x in xs for y in ys for z in zs])
    u, v = cam_pixel(corners)
    return [
        max(0.0, u.min()),
        max(0.0, v.min()),
        min(959.0, u.max()),
        min(539.0, v.max()),
    ]


def ground_dist(p):
    return math.hypot(p[0], p[2])


# ---------------------------------------------------------------- checks

results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(f"  [{'ok' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))


def test_design_numbers():
    print("design numbers (no bench calibration yet)")
    d = df.design_calibration()
    ch, cv = d.cam.fov_deg()
    th, tv = d.tof.fov_deg()
    check(
        "camera picture is a centre crop: ~56.4 x 33.5 deg",
        abs(ch - 56.4) < 0.3 and abs(cv - 33.5) < 0.4,
        f"{ch:.1f} x {cv:.1f}",
    )
    check(
        "ToF 70 deg diagonal on its 4:3 sensor: ~58.5 x 45.6 deg",
        abs(th - 58.5) < 0.3 and abs(tv - 45.6) < 0.3,
        f"{th:.1f} x {tv:.1f}",
    )
    s = d.summary()
    check(
        "design pose: 3.8 cm apart, ToF 20 deg further down",
        abs(s["baseline_cm"] - 3.8) < 0.05 and abs(s["tof_down_deg"] - 20.0) < 0.05,
        f"{s['baseline_cm']} cm, {s['tof_down_deg']} deg",
    )


def test_geometry():
    print("each depth pixel lands where its true point is in the camera picture")
    depth, conf, pts = render(
        [box(0.2, 0.9, 0.75, 0.70, 1.3, 1.9)], wall_z=3.5, noise=0.0
    )
    proj = df.DepthProjector(TRUE)
    flat = depth.reshape(-1)
    idx = RNG.choice(
        np.flatnonzero((flat >= config.TOF_MIN_MM) & (flat <= config.TOF_MAX_MM)),
        60,
        replace=False,
    )
    worst, landed = 0.0, 0
    for i in idx:
        one = np.zeros_like(flat)
        one[i] = flat[i]
        fr = proj.project(one.reshape(depth.shape), None)
        u, v = cam_pixel(pts[i][None, :])
        if len(fr):
            landed += 1
            worst = max(worst, math.hypot(fr.u[0] - u[0], fr.v[0] - v[0]))
        elif 0 <= u[0] < 960 and 0 <= v[0] < 540:
            worst = max(worst, 999.0)  # lost a point that is in the picture
    check(
        "with distortion on both lenses, within 0.05 px",
        worst < 0.05,
        f"{landed} of 60 in the picture, worst {worst:.4f} px",
    )


def test_boxes():
    print("each detection box takes the distance of the thing in it")
    proj = df.DepthProjector(TRUE)
    person = box(0.05, 0.55, 1.70, 0.0, 1.85, 2.15)
    pole = box(-0.86, -0.74, 2.5, 0.0, 2.94, 3.06)
    depth, conf, _ = render([person, pole], wall_z=3.5)
    fr = proj.project(depth, conf)
    d = fr.box_depth(detection_box(*person))
    truth = ground_dist([0.30, 0, 1.85])
    check(
        "person 1.9 m ahead, wall 3.5 m behind: the person",
        d is not None and abs(d["distance_m"] - truth) < 0.08,
        f"{d and d['distance_m']} m (true {truth:.2f}), {d and d['points']} points",
    )
    d = fr.box_depth(detection_box(*pole))
    truth = ground_dist([-0.8, 0, 2.94])
    check(
        "pole 3 m ahead, 12 cm wide: the pole",
        d is not None and abs(d["distance_m"] - truth) < 0.1,
        f"{d and d['distance_m']} m (true {truth:.2f}), {d and d['points']} points, "
        f"bearing {d and d['bearing_deg']} deg",
    )
    near = box(-0.25, 0.25, 1.70, 0.0, 1.0, 1.3)
    depth, conf, _ = render([near], wall_z=3.5)
    fr = proj.project(depth, conf)
    bb = detection_box(*near)
    d = fr.box_depth(bb)
    truth = ground_dist([0.0, 0, 1.0])
    check(
        "person 1 m ahead, head cut off by the picture: still measured",
        bb[1] <= 0.5 and d is not None and abs(d["distance_m"] - truth) < 0.06,
        f"box top {bb[1]:.0f} px, {d and d['distance_m']} m (true {truth:.2f})",
    )
    far = box(-0.25, 0.25, 1.70, 0.0, 5.0, 5.3)
    depth, conf, _ = render([far], wall_z=8.0)
    fr = proj.project(depth, conf)
    check(
        "person 5 m ahead (past the ToF's 4 m): nothing, the box size is used",
        fr.box_depth(detection_box(*far)) is None,
    )
    sign = box(-0.3, 0.3, 1.9, 1.5, 2.5, 2.55)  # a sign at head height: in the
    # camera's picture, above the ToF's view
    depth, conf, _ = render([sign], wall_z=None)
    fr = proj.project(depth, conf)
    bb = detection_box(*sign)
    check(
        "a sign above the ToF's view: nothing",
        fr.box_depth(bb) is None,
        f"box {[round(b) for b in bb]}",
    )


def test_wrong_ways():
    print("why each pixel needs its own distance and the bench numbers")
    table = box(0.2, 0.9, 0.75, 0.70, 1.3, 1.9)
    depth, conf, pts = render([table], wall_z=3.5, noise=0.0)
    ok = np.isfinite(pts[:, 0])
    u, v = cam_pixel(pts[ok])
    inside = (u >= 0) & (u < 960) & (v >= 0) & (v < 540)
    tof = TRUE.tof
    vv, uu = np.mgrid[0 : tof.height, 0 : tof.width]
    su = (uu.ravel()[ok] + 0.5) / tof.width * 960 - 0.5  # stretched over the picture
    sv = (vv.ravel()[ok] + 0.5) / tof.height * 540 - 0.5
    stretch = np.hypot(su - u, sv - v)[inside]
    xn, yn = tof.normalised(uu.ravel()[ok].astype(float), vv.ravel()[ok].astype(float))
    dirs = np.stack([xn, yn, np.ones_like(xn)], 1) @ TRUE.R.T  # angles only: no offset
    au, av = TRUE.cam.project(dirs)
    angles = np.hypot(au - u, av - v)[inside]
    dist = np.hypot(pts[ok][:, 0], pts[ok][:, 2])[inside]
    near_k, far_k = dist < 1.6, dist > 3.0
    design = df.design_calibration()  # the design's numbers on this rig
    dp = df.DepthProjector(design)
    P = dp.rays[ok] * (depth.reshape(-1)[ok, None] / 1000.0)
    du, dv = design.cam.project(P @ design.R.T + design.t)
    design_err = np.hypot(du - u, dv - v)[inside]
    print(
        f"     stretched depth picture: median {np.median(stretch):.0f} px off (of 960)"
    )
    print(
        f"     angles only: {np.median(angles[near_k]):.1f} px off within 1.6 m, "
        f"{np.median(angles[far_k]):.1f} px past 3 m"
    )
    print(
        f"     design numbers on this rig (1 deg off, distorted lenses): median "
        f"{np.median(design_err):.1f} px, worst {np.max(design_err):.0f} px"
    )
    check("stretching is grossly wrong (> 40 px)", np.median(stretch) > 40)
    check(
        "angles only is off up close, less far away",
        np.median(angles[near_k]) > np.median(angles[far_k]) > 0,
    )
    check(
        "the design numbers are off by several px: calibrate on the bench",
        np.median(design_err) > 3,
    )


def _board_views(kind: str, n: int):
    """Checkerboard sightings (board points, ToF corners or None, camera corners or None):
    "both" where the two pictures overlap, "cam" / "tof" across one picture only."""
    grid = tc.board_points(7, 5, 40.0) / 1000.0
    grid -= grid.mean(axis=0)
    views, tries = [], 0
    while len(views) < n and tries < 20000:
        tries += 1
        Rb = (
            _rot("x", RNG.uniform(-35, 35))
            @ _rot("y", RNG.uniform(-35, 35))
            @ _rot("z", RNG.uniform(-15, 15))
        )
        if kind == "tof":  # spread over the ToF's picture
            pt0 = np.array(
                [RNG.uniform(-0.4, 0.4), RNG.uniform(-0.3, 0.3), RNG.uniform(0.6, 1.0)]
            )
            centre = TRUE.R @ pt0 + TRUE.t
        else:
            centre = np.array(
                [
                    RNG.uniform(-0.45, 0.45),
                    RNG.uniform(0.1, 0.5)
                    if kind == "both"
                    else RNG.uniform(-0.35, 0.35),
                    RNG.uniform(0.6, 1.3),
                ]
            )
        pc = grid @ Rb.T + centre  # camera frame
        pt = (pc - TRUE.t) @ TRUE.R  # ToF frame: R^T (P - t)
        if (pc[:, 2] <= 0.1).any() or (pt[:, 2] <= 0.1).any():
            continue
        cu, cv = TRUE.cam.project(pc)
        tu, tv = TRUE.tof.project(pt)
        in_cam = bool(((cu > 5) & (cu < 954) & (cv > 5) & (cv < 534)).all())
        in_tof = bool(((tu > 3) & (tu < 236) & (tv > 3) & (tv < 176)).all())
        if (
            (kind == "both" and not (in_cam and in_tof))
            or (kind == "cam" and (not in_cam or in_tof))
            or (kind == "tof" and (not in_tof or in_cam))
        ):
            continue
        c = (
            (np.stack([cu, cv], 1) + RNG.normal(0, 0.15, (len(cu), 2)))
            .astype(np.float32)
            .reshape(-1, 1, 2)
        )
        t = (
            (np.stack([tu, tv], 1) + RNG.normal(0, 0.08, (len(tu), 2)))
            .astype(np.float32)
            .reshape(-1, 1, 2)
        )
        views.append(
            (tc.board_points(7, 5, 40.0), t if in_tof else None, c if in_cam else None)
        )
    return views


def placement_error(calib: df.Calibration, truth: df.Calibration = TRUE):
    """Median and 95th-percentile px between where `calib` places a street's depth dots and where they truly are."""
    scene = [box(0.2, 0.9, 0.75, 0.70, 1.3, 1.9), box(0.05, 0.55, 1.70, 0.0, 2.3, 2.6)]
    depth, conf, pts = render(scene, wall_z=3.5, noise=0.0, calib=truth)
    ok = (depth.reshape(-1) >= config.TOF_MIN_MM) & (
        depth.reshape(-1) <= config.TOF_MAX_MM
    )
    u, v = cam_pixel(pts[ok], truth.cam)
    P = df.DepthProjector(calib).rays[ok] * (depth.reshape(-1)[ok, None] / 1000.0)
    pu, pv = calib.cam.project(P @ calib.R.T + calib.t)
    inside = (u >= 0) & (u < 960) & (v >= 0) & (v < 540)
    err = np.hypot(pu - u, pv - v)[inside]
    return float(np.median(err)), float(np.percentile(err, 95))


def test_calibration():
    print("the bench calibration gets the rig back from checkerboard corners")
    pairs_only = _board_views("both", 22)
    res0 = tc.calibrate(pairs_only, (240, 180), (960, 540))
    views = pairs_only + _board_views("cam", 15) + _board_views("tof", 10)
    res = tc.calibrate(views, (240, 180), (960, 540))
    R = np.array(res["R"])
    t = np.array(res["t_m"])
    ang = math.degrees(math.acos(min(1.0, (np.trace(R.T @ TRUE.R) - 1.0) / 2.0)))
    ang0 = math.degrees(
        math.acos(min(1.0, (np.trace(np.array(res0["R"]).T @ TRUE.R) - 1.0) / 2.0))
    )
    terr = np.linalg.norm(t - TRUE.t) * 1000.0
    # The aim alone is a poor yardstick (a little aim error is traded against the lens's
    # centre); where the dots land, below, is the one that counts.
    check(
        "22 shared views + 15 camera-only + 10 ToF-only: aim within 0.5 deg",
        ang < 0.5 and ang0 < 0.5,
        f"{ang:.3f} deg (shared views alone: {ang0:.3f} deg)",
    )
    check("position within 4 mm", terr < 4.0, f"{terr:.1f} mm (rms: {res['rms']})")
    fx_err = abs(res["camera"]["fx"] / TRUE_CAM.fx - 1.0) * 100.0
    tfx_err = abs(res["tof"]["fx"] / TRUE_TOF.fx - 1.0) * 100.0
    check(
        "focal lengths within 1 %",
        fx_err < 1.0 and tfx_err < 1.0,
        f"camera {fx_err:.2f} %, ToF {tfx_err:.2f} %",
    )
    cal = df.Calibration(
        df.Lens.from_dict(res["tof"]), df.Lens.from_dict(res["camera"]), R, t, "bench"
    )
    med, p95 = placement_error(cal)
    cal0 = df.Calibration(
        df.Lens.from_dict(res0["tof"]),
        df.Lens.from_dict(res0["camera"]),
        np.array(res0["R"]),
        np.array(res0["t_m"]),
        "bench",
    )
    med0, p950 = placement_error(cal0)
    dmed, dp95 = placement_error(df.design_calibration())
    check(
        "its dots land within 1.5 px (95 % within 3 px)",
        med < 1.5 and p95 < 3.0,
        f"median {med:.2f} px, 95 % {p95:.2f} px; shared views alone {med0:.2f} / {p950:.2f}; "
        f"design numbers {dmed:.1f} / {dp95:.1f}",
    )
    with tempfile.TemporaryDirectory() as tmp:  # the file it writes loads back
        path = os.path.join(tmp, "cal.json")
        big = dict(res)
        big["camera"] = df.Lens.from_dict(res["camera"]).resized(1920, 1080).to_dict()
        with open(path, "w") as f:
            json.dump(big, f)
        cal = df.load_calibration(path)
        check(
            "a file calibrated at 1920 x 1080 loads, scaled to the 960 x 540 picture",
            cal.source == "bench" and abs(cal.cam.fx - res["camera"]["fx"]) < 0.01,
            f"fx {cal.cam.fx:.2f} vs {res['camera']['fx']:.2f}",
        )
        with open(path, "w") as f:
            f.write("{not json")
        cal = df.load_calibration(path)
        check(
            "a broken file: the design is used, and said",
            cal.source == "design" and "unusable" in cal.note,
        )


def _board_texture(square=35, margin=35):
    """10 x 7 squares (9 x 6 inner corners), 1 px per mm, with a white margin."""
    w, h = 10 * square + 2 * margin, 7 * square + 2 * margin
    img = np.full((h, w), 255, np.uint8)
    for j in range(7):
        for i in range(10):
            if (i + j) % 2 == 0:
                img[
                    margin + j * square : margin + (j + 1) * square,
                    margin + i * square : margin + (i + 1) * square,
                ] = 20
    return img, margin + square  # texture px of the first inner corner


def _render_board(tex, first, Rb, tb, lens: df.Lens, oversample=1):
    """The board as a lens sees it (no distortion): texture px -> board mm -> lens -> pixel."""
    import cv2

    K = lens.matrix().copy()
    K[:2] *= oversample
    K[0, 2] += (oversample - 1) / 2.0
    K[1, 2] += (oversample - 1) / 2.0
    A = np.array([[1, 0, -first], [0, 1, -first], [0, 0, 1]], float)  # texture px -> mm
    Hm = K @ np.column_stack([Rb[:, 0] / 1000.0, Rb[:, 1] / 1000.0, tb]) @ A
    size = (lens.width * oversample, lens.height * oversample)
    img = cv2.warpPerspective(tex, Hm, size, flags=cv2.INTER_AREA, borderValue=128)
    if oversample > 1:
        img = cv2.resize(img, (lens.width, lens.height), interpolation=cv2.INTER_AREA)
    noise = RNG.normal(0, 3, img.shape)
    return np.clip(img + noise, 0, 255).astype(np.uint8)


def test_pictures():
    print("the whole bench procedure on rendered checkerboard pictures")
    import cv2

    nd = lambda L: df.Lens(L.fx, L.fy, L.cx, L.cy, L.width, L.height)  # noqa: E731
    truth = df.Calibration(nd(TRUE_TOF), nd(TRUE_CAM), TRUE_R, TRUE_T, "bench")
    tex, first = _board_texture()
    c = np.array([4 * 35.0, 2.5 * 35.0, 0.0]) / 1000.0  # the inner corners' middle
    outline = (
        np.array(
            [
                [-35, -35, 0],
                [10 * 35 - 35, -35, 0],
                [10 * 35 - 35, 7 * 35 - 35, 0],
                [-35, 7 * 35 - 35, 0],
            ],
            float,
        )
        - 0.0
    ) / 1000.0
    with tempfile.TemporaryDirectory() as tmp:
        n, tries = 0, 0
        while n < 20 and tries < 20000:
            tries += 1
            Rb = (
                _rot("x", RNG.uniform(-30, 30))
                @ _rot("y", RNG.uniform(-30, 30))
                @ _rot("z", RNG.uniform(-12, 12))
            )
            centre = np.array(
                [RNG.uniform(-0.3, 0.3), RNG.uniform(0.15, 0.45), RNG.uniform(0.7, 1.1)]
            )
            tb_c = centre - Rb @ c  # board -> camera frame
            Rt, tb_t = (
                truth.R.T @ Rb,
                truth.R.T @ (tb_c - truth.t),
            )  # board -> ToF frame
            pc = outline @ Rb.T + tb_c
            pt = outline @ Rt.T + tb_t
            if (pc[:, 2] < 0.2).any() or (pt[:, 2] < 0.2).any():
                continue
            cu, cv = truth.cam.project(pc)
            tu, tv = truth.tof.project(pt)
            if not (
                ((cu > 2) & (cu < 957) & (cv > 2) & (cv < 537)).all()
                and ((tu > 2) & (tu < 237) & (tv > 2) & (tv < 177)).all()
            ):
                continue
            n += 1
            stem = os.path.join(tmp, f"{n:03d}")
            cv2.imwrite(
                stem + "_cam.png", _render_board(tex, first, Rb, tb_c, truth.cam)
            )
            cv2.imwrite(
                stem + "_tof.png", _render_board(tex, first, Rt, tb_t, truth.tof, 4)
            )
        out = os.path.join(tmp, "cal.json")
        import contextlib
        import io

        log = io.StringIO()
        with contextlib.redirect_stdout(log):
            rc = tc.main(
                [
                    "solve",
                    "--pairs",
                    tmp,
                    "--board",
                    "9x6",
                    "--square-mm",
                    "35",
                    "--out",
                    out,
                ]
            )
        lines = log.getvalue().strip().splitlines()
        cal = df.load_calibration(out) if rc == 0 else None
        if cal is None or cal.source != "bench":
            check("solve on 20 rendered pairs", False, " | ".join(lines[-3:]))
            return
        med, p95 = placement_error(cal, truth)
        check(
            f"solve on {n} rendered pairs (ToF 240 x 180): dots within 1.5 px",
            med < 1.5 and p95 < 3.0,
            f"median {med:.2f} px, 95 % {p95:.2f} px; {[l for l in lines if 'usable' in l][0]}; "
            f"{[l for l in lines if l.startswith('error')][0]}",
        )


def test_wall():
    print("the wall check tells how the ToF reads its distance")
    for radial in (True, False):
        flat = box(-3, 3, 3.0, -1.0, 0.9, 0.95)
        depth, _, _ = render([flat], wall_z=None, radial=radial, noise=0.002)
        fit = tc.wall_fit(depth, TRUE_TOF)
        said = fit["radial"] < fit["axis"]
        check(
            f"read along the {'ray' if radial else 'axis'}: told so",
            said == radial,
            f"plane error ray {fit['radial']:.1f} mm, axis {fit['axis']:.1f} mm",
        )


def test_wiring():
    print("the detection pass, the distances and the guidance")
    from utils import tof_camera, simple_guidance as sg, sensor_fusion as sf

    person = box(0.05, 0.55, 1.70, 0.0, 1.85, 2.15)
    depth, conf, _ = render([person], wall_z=3.5)
    old = (df.TOF_ENABLED, df._projector, config.TOF_USE_DISTANCE, sg.TOF_USE_DISTANCE)
    try:
        df.TOF_ENABLED = True
        df._projector = df.DepthProjector(TRUE)
        t0 = 1000.0
        with tof_camera._lock:
            tof_camera._frames.clear()
            tof_camera._frames.append(tof_camera._Frame(t0, depth, conf))
        bb = detection_box(*person)
        dets = [{"class_name": "person", "confidence": 0.9, "bbox": bb}]
        n = df.annotate(dets, t0 + 0.02)
        check(
            "a picture 20 ms from a ToF frame: its box gets a depth",
            n == 1 and "depth" in dets[0],
            f"{dets[0].get('depth')}",
        )
        late = [{"class_name": "person", "confidence": 0.9, "bbox": bb}]
        check(
            "a picture 200 ms away: none (TOF_PAIR_MAX_S)",
            df.annotate(late, t0 + 0.2) == 0,
        )
        config.TOF_USE_DISTANCE = False
        sg.TOF_USE_DISTANCE = False
        info = sf.detection_distance("front", dets[0], None, t0, explain=True)
        check(
            "TOF_USE_DISTANCE off: the box size decides, the ToF is shown beside",
            info["source"] != "tof" and info.get("tof_ft") is not None,
            f"{info['source']} {info['distance_ft']} ft, ToF {info.get('tof_ft')} ft",
        )
        r_off, _ = sg._box_range(dets[0], "person", tuple(bb))
        config.TOF_USE_DISTANCE = True
        sg.TOF_USE_DISTANCE = True
        ft, src = sf.detection_distance("front", dets[0], None, t0)
        r_on, cut = sg._box_range(dets[0], "person", tuple(bb))
        check(
            "TOF_USE_DISTANCE on: the ToF's distance everywhere",
            src == "tof"
            and abs(r_on - dets[0]["depth"]["distance_m"]) < 1e-9
            and not cut,
            f"dashboard {ft} ft; guidance {r_on:.2f} m (box size said {r_off:.2f} m)",
        )
    finally:
        df.TOF_ENABLED, df._projector, config.TOF_USE_DISTANCE, sg.TOF_USE_DISTANCE = (
            old
        )
        with tof_camera._lock:
            tof_camera._frames.clear()


def test_cost():
    print("cost on this machine")
    person = box(0.05, 0.55, 1.70, 0.0, 1.85, 2.15)
    depth, conf, _ = render([person], wall_z=3.5)
    proj = df.DepthProjector(TRUE)
    proj.project(depth, conf)
    t = time.perf_counter()
    for _ in range(30):
        fr = proj.project(depth, conf)
    per = (time.perf_counter() - t) / 30 * 1000.0
    bb = detection_box(*person)
    t = time.perf_counter()
    for _ in range(200):
        fr.box_depth(bb)
    per_box = (time.perf_counter() - t) / 200 * 1000.0
    print(f"     placing one 240 x 180 frame: {per:.1f} ms; one box: {per_box:.2f} ms")
    check(
        "under 25 ms a frame (only frames a detection pass uses are placed)", per < 25.0
    )


def main() -> int:
    for test in (
        test_design_numbers,
        test_geometry,
        test_boxes,
        test_wrong_ways,
        test_calibration,
        test_pictures,
        test_wall,
        test_wiring,
        test_cost,
    ):
        test()
    bad = results.count(False)
    print(f"\n{len(results) - bad}/{len(results)} passed")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
