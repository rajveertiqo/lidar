#!/usr/bin/env python3
"""Ground hazards (utils/ground_hazards.py) on ray-traced scenes, without the sensor
(2026-11-06).

Each scene is boxes in the level frame (x right, y down, z ahead, the front camera's lens at
the origin, TOF_MOUNT_HEIGHT_M above the floor): stairs up and down, kerbs, drops, a hole,
poles, a low box, an overhang, a table, walls, a person, ramps. It is rendered through the
ToF as designed (FUSION_*), with the body's sway, range noise and dropped pixels, and through
the front camera as flat-shaded faces (tops light, fronts dark, each box its own shade) for
the camera's confirmation.

    venv/bin/python -m tools.ground_hazard_sim            every scene, what it finds
    venv/bin/python -m tools.ground_hazard_sim stairs_down
"""

import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import config  # noqa: E402
from utils import depth_fusion as df  # noqa: E402
from utils import ground_hazards as gh  # noqa: E402

H = config.TOF_MOUNT_HEIGHT_M
CAL = df.design_calibration()
LEVEL = df.pitch_down(config.FUSION_CAMERA_DOWN_DEG)
Box = Tuple[np.ndarray, np.ndarray]


def slab(x0, x1, top, z0, z1, bottom=None) -> Box:
    """A box from `bottom` to `top` above the floor (m), x0..x1 across, z0..z1 ahead."""
    bottom = top - 0.3 if bottom is None else bottom
    return (np.array([x0, H - top, z0], float), np.array([x1, H - bottom, z1], float))


def floor(z0=-1.0, z1=14.0, top=0.0) -> Box:
    return slab(-6.0, 6.0, top, z0, z1)


# ---------------------------------------------------------------- the scenes


def flat():
    return [floor()]


def stairs_up(z0=2.0, rise=0.17, run=0.28, n=7):
    return [floor()] + [
        slab(-1.6, 1.6, k * rise, z0 + (k - 1) * run, z0 + n * run + 4.0, bottom=0.0)
        for k in range(1, n + 1)
    ]


def stairs_down(z0=2.0, rise=0.17, run=0.28, n=8):
    out = [floor(z1=z0)]
    for k in range(1, n + 1):
        z1 = z0 + k * run if k < n else z0 + k * run + 5.0
        out.append(slab(-1.6, 1.6, -k * rise, z0 + (k - 1) * run, z1))
    out.append(slab(-6.0, -1.6, 0.0, z0, 14.0))  # the floor either side of the flight
    out.append(slab(1.6, 6.0, 0.0, z0, 14.0))
    return out


def kerb_up(z0=2.4, rise=0.13):
    return [floor(), slab(-6.0, 6.0, rise, z0, 14.0, bottom=0.0)]


def kerb_down(z0=2.2, drop=0.15):
    return [floor(z1=z0), slab(-6.0, 6.0, -drop, z0, 14.0)]


def drop(z0=1.5, depth=1.0):
    return [floor(z1=z0), slab(-6.0, 6.0, -depth, z0, 14.0)]


def deep_drop(z0=2.5, depth=6.0):
    """A platform's edge: the ground below is past the ToF's 4 m."""
    return [floor(z1=z0), slab(-6.0, 6.0, -depth, z0, 30.0)]


def hole(z0=2.0, length=0.5, depth=0.2):
    return [floor(z1=z0), slab(-6.0, 6.0, -depth, z0, z0 + length), floor(z0=z0 + length)]


def pole(z=2.5, x=0.0, width=0.10):
    return [floor(), slab(x - width / 2, x + width / 2, 3.0, z, z + width, bottom=0.0)]


def side_pole():
    return pole(z=2.2, x=1.1)


def low_box(z=2.0, h=0.25):
    return [floor(), slab(-0.25, 0.25, h, z, z + 0.4, bottom=0.0)]


def overhang(z=1.8, low=0.95, top=1.10):
    """A branch, or a sign or shelf: nothing under it but air."""
    return [floor(), slab(-0.6, 0.6, top, z, z + 0.6, bottom=low)]


def table(z=1.9):
    top = slab(-0.6, 0.6, 0.75, z, z + 0.8, bottom=0.71)
    legs = [
        slab(lx - 0.025, lx + 0.025, 0.71, lz, lz + 0.05, bottom=0.0)
        for lx in (-0.55, 0.55)
        for lz in (z + 0.05, z + 0.70)
    ]
    return [floor(), top] + legs


def wall(z=2.6):
    return [floor(), slab(-4.0, 4.0, 3.0, z, z + 0.2, bottom=0.0)]


def person(z=2.2):
    return [floor(), slab(-0.22, 0.22, 1.75, z, z + 0.25, bottom=0.0)]


def ramp(z0=1.6, deg=5.0, up=True):
    """1 cm steps every few cm: a ramp to the ToF."""
    s = math.tan(math.radians(deg))
    run = 0.01 / s
    out = [floor(z1=z0 if not up else 14.0)]
    k = 1
    z = z0
    while z < 6.0:
        h = k * 0.01 * (1 if up else -1)
        out.append(slab(-6.0, 6.0, h, z, 14.0 if up else z + run, bottom=0.0 if up else None))
        z += run
        k += 1
    return out


def corridor():
    """Walls either side 0.9 m away: nothing on the way."""
    return [floor(), slab(-4.0, -0.9, 3.0, -1.0, 14.0, bottom=0.0), slab(0.9, 4.0, 3.0, -1.0, 14.0, bottom=0.0)]


SCENES = {
    "flat": (flat, None),
    "corridor": (corridor, None),
    "ramp_up_5": (lambda: ramp(deg=5.0), None),
    "ramp_down_5": (lambda: ramp(deg=5.0, up=False), None),
    "stairs_up": (stairs_up, ("stairs_up", 2.0)),
    "stairs_down": (stairs_down, ("stairs_down", 2.0)),
    "kerb_up": (kerb_up, ("step_up", 2.4)),
    "kerb_down": (kerb_down, ("step_down", 2.2)),
    "drop": (drop, ("drop", 1.5)),
    "deep_drop": (deep_drop, ("drop", 2.5)),
    "hole": (hole, ("hole", 2.0)),
    "pole": (pole, ("pole", 2.5)),
    "side_pole": (side_pole, None),
    "low_box": (low_box, ("low_obstacle", 2.0)),
    "overhang": (overhang, ("overhang", 1.8)),
    "table": (table, ("obstacle", 1.9)),
    "wall": (wall, ("obstacle", 2.6)),
    "person": (person, ("obstacle", 2.2)),
}


# ---------------------------------------------------------------- rendering


def _rot(axis: str, deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return {
        "x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
        "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]),
    }[axis]


def _hit(O, D, boxes: List[Box]):
    """Each ray's distance to the nearest box, which box, and which face axis (0 x, 1 y, 2 z)."""
    s = np.full(len(D), np.inf)
    which = np.full(len(D), -1)
    face = np.full(len(D), -1)
    with np.errstate(divide="ignore", invalid="ignore"):
        for b, (lo, hi) in enumerate(boxes):
            t1, t2 = (lo - O) / D, (hi - O) / D
            near = np.minimum(t1, t2)
            tmin = np.nanmax(near, axis=1)
            tmax = np.nanmin(np.maximum(t1, t2), axis=1)
            hit = (tmax >= np.maximum(tmin, 0)) & (tmin > 0) & (tmin < s)
            s = np.where(hit, tmin, s)
            which = np.where(hit, b, which)
            face = np.where(hit, np.nanargmax(near, axis=1), face)
    return s, which, face


def sway(pitch_deg=0.0, roll_deg=0.0) -> np.ndarray:
    """The body's lean: the rig turned in the level frame (pitch + = looking further down)."""
    return _rot("x", -pitch_deg) @ _rot("z", roll_deg)


def render_tof(boxes, pitch=0.0, roll=0.0, noise=0.01, dropout=0.02, rng=None, calib=CAL):
    """(depth mm, confidence) as the ToF would read the scene."""
    rng = rng or np.random.default_rng(0)
    tof = calib.tof
    vv, uu = np.mgrid[0 : tof.height, 0 : tof.width].astype(np.float64)
    xn, yn = tof.normalised(uu.ravel(), vv.ravel())
    unit = np.stack([xn, yn, np.ones_like(xn)], axis=1)
    unit /= np.linalg.norm(unit, axis=1, keepdims=True)
    S = sway(pitch, roll)
    D = unit @ (S @ LEVEL @ calib.R).T
    O = S @ LEVEL @ calib.t
    s, _, _ = _hit(O, D, boxes)
    reading = s if config.TOF_DEPTH_IS_RADIAL else s * unit[:, 2]
    depth = reading * 1000.0 * (1.0 + noise * rng.standard_normal(len(s)))
    ok = np.isfinite(depth) & (depth <= config.TOF_MAX_MM) & (rng.random(len(s)) >= dropout)
    depth = np.where(ok, depth, 0.0)
    conf = np.where(ok, 200.0, 0.0)
    return depth.reshape(tof.height, tof.width), conf.reshape(tof.height, tof.width)


def render_picture(boxes, pitch=0.0, roll=0.0, calib=CAL, rng=None) -> np.ndarray:
    """The front camera's picture of the scene (BGR), flat-shaded: each box its own shade,
    tops lighter than fronts, the sky light."""
    rng = rng or np.random.default_rng(1)
    cam = calib.cam
    vv, uu = np.mgrid[0 : cam.height, 0 : cam.width].astype(np.float64)
    xn, yn = cam.normalised(uu.ravel(), vv.ravel())
    unit = np.stack([xn, yn, np.ones_like(xn)], axis=1)
    unit /= np.linalg.norm(unit, axis=1, keepdims=True)
    D = unit @ (sway(pitch, roll) @ LEVEL).T
    s, which, face = _hit(np.zeros(3), D, boxes)
    # Each box its own shade (the golden-ratio sequence: no two alike nearby — a repeat
    # every 4 gave a floor and the landing below a flight the same shade, no edge).
    albedo = np.array([1.0 - 0.3 * ((b * 0.618034) % 1.0) for b in range(len(boxes))] + [1.0])
    shade = np.select([face == 1, face == 2, face == 0], [175.0, 95.0, 130.0], 215.0)
    img = shade * albedo[which] + rng.normal(0.0, 3.0, len(s))
    img = np.clip(img, 0, 255).astype(np.uint8).reshape(cam.height, cam.width)
    return np.dstack([img, img, img])


def camera_inputs(picture, t, view=None, detections=()):
    return {"picture": picture, "t": t, "view": view or {}, "detections": list(detections), "simulated": False}


def run_scene(name, frames=6, pitch=0.0, roll=0.0, seed=0, with_camera=True, picture=None):
    """The snapshot after `frames` frames of the scene (the same scene, fresh noise each)."""
    make, _ = SCENES[name]
    boxes = make()
    rng = np.random.default_rng(seed)
    hz = gh.GroundHazards(df.DepthProjector(CAL))
    pic = picture
    if with_camera and pic is None:
        pic = render_picture(boxes, pitch, roll)
    snap = None
    for i in range(frames):
        depth, conf = render_tof(boxes, pitch, roll, rng=rng)
        t = 1000.0 + i * 0.1
        cam = (lambda t=t: camera_inputs(pic, t)) if pic is not None else None
        snap = hz.step(depth, conf, t, cam)
    return snap


def main(argv=None) -> int:
    names = (argv if argv is not None else sys.argv[1:]) or list(SCENES)
    bad = 0
    for name in names:
        t0 = time.perf_counter()
        snap = run_scene(name)
        ms = (time.perf_counter() - t0) * 1000.0
        want = SCENES[name][1]
        top = snap.get("top")
        found = [(h["kind"], h["ahead_m"], h["camera"].get("status"), h["speak"]) for h in snap["hazards"]]
        ok = (top is None) if want is None else (top is not None and top["kind"] == want[0] and abs(top["ahead_m"] - want[1]) <= 0.25)
        bad += not ok
        plane = snap.get("plane") or {}
        print(
            f"[{'ok' if ok else 'FAIL'}] {name:12s} want {want}  top "
            f"{(top['kind'], top['ahead_m']) if top else None}  found {found}  "
            f"ground {plane.get('height_m')} m  ({ms / 6:.0f} ms a frame incl. render)"
        )
    print(f"\n{len(names) - bad}/{len(names)} as expected")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
