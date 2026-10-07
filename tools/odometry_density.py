#!/usr/bin/env python3
"""Scan-match odometry against known motion, at the density the rig delivers."""

import argparse
import math
import os
import random
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

SCENES = {
    # side walls only in range: forward travel is unobservable at any angle
    "parallel": dict(hw=2500.0, length=60000.0, back=-60000.0),
    # the same corridor with a wall 3 m behind: forward travel is observable
    "backwall": dict(hw=2500.0, length=16000.0, back=-3000.0),
    # walls every way
    "room": dict(hw=2500.0, length=4000.0, back=-4000.0),
}


def scan(x, y, heading_deg, scene, n_beams, rng=None):
    """One revolution of `scene` from a pose, as [(bearing_deg, range_mm)]."""
    from drift_sim import ray_cast
    g = SCENES[scene]
    phase = rng.uniform(0.0, 360.0 / n_beams) if rng is not None else 0.0
    pts = ray_cast(x, y, heading_deg + phase, g["hw"], length_mm=g["length"],
                   back_mm=g["back"], n_beams=n_beams)
    if not phase:
        return pts
    return [((b + phase) % 360.0, r) for b, r in pts]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bin-deg", type=float, default=None,
                    help="HEADING_BIN_DEG to evaluate (default: config's)")
    ap.add_argument("--beams", default="1080,250,164",
                    help="points per revolution to test (comma-separated)")
    ap.add_argument("--seconds", type=float, default=2.0)
    ap.add_argument("--no-phase-jitter", action="store_true",
                    help="sample the same bearings every revolution (the "
                         "pre-2026-09-29 behaviour, which aliases yaw)")
    ap.add_argument("--seed", type=int, default=1,
                    help="seed for the per-revolution bearing phase")
    args = ap.parse_args()

    import config
    if args.bin_deg is not None:
        scale = config.HEADING_BIN_DEG / args.bin_deg
        config.HEADING_MATCH_MIN_BINS = max(1, int(round(config.HEADING_MATCH_MIN_BINS * scale)))
        config.HEADING_SENS_SMOOTH_BINS = max(1, int(round(config.HEADING_SENS_SMOOTH_BINS * scale)) | 1)
        config.HEADING_BIN_DEG = args.bin_deg
    import numpy as np
    from unittest import mock
    from utils import heading_tracker as ht

    def run(scene, theta, beams, speed, yaw_dps=0.0, hz=15.0):
        np.random.seed(1)
        random.seed(1)
        rng = None if args.no_phase_jitter else random.Random(args.seed)
        with mock.patch.object(ht.HeadingTracker, "_gyro_brief", return_value=None):
            tr = ht.HeadingTracker()
            th = math.radians(theta)
            x = y = 0.0
            t, hd = 1000.0, float(theta)
            st = None
            for _ in range(int(args.seconds * hz)):
                st = tr.update(scan(x, y, hd, scene, beams, rng), t)
                t += 1.0 / hz
                x += speed * 1000.0 / hz * math.cos(th)
                y += speed * 1000.0 / hz * math.sin(th)
                hd += yaw_dps / hz
        return st

    print(f"HEADING_BIN_DEG={config.HEADING_BIN_DEG}  min bins={config.HEADING_MATCH_MIN_BINS}"
          f"  smoothing={config.HEADING_SENS_SMOOTH_BINS}  ({args.seconds:g} s walks, 15 Hz, "
          f"bearing phase {'LOCKED' if args.no_phase_jitter else f'random per revolution, seed {args.seed}'})")
    print(f"{'scene':9} {'beams':>5} {'deg':>3} {'true':>5}  {'pace':>6} {'source':9} "
          f"{'still':>5} {'obs':>5} {'fwd':>6}")
    bad = 0
    for scene in SCENES:
        for beams in (int(b) for b in args.beams.split(",")):
            for theta in (0, 12):
                for true in (1.2, 0.0):
                    st = run(scene, theta, beams, true)
                    pace = st["speed_measured_mps"] if st["moving_confirmed"] else None
                    wrong_still = bool(st["confidently_still"]) and true > 0
                    bad += wrong_still
                    print(f"{scene:9} {beams:5d} {theta:3d} {true:5.1f}  "
                          f"{'-' if pace is None else f'{pace:.2f}':>6} {st['speed_source']:9} "
                          f"{str(st['confidently_still']):>5} {str(st['speed_observable']):>5} "
                          f"{st['forward_sensitivity']:6.3f}"
                          + ("   <-- WALKING REPORTED STILL" if wrong_still else ""))
    for beams in (int(b) for b in args.beams.split(",")):
        st = run("room", 0, beams, 0.0, yaw_dps=20.0)
        rest = run("room", 0, beams, 0.0, yaw_dps=0.0)
        print(f"yaw, {beams} beams: turning 20.0 deg/s on the spot reads {st['yaw_rate_dps']} deg/s "
              f"(quality {st['yaw_quality']}); standing still, not turning, reads "
              f"{rest['yaw_rate_dps']} deg/s")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
