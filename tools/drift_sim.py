#!/usr/bin/env python3
"""Drift simulator — exercise and tune the heading/drift layer WITHOUT walking."""

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from utils import drift_guidance as dg  # noqa: E402
from utils import heading_tracker as ht  # noqa: E402


def ray_cast(
    x,
    y,
    heading_deg,
    half_width_mm,
    length_mm=25000.0,
    n_beams=1080,
    noise_mm=8.0,
    max_range_mm=12000.0,
    back_mm=-3000.0,
):
    """One LiDAR scan of a straight corridor, from a pose inside it."""
    points = []
    for i in range(n_beams):
        bearing = i * 360.0 / n_beams
        world = math.radians(heading_deg + bearing)
        dx, dy = math.cos(world), math.sin(world)
        hits = []
        for wall_y in (half_width_mm, -half_width_mm):
            if abs(dy) > 1e-9:
                t = (wall_y - y) / dy
                if t > 1.0 and back_mm <= x + dx * t <= length_mm:
                    hits.append(t)
        for wall_x in (length_mm, back_mm):
            if abs(dx) > 1e-9:
                t = (wall_x - x) / dx
                if t > 1.0 and -half_width_mm <= y + dy * t <= half_width_mm:
                    hits.append(t)
        if hits:
            t = min(hits)
            if t < max_range_mm:
                points.append((bearing, t + np.random.uniform(-noise_mm, noise_mm)))
    return points


class SimGyro:
    """A gyro that sees the simulated truth, plus a zero-rate offset."""

    def __init__(self, bias_dps, trusted_from_start=True):
        self.bias_dps = float(bias_dps)
        self.truth = [(1000.0, 0.0)]  # (t, true cumulative heading)
        self._trusted = trusted_from_start
        self._correction = 0.0
        self.corrections = 0

    def observe(self, clock, true_heading_deg):
        """Ground truth from the simulator, before any sensor sees it."""
        self.truth.append((clock, true_heading_deg))
        if len(self.truth) > 4000:
            del self.truth[:1000]

    def _at(self, t):
        prev = self.truth[0]
        for entry in self.truth:
            if entry[0] >= t:
                if entry[0] == prev[0]:
                    return entry[1]
                f = (t - prev[0]) / (entry[0] - prev[0])
                return prev[1] + (entry[1] - prev[1]) * f
            prev = entry
        return prev[1]

    def healthy(self, now=None):
        return True

    def bias_trusted(self):
        return self._trusted

    def yaw_between(self, t0, t1):
        if t1 <= t0:
            return None
        turned = self._at(t1) - self._at(t0)
        offset = (self.bias_dps - self._correction) * (t1 - t0)
        return turned + offset

    def correct_bias(self, error_dps, dt):
        self.corrections += 1
        if abs(error_dps) > 6.0:
            return
        self._correction -= 0.02 * error_dps * dt

    def note_still(self, still, now):
        pass

    def state(self):
        return {
            "healthy": True,
            "walking": None,
            "cadence_spm": None,
            "bias_dps": round(self._correction, 3),
            "tilt_deg": 0.0,
            "tilt_warn": False,
            "dropouts": 0,
            "bias_trusted": self._trusted,
            "bias_trust_reason": "simulated",
        }


def install_gyro(bias_dps):
    """Fit a SimGyro to the heading tracker, and let it steer."""
    gyro = SimGyro(bias_dps)
    ht.get_gyro = lambda: gyro
    ht.GYRO_MOUNTING_CONFIRMED = True
    return gyro


def run(args):
    np.random.seed(args.seed)
    half = args.width * 304.8 / 2.0
    back = args.back * 1000.0
    dt = 1.0 / args.rate

    tracker = ht.HeadingTracker()
    dg.reset_state()

    ht.get_tracker = lambda: tracker

    gyro = install_gyro(args.gyro) if args.gyro is not None else None

    x = y = 0.0
    heading = 0.0
    clock = 1000.0
    spoken = []
    contact_at = None

    where = "Open space" if args.open else "Corridor"
    print(
        f"{where} {args.width:.1f} ft wide, user at {args.speed:.2f} m/s, "
        f"LiDAR {args.rate:.0f} Hz."
    )
    if gyro is not None:
        print(f"Gyro fitted, zero-rate offset {args.gyro:+.2f} deg/s.")
    else:
        print("No gyro — heading from scan matching alone.")
    if args.veer:
        side = "LEFT" if args.veer > 0 else "RIGHT"
        print(
            f"From t={args.veer_start:.1f}s they veer {side} at "
            f"{abs(args.veer):.1f} deg/s and never notice it."
        )
    else:
        print("CONTROL RUN: no systematic veer, only sway. Expect silence.")
    print()
    print(
        f"{'t':>5} {'y(ft)':>6} {'true':>7} {'est':>7} {'m/s':>5} {'src':>8} "
        f"{'yaw':>8} {'deg/s':>6} {'TTC':>5} {'level':>9}  instruction"
    )
    print("-" * 117)

    steps = int(args.duration * args.rate)
    for step in range(steps):
        if gyro is not None:
            gyro.observe(clock, heading)
        scan = ray_cast(x, y, heading, half, back_mm=back)
        state = tracker.update(scan, clock)
        lidar_state = {
            "points": {
                "angles": [round(a, 1) for a, d in scan],
                "distances": [int(d) for a, d in scan],
            },
            "sectors": {},
            "cones": {},
        }
        result = dg.analyze(
            lidar_state=lidar_state,
            radar_state={"sectors": {}},
            camera_state={},
            heading=state,
            now=clock,
        )

        elapsed = clock - 1000.0
        if result["speak"]:
            spoken.append((elapsed, y, result["instruction"]))
        if result["speak"] or step % max(1, int(args.rate)) == 0:
            mark = "  <<< SPOKEN" if result["speak"] else ""
            print(
                f"{elapsed:5.1f} {y / 304.8:+6.2f} {heading:+7.1f} "
                f"{state['heading_deg']:+7.1f} {state['speed_mps']:5.2f} "
                f"{state['speed_source']:>8} {state['yaw_source']:>8} "
                f"{state['yaw_rate_dps']:+6.2f} "
                f"{str(result['ttc_s']):>5} {result['level']:>9}  "
                f"{result['instruction'][:44]}{mark}"
            )

        if elapsed >= args.veer_start:
            heading -= args.veer * dt
        intended = -args.veer * max(0.0, elapsed - args.veer_start)
        heading += np.random.uniform(-args.sway, args.sway) - 0.25 * (
            heading - intended
        )
        travel = args.speed * 1000.0 * dt
        x += travel * math.cos(math.radians(heading))
        y += travel * math.sin(math.radians(heading))
        clock += dt

        if abs(y) >= half - dg.DRIFT_BODY_RADIUS_MM:
            contact_at = (elapsed, y)
            break

    print()
    print("=" * 108)
    if spoken:
        print("Spoken instructions:")
        for when, at_y, message in spoken:
            print(f"  t={when:5.1f}s  y={at_y / 304.8:+.2f} ft   {message}")
        first_t, first_y, _ = spoken[0]
        wall = -half if args.veer > 0 else half
        print()
        print(
            f"  FIRST WARNING  t={first_t:.1f}s, "
            f"{abs(wall - first_y) / 304.8:.2f} ft from the wall"
        )
        print(
            f"  SPOKEN COUNT   {len(spoken)} over {clock - 1000.0:.1f}s "
            f"(one every {(clock - 1000.0) / len(spoken):.1f}s)"
        )
    else:
        print("Spoken instructions: NONE")

    if contact_at:
        print(
            f"  WALL CONTACT   t={contact_at[0]:.1f}s at "
            f"y={contact_at[1] / 304.8:+.2f} ft"
        )
        if spoken:
            print(
                f"  WARNING LEAD   {contact_at[0] - spoken[0][0]:.1f}s before contact"
            )
    else:
        print(f"  No wall contact in {args.duration:.0f}s.")


def accuracy(args):
    """Odometry error against known motion — the layer everything else rests on."""
    np.random.seed(args.seed)
    half = args.width * 304.8 / 2.0
    print("Scan-matching accuracy against known ground truth.\n")

    back = args.back * 1000.0
    print("  ROTATION (turn in place)")
    for truth in (-12.0, -5.0, -1.0, 0.0, 0.5, 2.0, 8.0):
        a = ht.build_profile(ray_cast(0, 0, 0, half, back_mm=back))
        b = ht.build_profile(ray_cast(0, 0, truth, half, back_mm=back))
        est = ht.estimate_motion(a, b, ht.estimate_yaw(a, b)["yaw_deg"])
        print(
            f"    true {truth:+6.1f} deg -> est {est['yaw_deg']:+6.2f} deg  "
            f"(error {est['yaw_deg'] - truth:+5.2f})"
        )

    print()
    print("  TRANSLATION (one scan interval of walking)")
    for fwd, right in ((100.0, 0.0), (60.0, 0.0), (0.0, 50.0), (90.0, 25.0)):
        a = ht.build_profile(ray_cast(0, 0, 0, half, back_mm=back))
        b = ht.build_profile(ray_cast(fwd, right, 0, half, back_mm=back))
        est = ht.estimate_motion(a, b, 0.0)
        print(
            f"    true fwd {fwd:5.0f} right {right:5.0f} mm -> "
            f"est fwd {est['dx_mm']:6.1f} right {est['dy_mm']:6.1f} mm  "
            f"(quality {est['quality']:.2f})"
        )


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "--veer",
        type=float,
        default=3.0,
        help="systematic turn rate in deg/s, + = left (default 3)",
    )
    p.add_argument(
        "--veer-start",
        type=float,
        default=1.0,
        help="seconds before the veer begins (default 1)",
    )
    p.add_argument(
        "--sway",
        type=float,
        default=0.15,
        help="random heading noise per scan, deg (default 0.15)",
    )
    p.add_argument("--speed", type=float, default=1.25, help="m/s (default 1.25)")
    p.add_argument(
        "--width",
        type=float,
        default=10.0,
        help="corridor width in FEET (default 10.0)",
    )
    p.add_argument("--rate", type=float, default=15.0, help="LiDAR Hz (default 15)")
    p.add_argument("--duration", type=float, default=20.0, help="seconds (default 20)")
    p.add_argument("--seed", type=int, default=11)
    p.add_argument(
        "--accuracy",
        action="store_true",
        help="report odometry error instead of running a walk",
    )
    p.add_argument(
        "--back",
        type=float,
        default=-3.0,
        help="wall behind the user, metres (default -3)",
    )
    p.add_argument(
        "--gyro",
        type=float,
        default=None,
        metavar="DPS",
        help="fit a gyro with this zero-rate offset in deg/s "
        "(omit for no gyro; 0 for a perfect one)",
    )
    p.add_argument(
        "--open",
        action="store_true",
        help="an open plaza instead of a corridor: every wall is "
        "pushed beyond the scan matcher's range, which is the "
        "case the gyro exists for",
    )
    args = p.parse_args()

    if args.open:
        if "--width" not in sys.argv:
            args.width = 60.0
        if "--back" not in sys.argv:
            args.back = -25.0

    if args.accuracy:
        accuracy(args)
    else:
        run(args)


if __name__ == "__main__":
    main()
