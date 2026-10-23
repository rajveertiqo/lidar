#!/usr/bin/env python3
"""Can the scan band actually anchor camera depth?"""

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

VIEWS = {"front": 0.0, "right": 90.0, "back": 180.0, "left": 270.0}

DEFAULT_HFOV_DEG = 70.0

DEFAULT_PRIOR_REL_NOISE = 0.10

DEFAULT_TOLERANCE = 0.10

MIN_ANCHORS = 12

MIN_Z_M, MAX_Z_M = 0.25, 8.0


def anchors_for_view(angles, dists_mm, centre_deg, hfov_deg):
    """Optical-axis Z (metres) for the returns inside one camera's FOV."""
    half = hfov_deg / 2.0
    out = []
    for a, d in zip(angles, dists_mm):
        off = abs((a - centre_deg + 180.0) % 360.0 - 180.0)
        if off > half:
            continue
        z = (d / 1000.0) * math.cos(math.radians(off))
        if MIN_Z_M <= z <= MAX_Z_M:
            out.append(z)
    return out


def identifiability(zs, rel_noise, tolerance):
    """Would a scale+offset fit on these anchors have been identifiable?"""
    n = len(zs)
    if n < MIN_ANCHORS:
        return {
            "n": n,
            "cv": 0.0,
            "scale_err": float("inf"),
            "ok": False,
            "why": "too few anchors",
        }

    u = [1.0 / z for z in zs]
    mean_u = statistics.fmean(u)
    std_u = statistics.stdev(u)
    cv = (std_u / mean_u) if mean_u > 0 else 0.0

    if cv <= 1e-9:
        return {
            "n": n,
            "cv": cv,
            "scale_err": float("inf"),
            "ok": False,
            "why": "no depth diversity",
        }

    err = rel_noise / (math.sqrt(n) * cv)
    return {
        "n": n,
        "cv": cv,
        "z_span": max(zs) - min(zs),
        "scale_err": err,
        "ok": err <= tolerance,
        "why": "" if err <= tolerance else "insufficient depth diversity",
    }


def analyse(scans, hfov, rel_noise, tolerance):
    """Per-view identifiable-fit rate over a sequence of scans."""
    per_view = {v: [] for v in VIEWS}
    for sc in scans:
        angles = sc["angles"]
        dists = sc["distances"]
        for view, centre in VIEWS.items():
            zs = anchors_for_view(angles, dists, centre, hfov)
            per_view[view].append(identifiability(zs, rel_noise, tolerance))
    return per_view


def _med(xs, default=0.0):
    return statistics.median(xs) if xs else default


def report(per_view, hfov, rel_noise, tolerance, n_scans):
    lines = [
        f"Anchor identifiability — {n_scans} scans, HFOV {hfov:.0f}° "
        f"(UNCALIBRATED), prior noise {rel_noise:.0%}, tolerance {tolerance:.0%}",
        "",
        f"  {'view':<8}{'anchors/scan':>14}{'Z span':>10}{'CV(1/Z)':>10}"
        f"{'pred scale err':>16}{'identifiable':>14}",
        "  " + "-" * 72,
    ]

    all_rows = []
    for view, rows in per_view.items():
        all_rows += rows
        ns = [r["n"] for r in rows]
        okd = [r for r in rows if r["ok"]]
        usable = [r for r in rows if r["n"] >= MIN_ANCHORS]
        cvs = [r["cv"] for r in usable]
        spans = [r.get("z_span", 0.0) for r in usable]
        errs = [r["scale_err"] for r in usable if math.isfinite(r["scale_err"])]
        rate = len(okd) / len(rows) if rows else 0.0

        lines.append(
            f"  {view:<8}{_med(ns):>8.0f} ({min(ns) if ns else 0:>3.0f})"
            f"{_med(spans):>9.1f}m{_med(cvs):>10.3f}"
            f"{_med(errs, float('nan')):>15.0%}{rate:>13.0%}"
        )

    total = len(all_rows)
    ok = sum(1 for r in all_rows if r["ok"])
    few = sum(1 for r in all_rows if r["why"] == "too few anchors")
    flat = sum(
        1
        for r in all_rows
        if r["why"] in ("no depth diversity", "insufficient depth diversity")
    )

    lines += [
        "  " + "-" * 72,
        f"  OVERALL: {ok}/{total} ({ok / total:.0%}) of (scan, view) pairs support "
        f"an identifiable scale+offset fit."
        if total
        else "  no data",
        "",
        "  Of the failures:",
        f"    too few anchors in view        {few:>6}  ({few / total:.0%})"
        if total
        else "",
        f"    insufficient depth diversity   {flat:>6}  ({flat / total:.0%})"
        if total
        else "",
    ]
    return "\n".join(l for l in lines if l is not None)


def verdict(per_view, tolerance):
    """What the number means for E2, said plainly."""
    all_rows = [r for rows in per_view.values() for r in rows]
    if not all_rows:
        return ""
    rate = sum(1 for r in all_rows if r["ok"]) / len(all_rows)
    flat = sum(
        1
        for r in all_rows
        if r["why"] in ("no depth diversity", "insufficient depth diversity")
    )
    flat_share = flat / len(all_rows)

    out = ["", "READ THIS AS:"]
    if rate >= 0.6:
        out.append(
            f"  {rate:.0%} — the scan band usually identifies a fit. E2 is "
            f"worth running as specified; report metrics on the "
            f"identifiable subset AND the rest, separately."
        )
    elif rate >= 0.25:
        out.append(
            f"  {rate:.0%} — E2 works on a minority of frames. As specified "
            f"it will help sometimes and hurt otherwise, which is what "
            f"page 8 already showed. Do NOT average over both; the "
            f"refusal path has to be a first-class output, and the "
            f"headline metric has to be conditioned on it."
        )
    else:
        out.append(
            f"  {rate:.0%} — the scan band rarely identifies a fit on this "
            f"rig. E2 as specified is not the main line. Either constrain "
            f"the offset (fit SCALE ONLY, b fixed, which needs no "
            f"diversity) or go to E5, where the network learns the "
            f"completion instead of a 2-parameter fit recovering it."
        )

    if flat_share > 0.4:
        out.append("")
        out.append(
            f"  {flat_share:.0%} of all pairs fail on DEPTH DIVERSITY, not "
            f"anchor count. More LiDAR points would not help. What helps "
            f"is anchors at genuinely different depths in one view — "
            f"which a planar scanner facing a wall cannot produce, and "
            f"which accumulating the band ACROSS MOTION can "
            f"(utils/heading_tracker already supplies the pose)."
        )
    return "\n".join(out)


def collect_live(host, port, n_scans, timeout):
    """Pull raw scans off /lidar/ws. Needs the server running."""
    import asyncio
    import websockets

    async def _run():
        url = f"ws://{host}:{port}/lidar/ws"
        scans, t0 = [], time.time()
        async with websockets.connect(url, open_timeout=10) as ws:
            while len(scans) < n_scans and (time.time() - t0) < timeout:
                msg = await asyncio.wait_for(ws.recv(), timeout=5)
                pts = json.loads(msg).get("points") or {}
                if pts.get("angles") and pts.get("distances"):
                    scans.append(
                        {"angles": pts["angles"], "distances": pts["distances"]}
                    )
                    if len(scans) % 25 == 0:
                        print(f"  ... {len(scans)}/{n_scans} scans", file=sys.stderr)
        return scans

    return asyncio.run(_run())


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--live", action="store_true", help="capture from the server")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--scans", type=int, default=200)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--save", metavar="FILE", help="write captured scans for replay")
    ap.add_argument("--replay", metavar="FILE", help="analyse a saved capture")
    ap.add_argument("--fov", type=float, default=DEFAULT_HFOV_DEG)
    ap.add_argument(
        "--fov-sweep",
        action="store_true",
        help="repeat across plausible FOVs, since it is uncalibrated",
    )
    ap.add_argument(
        "--noise",
        type=float,
        default=DEFAULT_PRIOR_REL_NOISE,
        help="assumed relative inverse-depth error of the prior",
    )
    ap.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE)
    args = ap.parse_args()

    if args.replay:
        scans = json.load(open(args.replay))["scans"]
        print(f"Replaying {len(scans)} scans from {args.replay}\n")
    elif args.live:
        print(
            f"Capturing {args.scans} scans from {args.host}:{args.port} ...",
            file=sys.stderr,
        )
        try:
            scans = collect_live(args.host, args.port, args.scans, args.timeout)
        except Exception as e:
            print(
                f"capture failed: {type(e).__name__}: {e}\n"
                f"Is the server running?  ./start_lidar.sh",
                file=sys.stderr,
            )
            return 1
        if args.save:
            Path(args.save).write_text(json.dumps({"scans": scans}))
            print(f"saved {len(scans)} scans to {args.save}", file=sys.stderr)
    else:
        ap.error("need --live or --replay")

    if not scans:
        print("no scans captured", file=sys.stderr)
        return 1

    fovs = [55.0, 70.0, 85.0] if args.fov_sweep else [args.fov]
    for fov in fovs:
        per_view = analyse(scans, fov, args.noise, args.tolerance)
        print(report(per_view, fov, args.noise, args.tolerance, len(scans)))
        if not args.fov_sweep:
            print(verdict(per_view, args.tolerance))
        print()

    if args.fov_sweep:
        print(
            "If the identifiable rate moves materially across those three, the "
            "uncalibrated FOV is doing the work and C0 has to happen first."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
