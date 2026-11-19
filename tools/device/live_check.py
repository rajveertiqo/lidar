#!/usr/bin/env python3
"""What the real sensors do, from the running app: read-only, GET requests only.

    venv/bin/python tools/device/live_check.py              # 60 s, the app on this Pi
    venv/bin/python tools/device/live_check.py -s 120 --base http://192.168.0.221:8000

The desk simulators assume things about the radar the device has not measured: a board
missing a frame now and then (tools/sim/walk_sim --drop), double-bounce ghosts at twice a
wall's range (--ghosts), a few cm of range noise (--noise). This polls /radar/state for
`-s` seconds and reports, per board, from its own frames (each new `last_update`):

  frames/s     how often it reports (RADAR_UPDATE_RATE is asked for)
  missed       frames with no echo at all between two that heard something (walk_sim --drop)
  ghosts       frames listing an echo at twice another of its own (GLASS_GHOST_TOL_MM; --ghosts)
  jitter       the nearest echo's frame-to-frame change, median, standing still (--noise)

and, once each, the camera's side of the live picture's delay (/camera/stats "pipeline":
the sensor's frame rate, how long after the sensor read a picture out the app had it, how
many were handed over already finished) and whether the LiDAR reader keeps up (/lidar/reader).
Stand the rig still facing a room with walls 1-3 m off for the radar numbers: walking, the
jitter is the walking. (added 2026-11-19)"""

import argparse
import json
import statistics
import sys
import time
import urllib.error
import urllib.request

GHOST_TOL_MM = 150.0  # app.config.GLASS_GHOST_TOL_MM (not imported: runnable off the Pi too)


def fetch(base, path):
    """(payload, error string). Never raises: a missing endpoint is a result."""
    try:
        with urllib.request.urlopen(base + path, timeout=5) as r:
            return json.loads(r.read().decode()), None
    except urllib.error.HTTPError as e:
        return None, f"HTTP {e.code}"
    except (urllib.error.URLError, OSError) as e:
        return None, f"unreachable ({getattr(e, 'reason', e)})"


def board_frames(samples):
    """{board key: [(stamp, [echo mm, ...]), ...]}: each board's own frames, in order, one
    per new last_update, from /radar/state snapshots."""
    out = {}
    for snap in samples:
        for key, row in ((snap or {}).get("sectors") or {}).items():
            if not str(key).startswith("deg") or not isinstance(row, dict):
                continue
            stamp = row.get("last_update")
            if not isinstance(stamp, (int, float)) or not row.get("online"):
                continue
            frames = out.setdefault(key, [])
            if frames and stamp <= frames[-1][0]:
                continue  # the same frame polled again
            peaks = [float(e) for e in (row.get("peaks_mm") or []) if isinstance(e, (int, float))]
            frames.append((float(stamp), sorted(peaks)))
    return out


def board_stats(frames):
    """One board's numbers (see the module docstring) from its frames."""
    n = len(frames)
    if n < 2:
        return {"frames": n}
    span = frames[-1][0] - frames[0][0]
    heard = [bool(p) for _t, p in frames]
    missed = sum(1 for i in range(1, n - 1) if not heard[i] and heard[i - 1] and heard[i + 1])
    ghosts = sum(
        1 for _t, p in frames
        if any(abs(e - 2.0 * x) <= GHOST_TOL_MM for e in p for x in p if x < e)
    )
    nearest = [p[0] for _t, p in frames if p]
    steps = [abs(b - a) for a, b in zip(nearest, nearest[1:])]
    return {
        "frames": n,
        "frames_per_s": round((n - 1) / span, 2) if span > 0 else None,
        "silent": round(1.0 - sum(heard) / n, 3),
        "missed": round(missed / max(1, n - 2), 3),
        "ghosts": round(ghosts / n, 3),
        "jitter_mm": round(statistics.median(steps), 1) if steps else None,
        "nearest_mm": round(statistics.median(nearest), 0) if nearest else None,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--base", default="http://127.0.0.1:8000")
    ap.add_argument("-s", "--seconds", type=float, default=60.0)
    ap.add_argument("--hz", type=float, default=20.0, help="polls a second (above the boards')")
    args = ap.parse_args()
    base = args.base.rstrip("/")

    first, err = fetch(base, "/radar/state")
    if err:
        print(f"the app is not answering at {base}: {err}")
        return 2
    print(f"polling {base}/radar/state for {args.seconds:.0f} s ...")
    samples, end = [first], time.monotonic() + args.seconds
    while time.monotonic() < end:
        snap, err = fetch(base, "/radar/state")
        if snap is not None:
            samples.append(snap)
        time.sleep(1.0 / args.hz)

    print("\nradar, per board (its own frames)")
    print(f"  {'board':8} {'frames':>6} {'per s':>6} {'silent':>7} {'missed':>7} {'ghosts':>7} "
          f"{'jitter':>7} {'nearest':>8}")
    for key, frames in sorted(board_frames(samples).items()):
        st = board_stats(frames)
        if st["frames"] < 2:
            print(f"  {key:8} {st['frames']:>6}  (too few frames)")
            continue
        pct = lambda v: f"{100 * v:6.1f}%"  # noqa: E731
        print(f"  {key:8} {st['frames']:>6} {st['frames_per_s'] or 0:>6.1f} {pct(st['silent'])} "
              f"{pct(st['missed'])} {pct(st['ghosts'])} {st['jitter_mm'] or 0:>5.0f}mm "
              f"{st['nearest_mm'] or 0:>6.0f}mm")

    stats, err = fetch(base, "/camera/stats")
    pipe = (stats or {}).get("pipeline") or {}
    print("\ncamera (its side of the live picture's delay)")
    if err or not pipe:
        print(f"  not available: {err or 'no pipeline timings yet'}")
    else:
        print(f"  sensor {pipe.get('sensor_fps')} pictures/s, the app had each "
              f"{pipe.get('lag_ms')} ms after the sensor read it out (worst {pipe.get('lag_ms_worst')}), "
              f"{100 * (pipe.get('held') or 0):.0f}% handed over already finished")

    reader, err = fetch(base, "/lidar/reader")
    print("\nlidar reader")
    print(f"  {err}" if err else "  " + ", ".join(f"{k} {v}" for k, v in (reader or {}).items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
