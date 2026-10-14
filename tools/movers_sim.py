#!/usr/bin/env python3
"""Moving things without leaving the desk: is each person or vehicle coming at the user caught, from the right side, and nothing still ever called moving?"""

import argparse
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from utils.lidar_movers import MoverTracker                 # noqa: E402
from utils.radar_movers import RadarMoverTracker            # noqa: E402

HZ = 15.0                    # scans a second
BEAMS = 264                  # the rig's density: ~264 returns a revolution
PRECESS_DEG = 0.32           # the rig's sample angles creep this much a revolution
NOISE_M = 0.005              # range noise on every return
YAW_BIAS_DEG = -0.13         # heading_tracker's per-scan yaw error, measured standing still


# ---------------------------------------------------------------- geometry (x forward, y right, m)

def _ray_circle(ox, oy, dx, dy, cx, cy, r):
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    disc = b * b - (fx * fx + fy * fy - r * r)
    if disc < 0:
        return None
    s = -b - math.sqrt(disc)
    return s if s > 1e-6 else None


def _ray_seg(ox, oy, dx, dy, x1, y1, x2, y2):
    ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    s = ((x1 - ox) * ey - (y1 - oy) * ex) / den
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
    return s if s > 1e-6 and 0.0 <= u <= 1.0 else None


def _box(cx, cy, length, width, heading):
    c, s = math.cos(heading), math.sin(heading)
    pts = [(cx + c * a - s * b, cy + s * a + c * b)
           for a, b in ((length / 2, width / 2), (length / 2, -width / 2),
                        (-length / 2, -width / 2), (-length / 2, width / 2))]
    return [(*pts[i], *pts[(i + 1) % 4]) for i in range(4)]


def _corridor():
    """A corridor 2 m wide with a 1 m doorway every 2.8 m, rooms either side."""
    segs, x = [], -5.0
    while x < 30.0:
        segs += [(x, -1.0, x + 1.8, -1.0), (x, 1.0, x + 1.8, 1.0)]
        x += 2.8
    return segs + [(-5, -4, 30, -4), (-5, 4, 30, 4), (30, -4, 30, 4), (-5, -4, -5, 4)]


ROOM = [(-5, -3.5, 12, -3.5), (12, -3.5, 12, 3.5), (12, 3.5, -5, 3.5), (-5, 3.5, -5, -3.5)]
CORRIDOR = _corridor()
STILL = [(2.0, -2.2, 0.22), (4.0, 2.4, 0.25), (6.5, -1.8, 0.2), (-2.0, 2.2, 0.25),
         (3.2, -2.8, 0.2), (7.5, 2.0, 0.3), (-3.0, -2.0, 0.22), (9.0, -1.5, 0.25)]
CLUTTER = STILL + [(1.5, 1.2, 0.3), (2.5, -1.0, 0.25), (5.0, 0.9, 0.35), (5.5, -2.5, 0.3),
                   (8.0, -0.2, 0.3), (-1.5, -1.0, 0.3), (-1.0, 1.5, 0.25), (10.5, 1.0, 0.3)]
POLES = [(3.0, -1.5, 0.1), (5.0, 2.0, 0.1), (8.0, -2.5, 0.1), (-3.0, 2.0, 0.1),
         (1.0, 3.0, 0.1), (-4.0, -2.5, 0.1), (6.5, 3.0, 0.1)]
BLIND = (170.0, 290.0)       # the rig's back-left arc that returns nothing


def person(x, y, vx, vy, r=0.22):
    return {"x": x, "y": y, "vx": vx, "vy": vy, "r": r}


def car(x, y, vx, vy, heading):
    return {"x": x, "y": y, "vx": vx, "vy": vy, "box": (4.2, 1.8, heading)}


def scan(user, psi, phase, movers, walls, still, blind=None, drop=0.0):
    """One revolution as (angle deg, distance mm) from the user at `user`, facing `psi`."""
    ux, uy = user
    pts = []
    for k in range(BEAMS):
        b = (phase + k * 360.0 / BEAMS) % 360.0
        if (blind and blind[0] <= b <= blind[1]) or (drop and random.random() < drop):
            continue
        a = psi + math.radians(b)
        dx, dy = math.cos(a), math.sin(a)
        hits = [_ray_seg(ux, uy, dx, dy, *s) for s in walls]
        hits += [_ray_circle(ux, uy, dx, dy, *c) for c in still]
        for m in movers:
            if "box" in m:
                hits += [_ray_seg(ux, uy, dx, dy, *s) for s in _box(m["x"], m["y"], *m["box"])]
            else:
                hits.append(_ray_circle(ux, uy, dx, dy, m["x"], m["y"], m["r"]))
        hits = [h for h in hits if h is not None]
        if hits and min(hits) <= 12.0:
            pts.append((b, (min(hits) + random.gauss(0.0, NOISE_M)) * 1000.0))
    return pts


# ---------------------------------------------------------------- one scene

def run_scene(seconds, movers=(), walk=0.0, turn_dps=0.0, walls=ROOM, still=STILL,
              blind=None, drop=0.0, seed=1):
    """(first warning as (t, word, mode, side) or None, scans that called a still thing moving)."""
    random.seed(seed)
    tracker = MoverTracker()
    user, psi, phase = [0.0, 0.0], 0.0, 0.0
    ms = [dict(m) for m in movers]
    first, false_moving = None, 0
    for i in range(int(seconds * HZ)):
        t = i / HZ
        dpsi = math.radians(turn_dps / HZ)
        heading = {"scan_yaw_deg": (math.degrees(dpsi) + YAW_BIAS_DEG
                                    + random.gauss(0.0, 0.05)) if i else None}
        tracker.update(scan(user, psi, phase, ms, walls, still, blind, drop), 100.0 + t, heading)
        snap = tracker.snapshot()
        truth = []
        for m in ms:
            dx, dy = m["x"] - user[0], m["y"] - user[1]
            truth.append((dx * math.cos(psi) + dy * math.sin(psi),
                          -dx * math.sin(psi) + dy * math.cos(psi),
                          0.8 + (m["box"][0] / 2 if "box" in m else 0.0)))
        false_moving += sum(1 for trk in snap["tracks"]
                            if not any(math.hypot(trk["x_m"] - x, trk["y_m"] - y) < tol
                                       for x, y, tol in truth))
        th = snap["threat"]
        if th and first is None:
            first = (round(t, 2), th["word"], th["mode"], th["side"], th["ttc_s"])
        user[0] += walk * math.cos(psi) / HZ
        user[1] += walk * math.sin(psi) / HZ
        psi += dpsi
        phase = (phase + PRECESS_DEG) % (360.0 / BEAMS)
        for m in ms:
            m["x"] += m["vx"] / HZ
            m["y"] += m["vy"] / HZ
    return first, false_moving


# (name, expected (word, mode, side) or None, scene)
SCENES = [
    ("standing still in a room", None, dict(seconds=8)),
    ("walking 1.2 m/s in a room", None, dict(seconds=6, walk=1.2)),
    ("walking 1.2 m/s in a bare room (no still things)", None, dict(seconds=6, walk=1.2, still=[])),
    ("turning on the spot at 40 deg/s", None, dict(seconds=6, turn_dps=40)),
    ("walking 1.0 m/s and turning 15 deg/s", None, dict(seconds=6, walk=1.0, turn_dps=15)),
    ("walking 1.3 m/s down a corridor with doorways", None,
     dict(seconds=12, walk=1.3, walls=CORRIDOR, still=[])),
    ("clutter, 15% of beams lost, walking and turning", None,
     dict(seconds=8, walk=1.4, turn_dps=20, still=CLUTTER, drop=0.15)),
    ("standing still 30 s, 15% of beams lost", None, dict(seconds=30, drop=0.15)),
    ("person crossing close ahead from the left", ("Person", "crossing", "left"),
     dict(seconds=6, movers=[person(0.8, -3.0, 0.0, 1.3)])),
    ("person crossing close ahead from the right", ("Person", "crossing", "right"),
     dict(seconds=6, movers=[person(1.0, 3.0, 0.0, -1.3)])),
    ("person crossing 3 m ahead (passes clear)", None,
     dict(seconds=6, movers=[person(3.0, -3.0, 0.0, 1.3)])),
    ("person coming head-on", ("Person", "toward", "right"),
     dict(seconds=6, movers=[person(7.0, 0.3, -1.3, 0.0)])),
    ("person coming from behind", ("Person", "behind", "left"),
     dict(seconds=6, movers=[person(-4.5, -0.2, 1.4, 0.0)])),
    ("walking 1.2, person on a collision course from the right", ("Person", "crossing", "right"),
     dict(seconds=5, walk=1.2, movers=[person(3.0, 3.25, 0.0, -1.3)])),
    ("walking 1.2, person coming head-on", ("Person", "toward", "right"),
     dict(seconds=5, walk=1.2, movers=[person(9.5, 0.2, -1.3, 0.0)])),
    ("walking 1.0 in a corridor, person coming head-on", ("Person", "toward", "right"),
     dict(seconds=8, walk=1.0, walls=CORRIDOR, still=[], movers=[person(12.0, 0.2, -1.3, 0.0)])),
    ("person walking away", None, dict(seconds=6, movers=[person(1.5, 0.5, 1.3, 0.0)])),
    ("person passing 1.5 m to the side", None,
     dict(seconds=6, movers=[person(-3.0, -1.5, 1.3, 0.0)])),
    ("outdoors, nothing moving", None, dict(seconds=6, walls=[], still=POLES)),
    ("outdoors, car head-on at 4 m/s", ("Vehicle", "toward", "right"),
     dict(seconds=4, walls=[], still=POLES, movers=[car(10.0, 0.5, -4.0, 0.0, 0.0)])),
    ("outdoors, bicycle crossing from the left at 4 m/s", ("Vehicle", "crossing", "left"),
     dict(seconds=4, walls=[], still=POLES, movers=[person(1.2, -6.0, 0.0, 4.0, r=0.3)])),
    ("blind back-left arc, standing still", None, dict(seconds=8, blind=BLIND)),
    ("blind back-left arc, person from behind-right", ("Person", "behind", "right"),
     dict(seconds=6, blind=BLIND, movers=[person(-3.5, 1.6, 1.2, -0.45)])),
]


# ---------------------------------------------------------------- the radar (utils/radar_movers)

RADAR_HZ = 6.0               # each board's frame rate
RADAR_BOARDS = (30.0, 90.0, 150.0, 210.0, 270.0, 330.0)
RADAR_HALF_BEAM = 32.5
RADAR_NOISE_M = 0.01
RADAR_DROP = 0.1             # an echo missing from a frame


def _ego_points(user, psi, walls, still, movers):
    """Every surface as sample points (x ahead, y right, m) around the user."""
    def ego(wx, wy):
        dx, dy = wx - user[0], wy - user[1]
        return (dx * math.cos(psi) + dy * math.sin(psi), -dx * math.sin(psi) + dy * math.cos(psi))
    objects = []
    for x1, y1, x2, y2 in walls:
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.05))
        objects.append([ego(x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n) for i in range(n + 1)])
    for cx, cy, r in still:
        objects.append([ego(cx + r * math.cos(a), cy + r * math.sin(a))
                        for a in (k * math.pi / 8 for k in range(16))])
    for m in movers:
        if "box" in m:
            for seg in _box(m["x"], m["y"], *m["box"]):
                objects.append(_ego_points(user, psi, [seg], [], [])[0])
        else:
            objects.append([ego(m["x"] + m["r"] * math.cos(a), m["y"] + m["r"] * math.sin(a))
                            for a in (k * math.pi / 8 for k in range(16))])
    return objects


def radar_rows(user, psi, walls, still, movers, t):
    """One frame per board: the nearest distance of each thing in its beam, in mm."""
    rows = {}
    for bearing in RADAR_BOARDS:
        peaks = []
        for pts in _ego_points(user, psi, walls, still, movers):
            ds = [math.hypot(x, y) for x, y in pts
                  if abs((math.degrees(math.atan2(y, x)) - bearing + 180.0) % 360.0 - 180.0)
                  <= RADAR_HALF_BEAM]
            if ds and 0.25 <= min(ds) <= 4.0 and random.random() > RADAR_DROP:
                peaks.append(min(ds) + random.gauss(0.0, RADAR_NOISE_M))
        merged = []
        for d in sorted(peaks):
            if not merged or d - merged[-1] > 0.15:
                merged.append(d)
        rows[f"deg{int(bearing):03d}"] = {"online": True, "degraded": False, "stale": False,
                                         "last_update": t,
                                         "peaks_mm": [round(d * 1000.0, 1) for d in merged[:4]]}
    return {"sectors": rows}


def run_radar_scene(seconds, movers=(), walk=0.0, turn_dps=0.0, walls=ROOM, still=STILL,
                    blind=BLIND, radar_only=(), seed=1):
    """(first radar warning as (t, word, mode, side) or None, its ttc). `radar_only`: walls
    (glass) and things (below the laser's plane) the radar hears and the laser does not."""
    random.seed(seed)
    lidar, radar = MoverTracker(), RadarMoverTracker()
    user, psi, phase = [0.0, 0.0], 0.0, 0.0
    ms = [dict(m) for m in movers]
    hidden_walls = [w for w in radar_only if isinstance(w, tuple)]
    hidden_things = [{"r": 0.2, **m} for m in radar_only if isinstance(m, dict)]
    rows, first, next_frame = {"sectors": {}}, None, 0.0
    for i in range(int(seconds * HZ)):
        t = 100.0 + i / HZ
        dpsi = math.radians(turn_dps / HZ)
        pts = scan(user, psi, phase, ms, walls, still, blind)
        lidar.update(pts, t, {"scan_yaw_deg": (math.degrees(dpsi) + YAW_BIAS_DEG
                                               + random.gauss(0.0, 0.05)) if i else None})
        if t >= next_frame:
            rows = radar_rows(user, psi, walls + hidden_walls, still, ms + hidden_things, t)
            next_frame = t + 1.0 / RADAR_HZ
        if i % 3 == 0:                                       # the 5 Hz guidance tick
            snap = lidar.snapshot()
            radar.update(rows, snap["blobs"], snap["ego"],
                         turn_dps=turn_dps + YAW_BIAS_DEG * HZ)
            th = radar.snapshot()["threat"]
            if th and first is None:
                first = (round(t - 100.0, 2), th["word"], th["mode"], th["side"], th["ttc_s"])
        user[0] += walk * math.cos(psi) / HZ
        user[1] += walk * math.sin(psi) / HZ
        psi += dpsi
        phase = (phase + PRECESS_DEG) % (360.0 / BEAMS)
        for m in ms + hidden_things:
            m["x"] += m["vx"] / HZ
            m["y"] += m["vy"] / HZ
    return first


GLASS_AHEAD = [(5.0, -1.0, 5.0, 1.0)]          # a pane across the path, 5 m ahead

RADAR_SCENES = [
    ("radar: standing still in a room", None, dict(seconds=8)),
    ("radar: walking 1.2 m/s in a room", None, dict(seconds=6, walk=1.2)),
    ("radar: walking 1.2 m/s down a bare corridor", None,
     dict(seconds=8, walk=1.2, walls=CORRIDOR, still=[])),
    ("radar: walking 1.0 m/s toward a glass pane", None,
     dict(seconds=4, walk=1.0, radar_only=[*GLASS_AHEAD])),
    ("radar: person from behind-left, in the LiDAR's blind arc", ("Something", "behind", "left"),
     dict(seconds=6, movers=[person(-3.3, -1.6, 1.17, 0.57)])),
    ("radar: turning on the spot at 40 deg/s in a room", None, dict(seconds=6, turn_dps=40)),
    ("radar: person from behind-right (the LiDAR sees it: left to it)", None,
     dict(seconds=6, movers=[person(-2.5, 3.0, 0.83, -1.0)])),
    ("radar: something low rolling in from the left, under the laser", ("Something", "crossing", "left"),
     dict(seconds=6, radar_only=[{"x": 0.3, "y": -3.2, "vx": 0.0, "vy": 1.0}])),
    ("radar: person jogging 2 m/s from behind-left, in the blind arc", ("Something", "behind", "left"),
     dict(seconds=5, movers=[person(-4.4, -1.6, 1.88, 0.68)])),
]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("-k", metavar="TEXT", help="only the scenes whose name contains TEXT")
    args = p.parse_args()
    failures = 0
    for name, expect, scene in SCENES:
        if args.k and args.k not in name:
            continue
        first, false_moving = run_scene(**scene)
        got = first[1:4] if first else None
        ok = got == expect and false_moving == 0
        failures += not ok
        when = f"warned at {first[0]:.2f} s, {first[4]:.1f} s ahead" if first else "no warning"
        print(f"{'ok' if ok else 'FAIL':<5} {name:<56} -> {got or '-'}  ({when}"
              + (f", {false_moving} scans called a still thing moving" if false_moving else "") + ")")
    for name, expect, scene in RADAR_SCENES:
        if args.k and args.k not in name:
            continue
        first = run_radar_scene(**scene)
        got = first[1:4] if first else None
        failures += got != expect
        when = f"warned at {first[0]:.2f} s, {first[4]:.1f} s ahead" if first else "no warning"
        print(f"{'ok' if got == expect else 'FAIL':<5} {name:<56} -> {got or '-'}  ({when})")
    print(f"\n{failures} failure(s)" if failures else "\nall scenes as expected")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
