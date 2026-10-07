#!/usr/bin/env python3
"""Glass scenes without leaving the desk: is the pane named, and only the pane?"""

import argparse
import math
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from utils import simple_guidance as sg                     # noqa: E402
from utils import walk_policy                              # noqa: E402

FT = 304.8
LASER_STEP_DEG = 360.0 / 260.0      # the rig's density: ~260 returns a revolution
LASER_MAX_MM = 12000.0
BEAM_EDGE = 1.0 / math.sin(math.radians(30.0 + 65.0 / 2.0))   # a side wall at the beam's edge
DOOR_HALF_MM = 450.0                # a 0.9 m door
BACK_WALL_MM = 6000.0               # the room behind the glass
LASER_NOISE_MM = 15.0               # range noise on every laser return


# ---------------------------------------------------------------- geometry (x forward, y right, mm)

def wall_across(x, half_gap=None, reach=4000.0, angle_deg=0.0):
    """A wall across the path at forward distance x, optionally with a gap on the walking line."""
    pieces = ([(-reach, reach)] if half_gap is None else [(-reach, -half_gap), (half_gap, reach)])
    t = math.radians(angle_deg)
    out = []
    for y1, y2 in pieces:
        out.append((x - y1 * math.sin(t), y1 * math.cos(t), x - y2 * math.sin(t), y2 * math.cos(t)))
    return out


def side_wall(side, lateral, x_from=-2000.0, x_to=8000.0):
    y = lateral if side == "right" else -lateral
    return [(x_from, y, x_to, y)]


def post(x, y, half=150.0):
    return [(x - half, y - half, x + half, y - half), (x + half, y - half, x + half, y + half),
            (x + half, y + half, x - half, y + half), (x - half, y + half, x - half, y - half)]


def back_wall():
    return wall_across(BACK_WALL_MM, reach=8000.0)


def ray_cast(segments):
    rng = random.Random(len(segments))     # fixed noise, so runs repeat
    angles, dists = [], []
    b = 0.37                                   # an arbitrary phase, fixed so runs repeat
    while b < 360.0:
        r = math.radians(b)
        dx, dy = math.cos(r), math.sin(r)
        best = None
        for x1, y1, x2, y2 in segments:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-9:
                continue
            t = (x1 * ey - y1 * ex) / den
            s = (x1 * dy - y1 * dx) / den
            if t > 1.0 and 0.0 <= s <= 1.0 and (best is None or t < best):
                best = t
        if best is not None and best <= LASER_MAX_MM:
            angles.append(round(b, 2))
            dists.append(round(best + rng.uniform(-LASER_NOISE_MM, LASER_NOISE_MM), 1))
        b += LASER_STEP_DEG
    return angles, dists


# ---------------------------------------------------------------- sensor states

def _gap(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def lidar_state(segments):
    import config as c
    angles, dists = ray_cast(segments)
    sectors = {}
    for name, (lo, hi) in c.LIDAR_SECTORS.items():
        ds = [d for a, d in zip(angles, dists)
              if (lo <= a < hi if lo < hi else (a >= lo or a < hi))]
        sectors[name] = {"min_dist": min(ds) if ds else None, "count": len(ds)}
    cones = {}
    for name, centre in c.GUIDANCE_CONE_CENTRES.items():
        best = line = None
        for a, d in zip(angles, dists):
            g = _gap(a, centre)
            in_cone = g <= c.GUIDANCE_CONE_DEG / 2.0
            in_corr = (name == "front" and g < 90.0 and d <= c.GUIDANCE_CORRIDOR_MAX_MM
                       and abs(d * math.sin(math.radians(g))) <= c.GUIDANCE_CORRIDOR_HALF_WIDTH_MM)
            if in_cone or in_corr:
                best = d if best is None else min(best, d)
            if in_cone:
                line = d if line is None else min(line, d)
        cones[name] = {"min_dist": best}
        if name == "front":
            cones["front_line"] = {"min_dist": line}
    return {"points": {"angles": angles, "distances": dists}, "sectors": sectors,
            "cones": cones, "rpm": 900, "last_update": time.time(), "simulated": False}


def radar_state(echoes):
    """echoes: {board key: mm or None}; every board online, deaf past its reach."""
    import config as c
    now = time.time()
    secs = {}
    for key in ("deg030", "deg090", "deg150", "deg210", "deg270", "deg330"):
        d = echoes.get(key)
        d = None if d is None or d > c.RADAR_END_MM else d
        secs[key] = {"online": True, "degraded": False, "min_dist": d,
                     "peaks_mm": [] if d is None else [d], "last_update": now}
    for card, keys in (("front", ("deg030", "deg330")), ("right", ("deg090",)),
                       ("back", ("deg150", "deg210")), ("left", ("deg270",))):
        ds = [secs[k]["min_dist"] for k in keys if secs[k]["min_dist"]]
        secs[card] = {"online": True, "degraded": False, "min_dist": min(ds) if ds else None,
                      "sources": keys, "sources_live": keys, "last_update": now}
    return {"sectors": secs, "boards": [], "missing": [], "last_update": now}


# ---------------------------------------------------------------- scenes

def scenes():
    out = []

    def add(kind, name, segments, echoes):
        out.append((kind, name, segments, echoes))

    for ft in (1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.5, 6.0, 8.0, 9.5, 11.0, 12.0):
        d = ft * FT
        add("glass", f"glass door in a wall, {ft} ft",
            wall_across(d, DOOR_HALF_MM) + back_wall(), {"deg030": d, "deg330": d})
    for side, near in (("left", "deg330"), ("right", "deg030")):
        far = "deg030" if near == "deg330" else "deg330"
        for ft in (1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 9.5, 11.0, 12.0):
            d, lat = ft * FT, 1.1 * FT
            add("glass", f"pane {ft} ft ahead, wall 1.1 ft to the {side}",
                side_wall(side, lat, x_to=d) + back_wall(),
                {near: min(d, lat * BEAM_EDGE), far: d})
    for ft in (2.0, 2.6, 3.4):
        d, lat = ft * FT, 1000.0
        # Farther than ~2.3 ft the pane is about as far as the side walls: range alone cannot tell them apart.
        add("glass" if ft <= 2.0 else "limit",
            f"glass door ending a 2 m corridor, {ft} ft" + ("" if ft <= 2.0 else " (as far as its walls)"),
            side_wall("left", lat) + side_wall("right", lat) + wall_across(d, DOOR_HALF_MM)
            + back_wall(), {"deg030": min(d, lat * BEAM_EDGE), "deg330": min(d, lat * BEAM_EDGE)})
    for ft in (2.0, 4.0, 6.0, 9.5, 12.0):
        d = ft * FT
        add("glass", f"glass wall across, {ft} ft, room beyond", back_wall(),
            {"deg030": d, "deg330": d})

    for lat, how in ((750.0, "edge"), (750.0, "abeam"), (450.0, "edge"), (450.0, "abeam"),
                     (1500.0, "abeam")):
        e = lat * BEAM_EDGE if how == "edge" else lat - 50.0
        add("clear", f"corridor {2 * lat / 1000:.1f} m wide, walls echo at the {how}",
            side_wall("left", lat) + side_wall("right", lat) + back_wall(),
            {"deg030": e, "deg330": e})
    for lat in (500.0, 1000.0):
        add("clear", f"wall on the left {lat / FT:.1f} ft away",
            side_wall("left", lat) + back_wall(), {"deg330": lat * BEAM_EDGE})
    for ft in (2.0, 4.0, 6.0):
        d = ft * FT
        add("clear", f"solid wall ahead, {ft} ft", wall_across(d), {"deg030": d, "deg330": d})
    add("clear", "open space", [], {})
    x, y = 1200.0 * math.cos(math.radians(40)), 1200.0 * math.sin(math.radians(40))
    add("clear", "post 4 ft away at 40 deg right", post(x, y) + back_wall(),
        {"deg030": 1200.0 - 150.0})
    add("clear", "post 4 ft away at 40 deg right, radar 0.2 m ahead of the laser",
        post(x, y) + back_wall(), {"deg030": 1200.0 - 350.0})
    x, y = 1500.0 * math.cos(math.radians(-20)), 1500.0 * math.sin(math.radians(-20))
    # The door-in-wall rule (2026-09-29) reads this as a pane: known, not yet decided.
    add("limit", "post 5 ft away at 20 deg left, seen by both boards", post(x, y) + back_wall(),
        {"deg030": 1500.0 - 150.0, "deg330": 1500.0 - 150.0})

    for ft in (2.0, 3.0, 4.0, 6.0):
        d = ft * FT
        add("limit", f"OPEN doorway, {ft} ft (radar echoes the frames)",
            wall_across(d, DOOR_HALF_MM) + back_wall(),
            {"deg030": math.hypot(d, DOOR_HALF_MM), "deg330": math.hypot(d, DOOR_HALF_MM)})
    for ft in (1.5, 3.0):
        d = ft * FT
        add("limit", f"pane {ft} ft ahead seen by ONE board (off square)",
            back_wall(), {"deg030": d})
    return out


def run(verbose=False):
    failures = 0
    for kind, name, segments, echoes in scenes():
        walk_policy._last_measured.clear()
        walk_policy._remember_turn(None, 0.0)
        lidar, radar = lidar_state(segments), radar_state(echoes)
        wd = walk_policy.decide(lidar, radar)
        raw = sg.decide(wd, lidar, now=time.time(), radar_state=radar)
        said = raw["instruction"]
        named = said in (sg.BLOCKED_GLASS, sg.GLASS_AHEAD) + sg._GLASS_BESIDE
        if kind == "glass":
            verdict = "ok" if named else "MISSED"
        elif kind == "clear":
            verdict = "ok" if not named else "FALSE GLASS"
        else:
            verdict = "glass" if named else "no glass"
        failures += verdict in ("MISSED", "FALSE GLASS")
        front = wd["directions"]["front"]
        print(f"{verdict:<12} {name:<62} -> {said!r}"
              + (f"   [front {front['distance_ft']} ft, {front['fusion']}]" if verbose else ""))
    return failures


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("-v", "--verbose", action="store_true", help="show the fused front reading")
    args = p.parse_args()
    failures = run(args.verbose)
    print(f"\n{failures} failure(s)" if failures else "\nall scenes as expected")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
