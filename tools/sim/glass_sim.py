#!/usr/bin/env python3
"""Glass scenes without leaving the desk: is the pane named, and only the pane?

    venv/bin/python tools/sim/glass_sim.py             # every scene, then the door grids
    venv/bin/python tools/sim/glass_sim.py --no-grids  # the scenes only

The door grids (2026-11-13, design notes 24 and 25): glass doors met 10-50 deg off, which
must be named up to 20 deg off from 11 ft and up to 45 deg off from 5 ft (50 deg off, a block
within 4 ft, glass or not: the frame is on the path); and with someone
standing by the door (0.65-0.85 m along its wall, up to 0.4 m in front of it, 0-20 deg off),
which walking in from 8 ft must be named by 4 ft. What lies past those is printed as a limit,
not failed (docs/known-limits.md)."""

import argparse
import math
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from app.guidance import simple_guidance as sg  # noqa: E402
from app.perception import walk_policy  # noqa: E402

FT = 304.8
LASER_STEP_DEG = 360.0 / 260.0  # the rig's density: ~260 returns a revolution
LASER_MAX_MM = 12000.0
BEAM_EDGE = 1.0 / math.sin(math.radians(30.0 + 65.0 / 2.0))  # a side wall at the beam's edge
DOOR_HALF_MM = 450.0  # a 0.9 m door
BACK_WALL_MM = 6000.0  # the room behind the glass
LASER_NOISE_MM = 15.0  # range noise on every laser return


# ---------------------------------------------------------------- geometry (x forward, y right, mm)


def wall_across(x, half_gap=None, reach=4000.0, angle_deg=0.0):
    """A wall across the path at forward distance x, optionally with a gap on the walking line."""
    pieces = [(-reach, reach)] if half_gap is None else [(-reach, -half_gap), (half_gap, reach)]
    t = math.radians(angle_deg)
    out = []
    for y1, y2 in pieces:
        out.append(
            (
                x - y1 * math.sin(t),
                y1 * math.cos(t),
                x - y2 * math.sin(t),
                y2 * math.cos(t),
            )
        )
    return out


def side_wall(side, lateral, x_from=-2000.0, x_to=8000.0):
    y = lateral if side == "right" else -lateral
    return [(x_from, y, x_to, y)]


def post(x, y, half=150.0):
    return [
        (x - half, y - half, x + half, y - half),
        (x + half, y - half, x + half, y + half),
        (x + half, y + half, x - half, y + half),
        (x - half, y + half, x - half, y - half),
    ]


def back_wall():
    return wall_across(BACK_WALL_MM, reach=8000.0)


def ray_cast(segments):
    rng = random.Random(len(segments))  # fixed noise, so runs repeat
    angles, dists = [], []
    b = 0.37  # an arbitrary phase, fixed so runs repeat
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
    from app import config as c

    angles, dists = ray_cast(segments)
    sectors = {}
    for name, (lo, hi) in c.LIDAR_SECTORS.items():
        ds = [
            d for a, d in zip(angles, dists) if (lo <= a < hi if lo < hi else (a >= lo or a < hi))
        ]
        sectors[name] = {"min_dist": min(ds) if ds else None, "count": len(ds)}
    cones = {}
    for name, centre in c.GUIDANCE_CONE_CENTRES.items():
        best = line = None
        for a, d in zip(angles, dists):
            g = _gap(a, centre)
            in_cone = g <= c.GUIDANCE_CONE_DEG / 2.0
            in_corr = (
                name == "front"
                and g < 90.0
                and d <= c.GUIDANCE_CORRIDOR_MAX_MM
                and abs(d * math.sin(math.radians(g))) <= c.GUIDANCE_CORRIDOR_HALF_WIDTH_MM
            )
            if in_cone or in_corr:
                best = d if best is None else min(best, d)
            if in_cone:
                line = d if line is None else min(line, d)
        cones[name] = {"min_dist": best}
        if name == "front":
            cones["front_line"] = {"min_dist": line}
    return {
        "points": {"angles": angles, "distances": dists},
        "sectors": sectors,
        "cones": cones,
        "rpm": 900,
        "last_update": time.time(),
        "simulated": False,
    }


def radar_state(echoes):
    """echoes: {board key: mm, a list of mm (every echo it lists), or None}; every board
    online, deaf past its reach."""
    from app import config as c

    now = time.time()
    secs = {}
    for key in ("deg030", "deg090", "deg150", "deg210", "deg270", "deg330"):
        d = echoes.get(key)
        heard = d if isinstance(d, (list, tuple)) else [] if d is None else [d]
        heard = sorted(e for e in heard if e <= c.RADAR_END_MM)
        secs[key] = {
            "online": True,
            "degraded": False,
            "min_dist": heard[0] if heard else None,
            "peaks_mm": heard,
            "last_update": now,
        }
    for card, keys in (
        ("front", ("deg030", "deg330")),
        ("right", ("deg090",)),
        ("back", ("deg150", "deg210")),
        ("left", ("deg270",)),
    ):
        ds = [secs[k]["min_dist"] for k in keys if secs[k]["min_dist"]]
        secs[card] = {
            "online": True,
            "degraded": False,
            "min_dist": min(ds) if ds else None,
            "sources": keys,
            "sources_live": keys,
            "last_update": now,
        }
    return {"sectors": secs, "boards": [], "missing": [], "last_update": now}


def heard(segments, bearing, half=65.0 / 2.0):
    """The nearest range (mm) of `segments` inside a board's beam, `half` deg either side of
    its `bearing`, by 1 deg rays: what that board hears off them first. None: nothing there."""
    best = None
    for k in range(-int(half), int(half) + 1):
        r = math.radians(bearing + k)
        dx, dy = math.cos(r), math.sin(r)
        for x1, y1, x2, y2 in segments:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-9:
                continue
            t, u = (x1 * ey - y1 * ex) / den, (x1 * dy - y1 * dx) / den
            if t > 0 and 0.0 <= u <= 1.0:
                best = t if best is None else min(best, t)
    return best


def door_scene(ft, angle=0.0, person=None):
    """A glass door in a wall `ft` ahead, met `angle` deg off (+: its right side nearer), and
    maybe someone standing by it, person = (mm along the wall from the door's centre, +:
    right; mm in front of the wall): (segments, echoes). Each front board hears, by ray, the
    pane, the wall and the person, as many echoes; the laser looks through the pane."""
    d, t = ft * FT, math.radians(angle)
    wall = wall_across(d, DOOR_HALF_MM, angle_deg=angle)
    pane = wall_across(d, None, reach=DOOR_HALF_MM, angle_deg=angle)
    someone = []
    if person:
        along, in_front = person
        someone = post(
            d - along * math.sin(t) - in_front * math.cos(t),
            along * math.cos(t) - in_front * math.sin(t),
            half=170.0,
        )
    echoes = {}
    for key, bearing in (("deg030", 30.0), ("deg330", -30.0)):
        hits = (heard(s, bearing) for s in (pane, wall, someone) if s)
        echoes[key] = sorted(h for h in hits if h)
    return wall + someone + back_wall(), echoes


# ---------------------------------------------------------------- walking (radar tracking)


def walk_toward(
    lat_l,
    lat_r,
    pane_mm=None,
    pace=1.2,
    hz=10.0,
    seconds=4.0,
    extra=None,
    measured_pace=True,
    noise_mm=0.0,
    drop=0.0,
    seed=0,
):
    """Walk down a passage (walls lat_l / lat_r mm to each side) at `pace` m/s, the radar at
    `hz` frames a second: (seconds walked, pane distance mm or None, the front read) per frame.
    `noise_mm`: range noise on every echo (1 sigma); `drop`: the chance a board hears nothing
    in a frame.
    Each front board hears its own wall at its beam's edge (where the nearest echo of a side
    wall stays as you walk), and the pane straight ahead. `extra(s)`: more echoes per board,
    {key: [mm]}, s metres walked. The laser looks through the pane."""
    from unittest import mock

    walk_policy._near_frames.clear()
    walk_policy._remember_turn(None, 0.0)
    rng = random.Random(seed)
    out, t0 = [], 1000.0
    for k in range(int(seconds * hz) + 1):
        walked = pace * k / hz * 1000.0
        pane = None if pane_mm is None else pane_mm - walked
        if pane is not None and pane < 150.0:
            break
        segs = side_wall("left", lat_l) + side_wall("right", lat_r)
        segs += wall_across(BACK_WALL_MM - walked, reach=8000.0)
        echoes = {"deg330": [lat_l * BEAM_EDGE], "deg030": [lat_r * BEAM_EDGE]}
        if pane is not None:
            for key in echoes:
                echoes[key].append(pane)
        for key, more in ((extra or (lambda s: {}))(walked)).items():
            echoes[key] = echoes.get(key, []) + list(more)
        echoes = {
            key: [] if rng.random() < drop else [e + rng.gauss(0.0, noise_mm) for e in es]
            for key, es in echoes.items()
        }
        radar = radar_state(echoes)
        stamp = t0 + k / hz
        for row in radar["sectors"].values():
            row["last_update"] = stamp
        front = radar["sectors"]["front"]
        front["peaks_mm"] = sorted(
            e for key in ("deg030", "deg330") for e in radar["sectors"][key]["peaks_mm"]
        )
        lidar = lidar_state(segs)
        with mock.patch.object(
            walk_policy, "_walking_pace", lambda: pace if measured_pace else None
        ):
            read = walk_policy.read_direction(lidar, radar, "front")
        out.append((walked / 1000.0, pane, read))
    return out


# ---------------------------------------------------------------- scenes


def scenes():
    out = []

    def add(kind, name, segments, echoes):
        out.append((kind, name, segments, echoes))

    for ft in (1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.5, 6.0, 8.0, 9.5, 11.0, 12.0):
        d = ft * FT
        add(
            "glass",
            f"glass door in a wall, {ft} ft",
            wall_across(d, DOOR_HALF_MM) + back_wall(),
            {"deg030": d, "deg330": d},
        )
    for side, near in (("left", "deg330"), ("right", "deg030")):
        far = "deg030" if near == "deg330" else "deg330"
        for ft in (1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 9.5, 11.0, 12.0):
            d, lat = ft * FT, 1.1 * FT
            add(
                "glass",
                f"pane {ft} ft ahead, wall 1.1 ft to the {side}",
                side_wall(side, lat, x_to=d) + back_wall(),
                {near: min(d, lat * BEAM_EDGE), far: d},
            )
    for ft in (2.0, 2.6, 3.0, 3.4):
        d, lat = ft * FT, 1000.0
        # Past ~2.3 ft the pane is about as far as the side walls' echo. A limit until
        # 2026-11-09: the door's echo, each board's nearest, was set aside as the corridor's
        # wall. Now the laser finds the side walls themselves (walk_policy._side_walls_mm).
        add(
            "glass",
            f"glass door ending a 2 m corridor, {ft} ft"
            + ("" if ft <= 2.0 else " (as far as its walls)"),
            side_wall("left", lat)
            + side_wall("right", lat)
            + wall_across(d, DOOR_HALF_MM)
            + back_wall(),
            {"deg030": min(d, lat * BEAM_EDGE), "deg330": min(d, lat * BEAM_EDGE)},
        )
    # A pane across a narrow passage, its whole width (no frame the laser sees): each front
    # board hears the pane and its own side's wall. Until 2026-11-10 the walls "explained" the
    # pane's echo, and it said "Walk straight" (walk_policy._wall_too_far).
    for lat_l, lat_r, ft in ((914.0, 914.0, 2.0), (914.0, 914.0, 2.6), (366.0, 600.0, 1.5)):
        d = ft * FT
        add(
            "glass",
            f"pane across a passage, walls {lat_l / FT:.1f} / {lat_r / FT:.1f} ft, {ft} ft",
            side_wall("left", lat_l) + side_wall("right", lat_r) + back_wall(),
            {"deg330": [d, lat_l * BEAM_EDGE], "deg030": [d, lat_r * BEAM_EDGE]},
        )
    # Farther than the walls are to the side, a wall can echo from the pane's range too: the
    # radar's range cannot tell them apart (docs/known-limits.md).
    for lat_l, lat_r, ft in ((914.0, 914.0, 3.0), (600.0, 366.0, 1.8), (457.0, 457.0, 1.8)):
        d = ft * FT
        add(
            "limit",
            f"pane across a passage, walls {lat_l / FT:.1f} / {lat_r / FT:.1f} ft, {ft} ft",
            side_wall("left", lat_l) + side_wall("right", lat_r) + back_wall(),
            {"deg330": [d, lat_l * BEAM_EDGE], "deg030": [d, lat_r * BEAM_EDGE]},
        )
    # The same passages with no pane must stay clear.
    for lat_l, lat_r in ((914.0, 914.0), (366.0, 600.0), (600.0, 366.0), (457.0, 457.0)):
        add(
            "clear",
            f"passage, walls {lat_l / FT:.1f} / {lat_r / FT:.1f} ft, no pane",
            side_wall("left", lat_l) + side_wall("right", lat_r) + back_wall(),
            {
                "deg330": [lat_l - 50.0, lat_l * BEAM_EDGE],
                "deg030": [lat_r - 50.0, lat_r * BEAM_EDGE],
            },
        )
    for ft in (2.0, 4.0, 6.0, 9.5, 12.0):
        d = ft * FT
        add(
            "glass",
            f"glass wall across, {ft} ft, room beyond",
            back_wall(),
            {"deg030": d, "deg330": d},
        )

    for lat, how in (
        (750.0, "edge"),
        (750.0, "abeam"),
        (450.0, "edge"),
        (450.0, "abeam"),
        (1500.0, "abeam"),
    ):
        e = lat * BEAM_EDGE if how == "edge" else lat - 50.0
        add(
            "clear",
            f"corridor {2 * lat / 1000:.1f} m wide, walls echo at the {how}",
            side_wall("left", lat) + side_wall("right", lat) + back_wall(),
            {"deg030": e, "deg330": e},
        )
    for lat in (500.0, 1000.0):
        add(
            "clear",
            f"wall on the left {lat / FT:.1f} ft away",
            side_wall("left", lat) + back_wall(),
            {"deg330": lat * BEAM_EDGE},
        )
    for ft in (2.0, 4.0, 6.0):
        d = ft * FT
        add(
            "clear",
            f"solid wall ahead, {ft} ft",
            wall_across(d),
            {"deg030": d, "deg330": d},
        )
    add("clear", "open space", [], {})
    x, y = 1200.0 * math.cos(math.radians(40)), 1200.0 * math.sin(math.radians(40))
    add(
        "clear",
        "post 4 ft away at 40 deg right",
        post(x, y) + back_wall(),
        {"deg030": 1200.0 - 150.0},
    )
    add(
        "clear",
        "post 4 ft away at 40 deg right, radar 0.2 m ahead of the laser",
        post(x, y) + back_wall(),
        {"deg030": 1200.0 - 350.0},
    )
    x, y = 1500.0 * math.cos(math.radians(-20)), 1500.0 * math.sin(math.radians(-20))
    # The door-in-wall rule (2026-09-29) reads this as a pane: known, not yet decided.
    add(
        "limit",
        "post 5 ft away at 20 deg left, seen by both boards",
        post(x, y) + back_wall(),
        {"deg030": 1500.0 - 150.0, "deg330": 1500.0 - 150.0},
    )

    for ft in (2.0, 3.0, 4.0, 6.0):
        d = ft * FT
        add(
            "limit",
            f"OPEN doorway, {ft} ft (radar echoes the frames)",
            wall_across(d, DOOR_HALF_MM) + back_wall(),
            {
                "deg030": math.hypot(d, DOOR_HALF_MM),
                "deg330": math.hypot(d, DOOR_HALF_MM),
            },
        )
    for ft in (1.5, 3.0):
        d = ft * FT
        add(
            "limit",
            f"pane {ft} ft ahead seen by ONE board (off square)",
            back_wall(),
            {"deg030": d},
        )
    return out


GLASS_SAID = sg.BLOCKED_GLASS_ANY + (sg.GLASS_AHEAD,) + sg._GLASS_BESIDE


def _says(segments, echoes):
    walk_policy._last_measured.clear()
    walk_policy._near_frames.clear()
    walk_policy._remember_turn(None, 0.0)
    lidar, radar = lidar_state(segments), radar_state(echoes)
    return sg.decide(walk_policy.decide(lidar, radar), lidar, now=time.time(), radar_state=radar)[
        "instruction"
    ]


def door_grids():
    """Glass doors met at an angle, and with someone standing by them (design notes 24, 25):
    a line per grid, and one per miss that counts. Returns the failures."""
    failures = 0
    # Met at an angle, nobody near: found from 11 ft up to 20 deg off, and from 5 ft up to 45
    # (farther, the tilted wall's far side lies past the echo's match). 50 deg off, the door
    # leaves 0.58 m across the way, less than the walking corridor: its frame is on the path,
    # and within the block distance it must be a block, glass or not.
    must = 0
    late = {False: [0, 0], True: [0, 0]}  # steep (50 deg)? -> [named glass, scenes]
    for angle in (
        -50.0,
        -45.0,
        -40.0,
        -30.0,
        -25.0,
        -20.0,
        -15.0,
        -10.0,
        10.0,
        15.0,
        20.0,
        25.0,
        30.0,
        35.0,
        40.0,
        45.0,
        50.0,
    ):
        for ft in (1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.5, 5.0, 6.0, 8.0, 11.0):
            said = _says(*door_scene(ft, angle))
            if abs(angle) <= 20.0 or (abs(angle) <= 45.0 and ft <= 5.0):
                must += 1
                if said not in GLASS_SAID:
                    failures += 1
                    print(
                        f"{'MISSED':<12} {f'glass door {ft} ft, met {angle:+.0f} deg off':<62} -> {said!r}"
                    )
            else:
                row = late[abs(angle) > 45.0]
                row[0] += said in GLASS_SAID
                row[1] += 1
                warned = said in GLASS_SAID or sg._LEVEL.get(said, -1) >= sg._BLOCKED
                if abs(angle) > 45.0 and ft <= 4.0 and not warned:
                    failures += 1
                    print(
                        f"{'MISSED':<12} {f'glass door {ft} ft, met {angle:+.0f} deg off: no block':<62} -> {said!r}"
                    )
    print(
        f"{'ok' if not failures else 'MISSED':<12} {f'glass doors met 10-45 deg off: {must} must be named':<62} "
        f"-> {must - failures} named"
    )
    print(
        f"{'limit':<12} {'  25-45 deg off, 6-11 ft (named from 5 ft)':<62} -> {late[False][0]} of {late[False][1]} named"
    )
    print(
        f"{'limit':<12} {'  50 deg off, 1-11 ft, named glass (else a block within 4 ft)':<62} "
        f"-> {late[True][0]} of {late[True][1]} named"
    )
    # Someone standing by the door, 0.5-0.8 m along the wall, up to 0.4 m in front of it.
    # Walking in, it must be named by WALK_BLOCK_FT (4 ft); single scans nearer can miss.
    before, single, single_missed = failures, 0, 0
    for angle in (0.0, 10.0, -10.0, 20.0, -20.0):
        for along in (650.0, 850.0, -650.0, -850.0):
            for in_front in (0.0, 100.0, 250.0, 400.0):
                named = {}
                for ft in (8.0, 6.0, 4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5):
                    named[ft] = _says(*door_scene(ft, angle, (along, in_front))) in GLASS_SAID
                single += len(named)
                single_missed += sum(not n for n in named.values())
                if not any(named[ft] for ft in (8.0, 6.0, 4.5, 4.0)):
                    failures += 1
                    who = f"{along / 1000:+.2f} m along, {in_front / 1000:.2f} m in front"
                    print(
                        f"{'MISSED':<12} {f'door, someone {who}, {angle:+.0f} deg: not by 4 ft':<62} "
                        f"-> named at {[ft for ft, n in named.items() if n]}"
                    )
    approaches = 5 * 4 * 4
    print(
        f"{'ok' if failures == before else 'MISSED':<12} "
        f"{f'glass door, someone by it: {approaches} walks in, named by 4 ft':<62} "
        f"-> {approaches - (failures - before)} named"
    )
    print(
        f"{'limit':<12} {'  single scans 1.5-8 ft':<62} -> {single - single_missed} of {single} named"
    )
    return failures


def run(verbose=False):
    failures = 0
    for kind, name, segments, echoes in scenes():
        walk_policy._last_measured.clear()
        walk_policy._near_frames.clear()  # each scene stands alone: no radar frames before it
        walk_policy._remember_turn(None, 0.0)
        lidar, radar = lidar_state(segments), radar_state(echoes)
        wd = walk_policy.decide(lidar, radar)
        raw = sg.decide(wd, lidar, now=time.time(), radar_state=radar)
        said = raw["instruction"]
        named = said in sg.BLOCKED_GLASS_ANY + (sg.GLASS_AHEAD,) + sg._GLASS_BESIDE
        if kind == "glass":
            verdict = "ok" if named else "MISSED"
        elif kind == "clear":
            verdict = "ok" if not named else "FALSE GLASS"
        else:
            verdict = "glass" if named else "no glass"
        failures += verdict in ("MISSED", "FALSE GLASS")
        front = wd["directions"]["front"]
        print(
            f"{verdict:<12} {name:<62} -> {said!r}"
            + (f"   [front {front['distance_ft']} ft, {front['fusion']}]" if verbose else "")
        )
    return failures


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("-v", "--verbose", action="store_true", help="show the fused front reading")
    p.add_argument(
        "--no-grids", action="store_true", help="skip the door grids (design notes 24, 25)"
    )
    args = p.parse_args()
    failures = run(args.verbose)
    if not args.no_grids:
        print()
        failures += door_grids()
    print(f"\n{failures} failure(s)" if failures else "\nall scenes as expected")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
