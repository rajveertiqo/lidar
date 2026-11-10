#!/usr/bin/env python3
"""Random scenes through the real decision path, checked two ways (added 2026-11-02).

    venv/bin/python -m tools.sim.scene_hunt mirror -n 400      # a scene and its mirror image
    venv/bin/python -m tools.sim.scene_hunt rules -n 1500      # safety rules vs the true geometry
    venv/bin/python -m tools.sim.scene_hunt rules --ghosts 0.5 # radar double-bounce ghosts too

A scene: a corridor (independent left/right walls), maybe a wall ahead, 0-3 round posts,
maybe a glass pane across the path that only the radar hears. The LiDAR scan goes through
the real sector/cone analysis; each radar board lists every surface in its beam (as the A121
lists its peaks), nearest first. Decisions: walk_policy.decide() and simple_guidance.decide()
— one poll, no smoothing.

mirror: the scene mirrored left <-> right must get the mirrored sentence ("Turn left" <->
"Turn right"). The LiDAR's blind arc is one-sided by design, so it is switched off here.
rules: with the rig's blind arc, against the scene's TRUE geometry: a solid within 4 ft on
the walking line gets a block warning; within 8 ft, never "Walk straight"; glass within 4 ft,
never "Walk straight"; "Turn left/right" only with 2 ft of room that way; no glass sentence
with no glass (a false alarm)."""

import argparse
import collections
import math
import os
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
os.chdir(Path(__file__).resolve().parents[2])

from app import config as c  # noqa: E402
import app.sensors.lidar_utils as lu  # noqa: E402
import app.sensors.radar_utils as ru  # noqa: E402
import app.perception.route_planner as rp  # noqa: E402
import app.guidance.simple_guidance as sg  # noqa: E402
from app.perception import walk_policy as wp  # noqa: E402
from app.core.shared_state import get_lidar_state, get_radar_state, update_lidar_state  # noqa: E402

FT = 304.8
GLASS_HALF_MM = 600.0  # a pane 1.2 m wide, centred on the path
GLASS_WORDS = (sg.GLASS_AHEAD,) + sg.BLOCKED_GLASS_ANY + sg._GLASS_BESIDE


def random_scene(rnd):
    return {
        "left": rnd.choice([None, rnd.uniform(200, 3000)]),
        "right": rnd.choice([None, rnd.uniform(200, 3000)]),
        "ahead": rnd.choice([None, None, rnd.uniform(400, 5000)]),
        "posts": [(rnd.uniform(300, 4000), rnd.uniform(-1500, 1500), rnd.uniform(40, 300))
                  for _ in range(rnd.randint(0, 3))],
        "glass": rnd.choice([None, None, None, rnd.uniform(400, 3500)]),
    }


def mirror(scene):
    return {**scene, "left": scene["right"], "right": scene["left"],
            "posts": [(x, -y, r) for x, y, r in scene["posts"]]}


def surfaces_along(scene, bearing, laser=True):
    """(name, range mm) of every surface on one ray: x ahead, y right, bearing clockwise."""
    a = math.radians(bearing)
    dx, dy = math.cos(a), math.sin(a)
    out = []
    if dy > 1e-9 and scene["right"]:
        out.append(("right", scene["right"] / dy))
    if dy < -1e-9 and scene["left"]:
        out.append(("left", scene["left"] / -dy))
    if dx > 1e-9 and scene["ahead"]:
        out.append(("ahead", scene["ahead"] / dx))
    if dx < -1e-9:
        out.append(("behind", 6000.0 / -dx))
    for k, (px, py, r) in enumerate(scene["posts"]):
        proj = px * dx + py * dy
        d2 = px * px + py * py - proj * proj
        if proj > 0 and d2 < r * r:
            out.append((f"post{k}", proj - math.sqrt(r * r - d2)))
    if not laser and scene["glass"] and dx > 1e-9 and abs(scene["glass"] / dx * dy) <= GLASS_HALF_MM:
        out.append(("glass", scene["glass"] / dx))
    return out


def laser_range(scene, bearing):
    return min([d for _n, d in surfaces_along(scene, bearing)] + [12000.0])


def board_echoes(scene, board, ghosts, rnd):
    """Every surface in a board's beam (its nearest return), nearest first — and, with
    probability `ghosts` each, a double-bounce ghost at twice a wall's range."""
    best = {}
    for off in range(-int(c.RADAR_BEAM_H_DEG / 2), int(c.RADAR_BEAM_H_DEG / 2) + 1, 2):
        hits = surfaces_along(scene, board + off, laser=False)
        if hits:  # the first surface on the ray: the radar does not hear through a wall
            name, d = min(hits, key=lambda h: h[1])
            best[name] = min(best.get(name, 1e9), d)
    walls_seen = [d for n, d in best.items() if n in ("left", "right", "ahead")]
    peaks = [d for d in best.values() if c.RADAR_START_MM <= d <= c.RADAR_END_MM]
    peaks += [2.0 * d for d in walls_seen if rnd.random() < ghosts and 2.0 * d <= c.RADAR_END_MM]
    # All peaks, nearest first, but the nearest LiDAR-shadowed ones too: what the board hears
    # it reports, laser or not.
    return sorted(round(p, 1) for p in peaks)


def publish(scene, blind, ghosts=0.0, rnd=None):
    rnd = rnd or random.Random(0)
    lu._previous_scan = None
    lu._self_since.clear()
    raw = [(15, b, laser_range(scene, b)) for b in (k * 360.0 / 520 for k in range(520))]
    swept = [a for _q, a, _d in raw]
    scan = [p for p in raw if not (blind and lu._in_blind_arc(lu.correct_angle(p[1])))]
    points = [(lu.correct_angle(a), d) for _q, a, d in scan]
    update_lidar_state(
        {"angles": [round(a, 1) for a, _ in points], "distances": [int(d) for _, d in points]},
        lu.analyze_sectors(points), 900.0, 1, lu._mark_unswept(lu.analyze_cones(points), swept),
        captured_at=time.time(), valid_fraction=lu._valid_fraction(points, swept))
    for board in c.RADAR_BEARINGS.values():
        peaks = board_echoes(scene, board, ghosts, rnd)
        ru._publish(c.radar_board_key(board), {
            "min_dist": peaks[0] if peaks else None, "peaks_mm": peaks, "online": True,
            "degraded": False, "lag_s": 0.0, "phase": "streaming", "error": None})


def decide(scene, blind=True, ghosts=0.0, rnd=None):
    publish(scene, blind, ghosts, rnd)
    wp._last_turn.update(direction=None, at=0.0)
    wp._last_measured.clear()
    # Each scene stands alone (as in glass_sim): the near-echo steadiness check otherwise
    # judged a scene's pane against the radar frames of the ~150 scenes before it, and the
    # verdict turned on how fast the run went (2026-11-10).
    wp._near_frames.clear()
    lidar, radar = get_lidar_state(), get_radar_state()
    walk = wp.decide(lidar, radar)
    return walk, sg.decide(walk, lidar, time.time(), radar_state=radar, camera_state={})


def truth(scene):
    """(on-line distance of the nearest solid ahead, true side rooms), mm."""
    line = scene["ahead"] or 1e9
    for px, py, r in scene["posts"]:
        if px > 0 and abs(py) - r <= c.GUIDANCE_CORRIDOR_HALF_WIDTH_MM - 50.0:  # clearly on it
            line = min(line, px - r)
    room = {s: min(laser_range(scene, b + o) for o in range(-5, 6))
            for s, b in (("right", 90.0), ("left", 270.0))}
    return line, room


def _flip(text):
    return text.replace("left", "\0").replace("right", "left").replace("\0", "right")


def _describe(scene):
    r = lambda v: v and round(v)  # noqa: E731
    return (f"left {r(scene['left'])} right {r(scene['right'])} ahead {r(scene['ahead'])} "
            f"glass {r(scene['glass'])} posts {[(round(x), round(y), round(q)) for x, y, q in scene['posts']]}")


def run_mirror(n, seed):
    for mod in (lu, wp, rp):
        mod.LIDAR_BLIND_ARCS_DEG = ()  # one-sided by design: off, so scenes can mirror
    rnd, bad = random.Random(seed), []
    for i in range(n):
        scene = random_scene(rnd)
        a = decide(scene, blind=False)[1]["instruction"]
        b = decide(mirror(scene), blind=False)[1]["instruction"]
        if _flip(a) != b:
            bad.append((i, a, b, scene))
    print(f"{n} scenes, {len(bad)} not mirrored")
    for i, a, b, scene in bad[:20]:
        print(f"  #{i:4d} {a!r:40s} | mirror: {b!r}\n         {_describe(scene)}")
    return len(bad)


def run_rules(n, seed, ghosts):
    rnd, said = random.Random(seed), collections.Counter()
    bad = collections.defaultdict(list)
    for i in range(n):
        scene = random_scene(rnd)
        _walk, raw = decide(scene, True, ghosts, rnd)
        text, level = raw["instruction"], sg._LEVEL.get(raw["instruction"], -1)
        said[text] += 1
        line, room = truth(scene)
        glass = scene["glass"] if scene["glass"] and scene["glass"] < line else None
        m = 60.0
        if text not in sg.ALLOWED:
            bad["not in the vocabulary"].append((i, text, scene))
        if line < c.WALK_BLOCK_FT * FT - m and level < sg._BLOCKED:
            bad["solid on the line < 4 ft, no block warning"].append((i, text, scene))
        if line < c.SIMPLE_OBSTACLE_AHEAD_FT * FT - m and text == sg.WALK_STRAIGHT:
            bad["solid on the line < 8 ft, 'Walk straight'"].append((i, text, scene))
        if glass and glass < c.WALK_BLOCK_FT * FT - m and text == sg.WALK_STRAIGHT:
            bad["glass < 4 ft ahead, 'Walk straight'"].append((i, text, scene))
        if not scene["glass"] and text in GLASS_WORDS:
            bad["no glass, a glass sentence (false alarm)"].append((i, text, scene))
        for side, turn in (("left", sg.TURN_LEFT), ("right", sg.TURN_RIGHT)):
            if (text == turn or sg._WAY_OUT.get(text) == side) and room[side] < c.WALK_MIN_TURN_FT * FT - m:
                bad[f"'{text}' with < 2 ft of room that way"].append((i, text, scene))
    print(f"{n} scenes{f', radar ghosts {ghosts:.0%}' if ghosts else ''}. Said: "
          + ", ".join(f"{t!r} {k}" for t, k in said.most_common(6)))
    print("  no rule broken" if not bad else "")
    for rule, cases in bad.items():
        print(f"  {rule}: {len(cases)}")
        for i, text, scene in cases[:3]:
            print(f"     #{i} said {text!r}  |  {_describe(scene)}")
    return sum(len(v) for v in bad.values())


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("check", choices=("mirror", "rules"))
    ap.add_argument("-n", type=int, default=400)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--ghosts", type=float, default=0.0, help="chance of a double-bounce ghost per wall echo")
    args = ap.parse_args()
    if args.check == "mirror":
        found = run_mirror(args.n, args.seed)
    else:
        found = run_rules(args.n, args.seed, args.ghosts)
    return 1 if found else 0


if __name__ == "__main__":
    sys.exit(main())
