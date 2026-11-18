#!/usr/bin/env python3
"""Walk into scenes at walking pace through the SPOKEN path: what is said, and how far off.

    venv/bin/python -m tools.sim.walk_sim              # every scene
    venv/bin/python -m tools.sim.walk_sim -k glass -v  # the glass scenes, every tick
    venv/bin/python -m tools.sim.walk_sim --stop       # stop at the first block, stand 4 s
    venv/bin/python -m tools.sim.walk_sim --turn-on glass-closing --noise 10 --seed 3

The scene hunt (tools/sim/scene_hunt) judges one poll of the decision per scene. What the
wearer hears also passes through time: a block is confirmed on a second scan, sentences are
held, settled and repeated by simple_guidance.guidance's speech gate. Here the wearer walks
straight at `--pace` through a scene (scene_hunt's: side walls, a wall ahead, round posts, a
pane only the radar hears), and every LiDAR scan (~23 a second, as on the rig) is decided on
by walk_policy.decide and simple_guidance.guidance, on the scene's clock (as
tools/field/walk_replay does); the radar boards report RADAR_UPDATE_RATE times a second. Each
spoken sentence is listed with how far the thing on the walking line was.

Checked, per scene: something on the walking line gets a block warning spoken before it is
within WALK_BLOCK_FT less 1 ft (the wearer's reaction and the voice); "Walk straight" is never
spoken with it within WALK_BLOCK_FT; no glass sentence where there is no glass; where nothing
is on the line, no block warning at all; and, with --stop, nothing said more than
STAND_MAX_SPOKEN times while standing at the block (chatter), and the way out not flipped.
(added 2026-11-18)"""

import argparse
import contextlib
import os
import random
import sys
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
os.chdir(Path(__file__).resolve().parents[2])

from app import config as c  # noqa: E402
import app.sensors.lidar_utils as lu  # noqa: E402
import app.sensors.radar_utils as ru  # noqa: E402
import app.guidance.simple_guidance as sg  # noqa: E402
from app.perception import walk_policy as wp  # noqa: E402
from app.core.shared_state import get_lidar_state, get_radar_state, update_lidar_state  # noqa: E402
from tools.sim import scene_hunt as sh  # noqa: E402
from tools.field.walk_replay import FEATURES, turned_on  # noqa: E402

FT = 304.8
SCAN_HZ = c.LIDAR_RPM / 60.0  # a guidance tick per scan, as pi_voice runs it
RADAR_HZ = c.RADAR_UPDATE_RATE
START_MM = 6000.0  # where the thing on the line starts
LATE_FT = c.WALK_BLOCK_FT - 1.0  # a block first said nearer than this is late
STAND_S = 4.0
STAND_MAX_SPOKEN = 2

# name -> scene (scene_hunt's: mm, x ahead, y right; posts (x, y, radius)). Things on the
# walking line start START_MM ahead.
SCENES = {
    "post ahead, 0.2 m": dict(posts=[(START_MM, 0, 100)]),
    "pole ahead, 5 cm": dict(posts=[(START_MM, 0, 25)]),
    "pillar ahead, 0.6 m": dict(posts=[(START_MM, 0, 300)]),
    "post ahead, 0.2 m off the middle": dict(posts=[(START_MM, 200, 100)]),
    "wall across": dict(ahead=START_MM),
    "wall across a 1.8 m corridor": dict(ahead=START_MM, left=900, right=900),
    "wall across a 1.2 m corridor": dict(ahead=START_MM, left=600, right=600),
    "post in a 2.4 m corridor": dict(left=1200, right=1200, posts=[(START_MM, 0, 100)]),
    "glass across open floor": dict(glass=START_MM),
    "glass across a 3 m hall": dict(glass=START_MM, left=1500, right=1500),
    "glass across a 2.4 m corridor": dict(glass=START_MM, left=1200, right=1200),
    "empty 1.8 m corridor": dict(left=900, right=900),
    "empty 1.2 m corridor": dict(left=600, right=600),
    "posts beside the way": dict(posts=[(3000, -800, 100), (5000, 800, 100)]),
    "open floor": dict(),
}


def scene_at(scene, walked):
    """The scene from where the wearer is, `walked` mm along."""
    s = {"left": None, "right": None, "ahead": None, "glass": None, "posts": [], **scene}
    return {
        **s,
        "ahead": None if s["ahead"] is None else s["ahead"] - walked,
        "glass": None if s["glass"] is None else s["glass"] - walked,
        "posts": [(x - walked, y, r) for x, y, r in s["posts"]],
    }


def on_line_mm(s):
    """(nearest solid on the walking line, the pane) mm, None for none."""
    line, _room = sh.truth(s)
    line = line if line < 1e8 and line > -1e3 else None
    glass = s["glass"] if s["glass"] is not None and s["glass"] > 0 else None
    return line, glass


def publish(s, scan_index, now, radar, rnd, noise_mm, previous=None):
    """One LiDAR scan (and, with `radar`, one frame of every board) of scene `s`, stamped
    `now`, as scene_hunt.publish does but numbered (a block is confirmed on a second scan)
    and with the scan before it, as lidar_utils._publish_scan analyses it (a thin pole the
    laser's beams step over in one scan is held from the last). Returns this scan's points."""
    raw = [(15, b, sh.laser_range(s, b) + rnd.gauss(0.0, noise_mm))
           for b in (k * 360.0 / 520 for k in range(520))]
    swept = [a for _q, a, _d in raw]
    scan = [p for p in raw if not lu._in_blind_arc(lu.correct_angle(p[1]))]
    points = [(lu.correct_angle(a), d) for _q, a, d in scan]
    update_lidar_state(
        {"angles": [round(a, 1) for a, _ in points], "distances": [int(d) for _, d in points]},
        lu.analyze_sectors(points, previous), c.LIDAR_RPM, scan_index,
        lu._mark_unswept(lu.analyze_cones(points, previous), swept),
        captured_at=now, valid_fraction=lu._valid_fraction(points, swept))
    if radar:
        for board in c.RADAR_BEARINGS.values():
            peaks = [round(p + rnd.gauss(0.0, noise_mm * 2.0), 1)
                     for p in sh.board_echoes(s, board, 0.0, rnd)]
            ru._publish(c.radar_board_key(board), {
                "min_dist": min(peaks) if peaks else None, "peaks_mm": sorted(peaks),
                "online": True, "degraded": False, "lag_s": 0.0, "phase": "streaming",
                "error": None})
    return points


def walk(scene, pace=1.2, stop=False, noise_mm=0.0, seed=0, t0=1000.0):
    """[(t, walked mm, on-line mm, pane mm, sentence, spoken)] per scan, walking at `pace`
    m/s until something on the line is 0.3 m off (or 8 s); with `stop`, standing still
    from the first block warning spoken, for STAND_S."""
    rnd = random.Random(seed)
    clock = {"t": t0}
    sg.reset()
    wp._last_measured.clear()
    wp._remember_turn(None, 0.0)
    wp._near_frames.clear()
    lu._self_since.clear()
    out, walked, stood_at, k, previous = [], 0.0, None, 0, None
    with contextlib.ExitStack() as stack:
        stack.enter_context(mock.patch("time.time", lambda: clock["t"]))
        stack.enter_context(mock.patch("time.monotonic", lambda: clock["t"]))
        stack.enter_context(mock.patch.object(wp, "_walking_pace", lambda: None if stood_at else pace))
        while True:
            now = clock["t"]
            s = scene_at(scene, walked)
            line, pane = on_line_mm(s)
            nearest = min([d for d in (line, pane) if d is not None], default=None)
            if (nearest is not None and nearest < 300.0) or now - t0 > 8.0:
                break
            if stood_at is not None and now - stood_at > STAND_S:
                break
            radar = int((now - t0) * RADAR_HZ) != int((now - t0 - 1.0 / SCAN_HZ) * RADAR_HZ) or k == 0
            previous = publish(s, k + 1, now, radar, rnd, noise_mm, previous)
            lidar, radar_state = get_lidar_state(), get_radar_state()
            decision = wp.decide(lidar, radar_state, {})
            got = sg.guidance(decision, lidar, now=now, radar_state=radar_state, camera_state={})
            text, spoken = got.get("instruction"), bool(got.get("speak"))
            out.append((now - t0, walked, line, pane, text, spoken))
            if stop and stood_at is None and spoken and sg._LEVEL.get(text, -1) >= sg._BLOCKED:
                stood_at = now
            k += 1
            clock["t"] = now + 1.0 / SCAN_HZ
            if stood_at is None:
                walked += pace * 1000.0 / SCAN_HZ
    return out


def judge(name, scene, ticks, stop):
    """The rules broken on this walk, as text."""
    bad = []
    blocks = [(t, line, pane, text) for t, _w, line, pane, text, sp in ticks
              if sp and sg._LEVEL.get(text, -1) >= sg._BLOCKED]
    has_line = any(line is not None for _t, _w, line, _p, _x, _s in ticks)
    has_pane = scene.get("glass") is not None
    if has_line or has_pane:
        if not blocks:
            bad.append("no block warning spoken")
        else:
            _t, line, pane, text = blocks[0]
            at = min(d for d in (line, pane) if d is not None) / FT
            if at < LATE_FT:
                bad.append(f"first block warning late: {at:.1f} ft ({text!r})")
    elif blocks:
        _t, line, _p, text = blocks[0]
        bad.append(f"a block warning with nothing on the walking line: {text!r}")
    for _t, _w, line, pane, text, sp in ticks:
        near = min([d for d in (line, pane) if d is not None], default=None)
        if sp and text == sg.WALK_STRAIGHT and near is not None and near < c.WALK_BLOCK_FT * FT:
            bad.append(f"'Walk straight' spoken {near / FT:.1f} ft from it")
            break
    if not has_pane and any(sp and text in sh.GLASS_WORDS for *_r, text, sp in ticks):
        bad.append("a glass sentence with no glass")
    if stop and blocks:
        stood = [(text, sp) for t, *_r, text, sp in ticks if t > blocks[0][0]]
        said = [text for text, sp in stood if sp]
        if len(said) > STAND_MAX_SPOKEN:
            bad.append(f"{len(said)} sentences while standing at the block: {said}")
        ways = {sg._WAY_OUT.get(text) for text, _sp in stood} - {None}
        if len(ways & {"left", "right"}) > 1:
            bad.append(f"the way out flipped while standing: {sorted(ways)}")
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-k", default="", help="only the scenes whose name contains this")
    ap.add_argument("--pace", type=float, default=1.2, help="m/s (default 1.2)")
    ap.add_argument("--stop", action="store_true", help="stand at the first block for 4 s")
    ap.add_argument("--noise", type=float, default=0.0, help="laser range noise, mm (1 sigma)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("-v", action="store_true", help="every scan, not just what is spoken")
    ap.add_argument("--turn-on", action="append", default=[], metavar="FEATURE",
                    choices=sorted(FEATURES) + ["all"],
                    help="walk with a not-yet-spoken feature on (as walk_replay --turn-on)")
    args = ap.parse_args()
    names = sorted(FEATURES) if "all" in args.turn_on else args.turn_on
    failed = 0
    for i, (name, scene) in enumerate(SCENES.items()):
        if args.k not in name:
            continue
        with turned_on(names):
            ticks = walk(scene, args.pace, args.stop, args.noise, args.seed, t0=1000.0 + 100.0 * i)
        bad = judge(name, scene, ticks, args.stop)
        failed += bool(bad)
        print(f"{'FAIL' if bad else ' ok '}  {name}")
        for t, _walked, line, pane, text, sp in ticks:
            if sp or args.v:
                near = min([d for d in (line, pane) if d is not None], default=None)
                where = f"{near / FT:5.1f} ft" if near is not None else "   -   "
                print(f"        {t:4.1f}s  {where}  {'🔊' if sp else '  '} {text}")
        for b in bad:
            print(f"      ✗ {b}")
    print(f"\n{failed} scene(s) breaking a rule" if failed else "\nall scenes as expected")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
