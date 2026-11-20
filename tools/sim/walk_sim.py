#!/usr/bin/env python3
"""Walk into scenes at walking pace through the SPOKEN path: what is said, and how far off.

    venv/bin/python -m tools.sim.walk_sim              # every scene
    venv/bin/python -m tools.sim.walk_sim -k glass -v  # the glass scenes, every tick
    venv/bin/python -m tools.sim.walk_sim --stop       # stop at the first block, stand 4 s
    venv/bin/python -m tools.sim.walk_sim --turn-on glass-closing --noise 10 --seed 3
    venv/bin/python -m tools.sim.walk_sim --ghosts 0.5 --drop 0.2 --noise 10 --seed 2
    venv/bin/python -m tools.sim.walk_sim --follow     # rooms, the wearer doing what is said
    venv/bin/python -m tools.sim.walk_sim --faults     # a sensor failing on the way in
    venv/bin/python -m tools.sim.walk_sim --follow --record /tmp/walks   # as recordings
    venv/bin/python -m tools.sim.walk_sim --follow --turn-on movers --timing  # cost per stage
    venv/bin/python -m tools.sim.walk_sim --follow --wait 3 --side-camera-clear

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

--follow: rooms (walls and panes at any angle, posts: Room) the wearer walks through doing
what is said, REACT_S after it is said (_action): "turn left/right" a quarter turn, "turn
around" a half, "move left/right" a 0.3 m step, "turn slightly" 15-20 deg, "Stop" standing;
with --wait, after a turn they wait for "Walk straight" (at most that long). Checked: nothing
walked into, the way out of a dead end or a corner reached, no back-and-forth turning, no
"Stop, no clear way" where there is a way. A room failing by a known limit says so (`limit`).
(added 2026-11-18) Rooms with cars (`-k car`) have a front camera that sees them (Room.camera):
a car on a collision course warned of VEHICLE_LEAD_S ahead, none called approaching that is not
coming near; the wearer steps VEHICLE_STEP_MM aside on "Vehicle approaching, move ..." and
stands until "Walk straight" (2026-11-20)."""

import argparse
import contextlib
import math
import os
import random
import sys
import time
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
from app.perception import heading_tracker as ht  # noqa: E402
from app.perception.lidar_movers import get_movers  # noqa: E402
from app.perception.motion_tracker import get_motion_tracker  # noqa: E402
from app.perception.radar_movers import get_radar_movers  # noqa: E402

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


# name -> (scene, faults): a sensor failing on the way in (--faults; walk's `faults`).
FAULT_SCENES = {
    "LiDAR stalls 2 s, walking at a wall": (dict(ahead=START_MM), [("lidar", 2.0, 4.0)]),
    "LiDAR stalls 4 s on open floor": (dict(), [("lidar", 2.0, 6.0)]),
    "LiDAR sparse 3 s, walking at a wall": (dict(ahead=START_MM), [("lidar-sparse", 2.0, 5.0)]),
    "front radar lost, walking at a pane": (dict(glass=START_MM), [("radar-front", 1.0, 99.0)]),
    "front radar lost 2 s, walking at a pane": (dict(glass=START_MM),
                                                [("radar-front", 2.0, 4.0)]),
}


def judge_faults(scene, faults, ticks):
    """The rules broken with a sensor failing, as text: nothing "Walk straight" off a LiDAR
    that has stopped; a stall past SIMPLE_LIDAR_FRESH_S told ("Guidance stopped"), and the
    front radar's loss past SIMPLE_GLASS_LOSS_CONFIRM_S ("Glass sensor off", or a hazard
    standing); once scans come back, a block that is near now said within 0.5 s."""
    bad = []
    said = [(t, text) for t, _w, _l, _p, text, sp in ticks if sp]
    end = ticks[-1][0]
    for kind, a, b in faults:
        if kind == "lidar":
            during = [t for t, text in said if a <= t < b + 0.2 and text == sg.WALK_STRAIGHT]
            if during:
                bad.append(f"'Walk straight' {during[0]:.1f}s, the LiDAR stalled")
            told_by = a + c.SIMPLE_LIDAR_FRESH_S + 0.5
            if min(b, end) >= told_by and not any(
                a <= t <= told_by and text == sg.SENSORS_OFFLINE for t, text in said
            ):
                bad.append(f"a {b - a:.0f} s stall not told by {told_by:.1f}s")
            back = [(t, line) for t, _w, line, _p, _x, _s in ticks if t >= b]
            if back and back[0][1] is not None and back[0][1] < c.WALK_BLOCK_FT * FT:
                if not any(b <= t <= b + 0.5 and sg._LEVEL.get(text, -1) >= sg._BLOCKED
                           for t, text in said):
                    bad.append(f"back at {b:.1f}s inside the block, no block by {b + 0.5:.1f}s")
        if kind == "radar-front":
            told_by = a + c.SIMPLE_GLASS_LOSS_CONFIRM_S + 0.5
            if min(b, end) >= told_by:
                standing = [text for t, *_r, text, _sp in ticks if t <= told_by][-1]
                if not any(a <= t <= told_by and text == sg.GLASS_SENSOR_OFF for t, text in said) \
                        and standing == sg.WALK_STRAIGHT:
                    bad.append(f"the front radar lost from {a:.1f}s, not told by {told_by:.1f}s")
    return bad


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


def publish(laser, echoes, scan_index, now, radar, rnd, noise_mm, previous=None, drop=0.0,
            track=False, faults=()):
    """One LiDAR scan (`laser(bearing)`: mm) and, with `radar`, one frame of every board
    (`echoes(board bearing)`: its echoes, mm), stamped `now`, as scene_hunt.publish does but
    numbered (a block is confirmed on a second scan) and with the scan before it, as
    lidar_utils._publish_scan analyses it (a thin pole the laser's beams step over in one scan
    is held from the last). `drop`: the chance a board hears nothing in a frame (as
    glass_sim.walk_toward). `track`: the heading, mover and Kalman trackers follow this scan,
    as _publish_scan has them do. `faults` now: "lidar", no scan at all (a stall: the state
    keeps the last one); "lidar-sparse", 15% of its returns; "radar-front", both front boards
    offline. Returns this scan's points
    (None in a stall: the next is analysed without one, as past LIDAR_PERSIST_MAX_AGE_S)."""
    if "lidar" in faults:
        points = None
    else:
        points = _publish_lidar(laser, scan_index, now, rnd, noise_mm, previous, track,
                                keep=0.15 if "lidar-sparse" in faults else 1.0)
    if radar:
        for board in c.RADAR_BEARINGS.values():
            key = c.radar_board_key(board)
            if "radar-front" in faults and key in _front_boards():
                ru._publish(key, {"min_dist": None, "peaks_mm": [], "online": False,
                                  "degraded": False, "lag_s": None, "phase": "lost",
                                  "error": "no frames (walk_sim fault)"})
                continue
            heard = [] if drop and rnd.random() < drop else echoes(board)
            peaks = [round(p + rnd.gauss(0.0, noise_mm * 2.0), 1) for p in heard]
            ru._publish(key, {
                "min_dist": min(peaks) if peaks else None, "peaks_mm": sorted(peaks),
                "online": True, "degraded": False, "lag_s": 0.0, "phase": "streaming",
                "error": None})
    return points


def _front_boards():
    """The boards the front cardinal is made of, as the guidance reads them (review: not
    deg030 / deg330 by name, which a changed RADAR_BEARINGS would leave online)."""
    from app.sensors.radar_calibration import cardinal_sources

    return set(cardinal_sources().get("front") or ())


def _publish_lidar(laser, scan_index, now, rnd, noise_mm, previous, track, keep=1.0):
    raw = [(15, b, laser(b) + rnd.gauss(0.0, noise_mm))
           for b in (k * 360.0 / 520 for k in range(520))]
    swept = [a for _q, a, _d in raw]
    if keep < 1.0:  # "lidar-sparse": most returns lost (a hand over it, a dirty window)
        lose = random.Random(scan_index)  # its own draws: the seed's noise stays comparable
        raw = [p for p in raw if lose.random() < keep]
    scan = [p for p in raw if not lu._in_blind_arc(lu.correct_angle(p[1]))]
    points = [(lu.correct_angle(a), d) for _q, a, d in scan]
    update_lidar_state(
        {"angles": [round(a, 1) for a, _ in points], "distances": [int(d) for _, d in points]},
        lu.analyze_sectors(points, previous), c.LIDAR_RPM, scan_index,
        lu._mark_unswept(lu.analyze_cones(points, previous), swept),
        captured_at=now, valid_fraction=lu._valid_fraction(points, swept))
    if track:
        lu._update_movers(points, lu._step_heading(points, now), now)
    return points


@contextlib.contextmanager
def _scene_clock(t0, pace):
    """A fresh guidance state on a clock of the scene's own, from `t0`: yields {"t": now};
    `pace()` is the walking pace walk_policy reads (None standing or turning)."""
    clock = {"t": t0}
    sg.reset()
    wp._last_measured.clear()
    wp._remember_turn(None, 0.0)
    wp._near_frames.clear()
    wp._board_peaks.clear()
    lu._self_since.clear()
    ht._tracker = ht.HeadingTracker()  # no reset of its own
    get_movers().reset()
    get_motion_tracker().reset()
    get_radar_movers().reset()
    with contextlib.ExitStack() as stack:
        stack.enter_context(mock.patch("time.time", lambda: clock["t"]))
        stack.enter_context(mock.patch("time.monotonic", lambda: clock["t"]))
        stack.enter_context(mock.patch.object(wp, "_walking_pace", pace))
        yield clock


# The guidance output a recording keeps (unified_guidance._record).
_RECORDED_OUT = ("instruction", "speak", "cue_seq", "quiet_reason", "rule", "reason",
                 "mover_shadow", "route_shadow", "wall_shadow", "ground_shadow", "closing_shadow")


class Recording:
    """A walk written as the device's walk recorder writes one (app/core/walk_recorder: a
    gzipped JSON line per guidance tick, {"t", "mono", "in", "out"}), so tools/field/
    walk_replay.py and walk_report.py read it as a real walk (2026-11-20)."""

    def __init__(self, path):
        import gzip

        from app.core.walk_recorder import _plain

        self._plain = _plain
        self._fh = gzip.open(path, "wt", encoding="utf-8")

    def tick(self, now, inputs, got):
        import json

        out = {k: got.get(k) for k in _RECORDED_OUT}
        self._fh.write(json.dumps({"t": now, "mono": now, "in": inputs, "out": out},
                                  default=self._plain) + "\n")

    def mark(self, now, kind, text="", label="", said=""):
        import json

        from app.core.walk_recorder import _mark_line

        self._fh.write(json.dumps(_mark_line(now, now, kind, text, label, said)) + "\n")

    def close(self):
        self._fh.close()


def _tick(laser, echoes, k, now, t0, rnd, noise_mm, previous, cameras=None, drop=0.0,
          track=False, faults=(), record=None):
    """Scan k (the radar on its own rate), decided on as pi_voice's tick does: (sentence,
    spoken, this scan's points, a held "would say" of a feature not yet spoken). `cameras`:
    "clear", the side cameras showing nobody (scene_hunt.clear_cameras), else none. `track`:
    the moving-thing trackers too, and their snapshots and the wearer's motion to the
    guidance, as unified_guidance gives them."""
    radar = k == 0 or int((now - t0) * RADAR_HZ) != int((now - t0 - 1.0 / SCAN_HZ) * RADAR_HZ)
    previous = publish(laser, echoes, k + 1, now, radar, rnd, noise_mm, previous, drop, track,
                       faults)
    lidar, radar_state = get_lidar_state(), get_radar_state()
    camera = cameras if isinstance(cameras, dict) else sh.clear_cameras() if cameras == "clear" else {}
    decision = wp.decide(lidar, radar_state, camera)
    motion = movers = None
    if track:
        motion = ht.get_tracker().state()
        movers = {**get_movers().snapshot(), "kalman": get_motion_tracker().snapshot()}
        if c.RADAR_MOVERS_ENABLED:
            get_radar_movers().update(radar_state, movers.get("blobs"), movers.get("ego"),
                                      turn_dps=(motion or {}).get("yaw_rate_dps"),
                                      lidar_state=lidar)
            movers["radar"] = get_radar_movers().snapshot()
    got = sg.guidance(decision, lidar, now=now, radar_state=radar_state, camera_state=camera,
                      motion=motion, movers=movers)
    if record is not None:  # the device's inputs to that call (unified_guidance._record)
        record.tick(now, {"walk_decision": decision, "lidar": lidar, "radar": radar_state,
                          "camera": camera, "motion": motion, "movers": movers, "ground": None,
                          "hold": None, "hold_lidar_absent": False, "at_crossing": False}, got)
    shadow = got.get("mover_shadow") or got.get("route_shadow") or None
    return got.get("instruction"), bool(got.get("speak")), previous, shadow


def walk(scene, pace=1.2, stop=False, noise_mm=0.0, seed=0, t0=1000.0, ghosts=0.0, drop=0.0,
         faults=(), record=None):
    """[(t, walked mm, on-line mm, pane mm, sentence, spoken)] per scan, walking at `pace`
    m/s until something on the line is 0.3 m off (or 8 s); with `stop`, standing still
    from the first block warning spoken, for STAND_S. `ghosts`: the chance of a double-bounce
    ghost per wall echo (scene_hunt.board_echoes); `drop`: of a board hearing nothing.
    `faults`: (kind, from s, to s) a sensor fails in (publish). `record`: a Recording."""
    rnd = random.Random(seed)
    out, walked, stood_at, k, previous = [], 0.0, None, 0, None
    with _scene_clock(t0, lambda: None if stood_at else pace) as clock:
        while True:
            now = clock["t"]
            s = scene_at(scene, walked)
            line, pane = on_line_mm(s)
            nearest = min([d for d in (line, pane) if d is not None], default=None)
            if (nearest is not None and nearest < 300.0) or now - t0 > 8.0:
                break
            if stood_at is not None and now - stood_at > STAND_S:
                break
            text, spoken, previous, _shadow = _tick(
                lambda b, s=s: sh.laser_range(s, b),
                lambda b, s=s: sh.board_echoes(s, b, ghosts, rnd),
                k, now, t0, rnd, noise_mm, previous, drop=drop,
                faults={kind for kind, a, b in faults if a <= now - t0 < b}, record=record)
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


# ---------------------------------------------------------------- doing what is said

BODY_MM = 250.0  # the wearer's half-width: a surface nearer the middle than this is walked into
REACT_S = 0.5  # from a sentence spoken to the wearer acting on it
TURN_DPS = 90.0  # turning on the spot
STEP_MPS = 0.3  # a step to the side
FOLLOW_S = 25.0
STOOD_S = 5.0  # "Stop, no clear way" and nothing else said this long: given up
HOLD_S = 6.0  # "..., stop" for something coming: stood until "Walk straight", this long at most
MOVER_STEP_MM = c.ROUTE_STEP_ASIDE_M * 1000.0  # "..., move left": a step that far aside
CAR_LENGTH_MM = 4500.0  # a car mover's length (its `radius`: half its width)
VEHICLE_STEP_MM = 1500.0  # "Vehicle approaching, move left": a car wants more room than a person
VEHICLE_STEP_MPS = 1.0  # ... taken quickly, then stood until "Walk straight" (HOLD_S at most)
CAM_HZ = 10.0  # the front camera's detector frames, a second (route_sim's)
VEHICLE_LEAD_S = 2.0  # a vehicle warning this long before the vehicle would reach the wearer
VEHICLE_FAR_MM = 3000.0  # ... and none for a car whose side passes farther off than this
FLIPS_MAX = 2  # left, right, left: the third change of side is back and forth


class Room:
    """Walls and panes (x1, y1, x2, y2: the laser looks through a pane, the radar hears it)
    and round posts (x, y, radius), in the world frame: mm, x ahead of where the wearer
    starts, y to the right of it, headings clockwise from x. `goal(x, y)`: out of the room's
    trouble (None: nothing to reach, only nothing to walk into). `limit`: why it is not, a
    known limit (docs/known-limits.md): reported, not counted as a failure. `movers`: people
    walking straight on, (x, y, vx, vy, radius[, class]), mm and mm/s, where they are at `t` s;
    the class "person" unless given. A "car" is a box CAR_LENGTH_MM long and twice `radius`
    wide, lying along the way it drives (parked: along x), to the laser, radar and camera.
    `needs`: a feature not yet spoken (walk_replay.FEATURES) the room is about: off, the walk
    is shown and not counted; on (--turn-on), judged. `camera`: the front camera sees the
    movers (camera_state), and the side cameras show nobody."""

    def __init__(self, walls=(), glass=(), posts=(), goal=None, limit=None, movers=(),
                 needs=None, camera=False):
        self.limit = limit
        self.needs = needs
        self.camera = camera
        self.movers = [(f"mover{i}", tuple(m) + (("person",) if len(m) == 5 else ()))
                       for i, m in enumerate(movers)]
        self.t = 0.0
        self.segs = [(f"wall{i}", w, False) for i, w in enumerate(walls)]
        self.segs += [(f"glass{i}", g, True) for i, g in enumerate(glass)]
        self.posts = [(f"post{i}", p) for i, p in enumerate(posts)]
        self.goal = goal

    def circles(self):
        """The posts, and the people where they are at `t`."""
        return self.posts + [(n, (x + vx * self.t, y + vy * self.t, r))
                             for n, (x, y, vx, vy, r, cls) in self.movers if cls != "car"]

    def car_corners(self, m):
        """A car mover's four corners at `t`, round the box."""
        x, y, vx, vy, r, _cls = m
        cx, cy = x + vx * self.t, y + vy * self.t
        a = math.atan2(vy, vx) if (vx or vy) else 0.0
        ux, uy, hl = math.cos(a), math.sin(a), CAR_LENGTH_MM / 2.0
        return [(cx + ux * sl * hl - uy * sw * r, cy + uy * sl * hl + ux * sw * r)
                for sl, sw in ((1, 1), (1, -1), (-1, -1), (-1, 1))]

    def segments(self):
        """The walls and panes, and each car's sides where it is at `t` (named for the car)."""
        out = list(self.segs)
        for n, m in self.movers:
            if m[5] == "car":
                k = self.car_corners(m)
                out += [(n, k[i] + k[(i + 1) % 4], False) for i in range(4)]
        return out

    def camera_state(self, x, y, heading, rnd):
        """The cameras at `t`: the front one's detections of the movers, as boxes in its frame
        (route_sim.camera: a box's height by the class's height and its range, 5 % jitter),
        the side ones a fresh picture naming nobody (scene_hunt.clear_cameras)."""
        w, h = sg._QUAD_W, sg._QUAD_H
        focal = (w / 2.0) / math.tan(math.radians(sg.SIMPLE_CAMERA_HFOV_DEG) / 2.0)
        r_h = math.radians(heading)
        half = math.radians(sg.SIMPLE_CAMERA_HFOV_DEG) / 2.0

        def seen(px, py):  # (ahead, right) of the camera, m
            dx, dy = (px - x) / 1000.0, (py - y) / 1000.0
            return (dx * math.cos(r_h) + dy * math.sin(r_h),
                    -dx * math.sin(r_h) + dy * math.cos(r_h))

        dets = []
        for _n, (mx, my, vx, vy, _r, cls) in self.movers:
            if cls == "car":  # the box round its corners, cut off at the picture's sides
                pts = [seen(*p) for p in self.car_corners((mx, my, vx, vy, _r, cls))]
                if min(f for f, _r2 in pts) < 0.5:
                    continue
                brgs = [math.atan2(r2, f) for f, r2 in pts]
                lo, hi = max(min(brgs), -half), min(max(brgs), half)
                if lo >= hi:
                    continue
                hp = (focal * sg._object_height_m(cls) / min(math.hypot(*p) for p in pts)
                      * (1.0 + rnd.gauss(0.0, 0.05)))
                if hp >= h - 6.0:
                    continue
                x1 = w / 2.0 + focal * math.tan(lo) + (rnd.gauss(0.0, 2.0) if lo > -half else 0.0)
                x2 = w / 2.0 + focal * math.tan(hi) + (rnd.gauss(0.0, 2.0) if hi < half else 0.0)
                dets.append({"class_name": cls, "confidence": 0.9,
                             "bbox": [max(0.0, x1), h / 2 - hp / 2, min(w, x2), h / 2 + hp / 2]})
                continue
            fwd, right = seen(mx + vx * self.t, my + vy * self.t)
            if fwd < 0.5:
                continue
            brg = math.atan2(right, fwd)
            if abs(math.degrees(brg)) > sg.SIMPLE_CAMERA_HFOV_DEG / 2.0 - 2.0:
                continue
            hp = focal * sg._object_height_m(cls) / math.hypot(fwd, right) * (1.0 + rnd.gauss(0.0, 0.05))
            if hp >= h - 6.0:
                continue
            cx = w / 2.0 + focal * math.tan(brg) + rnd.gauss(0.0, 2.0)
            width = hp * 0.4
            dets.append({"class_name": cls, "confidence": 0.9,
                         "bbox": [cx - width / 2, h / 2 - hp / 2, cx + width / 2, h / 2 + hp / 2]})
        now = time.time()
        cams = sh.clear_cameras()
        cams["front"] = {"detections": dets, "num_objects": len(dets), "frame_time": now,
                         "simulated": False, "view": {"dark": False, "blurred": False}}
        return cams

    def hits(self, x, y, angle_deg, laser):
        """(name, range mm) of every surface on the ray from (x, y) at `angle_deg`."""
        a = math.radians(angle_deg)
        dx, dy = math.cos(a), math.sin(a)
        out = []
        for name, (x1, y1, x2, y2), glass in self.segments():
            if glass and laser:
                continue
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            qx, qy = x1 - x, y1 - y
            t = (qx * ey - qy * ex) / den
            u = (qx * dy - qy * dx) / den
            if t > 1.0 and 0.0 <= u <= 1.0:
                out.append((name, t))
        for name, (px, py, r) in self.circles():
            qx, qy = px - x, py - y
            proj = qx * dx + qy * dy
            d2 = qx * qx + qy * qy - proj * proj
            if proj > 0 and d2 < r * r:
                out.append((name, proj - math.sqrt(r * r - d2)))
        return out

    def laser(self, x, y, heading):
        return lambda b: min([d for _n, d in self.hits(x, y, heading + b, True)] + [12000.0])

    def echoes(self, x, y, heading, ghosts=0.0, rnd=None):
        """A board's echoes as scene_hunt.board_echoes makes them: across its beam every 2 deg
        the first surface on the ray (no hearing through a wall), each surface's nearest; and,
        with chance `ghosts` each, a double-bounce ghost at twice a wall's range."""
        half = int(c.RADAR_BEAM_H_DEG / 2)

        def board(bearing):
            best = {}
            for off in range(-half, half + 1, 2):
                hits = self.hits(x, y, heading + bearing + off, False)
                if hits:
                    name, d = min(hits, key=lambda h: h[1])
                    best[name] = min(best.get(name, 1e9), d)
            peaks = [d for d in best.values() if c.RADAR_START_MM <= d <= c.RADAR_END_MM]
            if ghosts:
                peaks += [2.0 * d for n, d in best.items() if n.startswith("wall")
                          and rnd.random() < ghosts and 2.0 * d <= c.RADAR_END_MM]
            return sorted(round(d, 1) for d in peaks)
        return board

    def clearance(self, x, y):
        """mm from (x, y) to the nearest surface, panes included."""
        best = 1e9
        for _n, (x1, y1, x2, y2), _g in self.segments():
            ex, ey = x2 - x1, y2 - y1
            n2 = ex * ex + ey * ey
            u = 0.0 if n2 == 0 else max(0.0, min(1.0, ((x - x1) * ex + (y - y1) * ey) / n2))
            best = min(best, math.hypot(x - (x1 + u * ex), y - (y1 + u * ey)))
        for _n, (px, py, r) in self.circles():
            best = min(best, math.hypot(x - px, y - py) - r)
        return best


def _corridor(half, end, glass_end=False, back=-6000.0):
    walls = [(back, -half, end, -half), (back, half, end, half)]
    across = [(end, -half, end, half)]
    return (walls + across, []) if not glass_end else (walls, across)


def _corner(side, limit=None):
    """A 1.8 m corridor turning 90 deg to `side` (-1 left, +1 right) at 4.1-5.9 m."""
    k = float(side)
    walls = [(-3000, -900 * k, 5900, -900 * k), (5900, 7000 * k, 5900, -900 * k),
             (-3000, 900 * k, 4100, 900 * k), (4100, 7000 * k, 4100, 900 * k)]
    return Room(walls=walls, goal=lambda x, y: y * k > 3000, limit=limit)


def _long(half):
    return [(-6000, -half, 40000, -half), (-6000, half, 40000, half)]


_dead = _corridor(900, 5000)
_dead_narrow = _corridor(500, 5000)
_glass_door = _corridor(900, 5000, glass_end=True)
ROOMS = {
    "dead end, 1.8 m corridor": Room(walls=_dead[0], goal=lambda x, y: x < -2000),
    "dead end, 1.0 m corridor": Room(walls=_dead_narrow[0], goal=lambda x, y: x < -2000),
    "glass door closing a 1.8 m corridor": Room(
        walls=_glass_door[0], glass=_glass_door[1], goal=lambda x, y: x < -2000,
        limit="a pane across a narrow passage is named only nearer than its walls "
              "(passes with --turn-on glass-closing)"),
    "corner, the way on to the left": _corner(
        -1, limit="a corner turning left: the left's room is the radar's alone, and it hears "
                  "the way on's near wall at its beam's edge"),
    "corner, the way on to the right": _corner(+1),
    "T junction": Room(
        walls=[(-3000, -900, 4100, -900), (-3000, 900, 4100, 900), (4100, -7000, 4100, -900),
               (4100, 900, 4100, 7000), (5900, -7000, 5900, 7000)],
        goal=lambda x, y: abs(y) > 3000),
    "wall across open floor": Room(walls=[(4000, -8000, 4000, 8000)],
                                   goal=lambda x, y: abs(y) > 3000),
    "pane across open floor": Room(glass=[(4000, -8000, 4000, 8000)],
                                   goal=lambda x, y: abs(y) > 3000),
    "post on open floor": Room(posts=[(4000, 0, 150)], goal=lambda x, y: x > 5000 or abs(y) > 2500),
    # People walking straight on, whatever the wearer does (the worst case: real people give
    # way too). r 0.22 m, as movers_sim's.
    # Off (as on the device), each is walked into: blocked as a post, at ~3.5 ft, closing
    # 2.4 m/s, inside the reaction (design note 33). On, each is got past (note 36).
    "someone coming head-on, 2.4 m corridor": Room(
        walls=[(-6000, -1200, 20000, -1200), (-6000, 1200, 20000, 1200)],
        movers=[(9000, 0, -1200, 0, 220)], goal=lambda x, y: x > 9000, needs="movers"),
    "someone coming head-on, open floor": Room(
        movers=[(9000, 0, -1200, 0, 220)], goal=lambda x, y: x > 9000, needs="movers"),
    "someone crossing from the left": Room(
        movers=[(4500, -4500, 0, 1200, 220)], goal=lambda x, y: x > 7000, needs="movers"),
    "someone crossing from the right": Room(
        movers=[(4500, 4500, 0, -1200, 220)], goal=lambda x, y: x > 7000, needs="movers"),
    "someone walking slowly ahead": Room(movers=[(3000, 0, 500, 0, 220)]),
    "someone standing ahead": Room(
        movers=[(4000, 0, 0, 0, 220)], goal=lambda x, y: x > 5000 or abs(y) > 2500),
    # Cars (CAR_LENGTH_MM boxes), seen by the front camera too: its vehicle warnings are
    # spoken. The car never gives way. Coming in from 20 deg off the walking line on a
    # collision course (it keeps that bearing), at 18 km/h: in the front camera's view all the
    # way. Crossing from the side at 14 km/h, it stays at 73 deg until it is in front.
    "car coming head-on": Room(movers=[(30000, 0, -5000, 0, 900, "car")], camera=True,
                               goal=lambda x, y: x > 12000),
    "car coming in from ahead-left": Room(movers=[(28700, -10400, -4540, 2090, 900, "car")],
                                          camera=True, goal=lambda x, y: x > 12000),
    "car coming in from ahead-right": Room(movers=[(28700, 10400, -4540, -2090, 900, "car")],
                                           camera=True, goal=lambda x, y: x > 12000),
    "car crossing from the left": Room(
        movers=[(7000, -22000, 0, 4000, 900, "car")], camera=True, needs="movers",
        goal=lambda x, y: x > 10000, limit="a car crossing from the side: outside the front camera until in front, and "
                "the LiDAR's movers follow nothing over 2 m wide (its side: 4.5 m) or 8 m off"),
    "car crossing from the right": Room(
        movers=[(7000, 22000, 0, -4000, 900, "car")], camera=True, needs="movers",
        goal=lambda x, y: x > 10000, limit="a car crossing from the side: outside the front camera until in front, and "
                "the LiDAR's movers follow nothing over 2 m wide (its side: 4.5 m) or 8 m off"),
    "car passing in the next lane": Room(movers=[(30000, 4000, -5000, 0, 900, "car")],
                                         camera=True, goal=lambda x, y: x > 12000),
    "car driving off ahead": Room(movers=[(8000, 0, 3000, 0, 900, "car")], camera=True,
                                  goal=lambda x, y: x > 12000),
    "car parked in the way": Room(movers=[(10000, 0, 0, 0, 900, "car")], camera=True,
                                  goal=lambda x, y: x > 12000),
    "car parked beside the way": Room(movers=[(9000, 2200, 0, 0, 900, "car")], camera=True,
                                      goal=lambda x, y: x > 12000),
    "someone standing ahead, seen": Room(movers=[(5000, 0, 0, 0, 220, "person")], camera=True,
                                         goal=lambda x, y: x > 6000 or abs(y) > 2500),
    "0.9 m doorway straight ahead": Room(
        walls=[(4000, -8000, 4000, -450), (4000, 450, 4000, 8000)], goal=lambda x, y: x > 5500,
        limit="an open doorway reads as a glass door (its frames echo alike)"),
    # Long corridors, to walk down veering (--veer): told in time, and not nagged.
    "long 1.8 m corridor": Room(walls=_long(900), goal=lambda x, y: x > 20000),
    "long 2.4 m corridor": Room(walls=_long(1200), goal=lambda x, y: x > 20000),
    "long 3 m corridor": Room(walls=_long(1500), goal=lambda x, y: x > 20000),
}


def _action(text):
    """What the wearer does on hearing `text`: ("turn", deg, + right), ("step", mm, + right),
    ("step", mm, m/s, "hold") (quickly aside, then stood: a vehicle), ("hold", None) (stood
    until "Walk straight"), ("stop",), ("walk",) (walk on, ending a turn), or None (an
    approach: carry on)."""
    way = sg._WAY_OUT.get(text) or {sg.TURN_LEFT: "left", sg.TURN_RIGHT: "right",
                                     sg.TURN_AROUND: "around"}.get(text)
    if way:
        return ("turn", {"left": -90.0, "right": 90.0, "around": 180.0}[way])
    if text == sg.STOP_NO_WAY:
        return ("stop",)
    if text in (sg.VEHICLE_FROM_LEFT, sg.VEHICLE_FROM_RIGHT):  # "..., move right" / "move left"
        return ("step", VEHICLE_STEP_MM * (1.0 if text == sg.VEHICLE_FROM_LEFT else -1.0),
                VEHICLE_STEP_MPS, "hold")
    if text == sg.VEHICLE_APPROACHING or text in sg.OBJECT_IN_FRONT.values():
        return ("hold", None)  # no room to step aside, or named in front with no way given
    if text == sg.VEHICLE_AHEAD:  # "be careful": walked on, the cane out (an approach)
        return None
    mover = sg._MOVER_INFO.get(text)
    if mover:  # "..., stop": until "Walk straight"; "..., move left": a step aside
        if mover["move"] is None:
            return ("hold", None)
        return ("step", MOVER_STEP_MM * (1.0 if mover["move"] == "right" else -1.0))
    steer = sg._STEER_INFO.get(text)  # the camera's way past what it names, and the planner's
    if steer and text not in (sg.TURN_SLIGHT_LEFT, sg.TURN_SLIGHT_RIGHT):
        if steer["turn"] is None:  # "Walk straight between them"
            return ("walk",)
        deg = 90.0 if steer["where"] == "in front" else 20.0  # "in front, turn left": as a block's
        return ("turn", deg if steer["turn"] == "right" else -deg)
    if text == sg.WALK_STRAIGHT:
        return ("walk",)
    step = {sg.NEAR_LEFT: 300.0, sg.NEAR_RIGHT: -300.0, sg.GLASS_LEFT: 300.0, sg.GLASS_RIGHT: -300.0}
    if text in step:
        return ("step", step[text])
    slight = {sg.DRIFT_LEFT: 15.0, sg.DRIFT_RIGHT: -15.0,
              sg.TURN_SLIGHT_LEFT: -20.0, sg.TURN_SLIGHT_RIGHT: 20.0}
    if text in slight:
        return ("turn", slight[text])
    return None


def follow(room, pace=1.2, seconds=FOLLOW_S, noise_mm=0.0, seed=0, t0=1000.0, wait_s=None,
           cameras=None, ghosts=0.0, drop=0.0, shadows=None, veer_dps=0.0, faults=(),
           record=None):
    """Walk the wearer through `room` from (0, 0) facing x, doing what is said REACT_S after
    it is said (_action): [(t, x, y, heading, sentence, spoken, clearance mm)] per scan,
    until the goal, a collision, STOOD_S stood at "Stop", or `seconds`. A turn said again
    the way the wearer is already turning is the same turn. `wait_s`: after a turn, stand
    until "Walk straight" (or anything else to do) is said, this long at most (after a quarter
    turn or more: a slight turn is walked on); None: walk on at once. A room with movers runs the trackers (_tick `track`); `shadows`, a list, gets
    (t, sentence) of each "would say" of a feature not yet spoken. `veer_dps`: walking, the
    wearer turns this much a second without knowing (+ right), as drift_sim's. `faults`,
    `record`: as walk's."""
    rnd = random.Random(seed)
    track = bool(room.movers)
    cam, cam_frame = None, None  # room.camera: the cameras' last frame, CAM_HZ a second
    x = y = heading = 0.0
    doing, pending, stood_at = ("walk",), None, None
    out, k, previous = [], 0, None
    dt = 1.0 / SCAN_HZ

    big_turn = {"on": False}  # the turn under way is a quarter or more, not "turn slightly"

    def after_turn(now):
        # Waiting for "Walk straight" after a quarter turn or more; "turn slightly" is walked
        # on (2026-11-20: waiting after each one, the veering walks never got anywhere).
        return ("wait", now) if wait_s and big_turn["on"] else ("walk",)

    with _scene_clock(t0, lambda: pace if doing[0] == "walk" else None) as clock:
        while clock["t"] - t0 <= seconds:
            now = clock["t"]
            room.t = now - t0
            if room.camera and int((now - t0) * CAM_HZ) != cam_frame:
                cam_frame, cam = int((now - t0) * CAM_HZ), room.camera_state(x, y, heading, rnd)
            text, spoken, previous, shadow = _tick(
                room.laser(x, y, heading), room.echoes(x, y, heading, ghosts, rnd), k, now, t0,
                rnd, noise_mm, previous, cam if room.camera else cameras, drop, track,
                {kind for kind, a, b in faults if a <= now - t0 < b}, record)
            if shadow and shadows is not None:
                shadows.append((now - t0, shadow))
            clear = room.clearance(x, y)
            out.append((now - t0, x, y, heading, text, spoken, clear))
            if clear < BODY_MM or (room.goal and room.goal(x, y)):
                break
            if stood_at is not None and now - stood_at > STOOD_S:
                break
            if spoken and _action(text) is not None:
                pending = (now + REACT_S, _action(text))
            if pending and now >= pending[0]:
                act, pending = pending[1], None
                same_turn = (act[0] == "turn" and doing[0] == "turn"
                             and (act[1] > 0) == (doing[1] > 0))
                if not same_turn:
                    doing = ("hold", now) if act[0] == "hold" else act
                    if act[0] == "turn":
                        big_turn["on"] = abs(act[1]) >= 90.0
                    stood_at = now if act[0] == "stop" else None
            if doing[0] == "wait" and now - doing[1] >= (wait_s or 0.0):
                doing = ("walk",)
            if doing[0] == "hold" and now - doing[1] >= HOLD_S:
                doing = ("walk",)
            if doing[0] == "walk":
                heading += veer_dps * dt
                r = math.radians(heading)
                x, y = x + pace * 1000.0 * dt * math.cos(r), y + pace * 1000.0 * dt * math.sin(r)
            elif doing[0] == "turn":
                d = math.copysign(min(abs(doing[1]), TURN_DPS * dt), doing[1])
                heading += d
                doing = ("turn", doing[1] - d) if abs(doing[1] - d) > 1e-6 else after_turn(now)
            elif doing[0] == "step":
                mps = doing[2] if len(doing) > 2 else STEP_MPS
                d = math.copysign(min(abs(doing[1]), mps * 1000.0 * dt), doing[1])
                r = math.radians(heading + 90.0)
                x, y = x + d * math.cos(r), y + d * math.sin(r)
                doing = (("step", doing[1] - d) + doing[2:] if abs(doing[1] - d) > 1e-6
                         else ("hold", now) if doing[3:] == ("hold",) else ("walk",))
            k += 1
            clock["t"] = now + dt
    return out


def _vehicle_passes(room, tick, pace=1.2, ahead_s=10.0):
    """[(seconds to its nearest, how near its box comes to the wearer's middle mm, seconds
    until it is within BODY_MM of it or None)] of each moving car from `tick`, both going on as
    they are (the wearer at `pace` m/s, facing as they face), over the next `ahead_s`."""
    t, x, y, h, *_r = tick
    wx, wy = pace * 1000.0 * math.cos(math.radians(h)), pace * 1000.0 * math.sin(math.radians(h))
    out = []
    for _n, (mx, my, vx, vy, r, cls) in room.movers:
        if cls != "car" or (vx == 0 and vy == 0):
            continue
        a = math.atan2(vy, vx)
        ux, uy = math.cos(a), math.sin(a)
        best, contact = None, None
        for i in range(int(ahead_s * 20) + 1):
            tau = i / 20.0
            dx = x + wx * tau - (mx + vx * (t + tau))
            dy = y + wy * tau - (my + vy * (t + tau))
            along, across = dx * ux + dy * uy, -dx * uy + dy * ux
            d = math.hypot(max(abs(along) - CAR_LENGTH_MM / 2.0, 0.0), max(abs(across) - r, 0.0))
            if best is None or d < best[1]:
                best = (tau, d)
            if contact is None and d < BODY_MM:
                contact = tau
        out.append(best + (contact,))
    return out


def _vehicle_lead(room, tick):
    """Seconds from `tick` until the soonest car on a collision course would reach the
    wearer, or None: none on one."""
    leads = [c for _t, _miss, c in _vehicle_passes(room, tick) if c is not None]
    return min(leads) if leads else None


def judge_follow(room, ticks, shadows=()):
    """The rules broken doing what was said, as text. `shadows`: follow's "would say"s."""
    bad = []
    cars = [m for _n, m in room.movers if m[5] == "car"]
    if cars:
        warned = [k for k, (*_r, text, sp, _c) in enumerate(ticks)
                  if sp and (text in sg._VEHICLE_APPROACH or text in sg._MOVER_INFO)]
        far = [k for k in warned
               if all(miss > VEHICLE_FAR_MM for _t, miss, _c in _vehicle_passes(room, ticks[k]))]
        if all(m[2] == 0 and m[3] == 0 for m in cars):
            if warned:
                bad.append(f"a parked car called approaching: {ticks[warned[0]][4]!r}")
        elif far:
            bad.append(f"a vehicle warning with no car coming near: {ticks[far[0]][4]!r}")
        elif any(_vehicle_lead(room, tick) is not None for tick in ticks):
            if not warned:
                bad.append("a car on a collision course, no vehicle warning")
            else:
                lead = _vehicle_lead(room, ticks[warned[0]])
                if lead is not None and lead < VEHICLE_LEAD_S:
                    bad.append(f"the vehicle warning {lead:.1f} s before the car would reach them")
    if room.movers and all(m[2] == 0 and m[3] == 0 for _n, m in room.movers):
        called = [text for *_r, text, sp, _c in ticks if sp and text in sg._MOVER_INFO]
        called += [text for _t, text in shadows if text in sg._MOVER_INFO]
        if called:
            bad.append(f"someone standing still called moving: {called[0]!r}")
    t, x, y, h, *_r, clear = ticks[-1]
    if clear < BODY_MM:
        bad.append(f"walked into something at {t:.1f}s, ({x / 1000:.1f}, {y / 1000:.1f}) m")
    elif room.goal and not room.goal(x, y):
        bad.append(f"not out after {t:.0f}s: at ({x / 1000:.1f}, {y / 1000:.1f}) m, facing {h % 360:.0f} deg")
    sides = []
    for *_r, text, sp, _c in ticks:
        act = _action(text) if sp else None
        if act and act[0] == "turn" and abs(act[1]) == 90.0:
            sides.append("right" if act[1] > 0 else "left")
    flips = sum(1 for a, b in zip(sides, sides[1:]) if a != b)
    if flips >= FLIPS_MAX:
        bad.append(f"turned back and forth: {' '.join(sides)}")
    if room.goal and any(sp and text == sg.STOP_NO_WAY for *_r, text, sp, _c in ticks):
        bad.append("said 'Stop, no clear way' where there is a way")
    return bad


def _recording(args, name):
    """A Recording of this scene under --record DIR, or None."""
    if not args.record:
        return None
    os.makedirs(args.record, exist_ok=True)
    slug = "".join(ch if ch.isalnum() else "-" for ch in name).strip("-")
    return Recording(os.path.join(args.record, f"sim-{slug}.jsonl.gz"))


def run_follow(args, names):
    failed = 0
    for i, (name, room) in enumerate(ROOMS.items()):
        if args.k not in name:
            continue
        shadows = []
        rec = _recording(args, name)
        with turned_on(names):
            ticks = follow(room, args.pace, noise_mm=args.noise, seed=args.seed,
                           t0=1000.0 + 100.0 * i, wait_s=args.wait, cameras=args.cameras,
                           ghosts=args.ghosts, drop=args.drop, shadows=shadows,
                           veer_dps=args.veer, record=rec)
        if rec is not None:
            rec.close()
        bad = judge_follow(room, ticks, shadows)
        off = room.needs and room.needs not in names
        limit = room.limit if bad and not off else None
        failed += bool(bad) and not limit and not off
        tag = "off  " if off and bad else "limit" if limit else "FAIL " if bad else " ok  "
        print(f"{tag} {name}"
              + (f"  ({room.needs} not spoken: --turn-on {room.needs})" if off and bad else "")
              + (f"  ({limit})" if limit else "")
              + ("  (a known limit, now passing)" if room.limit and not bad and not off else ""))
        for t, x, y, h, text, sp, clear in ticks:
            if sp or args.v:
                print(f"        {t:4.1f}s  ({x / 1000:5.1f},{y / 1000:5.1f}) m {h % 360:4.0f} deg"
                      f"  {clear / FT:4.1f} ft clear  {'🔊' if sp else '  '} {text}")
        for t, shadow in shadows:
            print(f"        {t:4.1f}s  👁 would say: {shadow}")
        for b in bad:
            print(f"      ✗ {b}")
    print(f"\n{failed} room(s) breaking a rule" if failed else "\nall rooms as expected")
    return failed


@contextlib.contextmanager
def _timing():
    """Each guidance stage's cost per call while the walks run, as a table at the end (--timing,
    2026-11-20): the scan's analysis, walk_policy.decide, simple_guidance.guidance and, in rooms
    with people, the trackers; not the simulator's own geometry. A guidance tick runs per
    LiDAR scan, ~23 a second: ~43 ms in all. (The first block of a run imports cv2 for the
    spatial scan, ~40 ms once: on the device the camera code and the BEV grid have by then.)"""
    import statistics
    import time as _t

    cost = {}

    def timed(name, fn):
        def wrap(*a, **k):
            t = _t.perf_counter_ns()
            try:
                return fn(*a, **k)
            finally:
                cost.setdefault(name, []).append((_t.perf_counter_ns() - t) / 1e6)
        return wrap

    with contextlib.ExitStack() as stack:
        for mod, attr, name in ((lu, "analyze_sectors", "scan: sectors"), (lu, "analyze_cones", "scan: cones"),
                                (wp, "decide", "walk_policy.decide"), (sg, "guidance", "sg.guidance"),
                                (lu, "_update_movers", "trackers")):
            stack.enter_context(mock.patch.object(mod, attr, timed(name, getattr(mod, attr))))
        yield
    q = lambda v, f: sorted(v)[min(len(v) - 1, int(f * len(v)))]  # noqa: E731
    print(f"\n{'stage':20} {'calls':>7} {'median':>8} {'p95':>8} {'p99':>8} {'max':>8}  ms")
    for name, v in cost.items():
        print(f"{name:20} {len(v):7d} {statistics.median(v):8.2f} {q(v, .95):8.2f} "
              f"{q(v, .99):8.2f} {max(v):8.2f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-k", default="", help="only the scenes whose name contains this")
    ap.add_argument("--pace", type=float, default=1.2, help="m/s (default 1.2)")
    ap.add_argument("--stop", action="store_true", help="stand at the first block for 4 s")
    ap.add_argument("--noise", type=float, default=0.0, help="laser range noise, mm (1 sigma)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--ghosts", type=float, default=0.0, metavar="P",
                    help="the chance of a radar double-bounce ghost per wall echo")
    ap.add_argument("--drop", type=float, default=0.0, metavar="P",
                    help="the chance a radar board hears nothing in a frame")
    ap.add_argument("-v", action="store_true", help="every scan, not just what is spoken")
    ap.add_argument("--timing", action="store_true",
                    help="each guidance stage's cost per call, as a table at the end")
    ap.add_argument("--record", metavar="DIR",
                    help="write each walk as the device records one (sim-<name>.jsonl.gz), for "
                         "tools/field/walk_replay.py and walk_report.py")
    ap.add_argument("--faults", action="store_true",
                    help="walk in with a sensor failing on the way (FAULT_SCENES)")
    ap.add_argument("--follow", action="store_true",
                    help="the rooms, the wearer doing what is said (turns, steps, stops)")
    ap.add_argument("--wait", type=float, default=None, metavar="S",
                    help="with --follow: after a turn, wait up to S s for 'Walk straight'")
    ap.add_argument("--veer", type=float, default=0.0, metavar="DEG_S",
                    help="with --follow: walking, the wearer veers this much a second (+ right)")
    ap.add_argument("--side-camera-clear", dest="cameras", action="store_const", const="clear",
                    help="with --follow: the side cameras show nobody there (as scene_hunt)")
    ap.add_argument("--turn-on", action="append", default=[], metavar="FEATURE",
                    choices=sorted(FEATURES) + ["all"],
                    help="walk with a not-yet-spoken feature on (as walk_replay --turn-on)")
    args = ap.parse_args()
    if args.timing:
        with _timing():
            return _main(args)
    return _main(args)


def _main(args):
    names = sorted(FEATURES) if "all" in args.turn_on else args.turn_on
    if args.follow:
        return 1 if run_follow(args, names) else 0
    if args.faults:
        failed = 0
        for i, (name, (scene, faults)) in enumerate(FAULT_SCENES.items()):
            if args.k not in name:
                continue
            rec = _recording(args, name)
            with turned_on(names):
                ticks = walk(scene, args.pace, False, args.noise, args.seed,
                             t0=1000.0 + 100.0 * i, faults=faults, record=rec)
            if rec is not None:
                rec.close()
            bad = judge_faults(scene, faults, ticks)
            failed += bool(bad)
            print(f"{'FAIL' if bad else ' ok '}  {name}  {faults}")
            for t, _w, line, pane, text, sp in ticks:
                if sp or args.v:
                    near = min([d for d in (line, pane) if d is not None], default=None)
                    where = f"{near / FT:5.1f} ft" if near is not None else "   -   "
                    print(f"        {t:4.1f}s  {where}  {'🔊' if sp else '  '} {text}")
            for b in bad:
                print(f"      ✗ {b}")
        print(f"\n{failed} scene(s) breaking a rule" if failed else "\nall scenes as expected")
        return 1 if failed else 0
    failed = 0
    for i, (name, scene) in enumerate(SCENES.items()):
        if args.k not in name:
            continue
        rec = _recording(args, name)
        with turned_on(names):
            ticks = walk(scene, args.pace, args.stop, args.noise, args.seed,
                         t0=1000.0 + 100.0 * i, ghosts=args.ghosts, drop=args.drop, record=rec)
        if rec is not None:
            rec.close()
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
