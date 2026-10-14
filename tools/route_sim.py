#!/usr/bin/env python3
"""The Kalman tracker and the route planner without leaving the desk.

Tracker scenes (utils/motion_tracker): synthetic scans from tools/movers_sim (the rig's
density, angle creep, range noise and back-left blind arc) plus a synthetic front camera.
Is each person or vehicle coming at the user caught, from the right side, its speed right,
and is nothing still ever called moving?

Planner scenes (utils/route_planner through walk_policy and simple_guidance.decide, with
ROUTE_PLANNER_SPEAK and LIDAR_MOVERS_SPEAK turned on for the test): does it go straight
when the way is clear, steer past a post, turn the open way in a corner, turn around in a
dead end, stay off glass, wait for a person crossing, and step aside from one coming?
"""

import argparse
import math
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import config                                              # noqa: E402
from tools import movers_sim as ms                         # noqa: E402
from tools import glass_sim as gs                          # noqa: E402
from utils import simple_guidance as sg                    # noqa: E402
from utils import walk_policy                              # noqa: E402
from utils.lidar_movers import MoverTracker                # noqa: E402
from utils.motion_tracker import MotionTracker, VEHICLE    # noqa: E402
from utils.route_planner import RoutePlanner, get_planner  # noqa: E402

FT = 304.8
CAM_HZ = 10.0


# ---------------------------------------------------------------- a synthetic front camera

def camera(user, psi, things, t, rng):
    """The front camera's detections of `things` ({"x","y","cls"}), as boxes in its frame."""
    w, h = sg._QUAD_W, sg._QUAD_H
    focal = (w / 2.0) / math.tan(math.radians(sg.SIMPLE_CAMERA_HFOV_DEG) / 2.0)
    dets = []
    for m in things:
        dx, dy = m["x"] - user[0], m["y"] - user[1]
        fwd = dx * math.cos(psi) + dy * math.sin(psi)
        right = -dx * math.sin(psi) + dy * math.cos(psi)
        if fwd < 0.5:
            continue
        brg = math.atan2(right, fwd)
        if abs(math.degrees(brg)) > sg.SIMPLE_CAMERA_HFOV_DEG / 2.0 - 2.0:
            continue
        rng_m = math.hypot(fwd, right)
        height = sg._object_height_m(m["cls"])
        hp = focal * height / rng_m * (1.0 + rng.gauss(0.0, 0.05))      # box height, 5 % jitter
        if hp >= h - 6.0:
            continue
        cx = w / 2.0 + focal * math.tan(brg) + rng.gauss(0.0, 2.0)
        width = hp * (2.5 if m["cls"] == "car" else 0.4)
        dets.append({"class_name": m["cls"], "confidence": 0.9,
                     "bbox": [cx - width / 2, h / 2 - hp / 2, cx + width / 2, h / 2 + hp / 2]})
    return {"front": {"detections": dets, "frame_time": t}, "last_update": t}


# ---------------------------------------------------------------- tracker scenes

def track_scene(seconds, movers=(), cam_things=(), walk=0.0, odometry=True, walls=ms.ROOM,
                still=ms.STILL, seed=1):
    """(first warning (t, word, mode, side), scans calling a still thing moving, speed errors)."""
    random.seed(seed)
    rng = random.Random(seed + 100)
    tracker, lidar_movers = MotionTracker(), MoverTracker()
    user, psi, phase = [0.0, 0.0], 0.0, 0.0
    ms_list = [dict(m) for m in movers]
    cams = [dict(c) for c in cam_things]
    first, false_moving, errors = None, 0, []
    t0 = time.time()
    next_cam = 0.0
    for i in range(int(seconds * ms.HZ)):
        t = i / ms.HZ
        heading = {"scan_yaw_deg": (ms.YAW_BIAS_DEG + random.gauss(0.0, 0.05)) if i else None}
        if odometry and i:
            heading["scan_fwd_mm"] = walk / ms.HZ * 1000.0 + random.gauss(0.0, 8.0)
            heading["scan_right_mm"] = random.gauss(0.0, 8.0)
        else:
            heading.update({"speed_mps": config.HEADING_NOMINAL_SPEED_MPS, "moving": walk > 0})
        pts = ms.scan(user, psi, phase, ms_list, walls, still, ms.BLIND)
        lidar_movers.update(pts, t0 + t, heading)
        cam = None
        if cams and t >= next_cam:
            next_cam = t + 1.0 / CAM_HZ
            cam = camera(user, psi, cams, t0 + t, rng)
        tracker.update(pts, t0 + t, heading, camera_state=cam,
                       ego=lidar_movers.snapshot().get("ego"))
        snap = tracker.snapshot()
        truth = [(m["x"] - user[0], m["y"] - user[1], m["vx"], m["vy"]) for m in ms_list]
        truth += [(c["x"] - user[0], c["y"] - user[1], c["vx"], c["vy"]) for c in cams]
        for tr in snap["tracks"]:
            near = min(truth, key=lambda q: math.hypot(q[0] - tr["x"], q[1] - tr["y"]), default=None)
            if near is None or math.hypot(near[0] - tr["x"], near[1] - tr["y"]) > 2.5:
                false_moving += 1
            elif math.hypot(near[2], near[3]) > 0.3:
                errors.append(math.hypot(tr["vx"] - near[2], tr["vy"] - near[3]))
            else:
                false_moving += 1                     # a still truth called moving
        if snap["threat"] and first is None:
            th = snap["threat"]
            first = (round(t, 2), th["word"], th["mode"], th["side"], th["ttc_s"])
        user[0] += walk / ms.HZ
        phase = (phase + ms.PRECESS_DEG) % 360.0
        for m in ms_list + cams:
            m["x"] += m["vx"] / ms.HZ
            m["y"] += m["vy"] / ms.HZ
    return first, false_moving, errors


def tracker_scenes():
    P = ms.person
    out = []

    def add(name, expect, **kw):
        out.append((name, expect, kw))

    add("still room, standing", None, seconds=6.0)
    add("cluttered room, walking 1.2 m/s with odometry", None, seconds=6.0, walk=1.2,
        still=ms.CLUTTER)
    add("cluttered room, walking 1.2 m/s, odometry lost (still things only)", None,
        seconds=6.0, walk=1.2, still=ms.CLUTTER, odometry=False)
    add("poles in a corridor, walking", None, seconds=8.0, walk=1.2, walls=ms.CORRIDOR,
        still=ms.POLES)
    add("person crossing from the right into the user's way", ("Person", "crossing", "right"),
        seconds=6.0, walk=1.1, movers=[P(5.0, 5.9, 0.0, -1.3)], walls=[])
    add("person crossing 4 m ahead of a user standing still (passes clear)", None,
        seconds=6.0, movers=[P(4.0, 3.5, 0.0, -1.3)])
    add("person crossing from the left while walking", ("Person", ("crossing", "toward"), "left"),
        seconds=6.0, walk=1.1, movers=[P(6.0, -7.1, 0.0, 1.3)], walls=[])
    add("person crossing 2 m clear ahead of a walking user (no warning)", None,
        seconds=6.0, walk=1.1, movers=[P(6.0, -4.0, 0.0, 1.3)], walls=[])
    add("person walking straight at the user", ("Person", "toward", None),
        seconds=6.0, movers=[P(7.0, 0.2, -1.3, 0.0)])
    add("person passing 2 m to the side (no warning)", None,
        seconds=7.0, movers=[P(7.0, 2.0, -1.3, 0.0)])
    add("car (camera only) coming at the user from ahead-left", ("Vehicle", "toward", "left"),
        seconds=5.0, cam_things=[{"x": 14.0, "y": -3.5, "vx": -5.0, "vy": 1.2, "cls": "car"}])
    add("car (camera only) driving past 9 m ahead (passes clear)", None,
        seconds=6.0, cam_things=[{"x": 9.0, "y": -16.0, "vx": 0.0, "vy": 6.0, "cls": "car"}])
    add("parked car (camera only) while walking past", None, seconds=7.0, walk=1.2,
        cam_things=[{"x": 8.0, "y": -2.5, "vx": 0.0, "vy": 0.0, "cls": "car"}])
    add("two cars passing each other ahead, user on the footpath (no warning)", None,
        seconds=7.0, walk=1.0,
        cam_things=[{"x": 14.0, "y": -12.0, "vx": 0.0, "vy": 6.0, "cls": "car"},
                    {"x": 16.5, "y": 14.0, "vx": 0.0, "vy": -6.0, "cls": "car"}])
    return out


def run_tracker(verbose=False):
    failures = 0
    for name, expect, kw in tracker_scenes():
        first, false_moving, errors = track_scene(**kw)
        got = first[1:4] if first else None
        if expect is None:
            ok = first is None and false_moving == 0
        else:
            modes = expect[1] if isinstance(expect[1], tuple) else (expect[1],)
            ok = got is not None and got[0] == expect[0] and got[1] in modes and (
                expect[2] is None or got[2] == expect[2])
        med = sorted(errors)[len(errors) // 2] if errors else None
        failures += not ok
        print(f"{'ok' if ok else 'FAIL':<5} tracker: {name:<70} -> "
              f"{(got + (f'{first[4]} s ahead, at {first[0]} s',)) if first else '(no warning)'}"
              + (f"  speed error median {med:.2f} m/s" if med is not None else "")
              + (f"  [{false_moving} false-moving scans]" if false_moving else ""))
    return failures


# ---------------------------------------------------------------- planner scenes

def blind(lidar):
    """Take out the returns in the rig's blind arcs, as on the real mount."""
    pts = lidar["points"]
    keep = [(a, d) for a, d in zip(pts["angles"], pts["distances"])
            if not any((lo <= a % 360.0 <= hi) for lo, hi in config.LIDAR_BLIND_ARCS_DEG)]
    return {**lidar, "points": {"angles": [a for a, _ in keep], "distances": [d for _, d in keep]}}


def person_track(x, y, vx, vy, word="Person"):
    """A moving track as motion_tracker's snapshot gives one."""
    from utils.motion_tracker import body
    rad, half = body(word, "car" if word == VEHICLE else None)
    return {"source": "kalman", "id": 1, "word": word, "cls": None, "x": x, "y": y,
            "vx": vx, "vy": vy, "pos_var": 0.01, "vel_var": 0.02, "rad": rad, "half": half,
            "moving": True, "speed": math.hypot(vx, vy), "speed_sd": 0.1, "width": 0.4,
            "sources": ["lidar"], "x_m": x, "y_m": y, "distance_m": math.hypot(x, y),
            "bearing_deg": math.degrees(math.atan2(y, x)), "vx_mps": vx, "vy_mps": vy,
            "own_speed_mps": math.hypot(vx, vy), "width_m": 0.4, "alert": None}


def planner_scenes():
    W, S, P, B = gs.wall_across, gs.side_wall, gs.post, gs.back_wall
    out = []

    def add(name, segments, echoes, expect, tracks=()):
        out.append((name, segments, echoes, expect, list(tracks)))

    add("open space", [], {}, {"said": sg.WALK_STRAIGHT})
    add("corridor 2 m wide, clear", S("left", 1000) + S("right", 1000) + B(),
        {"deg270": 1000.0, "deg090": 1000.0}, {"said": sg.WALK_STRAIGHT})
    add("post on the walking line, 2 m ahead", P(2000, 0) + B(), {"deg030": 1850, "deg330": 1850},
        {"said": (sg.TURN_SLIGHT_LEFT, sg.TURN_SLIGHT_RIGHT)})
    add("post just right of the line, 2 m ahead", P(2000, 250) + B(), {"deg030": 1850},
        {"said": sg.TURN_SLIGHT_LEFT})
    add("corner: wall ahead 0.6 m, wall hugging the left (blind arc)",
        W(600) + S("left", 400), {"deg030": 600, "deg330": 600, "deg270": 400},
        {"blocked_side": "right"})
    add("corner: wall ahead 0.6 m, wall hugging the right",
        W(600) + S("right", 400), {"deg030": 600, "deg330": 600, "deg090": 400},
        {"blocked_side": "left"})
    add("dead end: walls ahead and both sides close",
        W(700) + S("left", 450, x_to=700) + S("right", 450, x_to=700),
        {"deg030": 700, "deg330": 700, "deg270": 450, "deg090": 450},
        {"blocked_side": "around"})
    add("glass wall across 4 ft, room beyond (never steered into)", B(),
        {"deg030": 4 * FT, "deg330": 4 * FT},
        {"said": (sg.GLASS_AHEAD, sg.BLOCKED_GLASS)})
    add("car crossing 2.5 m ahead as the user gets there (stop)", [], {},
        {"avoid": True, "mover": sg.MOVER_CROSSING[("Vehicle", "right")]},
        tracks=[person_track(2.5, 8.0, 0.0, -5.0, word="Vehicle")])
    add("car crossing 4 m ahead, gone before the user gets there (walk on)", [], {},
        {"said": sg.WALK_STRAIGHT, "no_mover": True},
        tracks=[person_track(4.0, 6.0, 0.0, -5.0, word="Vehicle")])
    add("person crossing the way 2.5 m ahead (go round it or wait)", [], {},
        {"avoid": True}, tracks=[person_track(2.5, 1.5, 0.0, -1.0)])
    add("person coming straight at the user (not straight on)", [], {},
        {"avoid": True}, tracks=[person_track(3.0, 0.0, -1.4, 0.0)])
    add("1.2 m corridor, person coming head-on, no room to pass (stop)",
        S("left", 600) + S("right", 600) + B(), {"deg270": 600.0, "deg090": 600.0},
        {"mover": sg.MOVER_TOWARD_STOP["Person"]}, tracks=[person_track(2.0, 0.0, -1.4, 0.0)])
    add("person walking alongside, 1.5 m right (nothing to do)", [], {},
        {"said": sg.WALK_STRAIGHT, "no_mover": True},
        tracks=[person_track(0.5, 1.5, 1.2, 0.0)])
    return out


def run_planner(verbose=False):
    failures = 0
    speak = (sg.ROUTE_PLANNER_SPEAK, sg.LIDAR_MOVERS_SPEAK)
    sg.ROUTE_PLANNER_SPEAK = sg.LIDAR_MOVERS_SPEAK = True
    try:
        for name, segments, echoes, expect, tracks in planner_scenes():
            walk_policy._last_measured.clear()
            walk_policy._remember_turn(None, 0.0)
            get_planner().reset()
            now = time.time()
            lidar = blind(gs.lidar_state(segments))
            lidar["last_update"] = now
            radar = gs.radar_state(echoes)
            wd = walk_policy.decide(lidar, radar)
            kal = {"t": now, "tracks": tracks, "threat": None, "stale": False}
            motion = {"speed_mps": 1.1, "speed_trusted": True, "heading_deg": 0.0,
                      "updated_mono": time.monotonic(), "moving_confirmed": True}
            raw = sg.decide(wd, lidar, now=now, radar_state=radar, motion=motion,
                            movers={"kalman": kal, "stale": True})
            said, route = raw["instruction"], raw.get("route") or {}
            ok = bool(route.get("ok"))
            if "said" in expect:
                want = expect["said"] if isinstance(expect["said"], tuple) else (expect["said"],)
                ok = ok and said in want
            if "blocked_side" in expect:
                ok = ok and raw["blocked"] and raw["side"] == expect["blocked_side"]
            if "mover" in expect:
                want = expect["mover"] if isinstance(expect["mover"], tuple) else (expect["mover"],)
                ok = ok and raw.get("mover_text") in want
            if expect.get("no_mover"):
                ok = ok and not raw.get("mover_text")
            if expect.get("avoid"):
                ok = ok and (route.get("by_mover") or route.get("speed") in ("slow", "wait")
                             or route.get("step_aside"))
            failures += not ok
            print(f"{'ok' if ok else 'FAIL':<5} planner: {name:<66} -> {said!r}"
                  + (f" side {raw['side']}" if raw.get("blocked") else "")
                  + (f" mover {raw['mover_text']!r}" if raw.get("mover_text") else "")
                  + (f"  [route {route.get('head_deg')} deg, {route.get('speed')}, straight "
                     f"{route.get('straight_m')} m, blocker {route.get('blocker')}]"
                     if verbose or not ok else "")
                  + (f"  ERROR {route.get('error')}" if route.get("error") else ""))
    finally:
        sg.ROUTE_PLANNER_SPEAK, sg.LIDAR_MOVERS_SPEAK = speak
    return failures


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("-v", "--verbose", action="store_true", help="show each plan")
    args = p.parse_args()
    failures = run_tracker(args.verbose) + run_planner(args.verbose)
    print(f"\n{failures} failure(s)" if failures else "\nall scenes as expected")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
