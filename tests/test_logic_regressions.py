"""Decision-logic regressions found by the mirror test and fuzzing (2026-10-30). Hardware-free;
run with venv/bin/python -m unittest tests.test_logic_regressions."""

import contextlib
import math
import os
import time
import unittest
import unittest.mock

from app.perception import drift_guidance, walk_policy
from app.guidance import simple_guidance as sg


def _ranges(ahead_mm, post):
    """(bearing, range) every 0.7 deg across the front: a wall ahead and one round post."""
    px, py, r = post
    out = []
    for k in range(-128, 129):
        b = k * 0.7
        a = math.radians(b)
        dx, dy = math.cos(a), math.sin(a)
        d = ahead_mm / dx if dx > 0.2 else 8000.0
        proj = px * dx + py * dy
        d2 = px * px + py * py - proj * proj
        if proj > 0 and d2 < r * r:
            d = min(d, proj - math.sqrt(r * r - d2))
        out.append((b % 360.0, d))
    return out


class SurfaceSplitTests(unittest.TestCase):
    """A post in front of a wall is the same object whichever side of the path it stands:
    the split was judged against the previous return, so sweeping toward the post merged it
    into the wall behind ("Wall ahead" for a 0.22 m post) and sweeping away did not."""

    def kinds(self, post):
        surfaces = drift_guidance.fit_surfaces(_ranges(2583.0, post))
        near = [s for s in surfaces if math.hypot(*s["centroid_mm"]) < 2300.0]
        return [s["kind"] for s in near]

    def test_post_is_an_object_on_either_side(self):
        right = self.kinds((2008.0, 322.0, 109.0))
        left = self.kinds((2008.0, -322.0, 109.0))
        self.assertEqual(right, ["object"])
        self.assertEqual(left, ["object"])


class LaserPointHardeningTests(unittest.TestCase):
    """A None angle, or lists of different lengths, no longer make the walk decision unknown."""

    def state(self, angles, dists):
        return {"points": {"angles": angles, "distances": dists}, "last_update": time.time()}

    def test_bad_pairs_are_dropped(self):
        a, d = walk_policy._laser_points(self.state([0.0, None, 2.0, float("nan")], [1000, 1100, None, 900]))
        self.assertEqual((a, d), ([0.0], [1000.0]))
        a, d = walk_policy._laser_points(self.state([0.0, 1.0, 2.0], [1000, 1100]))
        self.assertEqual(len(a), len(d))

    def test_glass_checks_survive_them(self):
        st = self.state([0.0, None, 1.0] + [float(i) for i in range(2, 50)],
                        [900, 950, 960] + [1000] * 40)  # a None angle, and a shorter list
        walk_policy._laser_explains(st, 30.0, 1000.0)
        walk_policy._matches_ahead(st, 3.0)


class TurnAwayFromObstacleTests(unittest.TestCase):
    """Room enough both ways: the turn goes away from the side the obstacle is on (it was a
    fixed order: on the rig always "Turn right", with the obstacle on the right too)."""

    def views(self, left=20.0, right=20.0):
        return {d: {"direction": d, "distance_ft": ft, "unknown": False, "open": False}
                for d, ft in (("left", left), ("right", right), ("back", 10.0))}

    def test_away_from_the_obstacle(self):
        self.assertEqual(walk_policy.choose_turn(self.views(), 2.5, obstacle_deg=20.0)["direction"], "left")
        self.assertEqual(walk_policy.choose_turn(self.views(), 2.5, obstacle_deg=-20.0)["direction"], "right")

    def test_held_turn_kept_and_near_ahead_not_judged(self):
        self.assertEqual(walk_policy.choose_turn(self.views(), 2.5, "right", obstacle_deg=20.0)["direction"], "right")
        self.assertEqual(walk_policy.choose_turn(self.views(18.0, 20.0), 2.5, obstacle_deg=1.0)["direction"], "right")

    def test_little_room_one_way_still_goes_by_room(self):
        self.assertEqual(walk_policy.choose_turn(self.views(5.0, 20.0), 2.5, obstacle_deg=-20.0)["direction"], "right")

    def test_the_rig_says_it(self):
        from tools.sim import scene_hunt as sh

        post = {"left": None, "right": None, "ahead": None, "glass": None, "posts": [(800.0, 300.0, 130.0)]}
        # Named since 2026-11-09: a post in front is not "Turn left" alone.
        self.assertEqual(sh.decide(post)[1]["instruction"], "Obstacle in front, turn left")
        self.assertEqual(sh.decide(sh.mirror(post))[1]["instruction"], "Obstacle in front, turn right")


class GlassBehindNearerEchoTests(unittest.TestCase):
    """A pane the radar hears behind a nearer echo the laser explains (2026-11-02)."""

    def said(self, scene, ghosts=0.0):
        import random

        from tools.sim import scene_hunt as sh

        return sh.decide(scene, ghosts=ghosts, rnd=random.Random(0))[1]["instruction"]

    def test_pane_at_a_side_posts_range(self):
        scene = {"left": None, "right": 2667.0, "ahead": None, "glass": 836.0,
                 "posts": [(3417.0, -135.0, 258.0), (3597.0, 1413.0, 196.0), (528.0, -776.0, 91.0)]}
        self.assertIn(self.said(scene), sg.BLOCKED_GLASS_TURN.values())  # with the way out
        self.assertEqual(self.said({**scene, "glass": None}), "Walk straight")

    def test_pane_behind_a_side_walls_echo(self):
        scene = {"left": 1227.0, "right": None, "ahead": None, "glass": 1084.0, "posts": []}
        self.assertIn(self.said(scene), sg.BLOCKED_GLASS_TURN.values())

    def test_double_bounce_ghosts_are_not_glass(self):
        corridor = {"left": 757.0, "right": 1067.0, "ahead": 3951.0, "glass": None, "posts": []}
        self.assertNotIn(self.said(corridor, ghosts=1.0), (sg.GLASS_AHEAD,) + sg.BLOCKED_GLASS_ANY)


class BlindSideGlassTests(unittest.TestCase):
    """No glass where the laser cannot look (LIDAR_BLIND_ARCS_DEG): every echo on the blind
    left read as glass, "left · glass" on the dashboard all the time on the desk (2026-11-05)."""

    def test_echo_on_the_blind_side_is_an_obstacle_not_glass(self):
        from tools.sim import scene_hunt as sh

        # A post straight to the left, 1.6 ft: only the blind arc sees it (a wall would also
        # show in the laser's front-left, and that match cleared it before the fix too).
        for post in ((0.0, -600.0, 100.0), (-200.0, -500.0, 100.0)):
            walk, _ = sh.decide({"left": None, "right": None, "ahead": None,
                                 "glass": None, "posts": [post]})
            left = walk["directions"]["left"]
            self.assertTrue(left["lidar_blind"])
            self.assertTrue(left["radar_measured"])  # the radar heard the post
            self.assertEqual(left["distance_ft"], left["radar_ft"])  # and it still counts
            self.assertFalse(left["glass"])

    def test_glass_where_the_laser_looks_is_still_glass(self):
        self.assertTrue(walk_policy._is_glass(None, 3.0))  # the rule itself is unchanged
        self.assertFalse(walk_policy.lidar_blind("right"))
        self.assertTrue(walk_policy.lidar_blind("left"))


class RadarEchoStrengthTests(unittest.TestCase):
    """Every echo a board lists keeps its own strength (peaks_db, same order as peaks_mm), from
    the board's row through the cardinal's (2026-11-02; only the nearest's was kept)."""

    def tearDown(self):
        from app.core.shared_state import reset_all_states

        reset_all_states()

    def test_strengths_stay_with_their_echoes(self):
        from app.sensors import radar_utils as ru
        from app.core.shared_state import get_radar_state

        ru._publish("deg330", {"min_dist": 900.0, "peaks_mm": [900.0, 2400.0],
                               "peaks_db": [18.5, 31.0], "online": True, "degraded": False})
        ru._publish("deg030", {"min_dist": 1200.0, "peaks_mm": [1200.0, 3100.0],
                               "peaks_db": [12.0, 9.0], "online": True, "degraded": False})
        front = get_radar_state()["sectors"]["front"]
        self.assertEqual(list(zip(front["peaks_mm"], front["peaks_db"])),
                         [(900.0, 18.5), (1200.0, 12.0), (2400.0, 31.0), (3100.0, 9.0)])

    def test_a_row_without_strengths_does_not_shift_them(self):
        from app.sensors import radar_utils as ru
        from app.core.shared_state import get_radar_state

        ru._publish("deg330", {"min_dist": 900.0, "peaks_mm": [900.0], "online": True, "degraded": False})
        ru._publish("deg030", {"min_dist": 1200.0, "peaks_mm": [1200.0], "peaks_db": [12.0],
                               "online": True, "degraded": False})
        front = get_radar_state()["sectors"]["front"]
        self.assertEqual(list(zip(front["peaks_mm"], front["peaks_db"])), [(900.0, None), (1200.0, 12.0)])


class AfternoonFixTests(unittest.TestCase):
    """2026-11-09 afternoon: the corridor glass door, a person behind a glass door, the
    camera's way past a vehicle, and a named block's way out."""

    def _front(self, segs, echoes, peaks=None):
        from tools.sim import glass_sim as gs

        walk_policy._near_frames.clear()
        radar = gs.radar_state(echoes)
        for k, p in (peaks or {}).items():
            radar["sectors"][k]["peaks_mm"] = p
        return walk_policy.read_direction(gs.lidar_state(segs), radar, "front")

    def test_corridor_glass_door_nearer_than_its_walls_echo(self):
        from tools.sim import glass_sim as gs

        lat = 1000.0
        for ft in (2.6, 3.0, 3.4):
            d = ft * gs.FT
            segs = (gs.side_wall("left", lat) + gs.side_wall("right", lat)
                    + gs.wall_across(d, gs.DOOR_HALF_MM) + gs.back_wall())
            wall_echo = lat * gs.BEAM_EDGE
            both = sorted({d, wall_echo})
            f = self._front(segs, {"deg030": d, "deg330": d},
                            {"deg030": both, "deg330": both})
            self.assertTrue(f["glass"], ft)
            self.assertAlmostEqual(f["distance_ft"], ft, delta=0.3)

    def test_plain_corridor_is_not_glass(self):
        from tools.sim import glass_sim as gs

        for lat in (450.0, 750.0, 1000.0, 1500.0):
            segs = gs.side_wall("left", lat) + gs.side_wall("right", lat) + gs.back_wall()
            e = lat * gs.BEAM_EDGE
            f = self._front(segs, {"deg030": e, "deg330": e})
            self.assertFalse(f["glass"], lat)
            self.assertFalse(f["distance_ft"] < 4.0, lat)

    def test_person_just_behind_a_glass_door_blocks_at_the_glass(self):
        from tools.sim import glass_sim as gs

        d = 3.0 * gs.FT
        for behind_ft in (0.7, 1.3, 1.9):
            segs = (gs.wall_across(d, gs.DOOR_HALF_MM) + gs.back_wall()
                    + gs.post(d + behind_ft * gs.FT + 150, 0.0, 150))
            f = self._front(segs, {"deg030": d, "deg330": d})
            self.assertAlmostEqual(f["distance_ft"], 3.0, delta=0.15, msg=behind_ft)

    def test_no_way_past_a_vehicle_not_judged_standing(self):
        det = {"class_name": "car", "confidence": 0.9, "bbox": [500.0, 300.0, 800.0, 600.0]}
        cam = {"front": {"frame_time": 100.0, "detections": [det]}}
        with unittest.mock.patch.object(sg, "_camera_sees", lambda *a: True):
            self.assertIsNone(sg.camera_path(cam, 100.0, None, []))  # moving / not yet judged
            self.assertIsNotNone(sg.camera_path(cam, 100.0, None, [det["bbox"]]))  # standing
            self.assertIsNotNone(sg.camera_path(cam, 100.0, None, None))  # no tracker

    def test_a_box_cut_at_both_sides_gives_no_way_past(self):
        det = {"class_name": "car", "confidence": 0.9,
               "bbox": [0.0, 300.0, float(sg._QUAD_W), 700.0]}
        cam = {"front": {"frame_time": 100.0, "detections": [det]}}
        with unittest.mock.patch.object(sg, "_camera_sees", lambda *a: True):
            self.assertIsNone(sg.camera_path(cam, 100.0))

    def _desk(self, tof_solids, segs=None):
        """The rig facing a desk (2026-11-09, 14:46): no laser return ahead, the radar hears
        the desk and what stands on it at 0.26 and 0.53 m."""
        from tools.sim import glass_sim as gs

        FT = gs.FT
        segs = segs if segs is not None else (
            [(1.4 * FT, 1.1 * FT, 1.4 * FT, 3.0 * FT)]
            + gs.side_wall("right", 4.1 * FT, x_from=-1.3 * FT, x_to=1.5 * FT)
        )
        return self._front(
            segs,
            {"deg330": 525.9, "deg030": 263.3},
            {"deg330": [525.9, 1288.2], "deg030": [263.3, 956.1]},
        ) if tof_solids is None else self._front_tof(segs, tof_solids)

    def _front_tof(self, segs, tof_solids):
        from tools.sim import glass_sim as gs

        walk_policy._near_frames.clear()
        radar = gs.radar_state({"deg330": 525.9, "deg030": 263.3})
        radar["sectors"]["deg330"]["peaks_mm"] = [525.9, 1288.2]
        radar["sectors"]["deg030"]["peaks_mm"] = [263.3, 956.1]
        return walk_policy.read_direction(
            gs.lidar_state(segs), radar, "front", None, tof_solids
        )

    def test_a_desk_the_tof_sees_and_the_radar_hears_is_not_glass(self):
        self.assertTrue(self._desk(None)["glass"])  # without the ToF: the old false glass
        f = self._desk([(0.547 / 0.3048, "low_obstacle"), (2.229 / 0.3048, "obstacle")])
        self.assertFalse(f["glass"])
        self.assertAlmostEqual(f["distance_ft"], 0.86, delta=0.05)  # still the block

    def test_a_low_thing_behind_a_pane_keeps_it_glass(self):
        from tools.sim import glass_sim as gs

        d = 3.0 * gs.FT
        walk_policy._near_frames.clear()
        radar = gs.radar_state({"deg030": d, "deg330": d})
        lidar = gs.lidar_state(gs.back_wall())
        for behind_ft in (0.5, 1.0):
            f = walk_policy.read_direction(
                lidar, radar, "front", None, [(3.0 + behind_ft, "low_obstacle")]
            )
            self.assertTrue(f["glass"], behind_ft)

    def test_a_nooks_side_wall_is_not_a_pane_in_the_corridor(self):
        """17:17 run: a 0.9 m nook facing a window blind 4.5 ft off; the right board heard its
        wall at ~1.1 ft, inside RADAR_CONE_TRUST_MM."""
        from tools.sim import glass_sim as gs

        FT = gs.FT
        segs = ([(-1.0 * FT, 1.2 * FT, 4.1 * FT, 1.2 * FT)]
                + [(4.5 * FT, -1.55 * FT, 4.5 * FT, 1.2 * FT)]
                + [(-2.3 * FT, -4 * FT, -2.3 * FT, 4 * FT)])
        f = self._front(segs, {"deg030": 330.0})
        self.assertFalse(f["glass"])
        # A pane 1 ft ahead with a wall beside it is heard by BOTH boards: still glass.
        d, lat = 1.0 * FT, 1.1 * FT
        f = self._front(gs.side_wall("left", lat, x_to=d) + gs.back_wall(),
                        {"deg330": min(d, lat * gs.BEAM_EDGE), "deg030": d})
        self.assertTrue(f["glass"])

    def test_the_tof_seeing_a_solid_thing_at_the_echo_is_not_glass(self):
        from tools.sim import glass_sim as gs
        from app.perception import ground_hazards as gh

        walk_policy._near_frames.clear()
        radar = gs.radar_state({"deg030": 410.0, "deg330": 410.0})
        lidar = gs.lidar_state(gs.back_wall())
        n = int(round(gh.TOF_NEAR_MAX_M / gh.TOF_NEAR_BIN_M))
        at_echo = [0] * n
        for b in (7, 8, 9):  # 0.35-0.50 m: the desk
            at_echo[b] = 300
        elsewhere = [0] * n
        elsewhere[20] = 900  # 1.0 m: something past the echo
        self.assertTrue(walk_policy.read_direction(lidar, radar, "front")["glass"])
        self.assertFalse(
            walk_policy.read_direction(lidar, radar, "front", None, None, at_echo)["glass"]
        )
        self.assertTrue(
            walk_policy.read_direction(lidar, radar, "front", None, None, elsewhere)["glass"]
        )

    def test_a_laser_measured_block_drops_the_held_glass(self):
        world = sg._World()
        base = {"blocked": True, "near": True, "wall": False, "front_ft": 1.2,
                "side": "left", "scan_index": 1, "in_front": True}
        for k in range(6):
            world.update({**base, "glass": True, "scan_index": k}, 100.0 + 0.07 * k)
        text, _ = world.update(
            {**base, "glass": False, "lidar_block": True, "front_ft": 0.5, "scan_index": 9},
            100.6,
        )
        self.assertNotIn(text, sg.BLOCKED_GLASS_ANY)

    def test_named_block_keeps_the_walk_policys_way_out(self):
        for side, want in (("right", sg.STEER_IN_FRONT[("Person", "right")]),
                           ("around", sg.OBJECT_IN_FRONT_AROUND["Person"])):
            text = sg._sentence(True, True, False, False, side, obj="Person")
            self.assertEqual(text, want)
            self.assertIn(text, sg.ALLOWED)


class ClockAndClipTests(unittest.TestCase):
    """2026-11-10: a wall clock stepped forward is not a dead LiDAR; a tester's mark saves
    what the sensors saw."""

    def _scan(self):
        from tools.sim import scene_hunt as sh
        from app.core.shared_state import get_lidar_state

        sh.publish({"left": 900, "right": None, "ahead": 2000, "posts": [], "glass": None}, True)
        return get_lidar_state()

    def test_a_forward_clock_step_is_not_guidance_stopped(self):
        lidar = self._scan()
        stepped = time.time() + 120.0  # NTP sets the clock forward mid-walk
        self.assertIsNone(sg._not_updating(lidar, stepped))
        self.assertTrue(sg._fresh(lidar, stepped))

    def test_a_scan_received_long_ago_is_still_stale(self):
        lidar = dict(self._scan())
        lidar["received_mono"] = time.monotonic() - 5.0
        self.assertIsNotNone(sg._not_updating(lidar, time.time()))
        self.assertFalse(sg._fresh(lidar, time.time()))

    def test_wrong_saves_a_replayable_clip(self):
        import gzip
        import json
        import tempfile
        from app.core import walk_recorder as wr

        with tempfile.TemporaryDirectory() as tmp, unittest.mock.patch.object(
            wr, "WALK_RECORD_DIR", tmp
        ), unittest.mock.patch.object(wr, "WALK_CLIP_AFTER_S", 0.1):
            rec = wr.WalkRecorder()
            m0 = time.monotonic()
            for i in range(30):
                rec.tick(time.time(), m0 - 1.5 + i * 0.05, {"lidar": {"i": i}}, {"speak": False})
            path = rec.clip("wrong", "a test")
            for i in range(2):  # after the mark
                rec.tick(time.time(), time.monotonic() + 0.01 * (i + 1), {"lidar": {"i": 100 + i}}, {})
            for _ in range(50):
                if os.path.exists(path):
                    break
                time.sleep(0.05)
            lines = [json.loads(x) for x in gzip.open(path, "rt")]
        marks = [i for i, x in enumerate(lines) if "mark" in x]
        self.assertEqual(sum(1 for x in lines if "in" in x), 32)
        self.assertEqual(len(marks), 1)
        self.assertEqual(lines[marks[0]]["mark"], "wrong")
        self.assertTrue(all("in" in x for x in lines[marks[0] + 1:]))  # only after-ticks follow



class LabelTests(unittest.TestCase):
    """2026-11-10, learning from walks, phase 1: a tester's "what was really there" label is
    saved with what the sensors saw, kept apart from Wrong clips, and scored against."""

    def _rec(self, tmp):
        from app.core import walk_recorder as wr

        stack = contextlib.ExitStack()
        stack.enter_context(unittest.mock.patch.object(wr, "WALK_RECORD_DIR", tmp))
        stack.enter_context(unittest.mock.patch.object(wr, "WALK_CLIP_AFTER_S", 0.05))
        return wr, stack

    @staticmethod
    def _wait(path):
        for _ in range(60):
            if os.path.exists(path):
                return
            time.sleep(0.05)

    def test_a_label_saves_a_clip_named_and_marked_with_it(self):
        import gzip
        import json
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            wr, stack = self._rec(tmp)
            with stack:
                rec = wr.WalkRecorder()
                m0 = time.monotonic()
                for i in range(20):
                    rec.tick(time.time(), m0 - 1.0 + i * 0.05, {"lidar": {"i": i}}, {})
                path = rec.clip("label", "wall | last said: x", label="wall", said="Obstacle ahead")
                self._wait(path)
                lines = [json.loads(x) for x in gzip.open(path, "rt")]
                counts = rec.label_counts()
        self.assertTrue(path.endswith("-wall.jsonl.gz"), path)
        mark = next(x for x in lines if "mark" in x)
        self.assertEqual((mark["mark"], mark["label"], mark["said"]), ("label", "wall", "Obstacle ahead"))
        self.assertEqual(counts["wall"], 1)
        self.assertEqual(sum(counts.values()), 1)

    def test_wrong_presses_never_prune_the_labelled_clips(self):
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            wr, stack = self._rec(tmp)
            with stack, unittest.mock.patch.object(wr, "WALK_CLIP_KEEP_FILES", 2):
                for i, name in enumerate(
                    ["clip-20261110-100000-000-glass.jsonl.gz"]
                    + [f"clip-20261110-1001{i:02d}-000-wrong.jsonl.gz" for i in range(5)]
                ):
                    with open(os.path.join(tmp, name), "w") as fh:
                        fh.write("x")
                    os.utime(os.path.join(tmp, name), (1e9 + i, 1e9 + i))
                wr._prune_clips()
                left = sorted(os.listdir(tmp))
        self.assertIn("clip-20261110-100000-000-glass.jsonl.gz", left)
        self.assertEqual(sum(1 for n in left if n.endswith("-wrong.jsonl.gz")), 2)
        self.assertEqual(wr.clip_label("clip-20261110-100000-000-wrong-person.jsonl.gz"), "person")
        self.assertEqual(wr.clip_label("clip-20261110-100000-000-wrong.jsonl.gz"), "")

    def test_the_mark_route_takes_a_label_and_refuses_an_unknown_one(self):
        import asyncio
        import tempfile
        from app.api.guidance_router import tester_mark

        with tempfile.TemporaryDirectory() as tmp:
            wr, stack = self._rec(tmp)
            with stack:
                ok = asyncio.run(tester_mark({"kind": "label", "label": "Glass"}))
                bad = asyncio.run(tester_mark({"kind": "label", "label": "sofa"}))
                bare = asyncio.run(tester_mark({"kind": "label"}))
                self._wait(os.path.join(tmp, os.path.basename(ok["clip"])))
        self.assertEqual((ok["ok"], ok["label"]), (True, "glass"))
        self.assertTrue(ok["clip"].endswith("-glass.jsonl.gz"))
        self.assertEqual((bad.status_code, bare.status_code), (400, 400))

    def test_what_a_sentence_amounts_to(self):
        from tools.field.label_report import label_class

        for text, want in (
            (sg.OBSTACLE_IN_FRONT["left"], "obstacle"),
            (sg.OBSTACLE_AHEAD, "obstacle"),
            (sg.WALL_AHEAD, "wall"),
            (sg.GLASS_AHEAD, "glass"),
            (sg.OBJECT_IN_FRONT["Person"], "person"),
            (sg.OBJECT_AHEAD["Bench"], "obstacle"),
            (sg.WALK_STRAIGHT, "nothing"),
            (sg.STAIRS_DOWN_STOP, "stairs"),
            (sg.STAIRS_UP_AHEAD, "stairs"),
            (sg.STEP_UP_AHEAD, "kerb"),
            (sg.DROP_AHEAD, "kerb"),
            (sg.LOW_OBSTACLE_AHEAD, "obstacle"),
            (sg.SENSORS_OFFLINE, "other"),
            ("", "nothing"),
        ):
            self.assertEqual(label_class(text), want, text)

    def test_the_report_reads_what_was_said_before_the_press(self):
        import gzip
        import json
        import tempfile
        from tools.field.label_report import labels_in

        said = sg.OBSTACLE_IN_FRONT["left"]
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "clip-20261110-100000-000-wall.jsonl.gz")
            with gzip.open(path, "wt") as fh:
                for i in range(40):  # 4 s of ticks; it said it at 1 s, and "Camera off" at 3 s
                    out = {"instruction": said, "speak": i == 10}
                    if i == 30:
                        out = {"instruction": "Camera off", "speak": True}
                    fh.write(json.dumps({"t": 100 + i / 10, "mono": 50 + i / 10, "in": {"i": i}, "out": out}) + "\n")
                fh.write(json.dumps({"t": 104, "mono": 54.0, "mark": "label", "label": "wall", "said": said}) + "\n")
            rows = labels_in(path, replay=False, export_s=1.0)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["then"], {"instruction": said, "class": "obstacle"})
        self.assertEqual([t["in"]["i"] for t in rows[0]["ticks"]], list(range(30, 40)))
        self.assertNotIn("now", rows[0])

    def test_the_dashboard_has_a_button_for_every_label(self):
        import re
        from app.config import MARK_LABELS

        with open(os.path.join(os.path.dirname(__file__), "..", "templates", "dashboard.html")) as fh:
            html = fh.read()
        self.assertEqual(tuple(re.findall(r'data-label="([a-z]+)"', html)), MARK_LABELS)
        self.assertEqual(len({k[0] for k in MARK_LABELS}), len(MARK_LABELS))  # one key each



class ToFWallTests(unittest.TestCase):
    """2026-11-10: the ToF names a wall — a flat, wide, upright surface across the path — so a
    window blind is "Blocked by wall", not "Obstacle in front". It renames only."""

    _ground = None

    @classmethod
    def ground(cls, now):
        """The ToF's analysis of a wall 1.2 m ahead (tools/sim/ground_hazard_sim), as of `now`."""
        if cls._ground is None:
            from tools.sim import ground_hazard_sim as gs

            cls._ground = gs.run_scene("wall_near", frames=3, with_camera=False, scenes=gs.WALL_SCENES)
        return {**cls._ground, "t": now}

    def test_flat_wide_surfaces_are_walls_and_people_and_furniture_are_not(self):
        from tools.sim import ground_hazard_sim as gs

        for name, want in (("wall_angled", True), ("blind", True), ("room_corner", True),
                           ("person", False), ("chair", False), ("cupboard", False)):
            found, why, _s, _ms = gs.wall_result(name)
            self.assertEqual(found is not None, want, f"{name}: {why}")

    def test_a_wall_must_hold_over_frames_and_be_fresh(self):
        from app.perception.ground_hazards import confirmed_wall

        wall = {"ahead_m": 1.2, "length_m": 2.0, "flat_m": 0.01, "reaches_top": True, "height_m": 1.2}
        snap = lambda surfaces, t=100.0: {"ok": True, "t": t, "front_surfaces": surfaces}
        self.assertIsNotNone(confirmed_wall(snap([None, wall, wall]), 100.1)[0])
        self.assertIsNone(confirmed_wall(snap([None, None, wall]), 100.1)[0])  # one frame
        self.assertIsNone(confirmed_wall(snap([wall, wall, None]), 100.1)[0])  # gone now
        self.assertIsNone(confirmed_wall(snap([None, wall, wall]), 101.0)[0])  # stale
        far = {**wall, "ahead_m": 2.5}
        self.assertIsNone(confirmed_wall(snap([None, far, wall]), 100.1)[0])  # not one thing
        narrow = {**wall, "length_m": 0.5}
        self.assertIsNone(confirmed_wall(snap([narrow, narrow, narrow]), 100.1)[0])

    def _block(self, ground_at=None, speak=False):
        """An unnamed thing 3.9 ft ahead (a round post: no laser wall), the ToF's wall."""
        from tools.sim import scene_hunt as sh
        from app.core.shared_state import get_lidar_state, get_radar_state

        sg.reset()
        got = None
        with unittest.mock.patch.object(sg, "TOF_WALL_SPEAK", speak):
            for i in range(6):
                sh.publish({"left": None, "right": None, "ahead": None,
                            "posts": [(1450, 0, 250)], "glass": None}, True)
                lidar, radar = get_lidar_state(), get_radar_state()
                now = time.time() + i * 0.15
                ground = self.ground(now)
                if ground_at is not None:
                    moved = {**ground["front_surfaces"][-1], "ahead_m": ground_at}
                    ground = {**ground, "front_surfaces": [moved] * 3}
                got = sg.guidance(walk_policy.decide(lidar, radar), lidar, now=now,
                                  radar_state=radar, camera_state={}, ground=ground)
        return got

    def test_the_tofs_wall_names_an_unnamed_block_keeping_its_turn(self):
        quiet, named = self._block(speak=False), self._block(speak=True)
        self.assertIn(quiet["instruction"], sg.OBSTACLE_IN_FRONT.values())
        turn = quiet["instruction"].rsplit(" ", 1)[-1]
        self.assertEqual(quiet["raw"]["wall_would_say"], sg.BLOCKED_WALL_TURN[turn])
        self.assertEqual(named["instruction"], sg.BLOCKED_WALL_TURN[turn])
        self.assertEqual(named["obstacle"], "wall")
        self.assertIn("the ToF", named["reason"])

    def test_a_tof_wall_at_another_distance_is_not_the_block(self):
        got = self._block(ground_at=2.6, speak=True)
        self.assertIn(got["instruction"], sg.OBSTACLE_IN_FRONT.values())
        self.assertIsNone(got["raw"]["tof_wall"])



class RestingOnADeskTests(unittest.TestCase):
    """2026-11-10: set down on a desk, the rig's radar heard the desk top 0.9 ft ahead and it
    said "Obstacle in front, turn left" at a window blind 4 ft off. The ToF finds the surface
    it rests on; an echo there that the laser sees past is the desk, not a thing in the way."""

    @staticmethod
    def _desk_frame():
        """The ToF ~2 cm above a desk ending 0.6 m ahead, looking ~16 deg UP (not 30 down, as
        worn): as the rig lay on the desk on 2026-11-10, by the desk's plane in its frame, the
        desk 0.2-0.35 m off in the picture's bottom rows. A wall 1.2 m ahead.
        tools/sim/ground_hazard_sim's renderer: its scenes are about the front camera's lens, the
        ToF 3.8 cm below it (the design calibration)."""
        from tools.sim import ground_hazard_sim as gs

        H = gs.H
        below = float(gs.CAL.t[1]) + 0.02
        boxes = [
            gs.floor(),
            gs.slab(-1.0, 1.0, H - below, -0.5, 0.6, bottom=H - 0.75),  # the desk top
            gs.slab(-3.0, 3.0, 3.0, 1.2, 1.4, bottom=0.0),  # the wall with the blind
        ]
        return gs.render_tof(boxes, pitch=-46.0)

    def test_the_desk_under_the_tof_is_found_and_left_out_of_what_is_near(self):
        from app.perception import depth_fusion as df
        from app.perception import ground_hazards as gh
        from tools.sim import ground_hazard_sim as gs

        depth, conf = self._desk_frame()
        res = gh.GroundAnalyser(df.DepthProjector(gs.CAL)).analyse(depth, conf, 1.0)
        self.assertIsNotNone(res["rest"])
        self.assertLess(res["rest"]["to_m"], 0.5)
        self.assertEqual(sum(res["near"][: int(0.45 / gh.TOF_NEAR_BIN_M)]), 0)

    def test_worn_there_is_no_resting_surface(self):
        from app.perception import depth_fusion as df
        from app.perception import ground_hazards as gh
        from tools.sim import ground_hazard_sim as gs

        for name in ("wall_near", "wall_close", "person_near", "table"):
            depth, conf = gs.render_tof(gs.WALL_SCENES[name][0]())
            res = gh.GroundAnalyser(df.DepthProjector(gs.CAL)).analyse(depth, conf, 1.0)
            self.assertIsNone(res.get("rest"), name)

    def _front(self, rest):
        """A wall 4 ft ahead in a 3 ft nook, and an echo 0.29 m ahead the laser sees past."""
        from tools.sim import scene_hunt as sh
        from app.core.shared_state import get_lidar_state, get_radar_state

        sh.publish({"left": 450, "right": 430, "ahead": 1225, "posts": [], "glass": 290}, True)
        walk_policy._near_frames.clear()
        return walk_policy.read_direction(get_lidar_state(), get_radar_state(), "front", tof_rest=rest)

    def test_an_echo_at_the_desk_is_the_desk(self):
        desk = {"below_m": 0.02, "to_m": 0.356, "points": 3500}
        on_desk, worn = self._front(desk), self._front(None)
        self.assertTrue(on_desk["rest_echo"])
        self.assertFalse(on_desk["glass"])
        self.assertAlmostEqual(on_desk["distance_ft"], 1225 / 304.8, delta=0.3)
        self.assertFalse(worn["rest_echo"])
        self.assertLess(worn["distance_ft"], 1.5)  # worn, the same echo still blocks

    def test_an_echo_past_where_the_desk_ends_still_counts(self):
        short = {"below_m": 0.02, "to_m": 0.10, "points": 3500}  # ends 0.1 m out
        got = self._front(short)
        self.assertFalse(got["rest_echo"])
        self.assertLess(got["distance_ft"], 1.5)


class CameraLightTests(unittest.TestCase):
    """2026-11-10: the sensor's own light reading (lux), per camera, so a walk can show whether
    the camera names less in dim light. Shown and recorded only; nothing said changes."""

    def setUp(self):
        from app.sensors import pi_camera_utils as pc

        self.pc = pc
        self._saved = (pc._light, dict(pc._views))
        pc._views.clear()

    def tearDown(self):
        self.pc._light, views = self._saved
        self.pc._views.clear()
        self.pc._views.update(views)

    def _lit(self, lux):
        self.pc._light = {"lux": lux, "exposure_ms": 30.0, "gain": 2.0, "mono": time.monotonic()}

    def test_the_sensor_reading_is_kept_and_never_raises(self):
        class Request:
            def __init__(self, md):
                self.md = md

            def get_metadata(self):
                if self.md is None:
                    raise RuntimeError("request already released")
                return self.md

        self.pc._light = {**self.pc._light, "mono": float("-inf")}
        self.pc._note_light(Request({"Lux": 382.4, "ExposureTime": 30000, "AnalogueGain": 2.0}))
        self.assertEqual(self.pc.light_reading(), {"lux": 382.4, "exposure_ms": 30.0, "gain": 2.0})
        self.pc._light = {**self.pc._light, "mono": float("-inf")}
        self.pc._note_light(Request(None))  # a failed read is dropped, not raised
        self.pc._light = {**self.pc._light, "mono": time.monotonic() - 5.0}
        self.assertIsNone(self.pc.light_reading()["lux"])  # stale: no reading

    def _check(self, front_level, whole_level=100.0, lux=400.0, after_s=0.0):
        import numpy as np

        self._lit(lux)
        mosaic = np.full((80, 80, 3), whole_level, dtype=np.uint8)
        quad = np.full((40, 40, 3), front_level, dtype=np.uint8)
        v = self.pc._view_check("front", quad, mosaic)
        if after_s:
            with self.pc._view_lock:
                held = self.pc._views["front"]
                held["mono"] -= after_s
                held["dim_since"] = held["dim_since"] and held["dim_since"] - after_s
            v = self.pc._view_check("front", quad, mosaic)
        return v

    def test_a_dim_camera_is_told_once_it_has_lasted(self):
        first = self._check(front_level=20)  # a fifth of the picture's light: 80 lux, ...
        self.assertFalse(first["dim"])
        self.pc._views.clear()
        first = self._check(front_level=14, lux=300.0)  # ... this one 42 lux
        self.assertLess(first["lux"], 45.0)
        self.assertFalse(first["dim"])  # not yet: CAMERA_DIM_CONFIRM_S
        later = self._check(front_level=14, lux=300.0, after_s=4.0)
        self.assertTrue(later["dim"])
        self.assertFalse(later["dark"])

    def test_a_lit_room_and_no_reading_are_not_dim(self):
        lit = self._check(front_level=100, after_s=4.0)
        self.assertAlmostEqual(lit["lux"], 400.0, delta=60.0)
        self.assertFalse(lit["dim"])
        self.pc._views.clear()
        self.pc._light = {**self.pc._light, "mono": time.monotonic() - 5.0}
        import numpy as np

        v = self.pc._view_check("front", np.full((40, 40, 3), 30, np.uint8), None)
        self.assertIsNone(v["lux"])
        self.assertFalse(v["dim"])

    def test_the_report_splits_labels_by_light(self):
        import gzip
        import io
        import json
        import tempfile
        from tools.field.label_report import labels_in, report

        rows = []
        with tempfile.TemporaryDirectory() as tmp:
            for i, (lux, said) in enumerate(((380.0, sg.WALL_AHEAD), (20.0, sg.WALK_STRAIGHT))):
                path = os.path.join(tmp, f"clip-20261110-10000{i}-000-wall.jsonl.gz")
                with gzip.open(path, "wt") as fh:
                    for k in range(10):
                        cam = {"front": {"view": {"lux": lux}}}
                        out = {"instruction": said, "speak": k == 2}
                        fh.write(json.dumps({"t": k, "mono": k, "in": {"camera": cam}, "out": out}) + "\n")
                    fh.write(json.dumps({"t": 10, "mono": 10.0, "mark": "label", "label": "wall"}) + "\n")
                rows += labels_in(path, replay=False)
        self.assertEqual([r["lux"] for r in rows], [380.0, 20.0])
        printed = io.StringIO()
        with contextlib.redirect_stdout(printed):
            report(rows, replay=False)
        text = printed.getvalue()
        self.assertRegex(text, r"good light\s+1\s+1 \(100%\)")
        self.assertRegex(text, r"dim\s+1\s+0 \(  0%\)")



class PaneAcrossANarrowPassageTests(unittest.TestCase):
    """2026-11-10: a pane across a whole narrow passage (no frame the laser sees) said "Walk
    straight": its echo was "explained" by side walls lying farther off than it. No part of a
    wall is nearer than its perpendicular distance (walk_policy._wall_too_far)."""

    def _front(self, lat_l, lat_r, echo_l, echo_r):
        from tools.sim import glass_sim as gs

        walk_policy._near_frames.clear()
        segs = gs.side_wall("left", lat_l) + gs.side_wall("right", lat_r) + gs.back_wall()
        radar = gs.radar_state({"deg330": echo_l, "deg030": echo_r})
        return walk_policy.read_direction(gs.lidar_state(segs), radar, "front")

    def test_a_pane_nearer_than_the_walls_is_glass(self):
        from tools.sim import glass_sim as gs

        for lat_l, lat_r, ft in ((914.0, 914.0, 2.0), (914.0, 914.0, 2.6), (366.0, 600.0, 1.5)):
            d = ft * gs.FT
            f = self._front(lat_l, lat_r, [d, lat_l * gs.BEAM_EDGE], [d, lat_r * gs.BEAM_EDGE])
            with self.subTest(walls=(lat_l, lat_r), ft=ft):
                self.assertTrue(f["glass"])
                self.assertAlmostEqual(f["distance_ft"], ft, delta=0.3)

    def test_the_walls_own_echoes_are_not_glass(self):
        from tools.sim import glass_sim as gs

        for lat in (457.0, 914.0, 1500.0):
            for wall_echo in (lat - 50.0, lat * gs.BEAM_EDGE):  # abeam, the beam's edge
                f = self._front(lat, lat, [wall_echo], [wall_echo])
                with self.subTest(lat=lat, echo=wall_echo):
                    self.assertFalse(f["glass"])

    def test_a_wall_cannot_explain_an_echo_nearer_than_itself(self):
        side = {True: 914.0}
        self.assertTrue(walk_policy._wall_too_far(60.0, 914.0 / math.sin(math.radians(60)), side, 792.0))
        self.assertFalse(walk_policy._wall_too_far(60.0, 914.0 / math.sin(math.radians(60)), side, 850.0))
        self.assertFalse(walk_policy._wall_too_far(10.0, 900.0, side, 500.0))  # not on the wall


class CloseWallOpenSideTests(unittest.TestCase):
    """2026-11-10: facing a flat wall 1.0-1.5 ft off in open space it said "turn around": each
    side board's beam heard the wall ahead at its edge (~1.86 x the front) and that echo was
    taken as the side's room, under the turn floor. An echo the laser places off the side's
    line, with that line clear past it, is not the side's room."""

    def _say(self, scene):
        from tools.sim import scene_hunt as sh
        from app.core.shared_state import get_lidar_state, get_radar_state

        walk_policy._last_turn.update(direction=None, at=0.0)
        walk_policy._last_measured.clear()
        walk_policy._near_frames.clear()
        sh.publish(scene, True)
        lidar, radar = get_lidar_state(), get_radar_state()
        walk = walk_policy.decide(lidar, radar)
        return sg.decide(walk, lidar, time.time(), radar_state=radar, camera_state={})["instruction"]

    def test_the_open_side_is_offered(self):
        ft = 304.8
        for f in (1.0, 1.3, 1.5):
            with self.subTest(wall_ft=f):
                said = self._say({"left": None, "right": None, "ahead": f * ft, "posts": [], "glass": None})
                self.assertEqual(said, sg.BLOCKED_WALL_TURN["right"])
        corner = {"left": 1.7 * ft, "right": None, "ahead": 1.0 * ft, "posts": [], "glass": None}
        self.assertEqual(self._say(corner), sg.BLOCKED_WALL_TURN["right"])  # was "Stop, no clear way"

    def test_something_on_the_sides_line_still_closes_it(self):
        ft = 304.8
        post_right = {"left": None, "right": None, "ahead": 1.0 * ft, "glass": None,
                      "posts": [(0.0, 1.9 * ft, 150.0)]}  # a post 1.9 ft to the right, abeam
        self.assertNotEqual(self._say(post_right), sg.BLOCKED_WALL_TURN["right"])

    def test_the_blind_side_keeps_the_radar(self):
        ft = 304.8  # the left is the LiDAR's blind arc: the radar is all there is
        corner = {"left": None, "right": 1.7 * ft, "ahead": 1.0 * ft, "posts": [], "glass": None}
        self.assertNotEqual(self._say(corner), sg.BLOCKED_WALL_TURN["left"])


class StatusNoticeCutTests(unittest.TestCase):
    """2026-11-10: a block waited out a status notice's airtime ("Power low", "Glass sensor
    off": ~1.1 s, ~1.4 m at walking pace). Now a block cuts into it, as into any guidance
    sentence, and the notice it cut is told again once the path is clear."""

    def test_a_block_cuts_into_a_status_notice(self):
        gate = sg._Gate()
        gate.admit(sg.WALK_STRAIGHT, 100.0)
        self.assertTrue(gate.admit(sg.WALK_STRAIGHT, 102.5)[0])  # held 2.5 s: said
        self.assertTrue(gate.admit_status(sg.POWER_LOW, 105.0)[0])
        block = sg.OBSTACLE_IN_FRONT["left"]
        self.assertTrue(gate.admit(block, 105.3)[0])  # 0.3 s into "Power low"
        self.assertEqual(gate.take_cut_status(), sg.POWER_LOW)
        self.assertEqual(gate.take_cut_status(), "")  # once

    def test_a_calmer_sentence_still_waits_and_cuts_nothing(self):
        gate = sg._Gate()
        gate.admit(sg.WALK_STRAIGHT, 100.0)
        gate.admit(sg.WALK_STRAIGHT, 102.5)
        gate.admit_status(sg.POWER_LOW, 105.0)
        said, _, why = gate.admit(sg.WALL_AHEAD, 105.3)
        self.assertFalse(said, why)
        self.assertEqual(gate.take_cut_status(), "")
        # a block once the notice has ended cut nothing either
        self.assertTrue(gate.admit(sg.OBSTACLE_IN_FRONT["left"], 107.0)[0])
        self.assertEqual(gate.take_cut_status(), "")

    def test_a_cut_notice_is_told_again_only_while_it_still_holds(self):
        n = sg._SensorNotice(sg.POWER_LOW, sg.POWER_NORMAL, 3.0, 10.0, 160.0, 20.0)
        n.pending(True, 0.0)
        n.pending(False, 1.0)
        self.assertEqual(n.pending(False, 4.0), sg.POWER_LOW)
        n.said(sg.POWER_LOW)
        self.assertIsNone(n.pending(False, 4.1))  # told
        n.unsay(sg.POWER_LOW)
        self.assertEqual(n.pending(False, 4.2), sg.POWER_LOW)  # again, no new 3 s wait
        n.unsay(sg.POWER_LOW)  # a second cut before it is said is the same one
        n.pending(True, 4.3)  # power back before it was heard: nothing to tell
        self.assertIsNone(n.pending(True, 30.0))
        n.unsay(sg.CAMERA_OFF)  # not its notice: ignored
        self.assertIsNone(n.pending(True, 31.0))

    def test_walking_into_a_post_during_power_low(self):
        from tools.sim import scene_hunt as sh
        from app.core.shared_state import get_lidar_state, get_radar_state

        sg.reset()
        t0, dt, said, post_at = time.time(), 1 / 15, [], None
        for i in range(int(12 / dt)):
            now = t0 + i * dt
            post = post_at is not None and post_at <= now < post_at + 3.0
            sh.publish({"left": None, "right": None, "ahead": None,
                        "posts": [(1000, 0, 250)] if post else [], "glass": None}, True)
            lidar, radar = get_lidar_state(), get_radar_state()
            got = sg.guidance(walk_policy.decide(lidar, radar), lidar, now=now,
                              system={"readable": True, "power_low": i > 5})
            if got.get("speak"):
                said.append((now - t0, got["instruction"]))
                if got["instruction"] == sg.POWER_LOW and post_at is None:
                    post_at = now + 0.1  # the post appears 0.1 s into the notice
        texts = [t for _, t in said]
        blocks = [at for at, t in said if t in sg.OBSTACLE_IN_FRONT.values()]
        self.assertTrue(blocks, texts)
        self.assertLess(blocks[0] - (post_at - t0), 0.4, texts)  # was 1.1 s
        self.assertEqual(texts.count(sg.POWER_LOW), 2, texts)  # cut, then told again...
        again = [at for at, t in said if t == sg.POWER_LOW][1]
        self.assertGreater(again, (post_at - t0) + 3.0, texts)  # ...once the post is gone

    def test_a_fall_in_front_cuts_in(self):
        from app.guidance.unified_guidance import is_urgent

        self.assertTrue(is_urgent({"rule": "ground_hazard", "instruction": sg.DROP_STOP}))
        self.assertTrue(is_urgent({"rule": "ground_hazard", "instruction": sg.STAIRS_DOWN_STOP}))
        self.assertFalse(is_urgent({"rule": "ground_hazard", "instruction": sg.STAIRS_UP_AHEAD}))
        self.assertTrue(is_urgent({"rule": "blocked", "instruction": sg.OBSTACLE_IN_FRONT["left"]}))
        self.assertFalse(is_urgent({"rule": "status", "instruction": sg.POWER_LOW}))


if __name__ == "__main__":
    unittest.main()
