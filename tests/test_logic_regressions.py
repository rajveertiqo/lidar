"""Decision-logic regressions found by the mirror test and fuzzing (2026-10-30). Hardware-free;
run with venv/bin/python -m unittest tests.test_logic_regressions."""

import contextlib
import math
import os
import threading
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
        a, d = walk_policy._laser_points(
            self.state([0.0, None, 2.0, float("nan")], [1000, 1100, None, 900])
        )
        self.assertEqual((a, d), ([0.0], [1000.0]))
        a, d = walk_policy._laser_points(self.state([0.0, 1.0, 2.0], [1000, 1100]))
        self.assertEqual(len(a), len(d))

    def test_glass_checks_survive_them(self):
        st = self.state(
            [0.0, None, 1.0] + [float(i) for i in range(2, 50)], [900, 950, 960] + [1000] * 40
        )  # a None angle, and a shorter list
        walk_policy._laser_explains(st, 30.0, 1000.0)
        walk_policy._matches_ahead(st, 3.0)


class TurnAwayFromObstacleTests(unittest.TestCase):
    """Room enough both ways: the turn goes away from the side the obstacle is on (it was a
    fixed order: on the rig always "Turn right", with the obstacle on the right too)."""

    def views(self, left=20.0, right=20.0):
        return {
            d: {"direction": d, "distance_ft": ft, "unknown": False, "open": False}
            for d, ft in (("left", left), ("right", right), ("back", 10.0))
        }

    def test_away_from_the_obstacle(self):
        self.assertEqual(
            walk_policy.choose_turn(self.views(), 2.5, obstacle_deg=20.0)["direction"], "left"
        )
        self.assertEqual(
            walk_policy.choose_turn(self.views(), 2.5, obstacle_deg=-20.0)["direction"], "right"
        )

    def test_held_turn_kept_and_near_ahead_not_judged(self):
        self.assertEqual(
            walk_policy.choose_turn(self.views(), 2.5, "right", obstacle_deg=20.0)["direction"],
            "right",
        )
        self.assertEqual(
            walk_policy.choose_turn(self.views(18.0, 20.0), 2.5, obstacle_deg=1.0)["direction"],
            "right",
        )

    def test_little_room_one_way_still_goes_by_room(self):
        self.assertEqual(
            walk_policy.choose_turn(self.views(5.0, 20.0), 2.5, obstacle_deg=-20.0)["direction"],
            "right",
        )

    def test_the_rig_says_it(self):
        from tools.sim import scene_hunt as sh

        post = {
            "left": None,
            "right": None,
            "ahead": None,
            "glass": None,
            "posts": [(800.0, 300.0, 130.0)],
        }
        # Named since 2026-11-09: a post in front is not "Turn left" alone.
        self.assertEqual(sh.decide(post)[1]["instruction"], "Obstacle in front, turn left")
        self.assertEqual(
            sh.decide(sh.mirror(post))[1]["instruction"], "Obstacle in front, turn right"
        )


class GlassBehindNearerEchoTests(unittest.TestCase):
    """A pane the radar hears behind a nearer echo the laser explains (2026-11-02)."""

    def said(self, scene, ghosts=0.0):
        import random

        from tools.sim import scene_hunt as sh

        return sh.decide(scene, ghosts=ghosts, rnd=random.Random(0))[1]["instruction"]

    def test_pane_at_a_side_posts_range(self):
        scene = {
            "left": None,
            "right": 2667.0,
            "ahead": None,
            "glass": 836.0,
            "posts": [(3417.0, -135.0, 258.0), (3597.0, 1413.0, 196.0), (528.0, -776.0, 91.0)],
        }
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
            walk, _ = sh.decide(
                {"left": None, "right": None, "ahead": None, "glass": None, "posts": [post]}
            )
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

        ru._publish(
            "deg330",
            {
                "min_dist": 900.0,
                "peaks_mm": [900.0, 2400.0],
                "peaks_db": [18.5, 31.0],
                "online": True,
                "degraded": False,
            },
        )
        ru._publish(
            "deg030",
            {
                "min_dist": 1200.0,
                "peaks_mm": [1200.0, 3100.0],
                "peaks_db": [12.0, 9.0],
                "online": True,
                "degraded": False,
            },
        )
        front = get_radar_state()["sectors"]["front"]
        self.assertEqual(
            list(zip(front["peaks_mm"], front["peaks_db"])),
            [(900.0, 18.5), (1200.0, 12.0), (2400.0, 31.0), (3100.0, 9.0)],
        )

    def test_a_row_without_strengths_does_not_shift_them(self):
        from app.sensors import radar_utils as ru
        from app.core.shared_state import get_radar_state

        ru._publish(
            "deg330", {"min_dist": 900.0, "peaks_mm": [900.0], "online": True, "degraded": False}
        )
        ru._publish(
            "deg030",
            {
                "min_dist": 1200.0,
                "peaks_mm": [1200.0],
                "peaks_db": [12.0],
                "online": True,
                "degraded": False,
            },
        )
        front = get_radar_state()["sectors"]["front"]
        self.assertEqual(
            list(zip(front["peaks_mm"], front["peaks_db"])), [(900.0, None), (1200.0, 12.0)]
        )


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
            segs = (
                gs.side_wall("left", lat)
                + gs.side_wall("right", lat)
                + gs.wall_across(d, gs.DOOR_HALF_MM)
                + gs.back_wall()
            )
            wall_echo = lat * gs.BEAM_EDGE
            both = sorted({d, wall_echo})
            f = self._front(segs, {"deg030": d, "deg330": d}, {"deg030": both, "deg330": both})
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
            segs = (
                gs.wall_across(d, gs.DOOR_HALF_MM)
                + gs.back_wall()
                + gs.post(d + behind_ft * gs.FT + 150, 0.0, 150)
            )
            f = self._front(segs, {"deg030": d, "deg330": d})
            self.assertAlmostEqual(f["distance_ft"], 3.0, delta=0.15, msg=behind_ft)

    def test_no_way_past_a_vehicle_not_judged_standing(self):
        from app.config import CAMERA_PX_SCALE as k  # the same car on a full-resolution picture

        det = {
            "class_name": "car",
            "confidence": 0.9,
            "bbox": [500.0 * k, 300.0 * k, 800.0 * k, 600.0 * k],
        }
        cam = {"front": {"frame_time": 100.0, "detections": [det]}}
        with unittest.mock.patch.object(sg, "_camera_sees", lambda *a: True):
            self.assertIsNone(sg.camera_path(cam, 100.0, None, []))  # moving / not yet judged
            self.assertIsNotNone(sg.camera_path(cam, 100.0, None, [det["bbox"]]))  # standing
            self.assertIsNotNone(sg.camera_path(cam, 100.0, None, None))  # no tracker

    def test_a_box_cut_at_both_sides_gives_no_way_past(self):
        det = {
            "class_name": "car",
            "confidence": 0.9,
            "bbox": [0.0, 300.0, float(sg._QUAD_W), 700.0],
        }
        cam = {"front": {"frame_time": 100.0, "detections": [det]}}
        with unittest.mock.patch.object(sg, "_camera_sees", lambda *a: True):
            self.assertIsNone(sg.camera_path(cam, 100.0))

    def _desk(self, tof_solids, segs=None):
        """The rig facing a desk (2026-11-09, 14:46): no laser return ahead, the radar hears
        the desk and what stands on it at 0.26 and 0.53 m."""
        from tools.sim import glass_sim as gs

        FT = gs.FT
        segs = (
            segs
            if segs is not None
            else (
                [(1.4 * FT, 1.1 * FT, 1.4 * FT, 3.0 * FT)]
                + gs.side_wall("right", 4.1 * FT, x_from=-1.3 * FT, x_to=1.5 * FT)
            )
        )
        return (
            self._front(
                segs,
                {"deg330": 525.9, "deg030": 263.3},
                {"deg330": [525.9, 1288.2], "deg030": [263.3, 956.1]},
            )
            if tof_solids is None
            else self._front_tof(segs, tof_solids)
        )

    def _front_tof(self, segs, tof_solids):
        from tools.sim import glass_sim as gs

        walk_policy._near_frames.clear()
        radar = gs.radar_state({"deg330": 525.9, "deg030": 263.3})
        radar["sectors"]["deg330"]["peaks_mm"] = [525.9, 1288.2]
        radar["sectors"]["deg030"]["peaks_mm"] = [263.3, 956.1]
        return walk_policy.read_direction(gs.lidar_state(segs), radar, "front", None, tof_solids)

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
        segs = (
            [(-1.0 * FT, 1.2 * FT, 4.1 * FT, 1.2 * FT)]
            + [(4.5 * FT, -1.55 * FT, 4.5 * FT, 1.2 * FT)]
            + [(-2.3 * FT, -4 * FT, -2.3 * FT, 4 * FT)]
        )
        f = self._front(segs, {"deg030": 330.0})
        self.assertFalse(f["glass"])
        # A pane 1 ft ahead with a wall beside it is heard by BOTH boards: still glass.
        d, lat = 1.0 * FT, 1.1 * FT
        f = self._front(
            gs.side_wall("left", lat, x_to=d) + gs.back_wall(),
            {"deg330": min(d, lat * gs.BEAM_EDGE), "deg030": d},
        )
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
        base = {
            "blocked": True,
            "near": True,
            "wall": False,
            "front_ft": 1.2,
            "side": "left",
            "scan_index": 1,
            "in_front": True,
        }
        for k in range(6):
            world.update({**base, "glass": True, "scan_index": k}, 100.0 + 0.07 * k)
        text, _ = world.update(
            {**base, "glass": False, "lidar_block": True, "front_ft": 0.5, "scan_index": 9},
            100.6,
        )
        self.assertNotIn(text, sg.BLOCKED_GLASS_ANY)

    def test_named_block_keeps_the_walk_policys_way_out(self):
        for side, want in (
            ("right", sg.STEER_IN_FRONT[("Person", "right")]),
            ("around", sg.OBJECT_IN_FRONT_AROUND["Person"]),
        ):
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

        with (
            tempfile.TemporaryDirectory() as tmp,
            unittest.mock.patch.object(wr, "WALK_RECORD_DIR", tmp),
            unittest.mock.patch.object(wr, "WALK_CLIP_AFTER_S", 0.1),
        ):
            rec = wr.WalkRecorder()
            m0 = time.monotonic()
            for i in range(30):
                rec.tick(time.time(), m0 - 1.5 + i * 0.05, {"lidar": {"i": i}}, {"speak": False})
            path = rec.clip("wrong", "a test")
            for i in range(2):  # after the mark
                rec.tick(
                    time.time(), time.monotonic() + 0.01 * (i + 1), {"lidar": {"i": 100 + i}}, {}
                )
            for _ in range(50):
                if os.path.exists(path):
                    break
                time.sleep(0.05)
            lines = [json.loads(x) for x in gzip.open(path, "rt")]
        marks = [i for i, x in enumerate(lines) if "mark" in x]
        self.assertEqual(sum(1 for x in lines if "in" in x), 32)
        self.assertEqual(len(marks), 1)
        self.assertEqual(lines[marks[0]]["mark"], "wrong")
        self.assertTrue(all("in" in x for x in lines[marks[0] + 1 :]))  # only after-ticks follow


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
        self.assertEqual(
            (mark["mark"], mark["label"], mark["said"]), ("label", "wall", "Obstacle ahead")
        )
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
                    fh.write(
                        json.dumps(
                            {"t": 100 + i / 10, "mono": 50 + i / 10, "in": {"i": i}, "out": out}
                        )
                        + "\n"
                    )
                fh.write(
                    json.dumps(
                        {"t": 104, "mono": 54.0, "mark": "label", "label": "wall", "said": said}
                    )
                    + "\n"
                )
            rows = labels_in(path, replay=False, export_s=1.0)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["then"], {"instruction": said, "class": "obstacle"})
        self.assertEqual([t["in"]["i"] for t in rows[0]["ticks"]], list(range(30, 40)))
        self.assertNotIn("now", rows[0])

    def test_the_dashboard_has_a_button_for_every_label(self):
        import re
        from app.config import MARK_LABELS

        with open(
            os.path.join(os.path.dirname(__file__), "..", "templates", "dashboard.html")
        ) as fh:
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

            cls._ground = gs.run_scene(
                "wall_near", frames=3, with_camera=False, scenes=gs.WALL_SCENES
            )
        return {**cls._ground, "t": now}

    def test_flat_wide_surfaces_are_walls_and_people_and_furniture_are_not(self):
        from tools.sim import ground_hazard_sim as gs

        for name, want in (
            ("wall_angled", True),
            ("blind", True),
            ("room_corner", True),
            ("person", False),
            ("chair", False),
            ("cupboard", False),
        ):
            found, why, _s, _ms = gs.wall_result(name)
            self.assertEqual(found is not None, want, f"{name}: {why}")

    def test_a_wall_must_hold_over_frames_and_be_fresh(self):
        from app.perception.ground_hazards import confirmed_wall

        wall = {
            "ahead_m": 1.2,
            "length_m": 2.0,
            "flat_m": 0.01,
            "reaches_top": True,
            "height_m": 1.2,
        }
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
                sh.publish(
                    {
                        "left": None,
                        "right": None,
                        "ahead": None,
                        "posts": [(1450, 0, 250)],
                        "glass": None,
                    },
                    True,
                )
                lidar, radar = get_lidar_state(), get_radar_state()
                now = time.time() + i * 0.15
                ground = self.ground(now)
                if ground_at is not None:
                    moved = {**ground["front_surfaces"][-1], "ahead_m": ground_at}
                    ground = {**ground, "front_surfaces": [moved] * 3}
                got = sg.guidance(
                    walk_policy.decide(lidar, radar),
                    lidar,
                    now=now,
                    radar_state=radar,
                    camera_state={},
                    ground=ground,
                )
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
        return walk_policy.read_direction(
            get_lidar_state(), get_radar_state(), "front", tof_rest=rest
        )

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
                        fh.write(
                            json.dumps({"t": k, "mono": k, "in": {"camera": cam}, "out": out})
                            + "\n"
                        )
                    fh.write(
                        json.dumps({"t": 10, "mono": 10.0, "mark": "label", "label": "wall"}) + "\n"
                    )
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
        self.assertTrue(
            walk_policy._wall_too_far(60.0, 914.0 / math.sin(math.radians(60)), side, 792.0)
        )
        self.assertFalse(
            walk_policy._wall_too_far(60.0, 914.0 / math.sin(math.radians(60)), side, 850.0)
        )
        self.assertFalse(walk_policy._wall_too_far(10.0, 900.0, side, 500.0))  # not on the wall


class CloseWallOpenSideTests(unittest.TestCase):
    """2026-11-10: facing a flat wall 1.0-1.5 ft off in open space it said "turn around": each
    side board's beam heard the wall ahead at its edge (~1.86 x the front) and that echo was
    taken as the side's room, under the turn floor. An echo the laser places off the side's
    line, with that line clear past it, is not the side's room."""

    def _say(self, scene, camera=None, left=False):
        """What it says in this scene; `camera`: the cameras' state (none by default).
        With `left`, (sentence, the left's distance in feet)."""
        from tools.sim import scene_hunt as sh
        from app.core.shared_state import get_lidar_state, get_radar_state

        walk_policy._last_turn.update(direction=None, at=0.0)
        walk_policy._last_measured.clear()
        walk_policy._near_frames.clear()
        sh.publish(scene, True)
        lidar, radar = get_lidar_state(), get_radar_state()
        walk = walk_policy.decide(lidar, radar, camera or {})
        said = sg.decide(walk, lidar, time.time(), radar_state=radar, camera_state={})[
            "instruction"
        ]
        return (said, walk["directions"]["left"]["distance_ft"]) if left else said

    def test_the_open_side_is_offered(self):
        ft = 304.8
        for f in (1.0, 1.3, 1.5):
            with self.subTest(wall_ft=f):
                said = self._say(
                    {"left": None, "right": None, "ahead": f * ft, "posts": [], "glass": None}
                )
                self.assertEqual(said, sg.BLOCKED_WALL_TURN["right"])
        corner = {"left": 1.7 * ft, "right": None, "ahead": 1.0 * ft, "posts": [], "glass": None}
        self.assertEqual(
            self._say(corner), sg.BLOCKED_WALL_TURN["right"]
        )  # was "Stop, no clear way"

    def test_something_on_the_sides_line_still_closes_it(self):
        ft = 304.8
        post_right = {
            "left": None,
            "right": None,
            "ahead": 1.0 * ft,
            "glass": None,
            "posts": [(0.0, 1.9 * ft, 150.0)],
        }  # a post 1.9 ft to the right, abeam
        self.assertNotEqual(self._say(post_right), sg.BLOCKED_WALL_TURN["right"])

    def test_the_blind_side_keeps_the_radar(self):
        ft = 304.8  # the left is the LiDAR's blind arc: with no camera the radar is all there is
        corner = {"left": None, "right": 1.7 * ft, "ahead": 1.0 * ft, "posts": [], "glass": None}
        self.assertNotEqual(self._say(corner), sg.BLOCKED_WALL_TURN["left"])
        for f in (1.3, 1.5):
            corner["ahead"] = f * ft
            self.assertNotEqual(self._say(corner), sg.BLOCKED_WALL_TURN["left"])


class BlindSideCloseWallTests(unittest.TestCase):
    """2026-11-13 (design note 21): facing a close wall with only the left open it said "turn
    around": the left board hears the wall ahead at its beam's edge, and the laser cannot look
    left to place it. The echo is now that wall when the laser sees the wall ahead run straight
    on past the echo's distance, the echo is at its range and at least 2 ft off, and the left
    camera's fresh picture names nobody."""

    FT = 304.8

    def setUp(self):
        self.say = CloseWallOpenSideTests._say.__get__(self)

    def _cam(self, age=0.3, dets=(), dark=False, simulated=False):
        return {
            "left": {
                "detections": list(dets),
                "num_objects": len(dets),
                "frame_time": time.time() - age,
                "simulated": simulated,
                "view": {"dark": dark, "blurred": False},
            }
        }

    def _corner(self, ahead_ft, left_ft=None):
        return {
            "left": left_ft and left_ft * self.FT,
            "right": 1.7 * self.FT,
            "ahead": ahead_ft * self.FT,
            "posts": [],
            "glass": None,
        }

    def test_the_left_is_offered_when_its_camera_sees_nobody(self):
        for f in (1.3, 1.5):
            with self.subTest(wall_ft=f):
                said, left = self.say(self._corner(f), self._cam(), left=True)
                self.assertEqual(said, sg.BLOCKED_WALL_TURN["left"])
                self.assertGreater(left, 10.0)  # open: the wall's echo was the wall ahead

    def test_without_a_clear_picture_it_stays_turn_around(self):
        person = [{"class_name": "person", "confidence": 0.8, "bbox": [100, 50, 300, 800]}]
        for name, cam in (
            ("a person", self._cam(dets=person)),
            ("4 s old", self._cam(age=4.0)),
            ("dark", self._cam(dark=True)),
            ("simulated", self._cam(simulated=True)),
            ("no camera", {}),
        ):
            with self.subTest(camera=name):
                self.assertEqual(self.say(self._corner(1.3), cam), sg.BLOCKED_WALL_TURN["around"])

    def test_a_wall_close_on_the_left_is_kept(self):
        said, left = self.say(self._corner(1.0, left_ft=1.7), self._cam(), left=True)
        self.assertNotEqual(said, sg.BLOCKED_WALL_TURN["left"])
        self.assertAlmostEqual(left, 1.7, delta=0.1)
        # Farther: the wall ahead's echo goes, the side wall's stays as the left's room.
        said, left = self.say(self._corner(1.3, left_ft=3.0), self._cam(), left=True)
        self.assertEqual(said, sg.BLOCKED_WALL_TURN["left"])
        self.assertAlmostEqual(left, 3.0, delta=0.1)

    def test_an_echo_under_2_ft_is_never_dropped(self):
        # 1.0 ft from the wall its echo is 1.9 ft off: a post there the camera missed would
        # leave under the 2 ft every turn needs, so it stays "turn around".
        self.assertEqual(self.say(self._corner(1.0), self._cam()), sg.BLOCKED_WALL_TURN["around"])

    def test_side_camera_clear(self):
        now = time.time()
        clear = walk_policy.side_camera_clear
        self.assertTrue(clear(self._cam(), "left", now))
        self.assertFalse(clear(self._cam(), "right", now))  # no right camera in it
        self.assertFalse(clear(self._cam(age=3.0), "left", now))
        self.assertFalse(clear(self._cam(age=-5.0), "left", now))  # from the future
        self.assertFalse(clear(None, "left", now))
        self.assertFalse(clear(self._cam(dets=[{"class_name": "dog"}]), "left", now))


class WallBesideAndSomethingOnThePathTests(unittest.TestCase):
    """2026-11-13, the scene hunt: a block put down to the wall beside the wearer ("Near right
    wall, move left") with something else on the path within 4 ft. A pillar standing out of
    the right wall, fitted into one tilted "wall" with it; and a post on the path's left half
    beside a wall inside the path on the right, which "move left" steered into. A block is the
    wall beside you only when nothing else is on the path (_else_on_the_path)."""

    def _say(self, scene):
        from tools.sim import scene_hunt as sh

        sg.reset()
        return sh.decide(scene)[1]["instruction"]

    def _scene(self, left, right, posts, glass=None):
        return {"left": left, "right": right, "ahead": None, "glass": glass, "posts": posts}

    def test_a_pillar_out_of_the_wall_blocks(self):
        for scene in (
            self._scene(None, 454, [(3695, 1180, 199), (1559, 283, 239), (925, 402, 158)], 3446),
            self._scene(1527, 453, [(900, 473, 259)], 2145),
        ):
            with self.subTest(scene=scene["posts"]):
                said = self._say(scene)
                self.assertGreaterEqual(sg._LEVEL.get(said, -1), sg._BLOCKED, said)
                self.assertNotEqual(said, sg.NEAR_RIGHT)

    def test_never_moved_toward_a_post_on_the_path(self):
        said = self._say(self._scene(None, 230, [(908, -296, 188), (2763, -1404, 49)], 1315))
        self.assertGreaterEqual(sg._LEVEL.get(said, -1), sg._BLOCKED, said)
        # The post is ahead on the left half: never "move left" (walking on, toward it). A
        # stop and a turn to face the open left takes it off the path, and is fine.
        self.assertNotIn("move left", said)
        self.assertNotEqual(said, sg.NEAR_RIGHT)

    def test_a_wall_alone_is_still_the_wall_beside_you(self):
        for right in (230, 280):  # inside the walking path, nothing else on it
            with self.subTest(right=right):
                self.assertEqual(self._say(self._scene(None, right, [])), sg.NEAR_RIGHT)


class _GlassDoorScenes:
    """A glass door in a wall, met at an angle, maybe someone beside it, as the boards hear it
    (glass_sim.door_scene)."""

    def _door(self, ft, angle=0.0, person=None):
        """(lidar, radar) states; person = (lat mm along the wall from the door's centre, dz mm:
        negative, in front of it)."""
        from tools.sim import glass_sim as gs

        segments, echoes = gs.door_scene(
            ft, angle, None if person is None else (person[0], -person[1])
        )
        walk_policy._near_frames.clear()
        return gs.lidar_state(segments), gs.radar_state(echoes)


class TwoPostsAnOpenWayTests(_GlassDoorScenes, unittest.TestCase):
    """2026-11-13, the scene hunt: two posts either side of an open way read as the wall a
    glass door is set in ("Glass ahead", "Blocked by glass, turn right" with no glass). A pane
    lies on its wall's line, and each front board hears that line where it comes nearest it;
    two posts are heard each by its own board at its own range (_pane_on_wall_line)."""

    def test_posts_either_side_of_an_open_way_are_not_glass(self):
        from tools.sim import scene_hunt as sh

        for scene in (  # both glass-door rules, and a "Blocked by glass, turn right"
            {"posts": [(1773, -1092, 62), (2034, 682, 247), (3714, -624, 142)]},
            {"posts": [(3764, -1041, 196), (3599, 1134, 212), (2586, 1209, 259)]},
            {"posts": [(1029, 605, 156), (718, -527, 162)]},
        ):
            scene.update(left=None, right=None, ahead=None, glass=None)
            with self.subTest(posts=scene["posts"]):
                sg.reset()
                walk, raw = sh.decide(scene)
                self.assertFalse(walk["directions"]["front"]["glass"])
                self.assertNotIn(raw["instruction"], sh.GLASS_WORDS)

    def test_a_glass_door_with_someone_beside_it_is_still_glass(self):
        # Just in front of the wall beside the door, they tilt the line the laser sees; both
        # boards still hear the pane alike.
        for ft in (4.5, 6.0):
            for lat in (-650.0, 650.0):
                with self.subTest(ft=ft, lat=lat):
                    lidar, radar = self._door(ft, person=(lat, -250.0))
                    self.assertTrue(walk_policy.read_direction(lidar, radar, "front")["glass"])

    def test_the_pane_lies_on_its_walls_line_at_any_angle(self):
        for angle in (0.0, 10.0, -10.0, 20.0, -20.0):
            for ft in (2.0, 4.5, 8.0):
                with self.subTest(angle=angle, ft=ft):
                    lidar, radar = self._door(ft, angle)
                    echo = min(e for _b, e in walk_policy._board_echoes(radar, "front", True))
                    on_line = walk_policy._pane_on_wall_line(lidar, radar, echo)
                    # Never ruled out. Close and at 20 deg, the near jamb is seen too edge-on
                    # to face the sensor: one side only, nothing to judge by (None).
                    self.assertIsNot(on_line, False)
                    if abs(angle) <= 10.0 or ft >= 4.5:
                        self.assertTrue(on_line)
        # Two posts: each board hears its own, nowhere near the line through them.
        from tools.sim import scene_hunt as sh
        from app.core.shared_state import get_lidar_state, get_radar_state

        sh.publish(
            {
                "left": None,
                "right": None,
                "ahead": None,
                "glass": None,
                "posts": [(1773, -1092, 62), (2034, 682, 247)],
            },
            True,
        )
        self.assertIs(
            walk_policy._pane_on_wall_line(get_lidar_state(), get_radar_state(), 1899.0), False
        )


class EchoTheLaserSeesTests(unittest.TestCase):
    """2026-11-17, the scene hunt: what the front boards heard was named glass though the
    laser sees it. (1) The front arc's outer sectors reach ~5 deg past the boards' beams: a
    post there, nearer than what they heard, hid that the laser sees that too
    (_each_board_explained). (2) A range both boards hear alike at what the laser ranges
    straight ahead kept two posts "a wall with a pane in it" (_pane_heard_on, `solid_mm`)."""

    ROOMS = {  # scene hunt seed 1: numbers 312 and 1203 are (1), 964 and 2435 are (2)
        "5 m hall, walls heard at the beams' edges, a post past the right one": dict(
            left=2764, right=2456, ahead=None, posts=[(587, 1310, 82)]
        ),
        "a post just past the left beam's edge": dict(
            left=None,
            right=None,
            ahead=None,
            posts=[(2052, -1052, 92), (321, -1172, 223), (3115, 816, 98)],
        ),
        "a post straight ahead, one nearer either side": dict(
            left=None,
            right=None,
            ahead=None,
            posts=[(3965, -166, 131), (3016, -823, 273), (3039, 851, 48)],
        ),
        "a wall ahead, posts either side": dict(
            left=None,
            right=None,
            ahead=3179,
            posts=[(1812, -581, 247), (1918, 1186, 241), (3214, 569, 226)],
        ),
    }

    def test_what_the_laser_sees_is_not_glass(self):
        from tools.sim import scene_hunt as sh

        for name, room in self.ROOMS.items():
            with self.subTest(name):
                sg.reset()
                walk, raw = sh.decide({**room, "glass": None})
                self.assertFalse(walk["directions"]["front"]["glass"])
                self.assertNotIn(raw["instruction"], sh.GLASS_WORDS)

    def test_heard_alike_at_the_lasers_own_range_is_no_pane(self):
        from tools.sim import scene_hunt as sh
        from app.core.shared_state import get_lidar_state, get_radar_state

        sh.publish(
            {**self.ROOMS["a post straight ahead, one nearer either side"], "glass": None}, True
        )
        # Both boards hear the post ahead at 3.84 m, where the laser ranges it: was None
        # ("heard alike, nothing to judge by"), and the posts either side a door's frame.
        self.assertIs(
            walk_policy._pane_on_wall_line(get_lidar_state(), get_radar_state(), 2856.0), False
        )

    def test_a_pane_in_those_rooms_is_still_glass(self):
        # Nearer than the hall's walls are to the side: farther, a wall at a beam's edge
        # echoes from the pane's range too (docs/known-limits.md), as it did before.
        from tools.sim import scene_hunt as sh

        for name, room in self.ROOMS.items():
            for pane in (1000.0, 1750.0, 2250.0):
                with self.subTest(name, pane=pane):
                    sg.reset()
                    walk, _raw = sh.decide({**room, "glass": pane})
                    self.assertTrue(walk["directions"]["front"]["glass"])


class NoTurnIntoAWallTests(unittest.TestCase):
    """2026-11-18, the scene hunt and a sweep of passages 0.4-2.4 m wide: turns toward
    something under 2 ft off. (1) A wall brushing the path on one side, the other side near
    too, said "Turn" away from it whatever the room that way: in a 0.65-0.8 m passage "Turn
    right", at a wall 1.1-1.6 ft off (simple_guidance, tight_turn). (2) On the left, which the
    laser cannot see down, a wall nearer than the radar starts measuring (RADAR_START_MM) read
    as nothing within 13 ft, and "Blocked by wall, turn left" went into it; so did a post at
    the front-left, its edge in the left board's beam (walk_policy._under_radar_start_mm)."""

    @staticmethod
    def _say(**scene):
        from tools.sim import scene_hunt as sh

        sg.reset()
        room = {"left": None, "right": None, "ahead": None, "glass": None, "posts": []}
        walk, raw = sh.decide({**room, **scene})
        return walk, raw["instruction"]

    def _not_toward(self, side, said):
        self.assertNotEqual(said, sg.TURN_LEFT if side == "left" else sg.TURN_RIGHT)
        self.assertNotEqual(sg._WAY_OUT.get(said), side)

    def test_no_turn_away_from_a_wall_into_another(self):
        for right in (350, 400, 450, 500):
            with self.subTest(right=right):
                self._not_toward("right", self._say(left=300, right=right)[1])
        # Room enough that way: still the turn away from the wall.
        self.assertEqual(self._say(left=300, right=900)[1], sg.TURN_RIGHT)

    def test_a_wall_nearer_than_the_radar_measures(self):
        for right in (250, 350, 900, 1500):
            with self.subTest(right=right):
                walk, said = self._say(left=200, right=right)
                left = walk["directions"]["left"]
                self.assertFalse(left["open"])  # was "13.1 ft (nothing detected)"
                self.assertAlmostEqual(left["under_radar_ft"], 200 / 304.8, delta=0.05)
                self.assertEqual(left["distance_ft"], left["under_radar_ft"])
                self._not_toward("left", said)

    def test_a_post_at_the_front_left_in_the_left_beam(self):
        rooms = (  # scene hunt seed 11 #957 and seed 12 #234 (ghosts 0.5)
            dict(right=374, posts=[(3718, -912, 45), (334, -86, 245)]),
            dict(right=560, posts=[(348, -173, 247)]),
        )
        for room in rooms:
            with self.subTest(**room):
                walk, said = self._say(**room)
                self.assertIsNotNone(walk["directions"]["left"]["under_radar_ft"])
                self._not_toward("left", said)

    def test_only_what_is_in_the_left_beam(self):
        # A post ahead on the right, its edge just over the middle: not the left's wall. A
        # first try took it for one, and turned the wearer right, into the post.
        walk, said = self._say(ahead=3333, posts=[(320, 173, 212)])  # seed 1 #1393
        self.assertIsNone(walk["directions"]["left"]["under_radar_ft"])
        self.assertEqual(sg._WAY_OUT.get(said), "left")


class ConsoleRoomTests(unittest.TestCase):
    """2026-11-18, on the desk: "Blocked by wall, turn left" with the laser's left wall 1.5 ft
    off, and nothing in the console line said what room the left was judged to have. A block's
    line now shows the walk policy's room each way and what a turn needs."""

    def test_a_block_shows_the_room_each_way(self):
        from app.guidance import test_console as tc

        raw = {
            "blocked": True,
            "front_ft": 1.0,
            "turn_room": {"left": 13.1, "right": 1.4},
            "turn_floor": 2.0,
        }
        self.assertIn("room L 13.1 R 1.4 ft (needs 2.0)", tc._evidence(raw))
        raw["turn_room"] = {"left": float("-inf"), "right": 1.4}  # no data that way
        self.assertIn("room L ? R 1.4 ft", tc._evidence(raw))

    def test_not_on_the_way_ahead(self):
        from app.guidance import test_console as tc

        raw = {"blocked": False, "front_ft": 9.0, "turn_room": {"left": 13.1, "right": 1.4}}
        self.assertNotIn("room", tc._evidence(raw))


class HeldPaneBlocksAtOnceTests(unittest.TestCase):
    """2026-11-18, tools/sim/walk_sim (the spoken path, walking in at 1.2 m/s): "Blocked by
    glass" came at 3.1 ft, a wall's at 3.8. A block only the radar measures waited
    SIMPLE_ALERT_CONFIRM_S (0.2 s), on top of a radar frame's age (RADAR_UPDATE_RATE). A pane
    already held as glass that comes within the block now blocks at once, as a block the
    laser measures does; one first heard inside the block still waits."""

    def tearDown(self):
        sg.reset()

    @staticmethod
    def _raw(pane_mm):
        from tools.sim import scene_hunt as sh

        room = {"left": None, "right": None, "ahead": None, "glass": pane_mm, "posts": []}
        return sh.decide(room)[1]

    @staticmethod
    def _said(polls):
        world = sg._World()
        return [world.update(dict(raw), t)[0] for t, raw in polls]

    def test_a_held_pane_blocks_at_once(self):
        far, near = self._raw(2400.0), self._raw(900.0)
        self.assertEqual(far["instruction"], sg.GLASS_AHEAD)
        said = self._said([(1000.0 + k * 0.05, far) for k in range(20)] + [(1001.0, near)])
        self.assertEqual(said[-2], sg.GLASS_AHEAD)
        self.assertIn(said[-1], sg.BLOCKED_GLASS_ANY)  # was 0.2 s later

    def test_a_pane_first_heard_inside_the_block_waits(self):
        near = self._raw(900.0)
        said = self._said([(1000.0 + k * 0.05, near) for k in range(6)])
        self.assertNotIn(said[2], sg.BLOCKED_GLASS_ANY)  # 0.1 s in
        self.assertIn(said[-1], sg.BLOCKED_GLASS_ANY)  # 0.25 s in

    def test_walking_in_a_pane_blocks_as_a_wall_does(self):
        from tools.sim import walk_sim as ws

        for name in ("glass across open floor", "wall across"):
            with self.subTest(name):
                ticks = ws.walk(ws.SCENES[name])
                self.assertEqual(ws.judge(name, ws.SCENES[name], ticks, False), [])
                first = next(
                    min(d for d in (line, pane) if d is not None)
                    for _t, _w, line, pane, text, spoken in ticks
                    if spoken and sg._LEVEL.get(text, -1) >= sg._BLOCKED
                )
                self.assertGreaterEqual(first / 304.8, 3.5)  # the pane's was 3.1 ft


class GlassDoorAtAnAngleTests(_GlassDoorScenes, unittest.TestCase):
    """2026-11-13: a glass door met 15-20 deg off said "Walk straight" from 3 to 6 ft. The
    tilted wall it is set in comes nearer than the pane past 45 deg on one side, and was taken
    for a wall beside the wearer (_door_in_tilted_wall)."""

    def test_found_at_an_angle_and_blocked_at_the_glass(self):
        for angle in (15.0, -15.0, 20.0, -20.0):
            for ft in (3.0, 3.5, 4.5, 6.0):
                with self.subTest(angle=angle, ft=ft):
                    lidar, radar = self._door(ft, angle)
                    front = walk_policy.read_direction(lidar, radar, "front")
                    self.assertTrue(front["glass"])
                    # The pane's nearest point: its distance times cos(angle).
                    near_ft = ft * math.cos(math.radians(angle))
                    self.assertAlmostEqual(front["distance_ft"], near_ft, delta=0.2)

    def test_a_plain_tilted_wall_is_a_wall(self):
        from tools.sim import glass_sim as gs

        for angle in (15.0, 20.0, 30.0):
            for ft in (3.0, 4.5, 6.0):
                with self.subTest(angle=angle, ft=ft):
                    d = ft * gs.FT
                    wall = gs.wall_across(d, angle_deg=angle)
                    echo = d * math.cos(math.radians(angle))
                    walk_policy._near_frames.clear()
                    front = walk_policy.read_direction(
                        gs.lidar_state(wall),
                        gs.radar_state({"deg030": echo, "deg330": echo}),
                        "front",
                    )
                    self.assertFalse(front["glass"])
                    self.assertFalse(
                        walk_policy._door_in_tilted_wall(
                            gs.lidar_state(wall),
                            gs.radar_state({"deg030": echo, "deg330": echo}),
                            echo / gs.FT,
                            front["lidar_ft"],
                            front["lidar_measured"],
                        )
                    )


class GlassDoorWithSomeoneNearTests(_GlassDoorScenes, unittest.TestCase):
    """2026-11-13: a glass door with someone standing 0.5-0.8 m to its side, in front of its
    wall, said "Walk straight" in 91 of 384 simulated scenes: the person, the nearest echo,
    was taken for what the boards heard, and the pane's own echo, farther, was judged with
    the person mixed into "the wall" (_wall_among, _door_on_line)."""

    def test_the_pane_behind_the_person_is_glass_at_its_range(self):
        # (angle, ft, person along the wall, in front of it): each missed before
        for angle, ft, lat, dz in (
            (0.0, 4.5, -850.0, -250.0),
            (10.0, 3.0, 850.0, -250.0),
            (20.0, 6.0, -650.0, -400.0),
            (20.0, 6.0, 650.0, -250.0),
        ):
            with self.subTest(angle=angle, ft=ft, lat=lat, dz=dz):
                lidar, radar = self._door(ft, angle, person=(lat, dz))
                front = walk_policy.read_direction(lidar, radar, "front")
                self.assertTrue(front["glass"])
                # Never later than the pane; a little early where the person, within
                # GLASS_PANE_ON_LINE_TOL_MM of the wall's line, is the nearer echo.
                near_ft = ft * math.cos(math.radians(angle))
                self.assertLessEqual(front["distance_ft"], near_ft + 0.2)
                self.assertGreaterEqual(front["distance_ft"], near_ft - 0.7)

    def test_named_by_the_block_distance_walking_in(self):
        for angle in (0.0, 20.0, -20.0):
            for lat, dz in ((850.0, -250.0), (-850.0, -100.0), (650.0, -400.0)):
                named = []
                for ft in (8.0, 6.0, 4.5, 4.0):
                    lidar, radar = self._door(ft, angle, person=(lat, dz))
                    named.append(walk_policy.read_direction(lidar, radar, "front")["glass"])
                with self.subTest(angle=angle, lat=lat, dz=dz):
                    self.assertTrue(any(named), named)

    def test_a_wall_ahead_keeps_its_distance(self):
        # A side wall's echo at a beam's edge is not "a door" in the wall ahead: the wall
        # the boards hear on its own line, which the laser ranges (first version: 6.9 -> 6.2 ft).
        from tools.sim import scene_hunt as sh

        sg.reset()
        walk, _raw = sh.decide(
            {"left": None, "right": 1659, "ahead": 2104, "glass": None, "posts": []}
        )
        front = walk["directions"]["front"]
        self.assertFalse(front["glass"])
        self.assertAlmostEqual(front["distance_ft"], 2104 / 304.8, delta=0.1)


class CameraExposureCapTests(unittest.TestCase):
    """2026-11-11: walking blurred the camera's 24-30 ms exposures; the cap keeps it at
    CAMERA_SHUTTER_CAP_MS while gain makes up the light, and the most light as before."""

    def test_the_short_profile_caps_then_keeps_the_most_light(self):
        try:
            import picamera2  # noqa: F401
        except ImportError:
            self.skipTest("picamera2 not installed (not a Pi)")
        from app.sensors import pi_camera_utils as pc

        normal = {"shutter": [100, 10000, 30000, 60000, 120000], "gain": [1.0, 2.0, 4.0, 6.0, 8.0]}
        tuning = {
            "version": 2.0,
            "algorithms": [
                {"rpi.agc": {"channels": [{"exposure_modes": {"normal": dict(normal)}}]}}
            ],
        }
        with (
            unittest.mock.patch("picamera2.Picamera2.load_tuning_file", lambda name: tuning),
            unittest.mock.patch.object(pc, "CAMERA_SHUTTER_CAP_MS", 10.0),
        ):
            got, ctrls = pc._short_exposure()
        short = got["algorithms"][0]["rpi.agc"]["channels"][0]["exposure_modes"]["short"]
        self.assertEqual(short["shutter"], [100.0, 10000.0, 10000.0, 10000.0, 120000.0])
        self.assertEqual(short["gain"], [1.0, 2.0, 4.0, 8.0, 8.0])  # then longer, as before
        self.assertIn("AeExposureMode", ctrls)
        with unittest.mock.patch.object(pc, "CAMERA_SHUTTER_CAP_MS", None):
            self.assertEqual(pc._short_exposure(), (None, {}))  # off: the sensor's own


class StuckCameraStopTests(unittest.TestCase):
    """2026-11-20, on the device: the picture stream stalled; its reopen hung in stop(),
    abandoned after CAMERA_STOP_TIMEOUT_S, and libcamera still held that camera, so every
    reopen after it failed ("Camera in Running state trying acquire()"), every 10 s, until the
    app restarted. The captures still waiting on a camera are cancelled before it is stopped
    (Picamera2's cancel_all_and_flush, its own advice for a camera that stopped answering);
    and while a stop is still stuck, a reopen is tried every CAMERA_STUCK_RETRY_S, saying why."""

    class Cam:
        def __init__(self, hang=None):
            self.calls, self.hang = [], hang

        def cancel_all_and_flush(self):
            self.calls.append("cancel")

        def stop(self):
            self.calls.append("stop")
            if self.hang is not None:
                self.hang.wait(5.0)

        def close(self):
            self.calls.append("close")

    def test_a_close_cancels_the_waiting_captures_first(self):
        from app.sensors import pi_camera_utils as pc

        cam = self.Cam()
        self.assertTrue(pc._close_camera(cam))
        self.assertEqual(cam.calls, ["cancel", "stop", "close"])
        self.assertFalse(pc._stuck_close.is_set())

    def test_a_stop_that_hangs_is_marked_stuck_until_it_lets_go(self):
        from app.sensors import pi_camera_utils as pc

        hang = threading.Event()
        cam = self.Cam(hang)
        with unittest.mock.patch.object(pc, "CAMERA_STOP_TIMEOUT_S", 0.2):
            self.assertFalse(pc._close_camera(cam))
        self.assertTrue(pc._stuck_close.is_set())
        hang.set()  # it lets go, late
        for _ in range(50):
            if not pc._stuck_close.is_set():
                break
            time.sleep(0.02)
        self.assertFalse(pc._stuck_close.is_set())
        self.assertEqual(cam.calls, ["cancel", "stop", "close"])

    def test_while_stuck_a_reopen_waits_long_and_says_why(self):
        try:
            import picamera2  # noqa: F401
        except ImportError:
            self.skipTest("picamera2 not installed (not a Pi)")
        from app.sensors import pi_camera_utils as pc

        def refuse(*a, **kw):
            raise RuntimeError("Camera __init__ sequence did not complete.")

        pc._stuck_close.set()
        try:
            with (
                unittest.mock.patch("picamera2.Picamera2", refuse),
                unittest.mock.patch.object(pc, "_set_board_mode", lambda: True),
                unittest.mock.patch.object(pc, "_short_exposure", lambda: (None, {})),
                unittest.mock.patch.object(pc, "running", True),
                self.assertLogs(pc._log, level="WARNING") as logs,
            ):
                opener = threading.Thread(target=pc._open_camera, daemon=True)
                opener.start()
                time.sleep(0.5)
                pc.running = False  # stopped: the wait ends within 0.25 s
                opener.join(2.0)
            self.assertFalse(opener.is_alive())
            said = " ".join(logs.output)
            self.assertIn("libcamera still holds it", said)
            self.assertIn(f"retrying in {pc.CAMERA_STUCK_RETRY_S:g}s", said)
        finally:
            pc._stuck_close.clear()


class SpokenOnlyOnTheDashboardTests(unittest.TestCase):
    """2026-11-11: the dashboard's "Spoken this run" listed the silent "would say" lines
    (route planner, ToF hazards, movers) and the tester's marks among what was said."""

    def test_only_what_was_spoken_is_listed(self):
        from app.api.guidance_router import field_test_snapshot
        from app.guidance import speech_log

        with unittest.mock.patch.object(speech_log, "SPEECH_LOG_TO_FILE", False):
            speech_log.reset()
            t = time.time()
            speech_log.say(
                "guidance",
                sg.BLOCKED_WALL_TURN["right"],
                level="urgent",
                now=t,
                via="phone",
                timing={"scan_age_ms": 59},
            )
            speech_log.say("route-planner", f"would say: {sg.TURN_RIGHT}", now=t + 0.1)
            speech_log.say("tof-hazards", f"would say: {sg.OBSTACLE_AHEAD}", now=t + 0.2)
            speech_log.say("tester", "WRONG", now=t + 0.3)
            speech_log.say("guidance", sg.WALK_STRAIGHT, now=t + 0.4)
            got = field_test_snapshot(limit=50)
            speech_log.reset()
        self.assertEqual(
            [r["text"] for r in got["lines"]], [sg.BLOCKED_WALL_TURN["right"], sg.WALK_STRAIGHT]
        )
        self.assertEqual(got["lines"][0]["timing"]["scan_age_ms"], 59)
        self.assertEqual(got["said_total"], 2)


class GroundAtTheMountHeightTests(unittest.TestCase):
    """2026-11-11: set down on a desk, the ToF took surfaces 0.45-0.96 m below it for the
    ground and confirmed a "hole" 0.65 m ahead: "Pothole ahead" (would say). The ground is
    TOF_MOUNT_HEIGHT_M below the camera, give or take TOF_MOUNT_HEIGHT_TOL_M."""

    def _snap(self, *boxes):
        from tools.sim import ground_hazard_sim as gs

        scenes = {"s": (lambda: list(boxes), None)}
        return gs.run_scene("s", frames=4, with_camera=False, scenes=scenes)

    def test_set_down_on_a_desk_nothing_on_the_ground_is_judged(self):
        import numpy as np
        from app.perception import depth_fusion as df
        from app.perception import ground_hazards as gh
        from tools.sim import ground_hazard_sim as gs

        # As the rig lay on the desk (RestingOnADeskTests), with a dip in the floor ahead
        hz = gh.GroundHazards(df.DepthProjector(gs.CAL))
        H, below = gs.H, float(gs.CAL.t[1]) + 0.02
        boxes = [
            gs.floor(z1=1.6),
            gs.slab(-6, 6, -0.2, 1.6, 2.1),
            gs.floor(z0=2.1),
            gs.slab(-1.0, 1.0, H - below, -0.5, 0.6, bottom=H - 0.75),
        ]
        rng = np.random.default_rng(0)
        for i in range(8):
            depth, conf = gs.render_tof(boxes, pitch=-46.0, rng=rng)
            snap = hz.step(depth, conf, 1000.0 + i * 0.1, None)
        self.assertIsNotNone(snap["rest"])
        self.assertFalse(snap["ok"])
        self.assertIn("set down", snap["why"])
        self.assertEqual(snap["hazards"], [])

    def test_a_level_surface_far_from_the_mount_height_is_not_the_ground(self):
        from tools.sim import ground_hazard_sim as gs

        # Carried low or set on a seat: a level floor 0.6 m below is not the ground
        snap = self._snap(gs.floor(top=gs.H - 0.6))
        self.assertFalse(snap["ok"])
        self.assertEqual(snap["hazards"], [])

    def test_the_floor_and_a_real_hole_are_still_found(self):
        from tools.sim import ground_hazard_sim as gs

        snap = self._snap(*gs.hole())
        self.assertTrue(snap["ok"])
        self.assertIn("hole", [h["kind"] for h in snap["hazards"]])


class PillarOutOfAWallTests(unittest.TestCase):
    """2026-11-11 (the scene hunt's last "no block"): a pillar standing out of the right wall
    1 ft ahead, its edge 0.1 m off the walking line, was "Near right wall, move left". Its face
    was fitted as a "wall" 28 deg across the path; a wall beside you runs along it."""

    def _say(self, scene):
        from tools.sim import scene_hunt as sh

        return sh.decide({"left": 2492.0, "ahead": 4585.0, "glass": None, **scene})[1][
            "instruction"
        ]

    def test_the_pillar_blocks(self):
        said = self._say({"right": 378.0, "posts": [(528.0, 320.0, 211.0)]})
        self.assertTrue(sg._is_block(said), said)

    def test_a_plain_wall_beside_is_still_a_near_wall(self):
        for lat in (250.0, 378.0):
            with self.subTest(lateral_mm=lat):
                self.assertEqual(self._say({"right": lat, "posts": []}), sg.NEAR_RIGHT)


class RadarClosingEchoTests(unittest.TestCase):
    """2026-11-11 (radar target tracking, as cars do): a pane across a narrow passage, farther
    than its walls are to the side, by its echo closing at walking pace on both front boards
    while the walls' echoes stay put. Logged as "would say" (GLASS_CLOSING_SPEAK False)."""

    FT = 304.8

    def _found(self, frames):
        return [(p, r["closing_echo"]) for _w, p, r in frames if r.get("closing_echo")]

    def test_a_pane_across_a_narrow_passage_is_found_as_it_closes(self):
        from tools.sim import glass_sim as gs

        for walls in ((457, 457), (914, 914), (600, 366), (366, 600)):
            with self.subTest(walls=walls):
                found = self._found(gs.walk_toward(*walls, 10 * self.FT, seconds=8))
                self.assertTrue(found)
                pane, c = found[0]
                self.assertAlmostEqual(c["closing_mps"], 1.2, delta=0.15)
                self.assertAlmostEqual(c["mm"], pane, delta=150.0)

    def test_spoken_it_is_glass_all_the_way_in(self):
        from tools.sim import glass_sim as gs

        for walls in ((457, 457), (914, 914), (600, 366)):
            with (
                self.subTest(walls=walls),
                unittest.mock.patch.object(walk_policy, "GLASS_CLOSING_SPEAK", True),
            ):
                frames = gs.walk_toward(*walls, 10 * self.FT, seconds=8)
                missed = [
                    round(p / self.FT, 1)
                    for _w, p, r in frames
                    if p and p >= 0.5 * self.FT and not r["glass"]
                ]
                self.assertEqual(missed, [])
        with unittest.mock.patch.object(
            walk_policy, "GLASS_CLOSING_SPEAK", False
        ):  # the gap it fills
            frames = gs.walk_toward(457, 457, 10 * self.FT, seconds=8)
            self.assertTrue(any(p and p >= 0.5 * self.FT and not r["glass"] for _w, p, r in frames))

    def test_walls_a_door_frame_and_standing_still_are_not(self):
        import math
        from tools.sim import glass_sim as gs

        frame = lambda key: lambda w: {key: [math.hypot(3000 - w, 457)]} if w < 3000 else {}  # noqa: E731
        cases = {
            "empty 0.9 m passage": gs.walk_toward(457, 457, None),
            "empty 1.8 m passage": gs.walk_toward(914, 914, None),
            "empty nook": gs.walk_toward(600, 366, None),
            "a door frame on the right": gs.walk_toward(457, 457, None, extra=frame("deg030")),
            "a door frame on the left": gs.walk_toward(457, 457, None, extra=frame("deg330")),
            "standing before a pane": gs.walk_toward(457, 457, 6 * self.FT, pace=0.0),
        }
        for seed in range(5):  # and with range noise, a board missing one frame in ten
            cases[f"noisy empty passage {seed}"] = gs.walk_toward(
                457, 457, None, noise_mm=20.0, drop=0.1, seed=seed
            )
        for name, frames in cases.items():
            with self.subTest(name):
                self.assertEqual(self._found(frames), [])

    def test_it_is_said_as_would_say(self):
        from tools.sim import glass_sim as gs

        frames = gs.walk_toward(457, 457, 10 * self.FT, seconds=8)
        r = next(r for _w, _p, r in frames if r.get("closing_echo") and not r["glass"])
        wd = {
            "directions": {"front": r},
            "glass_ahead": False,
            "blocked": False,
            "action": "walk_straight",
        }
        raw = sg.decide(wd, None, time.time(), radar_state=None, camera_state={})
        self.assertEqual(
            raw["instruction"], sg.WALK_STRAIGHT
        )  # not said: not yet checked on a walk
        self.assertEqual(raw["closing_would_say"], sg.GLASS_AHEAD)
        self.assertIn("closing at", raw["closing_reason"])


class PaneAheadNotBesideOnAMissedFrameTests(unittest.TestCase):
    """2026-11-19, tools/sim/walk_sim with radar frames dropped (--drop 0.2): walking at a pane
    across a 3 m hall, one front board missed a frame, the other alone heard the pane, and
    glass_beside read it as glass BESIDE: the block was dropped ("Glass on right, move left",
    the front the laser's 39 ft), the side flipped with the board that missed, and "Walk
    straight" stood to 1 ft. Both boards heard it alike a moment before, where it was as the
    wearer walked toward it: a pane ahead (walk_policy.pane_heard_alike_lately)."""

    def setUp(self):
        walk_policy._near_frames.clear()
        walk_policy._board_peaks.clear()

    tearDown = setUp

    @staticmethod
    def _frame(echoes, stamp):
        from tools.sim import glass_sim as gs

        rs = gs.radar_state(echoes)
        for row in rs["sectors"].values():
            row["last_update"] = stamp
        rs["sectors"]["front"]["peaks_mm"] = sorted(
            e for k in ("deg030", "deg330") for e in rs["sectors"][k]["peaks_mm"]
        )
        walk_policy._near_echo_steady(rs)  # recorded, as read_direction records each frame
        return rs

    def test_a_pane_ahead_one_board_missing_a_frame(self):
        self._frame({"deg330": [1560.0, 1680.0], "deg030": [1565.0, 1690.0]}, 1000.0)
        now = self._frame({"deg330": None, "deg030": [1360.0, 1690.0]}, 1000.17)
        self.assertIsNone(sg.glass_beside(now))  # was "right"

    def test_glass_beside_is_still_beside(self):
        for k in range(4):  # only the right board ever hears it; the left hears nothing
            now = self._frame({"deg330": None, "deg030": [1300.0]}, 1000.0 + k / 6.0)
        self.assertEqual(sg.glass_beside(now), "right")

    def test_not_from_long_ago(self):
        self._frame({"deg330": [1560.0], "deg030": [1565.0]}, 1000.0)
        now = self._frame({"deg330": None, "deg030": [1300.0]}, 1001.5)  # past GLASS_BESIDE_ALIKE_S
        self.assertEqual(sg.glass_beside(now), "right")

    def test_not_from_frames_of_another_clock(self):
        # Frames left in _board_peaks from a run on a later clock, _near_frames cleared: not
        # "lately" for this one (review: walk_sim and the tests clear _near_frames alone).
        self._frame({"deg330": [1330.0], "deg030": [1335.0]}, 2000.0)
        walk_policy._near_frames.clear()
        now = self._frame({"deg330": None, "deg030": [1300.0]}, 1000.0)
        self.assertEqual(sg.glass_beside(now), "right")

    def test_not_from_a_history_that_stopped(self):
        # Aged from THIS radar frame, not the newest recorded (review).
        self._frame({"deg330": [1330.0], "deg030": [1335.0]}, 1000.0)
        from tools.sim import glass_sim as gs

        now = gs.radar_state({"deg330": None, "deg030": [1300.0]})
        for row in now["sectors"].values():
            row["last_update"] = 1005.0  # not recorded
        self.assertEqual(sg.glass_beside(now), "right")

    def test_walking_in_with_frames_missed(self):
        from tools.sim import walk_sim as ws

        name = "glass across a 3 m hall"
        i = list(ws.SCENES).index(name)  # as the command line runs it
        ticks = ws.walk(
            ws.SCENES[name], noise_mm=10, seed=2, ghosts=0.5, drop=0.2, t0=1000.0 + 100.0 * i
        )
        self.assertEqual(ws.judge(name, ws.SCENES[name], ticks, False), [])  # was no block


class CorridorWallsNotAGlassDoorTests(unittest.TestCase):
    """2026-11-19, walk_sim --follow with a noisy radar (ghosts 0.5, frames dropped 0.2, range
    noise): walking out of a 1 m dead end, "Stop, no clear way", then "Blocked by glass, turn
    around", back to the dead end. The left wall, half in the blind arc, was not found as a
    side wall; its returns read as facing ahead-left, one right-wall return at 29.9 deg as
    facing ahead-right, a wall was fitted through them, and the walls' own echo was a glass
    door (_door_in_wall_ft). Its own side-wall check now starts at 25 deg."""

    def test_walking_out_of_a_1_m_dead_end_with_a_noisy_radar(self):
        from tools.sim import walk_sim as ws

        name = "dead end, 1.0 m corridor"
        i = list(ws.ROOMS).index(name)  # as the command line runs it
        ticks = ws.follow(
            ws.ROOMS[name], noise_mm=10, seed=1, ghosts=0.5, drop=0.2, t0=1000.0 + 100.0 * i
        )
        self.assertEqual(ws.judge_follow(ws.ROOMS[name], ticks), [])
        said = {text for *_r, text, spoken, _c in ticks if spoken}
        self.assertFalse(said & set(sg.BLOCKED_GLASS_ANY + (sg.GLASS_AHEAD,)), said)

    def test_from_25_deg_in_the_door_rule_only(self):
        # In _laser_explains (_wall_too_far) 25 deg lost real panes in the scene hunt.
        side = {True: 500.0}
        a = 29.9
        d = 500.0 / math.sin(math.radians(a))
        self.assertFalse(walk_policy._on_side_wall(a, d, side))
        self.assertTrue(walk_policy._on_side_wall(a, d, side, min_deg=25.0))


class PeopleComingWalkedTests(unittest.TestCase):
    """2026-11-19, walk_sim --follow with people walking straight on, the trackers run as on the
    device: the LiDAR mover warnings (LIDAR_MOVERS_SPEAK, not yet spoken) get the wearer past
    someone coming head-on on open floor and someone crossing; nobody standing still is ever
    called moving, spoken or "would say". Evidence for the walk that is to turn them on."""

    def _walk(self, name, on=True):
        from tools.field.walk_replay import turned_on
        from tools.sim import walk_sim as ws

        shadows = []
        with turned_on(["movers"] if on else []):
            ticks = ws.follow(ws.ROOMS[name], shadows=shadows)
        return ws.judge_follow(ws.ROOMS[name], ticks, shadows), ticks

    def test_past_someone_coming_with_the_warnings_on(self):
        for name in (
            "someone coming head-on, open floor",
            "someone crossing from the left",
            "someone coming head-on, 2.4 m corridor",
        ):
            with self.subTest(name):
                self.assertEqual(self._walk(name)[0], [])

    def test_nobody_standing_is_called_moving(self):
        for on in (False, True):
            with self.subTest(on=on):
                self.assertEqual(self._walk("someone standing ahead", on)[0], [])


class CarsWalkedTests(unittest.TestCase):
    """2026-11-20, walk_sim --follow with cars (4.5 x 1.8 m boxes) seen by a synthetic front
    camera, the wearer doing what is said: the camera's spoken vehicle warning comes 2 s or more
    before a car on a collision course would reach them, head-on or from 20 deg off, and sends
    them behind it (the way it is going, not where it is); a car passing 3 m off, driving away,
    or parked is never called approaching; a person the camera names is steered past."""

    @staticmethod
    def _walk(name, t0=1000.0):
        from tools.sim import walk_sim as ws

        shadows = []
        ticks = ws.follow(ws.ROOMS[name], shadows=shadows, t0=t0)
        said = [text for *_r, text, sp, _c in ticks if sp]
        return ws.judge_follow(ws.ROOMS[name], ticks, shadows), said

    def test_warned_in_time_and_sent_behind_the_car(self):
        for name, way in (
            ("car coming head-on", None),
            ("car coming in from ahead-left", sg.VEHICLE_FROM_RIGHT),
            ("car coming in from ahead-right", sg.VEHICLE_FROM_LEFT),
        ):
            with self.subTest(name):
                bad, said = self._walk(name)
                self.assertEqual(bad, [])
                warned = [t for t in said if t in sg._VEHICLE_APPROACH]
                if way:  # coming from the left, going right: "move left", behind it
                    self.assertEqual(warned[0], way)

    def test_no_warning_for_a_car_not_coming(self):
        for name in (
            "car passing in the next lane",
            "car driving off ahead",
            "car parked in the way",
            "car parked beside the way",
        ):
            with self.subTest(name):
                bad, said = self._walk(name)
                self.assertEqual(bad, [])
                self.assertFalse(set(said) & set(sg._VEHICLE_APPROACH))

    def test_the_judge_tells_a_collision_course(self):
        from tools.sim import walk_sim as ws

        head_on = ws.Room(movers=[(30000, 0, -5000, 0, 900, "car")])
        self.assertAlmostEqual(
            ws._vehicle_lead(head_on, (0.0, 0.0, 0.0, 0.0)),
            (30000 - 2250 - ws.BODY_MM) / 6200.0,
            delta=0.06,
        )
        next_lane = ws.Room(movers=[(30000, 4000, -5000, 0, 900, "car")])
        self.assertIsNone(ws._vehicle_lead(next_lane, (0.0, 0.0, 0.0, 0.0)))
        self.assertAlmostEqual(
            ws._vehicle_passes(next_lane, (0.0, 0.0, 0.0, 0.0))[0][1], 3100.0, delta=1.0
        )

    def test_a_person_the_camera_names_is_steered_past(self):
        bad, said = self._walk("someone standing ahead, seen")
        self.assertEqual(bad, [])
        self.assertIn(sg.STEER_PAST[("Person", "ahead", "right")], said)


class CrossingCarsTests(unittest.TestCase):
    """2026-11-20, walk_sim: a car crossing toward the walking line at 14 km/h was warned of
    0.7 s before it would reach the wearer from the left, never from the right (the LiDAR's
    mover warnings, not yet spoken; the front camera does not see it until it is in front).
    Its front is what the LiDAR sees, past LIDAR_MOVERS_MAX_RANGE_MM until ~2 s out: blobs out
    to LIDAR_MOVERS_VEHICLE_MAX_RANGE_MM now reach the Kalman tracker, moving there only at a
    vehicle's speed. From the right its front came joined to its side seen edge-on, one run
    over 2 m: a too-wide run bent at a corner is two faces. The simulator's "nothing there",
    a return at 12 m, joined it into one ring round the wearer: a return at the LiDAR's reach
    is none. Alone on the road it was every return, dropped as a room's outline: only a run
    that closes round is. And the Kalman's "steady heading" held the filter's first guesses
    at its velocity 1.2 s: it is judged over the last MOTION_VEHICLE_STEADY_S of sightings.
    Now ~1.6 s ahead either way, the wearer stopped short of it. And with range noise, a car
    head-on that the wearer had stepped away from flipped the camera's spoken side ("move
    left" after "move right": back into it, 3 seeds in 6): the step is behind a car only when
    it goes across the line, its whole width in the time left."""

    @staticmethod
    def _seg(points):
        from app.perception.lidar_movers import segment

        return segment(points)

    @staticmethod
    def _arc(x, y, half_w, step=0.7):
        """A face across the line of sight at (x, y) m, half_w wide: (deg, mm) returns."""
        out = []
        for k in range(-40, 41):
            yy = y + half_w * k / 40.0
            out.append((math.degrees(math.atan2(yy, x)) % 360.0, math.hypot(x, yy) * 1000.0))
        return out

    def test_a_car_alone_on_the_road_is_a_blob(self):
        blobs = self._seg(self._arc(6.0, 1.0, 0.9))
        self.assertEqual(len(blobs), 1)
        self.assertTrue(blobs[0]["clean"])
        self.assertFalse(blobs[0]["far"])

    def test_the_reach_ring_is_no_surface(self):
        ring = [(k * 360.0 / 520, 12000.0) for k in range(520)]
        car = self._arc(9.0, 3.0, 0.9)
        lo, hi = min(a for a, _d in car), max(a for a, _d in car)
        blobs = self._seg(car + [p for p in ring if not lo - 1.0 <= p[0] <= hi + 1.0])
        self.assertEqual(len(blobs), 1)
        self.assertTrue(blobs[0]["far"])  # 9.5 m: past the people's range

    def test_a_front_and_its_side_edge_on_are_two_faces(self):
        # walk_sim's scan of the car crossing from the right, 2.5 s in: its front 10.2-10.8 m
        # off, then its side running off nearly edge-on to 11.7 m.
        car = [
            (63.7, 10780),
            (64.4, 10716),
            (65.1, 10655),
            (65.8, 10597),
            (66.5, 10540),
            (67.2, 10486),
            (67.8, 10433),
            (68.5, 10383),
            (69.2, 10335),
            (69.9, 10288),
            (70.6, 10244),
            (71.3, 10201),
            (72.0, 10160),
            (72.7, 10332),
            (73.4, 10750),
            (74.1, 11204),
            (74.8, 11701),
        ]
        blobs = self._seg(car)
        self.assertTrue(any(abs(b["width"] - 1.6) < 0.3 and b["far"] for b in blobs), blobs)

    def test_a_rooms_outline_is_still_dropped(self):
        room = [(k * 360.0 / 520, 3000.0) for k in range(520)]
        self.assertEqual(self._seg(room), [])

    def test_crossing_cars_warned_and_stopped_short(self):
        from tools.field.walk_replay import turned_on
        from tools.sim import walk_sim as ws

        for name in ("car crossing from the left", "car crossing from the right"):
            with self.subTest(name), turned_on(["movers"]):
                ticks = ws.follow(ws.ROOMS[name], t0=3000.0)
                warned = [
                    k
                    for k, (*_r, text, sp, _c) in enumerate(ticks)
                    if sp and text in sg._MOVER_INFO
                ]
                self.assertTrue(warned)
                self.assertGreaterEqual(ws._vehicle_lead(ws.ROOMS[name], ticks[warned[0]]), 1.2)
                self.assertGreaterEqual(ticks[-1][-1], ws.BODY_MM)  # not walked into

    def test_a_car_head_on_with_range_noise(self):
        from tools.sim import walk_sim as ws

        room = ws.ROOMS["car coming head-on"]
        for seed in (2, 3):
            with self.subTest(seed=seed):
                ticks = ws.follow(room, noise_mm=10.0, seed=seed, t0=2600.0)
                self.assertEqual(ws.judge_follow(room, ticks), [])


class GroundHazardsWalkedTests(unittest.TestCase):
    """2026-11-20, walk_sim --follow with the ToF's hazards on the ground (TOF_HAZARDS_SPEAK, not
    yet spoken), rendered from the walking wearer with the body's sway and the front camera's
    picture: (1) "Drop in front, stop" came in time, then "Walk straight" a metre from the edge,
    and the wearer walked off it: nearer than ~1.1 m the ToF sees only the lower ground (or
    past 4 m nothing) and finds no floor; a drop or stairs down said "..., stop" is now held
    while no ground is found. (2) A platform edge was "Drop ahead" at 2.2 m, then silent: the
    camera's confirmation was undone by "no evidence" as the edge slid off the bottom of its
    picture. (3) A kerb at the ToF's far end was "Low obstacle ahead", and its airtime held
    "Step up ahead" to 1.4 m before it; stairs up waited the settle after "Step up ahead"."""

    @staticmethod
    def _frames(boxes, poses, hz=None):
        import numpy as np

        from app.perception import depth_fusion as df
        from app.perception import ground_hazards as gh
        from tools.sim import ground_hazard_sim as ghs

        hz = hz or gh.GroundHazards(df.DepthProjector(ghs.CAL))
        rng, snaps = np.random.default_rng(0), []
        for i, pose in enumerate(poses):
            depth, conf = ghs.render_tof(boxes, rng=rng, pose=pose)
            snaps.append(hz.step(depth, conf, 1000.0 + i * 0.1, None))
        return hz, snaps

    def test_a_drop_said_stop_is_held_until_the_floor_is_seen_again(self):
        from tools.sim import ground_hazard_sim as ghs

        boxes = ghs.drop(z0=4.0, depth=1.0)
        walk = [(0.0, 0.12 * i, 0.0) for i in range(25)]  # up to 1.1 m short of the edge
        hz, snaps = self._frames(boxes, walk + [(0.0, 2.9, 0.0)] * 10)
        self.assertFalse(snaps[-1]["plane"])  # no floor in view: only the ground below
        self.assertEqual((snaps[-1]["top"] or {}).get("kind"), "drop")
        self.assertTrue(snaps[-1]["held"])
        _hz, turned = self._frames(boxes, [(0.0, 2.9, 90.0)] * 6, hz)  # turned away
        self.assertIsNone(turned[-1]["top"])
        self.assertFalse(turned[-1]["held"])

    def test_the_cameras_confirmation_kept_walking_up_to_it(self):
        from app.perception.ground_hazards import _Tracker

        def status(confirmed_at, then_at):
            """The camera's word on the drop after `confirmed_at` (confirmed) and `then_at`
            ("no evidence", five frames: no confirmation left in the tracker's window)."""
            tr, i, out = _Tracker(), 0, None
            for ahead, cam in [(a, "confirmed") for a in confirmed_at] + [
                (a, "no_evidence") for a in then_at
            ]:
                hz = {"kind": "drop", "ahead_m": ahead, "in_path": True, "camera": {"status": cam}}
                out, i = tr.update([hz], 1000.0 + 0.1 * i), i + 1
            return out[0]["camera"]["status"]

        # Walking up to it: off the bottom of the picture, "no evidence" is no evidence.
        self.assertEqual(status((2.45, 2.35, 2.25), (2.15, 2.05, 1.95, 1.85, 1.75)), "confirmed")
        # Farther off than where it was confirmed: the camera's word stands.
        self.assertEqual(status((2.0, 2.0, 2.0), (2.1, 2.2, 2.3, 2.4, 2.5)), "no_evidence")

    def test_a_kerb_at_the_far_end_is_not_a_low_obstacle(self):
        from tools.sim import ground_hazard_sim as ghs

        _hz, snaps = self._frames(ghs.kerb_up(z0=3.6), [(0.0, 0.0, 0.0)] * 5)
        self.assertNotIn("low_obstacle", [h["kind"] for s in snaps for h in s["seen"]])
        _hz, snaps = self._frames(ghs.low_box(z=2.0), [(0.0, 0.0, 0.0)] * 5)
        self.assertIn("low_obstacle", [h["kind"] for h in snaps[-1]["seen"]])

    def test_walked(self):
        from tools.field.walk_replay import turned_on
        from tools.sim import walk_sim as ws

        for name in ("drop 1 m deep, 6 m out", "platform edge, 6 m out", "kerb up, 6 m out"):
            with self.subTest(name), turned_on(["tof-hazards"]):
                room = ws.ROOMS[name]
                ticks = ws.follow(room, t0=4000.0)
                self.assertEqual(ws.judge_follow(room, ticks), [])


class RadarEchoTheLaserSeesTests(unittest.TestCase):
    """2026-11-20 (walk_sim): a board's nearest point on a surface facing the walk lies at its
    beam's edge and slides along it, closing faster than the walk: walking up to the back of
    a parked car, and to a pane across open floor, "Something approaching from right, stop"
    (a "would say"; at the pane the wearer, told to stop and not to turn, walked into it). A
    still surface may close at walk / cos(the beam edge's angle off the way) on a board
    (radar_movers.edge_slide), and an echo at a range the laser has a return at inside the
    board's beam is the LiDAR's (laser_sees): the car's side, over 2 m wide, is no blob."""

    WALKING = {"valid": True, "fwd_mps": 1.2, "right_mps": 0.0, "rms_mps": 0.05}

    @staticmethod
    def _threat(board, closing_mps, laser=False, ego=None, start_mm=3900.0):
        from app.perception.radar_movers import RadarMoverTracker

        tracker, threat = RadarMoverTracker(), None
        bearing = float(board[3:])
        for i in range(10):  # 6 frames a second, while the echo is inside the boards' reach
            t, r_mm = 100.0 + i / 6.0, start_mm - closing_mps * 1000.0 * i / 6.0
            if r_mm < 300.0:
                break
            radar = {
                "sectors": {
                    board: {
                        "online": True,
                        "degraded": False,
                        "stale": False,
                        "last_update": t,
                        "peaks_mm": [r_mm],
                    }
                }
            }
            lidar = (
                {"points": {"angles": [0.0, bearing, 180.0], "distances": [9000.0, r_mm, 9000.0]}}
                if laser
                else None
            )
            tracker.update(radar, [], ego, turn_dps=0.0, lidar_state=lidar)
            threat = threat or tracker.snapshot()["threat"]
        return threat

    def test_an_echo_off_a_surface_the_laser_sees_is_not_something_coming(self):
        self.assertIsNotNone(self._threat("deg030", 2.3))  # faster than any walk: coming
        self.assertIsNone(self._threat("deg030", 2.3, laser=True))

    def test_a_side_board_allows_the_beam_edge_slide(self):
        from app.perception import radar_movers

        self.assertAlmostEqual(radar_movers.edge_slide(90.0), 1.0 / math.cos(math.radians(57.5)))
        self.assertEqual(radar_movers.edge_slide(30.0), 1.0)
        self.assertEqual(radar_movers.edge_slide(210.0), 1.0)
        slide = 1.2 / math.cos(math.radians(57.5))  # walking 1.2 m/s at a facing surface
        self.assertIsNone(self._threat("deg090", slide, ego=self.WALKING))
        self.assertIsNotNone(self._threat("deg090", 3.6, ego=self.WALKING))  # really coming
        with unittest.mock.patch.object(radar_movers, "edge_slide", lambda b: 1.0):
            self.assertIsNotNone(self._threat("deg090", slide, ego=self.WALKING))  # what it was

    def test_walked_past_a_parked_car_and_up_to_a_pane(self):
        from tools.field.walk_replay import turned_on

        bad, _said = CarsWalkedTests._walk("car parked beside the way", t0=3500.0)
        self.assertEqual(bad, [])  # was "... standing still called moving: 'Something ..."
        with turned_on(["movers"]):
            bad, _said = CarsWalkedTests._walk("pane across open floor", t0=1800.0)
        self.assertEqual(bad, [])  # was "Something approaching from right", walked into it

    def test_a_return_elsewhere_does_not_hide_it(self):
        from app.perception.radar_movers import laser_sees

        lidar = {"points": {"angles": [58.0, 180.0, None], "distances": [3000.0, 1500.0, 1.0]}}
        self.assertTrue(laser_sees(lidar, 90.0, 3.0))
        self.assertFalse(laser_sees(lidar, 90.0, 1.5))  # behind: another board's
        self.assertFalse(laser_sees(lidar, 90.0, 2.0))  # nearer than the laser's surface
        self.assertFalse(laser_sees(None, 90.0, 3.0))


class CorridorMiddleNoPingPongTests(unittest.TestCase):
    """2026-11-19, walk_sim --follow --veer: down the middle of a 2.4 m corridor both walls lie
    at ~3.9 ft, inside SIMPLE_WALL_NEAR_FT; a few cm off made one "near", "Near left wall, move
    right", and the step past the middle the other: "move right / move left" every 2 s, and
    the drift (heading) sentence never said. Both walls in the band: "move" only with the near
    one SIMPLE_WALL_NEAR_OFFSET_FT nearer, or nearer than SIMPLE_WALL_NEAR_ANYWAY_FT. And near
    a wall while still heading into it: the turn away, not the step."""

    @staticmethod
    def _say(**scene):
        from tools.sim import scene_hunt as sh

        sg.reset()
        room = {"left": None, "right": None, "ahead": None, "glass": None, "posts": []}
        return sh.decide({**room, **scene})[1]["instruction"]

    def test_about_the_middle_of_a_corridor(self):
        self.assertEqual(self._say(left=1222, right=1180), sg.WALK_STRAIGHT)  # seed 12 #1497

    def test_a_step_off_the_middle_is_still_said(self):
        self.assertEqual(self._say(left=600, right=1800), sg.NEAR_LEFT)

    def test_a_wall_this_close_is_said_whatever_the_other_side(self):
        # seed 13 #741: the right wall 0.40 m off, posts at the left
        said = self._say(right=397, posts=[(704, -886, 234), (2400, -592, 76)])
        self.assertEqual(said, sg.NEAR_RIGHT)

    def test_near_and_heading_into_it_is_the_turn(self):
        self.assertEqual(
            sg._sentence(False, False, False, False, None, drift="left", close="left"),
            sg.DRIFT_LEFT,
        )
        self.assertEqual(sg._sentence(False, False, False, False, None, close="left"), sg.NEAR_LEFT)

    def test_near_then_heading_into_it_is_said_at_once(self):
        # Near the right wall, said; seconds later heading into it: the turn away is news,
        # not a change to settle 2 s (it was; "Wall ahead" came first, then a block).
        gate = sg._Gate()
        self.assertTrue(gate.admit(sg.NEAR_RIGHT, 100.0, episode=1)[0])
        self.assertTrue(gate.admit(sg.DRIFT_RIGHT, 105.0, episode=1)[0])

    def test_still_drifting_in_is_said_again(self):
        # 2026-11-20: an advisory is said once an episode; a drift into the same wall after
        # the turn was not, and "Wall ahead" then a block followed. The drifting sentence is
        # said again once it has stood SIMPLE_DRIFT_REPEAT_S, the voice free; nothing else is.
        from app import config as cfg

        gate = sg._Gate()
        self.assertTrue(gate.admit(sg.DRIFT_RIGHT, 100.0, episode=1)[0])
        said = [
            t
            for t in (100.0 + 0.1 * k for k in range(1, 80))
            if gate.admit(sg.DRIFT_RIGHT, t, episode=1)[0]
        ]
        self.assertTrue(said)
        self.assertGreaterEqual(said[0], 100.0 + cfg.SIMPLE_DRIFT_REPEAT_S - 1e-6)
        gate = sg._Gate()
        self.assertTrue(gate.admit(sg.NEAR_RIGHT, 100.0, episode=1)[0])
        self.assertFalse(
            any(gate.admit(sg.NEAR_RIGHT, 100.0 + 0.1 * k, episode=1)[0] for k in range(1, 80))
        )

    def test_veering_4_deg_s_down_a_3_m_corridor(self):
        from tools.sim import walk_sim as ws

        room = ws.ROOMS["long 3 m corridor"]
        self.assertEqual(ws.judge_follow(room, ws.follow(room, veer_dps=4.0)), [])

    def test_veering_down_a_3_m_corridor(self):
        from tools.sim import walk_sim as ws

        room = ws.ROOMS["long 3 m corridor"]
        for veer in (3.0, -3.0):
            with self.subTest(veer=veer):
                ticks = ws.follow(room, veer_dps=veer)
                self.assertEqual(ws.judge_follow(room, ticks), [])
                said = [text for *_r, text, spoken, _c in ticks if spoken]
                self.assertFalse(set(said) & set(sg.BLOCKED_WALL_ANY), said)

    def test_veering_down_a_2_4_m_corridor(self):
        from tools.sim import walk_sim as ws

        room = ws.ROOMS["long 2.4 m corridor"]
        ticks = ws.follow(room, veer_dps=-3.0)
        self.assertEqual(ws.judge_follow(room, ticks), [])
        said = [text for *_r, text, spoken, _c in ticks if spoken]
        flips = sum(1 for a, b in zip(said, said[1:]) if {a, b} == {sg.NEAR_LEFT, sg.NEAR_RIGHT})
        self.assertEqual(flips, 0, said)


class SensorFaultsWalkedTests(unittest.TestCase):
    """2026-11-19, walk_sim --faults: a sensor failing on the way in. No "Walk straight" off a
    LiDAR that stopped; a 2 s stall told ("Guidance stopped, use your cane"), the front
    radar's loss told ("Glass sensor off"); back inside the block, the block said at once; a
    stall while turning around at a dead end leaves no stale way out after it. Each rule was
    seen to fail with its notice held back."""

    def test_the_fault_scenes(self):
        from tools.sim import walk_sim as ws

        for i, (name, (scene, faults)) in enumerate(ws.FAULT_SCENES.items()):
            with self.subTest(name):
                ticks = ws.walk(scene, faults=faults, t0=1000.0 + 100.0 * i)
                self.assertEqual(ws.judge_faults(scene, faults, ticks), [])

    def test_a_stall_while_turning_around(self):
        from tools.sim import walk_sim as ws

        room = ws.ROOMS["dead end, 1.8 m corridor"]
        ticks = ws.follow(room, faults=[("lidar", 3.4, 5.0)])
        self.assertEqual(ws.judge_follow(room, ticks), [])
        said = [text for *_r, text, spoken, _c in ticks if spoken]
        self.assertFalse({sg.BLOCKED_WALL_TURN["left"], sg.BLOCKED_WALL_TURN["right"]} & set(said))


class ToldToStopTests(unittest.TestCase):
    """2026-11-19: told to step aside in a bare corridor, the wearer was still "walking" to the
    Kalman tracker (its own motion unmeasured: the assumed pace), someone coming at 1.2 m/s
    looked still, the warning ended, and "Near left wall, move right" sent them back into that
    person's way. The guidance now tells the tracker (told_still), which takes the wearer's
    pace as 0 where nothing measures it, until "Walk straight" or MOTION_TOLD_STILL_S."""

    def test_the_tracker_takes_the_wearer_as_standing(self):
        from app.perception.motion_tracker import MotionTracker
        from app import config as cfg

        walking = {"moving": True, "speed_mps": 1.2}  # nothing measured: the nominal pace
        step = MotionTracker._own_step
        self.assertEqual(step(walking, None, 0.1)[4], "assumed")
        self.assertEqual(step(walking, None, 0.1, told_still=True)[:2], (0.0, 0.0))
        self.assertEqual(step(walking, None, 0.1, told_still=True)[4], "told to stop")
        measured = {"scan_fwd_mm": 120.0, "scan_right_mm": 0.0}  # odometry rules still
        self.assertEqual(step(measured, None, 0.1, told_still=True)[4], "odometry")
        tr = MotionTracker()
        tr.update([], 100.0, walking)  # the first scan sets the clock: no step yet
        tr.told_still(100.0)
        tr.update([], 100.5, walking)
        self.assertEqual(tr.snapshot()["own"]["source"], "told to stop")
        tr.update([], 100.5 + 0.3, walking)
        tr.told_still(100.9, still=False)  # "Walk straight"
        tr.update([], 101.0, walking)
        self.assertEqual(tr.snapshot()["own"]["source"], "assumed")
        tr.told_still(102.0)
        late = 102.0 + cfg.MOTION_TOLD_STILL_S + 0.1  # updates every 0.5 s up to then
        t = 101.0
        while t < late:
            t = min(late, t + 0.5)
            tr.update([], t, walking)
        self.assertEqual(tr.snapshot()["own"]["source"], "assumed")

    def test_said_to_the_tracker_only_for_a_stop_or_a_step(self):
        from unittest import mock

        told = []
        fake = type("T", (), {"told_still": lambda self, t, still=True: told.append(still)})()
        with mock.patch("app.perception.motion_tracker.get_motion_tracker", lambda: fake):
            sg._tell_tracker(sg.MOVER_TOWARD[("Person", "left")], 1.0)
            sg._tell_tracker(sg.STOP_NO_WAY, 2.0)
            sg._tell_tracker(sg.BLOCKED_WALL_TURN["left"], 3.0)  # a turn: walks on after
            sg._tell_tracker(sg.WALK_STRAIGHT, 4.0)
        self.assertEqual(told, [True, True, False])


class NewWayOutOffAHoldTests(unittest.TestCase):
    """2026-11-18, tools/sim/walk_sim --follow (the wearer doing what is said): at a dead end,
    "Blocked by wall, turn around"; turning, the wearer faced the side wall and the way out
    became "turn right", said once the turn was done, facing the open corridor, with only the
    block's release hold keeping it. They turned into the wall, and back to the dead end. A
    block's new way out is now said only while this poll still measures the block, as its
    repeat already was (2026-11-09)."""

    def test_not_said_off_a_releasing_hold(self):
        gate = sg._Gate()
        around, right = sg.BLOCKED_WALL_TURN["around"], sg.BLOCKED_WALL_TURN["right"]
        self.assertTrue(gate.admit(around, 100.0, episode=1)[0])
        for k in range(1, 41):  # 4 s, past SIMPLE_SETTLE_S
            said, _seq, why = gate.admit(right, 100.0 + 0.1 * k, episode=1, evidence=False)
            self.assertFalse(said, why)
        self.assertIn("no longer measured", why)

    def test_a_closed_side_is_still_said_at_once(self):
        # Told "turn right", someone stepped into the right: its correction is news whatever
        # holds the block (2026-11-09), and the guard above does not hold it (review).
        gate = sg._Gate()
        right, left = sg.BLOCKED_WALL_TURN["right"], sg.BLOCKED_WALL_TURN["left"]
        self.assertTrue(gate.admit(right, 100.0, episode=1)[0])
        said = [
            gate.admit(left, 100.0 + 0.1 * k, episode=1, closed=True, evidence=False)
            for k in range(1, 11)
        ]
        self.assertTrue(any(ok for ok, _seq, _why in said), said)

    def test_a_renamed_block_is_said_while_measured_only(self):
        # One rule (2026-11-19): any block, renamed or not, is said only while measured.
        wall, glass = sg.BLOCKED_WALL_TURN["right"], sg.BLOCKED_GLASS_TURN["right"]
        gate = sg._Gate()
        self.assertTrue(gate.admit(wall, 100.0, episode=1)[0])
        said = [
            gate.admit(glass, 100.0 + 0.1 * k, episode=1, evidence=False)[0] for k in range(1, 41)
        ]
        self.assertFalse(any(said))
        gate = sg._Gate()
        self.assertTrue(gate.admit(wall, 100.0, episode=1)[0])
        said = [gate.admit(glass, 100.0 + 0.1 * k, episode=1) for k in range(1, 41)]
        self.assertTrue(any(ok for ok, _seq, _why in said))

    def test_a_block_after_a_mover_warning_only_while_measured(self):
        # 2026-11-19, walk_sim: someone passing in a corridor, "Stop, no clear way" held from a
        # second before was said once "Person approaching from right, stop" ended, the way
        # clear again (the news path after an approach warning skipped the old guards).
        gate = sg._Gate()
        mover = sg.MOVER_CROSSING[("Person", "right")]
        self.assertTrue(gate.admit(mover, 100.0, episode=1)[0])
        said = [
            gate.admit(sg.STOP_NO_WAY, 100.0 + 0.1 * k, episode=1, evidence=False)[0]
            for k in range(1, 60)
        ]
        self.assertFalse(any(said))

    def test_said_while_measured(self):
        gate = sg._Gate()
        around, right = sg.BLOCKED_WALL_TURN["around"], sg.BLOCKED_WALL_TURN["right"]
        self.assertTrue(gate.admit(around, 100.0, episode=1)[0])
        said = [gate.admit(right, 100.0 + 0.1 * k, episode=1)[0] for k in range(1, 41)]
        self.assertTrue(any(said))  # once settled

    def test_the_dead_end_walked_out_of(self):
        from tools.sim import walk_sim as ws

        room = ws.ROOMS["dead end, 1.8 m corridor"]
        for wait in (None, 3.0):
            with self.subTest(wait=wait):
                ticks = ws.follow(room, wait_s=wait)
                self.assertEqual(ws.judge_follow(room, ticks), [])
                said = [text for *_r, text, spoken, _c in ticks if spoken]
                self.assertNotIn(sg.BLOCKED_WALL_TURN["right"], said)
                self.assertNotIn(sg.BLOCKED_WALL_TURN["left"], said)


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
            sh.publish(
                {
                    "left": None,
                    "right": None,
                    "ahead": None,
                    "posts": [(1000, 0, 250)] if post else [],
                    "glass": None,
                },
                True,
            )
            lidar, radar = get_lidar_state(), get_radar_state()
            got = sg.guidance(
                walk_policy.decide(lidar, radar),
                lidar,
                now=now,
                system={"readable": True, "power_low": i > 5},
            )
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


class LiveCheckTests(unittest.TestCase):
    """tools/device/live_check.py (2026-11-19): each radar board's own frames from /radar/state
    polls, and the numbers the simulators assume (missed frames, ghosts, jitter)."""

    def test_the_numbers(self):
        from tools.device import live_check as lc

        def snap(t, peaks):
            return {
                "sectors": {
                    "deg030": {"last_update": t, "online": True, "peaks_mm": peaks},
                    "front": {"last_update": t, "peaks_mm": peaks},
                }
            }

        samples = [
            snap(1.0, [1000.0]),
            snap(1.0, [1000.0]),
            snap(1.17, []),
            snap(1.33, [1010.0, 2005.0]),
            snap(1.5, [1004.0]),
            snap(1.67, [998.0]),
        ]
        frames = lc.board_frames(samples)
        self.assertEqual(list(frames), ["deg030"])  # boards only, not the cardinals
        st = lc.board_stats(frames["deg030"])
        self.assertEqual(st["frames"], 5)  # the same frame polled twice counts once
        self.assertAlmostEqual(st["missed"], 1 / 3, places=3)  # silent between two that heard
        self.assertAlmostEqual(st["ghosts"], 1 / 5, places=3)  # 2005 is twice 1010, within 150
        self.assertEqual(st["jitter_mm"], 6.0)


class WalkReportTests(unittest.TestCase):
    """2026-11-20, for the first walks on the device: walk_sim writes its walks as the walk
    recorder does (Recording), walk_replay reproduces them, and tools/field/walk_report.py sums
    one up. A second replay of the same walk in one process saw no pane closing (the radar
    frames of the first were left): replay_ticks now clears them."""

    def _record(self, room_name, path, **kw):
        from tools.sim import walk_sim as ws

        rec = ws.Recording(path)
        ticks = ws.follow(ws.ROOMS[room_name], record=rec, **kw)
        rec.close()
        return ticks

    def test_a_simulated_walk_replays_exactly(self):
        import tempfile
        from tools.field.walk_replay import replay_ticks

        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "walk.jsonl.gz")
            self._record("dead end, 1.8 m corridor", path)
            same = [
                rec["out"]["instruction"] == got["instruction"]
                and bool(rec["out"].get("speak")) == bool(got.get("speak"))
                for rec, got in replay_ticks(path)
                if got is not None
            ]
        self.assertTrue(same)
        self.assertTrue(all(same))

    def test_a_feature_turned_on_after_a_replay(self):
        import tempfile
        from tools.field import walk_report as wr

        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "walk.jsonl.gz")
            self._record("glass door closing a 1.8 m corridor", path)
            lines = wr.replay_lines(path, ["glass-closing"], 1000.0)
        self.assertIn("100.0% of ticks said the same", lines[0])
        self.assertNotIn(" 0 announcement", lines[1])  # was 0: no pane closing seen

    def test_the_checks(self):
        from tools.field import walk_report as wr

        rows = [
            (0.0, sg.WALK_STRAIGHT, 3.0, True, "clear"),  # something 3 ft ahead, measured
            (1.0, sg.BLOCKED_WALL_TURN["left"], 3.5, True, "blocked"),
            (3.0, sg.BLOCKED_WALL_TURN["right"], 3.5, True, "blocked"),  # flipped in 2 s
            (10.0, sg.NEAR_LEFT, None, False, "near_wall"),
            (12.0, sg.NEAR_RIGHT, None, False, "near_wall"),  # ping-pong
            (13.0, sg.NEAR_RIGHT, None, False, "near_wall"),
        ]  # again in 1 s
        hits = [len(h) for _rule, h in wr.check_lines(rows, 0.0)]
        self.assertEqual(hits, [1, 1, 1, 1])


if __name__ == "__main__":
    unittest.main()
