"""Decision-logic regressions found by the mirror test and fuzzing (2026-10-30). Hardware-free;
run with venv/bin/python -m unittest tools.test_logic_regressions."""

import math
import os
import time
import unittest
import unittest.mock

from utils import drift_guidance, walk_policy
from utils import simple_guidance as sg


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
        from tools import scene_hunt as sh

        post = {"left": None, "right": None, "ahead": None, "glass": None, "posts": [(800.0, 300.0, 130.0)]}
        # Named since 2026-11-09: a post in front is not "Turn left" alone.
        self.assertEqual(sh.decide(post)[1]["instruction"], "Obstacle in front, turn left")
        self.assertEqual(sh.decide(sh.mirror(post))[1]["instruction"], "Obstacle in front, turn right")


class GlassBehindNearerEchoTests(unittest.TestCase):
    """A pane the radar hears behind a nearer echo the laser explains (2026-11-02)."""

    def said(self, scene, ghosts=0.0):
        import random

        from tools import scene_hunt as sh

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
        from tools import scene_hunt as sh

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
        from utils.shared_state import reset_all_states

        reset_all_states()

    def test_strengths_stay_with_their_echoes(self):
        from utils import radar_utils as ru
        from utils.shared_state import get_radar_state

        ru._publish("deg330", {"min_dist": 900.0, "peaks_mm": [900.0, 2400.0],
                               "peaks_db": [18.5, 31.0], "online": True, "degraded": False})
        ru._publish("deg030", {"min_dist": 1200.0, "peaks_mm": [1200.0, 3100.0],
                               "peaks_db": [12.0, 9.0], "online": True, "degraded": False})
        front = get_radar_state()["sectors"]["front"]
        self.assertEqual(list(zip(front["peaks_mm"], front["peaks_db"])),
                         [(900.0, 18.5), (1200.0, 12.0), (2400.0, 31.0), (3100.0, 9.0)])

    def test_a_row_without_strengths_does_not_shift_them(self):
        from utils import radar_utils as ru
        from utils.shared_state import get_radar_state

        ru._publish("deg330", {"min_dist": 900.0, "peaks_mm": [900.0], "online": True, "degraded": False})
        ru._publish("deg030", {"min_dist": 1200.0, "peaks_mm": [1200.0], "peaks_db": [12.0],
                               "online": True, "degraded": False})
        front = get_radar_state()["sectors"]["front"]
        self.assertEqual(list(zip(front["peaks_mm"], front["peaks_db"])), [(900.0, None), (1200.0, 12.0)])


class AfternoonFixTests(unittest.TestCase):
    """2026-11-09 afternoon: the corridor glass door, a person behind a glass door, the
    camera's way past a vehicle, and a named block's way out."""

    def _front(self, segs, echoes, peaks=None):
        from tools import glass_sim as gs

        walk_policy._near_frames.clear()
        radar = gs.radar_state(echoes)
        for k, p in (peaks or {}).items():
            radar["sectors"][k]["peaks_mm"] = p
        return walk_policy.read_direction(gs.lidar_state(segs), radar, "front")

    def test_corridor_glass_door_nearer_than_its_walls_echo(self):
        from tools import glass_sim as gs

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
        from tools import glass_sim as gs

        for lat in (450.0, 750.0, 1000.0, 1500.0):
            segs = gs.side_wall("left", lat) + gs.side_wall("right", lat) + gs.back_wall()
            e = lat * gs.BEAM_EDGE
            f = self._front(segs, {"deg030": e, "deg330": e})
            self.assertFalse(f["glass"], lat)
            self.assertFalse(f["distance_ft"] < 4.0, lat)

    def test_person_just_behind_a_glass_door_blocks_at_the_glass(self):
        from tools import glass_sim as gs

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
        from tools import glass_sim as gs

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
        from tools import glass_sim as gs

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
        from tools import glass_sim as gs

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
        from tools import glass_sim as gs

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
        from tools import glass_sim as gs
        from utils import ground_hazards as gh

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
        import random
        from tools import scene_hunt as sh
        from utils.shared_state import get_lidar_state

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
        from utils import walk_recorder as wr

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


if __name__ == "__main__":
    unittest.main()
