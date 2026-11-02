"""Decision-logic regressions found by the mirror test and fuzzing (2026-10-30). Hardware-free;
run with venv/bin/python -m unittest tools.test_logic_regressions."""

import math
import time
import unittest

from utils import drift_guidance, walk_policy


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
        self.assertEqual(sh.decide(post)[1]["instruction"], "Turn left")
        self.assertEqual(sh.decide(sh.mirror(post))[1]["instruction"], "Turn right")


class GlassBehindNearerEchoTests(unittest.TestCase):
    """A pane the radar hears behind a nearer echo the laser explains (2026-11-02)."""

    def said(self, scene, ghosts=0.0):
        import random

        from tools import scene_hunt as sh

        return sh.decide(scene, ghosts=ghosts, rnd=random.Random(0))[1]["instruction"]

    def test_pane_at_a_side_posts_range(self):
        scene = {"left": None, "right": 2667.0, "ahead": None, "glass": 836.0,
                 "posts": [(3417.0, -135.0, 258.0), (3597.0, 1413.0, 196.0), (528.0, -776.0, 91.0)]}
        self.assertEqual(self.said(scene), "Blocked by glass")
        self.assertEqual(self.said({**scene, "glass": None}), "Walk straight")

    def test_pane_behind_a_side_walls_echo(self):
        scene = {"left": 1227.0, "right": None, "ahead": None, "glass": 1084.0, "posts": []}
        self.assertEqual(self.said(scene), "Blocked by glass")

    def test_double_bounce_ghosts_are_not_glass(self):
        corridor = {"left": 757.0, "right": 1067.0, "ahead": 3951.0, "glass": None, "posts": []}
        self.assertNotIn(self.said(corridor, ghosts=1.0), ("Glass ahead", "Blocked by glass"))


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


if __name__ == "__main__":
    unittest.main()
