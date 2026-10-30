"""Do the components agree with each other? Hardware-free; run with
venv/bin/python -m unittest tools.test_component_sync.

1. One post at a known bearing: the LiDAR sectors, the radar, the hazard list (clock), the
   BEV grid and the camera bearing must all put it in the same place.
2. A turn, a walk forward and a step sideways: the heading tracker's signs must follow the
   gyro's convention (a right turn is positive yaw).
3. Settings that only work together: distance bands in order, every freshness window longer
   than its sensor's period, the fast confirms faster than the slow, the phone's limits the
   server's, the hang notice inside the heartbeat inside the watchdog."""

import math
import re
import time
import unittest
from unittest.mock import patch

import config as c
from utils import lidar_utils as lu, radar_utils as ru, sensor_fusion as sf
from utils import shared_state as ss, simple_guidance as sg
from utils import bev_grid, heading_tracker as ht

POST_MM, ROOM_MM, POST_R = 1500.0, 6000.0, 120.0


def _post_scan(bearing):
    """A round room of ROOM_MM and a post POST_MM away at `bearing` (raw = corrected angles)."""
    out = []
    for k in range(720):
        a = k * 0.5
        if lu._in_blind_arc(a):
            continue
        gap = math.radians((a - bearing + 180.0) % 360.0 - 180.0)
        along, off = POST_MM * math.cos(gap), POST_MM * math.sin(gap)
        d = ROOM_MM
        if abs(off) < POST_R and along > 0:
            d = along - math.sqrt(POST_R**2 - off**2)
        out.append((15, a, d))
    return out


class DirectionAgreementTests(unittest.TestCase):
    def setUp(self):
        self.tracker = ht._tracker
        ht._tracker = ht.HeadingTracker()
        bev_grid.reset_grid()
        lu._previous_scan = None
        self.quiet = patch("builtins.print")
        self.quiet.start()

    def tearDown(self):
        self.quiet.stop()
        ht._tracker = self.tracker
        bev_grid.reset_grid()
        ss.reset_all_states()

    def place_post(self, bearing):
        ht._tracker = ht.HeadingTracker()  # each bearing on its own, with no memory of the last
        bev_grid.reset_grid()
        lu._previous_scan = None
        for board in c.RADAR_BEARINGS.values():  # the radar first, as on the device
            gap = abs((board - bearing + 180.0) % 360.0 - 180.0)
            echo = POST_MM + 20.0 if gap <= 30.0 else None
            ru._publish(c.radar_board_key(board), {
                "min_dist": echo, "peaks_mm": [echo] if echo else [], "online": True,
                "degraded": False, "lag_s": 0.0, "phase": "streaming", "error": None})
        t = time.time()
        for i in range(4):
            scan = _post_scan(bearing)
            lu._publish_scan(None, scan, 500 + i, 15.0, {}, t + i / 15.0, [p[1] for p in scan])
            time.sleep(1.0 / 15.0)  # at the scan rate: the BEV grid extracts at BEV_EXTRACT_HZ

    def test_every_component_places_the_post_alike(self):
        expect_radar = {0.0: "front", 45.0: "front", 90.0: "right", 135.0: "back", 315.0: "front"}
        for bearing, cardinal in expect_radar.items():
            with self.subTest(bearing=bearing):
                self.place_post(bearing)
                lidar, radar = ss.get_lidar_state(), ss.get_radar_state()
                near = [s for s, v in lidar["sectors"].items()
                        if isinstance(v.get("min_dist"), (int, float)) and abs(v["min_dist"] - POST_MM) < 150]
                self.assertIn(lu.get_sector(bearing), near, "LiDAR sector")
                heard = [s for s in c.RADAR_SECTORS
                         if isinstance(radar["sectors"].get(s, {}).get("min_dist"), (int, float))]
                self.assertIn(cardinal, heard, "radar side")
                hazards = [h for h in sf.fuse_guidance().get("hazards") or []
                           if abs((h.get("distance_ft") or 0) - POST_MM / 304.8) < 0.8]
                self.assertTrue(hazards, "hazard list")
                self.assertEqual(hazards[0].get("clock"), sf.angle_to_clock(bearing), "hazard clock")
                objs = bev_grid.get_grid().snapshot().get("objects") or []
                laser = [math.degrees(math.atan2(o["centroid_mm"][1], o["centroid_mm"][0])) % 360
                         for o in objs if o["kind"] != "obstacle"
                         and abs(math.hypot(*o["centroid_mm"]) - POST_MM) < 400]
                self.assertTrue(any(abs((b - bearing + 180) % 360 - 180) < 5 for b in laser), f"BEV {laser}")

    def test_camera_bearing_matches_the_voice(self):
        qw = sg._QUAD_W
        focal = (qw / 2.0) / math.tan(math.radians(c.SIMPLE_CAMERA_HFOV_DEG) / 2.0)
        for bearing in (-20.0, 0.0, 20.0):
            cx = qw / 2.0 + focal * math.tan(math.radians(bearing))
            lo, hi = sf._box_bearings("front", cx - 40, cx + 40)
            mid = (lo + hi) / 2.0
            with self.subTest(bearing=bearing):
                self.assertAlmostEqual((mid - bearing + 180) % 360 - 180, 0.0, delta=1.5)


class MotionSignTests(unittest.TestCase):
    """The scan matcher's signs are the gyro's: right turn +yaw, forward +speed, right +."""

    WALLS = [((-2000, -1200), (4000, -1200)), ((4000, -1200), (4000, 1800)),
             ((4000, 1800), (-2000, 1800)), ((-2000, 1800), (-2000, -1200))]
    POSTS = [(2500, 900, 100), (1200, -700, 80)]

    def cast(self, px, py, yaw):
        pts = []
        for k in range(260):
            b = k * 360 / 260
            a = math.radians(b + yaw)
            dx, dy = math.cos(a), math.sin(a)
            best = 1e9
            for (x1, y1), (x2, y2) in self.WALLS:
                ex, ey = x2 - x1, y2 - y1
                den = dx * ey - dy * ex
                if abs(den) < 1e-9:
                    continue
                t = ((x1 - px) * ey - (y1 - py) * ex) / den
                u = ((x1 - px) * dy - (y1 - py) * dx) / den
                if t > 0 and 0 <= u <= 1:
                    best = min(best, t)
            for cx, cy, r in self.POSTS:
                fx, fy = cx - px, cy - py
                proj = fx * dx + fy * dy
                d2 = fx * fx + fy * fy - proj * proj
                if proj > 0 and d2 < r * r:
                    best = min(best, proj - math.sqrt(r * r - d2))
            if best < 1e8:
                pts.append((b, best))
        return pts

    def run_motion(self, step, noise_mm=15.0):
        # A real laser's range noise: in a noise-free room the fit's quality, which is judged
        # against the spread of the ranges' changes, collapses to 0 walking forward.
        import random

        rnd = random.Random(1)
        tr, px, py, yaw = ht.HeadingTracker(), 0.0, 0.0, 0.0
        for i in range(40):
            px, py, yaw = step(px, py, yaw)
            scan = [(b, d + rnd.gauss(0.0, noise_mm)) for b, d in self.cast(px, py, yaw)]
            tr.update(scan, 1000.0 + i / 15.0)
        return tr.state()

    def test_signs(self):
        self.assertGreater(self.run_motion(lambda x, y, w: (x, y, w + 3.0))["yaw_rate_dps"], 40)
        self.assertLess(self.run_motion(lambda x, y, w: (x, y, w - 3.0))["yaw_rate_dps"], -40)
        fwd = self.run_motion(lambda x, y, w: (x + 80.0, y, w))
        self.assertEqual(fwd["speed_source"], "odometry")
        self.assertAlmostEqual(fwd["speed_mps"], 1.2, delta=0.15)
        self.assertGreater(self.run_motion(lambda x, y, w: (x, y + 40.0, w))["right_mps"], 0.45)

    def test_turning_on_the_spot_is_still_to_the_voice(self):
        state = self.run_motion(lambda x, y, w: (x, y, w + 3.0))
        self.assertEqual(sg.user_motion(state), (True, 0.0))


class SettingsAgreeTests(unittest.TestCase):
    def test_relations(self):
        from ble_bridge import core as ble, gatt_server
        from utils import mobile_delivery

        unit = open("deploy/shappe-guide.service").read()
        watchdog = float(re.search(r"^WatchdogSec=(\d+)", unit, re.M).group(1))
        scan, radar = 60.0 / c.LIDAR_RPM, 1.0 / c.RADAR_UPDATE_RATE
        front_cam, side_cam = 0.05, 3 * c.DETECTION_SIDE_MIN_INTERVAL_S
        checks = {
            "distance bands in order": c.WALK_MIN_TURN_FT < c.WALK_BLOCK_FT
            < c.SIMPLE_OBSTACLE_AHEAD_FT < c.SIMPLE_WALL_APPROACH_FT,
            "radar reaches the glass-ahead band":
                c.RADAR_NO_RETURN_FT >= c.SIMPLE_WALL_APPROACH_FT + c.SIMPLE_BAND_MARGIN_FT,
            "near-wall band inside the drift band": c.SIMPLE_WALL_NEAR_FT < c.SIMPLE_DRIFT_NEAR_FT,
            "warn fast, clear slowly": c.SIMPLE_ALERT_CONFIRM_S < c.SIMPLE_CLEAR_CONFIRM_S,
            "LiDAR confirm faster than 0.2 s":
                c.SIMPLE_LIDAR_CONFIRM_SCANS * scan <= c.SIMPLE_ALERT_CONFIRM_S + 1e-9,
            "camera confirm faster than 0.2 s":
                c.SIMPLE_CAMERA_CONFIRM_FRAMES * front_cam < c.SIMPLE_ALERT_CONFIRM_S,
            "LiDAR reader keeps every scan": lu.MIN_SCAN_INTERVAL <= scan,
            "LiDAR stale after >= 5 scans": c.LIDAR_STALE_S >= 5 * scan,
            "'Guidance stopped' after the stale window": c.SIMPLE_LIDAR_FRESH_S >= c.LIDAR_STALE_S,
            "radar stale after >= 5 frames": c.RADAR_STALE_S >= 5 * radar,
            "radar lag limit >= 2 frames": c.RADAR_MAX_LAG_S >= 2 * radar,
            "heading stale with the LiDAR": c.HEADING_STATE_STALE_S == c.LIDAR_STALE_S,
            "side labels refreshed before expiry": side_cam < c.SIMPLE_CAMERA_FRESH_S,
            "vehicle span holds its confirm frames":
                c.SIMPLE_VEHICLE_MIN_SPAN_S / front_cam >= c.SIMPLE_VEHICLE_CONFIRM_FRAMES,
            "vehicle window longer than its span": c.SIMPLE_VEHICLE_WINDOW_S > c.SIMPLE_VEHICLE_MIN_SPAN_S,
            "guidance floor below the scan rate": c.PI_VOICE_TICK_HZ <= 1.0 / scan,
            "phone 'too old' = server stale": abs(ble.ALERT_MAX_AGE_MS / 1000 - c.GUIDANCE_RESULT_STALE_S) < 1e-9,
            "phone audio report renewed >= 2x": mobile_delivery.REPORT_TTL_S >= 2 * gatt_server.HEALTH_S,
            "hang told before the heartbeat lapses": c.GUIDANCE_STALL_SAY_S < c.SERVICE_HEARTBEAT_MAX_AGE_S,
            "heartbeat inside the watchdog": c.SERVICE_HEARTBEAT_MAX_AGE_S < watchdog,
            "fault inside the hang notice": c.GUIDANCE_FAIL_S < c.GUIDANCE_STALL_SAY_S,
            "camera HFOV window holds the measured HFOV":
                c.DETECTION_DISTANCE_HFOV_DEG[0] <= c.SIMPLE_CAMERA_HFOV_DEG <= c.DETECTION_DISTANCE_HFOV_DEG[1],
        }
        for name, ok in checks.items():
            with self.subTest(name):
                self.assertTrue(ok)


if __name__ == "__main__":
    unittest.main()
