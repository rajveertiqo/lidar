"""Sensor/navigation regressions; run with venv/bin/python -m unittest tools.test_navigation_regressions."""

import time
import unittest
from unittest.mock import patch

import numpy as np
from fastapi import WebSocketDisconnect

from config import LIDAR_SECTOR_ORDER
from routers import navigation_router as navigation
from tools.drift_sim import ray_cast
from utils import heading_tracker


class HeadingMotionTests(unittest.TestCase):
    def test_only_stationary_scans_confirm_stillness_without_gyro(self):
        rng_state = np.random.get_state()
        self.addCleanup(np.random.set_state, rng_state)
        for name, dx, dy in (
            ("standing", 0, 0),
            ("forward", 70, 0),
            ("backward", -70, 0),
            ("sideways", 0, 40),
        ):
            with self.subTest(motion=name), patch.object(heading_tracker, "GYRO_ENABLED", False):
                np.random.seed(42)
                tracker = heading_tracker.HeadingTracker()
                now = time.time()
                for i in range(25):
                    scan = ray_cast(
                        i * dx, i * dy, 0, 2000,
                        length_mm=6500, back_mm=-6500, noise_mm=2,
                    )
                    state = tracker.update(scan, now + i * 0.1)
                self.assertTrue(state["speed_observable"])
                self.assertEqual(state["confidently_still"], name == "standing")
                self.assertEqual(state["motion_direction"], name if name != "standing" and name != "sideways"
                                 else "still" if name == "standing" else "right")
                if name == "backward":
                    self.assertLess(state["speed_mps"], -0.5)
                if name == "sideways":
                    self.assertTrue(state["velocity_trusted"])
                    self.assertGreater(state["right_mps"], 0.3)


class NavigationTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        now = time.time()
        self.lidar = {
            "last_update": now,
            "valid_fraction": 1.0,
            "sectors": {
                s: {"min_dist": 6000, "count": 20} for s in LIDAR_SECTOR_ORDER
            },
            "cones": {
                s: {"min_dist": 6000} for s in ("front", "left", "right", "back")
            },
        }
        self.camera = {"last_update": now, "front": {"detections": []}}
        for name, value in (
            ("get_lidar_state", self.lidar),
            ("get_camera_state", self.camera),
        ):
            mock = patch.object(navigation, name, return_value=value)
            mock.start()
            self.addCleanup(mock.stop)
        mock = patch.object(navigation, "update_navigation_state")
        mock.start()
        self.addCleanup(mock.stop)

    async def test_invalid_scans_never_authorize_forward_motion(self):
        for scan in (
            [], [{}], [{"distance_mm": 6000}],
            [{"angle_deg": 0, "distance_mm": -1}],
            [{"angle_deg": 0, "distance_mm": float("nan")}],
            [{"angle_deg": float("inf"), "distance_mm": 6000}],
            [{"angle_deg": 0, "distance_mm": True}],
            [{"angle_deg": 0, "distance_mm": 50001}],
        ):
            with self.subTest(scan=scan):
                result = await navigation.make_navigation_decision(
                    navigation.NavigationRequest(scan_data=scan)
                )
                self.assertEqual(result["action"], "no_lidar_data")
                self.assertEqual(result["speed"]["multiplier"], 0)

    async def test_valid_measurement_survives_mixed_invalid_input(self):
        result = await navigation.make_navigation_decision(
            navigation.NavigationRequest(scan_data=[
                {}, {"angle_deg": 0, "distance_mm": 6000},
                {"angle_deg": 90, "distance_mm": float("nan")},
            ])
        )
        self.assertEqual(result["action"], "go_forward")
        self.assertEqual(result["sensor_health"]["lidar_points"], 1)

    async def test_sparse_scan_with_fresh_timestamp_is_unavailable(self):
        self.lidar["valid_fraction"] = 0.01
        result = await navigation.make_navigation_decision_from_state()
        self.assertEqual(result["action"], "no_lidar_data")
        self.assertIn("almost no returns", result["sensor_health"]["lidar_reason"])

    async def test_only_current_real_camera_frame_can_trigger_crossing(self):
        for label, stamp, simulated, crossing in (
            ("fresh", time.time(), False, True),
            ("stale", time.time() - 3600, False, False),
            ("simulated", time.time(), True, False),
        ):
            with self.subTest(frame=label):
                self.camera["front"] = {
                    "frame_time": stamp,
                    "simulated": simulated,
                    "detections": [{"class_name": "traffic light", "confidence": 0.9}],
                }
                result = await navigation.make_navigation_decision_from_state()
                self.assertEqual(result["crossing_mode"], crossing)
                self.assertEqual(result["action"], "wait_for_signal" if crossing else "go_forward")

    async def test_websocket_rejects_invalid_scan(self):
        class Socket:
            result = None

            async def accept(self):
                pass

            async def receive_json(self):
                if self.result is not None:
                    raise WebSocketDisconnect()
                return {"scan_data": [{}]}

            async def send_json(self, result):
                self.result = result

        socket = Socket()
        await navigation.navigation_websocket(socket)
        self.assertEqual(socket.result["action"], "no_lidar_data")


if __name__ == "__main__":
    unittest.main()
