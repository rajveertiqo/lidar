"""Hardware-free checks for motion, clock changes and phone delivery readiness."""

import unittest
from unittest.mock import patch, Mock

from fastapi import HTTPException
from starlette.requests import Request

from utils import drift_guidance as drift, gyro_reader, mobile_delivery, pi_voice
from utils.motion_tracker import MotionTracker
from routers.guidance_router import MobileDeliveryReport, mobile_delivery_report


class GyroFreshnessTests(unittest.TestCase):
    def setUp(self):
        self.gyro = gyro_reader.GyroReader()
        self.gyro._connected = True
        self.gyro._buf.append(gyro_reader._Sample(1000, 0, 0, 1, 1, 50))

    def test_recent_sample_is_healthy(self):
        with patch.object(gyro_reader.time, "monotonic", return_value=50.01):
            self.assertTrue(self.gyro.healthy(1000.01))

    def test_wall_clock_catching_up_does_not_refresh_stalled_sensor(self):
        with patch.object(gyro_reader.time, "monotonic", return_value=65):
            self.assertFalse(self.gyro.healthy(1000.01))

    def test_future_sample_after_clock_step_is_unhealthy(self):
        with patch.object(gyro_reader.time, "monotonic", return_value=50.01):
            self.assertFalse(self.gyro.healthy(400))


class MotionPredictionTests(unittest.TestCase):
    def test_collision_prediction_uses_backward_and_sideways_motion(self):
        for speed, lateral, angle in ((-0.7, 0, 180), (0, 0.4, 90)):
            with self.subTest(angle=angle):
                heading = {
                    "speed_mps": speed, "right_mps": lateral,
                    "velocity_trusted": True, "moving": True,
                }
                result = drift.analyze(
                    {"points": {"angles": [angle], "distances": [1200]}},
                    {}, {}, heading=heading, now=1000,
                )
                self.assertIsNotNone(result["lidar_ttc_s"])
                self.assertIsNone(result["correction"])

    def test_motion_tracker_retains_signed_and_lateral_ego_velocity(self):
        fwd, right, *_ = MotionTracker._own_step({
            "speed_trusted": True, "velocity_trusted": True,
            "speed_mps": -0.7, "right_mps": 0.4,
        }, None, 0.1)
        self.assertAlmostEqual(fwd, -0.07)
        self.assertAlmostEqual(right, 0.04)


class MobileReadinessTests(unittest.TestCase):
    def tearDown(self):
        mobile_delivery.report(False, False)

    def test_connection_is_not_audio_readiness_and_reports_expire(self):
        with patch.object(mobile_delivery.time, "monotonic", return_value=100):
            mobile_delivery.report(True, False)
            self.assertTrue(mobile_delivery.status()["connected"])
            self.assertFalse(mobile_delivery.status()["usable"])
            mobile_delivery.report(True, True)
            self.assertTrue(mobile_delivery.status()["usable"])
        with patch.object(mobile_delivery.time, "monotonic", return_value=104):
            self.assertFalse(mobile_delivery.status()["usable"])
            self.assertFalse(mobile_delivery.status()["connected"])

    def test_voice_status_requires_an_available_delivery_route(self):
        with patch.object(pi_voice, "_running", False), patch.object(pi_voice, "AUDIO_ENABLED", True):
            mobile_delivery.report(False, False)
            self.assertFalse(pi_voice.status()["delivery_available"])
            mobile_delivery.report(True, True)
            self.assertTrue(pi_voice.status()["delivery_available"])
            with patch.object(pi_voice, "AUDIO_ENABLED", False):
                self.assertFalse(pi_voice.status()["delivery_available"])
        mobile_delivery.report(False, False)
        with (
            patch.object(pi_voice, "_running", True),
            patch.object(pi_voice, "AUDIO_ENABLED", True),
            patch.object(pi_voice._voice, "failing", False),
            patch.object(pi_voice, "audio_output", return_value={"name": "Speaker", "usable": True}),
        ):
            self.assertTrue(pi_voice.status()["delivery_available"])

    def test_health_is_down_without_any_confirmed_audio_route(self):
        import main
        from utils import pi_health, sensor_fusion, startup_gate
        with (
            patch.object(sensor_fusion, "get_system_health", return_value={"lidar_ok": True, "all_ok": True}),
            patch.object(pi_voice, "loop_status", return_value={"last_ok_age_s": 0.1}),
            patch.object(startup_gate, "state", return_value={"ready": True}),
            patch.object(pi_health, "read", return_value={}),
            patch.object(pi_voice, "status", return_value={
                "audio_enabled": True, "usable": False, "delivery_available": False,
            }),
        ):
            self.assertEqual(main.health_check().status_code, 503)
        with (
            patch.object(sensor_fusion, "get_system_health", return_value={"lidar_ok": True, "all_ok": True}),
            patch.object(pi_voice, "loop_status", return_value={"last_ok_age_s": 0.1}),
            patch.object(startup_gate, "state", return_value={"ready": True}),
            patch.object(pi_health, "read", return_value={}),
            patch.object(pi_voice, "status", return_value={
                "audio_enabled": True, "usable": False,
                "phone_usable": True, "delivery_available": True,
            }),
        ):
            self.assertEqual(main.health_check().status_code, 200)

    def test_only_local_bridge_can_report_phone_readiness(self):
        payload = MobileDeliveryReport(connected=True, audio_ready=True)
        def request(host):
            return Request({"type": "http", "client": (host, 1234), "headers": []})
        with self.assertRaises(HTTPException) as exc:
            mobile_delivery_report(payload, request("192.168.1.5"))
        self.assertEqual(exc.exception.status_code, 403)
        self.assertTrue(mobile_delivery_report(payload, request("127.0.0.1"))["ok"])

    def test_bridge_reports_only_subscribed_single_phone_with_ready_audio(self):
        from ble_bridge.gatt_server import Server
        server = Server.__new__(Server)
        server.source = Mock()
        server.bridge = Mock()
        server.bridge.phone_audio_ready.return_value = True
        server.connected = {"phone"}
        server.chars = {"alert": Mock(notifying=False)}
        server.report_delivery()
        server.source.delivery_status.assert_called_with(True, False)
        server.chars["alert"].notifying = True
        server.report_delivery()
        server.source.delivery_status.assert_called_with(True, True)
        server.connected.clear()
        server.report_delivery()
        server.source.delivery_status.assert_called_with(False, False)


if __name__ == "__main__":
    unittest.main()
