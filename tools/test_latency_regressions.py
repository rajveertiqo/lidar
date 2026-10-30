"""Hardware-free checks for the guidance-delay fixes of 2026-10-30; run with
venv/bin/python -m unittest tools.test_latency_regressions."""

import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

import config
from utils import gyro_reader, lidar_utils, pi_voice, simple_guidance as sg
from utils import unified_guidance, walk_policy
from utils import pi_camera_utils as pc
from utils.unified_navigation_system import detect_traffic_lights_from_detections


class SentenceOrderTests(unittest.TestCase):
    """Something on the path outranks a wall beside the user (it held "Wall ahead" to 4 ft)."""

    def say(self, **kw):
        args = dict(blocked=False, near=False, glass=False, wall=False, side=None)
        args.update(kw)
        return sg._sentence(**args)

    def test_wall_ahead_beats_a_near_side_wall(self):
        self.assertEqual(self.say(near=True, wall=True, close="left"), sg.WALL_AHEAD)

    def test_obstacle_ahead_beats_side_wall_and_drift(self):
        self.assertEqual(self.say(obstacle=True, close="right"), sg.OBSTACLE_AHEAD)
        self.assertEqual(self.say(obstacle=True, drift="left"), sg.OBSTACLE_AHEAD)

    def test_named_object_ahead_beats_a_near_side_wall(self):
        word = next(iter(sg.OBJECT_AHEAD))
        self.assertEqual(
            self.say(near=True, obj=word, close="left"), sg.OBJECT_AHEAD[word]
        )

    def test_side_wall_still_said_with_a_clear_path(self):
        self.assertEqual(self.say(close="left"), sg.NEAR_LEFT)

    def test_glass_ahead_still_first(self):
        self.assertEqual(
            self.say(near=True, glass=True, wall=True, close="left"), sg.GLASS_AHEAD
        )


class FusionTests(unittest.TestCase):
    """The nearer of LiDAR and radar, never their average (it put 3.5 ft at 4.25 ft)."""

    def test_nearer_wins_when_both_measured(self):
        ft, how = walk_policy.fuse(3.5, 5.0)
        self.assertEqual(ft, 3.5)
        self.assertLess(ft, config.WALK_BLOCK_FT)
        # "nearer" is _front_detail's word for a glass-gap disagreement only.
        self.assertFalse(how.startswith("nearer"))

    def test_glass_gap_still_named(self):
        ft, how = walk_policy.fuse(30.0, 1.0)
        self.assertEqual(ft, 1.0)
        self.assertTrue(how.startswith("nearer"))

    def test_the_measured_one_when_only_one_measured(self):
        ft, _ = walk_policy.fuse(12.0, config.RADAR_NO_RETURN_FT, True, False)
        self.assertEqual(ft, 12.0)

    def test_nothing_detected_unchanged(self):
        ft, _ = walk_policy.fuse(40.0, config.RADAR_NO_RETURN_FT, False, False)
        self.assertAlmostEqual(ft, (40.0 + config.RADAR_NO_RETURN_FT) / 2.0, places=2)


class GuidanceFaultTimeTests(unittest.TestCase):
    """A failing guidance step is a fault after GUIDANCE_FAIL_S, whatever the tick rate."""

    def tearDown(self):
        unified_guidance._note_guidance(None)

    def test_fault_after_its_time_not_its_ticks(self):
        boom = RuntimeError("boom")
        for _ in range(20):  # many ticks, at once: not yet a fault
            self.assertEqual(unified_guidance._note_guidance(boom), "")
        with patch.object(
            unified_guidance.time, "monotonic",
            return_value=time.monotonic() + config.GUIDANCE_FAIL_S + 0.01,
        ):
            self.assertIn("boom", unified_guidance._note_guidance(boom))
        unified_guidance._note_guidance(None)
        self.assertEqual(unified_guidance.guidance_fault(), "")


class ScanDrivenTickTests(unittest.TestCase):
    """The guidance loop decides on a new LiDAR scan at once, not at its next timer tick."""

    def test_tick_follows_the_scan(self):
        from utils import shared_state

        ticks = []

        def tick():
            ticks.append(time.monotonic())
            return {}

        with patch.object(pi_voice, "PI_VOICE_ENABLED", False), patch.object(
            pi_voice, "PI_VOICE_TICK_HZ", 2.0  # a 0.5 s timer: only a scan wakes it sooner
        ), patch.object(unified_guidance, "get_unified_guidance", tick), patch.object(
            unified_guidance, "guidance_fault", lambda: ""
        ):
            pi_voice.start()
            time.sleep(0.6)
            lags = []
            for _ in range(5):
                n, t0 = len(ticks), time.monotonic()
                shared_state.lidar_updated.set()  # a scan published
                while len(ticks) == n and time.monotonic() - t0 < 1.0:
                    time.sleep(0.001)
                lags.append(ticks[-1] - t0)
                time.sleep(0.12)
            pi_voice.stop()
            pi_voice._loop_thread.join(2.0)
            pi_voice._stall_watch.join(2.0)
        self.assertLess(max(lags), 0.05)  # was up to the timer period


class LidarConfirmTests(unittest.TestCase):
    """Two different LiDAR scans confirm a block (and a wall) at once; a repeat does not."""

    def raw(self, scan, lidar_block=True, wall=True):
        return {
            "blocked": True, "near": True, "glass": False, "wall": wall,
            "front_ft": 3.0, "lidar_block": lidar_block, "scan_index": scan,
        }

    def test_second_different_scan_confirms(self):
        world = sg._World()
        text, _ = world.update(self.raw(1), 100.0)
        self.assertNotEqual(sg._LEVEL.get(text, 0), sg._BLOCKED)
        text, _ = world.update(self.raw(2), 100.067)
        self.assertEqual(text, sg.BLOCKED_WALL)

    def test_same_scan_twice_does_not(self):
        world = sg._World()
        world.update(self.raw(1), 100.0)
        text, _ = world.update(self.raw(1), 100.05)  # a timer tick, no new scan
        self.assertNotEqual(sg._LEVEL.get(text, 0), sg._BLOCKED)

    def test_radar_only_block_keeps_the_confirm_time(self):
        world = sg._World()
        world.update(self.raw(1, lidar_block=False, wall=False), 100.0)
        text, _ = world.update(self.raw(2, lidar_block=False, wall=False), 100.067)
        self.assertNotEqual(sg._LEVEL.get(text, 0), sg._BLOCKED)
        text, _ = world.update(
            self.raw(3, lidar_block=False, wall=False), 100.0 + config.SIMPLE_ALERT_CONFIRM_S
        )
        self.assertEqual(sg._LEVEL.get(text, 0), sg._BLOCKED)


class StallWatchTests(unittest.TestCase):
    """A hung guidance loop is told by the voice itself; a failing one is not (fault_notice's)."""

    def setUp(self):
        self.said = []
        self.mode = "ok"
        self.release = threading.Event()

        def tick():
            if self.mode == "hang":
                self.release.wait(5.0)
            if self.mode == "fail":
                raise RuntimeError("boom")
            return {}

        def speak(text, urgent=False, seq=None, sink_usable=None):
            self.said.append((text, urgent))
            return "queued"

        self.patches = [
            patch.object(pi_voice, "PI_VOICE_ENABLED", False),
            patch.object(pi_voice, "GUIDANCE_STALL_SAY_S", 0.3),
            patch.object(pi_voice, "speak", speak),
            patch.object(unified_guidance, "get_unified_guidance", tick),
            patch.object(unified_guidance, "guidance_fault", lambda: ""),
        ]
        for p in self.patches:
            p.start()
        sg._fault["said_at"] = None

    def tearDown(self):
        self.release.set()
        pi_voice.stop()
        for t in (pi_voice._loop_thread, pi_voice._stall_watch):
            if t is not None:
                t.join(2.0)  # or the next start() finds the old loop still alive
        for p in reversed(self.patches):
            p.stop()
        sg._fault["said_at"] = None

    def test_hang_is_told_once_and_recovery_re_says(self):
        with patch("builtins.print"):
            pi_voice.start()
            time.sleep(0.5)
            self.mode = "hang"
            time.sleep(1.5)
            self.assertEqual(self.said, [(sg.SENSORS_OFFLINE, True)])
            self.mode = "ok"
            self.release.set()
            time.sleep(0.4)
        self.assertIsNotNone(sg._fault["said_at"])  # what stands is said again
        self.assertIsNone(pi_voice._stall_said_at)

    def test_failing_ticks_are_not_a_hang(self):
        with patch("builtins.print"):
            pi_voice.start()
            time.sleep(0.3)
            self.mode = "fail"
            time.sleep(1.2)
        self.assertEqual(self.said, [])


class TrafficLightSpellingTests(unittest.TestCase):
    def test_both_models_spellings(self):
        for name in ("traffic light", "traffic_light", "Traffic_Light"):
            with self.subTest(name=name):
                lights = detect_traffic_lights_from_detections(
                    {"front": [{"class_name": name, "confidence": 0.9}]}
                )
                self.assertTrue(lights["directions"]["front"]["has_light"])

    def test_a_sign_is_not_a_light(self):
        lights = detect_traffic_lights_from_detections(
            {"front": [{"class_name": "traffic_sign", "confidence": 0.9}]}
        )
        self.assertFalse(lights["directions"]["front"]["has_light"])


class SparseScanTests(unittest.TestCase):
    """The wearer's body (LIDAR_BLIND_ARCS_DEG) is not counted as missing returns."""

    swept = [i * 360.0 / 260.0 for i in range(260)]

    def fraction(self, sees):
        points = [
            (lidar_utils.correct_angle(a), 1500.0)
            for a in self.swept
            if sees(lidar_utils.correct_angle(a))
        ]
        return lidar_utils._valid_fraction(points, self.swept)

    def test_room_reads_full(self):
        self.assertEqual(self.fraction(lambda b: not lidar_utils._in_blind_arc(b)), 1.0)

    def test_one_building_in_range_is_a_reading(self):
        self.assertGreaterEqual(
            self.fraction(lambda b: 60 <= b < 90), config.LIDAR_MIN_VALID_FRACTION
        )

    def test_nothing_in_range_is_still_sparse(self):
        self.assertLess(self.fraction(lambda b: False), config.LIDAR_MIN_VALID_FRACTION)


class GyroSampleTests(unittest.TestCase):
    """One sample that cannot be folded in no longer ends the gyro thread."""

    def test_worker_survives_a_bad_sample(self):
        gyro = gyro_reader.GyroReader()

        class Sensor:
            def read(self):
                return {"gz": 1.0, "gx": 0.0, "gy": 0.0, "ax": 0.0, "ay": 0.0, "az": 1.0}

            def settings_intact(self):
                return True

        def connect():
            gyro._sensor, gyro._connected = Sensor(), True
            return True

        ingest, calls = gyro._ingest, []

        def flaky(*a, **k):
            calls.append(1)
            if len(calls) == 3:
                raise ValueError("bad sample")
            return ingest(*a, **k)

        with patch.object(gyro_reader, "GYRO_ENABLED", True), patch.object(
            gyro, "_connect", connect
        ), patch.object(gyro, "_ingest", flaky):
            gyro.start()
            time.sleep(0.3)
            alive = gyro._thread.is_alive()
            gyro.stop()
        self.assertTrue(alive)
        self.assertGreater(len(calls), 5)
        self.assertIn("sample skipped", gyro._last_error)


class FrontFirstDetectionTests(unittest.TestCase):
    """Cycling cameras: the front on every new picture; the sides capped, none starved."""

    def test_schedule(self):
        log, stored, cur = [], {}, {}

        class Engine:
            def infer_array(self, quad, bgr=False):
                log.append((cur["s"], time.monotonic()))
                time.sleep(0.04)  # the NPU
                return [], 40.0

            def close(self):
                pass

        def slice_(m, s):
            cur["s"] = s
            return real_slice(m, s)

        real_slice = pc._sector_slice
        quad = np.zeros((pc._QUAD[1], pc._QUAD[0], 3), np.uint8)
        order = [(s, n) for side in ("right", "left", "back")
                 for s, n in (("front", config.CAMERA_CYCLE_FRONT_FRAMES), (side, 1))]
        stop = threading.Event()

        def camera():
            i = 0
            while not stop.is_set():
                sector, n = order[i % len(order)]
                i += 1
                time.sleep(1 / 30)  # a frame lost to the switch
                for _ in range(n):
                    time.sleep(1 / 30)  # 30 fps: every front picture fits a 40 ms NPU
                    stored.setdefault(sector, []).append(time.monotonic())
                    pc._store_quad(sector, quad)

        with patch.object(pc, "_single", None), patch.object(
            pc, "_sector_slice", slice_
        ), patch.object(pc, "_view_check", lambda s, q: {
            "brightness": 100, "sharpness": 100, "dark": False, "blurred": False
        }), patch.object(pc, "update_camera_state", lambda d: None):
            cam = threading.Thread(target=camera, daemon=True)
            cam.start()
            gen = pc._generation
            det = threading.Thread(target=pc._detect, args=(Engine(), gen), daemon=True)
            det.start()
            time.sleep(4.0)
            pc._new_generation()  # retires the worker
            stop.set()
            det.join(2)
            cam.join(1)
        pc.current_frame, pc.current_quad_meta = None, {}

        fronts = [t for s, t in log if s == "front"]
        self.assertGreater(len(fronts), 0.8 * len(stored["front"]))  # ~every picture
        for side in ("right", "left", "back"):
            ts = [t for s, t in log if s == side]
            with self.subTest(side=side):
                self.assertTrue(ts, f"{side} never detected")
                gaps = [b - a for a, b in zip(ts, ts[1:])]
                self.assertLess(max(gaps, default=0.0), config.SIMPLE_CAMERA_FRESH_S)
        sides = sorted(t for s, t in log if s != "front")
        gaps = [b - a for a, b in zip(sides, sides[1:])]
        self.assertGreaterEqual(
            min(gaps), config.DETECTION_SIDE_MIN_INTERVAL_S - 0.05
        )


class VoiceHangTimeoutTests(unittest.TestCase):
    """A hung espeak-ng is given up on after its sentence's airtime plus a margin, not 8 s."""

    def test_limits_follow_airtime(self):
        short, long_ = sg.STOP_NO_WAY, sg.DRIFT_LEFT
        self.assertLess(pi_voice._say_timeout(long_), pi_voice._SAY_TIMEOUT_S)
        self.assertLess(pi_voice._say_timeout(short), pi_voice._say_timeout(long_))
        self.assertGreaterEqual(
            pi_voice._say_timeout(short), sg._airtime(short) + pi_voice._SAY_MARGIN_S - 1e-6
        )

    def test_hung_voice_given_up_early(self):
        events = []
        hang = ["python3", "-c", "import time; time.sleep(30)"]
        voice = pi_voice._Voice()
        with patch.object(pi_voice, "PI_VOICE_CMD", hang), patch.object(
            pi_voice, "_report", lambda seq, event, text, detail="": events.append(
                (event, time.monotonic())
            )
        ):
            voice.start()
            t0 = time.monotonic()
            voice.say(sg.STOP_NO_WAY, seq=1)
            deadline = t0 + pi_voice._SAY_TIMEOUT_S + 1.0
            while time.monotonic() < deadline and not any(e == "hung" for e, _ in events):
                time.sleep(0.05)
            voice.stop()
        hung = [t for e, t in events if e == "hung"]
        self.assertTrue(hung)
        self.assertLess(hung[0] - t0, pi_voice._say_timeout(sg.STOP_NO_WAY) + 0.5)
        self.assertLess(hung[0] - t0, pi_voice._SAY_TIMEOUT_S - 1.0)


class BleHealthOffCueLoopTests(unittest.TestCase):
    """A slow /health no longer holds the next /guidance/walk poll, and a cue, back."""

    def test_alerts_keep_their_pace_with_a_slow_health(self):
        from ble_bridge import gatt_server, protocol as P
        from ble_bridge.sources import make_health, make_walk
        from ble_bridge.test_ble_bridge import make_bridge

        bridge, _ = make_bridge()
        bridge.log = lambda key, message: None
        seq = iter(range(5, 10_000))

        class Source:
            def walk(self):
                return make_walk(next(seq), "Wall ahead", "Wall ahead")

            def health(self):
                time.sleep(0.5)  # the service slow to answer /health
                return make_health()

            def system_health(self):
                return {"front_glass_ok": True}

            def system(self):
                return {"soc_c": 50.0, "throttled": 0}

        server = gatt_server.Server.__new__(gatt_server.Server)
        server.bridge, server.source, server.stop = bridge, Source(), threading.Event()
        server.report_delivery = lambda: None
        sent = []
        server.post = sent.extend
        poller = threading.Thread(target=server.poll_loop, daemon=True)
        poller.start()
        time.sleep(1.5)
        server.stop.set()
        poller.join(2.0)
        alerts = sum(t == P.T_ALERT for _c, t, _p in sent)
        self.assertGreaterEqual(alerts, 11)  # ~10 a second; inline it was ~2 a second


if __name__ == "__main__":
    unittest.main()
