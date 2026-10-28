"""Desk tests for the BLE bridge prototype — no Bluetooth, no sensors, no network.

venv/bin/python -m unittest ble_bridge.test_ble_bridge -v
"""

import contextlib
import io
import json
import os
import sys
import tempfile
import threading
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from ble_bridge import protocol as P  # noqa: E402
from ble_bridge.core import (  # noqa: E402
    HEALTH_STALE_S,
    STATE_HEARTBEAT_S,
    UNREACHABLE_AFTER_S,
    Bridge,
    load_vocab,
)
from ble_bridge.sources import MockSource, SCENARIOS, make_health, make_walk  # noqa: E402


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def frames_of(outs, mtu=23):
    return [
        (char, P.encode(t, payload, i, mtu))
        for i, (char, t, payload) in enumerate(outs)
    ]


def decoded(outs):
    return [(char, P.decode_message(t, payload)) for char, t, payload in outs]


def strict_json(payload):
    """Standard JSON only: NaN / Infinity are what a phone's decoder would reject."""

    def refuse(constant):
        raise ValueError(f"non-standard JSON {constant}")

    return json.loads(payload.decode("utf-8"), parse_constant=refuse)


def assert_sendable(outs, mtu=23):
    for _c, t, payload in outs:
        P.encode(t, payload, 0, mtu)  # raises if too large for this MTU
        if t in P.JSON_TYPES:
            strict_json(payload)


class FakeService:
    def __init__(self):
        self.marks = []

    def speech_log(self, limit):
        return {
            "lines": [
                {
                    "time": "12:00:00",
                    "source": "simple-guidance",
                    "level": "urgent",
                    "text": "Blocked by glass",
                    "cue_seq": 3,
                    "via": "pi",
                    "outcome": "completed",
                    "detail": "x",
                }
            ]
        }

    def mark(self, kind, note):
        self.marks.append((kind, note))
        return {"ok": True, "kind": kind, "last_said": "Blocked by glass"}


def make_bridge(clock=None, mock=False):
    clock = clock or Clock()
    return Bridge(
        load_vocab(),
        {"model": "test", "software": "test", "mock": mock},
        service=FakeService(),
        clock=clock,
        wall=lambda: 1791504000.0,
    ), clock


class ProtocolTests(unittest.TestCase):
    def test_records_fit_the_smallest_mtu(self):
        rec = P.pack_alert(1, 2, 50, 3, 2, P.AF_ANNOUNCE, 3.6)
        frames = P.encode(P.T_ALERT, rec, 0, 23)
        self.assertEqual(len(frames), 1)
        self.assertEqual(len(frames[0]), 20)  # = ATT_MTU 23 - 3
        st = P.encode(P.T_STATE, P.pack_state(1, 2, 50, 3, 2, 0, 0xFFFF), 0, 23)
        self.assertEqual(len(st[0]), 20)

    def test_alert_round_trip(self):
        rec = P.pack_alert(7, 123456, 87.4, 5, 2, P.AF_ANNOUNCE | P.AF_URGENT, 3.64)
        a = P.unpack_alert(rec)
        self.assertEqual(
            (a["epoch"], a["cue_seq"], a["age_ms"], a["phrase_id"], a["level"]),
            (7, 123456, 87, 5, 2),
        )
        self.assertTrue(a["urgent"] and a["announce"] and not a["replay"])
        self.assertAlmostEqual(a["front_ft"], 3.6)

    def test_unknown_values(self):
        a = P.unpack_alert(
            P.pack_alert(1, 1, None, P.PHRASE_UNLISTED, -1, 0, float("nan"))
        )
        self.assertIsNone(a["age_ms"])
        self.assertIsNone(a["front_ft"])
        self.assertEqual(a["level"], -1)
        self.assertEqual(
            P.unpack_alert(P.pack_alert(1, 1, 10**9, 1, 0, 0, -3))["age_ms"],
            P.AGE_MAX_MS,
        )

    def test_fragmentation_round_trip_at_every_mtu(self):
        body = P.json_bytes(
            {"v": 1, "event": "health_changed", "data": {"warnings": ["x" * 150] * 4}}
        )
        for mtu in (23, 64, 185, 247, 517):
            frames = P.encode(P.T_EVENT, body, 65535, mtu)
            self.assertTrue(all(len(f) <= mtu - 3 for f in frames))
            r = P.Reassembler()
            got = [r.feed(f, 0.0) for f in frames]
            self.assertTrue(all(g is None for g in got[:-1]))
            self.assertEqual(got[-1], (P.T_EVENT, 65535, body))

    def test_reassembler_drops_gaps_and_stale_parts(self):
        body = b"y" * 100
        frames = P.encode(P.T_COMMAND, body, 9, 23)
        r = P.Reassembler()
        r.feed(frames[0], 0.0)
        with self.assertRaises(P.ProtocolError):
            r.feed(frames[2], 0.1)  # fragment 1 lost
        r2 = P.Reassembler(timeout_s=2.0)
        r2.feed(frames[0], 0.0)
        r2.feed(frames[1], 0.5)
        with self.assertRaises(P.ProtocolError):
            r2.feed(frames[2], 5.0)  # the partial message expired
        self.assertEqual(r2.dropped, 1)

    def test_header_validation(self):
        with self.assertRaises(P.ProtocolError) as cm:
            P.parse_header(b"\x02\x10\x00\x00\x00\x01")
        self.assertEqual(cm.exception.code, "PROTOCOL_MISMATCH")
        with self.assertRaises(P.ProtocolError):
            P.parse_header(b"\x01\x99\x00\x00\x00\x01")
        with self.assertRaises(P.ProtocolError):
            P.parse_header(b"\x01\x10\x00\x00\x01\x01")  # index 1 of count 1
        with self.assertRaises(P.ProtocolError):
            P.parse_header(b"\x01\x10")

    def test_sequence_arithmetic(self):
        self.assertTrue(P.seq16_newer(0, 65535))
        self.assertFalse(P.seq16_newer(65535, 0))
        self.assertTrue(P.cue_newer(2, 1, 1, 500))  # new epoch wins
        self.assertFalse(P.cue_newer(1, 4, 1, 4))  # duplicate
        self.assertTrue(P.cue_newer(1, 5, None, None))

    def test_too_large(self):
        with self.assertRaises(P.ProtocolError):
            P.encode(P.T_EVENT, b"z" * (P.MAX_MESSAGE_BYTES + 1), 0, 517)
        self.assertEqual(P.max_message_bytes(23), 255 * 14)
        P.encode(P.T_EVENT, b"z" * P.max_message_bytes(23), 0, 23)
        with self.assertRaises(P.ProtocolError):
            P.encode(P.T_EVENT, b"z" * (P.max_message_bytes(23) + 1), 0, 23)

    def test_huge_values_saturate_and_garbage_is_unknown(self):
        """An OverflowError here ended the poll thread for good (loop.age_s = 1e306)."""
        for big in (float("inf"), 1e306 * 1000.0, 1e308):
            a = P.unpack_alert(P.pack_alert(1, 1, big, 1, 1, 0, big))
            self.assertEqual((a["age_ms"], a["front_ft"]), (P.AGE_MAX_MS, 6553.4))
            self.assertEqual(
                P.unpack_state(P.pack_state(1, 1, big, 1, 1, 0, 0))["age_ms"],
                P.AGE_MAX_MS,
            )
        for junk in (10**400, "abc", [], float("-inf"), float("nan")):
            a = P.unpack_alert(P.pack_alert(1, 1, junk, 1, 1, 0, junk))
            self.assertEqual((a["age_ms"], a["front_ft"]), (None, None))

    def test_json_is_strict_and_always_utf8(self):
        with self.assertRaises(ValueError):
            P.json_bytes({"t": float("inf")})  # never sent as non-standard JSON
        self.assertEqual(strict_json(P.json_bytes({"t": "a\ud800b"})), {"t": "a?b"})


class VocabTests(unittest.TestCase):
    def test_registry_matches_the_device(self):
        from utils import simple_guidance as sg

        vocab = load_vocab()
        self.assertEqual(
            set(vocab["by_text"]),
            set(sg.ALLOWED),
            "registry out of date: python -m ble_bridge.make_vocab",
        )
        for text, entry in vocab["by_text"].items():
            self.assertEqual(entry["level"], sg._LEVEL[text], text)
            self.assertEqual(entry["action"], sg._ACTION.get(text, "unknown"), text)
            self.assertNotIn(entry["id"], (P.PHRASE_NONE, P.PHRASE_UNLISTED))

    def test_ids_unique(self):
        with open(
            os.path.join(HERE, "contract", "vocab_v1.json"), encoding="utf-8"
        ) as fh:
            reg = json.load(fh)
        ids = [p["id"] for p in reg["phrases"]]
        self.assertEqual(len(ids), len(set(ids)))


class BridgeTests(unittest.TestCase):
    def test_urgent_delivery_across_disconnect_and_reconnect(self):
        b, clock = make_bridge()
        b.on_connect()
        b.on_walk(make_walk(10))
        first = b.on_walk(make_walk(11, announced="Blocked", speak=True, urgent=True))
        self.assertTrue(P.unpack_alert(next(p for _, t, p in first if t == P.T_ALERT))["urgent"])
        b.on_disconnect()
        clock.t += 0.2
        # Polling continues off-link. Its old warning must be replay-only on read.
        b.on_walk(make_walk(12, announced="Blocked by glass", speak=True, urgent=True))
        b.on_connect()
        self.assertTrue(P.unpack_alert(b.alert_value())["replay"])
        repeated = b.on_walk(make_walk(12, announced="Blocked by glass", speak=True, urgent=True))
        self.assertFalse(any(t == P.T_ALERT for _, t, _ in repeated))
        new = b.on_walk(make_walk(13, announced="Blocked", speak=True, urgent=True))
        self.assertEqual(sum(t == P.T_ALERT for _, t, _ in new), 1)
        self.assertFalse(any(t == P.T_ALERT for _, t, _ in b.on_walk(make_walk(13))))

    def test_restart_does_not_replay_old_warning_and_next_urgent_is_delivered(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(100))
        b.on_walk(make_walk(101, announced="Blocked", speak=True, urgent=True))
        epoch = b.epoch
        restarted = b.on_walk(make_walk(0))
        self.assertNotEqual(b.epoch, epoch)
        self.assertFalse(any(t == P.T_ALERT for _, t, _ in restarted))
        new = b.on_walk(make_walk(1, announced="Blocked by glass", speak=True, urgent=True))
        record = P.unpack_alert(next(p for _, t, p in new if t == P.T_ALERT))
        self.assertEqual(record["epoch"], b.epoch)
        self.assertTrue(record["urgent"])

    def test_first_snapshot_only_learns(self):
        b, _ = make_bridge()
        outs = b.on_walk(make_walk(5, "Walk straight", "Walk straight", speak=True))
        self.assertFalse([o for o in outs if o[0] == "alert"])

    def test_cue_step_makes_one_alert_and_its_text(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(5, "Walk straight", "Walk straight"))
        clock.t += 0.1
        outs = b.on_walk(
            make_walk(
                6,
                "Blocked by glass",
                "Blocked by glass",
                speak=True,
                urgent=True,
                front_ft=3.6,
            )
        )
        alerts = [d for c, d in decoded(outs) if c == "alert"]
        self.assertEqual([a["kind"] for a in alerts], ["alert", "alert_text"])
        a = alerts[0]
        self.assertEqual(a["phrase_id"], b.vocab["by_text"]["Blocked by glass"]["id"])
        self.assertTrue(a["urgent"])
        self.assertEqual(a["level"], 2)
        self.assertAlmostEqual(a["front_ft"], 3.6)
        self.assertEqual(alerts[1]["text"], "Blocked by glass")
        clock.t += 0.1  # the same cue again: nothing new
        again = b.on_walk(
            make_walk(6, "Blocked by glass", "Blocked by glass", front_ft=3.6)
        )
        self.assertFalse([o for o in again if o[0] == "alert"])

    def test_missed_admitting_tick_still_alerts(self):
        """The poll can miss the one tick where speak=True; the cue step is what counts."""
        b, clock = make_bridge()
        b.on_walk(make_walk(1, "Walk straight", "Walk straight"))
        clock.t += 0.3
        outs = b.on_walk(make_walk(2, "Turn left", "Turn left", speak=False))
        alert = [d for c, d in decoded(outs) if d["kind"] == "alert"][0]
        self.assertTrue(alert["urgent"])  # from the registry, not the tick

    def test_device_voice_and_audio_flags(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(1, audio_enabled=False))
        clock.t += 0.1
        a = [
            d
            for c, d in decoded(
                b.on_walk(
                    make_walk(
                        2, "Wall ahead", "Wall ahead", speak=True, audio_enabled=False
                    )
                )
            )
            if d["kind"] == "alert"
        ][0]
        self.assertFalse(a["audio_enabled"])
        self.assertFalse(a["device_voice"])
        clock.t += 0.1
        a = [
            d
            for c, d in decoded(
                b.on_walk(
                    make_walk(
                        3,
                        "Wall ahead",
                        "Wall ahead",
                        speak=True,
                        device_voice_usable=True,
                        accepted_seq=3,
                    )
                )
            )
            if d["kind"] == "alert"
        ][0]
        self.assertTrue(a["device_voice"] and a["audio_enabled"])

    def test_unlisted_sentence_uses_text(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(1))
        clock.t += 0.1
        outs = decoded(b.on_walk(make_walk(2, "", "Ready.", speak=True)))
        alert = [d for c, d in outs if d["kind"] == "alert"][0]
        self.assertEqual(alert["phrase_id"], P.PHRASE_UNLISTED)
        self.assertEqual(
            [d for c, d in outs if d["kind"] == "alert_text"][0]["text"], "Ready."
        )

    def test_service_restart_bumps_epoch(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(40))
        epoch = b.epoch
        clock.t += 0.1
        outs = decoded(b.on_walk(make_walk(0)))
        self.assertEqual(b.epoch, epoch + 1)
        self.assertIn("service_restarted", [d.get("event") for c, d in outs])
        clock.t += 0.1
        alert = [
            d
            for c, d in decoded(
                b.on_walk(make_walk(1, "Walk straight", "Walk straight"))
            )
            if d["kind"] == "alert"
        ][0]
        self.assertEqual((alert["epoch"], alert["cue_seq"]), (epoch + 1, 1))

    def test_replay_flag_on_read(self):
        b, clock = make_bridge()
        self.assertIsNone(b.alert_value())
        b.on_walk(make_walk(1))
        clock.t += 0.1
        b.on_walk(make_walk(2, "Wall ahead", "Wall ahead", speak=True))
        self.assertTrue(P.unpack_alert(b.alert_value())["replay"])

    def test_state_heartbeat_and_rate_limit(self):
        b, clock = make_bridge()
        first = b.on_walk(make_walk(1, "Walk straight"))
        self.assertEqual([o[0] for o in first if o[0] == "state"], ["state"])
        clock.t += 0.05
        self.assertFalse(
            [o for o in b.tick() if o[0] == "state"]
        )  # unchanged, too soon
        clock.t += 0.1
        self.assertFalse(
            [o for o in b.on_walk(make_walk(1, "Wall ahead")) if o[0] == "state"]
        )
        clock.t += 0.1  # changed, >= 0.2 s
        st = [d for c, d in decoded(b.tick()) if c == "state"]
        self.assertEqual(st[0]["phrase_id"], b.vocab["by_text"]["Wall ahead"]["id"])
        clock.t += STATE_HEARTBEAT_S + 0.01
        self.assertTrue([o for o in b.tick() if o[0] == "state"])  # heartbeat

    def test_health_events(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(1))
        evs = [
            d["event"]
            for c, d in decoded(
                b.on_health(
                    make_health(),
                    {"front_glass_ok": True},
                    {"soc_c": 50.0, "throttled": 0},
                )
            )
            if d["kind"] == "event"
        ]
        self.assertIn("device_ready", evs)
        clock.t += 1
        evs = [
            d
            for c, d in decoded(
                b.on_health(
                    make_health(
                        "degraded",
                        radar=False,
                        warnings=["Radar offline — glass may not be detected"],
                    ),
                    {"front_glass_ok": False},
                    {"soc_c": 83.0, "throttled": 0x5},
                )
            )
            if d["kind"] == "event"
        ]
        names = [e["event"] for e in evs]
        self.assertIn("sensor_degraded", names)
        self.assertEqual(
            {e["data"]["sensor"] for e in evs if e["event"] == "sensor_degraded"},
            {"radar", "front_glass"},
        )
        for name in ("thermal_warning", "power_warning", "health_changed"):
            self.assertIn(name, names)
        self.assertNotIn("device_ready", names)  # still ready: not announced again
        state = [
            d
            for c, d in decoded(b.tick() or [("state", P.T_STATE, b.state_value())])
            if c == "state"
        ][-1]
        self.assertEqual(state["status"], "degraded")
        self.assertTrue(
            state["health"]["thermal_warn"] and state["health"]["undervoltage"]
        )

    def test_one_slow_reply_is_not_an_outage(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(1))
        b.on_health(make_health())
        clock.t += 0.5
        events = lambda outs: [
            d["event"] for c, d in decoded(outs) if d["kind"] == "event"
        ]
        self.assertEqual(events(b.on_unreachable("timed out")), [])
        self.assertEqual(P.unpack_state(b.state_value())["status"], "ok")
        clock.t += UNREACHABLE_AFTER_S  # still no answer: now it is one
        self.assertEqual(events(b.on_unreachable("timed out")), ["service_unreachable"])

    def test_health_unanswered_is_down_not_ok(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(1))
        b.on_health(make_health())
        events = lambda outs: [
            d["event"] for c, d in decoded(outs) if d["kind"] == "event"
        ]
        seen = []
        for _ in range(int(HEALTH_STALE_S) + 2):  # /guidance/walk answers, /health not
            clock.t += 1
            seen += events(b.on_walk(make_walk(1)) + b.on_unreachable("timed out"))
            seen += events(b.tick())
        self.assertNotIn("service_unreachable", seen)
        self.assertIn("health_changed", seen)
        self.assertEqual(P.unpack_state(b.state_value())["status"], "down")
        self.assertIn("health_changed", events(b.on_health(make_health())))
        clock.t += 0.3  # STATE goes out at most every STATE_MIN_INTERVAL_S
        b.tick()
        self.assertEqual(P.unpack_state(b.state_value())["status"], "ok")

    def test_unreachable_is_reported_once_and_clears(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(1))
        b.on_health(make_health())
        clock.t += UNREACHABLE_AFTER_S
        first = [
            d["event"]
            for c, d in decoded(b.on_unreachable("connection refused"))
            if d["kind"] == "event"
        ]
        clock.t += 1
        second = [
            d
            for c, d in decoded(b.on_unreachable("connection refused"))
            if d["kind"] == "event"
        ]
        self.assertEqual(first, ["service_unreachable"])
        self.assertEqual(second, [])
        st = P.unpack_state(b.state_value())
        self.assertEqual(st["status"], "unreachable")
        self.assertFalse(any(st["health"].values()))
        clock.t += 1
        b.on_walk(make_walk(1))
        evs = [
            d["event"]
            for c, d in decoded(b.on_health(make_health()))
            if d["kind"] == "event"
        ]
        self.assertIn("device_ready", evs)

    def test_mock_identity_is_always_marked_simulated(self):
        b, _ = make_bridge(mock=True)
        b.on_walk(make_walk(1))
        b.on_health(make_health())
        self.assertTrue(P.unpack_state(b.state_value())["health"]["simulated_input"])
        self.assertTrue(b.protocol_info()["mock"])

    def test_protocol_info_fits_one_long_read(self):
        """Served as ONE frame by a GATT read: must fit 512 B even with long real strings."""
        long_identity = {
            "model": "Shappe Guide (Raspberry Pi 5)" * 3,
            "mock": False,
            "software": "guide e209c48+dirty; ble-bridge 0.1; protocol 1" * 3,
        }
        b = Bridge(load_vocab(), long_identity, clock=Clock(), wall=lambda: 0.0)
        frame = P.encode(P.T_PROTOCOL_INFO, P.json_bytes(b.protocol_info()), 0, 517)
        self.assertEqual(len(frame), 1)
        self.assertLessEqual(len(frame[0]), 480)  # margin for future fields
        hello = P.parse_json(
            b.on_command(P.json_bytes({"v": 1, "id": 1, "cmd": "hello"}))[0][2]
        )
        self.assertEqual(
            hello["result"]["model"], long_identity["model"]
        )  # strings live here

    def test_new_subscriber_gets_state_at_once(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(1, "Walk straight"))
        clock.t += 0.05
        self.assertFalse(b.tick())  # timers say nothing is due ...
        self.assertEqual(
            [o[0] for o in b.tick(force=True)], ["state"]
        )  # ... a subscriber still gets one


class CommandTests(unittest.TestCase):
    def test_phone_audio_readiness_requires_explicit_renewed_heartbeat(self):
        b, clock = make_bridge()
        b.on_connect()
        self.cmd(b, {"v": 1, "id": 1, "cmd": "hello"})
        self.assertFalse(b.phone_audio_ready())
        self.cmd(b, {"v": 1, "id": 2, "cmd": "ping", "args": {"audio_ready": True}})
        self.assertTrue(b.phone_audio_ready())
        clock.t += 6
        self.assertFalse(b.phone_audio_ready())
        self.cmd(b, {"v": 1, "id": 3, "cmd": "ping", "args": {"audio_ready": True}})
        self.assertTrue(b.phone_audio_ready())
        b.on_disconnect()
        b.on_connect()
        self.assertFalse(b.phone_audio_ready())
        bad = self.cmd(b, {"v": 1, "id": 4, "cmd": "ping", "args": {"audio_ready": "true"}})
        self.assertEqual(bad[0]["error"]["code"], "BAD_ARGS")

    def cmd(self, b, obj):
        raw = obj if isinstance(obj, bytes) else P.json_bytes(obj)
        outs = b.on_command(raw)
        return [d for c, d in decoded(outs)]

    def test_hello_sets_mtu(self):
        b, _ = make_bridge()
        r = self.cmd(
            b,
            {"v": 1, "id": 1, "cmd": "hello", "args": {"protocol": 1, "att_mtu": 185}},
        )[0]
        self.assertTrue(r["ok"])
        self.assertEqual(r["result"]["att_mtu"], 185)
        self.assertEqual(b.att_mtu, 185)
        self.assertEqual(r["result"]["vocab"]["revision"], b.vocab["revision"])

    def test_errors(self):
        b, _ = make_bridge()
        cases = [
            (b"{not json", "BAD_JSON"),
            ({"v": 2, "id": 1, "cmd": "ping"}, "PROTOCOL_MISMATCH"),
            ({"v": 1, "id": "x", "cmd": "ping"}, "BAD_ARGS"),
            ({"v": 1, "id": 2, "cmd": "launch"}, "UNKNOWN_COMMAND"),
            (
                {
                    "v": 1,
                    "id": 3,
                    "cmd": "set_device_settings",
                    "args": {"voice_rate": 200},
                },
                "NOT_SUPPORTED",
            ),
            (
                {"v": 1, "id": 4, "cmd": "alert_ack", "args": {"outcome": "loud"}},
                "BAD_ARGS",
            ),
            (
                {"v": 1, "id": 5, "cmd": "get_speech_log", "args": {"limit": 99}},
                "BAD_ARGS",
            ),
            ({"v": 1, "id": 6, "cmd": "hello", "args": {"att_mtu": 9000}}, "BAD_ARGS"),
        ]
        for msg, code in cases:
            r = self.cmd(b, msg)[0]
            self.assertFalse(r["ok"], msg)
            self.assertEqual(r["error"]["code"], code, msg)

    def test_rate_limit(self):
        b, clock = make_bridge()
        codes = [
            self.cmd(b, {"v": 1, "id": i, "cmd": "ping"})[0]
            .get("error", {})
            .get("code")
            for i in range(12)
        ]
        self.assertEqual(codes[:10], [None] * 10)
        self.assertEqual(codes[10], "BUSY")
        clock.t += 1.1
        self.assertTrue(self.cmd(b, {"v": 1, "id": 99, "cmd": "ping"})[0]["ok"])

    def test_session_lifecycle(self):
        b, clock = make_bridge()
        out = self.cmd(b, {"v": 1, "id": 1, "cmd": "session_start"})
        sid = out[0]["result"]["id"]
        self.assertEqual(out[1]["event"], "session_started")
        again = self.cmd(b, {"v": 1, "id": 2, "cmd": "session_start"})
        self.assertEqual(
            again[0]["result"]["id"], sid
        )  # idempotent: resumes, never forks
        self.assertEqual(len(again), 1)  # ... and announces nothing new
        clock.t += 61
        stop = self.cmd(b, {"v": 1, "id": 3, "cmd": "session_stop"})
        self.assertEqual(stop[0]["result"]["duration_s"], 61.0)
        self.assertEqual(stop[1]["event"], "session_stopped")
        self.assertEqual(
            self.cmd(b, {"v": 1, "id": 4, "cmd": "session_stop"})[0]["error"]["code"],
            "BAD_ARGS",
        )

    def test_queries(self):
        b, _ = make_bridge()
        b.on_walk(
            make_walk(
                1,
                "Person ahead",
                front_ft=9.0,
                obstacles=[
                    {
                        "object": "person",
                        "clock": 12,
                        "sector": "front",
                        "distance_ft": 9.0,
                        "urgency": "clear",
                        "source": "both",
                        "shape": None,
                    }
                ],
            )
        )
        b.on_health(make_health())
        cur = self.cmd(b, {"v": 1, "id": 1, "cmd": "get_current"})[0]["result"]
        self.assertEqual(cur["instruction"], "Person ahead")
        sur = self.cmd(b, {"v": 1, "id": 2, "cmd": "get_surroundings"})[0]["result"]
        self.assertEqual(sur["items"][0]["object"], "person")
        st = self.cmd(b, {"v": 1, "id": 3, "cmd": "get_status"})[0]["result"]
        self.assertEqual(st["status"], "ok")
        self.assertIsNone(st["battery"])
        log = self.cmd(
            b, {"v": 1, "id": 4, "cmd": "get_speech_log", "args": {"limit": 5}}
        )[0]
        self.assertEqual(log["result"]["entries"][0]["text"], "Blocked by glass")
        mark = self.cmd(
            b, {"v": 1, "id": 5, "cmd": "mark", "args": {"kind": "missed"}}
        )[0]
        self.assertTrue(mark["ok"])
        self.assertEqual(b.service.marks, [("missed", "")])
        ack = self.cmd(
            b,
            {
                "v": 1,
                "id": 6,
                "cmd": "alert_ack",
                "args": {"epoch": 1, "cue_seq": 2, "outcome": "spoken"},
            },
        )[0]
        self.assertTrue(ack["ok"])
        diag = self.cmd(b, {"v": 1, "id": 7, "cmd": "get_diagnostics"})[0]["result"]
        self.assertEqual(diag["acks"][-1]["outcome"], "spoken")

    def test_every_response_fits_the_message_limit(self):
        b, _ = make_bridge()
        for name in (
            "hello",
            "get_status",
            "get_current",
            "get_surroundings",
            "get_diagnostics",
        ):
            for _c, t, payload in b.on_command(
                P.json_bytes({"v": 1, "id": 1, "cmd": name})
            ):
                self.assertLessEqual(len(payload), P.MAX_MESSAGE_BYTES)
                P.encode(t, payload, 0, 23)  # must not raise at the smallest MTU


class RobustnessTests(unittest.TestCase):
    """The 2026-10-22 fuzzing pass: a bad value from the service, the app or the disk must
    not stop alerts, end a thread, or leave a command without a sendable answer."""

    def cmd(self, b, obj):
        b.clock.t += 0.2  # stay under the command rate limit
        outs = b.on_command(obj if isinstance(obj, bytes) else P.json_bytes(obj))
        assert_sendable(outs, b.att_mtu)
        return [d for c, d in decoded(outs)]

    def test_unwritable_state_file_keeps_alerts_flowing(self):
        """Full or read-only SD card: the bridge starts, a service restart still bumps the epoch
        (in memory) and alerts go on. One log line, not one per failed save."""
        with tempfile.TemporaryDirectory() as d:
            blocker = os.path.join(d, "a_file")
            open(
                blocker, "w"
            ).close()  # its "directory" is a file: unwritable, even as root
            clock = Clock()
            with contextlib.redirect_stdout(io.StringIO()) as log:
                b = Bridge(
                    load_vocab(),
                    {"mock": False},
                    clock=clock,
                    wall=lambda: 0.0,
                    state_path=os.path.join(blocker, "state.json"),
                )
                b.on_walk(make_walk(50))
                epoch = b.epoch
                clock.t += 0.1
                restarted = decoded(b.on_walk(make_walk(2)))  # the service restarted
                clock.t += 0.1
                alerts = [
                    d
                    for c, d in decoded(
                        b.on_walk(make_walk(3, "Wall ahead", "Wall ahead"))
                    )
                    if d["kind"] == "alert"
                ]
        self.assertIn("service_restarted", [d.get("event") for c, d in restarted])
        self.assertEqual(b.epoch, epoch + 1)
        self.assertEqual((alerts[0]["epoch"], alerts[0]["cue_seq"]), (epoch + 1, 3))
        self.assertEqual(log.getvalue().count("cannot save"), 1)

    def test_unusable_state_file_counts_as_absent(self):
        bad = [
            "[1,2]",
            "null",
            '{"epoch":"x"}',
            '{"epoch":null}',
            "",
            '{"epoch":',
            '{"device_id":"0123456789abcdef","epoch":NaN}',
            '{"device_id":"0123456789abcdef","epoch":1e400}',
            '{"device_id":"' + "d" * 600 + '","epoch":3}',
            '{"device_id":5,"epoch":3}',
        ]
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "state.json")
            for content in bad:
                with open(path, "w") as fh:
                    fh.write(content)
                with contextlib.redirect_stdout(io.StringIO()) as log:
                    b = Bridge(
                        load_vocab(),
                        {"mock": False},
                        clock=Clock(),
                        wall=lambda: 0.0,
                        state_path=path,
                    )
                self.assertEqual(b.epoch, 1, content)
                self.assertRegex(b.device_id, "^[0-9a-f]{16}$")
                self.assertIn("ignored", log.getvalue())
                info = P.encode(
                    P.T_PROTOCOL_INFO, P.json_bytes(b.protocol_info()), 0, 517
                )
                self.assertEqual(len(info), 1)  # _read_info serves frame 0 only
            with open(path, "w") as fh:  # ... and a good one is still kept
                json.dump({"device_id": "0123456789abcdef", "epoch": 41}, fh)
            b = Bridge(
                load_vocab(),
                {"mock": False},
                clock=Clock(),
                wall=lambda: 0.0,
                state_path=path,
            )
            self.assertEqual((b.device_id, b.epoch), ("0123456789abcdef", 42))

    def test_damaged_state_file_keeps_a_valid_device_id(self):
        cases = [
            ('{"device_id":"0123456789abcdef","epoch":NaN}', 1),
            ('{"device_id": "0123456789abcdef", "epoch": 41', 42),  # cut off mid-write
            ('\udcff{"device_id":"0123456789abcdef","epoch":7}', 8),  # a bad byte
        ]
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "state.json")
            for content, epoch in cases:
                with open(path, "wb") as fh:
                    fh.write(content.encode("utf-8", "surrogateescape"))
                with contextlib.redirect_stdout(io.StringIO()) as log:
                    b = Bridge(
                        load_vocab(),
                        {"mock": False},
                        clock=Clock(),
                        wall=lambda: 0.0,
                        state_path=path,
                    )
                self.assertEqual((b.device_id, b.epoch), ("0123456789abcdef", epoch))
                self.assertIn("kept its device_id", log.getvalue())

    @unittest.skipIf(os.geteuid() == 0, "root reads a mode-000 file")
    def test_unreadable_state_file_is_left_untouched(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "state.json")
            with open(path, "w") as fh:
                json.dump({"device_id": "0123456789abcdef", "epoch": 41}, fh)
            os.chmod(path, 0)
            try:
                with contextlib.redirect_stdout(io.StringIO()) as log:
                    b = Bridge(
                        load_vocab(),
                        {"mock": False},
                        clock=Clock(),
                        wall=lambda: 0.0,
                        state_path=path,
                    )
            finally:
                os.chmod(path, 0o600)
            self.assertIn("left untouched", log.getvalue())
            with open(path) as fh:  # the next start can still read the ID
                self.assertEqual(json.load(fh)["device_id"], "0123456789abcdef")
            self.assertNotEqual(b.device_id, "0123456789abcdef")

    def test_bad_walk_values_are_unknown_not_fatal(self):
        """Each of these raised from on_walk (and most from every later tick)."""
        cases = [
            ("loop", "age_s", 1e306),
            ("loop", "age_s", float("inf")),
            ("loop", "age_s", "abc"),
            ("loop", "age_s", 10**400),
            ("path", "front_ft", float("inf")),
            ("path", "front_ft", 10**400),
            ("notify", "instruction", "Walk \ud800"),
            ("notify", "instruction", "\U0001f6b6" * 1300),
            ("notify", "pi_voice", [1]),
            (None, "loop", [1]),
            (None, "path", 5),
            (None, "notify", "oops"),
        ]
        for where, key, value in cases:
            b, clock = make_bridge()
            b.on_walk(make_walk(5))
            clock.t += 1.5
            snap = make_walk(
                6, "Walk straight", "Walk straight", speak=True, front_ft=10.0
            )
            (snap if where is None else snap[where])[key] = value
            outs = b.on_walk(snap)
            clock.t += 1.5
            outs += b.tick() + b.on_unreachable("refused")
            assert_sendable(outs)
            alerts = [
                d for c, d in decoded(outs) if d["kind"] in ("alert", "alert_text")
            ]
            if key != "notify":  # without notify there is no cue step
                self.assertEqual(alerts[0]["cue_seq"], 6, (where, key))
            for text in (d["text"] for d in alerts if d["kind"] == "alert_text"):
                self.assertLessEqual(len(text.encode("utf-8")), 200)
        b, _ = make_bridge()
        for snap in (None, [], "x", 5):
            assert_sendable(b.on_walk(snap) + b.tick(force=True))

    def test_wrong_typed_health_fields_do_not_poison_later_calls(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(1))
        good = (
            make_health(),
            {"front_glass_ok": True},
            {"soc_c": 50.0, "throttled": 0},
        )
        b.on_health(*good)
        bad = [
            ({**make_health(), "sensors": ["lidar_ok"]}, {}, {}),
            ({**make_health(), "warnings": 5}, {}, {}),
            ({**make_health(), "warnings": {"a": 1}}, {}, {}),
            ({**make_health(), "status": []}, {}, {}),
            ({**make_health(guiding=False), "guidance_loop": "x"}, {}, {}),
            (make_health(), [1], {"throttled": "0x50005", "soc_c": "hot"}),
            (make_health(), {}, {"throttled": -1, "soc_c": float("inf")}),
            ([1], None, None),
        ]
        for h, sh, sy in bad:
            clock.t += 1
            assert_sendable(b.on_health(h, sh, sy) + b.tick(force=True))
            clock.t += 1
            assert_sendable(b.on_health(*good) + b.tick(force=True))
            st = P.unpack_state(
                b.state_value()
            )  # the next good snapshot is whole again
            self.assertEqual(st["status"], "ok", h)
            self.assertTrue(
                st["health"]["lidar_ok"] and st["health"]["front_glass_ok"], h
            )
            self.assertFalse(
                st["health"]["thermal_warn"] or st["health"]["undervoltage"], h
            )

    def test_long_strings_are_capped(self):
        b, clock = make_bridge()
        w = make_walk(1)
        w["notify"]["pi_voice"]["error"] = "y" * 5000
        b.on_walk(w)
        b.on_health(make_health())
        clock.t += 1
        h = make_health(guiding=False)
        h["guidance_loop"]["error"] = "KeyError: " + "x" * 5000
        outs = b.on_health(h)
        assert_sendable(outs)
        self.assertIn("guidance_stalled", [d.get("event") for c, d in decoded(outs)])
        st = self.cmd(b, {"v": 1, "id": 1, "cmd": "get_status"})[0]["result"]
        self.assertLessEqual(
            max(len(st["loop"]["error"]), len(st["voice"]["error"])), 200
        )
        for warning in (
            "—" * 400,
            "\U0001f6b6" * 400,
            "\x01" * 400,
        ):  # 3, 4 and 6 bytes a character
            clock.t += 1
            assert_sendable(b.on_health(make_health(warnings=[warning] * 20)))

    def test_echoed_and_stored_app_values_are_bounded(self):
        b, _ = make_bridge()
        for raw in (
            P.json_bytes({"v": 1, "id": 2, "cmd": "ping", "args": {"t": "x" * 3600}}),
            b'{"v":1,"id":2,"cmd":"ping","args":{"t":NaN}}',
        ):
            self.assertEqual(self.cmd(b, raw)[0]["error"]["code"], "BAD_ARGS")
        self.assertEqual(
            self.cmd(b, {"v": 1, "id": 3, "cmd": "ping", "args": {"t": 1791504000123}})[
                0
            ]["result"]["t"],
            1791504000123,
        )
        for args in (
            {"app_latency_ms": "L" * 900},
            {"app_latency_ms": -1},
            {"app_latency_ms": True},
            {"epoch": 10**3000},
            {"cue_seq": -1},
        ):
            ack = {"epoch": 1, "cue_seq": 2, "outcome": "spoken", **args}
            r = self.cmd(b, {"v": 1, "id": 4, "cmd": "alert_ack", "args": ack})[0]
            self.assertEqual(r["error"]["code"], "BAD_ARGS", args)
        for i in range(5):
            self.cmd(
                b,
                {
                    "v": 1,
                    "id": 5,
                    "cmd": "alert_ack",
                    "args": {
                        "epoch": 1,
                        "cue_seq": i,
                        "outcome": "spoken",
                        "app_latency_ms": 120,
                    },
                },
            )
        diag = self.cmd(b, {"v": 1, "id": 6, "cmd": "get_diagnostics"})[0]["result"]
        self.assertEqual([a["app_latency_ms"] for a in diag["acks"]], [120] * 5)

    def test_speech_log_shrinks_to_fit_the_mtu(self):
        """20 entries can pass 255 fragments at MTU 23: the reply was lost without a word."""
        longest = max(load_vocab()["by_text"], key=len)
        line = {
            "time": "23:59:59",
            "source": "simple-guidance",
            "level": "advisory",
            "text": longest,
            "via": "pi+phone",
            "outcome": "interrupted",
        }

        class LongLog(FakeService):
            def speech_log(self, limit):
                return {
                    "lines": [dict(line, cue_seq=100000 + i) for i in range(50)][
                        -limit:
                    ]
                }

        b, _ = make_bridge()
        b.service = LongLog()
        for mtu, all_fit in ((23, False), (185, True)):
            b.set_mtu(link=mtu)
            entries = self.cmd(
                b, {"v": 1, "id": 1, "cmd": "get_speech_log", "args": {"limit": 20}}
            )[0]["result"]["entries"]
            self.assertEqual(
                entries[-1]["cue_seq"], 100049
            )  # the newest is kept, the oldest go
            self.assertEqual(len(entries) == 20, all_fit, mtu)

    def test_a_reply_too_large_for_the_link_is_answered_too_large(self):
        class Chatty(FakeService):
            def mark(self, kind, note):
                return {"ok": True, "detail": "z" * 4000}

        b, _ = make_bridge()
        b.service = Chatty()
        r = self.cmd(b, {"v": 1, "id": 77, "cmd": "mark", "args": {"kind": "note"}})[0]
        self.assertEqual(
            (r["id"], r["ok"], r["error"]["code"]), (77, False, "TOO_LARGE")
        )
        body = P.json_bytes({"v": 1, "id": 78, "ok": True, "result": {"x": "y" * 4000}})
        r = strict_json(
            b.too_large(body, "the MTU fell")
        )  # what Server.send falls back to
        self.assertEqual((r["id"], r["error"]["code"]), (78, "TOO_LARGE"))

    def test_mtu_is_the_smaller_of_the_link_and_the_apps_claim(self):
        b, _ = make_bridge()
        b.on_connect()
        b.set_mtu(link=185)  # BlueZ's write option "mtu"
        r = self.cmd(
            b,
            {"v": 1, "id": 1, "cmd": "hello", "args": {"protocol": 1, "att_mtu": 517}},
        )[0]
        self.assertEqual((r["result"]["att_mtu"], b.att_mtu), (185, 185))
        self.cmd(
            b,
            {"v": 1, "id": 2, "cmd": "hello", "args": {"protocol": 1, "att_mtu": 100}},
        )
        self.assertEqual(b.att_mtu, 100)  # a smaller claim is believed
        b.on_disconnect()
        self.assertEqual(b.att_mtu, P.MIN_ATT_MTU)

    def test_command_error_path_cannot_fail(self):
        """The error RESPONSE itself raised UnicodeEncodeError and killed the command thread."""
        b, _ = make_bridge()
        r = self.cmd(b, b'{"v":"\\ud800","id":7,"cmd":"ping"}')
        self.assertEqual(
            [(x["id"], x["error"]["code"]) for x in r], [(7, "PROTOCOL_MISMATCH")]
        )
        w = make_walk(1, "Walk straight")
        w["path"]["front_ft"] = float("nan")  # NaN in a reply: an error, never bad JSON
        b.on_walk(w)
        r = self.cmd(b, {"v": 1, "id": 8, "cmd": "get_current"})[0]
        self.assertEqual((r["id"], r["ok"]), (8, False))
        r = self.cmd(b, {"v": 1, "id": 2**40, "cmd": "ping"})[0]
        self.assertEqual((r["id"], r["error"]["code"]), (None, "BAD_ARGS"))


class ScenarioTests(unittest.TestCase):
    def run_scenario(self, name, seconds=30.0):
        clock = Clock()
        src = MockSource(name, clock=clock)
        b, _ = make_bridge(clock)
        said, events = [], []
        next_health = 0.0
        while clock.t - 1000.0 < seconds:
            outs = []
            try:
                outs += b.on_walk(src.walk())
                if clock.t >= next_health:
                    next_health = clock.t + 1.0
                    outs += b.on_health(src.health(), src.system_health(), src.system())
            except ConnectionError as exc:
                outs += b.on_unreachable(str(exc))
            outs += b.tick()
            for c, d in decoded(outs):
                if d["kind"] == "alert_text":
                    said.append(d["text"])
                elif d["kind"] == "event":
                    events.append(d["event"])
            clock.t += 0.1
        return said, events

    def test_every_scenario_runs(self):
        for name in SCENARIOS:
            said, events = self.run_scenario(name)
            self.assertIn("device_ready", events, name)
            expected = [s["say"] for s in SCENARIOS[name] if "say" in s][
                1:
            ]  # first one is learned
            self.assertEqual(said[: len(expected)], expected, name)

    def test_lidar_loss_is_down_not_stalled(self):
        """As main.health_check: losing the LiDAR is status down + sensor_degraded; the loop
        keeps ticking, so guidance_stalled must NOT fire (it means the loop itself stopped)."""
        said, events = self.run_scenario("sensor_failure")
        self.assertIn("sensor_degraded", events)
        self.assertNotIn("guidance_stalled", events)
        self.assertIn("Guidance stopped, use your cane", said)

    def test_crash_scenario_reports_and_recovers(self):
        said, events = self.run_scenario("service_crash")
        self.assertIn("service_unreachable", events)
        self.assertIn("service_restarted", events)
        self.assertGreaterEqual(events.count("device_ready"), 2)


@unittest.skipUnless(
    __import__("importlib").util.find_spec("jsonschema"), "jsonschema not installed"
)
class SchemaTests(unittest.TestCase):
    """Every JSON message the bridge makes validates against contract/schema_v1.json."""

    def test_messages_match_the_schema(self):
        import jsonschema

        with open(
            os.path.join(HERE, "contract", "schema_v1.json"), encoding="utf-8"
        ) as fh:
            schema = json.load(fh)

        def check(kind, obj):
            jsonschema.validate(obj, {**schema, "$ref": f"#/$defs/{kind}"})

        clock = Clock()
        b, _ = make_bridge(clock)
        check("protocol_info", b.protocol_info())
        seen = set()
        for name in SCENARIOS:
            src = MockSource(name, clock=clock)
            for _ in range(300):
                try:
                    outs = b.on_walk(src.walk()) + b.on_health(
                        src.health(), src.system_health(), src.system()
                    )
                except ConnectionError as exc:
                    outs = b.on_unreachable(str(exc))
                for _c, t, payload in outs:
                    if t == P.T_EVENT:
                        obj = P.parse_json(payload)
                        check("event", obj)
                        seen.add(obj["event"])
                clock.t += 0.1
        for cmd in (
            "hello",
            "ping",
            "get_status",
            "get_current",
            "get_surroundings",
            "get_diagnostics",
            "session_start",
            "session_stop",
            "launch",
            "set_device_settings",
        ):
            msg = {"v": 1, "id": 7, "cmd": cmd}
            if cmd != "launch":
                check("command", msg)
            for _c, t, payload in b.on_command(P.json_bytes(msg)):
                check(
                    "response" if t == P.T_RESPONSE else "event", P.parse_json(payload)
                )
        self.assertTrue(
            {"device_ready", "sensor_degraded", "service_unreachable"} <= seen
        )


@unittest.skipUnless(
    __import__("importlib").util.find_spec("dbus"), "dbus-python not installed"
)
class GattImportTest(unittest.TestCase):
    def test_imports(self):
        from ble_bridge import gatt_server

        for level in gatt_server.SECURITY.values():
            # EVENTS has no read callback: it must not advertise a read.
            self.assertFalse(any("read" in f for f in level["notify_only"]))
        self.assertEqual(
            gatt_server.SECURITY["encrypt"]["notify_only"], ["encrypt-notify"]
        )
        self.assertLessEqual(len("Shappe-M".encode()), gatt_server.MAX_NAME_BYTES)

    def test_connection_tracking(self):
        # A phone new to BlueZ arrives in InterfacesAdded already connected, with no
        # PropertiesChanged after it: it must still count as exactly one connection.
        import contextlib
        import io
        import dbus
        from ble_bridge import gatt_server

        class Bus:
            def get_object(self, *_):
                raise dbus.exceptions.DBusException("device gone")

        class Hooks:
            connects = disconnects = 0

            def on_connect(self):
                self.connects += 1

            def on_disconnect(self):
                self.disconnects += 1

        s = gatt_server.Server.__new__(gatt_server.Server)
        s.bus, s.bridge, s.connected = Bus(), Hooks(), set()
        s.seq = {"alert": 5, "state": 5, "events": 5}
        dev, iface = "/org/bluez/hci0/dev_58_E1_DD_9F_94_B8", gatt_server.DEVICE_IFACE
        with contextlib.redirect_stdout(io.StringIO()) as out:
            s.on_interfaces_added(dev, {iface: {"Connected": True}})
            s.on_device_props(
                iface, {"Connected": True}, [], path=dev
            )  # repeat: ignored
            s.on_device_props(iface, {"Connected": False}, [], path=dev)
            s.on_interfaces_removed(dev, [iface])  # already counted
        self.assertEqual((s.bridge.connects, s.bridge.disconnects), (1, 1))
        self.assertEqual(s.seq, {"alert": 0, "state": 0, "events": 0})
        self.assertEqual(s.connected, set())
        self.assertIn(f"connected: {dev} (1 connected)", out.getvalue())

    def test_send_never_drops_the_rest_of_a_batch(self):
        """A message too large for the MTU raised inside send() and lost the rest, STATE too."""
        from ble_bridge import gatt_server

        class Char:
            notifying = True

            def __init__(self):
                self.frames = []

            def notify(self, value):
                self.frames.append(bytes(value))

        b, _ = make_bridge()
        s = gatt_server.Server.__new__(gatt_server.Server)
        s.bridge, s.seq = b, {"alert": 0, "state": 0, "events": 0}
        s.chars = {name: Char() for name in s.seq}
        reply = P.json_bytes(
            {"v": 1, "id": 9, "ok": True, "result": {"x": "y" * 3600}}
        )  # > 3570 B
        event = P.json_bytes(
            {"v": 1, "event": "health_changed", "data": {"x": "z" * 5000}}
        )
        with contextlib.redirect_stdout(io.StringIO()):
            s.send(
                [
                    ("events", P.T_RESPONSE, reply),
                    ("events", P.T_EVENT, event),
                    ("state", P.T_STATE, b.state_value()),
                ]
            )
        self.assertEqual(len(s.chars["state"].frames), 1)  # STATE still went out
        r = P.Reassembler()
        msgs = [m for m in (r.feed(f, 0.0) for f in s.chars["events"].frames) if m]
        self.assertEqual(
            [
                (m["id"], m["error"]["code"])
                for m in (P.parse_json(p) for _t, _s, p in msgs)
            ],
            [(9, "TOO_LARGE")],
        )  # the app still gets an answer

    def test_poll_loop_survives_any_exception(self):
        """loop.age_s = 1e306 ended the poll thread for good; the process lived on, silent."""
        from ble_bridge import gatt_server

        b, _ = make_bridge()
        logged = []
        b.log = lambda key, message: logged.append(message)
        bad = make_walk(6, "Walk straight", "Walk straight", speak=True)
        bad["loop"]["age_s"] = 1e306
        walks = iter(
            [make_walk(5), bad]
            + [make_walk(7 + i, "Wall ahead", "Wall ahead") for i in range(500)]
        )

        class Source:
            def walk(self):
                return next(walks)

            def health(self):
                return make_health()

            def system_health(self):
                return {"front_glass_ok": True}

            def system(self):
                return {"soc_c": 50.0, "throttled": 0}

        real_tick, failed = b.tick, []

        def tick(force=False):  # a bug anywhere in the loop, once
            if not failed:
                failed.append(True)
                raise RuntimeError("injected")
            return real_tick(force)

        b.tick = tick
        s = gatt_server.Server.__new__(gatt_server.Server)
        s.bridge, s.source, s.stop = b, Source(), threading.Event()
        sent = []
        s.post = sent.extend
        th = threading.Thread(target=s.poll_loop, name="ble-poll-test", daemon=True)
        th.start()
        deadline = time.monotonic() + 5.0
        while (
            time.monotonic() < deadline
            and sum(t == P.T_ALERT for _c, t, _p in sent) < 3
        ):
            time.sleep(0.05)
        alive = th.is_alive()
        s.stop.set()
        th.join(2.0)
        self.assertTrue(alive)
        alerts = [P.unpack_alert(p) for _c, t, p in sent if t == P.T_ALERT]
        self.assertEqual([a["cue_seq"] for a in alerts[:3]], [6, 7, 8])
        self.assertEqual(alerts[0]["age_ms"], P.AGE_MAX_MS)  # the huge age, saturated
        self.assertTrue(any("RuntimeError: injected" in m for m in logged))


if __name__ == "__main__":
    unittest.main()
