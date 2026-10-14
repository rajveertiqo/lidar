"""Desk tests for the BLE bridge prototype — no Bluetooth, no sensors, no network.

    venv/bin/python -m unittest ble_bridge.test_ble_bridge -v
"""

import json
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from ble_bridge import protocol as P                       # noqa: E402
from ble_bridge.core import Bridge, load_vocab, STATE_HEARTBEAT_S   # noqa: E402
from ble_bridge.sources import MockSource, SCENARIOS, make_health, make_walk   # noqa: E402


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def frames_of(outs, mtu=23):
    return [(char, P.encode(t, payload, i, mtu)) for i, (char, t, payload) in enumerate(outs)]


def decoded(outs):
    return [(char, P.decode_message(t, payload)) for char, t, payload in outs]


class FakeService:
    def __init__(self):
        self.marks = []

    def speech_log(self, limit):
        return {"lines": [{"time": "12:00:00", "source": "simple-guidance", "level": "urgent",
                           "text": "Blocked by glass", "cue_seq": 3, "via": "pi",
                           "outcome": "completed", "detail": "x"}]}

    def mark(self, kind, note):
        self.marks.append((kind, note))
        return {"ok": True, "kind": kind, "last_said": "Blocked by glass"}


def make_bridge(clock=None, mock=False):
    clock = clock or Clock()
    return Bridge(load_vocab(), {"model": "test", "software": "test", "mock": mock},
                  service=FakeService(), clock=clock, wall=lambda: 1791504000.0), clock


class ProtocolTests(unittest.TestCase):
    def test_records_fit_the_smallest_mtu(self):
        rec = P.pack_alert(1, 2, 50, 3, 2, P.AF_ANNOUNCE, 3.6)
        frames = P.encode(P.T_ALERT, rec, 0, 23)
        self.assertEqual(len(frames), 1)
        self.assertEqual(len(frames[0]), 20)              # = ATT_MTU 23 - 3
        st = P.encode(P.T_STATE, P.pack_state(1, 2, 50, 3, 2, 0, 0xFFFF), 0, 23)
        self.assertEqual(len(st[0]), 20)

    def test_alert_round_trip(self):
        rec = P.pack_alert(7, 123456, 87.4, 5, 2, P.AF_ANNOUNCE | P.AF_URGENT, 3.64)
        a = P.unpack_alert(rec)
        self.assertEqual((a["epoch"], a["cue_seq"], a["age_ms"], a["phrase_id"], a["level"]),
                         (7, 123456, 87, 5, 2))
        self.assertTrue(a["urgent"] and a["announce"] and not a["replay"])
        self.assertAlmostEqual(a["front_ft"], 3.6)

    def test_unknown_values(self):
        a = P.unpack_alert(P.pack_alert(1, 1, None, P.PHRASE_UNLISTED, -1, 0, float("nan")))
        self.assertIsNone(a["age_ms"])
        self.assertIsNone(a["front_ft"])
        self.assertEqual(a["level"], -1)
        self.assertEqual(P.unpack_alert(P.pack_alert(1, 1, 10 ** 9, 1, 0, 0, -3))["age_ms"], P.AGE_MAX_MS)

    def test_fragmentation_round_trip_at_every_mtu(self):
        body = P.json_bytes({"v": 1, "event": "health_changed", "data": {"warnings": ["x" * 150] * 4}})
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
            r.feed(frames[2], 0.1)                        # fragment 1 lost
        r2 = P.Reassembler(timeout_s=2.0)
        r2.feed(frames[0], 0.0)
        r2.feed(frames[1], 0.5)
        with self.assertRaises(P.ProtocolError):
            r2.feed(frames[2], 5.0)                       # the partial message expired
        self.assertEqual(r2.dropped, 1)

    def test_header_validation(self):
        with self.assertRaises(P.ProtocolError) as cm:
            P.parse_header(b"\x02\x10\x00\x00\x00\x01")
        self.assertEqual(cm.exception.code, "PROTOCOL_MISMATCH")
        with self.assertRaises(P.ProtocolError):
            P.parse_header(b"\x01\x99\x00\x00\x00\x01")
        with self.assertRaises(P.ProtocolError):
            P.parse_header(b"\x01\x10\x00\x00\x01\x01")    # index 1 of count 1
        with self.assertRaises(P.ProtocolError):
            P.parse_header(b"\x01\x10")

    def test_sequence_arithmetic(self):
        self.assertTrue(P.seq16_newer(0, 65535))
        self.assertFalse(P.seq16_newer(65535, 0))
        self.assertTrue(P.cue_newer(2, 1, 1, 500))         # new epoch wins
        self.assertFalse(P.cue_newer(1, 4, 1, 4))          # duplicate
        self.assertTrue(P.cue_newer(1, 5, None, None))

    def test_too_large(self):
        with self.assertRaises(P.ProtocolError):
            P.encode(P.T_EVENT, b"z" * (P.MAX_MESSAGE_BYTES + 1), 0, 517)


class VocabTests(unittest.TestCase):
    def test_registry_matches_the_device(self):
        from utils import simple_guidance as sg
        vocab = load_vocab()
        self.assertEqual(set(vocab["by_text"]), set(sg.ALLOWED),
                         "registry out of date: python -m ble_bridge.make_vocab")
        for text, entry in vocab["by_text"].items():
            self.assertEqual(entry["level"], sg._LEVEL[text], text)
            self.assertEqual(entry["action"], sg._ACTION.get(text, "unknown"), text)
            self.assertNotIn(entry["id"], (P.PHRASE_NONE, P.PHRASE_UNLISTED))

    def test_ids_unique(self):
        with open(os.path.join(HERE, "contract", "vocab_v1.json"), encoding="utf-8") as fh:
            reg = json.load(fh)
        ids = [p["id"] for p in reg["phrases"]]
        self.assertEqual(len(ids), len(set(ids)))


class BridgeTests(unittest.TestCase):
    def test_first_snapshot_only_learns(self):
        b, _ = make_bridge()
        outs = b.on_walk(make_walk(5, "Walk straight", "Walk straight", speak=True))
        self.assertFalse([o for o in outs if o[0] == "alert"])

    def test_cue_step_makes_one_alert_and_its_text(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(5, "Walk straight", "Walk straight"))
        clock.t += 0.1
        outs = b.on_walk(make_walk(6, "Blocked by glass", "Blocked by glass", speak=True,
                                   urgent=True, front_ft=3.6))
        alerts = [d for c, d in decoded(outs) if c == "alert"]
        self.assertEqual([a["kind"] for a in alerts], ["alert", "alert_text"])
        a = alerts[0]
        self.assertEqual(a["phrase_id"], b.vocab["by_text"]["Blocked by glass"]["id"])
        self.assertTrue(a["urgent"])
        self.assertEqual(a["level"], 2)
        self.assertAlmostEqual(a["front_ft"], 3.6)
        self.assertEqual(alerts[1]["text"], "Blocked by glass")
        clock.t += 0.1                                     # the same cue again: nothing new
        again = b.on_walk(make_walk(6, "Blocked by glass", "Blocked by glass", front_ft=3.6))
        self.assertFalse([o for o in again if o[0] == "alert"])

    def test_missed_admitting_tick_still_alerts(self):
        """The poll can miss the one tick where speak=True; the cue step is what counts."""
        b, clock = make_bridge()
        b.on_walk(make_walk(1, "Walk straight", "Walk straight"))
        clock.t += 0.3
        outs = b.on_walk(make_walk(2, "Turn left", "Turn left", speak=False))
        alert = [d for c, d in decoded(outs) if d["kind"] == "alert"][0]
        self.assertTrue(alert["urgent"])                   # from the registry, not the tick

    def test_device_voice_and_audio_flags(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(1, audio_enabled=False))
        clock.t += 0.1
        a = [d for c, d in decoded(b.on_walk(make_walk(2, "Wall ahead", "Wall ahead", speak=True,
                                                      audio_enabled=False)))
             if d["kind"] == "alert"][0]
        self.assertFalse(a["audio_enabled"])
        self.assertFalse(a["device_voice"])
        clock.t += 0.1
        a = [d for c, d in decoded(b.on_walk(make_walk(3, "Wall ahead", "Wall ahead", speak=True,
                                                      device_voice_usable=True, accepted_seq=3)))
             if d["kind"] == "alert"][0]
        self.assertTrue(a["device_voice"] and a["audio_enabled"])

    def test_unlisted_sentence_uses_text(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(1))
        clock.t += 0.1
        outs = decoded(b.on_walk(make_walk(2, "", "Ready.", speak=True)))
        alert = [d for c, d in outs if d["kind"] == "alert"][0]
        self.assertEqual(alert["phrase_id"], P.PHRASE_UNLISTED)
        self.assertEqual([d for c, d in outs if d["kind"] == "alert_text"][0]["text"], "Ready.")

    def test_service_restart_bumps_epoch(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(40))
        epoch = b.epoch
        clock.t += 0.1
        outs = decoded(b.on_walk(make_walk(0)))
        self.assertEqual(b.epoch, epoch + 1)
        self.assertIn("service_restarted", [d.get("event") for c, d in outs])
        clock.t += 0.1
        alert = [d for c, d in decoded(b.on_walk(make_walk(1, "Walk straight", "Walk straight")))
                 if d["kind"] == "alert"][0]
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
        self.assertFalse([o for o in b.tick() if o[0] == "state"])          # unchanged, too soon
        clock.t += 0.1
        self.assertFalse([o for o in b.on_walk(make_walk(1, "Wall ahead")) if o[0] == "state"])
        clock.t += 0.1                                                     # changed, >= 0.2 s
        st = [d for c, d in decoded(b.tick()) if c == "state"]
        self.assertEqual(st[0]["phrase_id"], b.vocab["by_text"]["Wall ahead"]["id"])
        clock.t += STATE_HEARTBEAT_S + 0.01
        self.assertTrue([o for o in b.tick() if o[0] == "state"])          # heartbeat

    def test_health_events(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(1))
        evs = [d["event"] for c, d in decoded(b.on_health(make_health(), {"front_glass_ok": True},
                                                          {"soc_c": 50.0, "throttled": 0}))
               if d["kind"] == "event"]
        self.assertIn("device_ready", evs)
        clock.t += 1
        evs = [d for c, d in decoded(b.on_health(
            make_health("degraded", radar=False, warnings=["Radar offline — glass may not be detected"]),
            {"front_glass_ok": False}, {"soc_c": 83.0, "throttled": 0x5}))
               if d["kind"] == "event"]
        names = [e["event"] for e in evs]
        self.assertIn("sensor_degraded", names)
        self.assertEqual({e["data"]["sensor"] for e in evs if e["event"] == "sensor_degraded"},
                         {"radar", "front_glass"})
        for name in ("thermal_warning", "power_warning", "health_changed"):
            self.assertIn(name, names)
        self.assertNotIn("device_ready", names)            # still ready: not announced again
        state = [d for c, d in decoded(b.tick() or [("state", P.T_STATE, b.state_value())])
                 if c == "state"][-1]
        self.assertEqual(state["status"], "degraded")
        self.assertTrue(state["health"]["thermal_warn"] and state["health"]["undervoltage"])

    def test_unreachable_is_reported_once_and_clears(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(1))
        b.on_health(make_health())
        clock.t += 1
        first = [d["event"] for c, d in decoded(b.on_unreachable("connection refused"))
                 if d["kind"] == "event"]
        clock.t += 1
        second = [d for c, d in decoded(b.on_unreachable("connection refused")) if d["kind"] == "event"]
        self.assertEqual(first, ["service_unreachable"])
        self.assertEqual(second, [])
        st = P.unpack_state(b.state_value())
        self.assertEqual(st["status"], "unreachable")
        self.assertFalse(any(st["health"].values()))
        clock.t += 1
        b.on_walk(make_walk(1))
        evs = [d["event"] for c, d in decoded(b.on_health(make_health())) if d["kind"] == "event"]
        self.assertIn("device_ready", evs)

    def test_mock_identity_is_always_marked_simulated(self):
        b, _ = make_bridge(mock=True)
        b.on_walk(make_walk(1))
        b.on_health(make_health())
        self.assertTrue(P.unpack_state(b.state_value())["health"]["simulated_input"])
        self.assertTrue(b.protocol_info()["mock"])

    def test_protocol_info_fits_one_long_read(self):
        """Served as ONE frame by a GATT read: must fit 512 B even with long real strings."""
        long_identity = {"model": "Shappe Guide (Raspberry Pi 5)" * 3, "mock": False,
                         "software": "guide e209c48+dirty; ble-bridge 0.1; protocol 1" * 3}
        b = Bridge(load_vocab(), long_identity, clock=Clock(), wall=lambda: 0.0)
        frame = P.encode(P.T_PROTOCOL_INFO, P.json_bytes(b.protocol_info()), 0, 517)
        self.assertEqual(len(frame), 1)
        self.assertLessEqual(len(frame[0]), 480)          # margin for future fields
        hello = P.parse_json(b.on_command(P.json_bytes({"v": 1, "id": 1, "cmd": "hello"}))[0][2])
        self.assertEqual(hello["result"]["model"], long_identity["model"])   # strings live here

    def test_new_subscriber_gets_state_at_once(self):
        b, clock = make_bridge()
        b.on_walk(make_walk(1, "Walk straight"))
        clock.t += 0.05
        self.assertFalse(b.tick())                         # timers say nothing is due ...
        self.assertEqual([o[0] for o in b.tick(force=True)], ["state"])   # ... a subscriber still gets one


class CommandTests(unittest.TestCase):
    def cmd(self, b, obj):
        raw = obj if isinstance(obj, bytes) else P.json_bytes(obj)
        outs = b.on_command(raw)
        return [d for c, d in decoded(outs)]

    def test_hello_sets_mtu(self):
        b, _ = make_bridge()
        r = self.cmd(b, {"v": 1, "id": 1, "cmd": "hello", "args": {"protocol": 1, "att_mtu": 185}})[0]
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
            ({"v": 1, "id": 3, "cmd": "set_device_settings", "args": {"voice_rate": 200}}, "NOT_SUPPORTED"),
            ({"v": 1, "id": 4, "cmd": "alert_ack", "args": {"outcome": "loud"}}, "BAD_ARGS"),
            ({"v": 1, "id": 5, "cmd": "get_speech_log", "args": {"limit": 99}}, "BAD_ARGS"),
            ({"v": 1, "id": 6, "cmd": "hello", "args": {"att_mtu": 9000}}, "BAD_ARGS"),
        ]
        for msg, code in cases:
            r = self.cmd(b, msg)[0]
            self.assertFalse(r["ok"], msg)
            self.assertEqual(r["error"]["code"], code, msg)

    def test_rate_limit(self):
        b, clock = make_bridge()
        codes = [self.cmd(b, {"v": 1, "id": i, "cmd": "ping"})[0].get("error", {}).get("code")
                 for i in range(12)]
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
        self.assertEqual(again[0]["result"]["id"], sid)    # idempotent: resumes, never forks
        self.assertEqual(len(again), 1)                    # ... and announces nothing new
        clock.t += 61
        stop = self.cmd(b, {"v": 1, "id": 3, "cmd": "session_stop"})
        self.assertEqual(stop[0]["result"]["duration_s"], 61.0)
        self.assertEqual(stop[1]["event"], "session_stopped")
        self.assertEqual(self.cmd(b, {"v": 1, "id": 4, "cmd": "session_stop"})[0]["error"]["code"],
                         "BAD_ARGS")

    def test_queries(self):
        b, _ = make_bridge()
        b.on_walk(make_walk(1, "Person ahead", front_ft=9.0, obstacles=[
            {"object": "person", "clock": 12, "sector": "front", "distance_ft": 9.0,
             "urgency": "clear", "source": "both", "shape": None}]))
        b.on_health(make_health())
        cur = self.cmd(b, {"v": 1, "id": 1, "cmd": "get_current"})[0]["result"]
        self.assertEqual(cur["instruction"], "Person ahead")
        sur = self.cmd(b, {"v": 1, "id": 2, "cmd": "get_surroundings"})[0]["result"]
        self.assertEqual(sur["items"][0]["object"], "person")
        st = self.cmd(b, {"v": 1, "id": 3, "cmd": "get_status"})[0]["result"]
        self.assertEqual(st["status"], "ok")
        self.assertIsNone(st["battery"])
        log = self.cmd(b, {"v": 1, "id": 4, "cmd": "get_speech_log", "args": {"limit": 5}})[0]
        self.assertEqual(log["result"]["entries"][0]["text"], "Blocked by glass")
        mark = self.cmd(b, {"v": 1, "id": 5, "cmd": "mark", "args": {"kind": "missed"}})[0]
        self.assertTrue(mark["ok"])
        self.assertEqual(b.service.marks, [("missed", "")])
        ack = self.cmd(b, {"v": 1, "id": 6, "cmd": "alert_ack",
                           "args": {"epoch": 1, "cue_seq": 2, "outcome": "spoken"}})[0]
        self.assertTrue(ack["ok"])
        diag = self.cmd(b, {"v": 1, "id": 7, "cmd": "get_diagnostics"})[0]["result"]
        self.assertEqual(diag["acks"][-1]["outcome"], "spoken")

    def test_every_response_fits_the_message_limit(self):
        b, _ = make_bridge()
        for name in ("hello", "get_status", "get_current", "get_surroundings", "get_diagnostics"):
            for _c, t, payload in b.on_command(P.json_bytes({"v": 1, "id": 1, "cmd": name})):
                self.assertLessEqual(len(payload), P.MAX_MESSAGE_BYTES)
                P.encode(t, payload, 0, 23)                # must not raise at the smallest MTU


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
            expected = [s["say"] for s in SCENARIOS[name] if "say" in s][1:]   # first one is learned
            self.assertEqual(said[:len(expected)], expected, name)

    def test_lidar_loss_is_down_not_stalled(self):
        """As main.health_check: losing the LiDAR is status down + sensor_degraded; the loop
        keeps ticking, so guidance_stalled must NOT fire (it means the loop itself stopped)."""
        said, events = self.run_scenario("sensor_failure")
        self.assertIn("sensor_degraded", events)
        self.assertNotIn("guidance_stalled", events)
        self.assertIn("Sensors offline", said)

    def test_crash_scenario_reports_and_recovers(self):
        said, events = self.run_scenario("service_crash")
        self.assertIn("service_unreachable", events)
        self.assertIn("service_restarted", events)
        self.assertGreaterEqual(events.count("device_ready"), 2)


@unittest.skipUnless(__import__("importlib").util.find_spec("jsonschema"), "jsonschema not installed")
class SchemaTests(unittest.TestCase):
    """Every JSON message the bridge makes validates against contract/schema_v1.json."""

    def test_messages_match_the_schema(self):
        import jsonschema
        with open(os.path.join(HERE, "contract", "schema_v1.json"), encoding="utf-8") as fh:
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
                    outs = b.on_walk(src.walk()) + b.on_health(src.health(), src.system_health(), src.system())
                except ConnectionError as exc:
                    outs = b.on_unreachable(str(exc))
                for _c, t, payload in outs:
                    if t == P.T_EVENT:
                        obj = P.parse_json(payload)
                        check("event", obj)
                        seen.add(obj["event"])
                clock.t += 0.1
        for cmd in ("hello", "ping", "get_status", "get_current", "get_surroundings", "get_diagnostics",
                    "session_start", "session_stop", "launch", "set_device_settings"):
            msg = {"v": 1, "id": 7, "cmd": cmd}
            if cmd != "launch":
                check("command", msg)
            for _c, t, payload in b.on_command(P.json_bytes(msg)):
                check("response" if t == P.T_RESPONSE else "event", P.parse_json(payload))
        self.assertTrue({"device_ready", "sensor_degraded", "service_unreachable"} <= seen)


@unittest.skipUnless(__import__("importlib").util.find_spec("dbus"), "dbus-python not installed")
class GattImportTest(unittest.TestCase):
    def test_imports(self):
        from ble_bridge import gatt_server
        for level in gatt_server.SECURITY.values():
            # EVENTS has no read callback: it must not advertise a read.
            self.assertFalse(any("read" in f for f in level["notify_only"]))
        self.assertEqual(gatt_server.SECURITY["encrypt"]["notify_only"], ["encrypt-notify"])
        self.assertLessEqual(len("Shappe-M".encode()), gatt_server.MAX_NAME_BYTES)


if __name__ == "__main__":
    unittest.main()
