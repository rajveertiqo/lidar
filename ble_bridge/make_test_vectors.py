"""Write contract/test_vectors.json — golden frames for any decoder (Swift, Kotlin, Dart, JS).

PROTOTYPE tool. Each vector is the exact bytes the device sends (hex, one string
per notification) for a known message, plus what a correct decoder must produce.
A mobile decoder that reproduces every `expect` from every `frames_hex` agrees
with the device bit for bit.

    venv/bin/python -m ble_bridge.make_test_vectors
"""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from ble_bridge import protocol as P            # noqa: E402
from ble_bridge.core import Bridge, load_vocab  # noqa: E402
from ble_bridge.sources import make_health, make_walk   # noqa: E402

OUT = os.path.join(HERE, "contract", "test_vectors.json")


class _Clock:
    t = 5000.0

    def __call__(self):
        return self.t


def _vector(name, note, char, msg_type, payload, seq, mtu):
    frames = P.encode(msg_type, payload, seq, mtu)
    r = P.Reassembler()
    done = None
    for f in frames:
        done = r.feed(f, 0.0)
    expect = P.decode_message(done[0], done[2])
    return {"name": name, "note": note, "characteristic": char, "att_mtu": mtu,
            "frames_hex": [f.hex() for f in frames], "expect": expect}


def main() -> int:
    clock = _Clock()
    vocab = load_vocab()
    b = Bridge(vocab, {"model": "test vectors", "software": "test", "mock": False},
               clock=clock, wall=lambda: 1791504000.0)
    b._persist.data.update(device_id="0123456789abcdef", epoch=3)
    vectors = []

    def outs_to_vectors(outs, prefix, mtu=23, note=""):
        for i, (char, t, payload) in enumerate(outs):
            vectors.append(_vector(f"{prefix}.{i}", note, char, t, payload, len(vectors) & 0xFFFF, mtu))

    b.on_walk(make_walk(41, "Walk straight", "Walk straight", age_s=0.08))   # learned, no alert
    b.on_health(make_health(), {"front_glass_ok": True}, {"soc_c": 52.0, "throttled": 0})
    clock.t += 0.2
    outs = b.on_walk(make_walk(42, "Blocked by glass", "Blocked by glass", speak=True, urgent=True,
                               front_ft=3.6, age_s=0.11))
    outs_to_vectors([o for o in outs if o[0] == "alert"], "alert_blocked_glass_mtu23",
                    note="urgent alert + its text at the smallest MTU (text spans fragments)")
    outs_to_vectors([o for o in outs if o[0] == "alert"], "alert_blocked_glass_mtu185", mtu=185,
                    note="the same alert at a typical iOS MTU")
    clock.t += 0.2
    outs = b.on_walk(make_walk(43, "Walk straight", "Walk straight", age_s=0.09,
                               device_voice_usable=True, accepted_seq=43))
    outs_to_vectors([o for o in outs if o[0] == "alert"], "alert_device_already_spoke",
                    note="DEVICE_VOICE set: the Pi's own voice accepted this cue — the app must not repeat it")
    clock.t += 1.1
    outs_to_vectors(b.tick(), "state_heartbeat", note="STATE heartbeat (1 Hz minimum)")
    clock.t += 1.0
    outs = b.on_health(make_health("degraded", radar=False,
                                   warnings=["Radar offline — glass may not be detected"]),
                       {"front_glass_ok": False}, {"soc_c": 52.0, "throttled": 0})
    outs_to_vectors([o for o in outs if o[0] == "events"], "events_radar_lost",
                    note="sensor_degraded x2 + health_changed, JSON, fragmented at MTU 23")
    clock.t += 1.0
    outs_to_vectors(b.on_unreachable("ConnectionRefusedError: [Errno 111] Connection refused"),
                    "service_unreachable", note="the navigation service stopped answering")
    for i, cmd in enumerate(({"v": 1, "id": 1, "cmd": "hello", "args": {"protocol": 1, "att_mtu": 185}},
                             {"v": 1, "id": 2, "cmd": "launch"})):
        vectors.append(_vector(f"command.{i}", "app -> device COMMAND write", "command",
                               P.T_COMMAND, P.json_bytes(cmd), i, 23))
        outs_to_vectors(b.on_command(P.json_bytes(cmd)), f"response.{i}", mtu=185,
                        note="device -> app RESPONSE")
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as fh:
        json.dump({"protocol": P.PROTOCOL_VERSION, "vocab_revision": vocab["revision"],
                   "vocab_sha": vocab["sha"], "vectors": vectors}, fh, indent=1, ensure_ascii=False)
        fh.write("\n")
    print(f"{len(vectors)} vectors -> {os.path.relpath(OUT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
