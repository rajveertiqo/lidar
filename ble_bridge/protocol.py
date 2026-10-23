"""Shappe BLE protocol v1 — frames, the two binary records, and JSON messages.

PROTOTYPE. The normative contract is docs/06-interface-control-document.md; this
module is its reference implementation and the two must agree. Pure Python, no
Bluetooth: the GATT server (gatt_server.py) only moves the bytes made here.

Every characteristic value is one FRAME:

    off  size  field
    0    1     version       PROTOCOL_VERSION (1)
    1    1     type          T_* below
    2    2     seq           uint16 LE, per characteristic, per connection
    4    1     frag_index    0 .. frag_count-1
    5    1     frag_count    1 .. 255
    6    ..    payload       this fragment's bytes

A message longer than one notification is cut into fragments that share `seq`.
ALERT and STATE never fragment: header + 14-byte record = 20 bytes, which fits the
smallest ATT MTU (23) with nothing negotiated.
"""

import json
import struct
from typing import Any, Dict, List, Optional, Tuple

PROTOCOL_VERSION = 1

# --- UUIDs (application-specific, randomly generated 2026-10-09; never reuse) ---
_BASE = "37e9{:04x}-0882-43f6-ae9b-b733c720c46d"
SERVICE_UUID = _BASE.format(0x0001)
PROTOCOL_INFO_UUID = _BASE.format(0x0002)
ALERT_UUID = _BASE.format(0x0003)
STATE_UUID = _BASE.format(0x0004)
COMMAND_UUID = _BASE.format(0x0005)
EVENTS_UUID = _BASE.format(0x0006)

# Bluetooth SIG Device Information Service (16-bit assigned numbers).
DIS_UUID = "0000180a-0000-1000-8000-00805f9b34fb"
DIS_MANUFACTURER_UUID = "00002a29-0000-1000-8000-00805f9b34fb"
DIS_MODEL_UUID = "00002a24-0000-1000-8000-00805f9b34fb"
DIS_HARDWARE_REV_UUID = "00002a27-0000-1000-8000-00805f9b34fb"
DIS_FIRMWARE_REV_UUID = "00002a26-0000-1000-8000-00805f9b34fb"
DIS_SOFTWARE_REV_UUID = "00002a28-0000-1000-8000-00805f9b34fb"

# --- Message types ---
T_PROTOCOL_INFO = 0x01  # JSON, PROTOCOL_INFO (read)
T_ALERT = 0x10  # binary record, ALERT (notify/read)
T_ALERT_TEXT = 0x11  # epoch u16 + cue_seq u32 + UTF-8 sentence, ALERT (notify)
T_STATE = 0x20  # binary record, STATE (notify/read)
T_RESPONSE = 0x30  # JSON, EVENTS (notify)
T_EVENT = 0x31  # JSON, EVENTS (notify)
T_COMMAND = 0x40  # JSON, COMMAND (write), app -> device
TYPES = {
    T_PROTOCOL_INFO,
    T_ALERT,
    T_ALERT_TEXT,
    T_STATE,
    T_RESPONSE,
    T_EVENT,
    T_COMMAND,
}
JSON_TYPES = {T_PROTOCOL_INFO, T_RESPONSE, T_EVENT, T_COMMAND}

HEADER = struct.Struct("<BBHBB")
HEADER_SIZE = HEADER.size  # 6
ATT_NOTIFY_OVERHEAD = 3  # opcode + handle
MIN_ATT_MTU = 23
MAX_FRAGMENTS = 255
MAX_MESSAGE_BYTES = 4096  # any single message, after reassembly
REASSEMBLY_TIMEOUT_S = 2.0

# ALERT and STATE share one 14-byte layout:
#   epoch u16 | cue_seq u32 | age_ms u16 | phrase_id u16 | level i8 | flags/status u8 | u16
RECORD = struct.Struct("<HIHHbBH")
RECORD_SIZE = RECORD.size  # 14

PHRASE_NONE = 0  # no sentence (quiet / nothing decided)
PHRASE_UNLISTED = 0xFFFF  # a sentence not in the app's registry: use ALERT_TEXT
DIST_UNKNOWN = 0xFFFF
AGE_MAX_MS = 0xFFFE  # saturates; 0xFFFF = unknown
AGE_UNKNOWN = 0xFFFF
LEVEL_NONE = -1

# ALERT flags (u8)
AF_ANNOUNCE = 0x01  # the device's gate admitted this sentence (always set in v1)
AF_URGENT = 0x02  # the device classed it urgent (blocked / vehicle / mover)
AF_DEVICE_VOICE = 0x04  # the Pi's own voice accepted this cue: do not speak it again
AF_AUDIO_ENABLED = 0x08  # config AUDIO_ENABLED; 0 = no device, Pi or phone, may speak
AF_REPLAY = 0x10  # a read / resubscribe copy of an old alert: NEVER speak it
AF_LOOP_STALE = 0x20  # the guidance loop was not updating when this was sent

# STATE status (u8)
STATUS_OK = 0
STATUS_DEGRADED = 1
STATUS_DOWN = 2  # /health "down": not guiding, or LiDAR not OK
STATUS_UNREACHABLE = 3  # the bridge cannot reach the navigation service at all
STATUS_NAMES = {0: "ok", 1: "degraded", 2: "down", 3: "unreachable"}

# STATE health bits (u16)
HB_GUIDING = 1 << 0
HB_LIDAR_OK = 1 << 1
HB_RADAR_OK = 1 << 2
HB_CAMERA_OK = 1 << 3
HB_HAILO_OK = 1 << 4
HB_FRONT_GLASS_OK = 1 << 5
HB_AUDIO_ENABLED = 1 << 6
HB_DEVICE_VOICE_USABLE = 1 << 7
HB_SIMULATED_INPUT = 1 << 8
HB_THERMAL_WARN = 1 << 9
HB_UNDERVOLTAGE = 1 << 10
HB_LOOP_STALE = 1 << 11
HEALTH_BITS = {
    "guiding": HB_GUIDING,
    "lidar_ok": HB_LIDAR_OK,
    "radar_ok": HB_RADAR_OK,
    "camera_ok": HB_CAMERA_OK,
    "hailo_ok": HB_HAILO_OK,
    "front_glass_ok": HB_FRONT_GLASS_OK,
    "audio_enabled": HB_AUDIO_ENABLED,
    "device_voice_usable": HB_DEVICE_VOICE_USABLE,
    "simulated_input": HB_SIMULATED_INPUT,
    "thermal_warn": HB_THERMAL_WARN,
    "undervoltage": HB_UNDERVOLTAGE,
    "loop_stale": HB_LOOP_STALE,
}

# Error codes carried in RESPONSE.error.code
ERRORS = (
    "BAD_FRAME",
    "BAD_JSON",
    "PROTOCOL_MISMATCH",
    "UNKNOWN_COMMAND",
    "BAD_ARGS",
    "NOT_SUPPORTED",
    "NOT_AUTHORIZED",
    "TOO_LARGE",
    "BUSY",
    "DEVICE_UNAVAILABLE",
    "INTERNAL",
)


class ProtocolError(ValueError):
    """A frame or message that breaks the contract; `code` is one of ERRORS."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


# --- sequence arithmetic (RFC 1982 serial numbers) ---


def seq16_newer(a: int, b: int) -> bool:
    """True if uint16 sequence `a` is after `b`, across wrap-around."""
    return a != b and ((a - b) & 0xFFFF) < 0x8000


def cue_newer(
    epoch_a: int, cue_a: int, epoch_b: Optional[int], cue_b: Optional[int]
) -> bool:
    """True if alert (epoch_a, cue_a) should replace the last one handled.

    A different epoch means the device's counter restarted: always newer.
    """
    if epoch_b is None or cue_b is None or epoch_a != epoch_b:
        return True
    return cue_a > cue_b


# --- frames ---


def fragment_payload_size(att_mtu: int) -> int:
    """Payload bytes per frame for a negotiated ATT MTU."""
    mtu = max(MIN_ATT_MTU, int(att_mtu))
    return min(mtu, 517) - ATT_NOTIFY_OVERHEAD - HEADER_SIZE


def max_message_bytes(att_mtu: int = MIN_ATT_MTU) -> int:
    """The longest message `encode` accepts at this ATT MTU: 255 x 14 = 3570 B at MTU 23."""
    return min(MAX_MESSAGE_BYTES, MAX_FRAGMENTS * fragment_payload_size(att_mtu))


def encode(
    msg_type: int, payload: bytes, seq: int, att_mtu: int = MIN_ATT_MTU
) -> List[bytes]:
    """One message -> the frames to send, in order."""
    if msg_type not in TYPES:
        raise ProtocolError("BAD_FRAME", f"unknown type 0x{msg_type:02x}")
    if len(payload) > MAX_MESSAGE_BYTES:
        raise ProtocolError("TOO_LARGE", f"{len(payload)} bytes > {MAX_MESSAGE_BYTES}")
    room = fragment_payload_size(att_mtu)
    chunks = [payload[i : i + room] for i in range(0, len(payload), room)] or [b""]
    if len(chunks) > MAX_FRAGMENTS:
        raise ProtocolError("TOO_LARGE", f"{len(chunks)} fragments > {MAX_FRAGMENTS}")
    seq &= 0xFFFF
    return [
        HEADER.pack(PROTOCOL_VERSION, msg_type, seq, i, len(chunks)) + c
        for i, c in enumerate(chunks)
    ]


def parse_header(frame: bytes) -> Tuple[int, int, int, int, bytes]:
    """frame -> (type, seq, frag_index, frag_count, fragment payload)."""
    if len(frame) < HEADER_SIZE:
        raise ProtocolError(
            "BAD_FRAME", f"frame of {len(frame)} bytes is shorter than the header"
        )
    version, msg_type, seq, index, count = HEADER.unpack_from(frame)
    if version != PROTOCOL_VERSION:
        raise ProtocolError(
            "PROTOCOL_MISMATCH", f"frame version {version}, expected {PROTOCOL_VERSION}"
        )
    if msg_type not in TYPES:
        raise ProtocolError("BAD_FRAME", f"unknown type 0x{msg_type:02x}")
    if count < 1 or index >= count:
        raise ProtocolError("BAD_FRAME", f"fragment {index} of {count}")
    return msg_type, seq, index, count, bytes(frame[HEADER_SIZE:])


class Reassembler:
    """Rebuilds messages from one characteristic's frames (one per characteristic).

    Fragments of a message arrive in order on one link; anything else — a gap, a
    new seq before the old message finished, a timeout — drops the partial
    message rather than guessing.
    """

    def __init__(self, timeout_s: float = REASSEMBLY_TIMEOUT_S):
        self.timeout_s = timeout_s
        self._key: Optional[Tuple[int, int]] = None
        self._parts: List[bytes] = []
        self._count = 0
        self._started = 0.0
        self.dropped = 0

    def feed(self, frame: bytes, now: float) -> Optional[Tuple[int, int, bytes]]:
        """Add one frame; returns (type, seq, payload) when a message completes."""
        msg_type, seq, index, count, part = parse_header(frame)
        if self._key is not None and (now - self._started > self.timeout_s):
            self._reset(dropped=True)
        if index == 0:
            if self._key is not None:
                self._reset(dropped=True)
            if count == 1:
                return msg_type, seq, part
            self._key, self._parts, self._count, self._started = (
                (msg_type, seq),
                [part],
                count,
                now,
            )
            return None
        if (
            self._key != (msg_type, seq)
            or index != len(self._parts)
            or count != self._count
        ):
            self._reset(dropped=self._key is not None)
            raise ProtocolError(
                "BAD_FRAME", f"fragment {index}/{count} of seq {seq} out of order"
            )
        self._parts.append(part)
        if sum(len(p) for p in self._parts) > MAX_MESSAGE_BYTES:
            self._reset(dropped=True)
            raise ProtocolError("TOO_LARGE", "reassembled message too large")
        if len(self._parts) == self._count:
            payload = b"".join(self._parts)
            key = self._key
            self._reset(dropped=False)
            return key[0], key[1], payload
        return None

    def _reset(self, dropped: bool) -> None:
        if dropped:
            self.dropped += 1
        self._key, self._parts, self._count, self._started = None, [], 0, 0.0


# --- the binary records ---


def _u16(
    value: Optional[float],
    scale: float = 1.0,
    unknown: int = DIST_UNKNOWN,
    cap: int = 0xFFFE,
) -> int:
    if value is None:
        return unknown
    try:
        v = float(value) * scale
    except (
        TypeError,
        ValueError,
        OverflowError,
    ):  # not a number, or an int beyond a float
        return unknown
    if v != v or v < 0:  # NaN or negative: not a measurement
        return unknown
    return cap if v >= cap else int(round(v))  # huge or infinite: saturate


def pack_alert(
    epoch: int,
    cue_seq: int,
    age_ms: Optional[float],
    phrase_id: int,
    level: int,
    flags: int,
    front_ft: Optional[float],
) -> bytes:
    """The 14-byte ALERT record. front_ft is sent in tenths of a foot."""
    return RECORD.pack(
        epoch & 0xFFFF,
        cue_seq & 0xFFFFFFFF,
        _u16(age_ms, unknown=AGE_UNKNOWN, cap=AGE_MAX_MS),
        phrase_id & 0xFFFF,
        max(-1, min(3, int(level))),
        flags & 0xFF,
        _u16(front_ft, scale=10.0),
    )


def unpack_alert(payload: bytes) -> Dict[str, Any]:
    if len(payload) < RECORD_SIZE:
        raise ProtocolError("BAD_FRAME", f"ALERT record of {len(payload)} bytes")
    epoch, cue, age, phrase, level, flags, front = RECORD.unpack_from(payload)
    return {
        "epoch": epoch,
        "cue_seq": cue,
        "age_ms": None if age == AGE_UNKNOWN else age,
        "phrase_id": phrase,
        "level": level,
        "announce": bool(flags & AF_ANNOUNCE),
        "urgent": bool(flags & AF_URGENT),
        "device_voice": bool(flags & AF_DEVICE_VOICE),
        "audio_enabled": bool(flags & AF_AUDIO_ENABLED),
        "replay": bool(flags & AF_REPLAY),
        "loop_stale": bool(flags & AF_LOOP_STALE),
        "front_ft": None if front == DIST_UNKNOWN else front / 10.0,
    }


def pack_state(
    epoch: int,
    cue_seq: int,
    age_ms: Optional[float],
    phrase_id: int,
    level: int,
    status: int,
    health_bits: int,
) -> bytes:
    """The 14-byte STATE record (also the 1 Hz heartbeat)."""
    return RECORD.pack(
        epoch & 0xFFFF,
        cue_seq & 0xFFFFFFFF,
        _u16(age_ms, unknown=AGE_UNKNOWN, cap=AGE_MAX_MS),
        phrase_id & 0xFFFF,
        max(-1, min(3, int(level))),
        status & 0xFF,
        health_bits & 0xFFFF,
    )


def unpack_state(payload: bytes) -> Dict[str, Any]:
    if len(payload) < RECORD_SIZE:
        raise ProtocolError("BAD_FRAME", f"STATE record of {len(payload)} bytes")
    epoch, cue, age, phrase, level, status, bits = RECORD.unpack_from(payload)
    return {
        "epoch": epoch,
        "cue_seq": cue,
        "age_ms": None if age == AGE_UNKNOWN else age,
        "phrase_id": phrase,
        "level": level,
        "status": STATUS_NAMES.get(status, f"unknown({status})"),
        "health": {name: bool(bits & bit) for name, bit in HEALTH_BITS.items()},
    }


ALERT_TEXT_PREFIX = struct.Struct("<HI")


def pack_alert_text(epoch: int, cue_seq: int, text: str) -> bytes:
    return ALERT_TEXT_PREFIX.pack(epoch & 0xFFFF, cue_seq & 0xFFFFFFFF) + text.encode(
        "utf-8"
    )


def unpack_alert_text(payload: bytes) -> Dict[str, Any]:
    if len(payload) < ALERT_TEXT_PREFIX.size:
        raise ProtocolError("BAD_FRAME", "ALERT_TEXT shorter than its prefix")
    epoch, cue = ALERT_TEXT_PREFIX.unpack_from(payload)
    try:
        text = payload[ALERT_TEXT_PREFIX.size :].decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ProtocolError("BAD_FRAME", f"ALERT_TEXT not UTF-8: {exc}")
    return {"epoch": epoch, "cue_seq": cue, "text": text}


# --- JSON messages ---


def json_bytes(obj: Dict[str, Any]) -> bytes:
    """Compact UTF-8 JSON, the payload of every JSON-typed message. Strict: NaN or
    Infinity raise ValueError; a lone surrogate (no UTF-8 for it) is sent as '?'."""
    return json.dumps(
        obj, separators=(",", ":"), ensure_ascii=False, allow_nan=False
    ).encode("utf-8", "replace")


def parse_json(payload: bytes) -> Dict[str, Any]:
    try:
        obj = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError("BAD_JSON", str(exc))
    if not isinstance(obj, dict):
        raise ProtocolError("BAD_JSON", "a message must be a JSON object")
    return obj


def decode_message(msg_type: int, payload: bytes) -> Dict[str, Any]:
    """A reassembled message -> a dict tagged with its kind (for tools and tests)."""
    if msg_type == T_ALERT:
        return {"kind": "alert", **unpack_alert(payload)}
    if msg_type == T_STATE:
        return {"kind": "state", **unpack_state(payload)}
    if msg_type == T_ALERT_TEXT:
        return {"kind": "alert_text", **unpack_alert_text(payload)}
    kind = {
        T_PROTOCOL_INFO: "protocol_info",
        T_RESPONSE: "response",
        T_EVENT: "event",
        T_COMMAND: "command",
    }[msg_type]
    return {"kind": kind, **parse_json(payload)}
