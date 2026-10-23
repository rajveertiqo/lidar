"""Bridge core — navigation-service snapshots in, protocol messages out.

PROTOTYPE. No Bluetooth and no network here: the GATT server feeds this class
what the navigation service reported and sends what it returns, so the whole
behaviour is testable on a desk (test_ble_bridge.py).

What it relies on in the navigation service (all CONFIRMED in the code):
  GET /guidance/walk   -> utils.unified_guidance.latest_guidance(): notify.cue_seq,
                          notify.instruction, notify.speak, notify.level,
                          notify.pi_voice, path.instruction, path.front_ft,
                          obstacles.items, loop.age_s / loop.stale
  GET /health          -> status, guiding, sensors.*_ok, warnings
  GET /guidance/status -> sensor_fusion.get_system_health(): front_glass_ok
  GET /guidance/log    -> speech_log.recent()
  POST /guidance/mark  -> a tester's verdict stamped into the speech log

The one rule copied from the dashboard (templates/dashboard.html): an
announcement is a STEP of notify.cue_seq — never a poll, never notify.speak on
its own — and the first value seen is only learned.
"""

import json
import math
import os
import re
import threading
import time
import uuid
from collections import deque
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

from . import protocol as P

HERE = os.path.dirname(os.path.abspath(__file__))
VOCAB_PATH = os.path.join(HERE, "contract", "vocab_v1.json")

STATE_MIN_INTERVAL_S = 0.2  # STATE notifies at most 5 times a second ...
STATE_HEARTBEAT_S = 1.0  # ... and at least once a second (the heartbeat)
ALERT_MAX_AGE_MS = 2000  # = config.GUIDANCE_RESULT_STALE_S; the app drops older
THERMAL_WARN_C = 80.0  # Pi 5 firmware begins to throttle near here
MAX_WARNINGS = 8
MAX_WARNING_CHARS = 160
MAX_WARNINGS_BYTES = (
    1400  # all of them as JSON: health_changed must fit a message at MTU 23
)
MAX_TEXT_CHARS = 200  # any other free text from the service or the app
MAX_ALERT_TEXT_BYTES = 200  # ICD: ALERT_TEXT carries at most 200 B of UTF-8
MAX_APP_LATENCY_MS = 3_600_000
MAX_PING_T = (
    2**53
)  # the app's clock in ms: any number a JavaScript app can hold exactly
COMMANDS_PER_S = 10
LOG_EVERY_S = 30.0  # a fault that repeats every poll is printed at most this often

# vcgencmd get_throttled bits
_UV_NOW, _CAPPED_NOW, _THROTTLED_NOW, _SOFT_TEMP_NOW = 1 << 0, 1 << 1, 1 << 2, 1 << 3

Out = Tuple[
    str, int, bytes
]  # (characteristic: "alert" | "state" | "events", type, payload)


class CommandError(Exception):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


def load_vocab(path: str = VOCAB_PATH) -> Dict[str, Any]:
    with open(path, encoding="utf-8") as fh:
        reg = json.load(fh)
    by_text = {p["text"]: p for p in reg["phrases"] if not p.get("retired")}
    return {
        "by_text": by_text,
        "revision": reg["revision"],
        "sha": reg["sha"],
        "version": reg["version"],
    }


def _iso(t: float) -> str:
    return (
        time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(t))
        + f".{int(t * 1000) % 1000:03d}Z"
    )


# --- inputs of the wrong type count as absent: one bad snapshot must not poison the next ---


def _dict(value: Any) -> Dict[str, Any]:
    """A field that should be an object; anything else counts as absent."""
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> list:
    """A field that should be an array; anything else counts as empty."""
    return value if isinstance(value, (list, tuple)) else []


def _num(value: Any) -> Any:
    """`value` if it is a finite number (not a bool), else None: unknown."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        return value if math.isfinite(value) else None
    except OverflowError:  # an int too large for a float
        return None


def _text(value: Any, limit: int = MAX_TEXT_CHARS) -> Any:
    """Free text from the service or the app, cut so that one field cannot outgrow a message."""
    return value[:limit] if isinstance(value, str) else value


def _utf8(text: str, limit: int) -> str:
    """`text` cut to at most `limit` bytes of UTF-8, on a character boundary."""
    return text.encode("utf-8", "replace")[:limit].decode("utf-8", "ignore")


def _warnings(raw: Any) -> Tuple[str, ...]:
    """/health warnings as sent: at most 8, each at most 160 characters, and together at most
    MAX_WARNINGS_BYTES of JSON (non-ASCII text and escapes can be 4-6 bytes a character)."""
    out: List[str] = []
    for w in _list(raw)[:MAX_WARNINGS]:
        w = str(w)[:MAX_WARNING_CHARS]
        if len(P.json_bytes({"warnings": out + [w]})) > MAX_WARNINGS_BYTES:
            break
        out.append(w)
    return tuple(out)


class RateLimitedLog:
    """print() for faults that can repeat every poll: each key at most once per `every_s`,
    then with a count of the lines held back, so a stuck fault cannot flood the journal."""

    def __init__(
        self, clock: Callable[[], float] = time.monotonic, every_s: float = LOG_EVERY_S
    ):
        self.clock, self.every_s = clock, every_s
        self._last: Dict[str, float] = {}
        self._held: Dict[str, int] = {}

    def __call__(self, key: str, message: str) -> None:
        now = self.clock()
        if key in self._last and now - self._last[key] < self.every_s:
            self._held[key] = self._held.get(key, 0) + 1
            return
        held = self._held.pop(key, 0)
        self._last[key] = now
        print(
            f"[ble] {message}"
            + (f" (+{held} more since the last report)" if held else "")
        )


_DEVICE_ID = re.compile(
    r"[0-9a-f]{16}"
)  # as save() writes it, and contract/schema_v1.json


def _saved_state(saved: Any) -> Dict[str, Any]:
    """device_id and epoch from the state file; ValueError unless they are what save() writes.
    (A long device_id would also push PROTOCOL_INFO past the one frame a read serves.)"""
    if not isinstance(saved, dict):
        raise ValueError(f"not a JSON object but {type(saved).__name__}")
    device_id, epoch = saved.get("device_id"), saved.get("epoch")
    if not isinstance(device_id, str) or not _DEVICE_ID.fullmatch(device_id):
        raise ValueError(f"device_id {str(device_id)[:40]!r} is not 16 hex digits")
    if not isinstance(epoch, int) or isinstance(epoch, bool):
        raise ValueError(f"epoch {str(epoch)[:40]!r} is not an integer")
    return {"device_id": device_id, "epoch": epoch & 0xFFFF}


class _Persist:
    """device_id and epoch, kept across restarts (atomic write). A state file that cannot be
    used counts as absent, and one that cannot be written (full or read-only SD card) leaves
    the values in memory: neither may stop the bridge."""

    def __init__(self, path: Optional[str], log: Callable[[str, str], None]):
        self.path, self.log = path, log
        self.data = {"device_id": uuid.uuid4().hex[:16], "epoch": 0}
        if path and os.path.exists(path):
            try:
                with open(path, encoding="utf-8") as fh:
                    self.data.update(_saved_state(json.load(fh)))
            except Exception as exc:  # unreadable, not JSON, or not what save() writes
                log(
                    "state-load",
                    f"state file {path} ignored ({type(exc).__name__}: {exc}); "
                    f"starting as a new device_id {self.data['device_id']}",
                )

    def save(self) -> None:
        if not self.path:
            return
        try:
            os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
            tmp = self.path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(self.data, fh)
            os.replace(tmp, self.path)
        except OSError as exc:
            self.log(
                "state-save",
                f"cannot save {self.path} ({exc}); "
                f"epoch {self.data['epoch']} is kept in memory only",
            )


class Bridge:
    def __init__(
        self,
        vocab: Dict[str, Any],
        identity: Dict[str, str],
        service: Any = None,
        state_path: Optional[str] = None,
        clock: Callable[[], float] = time.monotonic,
        wall: Callable[[], float] = time.time,
    ):
        self.vocab = vocab
        self.identity = identity
        self.service = service  # HttpSource / MockSource: speech_log(), mark()
        self.clock, self.wall = clock, wall
        self.log = RateLimitedLog(clock)
        self._lock = threading.RLock()
        self._persist = _Persist(state_path, self.log)
        # A new bridge process cannot know where the old cue counter was: new epoch.
        self._persist.data["epoch"] = (self.epoch + 1) & 0xFFFF
        self._persist.save()
        self.started = clock()
        self.last_cue: Optional[int] = None
        self.last_alert: Optional[bytes] = None
        self.walk: Dict[str, Any] = {}
        self.walk_at: Optional[float] = None
        self.health: Dict[str, Any] = {}
        self.sys_health: Dict[str, Any] = {}
        self.system: Dict[str, Any] = {}
        self.status = P.STATUS_UNREACHABLE
        self.unreachable_reason = "not polled yet"
        self._state_key: Optional[tuple] = None
        self._state_sent = -1e9
        self._state_payload = P.pack_state(
            self.epoch, 0, None, P.PHRASE_NONE, P.LEVEL_NONE, P.STATUS_UNREACHABLE, 0
        )
        self._flags: Dict[str, bool] = {}
        self._warnings: Tuple[str, ...] = ()
        self._ready_announced = False
        self.session: Optional[Dict[str, Any]] = None
        self.att_mtu = P.MIN_ATT_MTU
        self._link_mtu: Optional[int] = (
            None  # what BlueZ negotiated (the write option "mtu")
        )
        self._app_mtu: Optional[int] = None  # what the app says it has (hello att_mtu)
        self.acks: Deque[Dict[str, Any]] = deque(maxlen=50)
        self.stats = {
            "alerts": 0,
            "events": 0,
            "commands": 0,
            "command_errors": 0,
            "poll_errors": 0,
            "epoch_bumps": 0,
        }
        self._cmd_times: Deque[float] = deque(maxlen=COMMANDS_PER_S)

    # --- identity ---

    @property
    def epoch(self) -> int:
        return int(self._persist.data["epoch"])

    @property
    def device_id(self) -> str:
        return str(self._persist.data["device_id"])

    def _bump_epoch(self) -> None:
        self._persist.data["epoch"] = (self.epoch + 1) & 0xFFFF
        self.stats["epoch_bumps"] += 1
        self._persist.save()  # never raises: an unwritable disk keeps the epoch in memory

    def protocol_info(self) -> Dict[str, Any]:
        """PROTOCOL_INFO's value. It is read as ONE frame, so it must stay well under the
        512-byte attribute limit: model and software strings are left to the Device
        Information characteristics and the hello result (test: long identity strings)."""
        return {
            "v": P.PROTOCOL_VERSION,
            "protocol": P.PROTOCOL_VERSION,
            "min_protocol": 1,
            "device_id": self.device_id,
            "mock": bool(self.identity.get("mock")),
            "vocab": {
                "version": self.vocab["version"],
                "revision": self.vocab["revision"],
                "sha": self.vocab["sha"],
            },
            "epoch": self.epoch,
            "units": {"distance": "ft"},
            "limits": {
                "max_message_bytes": P.MAX_MESSAGE_BYTES,
                "alert_max_age_ms": ALERT_MAX_AGE_MS,
                "heartbeat_ms": int(STATE_HEARTBEAT_S * 1000),
            },
            "commands": sorted(_COMMANDS),
        }

    # --- inputs from the navigation service ---

    def on_walk(self, snap: Dict[str, Any]) -> List[Out]:
        """One /guidance/walk snapshot (latest_guidance()). Returns messages to send.
        A field of the wrong type counts as absent, here and in every later call."""
        out: List[Out] = []
        with self._lock:
            now = self.clock()
            snap = _dict(snap)
            self.walk, self.walk_at = snap, now
            if self.status == P.STATUS_UNREACHABLE:
                self.status = P.STATUS_DOWN  # reachable again; /health will refine
                self.unreachable_reason = ""
            n = _dict(snap.get("notify"))
            cue = n.get("cue_seq")
            if isinstance(cue, int) and not isinstance(cue, bool):
                if self.last_cue is None:
                    self.last_cue = cue  # first sight: learn only
                elif cue < self.last_cue:
                    self.last_cue = cue
                    self._bump_epoch()  # the service restarted
                    out += self._event("service_restarted", {"epoch": self.epoch})
                elif cue > self.last_cue:
                    self.last_cue = cue
                    out += self._alert(snap, cue)
            out += self._maybe_state(now)
        return out

    def _alert(self, snap: Dict[str, Any], cue: int) -> List[Out]:
        n = _dict(snap.get("notify"))
        loop = _dict(snap.get("loop"))
        pv = _dict(n.get("pi_voice"))
        text = str(n.get("instruction") or "")
        entry = self.vocab["by_text"].get(text)
        phrase = (
            entry["id"] if entry else (P.PHRASE_NONE if not text else P.PHRASE_UNLISTED)
        )
        level = entry["level"] if entry else (2 if n.get("level") == "urgent" else 1)
        exact = (
            bool(n.get("speak")) and n.get("cue_seq") == cue
        )  # the admitting tick itself
        urgent = (
            (n.get("level") == "urgent") if exact else bool(entry and entry["urgent"])
        )
        flags = P.AF_ANNOUNCE
        flags |= P.AF_URGENT if urgent else 0
        flags |= (
            P.AF_DEVICE_VOICE
            if (pv.get("usable") and pv.get("accepted_seq") == cue)
            else 0
        )
        flags |= P.AF_AUDIO_ENABLED if pv.get("audio_enabled") else 0
        flags |= P.AF_LOOP_STALE if loop.get("stale") else 0
        age_s = _num(loop.get("age_s"))  # not a finite number: unknown
        age_ms = None if age_s is None else age_s * 1000.0
        front = _dict(snap.get("path")).get("front_ft")
        record = P.pack_alert(self.epoch, cue, age_ms, phrase, level, flags, front)
        self.last_alert = record
        self.stats["alerts"] += 1
        out: List[Out] = [("alert", P.T_ALERT, record)]
        if text:
            out.append(
                (
                    "alert",
                    P.T_ALERT_TEXT,
                    P.pack_alert_text(
                        self.epoch, cue, _utf8(text, MAX_ALERT_TEXT_BYTES)
                    ),
                )
            )
        return out

    def on_health(
        self,
        health: Dict[str, Any],
        sys_health: Optional[Dict[str, Any]] = None,
        system: Optional[Dict[str, Any]] = None,
    ) -> List[Out]:
        """/health (+ /guidance/status, + the Pi's own temperature/throttle flags).
        A field of the wrong type counts as unknown, here and in every later call."""
        out: List[Out] = []
        with self._lock:
            self.health = _dict(health)
            self.sys_health = _dict(sys_health)
            self.system = _dict(system)
            status = self.health.get("status")
            self.status = {"ok": P.STATUS_OK, "degraded": P.STATUS_DEGRADED}.get(
                status if isinstance(status, str) else "", P.STATUS_DOWN
            )
            self.unreachable_reason = ""
            flags = self._health_flags()
            prev = self._flags
            for name in (
                "lidar_ok",
                "radar_ok",
                "camera_ok",
                "hailo_ok",
                "front_glass_ok",
            ):
                if name in prev and prev[name] != flags[name]:
                    sensor = name[:-3]
                    out += self._event(
                        "sensor_recovered" if flags[name] else "sensor_degraded",
                        {"sensor": sensor},
                    )
            if "guiding" in prev and prev["guiding"] != flags["guiding"]:
                error = _dict(self.health.get("guidance_loop")).get("error")
                out += self._event(
                    "guidance_resumed" if flags["guiding"] else "guidance_stalled",
                    {"reason": str(error)[:MAX_WARNING_CHARS] if error else ""},
                )
            if flags["thermal_warn"] != prev.get("thermal_warn", False):
                out += self._event(
                    "thermal_warning" if flags["thermal_warn"] else "thermal_ok",
                    self._thermal(),
                )
            if flags["undervoltage"] and not prev.get("undervoltage", False):
                out += self._event(
                    "power_warning", {"undervoltage": True, **self._thermal()}
                )
            warnings = _warnings(self.health.get("warnings"))
            if warnings != self._warnings or prev.get("_status") != self.status:
                self._warnings = warnings
                out += self._event("health_changed", self._health_summary())
            ready = self.status in (P.STATUS_OK, P.STATUS_DEGRADED)
            if ready and not self._ready_announced:
                out += self._event("device_ready", self._health_summary())
            self._ready_announced = ready
            self._flags = {**flags, "_status": self.status}
            out += self._maybe_state(self.clock())
        return out

    def on_unreachable(self, reason: str) -> List[Out]:
        """The navigation service did not answer (crashed, restarting, or overloaded)."""
        out: List[Out] = []
        reason = str(reason)[:MAX_WARNING_CHARS]
        with self._lock:
            self.stats["poll_errors"] += 1
            if self.status != P.STATUS_UNREACHABLE:
                self.status = P.STATUS_UNREACHABLE
                self.unreachable_reason = reason
                self._ready_announced = False
                out += self._event("service_unreachable", {"reason": reason})
            out += self._maybe_state(self.clock())
        return out

    def tick(self, force: bool = False) -> List[Out]:
        """Called about every 100 ms: the heartbeat, even when nothing changes.
        force=True sends STATE now, whatever the timers say (a phone just subscribed)."""
        with self._lock:
            return self._maybe_state(self.clock(), force=force)

    # --- STATE ---

    def _health_flags(self) -> Dict[str, bool]:
        sensors = _dict(self.health.get("sensors"))
        pv = _dict(_dict(self.walk.get("notify")).get("pi_voice"))
        loop = _dict(self.walk.get("loop"))
        throttled = self._throttled()
        soc = _num(self.system.get("soc_c"))
        warnings = " ".join(str(w) for w in _list(self.health.get("warnings")))
        return {
            "guiding": bool(self.health.get("guiding")) and not loop.get("stale", True),
            "lidar_ok": bool(sensors.get("lidar_ok")),
            "radar_ok": bool(sensors.get("radar_ok")),
            "camera_ok": bool(sensors.get("camera_ok")),
            "hailo_ok": bool(sensors.get("hailo_ok")),
            "front_glass_ok": bool(self.sys_health.get("front_glass_ok")),
            "audio_enabled": bool(pv.get("audio_enabled")),
            "device_voice_usable": bool(pv.get("usable")),
            # A mock device always says so: nothing it reports is a measurement.
            "simulated_input": bool(self.identity.get("mock"))
            or "SIMULATED" in warnings,
            "thermal_warn": bool(
                (soc is not None and soc >= THERMAL_WARN_C)
                or throttled & (_CAPPED_NOW | _THROTTLED_NOW | _SOFT_TEMP_NOW)
            ),
            "undervoltage": bool(throttled & _UV_NOW),
            "loop_stale": bool(loop.get("stale", True)),
        }

    def _health_bits(self) -> int:
        if self.status == P.STATUS_UNREACHABLE:
            return 0
        flags = self._health_flags()
        return sum(P.HEALTH_BITS[k] for k, v in flags.items() if v)

    def _throttled(self) -> int:
        """vcgencmd get_throttled bits; 0 (nothing known) unless a non-negative integer."""
        bits = self.system.get("throttled")
        return (
            bits
            if isinstance(bits, int) and not isinstance(bits, bool) and bits >= 0
            else 0
        )

    def _maybe_state(self, now: float, force: bool = False) -> List[Out]:
        path = _dict(self.walk.get("path"))
        loop = _dict(self.walk.get("loop"))
        text = (
            str(path.get("instruction") or "")
            if self.status != P.STATUS_UNREACHABLE
            else ""
        )
        entry = self.vocab["by_text"].get(text)
        phrase = (
            entry["id"] if entry else (P.PHRASE_NONE if not text else P.PHRASE_UNLISTED)
        )
        level = entry["level"] if entry else P.LEVEL_NONE
        bits = self._health_bits()
        key = (self.epoch, self.last_cue, phrase, level, self.status, bits)
        due = now - self._state_sent >= STATE_HEARTBEAT_S
        changed = (
            key != self._state_key and now - self._state_sent >= STATE_MIN_INTERVAL_S
        )
        if not (due or changed or force):
            return []
        age_s = _num(loop.get("age_s"))  # not a finite number: unknown
        if self.walk_at is not None and age_s is not None:
            age_s = age_s + (now - self.walk_at)  # how old the decision is NOW
        self._state_payload = P.pack_state(
            self.epoch,
            self.last_cue or 0,
            None if age_s is None else age_s * 1000.0,
            phrase,
            level,
            self.status,
            bits,
        )
        self._state_key, self._state_sent = key, now
        return [("state", P.T_STATE, self._state_payload)]

    def state_value(self) -> bytes:
        with self._lock:
            return self._state_payload

    def alert_value(self) -> Optional[bytes]:
        """ALERT's read value: the last alert, marked REPLAY so it is never spoken."""
        with self._lock:
            if self.last_alert is None:
                return None
            rec = bytearray(self.last_alert)
            rec[11] |= P.AF_REPLAY  # flags byte of the record
            return bytes(rec)

    # --- events ---

    def _event(self, name: str, data: Dict[str, Any]) -> List[Out]:
        self.stats["events"] += 1
        msg = {
            "v": P.PROTOCOL_VERSION,
            "event": name,
            "ts": _iso(self.wall()),
            "uptime_ms": int((self.clock() - self.started) * 1000),
            "session_id": (self.session or {}).get("id"),
            "data": data,
        }
        return [("events", P.T_EVENT, P.json_bytes(msg))]

    def _thermal(self) -> Dict[str, Any]:
        return {
            "soc_c": _num(self.system.get("soc_c")),
            "throttled": f"0x{self._throttled():x}",
        }

    def _health_summary(self) -> Dict[str, Any]:
        sensors = _dict(self.health.get("sensors"))
        return {
            "status": P.STATUS_NAMES[self.status],
            "guiding": bool(self.health.get("guiding")),
            "sensors": {k[:-3]: bool(v) for k, v in sensors.items()},
            "warnings": list(self._warnings),
        }

    # --- connection lifecycle ---

    def set_mtu(self, link: Optional[int] = None, app: Optional[int] = None) -> int:
        """The ATT MTU BlueZ negotiated (`link`) and/or the one the app claims (`app`). Frames
        are cut for the smaller of those known: a claim above the link's would lose them."""
        with self._lock:
            if link is not None:
                self._link_mtu = max(P.MIN_ATT_MTU, min(int(link), 517))
            if app is not None:
                self._app_mtu = max(P.MIN_ATT_MTU, min(int(app), 517))
            known = [m for m in (self._link_mtu, self._app_mtu) if m is not None]
            self.att_mtu = min(known) if known else P.MIN_ATT_MTU
            return self.att_mtu

    def on_connect(self) -> List[Out]:
        with self._lock:
            self._link_mtu = self._app_mtu = None
            self.att_mtu = P.MIN_ATT_MTU
            return []

    def on_disconnect(self) -> None:
        with self._lock:
            self._link_mtu = self._app_mtu = None
            self.att_mtu = P.MIN_ATT_MTU

    # --- commands ---

    def on_command(self, payload: bytes) -> List[Out]:
        """A reassembled COMMAND message -> its RESPONSE (and any events). Never raises."""
        cmd_id: Any = None
        try:
            msg = P.parse_json(payload)
            cmd_id = msg.get("id")
            if msg.get("v") != P.PROTOCOL_VERSION:
                raise CommandError(
                    "PROTOCOL_MISMATCH", f"command v={msg.get('v')}, device speaks v1"
                )
            if (
                not isinstance(cmd_id, int)
                or isinstance(cmd_id, bool)
                or not 0 <= cmd_id <= 0xFFFFFFFF
            ):
                raise CommandError("BAD_ARGS", "id must be an unsigned 32-bit integer")
            name = msg.get("cmd")
            args = msg.get("args") or {}
            if not isinstance(args, dict):
                raise CommandError("BAD_ARGS", "args must be an object")
            handler = _COMMANDS.get(name)
            if handler is None:
                raise CommandError("UNKNOWN_COMMAND", f"unknown command {name!r}")
            with self._lock:
                now = self.clock()
                if (
                    len(self._cmd_times) == COMMANDS_PER_S
                    and now - self._cmd_times[0] < 1.0
                ):
                    raise CommandError(
                        "BUSY", f"more than {COMMANDS_PER_S} commands a second"
                    )
                self._cmd_times.append(now)
                self.stats["commands"] += 1
            result, extra = handler(self, args)
            body = P.json_bytes(
                {"v": P.PROTOCOL_VERSION, "id": cmd_id, "ok": True, "result": result}
            )
            limit = P.max_message_bytes(self.att_mtu)
            if (
                len(body) > limit
            ):  # encode would refuse it at send time, unseen by the app
                raise CommandError(
                    "TOO_LARGE",
                    f"reply of {len(body)} B is over the {limit} B "
                    f"a message can carry at ATT MTU {self.att_mtu}",
                )
            return [("events", P.T_RESPONSE, body)] + extra
        except (CommandError, P.ProtocolError) as exc:
            code, message = exc.code, str(exc)
        except Exception as exc:  # never let a command kill the bridge
            code, message = "INTERNAL", f"{type(exc).__name__}: {exc}"
        with self._lock:
            self.stats["command_errors"] += 1
        return [("events", P.T_RESPONSE, _error_body(cmd_id, code, message))]

    def bad_frame(self, code: str, message: str) -> List[Out]:
        """A COMMAND write that could not be reassembled: answer with id null."""
        return [("events", P.T_RESPONSE, _error_body(None, code, message))]

    def too_large(self, response: bytes, message: str) -> bytes:
        """A RESPONSE that no longer fits the link (the MTU fell after it was built): the
        same command id answered with TOO_LARGE, so the app is not left waiting."""
        try:
            cmd_id = json.loads(response.decode("utf-8")).get("id")
        except Exception:
            cmd_id = None
        with self._lock:
            self.stats["command_errors"] += 1
        return _error_body(cmd_id, "TOO_LARGE", message)


def _error_body(cmd_id: Any, code: str, message: str) -> bytes:
    """An error RESPONSE, made only of values that cannot fail to encode or outgrow a message."""
    if (
        not isinstance(cmd_id, int)
        or isinstance(cmd_id, bool)
        or not 0 <= cmd_id <= 0xFFFFFFFF
    ):
        cmd_id = None
    return P.json_bytes(
        {
            "v": P.PROTOCOL_VERSION,
            "id": cmd_id,
            "ok": False,
            "error": {"code": code, "message": str(message)[:200]},
        }
    )


# A reply's bytes around its result (with the longest id): a result may use the rest.
_REPLY_OVERHEAD = len(
    P.json_bytes({"v": P.PROTOCOL_VERSION, "id": 0xFFFFFFFF, "ok": True, "result": {}})
) - len(b"{}")


# --- command handlers: (bridge, args) -> (result, extra messages) ---


def _cmd_hello(b: Bridge, args: Dict[str, Any]):
    proto = args.get("protocol", 1)
    if proto != P.PROTOCOL_VERSION:
        raise CommandError(
            "PROTOCOL_MISMATCH", f"app speaks v{proto}, device speaks v1"
        )
    mtu = args.get("att_mtu")
    if mtu is not None:
        if not isinstance(mtu, int) or not P.MIN_ATT_MTU <= mtu <= 517:
            raise CommandError("BAD_ARGS", "att_mtu must be an integer 23..517")
        b.set_mtu(app=mtu)  # never above what BlueZ negotiated for this link
    with b._lock:
        info = b.protocol_info()
        info["model"] = b.identity.get("model", "")
        info["software"] = b.identity.get("software", "")
        info["session"] = (
            {k: v for k, v in b.session.items() if not k.startswith("_")}
            if b.session
            else None
        )
        info["status"] = P.STATUS_NAMES[b.status]
        info["att_mtu"] = b.att_mtu
    return info, []


def _cmd_ping(b: Bridge, args: Dict[str, Any]):
    t = args.get("t")
    if t is not None and (
        _num(t) is None or abs(t) > MAX_PING_T
    ):  # echoed: keep it small
        raise CommandError("BAD_ARGS", "t must be a number (the app's clock, ms)")
    return {"t": t, "uptime_ms": int((b.clock() - b.started) * 1000)}, []


def _cmd_get_status(b: Bridge, args: Dict[str, Any]):
    with b._lock:
        h = b.health
        loop = _dict(h.get("guidance_loop"))
        pv = _dict(_dict(b.walk.get("notify")).get("pi_voice"))
        return {
            "status": P.STATUS_NAMES[b.status],
            "unreachable_reason": b.unreachable_reason or None,
            "guiding": bool(h.get("guiding")),
            "sensors": {k[:-3]: bool(v) for k, v in _dict(h.get("sensors")).items()},
            "front_glass_ok": bool(b.sys_health.get("front_glass_ok")),
            "radar_down": [_text(r) for r in _list(h.get("radar_down"))[:6]],
            "warnings": list(b._warnings),
            "loop": {
                k: _text(loop.get(k)) for k in ("running", "last_ok_age_s", "error")
            },
            "voice": {
                k: _text(pv.get(k))
                for k in ("audio_enabled", "usable", "output", "error")
            },
            "system": b._thermal(),
            "battery": None,  # NOT IMPLEMENTED: no battery gauge on this device
        }, []


def _cmd_get_current(b: Bridge, args: Dict[str, Any]):
    with b._lock:
        p = _dict(b.walk.get("path"))
        loop = _dict(b.walk.get("loop"))
        text = str(p.get("instruction") or "")
        entry = b.vocab["by_text"].get(text) or {}
        return {
            "instruction": _text(text),
            "phrase_id": entry.get("id", 0 if not text else 0xFFFF),
            "action": p.get("action"),
            "rule": p.get("rule"),
            "side": p.get("turn_side"),
            "obstacle": p.get("obstacle"),
            "front_ft": p.get("front_ft"),
            "glass_ahead": p.get("glass_ahead"),
            "reason": str(p.get("reason") or "")[:200],
            "age_s": loop.get("age_s"),
            "stale": loop.get("stale", True),
        }, []


def _cmd_get_surroundings(b: Bridge, args: Dict[str, Any]):
    with b._lock:
        obs = _dict(b.walk.get("obstacles"))
        items = [
            {
                k: _dict(o).get(k)
                for k in (
                    "object",
                    "clock",
                    "sector",
                    "distance_ft",
                    "urgency",
                    "source",
                    "shape",
                )
            }
            for o in _list(obs.get("items"))[:6]
        ]
        loop = _dict(b.walk.get("loop"))
        return {
            "level": obs.get("level"),
            "summary": obs.get("summary"),
            "items": items,
            "age_s": loop.get("age_s"),
            "stale": loop.get("stale", True),
        }, []


def _cmd_get_speech_log(b: Bridge, args: Dict[str, Any]):
    limit = args.get("limit", 10)
    if not isinstance(limit, int) or not 1 <= limit <= 20:
        raise CommandError("BAD_ARGS", "limit must be an integer 1..20")
    if b.service is None:
        raise CommandError("DEVICE_UNAVAILABLE", "no navigation service")
    try:
        log = b.service.speech_log(limit)
    except Exception as exc:
        raise CommandError("DEVICE_UNAVAILABLE", f"speech log: {exc}")
    keep = ("time", "source", "level", "text", "cue_seq", "via", "outcome")
    entries = [
        {k: _text(e.get(k)) for k in keep}
        for e in _list(_dict(log).get("lines"))
        if isinstance(e, dict)
    ][-limit:]
    # 20 entries can outgrow a message at MTU 23 (255 x 14 B): drop the oldest until it fits.
    room = P.max_message_bytes(b.att_mtu) - _REPLY_OVERHEAD
    while entries and len(P.json_bytes({"entries": entries})) > room:
        entries.pop(0)
    return {"entries": entries}, []


def _cmd_get_diagnostics(b: Bridge, args: Dict[str, Any]):
    with b._lock:
        return {
            "stats": dict(b.stats),
            "epoch": b.epoch,
            "last_cue": b.last_cue,
            "att_mtu": b.att_mtu,
            "walk_age_s": None
            if b.walk_at is None
            else round(b.clock() - b.walk_at, 2),
            "acks": list(b.acks)[-5:],
        }, []


def _cmd_session_start(b: Bridge, args: Dict[str, Any]):
    with b._lock:
        created = b.session is None
        if created:
            b.session = {
                "id": uuid.uuid4().hex[:16],
                "started": _iso(b.wall()),
                "_mono": b.clock(),
            }
        sess = {k: v for k, v in b.session.items() if not k.startswith("_")}
        # Idempotent: a repeat returns the running session and announces nothing.
        return sess, (b._event("session_started", sess) if created else [])


def _cmd_session_stop(b: Bridge, args: Dict[str, Any]):
    with b._lock:
        if b.session is None:
            raise CommandError("BAD_ARGS", "no session is running")
        sess = {k: v for k, v in b.session.items() if not k.startswith("_")}
        sess["duration_s"] = round(b.clock() - b.session["_mono"], 1)
        events = b._event("session_stopped", sess)
        b.session = None
        return sess, events


_ACK_OUTCOMES = ("spoken", "haptic", "shown", "suppressed", "expired", "duplicate")


def _cmd_alert_ack(b: Bridge, args: Dict[str, Any]):
    outcome = args.get("outcome")
    if outcome not in _ACK_OUTCOMES:
        raise CommandError(
            "BAD_ARGS", f"outcome must be one of {', '.join(_ACK_OUTCOMES)}"
        )
    # Kept for get_diagnostics: every value is bounded, or five long ones would outgrow it.
    for key, top in (("epoch", 0xFFFF), ("cue_seq", 0xFFFFFFFF)):
        value = args.get(key)
        if (
            not isinstance(value, int)
            or isinstance(value, bool)
            or not 0 <= value <= top
        ):
            raise CommandError("BAD_ARGS", f"{key} must be an integer 0..{top}")
    latency = args.get("app_latency_ms")
    if latency is not None and (
        _num(latency) is None or not 0 <= latency <= MAX_APP_LATENCY_MS
    ):
        raise CommandError(
            "BAD_ARGS", f"app_latency_ms must be a number 0..{MAX_APP_LATENCY_MS}"
        )
    with b._lock:
        b.acks.append(
            {
                "epoch": args["epoch"],
                "cue_seq": args["cue_seq"],
                "outcome": outcome,
                "app_latency_ms": latency,
                "at_uptime_ms": int((b.clock() - b.started) * 1000),
            }
        )
    return {}, []  # diagnostic only: the device never waits for an ack


def _cmd_mark(b: Bridge, args: Dict[str, Any]):
    kind = args.get("kind", "note")
    if kind not in ("right", "wrong", "missed", "note"):
        raise CommandError("BAD_ARGS", "kind must be right, wrong, missed or note")
    note = str(args.get("note", ""))[:200]
    if b.service is None:
        raise CommandError("DEVICE_UNAVAILABLE", "no navigation service")
    try:
        return b.service.mark(kind, note), []
    except Exception as exc:
        raise CommandError("DEVICE_UNAVAILABLE", f"mark: {exc}")


def _cmd_set_device_settings(b: Bridge, args: Dict[str, Any]):
    raise CommandError(
        "NOT_SUPPORTED", "device settings cannot be changed over Bluetooth in v1"
    )


_COMMANDS = {
    "hello": _cmd_hello,
    "ping": _cmd_ping,
    "get_status": _cmd_get_status,
    "get_current": _cmd_get_current,
    "get_surroundings": _cmd_get_surroundings,
    "get_speech_log": _cmd_get_speech_log,
    "get_diagnostics": _cmd_get_diagnostics,
    "session_start": _cmd_session_start,
    "session_stop": _cmd_session_stop,
    "alert_ack": _cmd_alert_ack,
    "mark": _cmd_mark,
    "set_device_settings": _cmd_set_device_settings,
}
