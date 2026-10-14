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
import os
import threading
import time
import uuid
from collections import deque
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

from . import protocol as P

HERE = os.path.dirname(os.path.abspath(__file__))
VOCAB_PATH = os.path.join(HERE, "contract", "vocab_v1.json")

STATE_MIN_INTERVAL_S = 0.2     # STATE notifies at most 5 times a second ...
STATE_HEARTBEAT_S = 1.0        # ... and at least once a second (the heartbeat)
ALERT_MAX_AGE_MS = 2000        # = config.GUIDANCE_RESULT_STALE_S; the app drops older
THERMAL_WARN_C = 80.0          # Pi 5 firmware begins to throttle near here
MAX_WARNINGS = 8
MAX_WARNING_CHARS = 160
COMMANDS_PER_S = 10

# vcgencmd get_throttled bits
_UV_NOW, _CAPPED_NOW, _THROTTLED_NOW, _SOFT_TEMP_NOW = 1 << 0, 1 << 1, 1 << 2, 1 << 3

Out = Tuple[str, int, bytes]   # (characteristic: "alert" | "state" | "events", type, payload)


class CommandError(Exception):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


def load_vocab(path: str = VOCAB_PATH) -> Dict[str, Any]:
    with open(path, encoding="utf-8") as fh:
        reg = json.load(fh)
    by_text = {p["text"]: p for p in reg["phrases"] if not p.get("retired")}
    return {"by_text": by_text, "revision": reg["revision"], "sha": reg["sha"],
            "version": reg["version"]}


def _iso(t: float) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(t)) + f".{int(t * 1000) % 1000:03d}Z"


class _Persist:
    """device_id and epoch, kept across restarts (atomic write)."""

    def __init__(self, path: Optional[str]):
        self.path = path
        self.data = {"device_id": uuid.uuid4().hex[:16], "epoch": 0}
        if path and os.path.exists(path):
            try:
                with open(path, encoding="utf-8") as fh:
                    self.data.update(json.load(fh))
            except (OSError, ValueError):
                pass

    def save(self) -> None:
        if not self.path:
            return
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(self.data, fh)
        os.replace(tmp, self.path)


class Bridge:
    def __init__(self, vocab: Dict[str, Any], identity: Dict[str, str],
                 service: Any = None, state_path: Optional[str] = None,
                 clock: Callable[[], float] = time.monotonic,
                 wall: Callable[[], float] = time.time):
        self.vocab = vocab
        self.identity = identity
        self.service = service          # HttpSource / MockSource: speech_log(), mark()
        self.clock, self.wall = clock, wall
        self._lock = threading.RLock()
        self._persist = _Persist(state_path)
        # A new bridge process cannot know where the old cue counter was: new epoch.
        self._persist.data["epoch"] = (int(self._persist.data.get("epoch", 0)) + 1) & 0xFFFF
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
        self._state_payload = P.pack_state(self.epoch, 0, None, P.PHRASE_NONE, P.LEVEL_NONE,
                                           P.STATUS_UNREACHABLE, 0)
        self._flags: Dict[str, bool] = {}
        self._warnings: Tuple[str, ...] = ()
        self._ready_announced = False
        self.session: Optional[Dict[str, Any]] = None
        self.att_mtu = P.MIN_ATT_MTU
        self.acks: Deque[Dict[str, Any]] = deque(maxlen=50)
        self.stats = {"alerts": 0, "events": 0, "commands": 0, "command_errors": 0,
                      "poll_errors": 0, "epoch_bumps": 0}
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
        self._persist.save()
        self.stats["epoch_bumps"] += 1

    def protocol_info(self) -> Dict[str, Any]:
        """PROTOCOL_INFO's value. It is read as ONE frame, so it must stay well under the
        512-byte attribute limit: model and software strings are left to the Device
        Information characteristics and the hello result (test: long identity strings)."""
        return {
            "v": P.PROTOCOL_VERSION, "protocol": P.PROTOCOL_VERSION, "min_protocol": 1,
            "device_id": self.device_id, "mock": bool(self.identity.get("mock")),
            "vocab": {"version": self.vocab["version"], "revision": self.vocab["revision"],
                      "sha": self.vocab["sha"]},
            "epoch": self.epoch,
            "units": {"distance": "ft"},
            "limits": {"max_message_bytes": P.MAX_MESSAGE_BYTES,
                       "alert_max_age_ms": ALERT_MAX_AGE_MS,
                       "heartbeat_ms": int(STATE_HEARTBEAT_S * 1000)},
            "commands": sorted(_COMMANDS),
        }

    # --- inputs from the navigation service ---

    def on_walk(self, snap: Dict[str, Any]) -> List[Out]:
        """One /guidance/walk snapshot (latest_guidance()). Returns messages to send."""
        out: List[Out] = []
        with self._lock:
            now = self.clock()
            self.walk, self.walk_at = snap, now
            if self.status == P.STATUS_UNREACHABLE:
                self.status = P.STATUS_DOWN       # reachable again; /health will refine
                self.unreachable_reason = ""
            n = snap.get("notify") or {}
            cue = n.get("cue_seq")
            if isinstance(cue, int) and not isinstance(cue, bool):
                if self.last_cue is None:
                    self.last_cue = cue                       # first sight: learn only
                elif cue < self.last_cue:
                    self._bump_epoch()                        # the service restarted
                    self.last_cue = cue
                    out += self._event("service_restarted", {"epoch": self.epoch})
                elif cue > self.last_cue:
                    self.last_cue = cue
                    out += self._alert(snap, cue)
            out += self._maybe_state(now)
        return out

    def _alert(self, snap: Dict[str, Any], cue: int) -> List[Out]:
        n = snap.get("notify") or {}
        loop = snap.get("loop") or {}
        pv = n.get("pi_voice") or {}
        text = str(n.get("instruction") or "")
        entry = self.vocab["by_text"].get(text)
        phrase = entry["id"] if entry else (P.PHRASE_NONE if not text else P.PHRASE_UNLISTED)
        level = entry["level"] if entry else (2 if n.get("level") == "urgent" else 1)
        exact = bool(n.get("speak")) and n.get("cue_seq") == cue    # the admitting tick itself
        urgent = (n.get("level") == "urgent") if exact else bool(entry and entry["urgent"])
        flags = P.AF_ANNOUNCE
        flags |= P.AF_URGENT if urgent else 0
        flags |= P.AF_DEVICE_VOICE if (pv.get("usable") and pv.get("accepted_seq") == cue) else 0
        flags |= P.AF_AUDIO_ENABLED if pv.get("audio_enabled") else 0
        flags |= P.AF_LOOP_STALE if loop.get("stale") else 0
        age_s = loop.get("age_s")
        age_ms = None if age_s is None else float(age_s) * 1000.0
        front = (snap.get("path") or {}).get("front_ft")
        record = P.pack_alert(self.epoch, cue, age_ms, phrase, level, flags, front)
        self.last_alert = record
        self.stats["alerts"] += 1
        out: List[Out] = [("alert", P.T_ALERT, record)]
        if text:
            out.append(("alert", P.T_ALERT_TEXT, P.pack_alert_text(self.epoch, cue, text)))
        return out

    def on_health(self, health: Dict[str, Any], sys_health: Optional[Dict[str, Any]] = None,
                  system: Optional[Dict[str, Any]] = None) -> List[Out]:
        """/health (+ /guidance/status, + the Pi's own temperature/throttle flags)."""
        out: List[Out] = []
        with self._lock:
            self.health = health or {}
            self.sys_health = sys_health or {}
            self.system = system or {}
            self.status = {"ok": P.STATUS_OK, "degraded": P.STATUS_DEGRADED}.get(
                self.health.get("status"), P.STATUS_DOWN)
            self.unreachable_reason = ""
            flags = self._health_flags()
            prev = self._flags
            for name in ("lidar_ok", "radar_ok", "camera_ok", "hailo_ok", "front_glass_ok"):
                if name in prev and prev[name] != flags[name]:
                    sensor = name[:-3]
                    out += self._event("sensor_recovered" if flags[name] else "sensor_degraded",
                                       {"sensor": sensor})
            if "guiding" in prev and prev["guiding"] != flags["guiding"]:
                out += self._event("guidance_resumed" if flags["guiding"] else "guidance_stalled",
                                   {"reason": (self.health.get("guidance_loop") or {}).get("error", "")})
            if flags["thermal_warn"] != prev.get("thermal_warn", False):
                out += self._event("thermal_warning" if flags["thermal_warn"] else "thermal_ok",
                                   self._thermal())
            if flags["undervoltage"] and not prev.get("undervoltage", False):
                out += self._event("power_warning", {"undervoltage": True, **self._thermal()})
            warnings = tuple(str(w)[:MAX_WARNING_CHARS]
                             for w in (self.health.get("warnings") or [])[:MAX_WARNINGS])
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
        with self._lock:
            self.stats["poll_errors"] += 1
            if self.status != P.STATUS_UNREACHABLE:
                self.status = P.STATUS_UNREACHABLE
                self.unreachable_reason = reason
                self._ready_announced = False
                out += self._event("service_unreachable", {"reason": reason[:MAX_WARNING_CHARS]})
            out += self._maybe_state(self.clock())
        return out

    def tick(self, force: bool = False) -> List[Out]:
        """Called about every 100 ms: the heartbeat, even when nothing changes.
        force=True sends STATE now, whatever the timers say (a phone just subscribed)."""
        with self._lock:
            return self._maybe_state(self.clock(), force=force)

    # --- STATE ---

    def _health_flags(self) -> Dict[str, bool]:
        sensors = self.health.get("sensors") or {}
        pv = ((self.walk or {}).get("notify") or {}).get("pi_voice") or {}
        loop = (self.walk or {}).get("loop") or {}
        throttled = int(self.system.get("throttled") or 0)
        soc = self.system.get("soc_c")
        warnings = " ".join(str(w) for w in (self.health.get("warnings") or []))
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
            "simulated_input": bool(self.identity.get("mock")) or "SIMULATED" in warnings,
            "thermal_warn": bool((soc is not None and soc >= THERMAL_WARN_C)
                                 or throttled & (_CAPPED_NOW | _THROTTLED_NOW | _SOFT_TEMP_NOW)),
            "undervoltage": bool(throttled & _UV_NOW),
            "loop_stale": bool(loop.get("stale", True)),
        }

    def _health_bits(self) -> int:
        if self.status == P.STATUS_UNREACHABLE:
            return 0
        flags = self._health_flags()
        return sum(P.HEALTH_BITS[k] for k, v in flags.items() if v)

    def _maybe_state(self, now: float, force: bool = False) -> List[Out]:
        path = (self.walk or {}).get("path") or {}
        loop = (self.walk or {}).get("loop") or {}
        text = str(path.get("instruction") or "") if self.status != P.STATUS_UNREACHABLE else ""
        entry = self.vocab["by_text"].get(text)
        phrase = entry["id"] if entry else (P.PHRASE_NONE if not text else P.PHRASE_UNLISTED)
        level = entry["level"] if entry else P.LEVEL_NONE
        bits = self._health_bits()
        key = (self.epoch, self.last_cue, phrase, level, self.status, bits)
        due = now - self._state_sent >= STATE_HEARTBEAT_S
        changed = key != self._state_key and now - self._state_sent >= STATE_MIN_INTERVAL_S
        if not (due or changed or force):
            return []
        age_s = loop.get("age_s")
        if self.walk_at is not None and age_s is not None:
            age_s = float(age_s) + (now - self.walk_at)      # how old the decision is NOW
        self._state_payload = P.pack_state(self.epoch, self.last_cue or 0,
                                           None if age_s is None else age_s * 1000.0,
                                           phrase, level, self.status, bits)
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
            rec[11] |= P.AF_REPLAY            # flags byte of the record
            return bytes(rec)

    # --- events ---

    def _event(self, name: str, data: Dict[str, Any]) -> List[Out]:
        self.stats["events"] += 1
        msg = {"v": P.PROTOCOL_VERSION, "event": name, "ts": _iso(self.wall()),
               "uptime_ms": int((self.clock() - self.started) * 1000),
               "session_id": (self.session or {}).get("id"), "data": data}
        return [("events", P.T_EVENT, P.json_bytes(msg))]

    def _thermal(self) -> Dict[str, Any]:
        return {"soc_c": self.system.get("soc_c"),
                "throttled": f"0x{int(self.system.get('throttled') or 0):x}"}

    def _health_summary(self) -> Dict[str, Any]:
        sensors = self.health.get("sensors") or {}
        return {"status": P.STATUS_NAMES[self.status],
                "guiding": bool(self.health.get("guiding")),
                "sensors": {k[:-3]: bool(v) for k, v in sensors.items()},
                "warnings": list(self._warnings)}

    # --- connection lifecycle ---

    def on_connect(self) -> List[Out]:
        with self._lock:
            self.att_mtu = P.MIN_ATT_MTU
            return []

    def on_disconnect(self) -> None:
        with self._lock:
            self.att_mtu = P.MIN_ATT_MTU

    # --- commands ---

    def on_command(self, payload: bytes) -> List[Out]:
        """A reassembled COMMAND message -> its RESPONSE (and any events)."""
        cmd_id: Any = None
        try:
            msg = P.parse_json(payload)
            cmd_id = msg.get("id")
            if msg.get("v") != P.PROTOCOL_VERSION:
                raise CommandError("PROTOCOL_MISMATCH", f"command v={msg.get('v')}, device speaks v1")
            if not isinstance(cmd_id, int) or isinstance(cmd_id, bool) or not 0 <= cmd_id <= 0xFFFFFFFF:
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
                if len(self._cmd_times) == COMMANDS_PER_S and now - self._cmd_times[0] < 1.0:
                    raise CommandError("BUSY", f"more than {COMMANDS_PER_S} commands a second")
                self._cmd_times.append(now)
                self.stats["commands"] += 1
            result, extra = handler(self, args)
            body = {"v": P.PROTOCOL_VERSION, "id": cmd_id, "ok": True, "result": result}
            return [("events", P.T_RESPONSE, P.json_bytes(body))] + extra
        except (CommandError, P.ProtocolError) as exc:
            code, message = exc.code, str(exc)
        except Exception as exc:                     # never let a command kill the bridge
            code, message = "INTERNAL", f"{type(exc).__name__}: {exc}"
        with self._lock:
            self.stats["command_errors"] += 1
        body = {"v": P.PROTOCOL_VERSION, "id": cmd_id if isinstance(cmd_id, int) else None,
                "ok": False, "error": {"code": code, "message": message[:200]}}
        return [("events", P.T_RESPONSE, P.json_bytes(body))]

    def bad_frame(self, code: str, message: str) -> List[Out]:
        """A COMMAND write that could not be reassembled: answer with id null."""
        body = {"v": P.PROTOCOL_VERSION, "id": None, "ok": False,
                "error": {"code": code, "message": message[:200]}}
        return [("events", P.T_RESPONSE, P.json_bytes(body))]


# --- command handlers: (bridge, args) -> (result, extra messages) ---

def _cmd_hello(b: Bridge, args: Dict[str, Any]):
    proto = args.get("protocol", 1)
    if proto != P.PROTOCOL_VERSION:
        raise CommandError("PROTOCOL_MISMATCH", f"app speaks v{proto}, device speaks v1")
    mtu = args.get("att_mtu")
    if mtu is not None:
        if not isinstance(mtu, int) or not P.MIN_ATT_MTU <= mtu <= 517:
            raise CommandError("BAD_ARGS", "att_mtu must be an integer 23..517")
        with b._lock:
            b.att_mtu = mtu
    with b._lock:
        info = b.protocol_info()
        info["model"] = b.identity.get("model", "")
        info["software"] = b.identity.get("software", "")
        info["session"] = ({k: v for k, v in b.session.items() if not k.startswith("_")}
                           if b.session else None)
        info["status"] = P.STATUS_NAMES[b.status]
        info["att_mtu"] = b.att_mtu
    return info, []


def _cmd_ping(b: Bridge, args: Dict[str, Any]):
    return {"t": args.get("t"), "uptime_ms": int((b.clock() - b.started) * 1000)}, []


def _cmd_get_status(b: Bridge, args: Dict[str, Any]):
    with b._lock:
        h = b.health
        return {
            "status": P.STATUS_NAMES[b.status],
            "unreachable_reason": b.unreachable_reason or None,
            "guiding": bool(h.get("guiding")),
            "sensors": {k[:-3]: bool(v) for k, v in (h.get("sensors") or {}).items()},
            "front_glass_ok": bool(b.sys_health.get("front_glass_ok")),
            "radar_down": list(h.get("radar_down") or [])[:6],
            "warnings": list(b._warnings),
            "loop": {k: (h.get("guidance_loop") or {}).get(k)
                     for k in ("running", "last_ok_age_s", "error")},
            "voice": {k: (((b.walk or {}).get("notify") or {}).get("pi_voice") or {}).get(k)
                      for k in ("audio_enabled", "usable", "output", "error")},
            "system": b._thermal(),
            "battery": None,            # NOT IMPLEMENTED: no battery gauge on this device
        }, []


def _cmd_get_current(b: Bridge, args: Dict[str, Any]):
    with b._lock:
        p = (b.walk or {}).get("path") or {}
        loop = (b.walk or {}).get("loop") or {}
        text = str(p.get("instruction") or "")
        entry = b.vocab["by_text"].get(text) or {}
        return {"instruction": text, "phrase_id": entry.get("id", 0 if not text else 0xFFFF),
                "action": p.get("action"), "rule": p.get("rule"), "side": p.get("turn_side"),
                "obstacle": p.get("obstacle"), "front_ft": p.get("front_ft"),
                "glass_ahead": p.get("glass_ahead"), "reason": str(p.get("reason") or "")[:200],
                "age_s": loop.get("age_s"), "stale": loop.get("stale", True)}, []


def _cmd_get_surroundings(b: Bridge, args: Dict[str, Any]):
    with b._lock:
        obs = (b.walk or {}).get("obstacles") or {}
        items = [{k: o.get(k) for k in ("object", "clock", "sector", "distance_ft",
                                         "urgency", "source", "shape")}
                 for o in (obs.get("items") or [])[:6]]
        loop = (b.walk or {}).get("loop") or {}
        return {"level": obs.get("level"), "summary": obs.get("summary"), "items": items,
                "age_s": loop.get("age_s"), "stale": loop.get("stale", True)}, []


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
    entries = [{k: e.get(k) for k in keep} for e in (log.get("lines") or [])][-limit:]
    return {"entries": entries}, []


def _cmd_get_diagnostics(b: Bridge, args: Dict[str, Any]):
    with b._lock:
        return {"stats": dict(b.stats), "epoch": b.epoch, "last_cue": b.last_cue,
                "att_mtu": b.att_mtu,
                "walk_age_s": None if b.walk_at is None else round(b.clock() - b.walk_at, 2),
                "acks": list(b.acks)[-5:]}, []


def _cmd_session_start(b: Bridge, args: Dict[str, Any]):
    with b._lock:
        created = b.session is None
        if created:
            b.session = {"id": uuid.uuid4().hex[:16], "started": _iso(b.wall()),
                         "_mono": b.clock()}
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
        raise CommandError("BAD_ARGS", f"outcome must be one of {', '.join(_ACK_OUTCOMES)}")
    for key in ("epoch", "cue_seq"):
        if not isinstance(args.get(key), int):
            raise CommandError("BAD_ARGS", f"{key} must be an integer")
    with b._lock:
        b.acks.append({"epoch": args["epoch"], "cue_seq": args["cue_seq"], "outcome": outcome,
                       "app_latency_ms": args.get("app_latency_ms"),
                       "at_uptime_ms": int((b.clock() - b.started) * 1000)})
    return {}, []          # diagnostic only: the device never waits for an ack


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
    raise CommandError("NOT_SUPPORTED", "device settings cannot be changed over Bluetooth in v1")


_COMMANDS = {
    "hello": _cmd_hello, "ping": _cmd_ping, "get_status": _cmd_get_status,
    "get_current": _cmd_get_current, "get_surroundings": _cmd_get_surroundings,
    "get_speech_log": _cmd_get_speech_log, "get_diagnostics": _cmd_get_diagnostics,
    "session_start": _cmd_session_start, "session_stop": _cmd_session_stop,
    "alert_ack": _cmd_alert_ack, "mark": _cmd_mark,
    "set_device_settings": _cmd_set_device_settings,
}
