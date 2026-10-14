"""Where the bridge's data comes from: the live navigation service, or a mock.

PROTOTYPE. Both sources offer the same methods, so the GATT server cannot tell
them apart — which is what lets a mobile developer work against `--mock` with no
LiDAR, camera or radar attached (docs/05, "Developing without the hardware").
"""

import json
import subprocess
import time
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional

_THERMAL_ZONE = "/sys/class/thermal/thermal_zone0/temp"
_THROTTLE_CACHE_S = 5.0
_throttle_cache = {"at": -1e9, "value": 0}


def read_system() -> Dict[str, Any]:
    """The Pi's SoC temperature (degC) and `vcgencmd get_throttled` bits; None when unreadable."""
    soc = None
    try:
        with open(_THERMAL_ZONE) as fh:
            soc = round(int(fh.read().strip()) / 1000.0, 1)
    except (OSError, ValueError):
        pass
    now = time.monotonic()
    if now - _throttle_cache["at"] >= _THROTTLE_CACHE_S:
        _throttle_cache["at"] = now
        try:
            out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                                 text=True, timeout=2).stdout
            _throttle_cache["value"] = int(out.strip().split("=", 1)[1], 16)
        except Exception:
            _throttle_cache["value"] = 0
    return {"soc_c": soc, "throttled": _throttle_cache["value"]}


class HttpSource:
    """The running navigation service, over loopback HTTP (read-only, plus /guidance/mark)."""

    def __init__(self, base: str = "http://127.0.0.1:8000", timeout: float = 0.5):
        self.base = base.rstrip("/")
        self.timeout = timeout

    def _request(self, path: str, body: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.base + path, data=data,
                                     headers={"Accept": "application/json",
                                              **({"Content-Type": "application/json"} if data else {})})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return json.loads(resp.read().decode())
        except urllib.error.HTTPError as exc:
            if exc.code == 503 and path == "/health":     # "down" still carries a body
                return json.loads(exc.read().decode())
            raise

    def walk(self) -> Dict[str, Any]:
        return self._request("/guidance/walk")

    def health(self) -> Dict[str, Any]:
        return self._request("/health")

    def system_health(self) -> Dict[str, Any]:
        return self._request("/guidance/status")

    def system(self) -> Dict[str, Any]:
        return read_system()

    def speech_log(self, limit: int) -> Dict[str, Any]:
        return self._request(f"/guidance/log?limit={int(limit)}")

    def mark(self, kind: str, note: str) -> Dict[str, Any]:
        return self._request("/guidance/mark", {"kind": kind, "note": note})


# --- the mock device -------------------------------------------------------

def make_walk(cue: int, instruction: str = "", announced: str = "", speak: bool = False,
              urgent: bool = False, front_ft: Optional[float] = None, stale: bool = False,
              audio_enabled: bool = True, device_voice_usable: bool = False,
              accepted_seq: Optional[int] = None, obstacles: Optional[List[Dict]] = None,
              age_s: float = 0.05) -> Dict[str, Any]:
    """A /guidance/walk snapshot with the fields the bridge reads, shaped as latest_guidance()."""
    return {
        "notify": {"speak": speak, "cue_seq": cue, "instruction": announced,
                   "level": "urgent" if urgent else "advisory", "source": "simple-guidance",
                   "reason": "", "quiet_reason": "" if speak else "nothing new to say",
                   "pi_voice": {"enabled": True, "audio_enabled": audio_enabled,
                                "running": device_voice_usable, "usable": device_voice_usable,
                                "sink_usable": device_voice_usable,
                                "accepted_seq": accepted_seq, "output": None, "error": ""}},
        "path": {"instruction": instruction, "front_ft": front_ft, "action": "",
                 "rule": "", "reason": "mock", "turn_side": None, "obstacle": None},
        "obstacles": {"level": "clear", "summary": "", "count": len(obstacles or []),
                      "items": obstacles or []},
        "loop": {"age_s": None if stale else age_s, "stale": stale},
    }


def make_health(status: str = "ok", lidar: bool = True, radar: bool = True,
                camera: bool = True, hailo: bool = True, warnings: Optional[List[str]] = None,
                guiding: bool = True) -> Dict[str, Any]:
    """As main.health_check: `guiding` is the guidance LOOP ticking (it keeps ticking
    when the LiDAR is lost — it is what says "Sensors offline"); a lost LiDAR shows as
    status "down" with lidar_ok False."""
    return {"status": status, "guiding": guiding,
            "sensors": {"lidar_ok": lidar, "radar_ok": radar, "camera_ok": camera,
                        "hailo_ok": hailo},
            "radar_down": [] if radar else ["front", "left", "right", "back"],
            "warnings": warnings or [],
            "guidance_loop": {"running": True, "last_ok_age_s": 0.1, "error": ""},
            "service": {}}


# Each step: (seconds into the scenario, what changes). "say" steps the cue.
SCENARIOS: Dict[str, List[Dict[str, Any]]] = {
    "sidewalk": [
        {"at": 0.0, "now": "Walk straight", "say": "Walk straight", "front_ft": 30.0},
        {"at": 4.0, "now": "Person ahead", "say": "Person ahead", "front_ft": 11.0,
         "obstacles": [{"object": "person", "clock": 12, "sector": "front",
                        "distance_ft": 11.0, "urgency": "clear", "source": "both"}]},
        {"at": 8.0, "now": "Person in front, turn left", "say": "Person in front, turn left",
         "urgent": True, "front_ft": 3.5,
         "obstacles": [{"object": "person", "clock": 12, "sector": "front",
                        "distance_ft": 3.5, "urgency": "warning", "source": "both"}]},
        {"at": 12.0, "now": "Walk straight", "say": "Walk straight", "front_ft": 25.0},
        {"at": 16.0, "now": "Pole ahead, turn slightly right", "say": "Pole ahead, turn slightly right",
         "front_ft": 8.0},
        {"at": 20.0, "now": "Vehicle approaching, move right", "say": "Vehicle approaching, move right",
         "urgent": True, "front_ft": 20.0},
        {"at": 24.0, "now": "Walk straight", "say": "Walk straight", "front_ft": 30.0},
    ],
    "glass_door": [
        {"at": 0.0, "now": "Walk straight", "say": "Walk straight", "front_ft": 14.0},
        {"at": 3.0, "now": "Glass ahead", "say": "Glass ahead", "front_ft": 9.0},
        {"at": 6.0, "now": "Blocked by glass", "say": "Blocked by glass", "urgent": True,
         "front_ft": 3.6},
        {"at": 9.0, "now": "Turn right", "say": "Turn right", "urgent": True, "front_ft": 3.6},
        {"at": 13.0, "now": "Walk straight", "say": "Walk straight", "front_ft": 22.0},
    ],
    "sensor_failure": [
        {"at": 0.0, "now": "Walk straight", "say": "Walk straight", "front_ft": 20.0},
        {"at": 4.0, "health": {"status": "degraded", "radar": False,
                               "warnings": ["Radar offline — glass may not be detected"]},
         "front_glass_ok": False},
        {"at": 7.0, "now": "Walk straight", "say": "Glass sensor off"},
        {"at": 11.0, "health": {"status": "down", "lidar": False, "radar": False,
                                "warnings": ["LiDAR signal lost — rely on your cane"]},
         "now": "Sensors offline", "say": "Sensors offline"},
        {"at": 17.0, "health": {"status": "ok"}, "front_glass_ok": True,
         "now": "Walk straight", "say": "Walk straight", "front_ft": 20.0},
    ],
    "service_crash": [
        {"at": 0.0, "now": "Walk straight", "say": "Walk straight", "front_ft": 20.0},
        {"at": 4.0, "unreachable": "connection refused"},
        {"at": 10.0, "restart": True, "now": "", "health": {"status": "down", "lidar": False}},
        {"at": 13.0, "health": {"status": "ok"}, "now": "Walk straight", "say": "Walk straight",
         "front_ft": 20.0},
    ],
    "thermal": [
        {"at": 0.0, "now": "Walk straight", "say": "Walk straight", "front_ft": 20.0},
        {"at": 4.0, "system": {"soc_c": 82.5, "throttled": 0x6}},
        {"at": 10.0, "system": {"soc_c": 61.0, "throttled": 0x0}},
    ],
}


class MockSource:
    """Replays a scripted scenario on a loop, with the same methods as HttpSource."""

    def __init__(self, scenario: str = "sidewalk", clock=time.monotonic,
                 audio_enabled: bool = True, device_voice: bool = False, loop_s: float = 30.0):
        if scenario not in SCENARIOS:
            raise ValueError(f"unknown scenario {scenario!r}; one of {', '.join(SCENARIOS)}")
        self.steps = SCENARIOS[scenario]
        self.clock = clock
        self.audio_enabled, self.device_voice = audio_enabled, device_voice
        self.loop_s = max(loop_s, self.steps[-1]["at"] + 4.0)
        self.t0 = clock()
        self.cue = 7                 # an arbitrary start: the bridge must only learn it
        self._applied = -1
        self._cycle = -1
        self._said_at = -1e9
        self._marks: List[Dict[str, Any]] = []
        self._reset_world()

    def _reset_world(self) -> None:
        self.now_text, self.announced, self.front_ft = "", "", None
        self.urgent, self.obstacles = False, []
        self.health_kw: Dict[str, Any] = {"status": "ok"}
        self.front_glass_ok, self.unreachable = True, ""
        self.sys = {"soc_c": 55.0, "throttled": 0}

    def _advance(self) -> None:
        elapsed = self.clock() - self.t0
        cycle, t = int(elapsed // self.loop_s), elapsed % self.loop_s
        if cycle != self._cycle:
            self._cycle, self._applied = cycle, -1
            self._reset_world()
        for i, step in enumerate(self.steps):
            if i <= self._applied or step["at"] > t:
                continue
            self._applied = i
            self.unreachable = step.get("unreachable", "")
            if step.get("restart"):
                self.cue = 0                     # a restarted service counts from zero
            if "health" in step:
                self.health_kw = dict(step["health"])
            if "front_glass_ok" in step:
                self.front_glass_ok = step["front_glass_ok"]
            if "system" in step:
                self.sys = dict(step["system"])
            if "now" in step:
                self.now_text = step["now"]
                self.front_ft = step.get("front_ft")
                self.obstacles = step.get("obstacles", [])
            if "say" in step:
                self.cue += 1
                self.announced, self.urgent = step["say"], bool(step.get("urgent"))
                self._said_at = self.clock()

    def _check(self) -> None:
        self._advance()
        if self.unreachable:
            raise ConnectionError(self.unreachable)

    def walk(self) -> Dict[str, Any]:
        self._check()
        speaking = self.clock() - self._said_at < 0.25     # the admitting tick, roughly
        return make_walk(self.cue, self.now_text, self.announced, speak=speaking,
                         urgent=self.urgent, front_ft=self.front_ft,
                         audio_enabled=self.audio_enabled,
                         device_voice_usable=self.device_voice,
                         accepted_seq=self.cue if self.device_voice else None,
                         obstacles=self.obstacles)

    def health(self) -> Dict[str, Any]:
        self._check()
        return make_health(**self.health_kw)

    def system_health(self) -> Dict[str, Any]:
        self._check()
        return {"front_glass_ok": self.front_glass_ok}

    def system(self) -> Dict[str, Any]:
        return dict(self.sys)

    def speech_log(self, limit: int) -> Dict[str, Any]:
        self._check()
        return {"lines": [{"time": time.strftime("%H:%M:%S"), "source": "simple-guidance",
                           "level": "urgent" if self.urgent else "advisory",
                           "text": self.announced, "cue_seq": self.cue, "via": "none",
                           "outcome": "muted"}][-limit:]}

    def mark(self, kind: str, note: str) -> Dict[str, Any]:
        self._check()
        self._marks.append({"kind": kind, "note": note})
        return {"ok": True, "kind": kind, "last_said": self.announced or "nothing yet"}
