"""Scan before you speak."""

from __future__ import annotations

import threading
import time
from typing import Any, Dict, List, Optional

from config import (
    STARTUP_LIDAR_ABSENT_S, STARTUP_OPTIONAL_WAIT_S, STARTUP_SETTLE_S, STARTUP_READY_CUE,
    STARTUP_MIN_SAMPLES, STARTUP_STABLE_FT, STARTUP_STABLE_MAX_WAIT_S,
)

# Clearance at or beyond which a direction is worth calling open, in FEET.
_OPEN_FT = 8.0
# Nearer than this and a direction is worth naming as close.
_CLOSE_FT = 3.0


def orient(clearances: Dict[str, Any]) -> str:
    """One line describing where the user is standing."""
    def ft(direction):
        v = clearances.get(direction)
        return float(v) if isinstance(v, (int, float)) else None

    front, left, right = ft("front"), ft("left"), ft("right")
    if front is None:
        return ""

    if front < 2.0:
        where = "Something right in front of you."
        named = "front"
    elif front >= _OPEN_FT:
        where = f"Open ahead, about {front:.0f} feet."
        named = None
    else:
        sides = {d: ft(d) for d in ("left", "right", "back")}
        sides = {d: v for d, v in sides.items() if v is not None}
        best = max(sides, key=sides.get) if sides else None
        if best and sides[best] >= _OPEN_FT:
            named = best
            side_words = "behind you" if best == "back" else f"to your {best}"
            where = (f"Not much room ahead, about {front:.0f} feet. "
                     f"Most space is {side_words}.")
        else:
            named = None
            where = f"Tight all round, about {front:.0f} feet ahead."

    if named is not None:
        return where
    close = ""
    if left is not None and right is not None:
        near, far = (("left", left), ("right", right)) if left < right \
            else (("right", right), ("left", left))
        if near[1] < _CLOSE_FT and far[1] - near[1] > 1.0:
            close = f" Something close on your {near[0]}."
    return where + close


class _ReadyGate:
    """Holds the voice until the sensors have settled. One-way: never re-arms."""

    def __init__(self):
        self._lock = threading.Lock()
        self._healthy_since: Optional[float] = None
        self._samples: List[float] = []      # front clearance, ft, while settling
        self._ready = False
        self._announced = False
        self._lidar_since: Optional[float] = None
        self._lidar_down_since: Optional[float] = None

    def check(self, health: Dict[str, Any],
              clearances: Optional[Dict[str, Any]] = None,
              now: Optional[float] = None) -> Dict[str, Any]:
        """Judge readiness from one health snapshot."""
        now = time.time() if now is None else now
        with self._lock:
            if self._ready:
                return {"ready": True, "cue": "", "quiet_reason": "",
                        "waited_s": None, "lidar_absent": False}

            if STARTUP_SETTLE_S <= 0:
                self._ready = True
                return {"ready": True, "cue": "", "quiet_reason": "",
                        "waited_s": 0.0, "lidar_absent": False}

            lidar_up = bool(health.get("lidar_ok"))
            if self._lidar_since is not None and now < self._lidar_since:
                self._lidar_since = now                   # clock stepped back
            if lidar_up and self._lidar_since is None:
                self._lidar_since = now
            if lidar_up:
                self._lidar_down_since = None
            elif self._lidar_down_since is None or now < self._lidar_down_since:
                self._lidar_down_since = now
            waited_for_optional = (self._lidar_since is not None and
                                   now - self._lidar_since >= STARTUP_OPTIONAL_WAIT_S)
            lidar_absent = (not lidar_up and
                            now - self._lidar_down_since >= STARTUP_LIDAR_ABSENT_S)
            missing = [name for name, ok in (
                ("lidar", lidar_up),
                ("radar", health.get("radar_ok") or waited_for_optional),
                ("camera", health.get("camera_ok") or waited_for_optional),
            ) if not ok]

            if missing:
                self._healthy_since = None
                self._samples.clear()
                return {"ready": False, "cue": "",
                        "quiet_reason": (f"scanning — waiting for {', '.join(missing)}"
                                         + (" (not up for "
                                            f"{now - self._lidar_down_since:.0f}s)"
                                            if lidar_absent else "")),
                        "waited_s": 0.0, "lidar_absent": lidar_absent}

            if self._healthy_since is None:
                self._healthy_since = now

            front = (clearances or {}).get("front")
            if isinstance(front, (int, float)):
                self._samples.append(float(front))
                del self._samples[:-30]

            waited = now - self._healthy_since
            if waited < STARTUP_SETTLE_S:
                return {"ready": False, "cue": "",
                        "quiet_reason": (f"scanning — {STARTUP_SETTLE_S - waited:.1f}s "
                                         f"more to settle"),
                        "waited_s": round(waited, 2), "lidar_absent": False}

            if len(self._samples) < STARTUP_MIN_SAMPLES:
                return {"ready": False, "cue": "",
                        "quiet_reason": (f"scanning — {len(self._samples)}/"
                                         f"{STARTUP_MIN_SAMPLES} readings so far"),
                        "waited_s": round(waited, 2), "lidar_absent": False}

            spread = max(self._samples) - min(self._samples)
            if (spread > STARTUP_STABLE_FT
                    and waited < STARTUP_SETTLE_S + STARTUP_STABLE_MAX_WAIT_S):
                self._samples = self._samples[-STARTUP_MIN_SAMPLES:]
                return {"ready": False, "cue": "",
                        "quiet_reason": (f"scanning — the view is still moving "
                                         f"({spread:.1f} ft spread)"),
                        "waited_s": round(waited, 2), "lidar_absent": False}

            self._ready = True
            if self._announced:
                cue = ""
            else:
                summary = orient(clearances or {})
                cue = f"{STARTUP_READY_CUE} {summary}".strip() if summary \
                    else STARTUP_READY_CUE
            self._announced = True
            return {"ready": True, "cue": cue, "quiet_reason": "",
                    "waited_s": round(waited, 2), "settled_ft": round(spread, 2),
                    "lidar_absent": False}

    def state(self) -> Dict[str, Any]:
        with self._lock:
            return {"ready": self._ready, "announced": self._announced,
                    "healthy_since": self._healthy_since}

    def reset(self) -> None:
        """Re-arm the hold. For tests, and for a device that has been restarted without the process restarting with it."""
        with self._lock:
            self._healthy_since = None
            self._samples.clear()
            self._ready = False
            self._announced = False
            self._lidar_since = None
            self._lidar_down_since = None


_gate = _ReadyGate()


def check(health: Optional[Dict[str, Any]] = None,
          clearances: Optional[Dict[str, Any]] = None,
          now: Optional[float] = None) -> Dict[str, Any]:
    """Readiness for the guidance path."""
    try:
        if health is None:
            from utils.sensor_fusion import get_system_health
            health = get_system_health() or {}
        if clearances is None:
            from utils import walk_policy
            from utils.shared_state import get_lidar_state, get_radar_state
            views = walk_policy.read_all(get_lidar_state(), get_radar_state())
            clearances = {d: v.get("distance_ft") for d, v in views.items()
                          if not v.get("unknown")}
        return _gate.check(health, clearances, now=now)
    except Exception as exc:
        return {"ready": True, "cue": "", "waited_s": None, "lidar_absent": False,
                "quiet_reason": f"startup gate error, allowing speech: {exc}"}


def reset() -> None:
    _gate.reset()


def state() -> Dict[str, Any]:
    return _gate.state()
