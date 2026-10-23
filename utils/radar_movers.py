"""Moving things only the radar hears — where the LiDAR is blind, or above or below its plane."""

import math
import re
import threading
import time
from typing import Any, Dict, List, Optional

import numpy as np

from config import (
    GLASS_BOARD_VIEW_MARGIN_DEG,
    LIDAR_MOVERS_HORIZON_S,
    LIDAR_MOVERS_WALK_MAX_MPS,
    LIDAR_STALE_S,
    RADAR_AGREE_MM,
    RADAR_BEAM_H_DEG,
    RADAR_END_MM,
    RADAR_MOVERS_CONFIRM_FRAMES,
    RADAR_MOVERS_GATE_M,
    RADAR_MOVERS_LOST_S,
    RADAR_MOVERS_MAX_TURN_DPS,
    RADAR_MOVERS_MIN_CLOSING_MPS,
    RADAR_MOVERS_MIN_SAMPLES,
    RADAR_MOVERS_MIN_SPAN_S,
    RADAR_MOVERS_WINDOW_S,
    RADAR_START_MM,
)

_BOARD_KEY = re.compile(r"^deg(\d{3})$")


def _signed(bearing: float) -> float:
    return (bearing + 180.0) % 360.0 - 180.0


def _theil_sen(t: np.ndarray, v: np.ndarray) -> Optional[float]:
    i, j = np.triu_indices(len(t), 1)
    dt = t[j] - t[i]
    ok = dt > 1e-3
    return float(np.median((v[j] - v[i])[ok] / dt[ok])) if ok.any() else None


def lidar_sees(blobs, bearing: float, range_m: float) -> bool:
    """Does the LiDAR have a blob (lidar_movers: a thing up to 2 m wide, never a wall) at
    this distance inside this board's beam? Then it is the LiDAR's to follow."""
    half = RADAR_BEAM_H_DEG / 2.0 + GLASS_BOARD_VIEW_MARGIN_DEG
    agree = RADAR_AGREE_MM / 1000.0
    return any(
        abs(near - range_m) <= agree and abs(_signed(float(b) - bearing)) <= half
        for b, near in blobs or ()
    )


class RadarMoverTracker:
    """Each board's echoes followed in range; the ones closing in by themselves."""

    def __init__(self):
        self._lock = threading.Lock()
        self._tracks: Dict[str, List[Dict[str, Any]]] = {}
        self._frame: Dict[str, float] = {}
        self._quiet_until = float("-inf")  # monotonic: a turn is still in the fit
        self._snap: Dict[str, Any] = self._empty()

    @staticmethod
    def _empty() -> Dict[str, Any]:
        return {"updated_mono": None, "tracks": [], "threat": None, "boards": 0}

    def reset(self) -> None:
        with self._lock:
            self.__init__()

    def update(
        self,
        radar_state: Optional[Dict[str, Any]],
        lidar_blobs=None,
        ego: Optional[Dict[str, Any]] = None,
        turn_dps: Optional[float] = None,
    ) -> None:
        """Fold in each board's newest frame. `lidar_blobs` and `ego` are lidar_movers'.

        A board ranges the nearest point of a wall inside its beam, and that point slides
        along the wall: walking, a still wall can close at up to the user's whole walking
        speed on ANY board, so an echo must close faster than that. `ego` is lidar_movers'
        estimate of the walk; without it, any walk up to LIDAR_MOVERS_WALK_MAX_MPS. A turn
        sweeps the beams across walls, so nothing is judged until it is out of the fit."""
        ego = ego or {}
        walk = (
            math.hypot(
                float(ego.get("fwd_mps") or 0.0), float(ego.get("right_mps") or 0.0)
            )
            + float(ego.get("rms_mps") or 0.0)
            if ego.get("valid")
            else LIDAR_MOVERS_WALK_MAX_MPS
        )
        mono = time.monotonic()
        if turn_dps is None or abs(float(turn_dps)) > RADAR_MOVERS_MAX_TURN_DPS:
            self._quiet_until = mono + RADAR_MOVERS_WINDOW_S
        turning = mono < self._quiet_until
        with self._lock:
            live = 0
            for key, row in ((radar_state or {}).get("sectors") or {}).items():
                m = _BOARD_KEY.match(str(key))
                if not m:
                    continue  # a cardinal, or an unmeasured board
                if not row.get("online") or row.get("degraded") or row.get("stale"):
                    self._tracks.pop(key, None)
                    self._frame.pop(key, None)
                    continue
                live += 1
                t = row.get("last_update")
                if not isinstance(t, (int, float)) or t == self._frame.get(key):
                    continue  # the same frame, read again
                if key in self._frame and not (
                    0.0 < t - self._frame[key] <= RADAR_MOVERS_LOST_S
                ):
                    self._tracks.pop(key, None)  # a gap, or the clock went backwards
                self._frame[key] = t
                peaks = [
                    p / 1000.0
                    for p in row.get("peaks_mm") or []
                    if isinstance(p, (int, float))
                    and RADAR_START_MM <= p <= RADAR_END_MM
                ]
                self._follow(key, float(m.group(1)), peaks, float(t))
            out = []
            for key, tracks in self._tracks.items():
                for tr in tracks:
                    if tr["hist"][-1][0] == self._frame.get(key):
                        self._judge(tr, walk, lidar_blobs, turning)
                        if tr["closing_frames"]:
                            out.append(tr)
            self._snap = self._snapshot(out, live, turning)

    def _follow(self, key: str, bearing: float, peaks: List[float], t: float) -> None:
        tracks = self._tracks.setdefault(key, [])
        pairs = []
        for ti, tr in enumerate(tracks):
            last_t, last_r = tr["hist"][-1]
            rate, gate = tr["rate"], RADAR_MOVERS_GATE_M
            if rate is None:  # young: its last step, and a wider gate
                (t0, r0), gate = (
                    (tr["hist"][-2] if len(tr["hist"]) > 1 else (None, None)),
                    2 * gate,
                )
                rate = (
                    (last_r - r0) / (last_t - t0)
                    if t0 is not None and last_t > t0
                    else 0.0
                )
            pred = last_r + rate * (t - last_t)
            for pi, r in enumerate(peaks):
                if abs(r - pred) <= gate:
                    pairs.append((abs(r - pred), ti, pi))
        used_t, used_p = set(), set()
        for _, ti, pi in sorted(pairs):
            if ti in used_t or pi in used_p:
                continue
            used_t.add(ti)
            used_p.add(pi)
            tracks[ti]["hist"].append((t, peaks[pi]))
        for pi, r in enumerate(peaks):
            if pi not in used_p:
                tracks.append(
                    {
                        "bearing": bearing,
                        "hist": [(t, r)],
                        "rate": None,
                        "closing_frames": 0,
                    }
                )
        kept = []
        for tr in tracks:
            if t - tr["hist"][-1][0] > RADAR_MOVERS_LOST_S:
                continue
            tr["hist"] = [h for h in tr["hist"] if t - h[0] <= RADAR_MOVERS_WINDOW_S]
            kept.append(tr)
        self._tracks[key] = kept

    @staticmethod
    def _judge(tr: Dict[str, Any], walk: float, lidar_blobs, turning: bool) -> None:
        """Closing in faster than the user's own walk can explain, where the LiDAR does not see it?"""
        h = tr["hist"]
        tr["alert"] = None
        if (
            turning
            or len(h) < RADAR_MOVERS_MIN_SAMPLES
            or h[-1][0] - h[0][0] < RADAR_MOVERS_MIN_SPAN_S - 1e-6
        ):
            tr["closing_frames"] = 0
            return
        arr = np.asarray(h, dtype=np.float64)
        rate = _theil_sen(arr[:, 0], arr[:, 1])
        if rate is None:
            return
        tr["rate"] = rate
        tr["range"] = float(np.median(arr[:, 1] - rate * (arr[:, 0] - h[-1][0])))
        tr["own_closing"] = -rate - walk
        closing = rate < 0.0 and tr["own_closing"] >= RADAR_MOVERS_MIN_CLOSING_MPS
        tr["closing_frames"] = tr["closing_frames"] + 1 if closing else 0
        if tr["closing_frames"] < RADAR_MOVERS_CONFIRM_FRAMES:
            return
        ttc = tr["range"] / -rate
        tr["lidar_sees"] = lidar_sees(lidar_blobs, tr["bearing"], tr["range"])
        if ttc > LIDAR_MOVERS_HORIZON_S or tr["lidar_sees"]:
            return
        b = _signed(tr["bearing"])
        side = "right" if b > 0 else "left"
        mode = (
            "behind" if abs(b) >= 135.0 else "toward" if abs(b) <= 45.0 else "crossing"
        )
        tr["alert"] = {
            "mode": mode,
            "side": side,
            "pass_side": side,
            "ttc_s": round(ttc, 1),
            "miss_m": None,
        }

    @staticmethod
    def _snapshot(
        closing: List[Dict[str, Any]], boards: int, turning: bool
    ) -> Dict[str, Any]:
        tracks = [
            {
                "source": "radar",
                "word": "Something",
                "bearing_deg": round(_signed(tr["bearing"]), 1),
                "distance_m": round(tr["range"], 2),
                "closing_mps": round(-tr["rate"], 2),
                "own_speed_mps": round(tr["own_closing"], 2),
                "confirmed": tr["closing_frames"] >= RADAR_MOVERS_CONFIRM_FRAMES,
                "lidar_sees": bool(tr.get("lidar_sees")),
                "alert": dict(tr["alert"]) if tr["alert"] else None,
            }
            for tr in closing
        ]
        alerts = [m for m in tracks if m["alert"]]
        threat = None
        if alerts:
            m = min(alerts, key=lambda a: a["alert"]["ttc_s"])
            threat = {**{k: v for k, v in m.items() if k != "alert"}, **m["alert"]}
        return {
            "updated_mono": time.monotonic(),
            "tracks": tracks,
            "threat": threat,
            "boards": boards,
            "paused_for_turn": turning,
        }

    def snapshot(self) -> Dict[str, Any]:
        """The newest closing echoes; empty once older than LIDAR_STALE_S."""
        with self._lock:
            snap = self._snap
        mono = snap.get("updated_mono")
        if mono is None or time.monotonic() - mono > LIDAR_STALE_S:
            return {**self._empty(), "stale": True}
        return {**snap, "stale": False}


_tracker = RadarMoverTracker()


def get_radar_movers() -> RadarMoverTracker:
    return _tracker
