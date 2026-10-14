"""Moving things in the LiDAR scan — what is coming at the user, from which side, and when."""

import math
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from config import (
    LIDAR_MOVERS_BREAK_MM, LIDAR_MOVERS_BREAK_PER_M, LIDAR_MOVERS_CONFIRM_SCANS,
    LIDAR_MOVERS_EGO_MAX_RMS_MPS, LIDAR_MOVERS_EGO_MIN_TRACKS, LIDAR_MOVERS_GATE_M,
    LIDAR_MOVERS_HEAD_ON_DEG, LIDAR_MOVERS_HORIZON_S, LIDAR_MOVERS_LOST_S,
    LIDAR_MOVERS_MAX_GAP_DEG, LIDAR_MOVERS_MAX_RANGE_MM, LIDAR_MOVERS_MAX_WIDTH_M,
    LIDAR_MOVERS_MIN_POINTS, LIDAR_MOVERS_MIN_RANGE_MM, LIDAR_MOVERS_MIN_SAMPLES,
    LIDAR_MOVERS_MIN_SPAN_S, LIDAR_MOVERS_MIN_SPEED_MPS, LIDAR_MOVERS_MISS_M,
    LIDAR_MOVERS_NOISE_PER_M, LIDAR_MOVERS_PERSON_MAX_WIDTH_M,
    LIDAR_MOVERS_VEHICLE_SPEED_MPS, LIDAR_MOVERS_WALK_MAX_MPS, LIDAR_MOVERS_WINDOW_S,
    LIDAR_STALE_S, SIMPLE_VEHICLE_MISS_M,
)

_OCCLUDER_M = 0.2          # a neighbour this much nearer hides that end of a blob
_EGO_MAX_FWD_MPS = 2.5     # a fitted user motion faster than a walk is a bad fit
_EGO_MAX_SIDE_MPS = 1.5
_EGO_MIN_SPREAD_M = 1.0    # still things this far apart pin down a turn


def segment(points) -> List[Dict[str, Any]]:
    """Blobs in one scan, in metres (x ahead, y to the right).

    `clean` is False when something nearer hides either end: the blob's visible part
    then changes with the view, not with its own motion, so it is not followed."""
    pts = sorted((float(a) % 360.0, float(d) / 1000.0) for a, d in points
                 if isinstance(d, (int, float)) and d > 0)
    n = len(pts)
    if n < LIDAR_MOVERS_MIN_POINTS:
        return []
    ang = [math.radians(a) for a, _ in pts]
    rng = [r for _, r in pts]
    xs = [r * math.cos(a) for a, r in zip(ang, rng)]
    ys = [r * math.sin(a) for a, r in zip(ang, rng)]

    def gap_deg(i: int, j: int) -> float:
        return (pts[j][0] - pts[i][0]) % 360.0

    def joined(i: int, j: int) -> bool:
        if gap_deg(i, j) > LIDAR_MOVERS_MAX_GAP_DEG:
            return False
        reach = LIDAR_MOVERS_BREAK_MM / 1000.0 + LIDAR_MOVERS_BREAK_PER_M * min(rng[i], rng[j])
        return math.hypot(xs[j] - xs[i], ys[j] - ys[i]) <= reach

    runs: List[List[int]] = [[0]]
    for i in range(1, n):
        if joined(i - 1, i):
            runs[-1].append(i)
        else:
            runs.append([i])
    if len(runs) > 1 and joined(n - 1, 0):        # one blob across straight ahead
        runs[0] = runs.pop() + runs[0]

    def hides(neighbour: int, gap: float, end: int) -> bool:
        """Does the return just past a blob's end lie nearer, close enough in angle to hide it?"""
        return gap <= LIDAR_MOVERS_MAX_GAP_DEG and rng[neighbour] < rng[end] - _OCCLUDER_M

    out = []
    for run in runs:
        if len(run) < LIDAR_MOVERS_MIN_POINTS or len(run) >= n:
            continue
        first, last = run[0], run[-1]
        before, after = (first - 1) % n, (last + 1) % n
        width = math.hypot(xs[last] - xs[first], ys[last] - ys[first])
        cx = sum(xs[i] for i in run) / len(run)
        cy = sum(ys[i] for i in run) / len(run)
        near = min(rng[i] for i in run)
        if width > LIDAR_MOVERS_MAX_WIDTH_M:
            continue
        if not (LIDAR_MOVERS_MIN_RANGE_MM / 1000.0 <= near
                and math.hypot(cx, cy) <= LIDAR_MOVERS_MAX_RANGE_MM / 1000.0):
            continue
        clean = not (hides(before, gap_deg(before, first), first)
                     or hides(after, gap_deg(last, after), last))
        out.append({"x": cx, "y": cy, "width": width, "points": len(run),
                    "near_m": near, "clean": clean})
    return out


def _theil_sen(t: np.ndarray, v: np.ndarray) -> Optional[float]:
    """Median of the pairwise slopes: a speed that one bad match cannot make."""
    i, j = np.triu_indices(len(t), 1)
    dt = t[j] - t[i]
    ok = dt > 1e-3
    if not ok.any():
        return None
    return float(np.median((v[j] - v[i])[ok] / dt[ok]))


def _to_segment_dist(vx: float, vy: float, length: float) -> float:
    """Distance from velocity (vx, vy) to the line (0,0)..(-length, 0): the least
    speed of its own a thing must have, if the user may be walking ahead at up to
    `length` m/s (which moves every still thing back at that speed)."""
    along = min(max(-vx, 0.0), length)
    return math.hypot(vx + along, vy)


class MoverTracker:
    """Blobs followed scan to scan; the ones that move by themselves, and where to."""

    def __init__(self):
        self._lock = threading.Lock()
        self._tracks: List[Dict[str, Any]] = []
        self._next_id = 1
        self._last_t: Optional[float] = None
        self._snap: Dict[str, Any] = self._empty()

    @staticmethod
    def _empty() -> Dict[str, Any]:
        return {"t": None, "updated_mono": None, "tracks": [], "threat": None,
                "ego": {"valid": False}, "blobs": [], "followed": 0}

    def reset(self) -> None:
        with self._lock:
            self.__init__()

    # ---- one scan ---------------------------------------------------------

    def update(self, points, t: float, heading: Optional[Dict[str, Any]] = None) -> None:
        """Fold one scan in. `heading` is heading_tracker's state for this scan."""
        blobs = segment(points)
        with self._lock:
            if self._last_t is not None and not (0.0 < t - self._last_t <= LIDAR_MOVERS_LOST_S):
                self._tracks = []                 # a gap, or the clock went backwards
            self._last_t = t
            yaw = (heading or {}).get("scan_yaw_deg")
            self._turn(math.radians(float(yaw)) if isinstance(yaw, (int, float)) else 0.0)
            self._follow([b for b in blobs if b["clean"]], t)
            fitted = self._fit(t)
            ego = self._ego(fitted)
            for tr in fitted:
                self._judge(tr, ego)
            # A track not seen on this scan keeps its judgement, but reports nothing.
            self._snap = self._snapshot(t, ego, blobs, fitted)

    def _turn(self, yaw: float) -> None:
        """The user turned `yaw` radians to the right since the last scan: every
        remembered position swings left by as much, into this scan's frame."""
        if abs(yaw) < 1e-9:
            return
        c, s = math.cos(yaw), math.sin(yaw)
        for tr in self._tracks:
            tr["hist"] = [(ts, x * c + y * s, y * c - x * s, w) for ts, x, y, w in tr["hist"]]
            if tr["v"] is not None:
                vx, vy = tr["v"]
                tr["v"] = (vx * c + vy * s, vy * c - vx * s)

    def _follow(self, blobs: List[Dict[str, Any]], t: float) -> None:
        """Match this scan's blobs to the tracks: nearest to where each was heading."""
        pairs = []
        for ti, tr in enumerate(self._tracks):
            _, x, y, w = tr["hist"][-1]
            dt = t - tr["hist"][-1][0]
            vx, vy = tr["v"] or (0.0, 0.0)
            px, py = x + vx * dt, y + vy * dt
            gate = LIDAR_MOVERS_GATE_M * (1.0 if tr["v"] is not None else 2.0)
            for bi, b in enumerate(blobs):
                d = math.hypot(b["x"] - px, b["y"] - py)
                if d <= gate and abs(b["width"] - w) <= max(0.4, 0.5 * w):
                    pairs.append((d, ti, bi))
        used_t, used_b = set(), set()
        for _, ti, bi in sorted(pairs):
            if ti in used_t or bi in used_b:
                continue
            used_t.add(ti)
            used_b.add(bi)
            b = blobs[bi]
            self._tracks[ti]["hist"].append((t, b["x"], b["y"], b["width"]))
        for bi, b in enumerate(blobs):
            if bi not in used_b:
                self._tracks.append({"id": self._next_id, "hist": [(t, b["x"], b["y"], b["width"])],
                                     "v": None, "moving_scans": 0, "moving": False,
                                     "alert": None})
                self._next_id += 1
        kept = []
        for tr in self._tracks:
            if t - tr["hist"][-1][0] > LIDAR_MOVERS_LOST_S:
                continue
            tr["hist"] = [h for h in tr["hist"] if t - h[0] <= LIDAR_MOVERS_WINDOW_S]
            kept.append(tr)
        self._tracks = kept

    def _fit(self, t: float) -> List[Dict[str, Any]]:
        """Speed and position now for every track seen on this scan with enough history."""
        fitted = []
        for tr in self._tracks:
            h = tr["hist"]
            if (h[-1][0] != t or len(h) < LIDAR_MOVERS_MIN_SAMPLES
                    or t - h[0][0] < LIDAR_MOVERS_MIN_SPAN_S - 1e-6):
                continue
            arr = np.asarray(h, dtype=np.float64)
            ts, xs, ys = arr[:, 0], arr[:, 1], arr[:, 2]
            vx, vy = _theil_sen(ts, xs), _theil_sen(ts, ys)
            if vx is None or vy is None:
                continue
            tr["v"] = (vx, vy)
            tr["x"] = float(np.median(xs - vx * (ts - t)))
            tr["y"] = float(np.median(ys - vy * (ts - t)))
            tr["width"] = float(np.median(arr[:, 3]))
            fitted.append(tr)
        return fitted

    @staticmethod
    def _ego(fitted: List[Dict[str, Any]]) -> Dict[str, Any]:
        """The user's own motion, from the things that are standing still.

        Every still thing moves the same way relative to the user, (-u_fwd + w*y,
        -u_right - w*x) for a walk u and a turn w; things moving by themselves do not
        fit that and are trimmed out. Needs EGO_MIN_TRACKS spread-out still things."""
        if len(fitted) < max(2, LIDAR_MOVERS_EGO_MIN_TRACKS):
            return {"valid": False, "why": f"{len(fitted)} things followed"}
        pos = np.array([(tr["x"], tr["y"]) for tr in fitted])
        vel = np.array([tr["v"] for tr in fitted])
        spread = float(np.max(np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)))
        with_turn = spread >= _EGO_MIN_SPREAD_M

        def solve(mask):
            p, v = pos[mask], vel[mask]
            if with_turn:
                a = np.zeros((2 * len(p), 3))
                a[0::2, 0], a[0::2, 2] = -1.0, p[:, 1]
                a[1::2, 1], a[1::2, 2] = -1.0, -p[:, 0]
                sol = np.linalg.lstsq(a, v.reshape(-1), rcond=None)[0]
                ux, uy, w = (float(s) for s in sol)
            else:
                ux, uy, w = -float(np.median(v[:, 0])), -float(np.median(v[:, 1])), 0.0
            pred = np.stack([-ux + w * pos[:, 1], -uy - w * pos[:, 0]], axis=1)
            return ux, uy, w, np.linalg.norm(vel - pred, axis=1)

        inliers = np.ones(len(fitted), dtype=bool)
        for _ in range(3):
            ux, uy, w, res = solve(inliers)
            keep = res <= max(0.25, 2.5 * float(np.median(res[inliers])))
            if (keep == inliers).all():
                break
            if keep.sum() < LIDAR_MOVERS_EGO_MIN_TRACKS:
                inliers = keep
                break
            inliers = keep
        n_in = int(inliers.sum())
        rms = float(np.sqrt(np.mean(res[inliers] ** 2))) if n_in else float("inf")
        valid = (n_in >= LIDAR_MOVERS_EGO_MIN_TRACKS and 2 * n_in >= len(fitted)
                 and rms <= LIDAR_MOVERS_EGO_MAX_RMS_MPS
                 and abs(ux) <= _EGO_MAX_FWD_MPS and abs(uy) <= _EGO_MAX_SIDE_MPS)
        return {"valid": bool(valid), "fwd_mps": round(ux, 2), "right_mps": round(uy, 2),
                "turn_dps": round(math.degrees(w), 1), "still_things": n_in,
                "followed": len(fitted), "rms_mps": round(rms, 3),
                "why": "" if valid else (f"only {n_in} of {len(fitted)} agree"
                                         if n_in < LIDAR_MOVERS_EGO_MIN_TRACKS or 2 * n_in < len(fitted)
                                         else f"spread {rms:.2f} m/s" if rms > LIDAR_MOVERS_EGO_MAX_RMS_MPS
                                         else "not a walking speed")}

    @staticmethod
    def _judge(tr: Dict[str, Any], ego: Dict[str, Any]) -> None:
        """Is it moving by itself, and is it coming at the user?"""
        x, y = tr["x"], tr["y"]
        vx, vy = tr["v"]                       # relative to the user
        r = math.hypot(x, y)
        need = max(LIDAR_MOVERS_MIN_SPEED_MPS, LIDAR_MOVERS_NOISE_PER_M * r)
        if ego["valid"]:
            w = math.radians(ego["turn_dps"])
            own = (vx + ego["fwd_mps"] - w * y, vy + ego["right_mps"] + w * x)
            own_speed = math.hypot(*own)
            need += ego["rms_mps"]
            walk = math.hypot(ego["fwd_mps"], ego["right_mps"])
        else:
            own = (None, vy)                   # sideways is the part no walk explains
            own_speed = _to_segment_dist(vx, vy, LIDAR_MOVERS_WALK_MAX_MPS)
            walk = LIDAR_MOVERS_WALK_MAX_MPS
        # Walking past something near shows a different face of it each scan, and its
        # centre slides by about its size times the turn of the line of sight.
        need += 0.5 * tr["width"] * walk / max(r, 0.3)
        if tr["moving"]:
            need *= 0.7                        # hysteresis: let go more slowly
        tr["moving_scans"] = tr["moving_scans"] + 1 if own_speed >= need else 0
        tr["moving"] = tr["moving_scans"] >= LIDAR_MOVERS_CONFIRM_SCANS
        tr["own_speed"] = own_speed
        tr["own"] = own
        vehicle = (tr["width"] > LIDAR_MOVERS_PERSON_MAX_WIDTH_M
                   or own_speed >= LIDAR_MOVERS_VEHICLE_SPEED_MPS)
        tr["word"] = "Vehicle" if vehicle else "Person"
        tr["alert"] = None
        if not tr["moving"]:
            return
        pv, vv = x * vx + y * vy, vx * vx + vy * vy
        if pv >= 0.0 or vv < 1e-6:
            return                             # not closing
        t_cpa = -pv / vv
        cx, cy = x + vx * t_cpa, y + vy * t_cpa
        miss = math.hypot(cx, cy)
        if t_cpa > LIDAR_MOVERS_HORIZON_S or miss > (SIMPLE_VEHICLE_MISS_M if vehicle
                                                     else LIDAR_MOVERS_MISS_M):
            return
        bearing = math.degrees(math.atan2(y, x))
        if abs(bearing) > 135.0:
            mode, side = "behind", "left" if bearing < 0 else "right"
        elif abs(bearing) <= 45.0:
            head_on = -pv / (math.sqrt(vv) * max(r, 1e-6))
            if head_on >= math.cos(math.radians(LIDAR_MOVERS_HEAD_ON_DEG)):
                mode, side = "toward", "left" if bearing < 0 else "right"
            else:                              # crossing: from the side it moves away from
                mode, side = "crossing", "left" if own[1] > 0 else "right"
        else:
            mode, side = "crossing", "left" if bearing < 0 else "right"
        tr["alert"] = {
            "mode": mode, "side": side,
            "pass_side": "right" if cy > 0.15 else "left" if cy < -0.15 else None,
            "ttc_s": round(t_cpa, 1), "miss_m": round(miss, 2),
        }

    def _snapshot(self, t: float, ego: Dict[str, Any], blobs: List[Dict[str, Any]],
                  fitted: List[Dict[str, Any]]) -> Dict[str, Any]:
        tracks = []
        for tr in fitted:
            if not tr["moving"]:
                continue
            x, y = tr["x"], tr["y"]
            tracks.append({
                "source": "lidar", "id": tr["id"], "word": tr["word"],
                "x_m": round(x, 2), "y_m": round(y, 2),
                "distance_m": round(math.hypot(x, y), 2),
                "bearing_deg": round(math.degrees(math.atan2(y, x)), 1),
                "vx_mps": round(tr["v"][0], 2), "vy_mps": round(tr["v"][1], 2),
                "own_speed_mps": round(tr["own_speed"], 2),
                "width_m": round(tr["width"], 2),
                "alert": dict(tr["alert"]) if tr["alert"] else None,
            })
        alerts = [m for m in tracks if m["alert"]]
        threat = None
        if alerts:
            m = min(alerts, key=lambda a: a["alert"]["ttc_s"])
            threat = {**{k: v for k, v in m.items() if k != "alert"}, **m["alert"]}
        return {"t": t, "updated_mono": time.monotonic(), "tracks": tracks,
                "threat": threat, "ego": ego, "followed": len(self._tracks),
                # Every blob in this scan, as (bearing deg, nearest m): what the LiDAR
                # can follow, so the radar leaves it alone (utils/radar_movers).
                "blobs": [(round(math.degrees(math.atan2(b["y"], b["x"])), 1),
                           round(b["near_m"], 2)) for b in blobs]}

    # ---- read -------------------------------------------------------------

    def snapshot(self) -> Dict[str, Any]:
        """The newest scan's movers; empty once it is older than LIDAR_STALE_S."""
        with self._lock:
            snap = self._snap
        mono = snap.get("updated_mono")
        if mono is None or time.monotonic() - mono > LIDAR_STALE_S:
            return {**self._empty(), "stale": True}
        return {**snap, "stale": False}


_tracker = MoverTracker()


def get_movers() -> MoverTracker:
    return _tracker
