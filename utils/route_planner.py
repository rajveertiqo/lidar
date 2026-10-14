"""The route — the best way through what is around the user, now and as things move
(added 2026-10-13, from the walk simulator).

Everything sensed becomes points on the ground in the user's frame (metres, x ahead, y to
the right): the LiDAR's returns; glass where walk_policy / glass_beside put it (across the
front, or on one side); the near face of each thing the front camera names, at the range
its size gives (the LiDAR's one plane can pass over a table or a low car); and, in an arc
the LiDAR cannot see from where it is worn (LIDAR_BLIND_ARCS_DEG), the surfaces at its
edges carried on into it plus the nearest radar echo the LiDAR cannot account for — so no
return there is never taken for open space.

For each direction from ROUTE_MAX_TURN_DEG left to right, in ROUTE_STEP_DEG steps, the
planner measures how far a lane ROUTE_LANE_HALF_M wide each side stays clear (the user's
own half-width, ROUTE_LANE_TIGHT_M, when nothing wider fits). Moving things (the Kalman
tracks of utils/motion_tracker) are left out of that and checked in time instead: walking
a direction at a pace, does the user come within one standard deviation of where a track's
body will be, inside ROUTE_LOOKAHEAD_S? The best way is the smallest turn that gives
ROUTE_NEED_M — straight if it will do, else the way chosen a moment ago while it stays
clear — at walking pace, else at half pace, else none: wait, or, if something is coming
at the spot the user stands on, step aside where it stays clear longest. A moving thing may
bend the way by ROUTE_STEER_MAX_DEG at most: past that, slowing or waiting beats a swerve.

Advisory: utils/simple_guidance speaks from it only with ROUTE_PLANNER_SPEAK (mover
sentences only with LIDAR_MOVERS_SPEAK); otherwise it is shown and logged as "would say".
"""

import math
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from config import (
    HEADING_NOMINAL_SPEED_MPS, LIDAR_BLIND_ARCS_DEG, LIDAR_MOVERS_MIN_RANGE_MM,
    RADAR_BEAM_H_DEG, RADAR_END_MM,
    RADAR_AGREE_MM, GLASS_LASER_BEYOND_MM, ROUTE_AROUND_M, ROUTE_HOLD_S, ROUTE_LANE_HALF_M,
    ROUTE_LANE_TIGHT_M, ROUTE_LOOKAHEAD_S, ROUTE_MAX_TURN_DEG, ROUTE_NEED_M, ROUTE_REACH_M,
    ROUTE_STEER_MAX_DEG, ROUTE_STEP_ASIDE_M, ROUTE_STEP_DEG, SIMPLE_NAMED_OBJECTS,
    SIMPLE_OBJECT_MIN_CONF,
)
from utils.motion_tracker import gap_at, predicted_sd

_USER_RAD_M = 0.25             # the user's own body, for meeting a moving thing
_CLOSING_M = 0.05              # a meeting counts only when it gets nearer than it is now
_SLOW_FRACTION = 0.5
_MIN_NEED_M = 1.2
_TRACK_PAD_M = 0.35            # laser returns this near a moving track are that track
_ARC_STEP_DEG = 5.0
_WORDS = {c.lower(): w for c, w in SIMPLE_NAMED_OBJECTS.items()}


def _wrap(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _in_arc(bearing: float, arc: Tuple[float, float]) -> bool:
    """Is clockwise bearing `bearing` inside the clockwise arc (start, end)?"""
    a0, a1 = arc[0] % 360.0, arc[1] % 360.0
    b = bearing % 360.0
    return a0 <= b <= a1 if a0 <= a1 else (b >= a0 or b <= a1)


def _xy(bearing_deg: float, r: float) -> Tuple[float, float]:
    a = math.radians(bearing_deg)
    return r * math.cos(a), r * math.sin(a)


class _Points:
    """Points on the ground, each tagged with what put it there."""

    def __init__(self, moving: List[Dict[str, Any]]):
        self.xs: List[float] = []
        self.ys: List[float] = []
        self.tags: List[str] = []
        self._moving = moving

    def add(self, x: float, y: float, tag: str, check_movers: bool = False) -> None:
        if x < -1.0 or x > ROUTE_REACH_M + 0.5 or abs(y) > ROUTE_REACH_M + 0.5:
            return
        if check_movers and any(gap_at(tr, 0.0, x, y) < _TRACK_PAD_M for tr in self._moving):
            return                       # that is a moving thing: planned for in time instead
        self.xs.append(x)
        self.ys.append(y)
        self.tags.append(tag)


def _laser(lidar_state: Dict[str, Any]) -> List[Tuple[float, float, float, float]]:
    """(bearing deg clockwise, range m, x, y) for each return of the newest scan; nearer
    than LIDAR_MOVERS_MIN_RANGE_MM is the wearer's own arm or cane, not the way."""
    pts = (lidar_state or {}).get("points") or {}
    out = []
    for a, d in zip(pts.get("angles") or [], pts.get("distances") or []):
        if isinstance(d, (int, float)) and d >= LIDAR_MOVERS_MIN_RANGE_MM:
            r = float(d) / 1000.0
            x, y = _xy(float(a), r)
            out.append((float(a), r, x, y))
    return out


def _blind_fill(points: _Points, laser: List[Tuple[float, float, float, float]],
                radar_state: Optional[Dict[str, Any]]) -> List[str]:
    """Fill each arc the LiDAR cannot see; returns notes on how."""
    notes = []
    if not LIDAR_BLIND_ARCS_DEG:
        return notes
    by_bearing = sorted(laser, key=lambda p: p[0] % 360.0)
    secs = (radar_state or {}).get("sectors") or {}
    for arc in LIDAR_BLIND_ARCS_DEG:
        known_x: List[float] = []
        known_y: List[float] = []
        # A surface seen at each edge runs on into the arc, straight, up to its middle.
        mid = (arc[0] + (((arc[1] - arc[0]) % 360.0) / 2.0)) % 360.0
        for edge, step in ((arc[0], -1), (arc[1], 1)):
            near = [p for p in by_bearing
                    if 0.0 < (step * _wrap(p[0] - edge)) <= 8.0]   # just outside this edge
            near.sort(key=lambda p: abs(_wrap(p[0] - edge)))
            if len(near) < 2:
                continue
            a, b = near[0], near[min(len(near) - 1, 3)]
            seg = math.hypot(a[2] - b[2], a[3] - b[3])
            if seg < 0.02 or seg > 0.6:
                continue                                       # not one surface
            ux, uy = (a[2] - b[2]) / seg, (a[3] - b[3]) / seg
            prev = _wrap(a[0] - mid)
            t = 0.1
            while t <= 4.0:
                x, y = a[2] + ux * t, a[3] + uy * t
                off = _wrap(math.degrees(math.atan2(y, x)) - mid)
                if abs(off) >= abs(prev) or off * prev < 0:
                    break                                      # reached the middle, or turned away
                prev = off
                points.add(x, y, "blind arc (carried)")
                known_x.append(x)
                known_y.append(y)
                t += 0.1
        # The radar: per board facing into the arc, its nearest echo nothing else accounts for.
        span = (arc[1] - arc[0]) % 360.0
        covered = [False] * (int(span // _ARC_STEP_DEG) + 1)
        half = RADAR_BEAM_H_DEG / 2.0
        for key, row in secs.items():
            if not str(key).startswith("deg") or not row.get("online") or row.get("degraded"):
                continue
            try:
                board = float(str(key)[3:])
            except ValueError:
                continue
            idx = [i for i in range(len(covered))
                   if abs(_wrap(arc[0] + i * _ARC_STEP_DEG - board)) <= half]
            if not idx:
                continue
            echo = RADAR_END_MM / 1000.0
            for mm in sorted(float(v) for v in (row.get("peaks_mm") or []) if v):
                e = mm / 1000.0
                lo, hi = e - RADAR_AGREE_MM / 1000.0, e + GLASS_LASER_BEYOND_MM / 1000.0
                seen = any(lo <= p[1] <= hi and abs(_wrap(p[0] - board)) <= half + 10.0
                           for p in laser)
                seen = seen or any(lo <= math.hypot(x, y) <= hi
                                   and abs(_wrap(math.degrees(math.atan2(y, x)) - board)) <= half + 10.0
                                   for x, y in zip(known_x, known_y))
                if not seen:
                    echo = e
                    break
            for i in idx:
                covered[i] = True
                x, y = _xy(arc[0] + i * _ARC_STEP_DEG, echo)
                points.add(x, y, "blind arc (radar)")
            notes.append(f"{key}: {echo:.1f} m")
        for i, ok in enumerate(covered):
            if not ok:                                         # no board hears it: nothing known
                x, y = _xy(arc[0] + i * _ARC_STEP_DEG, 0.3)
                points.add(x, y, "blind arc (unknown)")
    return notes


def _camera_faces(points: _Points, camera_state: Optional[Dict[str, Any]], now: float) -> None:
    """The near face of each thing the front camera names, at the range its size gives."""
    from utils import simple_guidance as sg
    cam = camera_state or {}
    if not sg._camera_sees(cam, now):
        return
    focal = (sg._QUAD_W / 2.0) / math.tan(math.radians(sg.SIMPLE_CAMERA_HFOV_DEG) / 2.0)
    for det in (cam.get("front") or {}).get("detections") or []:
        cls = str(det.get("class_name", "")).lower()
        word = _WORDS.get(cls)
        if word is None or float(det.get("confidence") or 0.0) < SIMPLE_OBJECT_MIN_CONF:
            continue
        try:
            x1, y1, x2, y2 = (float(v) for v in det.get("bbox") or ())
        except (TypeError, ValueError):
            continue
        if x2 <= x1 or y2 <= y1:
            continue
        rng, _cut = sg._box_range(det, cls, (x1, y1, x2, y2))     # measured, with the ToF
        if rng > ROUTE_REACH_M + 0.5:
            continue
        b1 = math.degrees(math.atan((x1 - sg._QUAD_W / 2.0) / focal))
        b2 = math.degrees(math.atan((x2 - sg._QUAD_W / 2.0) / focal))
        n = max(2, int(math.radians(b2 - b1) * rng / 0.15) + 1)
        for k in range(n):
            x, y = _xy(b1 + (b2 - b1) * k / (n - 1), rng)
            points.add(x, y, f"camera:{word}", check_movers=True)


def _clear_along(points: _Points, h_deg: float, half: float) -> Tuple[float, Optional[int]]:
    """How far (m) the user's lane can go in direction h before it touches a point, and which."""
    sn, cs = math.sin(math.radians(h_deg)), math.cos(math.radians(h_deg))
    best, who = ROUTE_REACH_M, None
    for k, (x, y) in enumerate(zip(points.xs, points.ys)):
        along = x * cs + y * sn
        if along <= 0.0 or along >= best + half:
            continue
        lat = abs(y * cs - x * sn)
        if lat < half:
            d = along - math.sqrt(half * half - lat * lat)
            if d < best:
                best, who = d, k
    return max(0.0, best), who


def _meets(moving: List[Dict[str, Any]], h_deg: float, pace: float,
           at: Tuple[float, float] = (0.0, 0.0)) -> Tuple[float, Optional[Dict[str, Any]]]:
    """Walking direction h at `pace` from `at`: how far before a moving thing's predicted body
    (grown by one standard deviation) meets the user, and which; (inf, None) if none does."""
    dx, dy = math.cos(math.radians(h_deg)), math.sin(math.radians(h_deg))
    first, who = float("inf"), None
    for tr in moving:
        g0 = gap_at(tr, 0.0, at[0], at[1])
        t = 0.1
        while t <= ROUTE_LOOKAHEAD_S + 1e-6 and t < first:
            g = gap_at(tr, t, at[0] + dx * pace * t, at[1] + dy * pace * t)
            if g - _USER_RAD_M - predicted_sd(tr, t) < 0.0 and g < g0 - _CLOSING_M:
                first, who = t, tr
                break
            t += 0.1
    return (pace * first if who else float("inf")), who


class RoutePlanner:
    """Holds the way chosen a moment ago, so the plan does not flip from poll to poll."""

    def __init__(self):
        self._lock = threading.Lock()
        self._held: Optional[Tuple[float, float]] = None     # (world heading deg, when)
        self._last: Dict[str, Any] = {}

    def reset(self) -> None:
        with self._lock:
            self._held, self._last = None, {}

    def last(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._last)

    def plan(self, lidar_state: Optional[Dict[str, Any]],
             radar_state: Optional[Dict[str, Any]] = None,
             camera_state: Optional[Dict[str, Any]] = None,
             tracks: Optional[Dict[str, Any]] = None,
             motion: Optional[Dict[str, Any]] = None,
             glass: Optional[Dict[str, Any]] = None,
             now: Optional[float] = None) -> Dict[str, Any]:
        """The plan for this poll. `tracks` is motion_tracker's snapshot, `motion`
        heading_tracker's state, `glass` {"ahead_m": .., "side": .., "side_m": ..}."""
        now = time.time() if now is None else now
        snap = tracks or {}
        moving = [] if snap.get("stale") else [tr for tr in snap.get("tracks") or []
                                                if tr.get("moving")]
        points = _Points(moving)
        laser = _laser(lidar_state or {})
        for _a, _r, x, y in laser:
            points.add(x, y, "laser", check_movers=bool(moving))
        g = glass or {}
        if isinstance(g.get("ahead_m"), (int, float)):
            for o in range(-30, 31, 5):
                x, y = _xy(o, float(g["ahead_m"]))
                points.add(x, y, "glass")
        elif g.get("side") in ("left", "right") and isinstance(g.get("side_m"), (int, float)):
            sign = -1.0 if g["side"] == "left" else 1.0
            for o in range(25, 61, 5):
                x, y = _xy(sign * o, float(g["side_m"]))
                points.add(x, y, "glass")
        try:
            _camera_faces(points, camera_state, now)
        except Exception:
            pass                                      # advisory: a bad frame costs only its objects
        blind = _blind_fill(points, laser, radar_state)

        heads = list(range(-ROUTE_MAX_TURN_DEG, ROUTE_MAX_TURN_DEG + 1, ROUTE_STEP_DEG))
        k0 = heads.index(0)
        lane = ROUTE_LANE_HALF_M
        found = [_clear_along(points, h, lane) for h in heads]
        far = max(d for d, _ in found)
        if far < ROUTE_AROUND_M:
            lane = ROUTE_LANE_TIGHT_M
            found = [_clear_along(points, h, lane) for h in heads]
            far = max(d for d, _ in found)
        clear = [d for d, _ in found]

        heading = (motion or {}).get("heading_deg")
        heading = float(heading) if isinstance(heading, (int, float)) else None
        with self._lock:
            held = self._held
        held_k = None
        if held is not None and heading is not None and now - held[1] <= ROUTE_HOLD_S:
            rel = round(_wrap(held[0] - heading) / ROUTE_STEP_DEG) * ROUTE_STEP_DEG
            held_k = heads.index(int(rel)) if int(rel) in heads else None

        def open_side(k: int) -> float:
            sign = math.copysign(1, heads[k]) if heads[k] else 0
            return sum(c for h, c in zip(heads, clear) if h and math.copysign(1, h) == sign)

        def choose(ok) -> int:
            if ok(k0):
                return k0
            if held_k is not None and ok(held_k):
                return held_k
            pick = -1
            for k, h in enumerate(heads):
                if not ok(k):
                    continue
                if pick < 0 or abs(h) < abs(heads[pick]) or (
                        abs(h) == abs(heads[pick]) and open_side(k) > open_side(pick)):
                    pick = k
            return pick

        k_static = choose(lambda k: clear[k] >= min(ROUTE_NEED_M, far) - 0.05)
        pace = float((motion or {}).get("speed_mps") or 0.0) if (motion or {}).get("speed_trusted") else 0.0
        walk = min(1.6, max(0.8, pace or HEADING_NOMINAL_SPEED_MPS))
        speed, pick, cause, effective = "wait", -1, None, clear
        for mode, sp in (("walk", walk), ("slow", walk * _SLOW_FRACTION)):
            need = min(max(_MIN_NEED_M, sp * 2.0), far) - 0.05
            met = [_meets(moving, h, sp) if moving else (float("inf"), None) for h in heads]
            eff = [min(c, m[0]) for c, m in zip(clear, met)]
            if mode == "walk":
                effective = eff
                cause = met[k_static][1]
            # A moving thing may bend the way by a small turn at most; a big swerve round it
            # (toward traffic, say) is worse than slowing down or waiting for it to pass.
            k = choose(lambda k: eff[k] >= need
                       and abs(heads[k] - heads[k_static]) <= ROUTE_STEER_MAX_DEG)
            if k >= 0:
                speed, pick = mode, k
                if mode == "slow":
                    cause = met[k_static][1] or cause
                break
        step_aside = None
        if pick < 0:
            pick = k_static
            # Waiting is safe only if nothing is coming at the spot the user stands on.
            coming = None
            for tr in moving:
                g0 = gap_at(tr, 0.0, 0.0, 0.0)
                t = 0.1
                while t <= 3.0 + 1e-6:
                    gg = gap_at(tr, t, 0.0, 0.0)
                    if gg - _USER_RAD_M - predicted_sd(tr, t) < 0.0 and gg < g0 - _CLOSING_M:
                        coming = tr
                        break
                    t += 0.1
                if coming:
                    break
            if coming:
                cause = coming
                best = None
                for h in range(-165, 181, 15):
                    if _clear_along(points, h, ROUTE_LANE_TIGHT_M)[0] < ROUTE_STEP_ASIDE_M:
                        continue
                    d = _meets(moving, h, walk * _SLOW_FRACTION)[0]
                    if best is None or d > best[1] + 0.05 or (abs(d - best[1]) <= 0.05
                                                               and abs(h) < abs(best[0])):
                        best = (h, d)
                if best is not None:
                    step_aside = best[0]
        by_mover = speed != "wait" and pick != k_static
        if speed == "walk" and not by_mover:
            cause = None
        head = step_aside if step_aside is not None else heads[pick]
        if heading is not None and step_aside is None and head:
            with self._lock:
                self._held = (heading + head, now)
        elif not head:
            with self._lock:
                self._held = None
        side = "right" if head > 0 else "left" if head < 0 else None
        if step_aside is not None and abs(step_aside) > 120:
            side = None                               # only back is clear: no side to step to
        _, blocker = found[k0]
        out = {
            "ok": True,
            "head_deg": head, "side": side,
            "dist_m": round(ROUTE_STEP_ASIDE_M if step_aside is not None else clear[pick], 2),
            "straight_m": round(clear[k0], 2),
            "around": far < ROUTE_AROUND_M,
            "speed": "slow" if step_aside is not None else speed,
            "step_aside": step_aside is not None,
            "by_mover": by_mover,
            "lane_m": round(2 * lane, 2),
            "pace_mps": round(walk, 2),
            "blocker": points.tags[blocker] if blocker is not None else None,
            "cause": ({k: cause.get(k) for k in ("id", "word", "x_m", "y_m", "distance_m",
                                                 "bearing_deg", "own_speed_mps", "speed_sd")}
                      if cause else None),
            "heads_deg": heads,
            "clear_m": [round(c, 2) for c in clear],
            "safe_m": [round(min(e, ROUTE_REACH_M), 2) for e in effective],
            "points": len(points.xs),
            "blind": blind,
        }
        with self._lock:
            self._last = out
        return out


_planner = RoutePlanner()


def get_planner() -> RoutePlanner:
    return _planner
