"""Hazards on the ground ahead, from the ToF depth camera, confirmed by the front camera
(2026-11-06): stairs up and down, single steps (kerbs), drops, holes, poles, low obstacles,
overhanging ones a cane passes under, and anything else in the way below the LiDAR.

The LiDAR is worn on the shoulder and measures one plane there, so it misses all of these
but the pole (docs/known-limits.md). The ToF looks down at the ground ~1-4 m ahead (its
view runs 7-53 deg below level), so each frame:

  1. Its points go into the level frame by the mount's design (FUSION_*), and the ground is
     FITTED to them (RANSAC: the near-level plane at the mount's height, near the last one,
     with the most points within TOF_GROUND_INLIER_M): the body's sway and the mount's
     error are measured, not assumed. Every point gets its height above that ground.
  2. Along the walking corridor (TOF_CORRIDOR_HALF_M either side), the ground's height every
     TOF_PROFILE_BIN_M ahead, cut where it jumps: the floor, treads, a kerb's top, a lower
     street. A sharp rise from the floor is a step up, two or more of a stair's rise and run
     are stairs up; a fall alike is a step or stairs down, a deeper one a drop, a fall and
     back up a hole. Going down, the treads show only as strips past each edge, so every
     level seen counts there. A floor that ends in nothing at all is a drop too deep for
     the ToF to measure: said only when the camera sees its edge.
  3. The points above the ground that are not steps, in 5 cm cells, joined into things: an
     overhang (the front of it is all above TOF_OVERHANG_MIN_H_M), a pole (narrow, from the
     ground up out of the ToF's view), a low obstacle (no taller than TOF_LOW_MAX_H_M), or
     an obstacle (taller: the LiDAR may see it, or not if it is below the shoulder).
  4. The front camera, where each lands on its picture (utils/depth_fusion's calibration):
     a detection box over it confirms and names it; else the picture's own edges do — two or
     more level edges for stairs, one for a step, drop or hole, an upright edge for a pole,
     an outline for anything else. A dark, blurred or missing picture is "unseen".
  6. The nearest tall thing on the path, seen from above: does its surface run flat across
     the path (a wall, a window blind, a corner of two walls), or not (a chair, a person)?
     front_surface, kept for the last TOF_WALL_CONFIRM[1] frames; wall_verdict and
     confirmed_wall judge it (config TOF_WALL_*, 2026-11-10), at guidance time, so a
     recorded walk replays with new thresholds.
  5. Seen in TOF_HAZARD_CONFIRM[0] of the last TOF_HAZARD_CONFIRM[1] analyses, then
     speak_rule: confirmed or unseen by the camera, it is said; with the camera looking and
     seeing nothing there, only a measured fall of at least TOF_FALL_ALONE_M — a ToF
     artifact must not stop the user, and a real edge on a plain floor must not be missed.

simple_guidance turns the top hazard into its sentence (it owns the closed vocabulary), and
says it only with TOF_HAZARDS_SPEAK; otherwise it is logged as "would say".
Frames: OpenCV's (x right, y down, z ahead); "level": the camera's frame turned up by
FUSION_CAMERA_DOWN_DEG. Heights are above the fitted ground, distances along it, in metres
(the guidance says them in feet).
"""

import collections
import math
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from config import (
    FUSION_CAMERA_DOWN_DEG,
    SIMPLE_NAMED_OBJECTS,
    TOF_CAMERA_EDGE_DENSITY,
    TOF_CAMERA_MARGIN_PX,
    TOF_CAMERA_MAX_AGE_S,
    TOF_CORRIDOR_HALF_M,
    TOF_ENABLED,
    TOF_FALL_ALONE_M,
    TOF_GROUND_INLIER_M,
    TOF_GROUND_MAX_TILT_DEG,
    TOF_GROUND_MIN_POINTS,
    TOF_HAZARD_CONFIRM,
    TOF_HAZARD_HZ,
    TOF_HAZARD_STOP_M,
    TOF_HAZARD_WARN_M,
    TOF_HAZARDS_ENABLED,
    TOF_LOW_MAX_H_M,
    TOF_MAX_MM,
    TOF_MIN_CONFIDENCE,
    TOF_MIN_MM,
    TOF_MOUNT_HEIGHT_M,
    TOF_MOUNT_HEIGHT_TOL_M,
    TOF_OBSTACLE_MIN_H_M,
    TOF_OBSTACLE_MIN_POINTS,
    TOF_OVERHANG_MIN_H_M,
    TOF_POLE_MAX_WIDTH_M,
    TOF_POLE_MIN_H_M,
    TOF_PROFILE_BIN_M,
    TOF_PROFILE_JUMP_M,
    TOF_PROFILE_MAX_M,
    TOF_PROFILE_MIN_POINTS,
    TOF_STAIR_RISE_M,
    TOF_STAIR_RUN_M,
    TOF_STEP_MAX_M,
    TOF_STEP_MIN_M,
    TOF_WALL_CONFIRM,
    TOF_WALL_FLAT_M,
    TOF_WALL_MIN_H_M,
    TOF_WALL_MIN_SPAN_M,
    TOF_WALL_MIN_WIDTH_M,
)
from utils import depth_fusion as df

# Worst first: the order a tie is broken in, and the dashboard's.
KINDS = (
    "drop",
    "stairs_down",
    "hole",
    "step_down",
    "stairs_up",
    "step_up",
    "pole",
    "overhang",
    "low_obstacle",
    "obstacle",
)
FALLS = ("drop", "stairs_down", "hole", "step_down")
_FROM_PROFILE = ("drop", "stairs_down", "hole", "step_down", "stairs_up", "step_up")
_SEVERITY = {k: i for i, k in enumerate(KINDS)}
_NAMES = {str(c).lower(): w for c, w in SIMPLE_NAMED_OBJECTS.items()}

_Z0 = 0.5  # the profile and the cells start this far ahead (the ToF sees ground from ~1.1 m)
_CELL = 0.05  # obstacle cells, m
_X_SPAN = 1.5  # obstacles are looked for this far either side
_GROUND_H = 0.05  # a surface this near the ground's height is the ground
_UP_GAP_M = 0.30  # a step up: the higher surface starts within this past the lower (its riser is seen)
_DOWN_GAP_M = 1.20  # a step down: the lower one within this (it is partly hidden past the edge)
_BUMP_M = 1.0  # up and back down (or down and back up) within this: a thing on the floor, a hole
_MISSING_M = 0.8  # the floor ends and nothing at all is seen for this far after it...
_MISSING_MAX_Z = 2.6  # ...where the floor should still be well seen: a drop past the ToF's reach
_TOP_ROWS = 3  # points in the ToF picture's top rows: the thing goes on above its view
_FRONT_M = 0.15  # an obstacle's front: its points this near its nearest
_CAMERA_PTS = 400  # at most this many of a hazard's points are placed on the picture
# A tall thing's surface from above (_surface): one sample per this many ToF columns, the
# median of the thing's points in them (at least _WALL_SAMPLE_MIN); the next sample along is
# the same surface when no further than _WALL_JUMP_M from the last, with at most
# _WALL_GAP_GROUPS empty column groups between; the view's edge is within _WALL_EDGE_GROUPS.
_WALL_COL_GROUP = 4
_WALL_SAMPLE_MIN = 8
_WALL_JUMP_M = 0.25
_WALL_GAP_GROUPS = 2
_WALL_EDGE_GROUPS = 1
_WALL_LINE_MIN_M = 0.30  # an arm is held to its line once it reaches this far (folds average out)
_WALL_MAX_POINTS = 6000  # a bigger thing is thinned to this many points for it


# ---------------------------------------------------------------- one frame: the geometry



# What stands within reach on the walking path, by its distance from the ToF, with no ground
# needed (2026-11-09): the guidance asks whether a radar echo the laser does not see is a solid
# thing there (walk_policy, a desk or a table below the laser's plane), not glass. Worn, the
# floor is never this near in the ToF's view (aimed 30 deg down from ~1.45 m: ~1.6 m at the
# least, the body leaning 15 deg), and points more than TOF_NEAR_BELOW_M below it are left out
# besides (a shorter wearer, leaning); on a desk, where no ground is found, it still answers.
TOF_NEAR_MAX_M = 1.3
TOF_NEAR_BIN_M = 0.05
TOF_NEAR_HALF_M = 0.30  # the walking corridor's half width (GUIDANCE_CORRIDOR_HALF_WIDTH_MM)
TOF_NEAR_BELOW_M = 1.0  # a table top is ~0.5-0.7 m below a worn ToF; the floor 1.2 m or more


# Resting on a surface (2026-11-10): the rig set down on a desk sees the desk top right under
# it, a flat layer facing up, and its radar hears it ~0.3 m ahead — the guidance blocked on it
# ("Obstacle in front, turn left", front 0.9 ft, the laser seeing the blind at 4 ft). A plane
# of at least TOF_REST_MIN_POINTS within TOF_REST_REACH_M of the ToF, TOF_REST_BELOW_M below
# it, facing up in the ToF's own frame (not the worn mount's level frame: set down, the rig is
# not tilted as it is worn), with its points within TOF_REST_FLAT_M of it. Worn, nothing level
# is ever that near under the ToF. Its points are left out of near_counts, and walk_policy
# takes a front echo there that the laser sees past for the surface, not a thing in the way.
TOF_REST_REACH_M = 0.45
TOF_REST_BELOW_M = (0.0, 0.40)  # on the rig the lens sits ~2 cm above the desk it is set on
TOF_REST_MIN_POINTS = 300
TOF_REST_FLAT_M = 0.02
TOF_REST_UP = 0.5  # |the plane's normal . the picture's up| at least this: facing up, not ahead


def rest_surface(P: np.ndarray) -> Tuple[Optional[Dict[str, Any]], np.ndarray]:
    """(the surface the ToF rests on, or None; which of P's points lie on it). P: the ToF's
    own frame (x right, y down, z ahead), metres."""
    on = np.zeros(len(P), dtype=bool)
    rng = np.sqrt(np.sum(P * P, axis=1))
    cand = np.flatnonzero((rng <= TOF_REST_REACH_M) & (P[:, 1] > 0.0))  # near, below the centre
    if cand.size < TOF_REST_MIN_POINTS:
        return None, on
    Q = P[cand]
    c = Q.mean(axis=0)
    _w, v = np.linalg.eigh((Q - c).T @ (Q - c))
    n = v[:, 0]  # the plane's normal: the direction it varies least
    if n[1] > 0:
        n = -n  # pointing up the picture, toward the ToF's side
    dist = float(-(c @ n))  # how far the plane is from the ToF
    res = np.abs((Q - c) @ n)
    inl = res <= TOF_REST_FLAT_M
    if (
        int(inl.sum()) < TOF_REST_MIN_POINTS
        or abs(n[1]) < TOF_REST_UP
        or not TOF_REST_BELOW_M[0] <= dist <= TOF_REST_BELOW_M[1]
    ):
        return None, on
    on[cand[inl]] = True
    return {
        "below_m": round(dist, 3),
        "to_m": round(float(np.percentile(rng[cand[inl]], 95)), 3),
        "points": int(inl.sum()),
    }, on


def near_counts(P: np.ndarray, x: np.ndarray, y: np.ndarray, z: np.ndarray) -> List[int]:
    """How many of this frame's points lie in the walking corridor at each TOF_NEAR_BIN_M of
    distance from the ToF, out to TOF_NEAR_MAX_M, no lower than TOF_NEAR_BELOW_M (y down)."""
    rng = np.sqrt(np.sum(P * P, axis=1))
    on = (
        (np.abs(x) <= TOF_NEAR_HALF_M)
        & (z > 0.0)
        & (y <= TOF_NEAR_BELOW_M)
        & (rng <= TOF_NEAR_MAX_M)
    )
    n = int(round(TOF_NEAR_MAX_M / TOF_NEAR_BIN_M))
    bins = np.minimum((rng[on] / TOF_NEAR_BIN_M).astype(int), n - 1)
    return np.bincount(bins, minlength=n)[:n].astype(int).tolist()

class GroundAnalyser:
    """One ToF frame -> the ground and the hazards on it. Its only state is the last ground,
    which steers the next fit (so a stair tread does not replace the floor)."""

    def __init__(self, projector: Optional[df.DepthProjector] = None, seed: int = 0):
        self._rng = np.random.default_rng(seed)
        self._plane: Optional[np.ndarray] = None  # (a, b, c): y = a x + b z + c, level frame
        self._plane_t = 0.0
        self.use(projector or df.get_projector())

    def use(self, projector: df.DepthProjector) -> None:
        """Place the points with this projector's rays and calibration."""
        self.projector = projector
        c = projector.calib
        level = df.pitch_down(FUSION_CAMERA_DOWN_DEG)
        self.to_level = level @ c.R  # ToF frame -> level frame
        self.t_level = level @ c.t
        self.width = int(c.tof.width)

    # -- the ground

    def _ransac(self, x, y, z, ci, prev) -> Optional[np.ndarray]:
        tilt_max = math.tan(math.radians(TOF_GROUND_MAX_TILT_DEG))
        sub = ci if ci.size <= 3000 else self._rng.choice(ci, 3000, replace=False)
        X = np.stack([x[sub], z[sub], np.ones(sub.size)], axis=1)
        Y = y[sub]
        tri = self._rng.integers(0, sub.size, size=(96, 3))
        A, B = X[tri], Y[tri]
        good = np.abs(np.linalg.det(A)) > 1e-4  # three points spread out, not in a line
        if not good.any():
            return None
        coef = np.linalg.solve(A[good], B[good][..., None])[..., 0]  # (k, 3): a, b, c
        ok = np.hypot(coef[:, 0], coef[:, 1]) < tilt_max
        if prev is not None:
            ok &= np.abs(coef[:, 2] - prev[2]) < 0.30
        else:
            ok &= np.abs(coef[:, 2] - TOF_MOUNT_HEIGHT_M) < TOF_MOUNT_HEIGHT_TOL_M
        coef = coef[ok]
        if not len(coef):
            return None
        inliers = (np.abs(Y[None, :] - coef @ X.T) < TOF_GROUND_INLIER_M).sum(axis=1)
        score = inliers.astype(np.float64)
        if prev is not None:  # the floor stays the floor: a tread must win clearly
            score *= np.where(np.abs(coef[:, 2] - prev[2]) < 0.08, 1.0, 0.6)
        return coef[int(np.argmax(score))]

    def _fit_ground(self, x, y, z, t) -> Optional[Tuple[np.ndarray, int, str]]:
        tilt_max = math.tan(math.radians(TOF_GROUND_MAX_TILT_DEG))
        prev = (
            self._plane
            if self._plane is not None and abs(t - self._plane_t) <= 1.0
            else None
        )
        near = (z > _Z0) & (z < 3.0) & (np.abs(x) < _X_SPAN)
        tries = []
        if prev is not None:
            tries.append(
                (near & (np.abs(y - (prev[0] * x + prev[1] * z + prev[2])) < 0.30), prev)
            )
        tries.append(
            (
                near
                & (
                    np.abs(y - TOF_MOUNT_HEIGHT_M)
                    < TOF_MOUNT_HEIGHT_TOL_M + tilt_max * np.hypot(x, z)
                ),
                None,
            )
        )
        for gate, p in tries:
            ci = np.flatnonzero(gate)
            if ci.size < TOF_GROUND_MIN_POINTS:
                continue
            best = self._ransac(x, y, z, ci, p)
            if best is None:
                continue
            Xa = np.stack([x[ci], z[ci], np.ones(ci.size)], axis=1)
            on = np.abs(y[ci] - Xa @ best) < 1.5 * TOF_GROUND_INLIER_M
            if on.sum() < TOF_GROUND_MIN_POINTS:
                continue
            coef = np.linalg.lstsq(Xa[on], y[ci][on], rcond=None)[0]
            if np.hypot(coef[0], coef[1]) >= tilt_max or coef[2] < 0.4:
                continue
            self._plane, self._plane_t = coef, t
            return coef, int(on.sum()), ("followed" if p is not None else "found")
        return None

    # -- the corridor's profile: steps, stairs, drops, holes

    def _profile(self, x, z, h):
        bin_m = TOF_PROFILE_BIN_M
        nb = int(math.ceil((TOF_PROFILE_MAX_M - _Z0) / bin_m))
        corr = np.flatnonzero(
            (np.abs(x) <= TOF_CORRIDOR_HALF_M) & (z >= _Z0) & (z < TOF_PROFILE_MAX_M)
        )
        zb = np.clip(((z[corr] - _Z0) / bin_m).astype(np.int64), 0, nb - 1)
        order = np.argsort(zb, kind="stable")
        zb, hs = zb[order], h[corr][order]
        counts = np.bincount(zb, minlength=nb)
        ends = np.cumsum(counts)
        starts = ends - counts
        prof = np.full(nb, np.nan)
        for i in np.flatnonzero(counts >= TOF_PROFILE_MIN_POINTS):
            # The lower quarter: the ground under whatever stands on it.
            prof[i] = np.percentile(hs[starts[i] : ends[i]], 25)
        centres = _Z0 + (np.arange(nb) + 0.5) * bin_m
        return centres, prof, counts

    @staticmethod
    def _pieces(prof, centres) -> List[Dict[str, Any]]:
        """Runs of bins whose heights join (no jump past TOF_PROFILE_JUMP_M, at most two
        bins missing): each one surface."""
        half = TOF_PROFILE_BIN_M / 2.0
        runs: List[List[int]] = []
        for i in np.flatnonzero(~np.isnan(prof)):
            if (
                runs
                and i - runs[-1][-1] <= 3
                and abs(prof[i] - prof[runs[-1][-1]]) <= TOF_PROFILE_JUMP_M
            ):
                runs[-1].append(int(i))
            else:
                runs.append([int(i)])
        out = []
        span = max(2, int(round(0.30 / TOF_PROFILE_BIN_M)))
        for b in runs:
            hv = prof[b]
            # Its height where it starts and ends: over 30 cm, leaving out the end bin, which
            # holds the riser between it and the next (the two bins nearest a 9 cm kerb read
            # it 7 mm short of a step, desk test 2026-11-06).
            if len(b) >= 4:
                h0, h1 = np.median(hv[1 : 1 + span]), np.median(hv[-1 - span : -1])
            else:
                h0, h1 = np.median(hv[:2]), np.median(hv[-2:])
            out.append(
                {
                    "n": len(b),
                    "z0": float(centres[b[0]] - half),
                    "z1": float(centres[b[-1]] + half),
                    "h0": float(h0),
                    "h1": float(h1),
                    "hm": float(np.median(hv)),
                }
            )
        return out

    def _ground_features(self, x, z, h) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
        centres, prof, counts = self._profile(x, z, h)
        pieces = self._pieces(prof, centres)
        info: Dict[str, Any] = {
            "profile": [
                [round(float(c), 2), None if np.isnan(p) else round(float(p), 3)]
                for c, p in zip(centres, prof)
            ]
        }
        g = next(
            (
                k
                for k, p in enumerate(pieces)
                if p["n"] >= 2 and abs(p["hm"]) <= _GROUND_H and p["z0"] <= 2.5
            ),
            None,
        )
        if g is None:
            info["floor"] = None
            return [], info
        # The floor goes on past a gap in the data (a dark seam) and past lone bins at its
        # height (a riser's own bin, where the riser starts low): only a surface of two bins
        # or more at another height, or a lone strip below it, ends it.
        floor = dict(pieces[g])
        k = g + 1
        while k < len(pieces):
            p = pieces[k]
            if abs(p["h0"] - floor["h1"]) >= TOF_STEP_MIN_M or p["z0"] - floor["z1"] > 0.5:
                break
            if p["n"] >= 2:
                floor.update(z1=p["z1"], h1=p["h1"], n=floor["n"] + p["n"])
            k += 1
        info["floor"] = [round(floor["z0"], 2), round(floor["z1"], 2)]
        after = pieces[k:]
        if not after:
            return self._missing(floor, centres, counts), info
        if after[0]["h0"] - floor["h1"] <= -TOF_STEP_MIN_M:
            return self._going_down(floor, after), info
        big = next((p for p in after if p["n"] >= 2), None)
        if big is not None and big["h0"] - floor["h1"] >= TOF_STEP_MIN_M:
            return self._going_up(floor, after), info
        return [], info

    def _missing(self, floor, centres, counts) -> List[Dict[str, Any]]:
        """The floor ends, and nothing at all comes back for _MISSING_M after it, where the
        floor should still be well seen: a drop deeper than the ToF reaches. Unmeasured."""
        end = floor["z1"]
        if end > _MISSING_MAX_Z:
            return []
        after = (centres > end) & (centres <= end + _MISSING_M)
        before = (centres > floor["z0"]) & (centres <= end)
        # Nothing measurable: the floor's own edge, read with ~1 % range noise, scatters a
        # few points just past it (strictly none found the drop in 2 frames of 5).
        if (
            not after.any()
            or counts[after].max() >= TOF_PROFILE_MIN_POINTS
            or counts[after].sum() > 2 * TOF_PROFILE_MIN_POINTS
        ):
            return []
        if np.median(counts[before]) < 3 * TOF_PROFILE_MIN_POINTS:
            return []  # a thin floor: it may simply fade out
        return [_feature("drop", end, depth=None, measured=False)]

    def _going_up(self, floor, after) -> List[Dict[str, Any]]:
        """Risers are seen going up: each tread is a piece of its own."""
        rise_lo, rise_hi = TOF_STAIR_RISE_M
        run_lo, run_hi = TOF_STAIR_RUN_M
        steps = []
        cur = floor
        for p in after:
            if p["n"] < 2:
                continue  # a riser's own bin: between two surfaces
            dh = p["h0"] - cur["h1"]
            gap = p["z0"] - cur["z1"]
            if abs(dh) < TOF_STEP_MIN_M:
                cur = p
                continue
            if dh > 0 and gap > _UP_GAP_M:
                break
            steps.append({"dh": dh, "edge": (cur["z1"] + p["z0"]) / 2.0, "to": p})
            cur = p
            if dh < 0 or dh > TOF_STEP_MAX_M or len(steps) >= 12:
                break
        if not steps or steps[0]["dh"] <= 0:
            return []
        s0 = steps[0]
        if s0["dh"] > TOF_STEP_MAX_M:
            return []  # a wall, or something on the floor: the cells judge it
        if (
            len(steps) >= 2
            and steps[1]["dh"] < 0
            and abs(steps[1]["to"]["hm"]) <= _GROUND_H
            and steps[1]["edge"] - s0["edge"] <= _BUMP_M
        ):
            return []  # up and back down: a thing on the floor (the cells')
        run = [s0] if rise_lo <= s0["dh"] <= rise_hi else []
        for st in steps[1:]:
            if not run or st["dh"] <= 0 or not rise_lo <= st["dh"] <= rise_hi:
                break
            if not run_lo <= st["edge"] - run[-1]["edge"] <= run_hi:
                break
            run.append(st)
        # Every tread of the climb, past the run judged too: none of it is an obstacle.
        climb = []
        for st in steps:
            if not 0 < st["dh"] <= TOF_STEP_MAX_M:
                break
            climb.append(st["to"])
        top = max(p["hm"] for p in climb)
        if len(run) >= 2:
            rise = float(np.mean([s["dh"] for s in run]))
            f = _feature("stairs_up", s0["edge"], height=top, steps=len(run), rise=rise, top=top)
            # The flight goes on past what the profile judged (to 3.6 m), up its slope: the
            # line through the treads' own heights (not the edges' spacing — a few cm off
            # there put the fifth tread over it, desk test 2026-11-06). Under it, and its
            # margin, nothing is an obstacle.
            slope, at0 = np.polyfit([p["z0"] for p in climb], [p["hm"] for p in climb], 1)
            f["_envelope"] = (s0["edge"], float(at0 + slope * s0["edge"]), float(max(0.0, slope)))
            return [f]
        f = _feature("step_up", s0["edge"], height=s0["dh"], steps=1, top=top)
        f["_envelope"] = (s0["edge"], top, 0.0)
        return [f]

    def _going_down(self, floor, after) -> List[Dict[str, Any]]:
        """Past a downward edge each tread shows as a strip a few cm long, the rest hidden
        behind the edge before it: every level seen counts, one bin or more."""
        rise_lo, rise_hi = TOF_STAIR_RISE_M
        run_lo, run_hi = TOF_STAIR_RUN_M
        edge = floor["z1"]
        if after[0]["z0"] - edge > _DOWN_GAP_M:
            return []
        levels = [after[0]]
        for p in after[1:]:
            if p["z0"] - levels[-1]["z1"] > _DOWN_GAP_M:
                break
            levels.append(p)
        depth = -levels[0]["hm"]
        # Down and back up to the floor soon after: a hole (a pothole, a trench).
        for p in levels[1:]:
            if abs(p["hm"]) <= _GROUND_H and p["z0"] - edge <= _BUMP_M:
                return [
                    _feature(
                        "hole",
                        edge,
                        depth=max(-q["hm"] for q in levels[: levels.index(p)]),
                        length=p["z0"] - edge,
                    )
                ]
        # Stairs: levels a stair's rise apart, their starts a stair's run apart.
        flight = [levels[0]]
        for p in levels[1:]:
            drop = flight[-1]["hm"] - p["hm"]
            if not rise_lo <= drop <= rise_hi:
                break
            if not run_lo * 0.6 <= p["z0"] - flight[-1]["z0"] <= run_hi:
                break
            flight.append(p)
        # (The first level may be several treads down, the ones between hidden by the edge.)
        if len(flight) >= 2 and depth <= 1.0:
            rises = [flight[k]["hm"] - flight[k + 1]["hm"] for k in range(len(flight) - 1)]
            hidden = max(0, int(round(depth / float(np.mean(rises)))) - 1)
            return [
                _feature(
                    "stairs_down",
                    edge,
                    depth=-flight[-1]["hm"],
                    steps=len(flight) + hidden,
                    rise=float(np.mean(rises)),
                )
            ]
        if depth <= TOF_STEP_MAX_M:
            return [_feature("step_down", edge, depth=depth, steps=1)]
        return [_feature("drop", edge, depth=depth)]

    # -- things standing on the ground

    def _obstacles(self, x, z, h, rows, features) -> List[Dict[str, Any]]:
        from scipy import ndimage

        pick = (
            (h >= TOF_OBSTACLE_MIN_H_M)
            & (z >= _Z0)
            & (z <= TOF_MAX_MM / 1000.0)
            & (np.abs(x) <= _X_SPAN)
        )
        for f in features:  # a kerb's top and a stair's treads and risers are not obstacles
            if f.get("_envelope"):
                edge, base, slope = f["_envelope"]
                past = np.maximum(0.0, z - edge)
                under = h <= base + slope * past + 0.10 + 0.05 * past  # wider further on
                pick &= ~((z >= edge - 0.15) & under)
        oi = np.flatnonzero(pick)
        if oi.size < TOF_OBSTACLE_MIN_POINTS:
            return []
        nx = int(round(2 * _X_SPAN / _CELL)) + 1
        nz = int(round((TOF_MAX_MM / 1000.0 - _Z0) / _CELL)) + 1
        cx = np.clip(((x[oi] + _X_SPAN) / _CELL).astype(np.int64), 0, nx - 1)
        cz = np.clip(((z[oi] - _Z0) / _CELL).astype(np.int64), 0, nz - 1)
        grid = np.zeros((nz, nx), dtype=bool)
        grid[cz, cx] = True
        lab, n = ndimage.label(grid, structure=np.ones((3, 3), dtype=bool))
        if n == 0:
            return []
        which = lab[cz, cx]
        order = np.argsort(which, kind="stable")
        cut = np.cumsum(np.bincount(which, minlength=n + 1))
        out = []
        for k in range(1, n + 1):
            sel = oi[order[cut[k - 1] : cut[k]]]
            if sel.size < TOF_OBSTACLE_MIN_POINTS:
                continue
            xs, zs, hs = x[sel], z[sel], h[sel]
            near = float(np.percentile(zs, 3))
            lo, hi = (float(v) for v in np.percentile(xs, [3, 97]))
            h_lo, h_hi = (float(v) for v in np.percentile(hs, [3, 97]))
            depth = float(np.percentile(zs, 97)) - near
            top_cut = bool(np.mean(rows[sel] < _TOP_ROWS) > 0.03)
            front = hs[zs <= near + _FRONT_M]
            front_lo = float(np.percentile(front, 5)) if front.size else h_lo
            width = hi - lo
            if front_lo >= TOF_OVERHANG_MIN_H_M:
                kind = "overhang"
            elif (
                width <= TOF_POLE_MAX_WIDTH_M
                and depth <= 0.35
                and h_lo <= 0.20
                and (h_hi >= TOF_POLE_MIN_H_M or top_cut)
            ):
                kind = "pole"
            elif h_hi <= TOF_LOW_MAX_H_M and not top_cut:
                kind = "low_obstacle"
            else:
                kind = "obstacle"
            f = _feature(
                kind,
                near,
                side=(lo + hi) / 2.0,
                width=width,
                height=h_hi,
                low=front_lo if kind == "overhang" else h_lo,
                in_path=lo <= TOF_CORRIDOR_HALF_M and hi >= -TOF_CORRIDOR_HALF_M,
                above_view=top_cut,
            )
            # Where it stands IN the walking corridor: its nearest point there, not anywhere
            # along it. A wall beside the path, the chest turned 5 deg toward it, is one long
            # cluster that reaches the corridor ~7 ft out but starts 2 ft beside the wearer
            # (ToF review, 2026-11-09). None when too few of its points are in the corridor.
            inside = np.abs(xs) <= TOF_CORRIDOR_HALF_M
            f["path_m"] = (
                round(float(np.percentile(zs[inside], 3)), 3)
                if int(inside.sum()) >= TOF_OBSTACLE_MIN_POINTS
                else None
            )
            if kind == "obstacle":
                # Is it a wall? Filled in by analyse() for the nearest one on the path only
                # (_surface), judged by wall_verdict at guidance time (2026-11-10).
                f["surface"] = {
                    "ahead_m": f["path_m"],
                    "length_m": 0.0,
                    "flat_m": None,
                    "facing_deg": None,
                    "spans_view": False,
                    "corner": False,
                    "reaches_top": False,
                    "height_m": round(h_hi, 2),
                    "above_view": top_cut,
                }
            f["_sel"] = sel
            out.append(f)
        return out

    def _surface(self, xs, zs, cs, rs) -> Optional[Dict[str, Any]]:
        """Seen from above, how far a thing's surface runs flat across the walking path:
        {"ahead_m" (where it crosses the path), "length_m", "flat_m", "facing_deg" (90:
        square on), "spans_view" (the ToF's view edge to edge), "corner" (two flat arms at an
        angle), "reaches_top" (it goes on above the ToF's view along most of that)}, or None
        when too little of it is on the path. Its points become a top-down line, one sample
        per _WALL_COL_GROUP columns, grown left and right from the sample nearest straight
        ahead while each next one stays within TOF_WALL_FLAT_M of the arm's line so far."""
        if xs.size > _WALL_MAX_POINTS:  # every k-th point: the same picture, evenly thinned
            k = -(-xs.size // _WALL_MAX_POINTS)
            xs, zs, cs, rs = xs[::k], zs[::k], cs[::k], rs[::k]
        ng = -(-self.width // _WALL_COL_GROUP)
        g = np.minimum(np.asarray(cs) // _WALL_COL_GROUP, ng - 1).astype(np.int64)
        order = np.lexsort((zs, g))  # by column group, nearest first within each
        gs, xo, zo = g[order], xs[order], zs[order]
        counts = np.bincount(gs, minlength=ng)
        ends = np.cumsum(counts)
        starts = ends - counts
        groups = np.flatnonzero(counts >= _WALL_SAMPLE_MIN)
        if groups.size < 3:
            return None
        # Each sample: the nearest surface in those columns (the points within 5 cm of their
        # lower quartile of distance), not all of them: a blind set back behind the wall
        # below its sill is one surface with it, the wall's, whichever shows more. Sorted
        # nearest first, those points start each group: their median is the middle one.
        quartile = np.full(ng, -np.inf)
        quartile[groups] = zo[starts[groups] + counts[groups] // 4] + 0.05
        front = zo <= quartile[gs]
        nf = np.bincount(gs[front], minlength=ng)
        sz = zo[starts[groups] + nf[groups] // 2]
        sx = np.bincount(gs[front], weights=xo[front], minlength=ng)[groups] / nf[groups]
        # The top rows, per sample: the existing above_view (3% of a thing's points in them)
        # never held for a wall filling the picture, ~1.7% of its rows (2026-11-10).
        top = np.bincount(g, weights=(np.asarray(rs) < _TOP_ROWS), minlength=ng)[groups] >= 2
        inside = np.flatnonzero(np.abs(sx) <= TOF_CORRIDOR_HALF_M)
        if inside.size == 0:
            return None
        seed = int(inside[np.argmin(np.abs(sx[inside]))])
        left, right = (_grow(sx, sz, groups, seed, step) for step in (-1, 1))
        both = sorted(set(left) | set(right))
        if len(both) < 3:
            return None
        _c, u, rms = _line(sx[both], sz[both])
        corner = rms > TOF_WALL_FLAT_M
        if corner:  # two arms at an angle: each its own line, the longer one's direction
            longer = max((left, right), key=lambda a: _span(sx, sz, a))
            if len(longer) >= 2:
                u = _line(sx[longer], sz[longer])[1]
            arms = [a for a in (left, right) if len(a) >= 3]
            if arms:
                rms = max(_line(sx[a], sz[a])[2] for a in arms)
        on_path = [i for i in both if abs(sx[i]) <= TOF_CORRIDOR_HALF_M]
        return {
            # Where the surface crosses the path, from its samples: the cluster's nearest 3%
            # of points jumped 2.5 -> 1.2 m between frames with floor noise in it.
            "ahead_m": round(float(min(sz[i] for i in on_path)), 3),
            "length_m": round(_span(sx, sz, left) + _span(sx, sz, right), 2),
            "flat_m": round(float(rms), 3),
            "facing_deg": round(math.degrees(math.atan2(abs(u[0]), abs(u[1])))),
            "spans_view": bool(
                groups[both[0]] <= _WALL_EDGE_GROUPS
                and groups[both[-1]] >= ng - 1 - _WALL_EDGE_GROUPS
            ),
            "corner": bool(corner),
            "reaches_top": bool(np.mean(top[both]) >= 0.5),
        }

    # -- all of it

    def analyse(
        self,
        depth_mm: np.ndarray,
        confidence: Optional[np.ndarray] = None,
        t: float = 0.0,
    ) -> Dict[str, Any]:
        """One ToF frame (H x W mm, and the sensor's confidence) -> {"ok", "why", "plane",
        "hazards" (each with "_pts": some of its points in the ToF's frame, for the camera),
        "profile"}."""
        started = time.perf_counter()
        out: Dict[str, Any] = {"t": float(t), "ok": False, "why": "", "plane": None, "hazards": []}
        rays = self.projector.rays
        d = np.asarray(depth_mm, dtype=np.float64).reshape(-1)
        if d.size != rays.shape[0]:
            out["why"] = f"a frame of {d.size} pixels; the ToF has {rays.shape[0]}"
            return out
        keep = np.isfinite(d) & (d >= TOF_MIN_MM) & (d <= TOF_MAX_MM)
        if confidence is not None:
            keep &= np.asarray(confidence, dtype=np.float64).reshape(-1) >= TOF_MIN_CONFIDENCE
        idx = np.flatnonzero(keep)
        if idx.size < TOF_GROUND_MIN_POINTS:
            out["why"] = f"only {idx.size} depth pixels"
            return out
        P = rays[idx] * (d[idx, None] / 1000.0)  # the ToF's frame
        L = P @ self.to_level.T + self.t_level
        x, y, z = L[:, 0], L[:, 1], L[:, 2]
        # Set down on a desk: the desk top under the ToF is not something on the path.
        rest, on_rest = rest_surface(P)
        out["rest"] = rest
        off = ~on_rest
        out["near"] = near_counts(P[off], x[off], y[off], z[off])
        fit = self._fit_ground(x, y, z, float(t))
        if fit is None:
            out["why"] = "no ground seen (nothing level near the mount's height)"
            out["ms"] = round((time.perf_counter() - started) * 1000.0, 1)
            return out
        (a, b, c), on, how = fit
        k = math.sqrt(1.0 + a * a + b * b)
        h = ((a * x + b * z + c) - y) / k  # above the ground (y is down)
        rows = idx // self.width
        features, info = self._ground_features(x, z, h)
        for f in features:  # the points the camera checks: across the corridor, at the edge
            lo_z = f["ahead_m"] - 0.15
            hi_z = f["ahead_m"] + (0.9 if f["kind"].startswith("stairs") else 0.25)
            f["_sel"] = np.flatnonzero(
                (np.abs(x) <= TOF_CORRIDOR_HALF_M) & (z >= lo_z) & (z <= hi_z)
            )
        hazards = features + self._obstacles(x, z, h, rows, features)
        # The nearest tall thing on the walking path, as a surface: a wall or not
        # (wall_verdict). None: nothing tall stands on the path.
        standing = [
            f for f in hazards if f.get("surface") and f.get("in_path") and f.get("path_m") is not None
        ]
        out["front_surface"] = None
        if standing:
            f = min(standing, key=lambda f: f["path_m"])
            sel = f["_sel"]
            f["surface"].update(self._surface(x[sel], z[sel], idx[sel] % self.width, rows[sel]) or {})
            out["front_surface"] = {"t": float(t), **f["surface"]}
        for f in hazards:
            sel = f.pop("_sel")
            if sel.size > _CAMERA_PTS:
                sel = sel[self._rng.choice(sel.size, _CAMERA_PTS, replace=False)]
            f["_pts"] = P[sel]
        out.update(
            ok=True,
            plane={
                "height_m": round(c / k, 3),
                # How the ground lies against the mount's design: it falls away ahead
                # (+, the unit looks higher than designed) and to the right (+).
                "slope_ahead_deg": round(math.degrees(math.atan(b)), 1),
                "slope_right_deg": round(math.degrees(math.atan(a)), 1),
                "points": on,
                "fit": how,
            },
            hazards=hazards,
            profile=info,
            ms=round((time.perf_counter() - started) * 1000.0, 1),
        )
        return out


def _line(px: np.ndarray, pz: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    """(centre, unit direction, RMS of the perpendicular residuals) of top-down points."""
    c = np.array([float(np.mean(px)), float(np.mean(pz))])
    d = np.stack([px - c[0], pz - c[1]], axis=1)
    _w, v = np.linalg.eigh(d.T @ d)
    return c, v[:, 1], float(np.sqrt(np.mean((d @ v[:, 0]) ** 2)))


def _span(sx: np.ndarray, sz: np.ndarray, arm: List[int]) -> float:
    """How far an arm of samples reaches, end to end."""
    return float(math.hypot(sx[arm[-1]] - sx[arm[0]], sz[arm[-1]] - sz[arm[0]])) if len(arm) > 1 else 0.0


def _grow(sx: np.ndarray, sz: np.ndarray, groups: np.ndarray, seed: int, step: int) -> List[int]:
    """The samples from `seed` outward (step -1: left, +1: right) that stay on one flat
    surface: each next one near the last (_WALL_JUMP_M, no more than _WALL_GAP_GROUPS empty
    column groups between) and, from the third on, within TOF_WALL_FLAT_M of the line
    through the arm so far, once that reaches _WALL_LINE_MIN_M (a curtain's folds: three
    samples on one fold's slope made a line that the next fold left). One stray sample may
    be passed over."""
    arm, last, k, passed = [seed], seed, seed + step, False
    # The arm's line from running sums (n, x, z, xx, zz, xz): no refit per sample.
    sums = [1.0, sx[seed], sz[seed], sx[seed] ** 2, sz[seed] ** 2, sx[seed] * sz[seed]]
    while 0 <= k < len(sx):
        if abs(int(groups[k]) - int(groups[last])) > _WALL_GAP_GROUPS + 1:
            break
        ok = math.hypot(sx[k] - sx[last], sz[k] - sz[last]) <= _WALL_JUMP_M
        if ok and len(arm) >= 3 and _span(sx, sz, arm) >= _WALL_LINE_MIN_M:
            n, mx, mz = sums[0], sums[1] / sums[0], sums[2] / sums[0]
            cxx, czz = sums[3] / n - mx * mx, sums[4] / n - mz * mz
            cxz = sums[5] / n - mx * mz
            a = 0.5 * math.atan2(2.0 * cxz, cxx - czz)  # the principal direction
            ok = abs((sx[k] - mx) * math.sin(a) - (sz[k] - mz) * math.cos(a)) <= TOF_WALL_FLAT_M
        if ok:
            arm.append(k)
            for i, v in enumerate((1.0, sx[k], sz[k], sx[k] ** 2, sz[k] ** 2, sx[k] * sz[k])):
                sums[i] += v
            last, passed = k, False
        elif not passed:
            passed = True
        else:
            break
        k += step
    return arm


def wall_verdict(s: Optional[Dict[str, Any]]) -> Tuple[bool, str]:
    """(is it a wall, why) for one frame's front_surface (config TOF_WALL_*): tall, flat, and
    TOF_WALL_MIN_WIDTH_M across, or the ToF's whole view and TOF_WALL_MIN_SPAN_M."""
    if not s:
        return False, "nothing tall standing on the path"
    height = float(s.get("height_m") or 0.0)
    if not (s.get("reaches_top") or s.get("above_view") or height >= TOF_WALL_MIN_H_M):
        return False, f"{height:.1f} m tall, its top in view: not a wall"
    flat = s.get("flat_m")
    length = float(s.get("length_m") or 0.0)
    if flat is None or float(flat) > TOF_WALL_FLAT_M:
        return False, "no flat surface across the path"
    if not (
        length >= TOF_WALL_MIN_WIDTH_M
        or (s.get("spans_view") and length >= TOF_WALL_MIN_SPAN_M)
    ):
        return False, f"flat for only {length:.2f} m across: narrower than a wall"
    what = "a corner of two walls" if s.get("corner") else "a flat surface"
    return True, (
        f"{what} {length:.1f} m across{' (the whole view)' if s.get('spans_view') else ''}, "
        f"{float(s.get('ahead_m') or 0.0) / 0.3048:.1f} ft ahead, flat to {float(flat) * 100:.0f} cm"
    )


def confirmed_wall(
    ground: Optional[Dict[str, Any]], now: float
) -> Tuple[Optional[Dict[str, Any]], str]:
    """(the newest front_surface, why) when it is a wall and was one in TOF_WALL_CONFIRM[0]
    of the last TOF_WALL_CONFIRM[1] ToF frames at about the same distance; (None, why not)
    otherwise, or when the analysis is older than TOF_HAZARD_FRESH_S."""
    from config import TOF_HAZARD_FRESH_S

    if not ground or not ground.get("ok"):
        return None, (ground or {}).get("why") or "no ToF analysis"
    t = ground.get("t")
    if not isinstance(t, (int, float)) or not -1.0 <= now - t <= TOF_HAZARD_FRESH_S:
        return None, "the ToF analysis is stale"
    recent = list(ground.get("front_surfaces") or [])[-int(TOF_WALL_CONFIRM[1]) :]
    latest = recent[-1] if recent else None
    ok, why = wall_verdict(latest)
    if not ok:
        return None, why
    agree = sum(
        1
        for s in recent
        if s
        and wall_verdict(s)[0]
        and abs(float(s.get("ahead_m") or 0.0) - float(latest.get("ahead_m") or 0.0)) <= 0.4
    )
    if agree < int(TOF_WALL_CONFIRM[0]):
        return None, f"a wall in {agree} of the last {len(recent)} ToF frames: not yet confirmed"
    return latest, why


def _feature(kind: str, ahead: float, **kw) -> Dict[str, Any]:
    """One hazard as the rest of the app reads it."""
    f: Dict[str, Any] = {
        "kind": kind,
        "ahead_m": round(float(ahead), 2),
        "ahead_ft": round(float(ahead) / 0.3048, 1),
        "side_m": 0.0,
        "width_m": round(2 * TOF_CORRIDOR_HALF_M, 2),
        "in_path": True,
        "measured": True,
    }
    for key, val in kw.items():
        if key == "side":
            f["side_m"] = round(float(val), 2)
        elif key == "width":
            f["width_m"] = round(float(val), 2)
        elif key in ("height", "depth", "rise", "low", "length", "top"):
            f[f"{key}_m"] = None if val is None else round(float(val), 3)
        else:
            f[key] = val
    f["bearing_deg"] = round(math.degrees(math.atan2(f["side_m"], max(0.1, float(ahead)))), 1)
    return f


# ---------------------------------------------------------------- the camera's word on it


def _unseen(how: str, box=None) -> Dict[str, Any]:
    return {"status": "unseen", "how": how, "name": None, "box": box}


def _overlap(a: Sequence[float], b: Sequence[float]) -> float:
    """The shared area as a share of the smaller box's."""
    w = min(a[2], b[2]) - max(a[0], b[0])
    h = min(a[3], b[3]) - max(a[1], b[1])
    if w <= 0 or h <= 0:
        return 0.0
    small = min((a[2] - a[0]) * (a[3] - a[1]), (b[2] - b[0]) * (b[3] - b[1]))
    return float(w * h / small) if small > 0 else 0.0


def _lines(gray: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """(edge map, line segments x1 y1 x2 y2) of a grey crop: Canny, then the probabilistic
    Hough transform. Fixed thresholds: ones from the crop's median brightness (lo 0.66 x,
    hi 1.33 x) missed a kerb's ~20-level step on a bright pavement (desk test, 2026-11-06);
    a dark picture is the view check's to refuse, not this one's."""
    import cv2

    edges = cv2.Canny(gray, 25, 75)
    h, w = gray.shape[:2]
    side = max(1, min(h, w))
    found = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180.0,
        threshold=max(15, int(0.10 * side)),
        minLineLength=max(15, int(0.20 * side)),
        maxLineGap=6,
    )
    # (N, 1, 4) in most OpenCV builds, (N, 4) in some: either way, one row per segment.
    segs = np.zeros((0, 4)) if found is None else np.asarray(found, np.float64).reshape(-1, 4)
    return edges, segs


def camera_check(
    hazard: Dict[str, Any],
    calib: df.Calibration,
    cam: Optional[Dict[str, Any]],
    t: float,
) -> Dict[str, Any]:
    """Does the front camera see it? {"status": "confirmed" | "no_evidence" | "unseen",
    "how", "name" (a SIMPLE_NAMED_OBJECTS word, from a detection over it), "box" (where it
    lands on the picture, px)}. `cam`: {"picture" (BGR), "t", "view", "detections",
    "simulated"} — the front camera's newest, or None."""
    import cv2

    pts = hazard.get("_pts")
    if cam is None or cam.get("picture") is None:
        return _unseen("no camera picture")
    if pts is None or len(pts) < 5:
        return _unseen("too few points to place on the picture")
    pic = cam["picture"]
    H, W = pic.shape[:2]
    Pc = np.asarray(pts, dtype=np.float64) @ calib.R.T + calib.t
    Pc = Pc[Pc[:, 2] > 0.1]
    if len(Pc) < 5:
        return _unseen("behind the camera")
    u, v = calib.cam.project(Pc)
    u = (u + 0.5) * (W / calib.cam.width) - 0.5  # onto this picture's size
    v = (v + 0.5) * (H / calib.cam.height) - 0.5
    inside = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    if inside.sum() < 5:
        return _unseen("outside the camera's picture")
    x1, x2 = (float(q) for q in np.percentile(u[inside], [2, 98]))
    y1, y2 = (float(q) for q in np.percentile(v[inside], [2, 98]))
    box = [int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))]
    if cam.get("simulated"):
        return _unseen("the camera picture is simulated", box)
    gap = abs(float(cam.get("t") or 0.0) - float(t))
    if gap > TOF_CAMERA_MAX_AGE_S:
        return _unseen(f"the camera picture is {gap:.2f} s from the ToF frame", box)
    view = cam.get("view") or {}
    if view.get("dark"):
        return _unseen("the camera picture is dark", box)
    if view.get("blurred"):
        return _unseen("the camera picture is blurred", box)

    kind = hazard["kind"]
    m = float(TOF_CAMERA_MARGIN_PX)
    near_box = (x1 - m / 2, y1 - m / 2, x2 + m / 2, y2 + m / 2)
    best = None
    for det in cam.get("detections") or ():
        try:
            bb = [float(q) for q in det.get("bbox") or ()]
        except (TypeError, ValueError):
            continue
        if len(bb) != 4:
            continue
        ov = _overlap(near_box, bb)
        if ov >= 0.3 and (best is None or ov > best[0]):
            best = (ov, det)
    if best is not None:
        cls = str(best[1].get("class_name") or "").lower()
        word = _NAMES.get(cls)
        if kind not in _FROM_PROFILE or (word == "Stairs" and kind.startswith("stairs")):
            return {"status": "confirmed", "how": f"the detector's {cls}", "name": word, "box": box}

    gray_all = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY) if pic.ndim == 3 else pic
    cx1, cx2 = int(max(0, x1 - m)), int(min(W, x2 + m))
    cy2 = int(min(H, y2 + m))
    cy1 = 0 if kind == "pole" else int(max(0, y1 - m))  # a pole goes on up the picture
    if cx2 - cx1 < 8 or cy2 - cy1 < 8:
        return {"status": "no_evidence", "how": "too small on the picture", "name": None, "box": box}
    crop = np.ascontiguousarray(gray_all[cy1:cy2, cx1:cx2])
    edges, segs = _lines(crop)
    ch, cw = crop.shape[:2]
    dx, dy = segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1]
    length = np.hypot(dx, dy)
    ang = np.degrees(np.arctan2(dy, dx))
    ang = np.where(ang > 90, ang - 180, np.where(ang < -90, ang + 180, ang))
    if kind in _FROM_PROFILE:
        level = (np.abs(ang) <= 25.0) & (length >= 0.25 * cw)
        rows = np.sort((segs[level, 1] + segs[level, 3]) / 2.0)
        distinct = int(rows.size > 0) + int(np.sum(np.diff(rows) > 6.0)) if rows.size else 0
        need = 2 if kind.startswith("stairs") else 1
        if distinct >= need:
            return {
                "status": "confirmed",
                "how": f"{distinct} level edge{'s' if distinct != 1 else ''} in the picture",
                "name": None,
                "box": box,
            }
        return {
            "status": "no_evidence",
            "how": f"{distinct} level edge{'s' if distinct != 1 else ''} where {need} were looked for",
            "name": None,
            "box": box,
        }
    if kind == "pole":
        mid = (segs[:, 0] + segs[:, 2]) / 2.0 + cx1
        upright = (np.abs(ang) >= 75.0) & (length >= max(20.0, 0.15 * ch))
        upright &= (mid >= x1 - m / 2) & (mid <= x2 + m / 2)
        rising = upright & (np.minimum(segs[:, 1], segs[:, 3]) + cy1 < y1 - 10)
        if rising.any():
            return {"status": "confirmed", "how": "an upright edge going on above the ToF's view", "name": None, "box": box}
        if upright.any():
            return {"status": "confirmed", "how": "an upright edge there", "name": None, "box": box}
        return {"status": "no_evidence", "how": "no upright edge there", "name": None, "box": box}
    density = float(np.count_nonzero(edges)) / float(max(1, edges.size))
    long_line = bool((length >= 0.4 * min(ch, cw)).any())
    if density >= TOF_CAMERA_EDGE_DENSITY or long_line:
        return {
            "status": "confirmed",
            "how": f"an outline there ({density * 100:.0f} % edges"
            + (", a long edge)" if long_line else ")"),
            "name": None,
            "box": box,
        }
    return {
        "status": "no_evidence",
        "how": f"a plain patch ({density * 100:.1f} % edges)",
        "name": None,
        "box": box,
    }


# ---------------------------------------------------------------- over time, and what to say


_CAMERA_MEMORY_S = 3.0  # the camera's verdict on a hazard it can no longer see is kept this long


class _Tracker:
    """A hazard is reported once seen in `need` of the last `window` analyses (the nearest
    in the path of each kind), and while it is still in the newest two.

    The camera's verdict is remembered while the hazard stays tracked: the camera sees the
    ground only from ~2 m ahead (it looks 10 deg down, 26 deg either way), so an edge it
    confirmed at 2.5 m is out of its picture by 1.5 m — "unseen" then means "as it was"."""

    def __init__(self, need: int = TOF_HAZARD_CONFIRM[0], window: int = TOF_HAZARD_CONFIRM[1]):
        self.need = int(need)
        self.hist: collections.deque = collections.deque(maxlen=int(window))
        self.memory: Dict[str, Tuple[float, Dict[str, Any]]] = {}  # kind -> (t, verdict)

    def update(self, hazards: List[Dict[str, Any]], t: float = 0.0) -> List[Dict[str, Any]]:
        by_kind: Dict[str, Dict[str, Any]] = {}
        for hz in hazards:
            if not hz.get("in_path"):
                continue
            k = hz["kind"]
            if k not in by_kind or hz["ahead_m"] < by_kind[k]["ahead_m"]:
                by_kind[k] = hz
        self.hist.append(by_kind)
        for kind in list(self.memory):  # lost from the whole window: forgotten
            if not any(kind in bk for bk in self.hist):
                del self.memory[kind]
        recent = list(self.hist)[-2:]
        out = []
        for kind in KINDS:
            seen = [bk[kind] for bk in self.hist if kind in bk]
            latest = next((bk[kind] for bk in reversed(recent) if kind in bk), None)
            if latest is None:
                continue
            cam = latest.get("camera") or {}
            if cam.get("status") in ("confirmed", "no_evidence"):
                self.memory[kind] = (float(t), cam)
            if len(seen) < self.need:
                continue
            ahead = [s["ahead_m"] for s in seen]
            if abs(latest["ahead_m"] - float(np.median(ahead))) > 0.8:
                continue  # not one thing being approached: something else each time
            cams = [s.get("camera") or {} for s in seen]
            said = next((c for c in reversed(cams) if c.get("status") == "confirmed"), None)
            if said is None and cam.get("status") == "unseen" and kind in self.memory:
                mt, verdict = self.memory[kind]
                if 0.0 <= float(t) - mt <= _CAMERA_MEMORY_S:
                    said = {
                        **verdict,
                        "how": f"{float(t) - mt:.1f} s ago, before it left the picture: "
                        f"{verdict.get('how')}",
                        "box": cam.get("box"),
                    }
            out.append({**latest, "frames": len(seen), "camera": said or cam})
        return out

    def reset(self) -> None:
        self.hist.clear()
        self.memory.clear()


def speak_rule(h: Dict[str, Any]) -> Tuple[bool, str]:
    """(whether the guidance may say this confirmed hazard, why)."""
    kind = h["kind"]
    if not h.get("in_path"):
        return False, "beside the walking path"
    if h["ahead_m"] > TOF_HAZARD_WARN_M:
        return False, f"{h['ahead_ft']} ft away: said from {TOF_HAZARD_WARN_M / 0.3048:.0f} ft"
    cam = h.get("camera") or {}
    status, how = cam.get("status"), cam.get("how") or ""
    if not h.get("measured", True):
        if status == "confirmed":
            return True, f"the floor ends with nothing past it, and the camera sees the edge ({how})"
        return False, "the floor ends with nothing past it: said only when the camera sees the edge"
    if status == "confirmed":
        return True, f"confirmed by the camera: {how}"
    if status == "unseen":
        return True, f"the camera cannot judge it ({how}): the ToF alone"
    fall = max(float(h.get("depth_m") or 0.0), float(h.get("height_m") or 0.0))
    if kind in FALLS and fall >= TOF_FALL_ALONE_M:
        return True, f"a fall of {fall / 0.3048:.1f} ft: said on the ToF alone ({how})"
    return False, f"the ToF only: the camera sees nothing there ({how})"


def choose_top(hazards: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """The one to say: a fall within TOF_HAZARD_STOP_M first, then the nearest, the worse
    kind on a tie."""
    sayable = [h for h in hazards if h.get("speak")]
    if not sayable:
        return None
    return min(
        sayable,
        key=lambda h: (
            0 if h["kind"] in FALLS and h["ahead_m"] <= TOF_HAZARD_STOP_M else 1,
            round(h["ahead_m"], 1),
            _SEVERITY.get(h["kind"], 99),
        ),
    )


def _public(h: Dict[str, Any]) -> Dict[str, Any]:
    return {k: v for k, v in h.items() if not k.startswith("_")}


class GroundHazards:
    """Analyses + camera checks + tracking: step() once per new ToF frame."""

    def __init__(self, projector: Optional[df.DepthProjector] = None):
        self._follow = projector is None  # the app's calibration, reloaded with it
        self.analyser = GroundAnalyser(projector)
        self.tracker = _Tracker()
        # The nearest tall thing on the path, as a surface, for the last few frames.
        self._surfaces: collections.deque = collections.deque(maxlen=int(TOF_WALL_CONFIRM[1]))
        self._lock = threading.Lock()
        self._snap: Dict[str, Any] = {
            "ok": False,
            "why": "no ToF frame analysed yet",
            "t": None,
            "hazards": [],
            "seen": [],
            "top": None,
            "plane": None,
        }

    def step(
        self,
        depth_mm: np.ndarray,
        confidence: Optional[np.ndarray],
        t: float,
        camera: Optional[Callable[[], Optional[Dict[str, Any]]]] = None,
    ) -> Dict[str, Any]:
        """Analyse one frame; `camera()` gives the front camera's inputs (camera_check),
        and is called only if something is there to check."""
        if self._follow and df.get_projector() is not self.analyser.projector:
            self.analyser.use(df.get_projector())
        res = self.analyser.analyse(depth_mm, confidence, t)
        calib = self.analyser.projector.calib
        cam, fetched = None, False
        for hz in res["hazards"]:
            if hz["in_path"] and hz["ahead_m"] <= TOF_HAZARD_WARN_M + 0.5:
                if not fetched:
                    fetched = True
                    try:
                        cam = camera() if camera is not None else None
                    except Exception as e:
                        cam = None
                        res.setdefault("camera_error", f"{type(e).__name__}: {e}")
                hz["camera"] = camera_check(hz, calib, cam, t)
            else:
                hz["camera"] = {"status": "skipped", "how": "beside the path or far off", "name": None, "box": None}
        self._surfaces.append(res.get("front_surface") if res["ok"] else None)
        seen = [_public(h) for h in res["hazards"]]
        confirmed = [_public(h) for h in self.tracker.update(res["hazards"] if res["ok"] else [], t)]
        for h in confirmed:
            h["speak"], h["why"] = speak_rule(h)
        snap = {
            "t": float(t),
            "analysed_at": time.time(),
            "ok": bool(res["ok"]),
            "why": res.get("why", ""),
            "plane": res.get("plane"),
            "hazards": confirmed,
            "seen": seen,
            "top": choose_top(confirmed),
            "ms": res.get("ms"),
            "calibration": calib.source,
            "speak_enabled": _speak_enabled(),
            "near": res.get("near"),
            "rest": res.get("rest"),
            "front_surfaces": list(self._surfaces),
        }
        if res.get("camera_error"):
            snap["camera_error"] = res["camera_error"]
        if res.get("profile"):
            snap["floor"] = res["profile"].get("floor")
        with self._lock:
            self._snap = snap
        return snap

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._snap)

    def reset(self) -> None:
        self.tracker.reset()
        self._surfaces.clear()


def _speak_enabled() -> bool:
    from config import TOF_HAZARDS_SPEAK

    return bool(TOF_HAZARDS_SPEAK)


# ---------------------------------------------------------------- the app's worker


_hz: Optional[GroundHazards] = None
_hz_lock = threading.Lock()
_thread: Optional[threading.Thread] = None
_stop = threading.Event()
_errors = {"count": 0, "last": None, "logged_at": 0.0}


def get_ground_hazards() -> GroundHazards:
    global _hz
    with _hz_lock:
        if _hz is None:
            _hz = GroundHazards()
        return _hz


def snapshot() -> Dict[str, Any]:
    """The newest analysis for the guidance and /tof/hazards (age_s: since its ToF frame)."""
    snap = get_ground_hazards().snapshot()
    snap["running"] = _thread is not None and _thread.is_alive()
    snap["age_s"] = None if snap.get("t") is None else round(time.time() - snap["t"], 2)
    snap["errors"] = dict(_errors)
    return snap


def _camera_inputs() -> Optional[Dict[str, Any]]:
    """The front camera's newest picture, its view check and detections (camera_check)."""
    from utils import pi_camera_utils as pc

    with pc.frame_lock:
        mosaic, t = pc.current_frame, pc.current_frame_t
        simulated = bool(getattr(pc, "current_frame_simulated", False))
        if mosaic is None:
            return None
        quad = pc._sector_slice(mosaic, "front").copy()
    with pc._gen_lock:
        e = pc.detection_stats.get("front") or {}
        dets = list(e.get("detections") or [])
        view = dict(e.get("view") or {})
        det_t = e.get("frame_time")
    if det_t is None or abs(float(det_t) - float(t)) > 0.5:
        dets = []  # from another moment: they would mark where things were
    return {
        "picture": pc._to_bgr(quad),
        "t": float(t),
        "view": view,
        "detections": dets,
        "simulated": simulated,
    }


def _run() -> None:
    from utils.tof_camera import latest_raw

    last = None
    period = 1.0 / max(1.0, float(TOF_HAZARD_HZ))
    while not _stop.wait(period):
        try:
            raw = latest_raw()
            if raw is None or raw[0] == last:
                continue
            last = raw[0]
            get_ground_hazards().step(raw[1], raw[2], raw[0], _camera_inputs)
        except Exception as e:
            _errors["count"] += 1
            _errors["last"] = f"{type(e).__name__}: {e}"
            now = time.time()
            if now - _errors["logged_at"] > 30.0:
                _errors["logged_at"] = now
                print(f"[WARN] ToF hazards: {_errors['last']}")


def start_ground_hazards() -> bool:
    """Start the analysis thread (only with TOF_ENABLED and TOF_HAZARDS_ENABLED)."""
    global _thread
    if not (TOF_ENABLED and TOF_HAZARDS_ENABLED):
        return False
    if _thread is not None and _thread.is_alive():
        return False
    _stop.clear()
    _thread = threading.Thread(target=_run, name="tof-hazards", daemon=True)
    _thread.start()
    return True


def stop_ground_hazards(timeout_s: float = 2.0) -> None:
    _stop.set()
    if _thread is not None:
        _thread.join(timeout_s)


def draw(
    picture_bgr: np.ndarray,
    snap: Optional[Dict[str, Any]],
    scale: Tuple[float, float] = (1.0, 1.0),
) -> np.ndarray:
    """Each confirmed hazard where it lands on the picture, labelled in feet: red if it would
    be said, amber if not (the reason is in /tof/hazards). `scale`: this picture's size
    against the camera's (draw_rgbd draws it smaller for the page)."""
    import cv2

    out = picture_bgr
    if not snap:
        return out
    sx, sy = scale
    big = out.shape[1] >= 900
    fs, thick = (0.6, 2) if big else (0.45, 1)
    font = cv2.FONT_HERSHEY_SIMPLEX
    for h in snap.get("hazards") or []:
        box = (h.get("camera") or {}).get("box")
        if not box:
            continue
        x1, x2 = int(round(box[0] * sx)), int(round(box[2] * sx))
        y1, y2 = int(round(box[1] * sy)), int(round(box[3] * sy))
        colour = (0, 0, 255) if h.get("speak") else (0, 170, 255)
        cam = (h.get("camera") or {}).get("status") or "?"
        text = f"{h['kind'].replace('_', ' ')} {h['ahead_ft']:.1f} ft [{cam}]"
        cv2.rectangle(out, (x1, y1), (x2, y2), colour, thick + 1)
        (tw, th), base = cv2.getTextSize(text, font, fs, thick)
        x1 = max(0, min(x1, out.shape[1] - tw - 7))  # the label inside the picture
        ty = y2 + th + 8 if y2 + th + 10 < out.shape[0] else max(th + 4, y1 - 6)
        cv2.rectangle(out, (x1, ty - th - 4), (x1 + tw + 6, ty + base), (0, 0, 0), -1)
        cv2.putText(out, text, (x1 + 3, ty), font, fs, colour, thick, cv2.LINE_AA)
    return out
