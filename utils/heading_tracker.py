"""Heading tracker — "am I still walking straight?", from the LiDAR and the gyro."""

from collections import deque
from typing import Any, Dict, List, NamedTuple, Optional, Sequence, Tuple

import math
import threading
import time

import numpy as np

from utils.gait_calibration import get_stride, is_measured, note_learned
from utils.gyro_reader import get_gyro

from config import (
    CLOCK_FUTURE_TOLERANCE_S,
    GYRO_ENABLED,
    GYRO_GAIT_CONFIRMED,
    GYRO_MOUNTING_CONFIRMED,
    GYRO_RECAL_MAX_YAW_DPS,
    GYRO_SEED_MOTION_FIT,
    GYRO_STRIDE_DEFAULT_M,
    GYRO_STRIDE_LEARN_TAU_S,
    GYRO_STRIDE_MAX_M,
    GYRO_STRIDE_MIN_M,
    HEADING_BIN_DEG,
    HEADING_FIT_MAX_RESEEDS,
    HEADING_INTERP_MAX_GAP_DEG,
    HEADING_INTERP_MAX_JUMP_MM,
    HEADING_MATCH_MAX_RANGE_MM,
    HEADING_MATCH_MIN_BINS,
    HEADING_MAX_LATERAL_MPS,
    HEADING_MAX_SPEED_MPS,
    HEADING_MAX_YAW_STEP_DEG,
    HEADING_MIN_FORWARD_SENS,
    HEADING_MIN_MATCH_QUALITY,
    HEADING_MIN_MOVING_MPS,
    HEADING_NOMINAL_SPEED_MPS,
    HEADING_SENS_SMOOTH_BINS,
    HEADING_SPEED_SE_LIMIT_MPS,
    HEADING_TRIM_FRACTION,
    HEADING_WINDOW_S,
)

N_BINS = int(round(360.0 / HEADING_BIN_DEG))
_BIN_CENTRES_RAD = np.radians((np.arange(N_BINS) + 0.5) * HEADING_BIN_DEG)

# Largest rotation searched, in bins.
_MAX_SHIFT = max(1, int(round(HEADING_MAX_YAW_STEP_DEG / HEADING_BIN_DEG)))

from config import HEADING_MAX_INCIDENCE_SEC, HEADING_SENS_JUMP_FRACTION  # noqa: E402


def build_profile(points: Sequence[Tuple[float, float]]) -> np.ndarray:
    """Resample a scan to the range at each bin's CENTRE bearing, NaN where nothing hit.

    Each centre is linearly interpolated between the two samples either side of it.
    Taking whichever sample fell in the bin instead made the profile depend on where
    the samples landed: the A2M12's bearings creep ~0.32 deg per revolution, and that
    read as a steady false turn (about -116 deg/min standing still, 2026-10-06)."""
    profile = np.full(N_BINS, np.nan, dtype=np.float64)
    if not len(points):
        return profile

    arr = np.asarray(points, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 2:
        return profile

    angles, dists = arr[:, 0], arr[:, 1]
    ok = np.isfinite(angles) & np.isfinite(dists) & (dists > 0)
    if not ok.any():
        return profile

    a = np.mod(angles[ok], 360.0)
    d = dists[ok]
    order = np.argsort(a, kind="stable")
    a, d = a[order], d[order]
    # Wrap: the last sample before 0 deg and the first after 360 deg.
    a = np.concatenate(([a[-1] - 360.0], a, [a[0] + 360.0]))
    d = np.concatenate(([d[-1]], d, [d[0]]))

    centres = (np.arange(N_BINS) + 0.5) * HEADING_BIN_DEG
    hi = np.searchsorted(a, centres, side="right")     # a[hi-1] <= centre < a[hi]
    a0, a1, d0, d1 = a[hi - 1], a[hi], d[hi - 1], d[hi]
    gap = a1 - a0
    w = (centres - a0) / gap
    # Never bridge a gap in the scan, nor the step from one object to another.
    bridge = (gap <= HEADING_INTERP_MAX_GAP_DEG) & (np.abs(d1 - d0) <= HEADING_INTERP_MAX_JUMP_MM)
    # Unbridged: the nearer-in-bearing sample, if it lies within this bin.
    left_nearer = (centres - a0) <= (a1 - centres)
    near_d = np.where(left_nearer, d0, d1)
    near_off = np.where(left_nearer, centres - a0, a1 - centres)
    profile = np.where(bridge, d0 + w * (d1 - d0),
                       np.where(near_off <= HEADING_BIN_DEG / 2.0, near_d, np.nan))
    return profile


def _match_cost(prev: np.ndarray, cur: np.ndarray, shift: int) -> Tuple[float, int]:
    """Median |range difference| between the two profiles at a given rotation."""
    rolled = np.roll(prev, -shift)
    both = np.isfinite(rolled) & np.isfinite(cur)
    n = int(both.sum())
    if n < HEADING_MATCH_MIN_BINS:
        return float("inf"), n
    return float(np.median(np.abs(cur[both] - rolled[both]))), n


def estimate_yaw(prev: np.ndarray, cur: np.ndarray) -> Dict[str, Any]:
    """Yaw turned between two profiles, in degrees, positive = to the RIGHT."""
    prev = np.where(prev <= HEADING_MATCH_MAX_RANGE_MM, prev, np.nan)
    cur = np.where(cur <= HEADING_MATCH_MAX_RANGE_MM, cur, np.nan)

    shifts = range(-_MAX_SHIFT, _MAX_SHIFT + 1)
    costs: List[float] = []
    bins: List[int] = []
    for s in shifts:
        c, n = _match_cost(prev, cur, s)
        costs.append(c)
        bins.append(n)

    cost_arr = np.asarray(costs)
    if not np.isfinite(cost_arr).any():
        return {"yaw_deg": 0.0, "quality": 0.0, "cost": None, "bins": 0}

    best_i = int(np.nanargmin(np.where(np.isfinite(cost_arr), cost_arr, np.nan)))
    best_cost = float(cost_arr[best_i])
    best_shift = list(shifts)[best_i]

    offset = 0.0
    if 0 < best_i < len(costs) - 1:
        c0, c1, c2 = cost_arr[best_i - 1], cost_arr[best_i], cost_arr[best_i + 1]
        if np.isfinite(c0) and np.isfinite(c2):
            denom = c0 - 2.0 * c1 + c2
            if denom > 1e-9:
                offset = float(np.clip(0.5 * (c0 - c2) / denom, -0.5, 0.5))

    finite = cost_arr[np.isfinite(cost_arr)]
    typical = float(np.median(finite)) if finite.size else 0.0
    quality = 0.0 if typical <= 1e-6 else float(np.clip(1.0 - best_cost / typical, 0.0, 1.0))

    return {
        "yaw_deg": (best_shift + offset) * HEADING_BIN_DEG,
        "quality": quality,
        "cost": best_cost,
        "bins": bins[best_i],
    }


def _smoothed(profile: np.ndarray, window: int) -> np.ndarray:
    """Moving average over `window` bearing bins, NaN-aware and wrapping."""
    if window <= 1:
        return profile
    present = np.isfinite(profile)
    values = np.where(present, profile, 0.0)
    half = window // 2
    total = sum(np.roll(values, k) for k in range(-half, half + 1))
    count = sum(np.roll(present.astype(np.float64), k)
                for k in range(-half, half + 1))
    averaged = np.where(count > 0, total / np.maximum(count, 1.0), np.nan)
    return np.where(present, averaged, np.nan)


def _straddles_range_jump(profile: np.ndarray) -> np.ndarray:
    """Bins whose sensitivity gradient mixes the two sides of a depth jump."""
    n = profile.size
    near = np.zeros(n, dtype=bool)
    present = np.flatnonzero(np.isfinite(profile))
    if present.size < 2:
        return near
    half = HEADING_SENS_SMOOTH_BINS // 2
    reach = 2 * half + 2                      # widest pair one stencil can hold
    nxt = np.roll(present, -1)
    gap = (nxt - present) % n
    a, b = profile[present], profile[nxt]
    jumped = ((gap > 0) & (gap <= reach)
              & (np.abs(b - a) > HEADING_SENS_JUMP_FRACTION * np.minimum(a, b) * gap))
    # span[j] = bins from a flagged return j to the next return (0 = no jump).
    span = np.zeros(n, dtype=np.int64)
    span[present[jumped]] = gap[jumped]
    for s in range(reach + 1):
        start = np.roll(span, half + 1 - s)   # span of the pair starting at i-h-1+s
        near |= (start > 0) & (start <= reach - s)
    return near


def _forward_sensitivity(cur: np.ndarray, valid: np.ndarray) -> float:
    """How much one millimetre of forward travel moves a range reading, RMS."""
    smooth = _smoothed(cur, HEADING_SENS_SMOOTH_BINS)
    bin_rad = math.radians(HEADING_BIN_DEG)
    grad = (np.roll(smooth, -1) - np.roll(smooth, 1)) / (2.0 * bin_rad)
    neighbours_ok = (np.isfinite(np.roll(smooth, -1))
                     & np.isfinite(np.roll(smooth, 1)))
    grad = np.where(neighbours_ok, grad, np.nan)

    with np.errstate(invalid="ignore", divide="ignore"):
        tan_psi = grad / cur
        sec_psi = np.sqrt(1.0 + tan_psi ** 2)
    alpha = _BIN_CENTRES_RAD - np.arctan(tan_psi)

    counted = valid & np.isfinite(grad)
    n = int(counted.sum())
    if n <= 0:
        return 0.0
    usable = (counted & ~_straddles_range_jump(cur)
              & (sec_psi <= HEADING_MAX_INCIDENCE_SEC))
    if not usable.any():
        return 0.0
    forward = -np.cos(alpha[usable]) * sec_psi[usable]
    lateral = -np.sin(alpha[usable]) * sec_psi[usable]
    yaw = grad[usable]
    yaw_norm = float(np.linalg.norm(yaw))
    others = np.column_stack(((yaw / yaw_norm) if yaw_norm > 0 else yaw, lateral))
    try:
        coef, *_ = np.linalg.lstsq(others, forward, rcond=None)
        independent = forward - others @ coef
    except np.linalg.LinAlgError:
        independent = np.zeros_like(forward)
    return float(np.linalg.norm(independent) / math.sqrt(n))


def estimate_motion(prev: np.ndarray, cur: np.ndarray,
                    coarse_yaw_deg: float) -> Dict[str, Any]:
    """Refine yaw and recover translation together, in a point-to-PLANE fit."""
    shift = int(round(coarse_yaw_deg / HEADING_BIN_DEG))
    rolled = np.roll(prev, -shift)

    bin_rad = math.radians(HEADING_BIN_DEG)
    grad = (np.roll(cur, -1) - np.roll(cur, 1)) / (2.0 * bin_rad)
    neighbours_ok = np.isfinite(np.roll(cur, -1)) & np.isfinite(np.roll(cur, 1))
    grad = np.where(neighbours_ok, grad, np.nan)

    with np.errstate(invalid="ignore", divide="ignore"):
        tan_psi = grad / cur                 # tan of the incidence angle
        psi = np.arctan(tan_psi)             # ray-to-normal angle
        sec_psi = np.sqrt(1.0 + tan_psi ** 2)

    alpha = _BIN_CENTRES_RAD - psi           # bearing of the surface normal

    valid = (
        np.isfinite(rolled) & np.isfinite(cur) & np.isfinite(grad)
        & (rolled <= HEADING_MATCH_MAX_RANGE_MM)
        & (cur <= HEADING_MATCH_MAX_RANGE_MM)
        & (sec_psi <= HEADING_MAX_INCIDENCE_SEC)
    )

    n = int(valid.sum())
    forward_sens = _forward_sensitivity(cur, valid)
    if n < HEADING_MATCH_MIN_BINS:
        return {"yaw_deg": coarse_yaw_deg, "dx_mm": 0.0, "dy_mm": 0.0,
                "quality": 0.0, "bins": n, "forward_sens": forward_sens}

    delta = cur[valid] - rolled[valid]       # negative = the world came closer
    design = np.column_stack((
        grad[valid],                                    # residual yaw, radians
        -np.cos(alpha[valid]) * sec_psi[valid],         # forward, mm
        -np.sin(alpha[valid]) * sec_psi[valid],         # right, mm
    ))

    def _solve(A, b):
        try:
            sol, *_ = np.linalg.lstsq(A, b, rcond=None)
            return sol
        except np.linalg.LinAlgError:
            return None

    sol = _solve(design, delta)
    if sol is None:
        return {"yaw_deg": coarse_yaw_deg, "dx_mm": 0.0, "dy_mm": 0.0,
                "quality": 0.0, "bins": n, "forward_sens": forward_sens}

    residual = np.abs(delta - design @ sol)
    keep_n = max(HEADING_MATCH_MIN_BINS, int(n * (1.0 - HEADING_TRIM_FRACTION)))
    kept_delta, kept_design = delta, design
    if keep_n < n:
        keep = np.argsort(residual)[:keep_n]
        refit = _solve(design[keep], delta[keep])
        if refit is not None:
            sol = refit
            kept_delta, kept_design = delta[keep], design[keep]
            residual = np.abs(kept_delta - kept_design @ sol)

    spread = float(np.median(np.abs(kept_delta - np.median(kept_delta)))) + 1e-6
    rms = float(np.sqrt(np.mean(residual ** 2))) if residual.size else 0.0
    quality = float(np.clip(1.0 - rms / (3.0 * spread), 0.0, 1.0))

    se = _standard_errors(kept_design, rms)

    wanted_deg = math.degrees(sol[0])
    refine_deg = float(np.clip(wanted_deg, -HEADING_BIN_DEG, HEADING_BIN_DEG))

    return {
        "yaw_deg": shift * HEADING_BIN_DEG + refine_deg,
        # The fit wanted to move further than one bin from its seed.
        "clipped": abs(wanted_deg) > HEADING_BIN_DEG,
        "dx_mm": float(sol[1]),
        "dy_mm": float(sol[2]),
        "dx_se_mm": se[1],
        "dy_se_mm": se[2],
        "forward_sens": forward_sens,
        "quality": quality,
        "bins": n,
    }


def fit_motion(prev: np.ndarray, cur: np.ndarray, coarse_yaw_deg: float) -> Dict[str, Any]:
    """estimate_motion, re-seeded from its own answer until it stops at the edge of its reach.

    The rotation-only search reads forward walking as a turn of a degree or two (the side
    walls' bearings slide back), and the yaw + translation fit can only move +-1 bin from
    its seed — so from a wrong seed it stopped at the edge, one bin off, every scan."""
    motion = estimate_motion(prev, cur, coarse_yaw_deg)
    seen = {int(round(coarse_yaw_deg / HEADING_BIN_DEG))}
    for _ in range(HEADING_FIT_MAX_RESEEDS):
        if not motion.get("clipped"):
            break
        shift = int(round(motion["yaw_deg"] / HEADING_BIN_DEG))
        if shift in seen:
            break
        seen.add(shift)
        motion = estimate_motion(prev, cur, motion["yaw_deg"])
    return motion


def _standard_errors(design: np.ndarray, rms: float) -> List[float]:
    """Standard error of each fitted parameter, or inf where unconstrained."""
    try:
        cov = np.linalg.inv(design.T @ design) * (rms ** 2)
        diag = np.diag(cov)
        return [float(math.sqrt(v)) if v > 0 else float("inf") for v in diag]
    except (np.linalg.LinAlgError, ValueError):
        return [float("inf")] * design.shape[1]


class _Sample(NamedTuple):
    """One scan-to-scan motion estimate, kept in the rolling window."""
    t: float          # timestamp of the later scan
    dt: float         # interval it covers
    yaw: float        # degrees turned, + = right
    vx: float         # forward, m/s
    vy: float         # rightward, m/s
    yaw_q: float      # scan-match quality, 0..1
    trans_q: float    # motion-model fit quality, 0..1
    vx_se: float      # standard error on vx, m/s (inf = unobservable)
    vy_se: float
    fwd_sens: float   # geometric observability of vx (0 = motion would not show)
    yaw_src: str      # "gyro", "odometry" or "none" — which sensor gave the yaw
    lidar_yaw: float  # the scan match's OWN yaw (NaN if untrusted) — never the gyro's


class HeadingTracker:
    """Rolling estimate of yaw, walking speed and lateral slip."""

    def __init__(self, window_s: float = HEADING_WINDOW_S):
        self._lock = threading.Lock()
        self._window_s = float(window_s)
        self._prev_profile: Optional[np.ndarray] = None
        self._prev_time: Optional[float] = None
        # (t, yaw_deg, vx_mps, vy_mps, yaw_quality, trans_quality)
        self._history: deque = deque(maxlen=256)
        self._heading_deg = 0.0        # cumulative since the last reference reset
        self._reference_time: Optional[float] = None
        self._last: Dict[str, Any] = _idle_state()
        self._stride_m = float(get_stride())
        self._stride_learned = bool(is_measured())
        self._stride_samples = 0
        self._stride_learn_t: Optional[float] = None   # last accepted sample
        # Whether the latest scan fed in was a measurement (see update()).
        self._input_trusted = True
        self._updated_at: Optional[float] = None
        self._updated_mono: Optional[float] = None

    # ---- ingest ----------------------------------------------------------

    def update(self, points: Sequence[Tuple[float, float]],
               timestamp: float, trusted: bool = True) -> Dict[str, Any]:
        """Feed one scan. Returns the current heading state."""
        profile = build_profile(points)
        state, bias_feedback, learned = self._ingest(profile, timestamp, trusted)
        if trusted:
            self._settle_gyro(state, bias_feedback, timestamp)
        if learned is not None:
            note_learned(learned)
        return state

    def _ingest(self, profile: np.ndarray, timestamp: float, trusted: bool = True
                ) -> Tuple[Dict[str, Any], Optional[Tuple[float, float]], Optional[float]]:
        """Fold one scan into the history."""
        with self._lock:
            self._updated_at, self._updated_mono = float(timestamp), time.monotonic()
            self._input_trusted = bool(trusted)
            prev_profile, prev_time = self._prev_profile, self._prev_time
            self._prev_profile, self._prev_time = profile, timestamp
            if self._reference_time is None:
                self._reference_time = timestamp

            dt = None if prev_time is None else timestamp - prev_time
            if dt is not None and dt < -CLOCK_FUTURE_TOLERANCE_S:
                # The wall clock stepped back: every sample now lies "in the future" and
                # would count as recent for as long as the step (a still user read as
                # veering 78 deg, fuzz test 2026-10-22). Start the history afresh.
                self._history.clear()
                self._reference_time = timestamp
            if prev_profile is None or dt is None or not (0.005 < dt < 1.0):
                self._last, learned = self._summarise(timestamp, learn=trusted)
                self._last.update(scan_yaw_deg=None, scan_fwd_mm=None, scan_right_mm=None)
                return dict(self._last), None, learned

            coarse = estimate_yaw(prev_profile, profile)
            trusted_yaw = coarse["quality"] >= HEADING_MIN_MATCH_QUALITY

            gyro_yaw, gyro_trusted = self._gyro_yaw(prev_time, timestamp)
            use_gyro = gyro_yaw is not None and gyro_trusted

            gyro_seeded = use_gyro and GYRO_SEED_MOTION_FIT
            if gyro_seeded:
                motion = estimate_motion(prev_profile, profile, gyro_yaw)
            else:
                motion = fit_motion(prev_profile, profile,
                                    coarse["yaw_deg"] if trusted_yaw else 0.0)

            lidar_yaw = None
            if trusted_yaw:
                if gyro_seeded:
                    lidar_yaw = fit_motion(prev_profile, profile,
                                           coarse["yaw_deg"])["yaw_deg"]
                else:
                    lidar_yaw = motion["yaw_deg"]

            if use_gyro:
                yaw_deg, yaw_src = gyro_yaw, "gyro"
            elif trusted_yaw:
                yaw_deg, yaw_src = motion["yaw_deg"], "odometry"
            else:
                yaw_deg, yaw_src = 0.0, "none"

            bias_feedback = None
            if gyro_yaw is not None and lidar_yaw is not None:
                bias_feedback = ((lidar_yaw - gyro_yaw) / dt, dt)

            vx = (motion["dx_mm"] / 1000.0) / dt
            vy = (motion["dy_mm"] / 1000.0) / dt
            vx_se = (motion.get("dx_se_mm", float("inf")) / 1000.0) / dt
            vy_se = (motion.get("dy_se_mm", float("inf")) / 1000.0) / dt

            # A fit outside pedestrian limits is a bad fit, not a fast walker.
            plausible = (abs(vx) <= HEADING_MAX_SPEED_MPS
                         and abs(vy) <= HEADING_MAX_LATERAL_MPS)
            trans_quality = motion["quality"] if plausible else 0.0
            if not plausible:
                vx = vy = 0.0
                vx_se = vy_se = float("inf")

            if yaw_src != "none":
                self._heading_deg += yaw_deg

            self._history.append(_Sample(timestamp, dt, yaw_deg, vx, vy,
                                         coarse["quality"], trans_quality,
                                         vx_se, vy_se,
                                         float(motion.get("forward_sens", 0.0)),
                                         yaw_src,
                                         float("nan") if lidar_yaw is None else lidar_yaw))
            self._trim(timestamp)
            self._last, learned = self._summarise(timestamp, learn=trusted)
            self._last["scan_yaw_deg"] = (round(yaw_deg, 3)
                                          if yaw_src != "none" else None)
            fit_ok = plausible and trans_quality >= HEADING_MIN_MATCH_QUALITY
            fwd_ok = (fit_ok and vx_se <= HEADING_SPEED_SE_LIMIT_MPS
                      and float(motion.get("forward_sens", 0.0)) >= HEADING_MIN_FORWARD_SENS)
            self._last["scan_fwd_mm"] = round(motion["dx_mm"], 1) if fwd_ok else None
            self._last["scan_right_mm"] = (round(motion["dy_mm"], 1)
                                           if fit_ok and vy_se <= HEADING_SPEED_SE_LIMIT_MPS
                                           else None)
            return dict(self._last), bias_feedback, learned

    # ---- gyro plumbing ---------------------------------------------------

    def _gyro_yaw(self, t0: Optional[float],
                  t1: float) -> Tuple[Optional[float], bool]:
        """(rotation the gyro saw between two scan times, is its bias trusted?)."""
        if not GYRO_ENABLED or not GYRO_MOUNTING_CONFIRMED or t0 is None:
            return None, False
        try:
            gyro = get_gyro()
            if not gyro.healthy(t1):
                return None, False
            return gyro.yaw_between(t0, t1), bool(gyro.bias_trusted())
        except Exception:
            return None, False

    def _settle_gyro(self, state: Dict[str, Any],
                     bias_feedback: Optional[Tuple[float, float]],
                     now: float) -> None:
        """Pay the gyro back: correct its bias, and tell it when we see stillness."""
        if not GYRO_ENABLED:
            return
        lidar_rate = state.get("lidar_yaw_rate_dps")
        still = (bool(state.get("confidently_still"))
                 and lidar_rate is not None
                 and abs(lidar_rate) <= GYRO_RECAL_MAX_YAW_DPS)
        try:
            gyro = get_gyro()
            if bias_feedback is not None:
                gyro.correct_bias(bias_feedback[0], bias_feedback[1])
            gyro.note_still(still, now)
        except Exception:
            pass

    def reset_reference(self, when: Optional[float] = None) -> None:
        """Zero the accumulated heading — call on a DELIBERATE turn."""
        with self._lock:
            self._heading_deg = 0.0
            self._reference_time = when
            self._history.clear()
            self._last = self._stamped(_idle_state())

    # ---- read ------------------------------------------------------------

    def state(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._last)

    # ---- internals -------------------------------------------------------

    def _trim(self, now: float) -> None:
        horizon = max(self._window_s, 12.0)   # keep enough for the veer window
        while self._history and (now - self._history[0].t) > horizon:
            self._history.popleft()

    def _window(self, now: float, seconds: float) -> List["_Sample"]:
        return [h for h in self._history
                if -CLOCK_FUTURE_TOLERANCE_S <= (now - h.t) <= seconds]

    def _stamped(self, state: Dict[str, Any]) -> Dict[str, Any]:
        """`state` with the age of the scan it rests on (NEW-026)."""
        state["updated_at"] = self._updated_at
        state["updated_mono"] = self._updated_mono
        return state

    def _summarise(self, now: float, learn: bool = True
                   ) -> Tuple[Dict[str, Any], Optional[float]]:
        """(state, stride to persist or None)."""
        input_trusted = bool(self._input_trusted)
        gait_trusted = bool(GYRO_GAIT_CONFIRMED) and input_trusted
        recent = self._window(now, self._window_s)
        if not recent:
            state = _idle_state()
            state["heading_deg"] = round(self._heading_deg, 2)
            state["input_trusted"] = input_trusted
            state["gait_trusted"] = gait_trusted
            return self._stamped(state), None

        span = sum(h.dt for h in recent)
        yaw_sum = sum(h.yaw for h in recent)
        yaw_rate = (yaw_sum / span) if span > 1e-6 else 0.0

        matched = [h for h in recent if math.isfinite(h.lidar_yaw)]
        matched_span = sum(h.dt for h in matched)
        lidar_yaw_rate = (sum(h.lidar_yaw for h in matched) / matched_span
                          if 2 * len(matched) >= len(recent) and matched_span > 1e-6
                          else None)

        vx = float(np.median([h.vx for h in recent]))
        vy = float(np.median([h.vy for h in recent]))
        yaw_q = float(np.median([h.yaw_q for h in recent]))
        trans_q = float(np.median([h.trans_q for h in recent]))
        vx_se = float(np.median([h.vx_se for h in recent]))
        vy_se = float(np.median([h.vy_se for h in recent]))
        fwd_sens = float(np.median([h.fwd_sens for h in recent]))

        speed = max(0.0, vx)

        sensitive = fwd_sens >= HEADING_MIN_FORWARD_SENS
        observable = (trans_q >= HEADING_MIN_MATCH_QUALITY
                      and vx_se <= HEADING_SPEED_SE_LIMIT_MPS
                      and sensitive)
        measured = observable and speed >= HEADING_MIN_MOVING_MPS

        gyro = self._gyro_brief()
        gyro_walking = bool(gyro and gyro.get("walking"))

        cadence = float((gyro or {}).get("cadence_spm") or 0.0)
        stepping = bool(gyro_walking and cadence > 0.0)

        learned = None
        if measured and stepping and learn and gait_trusted:
            implied = speed / (cadence / 60.0)
            if GYRO_STRIDE_MIN_M <= implied <= GYRO_STRIDE_MAX_M:
                last, self._stride_learn_t = self._stride_learn_t, now
                dt = self._window_s if last is None else min(max(now - last, 0.0),
                                                              self._window_s)
                alpha = min(1.0, dt / GYRO_STRIDE_LEARN_TAU_S)
                self._stride_m += alpha * (implied - self._stride_m)
                self._stride_samples += 1
                self._stride_learned = True
                learned = self._stride_m

        cadence_speed = (min((cadence / 60.0) * self._stride_m, HEADING_MAX_SPEED_MPS)
                         if stepping else None)

        confidently_still = (observable and speed < HEADING_MIN_MOVING_MPS
                             and not gyro_walking)

        return self._stamped({
            "moving": not confidently_still,
            "moving_confirmed": bool(measured),
            "confidently_still": bool(confidently_still),
            # Stepping, per the accelerometer. NOT a pace — see gyro_reader.
            "walking_confirmed": gyro_walking,
            "speed_mps": (round(speed, 3) if measured
                          else round(cadence_speed, 3) if cadence_speed is not None
                          else HEADING_NOMINAL_SPEED_MPS),
            "speed_source": ("odometry" if measured
                             else "cadence" if cadence_speed is not None
                             else "nominal"),
            "speed_trusted": bool(input_trusted
                                  and (measured or (cadence_speed is not None
                                                    and gait_trusted))),
            "gait_trusted": gait_trusted,
            # False for the development simulator's scans (see update()).
            "input_trusted": input_trusted,
            "speed_cadence_mps": (round(cadence_speed, 3)
                                  if cadence_speed is not None else None),
            "stride_m": round(self._stride_m, 3),
            "stride_learned": bool(self._stride_learned),
            "stride_samples": int(self._stride_samples),
            "speed_measured_mps": round(speed, 3),
            "speed_error_mps": round(vx_se, 3) if math.isfinite(vx_se) else None,
            "forward_sensitivity": round(fwd_sens, 3),
            "speed_observable": bool(observable),
            "lateral_mps": round(vy, 3) if vy_se <= HEADING_SPEED_SE_LIMIT_MPS else 0.0,
            "yaw_rate_dps": round(yaw_rate, 2),
            "lidar_yaw_rate_dps": (round(lidar_yaw_rate, 3)
                                   if lidar_yaw_rate is not None else None),
            "heading_deg": round(self._heading_deg, 2),
            "veer_deg": round(yaw_sum, 2),
            "yaw_quality": round(yaw_q, 3),
            "translation_quality": round(trans_q, 3),
            "yaw_source": recent[-1].yaw_src,
            "yaw_gyro_fraction": round(
                sum(1 for h in recent if h.yaw_src == "gyro") / len(recent), 2),
            "gyro": gyro,
            "samples": len(recent),
            "window_s": round(span, 2),
        }), learned

    def _gyro_brief(self) -> Optional[Dict[str, Any]]:
        """Compact reader status for the state dict, or None if there is no gyro."""
        if not GYRO_ENABLED:
            return None
        try:
            st = get_gyro().state()
        except Exception:
            return None
        return {
            "healthy": st.get("healthy"),
            "mounting_confirmed": bool(GYRO_MOUNTING_CONFIRMED),
            "walking": st.get("walking"),
            "cadence_spm": st.get("cadence_spm"),
            "bias_dps": st.get("bias_dps"),
            "bias_trusted": st.get("bias_trusted"),
            "bias_trust_reason": st.get("bias_trust_reason"),
            "tilt_deg": st.get("tilt_deg"),
            "tilt_warn": st.get("tilt_warn"),
            "dropouts": st.get("dropouts"),
        }

    def veer_over(self, seconds: float, now: float) -> Optional[float]:
        """Heading change accumulated over the last `seconds`, or None if the window is not yet full enough to say."""
        with self._lock:
            recent = self._window(now, seconds)
            if len(recent) < 3:
                return None
            if sum(h.dt for h in recent) < seconds * 0.5:
                return None          # too short a history to call it a trend
            return float(sum(h.yaw for h in recent))


def _idle_state() -> Dict[str, Any]:
    """What we report before any motion has been observed."""
    return {
        "moving": False,
        "moving_confirmed": False,
        "confidently_still": False,
        "walking_confirmed": False,
        "speed_mps": HEADING_NOMINAL_SPEED_MPS,
        "speed_source": "nominal",
        "speed_trusted": False,
        "gait_trusted": bool(GYRO_GAIT_CONFIRMED),
        "input_trusted": True,
        "speed_cadence_mps": None,
        "stride_m": round(float(GYRO_STRIDE_DEFAULT_M), 3),
        "stride_learned": False,
        "stride_samples": 0,
        "speed_measured_mps": 0.0,
        "speed_error_mps": None,
        "forward_sensitivity": 0.0,
        "speed_observable": False,
        "lateral_mps": 0.0,
        "yaw_rate_dps": 0.0,
        "lidar_yaw_rate_dps": None,
        "scan_yaw_deg": None,
        "scan_fwd_mm": None,
        "scan_right_mm": None,
        "heading_deg": 0.0,
        "veer_deg": 0.0,
        "yaw_quality": 0.0,
        "translation_quality": 0.0,
        "yaw_source": "none",
        "yaw_gyro_fraction": 0.0,
        "gyro": None,
        "samples": 0,
        "window_s": 0.0,
        "updated_at": None,
        "updated_mono": None,
    }


_tracker = HeadingTracker()


def get_tracker() -> HeadingTracker:
    return _tracker
