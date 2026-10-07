"""Gyro reader — the body's own rotation, sampled fast and kept honest by the LiDAR."""

from collections import deque
from typing import Any, Deque, Dict, NamedTuple, Optional, Tuple

import math
import threading
import time

from config import (
    CLOCK_FUTURE_TOLERANCE_S,
    GYRO_ACCEL_RANGE_G,
    GYRO_BIAS_GAIN,
    GYRO_CAL_MAX_STD_DPS,
    GYRO_BIAS_MAX_DPS,
    GYRO_BIAS_MAX_ERR_DPS,
    GYRO_BUFFER_S,
    GYRO_CADENCE_STALE_S,
    GYRO_CADENCE_WINDOW_S,
    GYRO_CALIBRATION_SAMPLES,
    GYRO_ENABLED,
    GYRO_I2C_ADDRESS,
    GYRO_I2C_BUS,
    GYRO_MAX_RATE_DPS,
    GYRO_MAX_READ_ERRORS,
    GYRO_MIN_COVERAGE,
    GYRO_MOUNTING_CONFIRMED,
    GYRO_RANGE_DPS,
    GYRO_RECAL_MIN_INTERVAL_S,
    GYRO_RECAL_ON_STILL,
    GYRO_RECAL_STILL_S,
    GYRO_RECONNECT_S,
    GYRO_SAMPLE_HZ,
    GYRO_SPI_BUS,
    GYRO_SPI_DEVICE,
    GYRO_SPI_HZ,
    GYRO_STALE_S,
    GYRO_STEP_DETECT,
    GYRO_STEP_PEAK_G,
    GYRO_STEP_REFRACTORY_S,
    GYRO_STEP_REF_TAU_S,
    GYRO_STEP_STD_G,
    GYRO_STEP_UPWARD_ONLY,
    GYRO_STEP_WINDOW_S,
    GYRO_TILT_WARN_DEG,
    GYRO_TRUST_ERR_DPS,
    GYRO_TRUST_SAMPLES,
    GYRO_TRANSPORT,
    GYRO_YAW_AXIS,
    GYRO_YAW_SIGN,
)

_AXIS_KEYS = {"x": ("gx", "ax"), "y": ("gy", "ay"), "z": ("gz", "az")}


class _Sample(NamedTuple):
    """One gyro sample, with the yaw integral evaluated at its timestamp."""
    t: float        # WALL clock, matching the LiDAR's time.time() capture stamps
    cum: float      # cumulative yaw since start, degrees, + = right
    raw: float      # yaw-axis rate BEFORE this module's residual bias, deg/s
    amag: float     # |a| in g — footfalls live here
    a_up: float     # accel component along the mounting's up axis, for tilt
    m: float = 0.0  # time.monotonic() of the same instant — see _mono_of()


def _mono_of(wall_stamp: float) -> float:
    """A wall-clock stamp expressed on the monotonic clock."""
    return time.monotonic() - max(0.0, time.time() - float(wall_stamp))


class GyroReader:
    """Owns the IMU, integrates yaw, and answers questions about an interval."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._buf: Deque[_Sample] = deque(maxlen=int(GYRO_SAMPLE_HZ * GYRO_BUFFER_S) + 64)
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

        self._sensor = None
        self._chip = None
        self._cum = 0.0             # running yaw integral, degrees
        self._bias_dps = 0.0        # residual zero-rate offset, LiDAR-corrected
        self._bias_trusted = False
        self._trust_reason = "not calibrated"
        self._agreements = 0        # consecutive small disagreements with LiDAR
        self._cal_std = None
        # The last calibration taken AT REST, reused when a later one is not.
        self._cal_bias: Optional[Tuple[float, float, float]] = None
        self._connected = False
        self._last_error: Optional[str] = None
        self._connects = 0
        self._dropouts = 0
        self._samples = 0
        self._rejected = 0          # samples thrown out as implausible rates

        # step detection — MONOTONIC stamps (F20, see _mono_of); None = no step yet
        self._steps = 0
        self._last_step_t: Optional[float] = None
        self._above_peak = False
        self._step_log: Deque[float] = deque(maxlen=512)
        self._amag_ref: Optional[float] = None   # slow mean of |a|, see below
        self._amag_n = 0            # samples folded into it, for the warm-up
        self._exc_log: Deque[Tuple[float, float]] = deque(
            maxlen=int(GYRO_SAMPLE_HZ * GYRO_BUFFER_S) + 64)

        # opportunistic re-zero — MONOTONIC stamps (F20); None = never re-zeroed
        self._still_since: Optional[float] = None
        self._last_recal: Optional[float] = None
        self._recals = 0

    # ---- lifecycle -------------------------------------------------------

    def start(self) -> None:
        if not GYRO_ENABLED:
            print("[GYRO] disabled in config — heading stays LiDAR-only")
            return
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._worker, name="gyro", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        t = self._thread
        if t and t.is_alive():
            t.join(timeout=2.0)
        self._disconnect()

    # ---- the question heading_tracker asks -------------------------------

    def bias_trusted(self) -> bool:
        """Whether the zero-rate offset is established well enough to steer on."""
        with self._lock:
            return self._bias_trusted

    def yaw_between(self, t0: float, t1: float) -> Optional[float]:
        """Degrees turned between two timestamps, + = right."""
        if t1 <= t0:
            return None
        with self._lock:
            if not self._connected or len(self._buf) < 2:
                return None
            first, last = self._buf[0].t, self._buf[-1].t
            # The newest sample is normally a few ms shy of the scan timestamp.
            lo, hi = max(t0, first), min(t1, last)
            if hi <= lo:
                return None
            covered = (hi - lo) / (t1 - t0)
            if covered < GYRO_MIN_COVERAGE:
                return None
            turned = self._cum_at(hi) - self._cum_at(lo)
            if turned is None:
                return None
            return float(turned / covered)

    def _cum_at(self, t: float) -> Optional[float]:
        """Cumulative yaw at an arbitrary time, linearly interpolated."""
        buf = self._buf
        n = len(buf)
        if n == 0:
            return None
        if t <= buf[0].t:
            return buf[0].cum
        if t >= buf[-1].t:
            return buf[-1].cum
        lo, hi = 0, n - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if buf[mid].t <= t:
                lo = mid
            else:
                hi = mid
        a, b = buf[lo], buf[hi]
        span = b.t - a.t
        if span <= 1e-9:
            return b.cum
        return a.cum + (b.cum - a.cum) * ((t - a.t) / span)

    # ---- the correction heading_tracker pays back -------------------------

    def correct_bias(self, error_dps: float, dt: float) -> None:
        """Nudge the residual bias toward what the scan match says."""
        if not (math.isfinite(error_dps) and math.isfinite(dt)) or dt <= 0:
            return
        if abs(error_dps) > GYRO_BIAS_MAX_ERR_DPS:
            return
        with self._lock:
            self._bias_dps -= GYRO_BIAS_GAIN * error_dps * dt
            self._bias_dps = max(-GYRO_BIAS_MAX_DPS,
                                 min(GYRO_BIAS_MAX_DPS, self._bias_dps))

            if abs(error_dps) <= GYRO_TRUST_ERR_DPS:
                self._agreements += 1
                if (not self._bias_trusted
                        and self._agreements >= GYRO_TRUST_SAMPLES):
                    self._bias_trusted = True
                    self._trust_reason = "agreed with scan matching"
                    print("[GYRO] bias confirmed against the LiDAR — "
                          "gyro is now the yaw source")
            else:
                self._agreements = 0

    def note_still(self, still: bool, now: float,
                   mono: Optional[float] = None) -> None:
        """Tell the reader whether the LiDAR can SEE that the user is stationary."""
        if not GYRO_RECAL_ON_STILL:
            return
        mono = _mono_of(now) if mono is None else float(mono)
        with self._lock:
            if not still:
                self._still_since = None
                return
            if self._still_since is None:
                self._still_since = mono
                return
            if (mono - self._still_since) < GYRO_RECAL_STILL_S:
                return
            if (self._last_recal is not None
                    and (mono - self._last_recal) < GYRO_RECAL_MIN_INTERVAL_S):
                return
            window = [s.raw for s in self._buf if (mono - s.m) <= GYRO_RECAL_STILL_S]
            if len(window) < 10:
                return
            offset = sum(window) / len(window)
            if abs(offset) > GYRO_BIAS_MAX_DPS:
                return          # a fault, not an offset — see correct_bias
            self._bias_dps = offset
            self._last_recal = mono
            self._still_since = None
            self._recals += 1
            self._agreements = 0
            if not self._bias_trusted:
                self._bias_trusted = True
                self._trust_reason = "re-zeroed while confirmed still"
                print("[GYRO] re-zeroed against a confirmed standstill — "
                      "gyro is now the yaw source")

    # ---- read ------------------------------------------------------------

    def healthy(self, now: Optional[float] = None) -> bool:
        now = time.time() if now is None else now
        with self._lock:
            return self._is_healthy(now)

    def _is_healthy(self, now: float) -> bool:
        """Caller holds the lock."""
        if not self._connected or not self._buf:
            return False
        return (now - self._buf[-1].t) <= GYRO_STALE_S

    def state(self) -> Dict[str, Any]:
        """Everything the dashboard, the walk log and /guidance/gyro need."""
        now = time.time()
        mono = time.monotonic()
        with self._lock:
            healthy = self._is_healthy(now)
            last = self._buf[-1] if self._buf else None
            age = (now - last.t) if last else None
            walking, cadence, std = self._gait(mono)
            tilt = self._tilt(last)
            return {
                "enabled": bool(GYRO_ENABLED),
                "healthy": bool(healthy),
                "connected": bool(self._connected),
                "chip": self._chip,
                "transport": GYRO_TRANSPORT,
                "sample_age_s": round(age, 3) if age is not None else None,
                "sample_rate_hz": self._measured_rate(mono),
                "yaw_cumulative_deg": round(self._cum, 2),
                "bias_dps": round(self._bias_dps, 4),
                "bias_trusted": bool(self._bias_trusted),
                "bias_trust_reason": self._trust_reason,
                "mounting_confirmed": bool(GYRO_MOUNTING_CONFIRMED),
                "calibration_spread_dps": self._cal_std,
                "yaw_axis": GYRO_YAW_AXIS,
                "yaw_sign": GYRO_YAW_SIGN,
                "walking": walking,
                "cadence_spm": cadence,
                "accel_std_g": std,
                "steps": self._steps,
                "tilt_deg": tilt,
                "tilt_warn": (tilt is not None and tilt >= GYRO_TILT_WARN_DEG),
                "connects": self._connects,
                "dropouts": self._dropouts,
                "recalibrations": self._recals,
                "samples": self._samples,
                "rejected_samples": self._rejected,
                "last_error": self._last_error,
            }

    def _measured_rate(self, mono: float) -> Optional[float]:
        recent = [s for s in self._buf if (mono - s.m) <= 1.0]
        if len(recent) < 2:
            return None
        span = recent[-1].m - recent[0].m
        return round((len(recent) - 1) / span, 1) if span > 1e-6 else None

    def _gait(self, now):
        """Walking yes/no, cadence and the |a| spread it was decided on."""
        if not GYRO_STEP_DETECT:
            return None, None, None
        window = [s.amag for s in self._buf if (now - s.m) <= GYRO_STEP_WINDOW_S]
        if len(window) < 8:
            return None, None, None
        mean = sum(window) / len(window)
        std = math.sqrt(sum((v - mean) ** 2 for v in window) / len(window))
        recent = self._step_times_within(now, GYRO_CADENCE_WINDOW_S)
        cadence = self._cadence_from(recent, now)
        walking = bool(std >= GYRO_STEP_STD_G and len(recent) >= 2)
        return walking, cadence, round(std, 4)

    @staticmethod
    def _cadence_from(recent, now: float) -> float:
        """Steps per minute from the INTERVALS between footfalls."""
        if len(recent) < 2:
            return 0.0
        if (now - recent[-1]) > GYRO_CADENCE_STALE_S:
            return 0.0
        span = recent[-1] - recent[0]
        if span <= 1e-6:
            return 0.0
        return round((len(recent) - 1) * 60.0 / span, 1)

    def _step_times_within(self, now: float, seconds: float):
        return [t for t in self._step_log if (now - t) <= seconds]

    def excursions(self, seconds: float) -> Tuple[float, ...]:
        """Signed |a| excursions from the resting mean over the last `seconds`."""
        now = time.monotonic()               # the log is stamped monotonic (F20)
        with self._lock:
            return tuple(e for t, e in self._exc_log if (now - t) <= seconds)

    def gait_tuning(self) -> Dict[str, Any]:
        """What GYRO_STEP_PEAK_G should be, measured rather than assumed."""
        st = self.state()
        exc = self.excursions(GYRO_BUFFER_S)
        ups = sorted((e for e in exc if e > 0), reverse=True)
        out: Dict[str, Any] = {
            "samples": len(exc),
            "window_s": GYRO_BUFFER_S,
            "looks_like_gait": False,
            "suggested_peak_g": None,
            "percentiles_g": None,
            "cleared_threshold": 0,
            "peak_g": GYRO_STEP_PEAK_G,
            "std_g": GYRO_STEP_STD_G,
            "accel_std_g": st.get("accel_std_g"),
            "tilt_deg": st.get("tilt_deg"),
        }
        if not ups:
            return out

        def pct(p: float) -> float:
            return round(ups[min(len(ups) - 1, int(len(ups) * p))], 4)

        out["percentiles_g"] = {"max": round(ups[0], 4), "p99": pct(0.01),
                                "p95": pct(0.05), "p75": pct(0.25),
                                "median": pct(0.50)}
        out["cleared_threshold"] = sum(1 for e in ups if e >= GYRO_STEP_PEAK_G)
        std = st.get("accel_std_g") or 0.0
        out["looks_like_gait"] = bool(st.get("walking")
                                      or std >= GYRO_STEP_STD_G)
        if out["looks_like_gait"]:
            out["suggested_peak_g"] = round(pct(0.05) * 0.5, 2)
        return out

    def _tilt(self, last: Optional[_Sample]) -> Optional[float]:
        """Angle between the mounting's up axis and gravity, in degrees."""
        if last is None:
            return None
        mag = last.amag
        if mag < 0.3 or mag > 3.0:
            return None         # under acceleration, gravity is not separable
        return round(math.degrees(math.acos(min(1.0, abs(last.a_up) / mag))), 1)

    # ---- device ----------------------------------------------------------

    def _connect(self) -> bool:
        from utils import mpu6500

        sensor = None
        try:
            sensor = mpu6500.MPU6500(
                transport=GYRO_TRANSPORT,
                bus=GYRO_I2C_BUS,
                address=GYRO_I2C_ADDRESS,
                spi_bus=GYRO_SPI_BUS,
                spi_device=GYRO_SPI_DEVICE,
                spi_hz=GYRO_SPI_HZ,
                gyro_range=GYRO_RANGE_DPS,
                accel_range=GYRO_ACCEL_RANGE_G,
            )
            chip = sensor.who_am_i()
            if chip in mpu6500.SPI_DEAD_BYTES:
                raise OSError(f"no chip on {sensor.describe()} (WHO_AM_I={chip:#04x})")
            sensor.wake()
            means, std = self._calibrate(sensor)
        except Exception as e:
            with self._lock:
                self._last_error = f"{type(e).__name__}: {e}"
            if sensor is not None:
                try:
                    sensor.close()
                except Exception:
                    pass
            return False

        with self._lock:
            self._sensor = sensor
            self._chip = sensor.chip_name()
            self._connected = True
            self._connects += 1
            self._last_error = None
            self._buf.clear()
            self._bias_dps = 0.0
            self._still_since = None
            self._agreements = 0
            self._cal_std = round(std, 3)
            self._bias_trusted = std <= GYRO_CAL_MAX_STD_DPS
            self._trust_reason = ("calibrated at rest" if self._bias_trusted
                                  else f"moving during calibration "
                                       f"(spread {std:.2f} dps)")
        print(f"[GYRO] {self._chip} on {sensor.describe()} — "
              f"yaw axis {GYRO_YAW_AXIS}, sign {GYRO_YAW_SIGN:+.0f}")
        if not GYRO_MOUNTING_CONFIRMED:
            print("[GYRO] mounting NOT confirmed — reporting only, not steering. "
                  "Heading stays on the LiDAR (see GYRO_MOUNTING_CONFIRMED).")
        if self._bias_trusted:
            print(f"[GYRO] calibrated at rest (spread {std:.2f} dps)")
        else:
            print(f"[GYRO] MOVING during calibration (spread {std:.2f} dps) — "
                  f"yaw stays on the LiDAR until the offset is established")
        return True

    def _calibrate(self, sensor) -> Tuple[Tuple[float, float, float], float]:
        """Average the gyro at rest, AND report how much it scattered."""
        sensor.gyro_bias = (0.0, 0.0, 0.0)
        axes = {"x": [], "y": [], "z": []}
        for _ in range(GYRO_CALIBRATION_SAMPLES):
            s = sensor.read()
            axes["x"].append(s["gx"])
            axes["y"].append(s["gy"])
            axes["z"].append(s["gz"])
            time.sleep(0.002)
        means = tuple(sum(axes[k]) / len(axes[k]) for k in ("x", "y", "z"))
        yaw = axes.get(GYRO_YAW_AXIS, axes["z"])
        m = sum(yaw) / len(yaw)
        std = math.sqrt(sum((v - m) ** 2 for v in yaw) / len(yaw))
        if std <= GYRO_CAL_MAX_STD_DPS:
            self._cal_bias = means
        sensor.gyro_bias = self._cal_bias or (0.0, 0.0, 0.0)
        return means, std

    def _disconnect(self) -> None:
        with self._lock:
            sensor, self._sensor = self._sensor, None
            was = self._connected
            self._connected = False
        if sensor is not None:
            try:
                sensor.close()
            except Exception:
                pass
        if was:
            print("[GYRO] link down — heading falls back to LiDAR scan matching")

    # ---- worker ----------------------------------------------------------

    def _worker(self) -> None:
        gyro_key, accel_key = _AXIS_KEYS.get(GYRO_YAW_AXIS, _AXIS_KEYS["z"])
        period = 1.0 / max(1.0, GYRO_SAMPLE_HZ)
        errors = 0
        last_t: Optional[float] = None
        last_mono: Optional[float] = None
        deadline = time.monotonic()

        while not self._stop.is_set():
            if not self._connected:
                if not self._connect():
                    self._stop.wait(GYRO_RECONNECT_S)
                    continue
                errors = 0
                last_t = last_mono = None
                deadline = time.monotonic()

            try:
                s = self._sensor.read()
                errors = 0
            except Exception as e:
                errors += 1
                with self._lock:
                    self._last_error = f"{type(e).__name__}: {e}"
                if errors >= GYRO_MAX_READ_ERRORS:
                    with self._lock:
                        self._dropouts += 1
                    self._disconnect()
                    last_t = last_mono = None
                self._stop.wait(period)
                continue

            now, mono = time.time(), time.monotonic()
            self._ingest(s, gyro_key, accel_key, now, last_t,
                         mono=mono, last_mono=last_mono)
            last_t, last_mono = now, mono

            deadline += period
            slack = deadline - time.monotonic()
            if slack > 0:
                self._stop.wait(slack)
            else:
                deadline = time.monotonic()  # fell behind; do not spiral

    def _ingest(self, s: Dict[str, float], gyro_key: str, accel_key: str,
                now: float, last_t: Optional[float],
                mono: Optional[float] = None,
                last_mono: Optional[float] = None) -> None:
        """Fold one sample in."""
        raw = float(s[gyro_key]) * GYRO_YAW_SIGN
        ax, ay, az = float(s["ax"]), float(s["ay"]), float(s["az"])
        amag = math.sqrt(ax * ax + ay * ay + az * az)
        a_up = float(s[accel_key])
        mono = _mono_of(now) if mono is None else float(mono)

        with self._lock:
            self._samples += 1
            rate = raw - self._bias_dps
            if not math.isfinite(rate) or abs(rate) > GYRO_MAX_RATE_DPS:
                self._rejected += 1
                rate = 0.0
            if last_mono is not None:
                dt = mono - last_mono
            else:
                dt = 0.0 if last_t is None else (now - last_t)
            if not (0.0 < dt < 1.0):
                dt = 0.0        # first sample after a (re)connect, or a stall
            self._cum += rate * dt
            if self._buf and now < self._buf[-1].t - CLOCK_FUTURE_TOLERANCE_S:
                self._buf.clear()
            self._buf.append(_Sample(now, self._cum, raw, amag, a_up, mono))
            self._detect_step(mono, amag)

    def _detect_step(self, now: float, amag: float) -> None:
        """Count footfalls from |a| excursions."""
        if not GYRO_STEP_DETECT:
            return
        self._amag_n += 1
        warm = max(1.0, GYRO_SAMPLE_HZ * GYRO_STEP_REF_TAU_S)
        if self._amag_ref is None:
            self._amag_ref = amag
        elif self._amag_n <= warm:
            self._amag_ref += (amag - self._amag_ref) / self._amag_n
        else:
            alpha = 1.0 / warm
            self._amag_ref += alpha * (amag - self._amag_ref)

        signed = amag - self._amag_ref
        self._exc_log.append((now, signed))
        excursion = signed if GYRO_STEP_UPWARD_ONLY else abs(signed)
        if excursion >= GYRO_STEP_PEAK_G:
            if not self._above_peak and (self._last_step_t is None
                                         or (now - self._last_step_t) >= GYRO_STEP_REFRACTORY_S):
                self._steps += 1
                self._last_step_t = now
                self._step_log.append(now)
            self._above_peak = True
        elif excursion < GYRO_STEP_PEAK_G * 0.6:
            self._above_peak = False


# One reader for the device.
_reader = GyroReader()


def get_gyro() -> GyroReader:
    return _reader


def start_gyro() -> None:
    _reader.start()


def stop_gyro() -> None:
    _reader.stop()


def print_tuning(tuning: Dict[str, Any]) -> None:
    """Human-readable form of gait_tuning(), for the terminal."""
    print("\n" + "-" * 60)
    print(f"  TUNING  (last {tuning['window_s']:.0f} s, "
          f"{tuning['samples']} samples)")

    tilt = tuning.get("tilt_deg")
    if tilt is not None:
        print(f"\n  up-axis tilt    {tilt:.1f} deg")
        print(f"    pitch/roll leaking into yaw = sin(tilt) = "
              f"{math.sin(math.radians(tilt)):.2f}")
        if tilt > 15.0:
            print("    ^ this is the reason NOT to set "
                  "GYRO_MOUNTING_CONFIRMED yet. Pad")
            print("      the outboard edge of the strap to bring it level.")
        print("    (pace is unaffected — it comes from |a|, a magnitude)")

    p = tuning.get("percentiles_g")
    if not p:
        print("\n  no upward excursions recorded")
        return
    print("\n  |a| upward excursion, g")
    for label, key in (("largest ", "max"), ("99th pct", "p99"),
                       ("95th pct", "p95"), ("75th pct", "p75"),
                       ("median  ", "median")):
        print(f"    {label}      {p[key]:.3f}")
    print(f"\n    GYRO_STEP_PEAK_G is {tuning['peak_g']}")
    print(f"    {tuning['cleared_threshold']} of {tuning['samples']} "
          f"samples cleared it")

    if not tuning.get("looks_like_gait"):
        print(f"\n    NOT A WALK — the |a| spread never reached the "
              f"{tuning['std_g']} g gate.")
        print("    Those excursions are handling noise, not footfalls, so no")
        print("    threshold is suggested from them: scaling to noise would")
        print("    tune the detector to call stillness walking.")
        print("    Strap the board on and walk a minute, then run this again.")
        return

    sug = tuning.get("suggested_peak_g")
    if sug is None:
        return
    if tuning["cleared_threshold"] == 0:
        print("    NOTHING cleared it — footfalls are not reaching the "
              "threshold on")
        print(f"    this mounting. Try GYRO_STEP_PEAK_G = {sug}")
    elif abs(sug - tuning["peak_g"]) >= 0.02:
        print(f"    this walk suggests   {sug}")
    else:
        print("    consistent with this walk — leave it")
    print("\n    Re-check false positives after changing it: sit still, "
          "fidget, and")
    print("    ride in a vehicle, and confirm `walking` stays no.")


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(
        description="Gyro reader check. Reads the sensor DIRECTLY, so stop the "
                    "guidance app first — one owner per SPI bus. While the app "
                    "is running, use /guidance/gait/test instead.")
    ap.add_argument("--check", action="store_true",
                    help="live yaw readout — turn the harness RIGHT, "
                         "yaw must go POSITIVE")
    ap.add_argument("--tune", action="store_true",
                    help="on exit, report the |a| excursions and tilt that "
                         "set GYRO_STEP_PEAK_G for this mounting")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop after this long (0 = until Ctrl+C)")
    args = ap.parse_args()

    start_gyro()
    started = time.time()
    print("Waiting for the sensor...")
    try:
        while True:
            st = _reader.state()
            if st["healthy"]:
                print(f"yaw {st['yaw_cumulative_deg']:+9.2f} deg   "
                      f"bias {st['bias_dps']:+6.3f} dps   "
                      f"tilt {st['tilt_deg']}   "
                      f"walking {st['walking']}  cadence {st['cadence_spm']}   "
                      f"{st['sample_rate_hz']} Hz", end="\r", flush=True)
            else:
                print(f"waiting — {st['last_error'] or 'no data'}"[:100],
                      end="\r", flush=True)
            if args.seconds and (time.time() - started) > args.seconds:
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        if args.tune:
            try:
                print_tuning(_reader.gait_tuning())
            except Exception as exc:
                print(f"tuning report unavailable: {type(exc).__name__}: {exc}")
        stop_gyro()
