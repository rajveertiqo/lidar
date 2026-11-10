"""Is this reading current?"""

import math
import time
from typing import Any, Optional

from config import CLOCK_FUTURE_TOLERANCE_S


def age_s(stamp: Any, now: Optional[float] = None) -> Optional[float]:
    """Seconds since `stamp`, or None when there is no usable stamp."""
    if isinstance(stamp, bool) or not isinstance(stamp, (int, float)):
        return None
    stamp = float(stamp)
    if not math.isfinite(stamp) or stamp <= 0.0:
        return None
    now = time.time() if now is None else float(now)
    return now - stamp


def is_fresh(stamp: Any, limit_s: float, now: Optional[float] = None) -> bool:
    """True only for a real stamp that is neither older than `limit_s` nor further in the future than CLOCK_FUTURE_TOLERANCE_S."""
    age = age_s(stamp, now)
    if age is None:
        return False
    return -CLOCK_FUTURE_TOLERANCE_S <= age <= float(limit_s)


def describe(stamp: Any, now: Optional[float] = None) -> str:
    """ "never", "1.4s old" or "3.0s in the future" — for reasons and logs."""
    age = age_s(stamp, now)
    if age is None:
        return "never reported"
    if age < -CLOCK_FUTURE_TOLERANCE_S:
        return f"stamped {-age:.1f}s in the future (clock stepped back)"
    return f"{max(0.0, age):.1f}s old"


# The key the stores write the monotonic receive time under.
RECEIVED_KEY = "received_mono"


def received_mono(stamp: Any = None) -> float:
    """time.monotonic() of a reading captured at wall-clock `stamp`: now, less however long ago the stamp says it was captured (never more than now)."""
    mono = time.monotonic()
    age = age_s(stamp)
    return mono - age if age is not None and age > 0.0 else mono


def reading_is_current(
    state: Any,
    limit_s: float,
    now: Optional[float] = None,
    stamp_key: str = "last_update",
) -> bool:
    """When the store recorded when it received the reading (RECEIVED_KEY, monotonic, its
    capture age already taken off): that it did so within `limit_s` — the wall clock does
    not decide. Without one: is_fresh(state[stamp_key]).

    Until 2026-11-10 the wall stamp had to pass too, and a wall clock stepped FORWARD (no RTC
    battery: NTP syncing when the phone's hotspot connects, mid-walk) made a 50 ms scan
    read 120 s old: "Guidance stopped, use your cane", the LiDAR healthy (timing review).
    A clock stepped back is the same: the receive time rules wherever there is one."""
    if not isinstance(state, dict):
        return False
    mono = state.get(RECEIVED_KEY)
    if mono is None:
        return is_fresh(state.get(stamp_key), limit_s, now)
    if (
        isinstance(mono, bool)
        or not isinstance(mono, (int, float))
        or not math.isfinite(mono)
    ):
        return False
    if age_s(state.get(stamp_key), now) is None:
        return False  # no capture stamp at all: no reading
    return time.monotonic() - float(mono) <= float(limit_s)


def describe_reading(
    state: Any, now: Optional[float] = None, stamp_key: str = "last_update"
) -> str:
    """describe(), but a reading held longer than its stamp admits (a clock that stepped back and caught up) says how long it really has been."""
    state = state if isinstance(state, dict) else {}
    mono = state.get(RECEIVED_KEY)
    if (
        isinstance(mono, (int, float))
        and not isinstance(mono, bool)
        and math.isfinite(mono)
    ):
        held = time.monotonic() - float(mono)
        age = age_s(state.get(stamp_key), now)
        if age is None or held > age + CLOCK_FUTURE_TOLERANCE_S:
            return f"received {max(0.0, held):.1f}s ago"
    return describe(state.get(stamp_key), now)


def received_within(state: Any, limit_s: float) -> Optional[bool]:
    """Did the store RECEIVE this reading within `limit_s`, on the monotonic clock?"""
    state = state if isinstance(state, dict) else {}
    mono = state.get(RECEIVED_KEY)
    if (
        isinstance(mono, bool)
        or not isinstance(mono, (int, float))
        or not math.isfinite(mono)
    ):
        return None
    return time.monotonic() - float(mono) <= float(limit_s)
