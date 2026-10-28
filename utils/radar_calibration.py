"""Runtime home for the radar bearing table."""

import json
import math
import numbers
import os
import re
import stat
import tempfile
import threading
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

import config as _config
from config import (
    RADAR_BEAM_H_DEG,
    RADAR_BEARING_OFFSET_DEG,
    RADAR_BEARING_STEP_DEG,
    RADAR_BEARINGS,
    RADAR_CARDINAL_CENTRES,
    _radar_ang_diff,
    radar_board_key,
)




BEARINGS_FILE = Path(__file__).resolve().parent.parent / "radar_bearings.json"

# Most entries one POST /radar/calibrate may carry (config).
MAX_CALIBRATE_ENTRIES = int(_config.RADAR_CALIBRATE_MAX_ENTRIES)

_SERIAL_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")


def is_board_serial(serial) -> bool:
    """Whether `serial` can name ONE board: a real USB serial, not discovery's "?" for a descriptor that has none."""
    return isinstance(serial, str) and bool(_SERIAL_RE.match(serial))


# Mode for a bearings file created fresh (an existing file keeps its own).
_NEW_FILE_MODE = 0o644

_lock = threading.Lock()
_measured: Optional[Dict[str, float]] = None  # serial -> bearing, from disk


class CalibrationRejected(ValueError):
    """A calibration request that must not be recorded. Nothing was changed."""


class CalibrationSaveError(RuntimeError):
    """The bearings file could not be written."""


def _finite_number(value) -> Optional[float]:
    """float(value) if it is a real, finite number, else None."""
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        return None
    try:
        out = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return out if math.isfinite(out) else None


def _load() -> Dict[str, float]:
    """Measured bearings from disk, or {} if none have been taken yet."""
    global _measured
    if _measured is not None:
        return _measured
    data = {}
    try:
        raw = json.loads(BEARINGS_FILE.read_text(encoding="utf-8"))
        for serial, bearing in (raw.get("bearings") or {}).items():
            if bearing is None:
                data[str(serial)] = None
                continue
            value = _finite_number(bearing)
            if value is None:
                print(
                    f"[RADAR] calibration file: bearing {bearing!r} for {serial} "
                    f"is not a finite number — treating that board as unmeasured"
                )
                data[str(serial)] = None
                continue
            data[str(serial)] = value % 360.0
    except FileNotFoundError:
        pass
    except Exception as exc:  # corrupt file must not brick boot
        data = {}
        print(
            f"[RADAR] calibration file unreadable ({type(exc).__name__}: {exc}) "
            f"— treating every board as unmeasured"
        )
    _measured = data
    return _measured


def _write_atomic(path: Path, text: str) -> None:
    """Replace `path` with `text` so that a reader -- or a power cut -- sees either the whole old file or the whole new one, never a torn half."""
    target = Path(os.path.realpath(path))
    directory = target.parent
    try:
        mode = stat.S_IMODE(os.stat(target).st_mode)
    except FileNotFoundError:
        mode = _NEW_FILE_MODE

    fd, tmp = tempfile.mkstemp(
        prefix=f".{target.name}.", suffix=".tmp", dir=str(directory)
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(text)
            fh.flush()
            os.fsync(fh.fileno())
        os.chmod(tmp, mode)
        os.replace(tmp, target)
    except BaseException:
        try:
            os.unlink(tmp)
        except FileNotFoundError:
            pass
        raise

    try:
        dfd = os.open(str(directory), os.O_RDONLY)
        try:
            os.fsync(dfd)
        finally:
            os.close(dfd)
    except OSError as exc:
        print(
            f"[RADAR] calibration saved, but the directory fsync failed "
            f"({type(exc).__name__}: {exc}) — a power cut in the next few "
            f"seconds could lose it"
        )


def get_bearings() -> Dict[str, Optional[float]]:
    """serial -> bearing (or None), for every serial on the roster."""
    with _lock:
        measured = _load()
        out = {s: measured.get(s, b) for s, b in RADAR_BEARINGS.items()}
        # A board calibrated but absent from the config roster still counts.
        for s, b in measured.items():
            out.setdefault(s, b)
        return out


_save_lock = threading.Lock()


def prepare_bearings(
    mapping: Dict[str, Optional[float]],
) -> Tuple[Dict[str, Optional[float]], str]:
    """Validate `mapping` against the current table and build what would be saved: (new measured table, file text)."""
    with _lock:
        measured = dict(_load())
    for serial, bearing in mapping.items():
        serial = str(serial)
        if bearing is None:
            measured[serial] = None
            continue
        value = _finite_number(bearing)
        if value is None:
            raise ValueError(f"bearing for {serial} is not a finite number")
        measured[serial] = value % 360.0

    effective = {s: measured.get(s, b) for s, b in RADAR_BEARINGS.items()}
    for s, b in measured.items():
        effective[s] = b

    seen: Dict[str, str] = {}
    for serial, bearing in effective.items():
        if bearing is None:
            continue
        key = radar_board_key(bearing)
        clash = seen.get(key)
        if clash:
            raise ValueError(
                f"bearing {bearing:.0f}° claimed by both {clash} and {serial} "
                f"— two boards cannot face the same way"
            )
        seen[key] = serial

    # allow_nan=False: NaN is not JSON, and every value is finite by now.
    text = (
        json.dumps({"bearings": measured}, indent=2, sort_keys=True, allow_nan=False)
        + "\n"
    )
    return measured, text


def persist_bearings(text: str) -> None:
    """Write the file atomically."""
    try:
        _write_atomic(BEARINGS_FILE, text)
    except OSError as exc:
        raise CalibrationSaveError(
            f"could not save {BEARINGS_FILE.name} ({type(exc).__name__}: {exc}) "
            f"— nothing was changed"
        ) from exc


def commit_bearings(measured: Dict[str, Optional[float]]) -> Dict[str, Optional[float]]:
    """Make an already-SAVED table the live one."""
    global _measured
    with _lock:
        _measured = dict(measured)
    return get_bearings()


def validate_request(
    mapping, discovered: Iterable[str] = (), connected: Iterable[str] = ()
) -> Dict[str, Optional[float]]:
    """Check one POST /radar/calibrate table; return it cleaned, or raise."""
    if not isinstance(mapping, dict) or not mapping:
        raise CalibrationRejected("expected {'bearings': {serial: degrees}}")
    if len(mapping) > MAX_CALIBRATE_ENTRIES:
        raise CalibrationRejected(
            f"{len(mapping)} entries — at most {MAX_CALIBRATE_ENTRIES} per request"
        )

    discovered, connected = list(discovered), list(connected)
    seen_twice = {s for s in discovered if discovered.count(s) > 1}
    seen_twice |= {s for s in connected if connected.count(s) > 1}
    with _lock:
        on_file = set(_load())
    settable = (set(RADAR_BEARINGS) | set(discovered) | set(connected)) - seen_twice
    clearable = settable | seen_twice | on_file

    slots = {radar_board_key(b): float(b) for b in ring_slots()}
    legal = ", ".join(f"{b:.0f}" for b in sorted(slots.values()))

    clean: Dict[str, Optional[float]] = {}
    for serial, value in mapping.items():
        if not is_board_serial(serial):
            raise CalibrationRejected(f"{serial!r} is not a board serial")

        if value is None:
            if serial not in clearable:
                raise CalibrationRejected(
                    f"unknown board {serial} — not on the roster, not connected "
                    f"and not in the calibration file, so there is nothing to clear"
                )
            clean[serial] = None
            continue

        if serial in seen_twice:
            raise CalibrationRejected(
                f"board serial {serial} answered on more than one port — the two "
                f"cannot be told apart, so neither can be given a bearing"
            )
        if serial not in settable:
            raise CalibrationRejected(
                f"unknown board {serial} — not on the roster and not connected; "
                f"a bearing can only be recorded for a board this device has seen"
            )

        number = value
        if isinstance(value, str):
            try:
                number = float(value.strip())
            except ValueError:
                raise CalibrationRejected(
                    f"bearing for {serial} is not a number: {value[:32]!r}"
                ) from None
        bearing = _finite_number(number)
        if bearing is None:
            raise CalibrationRejected(
                f"bearing for {serial} is not a finite number: {str(value)[:32]!r}"
            )
        if not 0.0 <= bearing < 360.0:
            raise CalibrationRejected(
                f"bearing for {serial} is {bearing:g} — degrees clockwise from "
                f"ahead, in [0, 360)"
            )
        slot = slots.get(radar_board_key(bearing))
        if slot is None:
            raise CalibrationRejected(
                f"bearing {bearing:g} for {serial} is not a slot on this mount "
                f"(the legal bearings are {legal})"
            )
        clean[serial] = slot
    return clean


def board_keys() -> Tuple[str, ...]:
    """State keys of every MEASURED board, in bearing order."""
    return tuple(
        radar_board_key(b)
        for b in sorted(v for v in get_bearings().values() if v is not None)
    )


def cardinal_sources() -> Dict[str, Tuple[str, ...]]:
    """cardinal -> the board keys whose beam overlaps that cardinal's sector."""
    bearings = sorted(v for v in get_bearings().values() if v is not None)
    half = RADAR_BEAM_H_DEG / 2.0 + 22.5
    return {
        name: tuple(
            radar_board_key(b) for b in bearings if _radar_ang_diff(b, centre) < half
        )
        for name, centre in RADAR_CARDINAL_CENTRES.items()
    }


def ring_slots() -> Tuple[float, ...]:
    """The intended mount: RADAR_BEARING_OFFSET_DEG + one per step around 360."""
    step = RADAR_BEARING_STEP_DEG
    return tuple(
        (RADAR_BEARING_OFFSET_DEG + i * step) % 360.0
        for i in range(int(round(360.0 / step)))
    )


def status() -> Dict[str, object]:
    bearings = get_bearings()
    return {
        "measured": sum(v is not None for v in bearings.values()),
        "configured": len(bearings),
        "file": str(BEARINGS_FILE),
        "file_exists": BEARINGS_FILE.exists(),
    }
