"""When a sentence is said: the speech gate (_Gate: debounce, urgency, repeats, cutting in),
each sentence's airtime, and whether the wearer has been told a sensor is lost or back
(_SensorNotice). Moved out of simple_guidance on 2026-11-11, which re-exports every name
and keeps the instances (_gate, _glass, ...) and what to say."""

import threading
from typing import Any, Dict, List, Optional, Tuple

from app.config import (
    NOTIFY_AIRTIME_BASE_S,
    NOTIFY_AIRTIME_PER_CHAR_S,
    NOTIFY_AIRTIME_PER_COMMA_S,
    SIMPLE_AFTER_SPEECH_S,
    SIMPLE_BLOCKED_REPEAT_S,
    SIMPLE_CLEAR_SAY_CONFIRM_S,
    SIMPLE_CONFIRM_S,
    SIMPLE_FLIP_SETTLE_S,
    SIMPLE_MIN_GAP_S,
    SIMPLE_OFFLINE_REPEAT_S,
    SIMPLE_REPEAT_MAX_S,
    SIMPLE_REPEAT_S,
    SIMPLE_SETTLE_S,
    WALK_MIN_TURN_FT,
)

from app.guidance.vocabulary import (
    ALLOWED,
    BLOCKED_GLASS,
    BLOCKED_WALL,
    DRIFT_LEFT,
    DRIFT_RIGHT,
    GLASS_LEFT,
    GLASS_RIGHT,
    GLASS_SENSOR_OFF,
    NEAR_LEFT,
    NEAR_RIGHT,
    OBJECT_IN_FRONT,
    OBSTACLE_AHEAD,
    SENSORS_OFFLINE,
    STEER_IN_FRONT,
    STOP_NO_WAY,
    TURN_AROUND,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_SLIGHT_LEFT,
    TURN_SLIGHT_RIGHT,
    VEHICLE_FROM_LEFT,
    VEHICLE_FROM_RIGHT,
    _APPROACHING,
    _BLOCKED,
    _GLASS_BESIDE,
    _LEVEL,
    _MOVER_INFO,
    _MOVER_SENTENCES,
    _PATH_AHEAD,
    _SIDE_WALL,
    _STEER_INFO,
    _TURNS,
    _VEHICLE,
    _WAY_OUT,
)


class _SensorNotice:
    """Whether the wearer has been told a sensor is lost, or back ("Glass sensor off",
    "Camera off", ...): a loss is told once it has lasted `loss_s`, a recovery once it has
    held `recover_s` (doubling with each loss told, up to `recover_max_s`), and nothing is
    told inside `grace_s` of start-up while the sensor is still coming up."""

    def __init__(
        self,
        off: str,
        on: str,
        loss_s: float,
        recover_s: float,
        recover_max_s: float,
        grace_s: float,
    ):
        self._off, self._on = off, on
        self._loss_s, self._recover_s = loss_s, recover_s
        self._recover_max_s, self._grace_s = recover_max_s, grace_s
        self._clear()

    def _clear(self) -> None:
        self.announced_ok = True
        self._since: Optional[float] = None
        self._first: Optional[float] = None  # first poll judged
        self._ever_ok = False  # full coverage seen at least once
        self._losses = 0  # announced losses, for the backoff
        self._ok_since: Optional[float] = None  # coverage unbroken since

    def _recover_need(self) -> float:
        return min(self._recover_s * 2 ** max(self._losses - 1, 0), self._recover_max_s)

    def pending(self, ok: bool, now: float, settling: bool = True) -> Optional[str]:
        """`settling`: the sensor is still coming up (e.g. front_glass_coverage: discovery has not reported, or an owed front board is starting or calibrating)."""
        if self._first is None or now < self._first:
            self._first = now
        if ok:
            self._ever_ok = True
            if self._ok_since is None or now < self._ok_since:
                self._ok_since = now
            if (
                self.announced_ok
                and self._losses
                and now - self._ok_since >= self._recover_max_s
            ):
                self._losses = 0  # a long quiet run: start afresh
        else:
            self._ok_since = None
        if ok == self.announced_ok:
            self._since = None
            return None
        if self._since is None or now < self._since:  # new run, or clock step
            self._since = now
        if ok:
            need = self._recover_need()
        else:
            need = self._loss_s
            if (
                not self._ever_ok
                and settling
                and now - self._first < self._grace_s - 1e-6
            ):
                return None  # still coming up
        if now - self._since >= need - 1e-6:
            return self._on if ok else self._off
        return None

    def said(self, text: str) -> None:
        self.announced_ok = text == self._on
        if text == self._off:
            self._losses += 1
        self._since = None

    def unsay(self, text: str) -> None:
        """`text` was cut off by a block before it was heard (2026-11-10): it is told again,
        with no new wait, the next time the path is clear and it still holds."""
        if text == self._off and not self.announced_ok:
            self.announced_ok = True
            self._losses = max(self._losses - 1, 0)
        elif text == self._on and self.announced_ok:
            self.announced_ok = False
        else:
            return
        self._since = float("-inf")

    def state(self) -> Dict[str, Any]:
        return {
            "announced": "on" if self.announced_ok else "off",
            "changing_since": self._since,
            "recover_after_s": self._recover_need(),
        }

    def reset(self) -> None:
        self._clear()


# Each sentence's airtime as the Pi voice renders it, trailing silence included: how long
# the voice is busy. A words/s guess ran short, so a block warning cut "Vehicle
# approaching, move right" before "move right" (2026-10-23).
_measured_airtime: Dict[str, float] = {}


def measure_airtimes(cmd: List[str]) -> int:
    """Time every sentence in ALLOWED with the voice command (espeak-ng --stdout, no sound); how many were timed. pi_voice runs it off-thread at start-up."""
    import subprocess

    timed = 0
    for text in sorted(ALLOWED):
        try:
            wav = subprocess.run(
                list(cmd) + ["--stdout", text], capture_output=True, timeout=5.0
            ).stdout
            if wav[:4] != b"RIFF" or wav[8:12] != b"WAVE":
                continue  # not espeak's WAV: keep the estimate
            byte_rate = int.from_bytes(wav[28:32], "little")
            if len(wav) > 44 and byte_rate > 0:
                _measured_airtime[text] = (len(wav) - 44) / byte_rate
                timed += 1
        except Exception:
            pass
    return timed


def _airtime(text: str) -> float:
    measured = _measured_airtime.get(text)
    if measured is not None:
        return measured
    return (
        NOTIFY_AIRTIME_BASE_S
        + NOTIFY_AIRTIME_PER_CHAR_S * len(text)
        + NOTIFY_AIRTIME_PER_COMMA_S * text.count(",")
    )


def _is_block(text: str) -> bool:
    """A sentence about the front being blocked (not a vehicle, a mover, a fall or a fault)."""
    return _way_out(text) is not None or text in (
        BLOCKED_WALL,
        BLOCKED_GLASS,
    ) + tuple(OBJECT_IN_FRONT.values()) + tuple(STEER_IN_FRONT.values())


def _way_out(text: str) -> Optional[str]:
    """The way out a BLOCK sentence gives (left, right, around, stop), or None."""
    if text in _WAY_OUT:
        return _WAY_OUT[text]
    return {TURN_LEFT: "left", TURN_RIGHT: "right", TURN_AROUND: "around", STOP_NO_WAY: "stop"}.get(text)


def _advised_side(text: str) -> Optional[str]:
    """The side a sentence sends the wearer toward (turn or move), or None."""
    if text in (TURN_LEFT, TURN_SLIGHT_LEFT):
        return "left"
    if text in (TURN_RIGHT, TURN_SLIGHT_RIGHT):
        return "right"
    if _WAY_OUT.get(text) in ("left", "right"):
        return _WAY_OUT[text]
    if text in _STEER_INFO:
        return _STEER_INFO[text]["turn"]
    if text in _MOVER_INFO:
        return _MOVER_INFO[text]["move"]
    if text in (NEAR_LEFT, DRIFT_LEFT, GLASS_LEFT, VEHICLE_FROM_LEFT):
        return "right"  # "..., move right" / "turn slightly right"
    if text in (NEAR_RIGHT, DRIFT_RIGHT, GLASS_RIGHT, VEHICLE_FROM_RIGHT):
        return "left"
    return None


def _turn_room(walk_decision: Dict[str, Any]) -> Dict[str, float]:
    """The walk policy's own room to the left and right (walk_policy._room_ft)."""
    from app.perception.walk_policy import _room_ft

    views = (walk_decision or {}).get("directions") or {}
    out = {}
    for s in ("left", "right"):
        try:
            if views.get(s):
                out[s] = float(_room_ft(views[s]))
        except Exception:
            pass
    return out


def _advice_closed(spoken: str, text: str, raw: Dict[str, Any]) -> bool:
    """Has the side the last sentence sent the wearer toward closed (under WALK_MIN_TURN_FT
    of room, or unknown) while something else is now to be said? While blocked, also when
    the walk policy's room that way is under the floor a turn needs from here (2026-11-09:
    someone stepped in 2.5 ft to the side given, 3 ft from the block, and the new way out
    waited the 6 s flip settle)."""
    side = _advised_side(spoken)
    if not side or not text or text == spoken:
        return False
    room = (raw.get("side_room") or {}).get(side)
    if isinstance(room, (int, float)) and room < WALK_MIN_TURN_FT:
        return True
    floor = raw.get("turn_floor")
    way = (raw.get("turn_room") or {}).get(side)
    return isinstance(floor, (int, float)) and isinstance(way, (int, float)) and way < floor


# Taken only to count a cue (_Gate.next_seq): never while waiting on anything else.
_SEQ_LOCK = threading.Lock()


class _Gate:
    """Decides whether a sentence is worth SAYING on this poll."""

    def __init__(self):
        self._spoken: str = ""  # last sentence actually announced
        self._level: int = -1
        self._spoken_at: float = 0.0
        self._busy_until: float = 0.0  # when that sentence finishes playing
        self._has_spoken: bool = False
        self._repeats: int = 0  # consecutive re-statements of _spoken
        self._pending: str = ""  # candidate waiting out the debounce
        self._pending_since: float = 0.0
        self._episode: Optional[int] = None  # the obstacle _spoken was about
        self._seq: int = 0  # never goes backwards, even on reset
        # The sentence the latest cue_seq counted: guidance OR a status notice.
        self._counted: str = ""
        self._status_busy_until: float = 0.0
        self._status: str = ""  # the last status notice admitted
        # A status notice a block cut off before it was heard, for guidance() to tell again
        # (take_cut_status).
        self._cut_status: str = ""

    def admit(
        self,
        text: str,
        now: float,
        episode: Optional[int] = None,
        closed: bool = False,
        evidence: bool = True,
    ) -> Tuple[bool, int, str]:
        """(say it, sequence number, why not). `closed`: the side the last sentence sent the
        wearer toward has no room now (_advice_closed) — its correction is news. `evidence`:
        this poll's own reading still shows the block (False while only a hold keeps it)."""
        if not text:
            self._pending, self._pending_since = "", 0.0
            return False, self._seq, "nothing to say"
        level = _LEVEL[text]
        if self._has_spoken and now < self._spoken_at:
            step = now - self._spoken_at
            self._spoken_at += step
            self._busy_until += step
            self._pending_since = min(self._pending_since + step, now)
        self._unstep_status(now)
        if text != self._pending:
            self._pending, self._pending_since = text, now

        if text == self._spoken and (episode is None or episode == self._episode):
            if level < _BLOCKED:
                return False, self._seq, "already said — not repeated"
            # Still blocked (perhaps after turning into something else) is said again
            # soon; "Guidance stopped" for as long as it lasts (2026-10-21).
            if text == SENSORS_OFFLINE:
                window = SIMPLE_OFFLINE_REPEAT_S
            else:
                base = (
                    SIMPLE_BLOCKED_REPEAT_S
                    if level == _BLOCKED and text not in _MOVER_SENTENCES
                    else SIMPLE_REPEAT_S
                )
                window = min(base * (2**self._repeats), SIMPLE_REPEAT_MAX_S)
            since = now - self._spoken_at
            if since < window:
                return (
                    False,
                    self._seq,
                    f"unchanged — said {since:.0f}s ago (repeats after {window:.0f}s)",
                )
            # A block is said again only while it is still measured, not off a hold that is
            # releasing: "Blocked by wall, turn right" again, facing an open path, told the
            # wearer to turn a second time (timing review, 2026-11-09).
            if _is_block(text) and not evidence:
                return False, self._seq, "not repeated — the block is no longer measured"
            return self._commit(text, level, now, episode)

        arrived = self._spoken in (DRIFT_LEFT, DRIFT_RIGHT) and text in (
            NEAR_LEFT,
            NEAR_RIGHT,
        )
        # A camera vehicle or a LiDAR mover coming at the user is news, and so is its passing.
        vehicle_news = text in _APPROACHING and self._spoken not in _APPROACHING
        after_vehicle = (
            self._spoken in _APPROACHING
            and text not in _APPROACHING
            and level >= _BLOCKED
        )
        offline_news = text == SENSORS_OFFLINE and self._spoken != SENSORS_OFFLINE
        # Back from "Guidance stopped": what stands now is said at once, not after the gap
        # (2026-11-09: a block waited ~3 s behind it). A block only if still measured — not
        # one a hold kept through the outage, from before the wearer turned.
        recovered = (
            self._spoken == SENSORS_OFFLINE
            and text != SENSORS_OFFLINE
            and (evidence or not _is_block(text))
        )
        # Something on the path after a wall beside you is news, said once that sentence ends (2026-10-01).
        path_news = self._spoken in _SIDE_WALL + _GLASS_BESIDE and text in _PATH_AHEAD
        # And a wall beside you, or a way past, after "Obstacle ahead" (2026-10-21): until
        # then that obstacle was "Walk straight", which opened no episode to settle in.
        side_news = (
            self._spoken == OBSTACLE_AHEAD
            and text in _SIDE_WALL + _GLASS_BESIDE + tuple(_STEER_INFO)
        )
        busy_until = max(
            self._busy_until if self._has_spoken else 0.0, self._status_busy_until
        )
        if level >= 1 and (
            level > self._level
            or arrived
            or vehicle_news
            or after_vehicle
            or offline_news
            or path_news
            or side_news
            or recovered
        ):
            # A block cuts into a guidance sentence like a vehicle does (2026-10-21):
            # waiting out a 2.5 s side-wall sentence cost ~3 m at walking pace, inside the
            # 4 ft block distance. Not into a vehicle or mover warning, which must be heard
            # whole. Into a status notice too (2026-11-10: a post 1 m ahead waited 1.1 s
            # behind "Power low"); the notice it cut is told again (take_cut_status).
            guidance_playing = self._has_spoken and now < self._busy_until
            # Any hazard cuts into a level-0 sentence ("Walk straight"): it warns of nothing,
            # and waiting it out cost ~0.8 s of walking toward the thing (2026-11-09).
            over_clear = (
                guidance_playing
                and self._level == 0
                and now >= self._status_busy_until
            )
            if (
                level < _VEHICLE
                and now < busy_until
                and not over_clear
                and (
                    level < _BLOCKED
                    or (guidance_playing and self._spoken in _APPROACHING)
                )
            ):
                return (
                    False,
                    self._seq,
                    f"more urgent — waiting "
                    f"{busy_until - now:.1f}s for the last sentence to end",
                )
            return self._commit(text, level, now, episode)

        # Nor through the gap after it: a block a hold kept through the outage is not said
        # unless this poll still measures it (timing review, 2026-11-09: the wearer turned
        # away during a stall, and "Blocked by glass, turn right" came on an open path).
        if self._spoken == SENSORS_OFFLINE and _is_block(text) and not evidence:
            return False, self._seq, "after the outage — the block is not measured now"
        held = now - self._pending_since
        need = SIMPLE_CLEAR_SAY_CONFIRM_S if level == 0 else SIMPLE_CONFIRM_S
        # One approach warning becoming another (review, 2026-11-06): until then only a
        # vehicle's flip was news, so "Person approaching from left, stop" -> "Person coming
        # toward you, move right" waited SIMPLE_SETTLE_S (6 s), past the 5 s horizon. Still
        # held SIMPLE_CONFIRM_S, so a side flickering tick to tick is not said.
        flipped = self._spoken in _APPROACHING and text in _APPROACHING
        if (
            level >= 1
            and self._has_spoken
            and episode is not None
            and episode == self._episode
        ):
            upgrade = (
                self._spoken in _TURNS
                and text not in _TURNS
                and level == self._level
            )
            upgrade = (
                upgrade
                or flipped
                or (self._spoken in _APPROACHING and text not in _APPROACHING)
            )
            upgrade = upgrade or self._spoken in (SENSORS_OFFLINE, STOP_NO_WAY)
            # A way out that changed while the one given still has room waits the settle: it
            # is still good advice, and saying every change flip-flopped "turn right / turn
            # left" near a post (turn review, 2026-11-09). One that CLOSED is `closed` below.
            # The advice just given is wrong now (review, 2026-11-06): "Turn right", then
            # someone stepped in 1.5 ft to the right, and "Turn left" waited 6 s.
            upgrade = upgrade or closed
            if not upgrade:
                need = SIMPLE_SETTLE_S
                if {_way_out(text), _way_out(self._spoken)} == {"left", "right"}:
                    need = SIMPLE_FLIP_SETTLE_S
        if held < need:
            return (
                False,
                self._seq,
                f"same obstacle — held {held:.1f}s of {need:.0f}s"
                if need >= SIMPLE_SETTLE_S
                else f"unconfirmed — held {held:.1f}s of {need:.1f}s",
            )
        free_at = max(
            self._spoken_at + SIMPLE_MIN_GAP_S, self._busy_until + SIMPLE_AFTER_SPEECH_S
        )
        if self._has_spoken and now < free_at and not (flipped or closed):
            return (
                False,
                self._seq,
                f"too soon — {free_at - now:.1f}s until the voice is free",
            )
        if now < self._status_busy_until:
            return (
                False,
                self._seq,
                f"too soon — "
                f"{self._status_busy_until - now:.1f}s until the status ends",
            )
        return self._commit(text, level, now, episode)

    def _unstep_status(self, now: float) -> None:
        if self._status_busy_until - now > 2.0 * _airtime(GLASS_SENSOR_OFF):
            self._status_busy_until = now

    def admit_status(
        self, text: str, now: float, cut_in: bool = False
    ) -> Tuple[bool, int, str]:
        """Admit a STATUS notice ("Glass sensor off/on", "Camera off/on"): its own cue_seq, and nothing else about the gate changes (audit pass 2, NEW-028). `cut_in`: do not wait for the sentence playing — "Guidance stopped" (2026-10-23)."""
        if self._has_spoken and now < self._spoken_at:
            self._busy_until = min(self._busy_until, now)  # clock stepped back
        self._unstep_status(now)
        busy_until = max(
            self._busy_until if self._has_spoken else 0.0, self._status_busy_until
        )
        if now < busy_until and not cut_in:
            return (
                False,
                self._seq,
                f"status — waiting "
                f"{busy_until - now:.1f}s for the last sentence to end",
            )
        if now < self._status_busy_until and self._status != text:
            self._cut_status = self._status  # "Guidance stopped" cut it off
        self._status_busy_until = now + _airtime(text)
        self._status = text
        self._counted = text
        return True, self.next_seq(), ""

    def _commit(
        self, text: str, level: int, now: float, episode: Optional[int] = None
    ) -> Tuple[bool, int, str]:
        if now < self._status_busy_until:  # said over a status notice: the voice cuts it
            self._cut_status = self._status
            self._status_busy_until = now
        same = text == self._spoken and (episode is None or episode == self._episode)
        self._repeats = self._repeats + 1 if same else 0
        self._spoken, self._level, self._spoken_at = text, level, now
        self._episode = episode
        self._busy_until = now + _airtime(text)
        self._has_spoken = True
        self._counted = text
        return True, self.next_seq(), ""

    def next_seq(self) -> int:
        """The next cue number. Under its own lock, not the guidance lock: the stall watch
        takes one while a hung tick holds that (stall_cue, 2026-11-09)."""
        with _SEQ_LOCK:
            self._seq += 1
            return self._seq

    def settle(self, level: int) -> None:
        """The situation the last sentence described has eased to `level`."""
        if 0 <= level < self._level:
            self._level = level

    def take_cut_status(self) -> str:
        """The status notice a sentence cut off before it was heard, once ("" if none)."""
        cut, self._cut_status = self._cut_status, ""
        return cut

    def forget_pending(self) -> None:
        """Drop the candidate waiting out the debounce, as an empty poll does."""
        self._pending, self._pending_since = "", 0.0

    def forget_said(self) -> None:
        """Forget the last guidance sentence, so the current one is said again: after "Guidance stopped" (a fault) or "Traffic light ahead", what was said before no longer stands (2026-10-21)."""
        self._spoken, self._level, self._episode, self._repeats = "", -1, None, 0

    @property
    def last(self) -> str:
        """The last GUIDANCE sentence announced (a status notice is not one)."""
        return self._spoken

    @property
    def counted(self) -> str:
        """The sentence the sequence number last counted — guidance or status."""
        return self._counted

    @property
    def level(self) -> int:
        """Urgency of the last sentence spoken, -1 before the first."""
        return self._level

    def state(self) -> Dict[str, Any]:
        return {"last": self._spoken, "cue_seq": self._seq, "pending": self._pending}

    def reset(self) -> None:
        with _SEQ_LOCK:
            seq = self._seq
            self.__init__()
            self._seq = seq
