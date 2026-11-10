#!/usr/bin/env python3
"""Replay a recorded walk through the CURRENT guidance code — what would it say now?

A recording (logs/walks/*.jsonl.gz, see app/core/walk_recorder.py) holds every guidance
tick's inputs and what was said. This feeds the inputs back, in order and on the
recorded clocks, through simple_guidance (and by default walk_policy too), and lists
every announcement that is now different: changed, no longer said, or newly said.
Tester marks (/guidance/mark) are shown where they fell.

    venv/bin/python tools/field/walk_replay.py logs/walks/walk-20261010-101500.jsonl.gz
    venv/bin/python tools/field/walk_replay.py --all logs/walks/*.jsonl.gz      # every line

Run it once on unchanged code first: "fidelity" should be 100% — if it is not, the
replay itself differs from the device (a state the recording does not carry), and the
differences it lists are not all yours.

--turn-on replays the walk with features that run on the device but are not yet spoken
switched on (2026-11-06): what the wearer WOULD have heard from them, and when — the
evidence for turning each one on in app/config.py. Each changed line names its feature:

    venv/bin/python tools/field/walk_replay.py --turn-on tof-hazards logs/walks/walk-*.jsonl.gz
    venv/bin/python tools/field/walk_replay.py --turn-on all logs/walks/walk-*.jsonl.gz
    venv/bin/python tools/field/walk_replay.py --turn-on tof-wall logs/walks/clip-*.jsonl.gz

Record a walk with SHAPPE_RECORD_WALK=1 ./start_lidar.sh (or POST /guidance/record), and
mark what happens on it with POST /guidance/mark (stairs here, a false alarm there).
"""

import contextlib

import argparse
import gzip
import json
import sys
import zlib
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

# What runs on the device but is said only once a walk shows it is right: --turn-on name ->
# the config flag that lets it speak.
FEATURES = {
    "tof-hazards": "TOF_HAZARDS_SPEAK",  # stairs, steps, drops, poles (app/perception/ground_hazards)
    "movers": "LIDAR_MOVERS_SPEAK",  # people and vehicles coming at the wearer
    "route": "ROUTE_PLANNER_SPEAK",  # the route planner's turns and steers
    "tof-distance": "TOF_USE_DISTANCE",  # the ToF's distance for what the camera names
    "tof-wall": "TOF_WALL_SPEAK",  # an unnamed block a wall, by the ToF (2026-11-10)
}


def turned_on(names):
    """The named features' flags True everywhere they are read: config, and every loaded
    utils module that imported one by value (simple_guidance, motion_tracker, ...)."""
    from app import config  # noqa: F401  (loaded, so it is patched with the rest)
    from app.guidance import simple_guidance  # noqa: F401

    stack = contextlib.ExitStack()
    for name in names:
        flag = FEATURES[name]
        for modname, mod in list(sys.modules.items()):
            if (modname == "app.config" or modname.startswith("app.")) and hasattr(mod, flag):
                stack.enter_context(mock.patch.object(mod, flag, True))
    return stack


def _source(sg, text, got):
    """Which not-yet-spoken feature a sentence came from, if one did."""
    raw = got.get("raw") or {}
    if text in sg._GROUND_SENTENCES or (text and text == raw.get("ground_would_say")):
        return "tof-hazards"
    if text in sg._MOVER_SENTENCES:
        return "movers"
    if text and text == raw.get("route_would_say"):
        return "route"
    if text and raw.get("tof_wall_named"):
        return "tof-wall"
    return ""


def _ticks(path):
    n = 0
    try:
        with gzip.open(path, "rt", encoding="utf-8") as fh:
            for n, line in enumerate(fh, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    yield json.loads(line)
                except json.JSONDecodeError:
                    print(f"  (line {n} unreadable — the recording was cut off there)")
                    return
    except (EOFError, OSError, zlib.error) as e:  # gzip.BadGzipFile is an OSError
        # A recording the app never closed — shut down mid-walk, or the power went —
        # replays up to its last whole line instead of aborting (scan 2026-10-23).
        print(f"  (the recording ends cut off after line {n}: {type(e).__name__})")


def replay_ticks(path, recompute_policy=True, turn_on=()):
    """Feed a recording through the current code on its recorded clocks: yields (line,
    guidance now) per tick, (line, None) per tester mark. time.time and time.monotonic are
    the recording's while the caller looks at each, so read no real clock there.
    tools/field/label_report.py replays the labelled clips with it."""
    from app.perception import ground_hazards
    from app.guidance import simple_guidance as sg
    from app.perception import walk_policy

    sg.reset()
    walk_policy._last_measured.clear()
    walk_policy._remember_turn(None, 0.0)

    # walk_policy reads the ToF itself (ground_hazards.snapshot(), while its worker runs): it
    # gets the analysis recorded with each tick, as the device's guidance did. Without it the
    # desk's echo replayed as "Blocked by glass" where the device said "Obstacle in front"
    # (2026-11-10: the ToF's "solid, not glass" check had nothing to look at).
    class _Running:
        @staticmethod
        def is_alive():
            return True

    tof = {"snap": None}
    clock = {"t": 0.0, "mono": 0.0}
    with (
        mock.patch("time.time", lambda: clock["t"]),
        mock.patch("time.monotonic", lambda: clock["mono"]),
        mock.patch.object(ground_hazards, "snapshot", lambda: dict(tof["snap"] or {})),
        mock.patch.object(ground_hazards, "_thread", _Running()),
        turned_on(turn_on),
    ):
        for rec in _ticks(path):
            clock["t"], clock["mono"] = float(rec["t"]), float(rec["mono"])
            if "mark" in rec:
                yield rec, None
                continue
            inp = rec["in"]
            tof["snap"] = inp.get("ground")
            wd = (
                walk_policy.decide(inp["lidar"], inp["radar"])
                if recompute_policy and inp.get("lidar")
                else inp["walk_decision"]
            )
            yield rec, sg.guidance(
                wd,
                inp["lidar"],
                now=clock["t"],
                hold=inp.get("hold"),
                hold_lidar_absent=bool(inp.get("hold_lidar_absent")),
                at_crossing=bool(inp.get("at_crossing")),
                radar_state=inp.get("radar"),
                camera_state=inp.get("camera"),
                motion=inp.get("motion"),
                movers=inp.get("movers"),
                ground=inp.get("ground"),
            )


def replay(path, recompute_policy=True, show_all=False, turn_on=()):
    """Print the differences; returns (ticks, said before, said now, differences).
    `turn_on`: FEATURES names to replay as if spoken."""
    from app.guidance import simple_guidance as sg

    rows = []  # (t, before, now, why now, feature) — announcements only
    ticks = same_ticks = 0
    t0 = None
    for rec, got in replay_ticks(path, recompute_policy, turn_on):
        t = float(rec["t"])
        t0 = t if t0 is None else t0
        if got is None:
            kind = rec["mark"] + (f" {rec['label']}" if rec.get("label") else "")
            rows.append((t - t0, "mark", kind, rec.get("text", ""), ""))
            continue
        out = rec["out"]
        ticks += 1
        before = out["instruction"] if out.get("speak") else ""
        now = got["instruction"] if got.get("speak") else ""
        same_ticks += before == now
        if before or now:
            rows.append((t - t0, before, now, got.get("reason", ""), _source(sg, now, got)))

    said_before = sum(1 for r in rows if r[1] != "mark" and r[1])
    said_now = sum(1 for r in rows if r[1] != "mark" and r[2])
    diffs = 0
    by_feature = {}
    print(f"\n{path}" + (f"   (turned on: {', '.join(turn_on)})" if turn_on else ""))
    for t, before, now, why, feature in rows:
        if before == "mark":
            print(f"  {t:7.1f}s  ** tester mark: {now.upper()}  {why}")
            continue
        differs = before != now
        diffs += differs
        if differs and feature:
            by_feature[feature] = by_feature.get(feature, 0) + 1
            why = f"[{feature}] {why}"
        if not (differs or show_all):
            continue
        tag = (
            "same   "
            if not differs
            else "NEW    "
            if not before
            else "GONE   "
            if not now
            else "CHANGED"
        )
        print(
            f"  {t:7.1f}s  {tag}  before: {before or '—':<38}  now: {now or '—':<38}"
            + (f"  ({why})" if now and differs else "")
        )
    fidelity = 100.0 * same_ticks / ticks if ticks else 0.0
    print(
        f"  {ticks} ticks · said before {said_before} · said now {said_now} · "
        f"{diffs} different · fidelity {fidelity:.1f}% of ticks identical"
    )
    if turn_on:
        print(
            "  changed by what was turned on: "
            + (", ".join(f"{k} {v}" for k, v in sorted(by_feature.items())) or "nothing")
            + "  (the rest: the code changed since the walk)"
        )
    return ticks, said_before, said_now, diffs


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("recordings", nargs="+", help="logs/walks/*.jsonl.gz")
    ap.add_argument(
        "--all",
        action="store_true",
        help="list every announcement, not only the ones that changed",
    )
    ap.add_argument(
        "--recorded-policy",
        action="store_true",
        help="use the recorded "
        "walk_policy decision instead of running the current walk_policy",
    )
    ap.add_argument(
        "--turn-on",
        default="",
        metavar="FEATURES",
        help="replay as if these not-yet-spoken features were spoken, comma-separated: "
        + ", ".join(FEATURES)
        + ", or all",
    )
    args = ap.parse_args()
    turn_on = [f.strip() for f in args.turn_on.split(",") if f.strip()]
    if turn_on == ["all"]:
        turn_on = list(FEATURES)
    unknown = [f for f in turn_on if f not in FEATURES]
    if unknown:
        ap.error(f"unknown feature(s) {', '.join(unknown)}: use {', '.join(FEATURES)} or all")
    total = 0
    for path in args.recordings:
        total += replay(
            path,
            recompute_policy=not args.recorded_policy,
            show_all=args.all,
            turn_on=turn_on,
        )[3]
    return 0 if total == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
