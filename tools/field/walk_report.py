#!/usr/bin/env python3
"""A recorded walk, summed up: what was said and when, how far off, what looks wrong.

    venv/bin/python tools/field/walk_report.py logs/walks/walk-20261120-103105.jsonl.gz
    venv/bin/python tools/field/walk_report.py --replay logs/walks/walk-*.jsonl.gz
    venv/bin/python tools/field/walk_report.py --replay --turn-on movers logs/walks/walk-*.jsonl.gz
    venv/bin/python tools/field/walk_report.py --timeline logs/walks/clip-*.jsonl.gz

A recording (SHAPPE_RECORD_WALK=1, or a tester's Wrong/Missed/label clip) holds every
guidance tick's inputs and what was said (app/core/walk_recorder). This reads it as it is
(no replay unless asked) and reports:

  sensors      the LiDAR's scan age and gaps, the front radar's (the glass sensor's) losses
  said         every sentence, how often, a minute; with --timeline each one, with the front
               distance when it was said
  blocks       each block warning and how far off the thing was: one said nearer than
               WALK_BLOCK_FT less 1 ft is late
  checks       what the desk simulators check (tools/sim/walk_sim), on the real walk: "Walk
               straight" with something the laser measures within WALK_BLOCK_FT; a way out
               turned left <-> right within FLIP_S; "Near left wall" <-> "Near right wall"
               within FLIP_S; the same sentence again within REPEAT_S
  marks        each tester mark, with what was said in the MARK_BEFORE_S before it
  would say    what each feature not yet spoken (walk_replay.FEATURES) would have said, as
               the device logged it ("mover_shadow", "route_shadow", ...)

--replay also feeds the walk through the CURRENT code (tools/field/walk_replay): how much of
it is said the same ("fidelity"), and, each not-yet-spoken feature turned on in turn, how
many announcements it changes: the evidence for turning each on. (added 2026-11-20)"""

import argparse
import gzip
import json
import statistics
import sys
import time
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from app import config as c  # noqa: E402
from app.guidance import simple_guidance as sg  # noqa: E402
from app.guidance.speech_gate import _is_block, _way_out  # noqa: E402

FLIP_S = 6.0  # a way out turned the other way this soon is back and forth
REPEAT_S = 5.0
MARK_BEFORE_S = 5.0
LATE_FT = c.WALK_BLOCK_FT - 1.0
SHADOWS = {
    "mover_shadow": "movers",
    "route_shadow": "route",
    "wall_shadow": "tof-wall",
    "ground_shadow": "tof-hazards",
    "closing_shadow": "glass-closing",
}


def read(path):
    """(ticks, marks, cut off?): the recording's lines, a cut-off end read up to its last
    whole line, as walk_replay reads one."""
    ticks, marks, cut = [], [], False
    try:
        with gzip.open(path, "rt", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    cut = True
                    break
                (marks if "mark" in rec else ticks).append(rec)
    except (EOFError, OSError, zlib.error):
        cut = True
    return ticks, marks, cut


def _front(rec):
    """(distance ft, measured by the laser?) of the front as the walk policy read it."""
    front = (((rec.get("in") or {}).get("walk_decision") or {}).get("directions") or {}).get(
        "front"
    ) or {}
    ft = front.get("distance_ft")
    return (float(ft) if isinstance(ft, (int, float)) else None), bool(front.get("lidar_measured"))


def _clock(t, t0):
    s = max(0.0, t - t0)
    return f"{int(s // 60)}:{s % 60:04.1f}"


def said_rows(ticks):
    """[(t, sentence, front ft, laser-measured, rule)] for each tick that spoke."""
    rows = []
    for rec in ticks:
        out = rec.get("out") or {}
        if out.get("speak") and out.get("instruction"):
            ft, measured = _front(rec)
            rows.append((float(rec["t"]), out["instruction"], ft, measured, out.get("rule") or ""))
    return rows


def sensor_lines(ticks, t0):
    lines = []
    ages, gaps, gap_start = [], [], None
    for rec in ticks:
        t = float(rec["t"])
        stamp = ((rec.get("in") or {}).get("lidar") or {}).get("last_update")
        age = t - float(stamp) if isinstance(stamp, (int, float)) and stamp > 0 else None
        if age is not None:
            ages.append(age)
        stale = age is None or age > c.LIDAR_STALE_S
        if stale and gap_start is None:
            gap_start = t
        if not stale and gap_start is not None:
            gaps.append((gap_start, t - gap_start))
            gap_start = None
    if gap_start is not None:
        gaps.append((gap_start, float(ticks[-1]["t"]) - gap_start))
    if ages:
        worst = max(gaps, key=lambda g: g[1]) if gaps else None
        lines.append(
            f"LiDAR: scan age median {1000 * statistics.median(ages):.0f} ms, worst "
            f"{max(ages):.2f} s; {len(gaps)} gap(s) over {c.LIDAR_STALE_S:g} s"
            + (f", the longest {worst[1]:.1f} s at {_clock(worst[0], t0)}" if worst else "")
        )
    else:
        lines.append("LiDAR: no scan in the recording")
    online = [
        bool(
            (
                (((rec.get("in") or {}).get("radar") or {}).get("sectors") or {}).get("front") or {}
            ).get("online")
        )
        for rec in ticks
    ]
    losses, longest, run = 0, 0.0, None
    for rec, ok in zip(ticks, online):
        t = float(rec["t"])
        if not ok and run is None:
            run, losses = t, losses + 1
        if ok and run is not None:
            longest, run = max(longest, t - run), None
    if run is not None:
        longest = max(longest, float(ticks[-1]["t"]) - run)
    lines.append(
        f"front radar (the glass sensor): online {100 * sum(online) / len(online):.1f}%"
        f" of ticks" + (f"; lost {losses} time(s), the longest {longest:.1f} s" if losses else "")
    )
    return lines


def check_lines(rows, t0):
    """What the desk simulators check, on the real walk: [(rule, [instances])]."""
    straight = [
        (t, ft)
        for t, text, ft, measured, _r in rows
        if text == sg.WALK_STRAIGHT and measured and ft is not None and ft < c.WALK_BLOCK_FT
    ]
    flips, near_flips, repeats = [], [], []
    last_way = last_near = None
    last_said = {}
    for t, text, *_r in rows:
        way = _way_out(text)
        if way in ("left", "right"):
            if last_way and last_way[1] != way and t - last_way[0] <= FLIP_S:
                flips.append((t, f"{last_way[1]} -> {way}"))
            last_way = (t, way)
        if text in (sg.NEAR_LEFT, sg.NEAR_RIGHT):
            if last_near and last_near[1] != text and t - last_near[0] <= FLIP_S:
                near_flips.append((t, text))
            last_near = (t, text)
        if text in last_said and t - last_said[text] <= REPEAT_S:
            repeats.append((t, text))
        last_said[text] = t
    return [
        (
            f"'Walk straight' said with something the laser measures within {c.WALK_BLOCK_FT:g} ft",
            [f"{_clock(t, t0)} ({ft:.1f} ft)" for t, ft in straight],
        ),
        (
            f"a way out turned left <-> right within {FLIP_S:g} s",
            [f"{_clock(t, t0)} {w}" for t, w in flips],
        ),
        (
            f"'Near left wall' <-> 'Near right wall' within {FLIP_S:g} s",
            [f"{_clock(t, t0)} {x!r}" for t, x in near_flips],
        ),
        (
            f"the same sentence again within {REPEAT_S:g} s",
            [f"{_clock(t, t0)} {x!r}" for t, x in repeats],
        ),
    ]


def shadow_lines(ticks, t0):
    """{feature: [(t, sentence)]}: each new "would say" as the device logged it."""
    out = {}
    for rec in ticks:
        o = rec.get("out") or {}
        for key, feature in SHADOWS.items():
            if o.get(key):
                out.setdefault(feature, []).append((float(rec["t"]), o[key]))
    return out


def replay_lines(path, features, t0):
    """[line]: the walk through the current code, and each feature turned on in turn."""
    from tools.field.walk_replay import replay_ticks

    def spoken(turn_on=()):
        said, same, n = {}, 0, 0
        for rec, got in replay_ticks(path, True, turn_on):
            if got is None:
                continue
            n += 1
            before = rec["out"]["instruction"] if rec["out"].get("speak") else ""
            now = got["instruction"] if got.get("speak") else ""
            same += before == now
            if now:
                said[float(rec["t"])] = now
        return said, (100.0 * same / n if n else 0.0)

    base, fidelity = spoken()
    lines = [
        f"through this code: {fidelity:.1f}% of ticks said the same ({len(base)} announcements)"
    ]
    for name in features:
        said, _f = spoken((name,))
        changed = sorted(t for t in set(base) | set(said) if base.get(t) != said.get(t))
        lines.append(
            f"  turned on, {name:14} {len(changed)} announcement(s) change"
            + (f", the first at {_clock(changed[0], t0)}" if changed else "")
        )
    return lines


def report(path, timeline=False, replay=False, features=()):
    ticks, marks, cut = read(path)
    print(f"\n{path}")
    if not ticks:
        print("  no guidance ticks in it" + (" (cut off)" if cut else ""))
        return
    t0, t1 = float(ticks[0]["t"]), float(ticks[-1]["t"])
    span = max(1e-6, t1 - t0)
    print(
        f"  {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t0))}, {_clock(t1, t0)} long, "
        f"{len(ticks)} ticks ({len(ticks) / span:.1f} a second), {len(marks)} tester mark(s)"
        + ("; the end is cut off" if cut else "")
    )

    print("\nsensors")
    for line in sensor_lines(ticks, t0):
        print(f"  {line}")

    rows = said_rows(ticks)
    print(f"\nsaid: {len(rows)} sentence(s), {60 * len(rows) / span:.1f} a minute")
    counts = {}
    for _t, text, *_r in rows:
        counts[text] = counts.get(text, 0) + 1
    for text, n in sorted(counts.items(), key=lambda kv: -kv[1])[:12]:
        print(f"  {n:4d}  {text}")
    if timeline:
        print("\ntimeline")
        for t, text, ft, measured, rule in rows:
            where = f"front {ft:5.1f} ft{'' if measured else ' (radar)'}" if ft is not None else ""
            print(f"  {_clock(t, t0):>8}  {text:<46} {where}  {rule}")

    blocks = [
        (t, text, ft) for t, text, ft, _m, _r in rows if _is_block(text) or text == sg.STOP_NO_WAY
    ]
    print(f"\nblocks: {len(blocks)}")
    late = 0
    for t, text, ft in blocks:
        flag = ""
        if ft is not None and ft < LATE_FT:
            late, flag = late + 1, f"   <- nearer than {LATE_FT:g} ft"
        print(f"  {_clock(t, t0):>8}  {text:<46} {'' if ft is None else f'{ft:.1f} ft'}{flag}")
    if late:
        print(f"  {late} said nearer than {LATE_FT:g} ft")

    print("\nchecks")
    for rule, hits in check_lines(rows, t0):
        print(
            f"  {'ok  ' if not hits else f'{len(hits):3d} '} {rule}"
            + (f": {', '.join(hits[:6])}" + (" ..." if len(hits) > 6 else "") if hits else "")
        )

    if marks:
        print("\ntester marks")
        for m in marks:
            t = float(m["t"])
            before = [
                f"{_clock(rt, t0)} {text!r}"
                for rt, text, *_r in rows
                if t - MARK_BEFORE_S <= rt <= t
            ]
            kind = m.get("mark", "?") + (f" {m['label']}" if m.get("label") else "")
            note = f' "{m["text"]}"' if m.get("text") else ""
            print(
                f"  {_clock(t, t0):>8}  {kind.upper()}{note}"
                + (f" (said then: {m['said']!r})" if m.get("said") else "")
            )
            print(
                f"            said in the {MARK_BEFORE_S:g} s before: {', '.join(before) or 'nothing'}"
            )

    shadows = shadow_lines(ticks, t0)
    print("\nwould say (features not yet spoken, as the device logged them)")
    if not shadows:
        print("  nothing")
    for feature, items in sorted(shadows.items()):
        firsts = ", ".join(f"{_clock(t, t0)} {x!r}" for t, x in items[:4])
        print(f"  {feature:14} {len(items):3d}: {firsts}" + (" ..." if len(items) > 4 else ""))

    if replay:
        print("\nreplayed")
        for line in replay_lines(path, features, t0):
            print(f"  {line}")


def main():
    from tools.field.walk_replay import FEATURES

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("paths", nargs="+", help="recordings (logs/walks/*.jsonl.gz)")
    ap.add_argument("--timeline", action="store_true", help="every sentence, when and how far")
    ap.add_argument(
        "--replay",
        action="store_true",
        help="also through the current code, each feature not yet spoken turned on",
    )
    ap.add_argument(
        "--turn-on",
        action="append",
        default=[],
        metavar="FEATURE",
        choices=sorted(FEATURES),
        help="with --replay: a feature to turn on, again for more (default: each)",
    )
    args = ap.parse_args()
    for path in args.paths:
        report(path, args.timeline, args.replay, args.turn_on or sorted(FEATURES))
    return 0


if __name__ == "__main__":
    sys.exit(main())
