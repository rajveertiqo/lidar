#!/usr/bin/env python3
"""Replay a recorded walk through the CURRENT guidance code — what would it say now?

A recording (logs/walks/*.jsonl.gz, see utils/walk_recorder.py) holds every guidance
tick's inputs and what was said. This feeds the inputs back, in order and on the
recorded clocks, through simple_guidance (and by default walk_policy too), and lists
every announcement that is now different: changed, no longer said, or newly said.
Tester marks (/guidance/mark) are shown where they fell.

    venv/bin/python tools/walk_replay.py logs/walks/walk-20261010-101500.jsonl.gz
    venv/bin/python tools/walk_replay.py --all logs/walks/*.jsonl.gz      # every line

Run it once on unchanged code first: "fidelity" should be 100% — if it is not, the
replay itself differs from the device (a state the recording does not carry), and the
differences it lists are not all yours.
"""

import argparse
import gzip
import json
import sys
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


def _ticks(path):
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


def replay(path, recompute_policy=True, show_all=False):
    """Print the differences; returns (ticks, said before, said now, differences)."""
    from utils import simple_guidance as sg
    from utils import walk_policy

    sg.reset()
    walk_policy._last_measured.clear()
    walk_policy._remember_turn(None, 0.0)

    clock = {"t": 0.0, "mono": 0.0}
    rows = []                          # (t, before, now, why now) — announcements only
    ticks = same_ticks = 0
    t0 = None
    with mock.patch("time.time", lambda: clock["t"]), \
         mock.patch("time.monotonic", lambda: clock["mono"]):
        for rec in _ticks(path):
            clock["t"], clock["mono"] = float(rec["t"]), float(rec["mono"])
            t0 = clock["t"] if t0 is None else t0
            if "mark" in rec:
                rows.append((clock["t"] - t0, "mark", rec["mark"], rec.get("text", "")))
                continue
            inp, out = rec["in"], rec["out"]
            wd = (walk_policy.decide(inp["lidar"], inp["radar"])
                  if recompute_policy and inp.get("lidar") else inp["walk_decision"])
            got = sg.guidance(wd, inp["lidar"], now=clock["t"], hold=inp.get("hold"),
                              hold_lidar_absent=bool(inp.get("hold_lidar_absent")),
                              at_crossing=bool(inp.get("at_crossing")),
                              radar_state=inp.get("radar"), camera_state=inp.get("camera"),
                              motion=inp.get("motion"), movers=inp.get("movers"))
            ticks += 1
            before = out["instruction"] if out.get("speak") else ""
            now = got["instruction"] if got.get("speak") else ""
            same_ticks += before == now
            if before or now:
                rows.append((clock["t"] - t0, before, now, got.get("reason", "")))

    said_before = sum(1 for r in rows if r[1] != "mark" and r[1])
    said_now = sum(1 for r in rows if r[1] != "mark" and r[2])
    diffs = 0
    print(f"\n{path}")
    for t, before, now, why in rows:
        if before == "mark":
            print(f"  {t:7.1f}s  ** tester mark: {now.upper()}  {why}")
            continue
        differs = before != now
        diffs += differs
        if not (differs or show_all):
            continue
        tag = ("same   " if not differs else "NEW    " if not before
               else "GONE   " if not now else "CHANGED")
        print(f"  {t:7.1f}s  {tag}  before: {before or '—':<38}  now: {now or '—':<38}"
              + (f"  ({why})" if now and differs else ""))
    fidelity = 100.0 * same_ticks / ticks if ticks else 0.0
    print(f"  {ticks} ticks · said before {said_before} · said now {said_now} · "
          f"{diffs} different · fidelity {fidelity:.1f}% of ticks identical")
    return ticks, said_before, said_now, diffs


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recordings", nargs="+", help="logs/walks/*.jsonl.gz")
    ap.add_argument("--all", action="store_true", help="list every announcement, not "
                    "only the ones that changed")
    ap.add_argument("--recorded-policy", action="store_true", help="use the recorded "
                    "walk_policy decision instead of running the current walk_policy")
    args = ap.parse_args()
    total = 0
    for path in args.recordings:
        total += replay(path, recompute_policy=not args.recorded_policy,
                        show_all=args.all)[3]
    return 0 if total == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
