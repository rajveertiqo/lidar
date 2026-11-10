#!/usr/bin/env python3
r"""What was really there: score the guidance code against the testers' labels, and export
them for training.

A tester's label button on the dashboard ("What was really there?": wall, glass, obstacle,
person, nothing) saves a clip of the sensor data around the press, its mark line carrying
the label (utils/walk_recorder.py, config.MARK_LABELS). This reads those clips (and any
full recordings with labels in them), replays each one through the CURRENT code
(tools/field/walk_replay.py), and for every label compares:

    then  what the device named when the label was pressed (recorded)
    now   what the current code names on the same sensor data (replayed)

What a press is about: the last thing it SAID in the LABEL_SAID_WINDOW_S before the press
(a status notice such as "Camera off" skipped), or what it stood on at the press if it said
nothing. Its sentence is turned into one of the labels: "Obstacle in front, turn left" is
obstacle, "Walk straight" nothing, a chair or a car obstacle, stairs a "ground" (no label).

On the Pi or, better, on a laptop with the repo checked out (training is never done on the
Pi). Copy the clips across first:

    rsync -av shappe@<pi-address>:Documents/lidar/logs/walks/ ~/shappe-walks/
    (Windows: scp "shappe@<pi-address>:Documents/lidar/logs/walks/*.jsonl.gz" $HOME\shappe-walks\)
    venv/bin/python tools/field/label_report.py ~/shappe-walks
    venv/bin/python tools/field/label_report.py ~/shappe-walks --export labels.jsonl

--turn-on tof-wall replays as if the ToF's wall naming were spoken: compare its "right now"
column with a run without it before turning TOF_WALL_SPEAK on.

--export writes one JSON line per label: the label, what was said, and every tick's inputs
from LABEL_EXPORT_S before the press (lidar, radar, camera, ToF ground, ...): the rows a
classifier is trained on. --recorded skips the replay (what was said then only, fast).

Exit status 1 when a label the device got right then is got wrong now: a change that
breaks one. A clip starts 10 s before its press without the device's held state, so run it
once on unchanged code first; what differs then is the replay, not the change.
"""

import argparse
import collections
import glob
import gzip
import json
import os
import sys
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

LABEL_SAID_WINDOW_S = 6.0  # a press is about what was said this long before it, at most
LABEL_EXPORT_S = 2.0  # the ticks exported with each label: this long before the press

# Things the device names that a tester labels "obstacle"; stairs, steps and drops are
# "ground" (no button: they are told apart by the ToF, not labelled here yet).
_GROUND = ("stairs", "step", "drop")


def label_class(text):
    """The label a sentence amounts to: one of config.MARK_LABELS, "ground", or "other"
    for one that is not about what is there (a status notice, a traffic light)."""
    from utils import simple_guidance as sg

    if not text:
        return "nothing"
    named = sg.obstacle_of(text)
    if named in ("wall", "glass", "person"):
        return named
    if named in _GROUND:
        return "ground"
    if named:  # a chair, a car, "something", a low obstacle, an overhang
        return "obstacle"
    if text in sg._WALK_ON:
        return "nothing"
    if _is_notice(text) or text == sg.TRAFFIC_LIGHT_AHEAD:
        return "other"
    return "obstacle"  # "Obstacle in front, turn left", "Turn left", "Stop, no clear way"


def _is_notice(text):
    from utils import simple_guidance as sg

    return text in sg._STATUS_NOTICES + sg._STARTUP_NOTICES + (sg.SENSORS_OFFLINE,)


class _About:
    """What the device was talking about: the last scene sentence said within
    LABEL_SAID_WINDOW_S, else what it stands on now."""

    def __init__(self):
        self.said = collections.deque()  # (mono, sentence)
        self.standing = ""

    def tick(self, mono, out):
        text = (out or {}).get("instruction") or ""
        if text and not _is_notice(text):
            self.standing = text
            if out.get("speak"):
                self.said.append((mono, text))
        while self.said and mono - self.said[0][0] > LABEL_SAID_WINDOW_S:
            self.said.popleft()

    def at(self, mono):
        recent = [t for m, t in self.said if mono - m <= LABEL_SAID_WINDOW_S]
        return recent[-1] if recent else self.standing


def _lines(path):
    """The file's JSON lines, as far as they are whole."""
    try:
        with gzip.open(path, "rt", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    yield json.loads(line)
                except json.JSONDecodeError:
                    return
    except (EOFError, OSError, zlib.error):
        return


def _has_labels(path):
    for rec in _lines(path):
        if "mark" in rec and rec.get("label"):
            return True
    return False


def labels_in(path, replay=True, export_s=LABEL_EXPORT_S, turn_on=()):
    """One dict per label in a recording or clip: file, t, label, said (what the device had
    said, as recorded), then/now ({"instruction", "class"}; now only when replayed), and
    ticks: the last `export_s` seconds of ticks before the press, each {t, mono, in, out}."""
    if not _has_labels(path):
        return []
    if replay:
        from tools.field.walk_replay import replay_ticks

        stream = replay_ticks(path, turn_on=turn_on)
    else:
        stream = ((rec, None) for rec in _lines(path))
    then, now = _About(), _About()
    recent = collections.deque()  # (mono, tick) for the export
    found = []
    for rec, got in stream:
        mono = float(rec.get("mono", 0.0))
        if "mark" in rec:
            label = rec.get("label")
            if not label:
                continue
            said_then = then.at(mono)
            row = {
                "file": os.path.basename(path),
                "t": rec.get("t"),
                "label": label,
                "said": rec.get("said", ""),
                "then": {"instruction": said_then, "class": label_class(said_then)},
                "ticks": [tick for m, tick in recent if mono - m <= export_s],
            }
            if replay:
                said_now = now.at(mono)
                row["now"] = {"instruction": said_now, "class": label_class(said_now)}
            found.append(row)
            continue
        then.tick(mono, rec.get("out"))
        if got is not None:
            now.tick(mono, got)
        recent.append(
            (mono, {"t": rec.get("t"), "mono": mono, "in": rec.get("in"), "out": rec.get("out")})
        )
        while recent and mono - recent[0][0] > max(export_s, 0.0):
            recent.popleft()
    return found


def _files(paths):
    out = []
    for p in paths:
        if os.path.isdir(p):
            out += sorted(glob.glob(os.path.join(p, "*.jsonl.gz")))
        elif os.path.exists(p):
            out.append(p)
        else:
            print(f"  {p}: not there (nothing labelled or recorded yet?)")
    return out


def _dedupe(rows):
    """A label in a full recording and in its own clip is one label."""
    seen, out = set(), []
    for r in rows:
        key = (r["label"], round(float(r.get("t") or 0.0) * 2))
        if key not in seen:
            seen.add(key)
            out.append(r)
    return out


def report(rows, replay=True):
    """Print the scores; returns how many labels right then are wrong now."""
    from config import MARK_LABELS

    if not rows:
        print("No labels yet: press What was really there? on the dashboard while walking.")
        return 0
    cols = list(MARK_LABELS) + ["ground", "other"]
    by = collections.defaultdict(list)
    for r in rows:
        by[r["label"]].append(r)

    def pct(k, n):
        return f"{k:3d} ({100.0 * k / n:3.0f}%)" if n else "   —     "

    print(f"\n{len(rows)} labels\n")
    print(f"  {'really there':<13} {'n':>4}   {'right then':<11}" + ("  right now" if replay else ""))
    for label in MARK_LABELS:
        rs = by.get(label, [])
        then_ok = sum(r["then"]["class"] == label for r in rs)
        line = f"  {label:<13} {len(rs):>4}   {pct(then_ok, len(rs)):<11}"
        if replay:
            line += "  " + pct(sum(r["now"]["class"] == label for r in rs), len(rs))
        print(line)
    then_all = sum(r["then"]["class"] == r["label"] for r in rows)
    line = f"  {'all':<13} {len(rows):>4}   {pct(then_all, len(rows)):<11}"
    if replay:
        line += "  " + pct(sum(r["now"]["class"] == r["label"] for r in rows), len(rows))
    print(line)

    side = "now" if replay else "then"
    print(f"\n  What it named {side} (rows: really there)")
    print("  " + " " * 13 + "".join(f"{c:>9}" for c in cols))
    for label in MARK_LABELS:
        counts = collections.Counter(r[side]["class"] for r in by.get(label, []))
        print(f"  {label:<13}" + "".join(f"{counts.get(c, 0):>9}" for c in cols))

    broken = [r for r in rows if replay and r["then"]["class"] == r["label"] != r["now"]["class"]]
    fixed = [r for r in rows if replay and r["then"]["class"] != r["label"] == r["now"]["class"]]
    wrong = [r for r in rows if r[side]["class"] != r["label"]]

    def show(title, rs):
        if not rs:
            return
        print(f"\n  {title}")
        for r in rs:
            said = r[side]["instruction"] or "—"
            extra = (
                f"   (then: {r['then']['instruction'] or '—'})"
                if replay and r["then"]["instruction"] != r[side]["instruction"]
                else ""
            )
            print(f"    {r['file']}  really {r['label']}, named {r[side]['class']}: “{said}”{extra}")

    show("BROKEN — right then, wrong now:", broken)
    show("Fixed — wrong then, right now:", fixed)
    show(f"Still wrong {side}:", [r for r in wrong if r not in broken])
    return len(broken)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "paths",
        nargs="*",
        default=[str(ROOT / "logs" / "walks")],
        help="clips, recordings or folders of them (default: logs/walks)",
    )
    ap.add_argument("--export", metavar="FILE", help="write the labels as JSON lines here")
    ap.add_argument(
        "--recorded", action="store_true", help="no replay: what was said then only"
    )
    ap.add_argument(
        "--turn-on",
        default="",
        metavar="FEATURES",
        help="replay as if these not-yet-spoken features were spoken (tools/field/walk_replay.py's: "
        "tof-wall, tof-hazards, ...): would the labels come out righter?",
    )
    ap.add_argument(
        "--seconds",
        type=float,
        default=LABEL_EXPORT_S,
        help=f"ticks exported before each press (default {LABEL_EXPORT_S})",
    )
    args = ap.parse_args()
    try:  # without it the replay quietly falls back, and scores labels wrong
        import numpy  # noqa: F401
    except ImportError:
        print("label_report needs numpy: pip install numpy (conda has it already)")
        return 2
    from tools.field.walk_replay import FEATURES

    turn_on = [f.strip() for f in args.turn_on.split(",") if f.strip()]
    unknown = [f for f in turn_on if f not in FEATURES]
    if unknown:
        ap.error(f"unknown feature(s) {', '.join(unknown)}: use {', '.join(FEATURES)}")
    files = _files(args.paths)
    rows = []
    for path in files:
        try:
            rows += labels_in(
                path, replay=not args.recorded, export_s=args.seconds, turn_on=turn_on
            )
        except Exception as e:  # one bad file must not hide the rest
            print(f"  {path}: could not read — {type(e).__name__}: {e}")
    rows = _dedupe(rows)
    print(f"{len(files)} files read")
    broken = report(rows, replay=not args.recorded)
    if args.export:
        with open(args.export, "w", encoding="utf-8") as fh:
            for r in rows:
                fh.write(json.dumps(r, separators=(",", ":")) + "\n")
        print(f"\n  {len(rows)} labels written to {args.export}")
    return 1 if broken else 0


if __name__ == "__main__":
    sys.exit(main())
