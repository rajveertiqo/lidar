# Field testing

`tools/field/walk_logger.py` records a walking test. Start the app, then:

```bash
./venv/bin/python tools/field/walk_logger.py     # Ctrl+C to stop and print a summary
```

It polls localhost twice a second and writes `logs/walk_<time>.csv` with the
guidance action/cue, the walk policy's own decision (blocked, chosen turn,
**glass ahead**, and its reason string), per-direction clearance in metres,
per-direction radar distance plus how many boards were online, the capture /
detection / segmentation FPS, all 8 LiDAR sectors, and CPU temp + throttle flag.

Press ENTER to drop a timestamped marker — type a note first (`glass door here`)
and it is recorded on the next row. That is how you line the CSV up against what
actually happened on the walk.

On exit it prints FPS min/avg, how often the path was blocked, how often glass
was flagged, whether any radar board dropped out, and a tally of each guidance
action. `logs/` is gitignored.

**How late was each sentence?** (2026-10-30) Every sentence the device says carries its
timing in the speech log (`/guidance/log`, `/guidance/log/text`, and `logs/speech_*.log` with
`SPEECH_LOG_TO_FILE`): how old the LiDAR scan it was decided on was (`scan 45 ms old`, captured
→ said), the camera frame's when the camera named it, the tick's own time, how soon the Pi
voice started it (`voice +170 ms` — the urgent tone included) and the front range it was said
at (`front 3.6 ft`: a block should come near `WALK_BLOCK_FT`, 4 ft). The test console shows
`⏱ scan 45 ms` on each line. After a walk, `curl <pi>:8000/guidance/latency` gives the medians
and worst cases, urgent sentences apart. The audio path after that (PipeWire, a Bluetooth
earpiece waking) is not in these figures: time it once by ear against the console.

**The urgent tone.** "Blocked…", "Vehicle approaching…", "Guidance stopped…" and the falls
("Stairs down in front, stop", "Drop in front, stop") start with a
0.1 s tone (`PI_VOICE_URGENT_TONE`), on the Pi and on the dashboard's phone voice (after one
tap on the page), so the wearer can react before the words are understood.

## Labelling what was really there (added 2026-11-10)

The dashboard's **What was really there?** card: Wall, Glass,
Obstacle, Person, Nothing, Stairs, Kerb / step (keys `W G O P N S K`). Press the one that names the thing the device
just talked about, or what is ahead if it named nothing. Press it whether the device was
right or wrong; "Nothing" means nothing was there (a false alarm). "Stairs" is a flight up
or down; "Kerb / step" is a single step up or down, a kerb, or a drop. Each press saves a clip
of the sensor data around it, `logs/walks/clip-<time>-<label>.jsonl.gz`, with the label
and what the device had said. The card counts the labels saved so far. Labelled clips are
kept apart from Wrong/Missed ones (`WALK_LABEL_KEEP_FILES`, 3000), so they are never
pruned by Wrong presses. The labels are `MARK_LABELS` in `app/config.py`.

On a laptop (training is never done on the Pi), copy the clips across and score the current
code against them:

```bash
rsync -av shappe@<pi-address>:Documents/lidar/logs/walks/ ~/shappe-walks/
./venv/bin/python tools/field/label_report.py ~/shappe-walks                        # scores
./venv/bin/python tools/field/label_report.py ~/shappe-walks --export labels.jsonl  # for training
```

On Windows (PowerShell has no rsync), from a checkout of this repo, with any Python 3 that
has numpy (conda's base does):

```powershell
mkdir $HOME\shappe-walks -Force
scp "shappe@<pi-address>:Documents/lidar/logs/walks/*.jsonl.gz" $HOME\shappe-walks\
python tools\field\label_report.py $HOME\shappe-walks
```

The report replays every labelled clip through the current code. It prints how many of
each label the device got right then (when pressed) and gets right now, a table of what it
named against what was there, and which labels a change fixed or broke. It exits 1 when a
label that was right is now wrong, so run it before and after every threshold change. A
clip starts mid-walk without the device's held state, so run it once on unchanged code
first. `--export` writes one JSON line per label, with every tick's inputs from 2 s
(`--seconds`) before the press: the rows a classifier is trained on.

**Glass at the end of a narrow corridor** (added 2026-11-11). The radar tracking of a pane's
echo (design note 18) is silent: the terminal shows it as `👁 would say: Glass ahead` (or
"Blocked by glass") from `radar tracking`, with how fast the echo closed. Walk toward a glass
door down a narrow corridor, and down corridors with NO glass (with doors along them too):
press Glass or Nothing when it shows. Score it with
`label_report.py ~/shappe-walks --turn-on glass-closing`.

**Stairs and kerbs** (added 2026-11-10). The ToF's ground warnings ("Stairs down ahead",
"Step up ahead", "Drop in front, stop") are not spoken yet (`TOF_HAZARDS_SPEAK`), so the
device says nothing about them and these labels score as misses. Score what they would say:

```bash
./venv/bin/python tools/field/label_report.py ~/shappe-walks --turn-on tof-hazards
```

Its stairs and kerb rows say how often they would be right. Its other rows say what turning
them on would cost, such as a wall taken for stairs or a false "Drop ahead" on a flat floor
(press Nothing for those). Turn `TOF_HAZARDS_SPEAK` on only when both look right. A good walk
for this: stairs up and down, kerbs both ways, and some flat floor and walls in between.

## The battery walk: one walk decides the silent features (added 2026-11-13)

Three features run on the device but stay silent until a walk shows they are right:

| Feature | Setting | Labels it is judged on |
| --- | --- | --- |
| Stairs, kerb and drop warnings (ToF) | `TOF_HAZARDS_SPEAK` | Stairs, Kerb / step |
| Glass across a narrow corridor (radar tracking) | `GLASS_CLOSING_SPEAK` | Glass |
| People and vehicles coming at you (LiDAR) | `LIDAR_MOVERS_SPEAK` | Person |

One walk with enough labels decides all three, and checks the heading, which has never been
checked on a real walk.

**Before you start**

- Start with the walk recorded: `SHAPPE_RECORD_WALK=1 ./start_lidar.sh`. The labels save their
  clips either way; the recording also lets `walk_replay.py` show the whole walk.
- The earpiece connected, the dashboard open on the tester's phone, the wearer with a cane.
- Two people, the wearer and the tester just behind with the phone; a third, the helper, for
  the people part.

**When to press.** Press what is really there about 1.5-2 m before it, **whether or not the
device said anything**: the silent features speak only in the terminal, so do not wait for a
sentence. On open floor with nothing ahead, press Nothing now and then.

**The route** (about 45 minutes)

| Part | Do | Press | Aim for |
| --- | --- | --- | --- |
| Stairs and kerbs | Walk up to a flight of stairs: going up 5 times, going down 5 times. Then kerbs or single steps: up 5, down 5 | Stairs, Kerb / step | 20 |
| Glass | Down a narrow corridor (under about 2 m wide) toward a glass door, 10 times. Down corridors with no glass toward a wall or a wooden door, 5 times | Glass; Wall | 10 + 5 |
| People | The helper walks at the wearer head-on 5 times, crosses in front 5 times, comes up from behind 3 times, and stands still in the path 3 times | Person | 16 |
| Everything else | Walls, open floor, chairs, poles along the way | Wall, Nothing, Obstacle | 25 or more |

Each feature needs at least 10 labels of its own and 20 others before it is judged
(`DECIDE_MIN_TARGET`, `DECIDE_MIN_REST` in `tools/field/label_report.py`).

**The heading check** (at the start and again at the end)

1. Stand still facing a wall. Open `http://<pi-ip>:8000/guidance/heading` on the phone and note
   `heading_deg`.
2. Turn on the spot a quarter turn right and refresh: it should have changed by about 90.
3. Turn back: within 10 of the first reading. A full turn: back within 15.
4. Walk a straight corridor for 20 m, centred: it should not say "Drifting toward ... wall".

Press Note (key 4) and type the readings, so they are in the speech log with the walk.

**After the walk**

```bash
rsync -av shappe@<pi-address>:Documents/lidar/logs/walks/ ~/shappe-walks/
./venv/bin/python tools/field/label_report.py ~/shappe-walks --decide
```

(Windows: copy with `scp` as above, then `python tools\field\label_report.py $HOME\shappe-walks --decide`.)

It replays every label with each feature off, then on, and prints one verdict per feature:

- **TURN ON**: turned on, it names at least 70% of its own labels, more than without it, and
  makes at most 5% of the other right labels wrong. It names the setting to change in
  `app/config.py`.
- **KEEP OFF**: it misses too many, adds nothing, or costs too much; the labels it would get
  wrong are listed (replay one with `walk_replay.py --turn-on <feature> <clip>`).
- **MORE LABELS**: how many more of which labels the next walk needs.

Then `label_report.py ~/shappe-walks` on its own gives the overall score.

## Walking the drift layer without leaving the desk

```bash
./venv/bin/python tools/sim/drift_sim.py            # 3 deg/s veer: warns 4.1 s out
./venv/bin/python tools/sim/drift_sim.py --veer 0   # control: no VEER cue
./venv/bin/python tools/sim/drift_sim.py --accuracy # odometry error vs known motion
```

The control run is silent about veering, which is the point of it — but it is
not silent. It still says "Wall straight ahead" three times, because a 20 s walk
at 1.25 m/s reaches the end of the simulated corridor and that is a real
collision course, correctly reported by the other drift layer. Read the control
run for the absence of a *veer* cue, not for the absence of output.
