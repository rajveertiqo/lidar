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

**The urgent tone.** "Blocked…", "Vehicle approaching…" and "Guidance stopped…" start with a
0.1 s tone (`PI_VOICE_URGENT_TONE`), on the Pi and on the dashboard's phone voice (after one
tap on the page), so the wearer can react before the words are understood.

## Labelling what was really there (added 2026-11-10)

The dashboard's verdict card has a second row, **What was really there?**: Wall, Glass,
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

## Walking the drift layer without leaving the desk

```bash
./venv/bin/python tools/sim/drift_sim.py            # 3 deg/s veer: warns 4.3 s out
./venv/bin/python tools/sim/drift_sim.py --veer 0   # control: no VEER cue
./venv/bin/python tools/sim/drift_sim.py --accuracy # odometry error vs known motion
```

The control run is silent about veering, which is the point of it — but it is
not silent. It still says "Wall straight ahead" three times, because a 20 s walk
at 1.25 m/s reaches the end of the simulated corridor and that is a real
collision course, correctly reported by the other drift layer. Read the control
run for the absence of a *veer* cue, not for the absence of output.
