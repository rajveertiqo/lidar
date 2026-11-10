# Running it

```bash
# One-time setup. --system-site-packages is REQUIRED: hailo_platform and
# picamera2 are apt-installed and are not on PyPI.
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -r requirements.txt

./start_lidar.sh          # http://<pi-ip>:8000

./run_tests.sh            # before a push: every test, hardware-free, ~30 s
./run_tests.sh --full     # and the desk simulators and tools/sim/scene_hunt.py, ~5 min
```

**Every run is saved** in `logs/run_<date>_<time>.log` (newest: `logs/latest.log`), with
the same lines as the terminal and the colour codes stripped. That includes a run started
with `--background` or by the service, and the shutdown lines after Ctrl+C. The newest 100
runs are kept (`LOG_KEEP_RUNS=… ./start_lidar.sh` to change it). If `logs/` can't be
written, the app still starts, with a warning. `logs/` is not committed.

```bash
less -R logs/latest.log       # the last run
tail -f logs/latest.log       # follow the one running now
```

The tests include the web pages (`tests/test_templates.py`): every inline script must parse
(`node --check`, when node is installed) and no stray character may sit in a page's code —
an editor once dropped a "🔊" into the /map page's script and the 3D view silently died.

The app still STARTS on a machine with no LiDAR, camera or NPU: a missing
device publishes nothing and health reports it down. Only with
`SHAPPE_SIMULATE_CAMERA=1` (without `picamera2`) and `SHAPPE_SIMULATE_LIDAR=1`
(without `rplidar`) is synthetic data produced, and it is never evidence of
anything.

## On the device: a service that starts at boot

A wearable cannot depend on someone typing `./start_lidar.sh`, and a blind user
cannot see that it has stopped. `deploy/install-service.sh` installs it as a
systemd **user** service (user, so the voice reaches PipeWire and the earpiece):

```bash
./deploy/install-service.sh                 # install + enable; prints the admin token
systemctl --user start shappe-guide         # start now (stop any ./start_lidar.sh first)
journalctl --user-unit=shappe-guide -f      # its log
./deploy/install-service.sh --remove        # undo
```

- **Starts at boot** with nobody logged in (the installer enables lingering).
- **Restarts** on a crash, and on a **hang**: the watchdog is pinged only while
  the guidance loop and the web server's event loop are both alive
  (`app/core/service.py`). A missing sensor does not restart it; the app already
  recovers from that and says "Sensors offline".
- **Never wedges on the way out.** A shutdown that has not finished in
  `SHUTDOWN_TIMEOUT_S` ends with `os._exit`, so a stuck camera cannot keep
  holding port 8000 and every serial port for the next start.

`GET /health` says whether the device can guide right now: `ok` / `degraded`
(guiding on the LiDAR, something else down) with HTTP 200, or `down` (no
LiDAR, or the guidance loop is not guiding) with HTTP 503.

**Who may change the device.** `/radar/calibrate`, `/radar/rediscover` and
`/guidance/gait/stride` change how it steers, so they need `SHAPPE_ADMIN_TOKEN`
in an `X-Admin-Token` header (the installer and `start_lidar.sh` write one to
`~/.config/shappe-guide.env`; the test pages ask for it once per browser). With
NO token configured (uvicorn or `python app/main.py` by hand) they are refused with
503 — they used to be left open. Any state-changing request or WebSocket
handshake whose `Origin` is another site is refused (403 / close 1008), whatever
the token; and one that carries an Origin must reach the device by an IP
address, `localhost`, its hostname or `hostname.local` — or a name listed in
`SHAPPE_ALLOWED_HOSTS` — so a DNS-rebinding page cannot pass as same-site.
Every response forbids framing, and cross-origin reads are refused unless
listed in `SHAPPE_CORS_ORIGINS`.

Hardware check: the LiDAR (by its `/dev/serial/by-id/` name, falling back to
`/dev/ttyUSB0` — `config.LIDAR_PORT_CANDIDATES`), `/dev/hailo0` (NPU), and the A121
boards on the USB hub. To find which radar board is physically where:

```bash
./venv/bin/python -m app.sensors.radar_utils --identify
```

## Six radar boards, four guidance directions

The array is **six** A121 boards every 60°, offset half a step so they
**straddle** the cardinal axes rather than sitting on them — bearings
30/90/150/210/270/330. Everything downstream still speaks in four directions,
and the six reduce to them two-and-one:

**Bearings were reassigned on 2026-09-04** — all six serials moved. The first
table was converted from the rig's `positions.json`, and the mount it described
was not the mount on the device; the current table in `config.RADAR_BEARINGS` is
the operator's own statement of which board covers which sector, which outranks
any file. The USB cabling did not change, so the boards were remounted rather
than rewired. The **grouping structure below is unchanged** by that: it depends
on the mount, not on which serial sits where.

Boards are known by the **sector** they cover (`0–60`, `60–120`, …), which is how
they are labelled on the device and in the operator's notes. A bearing is the
same fact in the form the geometry needs, `bearing = sector_start − 30`. That
conversion lives in one place, `config.radar_sector_label()`, and `/radar/map`
publishes `sector_label` per slot so no display re-derives it — a display that
did its own arithmetic is how a silent 30° error gets in.

| direction | fed by | boards | arc covered | centre |
|---|---|---|---|---|
| **front** | the two boards straddling ahead | `deg330`, `deg030` | 300°–60° | **0°** |
| **right** | the one board on the axis | `deg090` | 60°–120° | **90°** |
| **back** | the two boards straddling behind | `deg150`, `deg210` | 120°–240° | **180°** |
| **left** | the one board on the axis | `deg270` | 240°–300° | **270°** |

Every group is centred *exactly* on the LiDAR sector of the same name, which is
what makes the reduction safe: `sensor_fusion` injects a radar cardinal into the
LiDAR sector it genuinely overlaps, so a hazard is never announced in a
direction the board was not looking.

The grouping is **derived, not written down** — `cardinal_sources()` admits a
board when the angle to a cardinal's centre is under `RADAR_BEAM_H_DEG/2 + 22.5`
(= 55°). At 60° spacing that takes in a neighbour 30° away and rejects one 60°
away, which produces the table above on its own. Re-measure a bearing at
`/radar/test` and the mapping re-derives itself with no restart.

### Converting from the standalone rig

`/home/shappe/radar` records a board by the **start** of the 60° sector it
covers, and its `layout.py` sets `SECTOR_ZERO_OFFSET_DEG = -60` — so a board
labelled `0–60` there does not point at 0°, it points at the middle of its
sector, which the offset puts at `start − 30`. The one conversion between the
projects is therefore

```
bearing = (rig_sector_start - 30) mod 360
```

Skipping that offset is a silent **30° error** in the direction a glass door is
announced, so it is stated once in `config.RADAR_BEARINGS` and nowhere else.

### Coverage overlaps — but head-on flat glass is still unproven

The boards are mounted **rotated 90°**, so the A121's wide axis is the horizontal
one: **65° across**, 53° tall. At 60° spacing the six beams therefore **overlap by
5°** — there is no blind slit anywhere, and every bearing including dead ahead
sits inside at least one board's beam.

That closes the geometric question. It does not close the one that matters,
because that one was never about coverage. At 60 GHz a smooth pane is
**specular**, not diffuse: a monostatic board gets a strong return only near
*normal* incidence. A pane facing the user square-on has its normal at 0°, and
the two boards that cover 0° sit at 30° and 330° — so they illuminate it at 30°
incidence and its specular lobe leaves at 30° the other way, missing both
receivers. **Being inside a beam is not the same as getting an echo back.**

No rotation or regrouping of six boards at 60° spacing fixes that: with the
boards where they physically are, nothing sits on boresight. So how weak it is
remains a **measurement**. **FT-1.5b** (from the old `FIELD_TESTS.md`, since deleted) covers
it; until that is run, do not treat forward glass coverage as established.

## Radar hardware test — `/radar/test`

`/radar/view` asks *what is around the user, and can we steer on it?*
`/radar/test` asks the hardware question instead: **is every board present,
streaming, and keeping up?** One card per board, with the numbers that only
matter when something is wrong:

| Field | Why it is there |
|---|---|
| **measured Hz** | The rate actually achieved, not `RADAR_UPDATE_RATE` as requested. The real test of the USB link. |
| **lag** | How far behind real time the stream has fallen. Should sit at 0.000 s. |
| **reconnects** | Climbing here means an unstable cable or hub socket. A board that reconnects constantly looks perfectly healthy in any single reading — this counter is the only thing that shows it. |
| **strength / temp** | Signal quality, and a board running much hotter than its neighbours. |
| **other peaks** | Returns behind the nearest one. Glass in front of a wall shows as two. |
| **state** | `starting` / `calibrating` / `streaming` / `reconnecting` / `refused`. "Offline" and "not finished calibrating" look identical in a reading, and only one is worth chasing a cable for. |

It reads the **running app's own state**, which is the point. This started life
as a separate app on port 8010, and a serial port has exactly one owner — so
testing the boards meant stopping the guide, and whichever app started second
saw six dead radars while the hardware was fine. The boards can now be tested
while the guide is live, which is the only time the answer is worth much.

Wave a hand ~1.5 ft in front of one sensor: its distance drops and its bar
jumps. That is how a physical board is mapped to a serial — and the same page's
CALIBRATE panel is where that measurement is recorded, writing the one bearing
table the guidance layer actually steers on (see `RADAR_BEARINGS` in
`app/config.py` for why a bearing is measured and never inferred).

A board reports in one of three conditions, because they need different repairs:

- **discovered and streaming** — working.
- **discovered but refused** — the USB link is fine; the board has no measured
  bearing, so it is refused rather than pointed in a guessed direction.
  Calibrate it; do not touch the cable.
- **never answered** — a cable, a hub socket, or a hung module.

The header's last pill is the one that matters: **blind cardinals**. Six
streaming boards with no bearings leave every direction without radar, and that
reads as a perfectly healthy array everywhere else on the page.

## ToF depth camera — `/tof/live` (added 2026-11-03)

The Arducam ToF (240 × 180 distances, to 4 m, ~30 frames/s) sits in **CAM/DISP 0**;
the CamArray is in CAM/DISP 1. The ToF code written on 2026-10-13, before the sensor
was here, now runs it, with what `~/tof` (`test_tof.py`, `tof_live.py`) learned on the
real sensor:

- **Its own process.** The SDK holds Python's GIL while it waits for a frame: beside it
  a 1 ms timer thread ran once every 32 ms. In the app that would stall the LiDAR
  reader and the guidance loop, so `app/sensors/tof_camera.py` runs the SDK in a spawned
  child, as `tof_live.py` does, and frames come up a pipe. With it the same timer
  runs every 1.05 ms. A sensor that stops sending is reopened; a child that dies or
  says nothing for 30 s is restarted (`restarts` in `/tof/status`).
- **Found, not numbered.** The SDK opens `/dev/videoN`, and N moves with probe order
  once both ports carry a camera. The app uses the node media-ctl links to the
  `arducam-pivariety` sensor (`TOF_VIDEO_INDEX = None`); set it only to pin one.
- **One overlay per port.** `/boot/firmware/config.txt` carries exactly
  `dtoverlay=imx519,cam1` (the CamArray) and `dtoverlay=arducam-pivariety,cam0` (the ToF).
  A bare `dtoverlay=arducam-pivariety` means CAM1. It took the CamArray's port, so the
  IMX519 never became a camera, and I2C address 0x0c, so the lens driver failed
  (2026-11-05). `tools/device/tof_check.py` names the line to drop. At each camera open, libcamera
  tries every port and can't run the ToF, which the SDK reads directly. It used to print
  five ERROR lines (`arducam-pivariety_mono.json` not found, `Failed to register camera
  arducam-pivariety 10-000c: -22`). The app holds those back and prints one line instead:
  `[CAMERA] libcamera passed over the ToF's port … as expected`. Anything else libcamera
  says still comes through.
- **`/tof/live`** (the dashboard's *depth* chip): the live depth picture, red near to
  blue far, in feet. Tap or drag to read the distance under the crosshair (the median
  of the 3×3 around it); the confidence slider changes the picture only. Below it, once
  a second, the front camera's picture in one of two ways: *depth + distances*
  (`/tof/rgbd.jpg`, the default) or *dots* (`/tof/overlay.jpg`: every depth pixel drawn
  where its own distance places it — the calibration check). The range stays
  `TOF_RANGE_MM` (4 m): the 2 m mode reads anything past 2 m as near, so the page
  cannot switch it.
- **The front camera as a depth camera** (2026-11-06). The placed depth points also make
  a depth picture aligned to the camera's, as an RGB-D camera gives:
  `DepthFrame.depth_map()` in `app/perception/depth_fusion.py`, one distance per 4 × 4 block of
  camera pixels (`TOF_ALIGNED_CELL_PX`; a ToF pixel spans ~4.3), the nearest point
  winning where several land in one block, a one-block gap taking its nearer neighbour's.
  `/tof/rgbd.jpg` lays it over the picture with each front detection boxed and labelled
  with its measured distance ("no depth" past 4 m or above the ToF's view);
  `/tof/aligned.npy` hands scripts the newest one. It covers only where the two views
  overlap — the ToF looks 20° lower, so roughly the lower half of the picture — and it
  is placed by the design until the bench calibration below.
- **What the user hears does not change yet.** Each front detection gets a measured
  `depth` (the nearest surface in its box, `app/perception/depth_fusion.py`), shown beside the
  box-size distance (`tof_ft`). It replaces it only with `TOF_USE_DISTANCE`, after the
  bench calibration (`tools/device/tof_calibrate.py`: capture 15–25 checkerboard pairs with
  `POST /tof/calibrate/capture`, `solve`, then `POST /tof/calibrate/reload`). Until
  then `/health` says "Depth camera not calibrated".
- **One owner.** The app holds the sensor while `TOF_ENABLED`. Stop it before
  `tools/device/tof_check.py` (the hardware walk-through from `~/tof/test_tof.py`: boot
  config, driver, video node, SDK, frames) or `~/tof/tof_live.py`.

SDK: `./venv/bin/pip install --no-deps ArducamDepthCamera==0.1.24`. Its metadata asks
for numpy < 2, and a plain install swaps the app's numpy 2.2.4 for 1.26; it runs on
2.2.4 (checked 2026-11-03, 31 frames/s).

## Hazards on the ground — stairs, steps, drops, poles (added 2026-11-06)

The LiDAR is worn on the shoulder and measures one plane there, so stairs, kerbs, drops,
holes and everything below the shoulder — a chair, a bin, a bollard, a table — never
reach it. The ToF looks down at the ground ~1–4 m ahead, and `app/perception/ground_hazards.py`
reads it there, 10 times a second in a thread of its own (~15 ms each), so the guidance
tick pays nothing for it:

- **The ground is measured, not assumed.** Each frame a plane is fitted to the ToF's
  points (RANSAC: near-level, near `TOF_MOUNT_HEIGHT_M`, near the last one, most points
  within 3 cm), so the body's sway and the mount's error drop out. Every point gets its
  height above it; `/tof/hazards` reports the fitted height and tilt.
- **Along the walking corridor** (`TOF_CORRIDOR_HALF_M` either side) the ground's height
  every 5 cm ahead, cut where it jumps. A sharp rise of 7–25 cm from the floor is a
  **step up** (a kerb); two or more of a stair's rise and run, **stairs up**. A fall is a
  **step down**, **stairs down** (each tread past the edge shows only as a strip a few
  cm long — every level counts), a **drop** if deeper, a **hole** if it comes back up
  within a metre. A floor that ends in nothing at all is a drop the ToF cannot measure.
- **Things on the ground** — points 8 cm or more above it, joined in 5 cm cells: an
  **overhang** if its front is all above 50 cm (the cane passes under), a **pole** if
  narrow and rising out of the ToF's view, a **low obstacle** if no taller than 60 cm,
  otherwise an **obstacle**.
- **The camera confirms.** Each hazard is placed on the front camera's picture (the
  calibration above; until the bench one, the design's, searched 40 px around). A
  detection over it confirms and names it ("Bench ahead"); else the picture's own edges
  do: two level edges for stairs, one for a step, drop or hole, an upright edge for a
  pole, an outline for the rest. A dark, blurred or missing picture is *unseen*. The
  camera sees the ground only from ~2 m out (it looks 10° down, 26° either way), so its
  verdict is **kept for 3 s** while the hazard stays tracked: an edge confirmed at 2.5 m
  is still confirmed at 1.5 m, below the picture.
- **When it is said** (`speak_rule`). Seen in 3 of the last 5 analyses, in the path,
  within `TOF_HAZARD_WARN_M` (11 ft). Confirmed or unseen by the camera: said. The camera
  looking and seeing nothing there: only a *measured* fall of `TOF_FALL_ALONE_M` (6 in)
  or more is still said — a ToF artifact must not stop the user, and a real edge on a
  plain floor must not be missed. An unmeasured drop is said only with the camera's edge.
- **What the user hears.** `Stairs down ahead`, `Stairs up ahead`, `Step up ahead`,
  `Step down ahead`, `Drop ahead`, `Low obstacle ahead`, `Overhanging obstacle ahead`;
  within `TOF_HAZARD_STOP_M` (5 ft), urgent and with the tone: `Stairs down in front,
  stop` and `Drop in front, stop`. A pole, a pothole or a named thing use the sentences
  that were there (`Pole ahead`, `Pothole ahead`, `Bench ahead`; `Pole in front` when the
  laser blocks on it); an unnamed tall one is `Obstacle ahead`. A fall in front outranks
  everything but an approaching vehicle (a block's turn too); a fall ahead outranks a
  wall or glass ahead; the rest come after what the LiDAR names, ahead of its unnamed
  obstacle. Phrases 311–319 of the phone's registry (revision 7).
- **Spoken only with `TOF_HAZARDS_SPEAK`.** Until a test walk has judged them, as the
  movers were, they are logged as `would say: …` (speech log source `tof-hazards`, and
  the test console) and drawn on `/tof/rgbd.jpg`: red if it would be said, amber if not.
  `/tof/hazards` says why for each. `TOF_HAZARDS_ENABLED = False` stops the analysis.

Desk-tested on ray-traced scenes (`tools/sim/ground_hazard_sim.py`: stairs both ways,
kerbs, drops, a hole, poles, a table, ramps, walls, with sway, noise and dropped
pixels). **Not yet on real stairs.** Its limits:

- The ToF sees the ground from ~1.1 m ahead: nearer than that nothing is seen, so a
  hazard is said from further off or not at all.
- From further than ~2.1 m the ToF (1.45 m up) looks over each tread's lip of a standard
  flight going down: there it is a `Drop ahead`, and `Stairs down` from ~2.1 m in.
- A drop deeper than the ToF reaches (a platform's edge) is said only if the camera sees
  its edge — not in the dark.
- Above ~1.1–1.3 m (the top of the ToF's view, 7° below level) nothing is measured: a
  branch at head height is not seen. The camera sees it but cannot range it.
- The ToF is infrared: bright sunlight blinds it, and black or wet ground may return
  nothing (a floor that "ends" there is an unmeasured drop, so the camera must agree).

## Walls by the ToF (added 2026-11-10)

The LiDAR's wall test missed a window blind ("Obstacle in front, turn left"): a radar echo
off the desk below disagreed with the laser, so the test gave up. The ToF now judges the
nearest tall thing on the walking path: is it a wall?

- **How.** Seen from above, its points become a line across the view: one sample every 4 ToF
  columns, from the nearest surface in them, so the noise of hundreds of points averages
  out. The line grows left and right from straight ahead while each next sample stays within
  `TOF_WALL_FLAT_M` (5 cm) of the line so far. A corner of two walls is two such arms.
- **A wall** is flat for `TOF_WALL_MIN_WIDTH_M` (1.2 m), or across the whole view close up,
  and goes on above the view. It must hold in 2 of the last 3 frames at about the same
  distance. A chair, a bin, a person, a pillar, a box or a 0.9 m cupboard is not one. A
  wardrobe 1.2 m wide is: the same turn.
- **What changes.** An unnamed block (or "Obstacle ahead") becomes "Blocked by wall, turn
  …" (or "Wall ahead") when the ToF's wall stands within `TOF_WALL_MATCH_FT` of the laser's
  front range and the camera names nothing. The block, the turn and every distance stay the
  laser's and the walk policy's.
- **Spoken with `TOF_WALL_SPEAK`** (on since 2026-11-10; it renames only). Off, it is
  logged as `would say: …` instead (speech log source `tof-wall`). `/tof/live` shows the verdict live, with why.
  `tools/field/walk_replay.py --turn-on tof-wall` and `tools/field/label_report.py --turn-on tof-wall`
  show what walks and labels would get.
- **Cost:** 1–2 ms per ToF frame, on the ToF's own thread.
- **Limits.** It needs the floor in view (worn, not on a desk). A wall nearer than ~1 m with
  the body leaning forward leaves no floor to fit. Glass is invisible to it: glass stays the
  radar's.

Desk-tested on 19 ray-traced scenes, upright and leaning 6° forward or sideways: walls
near, far and at 40°, a room's corner, a corridor's end, a blind over a sill and a
curtain's folds are walls; a person, two people, a chair, a box, a cupboard, a pillar, a
table and an open corridor are not (57/57). With three times the range noise and 10% of
pixels dropped, 56/57: the one miss is the wall closer than 1 m with the body leaning.
`venv/bin/python -m tools.sim.ground_hazard_sim` runs them (`./run_tests.sh --full` too).

## Models

**The models ARE in git today** (49 MB). The plan was to keep them out — they
are compiled binaries git cannot diff or compress, so each retrain adds another
permanent copy to history, and the segmentation model was swapped twice in one
day during development — but no ignore rule was ever added. Moving them to Git
LFS, or out of the repo (`git rm --cached model/*.hef`), is still open.

What the app needs in `model/`:

| File                  | What it is                                           |
| --------------------- | ---------------------------------------------------- |
| `model/yolo26m.hef`   | Object detection (COCO-80)                           |
| `model/best.hef`      | Walkable-surface segmentation                        |
| `model/metadata.yaml` | Segmentation class names — **must match** `best.hef` |

A retrained model is rebuilt from its `.pt` with the Hailo conversion pipeline
(the `conversion.md` this used to link is not in the repo). `metadata.yaml`
must travel with `best.hef`, since the segmentation code reads the walkable class names
out of it at load — swap a model without swapping this and classes get
mislabelled.
