# Changelog

What changed in each release of Shappe Guide. The version is `APP_VERSION` in `app/config.py`
(the API reports it). Why each change was made is in
[docs/design-notes.md](docs/design-notes.md); every commit is in `git log`.

## [Unreleased]

### Added
- **`label_report.py --decide`**: one command after a walk says, for each feature that runs but
  is not spoken yet (stairs and kerbs, glass tracking, movers), TURN ON, KEEP OFF or MORE
  LABELS. Every label is replayed with the feature off and on: turned on it must name 70% of
  its own labels, more than without it, and make at most 5% of the other right labels wrong.
  **The battery walk** in [docs/field-testing.md](docs/field-testing.md) collects enough labels
  for all three in about 45 minutes, and checks the heading.
- **Radar tracking for glass across a narrow corridor** (silent, "would say", until a walk
  checks it: `GLASS_CLOSING_SPEAK`). A pane's echo closes at walking pace on both front
  boards while the walls' echoes stay put, so a pane past the walls' echoes is found. In the
  simulator it blocks panes that today get no block at all.
- **Stairs** and **Kerb / step** label buttons on the dashboard (keys `S`, `K`), so a walk can
  score the ToF's stair, kerb and drop warnings before they are spoken
  (`label_report.py --turn-on tof-hazards`). Walk recordings now keep what those warnings
  would have said (`ground_shadow`).
- **The camera's light, in lux**, from the sensor's own reading, for each camera: on
  `/camera/stats` and in every walk recording. Under `CAMERA_DIM_LUX` (45) `/health` warns
  "Camera dim", and `label_report.py` scores labels in good light and dim light apart.
  Nothing said changes.

### Fixed
- **A glass door with someone standing near it is found.** With a person beside the door,
  in front of its wall, it said "Walk straight" in about 1 in 4 simulated scenes, and
  walking in, 14 of 80 approaches were not warned by 4 ft: the person was taken for what
  the radar heard, and mixed into the wall the door is set in. Now the wall is found with
  them left out, and every approach is warned by 4 ft (design note 25).
- **A glass door met at an angle is found.** Met 15-20 deg off, a glass door said "Walk
  straight" from 3 to 6 ft away: the tilted wall around it was taken for a wall beside the
  wearer. It is now "Glass ahead", then "Blocked by glass" at the glass, from 11 ft up to
  20 deg off and from 6 ft up to 45 deg off (design note 24). A plain tilted wall is still a
  wall.
- **No "Glass ahead" between two posts.** Two posts either side of an open way read as the
  wall a glass door is set in, and it said "Glass ahead", or "Blocked by glass, turn right",
  with no glass there. A door's pane lies on its wall's line, where both front boards hear
  it; two posts are heard each at its own range (design note 23). The scene hunt's false
  glass fell from 32 to 22 in 3,000 scenes; no pane is missed that was found before.
- **A pillar or post on the path is no longer called "the wall beside you".** "Near right wall,
  move left" was said with a pillar standing out of the wall 2.5-2.9 ft ahead on the path, and
  once with a post on the path's left half at 2.4 ft, which "move left" walked toward. A block
  is put down to the wall beside the wearer only when nothing else is on the path within 4 ft
  (design note 22). The scene hunt's block rules now all hold; it measures a post with only
  its edge in the walking strip at its nearest point there.
- **Facing a close wall, an open left is offered** ("Blocked by wall, turn left"), where it said
  "turn around". The left is the LiDAR's blind side, so the left board's echo is taken for the
  wall ahead only when the laser sees that wall stretch out past it, the echo is at least 2 ft
  off, and the left camera's fresh picture shows nobody (design note 21). Without the left
  camera, or nearer than ~1.1 ft to the wall, it is "turn around" as before.
- **No false "Pothole ahead" when the device is set down.** On a desk the ToF took the floor
  below it for the ground and found a hole. Set down, the ground is no longer judged, and a
  ground must lie at the mount's height (1.45 m +- 0.45 m), as the settings already said.
- **A pillar standing out of a wall now blocks.** It was "Near right wall, move left", with
  its edge 0.1 m off the walking line. A block is the wall beside you only if what reaches
  furthest into your path lies on a wall running along it (`SIMPLE_ALONGSIDE_MAX_DEG`).
- **A block warning no longer waits behind a status notice.** A post appearing while
  "Power low" (or "Glass sensor off", "Camera on", ...) was being said was warned of 1.1 s
  late; now 0.2 s, as with nothing playing. The notice it cut is said again once the path is
  clear.
- "Stairs down in front, stop" and "Drop in front, stop" now cut in as urgent, with the
  urgent tone, once the ToF's ground warnings are spoken.
- **A glass pane across a narrow passage** (no frame the laser sees) is now found once it
  is nearer than the passage's walls: a 1.8 m passage says "Blocked by glass" at 2.6 ft, where
  it said "Walk straight". A side wall can no longer "explain" an echo nearer than itself
  (`GLASS_SIDE_WALL_NEAREST_TOL_MM`). Farther panes in narrow passages are still a known
  limit.
- **Facing a wall close up, an open right is offered.** A flat wall 1.0-1.5 ft ahead in
  open space said "Blocked by wall, turn around", and an inside corner "Stop, no clear way":
  the side radar heard the wall ahead at its beam's edge. Both now say "turn right". The left
  (the LiDAR's blind side) is unchanged.

### Changed
- **The dashboard redrawn** (`templates/dashboard.html`): a new look in dark and light, the
  last sentence said as the headline, coloured by its rule. On a phone the Right / Wrong /
  Missed / Note buttons are pinned to the bottom of the screen, and what a press saved shows
  above them. The label buttons have their own card; the run counts head "Spoken this run";
  the camera key folds away; the radar boards are in order of angle. The laser view has the
  radar circle the live map had: each board's beam out to its nearest echo, and the radar's
  reach (from `/radar/map`, as the radar scope). Same keys; the theme saved by the old page
  carries over.
- **The dashboard's big sentence is the last one SAID.** It showed the sentence the guide was
  still weighing, which flickers: the screen said "Blocked by wall, turn left" while the wearer
  had heard "turn right". The card shows that sentence and nothing else: the reason line, the
  live sensor chips ("wall ✗" under "Blocked by wall" read as a contradiction), "said 19 s
  ago" and "quiet — unconfirmed" are gone (the terminal has them). "Last said" could also show
  a stale sentence after the app restarted; it now starts again with the app.
- **The dashboard's "Spoken this run" lists only what was said**, as the terminal's 🔊 lines:
  the reason, the LiDAR scan's age and which voice said it. The silent "would say" lines (route
  planner, ToF hazards, movers) and the tester's marks had been listed with them, and read as
  said; they stay in the terminal and `/guidance/log/text`.
- **Stricter lint** (`pyproject.toml`): statement errors (E7) and bugbear (B) now block CI
  too. What they found, fixed: two bare `except:` that would also swallow Ctrl+C and
  shutdown (`lidar_utils`, `unified_navigation_system`), and four errors in the phone bridge
  re-raised without their cause (`raise ... from exc`).
- **`simple_guidance.py` split** (4,430 -> 3,690 lines): the closed vocabulary is now
  `app/guidance/vocabulary.py`, and the speech gate and sensor notices
  `app/guidance/speech_gate.py`. `simple_guidance` re-exports every name, so nothing that
  imports it changes; the tests and every simulator give the same results.
- **Sharper camera pictures while walking:** the exposure is kept to 10 ms while gain can make
  up the light (`CAMERA_SHUTTER_CAP_MS`); the sensor chose 24-30 ms indoors, which walking
  blurs. Dim light gets the same most exposure as before.
- **The app logs through `logging`** (`app/core/logs.py`), not `print()`: every line has its
  time, its level from WARNING up, and its module, and `SHAPPE_LOG_LEVEL` sets how much is
  written. Same place as before (the terminal and `logs/run_*.log`). The live guidance console,
  the start-up banner and the hand-run hardware checks still print.
- **The app is one package, `app/`**, split by job: `app/sensors/` (hardware), `app/perception/`
  (what is around, the walk decision), `app/guidance/` (what is said), `app/core/` (shared
  state, freshness, recording), `app/api/` (was `routers/`). `main.py` and `config.py` moved
  into it: settings are now in **`app/config.py`**. Module names are unchanged.
- It starts as `uvicorn app.main:app` (`./start_lidar.sh` does this; nothing to change in how
  you start it or in the service). The radar check is `python -m app.sensors.radar_utils`.

### Removed
- **The live map** (`/map`, the 3D test page): its page, engine and routes, and the dashboard's
  link to it. Display only; nothing the device says used it. The radar scope (`/radar/view`)
  stays.
- **The gyro (MPU-6500) and all its code**: unplugged since 2026-10-23. Its reader and
  driver, the stride learned from footfalls (`/guidance/gyro`, `/guidance/gait/*`, the gait
  test page), the gyro half of the heading tracker, the live map's gyro/LiDAR choice
  (`/map/settings`), `drift_sim --gyro`, `spidev` and the `GYRO_*` settings. Heading comes
  from the LiDAR alone, as it already did; nothing the device says changed.
- `shappe_guide.egg-info/`, build metadata committed by mistake (now ignored).
- The previous dashboard, `/classic` (`templates/index.html`), and its `/test` alias: the
  dashboard at `/` has everything used in testing.
- Fog / rain detection (`environment_detection`, `/environment/*`): switched off since it
  called a lit office "fog", and so never consulted.
- The road-crossing state machine (`traffic_light_detection`, `/crossing/*`): nothing on the
  device ever fed it. "Traffic light ahead" is unchanged (it comes from
  `unified_navigation_system`).
- Unused routes: `/camera/video_feed`, `/camera/stream/{sector}`, `/camera/seg_grid`,
  `/camera/frame`, `/camera/frames/all`, and the `/guidance/ws` websocket. Unused settings
  `LIDAR_TO_CAMERA`, `STREAM_FPS`, `TRAFFIC_SIGN_STALE_S`, `ENV_*`.

## [3.1.0] - 2026-11-10

### Added
- **"What was really there?" label buttons** on the dashboard (Wall, Glass, Obstacle,
  Person, Nothing; keys W G O P N). Each press saves a clip of the sensor data with the
  label. `tools/field/label_report.py` scores the code against the labels and exports them
  for training on a laptop.
- **The ToF names walls.** A flat, wide, upright surface across the path turns
  "Obstacle in front, turn left" into "Blocked by wall, turn left" (`TOF_WALL_SPEAK`). It
  renames only: the block and the turn are unchanged. `/tof/live` shows the verdict live.
- **Set down on a desk**, the ToF finds the surface the device rests on, and the radar's
  echo off it no longer blocks ("Obstacle in front" at 0.9 ft with a wall 4 ft away).
- Replayable clips on every Wrong or Missed press (`logs/walks/clip-*.jsonl.gz`).
- CI on every push (lint, tests, a type report), `pyproject.toml`, `requirements-dev.txt`,
  `.editorconfig`, this changelog.

### Changed
- Project layout: tests in `tests/`; tools in `tools/sim/`, `tools/field/`, `tools/device/`;
  the README split into `docs/`.
- A reading's freshness goes by the monotonic clock: a wall clock stepped by NTP mid-walk no
  longer says "Guidance stopped, use your cane".
- `tools/field/walk_replay.py` gives the decision code the ToF analysis recorded with each
  tick, so ToF-dependent sentences replay as they were said.

### Fixed
- `scipy` was missing from `requirements.txt` (the ToF's obstacle analysis needs it).
- The ToF's "goes on above its view" test never held for a wall filling the picture.

### Removed
- `tools/odometry_density.py` (unused), `pyvenv.cfg` (a virtualenv leftover).

## [3.0.0] and earlier

Before this changelog. See `git log` and [docs/design-notes.md](docs/design-notes.md).
