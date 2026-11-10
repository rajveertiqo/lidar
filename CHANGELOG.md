# Changelog

What changed in each release of Shappe Guide. The version is `APP_VERSION` in `app/config.py`
(the API reports it). Why each change was made is in
[docs/design-notes.md](docs/design-notes.md); every commit is in `git log`.

## [Unreleased]

### Added
- **Stairs** and **Kerb / step** label buttons on the dashboard (keys `S`, `K`), so a walk can
  score the ToF's stair, kerb and drop warnings before they are spoken
  (`label_report.py --turn-on tof-hazards`). Walk recordings now keep what those warnings
  would have said (`ground_shadow`).
- **The camera's light, in lux**, from the sensor's own reading, for each camera: on
  `/camera/stats` and in every walk recording. Under `CAMERA_DIM_LUX` (45) `/health` warns
  "Camera dim", and `label_report.py` scores labels in good light and dim light apart.
  Nothing said changes.

### Fixed
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
- **The app is one package, `app/`**, split by job: `app/sensors/` (hardware), `app/perception/`
  (what is around, the walk decision), `app/guidance/` (what is said), `app/core/` (shared
  state, freshness, recording), `app/api/` (was `routers/`). `main.py` and `config.py` moved
  into it: settings are now in **`app/config.py`**. Module names are unchanged.
- It starts as `uvicorn app.main:app` (`./start_lidar.sh` does this; nothing to change in how
  you start it or in the service). The radar check is `python -m app.sensors.radar_utils`.

### Removed
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
