# Changelog

What changed in each release of Shappe Guide. The version is `APP_VERSION` in `app/config.py`
(the API reports it). Why each change was made is in
[docs/design-notes.md](docs/design-notes.md); every commit is in `git log`.

## [Unreleased]

### Changed
- **The app is one package, `app/`**, split by job: `app/sensors/` (hardware), `app/perception/`
  (what is around, the walk decision), `app/guidance/` (what is said), `app/core/` (shared
  state, freshness, recording), `app/api/` (was `routers/`). `main.py` and `config.py` moved
  into it: settings are now in **`app/config.py`**. Module names are unchanged.
- It starts as `uvicorn app.main:app` (`./start_lidar.sh` does this; nothing to change in how
  you start it or in the service). The radar check is `python -m app.sensors.radar_utils`.

### Removed
- `shappe_guide.egg-info/`, build metadata committed by mistake (now ignored).

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
