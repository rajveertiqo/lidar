# Shappe Guide

A wearable navigation aid for blind pedestrians, running on a Raspberry Pi 5.

It watches the way ahead with sensors that fail in different ways, merges what they see, and
says one short sentence from a closed vocabulary: `"Walk straight"`, `"Wall ahead"`,
`"Blocked by glass, turn right"`. One voice, never two talking over each other, through an
earpiece on the Pi or the phone over Bluetooth LE.

> **Status: a prototype in field testing.** It is meant to be used alongside a white cane,
> never instead of one. What it can't do yet is listed in [Known limits](docs/known-limits.md).

## How it works

1. **Sense.** A LiDAR on the shoulder (one 360° plane), six radar boards (they hear glass the
   laser sees through), four cameras with an NPU (they name what is there) and a ToF depth
   camera (the ground ahead: steps, drops, walls).
2. **Decide.** `app/perception/walk_policy.py` fuses them into the room in each direction and
   the walk decision: clear, blocked, and the way out.
3. **Say.** `app/guidance/simple_guidance.py` turns that into one sentence; the speech gate
   decides when it is said: hazards at once, "clear" only once it holds.
4. **Speak.** `app/guidance/pi_voice.py` speaks it on the Pi (espeak-ng), or the phone does
   (`ble_bridge/`) when no earpiece is connected.

Every sentence, and when it is said: [What the user hears](docs/what-the-user-hears.md).

## Hardware

| Part                      | Role                                                                 |
| ------------------------- | -------------------------------------------------------------------- |
| Raspberry Pi 5            | Host                                                                 |
| Hailo-10H NPU             | Runs the object detector and the segmentation model                  |
| RPLIDAR A2M12             | 360° distances, ~23 scans/s                                          |
| UC-512 kit (4× IMX519)    | Four cameras in one 2×2 picture, 66° each, ~16 pictures/s            |
| 6× Acconeer A121 (60 GHz) | Radar every 60°, on a USB hub: hears glass                           |
| MPU-6500 IMU (SPI0)       | Yaw rate, gait and tilt: heading, not position                      |
| Arducam ToF (CAM/DISP 0)  | 240×180 distances to 4 m, ~30 frames/s: the ground ahead and walls   |

## Quick start

A new Pi needs drivers, boot settings and apt packages first: [docs/setup.md](docs/setup.md).

```bash
# --system-site-packages is required: picamera2 and the Hailo runtime come from apt.
python3 -m venv --system-site-packages venv
./venv/bin/pip install -r requirements.txt

./start_lidar.sh                # checks the devices, then starts the app
./start_lidar.sh --background   # for a walk: keeps running when the terminal closes
```

Then open `http://<pi-ip>:8000` on the phone: the field-test dashboard, with what was said, the
sensors, the cameras and the label buttons ([docs/dashboard.md](docs/dashboard.md)).

- Every run is logged to `logs/run_<date>_<time>.log` (newest: `logs/latest.log`);
  `grep WARNING logs/latest.log` shows only the problems. `SHAPPE_LOG_LEVEL=DEBUG` logs more.
- `http://<pi-ip>:8000/guidance/log/text` lists everything said this run, as plain text.
- To start at boot as a service: [docs/running.md](docs/running.md).

## Project layout

```
app/                    The guidance service (docs/architecture.md lists every module)
  main.py                 The FastAPI app: starts the sensors, the voice and the API
  config.py               Every threshold, in one place
  sensors/                LiDAR, radar, cameras, ToF, NPU and IMU drivers
  perception/             What is around: the walk decision, sensor fusion, the ToF's
                          hazards and walls, drift, movers, the route planner
  guidance/               What is said: the sentence (simple_guidance), the vocabulary,
                          the speech gate, the voice
  core/                   Shared state, freshness, health, logging, the walk recorder
  api/                    The HTTP API (docs/api.md)
templates/              The web pages: the dashboard at /, /tof/live, the radar pages
model/                  The Hailo models (.hef)
ble_bridge/             The phone link over Bluetooth LE (its own service)
deploy/                 The systemd service and its installer
tests/                  Unit tests, hardware-free
tools/sim/              Desk simulators: the real code on synthetic sensor data
tools/field/            Test walks: replay, label scoring, logs
tools/device/           Checks and calibration run against the real device
docs/                   The documentation (below)
```

## Development

```bash
./venv/bin/pip install -r requirements-dev.txt   # ruff, mypy, jsonschema (not needed to run it)

./run_tests.sh            # every test, hardware-free, ~40 s: before every push
./run_tests.sh --full     # and the desk simulators and the scene hunt, ~7 min
./venv/bin/ruff check .   # lint
./venv/bin/mypy           # types (a report for now)
```

CI runs the lint, the tests and the type report on every push
(`.github/workflows/tests.yml`). Settings are in `pyproject.toml`, the version in
`app/config.py` (`APP_VERSION`), and what changed in [CHANGELOG.md](CHANGELOG.md).

**Changing what it says.** Run `./run_tests.sh --full` before and after: the simulators and the
scene hunt must not get worse. On real walks, testers press the dashboard's label buttons
("what was really there"), and `tools/field/label_report.py` scores the current code against
every label ([Field testing](docs/field-testing.md)). A new kind of warning (stairs and drops,
moving people, the route planner) is logged as "would say" and stays silent until a walk has
checked it (`*_SPEAK` in `app/config.py`). Models are trained on a laptop from exported labels,
never on the Pi.

## Documentation

| Document | What it covers |
| --- | --- |
| [What the user hears](docs/what-the-user-hears.md) | Every sentence, and when it is said |
| [The safety model](docs/safety-model.md) | Why several sensors, how each is trusted, drift, movers |
| [Architecture](docs/architecture.md) | Every module, the threads, measured timings |
| [Running it](docs/running.md) | Starting, the service, logs, the radar array, the ToF, ground hazards, walls, models |
| [Setting up a new Pi](docs/setup.md) | Drivers, boot config, packages, hardware checks |
| [Field testing](docs/field-testing.md) | Test walks, timings, the label buttons, replaying walks at the desk |
| [Dashboard](docs/dashboard.md) | The pages and what each panel shows |
| [API](docs/api.md) | The HTTP endpoints |
| [Configuration](docs/configuration.md) | The settings in `app/config.py` that matter most |
| [Design notes](docs/design-notes.md) | Bugs found and fixed, and why the code is the way it is |
| [Known limits](docs/known-limits.md) | What it cannot do yet |
| [The phone link](ble_bridge/README.md) | The BLE bridge and its protocol |
