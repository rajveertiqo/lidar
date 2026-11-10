# Shappe Guide: a navigation aid for blind pedestrians

A wearable navigation aid for blind pedestrians, running on a Raspberry Pi 5.

It watches the world with sensors that fail in different ways (a LiDAR on the shoulder, six
radar boards, four cameras with an NPU, a ToF depth camera), merges them into one ranked
picture of what is around the user, and reduces that to one short spoken sentence:
`"Walk straight"`, `"Wall ahead"`, `"Chair in front"`. One voice, never two talking over each
other. It speaks on the Pi, or through the phone over Bluetooth LE.

## Hardware

| Part                      | Role                                                      |
| ------------------------- | --------------------------------------------------------- |
| Raspberry Pi 5            | Host                                                      |
| Hailo-10H NPU             | Runs both neural models (object detection + segmentation) |
| RPLIDAR A2M12             | 360° distance and direction, ~23 Hz at 1380 RPM (full)    |
| UC-512 kit (4× IMX519)    | Four camera directions in one 2×2 frame: each its full 66° view, 1164×874, ~16 pictures/s (new cameras, 2026-11-02) |
| 6× Acconeer A121 (60 GHz) | Radar, every 60° straddling the cardinals, on a USB hub  |
| MPU-6500 IMU (SPI0 CE0)   | Yaw rate, gait and tilt — heading, not position            |
| Arducam ToF (CAM/DISP 0)  | 240×180 distances to 4 m, ~30 frames/s: measures what the front camera names (since 2026-11-03; [below](docs/running.md#tof-depth-camera--toflive-added-2026-11-03)) |

## Quick start

```bash
# One-time setup (a new Pi: docs/setup.md). --system-site-packages is REQUIRED:
# hailo_platform and picamera2 are apt-installed and are not on PyPI.
python3 -m venv --system-site-packages venv
./venv/bin/pip install -r requirements.txt

./start_lidar.sh              # the app: open http://<pi-ip>:8000 on the phone
./start_lidar.sh --background # for a walk: keeps running when the terminal closes

./run_tests.sh                # before a push: every test, hardware-free, ~40 s
./run_tests.sh --full         # and the desk simulators and the scene hunt, ~7 min
```

Every run is logged to `logs/run_<date>_<time>.log` (newest: `logs/latest.log`). To start
at boot as a service, see [docs/running.md](docs/running.md).

## Project layout

```
app/                    The guidance service (docs/architecture.md lists every module)
  main.py                 The FastAPI app: starts the sensors, the voice and the API
  config.py               Every threshold, in one place
  sensors/                LiDAR, radar, cameras, ToF, NPU and IMU drivers
  perception/             What is around: the walk decision, the ToF's hazards and walls,
                          drift, movers, the route planner, sensor fusion
  guidance/               What is said: the closed vocabulary, the speech gate, the voice
  core/                   Shared state, freshness, health, the walk recorder, access
  api/                    The HTTP API (docs/api.md)
templates/              The web pages: the dashboard at /, /tof/live, the radar pages
model/                  The Hailo models (.hef)
ble_bridge/             The phone link over Bluetooth LE (its own service)
deploy/                 systemd service and installer
tests/                  Unit tests, hardware-free: ./run_tests.sh
tools/sim/              Desk simulators: the real code on synthetic sensor data
tools/field/            Test walks: replay, labels and logs
tools/device/           Checks and calibration run against the real device
docs/                   The documentation below
.github/workflows/      CI: lint, tests and a type report on every push
start_lidar.sh          Starts the app (checks the devices first)
run_tests.sh            Runs the tests (and, with --full, the simulators)
pyproject.toml          Project metadata and the lint / type / test settings
requirements.txt        What the device runs (pinned); requirements-dev.txt adds the dev tools
```

## Development

```bash
./venv/bin/pip install -r requirements-dev.txt   # ruff, mypy, jsonschema (not needed to run it)
./run_tests.sh                                   # the tests
./venv/bin/ruff check .                          # lint (CI fails on it)
./venv/bin/mypy                                  # types (a report for now)
```

CI runs all three on every push (`.github/workflows/tests.yml`). Settings live in
`pyproject.toml`, the version in `app/config.py` (`APP_VERSION`), and what changed in
[CHANGELOG.md](CHANGELOG.md).

## Documentation

| Document | What it covers |
| --- | --- |
| [What the user hears](docs/what-the-user-hears.md) | The closed vocabulary: every sentence and when it is said |
| [The safety model](docs/safety-model.md) | Why three sensors, how they are trusted, drift, the BEV grid, movers |
| [Architecture](docs/architecture.md) | Every module, the threads, measured timings |
| [Running it](docs/running.md) | Starting, the service, the radar array, the ToF, ground hazards, walls, models |
| [Setting up a new Pi](docs/setup.md) | Drivers, boot config, packages, hardware checks |
| [Field testing](docs/field-testing.md) | Test walks, timings, the label buttons, replaying walks at the desk |
| [Dashboard](docs/dashboard.md) | The pages and what each panel shows |
| [API](docs/api.md) | The HTTP endpoints |
| [Configuration](docs/configuration.md) | The settings in `app/config.py` that matter most |
| [Design notes](docs/design-notes.md) | Bugs found and fixed, and why the code is the way it is |
| [Known limits](docs/known-limits.md) | What it cannot do yet |
| [The phone link](ble_bridge/README.md) | The BLE bridge and its protocol |
