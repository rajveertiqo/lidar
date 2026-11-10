# Architecture

```
main.py                 FastAPI app + lifespan (starts camera, LiDAR, radar, gyro, voice)
config.py               Every threshold, in one place — the single source of truth

utils/
  simple_guidance.py        THE ONE SPEAKER: the closed vocabulary, warn-fast /
                            clear-slowly smoothing, and the gate that decides what is said
  pi_voice.py               The guidance loop (each LiDAR scan) and espeak-ng, so the device talks
                            without a browser; hands the voice to the phone when no
                            earpiece is connected
  startup_gate.py           Silent until the sensors have settled ("scan before you speak")
  spatial_scan.py           Corridor / room / open, and doorways from fitted walls —
                            a tie-break for "Turn left" vs "Turn right", never a new sentence
  service.py                systemd notify, heartbeats and the shutdown backstop
  access.py                 Admin-token guards for the endpoints that change how
                            the device steers
  test_console.py           The terminal during a field test: one line per sentence
  lidar_utils.py            Scanning, motor PWM soft-start, sector/cone bucketing,
                            cluster filtering (a lone reflection cannot fire a STOP)
  radar_utils.py            A121 array acquisition, 6 boards -> 4 directions
  pi_camera_utils.py        Capture, mosaic slicing, detection thread, segmentation thread
  hailo_inference.py        Detection engine; auto-detects the HEF's layer layout,
                            letterboxes to 640×640, DFL decode + per-class NMS
  hailo_device.py           The ONE shared VDevice — the NPU is a single device, and
                            a second VDevice fails with OUT_OF_PHYSICAL_DEVICES
  seg_inference.py          Walkable-surface segmentation engine
  yoloseg_decode.py         Pure-NumPy YOLO11-seg decoder (the HEF is exported
                            `nms:false`, so all post-processing is ours)
  walkable_guidance.py      Which segmentation classes are walkable ground (the
                            segmentation view draws them bright)
  walk_policy.py            THE decision, in FEET: walk straight unless blocked;
                            the nearer of LiDAR and radar wins; a gap names glass
  heading_tracker.py        Yaw, walking speed and sideways slip. Speed from
                            scan matching; yaw fused with the gyro below
  gyro_reader.py            MPU-6500 on SPI — yaw rate at 100 Hz, gait from the
                            accelerometer, and gravity-referenced tilt. Its bias
                            is corrected against the scan matcher
  drift_guidance.py         Predictive: projects the user's ACTUAL motion forward
                            and warns 3-6 s before they veer into something
  sensor_fusion.py          LiDAR + camera -> one ranked hazard list (clock positions)
  unified_navigation_system.py  Obstacle avoidance + road-crossing policy
  unified_guidance.py       Merges every source into ONE instruction
  traffic_light_detection.py    Crossing state machine
  environment_detection.py  Fog / rain / low-light from camera + LiDAR quality
  lidar_movers.py           People and vehicles MOVING in the LiDAR scan, from any
                            side: who is coming at the user, from where, and when.
                            Silent ("would say") until LIDAR_MOVERS_SPEAK
  radar_movers.py           Echoes only the radar hears closing in — the LiDAR's blind
                            arc, things above/below its plane ("Something …")
  bev_grid.py               Ego-centred BEV occupancy grid — obstacle SHAPE, doorways,
                            radar-stamped glass, blind arcs. ADVISORY, never stops
  shared_state.py           Thread-safe state shared between all sensor threads
  speech_log.py             WHAT THE DEVICE SAID, with wall and elapsed clocks.
                            Written where speech is committed, never by polling
  walk_recorder.py          Every guidance tick's inputs and what was said, for replay at the
                            desk; the clips saved on a tester's Wrong / Missed / label press
  freshness.py              Is this reading current? (monotonic receive time: a clock jump
                            is not a dead sensor)
  route_planner.py          The best way through what is around the user, now and as things
                            move. ADVISORY ("would say") until ROUTE_PLANNER_SPEAK
  motion_tracker.py         Moving things followed with a Kalman filter: where, how fast, how sure
  tof_camera.py             The ToF depth camera, captured in a child process
  depth_fusion.py           Camera + ToF: each depth pixel placed on the front camera's picture
  ground_hazards.py         The ToF's view of the path: stairs, steps, drops, poles, walls, and
                            the surface the rig rests on when set down
  pi_health.py              SoC temperature and the under-voltage / throttle flags
  hailo_stats.py            NPU utilisation and FPS, in-process (hailortcli monitor's job)
  live_map.py               What every sensor sees, placed around the user, for /map
  mobile_delivery.py        Whether the phone can speak, as the BLE bridge reports it
  mpu6500.py                MPU-6500 register driver (gyro_reader runs it)
  gait_calibration.py       Where the measured walking stride is kept
  radar_calibration.py      Where the radar bearing table is kept

routers/                API layer (see api.md)
templates/              The web pages: dashboard.html (/), tof_live.html (/tof/live),
                        radar_test.html, radar_map.html, map_test.html, gait_test.html,
                        index.html (the previous dashboard, /classic)
model/                  yolo26m.hef (detection), best.hef (segmentation), metadata.yaml
ble_bridge/             The phone link: a separate service (shappe-ble.service) speaking
                        the closed vocabulary to the phone over Bluetooth LE
deploy/                 systemd user service + installer (see running.md)
start_lidar.sh          Starts the app (checks the devices first); --background for a walk
run_tests.sh            Every test (tests/), and with --full the desk simulators

tests/                  Unit tests, hardware-free (./run_tests.sh)

tools/sim/              Desk simulators: the real code on synthetic sensor data
  scene_hunt.py           Random scenes and their mirror images against the safety rules
  glass_sim.py            Glass doors, panes beside walls, corridors and posts: is the pane
                          named, and only the pane?
  ground_hazard_sim.py    The ToF on ray-traced scenes: stairs, kerbs, drops, poles, and walls
                          (a wall at an angle, a corner, a blind, a curtain) against people and
                          furniture
  drift_sim.py            A virtual user down a synthetic corridor, to tune the drift warnings
  movers_sim.py           People, bikes and cars moving round the user, and still rooms,
                          through the real mover tracker
  route_sim.py            The route planner on tracker scenes
  tof_fusion_sim.py       The camera + ToF merge and its bench calibration, without the sensor

tools/field/            For test walks and the data they give
  walk_replay.py          Replays a recording or clip through the CURRENT code: what would it
                          say now? (--turn-on: features not yet spoken)
  label_report.py         Scores the code against the testers' "what was really there"
                          labels, and exports them for training (on a laptop)
  walk_logger.py          Guidance + FPS + CPU temperature to CSV during a walk
  collect_frames.py       Camera pictures from the running app, for the segmentation set

tools/device/           Run against the real device
  tof_check.py            The ToF camera, link by link (stop the app first)
  tof_calibrate.py        The ToF-to-camera bench calibration
  hailo_monitor.py        Live NPU utilisation and FPS of the running app
  bev_check.py            Is the BEV grid actually working? --checklist: the physical tests
  anchor_diversity.py     Can the scan band anchor camera depth? (the Spatial Plan's E2)

docs/                   This documentation
```

## Threading

Four independent loops, so a slow one cannot stall the others:

1. **Capture** — pulls raw mosaic frames from Picamera2 (~30 FPS).
2. **Detection** — every picture holds all four cameras: the FRONT on each, published
   at once, and a side camera at most every `DETECTION_SIDE_MIN_INTERVAL_S` (0.4 s, the
   one longest undetected first), so each side is re-detected within ~1.2 s. Guidance
   only reads the front: a side's ~40 ms ahead of it costs the front pictures.
   Letterboxed straight from the camera's array with OpenCV, not via PIL.
3. **Segmentation** — its own thread, and only while its front view is open
   (`SEG_ALWAYS`): its cue no longer reaches the voice, and the NPU time it
   took came out of detection.
4. **LiDAR** — its own thread, broadcasting scans over WebSocket.

Detection and segmentation share one NPU via HailoRT's scheduler, so
`SEG_TARGET_FPS` acts as a **cap** that stops segmentation from starving the
detector.

## Measured on-device (2026-09-25)

|                        | Before (all 4 cameras + seg) | Now                                   |
| ---------------------- | ---------------------------- | ------------------------------------- |
| Camera capture         | 30 FPS                       | 30 FPS                                |
| Front camera detection | 3.6 /s, up to ~0.5 s old     | 11 /s, 106 ms old (median)            |
| Each other camera      | 3.6 /s                       | 3.7 /s                                |
| Detection pass         | ~190 ms (4 quadrants)        | ~90 ms (2 quadrants, 79 ms of it NPU) |
| Letterbox per quadrant | 16 ms (PIL)                  | 4 ms (OpenCV)                         |
| Segmentation           | 10 Hz, always                | only while its view is open           |
| LiDAR                  | ~23 scans/s                  | ~15 scans/s (`LIDAR_RPM` 900)         |
| Guidance decision      | ~6 ms per poll               | unchanged                             |
