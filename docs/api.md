# API

**Guidance** — the endpoints that matter

| Endpoint                | Returns                                                                                                                                                                                                                                 |
| ----------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `GET /guidance/walk`    | **The one merged instruction.** Obstacle avoidance + surface steering + crossing, in safety-first priority order. `notify.speak` / `notify.cue_seq` is the _only_ field that means "announce this"; everything else is state to display. It is the guidance loop's newest result (one per LiDAR scan) (`loop.age_s`): a request never runs the pipeline, so no page decides or commits what is said. Older than `GUIDANCE_RESULT_STALE_S`, it reports rule `unavailable` and why |
| `GET /guidance/policy`  | The straight-walk decision on its own, in feet — the guidance loop's last one (`loop.age_s`), never re-run by the request                                                                                                                |
| `GET /guidance/gyro`    | IMU health, yaw bias and whether it is trusted, tilt, gait. `/guidance/heading`'s `yaw_source` says which sensor is steering; this says why |
| `GET /guidance/drift`   | Collision course + veer: what you will walk into, when, and the smallest correction                                                                                                                                                     |
| `GET /guidance/heading` | Raw LiDAR odometry — turn rate, speed, accumulated heading                                                                                                                                                                              |
| `GET /guidance/fused`   | Ranked hazard list (clock position + distance)                                                                                                                                                                                          |
| `GET /guidance/scene`   | On-demand "what's around me?" full readout                                                                                                                                                                                              |
| `GET /guidance/status`  | Sensor health, in plain English                                                                                                                                                                                                         |
| `GET /guidance/log`     | Every line the device has SAID this run, timestamped. Written at the commit point, so it is the conversation the user had — a poller would see a different one |
| `GET /guidance/log/text`| The same as plain text, for `curl` from a phone mid-walk |
| `GET /guidance/latency` | How late this run's sentences were: scan and camera age, tick time, Pi voice start, front range at the warning — median and worst, urgent apart |

**Sensors**

| Endpoint                      | Returns                                                       |
| ----------------------------- | ------------------------------------------------------------- |
| `WS  /lidar/ws`               | Live scan stream                                              |
| `GET /lidar/reader`           | Is the serial reader keeping up: `dropped_meas`, `resyncs`, `partial_scans` (all should stay near zero), backlog, measured rate, and the newest scan's age |
| `GET /lidar/movers`           | Things moving by themselves: position, speed, the user's own motion (`ego`), and the one coming at the user (`threat`) |
| `GET /lidar/bev`              | BEV grid extraction — obstacle shapes, doorways, blind arcs, `radar_confirmed` (= glass) |
| `GET /lidar/bev/summary`      | The same, as plain sentences in feet and clock positions      |
| `GET /lidar/bev/image`        | Top-down PNG: red occupied, dark free, grey unknown. A registration smear is obvious here and nearly invisible in the numbers |
| `GET /radar/movers`           | Echoes closing in by themselves; `threat` is one only the radar hears coming at the user |
| `GET /radar/state`            | Per-direction radar distance + staleness                      |
| `GET /radar/boards`           | Which board is on which port, plus per-board lag              |
| `GET /radar/config`           | Range, guidance zones, timing thresholds                      |
| `GET /radar/comparison`       | LiDAR vs radar per direction — the glass check                |
| `GET /radar/map`              | Everything the scope draws — mount geometry, per-slot boards, `sector_label`, cardinals, guidance bands, all in feet |
| `GET /radar/diagnostics`      | Per-board hardware telemetry — measured Hz, lag, reconnects   |
| `GET /radar/health`           | One-line verdict: every board streaming, every cardinal live  |
| `POST /radar/rediscover`      | Stop every reader and rediscover, after replugging a cable    |
| `GET /camera/live/{camera}`   | The newest picture of one camera, or `grid` (all four), as one small JPEG; `?after=<X-Frame-Id>` waits up to 1 s for a newer one. Pulled one at a time by the dashboard, so it cannot fall behind (2026-11-02: the MJPEG stream did, ~12 s after 10 s on a slow link). `?boxes=1` adds `X-Detections`: what the detector found in that very picture (the front's, waited for, ~55 ms) or a side camera's last (`age_ms` older) |
| `GET /camera/seg/{sector}`    | Colorized segmentation debug view                             |
| `GET /camera/stats`           | capture / detection / segmentation FPS, `light` (the sensor's lux, exposure and gain) and `views` (each camera's brightness, sharpness, lux, dark / blurred / dim), per-sector detections — each with `distance_ft`: of the things the laser ranges in the box's bearings, the one that fits the box's size (`distance_source` `lidar`), else the box-size estimate (`camera`; `camera_cut` = at most) |
| `GET /camera/distances`       | Why each detection's distance is what it is: bearings searched, every laser range there, the size estimate (a wide person box = sitting), the choice, and the lens angle it implies (`implied_hfov_deg`) |
| `GET /tof/live`               | The ToF's live depth page (dashboard: *depth*)                |
| `WS  /tof/ws`                 | Its frames, raw depth + confidence (at most `TOF_LIVE_MAX_FPS`), and `/tof/status` twice a second |
| `GET /tof/status`             | Is the ToF delivering: fps, frame age, device, restarts, which calibration places its dots |
| `GET /tof/depth.jpg`          | The newest depth frame as a picture                           |
| `GET /tof/overlay.jpg`        | The front camera's picture with the depth dots drawn on: the calibration check |
| `GET /tof/rgbd.jpg`           | The front camera as a depth camera: aligned depth laid over its picture, each detection labelled with its measured distance |
| `GET /tof/aligned.npy`        | The newest depth picture aligned to the front camera's (float32 m, NaN = none, per `TOF_ALIGNED_CELL_PX` block; `?what=ground` for along the ground) |
| `GET /tof/hazards`            | The ToF's hazards on the ground: the fitted ground, each confirmed hazard (kind, feet, the camera's word, would it be said and why), the one the guidance would say, every sighting of the newest frame |

**Navigation**

| Endpoint                 | Returns                                        |
| ------------------------ | ---------------------------------------------- |
| `GET /navigation/decide` | Obstacle-avoidance decision from current state |
| `GET /navigation/walls`  | Wall detection from the current scan           |

`GET /health`, `GET /api/status` for liveness. Full schema at `/docs`.
