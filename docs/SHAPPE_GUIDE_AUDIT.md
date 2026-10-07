# Shappe Guide — safety audit and repairs, 2026-09-29

Audience: the engineers who maintain this device, and whoever reviews these changes before
anything reaches a person who depends on it.

**What this is.** A code-level audit of the repository on 2026-09-29, in two passes:

- **Pass 1** (session ending ~11:59 IST) checked findings F1–F21 of `PROJECT_REPORT.md`
  against the code. It restored the removed tests and made targeted repairs.
- **Pass 2** (from 12:00 IST) did what pass 1 could not. Its own review was cut off by a
  usage limit, so pass 2:
  - reviewed pass 1's diff independently, in six areas;
  - re-ran pass 1's reproductions against a clean copy of the pre-audit code;
  - fixed what that review found, including defects pass 1 had itself introduced;
  - had its own diff reviewed independently (§9).

Where a finding could be reproduced, it was reproduced **before** it was fixed. Every result
below is labelled with how it is known.

**What this is not.** It is not evidence that the device is safe for independent navigation.
- Every change was exercised by hardware-free unit tests and simulations only.
- The application was **not run** in either pass, and no sensor was driven.
- No change has been deployed to a worn device.

§7 lists the physical validation still outstanding. A human review before commit is
still required.

Evidence labels:
- **VERIFIED IN CODE**
- **REPRODUCED IN TEST**: a test or script that fails on the code before the fix
- **PATCHED; HARDWARE VALIDATION PENDING**
- **VERIFIED ON DEVICE**: a read-only observation on this Pi, quoted
- **SUSPECTED; NOT REPRODUCED**
- **PRODUCT / CALIBRATION DECISION REQUIRED**

`PROJECT_REPORT.md` is not in the repository. It was untracked, and was removed from the
directory by someone other than either pass. Pass 2 recovered its full text (956 lines)
from pass 1's session transcript, into the session scratch area, and read it from there.
It was **not** restored into the repository.

---

## 1. Environment and scope

| | |
|---|---|
| Machine | The rig itself: Raspberry Pi 5 Model B Rev 1.1, kernel 6.18.39+rpt-rpi-2712, Python 3.13.5. Project venv: fastapi 0.141.1, uvicorn 0.52.1, numpy 2.2.4, rplidar 0.9.2, acconeer-exptool 7.18.2, hailo_platform 5.1.1; picamera2 via apt |
| Repository | HEAD `1921ac3` ("radar rpm changes", committed by the owner at 10:37). It is pass 1's starting diff (`5e2f1cf` plus the working-tree edits) and is unchanged since. Both passes' work is **uncommitted**: 41 tracked files modified; untracked `tests/`, `docs/`, `utils/freshness.py`, `tools/odometry_density.py` |
| Application | Not running at any point checked: no uvicorn process, nothing listening on :8000. `fuser` found no owner of `/dev/ttyUSB0`, `/dev/ttyACM0`, `/dev/spidev0.0`, `/dev/hailo0`, `/dev/video0` or `/dev/media0`. `shappe-guide.service` is **not installed** (no user unit files) |
| Hardware present (read-only enumeration, VERIFIED ON DEVICE) | **LiDAR:** Silicon Labs CP2102 (`cp210x`) on `/dev/ttyUSB0`, by-id `usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_01f49e0b5bd7b2478a89e10184961a3f-if00-port0`. **Radar:** six 1a86 dual-serial boards on one 7-port hub. Twelve `ttyACM` nodes, each board with `-if00` and `-if02` by-id links (serials 5AEC021929, …934, …938, …975, 5AEC022220, …273). **NPU:** Hailo-10H at PCIe `0001:01:00.0`, firmware loaded, `/dev/hailo0`. **IMU:** `/dev/spidev0.0` and `0.1` present |
| Power and thermal (VERIFIED ON DEVICE) | `vcgencmd get_throttled` = `0x0`; 59.8–63.7 °C over the session. No USB disconnect or under-voltage in this boot's kernel log |
| Clock (VERIFIED ON DEVICE) | This boot's kernel log is stamped "Sep 28 18:57" although the machine booted about 09:24 on 29 Sep. Pass 1 saw timesyncd step the clock **forward ~14.5 h** about 38 s after boot. The user unit has no ordering on time sync. This is the F20 condition |
| Admin token | `~/.config/shappe-guide.env` exists, mode 600, one `SHAPPE_ADMIN_TOKEN=` line. The value was **not** read or printed |
| Audio (pass 1, VERIFIED ON DEVICE) | The only sink is HDMI, which is the default. With no earpiece, the Pi voice is correctly not usable and the phone dashboard would speak |

**Not done, deliberately.** Neither pass ran the application, `camera_web.py`, a radar
probe, an IMU commissioning tool or the LiDAR. Starting the app would spin the motor at
the unconfirmed 900 RPM mapping (F14) and open every sensor. No register-level SPI/I²C
transaction was issued and no motor command was sent.

**Inspected.** Every first-party Python module on the speaking, sensing, fusion, NPU,
radar, motion, service and access paths. Also:
- all eight routers and all five dashboards' JavaScript;
- `start_lidar.sh` and `deploy/`;
- the tools that poll the app;
- the installed `rplidar` driver's constructor, and the HailoRT Python surface (read, not called).

The HEF models were **not** read as source; `model/metadata.yaml` was read for class
names only. Dormant layers — `companion`, `environment_detection`, the `/crossing` state
machine and `decisionMaker` — were read for any path into speech or into a clear-path
display.

---

## 2. Baseline

| Step | Command (from `/home/shappe/lidar`) | Result |
|---|---|---|
| Pass 1: the removed suite (31 modules) extracted from `24f5a95` into a copy of the current source | `./venv/bin/python -m unittest discover -s tests -t .` | 729 OK (40 s) |
| Pass 2 start: the suite as pass 1 left it | same | **904 OK (54.0 s)**. No `logs/`, `stride.json` or `radar_bearings.json` created |
| Pass 2: historical tests vs `24f5a95` | `diff` per file | Five historical files changed. Every change is a fixture (clock epoch, opt-in flags, the two new status sentences), with the 503 assertion noted in pass 1. **No assertion weakened.** |
| Pass 2: the **unmodified** historical suite against the current source | `git archive 24f5a95 tests`, run in a copy of the tree | 34 fail. All of them are one cause: the historical harness stamped its first scan `t = 0`, which the shared freshness rule reads as "never reported". Behaviour did not regress |
| Pass 2: the full current suite against a clean pre-audit copy (`git archive HEAD`) | same | 899 ran: 20 **failures** (real behavioural reproductions: F9 ×7, F17 ×4, F10/F20 ×2, F5, F21, NEW-002, NEW-003, the vocabulary), 150 errors (new APIs the old code lacks) |
| Pass 2: pass 1's reproduction scripts re-run on that clean copy | `repro_baseline.py`, `repro_f5.py`, `f21_repro.py` | Reproduced as pass 1 quoted them (§6) |
| Pass 2: the first 170 pass-2 tests against the tree **as pass 1 left it** (reviewer's snapshot) | `unittest` on the 7 `*_pass2` modules | **136 fail or error**. The 34 that pass there are guards and positive controls |
| Pass 2: after the independent review of pass 2 (§9) | same | 9 more tests (179 in the `*_pass2` modules). Each was shown to fail without its fix: the reviewer's scripts, or a mutation run |
| After the independent review | same | 1084 tests, OK (75.0 s) |
| **Final, after the follow-ups (§10)** | `./venv/bin/python -m unittest discover -s tests -t .` | **1101 tests, OK (75.9 s)** = 904 + 179 pass-2 + 17 follow-up + 1 positive control. No `logs/`, `stride.json`, `radar_bearings.json` or `core*` created in the repository |

**Pre-existing test files changed in pass 2.** No assertion was weakened; two were made stricter.

| File | Change | Reason |
|---|---|---|
| `tests/synthetic.py` | A scanning `lidar_state()` is stamped `time.time()`, not `1.0` | `walk_policy.decide()` now applies the freshness rule (NEW-035). A 1970 stamp is a 55-year-old scan |
| `test_guidance_audit.TestStaleLidarIsNotSteeredOn` | `lidar_is_fresh({})` now expected **False**. A state that has data and no clock is still True | NEW-036: an empty state passed as fresh (fails open). **Stricter** |
| `test_sensor_trust` (3 glass-notice tests) | The expectations follow the new policy: cold-start grace, recovery backoff; the grace is patched to 0 in the hazard-priority test | NEW-028. The properties kept: loss after an announced recovery is always said; "on" is never said while the front is dark; a hazard is never displaced |
| `test_sensor_trust.test_recovery_to_real_scans_resumes_guidance` | The loop's clock drives `time.time()` | The test stamped scans up to 16 s in the real future; `walk_policy` now (rightly) refuses those |
| `test_speech_delivery` | `outcome()` drains the voice's log queue first; two fixtures seed the sink cache with `time.monotonic()` | NEW-032: delivery events reach the log through a writer thread; the cache is monotonic |
| `test_radar_calibration_validation` (1 test) | Fakes `_usb_info` for its fake node | NEW-015: a board is opened only on a node whose sysfs serial proves it is that board |
| `test_sensor_audit.test_simulator_only_without_picamera2` | Patches `CAMERA_SIMULATION_ALLOWED=True` for its "not installed" case | NEW-018: camera simulation needs the opt-in |
| `test_access_control` (same-origin case) | Uses the dashboard's real form (`Host: 192.168.0.50:8000`) instead of `testserver` | NEW-022: `testserver`/`testserver` is exactly the shape of a rebinding request |
| `test_motion_trust` | `correct_bias.assert_not_called()` now runs with the gyro mounting patched confirmed and a healthy `yaw_between`, plus a trusted positive control | It could never fail before. **Stricter** (mutation-checked) |

**CI workflow: not restored (conflict).** `.github/workflows/tests.yml` was deleted by the
owner in `1921ac3`. Restoring it would undo an explicit, recent owner change (§8).

To run the suite:

```bash
cd /home/shappe/lidar && ./venv/bin/python -m unittest discover -s tests -t .
```

---

## 3. Architecture as traced

```
 DEVICE (owner thread)                        STAMP (clock)                         STORE (lock)
 RPLIDAR reader (_ScanStream)                 captured_at = wall - backlog          lidar_state (lidar_lock)
   -> lidar-analysis thread (monotonic pace)    + received_mono, simulated (1 write)
      -> heading_tracker (own lock; fsync after release) -> drift -> BEV grid
 6x A121 board threads (radar_utils)          row last_update = wall; + received_mono radar_state (radar_lock)
   node re-verified by USB serial each connect  lag/rate on monotonic
 gyro sampler (100 Hz)                        t = wall (yaw), m = monotonic (gait)  gyro buffer (own lock)
 camera capture -> detect (generation)        frame_time = capture wall, simulated  camera_state (camera_lock)
   -> seg thread (only while a view is open AND a segmenter serves it)            walking_guidance_state
                                  |
                                  v   each tick (5 Hz loop, monotonic pace) AND every GET /guidance/walk
 unified_guidance.get_unified_guidance()
   decisionMaker (legacy, display only; only on a fresh LiDAR)   at_crossing = FRONT camera light
   walk_policy.decide (feet; simulated/stale LiDAR = no LiDAR; owed boards)
   startup_gate.check (lidar_absent after STARTUP_LIDAR_ABSENT_S)
   current_motion(heading state) — stale -> None
   simple_guidance.guidance  [simple_guidance._lock: smoothing + gate + glass status]
        -> (text, speak, cue_seq, announced)
   ONE pi_voice.status() snapshot (cached sink, monotonic; prober thread owns wpctl)
   commit: pi_voice.speak(text, seq, sink_usable)  <- refuses seq <= last accepted
           test_console.spoken ; speech_log.say(cue, via)   (one printable line)
   notify {speak, cue_seq, instruction=announced, pi_voice={..., accepted_seq}}
                                  |
                                  v
 pi-voice thread: espeak-ng (no lock held) -> events -> bounded queue -> pi-voice-log thread -> speech_log
 dashboard: announces cue N unless pi_voice.usable AND accepted_seq == N   (piHasCue)
```

**Locks** were checked for cross-acquisition in both passes and by both independent reviews.
The only nestings, each in one order:

- `_save_lock → radar _key_lock → {shared_state.radar_lock, radar_calibration._lock, _cycle_lock}`
  (radar calibration).
- `pi_camera _gen_lock → {camera_lock, path_guidance_lock, seg_latest_lock}` (NPU publish).

These are leaf locks:
- `pi_voice._reports_lock`;
- the voice's own lock;
- the segmenter's `_close_lock`;
- the heading tracker's lock (fsync is now after release);
- `speech_log._lock` (file I/O, never on the voice's path now).

No sensor or guidance lock is held across a subprocess, an audio probe or a file write.

**Not read-only GETs.** These requests change device state:
- `GET /guidance/walk` (and `/guidance/spatial`) runs the whole pipeline and can admit a
  sentence. The gate makes repeats idempotent. `tools/walk_logger.py` and `tools/bev_check.py`
  both call it.
- Opening `/camera/seg/{right,left,back}` or `/camera/seg_grid` changes detection
  scheduling (F12).

---

## 4. Finding matrix — F1 to F21

Each entry gives the pass 1 result, then what pass 2's review found and did.

### F1 — test suite and CI removed
- **Status:** VERIFIED IN CODE. Tests restored (1075, OK). CI **not** restored: owner conflict (§8).

### F2 — loss of front radar (glass detection) was never said
- **Pass 1:** added "Glass sensor off" / "Glass sensor on", from one definition of front glass coverage.
- **Pass 2 review:** pass 1's notice had defects of its own. **REPRODUCED IN TEST** on the tree pass 1 left, NEW-028:
  - it replaced "Walk between them" steering;
  - it was admitted with a wall already in the raw poll ("Blocked by wall" came 1.2 s later);
  - it raised the gate's level, so a "Wall ahead" confirming just after it moved from 5.2 s to 8.0 s;
  - it caused "Walk straight" to be said again;
  - a board reconnecting once a minute produced 19 notices plus 20 extra "Walk straight" in 10 minutes;
  - every cold boot said "off … on".
- **Pass 2 change:** the notice is now a *status* with its own admission (`_Gate.admit_status`):
  - it has its own `cue_seq` and leaves the gate's memory of the guidance untouched;
  - it is said only where "Walk straight" stands **and** the raw poll is clear;
  - it waits only for the voice to be free;
  - recovery backs off (10 → 20 → 40 … ≤ 160 s) after each announced loss; a loss after an announced "on" is always said;
  - before coverage has ever been complete, and only **while the radar is still coming up** (discovery has not reported, or an owed front board is starting or calibrating), "off" waits, at most `SIMPLE_GLASS_STARTUP_GRACE_S` (30 s, **assumed**: set it from a timed cold boot). A front board discovery never found is announced on the usual 3 s confirm (review F3, §9).
- **Coverage definition (NEW-036):** it now checks *which* boards the mount owes the front, not how many.
- **Status:** PATCHED; HARDWARE VALIDATION PENDING. **User-facing behaviour change: review the wording and timings before field deployment.**

### F3 — a traffic light behind the user withheld "Walk straight"
- **Status:** REPRODUCED IN TEST (pass 1, re-run on clean HEAD: a back-camera light → `at_crossing=True`); PATCHED.
- **Pass 2:**
  - The dashboard showed "withheld" from the legacy any-camera flag; it now reads `at_crossing`.
  - Side-camera lights no longer withhold "Walk straight" either, and `at_crossing` fails open if the crossing helper raises. Both are **PRODUCT DECISIONS** (NEW-055).
  - Nothing reads colour, and nothing says it is safe to cross.

### F4 — commit/speak race
- **Pass 1:** the `cue_seq`-ordered handoff. Its tests (8 concurrent pollers → one sentence; deterministic interleaving) hold.
- **Pass 2 review:** two gaps.
  - Pi-XOR-phone was decided on *different* snapshots, so a cue could reach **nobody** (NEW-031). Reproduced: the loop admits the cue with the sink on HDMI ("via phone"); the earpiece appears; the dashboard's next poll sees `usable` and stays quiet.
  - `speak()` still blocked on speech-log writes (NEW-032).
- **Pass 2 change:**
  - `notify.pi_voice.accepted_seq`: the phone speaks cue N unless the Pi accepted N and is usable (`piHasCue`, both dashboards, tested under node). The failure left is both voices for one cue, never neither.
  - Delivery events go through a bounded queue to a writer thread.
- **Status:** PATCHED.

### F5 — simulated LiDAR in the safety path
- **Status:** REPRODUCED IN TEST (re-run on clean HEAD: "Walk straight" at +1.0 s, health `lidar_ok: True`); PATCHED.
- **Pass 2:**
  - The claim "simulated scans make the voice say 'Sensors offline'" was unreachable at a cold start: the startup hold required a LiDAR that was never up. Fixed with NEW-029.
  - `walk_policy` still let a simulated or stale scan cancel a real radar echo. Fixed with NEW-035.
  - The camera had the same automatic, unflagged simulation. Fixed with NEW-018.

### F6 — scan-match odometry at the rig's density
- **Status:** REPRODUCED IN TEST (simulation). **PRODUCT / CALIBRATION DECISION**; `HEADING_BIN_DEG` not changed.
- **Pass 2:**
  - The tool's bearing grid was phase-locked, which aliased yaw ("164 points: 20 °/s reads 30 °/s") and hid noise. Randomised per revolution now.
  - Re-measured, jittered phase:
    - turning 20 °/s reads 20.0–21.2 °/s;
    - **standing still reads 1.3–3.4 °/s of yaw at 164–250 points with 1° bins**, and 0.7–1.5 °/s with 2° bins (NEW-046).
  - Pace and stillness at 2°/3° bins as pass 1 reported (1.17–1.20 m/s in back-wall and room scenes).
  - The 2° bins pass 1 proposed would have exposed NEW-024. That is now fixed, so the proposal stands.
- **Hardware:** measured `points_per_scan` at the current RPM, then measured walks.

### F7 — NPU initialisation never retried
- **Pass 1:** retry with backoff, generations, closing a broken engine.
- **Pass 2 review:**
  - The broken-engine reset "after 5 s" only held for calls that fail fast. A stalled NPU fails through HailoRT's 30 s timeout, so the reset came after ~90 s with `hailo_ready` True (NEW-019).
  - "VDevice created in one place, cached only on success" was already true before the audit.
  - "A failed init releases what it entered" is effectively unreachable.
  - Lifecycle edge cases (NEW-040).
- **Pass 2 change:**
  - A streak is broken after no success for 5 s, on the monotonic clock.
  - The detector's `run()` timeout is 2000 ms (`HAILO_DETECT_RUN_TIMEOUT_MS`, derived from the repo's own measurements).
  - A segmenter is not closed under a running call.
  - A raising `close()` no longer kills the worker.
  - Stale frames are not re-detected.
  - Closed engines are not kept referenced.
- **Residual:** a cached VDevice that goes bad is never released. Reported honestly (`hailo_ready` False, "Detection engine not ready"), but not recovered. Proposal only (§8).
- **Status:** PATCHED; HARDWARE VALIDATION PENDING.

### F8 — radar calibration: no server validation; a restart blinded the array
- **Pass 1:** validation, atomic save, live re-key. Its 42 tests hold.
- **Pass 2 review:**
  - **HIGH (NEW-015), pre-existing, in neither report:** a board thread reconnected by device *node* and never checked which board now owned it. After two boards re-enumerate in swapped order, each streams the other under its own direction, and both rows look healthy. The code path was confirmed with a faked sysfs; the kernel renumbering itself is **SUSPECTED**.
  - Coalesced restarts ran a second blind cycle (NEW-016).
  - One serial answering on two nodes ran two threads on one key (NEW-017).
  - Duplicate JSON keys won silently.
  - `print()` ran under `_key_lock`.
  - Lag was measured on the wall clock.
  - A retired thread could `start()` a detector on a port the new generation owns.
  - Routers judged staleness themselves.
  - The Rediscover button was silent on the new 503.
- **Pass 2 change:**
  - On **every** connect the node's sysfs serial must equal the board's; otherwise the by-id `-if00` link for that serial is resolved and re-verified; otherwise nothing is opened and the row goes offline.
  - Read-only check on this rig: an entry pointed at the wrong node resolved to the right one.
  - Plus the fixes for each item above.
- **Status:** PATCHED; HARDWARE VALIDATION PENDING (a supervised replug and swap).

### F9 — markup injection; admin endpoints open without a token
- **Pass 1:** escaping, fail-closed admin (503), a cross-site write refusal. Pass 2's manual check of every `innerHTML` sink found none missed.
- **Pass 2 review:**
  - WebSockets bypassed the cross-site rule. A cross-site page changed `/navigation/ws` state and could read `/lidar/ws` (NEW-021).
  - DNS rebinding defeats Origin == Host (NEW-022).
  - `POST /guidance/mark` forged speech-log lines and sent ESC sequences to the terminal (NEW-023).
  - There were no anti-framing headers.
  - The token was echoed to the journal by the scripts (both pre-existing).
- **Pass 2 change:**
  - A pure-ASGI WebSocket check.
  - A Host allow-list for requests that carry an Origin: IP literal, `localhost`, hostname, `hostname.local`, or `SHAPPE_ALLOWED_HOSTS`.
  - One-line sanitising in the endpoint, `speech_log` and `test_console`.
  - `X-Frame-Options: DENY` and `frame-ancestors 'none'`.
  - The token is printed only to a TTY.
  - The scripts add a token line to an env file that lacks one.
- **Status:** PATCHED. Not exercised in a real browser.

### F10 — inconsistent staleness
- **Pass 1:** `utils/freshness.is_fresh`.
- **Pass 2 review:** remaining bypasses:
  - `/radar/map` and `/radar/test` computed staleness themselves;
  - the seg-view heartbeat;
  - camera liveness keyed on *publish* time;
  - `walk_policy` never judged LiDAR freshness at all.

  All of these now use the shared rule. `lidar_is_fresh(None)` / `({})` were True (NEW-036).
- **Status:** PATCHED.

### F11 — audio-sink probe on the urgent path
- **Pass 1:** background prober (holds).
- **Pass 2 change:**
  - The cache aged on the wall clock, so a boot-time step read a fresh answer as stale. It is monotonic now.
  - "Dummy Output" and `auto_null` counted as someone listening; both are refused now.
- **Status:** PATCHED; earpiece validation pending.

### F12 — debug segmentation views reduce detection to the front camera
- **Pass 2 change:**
  - A side view with **no** segmenter serving it still paused side/back detection; it no longer does.
  - Health said "all systems normal" beside its own "detection paused" warning. `all_ok` is now false whenever any warning is present.
  - Scheduling is otherwise unchanged; nothing was measured.
- **Status:** PATCHED (visibility and bounding only).

### F13 — LiDAR port by enumeration order
- **Status:** PATCHED (pass 1). The radar's nodes now follow the serial too (NEW-015).
- **Residual:** radar discovery still probes `ttyUSB*`, so the realpath exclusion depends on udev having created the LiDAR's by-id link (NEW-056, proposal).

### F14 — driver constructor bypasses the soft start
- **Status:** VERIFIED IN CODE; PATCHED (pass 1); HARDWARE VALIDATION PENDING.
- **Pass 2 review:** confirmed the patch cannot leave the motor running uncommanded.
- **Residual (NEW-057):** `/lidar/reader` reports `pwm_command` even after a failed ramp, and `source: hardware` while disconnected. **The 900 RPM target is not physically confirmed.**

### F15 — the speech log said HEARD
- **Pass 2:** the module and `/guidance/log` docstrings still said "actually SAID" / "the conversation the user actually had"; both corrected. Every field is one printable line.
- **Status:** PATCHED.

### F16 — obstacle episodes end on one clear poll
- **Status:** REPRODUCED IN TEST (mechanism). Field explanation **UNCERTAIN** (no logs on the device). **PRODUCT DECISION**; not changed.

### F17 — false stillness at an angle to parallel walls
- **Pass 1:** orthogonalised the forward sensitivity. Correct for pure parallel walls.
- **Pass 2 review:** **HIGH, latent (NEW-024).** The measure smoothed across depth discontinuities and so invented a forward-facing surface at occlusion edges. Walking 1.2 m/s down a corridor that widens ahead read `confidently_still` in:
  - 19/36 cases at 1080 beams;
  - 10–11/36 at 250 and 164 beams with 2° bins.

  `confidently_still` silences "Drifting toward … wall" and lets the gyro re-zero. It is latent at today's 1° bins, where the fit never runs at the rig's density.
- **Pass 2 change:** bins straddling a depth jump, or past the grazing cutoff, count as zero leverage. This keeps today's divisor, so the measure can only fall: max(new − old) = 0.0 over 13,920 frames.
  - The sweep's false stills go to **0/36** in all four configurations.
  - Back-wall and room pace are unchanged.
  - Cost: pole, box and recess scenes fall back to nominal pace (conservative).
- **Status:** PATCHED; measured walks pending.

### F18 — drift and open-space veer have no voice
- **Status:** VERIFIED IN CODE; **PRODUCT / SAFETY DECISION**; not changed.
- **New evidence (NEW-046):** LiDAR-only yaw at the rig's density wanders 1.3–3.4 °/s standing still in simulation, the size of the veers such a cue would announce. So a veer sentence also depends on the gyro being commissioned.

### F19 — the uncommissioned accelerometer drove learning
- **Pass 1:** `GYRO_GAIT_CONFIRMED` gate; the measured stride is protected.
- **Pass 2 review:**
  - Cadence pace was never clamped, reaching 2.8–4.4 m/s (NEW-025, fixed).
  - The heading state had no age (NEW-026, fixed).
  - An fsync ran under the tracker lock, latent until gait is commissioned (NEW-041, fixed).
  - Trust labels were True for simulated input (fixed).
  - Gait file edge cases: mode, directory fsync, override latch, files not marked learned. Fixed.
  - **PRODUCT DECISION (NEW-050):** the uncommissioned accelerometer's "walking" still overrides a LiDAR-measured standstill. It enables the vehicle ego-discount and suppresses the stopped-car "be careful". Reproduced with the project's own `VehiclePoller`: a car closing at 1.5 m/s from 10 m on a standing user was never announced once the accelerometer falsely said walking.

### F20 — wall-clock dependence
- **Pass 1:** future-stamp tolerance; monotonic heartbeats and watchdog.
- **Pass 2 review:**
  - **HIGH impact / low likelihood (NEW-034):** the future-stamp rule only *delayed* a dead sensor's revival. Once the wall clock caught up, the frozen scan was fresh again, and the guide said **"Walk straight" ~60 s after the LiDAR died** (reproduced).
  - Loop pacing in the guidance loop, the LiDAR analysis thread, the camera loops, radar lag and the gyro's step bookkeeping was still wall-clock. A backward step slept the voice loop or froze the LiDAR for its size (NEW-033).
  - A backward step ≥ 0.5 s under a working LiDAR said "Sensors offline" (NEW-037).
- **Pass 2 change:**
  - The stores record a **monotonic receive time** beside every wall stamp. `reading_is_current()` needs both clocks to agree, so it can only refuse more.
  - All pacing, intervals and bookkeeping are monotonic; wall stamps are kept for capture time, cross-thread comparison and display.
  - A future stamp on a scan received just now is a hold, not an outage.
- **Status:** PATCHED.

### F21 — signal-handler deadlock
- **Status:** REPRODUCED IN TEST (re-run on clean HEAD: exit 124; current: completes); PATCHED.
- **Pass 2:** pass 1's NEW-012 fix missed `cleanup_lidar()`'s own first print. In the same reentrant-print condition it raised after taking the handle, so the **motor kept running** and every later cleanup was a no-op. The motor is now stopped first and every print is guarded (NEW-039).

---

## 5. New findings

**Pass 1 (NEW-001 … NEW-014), with pass 2's updates**

| # | Finding | Status after pass 2 |
|---|---|---|
| NEW-001 | No LiDAR at boot → the wearer hears nothing, ever | **PATCHED as NEW-029** (user-facing change) |
| NEW-002 | A ~2 s run of missed glass echoes released the block | PATCHED (pass 1). The 35 % miss rate is an assumption |
| NEW-003 | A learned stride overwrote a measured one | PATCHED; pass 2 closed the files with no `source` (NEW-041) |
| NEW-004 | Simulated scans taught the tracker | PATCHED; pass 2 fixed the trust labels and made its test meaningful |
| NEW-005/006 | Radar key caching; file I/O under a sensor lock | PATCHED; pass 2 moved `print` out of the locks |
| NEW-007 | A failing engine kept `hailo_ready` | Superseded by NEW-019 |
| NEW-008 | The vehicle ego-discount uses unmeasured pace | Pace now clamped (NEW-025). The override itself is a **PRODUCT DECISION** (NEW-050) |
| NEW-009 | Camera-derived steering vs the rangefinders | Partly **PATCHED as NEW-030**. The vehicle "move left/right" is still a **PRODUCT DECISION** (NEW-049) |
| NEW-010 | A failing Pi voice: both voices for the recovery cue | Unchanged, noted |
| NEW-011 | The startup gate fails open on an exception | Unchanged, noted |
| NEW-012 | A reentrant print aborted the SIGTERM handover | PATCHED; completed by NEW-039 |
| NEW-013 | Body-less POSTs cross-site | PATCHED; WebSockets and rebinding closed (NEW-021/022) |
| NEW-014 | Test harness assumptions | Pass 2 found three more (§2) |

**Pass 2 (NEW-015 onward).** Each "PATCHED" row has a regression test in
`tests/test_*_pass2.py` that fails on the tree as pass 1 left it, unless it is marked
as a guard.

| # | Sev. | Finding (where) | Status | Change |
|---|---|---|---|---|
| NEW-015 | **High** | A radar board reconnects by node and streams whichever board now owns it (`radar_utils._stream_board`); `_usb_info` could climb to the hub's serial | REPRODUCED IN TEST (faked sysfs); renumbering SUSPECTED | Serial re-verified at every connect; by-id `-if00` fallback; else offline |
| NEW-016 | Med | Coalesced restart requests → a second 20–30 s blind cycle | REPRODUCED IN TEST | `_cycle_again` cleared once the new generation exists |
| NEW-017 | Med | One serial on two nodes → two threads on one key | REPRODUCED IN TEST | Same device: keep if00. Different devices: refuse both |
| NEW-018 | Med | Camera simulation automatic and unflagged; health green | REPRODUCED IN TEST | `SHAPPE_SIMULATE_CAMERA=1` opt-in; `simulated` in the same write; health `camera_simulated`; labels dropped |
| NEW-019 | Med | Stalled NPU reset after ~90 s, not 5 s | REPRODUCED IN TEST | Time-based streak; detector `run()` 2000 ms |
| NEW-020 | Med | A watchdog SIGABRT writes a full core dump (token, frames) into the app directory | VERIFIED IN CONFIG | `LimitCORE=0`, `TimeoutAbortSec=10` in the unit (**not installed**, §8) |
| NEW-021 | Low–Med | WebSockets outside the cross-site rule | REPRODUCED IN TEST | Pure-ASGI check, close 1008 |
| NEW-022 | Low–Med | DNS rebinding passes Origin == Host | REPRODUCED IN TEST | Host allow-list + `SHAPPE_ALLOWED_HOSTS` |
| NEW-023 | Low | Log-line forgery and ESC injection via `/guidance/mark` | REPRODUCED IN TEST | One printable line, 200 chars |
| NEW-024 | **High** (latent) | F17 incomplete: occlusion edges invent forward leverage | REPRODUCED IN TEST (sweep) | Depth-jump and grazing bins excluded; can only fall |
| NEW-025 | Med | Cadence pace unclamped (to 4.4 m/s) | REPRODUCED IN TEST | Clamped to `HEADING_MAX_SPEED_MPS` |
| NEW-026 | Med | Heading state has no age; reused indefinitely | REPRODUCED IN TEST | `updated_at`/`updated_mono`; `current_motion()` ignores stale (`HEADING_STATE_STALE_S`) |
| NEW-027 | Med (**pass-1 regression**) | The stale hold called `admit("")` and wiped the pending sentence: "Walk straight" never said with a scan age near 1 s | REPRODUCED IN TEST | The stale hold no longer reaches the gate |
| NEW-028 | Med (**pass-1 regression**) | The glass notice displaced steering, delayed hazards and chattered (F2) | REPRODUCED IN TEST | Status admission, raw-clear, backoff, cold-start grace only while the radar is still coming up |
| NEW-029 | Med | NEW-001; one stale poll restarted the optional-sensor wait | REPRODUCED IN TEST | `lidar_absent` after `STARTUP_LIDAR_ABSENT_S` (20 s) lets "Sensors offline" / vehicle approach through the hold |
| NEW-030 | Med | Blocked: the camera's turn overrode the rangefinders' (2.2 ft vs 15 ft → "turn left"); unblocked steering allowed towards an UNKNOWN side, and diagonals were never checked | REPRODUCED IN TEST | Unknown = 0 ft; diagonal sector checked; the camera's side only on `safer_side`'s terms |
| NEW-031 | Med | Pi/phone ownership on different snapshots → a cue heard by nobody | REPRODUCED IN TEST (node) | `accepted_seq`; `piHasCue` |
| NEW-032 | Med | Log writes in front of an urgent start (0.3 s write → 0.6 s late); wall-clock sink cache; dummy sinks | REPRODUCED IN TEST | Bounded queue + writer thread; monotonic; `_UNHEARD_SINKS` |
| NEW-033 | Med | Wall-clock pacing: a backward step slept the voice loop / LiDAR analysis / detection | REPRODUCED IN TEST | Monotonic pacing everywhere |
| NEW-034 | **High** impact, low likelihood | A dead sensor revived after a backward clock step: "Walk straight" ~60 s after LiDAR death | REPRODUCED IN TEST | Monotonic receive time; `reading_is_current` |
| NEW-035 | Med | `walk_policy`: simulated or stale LiDAR cancelled a real radar echo; a cleared front board's silence read 9.8 ft clear | REPRODUCED IN TEST | Non-fresh LiDAR is no LiDAR; owed-board rule |
| NEW-036 | Low | `lidar_is_fresh({})` True; front coverage by count | REPRODUCED IN TEST | Fixed |
| NEW-037 | Low | A backward step ≥ 0.5 s → false "Sensors offline" | REPRODUCED IN TEST | A future stamp on a scan received just now is a hold |
| NEW-038 | Low | "All systems normal" beside a warning | REPRODUCED IN TEST | `all_ok` false with any warning |
| NEW-039 | Low | `cleanup_lidar` in the signal handler could leave the motor running | REPRODUCED IN TEST | Motor stopped first; prints guarded |
| NEW-040 | Low | NPU lifecycle: segmenter closed under `run()`; a raising close killed the worker; a stale mosaic was re-detected; engines kept alive; seg view with no segmenter | REPRODUCED IN TEST | Fixed (see F7) |
| NEW-041 | Low (latent Med) | fsync under the tracker lock; stride-file edge cases | REPRODUCED IN TEST | Fixed |
| NEW-042 | Low | Gyro step detector frozen by a backward step; a forward step integrated as yaw (24.9° vs 19.9°) | REPRODUCED IN TEST | Monotonic bookkeeping |
| NEW-043 | Med | A LiDAR returning almost nothing (7 of 164 packets) reads "Walk straight … 40 ft" | REPRODUCED (script) | **CALIBRATION DECISION**: open space also returns little; needs a measured valid-fraction threshold |
| NEW-044 | Low–Med | Unswept front cone: `LIDAR_SCAN_MAX_GAP_DEG` 20° > the 10° cone | REPRODUCED IN TEST | **PATCHED (§10):** a cone with no return that no packet of the revolution swept is unknown, not open. A lost start flag merging two revolutions is not addressed |
| NEW-045 | Design trade-off, **High** impact | Any laser return at a similar range anywhere in the ~113° front arc cancels a glass echo (`walk_policy` / `sensor_fusion._lidar_arc_corroborates`) | REPRODUCED (script); SEEN IN THE FIELD 2026-09-30 (pane ahead at 0.9 ft, wall 1.1 ft to the left: "Turn right — front blocked by an obstacle") | **PARTLY PATCHED 2026-09-30:** an echo nearer than `RADAR_CONE_TRUST_MM` (~1.1 ft) is inside the corridor, so a wall in the arc no longer cancels it and one board is enough ("Blocked by glass"). Still open beyond that range: a pane 1.1–2.5 ft ahead with a wall 1.1 ft beside is not seen. PRODUCT DECISION + field test (match within the live board's beam). **PATCHED 2026-10-01 (desk simulation only, `tools/glass_sim.py`):** each front board's echo is judged against the laser in that board's own view, so a wall beside one board no longer hides the pane the other hears (1.5–4 ft now "Blocked by glass"); and the door-in-wall rule no longer mistakes the door's own wall for a side wall close up (1.5–3 ft). Field test still required |
| NEW-046 | Info | LiDAR-only yaw at rest: 1.3–3.4 °/s at the rig's density (simulation) | MEASURED (sim) | Informs F6, F18 |
| NEW-047 | Calibration | `RADAR_BEARINGS`' hub-port comments no longer match this rig's cabling (5 of 6 boards); no `radar_bearings.json` exists, so the config bearings are live | VERIFIED ON DEVICE (read-only sysfs) | **CALIBRATION REQUIRED**: confirm each board's physical direction (§8) |
| NEW-048 | Info | The dormant "Traffic light RED … Safe to cross!" path needs `SHAPPE_DEBUG=1` + manual override; with no LiDAR the legacy `decisionMaker` returned it into display fields | VERIFIED IN CODE | `decisionMaker` now runs only on a fresh LiDAR; `/navigation/decide` answers `no_lidar_data`; nothing is spoken |
| NEW-049 | Med | "Vehicle approaching, move left/right" checked no rangefinder side room: "move right" with a wall 1 ft to the right, or no reading there | REPRODUCED IN TEST | **DECIDED 2026-09-29 and PATCHED (§10):** new urgent sentence "Vehicle approaching" (no move) when that side has under 2 ft or no reading |
| NEW-050 | Med | The uncommissioned accelerometer's "walking" overrides a measured standstill (ego-discount on, stopped-car "be careful" off) | REPRODUCED (reviewer's script, `VehiclePoller`) | **DECIDED 2026-09-29: kept as is until the gait detector is commissioned** (the procedure is in `config.py`); still a known risk |
| NEW-051 | Low | `radar_calibration.set_bearings()`: an unused public commit-without-re-key API (a trap) | VERIFIED IN CODE | **PATCHED (§10):** renamed `_set_bearings_for_tests`; the unused import is gone |
| NEW-052 | Low | No uvicorn graceful-shutdown timeout; an open stream delays teardown to the 45 s backstop | VERIFIED IN CODE | **PATCHED (§10):** `SERVER_GRACEFUL_SHUTDOWN_S` = 5 s in both launchers |
| NEW-053 | Low | `/classic` loads plotly from a CDN without SRI; `/docs` from jsdelivr | VERIFIED IN CODE | Proposal: self-host or SRI |
| NEW-054 | Info | `GET /guidance/walk` is not read-only: `tools/walk_logger.py` and `tools/bev_check.py` run the whole pipeline when they poll it | VERIFIED IN CODE | Noted; the gate makes it idempotent for speech |
| NEW-055 | Product | F3 fix also drops side-camera lights; `at_crossing` fails open on an exception | VERIFIED IN CODE | **DECIDED 2026-09-29: front camera only**, kept |
| NEW-056 | Low | Radar discovery excluded the LiDAR only by its resolved by-id link (F13 residual) | REPRODUCED IN TEST | **PATCHED (§10):** also excluded by the LiDAR adapter's USB serial |
| NEW-057 | Low | `/lidar/reader` fields while down (F14 residual) | REPRODUCED IN TEST | **PATCHED (§10)** |

---

## 6. Measurements taken in this audit

All hardware-free, reproducible with the scripts and tests named.

**Pre-audit behaviour.** Pass 1's `repro_baseline.py` and `repro_f5.py`, re-run by pass 2 on
a fresh `git archive HEAD`:

```
F3  back-camera-only light -> crossing: True | at_crossing passed to the speaker: True
F10 scan 1.5 s old -> health lidar_ok: True | lidar_is_fresh: False | fused says: LiDAR not reporting
F20 dead board stamped 120 s ahead -> online: True
F4  voice queue after late older cue -> 'Wall ahead'
F11 urgent speak() blocked for 1.50s on the sink probe
F15 header says: ['# this is what was HEARD rather than what some client happened to see.']
F9  no admin token configured -> guarded endpoint OPEN
F5  health lidar_ok: True | lidar_is_fresh: True | spoken: [(1.0, 'Walk straight')]
F21 f21_repro.py: pre-audit exit=124 (hung); current: handler completed, exit 0
```

**Pass 1's own regressions**, from tests on the tree pass 1 left:

```
NEW-027 scan age 1.05 s on 1 poll in 4:  ['Blocked by wall']   (steady: Walk straight, Blocked by wall, Walk straight)
NEW-028 radar lost, wall appears as the hold lifts: (5.0,'Glass sensor off'), (8.0,'Wall ahead')  (radar OK: 5.2)
NEW-028 one 15 s dropout a minute, 10 min: 19 notices (now <= 8, one "Walk straight")
NEW-028 cold boot: ['Glass sensor off','Walk straight','Glass sensor on','Walk straight']  (now ['Walk straight'])
NEW-034 LiDAR dies, clock steps back 60 s: ['Walk straight','Sensors offline','Sensors offline','Walk straight', ...]
```

**Odometry** (`tools/odometry_density.py`, now with a random bearing phase per revolution;
2 s walks; ±2.5 m corridors):

| Scene | Beams / bins | Result |
|---|---|---|
| Widening corridor, walking 1.2 m/s, 36 cases | 1080/1°, 1080/2°, 250/2°, 164/2° | reported `confidently_still` 19/36, 10/36, 10/36, 11/36 → **0/36** after NEW-024 |
| Back wall / room | 1080 / 1° | pace 1.19–1.20 m/s; standing correctly still (unchanged) |
| Back wall / room | 250, 164 / 2° | pace 1.17–1.19 m/s (unchanged) |
| Turning 20 °/s on the spot / standing still | 164 / 1°, phase jittered | 21.2 °/s / **3.4 °/s** (the phase-locked 30.0 °/s was aliasing) |
| Standing still | 250 / 1° · 250 / 2° · 164 / 2° | 1.3 · 0.7 · 1.5 °/s of yaw |

**Speech chatter (F16):** a 1.4 s clear every 3.9 s gives 10 announcements in 38 s; a 1.0 s
clear gives 2 (pass 1).

---

## 7. Remaining limitations and physical validation

A software patch cannot validate any of these:

- Actual LiDAR RPM and points per revolution at the current setting (F14, F6).
- Head-on glass detection (FT-1.5b), and the radar's real echo dropout off glass. NEW-002 assumes 35 %.
- **Each radar board's physical direction.** The config's cabling notes no longer match the hub (NEW-047).
- A radar re-enumeration: replug two boards and confirm each keeps its own direction (NEW-015).
- IMU mounting, axis and sign (`GYRO_MOUNTING_CONFIRMED`), and gait commissioning (`GYRO_GAIT_CONFIRMED`).
- Heading and pace on measured walks (F6, F17, NEW-024).
- The cameras' real horizontal field of view. `SIMPLE_CAMERA_HFOV_DEG = 70` is assumed and drives every camera range estimate and the vehicle logic.
- The BEV physical checklist (`tools/bev_check.py --checklist`).
- Audio through the intended earpiece. Today the only sink is HDMI.
- The phone's announcement rule (`piHasCue`) in a real browser with a screen reader.
- Cold-boot timing of front radar coverage, to set `SIMPLE_GLASS_STARTUP_GRACE_S`.
- NPU recovery from a real `HAILO_OUT_OF_PHYSICAL_DEVICES` or a stalled device, with the new 2 s detector timeout.
- The planar LiDAR sees one plane: overhead and below-plane obstacles are not covered. The 3D perception plan is research, not this product.
- The traffic-light detector reports presence only. Nothing in the product may say it is safe to cross.
- Dashboards were checked by static analysis and node harnesses, not in a real browser.

Every physical check must be done on a bench or under supervision. No blind participant
should depend on behaviour that has not been validated.

---

## 8. Approval-gated actions (not done)

| Action | Why | Operational risk |
|---|---|---|
| Review and commit this working tree | Nothing has been committed or pushed. On 2026-09-29 the owner chose **not to commit yet** | Commit after review only |
| Restore `.github/workflows/tests.yml`: `git show 24f5a95:.github/workflows/tests.yml > .github/workflows/tests.yml` | Without CI the 1075 tests can be deleted again unnoticed | Reverses the owner's `1921ac3` deletion |
| Reinstall the unit (`./deploy/install-service.sh`) to pick up `LimitCORE=0` | NEW-020 | Installs, enables and starts a boot service (it is not installed now) |
| Bench run, **nobody wearing it**: `/lidar/reader` (`rpm`, `points_per_scan`, `pwm_command`), `/radar/health`, `/camera/hailo`, `/camera/stats` with and without `/camera/seg_grid`, `tools/hailo_monitor.py` | F6, F12, F14 need measured numbers | Spins the LiDAR at the unconfirmed PWM mapping and opens every sensor |
| Supervised radar direction check: a hand in front of each board on `/radar/test`, one at a time | NEW-047: the cabling notes are stale | A wrong bearing announces glass in the wrong direction; supervised only |
| Supervised replug tests: LiDAR (F13), two radar boards (NEW-015) | Identity recovery | Sensor disconnection; never while worn |
| Time a cold boot's front radar coverage; set `SIMPLE_GLASS_STARTUP_GRACE_S` | NEW-028's grace is an assumption | None |
| Motor start current and brown-out margin; confirm 900 RPM | F14 | Motor experiment |
| Commission gait, then yaw (procedures in `config.py`) | F19, F18, F6, NEW-050 | A wrong axis or sign inverts steering |
| `HEADING_BIN_DEG = 2.0` after measured walks | F6 (NEW-024 now fixed) | Changes pace, stillness and the vehicle and drift sentences |
| HailoRT `ConfiguredInferModel.shutdown()` in `close()`; discard a dead shared VDevice after N device-level failures with no engine alive | F7 residual | Needs hardware verification that it returns promptly and a VDevice reopens in-process |
| **Product decisions still open:** F16 episode minimum-clear time (needs a recorded walk); F18 veer sentence (needs the gyro commissioned); NEW-045 glass-echo arc matching (needs a field test); NEW-043 sparse-return threshold (needs `/lidar/reader`'s `packets_per_scan` / `points_per_scan` measured outdoors). Decided on 2026-09-29: NEW-049 implemented; NEW-050 and NEW-055 kept | Each trades one failure for another | User-facing behaviour |
| **Review the user-facing changes made here:** "Glass sensor off/on" wording, timings and backoff (F2 / NEW-028); "Sensors offline" through the start-up hold (NEW-029); camera steering vetoes (NEW-030); the phone speaking per cue (NEW-031); "Vehicle approaching" with no move (NEW-049) | They change what the wearer hears | Field test under supervision |
| Self-host `/classic`'s plotly (or add SRI) | NEW-053: needs the file fetched from the CDN into the repository | Low |
| Connect and validate the intended earpiece | F11, F15, NEW-031 | None |
| Deploy to a worn device | — | Only after all of the above, in a supervised field test |

`LIDAR_MIRROR`, the camera quadrant layout, `GYRO_MOUNTING_CONFIRMED`,
`GYRO_GAIT_CONFIRMED`, `SEG_CAN_STOP`, `SEG_ALWAYS`, `COMPANION_VOICE_ENABLED`,
`ENV_TRUST_ENABLED`, motor RPM/PWM, radar bearings, detection thresholds and the model
files were **not** changed.

---

## 9. Review of pass 2's own changes

An independent reviewer read the whole pass-2 diff, pass 1's tree snapshot against the
current tree, after the 1075-test run. It re-derived the changes to every pre-existing
test file and judged each legitimate (§2). It confirmed:
- `_forward_sensitivity` never rises: 3000 random and structured profiles, max(new − old) = 0.0;
- the lock orders above;
- the WebSocket and Host rules against the real hostname;
- the voice queue has no lost wakeup.

It found the following, all now fixed except where noted. Each fix has a test that was
shown to fail without it.

| # | Sev. | Finding | Fix |
|---|---|---|---|
| R1 | **High** (narrow window) | **Introduced by pass 2's NEW-027.** A "Walk straight" kept pending through a stale-scan hold was committed on the first fresh poll, whose raw reading was already a wall 3 ft ahead: `(8.8, 'Walk straight', raw='Blocked by wall')`, then the block 0.6 s later. The same race existed on fresh scans, but for one poll at the confirm instant | A clear-path sentence is never committed on a poll whose raw decision is blocked, names something near ahead, or is a vehicle; it stays pending. The reviewer's scripts now give `(9.0, 'Blocked by wall')` with no all-clear before it, and 8.0/8.2 s on fresh scans. A side-wall "Drifting" in the raw poll does not hold it: the first version did, and 6 historical tests caught that |
| R2 | Med | NEW-018 was incomplete: `simple_guidance._camera_sees` ignored `simulated`, so simulated people were steered round ("Walk between them") | `_camera_sees` is False for a simulated front frame |
| R3 | Med | The cold-start grace delayed "Glass sensor off" by a fixed 30 s even when discovery had finished without the front boards; the wearer got ~20 s of guidance first | Grace only while the radar is still coming up (`front_glass_coverage` `settling`), capped at 30 s |
| R4 | Low | Three glass-notice tests (two of pass 2's, one of pass 1's) could no longer fail: with the grace, no notice was ever pending | The grace is patched to 0 in them. The rules were shown to hold with the notice pending |
| R5 | Low | A side view missing altogether read as open for camera steering | Missing is no room, the same as unknown |
| R6 | Low (fails safe) | `_node_for` accepted a board's own second interface (if02, no server) if a hub reset gave it the old if00 number, so the board stayed offline on every retry | The shortcut applies to the first interface only; otherwise the by-id `-if00` link is used. The mutation run opens `ttyACM90` without the fix |
| R7 | Low | After 8 s of urgent cues cutting each other off, the Pi marks itself failing and the phone speaks too | Documented "both, never neither" behaviour; unchanged |
| R8 | Low | `/guidance/mark`'s "last said" named the last *guidance* sentence even when a status notice followed | Uses the sentence the cue counted (`state()["counted"]`) |
| R9 | Low | Not fixed. The gyro's yaw buffer is wall-clock, so after a backward step `yaw_between` returns None for up to 12 s (heading degrades to LiDAR-only). A few small non-safety constants are outside `config.py` (`_REPORT_QUEUE_MAX`, the 2 s reap wait, the 2× airtime factor) | Noted |

After these fixes: **1084 tests, OK (75.0 s)**.

---

## 10. Follow-ups: the remaining items that needed no decision and no hardware

Done after the owner asked for the remaining items to be completed. Every item has a test
in `tests/test_followups.py` (12 tests). Each failed before its fix. The two tests
adjusted during the work were mutation-checked: with the fix disabled they fail again.

| # | Change | Where |
|---|---|---|
| NEW-044 | The stream records every packet's bearing of the revolution it yields (`_ScanStream.swept`). A steering cone with no return that **no packet swept** is marked `unswept`, and `walk_policy` reads it as no information, not `LIDAR_NO_RETURN_FT`. No threshold: at any spacing the device can have, a swept 10° cone holds a packet. A cone with a return keeps it | `utils/lidar_utils.py` (`_mark_unswept`), `utils/walk_policy.py` |
| NEW-056 | Radar discovery also excludes any node whose USB serial is the LiDAR adapter's (read from the by-id name in config, and from what it resolves to), so a re-plugged LiDAR is never probed before udev has made its link | `utils/radar_utils.py` (`_lidar_serials`) |
| NEW-057 | `/lidar/reader`: `pwm_command` only once it has been sent (a failed ramp reports None; a successful re-assert records it), cleared when a session ends; `connected`; `packets_per_scan` beside `points_per_scan` (valid returns), the measurement NEW-043's threshold would be set from | `utils/lidar_utils.py` |
| NEW-051 | `set_bearings()` renamed `_set_bearings_for_tests()`, documented as test-only; removed from `radar_utils`' imports. The one way to change a bearing is `POST /radar/calibrate` → `apply_calibration` | `utils/radar_calibration.py`, `utils/radar_utils.py`, two test files (a call rename only) |
| NEW-052 | `--timeout-graceful-shutdown` in `start_lidar.sh`, `timeout_graceful_shutdown` in `main.py`, from `SERVER_GRACEFUL_SHUTDOWN_S` (5 s) | `config.py`, `main.py`, `start_lidar.sh` |
| F7 | The segmenter's `run()` is bounded like the detector's: `HAILO_SEG_RUN_TIMEOUT_MS` (2000 ms; ~50 ms per frame now, 262 ms on the slowest model run here) | `config.py`, `utils/seg_inference.py` |
| F20 (R9) | After a backward wall-clock step the gyro's yaw buffer restarts, so `yaw_between()` answers on the new clock at once instead of after up to 12 s | `utils/gyro_reader.py` |
| R9 | `PI_VOICE_REPORT_QUEUE_MAX`, `PI_VOICE_REAP_WAIT_S` moved to `config.py` | `config.py`, `utils/pi_voice.py` |
| NEW-049 (decided) | "Vehicle approaching" with no move when the side it would send the user to — away from the vehicle — has under `SIMPLE_STEER_SIDE_MIN_FT` or no reading, including while the LiDAR is offline. Level 3, said over everything, like the other vehicle warnings. Room is judged on each poll's readings for whichever side is held. 5 tests; one vocabulary-set line in `test_simple_guidance` | `utils/simple_guidance.py` (`vehicle_move_blocked`), `README.md` |

**Still not done, and why:**
- **Needs hardware or a person present:**
  - every physical check in §7;
  - HailoRT `shutdown()` in `close()`, and discarding a dead shared VDevice;
  - installing the unit, which starts the app and so the LiDAR motor.

  None of these was run, because nobody could confirm the device was not being worn, and the app must not be started unasked.
- **Needs a fetch into the repository:** NEW-053, self-hosting plotly.
- **Needs a decision or field data** (§8): F16, F18, NEW-043, NEW-045, and restoring CI. NEW-049 was decided and done; NEW-050 and NEW-055 were decided and kept; the owner chose not to commit yet.

---

## Appendix — files changed

**Pass 1** (see its finding entries): `utils/freshness.py` (new), `shared_state`,
`lidar_utils`, `sensor_fusion`, `walk_policy`, `simple_guidance`, `unified_guidance`,
`pi_voice`, `speech_log`, `heading_tracker`, `gait_calibration`, `service`, `access`,
`main.py`, `pi_camera_utils`, `hailo_device`, `hailo_inference`, `seg_inference`,
`radar_calibration`, `radar_utils`, `routers/*`, `templates/*`, `config.py`,
`start_lidar.sh`, `README.md`, `tools/odometry_density.py` (new), 33 restored + 11 new test
modules.

**Pass 2** (41 tracked files differ from pass 1's tree, plus 7 new test modules):

| Area | Files | Change |
|---|---|---|
| Speaking path | `utils/simple_guidance.py`, `utils/startup_gate.py`, `utils/unified_guidance.py` | NEW-026–030, 034, 037; status admission; `lidar_absent`; `current_motion`; legacy `decisionMaker` gated |
| Voice | `utils/pi_voice.py`, `templates/dashboard.html`, `templates/index.html` | NEW-031/032/033; `piHasCue`; the crossing display reads `at_crossing`; `/classic` tiles show "--" without LiDAR |
| Logs | `utils/speech_log.py`, `utils/test_console.py`, `routers/guidance_router.py` | One printable line; docstrings (F15); mark sanitising |
| Freshness and fusion | `utils/freshness.py`, `utils/shared_state.py`, `utils/sensor_fusion.py`, `utils/walk_policy.py` | Receive time; `reading_is_current`; owed boards; `all_ok`; camera liveness and simulation |
| LiDAR | `utils/lidar_utils.py` | Monotonic pacing; `cleanup_lidar` |
| Radar | `utils/radar_utils.py`, `utils/radar_calibration.py`, `routers/radar_router.py`, `templates/radar_test.html` | NEW-015/016/017 and the LOW fixes |
| NPU and camera | `utils/pi_camera_utils.py`, `utils/hailo_inference.py`, `utils/seg_inference.py`, `routers/pi_camera_router.py` | NEW-018/019/040 |
| Motion | `utils/heading_tracker.py`, `utils/gait_calibration.py`, `utils/gyro_reader.py`, `tools/odometry_density.py` | NEW-024/025/026/041/042 |
| Security and deploy | `main.py`, `utils/access.py`, `routers/navigation_router.py`, `deploy/shappe-guide.service`, `deploy/install-service.sh`, `start_lidar.sh` | NEW-020–023, 048 |
| Config and docs | `config.py`, `README.md`, `docs/SHAPPE_GUIDE_AUDIT.md` | Every new threshold, documented: `SIMPLE_GLASS_RECOVER_MAX_S`, `SIMPLE_GLASS_STARTUP_GRACE_S`, `STARTUP_LIDAR_ABSENT_S`, `HEADING_STATE_STALE_S`, `HEADING_MAX_INCIDENCE_SEC`, `HEADING_SENS_JUMP_FRACTION`, `HAILO_DETECT_RUN_TIMEOUT_MS`, `CAMERA_SIMULATION_ALLOWED`, `ALLOWED_HOSTS_TEXT`, `MARK_NOTE_MAX_CHARS` |
| Tests | `tests/test_{speech,voice,sensor,radar,npu,security,motion}_pass2.py` (new, 179 tests), plus the eight files in §2 | — |
