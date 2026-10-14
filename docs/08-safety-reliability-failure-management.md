# 08 — Safety, Reliability and Failure Management

> **This is an assistive prototype. It is not validated, certified or cleared as a medical or assistive device.** Nothing here claims compliance with any standard. Standards are listed for *scope and planning* only. No blind participant may depend on behaviour that has not been validated under supervision. The 2026-09-29 audit (`docs/SHAPPE_GUIDE_AUDIT.md` §7) says the same.

Status labels as in [01](01-project-overview.md).

---

## 1. Fundamental architecture rules

1. **The device owns safety.**
   - Sensing, hazard assessment and the decision to warn happen on the Raspberry Pi (CONFIRMED).
   - The phone is a **companion**: it relays and displays, and it never decides and never is the only warning channel (PROPOSED rule, ICD §8 A8).
2. **Failure is loud, not silent.** Every loss of a safety function must be announced:
   - "Sensors offline" (CONFIRMED);
   - "Glass sensor off" (CONFIRMED);
   - "Camera off" for the front camera: feed lost, detector stopped or a black picture (CONFIRMED in code 2026-10-09, desk-tested);
   - camera/NPU warnings in `/health` (CONFIRMED in this checkout);
   - link loss on the phone (PROPOSED).
3. **No positive safety claims.** Nothing says "clear", "safe" or "safe to cross". "Walk straight" is only said after confirmation, and is withheld at crossings and when stale (CONFIRMED).
4. **Uncertain means quiet.** Stale or simulated data never produces guidance (CONFIRMED).
5. **The companion link cannot change guidance.** No v1 BLE command alters thresholds or mutes the device (PROPOSED, enforced in the prototype).

## 2. Hazard identification

| ID | Hazard | Situation | Current coverage |
|---|---|---|---|
| H1 | Collision with a wall or large object | Walking into a structure | LiDAR, blocked < 4 ft (CONFIRMED) |
| H2 | Collision with glass | Glass door or wall | Radar/LiDAR disagreement (CONFIRMED in code); **head-on unproven** (NEEDS VALIDATION) |
| H3 | Collision with a thin object (pole, sign post) | Narrow return | LiDAR corridor + cluster rule; thin poles may give 1–2 returns (NEEDS VALIDATION) |
| H4 | Head-height obstacle | Branch, mirror, scaffold | **Not detected** (NOT IMPLEMENTED) |
| H5 | Drop-off | Stairs down, kerb, platform edge, hole | **Not detected** (NOT IMPLEMENTED) |
| H6 | Moving vehicle / bicycle | Driveway, shared path, crossing | Front camera approach (CONFIRMED); LiDAR movers silent (trial) |
| H7 | Moving person | Crowds | LiDAR movers silent; static "Person ahead/in front" (CONFIRMED) |
| H8 | Road crossing | Deciding when to cross | **Deliberately unsupported**; "Walk straight" withheld near a seen traffic light (CONFIRMED) |
| H9 | Wrong instruction | Bad turn advice, wrong side | Turn hysteresis and room check (CONFIRMED); radar bearings must be measured |
| H10 | Missing or late instruction | Latency, gating, link loss | Gate timing (CONFIRMED); **no local audio today** (gap) |
| H11 | Over-alerting and habituation | Too many sentences | Closed vocabulary, gaps, repeat back-off (CONFIRMED) |
| H12 | Masking environmental sound | Earphones blocking traffic sound | PROPOSED: bone-conduction or open audio |
| H13 | Device failure unnoticed | Crash, sensor loss, heat, power | Watchdog, `/health`, spoken offline notices (CONFIRMED); phone events (PROPOSED) |
| H14 | Burns, battery fire, laser | Hardware | LiDAR is Class 1 [S9]; battery NOT IMPLEMENTED (pack selection must follow §9) |
| H15 | Privacy harm | Images of bystanders, location of a vulnerable user | No storage found; LAN streams unauthenticated (§8) |

## 3. Sensor limitations and environmental conditions

| Condition | LiDAR [S9, S11] | Cameras | Radar [S12, S14] | Net effect |
|---|---|---|---|---|
| Strong sunlight | "Outdoor … without direct sunlight", so range and dropouts suffer | Glare; shared auto-exposure darkens other views | Unaffected | Fewer returns. **NEEDS VALIDATION** outdoors |
| Low light / night | Unaffected | Detection degrades | Unaffected | Object names unreliable |
| Rain, wet surfaces | Wet and dark surfaces have shorter range; puddles give specular reflection | Droplets on lenses | Rain clutter possible; the code has no rain handling | NEEDS VALIDATION. `ENV_TRUST_ENABLED=False` (its fog detector misfired) |
| Reflective or transparent | Glass and mirrors give missing or ghost points | May see through glass | **Main purpose** | Covered in front; head-on unproven |
| Crowds | Many movers; heading odometry degrades with a crowd moving with the user (README) | Many boxes | Many echoes | More "blocked"; movers silent |
| Stairs and drop-offs | One plane: invisible below | COCO has no stairs class | Not used | **Not covered** |
| Fast vehicles | 15 Hz scans; movers horizon 5 s | Front only; needs ≥ 3 frames | No velocity in the distance detector | Late or missed for fast side traffic |
| Cold or heat | 0–40 °C operating [S9] | — | −40…85 °C board [S18] | The Pi throttles at 80–85 °C; AI HAT+ 2 ambient ≤ 50 °C [S26] |

## 4. The critical gap: no local alert output

| Fact | Status |
|---|---|
| `AUDIO_ENABLED = False`: no sound anywhere (Pi or browser) | CONFIRMED (config.py) |
| The only audio sink is HDMI; `pi_voice` refuses HDMI sinks on purpose | CONFIRMED (`wpctl status`) |
| There is no haptic motor, buzzer or speaker | CONFIRMED (no code, no hardware) |
| Only the Wi-Fi dashboard can speak today (when audio is enabled) | CONFIRMED |

**Consequence.** If the phone is disconnected, locked, in a call, out of battery or force-quit, **the user receives no warning at all**. That contradicts rule 1.

**Proposed remedy:**

| Option | Pros | Cons | Recommendation |
|---|---|---|---|
| **USB audio adapter + wired bone-conduction headset** | Independent of every radio; leaves the ears open; `pi_voice` works unchanged (a USB sink is "usable") | A cable to the head | **Primary** |
| Bluetooth A2DP headset paired to the Pi | No cable | Shares the CYW43455 with BLE and Wi-Fi (coexistence NEEDS VALIDATION); 100–300 ms A2DP latency (typical, NEEDS VALIDATION); its own battery | Secondary |
| I²S DAC + small speaker | Simple | Not private; inaudible in traffic | Not recommended |
| Haptic driver (I²C) + 2 LRA motors (left/right shoulder) | Directional, silent, works during phone calls | New hardware and code | **Add** as a second channel |

**Acceptance (PROPOSED):**
- A level-2+ instruction is audible on the device within 600 ms p95 with **no phone present**.
- A link loss never mutes the device.
- Verified by the procedure in [03 §9](03-perception-and-navigation-pipeline.md).

## 5. Degraded operating modes (CONFIRMED device behaviour + PROPOSED app behaviour)

| Mode | Entered when | Device does | Phone shows / says |
|---|---|---|---|
| Normal | `/health` ok | Full guidance | "All sensors working" |
| No glass detection | Front radar coverage lost | "Glass sensor off"; LiDAR-only | CAUTION "Glass detection off" |
| No object names | Camera or NPU failure, or a black front picture | "Camera off"; geometry-only sentences; no vehicle warnings | CAUTION "Object names off" |
| No geometry | LiDAR lost, or its newest scan older than 2 s | "Sensors offline"; camera vehicle warnings only | SYSTEM FAILURE "Shappe cannot see. Use your cane." |
| Service down | Crash or restart | Silent until restart and the start-up gate | SYSTEM FAILURE "Shappe guidance stopped. Use your cane." |
| Thermal | ≥ 80 °C or throttling | Slower detection | WARNING |
| Link lost | No STATE 3 s | Device unaffected | WARNING "Lost contact with Shappe. Use your cane." |
| Simulated | Simulation flags | Warning "SIMULATED — do not navigate" | Persistent "Demo mode — not real" |

## 6. Preliminary FMEA

Severity (S), occurrence (O) and detectability (D) are 1–5 qualitative estimates **for prioritisation only**. They are not measured. RPN = S × O × D.

| Failure | Possible cause | User impact | Detection | Mitigation (status) | Verification | S | O | D | RPN |
|---|---|---|---|---|---|---|---|---|---|
| No warning delivered (link down, phone silent) | No local audio; phone locked, in a call or force-quit | Collision | Link-stale timer (PROPOSED) | **Fit local audio + haptics** (PROPOSED §4) | No-phone test, §4 acceptance | 5 | 4 | 4 | **80** |
| Drop-off not warned | One-plane LiDAR; no class | Fall | None | Cane remains primary; future depth/downward sensor (PROPOSED [09 §8](09-roadmap-and-integration-checklist.md)) | Field test with kerbs | 5 | 3 | 5 | **75** |
| Head-height obstacle not warned | One plane | Head injury | None | As above | Field test | 4 | 3 | 5 | 60 |
| Glass not detected head-on | Specular radar dropout; PLANAR on trial | Collision | Radar health only | Per-board judgement, door-in-wall rule (CONFIRMED); field test | Supervised glass test (audit FT-1.5b) | 5 | 3 | 4 | 60 |
| Vehicle approach missed | Side or rear traffic (front camera only); low light | Collision | None | Movers (silent trial), radar movers; never claim crossing safety | Supervised roadside test, no crossing | 5 | 3 | 4 | 60 |
| Stale alert spoken late on phone | BLE congestion, app suspended | Wrong action | `age_ms` + local age > 2 s → drop (PROPOSED A3) | ICD A3 | Delay-injection test | 4 | 2 | 2 | 16 |
| Duplicate or old alert replayed after reconnect | Reconnect reads | Confusion | REPLAY flag, (epoch, cue) dedupe (PROPOSED) | ICD A1/A2 | `service_crash` scenario | 3 | 3 | 2 | 18 |
| Wrong turn side | Wrong radar bearing; wrong camera channel map | Collision | `/radar/test` | Measured bearings by serial (CONFIRMED); NEW-047 stale notes | Supervised hand test per board | 4 | 2 | 3 | 24 |
| False "blocked" or chatter | Crowds, own cane or arm | Annoyance, ignoring alerts | Speech log, tester marks | Gate, cluster rules, 300 mm mover floor (CONFIRMED) | Recorded walks with `mark` | 3 | 3 | 2 | 18 |
| Navigation service crash | Software fault | Silence | systemd watchdog (CONFIRMED); bridge `service_unreachable` (PROPOSED) | `Restart=always`; phone SYSTEM FAILURE | Kill test | 5 | 2 | 2 | 20 |
| Guidance loop hangs while process alive | Deadlock | Silence | Watchdog withholds pings on a stale heartbeat (CONFIRMED) | Restart | Fault injection | 5 | 1 | 2 | 10 |
| LiDAR stops | USB, motor, power | No geometry | Freshness → "Sensors offline" (CONFIRMED) | Soft start; spoken offline notice | Unplug test (not worn) | 4 | 2 | 1 | 8 |
| NPU fails | Driver, heat | No names | `hailo_ok` (CONFIRMED) | Retry back-off (CONFIRMED) | Fault injection | 3 | 2 | 1 | 6 |
| Overheat throttling | Enclosure, sun | Slow detection | Bridge thermal events (PROPOSED) | Ventilation, heatsink, Active Cooler | 2 h hot-room run | 3 | 3 | 2 | 18 |
| Brown-out | Weak supply, LiDAR inrush | Reboot, silence | Under-voltage bit (PROPOSED) | 5 A supply, soft start (CONFIRMED) | Power meter, cold starts | 4 | 2 | 3 | 24 |
| Battery depleted | No gauge | Sudden shutdown | **None** (NOT IMPLEMENTED) | Fuel gauge + `battery_*` events (PROPOSED) | Endurance test | 4 | 3 | 5 | 60 |
| Spoofed device or MITM | Just Works pairing | False info on phone | `device_id` check (PROPOSED) | Passkey pairing, button window (PROPOSED) | Security test | 4 | 1 | 4 | 16 |
| Bystander privacy breach | LAN MJPEG streams unauthenticated | Privacy | None | Bind streams to localhost or authenticate (PROPOSED §8) | Network scan | 3 | 2 | 3 | 18 |

## 7. Verification of safety behaviour (summary; full plan in [09 §5](09-roadmap-and-integration-checklist.md))

- **Desk:**
  - `tools/glass_sim.py`, `tools/movers_sim.py`, `tools/drift_sim.py`, `tools/bev_check.py` (CONFIRMED present);
  - BLE: `ble_bridge/test_ble_bridge.py` (34 tests);
  - real walks replayed: record a walk (`SHAPPE_RECORD_WALK=1` or `POST /guidance/record {"on": true}`), then `tools/walk_replay.py logs/walks/<file>.jsonl.gz` feeds it through the current code and lists every sentence that would now be said differently (added 2026-10-09; desk-tested, 100% replay fidelity on a simulated walk).
  - The repository's own `tests/` suite is **not in this checkout** (gitignored, absent). The audit counted 1075 tests in `~/lidar`; restoring CI is an open audit action.
- **Bench (nobody wearing it):** the audit §8 bench list:
  - LiDAR RPM and points per scan;
  - radar health;
  - NPU recovery;
  - per-board radar direction.
- **Supervised field:** glass head-on, kerbs and stairs (to document the gap), roadside vehicles (never crossing), crowds, sun and rain.

Every session is logged with the speech log and `mark` (marks also go into a walk recording).

## 8. Privacy and data handling

| Topic | Current state (CONFIRMED) | Rule (PROPOSED) |
|---|---|---|
| On-device processing | All perception runs on the Pi | Keep it |
| Raw image retention | No code saves frames to disk. Frames live in memory | Never store frames by default. A field-test capture needs explicit opt-in and consent from bystanders where required |
| Video transmission | MJPEG over HTTP on the LAN (`/camera/video_feed`, `/camera/stream/{sector}`) to anyone who can reach port 8000. Host/Origin checks stop browser attacks, **not** direct access | Bind camera endpoints to localhost, or require the admin token, before any public use. **Never over BLE** |
| Audio recording | None | Keep it |
| Location | None | Keep it. GPS is out of scope for v1 |
| Diagnostic logs | Speech log in memory; a file only when `SPEECH_LOG_TO_FILE`; `walk_logger` CSV for field tests | Field-test logs stored locally, deleted after analysis; no faces or images |
| BLE contents | — | Phrase ids, distances, health. No images, no location. Encrypted link |
| Pairing removal and reset | — | "Forget device" on the phone, plus a device-side factory reset that removes all bonds and the state file |
| Phone app | — | Preferences local; logs opt-in, 7-day retention, user-deletable; no analytics without consent |
| Admin token | `SHAPPE_ADMIN_TOKEN` in `~/.config/shappe-guide.env`, required for calibration, rediscover and stride | Never sent over BLE |

## 9. Standards and regulation (scope only, not a compliance claim)

| Area | Standard / rule | Why it may matter |
|---|---|---|
| Risk management | ISO 14971:2019 | Framework for this FMEA if the device is treated as a medical device |
| Usability | IEC 62366-1:2015+A1:2020 | Use-error analysis for blind users |
| Software lifecycle | IEC 62304:2006+A1:2015 (Ed 2 not yet published) | Software safety classes and process |
| Quality system | ISO 13485:2016 | If commercialised as a medical device |
| Medical device status | EU MDR 2017/745 Art 2(1) ("alleviation of, or compensation for … disability"); US 21 CFR 886.5900 "Electronic vision aid" (Class I, 510(k)-exempt subject to 886.9) | Intended-use wording determines status; **get regulatory advice before any claims** |
| Assistive product classes | ISO 9999:2022 | Classification |
| Electrical safety | IEC 62368-1:2023 | AV/ICT equipment safety |
| Laser | IEC 60825-1:2014 | RPLIDAR is Class 1 [S9]; keep the vendor housing intact |
| Radio | EU RED 2014/53/EU (+ Delegated Reg. 2022/30 cybersecurity, EN 18031-1/-2/-3 applicable from 2025-08-01); ETSI EN 300 328 (2.4 GHz); FCC Part 15 (§15.247) | Bluetooth/Wi-Fi radios; 60 GHz radar modules need their own conformity |
| Batteries | UN 38.3; IEC 62133-2:2017+A1:2021 | Any lithium pack |
| Accessibility of the app | WCAG 2.2; EN 301 549 (V4.1.1, 2026); EU Accessibility Act 2019/882 (applies from 28 Jun 2025) | App and documentation accessibility |
| Human testing | Declaration of Helsinki (rev. Oct 2024); ISO 14155:2026 (if a clinical investigation) | Informed consent in accessible formats; witnessed consent where needed |

## 10. Reliability measures already in the code (CONFIRMED)

- **systemd user service:** `Type=notify`, `WatchdogSec=30`, `Restart=always`, `LimitCORE=0`.
- **Heartbeats:** for "guidance" and "event_loop"; watchdog pings are withheld when a heartbeat is stale.
- **Shutdown backstop:** `os._exit` if shutdown hangs.
- **Freshness everywhere:** wall clock **and** monotonic receive time; 0.5 s clock-step tolerance.
- **Radar resilience:** per-board reconnect loop, stall guard, identity by USB serial.
- **Camera resilience:** corrupt-frame detection with restart and front-only fallback.
- **NPU resilience:** retry back-off; 2 s inference timeout.
- **LiDAR resilience:** motor soft start; PWM re-assert every second; port selected by device path, not enumeration.
- **Web security:**
  - admin token for calibrate, rediscover and stride;
  - cross-site write and WebSocket refusal;
  - Host allow-list against DNS rebinding;
  - anti-framing headers.
