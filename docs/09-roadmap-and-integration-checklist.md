# 09 — Development Roadmap, Responsibilities, Testing and Integration Checklist

**Principle:** get one reliable, minimal, end-to-end path working first:

> device decision → ALERT over BLE → phone speaks it once, fresh, in the background

Then harden it. Everything else waits.

---

## 1. Where things stand (2026-10-09)

| Area | State |
|---|---|
| Perception, decision, speech gate, health | CONFIRMED in code; physical validation pending (audit §7) |
| Local audio | **Absent** (HDMI only, `AUDIO_ENABLED=False`) |
| BLE protocol + bridge | PROPOSED; prototype `ble_bridge/` desk-tested (34 tests), `--check` passes, **not yet over the air** |
| Mobile app | NOT IMPLEMENTED |
| Battery, haptics, button | NOT IMPLEMENTED |
| Repo hygiene | 17 modified + 3 untracked files from the movers/camera work are uncommitted. `ble_bridge/` and the docs are new and uncommitted. No `tests/` directory in this checkout |

## 2. Phases

| # | Phase | Objective | Prerequisites | Main tasks | Owner | Deliverables | Acceptance criteria | Risks | Depends on |
|---|---|---|---|---|---|---|---|---|---|
| 1 | Research and audit | Know what exists | Repo access | Read every file; inventory the hardware | HW + Mobile | Docs 01–03, 10 | Team agrees the docs match the code | Drift between docs and code | — |
| 2 | Hardware verification | Replace assumptions with measurements | Bench; nobody wearing it | Camera HFOV per camera; LiDAR RPM and points per scan; radar bearings (hand test); camera port check; power draw; 2 h thermal run | HW | Measured table replacing the NEEDS VALIDATION entries in 02 | Every NEEDS VALIDATION value in 02 §3–§8 has a measurement | Motor/brown-out events | 1 |
| 3 | Pipeline documentation | Shared understanding of the decisions | 1 | Review 03 with the code owner | HW | Signed-off 03 | No open "unclear" items | — | 1 |
| 4 | Protocol design | Freeze ICD v1 | 1, 3 | Review 04/06; decide the stack (§9 Q1); review the phrase registry | Shared | ICD v1.0, schema, vectors | Both sides sign off; vectors pass in the mobile decoder | Late protocol changes | 3 |
| 5 | Pi BLE prototype over the air | The bridge serves a phone | rfkill unblock (supervised); 4 | Run `--mock` then live; check MTU, pairing and notifications with nRF Connect/LightBlue and `btmon`; fix issues | HW | Working bridge; `btmon` traces | Mock: all 5 scenarios observed on iOS and Android reference phones. Live: STATE at 1 Hz, ALERT on each cue step | iOS pairing trigger (0x0F); coexistence | 4 |
| 6 | Mobile BLE prototype | App connects, decodes, displays | 4 (vectors) | Scan by UUID; bond; subscribe; decoder; state machine; FakeDeviceLink | Mobile | App build | 16/16 vectors decode; connect, reconnect and dedupe work with the mock | Platform BLE quirks | 4 (can start before 5) |
| 7 | End-to-end events | Real device → phone speech | 5, 6 | Live bridge + app; latency logging; `alert_ack` | Shared | E2E test report | Alert → phone audio ≤ 900 ms p95 (level ≥ 2), measured as in 03 §9; 0 duplicates in 30 min | Polling latency | 5, 6 |
| 8 | Accessibility and alerts | Usable without sight | 7 | Screens S0–S8; alert categories; haptics; audio-session handling | Mobile | Accessible app | §5.3 tests pass with VoiceOver and TalkBack users (consented) | Audio conflicts | 7 |
| 9 | Failure handling and security | Safe failure, secure link | 7 | Stale/epoch tests; production pairing design; Privacy=device; LAN stream lockdown; **local audio hardware** | Shared | Security note; fault-injection report | Every FMEA row (08 §6) has a passing verification or an accepted residual | Passkey UX | 7 |
| 10 | Lab / controlled testing | Measure detection limits | 2, 9 | Glass, poles, people, bicycles in a controlled hall; latency; thermal | HW + Shared | Test report with numbers | §5.4 criteria met or documented | Safety of testers | 9 |
| 11 | Supervised user testing | Real users, safely | 10; ethics approval | Protocol, consent (accessible, witnessed), supervision, stop rules | Shared + O&M instructor | Study report | No unsupervised use. Stop rules respected. Findings logged | Participant safety | 10 |
| 12 | Production readiness | Decide go/no-go | 11 | Regulatory advice; battery; enclosure; manufacturing; support | Shared | Readiness review | Explicit sign-off; **not** implied by passing tests | Regulatory status | 11 |

## 3. Ownership (who does what)

| Work item | Hardware/AI dev | Mobile dev | Shared |
|---|---|---|---|
| Sensor set-up, drivers, calibration, bearings | ✅ | | |
| Camera, AI inference, LiDAR, radar processing, fusion, risk estimation | ✅ | | |
| Event generation (`cue_seq`, health) | ✅ | | |
| BLE GATT server, advertising, pairing agent, command processing | ✅ | | |
| Device health monitoring, thermal/power events | ✅ | | |
| **Local safety alerts (audio + haptic hardware)** | ✅ | | |
| Bluetooth permissions, discovery, pairing UX, secure connection | | ✅ | |
| GATT discovery, subscriptions, decoding, reassembly | | ✅ | |
| Alert presentation, TTS, haptics, audio-session handling | | ✅ | |
| Accessibility (VoiceOver/TalkBack), screens S0–S8 | | ✅ | |
| Reconnection, lifecycle, background behaviour | | ✅ | |
| Mobile testing on reference phones | | ✅ | |
| Protocol definition, schema, vectors, versioning | | | ✅ |
| Alert priority mapping (07 §6) | | | ✅ |
| Security model, pairing method | | | ✅ |
| Integration and latency tests | | | ✅ |
| User acceptance tests (with an O&M professional) | | | ✅ |

**Handoffs:**

| # | From → To | Handoff |
|---|---|---|
| H-1 | HW → Mobile | ICD v1 + `contract/` (done in draft) |
| H-2 | HW → Mobile | `--mock` running on a shared Linux box or Pi (Phase 5) |
| H-3 | Mobile → HW | App build for live tests (Phase 7) |
| H-4 | HW → Mobile | Every phrase registry revision, with its sha |
| H-5 | Both | Change requests to the ICD via a versioned doc PR. **No silent changes** |

## 4. End-to-end integration checklist

**Device side:**
- [ ] `sudo rfkill unblock bluetooth`, persisted; `bluetoothctl show` → Powered yes
- [ ] `venv/bin/python -m unittest ble_bridge.test_ble_bridge` → 34 OK
- [ ] `ble_bridge.make_vocab --check` → up to date
- [ ] Mock advertises `Shappe-M`, visible in nRF Connect / LightBlue; pairing works (Just Works)
- [ ] PROTOCOL_INFO is readable only after pairing (unpaired read → error 0x0F)
- [ ] STATE notifications arrive at ≥ 1 Hz; ALERT on each step that announces a new sentence (a run's first sentence is only learned); EVENTS JSON reassembles at MTU 23 and 185+
- [ ] Live mode with `shappe-guide` running under supervision: STATE status reflects `/health`; killing `shappe-guide` gives `service_unreachable` within 1 s

**Phone side:**
- [ ] App decoder passes 16/16 `test_vectors.json`
- [ ] `hello` sends the real `att_mtu`
- [ ] De-duplication across `service_crash` (new epoch) → no repeated speech
- [ ] Stale drop: inject a 2.5 s delay → alert not spoken, ack `expired`
- [ ] `audio_enabled = 0` → no sound; `device_voice = 1` → no phone speech
- [ ] Link stale after 3 s → warning spoken; reconnect within back-off; re-subscribe; `hello`
- [ ] Locked phone, 30 min: alerts still spoken (iOS and Android reference devices)
- [ ] VoiceOver/TalkBack full flow without sight (S0–S8)

**ToF depth camera (when it arrives):**
- [ ] `config.txt`: `dtoverlay=arducam-pivariety,cam0`; `pip install ArducamDepthCamera`; `TOF_ENABLED = True`; `/tof/status` ok at ~30 fps
- [ ] `tools/tof_calibrate.py wall` on a flat wall → set `TOF_DEPTH_IS_RADIAL`
- [ ] 15–25 checkerboard captures (`POST /tof/calibrate/capture`), `tools/tof_calibrate.py solve` (pair error < 1 px), `/tof/calibrate/reload`; `/tof/overlay.jpg` shows a table's dots on the table, near and far
- [ ] `venv/bin/python tools/tof_fusion_sim.py` → 26/26
- [ ] Walk with `TOF_USE_DISTANCE = False`: compare "(ToF N ft)" with the LiDAR's distance on the dashboard before turning it on

**Together:**
- [ ] Latency report (p50/p95) per alert level
- [ ] 2 h soak: no crash, no leak (bridge RSS stable), no missed `cue_seq` steps vs `/guidance/log`

## 5. Test strategy and acceptance criteria

Each criterion below is a proposed engineering target, to be confirmed in Phase 4.

### 5.1 Hardware

| Test | Method | Pass |
|---|---|---|
| Camera function and channel map | `/camera/stats`; cover each lens in turn | Each covered lens maps to the expected direction; `camera_fps` front ≥ 20 |
| Camera HFOV | Target at a known distance at the frame edges | Measured HFOV recorded; `SIMPLE_CAMERA_HFOV_DEG` updated by the owner |
| Camera sync (cycle) | Timestamp spread in `/camera/stats` | Per-direction age ≤ 0.5 s |
| LiDAR | `/lidar/reader` | RPM 900 ± 5 %; points per scan recorded |
| Radar | `/radar/test` hand test per board; a glass pane at 1/2/3/4 m | Each board's bearing matches its serial; glass echo rate recorded |
| AI model execution | `/camera/hailo`, `tools/hailo_monitor.py` | Front detection ≥ 10/s |
| Continuous runtime | 4 h bench run | No restart; watchdog quiet |
| Thermal | 2 h at 30 °C ambient in the enclosure | SoC < 80 °C, no throttle bits |
| Battery endurance | Once a battery exists | ≥ 3 h at a typical walk load |
| Start-up and recovery | Cold boot ×10; kill the service ×10 | Ready (start-up gate) ≤ 60 s; `device_ready` after each kill |

### 5.2 Bluetooth

| Test | Pass |
|---|---|
| Discovery by service UUID (fore- and background) | Found within 5 s in the foreground |
| Pairing and authentication | Unpaired access refused; bonded reconnect needs no prompt |
| Event delivery | 0 lost cue steps in 30 min (compare `/guidance/log`) |
| Commands | All 12 commands answer within 1 s; errors as in ICD §6.3 |
| Reconnection | After walking out of range and back: Live again ≤ 15 s |
| Duplicate handling | 0 duplicate announcements across 20 reconnects |
| Data corruption | Fuzzed frames → `BAD_FRAME`; the bridge stays up |
| Stale rejection | Delayed alerts > 2 s never spoken |
| Device restart | New epoch handled; no replay |
| Range | Distance at which STATE stops, on the body (pocket vs hand) — recorded |
| OS background | iOS locked 30 min; Android Doze 30 min — alerts delivered (record failures honestly) |

### 5.3 Accessibility

| Test | Pass |
|---|---|
| TalkBack and VoiceOver full journey (S0–S8) | Completed by a screen-reader user without help |
| Alert clarity | ≥ 90 % of phrases correctly repeated back in street noise (supervised) |
| Focus order and labels | Automated scanner (Accessibility Scanner / Xcode Accessibility Inspector) has 0 errors |
| Non-visual onboarding and pairing | Completed without sight |
| Haptics | Level patterns distinguishable ≥ 90 % |
| Audio interruptions | Behaviour in 07 §9 observed for calls, music and the screen reader |
| Locked screen | As in 5.2 |

### 5.4 Safety

| Test | Pass |
|---|---|
| Obstacle detection limits | Distance and angle at first warning recorded for a wall, glass, pole, person and bicycle (≥ 20 trials each) |
| False alerts | Rate per 10 min of walking recorded; owner sets the limit |
| Missed hazards | Every miss logged with `mark missed`; none in the H1–H3 classes at < 4 ft in the controlled hall |
| Delayed alerts | Latency targets as in 03 §9 |
| Sensor outages | Each degraded mode (08 §5) is announced within 3 s |
| Connection loss | The device still warns locally (needs local audio) |
| Overheating, low battery | Events raised; behaviour documented |
| Ambiguous environments | Crowds, sun and rain documented, with supervisor stop rules |

**Passing software tests does not make the device deployable.** Live testing requires:
- ethics approval and informed consent in accessible formats (verbal consent witnessed and documented, per the Declaration of Helsinki 2024);
- a sighted supervisor within arm's reach;
- an O&M professional present;
- the participant using their own cane or dog throughout;
- no road crossings that rely on the device.

## 6. Implementation priorities (in order)

1. **Local audio output on the device** (USB audio + bone conduction) and a decision on `AUDIO_ENABLED`. This closes the critical gap.
2. Phase 2 measurements, especially camera HFOV, radar bearings and LiDAR RPM.
3. Freeze ICD v1; mobile decoder against the vectors; FakeDeviceLink UI.
4. Bridge over the air with the mock; then live.
5. End-to-end alerts with latency measurement.
6. Accessibility screens and the audio-session work.
7. Production pairing (spoken passkey + button), Privacy=device, LAN stream lockdown.
8. Battery + gauge → Battery Service; haptics.

## 7. Major technical risks

| Risk | Impact | Mitigation |
|---|---|---|
| No local alert channel | Silent device when the phone fails | Priority 1 |
| iOS background speech and BLE limits | Missed phone alerts | Device audio; AccessorySetupKit; test early |
| iOS pairing on 0x0F | Pairing never prompts | Test in Phase 5; fall back to authenticated flags |
| Camera HFOV assumption | Wrong camera ranges and vehicle logic | Measure (Phase 2) |
| Unproven glass detection | Collision with glass | Supervised glass tests |
| Coverage gaps (drop-offs, overhead) | Falls | Communicate clearly; future sensors |
| Radio coexistence (Wi-Fi + BLE + A2DP on one chip) | Latency and dropouts | Measure; prefer wired audio |
| Uncommitted, untested working tree; no `tests/` | Regressions | Owner reviews and commits; restore CI |

## 8. Future enhancements (not part of BLE v1)

| Feature | Value | Feasibility | Extra hardware | Safety implications | Complexity |
|---|---|---|---|---|---|
| Haptic feedback (shoulder LRAs) | Silent, directional, works during calls | High | I²C driver + 2 motors | Second channel; must not mask audio | Medium |
| Bone-conduction audio | Local voice with open ears | High | USB audio + headset | Closes the critical gap | Low |
| Battery management (gauge/UPS) | Battery level, safe shutdown | High | Fuel gauge / UPS HAT | Enables low-battery warnings | Medium |
| Drop-off / overhead sensing | Covers H4/H5 | Research | Downward ToF or depth camera / tilted LiDAR | Large safety value; false alarms | High |
| GPS-assisted / offline routes | Wayfinding | Medium (phone-side) | None (phone GPS) | Must never override obstacle warnings | High |
| Accessible destination selection | Wayfinding UX | Medium | None | As above | Medium |
| More object classes (custom model: poles, bollards, stairs, potholes) | The vocabulary already has the words | Medium | None (NPU) | Retraining, validation | Medium |
| Improved fusion (camera-LiDAR calibration per camera) | Better names and ranges | Medium | Calibration rig | Fewer wrong names | Medium |
| Indoor localisation / navigation | Buildings | Low | Beacons / maps | — | High |
| Caregiver features (opt-in) | Reassurance | Medium | None | Privacy, consent | Medium |
| Secure update mechanism | Maintainability | Medium | None | Signed updates | Medium |
| Field diagnostics upload (opt-in) | Faster debugging | Medium | None | Privacy | Medium |
| Multilingual voice | Reach | High (phrase ids) | None | Translation review of 300 phrases | Low–Medium |
| Device settings over BLE (v2) | Voice rate, verbosity | High | None | Needs authenticated ownership; never thresholds | Medium |

## 9. Open questions (decisions needed)

| # | Question | Owner |
|---|---|---|
| Q1 | Mobile stack: native (Swift + Kotlin), Flutter, React Native or KMP? Licence check for `flutter_blue_plus` (commercial) and `react-native-ble-plx` (licence metadata conflict) | Shared |
| Q2 | Local audio: wired bone conduction (recommended) or Bluetooth headset? | HW |
| Q3 | When to set `AUDIO_ENABLED = True`, and whether the phone should speak while the device is silent for trials | Owner |
| Q4 | Production pairing: spoken passkey + button? Is a physical button acceptable on the enclosure? | Shared |
| Q5 | One phone per device, or a caregiver phone too? | Owner |
| Q6 | Enable LiDAR mover speech (`LIDAR_MOVERS_SPEAK`) after field review? (Not to be flipped without the owner) | Owner |
| Q7 | Should "Walk straight" confirmations be phone-optional (default on)? | Shared |
| Q8 | Commit strategy for the uncommitted movers/camera work, `ble_bridge/` and these docs | Owner |
| Q9 | Regulatory positioning (wellness/assistive vs medical device) before any public claims | Owner |
| Q10 | Battery/enclosure design and the target runtime | HW |

## 10. Recommended development approach (summary)

1. Treat the device as the safety system and the phone as a companion. **Fit local audio first.**
2. Freeze the small v1 protocol now: 20-byte ALERT/STATE, JSON events and commands, append-only phrases, epoch + `cue_seq` de-duplication, 2 s freshness. Let the mobile developer build against the vectors and the mock immediately, in parallel with hardware measurement.
3. Keep BLE **out of process** from the guidance loop. Add no BLE command that can change guidance.
4. Bring the bridge up over the air under supervision; measure latency end to end; then iterate on pairing security and accessibility.
5. Validate on a bench, then in a controlled hall, then in supervised field tests. Claims follow evidence, never the reverse.
