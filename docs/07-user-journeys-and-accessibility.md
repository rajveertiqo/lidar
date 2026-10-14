# 07 — User Journeys and Accessibility Design

**For:** the mobile developer and the whole team. The app's users are **blind or severely visually impaired**. The design is non-visual first; the screen is a secondary view for sighted helpers and low-vision users.

Status: the app is **NOT IMPLEMENTED**. Everything about the app here is **PROPOSED**. Device behaviour quoted from code is **CONFIRMED**.

---

## 1. Design principles

| # | Principle | Basis |
|---|---|---|
| P1 | **No screen use while walking.** Everything during a walk comes as speech or haptics. Two on-demand questions ("What's ahead?", "What's around me?") sit first in the focus order, and on iOS the first is bound to VoiceOver's Magic Tap | Field reality; Apple HIG |
| P2 | **The device decides; the app relays.** The app never invents guidance or reassurance (ICD rule A8) | Safety model |
| P3 | **One voice at a time.** If the device's voice is speaking (`DEVICE_VOICE`), the phone stays quiet. The app never talks over a call or a screen-reader announcement in progress unless the alert is CRITICAL | ICD A5, A7 |
| P4 | **Silence when sound is disabled** (`AUDIO_ENABLED = 0`): show and log, no sound | ICD A4; device config today |
| P5 | **Ears stay open.** Recommend bone-conduction or open earphones. Never require noise-cancelling | O&M practice |
| P6 | **No time limits** on any step; no auto-dismissing dialogs | WCAG 2.2 SC 2.2.1 |
| P7 | **Targets ≥ 44 × 44 pt (iOS) / 48 × 48 dp (Android)**; text scales ≥ 200 %; contrast ≥ 4.5:1 (text), ≥ 3:1 (controls) | Apple HIG; Android a11y; WCAG 2.2 SC 1.4.3, 1.4.11, 2.5.8 |
| P8 | **Status messages are announced without moving focus**: Android live regions / pane titles (`announceForAccessibility` is deprecated in API 36); iOS `UIAccessibility.post(.announcement)` (iOS 17+ priority attributes) | WCAG 4.1.3; Android 16 notes; Apple |
| P9 | **Accessible authentication**: no cognitive tests, no visual-only codes; passkeys are spoken | WCAG 2.2 SC 3.3.8 |
| P10 | **Haptics complement audio, never replace it alone**, except in "silent" states | Apple HIG ("Use haptics in addition to audio cues") |
| P11 | **Plain, fixed wording** for app-owned messages, with a "repeat last" action. Distances in **feet**, matching the device | Device unit convention |

## 2. Scenario A: first-time setup

| Step | User | Device (CONFIRMED / PROPOSED) | App (PROPOSED) | User hears |
|---|---|---|---|---|
| 1 | Receives the hardware | — | — | — |
| 2 | Powers it on | systemd starts `shappe-guide`. The start-up gate holds speech until the LiDAR settles (CONFIRMED). `shappe-ble` starts and opens a 180 s pairing window (PROPOSED) | — | Device voice, when audio hardware exists: "Ready." is built by `startup_gate.orient()` but **not spoken today** (CONFIRMED gap). PROPOSED: "Shappe starting" … "Shappe ready" |
| 3 | Opens the app for the first time | — | Welcome screen. VoiceOver/TalkBack reads: purpose, the **safety statement** ("Shappe adds information. Keep using your cane or dog."), and a "Continue" button | The safety statement, read once in full; replayable from Settings |
| 4 | Grants Bluetooth permission | — | Explains why before the OS prompt (iOS `NSBluetoothAlwaysUsageDescription`; Android `BLUETOOTH_SCAN`/`CONNECT`). With AccessorySetupKit (iOS 18+) / CompanionDeviceManager (Android) the system picker replaces most of this | OS prompt via the screen reader |
| 5 | "Find my Shappe" | Advertising the service UUID | Scans by service UUID; lists devices ("Shappe", or "Shappe demo device"), sorted by signal strength | "One Shappe found. Double-tap to connect." |
| 6 | Connects | Encrypted characteristic → pairing request | OS pairing dialog (v1 Just Works: "Pair"; production: a 6-digit code spoken by the device into the earpiece) | "Shappe will say a 6-digit code in your earpiece. Type it in the pairing box." (production) |
| 7 | Pairing completes | Bond stored (BlueZ) | Reads PROTOCOL_INFO; stores `device_id`; subscribes; `hello` | "Connected to Shappe." |
| 8 | Readiness check | `/health` status | `get_status`; reads STATE bits | "Shappe ready. All sensors working." or "Shappe ready with problems: glass detection off." |
| 9 | Status on demand | — | "Device status" reads each sensor in plain words | — |
| 10 | Accessibility settings | — | Voice: system or screen-reader voice; speech rate; haptics on/off and strength; "Walk straight" confirmations on/off | Each change is confirmed by voice |
| 11 | Starts a walk | Guidance runs continuously anyway | `session_start` (labels the session only) | "Walk started. Shappe is guiding." |

## 3. Scenario B: walking on a sidewalk

The device speaks (or the app relays) only the gated sentence. "BLE" means what crosses the link.

| Situation | Relevant sensors | Device processing (CONFIRMED) | BLE | User hears (exact device sentence) |
|---|---|---|---|---|
| Person ahead, 9 ft | LiDAR range; front camera "person"; range agreement | `front_object` names it; level 1 | ALERT phrase 63 | "Person ahead" |
| Person in path, < 4 ft | LiDAR front < `WALK_BLOCK_FT`; camera name; turn choice | Blocked → steer-past sentence; level 2, urgent | ALERT phrase 267 (URGENT) | "Person in front, turn left" |
| Stationary vehicle ahead, user standing still | Camera vehicle class; box not growing | `_Vehicles`: stopped → "be careful", said once | ALERT phrase 18 | "Vehicle in front, be careful" |
| Moving bicycle approaching from the left | Front camera bicycle box growing (bicycle is in `SIMPLE_VEHICLE_CLASSES`); LiDAR mover | Vehicle approach → level 3, cuts in | ALERT phrase 15 (level 3) | "Vehicle approaching, move right" (or "Vehicle approaching" with no room) |
| Pole ahead and slightly right | LiDAR cluster; camera "traffic light/stop sign/parking meter" → word "Pole" (COCO cannot name a plain pole) | Steer past; level 1 | ALERT phrase 194 | "Pole ahead, turn slightly right" (or no name if the camera can't classify: "Turn left/right" once blocked) |
| Wall ahead / near a wall | LiDAR surface fit (`fit_surfaces`) | "Wall ahead" inside 12 ft; near wall inside 4 ft; drift inside 5 ft and closing | ALERT phrases 2 / 10 / drift | "Wall ahead" / "Near left wall, move right" / "Drifting toward left wall, turn slightly right" |
| Glass door ahead | Radar front boards much nearer than LiDAR (≥ 2 ft) | Glass flag; inside 12 ft "Glass ahead", < 4 ft "Blocked by glass" | ALERT phrases 3 / 5 | "Glass ahead" → "Blocked by glass" |
| Roadside obstacle (bin, bench, hydrant) | LiDAR + camera (COCO bench, fire hydrant) | As for the pole | ALERT | "Bench ahead" / "Hydrant ahead, turn slightly right" |
| Parked motorcycle | Camera "motorcycle" → word "Motorbike"; LiDAR | As a named object; vehicle sentences only if moving | ALERT | "Motorbike ahead" / "Motorbike in front, turn right" |
| Narrow path between two objects | Camera boxes plus LiDAR gap (`camera_path`) | Steering between them | ALERT | "Walk between them, slightly left" / "Walk straight between them" |
| Person walking toward the user (LiDAR mover) | LiDAR mover tracks | **Silent trial** (`LIDAR_MOVERS_SPEAK=False`), logged as "would say" | Nothing today | Nothing today; when enabled: "Person coming toward you, move left" |
| Overhead branch, kerb, stairs down | — | **Not detected** (NOT IMPLEMENTED) | Nothing | **Nothing. This is a known hazard; the cane must cover it** |

On-demand ("What's around me?"): `get_surroundings` returns up to 6 hazards. The app reads, for example, "Person, 12 o'clock, 9 feet. Obstacle, 3 o'clock, 4 feet." This is **inferred information**, so it is prefixed once per session with "Approximate:".

## 4. Scenario C: potential road-crossing hazard

| Aspect | Behaviour |
|---|---|
| Detection | The front camera sees a traffic light (COCO `traffic light`). Its **colour is never read** (CONFIRMED). The device sets `at_crossing` and **withholds "Walk straight" for 5 s** after the last sighting (`SIMPLE_CROSSING_HOLD_S`). Vehicles: the camera's approach logic still warns ("Vehicle approaching …") |
| What is NOT done | **The device never says it is safe to cross**, and neither may the app. No signal phase, no gap acceptance, no kerb detection |
| User hears | Vehicle warnings if any; otherwise silence where "Walk straight" would have been |
| App rule | The app MUST NOT add crossing advice. It MAY offer a static reminder from Settings ("Shappe cannot tell when it is safe to cross") |

## 5. Scenario D: Bluetooth disconnects during a walk

| Question | Answer |
|---|---|
| How does the hardware detect it? | BlueZ reports Device1 `Connected=false` and the bridge logs it (PROTOTYPE). Its per-connection counters restart when the next connection begins. The navigation service does not know or care |
| Does local navigation continue? | **Yes.** The guidance loop is in a separate process and does not depend on BLE (CONFIRMED architecture, PROPOSED bridge) |
| Do safety alerts continue? | **On the device: yes, but only once local audio hardware exists and `AUDIO_ENABLED` is on.** Today the device has no audible output, so a lost link means **no warnings at all**. This is a critical gap ([08 §4](08-safety-reliability-failure-management.md)) |
| What does the user hear? | From the phone, 3 s after the last STATE: "Lost contact with Shappe. Use your cane." (WARNING, plus a strong haptic). Repeated every 30 s while lost |
| What does the app display? | A persistent banner "Not connected — reconnecting" (live region), the last alert time, and the reconnect attempt count |
| How does reconnection happen? | Automatically: back-off 1, 2, 4, 8, 15 s; re-subscribe; `hello`. A new epoch resets de-duplication. Old alerts are never replayed (REPLAY flag). On success: "Shappe connected." |

**Re-pair help** (bond lost on either side), spoken step by step, each step confirmed with a button:
1. "Open Settings, then Bluetooth."
2. "Find Shappe and choose Forget This Device."
3. "On Shappe, open pairing" (PROPOSED button; in v1 restart the device: pairing is open for 3 minutes).
4. "Come back to this app and choose Find my Shappe."

## 6. Alert model

The app maps device and link signals onto five categories:

| Category | Triggers | Confidence requirement | Audio | Haptic (proposed) | Cooldown / repeat | Expiry | De-duplication | User response |
|---|---|---|---|---|---|---|---|---|
| **INFORMATION** | Device level 0 ("Walk straight"); connected, session started, ready | Device-gated | Speak; never interrupts; may be turned off | None | Device-controlled | 2 s | `(epoch, cue_seq)` | None |
| **CAUTION** | Device level 1 (advisories: "Wall ahead", "Person ahead", "Glass on left …"); `sensor_degraded` for camera, hailo, radar, front_glass | Device-gated | Speak; replaces queued speech; waits for current speech | 1 medium pulse | Device-controlled (≥ 3 s gap, 6 s settle) | 2 s | as above | Be aware |
| **WARNING** | Device level 2 urgent ("Blocked by …", "Turn …", "X in front …", movers); link stale; thermal; power | Device-gated; link stale after 3 s | **Interrupts** app speech; ducks other audio | 2 strong pulses | Device: blocked repeats 30 → 120 s. App-owned warnings: every 30 s while the condition lasts, max 5 times, then every 120 s | 2 s (device); while the condition lasts (app) | as above / by condition | Stop or adjust |
| **CRITICAL** | Device level 3: "Vehicle approaching …" | Device-gated (camera box growth, ≥ 3 frames) | **Interrupts everything** the app controls, including its own speech | 3 long pulses | Device-controlled | 2 s | as above | Act now |
| **SYSTEM FAILURE** | STATE status 2/3; `sensor_degraded: lidar`; `guidance_stalled` (loop stopped); `service_unreachable`; "Sensors offline" (phrase 14) | Device health / bridge | Interrupts non-critical speech | Long-short-long | Every 30 s while it lasts, max 5, then every 120 s | While it lasts | By condition | **Use your cane / stop using the device** |

**Example sentences:**

| Kind | Sentence | Notes |
|---|---|---|
| Device phrase | "Blocked by glass" | Device-owned, closed vocabulary |
| Device phrase | "Person in front, turn left" | Device-owned, closed vocabulary |
| Device phrase | "Vehicle approaching, move right" | Device-owned, closed vocabulary |
| Device phrase | "Sensors offline" | Device-owned, closed vocabulary |
| App-owned | "Lost contact with Shappe. Use your cane." | Fixed wording |
| App-owned | "Shappe guidance stopped. Use your cane." | Fixed wording |
| App-owned | "Glass detection off." | Fixed wording |
| App-owned | "Object names reduced." | Fixed wording (one camera missing) |
| App-owned | "Vehicle warnings off." | Fixed wording (AI engine down) |
| App-owned | "Object names off." | Fixed wording |
| App-owned | "Shappe is overheating." | Fixed wording |
| App-owned | "Shappe power is low." | Fixed wording |
| On request only | "Approximate: Person, 12 o'clock, 9 feet." | From `get_surroundings` |

There are deliberately **no** app phrases such as "Obstacle approximately two meters ahead": distance-bearing sentences are the device's job, and the device works in feet.

**Latency targets** (proposed, measured as in [03 §9](03-perception-and-navigation-pipeline.md)):
- CRITICAL: ≤ 800 ms from the device decision to phone audio onset;
- WARNING: ≤ 900 ms p95;
- link-stale detection: 3 s.

## 7. Screen-by-screen specification

Common rules for all screens:
- One heading per screen, announced on arrival (`.screenChanged` / `paneTitle`).
- The focus order is as listed.
- Every control has a label and a hint, and no element is unlabeled.
- Large text is supported, with no truncation.
- No auto-dismiss.
- A "Repeat last alert" action is available everywhere: a list item on Android, and the VoiceOver rotor or a custom action on iOS.

| # | Screen | Focus order (label → action) | Live / announced content |
|---|---|---|---|
| S0 | **Welcome** (first run) | Heading "Welcome to Shappe" → safety statement (static text, full) → "Continue" | — |
| S1 | **Find my Shappe** | Heading → "Search again" → device list (each item: "Shappe, strong signal" / "Shappe demo device — not real") → "Help" | "Searching…", "One Shappe found" (polite) |
| S2 | **Pairing help** | Heading → numbered steps (each a button "Done") → "Try again" | Each step spoken when focused |
| S3 | **Home** (connected) | Status line ("Connected · Guiding · All sensors OK") → **"What's ahead?"** → **"What's around me?"** → "Start walk / End walk" → "Device status" → "Settings" → "Last alert: Blocked by glass, 12 seconds ago" | Status changes (polite); alerts as in §6 (assertive for WARNING and above) |
| S4 | **Device status** | Heading → overall ("Degraded") → LiDAR → Radar (with directions down) → Cameras → AI engine → Glass detection → Temperature → Power → Battery ("Not available on this device") → Software versions → "Read all" | Updates politely |
| S5 | **Settings** | Voice (screen-reader voice / app voice) → Speech rate → Haptics (on/off, strength) → "Haptics when silent" → "Walk straight confirmations" (on/off) → Language (app messages) → "Safety statement" → "Forget this device" → "Privacy and logs" | Each change confirmed |
| S6 | **Diagnostics** | Link state, signal strength, MTU, protocol, vocabulary revision, device id → "Ping device" → last 10 alerts with latency → "Device speech log" → "Share diagnostics" (explicit consent dialog) | — |
| S7 | **Field test** (developer mode, hidden by default) | "Right", "Wrong", "Missed", "Note…" → `mark` | "Marked" |
| S8 | **Update needed** | Message "Shappe needs a newer app" → "Open store" | — |

**User journeys covered:**
- *Opening the app:* S3 if bonded, otherwise S0/S1.
- *Connecting:* S1 → OS dialog → S3.
- *Starting a session:* S3 "Start walk".
- *Receiving an alert:* no screen needed (speech/haptic); S3 shows it.
- *Adjusting settings:* S5.
- *Recovering from disconnection:* automatic, plus the S3 banner and S2 if the bond is lost.
- *Checking the battery:* S4 says "Not available on this device" until a gauge exists.
- *Ending a session:* S3 "End walk" → "Walk ended, 24 minutes."

## 8. Scenario E: sensor and system failures

| Failure | Detection (CONFIRMED unless noted) | Device behaviour | App notification | Degraded mode | User guidance |
|---|---|---|---|---|---|
| One camera fails | Cycle mode never visits a missing channel. `/health` warning "No camera connected: …" or "Camera sees only black: …"; status degraded | Guidance continues; no names from that direction. The front camera: "Camera off" (phrase 296) | `health_changed` → CAUTION "Object names reduced" | LiDAR + radar still range everything | Keep walking with care |
| LiDAR stops | `lidar_ok` false; "Sensors offline" (phrase 14) once the newest scan is older than 2 s; `/health` down | Says "Sensors offline". Camera vehicle warnings still possible | ALERT phrase 14 + STATE status 2 + `sensor_degraded: lidar` → SYSTEM FAILURE (the loop keeps running, so not `guidance_stalled`) | **No geometry: not usable for guidance** | "Shappe cannot see. Use your cane." |
| Radar unavailable | `radar_ok` false / `radar_down`; front coverage lost → "Glass sensor off" | Says "Glass sensor off" (level 1). Glass not detected | `sensor_degraded: radar/front_glass` → CAUTION "Glass detection off" | LiDAR-only; glass doors missed | Extra care near shop fronts and doors |
| AI accelerator fails | `hailo_ok` false; "Detection engine not ready" | No object names or vehicle warnings | `sensor_degraded: hailo` → CAUTION "Object names off" (+ "Vehicle warnings off") | Geometry only | Extra care near roads |
| Navigation app crashes | systemd watchdog (30 s) and `Restart=always`. The bridge sees polls fail | Restarts. Silent until the start-up gate passes | `service_unreachable` → SYSTEM FAILURE; later `service_restarted` + `device_ready` | None while down | "Shappe guidance stopped. Use your cane." … "Shappe ready." |
| Battery critical | **NOT IMPLEMENTED** (no gauge). Only under-voltage via `get_throttled` (PROPOSED in bridge) | Brown-out → reboot or shutdown | `power_warning` (under-voltage) → WARNING | — | "Shappe power is low. Plan to stop." |
| Overheating | SoC ≥ 80 °C or throttle bits (bridge, PROPOSED) | Throttles; detection slows → may report "Object detection stalled" | `thermal_warning` → WARNING | Slower naming | "Shappe is overheating. Move to shade." |

## 9. Background, locked phone and audio conflicts

| Situation | iOS (PROPOSED handling) | Android (PROPOSED handling) |
|---|---|---|
| Phone locked, app in background | `bluetooth-central` delivers notifications. The audio session must be active to speak: category `.playback` + `.duckOthers` + `.interruptSpokenAudioAndMixWithOthers`, mode `.voicePrompt`. **NEEDS VALIDATION** on current iOS | Foreground service `connectedDevice` with an ongoing notification ("Shappe connected — guiding"). TTS keeps working |
| App force-quit by the user | **Not relaunched** for Bluetooth (TN3115). On the next open: "Shappe was disconnected because the app was closed." | The service stops; the CompanionDeviceManager presence callback can restart it |
| Screen reader speaking | Prefer `AVSpeechSynthesizer` with `prefersAssistiveTechnologySettings` (the user's VoiceOver voice and rate). WARNING+ interrupts the app's own speech. VoiceOver announcements (`.announcement`, priority high for CRITICAL) only while in the foreground | TTS with `USAGE_ASSISTANCE_NAVIGATION_GUIDANCE`. TalkBack speech and app TTS can overlap; for WARNING+ use `AUDIOFOCUS_GAIN_TRANSIENT_MAY_DUCK`. **NEEDS VALIDATION** |
| Phone call | The audio session is interrupted by the system; the app cannot speak into a call. Use haptics for WARNING+ | Media is muted during calls (Android 12+). Use haptics for WARNING+ |
| Music / podcast | Ducked during alerts (`.duckOthers`) | Ducked by transient audio focus |
| Other navigation app speaking | Both duck; the newest wins. **NEEDS VALIDATION** | Same |

**This is why the device needs its own audio.** The phone cannot guarantee speech during calls or after a force-quit, or in some background states. The phone is a companion channel, never the only safety channel.
