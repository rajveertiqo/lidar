# 05 — Mobile Developer Integration Guide: AI Assistive Navigation Hardware

**For:** the developer building the companion app (framework not yet chosen).
**You need no hardware to start.** Use the mock device and test vectors (§20).

Status labels: **CONFIRMED** (exists in the device code) · **PROPOSED** (designed here; the prototype is in `ble_bridge/`) · **NOT IMPLEMENTED** · **NEEDS VALIDATION**.

---

## 1. What is this hardware project?

A wearable Raspberry Pi 5 with a spinning laser scanner (LiDAR), four cameras with an AI accelerator, six 60 GHz radars and a gyro. It continuously works out what is around a blind pedestrian and chooses **one short sentence** from a fixed list of 300 ("Walk straight", "Blocked by glass", "Person in front, turn left", "Vehicle approaching, move right"). It is designed to speak only when something new matters. See [01](01-project-overview.md).

## 2. Why was it developed?

To warn about what a cane cannot reach in time: glass, walls being closed on, people and vehicles approaching, and objects in the walking line, out to about 4 m all round. It **adds to** the cane and guide dog; it does not replace them.

## 3. Who is the user?

A blind or severely visually impaired adult who already travels with a cane or dog. The user uses VoiceOver or TalkBack, often has the phone in a pocket while walking, and needs to keep at least one ear on traffic sounds. Design for **zero screen interaction while walking**. See [07](07-user-journeys-and-accessibility.md).

## 4. How does the hardware work?

Sensors → fusion → one decision every 200 ms → a speech gate that decides *if* the sentence should be said. When the gate says yes, a counter called **`cue_seq` goes up by one**. That counter step is the only "announce now" signal anywhere in the system. The existing web dashboard already follows this rule, and so must your app. Details in [03](03-perception-and-navigation-pipeline.md).

## 5. What sensors are present, and what is each for?

| Sensor | Purpose | What your app sees |
|---|---|---|
| LiDAR | Distances all round (one horizontal plane) | `lidar_ok`; front clearance in feet |
| 4 cameras + Hailo NPU | What things are (person, car, bicycle …) | `camera_ok`, `hailo_ok`; object words inside sentences |
| 6 radars | Glass the laser can't see | `radar_ok`, `front_glass_ok`; "Glass ahead" |
| Gyro | Turning (report only today) | nothing |

## 6. What information does the device generate?

Raw points, echoes and boxes stay **on the device**. What it produces for others:
- the committed sentence (with level and urgency);
- a ranked hazard list;
- health and warnings;
- the speech log.

## 7. Which data reaches the app?

Only compact facts ([06](06-interface-control-document.md)):
- **ALERT:** a 20-byte record when the device announces something, plus the sentence text.
- **STATE:** a 20-byte record with the current sentence and health bits, at 1–5 Hz. It is also the heartbeat.
- **EVENTS:** JSON for health changes, sensor loss, thermal, sessions and command responses.
- **On request:** status, "what's around me", the current sentence and the speech log.

**No images, audio or location** ever cross the link.

## 8. How does Bluetooth communication work?

BLE. The Pi is the peripheral and GATT server; your app is the central and client. There is one custom service with five characteristics, plus the standard Device Information service. The device pushes ALERT, STATE and EVENTS by notification. The app writes JSON commands to COMMAND and gets JSON responses on EVENTS, matched by `id`. Why BLE: [04 §2](04-bluetooth-communication-architecture.md).

## 9. How does the app discover the hardware?

Scan **by service UUID `37e90001-0882-43f6-ae9b-b733c720c46d`**. The advertised name `Shappe` (or `Shappe-M`) is only a hint. Never rely on the name or MAC address; iOS doesn't expose the MAC anyway.

- **iOS 18+:** strongly consider **AccessorySetupKit**. It gives a system picker without the Bluetooth permission prompt, and from iOS 26 only ASK-set-up accessories get state-restoration relaunch in some cases (Apple TN3115).
- **Android 12+:** consider **CompanionDeviceManager** association. It gives background start rights, and scanning without location permission.

## 10. How does the user connect securely?

1. The device opens a **pairing window** (v1: 180 s after the bridge starts; PROPOSED: a physical button).
2. The app connects and reads PROTOCOL_INFO.
3. The device answers "Insufficient Encryption", and the OS shows its own pairing dialog. On Android, call `createBond()` first.
4. v1 uses **Just Works** (no code). Production (PROPOSED) uses a **6-digit passkey spoken by the device into the user's earpiece**, typed into the OS dialog.
5. After bonding, the app records `device_id` from PROTOCOL_INFO. On later connections it checks the same `device_id`.

Details and caveats (the iOS 0x0F vs 0x05 question): [04 §6](04-bluetooth-communication-architecture.md).

## 11. What BLE services and characteristics exist?

| Name | UUID | Props | Content |
|---|---|---|---|
| Device Information | `0x180A` | read | Manufacturer, Model, HW/FW/SW revisions |
| Shappe Navigation | `37e90001-0882-43f6-ae9b-b733c720c46d` | service | — |
| PROTOCOL_INFO | `37e90002-…` | read (encrypted) | JSON: versions, limits, commands, `device_id`, `mock` |
| ALERT | `37e90003-…` | read + notify (encrypted) | ALERT record + ALERT_TEXT |
| STATE | `37e90004-…` | read + notify (encrypted) | STATE record (heartbeat ≥ 1 Hz) |
| COMMAND | `37e90005-…` | write with response (encrypted) | JSON commands |
| EVENTS | `37e90006-…` | notify (encrypted) | JSON events and responses |

There is **no Battery Service**: the device has no battery gauge (NOT IMPLEMENTED). Show "Battery: not available".

## 12. What messages does the hardware send?

ALERT, ALERT_TEXT, STATE, EVENT and RESPONSE. All formats, enums and examples are in [06 §4–§6](06-interface-control-document.md).

## 13. What commands can the app send?

`hello`, `ping`, `get_status`, `get_current`, `get_surroundings`, `get_speech_log`, `get_diagnostics`, `session_start`, `session_stop`, `alert_ack`, `mark`. `set_device_settings` always returns `NOT_SUPPORTED` in v1. **No command can change how the device guides or silence it.** That is deliberate.

## 14. How should messages be decoded?

```
on notification(characteristic, bytes):
    frame = parseHeader(bytes)          # 6 bytes: ver, type, seq(u16 LE), fragIndex, fragCount
    if frame.version != 1: reportIncompatible(); return
    msg = reassembler[characteristic].feed(frame, nowMonotonic())   # null until complete
    if msg == null: return
    switch msg.type:
        0x10 ALERT       -> rec = unpackRecord(msg.payload)   # <H I H H b B H  (14 bytes, little-endian)
                            alerts.onAlert(rec, receivedAt = nowMonotonic())
        0x11 ALERT_TEXT  -> epoch = u16(0); cue = u32(2); text = utf8(payload[6..])
                            alerts.onText(epoch, cue, text)
        0x20 STATE       -> state.onState(unpackRecord(msg.payload))
        0x30 RESPONSE    -> pending.resolve(json.id, json)
        0x31 EVENT       -> events.on(json.event, json.data)
        default          -> ignore                      # forward compatibility
```

Validate your decoder against **every vector** in `ble_bridge/contract/test_vectors.json`; it is a CI test you should copy into your project.

## 15. How should the app handle warnings?

Follow **[06 §8 rules A1–A10](06-interface-control-document.md)** exactly. In short:

```
onAlert(rec, receivedAt):
    if rec.replay: return                                                    # A1: never announce a read copy
    if not cueNewer(rec.epoch, rec.cue, last.epoch, last.cue): ack(rec, "duplicate"); return   # A2
    last = (rec.epoch, rec.cue)
    age = rec.ageMs + (nowMonotonic() - receivedAt)
    if rec.loopStale or rec.ageMs == 0xFFFF or age > info.limits.alert_max_age_ms:             # A3
        display(rec); ack(rec, "expired"); return
    if not rec.audioEnabled: display(rec); maybeSilentHaptic(rec); ack(rec, "shown"); return
    if rec.deviceVoice:      display(rec); maybeHaptic(rec);       ack(rec, "haptic"/"shown"); return
    text = registry.text(rec.phraseId) ?? awaitAlertText(rec, timeout=300 ms)
    if text == null: ack(rec, "suppressed"); return
    speech.say(text, interrupt = rec.urgent or rec.level >= 2)   # newest wins; never queue a backlog
    haptic.pattern(rec.level, registry.side(rec.phraseId))
    ack(rec, "spoken", latency = now - receivedAt + rec.ageMs)
```

**The device is currently configured with `AUDIO_ENABLED = False`**, so every alert arrives with `audio_enabled = 0`, and **your app must stay silent**. This mirrors the dashboard. The hardware team will switch it on when field tests resume. Handle both states.

## 16. How should the app support blind users?

See [07](07-user-journeys-and-accessibility.md). The essentials:
- every function is reachable with VoiceOver/TalkBack in a fixed, short focus order;
- 44 pt / 48 dp minimum targets;
- no time limits;
- spoken onboarding;
- haptic patterns that complement speech;
- no information conveyed only visually;
- and the app never talks over the screen reader or a phone call.

## 17. How should reconnection work?

Use the state machine in [06 §9](06-interface-control-document.md):
- **Stale** after 3 s with no STATE: announce "Lost contact with Shappe. Use your cane."
- **Reconnect** with back-off 1/2/4/8/15 s.
- **Re-subscribe** after every reconnect, then `hello`.
- **Epoch.** A new `epoch` means the device restarted: reset the alert de-duplication.
- **Platform reconnect paths:**
  - iOS: `connect()` never times out [Apple], so you can leave a pending connect to the bonded peripheral; use state restoration.
  - Android: `connectGatt(autoConnect=true)` inside a `connectedDevice` foreground service.

## 18. What should happen if the hardware fails?

| Signal | Meaning | App says (CAUTION / WARNING / SYSTEM FAILURE per [07 §6](07-user-journeys-and-accessibility.md)) |
|---|---|---|
| STATE status 3 / `service_unreachable` | The navigation software stopped | SYSTEM FAILURE: "Shappe guidance stopped. Use your cane." |
| STATE status 2 / `sensor_degraded: lidar` (+ ALERT phrase 14 "Sensors offline") | LiDAR lost: no geometry | SYSTEM FAILURE: "Shappe cannot see. Use your cane." |
| `guidance_stalled` / STATE bit `guiding` = 0 | The guidance loop itself stopped ticking | SYSTEM FAILURE: "Shappe guidance stopped. Use your cane." |
| `sensor_degraded: radar` / `front_glass` | Glass detection reduced | CAUTION: "Glass detection off." |
| `sensor_degraded: camera` / `hailo` | No object names | CAUTION: "Object names off." |
| `thermal_warning` | SoC hot or throttling | WARNING: "Shappe is overheating." |
| `power_warning` | Under-voltage | WARNING: "Shappe power is low." |
| No STATE for 3 s | Link lost | WARNING: "Lost contact with Shappe. Use your cane." |
| `simulated_input` bit | Not real data | Persistent banner and spoken "Demo mode — not real" on connect |

## 19. How should the app behave while walking, locked or in the background?

| Platform | What works | Requirements |
|---|---|---|
| iOS | Notifications are delivered in the background with the `bluetooth-central` background mode. Wakes are short (about 10 s per event, per Apple's guide). Speaking needs an active audio session | Background modes `bluetooth-central` (+ `audio` only if you truly play audio, which is App Review 2.5.4). `AVAudioSession` category `.playback`, options `.duckOthers` + `.interruptSpokenAudioAndMixWithOthers`, mode `.voicePrompt` (Apple's guidance for navigation prompts). Use state restoration (`CBCentralManagerOptionRestoreIdentifierKey`). **No relaunch after the user force-quits** (TN3115) |
| Android | A connection may continue in the background, but is closed if the process is killed | Foreground service of type `connectedDevice` with its mandatory notification (API 34+). Permissions `BLUETOOTH_SCAN` (`neverForLocation`) and `BLUETOOTH_CONNECT`. TTS with `AudioAttributes.USAGE_ASSISTANCE_NAVIGATION_GUIDANCE` + `CONTENT_TYPE_SPEECH`, focus `AUDIOFOCUS_GAIN_TRANSIENT_MAY_DUCK`. On Android 15 audio focus needs the top app **or a foreground service**. Background haptics are limited to "attentional" usages |

**Do not promise unlimited background operation.** Test locked-phone behaviour on real devices ([09 §5](09-roadmap-and-integration-checklist.md)).

## 20. How can development start without hardware?

Four levels, cheapest first:

1. **Decoder unit tests.** Feed `ble_bridge/contract/test_vectors.json` into your decoder. No Bluetooth at all.
2. **Fake transport.** Implement your BLE layer behind an interface (`DeviceLink`). Write a `FakeDeviceLink` that replays the vectors and scripted sequences with timing. Build the whole UI against it, including the iOS Simulator, which has no Bluetooth.
3. **Mock peripheral over real BLE.** Any Linux machine with BlueZ ≥ 5.6x (or the Pi itself) runs:
   ```bash
   python3 -m ble_bridge.gatt_server --mock sidewalk   # or glass_door, sensor_failure, service_crash, thermal
   ```
   It serves the **same GATT table and code path** as the real device, from a scripted 30 s scenario loop, and advertises `Shappe-M` (a legacy advertisement fits at most 8 name bytes; read Device Information Model Number, `Shappe Guide MOCK`, to label it before pairing). Add `--mock-audio off` to test the `audio_enabled = 0` path (the device's current setting), and `--mock-device-voice` to test the "device already spoke it" path. Needed packages: `bluez`, `python3-dbus`, `python3-gi`.
4. **Generic tools.** nRF Connect for Mobile (Android: GATT server and advertiser) or LightBlue (virtual peripherals) for ad-hoc tests. On macOS or Windows, a port of the mock to `bless` is possible but **not implemented**.

## 21. What mock BLE simulator is needed?

The `--mock` mode above, PROPOSED and **implemented as a prototype**. Its scenarios:

| Scenario | What happens (30 s loop) |
|---|---|
| `sidewalk` | Walk straight → Person ahead → Person in front, turn left (urgent) → Walk straight → Pole ahead, turn slightly right → Vehicle approaching, move right (level 3) → Walk straight |
| `glass_door` | Glass ahead → Blocked by glass → Turn right → Walk straight |
| `sensor_failure` | Radar lost (degraded, glass off) → "Glass sensor off" → LiDAR lost (down, "Sensors offline") → recovered |
| `service_crash` | Navigation service unreachable for 6 s → restarts with `cue_seq` reset → `service_restarted`, new epoch → ready |
| `thermal` | SoC 82.5 °C, throttling → `thermal_warning` → back to normal |

**Differences from the real device:**
- the mock has perfect timing and no real sensor noise;
- `audio_enabled = 1` unless started with `--mock-audio off`;
- `device_voice` is off unless started with `--mock-device-voice`;
- `simulated_input` is always set;
- it never sends `power_warning` unless scripted;
- RF behaviour (range, interference, MTU) depends on the machine it runs on.

## 22. How can the integration be tested?

See [09 §5](09-roadmap-and-integration-checklist.md). Key tests:
- decoder vs vectors;
- de-duplication across reconnects (`service_crash`);
- stale-alert drop (delay the transport by 2.5 s);
- a locked phone for 30 min;
- a phone call during an alert;
- TalkBack/VoiceOver running during alerts;
- `alert_ack` latency p95.

## 23. Which hardware capabilities are already implemented?

| Capability | Status |
|---|---|
| Sensing, fusion, closed-vocabulary decisions, speech gate, health reporting | CONFIRMED |
| Local voice (espeak-ng) | CONFIRMED in code, but **muted** (`AUDIO_ENABLED=False`) and **no wearable audio hardware** |
| Web dashboard over Wi-Fi (phone browser) | CONFIRMED |
| BLE service, mock device, protocol | PROPOSED; prototype desk-tested, not yet over the air |

## 24. Which features still need development?

- Device side:
  - over-the-air bring-up of the BLE bridge;
  - authenticated pairing with a spoken passkey;
  - a pairing button;
  - local audio hardware;
  - a battery and gauge, then Battery Service and `battery_*` events;
  - haptics;
  - multi-phone policy.
- App side: everything.

See [09](09-roadmap-and-integration-checklist.md).

## 25. What are the exact API and protocol contracts?

- [06 — ICD](06-interface-control-document.md) (normative);
- [`schema_v1.json`](../ble_bridge/contract/schema_v1.json);
- [`vocab_v1.json`](../ble_bridge/contract/vocab_v1.json);
- [`test_vectors.json`](../ble_bridge/contract/test_vectors.json).

---

## 26. Sample pseudocode (portable)

```
const SERVICE   = "37e90001-0882-43f6-ae9b-b733c720c46d"
const INFO      = "37e90002-0882-43f6-ae9b-b733c720c46d"
const ALERT     = "37e90003-0882-43f6-ae9b-b733c720c46d"
const STATE     = "37e90004-0882-43f6-ae9b-b733c720c46d"
const COMMAND   = "37e90005-0882-43f6-ae9b-b733c720c46d"
const EVENTS    = "37e90006-0882-43f6-ae9b-b733c720c46d"

# --- scan ---
scanForPeripherals(serviceUUIDs=[SERVICE])          # iOS background scans REQUIRE the filter
onDiscovered(p, adv): if bondedDeviceKnown and p.id == bonded.id: connect(p) else offerInPicker(p)

# --- connect, secure, discover ---
connect(p):
    link = await p.connect()
    if android: await link.requestMtu(517); if not bonded: await link.createBond()
    await link.discoverServices([SERVICE, "180A"])
    info = decodeJson(await link.read(INFO))          # triggers pairing if not yet bonded
    if info.protocol != 1: state = Incompatible; say("Shappe needs an app update"); return
    if bonded.deviceId and info.device_id != bonded.deviceId: say("This is not your Shappe"); disconnect(); return
    if info.mock: banner("Demo device — not real")
    for c in [STATE, ALERT, EVENTS]: await link.subscribe(c, onNotification)   # serialise on Android
    hello = await command("hello", {protocol: 1, app: "ShappeApp", app_version: VERSION, att_mtu: link.mtu})
    state = Live; startHeartbeatWatchdog(3000 ms)

# --- commands ---
command(name, args = {}):
    id = nextId++
    bytes = json({v: 1, id, cmd: name, args})
    for frame in fragment(0x40, bytes, seq = cmdSeq++, mtu = link.mtu): await link.write(COMMAND, frame, withResponse = true)
    return await pending.wait(id, timeout = 3 s)    # resolved by a 0x30 RESPONSE on EVENTS

# --- heartbeat ---
onState(rec): lastStateAt = now(); dashboard.update(rec); if rec.status >= 2: systemAlert(rec.status)
watchdog every 500 ms: if now() - lastStateAt > 3000 ms and state == Live: state = Stale; warn("Lost contact with Shappe. Use your cane.")

# --- disconnect / reconnect ---
onDisconnected(reason): state = Reconnecting; reassemblers.clear(); pending.failAll("disconnected")
    if userInitiated: state = Idle; return
    retry with backoff [1, 2, 4, 8, 15, 15, …] s: connect(bonded.peripheral)
userDisconnect(): if session: await command("session_stop"); link.cancel(); state = Idle
```

**Platform notes (framework still to be decided):**
- **Native iOS (Core Bluetooth):** subscribe with `setNotifyValue(true, for:)`. MTU via `maximumWriteValueLength(for: .withoutResponse)` + 3 (the `.withResponse` value is the 512-byte attribute limit, not the MTU). Speech: `AVSpeechSynthesizer`, with `prefersAssistiveTechnologySettings` so VoiceOver's voice and rate apply. VoiceOver announcements via `UIAccessibility.post(.announcement)` (iOS 17+: priority attributes).
- **Native Android:** use one serial GATT operation queue (Android has no internal queue). Handle status 133 with a retry after `close()`. Notifications need both `setCharacteristicNotification` and the CCCD write. Speech: `TextToSpeech` with `setAudioAttributes`.
- **Flutter:** `flutter_reactive_ble` (BSD-3) or `flutter_blue_plus` (**commercial licence needed for for-profit use**, per its v1.6 licence).
- **React Native:** `react-native-ble-plx` (licence metadata conflicts: Apache-2.0 repo vs MIT on npm; check before use) or `react-native-ble-manager` (Apache-2.0, New Architecture only from RN 0.76).
- **Kotlin Multiplatform:** Kable (Apache-2.0).

Choosing the stack is an open decision ([09 §9](09-roadmap-and-integration-checklist.md)). Speech and audio-session handling are the hardest parts in any cross-platform stack; plan for native modules there.

## 27. State to keep in the app

| State | Persist? | Notes |
|---|---|---|
| Bonded device: platform id, `device_id`, name | Yes | Never the MAC on iOS (unavailable) |
| Last handled `(epoch, cue_seq)` | No | Reset when the connection's epoch changes |
| Phrase registry revision embedded in the app | Built in | Plus a fallback to ALERT_TEXT |
| User preferences (voice, rate, haptics, verbosity) | Yes, locally | Not sent to the device in v1 |
| Diagnostic log | Opt-in, 7-day default | Never uploaded without consent |
