# 10 — System Architecture and Sequence Diagrams

All diagrams are Mermaid. In each diagram, solid parts are **CONFIRMED** (code), and parts marked *(P)* are **PROPOSED** (the BLE bridge, the app, local audio, the button, the battery).

---

## 1. Layered system architecture

```mermaid
flowchart TB
  L1["Layer 1 - Sensing<br/>RPLIDAR A2M12 · 6x Waveshare A121 radar · UC-512 + 4x IMX519 · MPU-6500"]
  L2["Layer 2 - Interfaces and drivers<br/>CP2102 UART 256000 · CH342 ttyACM x6 (exptool) · CSI-2 + I2C 0x24 channel switch<br/>PCIe Gen3 + HailoRT 5.1.1 · SPI0 CE0 · BlueZ 5.82 over D-Bus"]
  L3["Layer 3 - Data processing<br/>lidar_utils · radar_utils · pi_camera_utils + hailo_inference · gyro_reader, heading_tracker<br/>into shared_state with wall-clock + monotonic freshness stamps"]
  L4["Layer 4 - Navigation intelligence<br/>walk_policy, then simple_guidance (sentence, smoothing, speech gate), then the unified_guidance commit point<br/>advisory only: drift · BEV grid · spatial scan · movers"]
  L5["Layer 5 - Communication<br/>FastAPI :8000 (dashboard, REST, WebSocket) · ble_bridge GATT server (P)"]
  L6["Layer 6 - Mobile app (P)<br/>discovery · connection · dashboard · alerts (TTS + haptics) · settings · diagnostics"]
  L7["Layer 7 - End user<br/>blind or VI pedestrian with cane or dog · VoiceOver / TalkBack"]
  L1 --> L2 --> L3 --> L4 --> L5 --> L6 --> L7
  L4 -. "pi_voice local audio: muted, no wearable sink today (P: bone conduction)" .-> L7
  L5 -. "Wi-Fi browser dashboard (today)" .-> L7
```

The per-module data flow is in [03 §2](03-perception-and-navigation-pipeline.md), and the BLE process layout is in [04 §8.1](04-bluetooth-communication-architecture.md).

## 2. Component diagram (processes and threads)

```mermaid
flowchart LR
  subgraph P1["process: shappe-guide (uvicorn main:app)"]
    direction TB
    T1["thread: LiDAR reader"] --> T2["thread: LiDAR analysis<br/>(+heading, movers, drift, BEV)"]
    T3["threads: radar supervisor x6"]
    T4["thread: camera capture (cycle)"] --> T5["thread: detection (Hailo)"]
    T6["thread: seg (only while viewed)"]
    T7["thread: gyro 100 Hz"]
    T8["thread: guidance loop 5 Hz<br/>(pi_voice._loop)"]
    T9["thread: pi-voice speaker"]
    T10["thread: watchdog / sd_notify"]
    EL["asyncio event loop<br/>(HTTP, WebSockets)"]
  end
  subgraph P2["process: shappe-ble (P)"]
    B1["poll thread 10 Hz"]
    B2["GLib main loop (D-Bus)"]
    B3["command workers"]
  end
  SYSD["systemd --user<br/>WatchdogSec=30"] --- P1
  SYSD -. "(P)" .- P2
  T8 -->|latest_guidance| EL
  B1 -->|"GET /guidance/walk, /health"| EL
  B3 -->|"GET /guidance/log, POST /mark"| EL
  B2 <-->|D-Bus| BLUEZ["bluetoothd"]
```

## 3. Communication flows

```mermaid
flowchart TB
  A["guidance loop"] -- "cue_seq step" --> B["latest_guidance snapshot"]
  B -- "HTTP 10 Hz" --> C["bridge: Bridge.on_walk"]
  C -- "ALERT 20 B + ALERT_TEXT" --> D["ALERT notify"]
  C -- "STATE 20 B, 1-5 Hz" --> E["STATE notify"]
  H["/health + /guidance/status + SoC"] -- "HTTP 1 Hz" --> F["bridge: Bridge.on_health"]
  F -- "JSON events" --> G["EVENTS notify"]
  APP["app"] -- "JSON command" --> CMD["COMMAND write"] --> W["bridge: on_command"] -- "RESPONSE" --> G
  D --> APP
  E --> APP
  G --> APP
```

## 4. Sequence diagrams

### 4.1 Hardware boot and initialisation

```mermaid
sequenceDiagram
  autonumber
  participant SD as systemd (user)
  participant SH as start_lidar.sh
  participant M as main.py lifespan
  participant S as Sensors
  participant G as guidance loop
  participant SG as startup_gate
  participant BR as shappe-ble (P)
  SD->>SH: start shappe-guide (Type=notify)
  SH->>SH: admin token, LiDAR port, radar check, hailortcli scan, deps
  SH->>M: exec uvicorn main:app :8000
  M->>S: start camera, gyro, LiDAR (soft start), radar (by serial)
  M->>G: pi_voice.start() - loop 5 Hz (voice muted: AUDIO_ENABLED False)
  M->>SD: READY=1, watchdog heartbeats (guidance, event_loop)
  loop every 200 ms
    G->>SG: check()
    SG-->>G: hold (scanning / settling) until LiDAR stable (6 samples within 1.5 ft)
  end
  SG-->>G: ready
  SD->>BR: start after shappe-guide (P)
  BR->>BR: power adapter, register GATT app + advert + agent, pairing window 180 s
  BR->>M: GET /health
  M-->>BR: status ok/degraded
  BR->>BR: event device_ready (only reaches a phone already subscribed - events are never queued)
```

### 4.2 Bluetooth discovery and pairing (P)

```mermaid
sequenceDiagram
  autonumber
  actor U as User
  participant APP as App
  participant OS as Phone OS BLE
  participant BZ as bluetoothd
  participant BR as Bridge
  U->>APP: Find my Shappe
  APP->>OS: scan(serviceUUIDs=[37e90001-...])
  BZ-->>OS: ADV_IND (flags, service UUID, "Shappe")
  OS-->>APP: discovered Shappe
  APP-->>U: One Shappe found (spoken)
  U->>APP: Connect
  APP->>OS: connect
  APP->>OS: read PROTOCOL_INFO
  OS->>BZ: ATT Read
  BZ-->>OS: Error 0x0F Insufficient Encryption
  OS-->>U: system pairing dialog (v1 Just Works / production passkey spoken by the device)
  OS->>BZ: SMP pairing (LE Secure Connections)
  BZ->>BR: Agent.RequestAuthorization (only inside the pairing window)
  BR-->>BZ: OK
  BZ-->>OS: bonded, link encrypted
  OS->>BZ: ATT Read (retry)
  BZ->>BR: ReadValue(PROTOCOL_INFO)
  BR-->>APP: frame 0x01 JSON (device_id, protocol, vocab)
  APP->>APP: store device_id as owner
```

### 4.3 First successful connection (handshake)

```mermaid
sequenceDiagram
  autonumber
  participant APP as App
  participant BR as Bridge
  APP->>BR: enable notify STATE
  BR-->>APP: STATE (sent at once to a new subscriber)
  APP->>BR: enable notify ALERT, EVENTS
  APP->>BR: COMMAND hello {protocol 1, att_mtu}
  BR-->>APP: RESPONSE ok (PROTOCOL_INFO + session + status)
  APP->>BR: COMMAND get_status
  BR-->>APP: RESPONSE status (sensors, warnings, battery null)
  APP-->>APP: state Live, heartbeat watchdog 3 s
  Note over APP: speaks "Connected to Shappe. All sensors working."
```

### 4.4 Starting a navigation session

```mermaid
sequenceDiagram
  autonumber
  actor U as User
  participant APP as App
  participant BR as Bridge
  participant NAV as Guidance loop
  Note over NAV: guidance is ALWAYS running - a session only labels the walk
  U->>APP: Start walk
  APP->>BR: COMMAND session_start
  BR-->>APP: RESPONSE {id, started}
  BR-->>APP: EVENT session_started
  APP-->>U: "Walk started. Shappe is guiding."
```

### 4.5 Camera obstacle detection

```mermaid
sequenceDiagram
  autonumber
  participant CAM as UC-512 (cycle)
  participant CAP as capture thread
  participant DET as detection thread
  participant NPU as Hailo-10H
  participant SS as shared_state
  participant SG as simple_guidance
  CAP->>CAM: I2C 0x24 = front channel
  CAM-->>CAP: frame (SensorTimestamp after switch)
  CAP->>SS: front quadrant + capture time
  DET->>NPU: letterbox 640 -> yolo26m.hef
  NPU-->>DET: boxes, classes, scores
  DET->>DET: DFL decode, NMS, keep DETECTION_CLASSES only
  DET->>SS: camera_state.front.detections (fresh 2 s)
  SG->>SS: read camera + LiDAR front_ft
  SG->>SG: front_object: centre band, size, range agrees with laser -> "Person"
```

### 4.6 LiDAR obstacle detection

```mermaid
sequenceDiagram
  autonumber
  participant L as RPLIDAR (15 Hz)
  participant R as reader thread
  participant A as analysis thread
  participant SS as shared_state
  participant WP as walk_policy
  L-->>R: serial packets 256000 baud
  R->>R: packet sync, drop backlog over 0.25 s, reject partial scans
  R->>A: mailbox (newest scan only)
  A->>A: 8 sectors, 4 cones 10 deg, corridor 300 mm to 2.5 m
  A->>A: cluster or persistence check, mark unswept
  A->>SS: lidar_state (stamp + monotonic receive)
  A->>A: heading -> movers -> drift (8 Hz) -> BEV
  WP->>SS: read
  WP->>WP: front_ft below 4 ft -> blocked, choose turn
```

### 4.7 Radar data processing

```mermaid
sequenceDiagram
  autonumber
  participant B as A121 board (x6)
  participant RT as board thread
  participant SG as StallGuard
  participant P as radar publish
  participant SS as shared_state
  B-->>RT: distance detector result (CFAR, PLANAR, P5, 0.25-4 m, 6 Hz)
  RT->>SG: progress tick
  alt no progress for 5 s (30 s while calibrating)
    SG->>RT: force link timeout -> reconnect, verify identity by USB serial
  end
  RT->>P: echoes (mm, strength) for this serial's bearing
  P->>P: per cardinal: nearest live echo of overlapping boards
  P->>SS: radar_state (online, min_dist, sources, sources_live)
```

### 4.8 Sensor fusion and hazard assessment

```mermaid
sequenceDiagram
  autonumber
  participant G as guidance loop (5 Hz)
  participant WP as walk_policy
  participant SG as simple_guidance
  participant W as _World
  participant GT as _Gate
  G->>WP: decide(lidar, radar)
  WP->>WP: per direction - average, or nearer measured wins if gap >= 2 ft (glass)
  WP-->>G: action, front_ft, glass_ahead, directions
  G->>SG: guidance(walk, lidar, radar, camera, motion, movers)
  SG->>SG: decide - vehicle > mover > blocked(glass > wall > ...) > advisories > Walk straight
  SG->>W: smooth (attack 0.2 s, release 1 s)
  W->>GT: candidate sentence + level
  GT->>GT: urgent cut-in? confirm 1 s? gap 3 s? settle 6 s? repeat back-off?
  GT-->>G: speak true/false, cue_seq (+1 when admitted)
```

### 4.9 Hazard notification delivery

```mermaid
sequenceDiagram
  autonumber
  participant UG as unified_guidance (commit)
  participant PV as pi_voice
  participant API as /guidance/walk
  participant BR as Bridge (P)
  participant APP as App (P)
  actor U as User
  UG->>PV: speak(text, urgent, seq) - "muted" today
  UG->>API: latest_guidance snapshot (cue_seq N)
  BR->>API: GET (10 Hz)
  API-->>BR: notify.cue_seq N (was N-1), instruction
  BR->>APP: ALERT 20 B {epoch, N, age_ms, phrase, level, flags}
  BR->>APP: ALERT_TEXT (fragmented if MTU 23)
  APP->>APP: not replay, newer cue, age below 2 s, audio enabled, not device voice
  APP-->>U: speak phrase (urgent interrupts) + haptic
  APP->>BR: COMMAND alert_ack {outcome spoken, latency}
```

### 4.10 Mobile-to-hardware command

```mermaid
sequenceDiagram
  autonumber
  actor U as User
  participant APP as App
  participant BZ as bluetoothd
  participant GS as gatt_server
  participant W as worker thread
  participant API as navigation service
  U->>APP: What's around me?
  APP->>BZ: Write COMMAND frames {v1, id 12, get_surroundings}
  BZ->>GS: WriteValue(frame, mtu)
  GS->>GS: reassemble (2 s limit)
  GS->>W: on_command (off the D-Bus thread)
  W->>W: read cached walk snapshot (obstacles.items)
  W-->>GS: RESPONSE {id 12, ok, items}
  GS->>APP: EVENTS notify (fragments)
  APP-->>U: "Approximate: Person, 12 o'clock, 9 feet."
```

### 4.11 Bluetooth disconnection

```mermaid
sequenceDiagram
  autonumber
  participant APP as App
  participant BR as Bridge
  participant NAV as Guidance loop
  participant V as Local audio (P)
  actor U as User
  Note over APP,BR: link drops (range, interference)
  BR->>BR: Device1 Connected=false - logged (counters restart on the next connect)
  NAV->>NAV: continues unchanged (separate process)
  NAV->>V: local warnings continue (only once local audio exists)
  APP->>APP: no STATE for 3 s -> Stale
  APP-->>U: "Lost contact with Shappe. Use your cane." + strong haptic
  APP->>APP: Reconnecting (backoff 1, 2, 4, 8, 15 s)
```

### 4.12 Reconnection

```mermaid
sequenceDiagram
  autonumber
  participant APP as App
  participant BR as Bridge
  APP->>BR: connect (bonded, encrypted automatically)
  APP->>BR: subscribe STATE, ALERT, EVENTS
  BR-->>APP: STATE {epoch E, cue N}
  APP->>BR: COMMAND hello
  BR-->>APP: RESPONSE (epoch, session)
  alt epoch changed (device or service restarted)
    APP->>APP: reset de-duplication
  end
  APP->>APP: never announce the ALERT read value (REPLAY)
  APP-->>APP: Live - "Shappe connected."
```

### 4.13 Low-battery warning

```mermaid
sequenceDiagram
  autonumber
  participant PS as Power supply
  participant FW as Pi firmware
  participant BR as Bridge
  participant APP as App
  actor U as User
  Note over PS,BR: TODAY: no battery gauge (NOT IMPLEMENTED). Only under-voltage is observable.
  PS-->>FW: voltage sags
  BR->>FW: vcgencmd get_throttled (every 5 s)
  FW-->>BR: bit 0 under-voltage now
  BR->>APP: EVENT power_warning + STATE bit 10
  APP-->>U: "Shappe power is low." (WARNING)
  Note over BR,APP: (P) with a fuel gauge - Battery Service 0x180F + battery_low / battery_critical events
```

### 4.14 Sensor failure

```mermaid
sequenceDiagram
  autonumber
  participant L as LiDAR
  participant SG as simple_guidance
  participant H as /health
  participant BR as Bridge
  participant APP as App
  actor U as User
  L--xSG: scans stop
  SG->>SG: stale -> hold, nothing new said
  SG->>SG: scan older than 2 s -> "Sensors offline" (level 2, cue_seq +1)
  H-->>BR: status down, lidar_ok false, warning "LiDAR signal lost"
  BR->>APP: ALERT phrase 14 + EVENT sensor_degraded{lidar} + health_changed
  BR->>APP: STATE status 2
  APP-->>U: "Shappe cannot see. Use your cane." (SYSTEM FAILURE)
```

### 4.15 Hardware / service restart

```mermaid
sequenceDiagram
  autonumber
  participant SD as systemd
  participant NAV as shappe-guide
  participant BR as Bridge
  participant APP as App
  NAV--xBR: HTTP refused (crash)
  BR->>APP: EVENT service_unreachable + STATE status 3 (all bits 0)
  APP-->>APP: SYSTEM FAILURE announcement
  SD->>NAV: Restart=always
  NAV-->>BR: /guidance/walk cue_seq 0 (was 57)
  BR->>BR: epoch +1, learn cue 0 (no alert)
  BR->>APP: EVENT service_restarted {epoch}
  NAV-->>BR: /health ok (speech may still be held by the start-up gate)
  BR->>APP: EVENT device_ready
  APP-->>APP: "Shappe ready."
```

### 4.16 Session termination

```mermaid
sequenceDiagram
  autonumber
  actor U as User
  participant APP as App
  participant BR as Bridge
  U->>APP: End walk
  APP->>BR: COMMAND session_stop
  BR-->>APP: RESPONSE {id, started, duration_s}
  BR-->>APP: EVENT session_stopped
  APP-->>U: "Walk ended, 24 minutes."
  Note over BR: device guidance keeps running - the session is a label, not a switch
  opt user disconnects
    APP->>BR: disconnect
  end
```
