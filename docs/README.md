# Shappe Guide: Engineering Documentation Package

**An AI-assisted obstacle-warning wearable for blind and visually impaired pedestrians, and the Bluetooth companion app to be built for it.**

| | |
|---|---|
| Prepared | 2026-10-09, from the code in `~/Documents/lidar` (HEAD `e209c48` plus the uncommitted working tree) and read-only checks on the device |
| Audience | The hardware/AI team and the mobile application developer |
| Single PDF | [`MASTER.pdf`](MASTER.pdf) (generated from these files by [`build_master.py`](build_master.py); also [`MASTER.md`](MASTER.md) and [`MASTER.html`](MASTER.html)) |
| Status labels | **CONFIRMED**: seen in code or on the device · **HW-VERIFIED**: manufacturer documentation · **PROPOSED**: recommended design, not in the product · **NOT IMPLEMENTED**: needed and missing · **NEEDS VALIDATION**: must be measured or tested on hardware |

## Documents

| # | Document | What it answers |
|---|---|---|
| 01 | [Project Overview and Purpose](01-project-overview.md) | What it is, why, for whom, what works today and what doesn't |
| 02 | [Complete Hardware Architecture](02-hardware-architecture.md) | Every component (15 questions each), field of view, power, thermal, conflicts, performance budget |
| 03 | [AI Perception and Navigation Pipeline](03-perception-and-navigation-pipeline.md) | The actual algorithms, from laser point to spoken sentence; latency; calibration |
| 04 | [Bluetooth Communication Architecture](04-bluetooth-communication-architecture.md) | Why BLE, roles, GATT, protocol rationale, security, privacy, **Pi implementation guide** |
| 05 | [Mobile Developer Integration Guide](05-mobile-developer-integration-guide.md) | **Start here if you build the app.** 25 questions answered, pseudocode, mock device |
| 06 | [Interface Control Document](06-interface-control-document.md) | The exact contract: UUIDs, bytes, JSON, enums, errors, state machine, traceability |
| 07 | [User Journeys and Accessibility Design](07-user-journeys-and-accessibility.md) | Scenarios A–E, alert model, screen-by-screen spec, background and audio behaviour |
| 08 | [Safety, Reliability and Failure Management](08-safety-reliability-failure-management.md) | Hazards, FMEA, degraded modes, the critical local-audio gap, privacy, standards scope |
| 09 | [Roadmap and Integration Checklist](09-roadmap-and-integration-checklist.md) | 12 phases, ownership, test plan with pass criteria, risks, open decisions |
| 10 | [Architecture and Sequence Diagrams](10-architecture-and-sequence-diagrams.md) | Layered, component and communication diagrams, plus 16 sequence diagrams |
| — | [References](references.md) | Every external source, with access dates and contradictions |
| — | [SHAPPE_GUIDE_AUDIT.md](SHAPPE_GUIDE_AUDIT.md) | The 2026-09-29 safety audit (pre-existing) |

**Machine-readable contract and prototype code** ([`../ble_bridge/`](../ble_bridge/README.md), PROTOTYPE):
- `contract/schema_v1.json`: JSON Schema;
- `contract/vocab_v1.json`: 300-phrase registry (revision 3);
- `contract/test_vectors.json`: 16 golden vectors (75 frames);
- the reference bridge and a mock device;
- 34 desk tests (all pass).

---

## Executive summary

**The device exists and works in code. The phone link does not exist yet, and the device cannot currently make any sound.**

- **Built (CONFIRMED):**
  - **Sensing:** a Raspberry Pi 5 fuses a 360° RPLIDAR A2M12 (15 Hz), six 60 GHz Acconeer A121 radars (for glass), and four IMX519 cameras read in turn through an Arducam UC-512, with YOLO on a Hailo-10H.
  - **Decisions:** it decides several times a second on **one sentence from a closed vocabulary of 300**. A speech gate (urgency levels 0–3, confirmation, minimum gaps, repeat back-off) decides when to say it, and a counter `cue_seq` marks each announcement.
  - **Safety rules:** LiDAR is the geometry backbone; radar overrides only for glass; cameras name things but never clear a path; stale or simulated data never produces guidance; nothing ever says it is safe to cross.
- **Not built:**
  - **Bluetooth:** none in the product (NOT IMPLEMENTED).
  - **Audio:** the device is **muted** (`AUDIO_ENABLED = False`), and its only audio output is HDMI, which the voice refuses. **With no phone, the device warns nobody. This is the most important gap.**
  - **Other hardware:** no battery gauge, no haptics, no pairing button.
  - **Detection gaps:** drop-offs (stairs, kerbs) and head-height obstacles are not detected.
- **Designed and prototyped (PROPOSED):**
  - **Architecture:** a BLE GATT service in a **separate bridge process** that reads the existing loopback API, so a Bluetooth fault cannot affect guidance.
  - **Protocol:** 20-byte ALERT and STATE records that fit the smallest BLE packet, JSON events and commands, an append-only phrase registry, `(epoch, cue_seq)` de-duplication, and a 2 s freshness rule.
  - **Prototype status:** desk-tested and BlueZ-checked, but **not yet run over the air** (the adapter is rfkill-blocked on purpose).
- **Recommendation:**
  1. Fit wired bone-conduction audio on the device first.
  2. Freeze ICD v1 so the mobile developer can start immediately against the test vectors and the `--mock` device.
  3. Bring BLE up over the air under supervision.
  4. Measure latency end to end.
  5. Validate in stages: bench, controlled hall, then supervised field tests.

---

## The ten closing items

### 1. Complete project architecture

There are seven layers:
1. **Sensing:** LiDAR, 6 × radar, 4 × camera, IMU.
2. **Interfaces:** CP2102 UART, CH342 ttyACM, CSI-2 with I²C channel switching, PCIe/HailoRT, SPI, BlueZ.
3. **Processing:** reader and analysis threads → `shared_state` with freshness stamps.
4. **Intelligence:** `walk_policy` → `simple_guidance` (sentence → smoothing → gate) → `unified_guidance` commit point → `latest_guidance` snapshot.
5. **Communication:** FastAPI :8000 today; `ble_bridge` (P).
6. **App (P).**
7. **User.**

See [10 §1–3](10-architecture-and-sequence-diagrams.md) and [03 §2](03-perception-and-navigation-pipeline.md).

### 2. Hardware-to-mobile communication architecture

- **Roles:** the Pi is the BLE peripheral and GATT server; the phone is the central.
- **Bridge process:** polls `/guidance/walk` at 10 Hz and `/health` at 1 Hz, then notifies ALERT on each `cue_seq` step, STATE at 1–5 Hz (the heartbeat), and EVENTS (JSON).
- **Commands:** the phone writes JSON commands; responses arrive on EVENTS.
- **Security:** encrypted characteristics; v1 uses Just Works (lab only); production uses a spoken passkey plus a button.

See [04](04-bluetooth-communication-architecture.md).

### 3. Detailed user journey

1. Power on.
2. Spoken onboarding with a safety statement.
3. Find the device by service UUID.
4. OS pairing dialog.
5. "Connected to Shappe. All sensors working."
6. "Start walk".
7. During the walk, device sentences (spoken by the device or relayed by the phone exactly once).
8. On link loss: "Lost contact with Shappe. Use your cane."
9. Automatic reconnect.
10. "End walk".

Scenarios A–E are in [07](07-user-journeys-and-accessibility.md).

### 4. Final BLE interface specification

The service is `37e90001-0882-43f6-ae9b-b733c720c46d`:

| Characteristic | UUID | Properties |
|---|---|---|
| PROTOCOL_INFO | `…0002` | read |
| ALERT | `…0003` | read/notify |
| STATE | `…0004` | read/notify |
| COMMAND | `…0005` | write |
| EVENTS | `…0006` | notify |

It also exposes Device Information `0x180A`.

The wire format:
- a 6-byte frame header;
- 14-byte records;
- 11 error codes, 12 commands and 14 events;
- normative app rules A1–A10.

See [06](06-interface-control-document.md) and `ble_bridge/contract/`.

### 5. Mobile application requirements

Modules:
- discovery;
- connection and reconnection (a state machine with a 3 s stale timer);
- dashboard;
- live alerts (A1–A10, five categories);
- accessibility settings;
- session controls;
- diagnostics;
- privacy.

Requirements:
- **Platform:** iOS `bluetooth-central` with an audio session in `.voicePrompt` mode; Android `connectedDevice` foreground service with `USAGE_ASSISTANCE_NAVIGATION_GUIDANCE`.
- **Accessibility:** VoiceOver and TalkBack throughout; targets 44 pt / 48 dp.
- **Behaviour:** no time limits; no app-invented guidance.

See [05](05-mobile-developer-integration-guide.md) and [07](07-user-journeys-and-accessibility.md).

### 6. Hardware and frontend responsibilities

| Party | Owns |
|---|---|
| Hardware/AI | Sensors, AI, fusion, events, GATT server, health, **local alerts** |
| Mobile | Permissions, pairing UX, decoding, TTS, haptics, accessibility, lifecycle |
| Shared | Protocol, alert mapping, security, latency and user tests |

See [09 §3](09-roadmap-and-integration-checklist.md).

### 7. End-to-end integration checklist

There are 17 checkable items across the device side, the phone side and joint tests. See [09 §4](09-roadmap-and-integration-checklist.md).

### 8. Implementation priorities

1. Local audio.
2. Hardware measurements: camera HFOV, radar bearings, LiDAR RPM.
3. Freeze the ICD, then the decoder and fake transport.
4. Bridge over the air: mock first, then live.
5. End-to-end alerts with latency measurement.
6. Accessibility and audio sessions.
7. Production pairing, privacy and LAN lockdown.
8. Battery gauge and haptics.

### 9. Major technical risks

- The silent device (no local audio).
- iOS background and speech limits.
- The iOS pairing trigger on ATT error 0x0F.
- The assumed camera HFOV (70° in the code vs ~57° derived).
- Unproven head-on glass detection.
- Drop-off and overhead blind spots.
- 2.4 GHz coexistence.
- An uncommitted working tree with no test suite in this checkout.

See [09 §7](09-roadmap-and-integration-checklist.md) and [08 §6](08-safety-reliability-failure-management.md).

### 10. Recommended development approach

1. The device owns safety; the phone is a companion.
2. Keep BLE out of process, and allow no command that can change guidance.
3. Freeze a small, versioned, test-vector-backed protocol now and develop app and hardware in parallel.
4. Measure before claiming anything; validate in stages under supervision.

---

## What this package did *not* do

- **The navigation service:** not started. It spins the LiDAR and opens every sensor, so it needs supervision.
- **Bluetooth:** the adapter was **not** unblocked and nothing was advertised. Only read-only `--check` ran.
- **Existing files:** no production file was modified, and nothing was committed. New files: `docs/01–10`, `docs/README.md`, `docs/references.md`, `docs/build_master.py`, `docs/MASTER.*` and `ble_bridge/`.
- **Device measurements:** quoted from the README (2026-09-25) or from read-only commands run on 2026-10-09. Everything else that needs hardware is marked NEEDS VALIDATION.
