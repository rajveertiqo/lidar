# 01 — Project Overview and Purpose

**Shappe Guide: an AI-assisted obstacle-warning wearable for blind and visually impaired pedestrians**

| | |
|---|---|
| Audience | Everyone on the project: the hardware/AI team and the mobile developer |
| Source of truth | The code in this repository (`~/Documents/lidar`, HEAD `e209c48` plus uncommitted work), inspected 2026-10-09 |
| Status labels | **CONFIRMED** = seen in the code or on the device · **HW-VERIFIED** = manufacturer documentation · **PROPOSED** = recommended, not built · **NOT IMPLEMENTED** = needed, missing · **NEEDS VALIDATION** = must be tested on hardware |

---

## 1. What the project is

### Level 1: simple explanation

The user wears a small computer, a Raspberry Pi 5, with sensors pointing in every direction:
- a spinning laser (LiDAR) that measures distance all the way round;
- four cameras that recognise what things are (a person, a car, a pole);
- six small radars that catch glass, which the laser sees straight through;
- a motion sensor (gyro).

Several times a second the device decides on **one short sentence** from a fixed list, for example "Walk straight", "Wall ahead", "Blocked by glass", "Person in front, turn left" or "Vehicle approaching, move right". It is built to say a sentence only when that sentence is new and matters, never to talk over itself, and to say nothing rather than guess.

The mobile app (to be built) will connect to the device over Bluetooth. It shows the device's state, speaks or vibrates its warnings when the device cannot, and lets the user check health and settings, all without needing to look at the screen.

### Level 2: engineering summary

| Layer | What exists today | Status |
|---|---|---|
| Sensing | RPLIDAR A2M12 (360°, 15 Hz at 900 RPM); Arducam UC-512 CamArray with 4 × IMX519, read one camera at a time (`CAMERA_CYCLE`); 6 × Waveshare A121 60 GHz radar boards on a USB hub; MPU-6500 IMU on SPI | CONFIRMED |
| AI | YOLO detector (`model/yolo26m.hef`, COCO-80) and segmentation (`model/best.hef`) on a Hailo-10H (Raspberry Pi AI HAT+ 2) through HailoRT 5.1.1 | CONFIRMED |
| Fusion and decision | LiDAR geometry, with radar overriding it only for glass. Cameras give identity only. `walk_policy` → `simple_guidance` (smoothing, then the speech gate) decides one sentence from a closed vocabulary of 300 sentences | CONFIRMED |
| Output | One commit point (`unified_guidance`) hands the sentence to `pi_voice` (espeak-ng) and to a web dashboard (FastAPI on port 8000) that can speak it in a phone browser | CONFIRMED |
| **Sound today** | **`AUDIO_ENABLED = False`. Nothing speaks, Pi or phone.** The only audio sink on the Pi is HDMI, which the voice refuses | CONFIRMED, see §6 |
| Bluetooth | **No Bluetooth code in the product.** The Pi 5 radio exists but is rfkill soft-blocked. A BLE bridge prototype has been written for these documents (`ble_bridge/`) | NOT IMPLEMENTED in product; PROPOSED prototype, desk-tested |
| Battery, haptics, buttons | None in the code or on the device | NOT IMPLEMENTED |

## 2. Why the project is needed

A blind pedestrian's main tools leave real gaps:

- **White cane.** It gives reliable, immediate contact information about the ground about one stride ahead. It cannot sense anything above waist height (a truck mirror, a branch), anything beyond its tip, or anything moving towards the user.
- **Guide dog.** It is excellent but scarce and expensive, and not suitable for everyone.
- **Phone navigation apps (GPS).** These say *where* to go but not *what is in the way*. GPS accuracy is metres, not the centimetres that matter at a door.
- **Glass.** Glass doors and walls are a classic hazard: vision-based aids and laser scanners can both miss them.

This project targets the band **between the cane tip and about 4 m**, all the way round the user. It warns about things the cane cannot reach in time: walls and glass the user is closing on, people and vehicles approaching, and objects in the walking line. It also gives a steering hint ("turn slightly left").

## 3. Who benefits

- **Primary user.** A blind or severely visually impaired adult pedestrian who already travels independently with a cane or guide dog and has orientation and mobility (O&M) training. The device **adds** information; it does not replace those skills.
- **Secondary users.** O&M instructors and field testers. The dashboard, speech log and "mark" commands exist for them today.
- **Developers.** The hardware/AI team and the mobile developer.

## 4. What the user experiences (the design intent)

The design in the code, CONFIRMED in `utils/simple_guidance.py`:

1. **One voice and a closed vocabulary.** Every sentence comes from a fixed list (`ALLOWED`: 300 sentences built from about 20 templates and the object words). Nothing free-form is ever spoken.
2. **Say it once.** A sentence is announced when it changes. A "blocked" situation repeats after 30 s, then 60 s, then 120 s at most.
3. **Urgency levels 0–3:**
   - 0: "Walk straight".
   - 1: advisories, for example "Wall ahead" or "Person ahead".
   - 2: blocked or turn, for example "Blocked by glass" or "Person in front, turn left".
   - 3: vehicle approaching.
   A more urgent sentence is spoken as soon as the current one ends, and a vehicle warning does not wait. Urgent sentences (blocked, vehicle, mover) also stop whatever the device voice is saying.
4. **Silence beats guessing.** With no LiDAR the device says "Sensors offline". With stale data it holds and says nothing new. At a road crossing it **withholds** "Walk straight" for 5 s after the last traffic light was seen, and **never** says it is safe to cross.

## 5. Core capabilities: confirmed vs not

| Capability | Status | Notes |
|---|---|---|
| Distance to walls and objects all round, in a horizontal plane | CONFIRMED | LiDAR, 8 × 45° sectors, 4 × 10° cones plus a body-width corridor |
| "Blocked" when anything is within 4 ft (~1.2 m) ahead, with a turn suggestion | CONFIRMED | `WALK_BLOCK_FT = 4.0` |
| Glass ahead or beside, via radar disagreeing with laser | CONFIRMED in code; NEEDS VALIDATION in the field | README: "head-on flat glass is still unproven" |
| Naming objects (person, car, bus, bicycle, dog, pole …) | CONFIRMED (cycle mode: up to 2 cameras with new frames per pass, front first) | COCO-80 model; the classes are filtered to `DETECTION_CLASSES` |
| Vehicle approaching (camera box growth) | CONFIRMED in code; NEEDS VALIDATION | Front camera only |
| Moving people and vehicles from LiDAR (`lidar_movers`) | CONFIRMED in code, **silent trial** | `LIDAR_MOVERS_SPEAK = False`: logged as "would say" |
| Drift and veer warnings | CONFIRMED in code, near-wall/drift sentences only | Collision-course drift is computed but not spoken |
| Overhead, low or drop-off hazards (stairs down, kerbs, potholes) | **NOT IMPLEMENTED** | A 2D LiDAR sees one plane; the COCO model has no stairs or pothole classes |
| Safe-to-cross decisions | **Deliberately not implemented** | Traffic-light colour is never read |
| Battery level, thermal alerts to the user | NOT IMPLEMENTED | The BLE prototype reads SoC temperature and throttle flags |
| Bluetooth companion app | NOT IMPLEMENTED; PROPOSED in docs 04–07 | Prototype bridge in `ble_bridge/` |

## 6. Realistic limitations (read before any field use)

- **The device is currently silent.** `AUDIO_ENABLED = False` (`config.py`) turns off the Pi voice and the dashboard's browser speech. Even with it on, the Pi has **no wearable audio output**: `wpctl` shows only the HDMI sink, which `pi_voice` refuses. Until a local audio device is fitted, the device cannot warn anyone by itself. This is the most important architectural gap; see [08 §4](08-safety-reliability-failure-management.md).
- **The device is not validated.** The README and the 2026-09-29 audit (`docs/SHAPPE_GUIDE_AUDIT.md`) list physical validation as pending:
  - glass detection head-on;
  - heading and drift on real walks;
  - mover warnings;
  - camera field of view.
- **Coverage gaps:**
  - one LiDAR plane (no overhead or kerb detection);
  - cameras cover four directions with gaps between them (each lens is ~57–67° wide; see [02 §4](02-hardware-architecture.md));
  - radar reaches 0.25–4.0 m.
- **This is not a certified medical or assistive device.** It must not be presented as one, or as a replacement for the cane, guide dog, O&M training or the user's own judgement.

## 7. Non-goals for the first Bluetooth integration

- No video or raw sensor streaming over Bluetooth.
- No phone-side obstacle detection.
- No route navigation or GPS.
- No remote changes to safety thresholds.

These are discussed as future options in [09 §8](09-roadmap-and-integration-checklist.md).

## 8. How to read this documentation set

| Doc | For |
|---|---|
| [02 Hardware](02-hardware-architecture.md) | What each part is, how it connects, power, heat, and limits |
| [03 Perception and navigation](03-perception-and-navigation-pipeline.md) | Exactly how a measurement becomes a sentence |
| [04 Bluetooth architecture](04-bluetooth-communication-architecture.md) | Why BLE, the GATT design, security, and the Pi-side implementation |
| [05 Mobile developer guide](05-mobile-developer-integration-guide.md) | **Start here if you are building the app** |
| [06 ICD](06-interface-control-document.md) | The exact contract: bytes, JSON, UUIDs, rules |
| [07 Journeys and accessibility](07-user-journeys-and-accessibility.md) | How a blind user uses it, and a screen-by-screen spec |
| [08 Safety](08-safety-reliability-failure-management.md) | Hazards, FMEA, degraded modes, privacy |
| [09 Roadmap](09-roadmap-and-integration-checklist.md) | Phases, owners, tests, checklist, open questions |
| [10 Diagrams](10-architecture-and-sequence-diagrams.md) | All Mermaid architecture and sequence diagrams |
| [References](references.md) | Every external source, with access dates |
