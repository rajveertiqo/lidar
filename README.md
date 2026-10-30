# Blind Assistance System

**Project documentation:** [Complete Project Guide](docs/PROJECT_GUIDE.md) — purpose,
use cases, current feature status, hardware/software stack, architecture, setup,
operation, APIs, simulation tools, training, troubleshooting, and limitations.
Reviewed against the working tree on 2026-10-27.

A wearable navigation aid for blind pedestrians, running on a Raspberry Pi 5.

It watches the world with three sensors that fail in different ways, merges them
into one ranked picture of what is around the user, and reduces that to one
short spoken sentence — `"Walk straight"`, `"Wall ahead"`, `"Chair in front"`.
One voice, never two talking over each other.

---

## What the user hears

> This section describes the guidance as it is NOW (since 2026-09-24). Parts of
> "The safety model" and "Dashboard" below describe the older multi-layer voice
> (drift correction, surface guide, BEV awareness). The drift layer still runs
> and is shown on `/classic`, but does not speak; the surface guide and the BEV
> caution were removed on 2026-10-05, since nothing read their output.

Everything spoken comes from one module, `utils/simple_guidance.py`, from a
closed vocabulary:

| Situation | Sentence |
| --- | --- |
| Path clear | `Walk straight` — said once, never repeated |
| Closing on a wall / glass (within ~12 ft) | `Wall ahead` / `Glass ahead` |
| Front blocked (under 4 ft — was 6.5 ft until 2026-10-01) by a wall / glass | `Blocked by wall` / `Blocked by glass` |
| Front blocked by something unnamed | `Turn left` / `Turn right`, toward more room |
| Walking toward a wall beside you that is under 5 ft away (only while actually moving toward it; 2.5 ft on 2026-09-30, 5 ft since 2026-10-01). Inside 4 ft the near-wall sentence below takes over | `Drifting toward right wall, turn slightly left` (and the mirror) |
| Already within 4 ft of a wall beside you (moving or not; 1.5 ft until 2026-10-01) | `Near right wall, move left` (and the mirror) |
| Glass only one front radar board hears, within 4 ft, and room to step away (added 2026-10-01) | `Glass on right, move left` (and the mirror) |
| Something that can hurt you on the path | `Person ahead` → `Person in front`; also vehicles, animals, `Pole` (stop sign / parking meter / traffic-light posts), `Hydrant`, `Bench` — never chairs, laptops or other furniture (still steered round, unnamed). Vehicles are named first. |
| Something that can hurt you in the way, within ~5 m (camera) | `Person on left, turn slightly right` (and the mirror), `Person ahead, turn slightly left`; two with a gap you fit through: `Walk between them, slightly left` / `Walk straight between them`; inside 4 ft: `Person in front, turn left` |
| A vehicle in the camera's box coming at you (its box growing faster than your own walking explains) | `Vehicle approaching, move left` if it is in the right half of the frame, `move right` if in the left — said over anything else. Just `Vehicle approaching` when the side it would send you to has under 2 ft of room or no reading (decided 2026-09-29) |
| A vehicle moving in the camera's box (crossing, or passing close) — or, while you stand still, a stopped one | `Vehicle in front, be careful` — outranks everything except a pane or wall actually blocking you |
| The LiDAR has stopped updating (or is simulated) | `Sensors offline` — after 2 s; between 1 and 2 s stale nothing new is said |
| The front radar (the only glass sensor) lost, fully or half its arc, for 3 s | `Glass sensor off` — once, only where "Walk straight" would be said and the raw reading is clear too; never over a hazard or steering. At a cold start it waits (up to 30 s) only while the front boards are still starting or calibrating |
| Front radar fully back for 10 s (20 s, 40 s … up to 160 s after each further loss) | `Glass sensor on` — once. It reports the SENSOR, not that every pane will be seen |
| The LiDAR never came up at start-up (20 s) | `Sensors offline` — through the start-up hold, which otherwise stays silent |

The two glass-sensor sentences were added by the 2026-09-29 audit
(`docs/SHAPPE_GUIDE_AUDIT.md`, F2) and are a user-facing change to review before
field deployment.

How it decides, in one paragraph each:

- **Named only when detected.** "Wall" means the laser return walk_policy's
  front reading came from lies on a flat fitted surface ≥ 600 mm wide. "Glass"
  means walk_policy's radar-vs-laser disagreement AND both front radar boards
  echo — glass on one board only is beside the user and is ignored. An echo
  nearer than `RADAR_CONE_TRUST_MM` (~1.1 ft) is inside the walking corridor:
  it is glass even from one board, and a side wall in the arc no longer hides
  it (NEW-045). Since 2026-10-01 each front board is also judged on its own: a
  board whose echo is nearer than anything the laser sees in that board's view
  is hearing glass, even when the other board's nearer echo is a wall beside
  you (a wall the laser sees nearly edge-on, `GLASS_GRAZING_DEG`, does not
  count: the radar hears almost nothing off it); and close to a glass door the
  wall it sits in no longer counts as a side wall. The radar reaches 4 m
  (13 ft) so a pane is named from the 12 ft "Glass ahead" band.
  `tools/glass_sim.py` replays these scenes. Objects are
  named only for something the rangefinders already have on the path, in the
  middle half of the front camera, and big enough to be near — and only for
  things that can hurt you (`SIMPLE_NAMED_OBJECTS` in `config.py`). The
  detector reports nothing else at all (`DETECTION_CLASSES`): a chair, table or
  laptop is dropped inside the engine, so it is not in the hazard list, the BEV
  labels or the dashboard either — the laser still steers round it. The
  detector on the rig is COCO-80, which has no tree or pole class; the list
  already covers the custom 129-class model's trees, poles and bollards for
  when it is deployed. A car the laser reads as a flat "wall" is named as the
  car when its camera-estimated range matches.
- **Getting past things.** Each harmful thing in the front camera is put in
  metres at its own range (its box height against its class's typical height,
  so the lens drops out). It is "in the way" if it overlaps the 0.8 m the user
  needs to pass. The guide aims for the free place nearest straight ahead — a
  gap between two things at least 0.8 m wide, or just past either end of the
  group — keeps the side it already gave, and never steers towards a side the
  laser has closer than 2 ft. Tunables: `SIMPLE_STEER_*`.
- **Vehicles are tracked, not just seen.** A car, truck, bus, motorbike,
  bicycle or train in the camera's box is followed frame to frame. Its box
  growing is it getting closer — the growth rate is 1 / time-to-contact, no
  range needed. The part of that growth your own walking explains is taken
  off, so walking up to a parked car is not "approaching". It must also be on
  course to pass within 2 m of your walking line, so a car in the next lane
  going past a footpath is not announced. Tunables: `SIMPLE_VEHICLE_*` in
  `config.py`; the lens (`SIMPLE_CAMERA_HFOV_DEG`, 70°) is unmeasured.
- **Warn fast, clear slowly.** A hazard is believed after two polls; it is let go
  after a second; the block and approach edges have 1 ft hysteresis. A more
  urgent sentence is said the moment the previous one finishes; anything calmer
  waits a 1 s confirmation and a 3 s gap.
- **One obstacle, one announcement.** Within one obstacle, a change that is not
  more urgent (a name flickering, the side flipping, a step back) must hold 6 s.
  Blocked sentences repeat after 30 s, 60 s, then every 2 min.
- **The Pi speaks for itself** (`utils/pi_voice.py`): the guidance loop runs on
  the device at 10 Hz (`PI_VOICE_TICK_HZ`; 5 Hz until 2026-10-30), and every sentence is spoken with `espeak-ng` through the
  default audio output — plug in or pair an earpiece and it follows. While no
  earpiece is connected (HDMI does not count), the dashboard's screen-reader
  banner is the voice instead; never both.
- **Startup.** Silent until the LiDAR has settled. The radar and camera are waited
  for up to 15 s, then guidance starts without them rather than never.

Every threshold is a `SIMPLE_*` / `PI_VOICE_*` constant in `config.py`.

---

## Hardware

| Part                      | Role                                                      |
| ------------------------- | --------------------------------------------------------- |
| Raspberry Pi 5            | Host                                                      |
| Hailo-10H NPU             | Runs both neural models (object detection + segmentation) |
| RPLIDAR A2M12             | 360° distance and direction, ~15 Hz at 900 RPM            |
| UC-512 kit (4× IMX519)    | Four camera directions, delivered as one 2×2 mosaic frame |
| 6× Acconeer A121 (60 GHz) | Radar, every 60° straddling the cardinals, on a USB hub  |
| MPU-6500 IMU (SPI0 CE0)   | Yaw rate, gait and tilt — heading, not position            |

---

## The safety model

This is the part worth reading before any code.

**Each sensor is trusted only for what it is actually good at.**

- **LiDAR — geometry.** How far, and in which direction, over a full 360° with
  no blind spots. This is the safety backbone and the primary authority on
  whether the user may move.
- **Cameras — identity.** _What_ the thing is (`person`, `car`, `chair`). They
  cannot measure distance and have blind gaps between fields of view.
- **Radar — glass.** This is the one that is not obvious. A laser passes
  straight **through** a glass door and reports the room beyond it, so a
  LiDAR-only guide walks the user face-first into it. 60 GHz radar reflects off
  glass. The radar exists solely to cover that failure.
- **IMU — rotation.** How fast the body is turning, over seconds, without
  needing the room to have any features in it. It measures no distance and no
  position, and it is never asked to.

**Consequences that are deliberate, not accidental:**

- **FEET is the one unit.** Every distance a person reads — instruction,
  dashboard tile, log column — is in feet. Millimetres survive only on the
  sensor wire and inside the geometry (body radius, surface fitting); the
  conversion happens once, at `walk_policy._mm_to_ft`. A device that mixes feet
  and metres invites a 2-metre gap being acted on as 2 feet.
- **The nearer of the two rangefinders wins.** Until 2026-10-30 they were
  averaged when within `WALK_GLASS_GAP_FT` of each other — but a radar reading
  1.5 ft past the laser (and up to a frame older) put a 3.5 ft wall at 4.25 ft,
  and "Blocked" came at ~3 ft instead of 4. A glass door reads ~1 ft on radar
  and ~30 ft on LiDAR, because the laser goes through it and ranges the room
  beyond; that disagreement (`WALK_GLASS_GAP_FT` or more) is what names the
  glass. When only one sensor measured anything, its reading is used.
  (`utils/walk_policy.fuse`)
- For the same reason, blocking is never decided on the two sensors
  **agreeing**: that would let a glass wall — which only the radar sees — pass
  as clear.
- **No return means "clear to my range", not unknown.** A silent sensor
  contributes its own reach — LiDAR **40 ft**, radar **13.1 ft** (9.8 ft until 2026-10-01) — and that
  figure is displayed like any other (both silent: their average). Treating it as "unknown →
  stop" would freeze the user in an empty corridor.
- **A dead sensor is NOT a silent one**, and the difference is safety-critical.
  An offline radar board also reports no distance; substituting 13.1 ft there
  turns a hardware fault into a claim of clear ground. So the max-range stand-in
  applies only to a sensor that is alive and saw nothing. When _neither_
  rangefinder is reporting ahead, the policy returns `unknown` — never "walk
  straight" off no data.
- **A substituted default can never trigger the glass rule.** The radar's reach
  is a quarter of the LiDAR's, so in any empty corridor the two "disagree" by 30
  ft; reading that as glass would flag every hallway. The nearer-wins override
  therefore requires the nearer figure to be a real measurement.
- **Segmentation does not steer.** Its walkable-surface cue never reached the
  voice, and on 2026-10-05 it was removed along with its gate; the
  segmentation model now only draws its debug view.
- **Stairs and people are never "walkable"**, even though the segmentation model
  labels them. A blind user stepping onto stairs expecting flat ground is
  precisely the failure this system exists to prevent, and being steered _at_ a
  person is the same shape of bug. (`walkable_guidance.HAZARD_NAMES`)
- **The walkable set is resolved by NAME, with no ID fallback.** Class numbering
  belongs to whichever dataset was exported last, and it has already changed
  under us: the 2026-08-06 model was `{0: Path, 1: Road, 2: Hallway}` and the
  2026-08-14 one is `{0: person, 1: path}`. A hardcoded `(0, 1, 2)` would now
  mark **person as walkable ground**. So an unresolvable model marks nothing
  walkable rather than guessing an ID.

### Nobody walks in a straight line

A sighted person corrects their heading continuously from vision without
noticing. A blind person cannot, so a two-degree veer that _feels_ like walking
straight is three feet of lateral error in five paces — which is how you walk
into the wall beside the corridor you thought you were walking down.

`walk_policy` cannot catch this. It asks "is something in front of me **now**?"
and stays quiet until an obstacle is inside `WALK_BLOCK_FT` (4 ft since 2026-10-01, about
1 s away), by which point the only instruction left is _Stop_ — alarming, and it
destroys the user's line of travel.

So there is a layer that works in **time** rather than distance
(`utils/drift_guidance.py`), and it warns 3–6 seconds out, where the fix is a
few degrees of steering:

1. **Collision course.** Project the user forward along how they are _actually_
   moving — speed and turn rate — and sweep their body as a disc along that
   path. The first thing it touches is what they will walk into, and when. This
   catches the oblique approach that a "what is straight ahead?" test cannot
   see: nothing is in front of you yet, and you will still meet that wall in
   four seconds.
2. **Open-space veer.** In a plaza there is nothing to converge on, so geometry
   says nothing and the only evidence is the heading quietly accumulating one
   way. This is what walks people in a slow circle.

Then it reports the **smallest** heading change that clears the path, because a
blind user handed a 40° instruction for a 5° problem has lost their line and
gained nothing. Zero is a valid answer: when the obstacle is only in the way
because of the user's own turn, the instruction is _"you are veering left —
straighten up"_, not a new heading to aim at.

**Walking speed comes from the LiDAR**, recovered by matching consecutive scans
(`utils/heading_tracker.py`). Two details are load-bearing:

- The obvious model — treat each return as a fixed point, so advancing `d`
  shortens its range by `d·cos(bearing)` — is **wrong for a wall**, and walls
  are most of what a corridor contains. Walking down a corridor does not change
  the range to the wall beside you at all. Measured: the point model reported 2
  mm of travel for a real 100 mm step. The fit is therefore **point-to-plane**,
  using the local surface orientation recovered from the range gradient.
  Measured after the fix: 1.19 m/s against a true 1.20, yaw rate 3.95°/s against
  a true 4.00.
- Fit _quality_ is not enough to know whether the speed means anything. Down a
  bare corridor the model fits beautifully with zero travel, because forward
  motion genuinely changes nothing about the scan — an excellent fit to an
  unknowable quantity. Only the estimate's own **standard error** exposes that,
  and it is what decides between reporting a measured pace and falling back to a
  nominal one. Without it a walking user reads as standing still, which silences
  the veer warning exactly where no wall geometry exists to catch them.

**Yaw comes from the gyro, kept honest by the LiDAR.** The two fail in opposite
places, which is the entire reason for carrying both:

- The scan match measures against the room, so it has no accumulating error —
  but it needs the room to have features. In an open plaza there is nothing
  inside its 8 m matching range and it goes silent, which is precisely where the
  open-space veer check is the only thing between the user and a slow curve into
  the road.
- The gyro measures the body directly and does not care what the room looks
  like — but a MEMS part's zero-rate offset walks with temperature, so an
  uncorrected integral slides away at degrees per minute.

So the gyro takes the yaw, and whenever the scan match is independently trusted,
the disagreement between the two is fed back as a slow correction to the gyro's
bias (`utils/gyro_reader.py`). Same complementary structure the attitude filter
uses to hold tilt steady, one level up. Measured on the bench: **0.06° of yaw
drift over 20 seconds**, against the 15° over 6 s that counts as a veer.

Simulated, in the plaza the gyro exists for
(`tools/drift_sim.py --open --veer 3`):

| | first warning | user was |
| --- | --- | --- |
| LiDAR alone | t = 12.3 s | 13.2 ft off course, and only because a wall finally came into range |
| with the gyro | **t = 6.9 s** | **3.6 ft off course**, warned by the veer check with nothing ahead to explain it |

**Three things the gyro is deliberately not allowed to do.**

- **It does not measure speed.** Double-integrating acceleration for a
  pedestrian drifts to metres within seconds. Pace stays the scan matcher's job
  and the nominal fallback is unchanged. The accelerometer is used only for
  whether the user is *stepping* — a yes/no the LiDAR cannot always give, since
  down a bare corridor forward motion is geometrically unobservable.
- **It cannot claim the user is still, only that they are moving.** The veto is
  one-way. A missed footfall silencing the veer warning for someone genuinely
  walking is the failure this layer exists to prevent; a spurious step costs a
  nuisance cue.
- **It may not steer on an unproven zero-rate offset.** A wearable is switched
  on and then walked off in, so the calibration that assumes stillness is the
  one most likely to be taken during motion — and a 2°/s offset is
  indistinguishable from the 2.5°/s veer being detected. Movement during
  calibration is visible in the *spread* of the samples, and a bias taken that
  way is marked untrusted: the gyro is kept out of the yaw path entirely, and
  the tracker behaves exactly as it did before one was fitted, until either the
  LiDAR vouches for a standstill or the two sensors have simply agreed for ten
  seconds running. The bias correction keeps running throughout, because that
  agreement is how the offset earns its way back in.

**It is not commissioned yet.** The board is not mounted, so which of its axes
points up when worn is unknown, and `GYRO_MOUNTING_CONFIRMED` is False: the
reader connects and reports health, tilt and gait on `/guidance/gyro`, and the
heading tracker ignores it completely and runs exactly as it did before one was
fitted. This is a switch rather than a note because guessing the mounting is
worse than waiting for it — a wrong axis or sign does not degrade the guidance,
it **inverts** it, and tells a user veering right to bear right. The
commissioning procedure is four steps in `config.py`, and the last one is
setting that flag.

**Verified working 2026-09-04**, against a standalone probe of the same part:
MPU-6500 on `/dev/spidev0.0`, 100 Hz, zero rejected samples and zero dropouts
across 350k+ samples, bias calibrated at rest (0.04 dps spread). Two things that
look like faults and are not:

- **`bias_dps` reads 0.0** while a direct probe measures a real per-axis bias
  (e.g. −3.65 dps on Z). Expected: `utils/mpu6500.py` returns *"gyro in deg/s
  (bias removed)"* and subtracts its own calibration on every read, so the
  reader's *additional* residual correction is legitimately ~0. The probe's own
  live output shows the same thing — rates near zero once calibrated.
- **`[GYRO] link down`** appearing at startup. On this device that has meant a
  second process already holding the SPI bus, not a wiring fault. See below.

**Still outstanding: the board is loose and off level** — tilt was observed
swinging 4.7°–15.3°, past the 12° `GYRO_TILT_WARN_DEG` threshold and worse than
the 9.1° recorded in `config.py`. Yaw barely cares (cos 15° ≈ 0.97), but
cross-axis leak does: sin 15° ≈ 0.26, so a quarter of every pitch and roll rate
lands in the yaw channel, and a walking torso pitches and rolls hard with every
step. Until it is mounted flat, tilt and cadence are not trustworthy — the step
count picks up desk knocks, not gait.

### One process at a time

Worth knowing before blaming a sensor. Every device here has exactly one owner,
and a second instance fails in a way that looks like six simultaneous hardware
faults:

```
ERROR: [Errno 98] error while attempting to bind on address ('0.0.0.0', 8000)
INFO Camera camera.cpp:1021 Pipeline handler in use by another process
[HailoRT] [error] HAILO_OUT_OF_PHYSICAL_DEVICES(74)
[WARN] Could not start motor: Attempting to use a port that is not open
[RADAR] no A121 boards answered
[GYRO] link down
```

That is **one** cause: something else holds port 8000, the camera, the NPU,
`/dev/ttyUSB0`, the six `/dev/ttyACM*` and `/dev/spidev0.0`. The `Errno 98` line
prints *after* the startup banner has already listed every URL, so it is easy to
scroll past and then distrust the hardware. Check before diagnosing:

```bash
ss -ltnp | grep :8000
fuser /dev/ttyUSB0 /dev/ttyACM* /dev/spidev0.0
```

A start that fails for no visible reason moments after a previous run is almost
always the previous run still shutting down.

**There is no simulated gyro.** The LiDAR and the camera have development
fallbacks, and only when their Python package is missing — radar and NPU never
simulate. They run only with `SHAPPE_SIMULATE_LIDAR=1` / `SHAPPE_SIMULATE_CAMERA=1`,
and their data is flagged `simulated`, which health, walk_policy, the hazard
list and the voice all treat as NO evidence (health says the LiDAR is down and
the camera simulated; the voice says "Sensors offline", never "Walk straight";
simulated camera labels name nothing). A fake *heading* is not — it would be consumed as a measurement of
which way a real person is walking. An absent or dropped gyro reports itself
absent and the tracker reverts to LiDAR-only. Degraded, never wrong.

Heading is **dead reckoning** either way: its error accumulates, so it means
something over seconds, never as an absolute bearing. The reference is zeroed
whenever the user is told to turn, so a deliberate turn is never reported back
to them as a veer.

**Restraint is a safety feature here.** A cue must hold for
`DRIFT_CONFIRM_SCANS` before it is issued, and is not repeated inside
`DRIFT_REPEAT_S` unless its urgency escalates. A guide that fires continuously
gets ignored or switched off, and one that is off warns nobody.

Every layer that can speak now owns a gate of that shape — drift
(`DRIFT_CONFIRM_SCANS` / `DRIFT_REPEAT_S`), and the surface guide before it was
removed — and a gated layer is authoritative
**both ways**: when it admits a cue, the cue goes out; when it holds one, no
other rung may announce it on that layer's behalf. Without the second half the
saving is illusory, because a cue that flickers off and back on inside its own
repeat window reads as a _changed_ instruction one level up and gets announced
anyway.

**One field means "say this".** `/guidance/walk` is polled several times a
second and always returns an instruction, because "Walk straight" is genuinely
the answer for minutes at a time. So `notify.speak` — and only that — means
announce, and `notify.cue_seq` advances only when it flips true, which makes
"say this once" unambiguous at any polling rate. Polling faster cannot produce
more announcements. An unchanged blocking sentence is re-announced after
`SIMPLE_REPEAT_S`, doubling each time up to `SIMPLE_REPEAT_MAX_S`, while a calmer
one is said once, because a missed _Bear left_ costs a few inches and a missed
_Stop_ costs the user their shins.

When the guide is quiet it says WHY, in words, in `notify.quiet_reason`
("already said — not repeated", "too soon — 1.2s until the voice is free",
"blocked — confirming what it is"). Silence that cannot be explained is indistinguishable from a crash,
and the only place this ever gets debugged is a footpath.

### Three resolutions, on purpose

The LiDAR's 360° is divided up three different ways, because one split cannot do
all three jobs:

| Split                                 | Used for                  | Why                                                                                                                                   |
| ------------------------------------- | ------------------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| **8 × 45° sectors**                   | Threat awareness          | Fine enough to say "person at 2 o'clock"                                                                                              |
| **4 cardinals**                       | Steering                  | "Bear 45° left" is not a usable instruction for a blind walker                                                                        |
| **10° cones** (+ body-width corridor) | "Is my path ahead clear?" | A 45° sector clips hallway **side walls** and reported them as an obstacle dead ahead — it said STOP down a completely clear corridor |

### Shape, not just distance — the BEV grid

All three splits above answer the same question at different widths: **how far
is the nearest thing this way.** None of them can answer **what shape is it**,
and some things worth knowing are only visible as a shape.

`utils/bev_grid.py` adds a fourth view: an ego-centred bird's-eye occupancy
grid, 16 m across at 50 mm per cell, accumulated over several scans and moved
under the user each frame by the heading tracker's ego-motion. Clusters in it
have an extent and an orientation, so they can be named.

| It can say                          | The per-bearing minimum cannot, because                                  |
| ----------------------------------- | ------------------------------------------------------------------------ |
| "a 3 m flat wall" vs "a 0.2 m pole" | A nearest range is one number; length and thickness are not in it        |
| "a 0.9 m doorway at 12 o'clock"     | A doorway reads as **far** — identical to open ground. What defines it is the two walls _beside_ it, and a minimum over a bearing keeps no record of those |
| "glass at 2.5 ft, radar only"       | The laser went **through** it and ranged the room beyond                 |
| "a 40° arc returned nothing"        | A silent bearing and a distant one are the same "far" to a minimum        |

**This is the 2D reduction of VoxelNet / PointPillars / VoTr voxelization, and
the reduction is forced.** Those architectures learn a feature per voxel from
the **height** distribution of the points inside it — PVTransformer's whole
contribution is that max-pooling that distribution loses too much, so replace
it with attention. The A2M12 sweeps one horizontal plane, so every voxel would
hold points at a single height and there would be nothing to pool or attend
over. What survives is the half that never needed height: rasterize, accumulate
with ego-motion, reason about shape. See the module docstring.

**Three cell states, and unknown is never folded into free.** Free space is
filled as one polygon through the scan's returns; a bearing that returned
_nothing_ pulls that polygon back to the ego, cutting an unfilled wedge rather
than painting clear ground out to max range. Matte black and specular glass
both return nothing, and certifying those as walkable is the one error a grid
could introduce that a per-bearing minimum never makes.

**The radar stamp is drawn after the free-space pass, and that order is a
safety property.** The laser has just marked every cell between here and the
room beyond a glass door as clear; stamping the radar echo afterwards puts the
door back. Only stamped where the radar is nearer by `WALK_GLASS_GAP_FT` — the
same margin `walk_policy.fuse` already calls glass — so ordinary agreement
between the two sensors does not double-draw a wall the laser already found.

**It is advisory and may not stop the user**, for a sharp reason. The grid integrates several scans
through an _estimated_ ego-motion, and a bad estimate smears it. A layer that
can be smeared must never hold the brake. Below `BEV_MIN_MOTION_QUALITY` it
resets to the current scan rather than warping by a motion it does not trust,
which degrades it to a single sweep — no worse than what the sector code
already works from.

**What reads it — two advisory paths, and nothing else.**

| Consumer | Does | Cannot |
| --- | --- | --- |
| `sensor_fusion.build_hazards` | Replaces "obstacle" with **wall** / **post** where the grid resolved that same return. Carried as a `shape` field even when the camera's label wins the name, so a post-shaped "chair" reports both | Create, remove, rank or escalate a hazard. Identical inputs with and without a shape produce identical urgencies |
| `spatial_scan` -> `simple_guidance.safer_side` | Breaks a tie between "Turn left" and "Turn right" toward the side with a doorway or open space | Pick a side the rangefinders give less than `WALK_MIN_TURN_FT` of room |

`body`-sized clusters are deliberately **not** promoted to a spoken word: the
grid can tell person-sized from wall-sized, but not a person from a wheelie
bin, and "body" would claim it could. A blob that really is a person gets named
by the camera, which can see that.

Measured on-device: **1.2 ms** per scan to fold in, **1.2–4.4 ms** to extract
(rate-capped to `BEV_EXTRACT_HZ` = 5, on the same analysis thread as the sector
stats the STOP decision reads).

**Checking it works.**

```bash
./venv/bin/python -m tools.bev_check              # diagnose the running server
./venv/bin/python -m tools.bev_check --watch      # keep it on screen while you move
./venv/bin/python -m tools.bev_check --checklist  # the physical tests to run
```

`/lidar/bev` answers "what does the grid see", which is the easy half. The hard
half is whether what it sees is trustworthy, and that is not one field — the
grid returns perfectly good-looking clusters while its registration silently
resets every frame, or while the radar is offline and glass is therefore
undetectable. `bev_check` gives each of those its own line with what to do
about it, and exits non-zero on a real failure so it is usable from a script.

The physical tests are the ones that matter, and `--checklist` prints them:
face a wall, stand a bottle on the floor, stand in a doorway, **stand in front
of a glass door**, hang black cloth, and turn 90° on the spot watching
`/lidar/bev/image` for a smear. The last line of that checklist is the one to
take seriously — while any of it is in view, `path.action` must never become a
stop because of something only the grid can see.

### Things that move — LiDAR movers (added 2026-10-07, silent for now)

The camera sees one direction, and on the rig two of the four cameras are dark.
The LiDAR sees all the way round, so `utils/lidar_movers.py` follows what moves
in it: each scan is cut into blobs (a person, a bike, the front of a car — up to
2 m wide; anything wider is a wall), each blob is followed from scan to scan,
and its speed is fitted over ~1.2 s.

**"It moves" vs "I move".** Walking makes everything look as if it moves. Every
*still* thing moves the same way relative to the user, so the user's own walk and
turn are fitted from the still things around them (`ego` on `/lidar/movers`),
and a mover is whatever does not fit. Where there are too few still things (a
bare corridor, an open space), any walk up to `LIDAR_MOVERS_WALK_MAX_MPS` is
allowed for: sideways motion and approaching from behind are still caught, but
something coming head-on at a walking pace is not. Walking past something near
shows a different face of it each scan, so the threshold grows with its size
over its range.

**What it would say** — one approach at a time, the soonest, and only if it
will pass within `LIDAR_MOVERS_MISS_M` (1 m; vehicles 2 m) inside
`LIDAR_MOVERS_HORIZON_S` (5 s):

| It is | Sentence |
| --- | --- |
| crossing in front, or coming from a side | `Person approaching from left, stop` (and the mirror) — let it pass |
| ahead, heading at the user | `Person coming toward you, move left` — away from the side it passes on, where the laser shows 2 ft of room; else `…, stop` |
| behind | `Person approaching from behind, move right` — else just `Person approaching from behind` |

"Person" or "Vehicle" is a guess from size and speed (wider than 0.9 m or faster
than 3 m/s is a vehicle); in front, the camera's name for a box at that bearing
wins. The camera's own `Vehicle approaching` outranks all of these.

**Silent until a test walk says otherwise.** `LIDAR_MOVERS_SPEAK = False`: the
dashboard shows each mover on the laser view and its "would say" sentence in a
chip, the speech log records `would say: …` lines (source `lidar-movers`, not
counted as speech), and the terminal prints them. Setting it True puts the same
sentences through the one voice and its gate (level 2: said as soon as the
sentence playing ends).

**The radar fills the LiDAR's gaps** (`utils/radar_movers.py`). Each board's
echoes are followed in distance, and one closing in by itself is a mover — but
only where the LiDAR has no blob (a thing up to 2 m wide) at that distance inside
that board's beam: its blind back-left arc, or something above or below the
laser's plane (a child, a dog, a low trolley). Where the LiDAR sees it, the
LiDAR's warning is used and the radar stays silent: one warning per thing. The
radar cannot tell what it is, so it says `Something approaching from behind,
move right` (and the same shapes as above). A board ranges the nearest point of
a wall in its beam, and that point slides along the wall, so while the user walks
an echo must close faster than the walk itself — on any board; and while the user
turns faster than `RADAR_MOVERS_MAX_TURN_DPS` (which sweeps the beams across walls),
radar warnings pause. In practice: standing still, someone walking or jogging up
is caught 1–3 s ahead; walking, only something much faster than the user, inside
the radar's 4 m reach — a person catching up at a walking pace is not.

**Limits.** 2D: one plane at the LiDAR's height. Reach is `LIDAR_MOVERS_MAX_RANGE_MM`
(8 m), so a car at speed is warned only ~1 s ahead — the camera covers vehicles in
front. A car crossing broadside shows a side longer than 2 m and is not followed.
Where the LiDAR returns nothing (on the rig, ~170°–290°, back-left) only the radar's 4 m reach is left.
A crowd moving together can pass for the user's own motion. Checked only in
simulation so far — `./venv/bin/python tools/movers_sim.py` (rooms, a corridor,
clutter with lost beams, a car, a bike, the blind arc, a glass pane, a turn, and
radar-only echoes) — not yet on a real walk.

---

## Architecture

```
main.py                 FastAPI app + lifespan (starts camera, LiDAR, radar, gyro, voice)
config.py               Every threshold, in one place — the single source of truth

utils/
  simple_guidance.py        THE ONE SPEAKER: the closed vocabulary, warn-fast /
                            clear-slowly smoothing, and the gate that decides what is said
  pi_voice.py               The 10 Hz guidance loop and espeak-ng, so the device talks
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

routers/                API layer (see below)
templates/dashboard.html  The dashboard at / — built for field testing
templates/index.html    The previous dashboard, still served at /classic
deploy/                 systemd user service + installer (see "Running it")
camera_web.py           Standalone live view of the four cameras (run with the app stopped)
tools/walk_logger.py    Logs guidance + FPS + CPU temp to CSV during a walking test
tools/drift_sim.py      Walks a virtual user down a synthetic corridor to tune the
                        drift thresholds without leaving the desk
tools/bev_check.py      Is the BEV grid actually working? Diagnoses each way it can
                        be quietly useless, and --checklist prints the physical tests
tools/movers_sim.py     People, bikes and cars moving round the user, and still rooms,
                        through the real mover tracker: caught, from the right side,
                        and nothing still ever called moving?
tools/glass_sim.py      Glass doors, panes beside walls, corridors and posts, through
                        the real policy: is the pane named, and only the pane?
tools/anchor_diversity.py  Can the scan band anchor camera depth? The go/no-go
                        measurement for the Spatial Plan's E2 — needs no depth model
model/                  yolo26m.hef (detection), best.hef (segmentation), metadata.yaml
```

### Threading

Four independent loops, so a slow one cannot stall the others:

1. **Capture** — pulls raw mosaic frames from Picamera2 (~30 FPS).
2. **Detection** — the FRONT camera on every new picture, published at once; a
   side camera at most every `DETECTION_SIDE_MIN_INTERVAL_S` (0.4 s, the one
   longest undetected first, preferably while the board shows a side), so each
   side is re-detected within ~1.3 s. Guidance only reads the front: a side's
   ~40 ms ahead of it cost the front pictures (2026-10-30; it was one side every
   pass). Letterboxed straight from the camera's array with OpenCV, not via PIL.
3. **Segmentation** — its own thread, and only while its front view is open
   (`SEG_ALWAYS`): its cue no longer reaches the voice, and the NPU time it
   took came out of detection.
4. **LiDAR** — its own thread, broadcasting scans over WebSocket.

Detection and segmentation share one NPU via HailoRT's scheduler, so
`SEG_TARGET_FPS` acts as a **cap** that stops segmentation from starving the
detector.

### Measured on-device (2026-09-25)

|                        | Before (all 4 cameras + seg) | Now                                   |
| ---------------------- | ---------------------------- | ------------------------------------- |
| Camera capture         | 30 FPS                       | 30 FPS                                |
| Front camera detection | 3.6 /s, up to ~0.5 s old     | 11 /s, 106 ms old (median)            |
| Each other camera      | 3.6 /s                       | 3.7 /s                                |
| Detection pass         | ~190 ms (4 quadrants)        | ~90 ms (2 quadrants, 79 ms of it NPU) |
| Letterbox per quadrant | 16 ms (PIL)                  | 4 ms (OpenCV)                         |
| Segmentation           | 10 Hz, always                | only while its view is open           |
| LiDAR                  | ~23 scans/s                  | unchanged                             |
| Guidance decision      | ~6 ms per poll               | unchanged                             |

---

## Running it

```bash
# One-time setup. --system-site-packages is REQUIRED: hailo_platform and
# picamera2 are apt-installed and are not on PyPI.
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -r requirements.txt

./start_lidar.sh          # http://<pi-ip>:8000
```

The app still STARTS on a machine with no LiDAR, camera or NPU: a missing
device publishes nothing and health reports it down. Only with
`SHAPPE_SIMULATE_CAMERA=1` (without `picamera2`) and `SHAPPE_SIMULATE_LIDAR=1`
(without `rplidar`) is synthetic data produced, and it is never evidence of
anything.

### On the device: a service that starts at boot

A wearable cannot depend on someone typing `./start_lidar.sh`, and a blind user
cannot see that it has stopped. `deploy/install-service.sh` installs it as a
systemd **user** service (user, so the voice reaches PipeWire and the earpiece):

```bash
./deploy/install-service.sh                 # install + enable; prints the admin token
systemctl --user start shappe-guide         # start now (stop any ./start_lidar.sh first)
journalctl --user-unit=shappe-guide -f      # its log
./deploy/install-service.sh --remove        # undo
```

- **Starts at boot** with nobody logged in (the installer enables lingering).
- **Restarts** on a crash, and on a **hang**: the watchdog is pinged only while
  the guidance loop and the web server's event loop are both alive
  (`utils/service.py`). A missing sensor does not restart it; the app already
  recovers from that and says "Sensors offline".
- **Never wedges on the way out.** A shutdown that has not finished in
  `SHUTDOWN_TIMEOUT_S` ends with `os._exit`, so a stuck camera cannot keep
  holding port 8000 and every serial port for the next start.

`GET /health` says whether the device can guide right now: `ok` / `degraded`
(guiding on the LiDAR, something else down) with HTTP 200, or `down` (no
LiDAR, or the guidance loop is not guiding) with HTTP 503.

**Who may change the device.** `/radar/calibrate`, `/radar/rediscover` and
`/guidance/gait/stride` change how it steers, so they need `SHAPPE_ADMIN_TOKEN`
in an `X-Admin-Token` header (the installer and `start_lidar.sh` write one to
`~/.config/shappe-guide.env`; the test pages ask for it once per browser). With
NO token configured (uvicorn or `python main.py` by hand) they are refused with
503 — they used to be left open. Any state-changing request or WebSocket
handshake whose `Origin` is another site is refused (403 / close 1008), whatever
the token; and one that carries an Origin must reach the device by an IP
address, `localhost`, its hostname or `hostname.local` — or a name listed in
`SHAPPE_ALLOWED_HOSTS` — so a DNS-rebinding page cannot pass as same-site.
Every response forbids framing, and cross-origin reads are refused unless
listed in `SHAPPE_CORS_ORIGINS`.

Hardware check: the LiDAR (by its `/dev/serial/by-id/` name, falling back to
`/dev/ttyUSB0` — `config.LIDAR_PORT_CANDIDATES`), `/dev/hailo0` (NPU), and the A121
boards on the USB hub. To find which radar board is physically where:

```bash
./venv/bin/python -m utils.radar_utils --identify
```

### Six radar boards, four guidance directions

The array is **six** A121 boards every 60°, offset half a step so they
**straddle** the cardinal axes rather than sitting on them — bearings
30/90/150/210/270/330. Everything downstream still speaks in four directions,
and the six reduce to them two-and-one:

**Bearings were reassigned on 2026-09-04** — all six serials moved. The first
table was converted from the rig's `positions.json`, and the mount it described
was not the mount on the device; the current table in `config.RADAR_BEARINGS` is
the operator's own statement of which board covers which sector, which outranks
any file. The USB cabling did not change, so the boards were remounted rather
than rewired. The **grouping structure below is unchanged** by that: it depends
on the mount, not on which serial sits where.

Boards are known by the **sector** they cover (`0–60`, `60–120`, …), which is how
they are labelled on the device and in the operator's notes. A bearing is the
same fact in the form the geometry needs, `bearing = sector_start − 30`. That
conversion lives in one place, `config.radar_sector_label()`, and `/radar/map`
publishes `sector_label` per slot so no display re-derives it — a display that
did its own arithmetic is how a silent 30° error gets in.

| direction | fed by | boards | arc covered | centre |
|---|---|---|---|---|
| **front** | the two boards straddling ahead | `deg330`, `deg030` | 300°–60° | **0°** |
| **right** | the one board on the axis | `deg090` | 60°–120° | **90°** |
| **back** | the two boards straddling behind | `deg150`, `deg210` | 120°–240° | **180°** |
| **left** | the one board on the axis | `deg270` | 240°–300° | **270°** |

Every group is centred *exactly* on the LiDAR sector of the same name, which is
what makes the reduction safe: `sensor_fusion` injects a radar cardinal into the
LiDAR sector it genuinely overlaps, so a hazard is never announced in a
direction the board was not looking.

The grouping is **derived, not written down** — `cardinal_sources()` admits a
board when the angle to a cardinal's centre is under `RADAR_BEAM_H_DEG/2 + 22.5`
(= 55°). At 60° spacing that takes in a neighbour 30° away and rejects one 60°
away, which produces the table above on its own. Re-measure a bearing at
`/radar/test` and the mapping re-derives itself with no restart.

#### Converting from the standalone rig

`/home/shappe/radar` records a board by the **start** of the 60° sector it
covers, and its `layout.py` sets `SECTOR_ZERO_OFFSET_DEG = -60` — so a board
labelled `0–60` there does not point at 0°, it points at the middle of its
sector, which the offset puts at `start − 30`. The one conversion between the
projects is therefore

```
bearing = (rig_sector_start - 30) mod 360
```

Skipping that offset is a silent **30° error** in the direction a glass door is
announced, so it is stated once in `config.RADAR_BEARINGS` and nowhere else.

#### Coverage overlaps — but head-on flat glass is still unproven

The boards are mounted **rotated 90°**, so the A121's wide axis is the horizontal
one: **65° across**, 53° tall. At 60° spacing the six beams therefore **overlap by
5°** — there is no blind slit anywhere, and every bearing including dead ahead
sits inside at least one board's beam.

That closes the geometric question. It does not close the one that matters,
because that one was never about coverage. At 60 GHz a smooth pane is
**specular**, not diffuse: a monostatic board gets a strong return only near
*normal* incidence. A pane facing the user square-on has its normal at 0°, and
the two boards that cover 0° sit at 30° and 330° — so they illuminate it at 30°
incidence and its specular lobe leaves at 30° the other way, missing both
receivers. **Being inside a beam is not the same as getting an echo back.**

No rotation or regrouping of six boards at 60° spacing fixes that: with the
boards where they physically are, nothing sits on boresight. So how weak it is
remains a **measurement**. **FT-1.5b** (from the old `FIELD_TESTS.md`, since deleted) covers
it; until that is run, do not treat forward glass coverage as established.

### Radar hardware test — `/radar/test`

`/radar/view` asks *what is around the user, and can we steer on it?*
`/radar/test` asks the hardware question instead: **is every board present,
streaming, and keeping up?** One card per board, with the numbers that only
matter when something is wrong:

| Field | Why it is there |
|---|---|
| **measured Hz** | The rate actually achieved, not `RADAR_UPDATE_RATE` as requested. The real test of the USB link. |
| **lag** | How far behind real time the stream has fallen. Should sit at 0.000 s. |
| **reconnects** | Climbing here means an unstable cable or hub socket. A board that reconnects constantly looks perfectly healthy in any single reading — this counter is the only thing that shows it. |
| **strength / temp** | Signal quality, and a board running much hotter than its neighbours. |
| **other peaks** | Returns behind the nearest one. Glass in front of a wall shows as two. |
| **state** | `starting` / `calibrating` / `streaming` / `reconnecting` / `refused`. "Offline" and "not finished calibrating" look identical in a reading, and only one is worth chasing a cable for. |

It reads the **running app's own state**, which is the point. This started life
as a separate app on port 8010, and a serial port has exactly one owner — so
testing the boards meant stopping the guide, and whichever app started second
saw six dead radars while the hardware was fine. The boards can now be tested
while the guide is live, which is the only time the answer is worth much.

Wave a hand ~1.5 ft in front of one sensor: its distance drops and its bar
jumps. That is how a physical board is mapped to a serial — and the same page's
CALIBRATE panel is where that measurement is recorded, writing the one bearing
table the guidance layer actually steers on (see `RADAR_BEARINGS` in
`config.py` for why a bearing is measured and never inferred).

A board reports in one of three conditions, because they need different repairs:

- **discovered and streaming** — working.
- **discovered but refused** — the USB link is fine; the board has no measured
  bearing, so it is refused rather than pointed in a guessed direction.
  Calibrate it; do not touch the cable.
- **never answered** — a cable, a hub socket, or a hung module.

The header's last pill is the one that matters: **blind cardinals**. Six
streaming boards with no bearings leave every direction without radar, and that
reads as a perfectly healthy array everywhere else on the page.

### Models

**The models ARE in git today** (49 MB). The plan was to keep them out — they
are compiled binaries git cannot diff or compress, so each retrain adds another
permanent copy to history, and the segmentation model was swapped twice in one
day during development — but no ignore rule was ever added. Moving them to Git
LFS, or out of the repo (`git rm --cached model/*.hef`), is still open.

What the app needs in `model/`:

| File                  | What it is                                           |
| --------------------- | ---------------------------------------------------- |
| `model/yolo26m.hef`   | Object detection (COCO-80)                           |
| `model/best.hef`      | Walkable-surface segmentation                        |
| `model/metadata.yaml` | Segmentation class names — **must match** `best.hef` |

A retrained model is rebuilt from its `.pt` with the Hailo conversion pipeline
(the `conversion.md` this used to link is not in the repo). `metadata.yaml`
must travel with `best.hef`, since the segmentation code reads the walkable class names
out of it at load — swap a model without swapping this and classes get
mislabelled.

---

## Field testing

`tools/walk_logger.py` records a walking test. Start the app, then:

```bash
./venv/bin/python tools/walk_logger.py     # Ctrl+C to stop and print a summary
```

It polls localhost twice a second and writes `logs/walk_<time>.csv` with the
guidance action/cue, the walk policy's own decision (blocked, chosen turn,
**glass ahead**, and its reason string), per-direction clearance in metres,
per-direction radar distance plus how many boards were online, the capture /
detection / segmentation FPS, all 8 LiDAR sectors, and CPU temp + throttle flag.

Press ENTER to drop a timestamped marker — type a note first (`glass door here`)
and it is recorded on the next row. That is how you line the CSV up against what
actually happened on the walk.

On exit it prints FPS min/avg, how often the path was blocked, how often glass
was flagged, whether any radar board dropped out, and a tally of each guidance
action. `logs/` is gitignored.

---


## Dashboard

`http://<pi-ip>:8000` is **the field-test dashboard** (`templates/dashboard.html`,
2026-09-24): what the device is saying and why, four verdict buttons that stamp
✓ / ✗ / ⚠ / ✎ into the speech log (keys 1-4), the laser view drawn with the
same corridor and rings the guide decides on, the front camera with the band
where objects get named, the speech log (kept in memory; written to
`logs/speech_*.log` only with `SPEECH_LOG_TO_FILE = True` in `config.py`), and
per-board sensor health. It is
also the voice when no earpiece is connected: one aria-live region for the
phone's screen reader, silent while the Pi speaks. The Pi's own voice
(`utils/pi_voice.py`, espeak-ng) is the production delivery path.

**The rest of this section describes the previous dashboard**, still served at
`/classic` (`templates/index.html`).

It ships with a **sunlight theme, on by default**, because field testing happens
outdoors where a dark UI is unreadable: the screen emits light, sun adds a fixed
veil of reflected light on top, and a dark background has too little luminance
to rise above it. White ground with black text keeps the highest contrast still
achievable outside. Toggle in the header, or force it with `?theme=night` /
`?theme=sun`. The night theme is better indoors and at dusk.

**The instruction banner across the top is the whole point of the page.** It
carries the one merged answer at 40-44px, colour-coded by urgency, with the
reason underneath, the forward clearance and time-to-contact beside it, and
whether the guide is speaking or silent — and if silent, why. Everything below
it is diagnostics.

It exists because the instruction used to be 19px of text inside the left-hand
card, below the traffic panel and the sector grid, surrounded by detail lines of
the same weight: "what should I do right now" had to be hunted for on a screen
showing forty other numbers. Three things came out of putting it at the top,
where being wrong is visible:

- **Urgency is taken from the policy, not the wording.** "Path blocked. Turn
  right." contains no stop and no caution keyword, so the string-matching ladder
  scored it CLEAR — survivable as a 3px border, and a 44px green banner over an
  obstacle two feet ahead. It now reads `walk.blocked`, and the card border
  below uses the same ladder so the two cannot disagree.
- **The crossing panel no longer restates steering.** It fell back to
  `urgent_message`, the legacy instruction computed off the 45° sectors, so the
  panel said "Turn RIGHT - 39.4ft clear" while the banner said "right has the
  most room at 4.0 ft" off the narrow cones. Two numbers for one decision, on
  screen together, is what a single merged instruction exists to prevent. The
  panel owns crossings; the banner owns steering. Away from an intersection it
  now says so plainly instead of showing an amber WAIT over an empty pavement.
- **Screen-reader output is gated on `cue_seq`, not on the poll.** A live region
  on the banner would read the whole thing two or three times a second, since it
  is rewritten on every poll. One hidden live region updates once per admitted
  cue instead — the same "announce this, once" contract the audio layer will
  honour.

**This Pi has no emoji font** (`fc-match emoji` resolves to DejaVu Sans; zero
emoji-capable fonts are installed), so every astral-plane pictograph rendered as
a hollow box in the browser on the device — including 🛑, which was the glyph for
a STOP. Each one used on the page was test-rendered; the failures were replaced
with BMP symbols that do render here (⛔ ⚠ ⏳ ● ○ ☀ ☾ and the arrows), and the
decorative ones in card titles were dropped rather than shown as boxes. Install
`fonts-noto-color-emoji` if you would rather have the pictographs back.

### Radar Scope (added 2026-09-04)

Under the LiDAR map in the centre column: the six-board array drawn as it
physically is. One wedge per board at its real **65°**, so neighbours overlap by
5° and the ring closes — 6 × 65° less 6 × 5° is exactly 360°, and each seam
carries a brighter sliver labelled `5°` marking the ground two boards see.

Read it as: the **dot** is how far (that distance is measured); the **arc**
through it is where along the beam the echo might be, because the A121 resolves
no angle inside its beam. Wedge captions are the **sector ranges the boards are
labelled by on the device** — `0–60° · front`, `60–120° · front`, and so on —
with the serial beneath, not the internal bearings. Dot colour is the real
guidance band from `/radar/map` (`danger` / `warning`), so a red dot means what
the device acts on. The red inner band is the near zone no board can see into.

Three things it is careful about, each learned by getting it wrong first:

- **Overlaps are unioned, not stacked.** Six translucent wedges laid two-deep at
  every seam rendered as pale radial spokes with darker ground between them,
  reading as if the spokes were the beams and the gaps were blind. Exactly
  backwards. One path with `fill-rule: nonzero` fills the union once.
- **One divider per seam, not two.** Stroking each wedge's closed outline drew a
  radial line down both its edges, so the 5° overlap between them looked like a
  spurious seventh sector.
- **A sliver only appears where both its boards are live.** An overlap needs two
  working boards; claiming one over a dead board would overstate coverage.

Everything is derived from `/radar/map`'s `geometry` — beam width, spacing, ring
phase, blind-zone radius — so the drawing follows the mount rather than a shape
hardcoded in the page. If the beams ever stopped overlapping, the seams would
draw as plain dividers and the caption would say there is a genuine blind slit.

### Motion (added 2026-09-04)

Walking speed, cadence, sideways slip and turn rate, from `/guidance/heading`.
All of it existed already; nothing on the page read that endpoint.

**Speed is LiDAR scan-match odometry, not integrated acceleration.** That is not
a shortcut — `heading_tracker`'s own docstring rules the accelerometer out for
pace, because double-integrating a torso-worn accel drifts to metres within
seconds. **Cadence** does come from the accelerometer, which is genuinely good
at counting footfalls and does not drift.

In FEET, like every other figure a person reads here, so "4.4 ft/s with 8 ft
ahead" is two seconds of arithmetic the user can actually do.

**No figure appears unless it was measured.** Four states, never collapsed:

| condition | shows |
|---|---|
| LiDAR simulated | `— LiDAR simulated`; slip and turn rate withheld |
| pace not resolvable | `not measurable` |
| measured | `4.4 ±0.3 ft/s`, with its standard error |
| confidently still | `stopped` |

This matters more than it sounds. `speed_mps` falls back to
`HEADING_NOMINAL_SPEED_MPS` so the drift layer has something for its
time-to-contact arithmetic — it was never a reading. The panel first showed that
fallback as `3.9 ft/s*`, marked with an asterisk and footnoted, next to a user
standing still. An asterisk does not fix that: the eye takes the number, not the
footnote. Worse, slip and turn rate came from scan-matching **simulated** sweeps
against each other, which does not produce zero — it produces plausible-looking
drift, and read `-6°/s` on a stationary device. `lidar_utils` already flags
`simulated` on the wire precisely so the dashboard can say so; the panel now
honours it and the explanatory note quotes no figure either.

It also surfaces two cross-checks, using the accelerometer to check the number
rather than to produce it:

- **Odometry says moving, accelerometer sees no footfalls** — being carried, or a
  scan-match artifact off something else that moved. Worth showing rather than
  averaging away.
- **Gyro board off level** — cadence and tilt are unreliable until it is mounted
  flat, and the panel says so instead of quietly reporting them.

The layout stacks to one column under 760px and drops the scan-rate pill under
540px, because field testing is done one-handed on a phone and the theme toggle
has to stay reachable. The scope is capped at 440px and centred rather than
filling the column — square at full width it stood 660px tall against the 280px
LiDAR chart above it. Note that a `viewBox` scales text with the drawing, so the
label sizes are set for that capped width; changing the cap needs them revisited.

---

## API

**Guidance** — the endpoints that matter

| Endpoint                | Returns                                                                                                                                                                                                                                 |
| ----------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `GET /guidance/walk`    | **The one merged instruction.** Obstacle avoidance + surface steering + crossing, in safety-first priority order. `notify.speak` / `notify.cue_seq` is the _only_ field that means "announce this"; everything else is state to display. It is the 10 Hz guidance loop's newest result (`loop.age_s`): a request never runs the pipeline, so no page decides or commits what is said. Older than `GUIDANCE_RESULT_STALE_S`, it reports rule `unavailable` and why |
| `GET /guidance/policy`  | The straight-walk decision on its own, in feet — the guidance loop's last one (`loop.age_s`), never re-run by the request                                                                                                                |
| `GET /guidance/gyro`    | IMU health, yaw bias and whether it is trusted, tilt, gait. `/guidance/heading`'s `yaw_source` says which sensor is steering; this says why |
| `GET /guidance/drift`   | Collision course + veer: what you will walk into, when, and the smallest correction                                                                                                                                                     |
| `GET /guidance/heading` | Raw LiDAR odometry — turn rate, speed, accumulated heading                                                                                                                                                                              |
| `GET /guidance/fused`   | Ranked hazard list (clock position + distance)                                                                                                                                                                                          |
| `GET /guidance/scene`   | On-demand "what's around me?" full readout                                                                                                                                                                                              |
| `GET /guidance/status`  | Sensor health, in plain English                                                                                                                                                                                                         |
| `WS  /guidance/ws`      | Push stream of fused guidance (~5 Hz)                                                                                                                                                                                                   |
| `GET /guidance/log`     | Every line the device has SAID this run, timestamped. Written at the commit point, so it is the conversation the user had — a poller would see a different one |
| `GET /guidance/log/text`| The same as plain text, for `curl` from a phone mid-walk |

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
| `GET /camera/stream/{sector}` | MJPEG per sector (`front`/`right`/`left`/`back`)              |
| `GET /camera/seg/{sector}`    | Colorized segmentation debug view                             |
| `GET /camera/stats`           | capture / detection / segmentation FPS, per-sector detections — each with `distance_ft`: of the things the laser ranges in the box's bearings, the one that fits the box's size (`distance_source` `lidar`), else the box-size estimate (`camera`; `camera_cut` = at most) |
| `GET /camera/distances`       | Why each detection's distance is what it is: bearings searched, every laser range there, the size estimate (a wide person box = sitting), the choice, and the lens angle it implies (`implied_hfov_deg`) |
| `GET /environment/status`     | Fog / rain / low light / visibility                           |

**Navigation & crossing**

| Endpoint                 | Returns                                        |
| ------------------------ | ---------------------------------------------- |
| `GET /navigation/decide` | Obstacle-avoidance decision from current state |
| `GET /navigation/walls`  | Wall detection from the current scan           |
| `GET /crossing/status`   | Traffic-light state machine + crossing safety  |
| `GET /crossing/safety`   | Short "is it safe to cross?" answer            |

`GET /health`, `GET /api/status` for liveness. Full schema at `/docs`.


---

## Configuration

Everything lives in `config.py`, grouped by subsystem. The values worth knowing:

| Setting                             | Meaning                                                                                              |
| ----------------------------------- | ---------------------------------------------------------------------------------------------------- |
| `GUIDANCE_DANGER_MM` = 800          | Imminent — stop                                                                                      |
| `GUIDANCE_WARNING_MM` = 1500        | Close — caution                                                                                      |
| `WALK_BLOCK_FT` = 4.0               | Front nearer than this is blocked (6.5 until 2026-10-01)                                             |
| `WALK_MIN_TURN_FT` = 2.0            | Room a turn target must have, else stop                                                              |
| `WALK_GLASS_GAP_FT` = 2.0           | Sensors disagreeing by this ⇒ glass (the nearer wins either way, since 2026-10-30)                   |
| `WALK_FUSION_GLASS_OVERRIDE` = True | False drops the "sensors disagree" explanation; the nearer still wins (it averaged until 2026-10-30) |
| `WALK_TURN_MARGIN_FT` = 1.5         | A turn must beat the forward clearance by this, else keep going                                      |
| `LIDAR_NO_RETURN_FT` = 40           | Silent LiDAR counts as this much clear                                                               |
| `RADAR_NO_RETURN_FT` = 13.1         | Silent radar counts as this much clear (= `RADAR_END_MM`, 4 m since 2026-10-01; was 9.8)                                                               |
| `DRIFT_ALERT_TTC_S` = 5.0           | Seconds-to-contact that starts a drift nudge                                                         |
| `DRIFT_URGENT_TTC_S` = 2.5          | Seconds-to-contact that makes it firm                                                                |
| `DRIFT_REPEAT_S` = 3.0              | Same drift cue not repeated inside this window                                                       |
| `DRIFT_ANALYZE_HZ` = 8.0            | Rate cap — it shares the LiDAR thread with the stop decision                                         |
| `SEG_TARGET_FPS` = 10.0             | Segmentation rate cap on the shared NPU                                                              |
| `LIDAR_STALE_S` = 1.0               | Same rule for a scan read outside the thread that produced it                                        |
| `LIDAR_MIRROR` = False              | **Verified — do not flip.** True sends "turn RIGHT" to the left                                      |

Guidance thresholds are imported by _both_ the navigation and fusion layers, so
the two cannot drift apart.

---

## Design notes: bugs found and fixed

Kept because each one explains why the code is the way it is.

1. **A bare corridor reported a walking user as standing still.** The one that
   mattered. `_summarise` judged the forward estimate by fit quality and standard
   error, and **both are scaled by the residual** — so a corridor, where walking
   along a wall genuinely changes nothing about the scan, fitted almost perfectly
   and produced an SE around 0.03 m/s, well inside the 0.35 limit. The estimate
   was called precisely known when it was not observable at all, and a user
   walking 1.2 m/s down a hallway read as `moving=False` — silencing the surface
   guide (removed since), in exactly the place where there is no
   wall geometry ahead to catch them instead.

   Fixed by adding a **third, geometric hurdle** that no residual can talk up:
   `HEADING_MIN_FORWARD_SENS`, the RMS sensitivity of a range reading to one
   millimetre of forward travel, read straight off the design matrix. Bare
   corridors score 0.02–0.12, a corridor with a visible back wall 0.29–0.44, an
   ordinary room 0.67–0.79; the threshold sits at 0.20. Below it the pace falls
   back to nominal **and the user is assumed to be walking** — never reported as
   standing still, because nothing was measured either way. Reported as
   `forward_sensitivity` / `speed_observable` on `/guidance/heading`, so a quiet
   guide can be explained. The gradient the fit itself uses is untouched; the
   sensitivity figure uses its own smoothed copy, because raw range noise
   manufactures gradients and an unsmoothed 1.5 m corridor scored *above* a
   genuinely observable one.

2. **`reset_all_states()` left live state behind.** `walking_guidance_state` and
   `drift_state` survived it, and the surviving cue kept its old `last_update`,
   so it read as **fresh** rather than stale — a guidance poll straight after a
   reset could be steered by a cue describing a surface from before it. The root
   cause was duplication: every store was written out twice, once at import and
   once in the reset, and the copies had drifted (the reset rebuilt `lidar_state`
   from a schema predating the steering cones, so `cones` vanished, while
   `navigation_state` gained keys in the reset copy only). Each store now has one
   factory used by both. The reset also took a single blanket `state_lock` while
   readers use `lidar_lock` / `radar_lock` / `path_guidance_lock`; it now resets
   each store under the lock its readers actually hold.

3. **A fusion override was explained as glass in both directions.** Found by
   running the app. The nearer-wins rule fires whenever the
   two rangefinders disagree, but the reason string always told the glass story,
   which only applies when the **radar** is the nearer one. On-device the laser
   read 1.4 ft against the radar's 3.5 ft and the reason still said "the laser
   looks through glass", while `glass_ahead` — which tests the direction properly
   — was correctly False. A LiDAR-nearer override now reads "the radar missed
   what the laser ranged; too low or too thin for its beam". A reason string that
   contradicts the flag beside it is worse than no reason at all.

## Walking the drift layer without leaving the desk

```bash
./venv/bin/python tools/drift_sim.py            # 3 deg/s veer: warns 4.3 s out
./venv/bin/python tools/drift_sim.py --veer 0   # control: no VEER cue
./venv/bin/python tools/drift_sim.py --accuracy # odometry error vs known motion
```

The control run is silent about veering, which is the point of it — but it is
not silent. It still says "Wall straight ahead" three times, because a 20 s walk
at 1.25 m/s reaches the end of the simulated corridor and that is a real
collision course, correctly reported by the other drift layer. Read the control
run for the absence of a *veer* cue, not for the absence of output.

---

## Known limits

- **The camera board's 4-camera merge can scramble the picture** (seen 2026-10-08:
  every camera clean on its own, the 2x2 picture corrupted on and off). So by default
  the app CYCLES the cameras (`CAMERA_CYCLE`): the board shows one camera at a time
  and is switched between frames; a frame is the new camera's when its sensor
  timestamp is after the switch. The front gets 2 frames to each other camera's 1
  (~24 fps front, ~6 fps each other with three cameras). Only cameras found at start
  are visited — switching to an empty channel stops the stream for good — and a
  missing one is reported by `/health`. Each quadrant carries its own capture time.
  With `CAMERA_CYCLE = False` the 2x2 picture is used, each camera picture checked
  first (`CAMERA_CORRUPT_PCT`) and a scrambled one skipped; the stream restarts when
  one stays corrupted, and the front camera alone is fallen back to if that fails.
  `CAMERA_SINGLE_CHANNEL = "front"` forces front-only. One auto-exposure serves every
  camera, so a bright view on one can darken the others. `/camera/stats` gives
  `camera_mode`, `camera_fps` and each camera's `corrupt` share.
- **Traffic-light colour cannot be read.** The stock COCO detector emits only
  the class `traffic light` with no colour, and a bounding box cannot tell you
  which lamp is lit. Colour is reported as `unknown` rather than guessed. The
  light's PRESENCE is still used — a light the FRONT camera sees withholds
  "Walk straight" for 5 s after the last sighting (a light seen only behind or
  beside the user no longer does, since the 2026-09-29 audit) — and nothing
  ever says it is safe to cross. It used to announce "No traffic lights detected" while doing
  so, because an unreadable colour lands in the same branch as an empty
  intersection; those two cases are now distinguished.
- **The 2D LiDAR sees one plane.** Overhead branches and low obstacles at foot
  height are missed. The radar helps with glass, not with height.
- **Heading is dead reckoning and unvalidated on real walks.** The scan matcher
  is accurate on synthetic corridors (0.02° per scan) but has not yet been
  checked against a measured walk. It degrades where a scan carries little
  structure — an open field, a bare corridor, a crowd moving with the user — so
  the drift warning is correspondingly weaker there. It degrades safely: an
  unobservable pace falls back to nominal rather than a fabricated figure, and
  the user is assumed to be walking rather than reported as standing still —
  which needed the sensitivity hurdle described under
  [Design notes](#design-notes-bugs-found-and-fixed), because
  fit quality and standard error are both residual-scaled and a corridor fits
  perfectly while observing nothing.
- **A rotating LiDAR sweeps over ~67 ms**, during which the user moves, so each
  scan is slightly skewed. Under 10 cm at walking pace, and not corrected for.
- **Segmentation is front-only.** The other three sectors are segmented on
  demand for the debug view and cost extra NPU time.
- **The voice is espeak-ng, and only audio.** `utils/pi_voice.py` speaks every
  committed sentence through the default PipeWire output (an earpiece when one
  is paired); with no earpiece the dashboard's screen-reader region carries it.
  There is no haptic output. Any other delivery layer must honour the same
  contract: act on `notify.cue_seq` advancing, never on the poll — a consumer
  that speaks `path.instruction` every poll talks over itself several times a
  second, which is the bug the gates above exist to prevent.
- **The environment -> camera-trust gate is wired but DISABLED**
  (`ENV_TRUST_ENABLED = False`), and the reason is the detector rather than the
  gate. Measured on live frames, `environment_detection` called a normal lit
  office "fog" at visibility 0.300 and returned the same verdict for a black
  dead camera — it is not measuring visibility. Validate it against a genuinely
  fogged scene and a clear one before switching it on; fixing the two dead
  cameras probably matters, since a half-black mosaic poisons any contrast
  statistic over the whole frame. The analysis itself now runs on the detection
  thread every 5 s, which it never did before — it was only ever computed
  inside `GET /environment/status`.
- **The traffic-light colour is never resolved**, so the crossing code only
  reports whether a light was seen and always says wait. The branches that
  acted on a colour, and the debug endpoint that injected one, were removed on
  2026-10-01. `parse_traffic_light_class` returns UNKNOWN for every class a
  bounding-box detector can emit, by design. Anyone adding colour
  classification: the value is a VEHICLE light, so red means cars are stopped
  (the removed code once had it the other way round), and nothing may say it is
  safe to cross on the camera alone.
- **The BEV grid feeds two ADVISORY paths and nothing else.** It names shapes
  in the hazard list and breaks left/right turn ties, and that is the whole of
  its authority — it cannot stop the user, clear a direction, or change any
  hazard's urgency. **Doorways are still display-only.** Steering a blind user
  through a detected gap would mean trusting an accumulated grid with the
  decision, and the wall-thickening limit below has to be settled on a real
  walk first.
- **A fast turn thickens walls in the grid.** Registration residual at 10-40°/s
  leaves a wall 170-220 mm thick against a `BEV_WALL_MAX_THICK_MM` of 250, so a
  sharper turn than tested can push one over the line and downgrade it from
  `wall` to the generic `obstacle`. It degrades safely — the obstacle is still
  there at the right range — but it costs the doorway search, which pairs only
  clusters that resolved as walls.
- **A doorway needs BOTH flanking walls resolved.** A single wall that simply
  ends is also an opening, but a wall end is produced just as readily by an
  occlusion or a sparse patch of returns as by a real corner, so those are not
  reported. A doorway seen edge-on, or one whose far side is out of range, is
  missed.
- **BEV camera labels are quadrant-level candidates, not per-object matches.**
  A cluster carries the class names its 90° camera sector currently reports.
  Pinning a label to a specific blob would need a calibrated per-camera FOV
  that this rig does not have.
- **`BEV_MIN_WEDGE_DEG` was tuned on one indoor room**, where 5° gave nine
  simultaneous blind arcs and 15° gave two. It has not had a field pass, and it
  trades directly against missing real glass.
