# The safety model

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

## Nobody walks in a straight line

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
(`tools/sim/drift_sim.py --open --veer 3`):

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

## One process at a time

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

## Three resolutions, on purpose

The LiDAR's 360° is divided up three different ways, because one split cannot do
all three jobs:

| Split                                 | Used for                  | Why                                                                                                                                   |
| ------------------------------------- | ------------------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| **8 × 45° sectors**                   | Threat awareness          | Fine enough to say "person at 2 o'clock"                                                                                              |
| **4 cardinals**                       | Steering                  | "Bear 45° left" is not a usable instruction for a blind walker                                                                        |
| **10° cones** (+ body-width corridor) | "Is my path ahead clear?" | A 45° sector clips hallway **side walls** and reported them as an obstacle dead ahead — it said STOP down a completely clear corridor |

## Shape, not just distance — the BEV grid

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
| `spatial_scan` -> `simple_guidance.safer_side` | Breaks a tie between "Turn left" and "Turn right" toward the side with a doorway or open space — only when the walk policy chose no side itself (it does whenever a side has room: see `WALK_TURN_AWAY_FROM_OBSTACLE`) | Pick a side the rangefinders give less than `WALK_MIN_TURN_FT` of room |

`body`-sized clusters are deliberately **not** promoted to a spoken word: the
grid can tell person-sized from wall-sized, but not a person from a wheelie
bin, and "body" would claim it could. A blob that really is a person gets named
by the camera, which can see that.

Measured on-device: **1.2 ms** per scan to fold in, **1.2–4.4 ms** to extract
(rate-capped to `BEV_EXTRACT_HZ` = 5, on the same analysis thread as the sector
stats the STOP decision reads).

**Checking it works.**

```bash
./venv/bin/python -m tools.device.bev_check              # diagnose the running server
./venv/bin/python -m tools.device.bev_check --watch      # keep it on screen while you move
./venv/bin/python -m tools.device.bev_check --checklist  # the physical tests to run
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

## Things that move — LiDAR movers (added 2026-10-07, silent for now)

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
simulation so far — `./venv/bin/python tools/sim/movers_sim.py` (rooms, a corridor,
clutter with lost beams, a car, a bike, the blind arc, a glass pane, a turn, and
radar-only echoes) — not yet on a real walk.
