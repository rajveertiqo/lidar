# What the user hears

> This describes the guidance as it is NOW (since 2026-09-24). Parts of
> [The safety model](safety-model.md) and [Dashboard](dashboard.md) describe the older multi-layer voice
> (drift correction, surface guide, BEV awareness). The drift layer still runs
> and is shown on `/classic`, but does not speak; the surface guide and the BEV
> caution were removed on 2026-10-05, since nothing read their output.

Everything spoken comes from one module, `app/guidance/simple_guidance.py`, from a
closed vocabulary:

| Situation | Sentence |
| --- | --- |
| Path clear | `Walk straight` — said once, never repeated |
| Closing on a wall / glass (within ~12 ft) | `Wall ahead` / `Glass ahead` |
| Front blocked (under 4 ft — was 6.5 ft until 2026-10-01) by a wall / glass | `Blocked by wall, turn left` / `Blocked by glass, turn right` / `…, turn around` — the way out, toward more room (since 2026-11-09; it was `Blocked by wall` alone, and the wearer was not told where to go) |
| Front blocked by something unnamed standing on the path (a pole, a post, a box) | `Obstacle in front, turn left` / `Obstacle in front, turn right` / `Obstacle in front, turn around` (named since 2026-11-09; it was `Turn left` alone, and the camera names no plain pole). A wall beside you that the front brushes is still `Turn left` / `Turn right`. The turn goes toward more room — with room both ways (`WALK_TURN_AMPLE_FT`), away from the side the obstacle is on (2026-11-02; it was a fixed order, so the rig said "Turn right" even with the obstacle on the right) |
| Walking toward a wall beside you that is under 5 ft away (only while actually moving toward it; 2.5 ft on 2026-09-30, 5 ft since 2026-10-01). Inside 4 ft the near-wall sentence below takes over | `Drifting toward right wall, turn slightly left` (and the mirror) |
| Already within 4 ft of a wall beside you (moving or not; 1.5 ft until 2026-10-01) | `Near right wall, move left` (and the mirror) |
| Glass only one front radar board hears, within 4 ft, and room to step away (added 2026-10-01) | `Glass on right, move left` (and the mirror) |
| Something that can hurt you on the path | `Person ahead` → `Person in front`; also vehicles, animals, `Pole` (stop sign / parking meter / traffic-light posts), `Hydrant`, `Bench` — never chairs, laptops or other furniture (still steered round, unnamed). Vehicles are named first. |
| Something that can hurt you in the way, within ~5 m (camera) | `Person on left, turn slightly right` (and the mirror), `Person ahead, turn slightly left`; two with a gap you fit through: `Walk between them, slightly left` / `Walk straight between them`; inside 4 ft: `Person in front, turn left`. With no way past from the camera (or one toward less room), the walk policy's: `Person in front, turn right` / `Person in front, turn around` (since 2026-11-09; it was `Person in front` alone) |
| A vehicle in the camera's box coming at you (its box growing faster than your own walking explains) | `Vehicle approaching, move left` / `move right`: away from the way it is moving across your path, so you step behind it (since 2026-11-09); one coming straight at you, away from the half of the frame it is in — said over anything else. Just `Vehicle approaching` when the side it would send you to has under 2 ft of room or no reading (decided 2026-09-29) |
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
  `tools/sim/glass_sim.py` replays these scenes. Objects are
  named only for something the rangefinders already have on the path, in the
  middle half of the front camera, and big enough to be near — and only for
  things that can hurt you (`SIMPLE_NAMED_OBJECTS` in `app/config.py`). The
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
  `app/config.py`; the lens (`SIMPLE_CAMERA_HFOV_DEG`, 70°) is unmeasured.
- **Warn fast, clear slowly.** A hazard is believed after two polls; it is let go
  after a second; the block and approach edges have 1 ft hysteresis. A more
  urgent sentence is said the moment the previous one finishes; anything calmer
  waits a 1 s confirmation and a 3 s gap.
- **One obstacle, one announcement.** Within one obstacle, a change that is not
  more urgent (a name flickering, the side flipping, a step back) must hold 6 s.
  Blocked sentences repeat after 30 s, 60 s, then every 2 min.
- **The Pi speaks for itself** (`app/guidance/pi_voice.py`): the guidance loop runs on
  the device on every LiDAR scan as it lands (~15 Hz; at least `PI_VOICE_TICK_HZ`, 10 Hz,
  without one — a 5 Hz timer until 2026-10-30). A block the LiDAR measures in two different
  scans is warned of at once (`SIMPLE_LIDAR_CONFIRM_SCANS`): an obstacle appearing 3 ft ahead
  is warned of in ~90 ms, not ~320 (simulated). What the front camera names, and its way past,
  on two different frames (`SIMPLE_CAMERA_CONFIRM_FRAMES`): "Person in front, turn left" in
  ~120 ms, not ~235; "Vehicle approaching" ~0.65 s, not ~1.0 (simulated at ~20 camera
  pictures a second; the full-view camera gives ~16, so ~30 ms more). Every sentence is spoken with `espeak-ng` through the
  default audio output — plug in or pair an earpiece and it follows. While no
  earpiece is connected (HDMI does not count), the dashboard's screen-reader
  banner is the voice instead; never both.
- **Startup.** Silent until the LiDAR has settled. The radar and camera are waited
  for up to 15 s, then guidance starts without them rather than never.

Every threshold is a `SIMPLE_*` / `PI_VOICE_*` constant in `app/config.py`.
