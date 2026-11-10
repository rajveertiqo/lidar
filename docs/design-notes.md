# Design notes: bugs found and fixed

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

4. **The cross-sensor review (2026-11-06).** Five independent reviews of the seams
   between the sensors, each finding run through the real decision code. The four
   the wearer heard that were unsafe are fixed, each checked against what is SAID
   on a walk:
   - **A glass door at a corridor's end was walked into.** Each front board's
     nearest echo is the corridor's wall at its beam's edge; the door's echo, behind
     it, never reached the door-in-a-wall rule. In a 1.6 m corridor the wearer heard
     only "Walk straight" all the way to the door. `walk_policy._door_in_wall_ft`
     now judges every echo both boards hear alike, the laser looking past it, and
     takes what FACES the sensor (`GLASS_FACING_DEG`) to be what echoed: a person
     or post on one side of the line explains the echo, the corridor's walls seen
     obliquely do not. Now "Glass ahead" ~10 ft out in 1.2-3 m corridors, door
     across or set in an end wall, "Blocked by glass" by ~2.5 ft. The price: in the
     scene hunt's random cluttered scenes, false "Glass ahead" 13 -> 20 of 1500
     (mostly posts either side at one depth, read as a door in a wall).
   - **A steer past a person went into glass beside.** "Person on left, turn
     slightly right" with a pane 2.8 ft to the right that one board hears (the pane
     is not in the side room the steer checks). No steer, the camera's or the
     route planner's, held or new, may turn toward glass a front board hears; and
     glass beside within `SIMPLE_GLASS_BESIDE_FT` now outranks steers and things
     named further ahead ("Person ahead" had held it back for good).
   - **A car coming from the side was warned 0.3-1 s before contact** (4.5 s
     head-on): its offset from the walking line was judged where it was, and a car
     on a collision course keeps its bearing. `_Vehicles` now also judges where it
     will pass — its nearest front corner's drift and its width
     (`SIMPLE_VEHICLE_WIDTH_M`), carried to contact, from the height's growth (the
     area grows as a car's side turns to view) — when that puts it within 1 m, and
     cars outside the centre band are judged for this alone. 10-30 deg: 4.2 s.
     Passing, crossing and parked cars are judged exactly as before.
   - **A changed instruction waited 6 s.** "Turn right", then someone stepped in
     1.5 ft to the right: "Turn left" came at 8 s. The gate now takes a change as
     news when the side last advised has closed (`_advice_closed`), and any approach
     warning turning into another (still confirmed for 1 s, so a flicker is not
     said). Now 1 s.

   Not fixed, on purpose: a car passing parallel within ~2.6 m (its box's inner edge
   is its far end, read too near) still gets "Vehicle approaching, move …" — a false
   alarm toward safety; the correction would read diagonal near misses too far. The
   rest of the review (shadow-mode movers, route planner, the ToF hazards) waits on
   those features being finished before they speak.

5. **"Blocked by glass" with no glass (2026-11-09).** On the rig, 11 times in 10
   minutes, with a hand near the LiDAR and a person ~1 m ahead. Glass is the radar
   hearing what the laser does not see. A hand at the LiDAR hides everything behind it
   from the laser, but not from the radar boards. Inside 250 mm the self filter also
   drops the hand for its first 0.5 s, so the laser saw nothing at all. Now
   `lidar_utils._front_blocked` marks the front cone `blocked_near_mm` when a return
   under `LIDAR_OCCLUDED_MM` lies within `LIDAR_FRONT_VIEW_DEG` (10°) of straight ahead
   in this scan, counted before the self filter. A front with no laser return is then
   `laser_hidden` (walk_policy), not glass. The radar's echo still blocks, said as
   "Obstacle in front, turn …", and as "Obstacle ahead" inside 8 ft. The console line
   shows `laser hidden` when this happens.

   The first version, the same morning, went wider, and real glass went unnamed on the
   rig. It also counted a front arc that returned almost nothing, held for a second.
   Glass with open space or a far room behind it returns exactly that. It also let the
   camera's object outrank glass, and the detector names trucks and horses indoors. Both
   are gone. Left as it was: a LiDAR window covered so closely that it returns nothing
   at all (a hand inside the LiDAR's minimum range) still reads as glass. Missing a real
   pane is the worse error. Covered that far, the LiDAR soon falls under 10% of its
   returns, and the user hears "Guidance stopped".

6. **A window blind: "Walk straight", then "Blocked by glass" (2026-11-09).** On the
   rig, facing a roller blind 2.6 ft ahead. The laser passed through the fabric and the
   window and got nothing back. The front-left radar heard nothing. The front-right
   board heard one faint echo (-17 dB at 1.0 ft) on about 1 poll in 5. Only the ToF saw
   the blind: an obstacle 0.8 m ahead, 1 m wide, confirmed by the camera. Now
   `walk_policy._tof_front` takes the nearest confirmed `obstacle` or `pole` the ToF
   has on the walking path (fresh, its worker running). It can only bring the front
   reading NEARER. It never stands in for the laser in the glass tests, and never drops
   the radar's echo. The first version did both, and a table or a frame just behind a pane
   unnamed the pane and moved the block past it. An echo within `GLASS_TOF_SAME_MM`
   (150 mm) of the ToF's solid thing, or past it, is that thing, not glass. Here: "Obstacle
   in front, turn around".

   The depth-camera review the same day (ToF review) tightened it further:
   - **Only where the laser has no return ahead.** Elsewhere the laser rules, and a pane
     it looks through keeps its name past the ToF seeing the frame.
   - **Measured where the thing stands in the walking corridor** (`path_m`, the cluster's
     nearest point inside it). A wall beside the path, the chest turned 5 deg toward it,
     had read as an obstacle 2.4 ft ahead: "Obstacle in front, turn around" in an empty
     hallway.
   - **Only when the camera itself confirmed it.** "Unseen", below the picture, was the
     wearer's own cane.

   The faint echo itself was clutter. A glass claim from an echo within
   `GLASS_NEAR_STEADY_FT` now needs that echo on `GLASS_NEAR_STEADY_FRAC` of the radar's
   frames since it first appeared in the last `GLASS_NEAR_STEADY_S`, and at least 4 of them.
   A pane that close echoes on nearly every frame; this echo came on 1 in 3-5. Until then it
   blocks as an obstacle. A pane turned to face 1.5 ft away: "Obstacle in front, turn right"
   at 0.3 s, renamed "Blocked by glass" ~2.7 s later. Walked up to: named from 11 ft, as before.

   Open: a plain surface only the ToF sees (no edge for the camera) is dropped ~5 ft out,
   and lost up close ("no ground seen"), so it still ends in "Walk straight". It is no worse
   than before 2026-11-09. To fix: keep a confirmed solid thing while it fills the view.
   Overhangs and low obstacles still wait on `TOF_HAZARDS_SPEAK`. A wall alongside has
   read as an overhang, and range noise as a low obstacle.

   With every block now saying its way out, the scene hunt's mirror test found ties.
   Both side radars heard a wall ahead at their beams' edge, so the left and right room
   read equal, and the fixed order always gave "left": toward a wall 9 ft off with the
   right open. `choose_turn` now breaks an exact tie on the laser's own side reading. A
   scene that is the same both ways to the side readings still goes left (6 of 300).

7. **A real-time voice (2026-11-09).** Timed through the real walk policy, _World and
   _Gate, the guidance itself costs ~6 ms a poll. The voice is slow because of the gate's
   waits. The changes:
   - **Shorter waits.** A sentence that is not more urgent is held `SIMPLE_CONFIRM_S` 0.4 s
     (was 1.0). A change about the same hazard is held `SIMPLE_SETTLE_S` 2 s (was 6).
     Sentences are spaced by `SIMPLE_MIN_GAP_S` 1.5 s (was 3), and never before the last
     one has finished, plus `SIMPLE_AFTER_SPEECH_S`.
   - **Hazards first.** Any hazard cuts into "Walk straight". "Walk straight" itself
     keeps 1.0 s (`SIMPLE_CLEAR_SAY_CONFIRM_S`): saying "clear" too early is the
     dangerous direction.
   - **Way-out changes.** A left <-> right flip, with the side given still having room,
     keeps 6 s (`SIMPLE_FLIP_SETTLE_S`). Near a post just off centre the walk policy
     alternates. A side that closes is said at once: under 2 ft, or under the walk
     policy's own turn floor from here (`turn_room`, `turn_floor`).
   - **A held block.** A block of something standing in front (`in_front`: the ToF's, a
     hidden laser's, a post off the narrow line) clears a held "Near … wall". That
     sentence had held the block back ~0.9 s, and alternated with it at the 4 ft line.
   - **Sparse scans.** A LiDAR scan with almost no returns is said as "Guidance stopped"
     only once such scans have lasted `SIMPLE_SPARSE_CONFIRM_S`. A single one cut in, then
     held the block back ~3 s (the 09:57-10:05 log, three times). After "Guidance stopped",
     what stands is said at once.
   - **Repeats need the block measured.** A block is repeated, or said after an outage,
     only while this poll still measures it. A releasing hold re-said "Blocked by wall,
     turn right" on an open path after the wearer had turned.
   - **Behind the wearer** (in `LIDAR_BLIND_ARCS_DEG`), the radar echo is not dismissed as
     off-axis when the laser explains it ON that line, only beside it. A person 2.3 ft
     directly behind got "…, turn around".

   Hazard -> sentence on the desk:

   | Case | Before | After |
   |---|---|---|
   | Crossing 8 ft -> "Obstacle ahead" | 0.76 s | 0.11 s |
   | Crossing 4 ft -> the block | 0.12 s | 0.12 s |
   | The way out closes -> the new one | 1.1 s | 0.5 s |
   | The same hazard changes kind | 6.3 s | 2.3 s |
   | Path clears -> "Walk straight" | 2.1 s | 2.1 s |

8. **Camera, vehicle and stall fixes (2026-11-09, afternoon).** From the morning's reviews,
   each reproduced on the desk first, and checked against the committed code:
   - **The vehicle step goes behind the car.** The side came from where the car was in the
     picture. A car from 20° right, heading left on a collision course, got "move left":
     where it was going, contact. Now the side is the way it is moving across
     (`_VEHICLE_HEADING_MPS`), held through cut-off boxes. Stepping as told now clears it by
     0.75-2.2 m (headings 190-210° and mirrored); warnings still come 4.1-4.8 s out.
   - **Turning on the spot is not a car approaching.** The turn's own growth of a box at
     the picture's side was never taken off, and its slide was one rate across the
     picture. Both now follow the box's bearing (pinhole). Parked cars 8-16 m off, turning
     10-40°/s: 20 false "Vehicle approaching" in 144 scenes before, 0 now.
   - **A short thing called "truck"** read 2.5 times as far as it was, so too little of the
     wearer's own walking came off its growth. The ToF's distance in its box, or the
     LiDAR's median across it, now sets that when it is much nearer
     (`_VEHICLE_MEASURED_NEARER`). Walked toward: no "Vehicle approaching", no "be careful".
   - **A box cut off at the picture's side** gives no way past on that side. A car across the
     path filling the frame had both sides tied, and "Car in front, turn right" went toward
     2.95 ft with 6.2 ft on the left. A blocked turn from the camera is now also judged by
     the walk policy's own room each way (`turn_room`), not the 45° sectors that see the
     car itself.
   - **A named block keeps its way out.** "Person in front" alone is now "Person in front,
     turn right", and in a 1 m corridor "Person in front, turn around" (new sentences;
     registry revision 10).
   - **A person just behind a glass door.** Their laser return "explained" the pane's echo,
     the echo was dropped, and the block came at the person, up to 1.9 ft late or not at all.
     The door-in-wall and single-board rules now take the echo as the distance whenever the
     laser on the line sees past it at all; the glass NAME still needs `WALK_GLASS_GAP_FT`.
     Glass door in a wall, and a glass wall across, now block at the pane for every
     distance tried. Left: a pane with a wall beside it and someone within 250 mm behind
     still blocks ~0.7 ft late (`GLASS_LASER_BEYOND_MM`).
   - **A hung loop reaches the phone.** The stall watch's "Guidance stopped" went to the Pi
     voice alone; with the phone as the voice, nobody heard it. It now takes its own cue
     number (`simple_guidance.stall_cue`, under a small lock of its own, never the
     guidance lock), and `latest_guidance` hands it to the bridge while it is the newest
     cue. The bridge sends it urgent, flagged loop-stale.
   - **The glass door at a corridor's end, again.** Nearer than the side walls' echo
     (~3.8 ft in a 2 m corridor), the door's echo IS each board's nearest, and was set
     aside as the corridor's wall; the side walls, square-on at the edge of the view,
     also read as "something beside". A 2 m corridor said "Walk straight" from 3.8 to
     2.2 ft, and "Blocked by glass" came at 1.5 ft. Now `walk_policy._side_walls_mm`
     finds the corridor's walls from the laser (returns that line up along the path):
     they are neither "beside" nor a pane, an echo the laser sees them at is theirs, and
     a nearest echo is the door's when the laser sees the wall it is set in at that depth
     on both sides. 1.2-3 m corridors: "Glass ahead" from ~11 ft, "Blocked by glass" at
     3.2-3.5 ft. With someone just behind the door, the door is still the distance. The
     scene hunt's false glass went down, 14 -> 11 of 1500 (7 -> 5 with radar ghosts), and
     its false blocks stayed the same 22. glass_sim's two corridor "limit" scenes are now
     required glass.
   - **A desk read as glass.** On the rig at a desk with a monitor (14:46 run): no laser
     return ahead, the radar steady on the desk and what stands on it (0.26 and 0.52 m),
     "Blocked by glass, turn left" every 2 min for 14 minutes. The ToF had it: a low
     obstacle 0.55 m ahead, camera-confirmed. Now an echo is not glass when the ToF has a
     solid thing confirmed on the path within `RADAR_AGREE_MM` of it AND a front board hears
     that thing itself (an echo within `GLASS_TOF_SAME_MM` of it). A low obstacle or an
     overhang counts wherever the laser looks (what its one plane passes over or under: a
     desk, a table, a counter, a shelf); a taller thing only where the laser has no return
     ahead, since a glass door's frame is one at the pane's range. Still blocked, said as
     "Obstacle in front, turn …". Checked: the glass scenes with a ToF thing 0.5-1.9 ft
     behind the pane, as an obstacle or a low obstacle, keep "glass" in all 316.
   - **Glass at a desk, again (17:17 run).** The rig on a desk in a 0.9 m nook, facing the
     window blind 4.5 ft off, the laser ranging it: "Blocked by glass, turn left" at 1.2 ft.
     Three causes, three changes:
     - The right board heard its wall at ~1.1 ft, inside `RADAR_CONE_TRUST_MM`, and the
       corridor-glass rule (NEW-045) took it for a pane in the walking corridor. Now it
       does not when every board that hears that echo has a side wall on its own side the
       laser sees at that range (`_echo_is_side_wall`); a pane ahead is heard by the board
       with no wall beside it too.
     - The left board heard the desk and what stands on it, below the laser's plane, and
       the ToF said nothing: on a desk it finds no floor ("no ground seen"), so no hazards.
       Now every ToF frame also counts its points on the walking path by distance, out to
       1.3 m and no lower than 1.0 m below it, with no ground needed
       (`ground_hazards.near_counts`, the snapshot's `near`). 200 or more at an echo's
       distance (`GLASS_TOF_SAME_MM`): that echo is a solid thing, not glass. Worn, the
       floor never comes that near in its view. A glass door's push bar within ~3.8 ft is
       such a thing too: "Obstacle in front", still the block ("Glass ahead" before it).
     - "Blocked by glass" was said with a person 0.5 ft away, the laser ranging them, from
       the 3 s glass hold. A block the laser itself measures, on a poll with no glass, now
       drops the held glass at once.
   - **No way past a moving vehicle.** `camera_path` treated every vehicle as standing: a
     car crossing 3-4 m ahead from the right got "Car on right, turn slightly left",
     toward where it was going. Now it plans a way past a vehicle only once `_Vehicles` has
     watched it long enough to judge, and judged it still (`still_boxes`); a moving one is
     left to the vehicle warnings. A parked car is still steered past. A vehicle measured
     going away across the line at a crossing car's pace (`_VEHICLE_PASSED_MPS`), the
     whole box past the middle, is no longer coming at the wearer. Left as it was: a car
     crossing 6-10 m ahead at an angle still gets "Vehicle approaching" just as it crosses
     the middle. Clearing it there would trust the box's predicted path in the middle of the
     picture, where a noisy one could clear a car coming straight on.

9. **Evidence on every mark, and a clock that can jump (2026-11-10).**
   - **"Wrong" and "Missed" save what the sensors saw.** The guidance ticks of the last
     ~12 s are kept in memory (`WALK_CLIP_ON_MARK`, ~0.3 ms and ~10 kB a tick). A tester's
     Wrong or Missed (`/guidance/mark`: the dashboard, the phone) writes those from
     `WALK_CLIP_BEFORE_S` (10 s) before it to `WALK_CLIP_AFTER_S` (2 s) after, as
     `logs/walks/clip-<time>-<kind>.jsonl.gz`, the recording's own format; the mark's reply
     names the file. `tools/field/walk_replay.py <clip>` replays it through the current code.
     The newest `WALK_CLIP_KEEP_FILES` (50) are kept. Until now a false alarm on the rig
     could only be rebuilt from screenshots (the 17:17 glass, 2026-11-09).
   - **A wall clock stepped forward is not a dead LiDAR.** With no RTC battery, the Pi's
     clock can jump when NTP syncs mid-walk (the phone's hotspot connecting). A 50 ms scan
     then read 120 s old, and the wearer heard "Guidance stopped, use your cane".
     `freshness.reading_is_current` now goes by the time the store received the reading
     (monotonic, its capture age taken off) wherever there is one.
   - **The scene hunt is repeatable.** It now clears the near-echo history per scene, as
     glass_sim does. Each pane had been judged against the radar frames of the ~150 scenes
     before it, and the verdict turned on how fast the run went.

10. **Learning from walks, phase 1: labels (2026-11-10).** The device said "Obstacle in front"
    at a window blind it should have called a wall. No rule change can be checked against one
    screenshot. The dashboard's label buttons (see Field testing) save what was really there
    with the sensor data, and `tools/field/label_report.py` scores the current code against every
    label. There is no learning on the device. Phase 1 tunes the existing thresholds against
    the labels. Phase 2, at ~500 labels, trains a small classifier on a laptop that only
    NAMES what the rules already block for (wall, glass, obstacle, person); the rules keep
    every stop, block and turn. Each phase must not break a label (the report's exit status).
    `simple_guidance.obstacle_of()` (what a sentence names) was taken out of `guidance()` so
    the report can ask it of a recorded sentence.

11. **A rule before a model: the ToF names walls (2026-11-10).** The blind needed no
    model. The ToF already looks at the path 10 times a second, and a wall is what is flat,
    wide and upright across it (see "Walls by the ToF"). The ToF sends its numbers (length,
    flatness, height, distance) for the last 3 frames, and the guidance judges them with the
    `TOF_WALL_*` thresholds. So a recorded walk replays with new thresholds, and the label
    buttons tune and check them. Two things found on the way: the ToF's "goes on above its
    view" test (3% of a thing's points in the picture's top 3 rows) never held for a wall
    filling the picture, which has ~1.7% there. And an obstacle's distance from its nearest
    3% of points jumped 2.5 -> 1.2 m between frames with range noise; the wall's distance
    comes from its surface samples instead.

12. **Set down on a desk, it blocked on the desk (2026-11-10).** In the window nook it said
    "Obstacle in front, turn left", front 0.9 ft, with the laser seeing the blind at 4.0 ft.
    The front radar boards heard the desk top 0.29 m ahead. The ToF saw it too, 3,547 points
    in a flat layer 2 cm under its lens, so the "solid, not glass" check (yesterday's desk
    fix) made the echo an obstacle. In a frame captured there, the desk lies within 2 mm of a
    plane, facing up, with the lens tilted ~16° up. `ground_hazards.rest_surface` finds such a
    surface in the ToF's own frame, not the worn level frame, since set down the rig is not
    tilted as worn: a flat layer facing up, within 0.45 m and no more than 0.4 m below the
    lens. Its points leave `near_counts`, and `walk_policy.read_direction` drops a front echo
    no further than the surface reaches (+`REST_ECHO_TOL_M`) that the laser sees past.
    Replayed with it, the 10:23 clip says "Wall ahead" on 247 of its 252 ticks, against 215
    "Obstacle in front" recorded. Worn, nothing level is that near under the ToF; tests check
    that four worn scenes find no such surface. `/tof/live` says when it thinks it is set down.
