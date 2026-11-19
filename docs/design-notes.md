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

13. **A block waited behind a status notice (2026-11-10).** A status notice ("Power low",
    "Glass sensor off", "Camera on", "Traffic light ahead") is said only on a clear path, and a
    block that came while it played waited for its airtime: a post 1 m ahead, appearing 0.1 s
    into "Power low", was warned of 1.1 s later, against 0.2 s with nothing playing (~1.2 m at
    walking pace, inside the 4 ft block distance). Notices were left uncut on purpose, as they
    are said once. Now a block cuts into a notice as it does into any guidance sentence
    (`_Gate.admit`), and the notice it cut is told again the next time the path is clear
    (`_Gate.take_cut_status`, `_SensorNotice.unsay`), unless what it reported has already
    ended: power back before "Power low" was heard means nothing is told. A calmer sentence
    ("Wall ahead") still waits for the notice. Separately, "Stairs down in front, stop" and
    "Drop in front, stop" were not urgent: once spoken (`TOF_HAZARDS_SPEAK`) they would have
    waited for the sentence playing, even a 2.5 s one. They now cut in with the urgent tone
    (`unified_guidance.is_urgent`).

14. **A pane across a narrow passage (2026-11-10).** A pane spanning a whole passage, with no
    frame the laser sees, said "Walk straight" 1.5-3 ft from it in passages 0.9-1.8 m wide. Each
    front board's echo was "explained" by the side walls: `_laser_explains` takes any laser
    return in a board's view from 400 mm nearer to `GLASS_LASER_BEYOND_MM` (250) farther than
    the echo, and in a narrow passage the walls fill that window. But no part of a wall is
    nearer than its perpendicular distance: a side wall the laser has found
    (`_side_walls_mm`) L mm to the side no longer explains an echo nearer than L less
    `GLASS_SIDE_WALL_NEAREST_TOL_MM` (100: a board's place on the ring, ~50 mm, and range
    error), in `_laser_explains` and `_side_wall_echo` (`_wall_too_far`). A 1.8 m passage now
    says "Blocked by glass" at 2.6 ft (was "Walk straight"), a 0.9 m nook with walls 1.2 / 2.0
    ft at 1.5 ft. The glass scenes, the scene hunt's false glass (5 of 1500, 5 with radar
    ghosts) and its mirror check (6 of 400) are unchanged. Left as a limit: a pane farther
    than the walls are to the side (see Known limits).

15. **Facing a wall close up, the open side was missed (2026-11-10).** In open space, a flat
    wall 1.0-1.5 ft ahead gave "Blocked by wall, turn around", and an inside corner (a wall on
    the left, the right open) "Stop, no clear way". Each side board's 65 deg beam hears the
    wall ahead at its edge, ~1.86 x the front's distance, and `fuse` took that echo as the
    side's room: 1.9-2.8 ft, under the turn floor (front + `WALK_TURN_MARGIN_FT`). The laser's
    own side line (+-5 deg) saw the right open to 39 ft. Now, for a side the laser can see, an
    echo the laser explains in that board's view (`_laser_explains`) while its line that way
    sees `WALK_GLASS_GAP_FT` past it is that surface off the line: the side's room is the
    board's next echo, or none (`read_direction`, `side_off_line`). Both cases now say "turn
    right". Not on the left, the LiDAR's blind arc (the wearer's body): there the radar is all
    there is, and a person standing there would be dropped with the wall, so facing a close
    wall with only the left open still says "turn around". The scene hunt never sends a turn
    toward a side with under 2 ft of room, before or after.

16. **Set down, the ToF found a pothole (2026-11-11).** On the desk in the window nook the
    terminal showed `would say: Pothole ahead`. Read on the desk, the ToF found the surface it
    rested on on every frame, yet took a level 0.45-0.96 m below it for the ground (the floor,
    or a plane tilted through it) and confirmed a "hole" 0.65 m ahead, and in a simulated desk a
    "step up". Two changes: set down (`rest_surface`), the ground is not judged at all ("set
    down on a surface: no ground to judge"; the near counts and the rest surface, which the
    glass and desk-echo checks use, still are); and a fitted ground must lie
    `TOF_MOUNT_HEIGHT_M` +- `TOF_MOUNT_HEIGHT_TOL_M` below the camera, as `config.py` already
    said. The gate on the points widens with distance for the tilt, and a plane followed frame
    to frame drifted; neither checked the plane itself. The 31 desk frames: no hazards. The
    ground, wall and ToF-fusion simulators: unchanged.

17. **A pillar out of a wall, no block (2026-11-11).** The scene hunt's last "solid on the line
    within 4 ft, no block warning": a pillar standing out of the right wall 1 ft ahead, its edge
    0.1 m off the walking line, said "Near right wall, move left". The walk policy had it
    blocked; `_front_wall` and `_brushing` then explained the block as the wall beside, because
    most of the pillar's returns lay on a "wall": its face, fitted as one, 28 deg across the
    path (rms 50 mm, which widens the match to 150 mm). A wall beside you runs along the path,
    and all of it lies on its line, the part nearest your path too. Now the return reaching
    furthest into the path must lie on a wall running within `SIMPLE_ALONGSIDE_MAX_DEG` (20) of
    the walking line. The pillar: "Blocked by wall, turn left"; a plain wall 0.25-0.38 m beside
    you is still "Near right wall, move left". The hunt: no block missed (was 1), false glass 5
    and the mirror check 6 unchanged. Not done: a flatness limit on those walls; in the 10:23
    nook clip 1 real wall fit in 10 is over 33 mm rms (clutter), and would have blocked.

18. **Radar tracking: a pane by its echo closing (2026-11-11).** Note 14's limit: farther ahead
    than a narrow passage's walls are to the side, a wall can echo from the pane's range, and
    no single radar frame can tell them apart. Cars track radar targets over time; so does
    this now (`walk_policy._closing_echo`). Walking toward it, a pane straight ahead closes at
    the full pace; a side wall's echoes come from points that move with the wearer (its
    nearest point, abeam or at the beam's edge) and stay put; a feature on one wall (a door
    frame) closes at pace x cos(its bearing) and is heard by one board. So: an echo both front
    boards hear alike is followed back on EACH board through the last 1.5 s of frames (as it
    was, and as a closing echo would have moved, since where a pane's echo crosses a wall's the
    nearest one back is the wall's), and must lie on a straight line closing at 0.8 of the
    measured pace (0.5 m/s at least, the only test when the pace is not measured: in a bare
    corridor the laser cannot measure it, and the gyro is off), with the laser on the walking
    line looking past it and nothing but the side walls at its range. Pooling the boards, a
    door frame on one wall crossing the other wall's echo read as a pane; per board it does
    not.
    In `glass_sim.walk_toward` (radar 10 frames a second, 20 mm noise, a board missing 1 frame
    in 10), what is said walking toward a pane, today and with it spoken: a 0.9 m passage from
    10 ft, "Glass ahead", then no block at all / "Blocked by glass" at 2.5 ft; a 0.9/2.0 ft
    nook, the same; a 1.8 m passage, blocked at 1.3 ft / 2.5 ft; first heard 5 ft off in the
    0.9 m passage, nothing at all / "Glass ahead" 2.2 ft, blocked 1.5 ft. Empty passages, a
    door frame on either wall, and standing still: never, in 45 runs with noise. Door frames
    on BOTH walls at the same distance do read as a pane: a range cannot tell them apart.
    That, and a real radar's echoes, are why it is logged as "would say" until a walk has
    checked it (`GLASS_CLOSING_SPEAK`). ~0.03 ms a frame. The scene hunt, glass, route,
    movers, ToF and drift simulators: unchanged.

19. **The gyro's code removed (2026-11-11).** The MPU-6500 on SPI was unplugged on 2026-10-23
    (`GYRO_ENABLED` False since) and does not answer (WHO_AM_I 0x00). Its code is gone:
    `gyro_reader`, the `mpu6500` driver, the stride learned from footfalls (`gait_calibration`,
    `/guidance/gait/*`, the gait test page), the gyro half of the heading tracker (yaw fused
    with it, a step-counted pace, `walking_confirmed`), the live map's gyro/LiDAR choice
    (`/map/settings`), `drift_sim --gyro`, `spidev`, and the `GYRO_*` settings. Heading and
    pace come from the LiDAR's scan matching alone, as they did with the gyro switched off,
    so nothing the device says changed: the tests (less the 4 that tested the gyro reader)
    and every simulator give the same results. What it costs is the plaza case in The safety
    model: a slow veer in open space is warned of at 12.3 s, not 6.9 (Known limits).

20. **The live map removed (2026-11-11).** The `/map` page (a 3D view of what every sensor saw
    around the wearer, display only) and its engine (`app/core/live_map.py`,
    `app/api/map_router.py`, `templates/map_test.html`) are gone, with the dashboard's link to it
    and the two radar helpers only it used (`radar_ring`, `radar_beam_deg`). Nothing the
    guidance used came from it: what is said, and every test and simulator, is unchanged. The
    radar's own scope (`/radar/view`, `/radar/map`) stays.

21. **Facing a close wall, the left offered too (2026-11-13).** Note 15 left the left side
    out: it is the LiDAR's blind arc (the wearer's body), so no laser line can show it clear
    past the left board's echo, and a person standing there would be dropped with the wall.
    Facing a wall 1.1-1.7 ft off with only the left open it said "turn around". Now the left
    board's echo is the wall ahead, heard at its beam's edge (`_front_wall_echo`), only when
    all of these hold: the laser's returns within 40 deg of ahead lie on one straight wall
    across the way (`BLIND_SIDE_WALL_RMS_MM`); the echo is at that wall's range inside the
    beam (`BLIND_SIDE_WALL_ECHO_TOL_MM`, to 1.3 x); in the part of the beam the laser can see,
    nothing stands nearer than that wall, and the wall reaches further out to the left than
    the echo is far (a wall on the left would end it at their corner, so a box keeps its left
    wall); the echo is at least `WALK_MIN_TURN_FT` off; and the left camera's picture is
    fresh (`BLIND_SIDE_CAMERA_FRESH_S`), real, not dark or blurred, and names nothing
    (`side_camera_clear`). Then the left's room is the board's next echo, or open. The guidance
    loop hands `walk_policy.decide` the tick's camera snapshot, the one it records, so
    `walk_replay` decides on the same. The first version took a left wall 1.4 ft off, its
    corner 0.7 ft ahead, for the wall ahead (the scene hunt's new `--side-camera-clear`, and a
    focused hunt of 3,000 close-wall scenes, found it); the reach test replaced an "up to the
    blind arc's edge" test. The 2 ft floor answers the worst case the hunt kept finding, a
    post hidden at the echo's range that the camera does not name: the turn still has the room
    every turn needs. What is left is the radar's 0.25 m minimum range (Known limits). The
    general scene hunt and the mirror check give the same results as before; with no camera
    nothing changes.

22. **"Near wall" with something else on the path (2026-11-13).** The scene hunt's block rules
    failed in 8 of 3,000 scenes. Four were its own measure: it took a post with only its edge
    in the walking strip to be as near as its front on its own centre line (3.8 ft for a post
    4.1-4.3 ft off in the strip); `truth` now takes the post's nearest point inside the strip.
    The other four were real, all "Near right wall, move left" over a block walk_policy had
    right. Two were a pillar standing out of the right wall: the drift layer fitted wall and
    pillar into one "wall" tilted 11 deg with a 46-52 mm residual (clean walls in the same
    scenes: 0.5-2.5 mm; the real LiDAR's walls, from the recorded clip: median 7 mm, 90% under
    33 mm), and `_on_surface`'s tolerance, 3 x that residual, took the pillar's tip 65-102 mm
    off the line for the wall, past note 17's fix. One was a post on the path's left half at
    2.4 ft beside a wall inside the path on the right: the block was the wall, and "move left"
    walked toward the post. Now a block is the wall beside you (`_front_wall`'s alongside,
    `_brushing`) only when nothing else is on the path within the block radius
    (`_else_on_the_path`): the wall's line taken from the laser's own returns lined up along
    the path (`walk_policy._side_walls_mm`), not the fitted surface, and no wall found that way
    meaning it is not only a wall. Across 3,000 hunt scenes 5 "Near ... wall" became blocks and
    no other sentence changed; every block rule now holds, the recorded clip replays the same,
    and the glass, route, drift, movers, ToF and ground simulators give the same results.

23. **"Glass ahead" between two posts (2026-11-13).** The scene hunt's one failing rule was
    "no glass, a glass sentence": 32 in 3,000 scenes, 4 of them "Blocked by glass, turn
    right", all with posts and no pane. 30 came from the two glass-door-in-a-wall rules. The
    older one (2026-09-29) took ANY laser return within 45 deg of ahead at the echo's range
    for the wall a door is set in; the newer one (`_door_in_wall_ft`, 2026-11-06) asked that
    both sides lie at one depth, which two posts 3-4 m off often do. Two posts either side
    of an open way look like a door frame to both. What tells them apart is where the radar
    hears: a pane lies on the line of the wall it is set in, and each front board hears that
    line where it comes nearest the board in its view, at whatever angle the wearer meets it;
    two posts are heard each by its own board at its own range. Now both rules also ask
    `_pane_on_wall_line`: the laser's facing returns at the echo's range, on both sides of
    the walking line, fitted to one straight line (rms within `BLIND_SIDE_WALL_RMS_MM`); each
    board must list an echo within `GLASS_PANE_ON_LINE_TOL_MM` (200 mm: the radar frame and
    the laser scan up to ~0.1 s apart at walking pace, a board's place on the ring) of that
    line's nearest point in its view. It rules a door out only when, as well, no range is
    heard alike by every board (`GLASS_SAME_ECHO_MM`): the first version did not ask, and
    someone standing 250 mm in front of the wall beside a glass door, 4.5-6 ft off, tilted
    the line and lost the pane; a pane across the way is heard alike by both boards. With
    returns on one side only, or not on one line, nothing changes. Against the code before
    it, 10 of the 32 became no glass ("Walk straight", one "Near right wall"), and nothing
    else changed in 3,000 scenes: no pane lost, no distance moved. glass_sim, a new grid of
    384 glass doors with a person beside them (0-20 deg, 1.5-8 ft), the mirror check, every
    other simulator and the recorded clip give the same results as before. What remains is
    posts heard alike, which the radar's range cannot tell from a door frame, and hunt
    artefacts (Known limits). The same grid showed a glass door met at an angle is missed at
    some ranges: note 24.

24. **A glass door met at an angle (2026-11-13).** glass_sim only met doors head-on. Met
    15 deg off, a glass door said "Walk straight" from 4.5 to 3 ft; 20 deg off, from 6 to
    4.5 ft; 25-45 deg off, from 11 to 8 ft (and with the laser's noise doubled to 30 mm, at
    -25 deg from 6 ft too). The older door-in-a-wall rule found nothing wrong with the echo:
    the tilted wall the door is set in comes nearer than the pane past 45 deg on one side,
    and `_matches_ahead` takes anything there for a wall beside the wearer that echoed. Every
    echo then had a laser return at its range in its board's view, so it was dropped as
    off-axis, and the laser looks through the pane. `_door_in_tilted_wall` is that rule for a
    tilted wall: the laser's facing returns at the echo's range on both sides of the walking
    line make one straight wall (`_facing_wall_line`, note 23); every front board hears a pane
    on it (`_pane_heard_on`); the laser on the walking line sees `RADAR_AGREE_MM` past where
    that line crosses it, so a plain tilted wall, which it ranges there, is still a wall; and
    the only thing "beside" is that wall running on (`_matches_ahead` with the line, within
    `_FRONT_WALL_ON_MM`). Without that last test, 2 of 3,000 hunt scenes became new false
    "Glass ahead": two posts on a line the boards heard alike, with a side wall or a third post
    past them. Now every door met 15-45 deg off is found from 6 ft, and up to 20 deg off from
    11 ft, with 15 or 30 mm of laser noise; past 25 deg the far side lies beyond the echo's
    match at 8 ft and more, and it is found from 6 ft (Known limits). In the scene hunt
    nothing changed against note 23 (its panes are met head-on); glass_sim, the mirror check,
    every simulator and the recorded clip are the same; a grid of 384 glass doors with
    someone beside them went from 101 missed to 91, none newly missed. Those 91 are a known
    limit, not fixed here.

25. **A glass door with someone standing near it (2026-11-13).** A grid of 384 simulated
    glass doors with a person 0.5-0.8 m to the side, 0-0.4 m in front of the wall, met
    0-20 deg off at 1.5-8 ft: 101 said "Walk straight" or "Near ... wall" (91 after note 24),
    and walking in, 14 of 80 set-ups were not named by 4 ft. Three causes. The person is the
    nearest echo and the laser sees them, so the rules on the nearest echo stand down; the
    pane's own echo, farther and heard alike by both boards, reaches only the door rule
    (`_door_in_wall_ft`). There (1) the person's returns, in front of the wall on their side,
    outnumbered the wall's, so the median depth was the person's, and the line fit failed;
    (2) a tilted wall failed its one-depth test; (3) close up, the person beside counted as
    what echoed. For (1), `_facing_wall_line`, when its returns are not all one line, takes
    the straight line through returns on both sides that most of them lie on (`_wall_among`:
    each pair of up to 8 returns a side, then fitted to those within `_FRONT_WALL_ON_MM`): the
    wall, with what stands in front of it left out. For (1) and (2), the door rule takes an
    echo as the door when `_door_on_line` says so: every board hears a pane on that wall's
    line, this echo is one of those, where the line comes nearest a board that lists it, and
    the laser on the walking line sees `RADAR_AGREE_MM` past the line. The first version
    asked only the first: a solid wall ahead is heard on its own line too, and a side wall's
    echo at a beam's edge became "the door", 0.5-1.3 ft nearer than the wall (dozens of hunt
    scenes; now a test). For (3), counting something beside only if every board hearing the
    echo has it in view found 18 more, but made ~25 new false "Glass ahead" in the hunt and
    broke 2 glass_sim scenes: not kept. Nor was note 24's "only the wall beside" test in the
    door rule: it removes 2 hunt false alarms (post pairs) but loses 36 doors with someone
    near. Now 30 of the 384 are missed, 18 of them at 1.5-2 ft, and every one of the 80
    approaches names the glass by 4 ft (HEAD: 14 did not). The hunt finds 1 more real pane
    and its false "Glass ahead" go 7 -> 8 and 15 -> 15 (post pairs of note 23's kind); glass_sim,
    the mirror check, every simulator and the recorded clip are the same as before.

26. **What the laser sees, not glass (2026-11-17).** The scene hunt's 8 false "Glass ahead" in
    1,500 scenes (15 with seed 7) had three causes. (1) `_arc_corroborates` asks whether the
    laser's NEAREST return across the front arc is at the radar's range. The arc is made of
    the LiDAR's 45 deg sectors that the beams mostly cover (`RADAR_CARDINAL_ARC_SECTORS`), so
    it reaches 67.5 deg each side, ~5 deg past the boards' beams. A post there, which no board
    heard, nearer than the walls or posts they did hear, left their echo "unexplained", and the
    laser looking past it straight ahead made it glass: a 5 m hall's side walls, heard at the
    beams' edges, with a post 1.4 m off at 66 deg, said "Glass ahead". Now
    `_each_board_explained` also corroborates: every front board live, and the nearest echo of
    each seen by the laser in that board's own view (`_laser_explains`, as the single-board and
    door rules already ask). (2) `_pane_heard_on` stands down when some range is heard alike by
    both boards, as a pane across the way is (note 23). A post straight ahead is too, in both
    views at once: with a post either side nearer, at one depth, they read as a glass door's
    frame. A range heard alike at what the laser ranges on the walking line, or past it
    (`RADAR_AGREE_MM`), is that solid thing, not a pane the laser looks through: no longer
    counted. (3) Not changed: two posts at about one depth, heard where a pane between them
    would be (note 23's limit; 4 of the 5 left, 10 of seed 7's 11; the other of those is a post
    standing 27 mm out of a wall), and one post almost hidden behind a nearer one,
    its last sliver a single laser return that `_grazing` takes for an edge-on surface (0.58 m
    in front of the wall behind it at 3 m, under the 87 deg depth-jump cut at 0.69 deg a step):
    `_grazing` judges every glass rule, so it is left. Against the code before it, 11 of 6,000
    hunt scenes (seeds 1 and 7) changed, each a false glass to the right sentence ("Walk
    straight", "Wall ahead": one was "Blocked by glass, turn right" at a wall 8 ft off), and no
    scene with a pane. A pane put in each of the four rooms (1-3.75 m) is named as before up to
    8.2-10.7 ft. Farther, in the two rooms of (1), the hall's walls or the posts echo from the
    pane's range too, and it is not named until nearer, as in the same rooms without the post
    past the beam (the narrow passage limit, Known limits); in the two of (2), the post or wall
    ahead is under `WALK_GLASS_GAP_FT` behind the pane, or the pane behind the wall: not glass by
    its own definition. Before, those far panes were named only through (1) or (2), the same
    mistakes as the false alarms. glass_sim (every scene, both door grids and their
    limits), the route, drift, movers, ground hazard and ToF simulators and the mirror check
    give the same results; the recorded clip is no longer on the Pi and was not replayed.

27. **No turn into a wall or post under 2 ft off (2026-11-18).** The scene hunt's rules allow a
    turn only with 2 ft of room that way. With radar ghosts it found one turn under that
    ("Blocked by wall, turn left" in a passage 0.49 m wide), and a sweep of passages 0.4-2.4 m
    wide (left and right walls 200-1200 mm, open or a wall ahead, 1,323 scenes) found 32, for
    two reasons. (1) `simple_guidance`'s tight turn: a wall brushing the path on one side,
    with the other side near (`SIMPLE_WALL_NEAR_FT`) or under `SIMPLE_MOVE_ROOM_FT`, said
    "Turn" away from it, the only turn not held to `WALK_MIN_TURN_FT`: with the left wall 300
    mm off and the right 350-500 mm, "Turn right" into the right wall. It now needs that room
    (`_clearance`, as the no-room check below it); without, the walk policy's way out is said:
    "Turn around" where the back is open, else "Stop, no clear way". (2) The left lies in the
    LiDAR's blind arc (170-290 deg), so its room is the radar's, and the radar measures
    nothing nearer than `RADAR_START_MM` (250 mm): a wall 200 mm off read as "13.1 ft (nothing
    detected)", the most room of any side. The laser sees that wall where it comes out of the
    blind arc, 290-302.5 deg, inside the left board's beam. `_under_radar_start_mm`: at least 3
    laser returns inside that side's board's beam (+-32.5 deg) nearer than `RADAR_START_MM`
    along the side's direction make the side's room the nearest of them (`under_radar_ft`), not
    open. Only inside the beam, where the radar would hear a thing but for its range: a first
    try took every return on the side's walking line (300 mm wide), and the near edge of a post
    ahead on the right, just over the middle, made the left look shut and the turn go right,
    into the post. Only the sides: at the back the blind arc's other edge is the wearer's body,
    and the arm filter (`LIDAR_SELF_RANGE_MM`) keeps what stays put. If the body shows past
    290 deg too, the left would read shut and left turns not be offered; nothing on the rig was
    recorded to check. The sweep: none of the 32 left. The hunt, scene by scene against the
    code before, seeds 1, 11, 13 (side cameras clear), 12 and 7 (ghosts 0.5), 7,500 scenes: 3
    changed. Two had a post at the front-left with its edge in the left beam within 250 mm,
    where "turn left" walked into it (its edge 89-101 mm from the turn's line): now "Turn
    around"; the third, the 0.49 m passage, "Stop, no clear way". The glass, route, drift,
    movers, ground hazard and ToF simulators and the mirror check give the same results. Not
    changed: the hunt's room test looks 5 deg either side of the turn; a post at the front-left
    outside the left beam, in the turn's lane, is not counted, and a check of the turn's whole
    lane counts the obstacle being faced, beside the shoulder after any turn in place, as well
    (6 and 10 such turns in seeds 1 and 11 before, 6 and 9 now, left and right alike).

28. **Walking in, through the spoken path; a held pane blocks at once (2026-11-18).** The scene
    hunt and glass_sim judge one poll of the decision (glass_sim's walk: `read_direction` per
    frame). What the wearer hears also passes through time: `simple_guidance._World` confirms a
    block on a second LiDAR scan or after `SIMPLE_ALERT_CONFIRM_S`, holds and releases each
    reading, and the speech gate settles and repeats sentences. `tools/sim/walk_sim.py` walks
    the wearer in at walking pace through scene_hunt's scenes, a guidance tick per LiDAR scan
    (`LIDAR_RPM` / 60, as `pi_voice` runs it), the radar at `RADAR_UPDATE_RATE`, each scan
    analysed with the one before it as `lidar_utils._publish_scan` does (without it a 5 cm
    pole, stepped over by the laser's beams on some scans, read as "Glass ahead"), on the
    scene's clock as `walk_replay` replays a walk. Rules: a block spoken by 3 ft
    (`WALK_BLOCK_FT` less 1 ft for the reaction and the voice), never "Walk straight" within
    4 ft, no glass sentence without glass, no block with nothing on the line; with `--stop`, at
    most 2 sentences and no flipped way out standing 4 s at the block. Solid things: "Obstacle
    ahead" / "Wall ahead" at 7-11 ft, the block at 3.6-3.8 ft. A pane: "Glass ahead" at 10.8 ft,
    but "Blocked by glass" only at 3.1 ft. A block the laser measures is confirmed on its
    second different scan (`SIMPLE_LIDAR_CONFIRM_SCANS`, ~90 ms); one only the radar hears
    kept the 0.2 s, its frames coming slower and repeating between ticks, on top of the
    frame's own age (up to 1/6 s): ~0.3 s, 0.7 ft at 1.2 m/s. But a pane held as glass for
    seconds that comes within the block has not newly appeared; only its range changed. Now
    the block's hold takes it at once when the glass hold is on and this poll reads glass and
    a block: 3.8 ft. A pane first heard inside the block still waits its 0.2 s (through the
    glass hold too). In a 2.4 m corridor the pane is lost between ~5.7 and 3.3 ft, the walls
    echoing from its range (Known limits), held by `SIMPLE_GLASS_LOSS_CONFIRM_S` as "Glass
    ahead" and blocked at 3.3 ft; with `GLASS_CLOSING_SPEAK` (note 18) at 3.8, and the 3 m
    hall's pane named from 9.6 ft instead of 3.6. All 15 scenes pass with 10 mm laser and 20
    mm radar range noise on 4 seeds, the closing rule off and on, at 0.8 and 1.6 m/s, and
    standing at the block. The glass, route and movers simulators give the same results (they
    judge the decision, not the holds).

29. **The wearer doing what is said (2026-11-18).** Note 28's walks go straight in. Whether
    the guide gets someone OUT needs them to act on it: `walk_sim --follow` walks rooms (walls
    and panes as segments at any angle, posts; the radar as scene_hunt's, the first surface
    on each ray across the beam, each surface's nearest) and the wearer, REACT_S (0.5 s)
    after a sentence, turns a quarter on "turn left/right" (90 deg/s), a half on "turn
    around", steps 0.3 m on "move left/right", 15-20 deg on "turn slightly", stands on
    "Stop"; "Walk straight" ends a turn or a wait; a turn said again the way they are already
    turning is the same turn; with `--wait`, after a turn they wait for "Walk straight", 3 s
    at most. Rules: nothing walked into (0.25 m from the middle), the room's way out reached
    in 25 s, no third change of turning side, no "Stop, no clear way" with a way. Found: at
    a 1.8 m dead end, "turn around"; at 67 deg into the turn the front was the side wall
    (still blocked) and the way out became "turn right", held by the speech gate's settle
    (`SIMPLE_SETTLE_S`, a way out changing within an episode) and said at 6.4 s, the turn
    done, facing the open corridor, the front clear since 5.8 s and only the block's
    `SIMPLE_CLEAR_CONFIRM_S` release keeping it. The wearer turned right into the side wall,
    was told "turn right" again, faced the dead end, and walked 2 m into it with the block
    held, unchanged, the whole time, so nothing new was said. The gate already would not
    REPEAT a block off a releasing hold (2026-11-09) nor say one after an outage; now a
    block's new way out within the episode is held the same way (`evidence`, the poll's own
    reading). Both wearers now turn around and walk out. Three rooms fail by known limits,
    named in the tool: the glass door closing a 1.8 m corridor is named at ~2.5 ft and walked
    into within the reaction (with `GLASS_CLOSING_SPEAK`, blocked at 3.9 ft and walked out
    of); an open 0.9 m doorway reads as a glass door, and the wearer is turned away; and a
    corridor turning LEFT is "turn around": the left board hears the way on's near wall at
    its beam's back edge (~1.1 m), which the blind laser cannot show off the left's line as
    it does on the right, so the left reads 3.7 ft, under the 4.8 ft floor (Known limits;
    the side cameras clear change nothing). All the same with 10 mm range noise on 2 seeds,
    both wearers. The straight walks, the scene hunt and glass_sim are unchanged (the gate
    is not in them).

30. **A noisier radar in the walks; a pane ahead with a board missing a frame (2026-11-19).**
    `walk_sim --ghosts P --drop P`: a double-bounce ghost at twice a wall echo's range (as
    the scene hunt), and a board hearing nothing in a frame (as glass_sim's walk). Ghosts alone
    break nothing. Dropped frames do, on glass in a hall or corridor: in 8 runs (range noise,
    ghosts 0.5, drops 0.1-0.2, 4 seeds) 5 were late (1.2-2.9 ft) or never blocked, and the
    code before note 28 as many or worse. Traced: one front board missed a frame and the
    other alone heard the pane straight ahead. `simple_guidance.glass_beside` reads an echo
    from ONE board, the other live and silent, as glass to that side (right for a glass wall
    beside open space), and with beside the walk policy's block is dropped, the front taken
    as the laser's (39 ft, through the pane). The side flipped with whichever board missed,
    so not even "Glass on right, move left" was confirmed, and "Walk straight" stood to 1 ft.
    The mistakes are not alike: a pane ahead read as beside is walked into; one beside read
    as ahead is a moment's needless warning. So beside now also needs the two boards NOT to
    have heard it alike in the last `GLASS_BESIDE_ALIKE_S`: `walk_policy.pane_heard_alike_lately`
    looks through the front frames `_near_echo_steady` records per board (`_board_peaks`) for
    one where every board heard an echo within `GLASS_SAME_ECHO_MM` of each other, at the
    echo's range now or as far farther as a brisk walk (1.5 m/s) closes it since. A glass wall
    beside open space never is (the other board hears nothing), nor, walking, a glass side
    wall beside a solid one (its echo does not close). 0.6 s missed one run where the left
    board missed four frames in a row; 1 s: 7 of 8 pass, the eighth a 2.4 m corridor's pane
    blocked at 2.6 ft, where the walk policy itself first measured it (the narrow-passage
    limit, a frame lost). After a turn, a pane that was ahead stays ahead up to 1 s: the safe
    side. Rooms (`--follow`) stressed alike: one run of four, a 1 m dead end, said "Stop" and
    then "Blocked by glass, turn around" walking out: the boards' ghosts of their own side
    walls (~1.1 m, heard alike) taken for a glass door by `_door_in_wall_ft`'s facing and depth
    tests (its ghost check is not what fails); the code before the same; ghosts or drops alone,
    no. Recorded in Known limits, not changed. 4,500 hunt scenes (seeds 1, 12 and 7, ghosts in
    two), glass_sim, the route and movers simulators: unchanged.

31. **A 1 m corridor's walls read as a glass door (2026-11-19).** Note 30's one stressed room
    failure, traced on the captured scans: `_door_in_wall_ft` returned the side walls' own
    echo (~0.6 m, heard alike by both front boards), not the radar ghosts at ~1.1 m (its
    ghost check holds). In a 1 m corridor the left wall shows to the laser only from 45 to 70
    deg off ahead (the blind arc), ~318 mm of it, under `_SIDE_WALL_MIN_SPAN_MM`, so
    `_side_walls_mm` does not find it; its returns at 41-45 deg read as a surface facing
    ahead-left. One return of the right wall (found) at 29.9 deg and ~995 mm, at the edge of
    the echo's match, fell short of `_on_side_wall`'s 30 deg and read as facing ahead-right;
    `_door_on_line` fitted a tilted wall through both sides, `in_wall` set the walls' own echo
    free of the "corridor's wall" check, and it was a door. Tried, and not kept: finding side
    walls from 30 deg (both sides: 13 more of glass_sim's 720 single scans of a door with
    someone beside it unnamed; the left only: 7, the person's front taken for a left wall,
    "Walk straight" at 2-2.5 ft); at least 3 facing returns each side of a door (doors met
    25-45 deg off named 10 of 27, was 17; 28 more single scans unnamed); `_on_side_wall` from
    25 deg everywhere (through `_laser_explains`' `_wall_too_far`, 13 of 6,000 hunt scenes
    lost real panes, e.g. one 5.9 ft ahead became "Near right wall"). Kept: from 25 deg only
    in the door rule's own side-wall check. The captured scans no door; the stressed rooms
    all as expected on 4 seeds; 6,000 hunt scenes (seeds 1, 12 and 7 with ghosts, 13 with the
    side cameras clear), glass_sim with both door grids scan by scan, the route and movers
    simulators: unchanged. Not changed: the left wall of a 1 m corridor is still not found
    as a side wall by the other glass rules.

32. **Review of notes 28-31 (2026-11-19).** A code review of the uncommitted work found, and
    these were changed: (1) the speech gate's "new way out off a releasing hold" (note 29) came
    before `closed` was judged, so a side that closed while the hold released (someone stepping
    into the side given) was not corrected; it now leaves `closed` alone, as it was said at
    once since 2026-11-09, and holds only a CHANGED way out, not a block renamed with the same
    one. (2) `pane_heard_alike_lately` (note 30) took the newest frame in `_board_peaks` as
    now; that store outlives a clear of `_near_frames`, so frames from another clock (the
    simulators and tests start each scene afresh) or a history that stopped recording counted
    as lately. It now reads only the frames still in `_near_frames`, as `_closing_echo` does,
    newest first, aged from the radar frame being judged. (3) walk_sim clears `_board_peaks`
    between scenes and gives the guidance the same side cameras as the walk policy. Not
    changed: the review's point that the release hold should freeze a block's way out where it
    holds it, rather than a third `evidence` check in the gate: a refactor for later. Tests
    for each fail on the code before. The walks (stressed and clean), rooms and 3,000 hunt
    scenes: as before.

33. **People coming at the wearer, walked (2026-11-19).** walk_sim's rooms take people walking
    straight on (`Room(movers=...)`, circles with a velocity: the laser, the radar and the
    collision see them where they are), and in a room with them each scan runs the trackers as
    `_publish_scan` does (`heading_tracker`, `lidar_movers`, the Kalman `motion_tracker`), the
    radar movers as the guidance loop does, and the guidance gets their snapshots and the
    wearer's motion. The wearer: "..., stop" stands until "Walk straight" (6 s at most);
    "..., move left/right" steps `ROUTE_STEP_ASIDE_M`. The people never give way (the worst
    case). As on the device (`LIDAR_MOVERS_SPEAK` False): someone head-on (2.4 m corridor, open
    floor) or crossing (either side) is walked into: blocked as a post at ~3.5 ft, closing at
    ~2.4 m/s, inside the reaction; the "would say" is there from ~15-18 ft ("Person coming
    toward you, move left", "Person approaching from left, stop"). On (`--turn-on movers`):
    open floor and both crossings get past, on 4 noise seeds and with the waiting wearer; a
    slower walker ahead is a block and a turn either way; someone standing is never called
    moving, said or "would say". Not in a corridor 2.0-3.6 m wide: told to move left, the
    wearer stops and steps; no wall or feature gives the laser their forward motion
    (`heading_tracker`: "assumed", `HEADING_NOMINAL_SPEED_MPS`), so the tracker still has them
    walking at 1.2 m/s, the person closing at 1.2 m/s reads as still, the threat ends, and
    "Near left wall, move right" (0.6 m from it after the step, inside `SIMPLE_WALL_NEAR_FT`)
    sends them back into the person's way. In the open the same happens unseen: nothing
    else to say, "Walk straight" from 0.6 m to the side passes. What would mend it, before the
    warnings are spoken: the guidance telling the tracker the wearer was told to stop or step
    (own speed 0 until "Walk straight"), or no near-wall step toward a person passing. Not
    done: it couples the guidance to the tracker, and wants the walk that turns the warnings
    on. The rooms about a feature not yet spoken (`Room(needs=...)`) are shown, not failed,
    with it off.

34. **Veering down a corridor (2026-11-19).** `walk_sim --follow --veer DEG_S`: walking, the
    wearer turns that much a second unawares (drift_sim's 3 deg/s by default there), down
    1.8, 2.4 and 3 m corridors 46 m long. 1.8 m: "Drifting toward left wall, turn slightly
    right", the 15 deg turn, "Walk straight", every ~5 s: right. 2.4 m: "Near left wall, move
    right" / "Near right wall, move left" every 2 s from the first second. Both walls lie at
    ~3.9 ft, inside `SIMPLE_WALL_NEAR_FT` (4); "near" is one side only (`one(near_sides)`), so a
    few cm off the middle made one near, and the 0.3 m step past the middle the other. Now,
    with both walls in the near band (`SIMPLE_WALL_NEAR_FT` + its margin: a corridor up to
    ~2.7 m), "move" needs the near wall `SIMPLE_WALL_NEAR_OFFSET_FT` (2 ft, ~0.3 m off the middle,
    a step) nearer than the other, or nearer than `SIMPLE_WALL_NEAR_ANYWAY_FT` (1.5 ft: in the
    hunt walls 0.40-0.46 m off lost "move" without it). And `_sentence`: near a wall AND
    drifting into it (converging and closing), the drifting sentence, whose turn mends the
    heading; "move" alone left it, held and not repeated, and the veer grew until the wall
    ahead was a block. 2.4 m now walked at +-3 deg/s. Left as it is (Known limits): 3 m at 3
    deg/s, and every width at 6: the drift is judged at ~9 deg toward the wall, the side wall
    reaches the walking line at ~12 ("Wall ahead", which outranks it, the drift sentence still
    in the speech gate's settle), and the block at ~15 deg turns a quarter, into the other
    wall; the route planner's "Turn slightly ..." would say the right thing, but with it on the
    walk is the same. The hunt: 25 of 4,500 scenes changed, each "Near ... wall, move ..." to
    "Walk straight" with both sides walled under 2 ft apart in range; glass, route and movers
    simulators, the straight walks and the rooms: as before.
    Then the 3 m corridor (the same day): the drift was judged at ~9 deg, but the gate held
    "Near right wall" -> "Drifting toward right wall" as a change to settle (`SIMPLE_SETTLE_S`,
    2 s), "Wall ahead" (the side wall on the walking line within `SIMPLE_WALL_APPROACH_FT`, not
    alongside: the line meets it, and the rule finds something else on the path) outranked
    it, and the block followed. The reverse, drifting then near, was already news
    (`arrived`); so is this now: the turn away said once the voice is free (0.05 s after it
    comes, was 2.05). 3 m at +-3 deg/s walked, no block. Left: 6 deg/s (Known limits), where
    the 15 deg turns at the voice's pace do not keep up and the shallow-angle block remains.

35. **A sensor failing on the way in (2026-11-19).** `walk_sim --faults` (and `faults=` on
    `walk` and `follow`): in a window of the walk the LiDAR publishes nothing (a stall: the
    state keeps the last scan), or 15% of its returns ("lidar-sparse": a hand over it, a dirty
    window), or both front radar boards go offline ("radar-front"). Rules (`judge_faults`),
    each seen to fail with its notice held back: no "Walk straight" off a LiDAR that has
    stopped; a stall past `SIMPLE_LIDAR_FRESH_S` told ("Guidance stopped, use your cane") by
    0.5 s after; the front radar's loss told ("Glass sensor off") by `SIMPLE_GLASS_LOSS_CONFIRM_S`
    + 0.5 s, unless a hazard sentence is standing (the notice is said only where "Walk straight"
    would be); back inside the block, a block said within 0.5 s. Found, all as designed: a
    2 s stall walking at a wall is "Guidance stopped" at 2 s and, the scans back with the wall
    at 3.8 ft, "Blocked by wall, turn right" at once; a 4 s stall on open floor, "Guidance
    stopped" at 2 s and "Walk straight" 1 s after; sparse scans walking at a wall, "Wall
    ahead" becomes "Obstacle ahead" (too few returns to fit the wall) and the block comes at
    3.8 ft; the front radar lost walking at a pane, "Glass sensor off" 3 s on (the pane then
    unseen: the notice is the designed mitigation); lost 2 s and back, the pane blocked 0.2 s
    after, at 3.1 ft. A 1.6 s stall while turning around at a dead end, the wearer eager or
    waiting: no stale way out after it (the gate's after-the-outage rule). Clean and with
    ghosts, drops and range noise: all as expected. No change to the guidance.
    Review of this (the same day): (1) the corridor mute judged the other side by its HELD
    wall (`side_walls(holding=True)`, extended past where it was last seen), so walking out
    into an opening a wall that had ended could mute "Near left wall"; it now takes the wall
    as seen (`walls`), within the near band: a corridor is two walls (10 of the hunt's 25
    changed scenes, posts and not a wall on the far side, have their "Near ... wall" back).
    (2) Could near -> drift as news, with drift -> near already news, let a drift reading
    flickering at its threshold trade the two back to back? Walked parallel to a wall 0.4-0.9 m
    off, in 1.8-3 m corridors, with range noise: no trade in any run; drifting needs the wall
    converging, and noise does not make a parallel wall converge. (A guard on the near
    sentence's standing time was tried and dropped: the near sentence's own airtime, ~2.2 s,
    is longer than the settle, so it never acted.)

36. **Told to stop, and a block only while measured (2026-11-19).** Note 33's corridor failure.
    (1) The Kalman tracker's own-motion fallback (`motion_tracker._own_step`): odometry, still
    things, a trusted heading speed, confidently still, else "assumed": the nominal 1.2 m/s.
    In a bare corridor or open floor the last is all there is, and the wearer, told "Person
    coming toward you, move left", stops and steps; still "walking" to the tracker, the person
    closing at 1.2 m/s reads as standing, the threat ends, and the near-wall sentence takes
    over. Now the guidance tells the tracker what it said (`_tell_tracker` ->
    `told_still`): a mover or vehicle warning or "Stop, no clear way" has them standing (pace
    0, "told to stop", a 0.3 m/s doubt for the step) until "Walk straight" or
    `MOTION_TOLD_STILL_S` (6 s), below every measured source. Not a block's turn: tried, the
    wearer turns and walks on (often before "Walk straight"), and taken as standing, their
    own walking made someone standing still look moving ("someone standing ahead" broke,
    with the warnings on and off). Erring the other way (walking again inside the window) makes
    something coming look faster: the safe side. (2) With that, the corridor walks met a
    gate case: someone passing beside, "Stop, no clear way" (blocked, no side room) was held
    by the block's release hold and said once "Person approaching from right, stop" ended,
    the person 4 m behind and the way clear. The gate had three rules against a block said off
    its hold (its repeat; after an outage; a new way out after a block), and this came by the
    news path after an approach warning, before any of them. Now one rule, before every path:
    a block, newly or again, only while the poll measures it (`evidence`), but a side given
    that closed. Head-on in 2.0-3.6 m corridors, 3 seeds and the waiting wearer: 12 of 12
    past (was 3). Walks, rooms (movers off and on, noise, ghosts and drops, veer), faults, the
    movers and route simulators: as before. The review's point (note 32) that the hold should
    not announce is met at the gate; the hold itself is unchanged.
