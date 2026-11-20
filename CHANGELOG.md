# Changelog

What changed in each release of Shappe Guide. The version is `APP_VERSION` in `app/config.py`
(the API reports it). Why each change was made is in
[docs/design-notes.md](docs/design-notes.md); every commit is in `git log`.

## [Unreleased]

### Added
- **Cars in the walking simulator** (design note 38). `walk_sim --follow` rooms take cars
  (4.5 x 1.8 m boxes) seen by a synthetic front camera, so the spoken vehicle warnings are walked
  with the wearer acting on them: head-on and from 20 deg off at 18 km/h, warned ~3.4 s ahead
  and sent behind the car; a car passing 3 m off, driving away, or parked is never called
  approaching; the camera's "Person ahead, turn slightly right" is followed. A car crossing from
  the side is not warned of in time (Known limits).
- **`walk_sim --timing`**: each guidance stage's cost per call, as a table. On this Pi 5 a tick
  is ~8-9 ms (scan analysis ~1, `walk_policy.decide` ~4.4, `sg.guidance` ~3.4; p99 ~17, the worst
  20) of the ~43 ms a 23 Hz scan allows, and no slower than on 2026-11-17. **The battery walk**
  (`docs/field-testing.md`) gains the live sensor check before setting off and the fixes since
  2026-11-18 to walk, each with what to hear and what is wrong.
- **`tools/field/walk_report.py`: a recorded walk, summed up**, for the first walks on the
  device. Sensor gaps (the LiDAR's scan age, the front radar's losses); every sentence, how
  often, and with `--timeline` when and how far off; each block and how far off it was, late
  ones flagged; the desk simulators' checks on the real walk ("Walk straight" with something
  measured within 4 ft, a way out flipped, near-wall ping-pong, quick repeats); each tester
  mark with what was said before it; what each feature not yet spoken would have said. With
  `--replay`, through the current code and with each such feature turned on: how many
  announcements it would change. **`walk_sim --record DIR`** writes the simulator's walks as the
  device records them, so the replay and the report can be tried at the desk (a simulated walk
  replays 100% the same).
- **`tools/device/live_check.py`: what the real sensors do, from the running app** (read-only,
  GET requests only). Per radar board, from its own frames: frames a second, frames missed
  between two that heard something, frames with a double-bounce ghost, and the nearest echo's
  jitter; the camera's side of the live picture's delay; the LiDAR reader. The walking
  simulator assumes these (`--drop`, `--ghosts`, `--noise`); this measures them.
- **A walking simulator through the spoken path** (`tools/sim/walk_sim.py`, design note 28). The
  scene hunt judges one poll of the decision; what the wearer hears also passes through time
  (a block confirmed on a second scan, holds, the speech gate). Here the wearer walks in at
  walking pace, every LiDAR scan (~23 a second) decided on by `walk_policy.decide` and
  `simple_guidance.guidance` on the scene's clock, the radar at its 6 frames a second, and
  each spoken sentence is listed with how far off the thing was. 15 scenes (posts, a 5 cm
  pole, walls, corridors, panes, empty corridors, open floor); checked: a block spoken by
  3 ft, never "Walk straight" within 4 ft, no glass sentence without glass, no block with
  nothing on the line; `--stop` stands at the block (no chatter, no flipped way out),
  `--noise`, `--ghosts`, `--drop` (radar frames heard empty), `--pace`, and `--turn-on` as
  `walk_replay`. `--follow`: rooms (dead ends, corners,
  a T junction, walls, a pane, a post, a doorway) the wearer walks through doing what is
  said (a quarter turn on "turn left", a step on "move right", ...; `--wait`: after a turn,
  waiting for "Walk straight"): nothing walked into, the way out reached, no back-and-forth.
  Three rooms fail by known limits and say so (design note 29). People walking straight on
  (head-on, crossing, slower ahead, standing), the mover trackers run as on the device: a
  room about a feature not yet spoken (the mover warnings) is shown, not failed, unless turned
  on (design note 33). `--veer`: the wearer turns unawares, down 1.8-3 m corridors (note 34).
  `--faults`: the LiDAR stalling or sparse, the front radar lost, on the way in (note 35).
  In `./run_tests.sh --full`.
- **A full-resolution camera switch, off** (`SHAPPE_CAMERA_FULL_RES=1`, `CAMERA_FULL_RES`): the
  sensor's whole 4656 x 3496 unbinned, so each camera gives its same 66 deg view at 2328 x 1748,
  4x the pixels. Expected: ~4-5 pictures a second against 15.7 (Arducam gives that mode ~9,
  the 4-in-1 board halves it), detection that much slower, and each picture 2-3x the work
  here (a frame's copy 14 ms, the detector's resize 10 ms, the all-four picture 28 ms; ~4 ms
  each now). The settings in camera pixels scale with it (the ToF's aligned cell and camera
  margin, the blur check's sample), the picture shares already do, and the ToF calibration
  rescales itself: every test passes both ways. Off until measured on the device.
- **Where the camera view's delay goes, measured** (`/camera/stats` `pipeline`). The camera
  view still lagged, and nothing said by how much before the app had a picture: its age
  began when the app received it. Each picture is now taken with its own metadata, and its
  sensor timestamp says how long after the sensor read it out the app had it; with the
  sensor's own frame rate (30 a second; the 4-in-1 board gives the app every second one, 15)
  and the share of pictures Picamera2 handed over already finished (up to a frame older). The camera tile's
  age and `X-Frame-Age-Ms` now count that delay too (`X-Capture-Lag-Ms`). Nothing else
  changes: the measurement is for choosing the fix.
- **glass_sim checks glass doors met at an angle and with someone standing by them** on every
  run: 138 doors met 10-45 deg off must be named, and 80 walks in toward a door with someone
  beside it must be warned by 4 ft (design notes 24, 25). On the code before those fixes it
  reports 28 failures. What is not yet named glass (doors met 25-50 deg off from afar, single
  close scans with someone by the door) is printed as a limit.
- **`label_report.py --decide`**: one command after a walk says, for each feature that runs but
  is not spoken yet (stairs and kerbs, glass tracking, movers), TURN ON, KEEP OFF or MORE
  LABELS. Every label is replayed with the feature off and on: turned on it must name 70% of
  its own labels, more than without it, and make at most 5% of the other right labels wrong.
  **The battery walk** in [docs/field-testing.md](docs/field-testing.md) collects enough labels
  for all three in about 45 minutes, and checks the heading.
- **Radar tracking for glass across a narrow corridor** (silent, "would say", until a walk
  checks it: `GLASS_CLOSING_SPEAK`). A pane's echo closes at walking pace on both front
  boards while the walls' echoes stay put, so a pane past the walls' echoes is found. In the
  simulator it blocks panes that today get no block at all.
- **Stairs** and **Kerb / step** label buttons on the dashboard (keys `S`, `K`), so a walk can
  score the ToF's stair, kerb and drop warnings before they are spoken
  (`label_report.py --turn-on tof-hazards`). Walk recordings now keep what those warnings
  would have said (`ground_shadow`).
- **The camera's light, in lux**, from the sensor's own reading, for each camera: on
  `/camera/stats` and in every walk recording. Under `CAMERA_DIM_LUX` (45) `/health` warns
  "Camera dim", and `label_report.py` scores labels in good light and dim light apart.
  Nothing said changes.

### Fixed
- **"Something approaching" off a parked car** (a "would say", `radar_movers`): walking up to the
  back of one, the side board's echo slid along it at the beam's edge, closing faster than the
  walk, and the LiDAR's car side, over 2 m wide, did not count as the LiDAR seeing it. A laser
  return at the echo's range inside the board's beam now does (`laser_sees`).
- **"Drifting toward ... wall, turn slightly ..." is said again while the wearer drifts back
  into that wall** (design note 37). An advisory was said once an episode: the turn mended the
  heading, the veer brought it back, and nothing more was said until "Wall ahead" and a block,
  a quarter turn into the other wall of the corridor. It is now said again once it has stood
  `SIMPLE_DRIFT_REPEAT_S` (2.5 s) and the voice is free. Veering 4 deg/s down a 3 m corridor is
  walked (was not); 3 deg/s in 1.8-3 m corridors as before; walking parallel to a wall, no
  repeat.
- **`/guidance/latency` keeps the start-up notices apart** ("Starting, please wait", "Ready":
  `startup`): decided while the models load, one took 1.4 s and stood as the run's worst tick.
- **A walk replayed twice in one process** (`walk_replay.replay_ticks`) left the walk policy's
  radar frames from the first pass, and the second, not newer, recorded none: with
  `GLASS_CLOSING_SPEAK` turned on it saw no pane closing (a glass door blocked at 3.2 s replayed
  as never changing). `walk_report --replay` and `label_report` (clips out of time order, or
  one clip against a feature turned on) were affected. The replay now clears them.
- **A block is said only while it is measured: one rule in the speech gate** (design note 36).
  Three rules held a block back off the hold that keeps it from flickering off (its repeat,
  after an outage, a new way out); a fourth case fell between them: someone passing in a
  corridor, "Stop, no clear way" held from a second before was said once "Person approaching
  from right, stop" ended, the way clear again. Now one rule: a block, newly or again, only
  while the poll measures it; a side given that closed is still said at once.
- **The mover warnings get the wearer past someone in a corridor** (still not spoken:
  `LIDAR_MOVERS_SPEAK`; design note 36). Told to step aside, the wearer stops; where nothing
  measures their own motion (a bare corridor, open floor) the Kalman tracker still had them
  walking 1.2 m/s, someone coming at 1.2 m/s looked still, the warning ended, and "Near left
  wall, move right" sent them back into that person's way. The guidance now tells the tracker
  (a mover or vehicle warning, "Stop, no clear way"; not a block's turn, after which they walk
  on): their pace is 0 until "Walk straight" or `MOTION_TOLD_STILL_S` (6 s). Head-on in 2.0-3.6 m
  corridors: 12 of 12 walks past, was 3.
- **No "move left / move right" ping-pong down a corridor** (design note 34). Down the middle
  of a 2.4 m corridor both walls lie at ~3.9 ft, inside the 4 ft near-wall band: a few cm off
  made one wall "near" ("Near left wall, move right"), and the step past the middle the other,
  every 2 s (`walk_sim --follow --veer 3`). With both walls in the band, "move" now needs the
  near one 2 ft nearer (`SIMPLE_WALL_NEAR_OFFSET_FT`), or under 1.5 ft away
  (`SIMPLE_WALL_NEAR_ANYWAY_FT`). And near a wall while still heading into it, the drifting
  sentence ("..., turn slightly right") is said, not the step: the step left the heading to
  grow until the wall ahead was a block. And "Near right wall" then "Drifting toward right
  wall" is news, said once the voice is free, not after the 2 s settle (it lost to "Wall
  ahead", then a block, a quarter turn into the other wall). Veering 3 deg/s, the 1.8, 2.4 and
  3 m corridors are walked; 25 of 4,500 hunt scenes changed, each a "move" off the middle of a
  corridor to "Walk straight".
- **A 1 m corridor's own walls are not a glass door** (design note 31). With a noisy radar
  (`walk_sim --follow --ghosts 0.5 --drop 0.2 --noise 10`), walking out of a 1 m dead end said
  "Stop, no clear way" and "Blocked by glass, turn around", back to the dead end. The left
  wall, half in the blind arc, is too short in view to be found as a side wall; its returns
  read as a surface facing ahead-left, one right-wall return at 29.9 deg (just short of the
  side-wall check's 30) as facing ahead-right, a wall with a door in it was fitted through
  them, and the walls' own echo (~0.6 m) became the pane. The glass-door rule's own side-wall
  check now starts at 25 deg. The stressed rooms: all as expected on 4 seeds (was 3 of 4);
  6,000 hunt scenes, glass_sim with both door grids scan by scan, and the route and movers
  simulators: unchanged.
- **A pane ahead is not "glass beside" when one radar board misses a frame** (design note 30).
  With radar frames dropped (`walk_sim --drop 0.2`), walking at a pane across a 3 m hall, one
  front board missed a frame and the other alone heard the pane: that reads as glass to one
  side, so the block was dropped ("Glass on right, move left", the front the laser's 39 ft
  through the pane), the side flipped with the board that missed, nothing was confirmed, and
  "Walk straight" stood to 1 ft. Now glass is beside only if the two boards did not hear it
  alike in the last `GLASS_BESIDE_ALIKE_S` (1 s), where it was as the wearer walked in
  (`walk_policy.pane_heard_alike_lately`). Stressed (range noise, ghosts 50%, frames dropped
  10-20%, 4 seeds): 7 of 8 runs pass, was 3; the one left blocks at 2.6 ft in a 2.4 m
  corridor, the narrow-passage limit and a lost frame. 4,500 hunt scenes, the glass, route
  and movers simulators: unchanged.
- **No new way out off a block that is releasing** (design note 29). At a dead end, "Blocked
  by wall, turn around"; turning, the wearer faced the side wall and the way out became
  "turn right", said once the turn was done, facing the open corridor, with only the block's
  1 s release keeping it. Doing what was said, they turned into the wall and back to the
  dead end, and walked into it (`walk_sim --follow`). A block's new way out is now said only
  while the poll still measures the block, as its repeat already was (2026-11-09): "turn
  around", then "Walk straight" and out.
- **"Blocked by glass" as soon as a wall's** (design note 28). Walking in at 1.2 m/s
  (`walk_sim`), a pane already named ("Glass ahead" from 10.8 ft) was "Blocked by glass" only
  at 3.1 ft, a wall at 3.8: a block the laser measures is confirmed on its second scan, one
  only the radar hears waited `SIMPLE_ALERT_CONFIRM_S` (0.2 s) on top of a radar frame's age.
  A pane already held as glass that comes within the block now blocks at once (its range
  changed, not what is there): 3.8 ft. One first heard inside the block still waits 0.2 s. In
  a 2.4 m corridor, 3.3 ft: there the walls echo from the pane's range from ~5.7 ft in and it
  is found again only nearer than them (Known limits); with `GLASS_CLOSING_SPEAK`, 3.8 ft
  there too, and in a 3 m hall named from 9.6 ft instead of 3.6, with no false alarm in any
  scene over 4 seeds of range noise (evidence for the walk that is to check it).
- **No turn into a wall or post under 2 ft off** (design note 27). Found by a sweep of
  passages 0.4-2.4 m wide and the scene hunt. (1) A wall brushing the path on one side with
  the other side near too said "Turn" away from it whatever the room that way: in a passage
  0.65-0.8 m wide, "Turn right" at a wall 1.1-1.6 ft off. That turn now needs
  `WALK_MIN_TURN_FT`, as every other turn does; without it the walk policy's way out is said
  ("Turn around", "Stop, no clear way"). (2) The left, which the laser cannot see down, had only
  the radar, and it measures nothing nearer than 250 mm (`RADAR_START_MM`): a wall 200 mm
  off read as "nothing within 13 ft" and "Blocked by wall, turn left" went into it, as did a
  post at the front-left with its edge that near in the left board's beam. What the laser sees
  of it in that beam, where it comes out of the blind arc, is now the left's room
  (`under_radar_ft` in each direction's view). The sweep: 32 such turns, now none. Over
  7,500 hunt scenes (five seeds, ghosts and clear side cameras among them) against the code
  before, 3 changed, each such a turn to "Turn around" or "Stop, no clear way"; the glass,
  route, drift, movers, ground-hazard and ToF simulators and the mirror check give the same
  results.
- **The dashboard's laser view keeps up.** It drew on a 200 ms timer, 5 of the LiDAR's ~23
  scans a second, each up to 0.2 s late; and every scan was pushed to every viewer as it
  came, so on a slow link they queued in the network buffers: a viewer reading one every
  150 ms saw the room 1.4 s late after a few seconds and 7 s late after 12 (the code before,
  with the real scan pipeline). Now the page draws each scan as it arrives, at the screen's
  next frame (~0.7 ms a draw, the dots in two paths), and asks for each (`next` on
  `/lidar/ws`, 3 ahead, as the camera's pictures): that same slow viewer stays ~0.47 s
  behind, never more, and a fast one gets all 23 a second, 5 ms after the scan (was 15: a
  scan now goes out before the odometry, movers and top-down grid run). A client that never
  asks (`tools/device/anchor_diversity.py`) is sent the newest whenever its last has gone.
- **Fewer false "Glass ahead" among posts and in wide halls.** Two ways what the laser sees was
  still named glass (design note 26). The front boards' echo counted as unexplained when the
  laser's nearest return across the whole front arc was not at its range, and that arc reaches
  ~5 deg past the boards' beams: a post just past a beam's edge, which no board heard, made the
  walls or posts they did hear "glass" (a 5 m hall's side walls, heard at the beams' edges:
  "Glass ahead"). Now each board's nearest echo, seen by the laser in that board's own view,
  is explained. And a range both boards hear alike at what the laser ranges straight ahead
  (a post there) no longer makes two posts nearer either side read as a glass door's frame.
  The scene hunt's false glass: 8 -> 5 in 1,500 scenes, 15 -> 11 with seed 7; in 6,000 scenes
  11 sentences changed, every one a false glass, and none with a pane in it. glass_sim and
  every other simulator give the same results as before.
- **Two confidence settings that never applied now say what runs.** Vehicles were "taken from
  0.45" (`SIMPLE_VEHICLE_MIN_CONF`) and traffic lights from 0.4, but the detector drops every
  detection under `DETECTION_CONF` (0.5) itself, so neither had applied since the first commit.
  Both are now 0.5 (traffic lights' as `TRAFFIC_LIGHT_MIN_CONF`), nothing the device does
  changes, and a test fails if any `*_MIN_CONF` is set under the detector's floor. Taking
  vehicles from 0.45 for real waits on a street walk. Checked alongside: the weak, far
  "person" boxes the dashboard shows (50-59 %, 20-66 ft, on chairs and a monitor) reach no
  sentence: naming wants the front camera, the middle of the view, a box a quarter of the
  picture tall and the laser's range; the motion tracker cannot see a still box move.
- **The dashboard's camera view keeps up over Wi-Fi.** On a laptop it showed 5.6 pictures a
  second of the cameras' 15 (the device itself was at full rate: 14.9 captured, 13 detected).
  Two things were in series on every picture: the server held it until the detector had
  finished it (~55 ms, so it went out ~76 ms old and every second one was missed even on the
  Pi), and the page asked for the next only once this one was drawn, adding the Wi-Fi's whole
  round trip (45-117 ms measured). The page now keeps requests out for the next three pictures
  by number (`/camera/live?ahead=`), passing over one a newer has overtaken, and the picture
  carries the detector's latest boxes, a picture or two behind, drawn as current under 0.2 s
  old. Simulated: 14.3-15 a second for round trips up to 160 ms (one at a time: 5.3 at 90 ms).
- **The scene hunt's mirror check no longer fails on ties.** It reported 6 of 400 scenes "not
  mirrored": all were a wall straight ahead with the sides alike to the decision (equal room,
  the block within 3 deg of ahead), where "turn left" either way is right; one was its own
  mirror image, which no fixed choice can pass. Those are now counted as ties. Over 4,000
  pairs there were 107 ties and no real mismatch; a planted left/right fault is still caught.
- **A glass door with someone standing near it is found.** With a person beside the door,
  in front of its wall, it said "Walk straight" in about 1 in 4 simulated scenes, and
  walking in, 14 of 80 approaches were not warned by 4 ft: the person was taken for what
  the radar heard, and mixed into the wall the door is set in. Now the wall is found with
  them left out, and every approach is warned by 4 ft (design note 25).
- **A glass door met at an angle is found.** Met 15-20 deg off, a glass door said "Walk
  straight" from 3 to 6 ft away: the tilted wall around it was taken for a wall beside the
  wearer. It is now "Glass ahead", then "Blocked by glass" at the glass, from 11 ft up to
  20 deg off and from 6 ft up to 45 deg off (design note 24). A plain tilted wall is still a
  wall.
- **No "Glass ahead" between two posts.** Two posts either side of an open way read as the
  wall a glass door is set in, and it said "Glass ahead", or "Blocked by glass, turn right",
  with no glass there. A door's pane lies on its wall's line, where both front boards hear
  it; two posts are heard each at its own range (design note 23). The scene hunt's false
  glass fell from 32 to 22 in 3,000 scenes; no pane is missed that was found before.
- **A pillar or post on the path is no longer called "the wall beside you".** "Near right wall,
  move left" was said with a pillar standing out of the wall 2.5-2.9 ft ahead on the path, and
  once with a post on the path's left half at 2.4 ft, which "move left" walked toward. A block
  is put down to the wall beside the wearer only when nothing else is on the path within 4 ft
  (design note 22). The scene hunt's block rules now all hold; it measures a post with only
  its edge in the walking strip at its nearest point there.
- **Facing a close wall, an open left is offered** ("Blocked by wall, turn left"), where it said
  "turn around". The left is the LiDAR's blind side, so the left board's echo is taken for the
  wall ahead only when the laser sees that wall stretch out past it, the echo is at least 2 ft
  off, and the left camera's fresh picture shows nobody (design note 21). Without the left
  camera, or nearer than ~1.1 ft to the wall, it is "turn around" as before.
- **No false "Pothole ahead" when the device is set down.** On a desk the ToF took the floor
  below it for the ground and found a hole. Set down, the ground is no longer judged, and a
  ground must lie at the mount's height (1.45 m +- 0.45 m), as the settings already said.
- **A pillar standing out of a wall now blocks.** It was "Near right wall, move left", with
  its edge 0.1 m off the walking line. A block is the wall beside you only if what reaches
  furthest into your path lies on a wall running along it (`SIMPLE_ALONGSIDE_MAX_DEG`).
- **A block warning no longer waits behind a status notice.** A post appearing while
  "Power low" (or "Glass sensor off", "Camera on", ...) was being said was warned of 1.1 s
  late; now 0.2 s, as with nothing playing. The notice it cut is said again once the path is
  clear.
- "Stairs down in front, stop" and "Drop in front, stop" now cut in as urgent, with the
  urgent tone, once the ToF's ground warnings are spoken.
- **A glass pane across a narrow passage** (no frame the laser sees) is now found once it
  is nearer than the passage's walls: a 1.8 m passage says "Blocked by glass" at 2.6 ft, where
  it said "Walk straight". A side wall can no longer "explain" an echo nearer than itself
  (`GLASS_SIDE_WALL_NEAREST_TOL_MM`). Farther panes in narrow passages are still a known
  limit.
- **Facing a wall close up, an open right is offered.** A flat wall 1.0-1.5 ft ahead in
  open space said "Blocked by wall, turn around", and an inside corner "Stop, no clear way":
  the side radar heard the wall ahead at its beam's edge. Both now say "turn right". The left
  (the LiDAR's blind side) is unchanged.

### Changed
- **A block's console line shows the room each way** ("room L 13.1 R 1.4 ft (needs 2.0)"): the
  walk policy's own, which the way out was chosen by. On the desk, "Blocked by wall, turn
  left" was said with the laser's left wall 1.5 ft off, and nothing showed why the left had
  the room (the left's is the radar's alone: the laser's blind arc). The start banner names
  `logs/speech_*.log` only when it is written (`SPEECH_LOG_TO_FILE`).
- **The depth and radar pages look like the dashboard** (`/tof/live`, `/radar/view`,
  `/radar/test`). They were the October pages, dark only: now the dashboard's colours, cards
  and header, light or dark in the theme the dashboard saved, and one row of buttons across
  all four pages (Dashboard, Depth, Radar, Boards; the dashboard gained Radar). Their scripts
  are as they were but for the colours they draw with, which now follow the theme, and two
  fixes where the eye lands: the radar map's "270°" was cut off at the left edge, and at 90
  and 270 deg a distance label ran along the radius into its own arc or dot ("1.9 ft" read
  ".9 ft"), so it now sits beside it. Laid out for a phone too: one column, icons only.
- **The dashboard redrawn** (`templates/dashboard.html`): a new look in dark and light, the
  last sentence said as the headline, coloured by its rule. On a phone the Right / Wrong /
  Missed / Note buttons are pinned to the bottom of the screen, and what a press saved shows
  above them. The label buttons have their own card; the run counts head "Spoken this run";
  the camera key folds away; the radar boards are in order of angle. The laser view has the
  radar circle the live map had: each board's beam out to its nearest echo, and the radar's
  reach (from `/radar/map`, as the radar scope). Same keys; the theme saved by the old page
  carries over.
- **The dashboard's big sentence is the last one SAID.** It showed the sentence the guide was
  still weighing, which flickers: the screen said "Blocked by wall, turn left" while the wearer
  had heard "turn right". The card shows that sentence and nothing else: the reason line, the
  live sensor chips ("wall ✗" under "Blocked by wall" read as a contradiction), "said 19 s
  ago" and "quiet — unconfirmed" are gone (the terminal has them). "Last said" could also show
  a stale sentence after the app restarted; it now starts again with the app.
- **The dashboard's "Spoken this run" lists only what was said**, as the terminal's 🔊 lines:
  the reason, the LiDAR scan's age and which voice said it. The silent "would say" lines (route
  planner, ToF hazards, movers) and the tester's marks had been listed with them, and read as
  said; they stay in the terminal and `/guidance/log/text`.
- **Stricter lint** (`pyproject.toml`): statement errors (E7) and bugbear (B) now block CI
  too. What they found, fixed: two bare `except:` that would also swallow Ctrl+C and
  shutdown (`lidar_utils`, `unified_navigation_system`), and four errors in the phone bridge
  re-raised without their cause (`raise ... from exc`).
- **`simple_guidance.py` split** (4,430 -> 3,690 lines): the closed vocabulary is now
  `app/guidance/vocabulary.py`, and the speech gate and sensor notices
  `app/guidance/speech_gate.py`. `simple_guidance` re-exports every name, so nothing that
  imports it changes; the tests and every simulator give the same results.
- **Sharper camera pictures while walking:** the exposure is kept to 10 ms while gain can make
  up the light (`CAMERA_SHUTTER_CAP_MS`); the sensor chose 24-30 ms indoors, which walking
  blurs. Dim light gets the same most exposure as before.
- **The app logs through `logging`** (`app/core/logs.py`), not `print()`: every line has its
  time, its level from WARNING up, and its module, and `SHAPPE_LOG_LEVEL` sets how much is
  written. Same place as before (the terminal and `logs/run_*.log`). The live guidance console,
  the start-up banner and the hand-run hardware checks still print.
- **The app is one package, `app/`**, split by job: `app/sensors/` (hardware), `app/perception/`
  (what is around, the walk decision), `app/guidance/` (what is said), `app/core/` (shared
  state, freshness, recording), `app/api/` (was `routers/`). `main.py` and `config.py` moved
  into it: settings are now in **`app/config.py`**. Module names are unchanged.
- It starts as `uvicorn app.main:app` (`./start_lidar.sh` does this; nothing to change in how
  you start it or in the service). The radar check is `python -m app.sensors.radar_utils`.

### Removed
- **The live map** (`/map`, the 3D test page): its page, engine and routes, and the dashboard's
  link to it. Display only; nothing the device says used it. The radar scope (`/radar/view`)
  stays.
- **The gyro (MPU-6500) and all its code**: unplugged since 2026-10-23. Its reader and
  driver, the stride learned from footfalls (`/guidance/gyro`, `/guidance/gait/*`, the gait
  test page), the gyro half of the heading tracker, the live map's gyro/LiDAR choice
  (`/map/settings`), `drift_sim --gyro`, `spidev` and the `GYRO_*` settings. Heading comes
  from the LiDAR alone, as it already did; nothing the device says changed.
- `shappe_guide.egg-info/`, build metadata committed by mistake (now ignored).
- The previous dashboard, `/classic` (`templates/index.html`), and its `/test` alias: the
  dashboard at `/` has everything used in testing.
- Fog / rain detection (`environment_detection`, `/environment/*`): switched off since it
  called a lit office "fog", and so never consulted.
- The road-crossing state machine (`traffic_light_detection`, `/crossing/*`): nothing on the
  device ever fed it. "Traffic light ahead" is unchanged (it comes from
  `unified_navigation_system`).
- Unused routes: `/camera/video_feed`, `/camera/stream/{sector}`, `/camera/seg_grid`,
  `/camera/frame`, `/camera/frames/all`, and the `/guidance/ws` websocket. Unused settings
  `LIDAR_TO_CAMERA`, `STREAM_FPS`, `TRAFFIC_SIGN_STALE_S`, `ENV_*`.

## [3.1.0] - 2026-11-10

### Added
- **"What was really there?" label buttons** on the dashboard (Wall, Glass, Obstacle,
  Person, Nothing; keys W G O P N). Each press saves a clip of the sensor data with the
  label. `tools/field/label_report.py` scores the code against the labels and exports them
  for training on a laptop.
- **The ToF names walls.** A flat, wide, upright surface across the path turns
  "Obstacle in front, turn left" into "Blocked by wall, turn left" (`TOF_WALL_SPEAK`). It
  renames only: the block and the turn are unchanged. `/tof/live` shows the verdict live.
- **Set down on a desk**, the ToF finds the surface the device rests on, and the radar's
  echo off it no longer blocks ("Obstacle in front" at 0.9 ft with a wall 4 ft away).
- Replayable clips on every Wrong or Missed press (`logs/walks/clip-*.jsonl.gz`).
- CI on every push (lint, tests, a type report), `pyproject.toml`, `requirements-dev.txt`,
  `.editorconfig`, this changelog.

### Changed
- Project layout: tests in `tests/`; tools in `tools/sim/`, `tools/field/`, `tools/device/`;
  the README split into `docs/`.
- A reading's freshness goes by the monotonic clock: a wall clock stepped by NTP mid-walk no
  longer says "Guidance stopped, use your cane".
- `tools/field/walk_replay.py` gives the decision code the ToF analysis recorded with each
  tick, so ToF-dependent sentences replay as they were said.

### Fixed
- `scipy` was missing from `requirements.txt` (the ToF's obstacle analysis needs it).
- The ToF's "goes on above its view" test never held for a wall filling the picture.

### Removed
- `tools/odometry_density.py` (unused), `pyvenv.cfg` (a virtualenv leftover).

## [3.0.0] and earlier

Before this changelog. See `git log` and [docs/design-notes.md](docs/design-notes.md).
