# Known limits

- **The camera picture (2026-11-02, new cameras).** The board shows all four cameras in
  one 2x2 frame in the IMX519's full-view mode (2328x1748): each camera its whole 66 deg
  view at 1164x874, 15.7 pictures a second (the board halves the sensor's 30 fps; the
  cropped 1920x1080 mode gives 30.7, at 56 deg and 960x540 — full view was chosen). The old board scrambled that frame, so until then the cameras
  were cycled one at a time on a 16:9 crop (56 deg, 960x540, sides ~3 pictures a second),
  each picture was checked for scrambling, and the front camera alone was fallen back to:
  all removed. A camera that is unplugged or covered shows as black ("Camera sees only
  black" on `/health`). The detector still takes 640x640, so the wider view makes each
  thing ~18% smaller to it: the farthest small objects are named a little later. One
  auto-exposure serves every camera, so a bright view on one can darken the others.
  One lens setting serves every camera too (`CAMERA_FOCUS_CODE`, set on the lens driver at
  each start: libcamera's focus never reached it, and until 2026-11-02 every lens rested
  past infinity, blurred). 1000 was the sharpest on a ceiling ~2 m away, and should hold
  from about a metre out to far — an estimate, not yet checked on people or outdoors.
- **Traffic-light colour cannot be read.** The stock COCO detector emits only
  the class `traffic light` with no colour, and a bounding box cannot tell you
  which lamp is lit. Colour is reported as `unknown` rather than guessed. The
  light's PRESENCE is still used — a light the FRONT camera sees withholds
  "Walk straight" for 5 s after the last sighting (a light seen only behind or
  beside the user no longer does, since the 2026-09-29 audit) — and nothing
  ever says it is safe to cross. It used to announce "No traffic lights detected" while doing
  so, because an unreadable colour lands in the same branch as an empty
  intersection; those two cases are now distinguished.
- **A pane across a narrow passage, farther than its walls.** A pane spanning a whole
  passage, with no frame the laser sees, is found only once it is nearer than the passage's
  walls are to the side (less ~100 mm): in a 1.8 m passage from ~2.6 ft, in a 0.9 m one from
  ~1.2-1.5 ft. Farther than that, a side wall can echo from the pane's range too, and the
  radar's range alone cannot tell them apart; it says "Walk straight". A frame or a wall
  around a door the laser sees is found sooner (the door-in-a-wall rule). The left wall,
  half in the LiDAR's blind arc, is often not found as a side wall at all, so a pane is found
  later in a passage whose nearer wall is on the left. Telling them apart further would take
  the echo's motion: a pane's range closes at walking pace, a wall beside stays put
  (`tools/sim/glass_sim.py`'s "limit" passage scenes).
- **Close to a wall, the left is not offered.** Facing a flat wall nearer than ~1.7 ft with
  only the left open, it says "turn around": the left side radar hears the wall ahead at its
  beam's edge, and the LiDAR cannot see to the left (the wearer's body) to place that echo.
  The right is offered (design note 15).
- **The 2D LiDAR sees one plane.** Overhead branches and low obstacles at foot
  height are missed. The radar helps with glass, not with height. The ToF now covers
  the ground 1–4 m ahead up to ~1.1 m (stairs, kerbs, drops, low and waist-high things:
  [above](running.md#hazards-on-the-ground--stairs-steps-drops-poles-added-2026-11-06)) — not yet
  spoken (`TOF_HAZARDS_SPEAK`), and nothing at head height.
- **Heading is dead reckoning and unvalidated on real walks.** The scan matcher
  is accurate on synthetic corridors (0.02° per scan) but has not yet been
  checked against a measured walk. It degrades where a scan carries little
  structure — an open field, a bare corridor, a crowd moving with the user — so
  the drift warning is correspondingly weaker there. It degrades safely: an
  unobservable pace falls back to nominal rather than a fabricated figure, and
  the user is assumed to be walking rather than reported as standing still —
  which needed the sensitivity hurdle described under
  [Design notes](design-notes.md), because
  fit quality and standard error are both residual-scaled and a corridor fits
  perfectly while observing nothing.
- **A rotating LiDAR sweeps over ~67 ms**, during which the user moves, so each
  scan is slightly skewed. Under 10 cm at walking pace, and not corrected for.
- **Segmentation is front-only.** The other three sectors are segmented on
  demand for the debug view and cost extra NPU time.
- **The voice is espeak-ng, and only audio.** `app/guidance/pi_voice.py` speaks every
  committed sentence through the default PipeWire output (an earpiece when one
  is paired); with no earpiece the dashboard's screen-reader region carries it.
  There is no haptic output. Any other delivery layer must honour the same
  contract: act on `notify.cue_seq` advancing, never on the poll — a consumer
  that speaks `path.instruction` every poll talks over itself several times a
  second, which is the bug the gates above exist to prevent.
- **No fog or rain detection; low light is measured, not acted on.** The camera's labels
  are believed whatever the weather. A fog detector existed but called a normal lit office
  "fog" and a black dead camera the same, so it was switched off, and removed on 2026-11-10.
  A new one would need checking against a genuinely fogged scene and a clear one before it
  gates anything. Light is now measured (2026-11-10): each camera's lux, from the sensor's own
  reading, on `/camera/stats` and in every walk recording. Under `CAMERA_DIM_LUX` (45: the
  sensor runs out of exposure there at 15 pictures a second) `/health` warns "Camera dim", and
  `label_report.py` splits its scores into good light and dim. Nothing said changes yet: in
  dim light a detector mostly names less rather than naming wrongly, and the LiDAR and radar
  carry on, so whether the wearer should be told waits on a dim walk's labels. Under ~5 lux
  (`CAMERA_DARK_LEVEL`) the picture is black and "Camera off" is said, as before.
- **The traffic-light colour is never resolved**: the guidance only knows a light was
  seen ("Traffic light ahead", from `unified_navigation_system`) and says wait. The branches
  that acted on a colour were removed on 2026-10-01, and the unused crossing state machine
  and its `/crossing` endpoints on 2026-11-10. Anyone adding colour
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
