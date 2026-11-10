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
