# Dashboard

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

## Radar Scope (added 2026-09-04)

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

## Motion (added 2026-09-04)

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
