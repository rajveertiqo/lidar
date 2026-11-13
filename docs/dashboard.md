# Dashboard

`http://<pi-ip>:8000` is **the field-test dashboard** (`templates/dashboard.html`; redrawn
2026-11-11, same data and endpoints as before). Top to bottom:

| Panel | What it shows |
| --- | --- |
| Header | Live / no connection / guidance stopped (one verdict, worst first), which voice is speaking, sentences a minute, the clock; **Log** downloads this run's speech log, **Depth** opens `/tof/live`, the last button switches auto / dark / light |
| **Last said** | The sentence the device last **said**, and nothing else, coloured by its rule (red stops the wearer, amber warns, green is clear), with the front distance in the corner. The sentence the guide is still weighing, why it is quiet, and the live readings behind it are in the terminal |
| **Was that right?** | Right / Wrong / Missed / Note, stamped into the speech log (keys 1-4). On a phone the four buttons are pinned to the bottom of the screen, and what a press saved shows above them |
| **What was really there?** | Wall, Glass, Obstacle, Person, Nothing, Stairs, Kerb / step (keys W G O P N S K): see [Field testing](field-testing.md#labelling-what-was-really-there-added-2026-11-10) |
| **What the laser and radar see** | The scan drawn with the same corridor and rings the guide decides on; under it the radar circle (each board's beam filled out to its nearest echo, its other echoes thin, its 13 ft reach dotted, a board off in red; the *radar* switch hides it), and each direction's distance (LiDAR, radar, glass) |
| **What the cameras see** | Front, Left, Right, Back or All 4, with the detector's boxes and the band where objects get named; "How to read the front camera" unfolds the key |
| **Spoken this run** | Said in the last minute, said this run, your marks; then only what was actually said, as the terminal's spoken lines: the reason, the LiDAR scan's age, the Pi or the phone. The silent "would say" lines and the marks are in the terminal and `/guidance/log/text` |
| **Device** | LiDAR, radar, camera, NPU, start-up and voice, each radar board, `/health`'s warnings, and "Also speak in this browser" |
| **Around you** | The nearest things and their clock direction; walking speed and whether a crossing is near |

The speech log is kept in memory; it is written to `logs/speech_*.log` only with
`SPEECH_LOG_TO_FILE = True` in `app/config.py`. The page is also the voice when no earpiece is
connected: one aria-live region for the phone's screen reader, silent while the Pi speaks. The
Pi's own voice (`app/guidance/pi_voice.py`, espeak-ng) is the production delivery path. The
page is read from disk on every request: an edit shows on a browser refresh, no restart.

## The other pages

| Page | What it is for |
| --- | --- |
| `/tof/live` | The ToF depth camera live: tap to read a distance; the wall check and whether the rig is set down on a surface |
| `/radar/test` | The radar boards one by one, and the CALIBRATE panel: hold a hand in front of each board to set its bearing |
| `/radar/view` | The radar scope: each board's wedge and echoes, as the array physically is |
| `/guidance/log/text` | Everything said this run, as plain text |
| `/docs` | The API, every endpoint and its schema |

The previous dashboard (`/classic`, `templates/index.html`) was removed on 2026-11-10:
everything used in testing is on the dashboard above. It is in `git log` before then.
