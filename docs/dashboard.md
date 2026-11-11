# Dashboard

`http://<pi-ip>:8000` is **the field-test dashboard** (`templates/dashboard.html`,
2026-09-24): what the device last **said** and why (only spoken sentences: the sentence the
guide is still weighing, and why it is quiet, are in the terminal, since 2026-11-11), four verdict buttons that stamp
✓ / ✗ / ⚠ / ✎ into the speech log (keys 1-4), the laser view drawn with the
same corridor and rings the guide decides on, the front camera with the band
where objects get named, **Spoken this run** (only what was actually said, as the terminal's
🔊 lines: the reason, the LiDAR scan's age, the Pi or the phone; the silent "would say" lines
and the marks are in the terminal and `/guidance/log/text`), the speech log (kept in memory;
written to `logs/speech_*.log` only with `SPEECH_LOG_TO_FILE = True` in `app/config.py`), and
per-board sensor health. It is
also the voice when no earpiece is connected: one aria-live region for the
phone's screen reader, silent while the Pi speaks. The Pi's own voice
(`app/guidance/pi_voice.py`, espeak-ng) is the production delivery path.

The verdict card's second row, **What was really there?** (keys W G O P N), labels what the
device just named: see [Field testing](field-testing.md#labelling-what-was-really-there-added-2026-11-10).

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
