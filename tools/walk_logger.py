#!/usr/bin/env python3
"""Walk logger — records the live guidance + system metrics during a walking test."""

import csv
import os
import queue
import subprocess
import threading
import time
from collections import Counter

import requests

BASE = "http://127.0.0.1:8000"
INTERVAL = 0.5
LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")

# 8 x 45° LiDAR sectors (threat detection resolution).
SECTORS = ("front", "front_right", "right", "back_right",
           "back", "back_left", "left", "front_left")

# The radar and the walk policy both work in the 4 cardinals only.
CARDINALS = ("front", "right", "back", "left")

FIELDS = ([
    "iso_time", "elapsed_s",
    "action", "cue", "source", "rule", "obstacle", "crossing",
    # why the policy said it
    "blocked", "turn_dir", "glass", "reason",
    # was it actually SAID (and what, exactly), and if not, why not
    "spoken", "said", "cue_seq", "quiet",
] + [f"clear_{c}_ft" for c in CARDINALS]
  + [f"radar_{c}_ft" for c in CARDINALS]
  + ["radar_online",
     "capture_fps", "detection_fps", "seg_fps", "inference_ms", "objects"]
  + [f"{s}_ft" for s in SECTORS]
  + ["temp_c", "throttled", "marker"])


def _vcgencmd(arg):
    try:
        out = subprocess.check_output(["vcgencmd", arg], timeout=2).decode()
        return out.strip().split("=", 1)[-1]
    except Exception:
        return ""


def _marker_reader(q):
    """Background: each ENTER (optionally preceded by typed text) drops a marker."""
    while True:
        try:
            note = input()
        except (EOFError, KeyboardInterrupt):
            return
        q.put(note.strip() or "mark")


def _round(v, n=2):
    """Round only real numbers; leave None/missing as blank rather than 0."""
    return round(v, n) if isinstance(v, (int, float)) else ""


def _radar_ft(sectors, direction):
    """Radar distance for one direction, in FEET ('' if absent/offline)."""
    entry = sectors.get(direction) or {}
    mm = entry.get("min_dist")
    return _round(mm / 304.8) if isinstance(mm, (int, float)) else ""


def _radar_online(sectors):
    """How many of the four CARDINALS are live."""
    return sum(1 for c in CARDINALS if (sectors.get(c) or {}).get("online"))


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(LOG_DIR, f"walk_{stamp}.csv")

    marker_q = queue.Queue()
    threading.Thread(target=_marker_reader, args=(marker_q,), daemon=True).start()

    print(f"[walk_logger] writing {path}")
    print("[walk_logger] ENTER = drop a marker (type a note first if you like), Ctrl+C = stop\n")

    fps_cap, fps_det, fps_seg, temps = [], [], [], []
    actions = Counter()
    announcements = 0    # times notify.cue_seq advanced — i.e. things SAID
    last_cue_seq = None
    glass_hits = 0       # polls where the policy flagged glass ahead
    blocked_hits = 0     # polls where the way ahead was blocked
    radar_degraded = 0   # polls with fewer than all 4 radar boards online
    freezes = 0          # polls that failed/timed out (app unresponsive)
    rows = 0
    start = time.time()

    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        try:
            while True:
                t0 = time.time()
                marker = ""
                while not marker_q.empty():
                    marker = marker_q.get_nowait()

                row = {k: "" for k in FIELDS}
                row["iso_time"] = time.strftime("%H:%M:%S")
                row["elapsed_s"] = round(t0 - start, 1)
                row["marker"] = marker
                try:
                    g = requests.get(f"{BASE}/guidance/walk", timeout=2).json()
                    s = requests.get(f"{BASE}/camera/stats", timeout=2).json()
                    try:
                        r = requests.get(f"{BASE}/radar/state", timeout=2).json()
                    except Exception:
                        r = {}

                    sec = g.get("sector_ft", {}) or {}
                    p = g.get("path", {}) or {}
                    walk = g.get("walk", {}) or {}
                    ntf = g.get("notify", {}) or {}
                    clear = p.get("clearance_ft", {}) or {}
                    rsec = r.get("sectors", {}) or {}
                    online = _radar_online(rsec)

                    seq = ntf.get("cue_seq")
                    said_now = 0
                    if isinstance(seq, int):
                        if last_cue_seq is not None and seq > last_cue_seq:
                            said_now = seq - last_cue_seq
                            announcements += said_now
                        last_cue_seq = seq

                    row.update({
                        "action": p.get("action", ""),
                        "cue": p.get("instruction", ""),
                        "source": p.get("source", ""),
                        "rule": p.get("rule", ""),
                        "obstacle": p.get("obstacle") or "",
                        "crossing": g.get("crossing", ""),
                        "blocked": walk.get("blocked", ""),
                        "turn_dir": walk.get("direction") or "",
                        "glass": p.get("glass_ahead", ""),
                        "reason": p.get("reason", ""),
                        "spoken": said_now,
                        "said": ntf.get("instruction", "") if said_now else "",
                        "cue_seq": ntf.get("cue_seq", ""),
                        "quiet": ntf.get("quiet_reason", ""),
                        "radar_online": f"{online}/{len(CARDINALS)}",
                        "capture_fps": _round(s.get("capture_fps"), 1),
                        "detection_fps": _round(s.get("detection_fps")),
                        "seg_fps": _round(s.get("seg_fps")),
                        "inference_ms": _round(s.get("inference_time"), 1),
                        "objects": s.get("total_objects", 0),
                        **{f"clear_{c}_ft": _round(clear.get(c)) for c in CARDINALS},
                        **{f"radar_{c}_ft": _radar_ft(rsec, c) for c in CARDINALS},
                        **{f"{k}_ft": sec.get(k) for k in SECTORS},
                    })

                    if isinstance(s.get("capture_fps"), (int, float)):
                        fps_cap.append(s["capture_fps"])
                    if isinstance(s.get("detection_fps"), (int, float)):
                        fps_det.append(s["detection_fps"])
                    if isinstance(s.get("seg_fps"), (int, float)):
                        fps_seg.append(s["seg_fps"])
                    if p.get("action"):
                        actions[p["action"]] += 1
                    if p.get("glass_ahead"):
                        glass_hits += 1
                    if walk.get("blocked"):
                        blocked_hits += 1
                    if rsec and online < len(CARDINALS):
                        radar_degraded += 1
                except Exception as e:
                    freezes += 1
                    row["cue"] = f"[poll failed: {e}]"

                row["temp_c"] = _vcgencmd("measure_temp").replace("'C", "")
                row["throttled"] = _vcgencmd("get_throttled")
                try:
                    temps.append(float(row["temp_c"]))
                except ValueError:
                    pass

                w.writerow(row); f.flush(); rows += 1
                if marker:
                    print(f"  ⏱ marker @ {row['elapsed_s']}s: {marker}")

                dt = INTERVAL - (time.time() - t0)
                if dt > 0:
                    time.sleep(dt)
        except KeyboardInterrupt:
            pass

    # ---- summary ----
    def mn(xs): return round(min(xs), 1) if xs else 0
    def av(xs): return round(sum(xs) / len(xs), 1) if xs else 0
    def pct(n): return f"{round(100 * n / rows)}%" if rows else "—"

    print("\n===== WALK SUMMARY =====")
    print(f"duration: {round(time.time()-start)}s   rows: {rows}   log: {path}")
    print(f"capture_fps   min/avg: {mn(fps_cap)} / {av(fps_cap)}")
    print(f"detection_fps min/avg: {mn(fps_det)} / {av(fps_det)}")
    print(f"seg_fps       min/avg: {mn(fps_seg)} / {av(fps_seg)}")
    print(f"temp max: {max(temps) if temps else '?'}°C   poll failures (freezes): {freezes}")
    print(f"blocked: {blocked_hits} ({pct(blocked_hits)})   "
          f"glass flagged: {glass_hits} ({pct(glass_hits)})")
    print(f"radar degraded (<4 boards online): {radar_degraded} ({pct(radar_degraded)})")
    print(f"guidance actions: {dict(actions)}")
    secs = max(1e-6, time.time() - start)
    print(f"announcements: {announcements} in {round(secs)}s"
          + (f"  (one every {round(secs / announcements)}s)" if announcements else "")
          + "   <- if this is anywhere near `rows`, the guide is nagging again")


if __name__ == "__main__":
    main()
