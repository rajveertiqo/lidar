#!/usr/bin/env python3
"""Is the BEV grid actually working?"""

import argparse
import json
import time
import urllib.error
import urllib.request

OK, WARN, FAIL, NA = "OK", "WARN", "FAIL", "--"

_MARK = {OK: "\033[0;32m  OK  \033[0m", WARN: "\033[1;33m WARN \033[0m",
         FAIL: "\033[0;31m FAIL \033[0m", NA: "\033[0;90m  --  \033[0m"}

MM_PER_FT = 304.8


def fetch(base: str, path: str):
    """(payload, error string). Never raises — a missing endpoint is a result."""
    try:
        with urllib.request.urlopen(base + path, timeout=5) as r:
            return json.loads(r.read().decode()), None
    except urllib.error.HTTPError as e:
        return None, f"HTTP {e.code}"
    except urllib.error.URLError as e:
        return None, f"unreachable ({e.reason})"
    except Exception as e:
        return None, f"{type(e).__name__}: {e}"


def collect(base: str):
    """Two BEV samples ~1 s apart, plus guidance and radar."""
    first, err = fetch(base, "/lidar/bev")
    if err:
        return None, None, None, None, err
    t0 = time.time()
    time.sleep(1.0)
    second, err2 = fetch(base, "/lidar/bev")
    dt = time.time() - t0
    guidance, _ = fetch(base, "/guidance/walk")
    radar, _ = fetch(base, "/radar/state")
    fused, _ = fetch(base, "/guidance/fused")
    return first, second or first, dt, (guidance, radar, fused), err2


def checks(a, b, dt, guidance, radar, fused=None):
    """[(status, name, detail)] — each one a distinct way to be quietly useless."""
    out = []

    if not a.get("available"):
        out.append((FAIL, "grid available",
                    a.get("error") or "BEV_ENABLED is False in config.py"))
        return out
    out.append((OK, "grid available", "module loaded, OpenCV present"))

    # ---- is the pipeline LIVE? ----
    fa, fb = a.get("frames") or 0, b.get("frames") or 0
    rate = (fb - fa) / dt if dt else 0.0
    if fb == 0:
        out.append((FAIL, "scans arriving",
                    "no scan has ever reached the grid — is the LiDAR on "
                    "/dev/ttyUSB0 and the scanner thread running?"))
    elif fb == fa:
        out.append((FAIL, "scans arriving",
                    f"frame count frozen at {fb} — the grid is serving a stale "
                    f"snapshot. Check the LiDAR analysis thread"))
    elif rate < 8.0:
        out.append((WARN, "scans arriving",
                    f"{rate:.1f} Hz — the A2M12 should feed ~15 Hz at 900 RPM. "
                    f"Accumulation is thinner than designed"))
    else:
        out.append((OK, "scans arriving", f"{rate:.1f} Hz, {fb} frames total"))

    if b.get("registered"):
        out.append((OK, "registration", "ego-motion applied, scans accumulating"))
    else:
        out.append((WARN, "registration",
                    f"resetting each frame ({b.get('reset_reason') or 'unknown'})"
                    f" — degraded to single-sweep. Safe, but no accumulation"))

    # ---- is extraction fresh and affordable? ----
    age = time.time() - (b.get("last_update") or 0)
    ms = b.get("compute_ms")
    if age > 3.0:
        out.append((FAIL, "extraction", f"last ran {age:.0f} s ago"))
    elif ms is None:
        out.append((WARN, "extraction", "no timing reported"))
    elif ms > 20.0:
        out.append((WARN, "extraction",
                    f"{ms:.1f} ms — shares a thread with the STOP decision's "
                    f"inputs; consider raising BEV_CELL_MM"))
    else:
        out.append((OK, "extraction", f"{ms:.1f} ms, {age:.1f} s ago, "
                                      f"{b.get('cells_occupied', 0)} cells occupied"))

    secs = ((radar or {}).get("sectors") or {})
    live = [d for d in ("front", "right", "back", "left")
            if (secs.get(d) or {}).get("online")]
    if not radar:
        out.append((WARN, "glass detection", "/radar/state unavailable"))
    elif not live:
        out.append((FAIL, "glass detection",
                    "NO radar direction online — glass is UNDETECTABLE. This is "
                    "the grid's main reason to exist; see start_lidar.sh's radar "
                    "check"))
    elif len(live) < 4:
        out.append((WARN, "glass detection",
                    f"only {'/'.join(live)} online — glass is undetectable on "
                    f"the other arcs"))
    else:
        out.append((OK, "glass detection", "all four directions live"))

    # ---- is it RESOLVING anything? ----
    objs = b.get("objects") or []
    kinds = {}
    for o in objs:
        kinds[o["kind"]] = kinds.get(o["kind"], 0) + 1
    if not objs:
        out.append((WARN, "shapes resolving",
                    "no clusters — point the rig at a wall 1-3 m away"))
    else:
        named = kinds.get("wall", 0) + kinds.get("pole", 0)
        detail = ", ".join(f"{n} {k}" for k, n in sorted(kinds.items()))
        out.append((OK if named else WARN, "shapes resolving",
                    detail + ("" if named else
                              "  (nothing resolved as wall/post — those are the "
                              "two that reach the hazard list)")))

    # ---- is any of it REACHING guidance? ----
    if not guidance:
        out.append((NA, "shape -> hazards", "/guidance/walk unavailable"))
        return out

    items = ((fused or {}).get("hazards")
             or (guidance.get("obstacles") or {}).get("items") or [])
    shaped = [i for i in items if i.get("shape")]
    promotable = kinds.get("wall", 0) + kinds.get("pole", 0)
    if not items:
        out.append((NA, "shape -> hazards", "no hazards in range to carry a shape"))
    elif shaped:
        out.append((OK, "shape -> hazards",
                    f"{len(shaped)} of {len(items)} hazards carry a shape "
                    f"({', '.join(sorted({i['shape'] for i in shaped}))})"))
    elif not promotable:
        out.append((NA, "shape -> hazards",
                    "nothing in view is wall- or post-shaped; only those two "
                    "are promoted"))
    else:
        out.append((WARN, "shape -> hazards",
                    f"{promotable} wall/post cluster(s) resolved but none of the "
                    f"{len(items)} hazards matched one — they sit outside "
                    f"BEV_SHAPE_MATCH_DEG/_MM of any sector minimum, or the "
                    f"sector did not cluster-confirm a return there"))

    return out


def render(a, b, dt, extra, base):
    guidance, radar, fused = extra or (None, None, None)
    lines = [f"BEV grid check — {base}", ""]

    rows = checks(a, b, dt, guidance, radar, fused)
    for status, name, detail in rows:
        lines.append(f"  [{_MARK[status]}] {name:<18} {detail}")

    lines += ["", "What it sees right now:"]
    objs = (b.get("objects") or [])[:6]
    if not objs:
        lines.append("  (nothing resolved)")
    for o in objs:
        tag = "  <- GLASS, radar only" if o.get("radar_confirmed") else ""
        lines.append(f"  {o['kind']:<9} {o['range_ft']:>5.1f} ft  "
                     f"{o['clock']:>2} o'clock  {o['length_mm'] / 1000:.1f} m long{tag}")
    for g in (b.get("gaps") or [])[:3]:
        lines.append(f"  {'opening':<9} {g['range_ft']:>5.1f} ft  "
                     f"{g['clock']:>2} o'clock  {g['width_ft']:.1f} ft wide")
    for w in (b.get("unknown_wedges") or [])[:3]:
        lines.append(f"  {'blind arc':<9} {'—':>5}     "
                     f"{w['centre_deg']:>3.0f}°       {w['width_deg']:.0f}° wide")

    lines += ["",
              "  Top-down picture:  " + base + "/lidar/bev/image?scale=2",
              "  Raw extraction:    " + base + "/lidar/bev",
              "  Plain sentences:   " + base + "/lidar/bev/summary"]
    return "\n".join(lines)


CHECKLIST = """
PHYSICAL CHECKS — what the numbers cannot tell you
==================================================
Run `python -m tools.bev_check --watch` in one terminal and keep
/lidar/bev/image?scale=2 open in a browser. Then do these one at a time.

Each is written as: what to do -> what proves it worked.

1. WALL.  Face a flat wall, 1-3 m, nothing between you and it.
   -> an object with kind "wall", length within ~20% of the wall's real width,
      range within ~15 cm of a tape measure.
   If it reports several "obstacle" clusters instead, the wall is fragmenting:
   the returns are too sparse at that range for the morphological close to
   bridge. Move closer and try again before changing anything.

2. POST.  Stand a bottle or chair leg ~1 m ahead with clear floor around it.
   -> kind "pole", reported as "post" in the hazard list. A 150 mm post is
      three cells wide, which is exactly BEV_MIN_CLUSTER_CELLS; anything
      thinner will legitimately not register.

3. DOORWAY.  Stand 2 m back from an open door, facing squarely through it,
   with visible wall on BOTH sides and open space beyond.
   -> a "gaps" entry, width within ~25 cm of the real door.
   Both flanking walls must resolve as kind "wall" first — check that before
   concluding the doorway search is broken. An alcove correctly gives nothing.

4. GLASS.  THE important one. Stand ~1 m from a glass door or window, square on.
   -> an object with radar_confirmed: true at roughly the real distance, and
      "GLASS" in this tool's output.
   -> /lidar/bev should show the LiDAR ranging something FAR at that bearing
      (the room beyond) while the radar says ~1 m. That disagreement IS the
      detection. If radar_confirmed stays false, check the radar board for that
      direction is online and has a measured bearing — glass detection is
      impossible without it, and the check above will say so.

5. BLIND ARC.  Hang black cloth, or face a mirror, filling ~40 deg of view.
   -> an "unknown_wedges" entry there. Critically, the picture must show that
      wedge as GREY (unknown), never dark (free). Grey is the whole safety
      property: a surface that returns nothing is not clear ground.

6. REGISTRATION.  Turn ~90 deg on the spot, slowly, over about 3 seconds.
   -> in the picture, walls stay THIN while you turn. A fan or smear spreading
      out from you means the ego-motion is wrong.
   -> this tool's "registration" line must stay OK. If it flips to resetting,
      read the reason: a low yaw quality in a bare corridor is expected and
      safe, a persistent one anywhere is not.

7. THE ADVISORY BOUNDARY.  While any of the above is in view, watch
   /guidance/walk.
   -> path.action must NEVER become a stop because of something only the grid
      can see. The grid may add "Caution: glass ... 12 o'clock" and may rename
      an obstacle to "wall". It may not halt anyone. If it ever does, that is
      a real bug — the whole layer is built on not having that authority.
"""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--watch", action="store_true", help="redraw until Ctrl+C")
    ap.add_argument("--interval", type=float, default=2.0)
    ap.add_argument("--checklist", action="store_true",
                    help="print the physical tests and exit")
    args = ap.parse_args()

    if args.checklist:
        print(CHECKLIST)
        return 0

    base = f"http://{args.host}:{args.port}"
    try:
        while True:
            a, b, dt, extra, err = collect(base)
            if a is None:
                print(f"BEV grid check — {base}\n\n  [{_MARK[FAIL]}] endpoint"
                      f"          {err}\n\n  Is the server running? "
                      f"./start_lidar.sh")
                if not args.watch:
                    return 1
            else:
                out = render(a, b, dt, extra, base)
                print("\033[2J\033[H" + out if args.watch else out)
            if not args.watch:
                bad = any(s == FAIL for s, _, _ in
                          checks(a, b, dt, *(extra or (None, None, None))))
                return 1 if bad else 0
            time.sleep(max(0.5, args.interval - 1.0))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
