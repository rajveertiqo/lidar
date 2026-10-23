#!/usr/bin/env python3
"""Live NPU utilization/FPS for the running server — what `hailortcli monitor` would have shown if the Hailo-10 runtime populated /tmp/hmon_files (it does not; see utils/hailo_stats.py for the evidence)."""

import argparse
import json
import sys
import time
import urllib.error
import urllib.request


def fetch(url: str):
    with urllib.request.urlopen(url, timeout=3) as r:
        return json.loads(r.read().decode())


def render(snap: dict) -> str:
    lines = [
        f"{'Model':<28}{'Utilization (%)':>17}{'FPS':>9}{'Avg ms':>9}{'Results':>9}",
        "-" * 72,
    ]
    models = snap.get("models", {})
    for name, s in sorted(models.items()):
        lines.append(
            f"{name:<28}{s['utilization']:>17}{s['fps']:>9}"
            f"{s['avg_infer_ms']:>9}{s['last_results']:>9}"
        )
    if not models:
        lines.append("(no inference recorded yet — is the camera thread running?)")
    lines.append("-" * 72)
    lines.append(
        f"{'NPU TOTAL':<28}{snap.get('total_utilization', 0.0):>17}"
        f"   (rolling {snap.get('window_sec', 0):.0f}s window)"
    )
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description="Live Hailo NPU utilization/FPS.")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--once", action="store_true", help="print one sample and exit")
    args = ap.parse_args()

    url = f"http://{args.host}:{args.port}/camera/hailo"

    try:
        while True:
            try:
                body = render(fetch(url))
            except urllib.error.URLError as e:
                body = f"server unreachable at {url}\n  {e.reason}"
            except Exception as e:  # noqa: BLE001 - display, don't die
                body = f"error reading {url}\n  {type(e).__name__}: {e}"

            if args.once:
                print(body)
                return 0
            # Clear screen + home, same as hailortcli monitor's redraw.
            sys.stdout.write("\033[2J\033[H" + body + "\n")
            sys.stdout.flush()
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print()
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
