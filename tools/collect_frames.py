#!/usr/bin/env python3
"""Save camera pictures from the RUNNING app, to build the segmentation training set.

    venv/bin/python tools/collect_frames.py                       # front camera, every 2 s
    venv/bin/python tools/collect_frames.py --sectors front,left,right --every 1

Asks the app (/camera/frame/{sector}) — it never opens a camera itself, so it cannot
upset the camera board. Skips pictures that are simulated, black, scrambled (the board's
4-in-1 fault) or nearly the same as the last one kept from that camera (standing still),
so a walk gives varied pictures. They go to logs/training_frames/<date>/ at the camera's
own 960x540; copy that folder to the laptop and upload it to Roboflow for labelling.
Stop with Ctrl+C.
"""

import argparse
import base64
import io
import sys
import time
from pathlib import Path

import numpy as np
import requests
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from config import CAMERA_CORRUPT_PCT, CAMERA_DARK_LEVEL  # noqa: E402
from utils.pi_camera_utils import _corrupt_pct  # noqa: E402


def _thumb(img: Image.Image) -> np.ndarray:
    return np.asarray(img.convert("L").resize((64, 36)), dtype=np.float32)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sectors", default="front", help="comma list: front,right,left,back")
    ap.add_argument("--every", type=float, default=2.0, help="seconds between pictures")
    ap.add_argument("--change", type=float, default=8.0, help="how different (mean grey "
                    "level, 0-255) a picture must be from the last one kept")
    ap.add_argument("--base", default="http://127.0.0.1:8000")
    ap.add_argument("--out", default=str(ROOT / "logs" / "training_frames"))
    args = ap.parse_args()

    sectors = [s.strip() for s in args.sectors.split(",") if s.strip()]
    out = Path(args.out) / time.strftime("%Y%m%d")
    out.mkdir(parents=True, exist_ok=True)
    last = {}
    kept = skipped = 0
    why = {"same": 0, "dark": 0, "scrambled": 0, "simulated": 0, "no picture": 0}
    print(f"saving to {out} — Ctrl+C to stop")
    try:
        while True:
            t0 = time.monotonic()
            for s in sectors:
                try:
                    r = requests.get(f"{args.base}/camera/frame/{s}", timeout=3).json()
                except Exception as e:
                    print(f"  app not answering ({type(e).__name__}) — is it running?")
                    why["no picture"] += 1
                    continue
                if not r.get("frame"):
                    why["no picture"] += 1
                    continue
                if r.get("simulated"):
                    why["simulated"] += 1
                    continue
                raw = base64.b64decode(r["frame"])
                img = Image.open(io.BytesIO(raw)).convert("RGB")
                arr = np.asarray(img)
                if float(arr[::4, ::4].mean()) < CAMERA_DARK_LEVEL:
                    why["dark"] += 1
                    continue
                if _corrupt_pct(arr) > CAMERA_CORRUPT_PCT:
                    why["scrambled"] += 1
                    continue
                th = _thumb(img)
                if s in last and float(np.abs(th - last[s]).mean()) < args.change:
                    why["same"] += 1
                    continue
                last[s] = th
                name = f"{s}-{time.strftime('%H%M%S')}-{kept:05d}.jpg"
                (out / name).write_bytes(raw)
                kept += 1
            skipped = sum(why.values())
            print(f"\r  kept {kept}  skipped {skipped} "
                  f"({', '.join(f'{k} {v}' for k, v in why.items() if v)})   ",
                  end="", flush=True)
            time.sleep(max(0.0, args.every - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    print(f"\n{kept} pictures in {out}")


if __name__ == "__main__":
    main()
