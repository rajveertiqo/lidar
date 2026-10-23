#!/usr/bin/env python3
"""Try a newly compiled segmentation model on the Pi BEFORE it replaces model/best.hef.

    venv/bin/python training/check_hef.py new/best.hef new/metadata.yaml  pictures/*.jpg

Stop the app first (the NPU is shared). Loads the model the way the app does, checks
its outputs are the layout the app decodes and that its class count matches
metadata.yaml, runs it on each picture (stretched to 640x640 like the device), prints
what share of each picture every class covers, and saves an overlay per picture in
check_hef_out/. Then, to install:

    cp model/best.hef model/best.hef.old && cp model/metadata.yaml model/metadata.yaml.old
    cp new/best.hef model/best.hef && cp new/metadata.yaml model/metadata.yaml
"""

import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


def main():
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    hef, meta_path, pictures = sys.argv[1], sys.argv[2], sys.argv[3:]

    from utils import seg_inference
    from utils.yoloseg_decode import BACKGROUND_ID

    meta = seg_inference.load_metadata(meta_path)
    names = meta["names"]
    if not names:
        sys.exit(f"{meta_path} lists no class names")
    # The engine reads model/metadata.yaml; point it at the new one instead.
    seg_inference.load_metadata = lambda path=None: meta
    engine = seg_inference.SegmentationEngine(hef)

    cls_channels = {engine._out_shapes[g["cls"]][-1] for _grid, g, _s in engine._scales}
    print(
        f"outputs: {len(engine._scales)} scales + prototypes {engine._out_shapes[engine._proto_name]}"
    )
    if cls_channels != {len(names)}:
        sys.exit(
            f"MISMATCH: the model has {sorted(cls_channels)} class channels but "
            f"{meta_path} names {len(names)} classes — wrong metadata.yaml for this model"
        )
    print(f"classes ({len(names)}): {', '.join(names[i] for i in sorted(names))}")
    print(
        f"walkable (drawn bright): {', '.join(names[i] for i in engine.walkable_ids) or 'none'}"
    )

    palette, _ = seg_inference._build_palette(names)
    out_dir = Path("check_hef_out")
    out_dir.mkdir(exist_ok=True)
    times = []
    for pic in pictures:
        img = Image.open(pic).convert("RGB")
        t0 = time.perf_counter()
        class_map, infer_ms = engine.infer_class_map(img)
        times.append((time.perf_counter() - t0) * 1000.0)
        total = class_map.size
        shares = {
            names[int(c)]: n / total
            for c, n in zip(*np.unique(class_map, return_counts=True))
            if int(c) != BACKGROUND_ID and int(c) in names
        }
        seen = ", ".join(
            f"{k} {v:.0%}" for k, v in sorted(shares.items(), key=lambda kv: -kv[1])
        )
        print(
            f"  {Path(pic).name}: {seen or 'nothing'}  ({infer_ms:.0f} ms on the NPU)"
        )
        base = np.asarray(img.resize(class_map.shape[::-1]), dtype=np.float32)
        colour = palette[class_map].astype(np.float32)
        mask = (class_map != BACKGROUND_ID)[..., None]
        blend = np.where(mask, 0.45 * base + 0.55 * colour, base).astype(np.uint8)
        side = np.concatenate(
            [np.asarray(img.resize(class_map.shape[::-1])), blend], axis=1
        )
        Image.fromarray(side).save(out_dir / f"{Path(pic).stem}-check.jpg", quality=85)
    engine.close()
    if times:
        print(
            f"\n{len(times)} pictures, {np.median(times):.0f} ms each (whole call); "
            f"overlays in {out_dir}/"
        )


if __name__ == "__main__":
    main()
