#!/usr/bin/env python3
"""Merge YOLO segmentation exports into ONE dataset shaped like the device's pictures.

Each source is a folder exported from Roboflow (or CVAT / Label Studio) in "YOLOv8" /
"YOLO11" segmentation format: a data.yaml with the class names, and train/ valid/ test/
folders each holding images/ and labels/. Public datasets name things their own way
("warning tactile", "sidewalk", "zebra crossing"), so every class name is mapped to
ours through ALIASES; anything unmapped is dropped and reported.

The device (utils/seg_inference.py) STRETCHES each 960x540 camera picture to 640x640 —
no letterbox. So every image here is centre-cropped to 16:9 (polygons clipped to the
crop) and then stretched to 640x640 the same way. Training on the same distortion the
model will see on the device matters more than any augmentation.

    python training/prepare_dataset.py --out dataset  exports/my-walks  exports/tactile-public
"""

import argparse
import hashlib
import sys
from collections import Counter
from pathlib import Path

import yaml
from PIL import Image, ImageOps

# The model's classes, in order. The app finds them BY NAME (utils/walkable_guidance.py),
# so keep these spellings. Never exactly 4, 32 or 64 classes: the app tells the model's
# output layers apart by their channel count (utils/seg_inference.py).
CLASSES = ["person", "path", "road", "crosswalk", "tactile", "stairs"]

# Other datasets' names -> ours (lower case; "-" and "_" read as spaces).
ALIASES = {
    "person": "person", "people": "person", "pedestrian": "person", "human": "person",
    "path": "path", "sidewalk": "path", "pavement": "path", "footpath": "path",
    "walkway": "path", "footway": "path", "floor": "path", "corridor": "path",
    "hallway": "path",
    "road": "road", "street": "road", "carriageway": "road", "lane": "road",
    "crosswalk": "crosswalk", "zebra": "crosswalk", "zebra crossing": "crosswalk",
    "pedestrian crossing": "crosswalk", "crossing": "crosswalk",
    "tactile": "tactile", "tactile paving": "tactile", "tactile pavement": "tactile",
    "tactile path": "tactile", "tactile tile": "tactile", "warning tactile": "tactile",
    "warning": "tactile", "directional": "tactile", "horizontal directional": "tactile",
    "vertical directional": "tactile", "truncated dome": "tactile",
    "truncated domes": "tactile", "blister": "tactile", "blister paving": "tactile",
    "stairs": "stairs", "stair": "stairs", "staircase": "stairs", "steps": "stairs",
    "step": "stairs",
}

ASPECT = 16.0 / 9.0          # the device's camera picture, 960x540
SIZE = 640                   # the model's input, stretched to a square on the device
MIN_AREA = 0.0002            # polygons smaller than this share of the picture are dropped
SPLITS = {"train": "train", "valid": "val", "val": "val", "test": "test"}
IMAGE_EXT = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def _norm(name) -> str:
    return " ".join(str(name).lower().replace("-", " ").replace("_", " ").split())


def _names(src: Path) -> dict:
    """{source class id: our class id or None} from the source's data.yaml."""
    meta = yaml.safe_load((src / "data.yaml").read_text(encoding="utf-8")) or {}
    names = meta.get("names") or {}
    if isinstance(names, list):
        names = dict(enumerate(names))
    out = {}
    for i, n in names.items():
        ours = ALIASES.get(_norm(n))
        out[int(i)] = CLASSES.index(ours) if ours in CLASSES else None
    return out, {int(i): str(n) for i, n in names.items()}


def _clip(poly, x0, y0, x1, y1):
    """Sutherland-Hodgman: the part of polygon `poly` inside the rectangle."""
    def cut(points, inside, cross):
        out = []
        for i, cur in enumerate(points):
            prev = points[i - 1]
            if inside(cur):
                if not inside(prev):
                    out.append(cross(prev, cur))
                out.append(cur)
            elif inside(prev):
                out.append(cross(prev, cur))
        return out

    def at_x(x):
        return lambda p, q: (x, p[1] + (q[1] - p[1]) * (x - p[0]) / (q[0] - p[0]))

    def at_y(y):
        return lambda p, q: (p[0] + (q[0] - p[0]) * (y - p[1]) / (q[1] - p[1]), y)

    for inside, cross in ((lambda p: p[0] >= x0, at_x(x0)), (lambda p: p[0] <= x1, at_x(x1)),
                          (lambda p: p[1] >= y0, at_y(y0)), (lambda p: p[1] <= y1, at_y(y1))):
        poly = cut(poly, inside, cross)
        if not poly:
            break
    return poly


def _area(poly) -> float:
    return abs(sum(poly[i - 1][0] * p[1] - p[0] * poly[i - 1][1]
                   for i, p in enumerate(poly))) / 2.0


def _split_for(name: str, split: str, has_val: bool) -> str:
    """A source without a validation split gives 10% of its training images to val."""
    if split == "train" and not has_val:
        if int(hashlib.md5(name.encode()).hexdigest(), 16) % 10 == 0:
            return "val"
    return split


def convert(sources, out: Path, quality: int = 92) -> int:
    if len(CLASSES) in (4, 32, 64):
        sys.exit(f"{len(CLASSES)} classes: the app cannot tell such a class head from a "
                 f"box / mask head (utils/seg_inference.py) — add or merge a class")
    for split in ("train", "val", "test"):
        (out / "images" / split).mkdir(parents=True, exist_ok=True)
        (out / "labels" / split).mkdir(parents=True, exist_ok=True)

    counts = {s: Counter() for s in ("train", "val", "test")}
    images = Counter()
    dropped_names, skipped_box, warnings = Counter(), 0, 0
    for src in sources:
        src = Path(src)
        mapping, names = _names(src)
        for i, ours in mapping.items():
            if ours is None:
                dropped_names[f"{src.name}: {names[i]}"] += 0
        tag = "".join(c if c.isalnum() else "-" for c in src.name)[:24]
        has_val = any((src / d / "images").is_dir() for d in ("valid", "val"))
        for folder, split in SPLITS.items():
            img_dir = src / folder / "images"
            if not img_dir.is_dir():
                continue
            for img_path in sorted(img_dir.iterdir()):
                if img_path.suffix.lower() not in IMAGE_EXT:
                    continue
                label_path = src / folder / "labels" / (img_path.stem + ".txt")
                target = _split_for(img_path.name, split, has_val)
                try:
                    img = ImageOps.exif_transpose(Image.open(img_path)).convert("RGB")
                except Exception as e:
                    print(f"  unreadable {img_path}: {e}")
                    warnings += 1
                    continue
                w, h = img.size
                # centre crop to 16:9, in normalised coordinates
                if w / h > ASPECT * 1.02:
                    cw = h * ASPECT / w
                    box = ((1 - cw) / 2, 0.0, (1 + cw) / 2, 1.0)
                elif w / h < ASPECT / 1.02:
                    ch = w / ASPECT / h
                    box = (0.0, (1 - ch) / 2, 1.0, (1 + ch) / 2)
                else:
                    box = (0.0, 0.0, 1.0, 1.0)
                x0, y0, x1, y1 = box
                lines = []
                if label_path.exists():
                    for raw in label_path.read_text().splitlines():
                        vals = raw.split()
                        if not vals:
                            continue
                        ours = mapping.get(int(float(vals[0])))
                        if ours is None:
                            dropped_names[f"{src.name}: {names.get(int(float(vals[0])), vals[0])}"] += 1
                            continue
                        xy = [float(v) for v in vals[1:]]
                        if len(xy) == 4:          # a box, not a polygon: no mask to learn
                            skipped_box += 1
                            continue
                        poly = _clip(list(zip(xy[0::2], xy[1::2])), x0, y0, x1, y1)
                        poly = [((px - x0) / (x1 - x0), (py - y0) / (y1 - y0)) for px, py in poly]
                        if len(poly) < 3 or _area(poly) < MIN_AREA:
                            continue
                        lines.append(f"{ours} " + " ".join(f"{min(max(v, 0.0), 1.0):.6f}"
                                                            for p in poly for v in p))
                        counts[target][CLASSES[ours]] += 1
                crop = img.crop((round(x0 * w), round(y0 * h), round(x1 * w), round(y1 * h)))
                name = f"{tag}-{img_path.stem}"
                crop.resize((SIZE, SIZE), Image.BILINEAR).save(
                    out / "images" / target / f"{name}.jpg", quality=quality)
                (out / "labels" / target / f"{name}.txt").write_text(
                    "\n".join(lines) + ("\n" if lines else ""))
                images[target] += 1

    (out / "data.yaml").write_text(yaml.safe_dump({
        "path": str(out.resolve()), "train": "images/train", "val": "images/val",
        "test": "images/test", "names": dict(enumerate(CLASSES))}, sort_keys=False))

    print(f"\nwrote {out / 'data.yaml'}")
    print(f"images: train {images['train']}, val {images['val']}, test {images['test']}")
    print(f"{'class':<12}{'train':>8}{'val':>8}{'test':>8}")
    for c in CLASSES:
        n = counts["train"][c]
        flag = "   <- fewer than 300 shapes to learn from" if n < 300 else ""
        print(f"{c:<12}{n:>8}{counts['val'][c]:>8}{counts['test'][c]:>8}{flag}")
    if dropped_names:
        print("\nclass names not mapped to ours (add them to ALIASES if they should count):")
        for k, n in sorted(dropped_names.items()):
            print(f"  {k}  ({n} shapes dropped)")
    if skipped_box:
        print(f"\n{skipped_box} box-only labels skipped (export as SEGMENTATION, not detection)")
    if not images["val"]:
        print("\nWARNING: no validation images — training cannot judge itself")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sources", nargs="+", help="exported dataset folders (each with data.yaml)")
    ap.add_argument("--out", default="dataset", help="output folder (default: dataset)")
    args = ap.parse_args()
    return convert(args.sources, Path(args.out))


if __name__ == "__main__":
    sys.exit(main())
