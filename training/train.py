#!/usr/bin/env python3
"""Train the segmentation model (runs on the Windows laptop, or Colab).

    python training/train.py --data dataset/data.yaml
    python training/train.py --data dataset/data.yaml --model yolov8s-seg.pt --epochs 200

Uses the GPU if PyTorch can see one (NVIDIA + CUDA build of PyTorch), else the CPU —
which works but takes days, not hours. At the end it prints how well each class is
found on the validation (and test) images and writes metadata.yaml beside best.pt:
the app reads the class names from it, and it must travel with the compiled model.
"""

import argparse
import sys
from pathlib import Path

import yaml


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--data", default="dataset/data.yaml")
    # YOLOv8-seg is the family Hailo's compiler handles most reliably; the model now on
    # the device is YOLO11-seg, which also compiled. "s" = small: fast enough to share
    # the NPU with object detection.
    ap.add_argument("--model", default="yolov8s-seg.pt")
    ap.add_argument("--epochs", type=int, default=150)
    ap.add_argument(
        "--batch", type=int, default=-1, help="-1 = as large as the GPU allows"
    )
    ap.add_argument("--name", default="shappe-seg")
    ap.add_argument(
        "--workers",
        type=int,
        default=4,
        help="data loader processes (lower it if Windows runs out of memory)",
    )
    args = ap.parse_args()

    import torch
    from ultralytics import YOLO

    data_yaml = Path(args.data).resolve()
    spec = yaml.safe_load(data_yaml.read_text(encoding="utf-8")) or {}
    if not Path(str(spec.get("path") or "")).is_dir():
        # The dataset was moved (another PC, Colab): it lives where its data.yaml is.
        spec["path"] = str(data_yaml.parent)
        data_yaml = data_yaml.with_name("data.local.yaml")
        data_yaml.write_text(yaml.safe_dump(spec, sort_keys=False), encoding="utf-8")
        print(f"dataset moved: using {data_yaml} (path: {spec['path']})")
    args.data = str(data_yaml)
    names = spec.get("names")
    names = list(names.values()) if isinstance(names, dict) else list(names or [])
    if len(names) in (4, 32, 64):
        sys.exit(
            f"{len(names)} classes cannot be used on the device (see prepare_dataset.py)"
        )
    device = 0 if torch.cuda.is_available() else "cpu"
    print(f"classes: {names}")
    print(
        f"training on: {torch.cuda.get_device_name(0) if device == 0 else 'CPU (slow!)'}"
    )

    model = YOLO(args.model)
    model.train(
        data=args.data,
        imgsz=640,  # the device runs the model at 640x640
        epochs=args.epochs,
        batch=args.batch,
        patience=40,  # stop early once validation stops improving
        device=device,
        workers=args.workers,
        name=args.name,
        exist_ok=False,
        # Pictures are already stretched 16:9 -> square like the device's (prepare_dataset).
        # Left/right flips are fine for these classes; up/down flips are not (the ground
        # is always at the bottom).
        fliplr=0.5,
        flipud=0.0,
        degrees=5.0,  # the harness tilts a little while walking
        hsv_v=0.5,  # strong light changes: shade, sun, street lights
    )

    best = Path(model.trainer.save_dir) / "weights" / "best.pt"
    print(f"\nbest weights: {best}")
    trained = YOLO(str(best))
    splits = ["val"]
    root = Path(spec["path"])
    test_dir = root / str(spec.get("test") or "images/test")
    if test_dir.is_dir() and any(test_dir.iterdir()):
        splits.append("test")
    for split in splits:
        m = trained.val(
            data=args.data, split=split, imgsz=640, device=device, plots=False
        )
        print(
            f"\n{split}: how often each class's shape is found (mask mAP50, 1.0 = always)"
        )
        for i, ci in enumerate(m.seg.ap_class_index):
            print(f"  {names[int(ci)]:<12} {m.seg.ap50[i]:.2f}")

    meta = best.parent / "metadata.yaml"
    meta.write_text(
        yaml.safe_dump(
            {"nc": len(names), "names": dict(enumerate(names))}, sort_keys=False
        )
    )
    print(f"\nwrote {meta} — copy it to the Pi together with the compiled best.hef")
    print(f"next: python training/export_onnx.py {best}")


if __name__ == "__main__":
    main()
