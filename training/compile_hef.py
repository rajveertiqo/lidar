#!/usr/bin/env python3
"""Compile the ONNX model into a .hef for the Pi's Hailo-10H.

Runs ONLY in Linux x86 — on the Windows laptop that means WSL2 Ubuntu — inside a Python
environment with Hailo's Dataflow Compiler (hailo_sdk_client) installed. The compiler
release must match the Pi's HailoRT (5.1.x on 2026-10-09), or the Pi will not load it.

    python3 training/compile_hef.py best.onnx --calib dataset/images/train

Steps: parse the ONNX up to the head's raw outputs (end_nodes.json from export_onnx.py),
quantise to 8 bits using --calib pictures, compile, write best.hef. The app feeds the
model RGB 0-255 pictures stretched to 640x640, so the /255 scaling is put INSIDE the
.hef here, and the calibration pictures are prepared the same way. Without an NVIDIA
GPU (always the case in WSL) the compiler uses its basic quantisation; it says so.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

PI_HAILORT = "5.1"  # `pip list | grep hailort` on the Pi
IMAGE_EXT = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def calibration_set(folder: Path, count: int) -> np.ndarray:
    """`count` pictures spread evenly over the folder, as the device feeds them."""
    files = sorted(p for p in folder.iterdir() if p.suffix.lower() in IMAGE_EXT)
    if len(files) < 64:
        sys.exit(
            f"only {len(files)} pictures in {folder}; give at least 64 (500+ is better)"
        )
    step = max(1, len(files) // count)
    picks = files[::step][:count]
    arr = np.stack(
        [
            np.asarray(
                Image.open(p).convert("RGB").resize((640, 640)), dtype=np.float32
            )
            for p in picks
        ]
    )
    print(f"calibration: {len(arr)} pictures from {folder}")
    return arr


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("onnx")
    ap.add_argument(
        "--calib",
        required=True,
        help="folder of pictures like the device's (dataset/images/train)",
    )
    ap.add_argument(
        "--count", type=int, default=512, help="calibration pictures (default 512)"
    )
    ap.add_argument("--end-nodes", help="end_nodes.json (default: next to the .onnx)")
    ap.add_argument("--out", default="best.hef")
    ap.add_argument("--hw-arch", default="hailo10h")
    args = ap.parse_args()

    import hailo_sdk_client
    from hailo_sdk_client import ClientRunner

    version = getattr(hailo_sdk_client, "__version__", "?")
    print(f"Dataflow Compiler {version}")
    if not str(version).startswith(PI_HAILORT):
        print(
            f"WARNING: the Pi runs HailoRT {PI_HAILORT}.x — a .hef from compiler {version} "
            f"may not load there. Use the {PI_HAILORT}.x compiler if you can."
        )

    onnx_path = Path(args.onnx)
    nodes_path = (
        Path(args.end_nodes)
        if args.end_nodes
        else onnx_path.with_name("end_nodes.json")
    )
    nodes = json.loads(nodes_path.read_text())
    print(f"end nodes from {nodes_path}: {len(nodes['end_nodes'])}")

    runner = ClientRunner(hw_arch=args.hw_arch)
    runner.translate_onnx_model(
        str(onnx_path),
        "shappe_seg",
        start_node_names=[nodes.get("start_node", "images")],
        end_node_names=nodes["end_nodes"],
        net_input_shapes={nodes.get("start_node", "images"): nodes["input_shape"]},
    )
    runner.save_har(str(Path(args.out).with_suffix(".parsed.har")))

    # The app hands the chip 0-255 RGB; the model was trained on 0-1.
    runner.load_model_script(
        "normalization1 = normalization([0.0, 0.0, 0.0], [255.0, 255.0, 255.0])\n"
    )
    runner.optimize(calibration_set(Path(args.calib), args.count))
    runner.save_har(str(Path(args.out).with_suffix(".quantized.har")))

    hef = runner.compile()
    Path(args.out).write_bytes(hef)
    print(f"\nwrote {args.out} ({len(hef) / 1e6:.1f} MB)")
    print("copy it and metadata.yaml (next to best.pt) to the Pi, then on the Pi:")
    print("  venv/bin/python training/check_hef.py best.hef metadata.yaml")


if __name__ == "__main__":
    main()
