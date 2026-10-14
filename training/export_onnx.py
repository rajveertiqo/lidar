#!/usr/bin/env python3
"""Export the trained model to ONNX for Hailo's compiler, and find its output layers.

    python training/export_onnx.py runs/segment/shappe-seg/weights/best.pt

Writes best.onnx and end_nodes.json beside best.pt. Hailo's compiler must stop at the
raw outputs of the model's head, before Ultralytics' own decoding: for each of the 3
scales a box layer (64 channels), a class layer (one channel per class) and a mask-
coefficient layer (32), plus the 160x160x32 mask prototypes. That is the layout the
app's decoder expects (utils/seg_inference.py, utils/yoloseg_decode.py). Getting these
names wrong is the most common reason a segmentation model will not compile.
"""

import json
import re
import sys
from pathlib import Path

# Final layer of each branch of the YOLOv8 / YOLO11 Segment head:
#   cv2 = box (64 ch), cv3 = class scores (nc ch), cv4 = mask coefficients (32 ch);
#   proto/cv3 + its activation = the mask prototypes.
_BRANCH = re.compile(r"^/model\.(\d+)/cv([234])\.(\d)/cv\2\.\3\.2/Conv$")
_PROTO = re.compile(r"^/model\.(\d+)/proto/cv3/act/Mul$")


def find_end_nodes(onnx_path) -> dict:
    """{"end_nodes": [...], "head": layer index} found in the ONNX graph, or exit."""
    import onnx
    graph = onnx.load(str(onnx_path)).graph
    names = [n.name for n in graph.node]
    branches = {}
    proto = None
    for n in names:
        m = _BRANCH.match(n)
        if m:
            branches[(int(m.group(1)), int(m.group(2)), int(m.group(3)))] = n
        elif _PROTO.match(n):
            proto = n
    heads = {k[0] for k in branches}
    if len(heads) != 1 or len(branches) != 9 or proto is None:
        sys.exit(f"could not find the segmentation head's 9 branch outputs + prototypes "
                 f"(found {len(branches)} branches, prototypes: {proto}). Open the .onnx in "
                 f"https://netron.app and look for the last Conv of each cv2/cv3/cv4 branch "
                 f"and proto/cv3's activation; or run `hailo parser onnx` in WSL and use "
                 f"the end nodes it suggests.")
    head = heads.pop()
    order = [branches[(head, kind, scale)] for scale in range(3) for kind in (2, 3, 4)]
    return {"end_nodes": order + [proto], "head": head, "start_node": graph.input[0].name}


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    pt = Path(sys.argv[1])
    from ultralytics import YOLO
    model = YOLO(str(pt))
    if model.task != "segment":
        sys.exit(f"{pt} is a {model.task} model, not segmentation")
    nc = len(model.names)
    onnx_path = Path(model.export(format="onnx", imgsz=640, opset=11, simplify=True,
                                  dynamic=False, batch=1))
    found = find_end_nodes(onnx_path)
    found.update({"classes": nc, "names": [model.names[i] for i in range(nc)],
                  "input_shape": [1, 3, 640, 640]})
    out = pt.parent / "end_nodes.json"
    out.write_text(json.dumps(found, indent=2))
    print(f"\nONNX: {onnx_path}")
    print(f"end nodes ({out}):")
    for n in found["end_nodes"]:
        print(f"  {n}")
    print("\nnext, in WSL Ubuntu with Hailo's Dataflow Compiler installed:")
    print(f"  python3 training/compile_hef.py {onnx_path.name} --calib dataset/images/train")


if __name__ == "__main__":
    main()
