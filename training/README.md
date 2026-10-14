# Training the segmentation model (on a Windows laptop)

This trains the camera model that marks **path, road, crosswalk, tactile paving, stairs
and people** in the front camera's picture, then turns it into a `.hef` file for the
Pi's Hailo-10H chip. The app already runs this kind of model (`model/best.hef`, today
with only `person, path, tactile`); a better one is the base for a "crossing ahead" cue.

```
 Pi                    Windows laptop                                  Pi
 collect pictures  ->  label (Roboflow) -> prepare -> train -> ONNX   ->  check -> install
                                              WSL2 Ubuntu: ONNX -> .hef
```

| Step | Where | Time |
|---|---|---|
| A. Collect pictures | Pi, while walking | a few walks |
| B. Set up the laptop | Windows | 30 min, once |
| C. Label | Roboflow (browser) | **most of the work**: 1–2 min per picture |
| D. Prepare + train | Windows (NVIDIA GPU) or Colab | 1–4 h on a GPU |
| E. Export to ONNX | Windows | 2 min |
| F. Compile to `.hef` | WSL2 Ubuntu on the same laptop | 30–120 min, once per model |
| G. Check + install | Pi | 10 min |

The class names matter: the app finds classes **by name**. Use exactly
`person, path, road, crosswalk, tactile, stairs` (set in `prepare_dataset.py`).

---

## A. Collect pictures (on the Pi)

Pictures from **this device's own camera**, on the streets it will be used on, matter
more than anything else: public datasets come from other countries, other paving, other
camera heights.

1. Start the app as usual (`./start_lidar.sh` in `~/Documents/lidar`).
2. In a second terminal, while walking:
   ```bash
   venv/bin/python tools/collect_frames.py --sectors front --every 2
   ```
   It skips black, scrambled and repeated pictures, so standing still adds nothing.
3. Cover: crossings **with and without** tactile paving, plain roads, footpaths, kerbs,
   stairs, indoor floors; morning, midday, evening, night, rain, shadows. Aim for
   **1,500–3,000** pictures.
4. Copy them to the laptop (PowerShell; Windows has `scp` built in):
   ```powershell
   scp -r shappe@<pi-address>:~/Documents/lidar/logs/training_frames C:\shappe\frames
   ```

## B. Set up the laptop (once)

1. **Does it have an NVIDIA GPU?** In PowerShell: `nvidia-smi`. A table = yes. "not
   recognized" = no → train on Google Colab instead (section H); everything else stays.
2. Install **Python 3.11** from python.org — tick *Add python.exe to PATH*.
3. Get this repository onto the laptop (`git clone` it, or copy the `training/` folder).
4. Make an environment and install:
   ```powershell
   cd C:\shappe\lidar
   py -3.11 -m venv .venv
   .venv\Scripts\Activate.ps1          # if blocked: Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
   # PyTorch FIRST, with the command pytorch.org/get-started/locally gives for
   # Windows + pip + your CUDA version, e.g.:
   pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
   pip install -r training\requirements.txt
   python -c "import torch; print('GPU:', torch.cuda.is_available())"   # must say True
   ```

## C. Label (Roboflow, in the browser)

1. Make a free account at roboflow.com → **New Project** → type **Instance Segmentation**.
2. Upload the pictures from step A.
3. Add the classes with exactly these names: `person path road crosswalk tactile stairs`.
4. Draw **polygons** (the *Smart Polygon* tool outlines a thing in one or two clicks).
   - `tactile` — every tactile tile: warning dots and guidance bars.
   - `crosswalk` — the painted crossing on the road.
   - `road` — where vehicles drive (not the crosswalk part).
   - `path` — footpath, pavement, indoor floor: where people walk.
   - `stairs` — flights of steps, up or down.
   - `person` — people.
   - **Outline every one in every picture.** A missed outline teaches the model to
     ignore that thing.
5. **Generate** a version with **no** resize and **no** augmentation (the scripts do
   their own), then **Export → YOLOv8** (segmentation) → download zip → unzip to
   `C:\shappe\exports\my-walks`.
6. Optional public data, same format: Roboflow Universe *tactile-paving-segmentation*
   (319 pictures, CC BY 4.0) → unzip to `C:\shappe\exports\tactile-public`.
   Other datasets work too — `prepare_dataset.py` lists any class names it cannot map;
   add them to `ALIASES` at its top.

## D. Prepare and train (Windows)

```powershell
python training\prepare_dataset.py --out C:\shappe\dataset C:\shappe\exports\my-walks C:\shappe\exports\tactile-public
python training\train.py --data C:\shappe\dataset\data.yaml
```

- `prepare_dataset.py` crops every picture to 16:9 and stretches it to 640×640 — exactly
  what the device does to the camera picture — and prints how many outlines each class
  has. Under ~300 for a class: label more of it.
- `train.py` uses the GPU, stops by itself when it stops improving, and prints a score
  per class (0–1). Aim for **0.7+ for tactile and crosswalk**; under 0.5 means more
  pictures of it. Out of memory: add `--batch 8 --workers 2`.
- Result: `runs\segment\shappe-seg\weights\best.pt` and `metadata.yaml` beside it.

## E. Export to ONNX (Windows)

```powershell
python training\export_onnx.py runs\segment\shappe-seg\weights\best.pt
```

Writes `best.onnx` and `end_nodes.json` (the output layers Hailo's compiler must stop
at) next to `best.pt`.

## F. Compile to `.hef` (WSL2 Ubuntu on the same laptop)

Hailo's compiler only runs on Linux. WSL2 gives Windows a real Ubuntu; it has no GPU
there, which makes compiling slower and quantisation simpler, but it works.

1. PowerShell **as administrator**: `wsl --install -d Ubuntu-22.04`, restart, pick a
   user name. Optional, if the laptop has 32 GB: give WSL more memory with a file
   `C:\Users\<you>\.wslconfig` containing `[wsl2]` and `memory=16GB`.
2. Download the compiler: hailo.ai → **Developer Zone** (free account) → Software
   Downloads → **Hailo-10H** → **Dataflow Compiler 5.1.x**, Python wheel for Linux
   x86_64. It must be **5.1.x**: the Pi runs HailoRT 5.1, and a `.hef` from another
   release may not load. The wheel's name says which Python it wants (`cp310` = 3.10,
   which Ubuntu 22.04 has).
3. In the Ubuntu window:
   ```bash
   sudo apt update && sudo apt install -y python3-venv python3-dev graphviz libgraphviz-dev pkg-config
   python3 -m venv ~/hailo && source ~/hailo/bin/activate
   pip install /mnt/c/Users/<you>/Downloads/hailo_dataflow_compiler-5.1.*.whl pillow numpy
   # copy the files in (WSL is much faster on its own disk than on /mnt/c)
   mkdir -p ~/seg && cp -r /mnt/c/shappe/lidar/training ~/seg/ \
       && cp /mnt/c/shappe/lidar/runs/segment/shappe-seg/weights/{best.onnx,end_nodes.json,metadata.yaml} ~/seg/ \
       && cp -r /mnt/c/shappe/dataset/images/train ~/seg/calib
   cd ~/seg
   python3 training/compile_hef.py best.onnx --calib calib
   ```
4. Result: `~/seg/best.hef`. Copy it back: `cp ~/seg/best.hef /mnt/c/shappe/`.

**If it fails while parsing** (often the segmentation head): run
`hailo parser onnx best.onnx --hw-arch hailo10h`, note the *end node* names it
suggests, put them into `end_nodes.json`, and run `compile_hef.py` again.

## G. Check and install (on the Pi)

```powershell
scp C:\shappe\best.hef C:\shappe\lidar\runs\segment\shappe-seg\weights\metadata.yaml shappe@<pi-address>:~/Documents/lidar/new_model/
```
Then on the Pi, **with the app stopped**:
```bash
cd ~/Documents/lidar
venv/bin/python training/check_hef.py new_model/best.hef new_model/metadata.yaml logs/training_frames/*/front-*.jpg
```
It refuses a model whose outputs the app cannot read or whose `metadata.yaml` does not
match, prints what each picture contains, and saves overlays in `check_hef_out/` — look
at them. If they look right:
```bash
cp model/best.hef model/best.hef.old && cp model/metadata.yaml model/metadata.yaml.old
cp new_model/best.hef model/best.hef && cp new_model/metadata.yaml model/metadata.yaml
```
Start the app and open `/camera/seg/front`. To go back: copy the `.old` files back.

## H. No NVIDIA GPU: train on Google Colab

Steps A–C, E–G stay the same. For D:
1. Zip `C:\shappe\dataset` and upload it to Google Drive; upload `training\train.py` too.
2. colab.research.google.com → New notebook → *Runtime → Change runtime type → T4 GPU*.
3. Cells:
   ```
   !pip install -q ultralytics
   from google.colab import drive; drive.mount('/content/drive')
   !unzip -q /content/drive/MyDrive/dataset.zip -d /content
   !python /content/drive/MyDrive/train.py --data /content/dataset/data.yaml
   ```
4. Download `runs/segment/shappe-seg/weights/` (best.pt + metadata.yaml) and continue
   with step E on Windows.
