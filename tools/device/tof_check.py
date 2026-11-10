#!/usr/bin/env python3
"""
Hardware test for the Arducam Time-of-Flight camera on the Pi's CSI port (from
~/tof/test_tof.py, 2026-11-03; the node search is utils/tof_camera's, the app's own).
Walks the chain the camera needs, in order, and says what to fix at the first
broken link:

    1. boot config   - dtoverlay=arducam-pivariety in /boot/firmware/config.txt
    2. kernel driver - arducam-pivariety bound to the camera over I2C
    3. video node    - the CSI receiver node the SDK opens (/dev/videoN)
    4. Python SDK    - the ArducamDepthCamera package
    5. streaming     - open, start depth, grab frames, check they make sense,
                       save the last frame as depth.png / confidence.png

PORTS (Pi 5): two connectors, CAM/DISP 0 and CAM/DISP 1. The overlay must name
the one the ToF ribbon is in, and that port can't also carry another camera's
overlay (e.g. imx519) - one camera per port, and only one of these two lines:
    dtoverlay=arducam-pivariety          # ToF in CAM/DISP 1
    dtoverlay=arducam-pivariety,cam0     # ToF in CAM/DISP 0 (the rig: CamArray in CAM/DISP 1)
Reboot after editing config.txt.

SDK: ./venv/bin/pip install --no-deps ArducamDepthCamera==0.1.24 (its metadata asks
for numpy<2, which would replace the app's 2.2.4; it runs on 2.2.4). No OpenCV needed.

Only one program can hold the camera, and the app holds it while TOF_ENABLED: stop
it first (Ctrl-C ./start_lidar.sh, or sudo systemctl stop shappe-guide).

    ./venv/bin/python -m tools.device.tof_check               # full check + 30 frames, saves images
    ./venv/bin/python -m tools.device.tof_check --live        # then print the centre distance until Ctrl+C
    ./venv/bin/python -m tools.device.tof_check --frames 200  # longer stream test
    ./venv/bin/python -m tools.device.tof_check --range 2000  # 2 m mode (finer) - beyond 2 m wraps round and reads near
    ./venv/bin/python -m tools.device.tof_check --index 0     # open /dev/video0, skip the node search
"""

import argparse
import glob
import os
import re
import struct
import subprocess
import sys
import time
import zlib
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from config import TOF_MIN_CONFIDENCE, TOF_RANGE_MM, TOF_VIDEO_INDEX  # noqa: E402
from utils.tof_camera import CSI_NODE_NAMES, DRIVER, node_via_media_ctl  # noqa: E402

CONFIG_PATHS = ("/boot/firmware/config.txt", "/boot/config.txt")
# Overlay names that claim a camera port - enough to spot two on one port.
CAMERA_OVERLAY_PREFIXES = ("imx", "ov", "arducam")
PORT_LABELS = {"cam0": "CAM/DISP 0", "cam1": "CAM/DISP 1"}


def say(tag, msg):
    print(f"  [{tag}] {msg}")


def hint(msg):
    print(f"         {msg}")


def read(path):
    try:
        with open(path) as f:
            return f.read().strip().rstrip("\x00")
    except OSError:
        return None


# --- 1. boot config ------------------------------------------------------
def camera_overlays(model):
    """(config path, [(line, overlay name, port)]) for the camera overlays that
    apply to this board - lines under [cm4], [pi4] etc. on a Pi 5 are skipped."""
    sections = {"all"}
    if "Pi 5" in model:
        sections.add("pi5")
    elif "Pi 4" in model:
        sections.add("pi4")
    for path in CONFIG_PATHS:
        text = read(path)
        if text is None:
            continue
        found, section = [], "all"
        for line in text.splitlines():
            line = line.split("#", 1)[0].strip()
            if line.startswith("[") and line.endswith("]"):
                section = line[1:-1].strip().lower()
            elif section in sections and line.startswith("dtoverlay="):
                name, *params = line[len("dtoverlay="):].split(",")
                if name.startswith(CAMERA_OVERLAY_PREFIXES):
                    # Pi 5 camera overlays default to CAM/DISP 1.
                    found.append((line, name, "cam0" if "cam0" in params else "cam1"))
        return path, found
    return None, []


def overlay_fixes(overlays, model):
    """The config.txt edit for each port the ToF could be in."""
    if "Pi 5" not in model:
        return [f"add 'dtoverlay={DRIVER}' under [all]"]
    fixes = []
    for port, label in PORT_LABELS.items():
        want = f"dtoverlay={DRIVER}" + (",cam0" if port == "cam0" else "")
        there = [line for line, name, p in overlays if p == port and name != DRIVER]
        if there:
            fixes.append(f"ToF in {label}: replace '{there[0]}' with '{want}'")
        else:
            fixes.append(f"ToF in {label}: add '{want}' under [all]")
    return fixes


def check_config(model):
    path, overlays = camera_overlays(model)
    if path is None:
        say("WARN", "no config.txt found - can't check the overlay")
        return False
    listed = ", ".join(f"{line[len('dtoverlay='):]} ({p})" for line, _, p in overlays)
    print(f"  camera overlays in {path}: {listed or 'none'}")
    tof_ports = {p for _, name, p in overlays if name == DRIVER}
    if not tof_ports:
        say("FAIL", f"no dtoverlay={DRIVER} - the kernel never looks for the ToF camera")
        for fix in overlay_fixes(overlays, model):
            hint(fix)
        hint(f"(sudo nano {path}, then sudo reboot)")
        return False
    for port in sorted(tof_ports):
        clash = [line for line, name, p in overlays if p == port and name != DRIVER]
        say("PASS", f"dtoverlay={DRIVER} on {port} ({PORT_LABELS[port]})")
        if clash:
            # Only a warning: step 2 shows whether the ToF actually won the port. Which line
            # goes depends on the camera in the port: a bare dtoverlay=arducam-pivariety (CAM1)
            # beside imx519,cam1 took the CamArray's port and its lens (2026-11-05).
            say("WARN", f"{port} also has {', '.join(clash)} - one camera per port, the")
            hint("two drivers fight over the connector (and I2C address 0x0c for imx519)")
            there = answered_on(port)
            other = [c for c in clash if c[len("dtoverlay="):].split(",")[0] in there]
            if DRIVER in there:
                drop = clash
            elif other:
                hint(f"{other[0]} answered in {PORT_LABELS[port]} at boot and the ToF didn't:")
                hint("that camera is in this port, so the ToF line goes")
                drop = [line for line, name, p in overlays if p == port and name == DRIVER]
            else:
                hint("neither answered at boot: keep the line for the camera in this port")
                drop = []
            if drop:
                hint(f"remove {', '.join(map(repr, drop))} from {path}, then sudo reboot")
    return True


def edited_since_boot(path):
    uptime = read("/proc/uptime")
    if not uptime or not os.path.exists(path):
        return False
    return os.path.getmtime(path) > time.time() - float(uptime.split()[0])


# --- 2. kernel driver ----------------------------------------------------
def i2c_names(pattern):
    return sorted(os.path.basename(p) for p in glob.glob(pattern)
                  if re.fullmatch(r"\d+-[0-9a-f]{4}", os.path.basename(p)))


def driver_log():
    try:
        out = subprocess.run(["dmesg"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.TimeoutExpired):
        return []
    return [line for line in out.splitlines() if "pivariety" in line.lower()]


def csi_port(i2c_name):
    """'cam0' / 'cam1' for an I2C device like '10-000c', from the device tree."""
    node = os.path.realpath(f"/sys/bus/i2c/devices/i2c-{i2c_name.split('-')[0]}/of_node")
    for port in PORT_LABELS:
        sym = read(f"/proc/device-tree/__symbols__/i2c_csi_dsi{port[-1]}")
        if sym and node.endswith(sym):
            return port
    return None


def where(i2c_name):
    port = csi_port(i2c_name)
    return f"I2C {i2c_name} ({PORT_LABELS[port]})" if port else f"I2C {i2c_name}"


def answered_on(port):
    """The drivers bound on a port's I2C bus in the running kernel ({'imx519', 'ak7375'}):
    a driver binds only if its chip answered."""
    names = set()
    for link in glob.glob("/sys/bus/i2c/devices/*/driver"):
        i2c_name = os.path.basename(os.path.dirname(link))
        if re.fullmatch(r"\d+-[0-9a-f]{4}", i2c_name) and csi_port(i2c_name) == port:
            names.add(os.path.basename(os.path.realpath(link)))
    return names


def check_driver(cfg_ok):
    bound = i2c_names(f"/sys/bus/i2c/drivers/{DRIVER}/*")
    # The overlay creates the I2C device; the driver binds only if the camera answers.
    declared = sorted(os.path.basename(os.path.dirname(p))
                      for p in glob.glob("/sys/bus/i2c/devices/*/name") if read(p) == DRIVER)
    if bound:
        say("PASS", f"{DRIVER} driver bound to the camera at {', '.join(map(where, bound))}")
        for name in declared:
            if name not in bound:
                say("INFO", f"nothing answered at {where(name)} - fine if the ToF isn't there")
        return True
    if not declared:
        say("FAIL", "the ToF overlay isn't loaded in the running kernel")
        path, _ = camera_overlays(pi_model())
        if cfg_ok or (path and edited_since_boot(path)):
            hint("config.txt looks right / was edited after this boot - sudo reboot")
        else:
            hint("fix the config.txt line above, then sudo reboot")
        return False
    say("FAIL", f"overlay loaded ({', '.join(map(where, declared))}) but the camera didn't answer")
    hint("power off, then check the ribbon: in the port the overlay names, the right")
    hint("way round, pushed fully in, latch closed - at both the Pi and the camera end")
    hint("and clear any port clash [WARN] in step 1")
    log = driver_log()
    for line in log[-6:]:
        hint(line)
    return False


# --- 3. video node -------------------------------------------------------
# node_via_media_ctl() is utils/tof_camera's: the node found here is the one the app opens.
def find_node(args):
    if args.index is not None:
        index = args.index
        print(f"  using --index {index}")
    else:
        try:
            index = node_via_media_ctl()
        except (FileNotFoundError, subprocess.TimeoutExpired):
            # No media-ctl: fine if there's only one CSI node to choose from.
            nodes = sorted(int(re.search(r"video(\d+)", p).group(1))
                           for p in glob.glob("/sys/class/video4linux/video*/name")
                           if read(p) in CSI_NODE_NAMES)
            index = nodes[0] if len(nodes) == 1 else None
            if len(nodes) > 1:
                say("FAIL", f"CSI nodes {nodes} and no media-ctl to tell which is the ToF")
                hint("sudo apt install v4l-utils, or pass --index N")
                return None
        if index is None:
            say("FAIL", "the driver is bound but no CSI video node is linked to it")
            hint("check dmesg for rp1-cfe / unicam errors, or pass --index N")
            return None
    dev = f"/dev/video{index}"
    if not os.path.exists(dev):
        say("FAIL", f"{dev} doesn't exist")
        return None
    if not os.access(dev, os.R_OK | os.W_OK):
        say("FAIL", f"no permission on {dev}")
        hint("sudo usermod -aG video $USER, then log out and back in")
        return None
    say("PASS", f"{dev}")
    return index


# --- 4. SDK --------------------------------------------------------------
def import_sdk():
    try:
        import ArducamDepthCamera as ac
    except ImportError as e:
        say("FAIL", f"ArducamDepthCamera not importable by {sys.executable} ({e})")
        if sys.prefix != sys.base_prefix:
            hint(f"{sys.executable} -m pip install ArducamDepthCamera")
        else:
            hint("pip install --break-system-packages ArducamDepthCamera")
            hint("(or install it in a venv and run this script with that venv's python)")
        return None
    say("PASS", f"ArducamDepthCamera {getattr(ac, '__version__', '?')}")
    return ac


# --- 5. streaming --------------------------------------------------------
def failed(ret):
    return ret is not None and int(ret) != 0


def err_name(ac, ret):
    try:
        return ac.TofErrorCode(int(ret)).name
    except Exception:
        return str(ret)


def read_frame(ac, cam, timeout_ms):
    """(depth mm, confidence), copied out before the SDK reuses its buffer, or None."""
    frame = cam.requestFrame(timeout_ms)
    if frame is None:
        return None
    try:
        if not isinstance(frame, ac.DepthData):
            return None
        depth = np.array(frame.depth_data, dtype=np.float32, copy=True)
        conf = np.array(frame.confidence_data, dtype=np.float32, copy=True)
    finally:
        cam.releaseFrame(frame)
    if conf.shape != depth.shape:
        conf = np.full_like(depth, np.inf)    # no confidence plane: trust every pixel
    return depth, conf


def frame_stats(depth, conf, rng, min_conf):
    """(valid-pixel mask, median distance in the centre ~10% of the view or None)."""
    valid = (conf >= min_conf) & (depth > 0) & (depth <= rng)
    h, w = depth.shape
    ch, cw = max(1, h // 20), max(1, w // 20)
    patch = (slice(h // 2 - ch, h // 2 + ch), slice(w // 2 - cw, w // 2 + cw))
    centre = depth[patch][valid[patch]]
    return valid, (float(np.median(centre)) if centre.size else None)


def write_png(path, img):
    """Stdlib PNG writer (gray HxW or RGB HxWx3 uint8), so OpenCV isn't needed."""
    img = np.ascontiguousarray(img, dtype=np.uint8)
    h, w = img.shape[:2]
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(h))

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n"
                + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2 if img.ndim == 3 else 0, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw))
                + chunk(b"IEND", b""))


def upscale(img, width=720):
    k = max(1, width // img.shape[1])
    return np.repeat(np.repeat(img, k, axis=0), k, axis=1)


def colorize(depth, valid, rng):
    """Near = red, far = blue, no reading = black."""
    t = np.clip(depth / rng, 0, 1)
    rgb = np.stack([np.clip(1.5 - np.abs(4 * t - k), 0, 1) for k in (1, 2, 3)], axis=-1)
    rgb = (rgb * 255).astype(np.uint8)
    rgb[~valid] = 0
    return upscale(rgb)


def save_frame(out_dir, depth, conf, rng, min_conf):
    os.makedirs(out_dir, exist_ok=True)
    np.save(os.path.join(out_dir, "depth_mm.npy"), depth)
    np.save(os.path.join(out_dir, "confidence.npy"), conf)
    valid, _ = frame_stats(depth, conf, rng, min_conf)
    write_png(os.path.join(out_dir, "depth.png"), colorize(depth, valid, rng))
    finite = np.where(np.isfinite(conf), conf, 0)
    top = finite.max()
    write_png(os.path.join(out_dir, "confidence.png"),
              upscale((finite / top * 255 if top > 0 else finite).astype(np.uint8)))
    say("INFO", f"last frame saved to {out_dir}/ (depth.png, confidence.png, *.npy)")


def check_frames(ac, cam, args, rng):
    depths, valid_pct, centres = [], [], []
    last = None
    misses = 0
    t0 = time.monotonic()
    for _ in range(args.frames):
        got = read_frame(ac, cam, args.timeout)
        if got is None:
            misses += 1
            continue
        last = got
        depths.append(got[0])
        valid, centre = frame_stats(*got, rng, args.confidence)
        valid_pct.append(100 * valid.mean())
        if centre is not None:
            centres.append(centre)
    elapsed = time.monotonic() - t0

    if not depths:
        say("FAIL", f"no frames in {args.frames} requests ({args.timeout} ms timeout each)")
        hint("the sensor is detected but not streaming - reseat the ribbon, check dmesg")
        return False
    say("WARN" if misses > args.frames // 10 else "PASS",
        f"{len(depths)}/{args.frames} frames, {len(depths) / elapsed:.1f} fps"
        + (f", {misses} timed out" if misses else ""))

    ok = True
    if all(d.max() <= 0 for d in depths):
        say("FAIL", "every frame is all zeros - it streams but measures nothing")
        hint("lens cap / tape over the emitter? otherwise the IR emitter may be faulty")
        ok = False
    elif len(depths) > 1 and all(np.array_equal(depths[0], d) for d in depths[1:]):
        say("FAIL", "every frame is identical - the stream is stuck")
        ok = False

    valid = float(np.median(valid_pct))
    if valid < 5:
        say("WARN", f"only {valid:.0f}% of pixels have a confident reading (>= {args.confidence:g})")
        hint("point it at a wall or desk 0.3-2 m away, out of direct sunlight")
    else:
        say("PASS", f"{valid:.0f}% of pixels have a confident reading")
    if centres:
        say("INFO", f"centre distance {np.median(centres) / 1000:.3f} m, "
                    f"frame-to-frame spread {np.std(centres):.0f} mm")
    else:
        say("WARN", "no confident reading at the centre of the view")

    if last is not None and not args.no_save:
        save_frame(args.out, *last, rng, args.confidence)
    return ok


def live(ac, cam, args, rng):
    print("\n  live - move your hand in front of the camera, Ctrl+C to stop")
    n, t0 = 0, time.monotonic()
    try:
        while True:
            got = read_frame(ac, cam, args.timeout)
            if got is None:
                print("\r  (no frame)" + " " * 40, end="", flush=True)
                continue
            n += 1
            valid, centre = frame_stats(*got, rng, args.confidence)
            dist = f"{centre / 1000:5.2f} m" if centre is not None else "   -- m"
            print(f"\r  centre {dist}   valid {100 * valid.mean():3.0f}%   "
                  f"{n / (time.monotonic() - t0):4.1f} fps   ", end="", flush=True)
    except KeyboardInterrupt:
        print()


def stream_test(ac, index, args):
    cam = ac.ArducamCamera()
    ret = cam.open(ac.Connection.CSI, index)
    if failed(ret):
        say("FAIL", f"SDK could not open /dev/video{index}: {err_name(ac, ret)}")
        hint("another program may have it open - the app holds it while TOF_ENABLED, as")
        hint("does ~/tof/tof_live.py - or it isn't the ToF's node (try --index)")
        return False
    try:
        ret = cam.start(ac.FrameType.DEPTH)
        if failed(ret):
            say("FAIL", f"could not start the depth stream: {err_name(ac, ret)}")
            return False
        try:
            ret = cam.setControl(ac.Control.RANGE, args.range)
            if failed(ret):
                say("WARN", f"range {args.range} mm rejected: {err_name(ac, ret)}")
            rng = cam.getControl(ac.Control.RANGE) or args.range
            info = cam.getCameraInfo()
            try:
                cam_id = cam.getCameraID()
            except Exception:
                cam_id = ""
            cam_id = f", id {cam_id}" if cam_id else ""
            kind = getattr(info.device_type, "name", info.device_type)
            say("PASS", f"opened {kind} {info.width}x{info.height}, range {rng} mm{cam_id}")
            ok = check_frames(ac, cam, args, rng)
            if ok and args.live:
                live(ac, cam, args, rng)
            return ok
        finally:
            cam.stop()
    finally:
        cam.close()


def pi_model():
    return read("/proc/device-tree/model") or "unknown board"


def main():
    ap = argparse.ArgumentParser(description="Arducam ToF camera hardware test")
    ap.add_argument("--frames", type=int, default=30,
                    help="frames to grab for the stream test (default 30)")
    ap.add_argument("--range", type=int, default=TOF_RANGE_MM, choices=[2000, 4000],
                    help=f"max distance in mm: 4000 or 2000 (finer) (default {TOF_RANGE_MM}, "
                         "config TOF_RANGE_MM)")
    ap.add_argument("--confidence", type=float, default=TOF_MIN_CONFIDENCE,
                    help="min confidence for a pixel to count as valid "
                         f"(default {TOF_MIN_CONFIDENCE:g}, config TOF_MIN_CONFIDENCE)")
    ap.add_argument("--timeout", type=int, default=2000, help="per-frame timeout, ms")
    ap.add_argument("--index", type=int, default=TOF_VIDEO_INDEX,
                    help="/dev/videoN to open (default: config TOF_VIDEO_INDEX, else "
                         "found via media-ctl)")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
                        os.path.abspath(__file__)))), "logs", "tof_check"),
                    help="where depth.png etc. are saved (default logs/tof_check)")
    ap.add_argument("--no-save", action="store_true", dest="no_save")
    ap.add_argument("--live", action="store_true",
                    help="after the test, print the centre distance until Ctrl+C")
    args = ap.parse_args()

    model = pi_model()
    print(f"Arducam ToF camera test - {model}\n")
    print("1. boot config")
    cfg_ok = check_config(model)
    print("2. kernel driver")
    drv_ok = check_driver(cfg_ok)
    print("3. video node")
    index = find_node(args) if drv_ok else None
    if not drv_ok:
        say("SKIP", "needs the driver")
    print("4. Python SDK")
    ac = import_sdk()
    if not drv_ok or index is None or ac is None:
        print("\nRESULT: FAIL - fix the [FAIL] items above and run again")
        return 1
    print("5. streaming")
    ok = stream_test(ac, index, args)
    print(f"\nRESULT: {'PASS - the ToF camera is working' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
