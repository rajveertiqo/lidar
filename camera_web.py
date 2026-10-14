#!/usr/bin/env python3
"""Live camera feed in a web browser (MJPEG over HTTP)."""

from __future__ import annotations

import argparse
import fcntl
import os
import socket
import sys
import threading
import time
from http import server
from socketserver import ThreadingMixIn

import numpy as np
import simplejpeg
from picamera2 import Picamera2


# ------------------------------------------------------------- camarray modes

# Register 0x24 on the HAT controller at i2c address 0x24.
CAMARRAY_MODES = {
    "4ch":    0x00,   # all four merged, synchronised, 1/4 res each
    "dual01": 0x01,   # sensors 0+1 side by side
    "dual23": 0x11,   # sensors 2+3 side by side
    "cam0":   0x02,
    "cam1":   0x12,
    "cam2":   0x22,
    "cam3":   0x32,
}

# How each mode tiles the composite, used as the default --split.
MODE_LAYOUT = {"4ch": "2x2", "dual01": "2x1", "dual23": "2x1"}

# Largest sensor mode whose crop covers the whole array - see the note in main().
SAFE_SENSOR_MODE = (2328, 1748)

FRAME_TIMEOUT_S = 5.0

TEARDOWN_TIMEOUT_S = 8.0


I2C_SLAVE = 0x0703      # linux/i2c-dev.h
HAT_ADDR = 0x24         # CamArray controller
MODE_REG = 0x24         # channel-mode register


def set_camarray_mode(bus, mode):
    """Write the channel mode to the CamArray HAT. Needs membership of group i2c."""
    value = CAMARRAY_MODES[mode]
    path = f"/dev/i2c-{bus}"
    try:
        fd = os.open(path, os.O_RDWR)
    except FileNotFoundError:
        sys.exit(f"{path} does not exist - try a different --i2c-bus")
    except PermissionError:
        sys.exit(f"no access to {path} - join the i2c group:\n"
                 f"  sudo adduser $USER i2c   (then log out and back in)")
    try:
        fcntl.ioctl(fd, I2C_SLAVE, HAT_ADDR)
        os.write(fd, bytes([MODE_REG, value]))     # write mode
        os.write(fd, bytes([MODE_REG]))            # seek register
        readback = os.read(fd, 1)[0]               # confirm it stuck
    except OSError as exc:
        sys.exit(f"i2c error on {path} addr {HAT_ADDR:#04x}: {exc}\n"
                 f"is the CamArray HAT really on this bus?")
    finally:
        os.close(fd)

    if readback != value:
        sys.exit(f"HAT rejected the mode (wrote {value:#04x}, read back {readback:#04x})")
    print(f"CamArray HAT -> {mode} (reg {MODE_REG:#04x} = {readback:#04x})")


# ---------------------------------------------------------------- frame broker

class FrameBroker:
    """Holds the newest JPEG for each view and wakes clients when one arrives."""

    def __init__(self, names):
        self._cond = threading.Condition()
        self._frames = {n: None for n in names}
        self._seq = 0

    def publish(self, frames):
        with self._cond:
            self._frames.update(frames)
            self._seq += 1
            self._cond.notify_all()

    def get(self, name, last_seq, timeout=5.0):
        """Block until a frame newer than last_seq exists, then return it."""
        with self._cond:
            if self._seq == last_seq:
                self._cond.wait(timeout)
            if self._seq == last_seq:
                return None, self._seq
            return self._frames.get(name), self._seq


# ---------------------------------------------------------------- capture loop

def _arm_teardown_watchdog():
    """Guarantee the process exits, so the camera is always released."""
    def bail():
        time.sleep(TEARDOWN_TIMEOUT_S)
        print(f"teardown still wedged after {TEARDOWN_TIMEOUT_S:.0f}s - forcing "
              f"exit so the camera is released", file=sys.stderr)
        sys.stderr.flush()
        os._exit(1)
    threading.Thread(target=bail, daemon=True).start()


def _capture_bounded(picam2, stop):
    """One frame, or None if it did not arrive within FRAME_TIMEOUT_S."""
    job = picam2.capture_array(wait=False)
    deadline = time.monotonic() + FRAME_TIMEOUT_S
    while True:
        try:
            return picam2.wait(job, timeout=0.25)
        except TimeoutError:      # concurrent.futures.TimeoutError on py3.11+
            if stop.is_set() or time.monotonic() >= deadline:
                return None


def capture_loop(picam2, broker, views, quality, stop):
    single = len(views) == 1
    while not stop.is_set():
        try:
            frame = _capture_bounded(picam2, stop)
        except Exception as exc:                      # camera stopped / yanked
            if not stop.is_set():
                print(f"capture failed: {exc} - shutting down rather than "
                      f"holding the camera", file=sys.stderr)
                stop.set()
            break

        if frame is None:                             # timed out or stopping
            if not stop.is_set():
                print(f"no frame for {FRAME_TIMEOUT_S:.0f}s - the camera is not "
                      f"delivering. Shutting down rather than holding it open.",
                      file=sys.stderr)
                stop.set()
            break

        out = {}
        for name, (x, y, w, h) in views.items():
            tile = frame if single else frame[y:y + h, x:x + w]
            # picamera2's "RGB888" hands back B,G,R in memory order.
            out[name] = simplejpeg.encode_jpeg(
                np.ascontiguousarray(tile), quality=quality, colorspace="BGR"
            )
        broker.publish(out)


# ---------------------------------------------------------------- http serving

PAGE_HEAD = """<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pi Camera Live</title>
<style>
  :root { color-scheme: dark; }
  body { margin:0; padding:16px; background:#111; color:#eee;
         font:14px system-ui,-apple-system,Segoe UI,Roboto,sans-serif; }
  h1 { font-size:16px; font-weight:600; margin:0 0 4px; }
  .meta { color:#888; font-size:12px; margin-bottom:16px; }
  .grid { display:grid; gap:12px; grid-template-columns:repeat(auto-fit,minmax(320px,1fr)); }
  .cell { background:#1b1b1b; border:1px solid #2c2c2c; border-radius:8px; overflow:hidden; }
  .cell h2 { font-size:12px; font-weight:600; margin:0; padding:8px 10px;
             color:#bbb; border-bottom:1px solid #2c2c2c; letter-spacing:.04em; }
  img { display:block; width:100%; height:auto; background:#000; }
</style></head><body>
"""


def build_page(views, size, note):
    w, h = size
    cells = "".join(
        f'<div class="cell"><h2>{n.upper()}</h2>'
        f'<img src="/stream/{n}.mjpg" alt="{n}"></div>'
        for n in views
    )
    return (
        PAGE_HEAD
        + "<h1>Raspberry Pi Camera &mdash; live</h1>"
        + f'<div class="meta">source {w}&times;{h} &middot; {note}</div>'
        + f'<div class="grid">{cells}</div></body></html>'
    ).encode("utf-8")


def make_handler(broker, views, page):
    class Handler(server.BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):            # keep the console quiet
            pass

        @staticmethod
        def _view_from(path, prefix, suffix):
            if path.startswith(prefix) and path.endswith(suffix):
                name = path[len(prefix):-len(suffix)]
                if name in views:
                    return name
            return None

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)
                return

            name = self._view_from(path, "/snapshot/", ".jpg")
            if name:
                jpeg, _ = broker.get(name, -1)
                if jpeg is None:
                    self.send_error(503, "no frame yet")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(jpeg)
                return

            name = self._view_from(path, "/stream/", ".mjpg")
            if name:
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                self.send_header("Pragma", "no-cache")
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=FRAME"
                )
                self.end_headers()
                seq = -1
                try:
                    while True:
                        jpeg, seq = broker.get(name, seq)
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--FRAME\r\n")
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(jpeg)))
                        self.end_headers()
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass                       # browser closed the tab
                return
            

            self.send_error(404)

    return Handler


class ThreadedServer(ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def parse_size(text):
    w, _, h = text.lower().partition("x")
    return int(w), int(h)


def build_views(layout, w, h):
    """Map view name -> (x, y, w, h) crop box."""
    if not layout:
        return {"cam0": (0, 0, w, h)}, "single full frame"
    cols, rows = parse_size(layout)
    tw, th = w // cols, h // rows
    views, i = {}, 0
    for r in range(rows):
        for c in range(cols):
            views[f"cam{i}"] = (c * tw, r * th, tw, th)
            i += 1
    return views, f"{layout} split, {tw}x{th} per tile"


def local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "127.0.0.1"


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--mode", choices=sorted(CAMARRAY_MODES),
                    help="set CamArray HAT channel mode before starting")
    ap.add_argument("--split", metavar="CxR",
                    help="slice each frame into a CxR grid (default follows --mode)")
    ap.add_argument("--size", default="1280x720", help="capture size (default 1280x720)")
    ap.add_argument("--port", type=int, default=8000, help="HTTP port (default 8000)")
    ap.add_argument("--quality", type=int, default=85, help="JPEG quality (default 85)")
    ap.add_argument("--camera", type=int, default=0, help="camera index (default 0)")
    ap.add_argument("--i2c-bus", type=int, default=11, help="i2c bus the HAT sits on (default 11)")
    args = ap.parse_args()

    if args.mode:
        set_camarray_mode(args.i2c_bus, args.mode)

    # A merged mode implies a split unless the user asked for something else.
    layout = args.split or MODE_LAYOUT.get(args.mode)

    w, h = parse_size(args.size)
    views, note = build_views(layout, w, h)

    picam2 = Picamera2(args.camera)
    picam2.configure(picam2.create_video_configuration(
        main={"size": (w, h), "format": "RGB888"},
        raw={"size": SAFE_SENSOR_MODE},
    ))
    picam2.start()

    broker = FrameBroker(views)
    stop = threading.Event()
    worker = threading.Thread(
        target=capture_loop, args=(picam2, broker, views, args.quality, stop),
        daemon=True,
    )
    worker.start()

    page = build_page(views, (w, h), note)
    httpd = ThreadedServer(("0.0.0.0", args.port), make_handler(broker, views, page))

    print(f"streaming {note}")
    print(f"open http://{local_ip()}:{args.port}/   (Ctrl-C to stop)")
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        while not stop.wait(0.5):
            pass
    except KeyboardInterrupt:
        print("\nstopping...")
    finally:
        stop.set()
        _arm_teardown_watchdog()
        httpd.shutdown()
        picam2.stop()
        picam2.close()
        if args.mode:
            try:
                set_camarray_mode(args.i2c_bus, "4ch")
            except SystemExit:
                print("warning: could not restore HAT to 4ch - "
                      "run: sudo i2cset -y %d 0x24 0x24 0x00" % args.i2c_bus)


if __name__ == "__main__":
    main()
