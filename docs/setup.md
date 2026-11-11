# Setting up a new Pi 5 for Shappe Guide

Checked against the working rig on 2026-09-28 (Raspberry Pi 5, Arducam CamArray UC-512 +
4× IMX519, Hailo-10H, RPLIDAR A2M12, 6× A121 radar).

## 1. Camera driver (Arducam Pivariety packages)

```bash
mkdir ~/drivers && cd ~/drivers
wget -O install_pivariety_pkgs.sh https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/install_pivariety_pkgs.sh
chmod +x install_pivariety_pkgs.sh
./install_pivariety_pkgs.sh -p libcamera_dev
./install_pivariety_pkgs.sh -p libcamera_apps
```

## 2. `/boot/firmware/config.txt` (then reboot)

```ini
camera_auto_detect=0
dtoverlay=imx519,cam1
dtoverlay=arducam-pivariety,cam0   # ToF depth camera in CAM/DISP 0. One camera per port:
                                   # no imx519,cam0, and no bare arducam-pivariety (= cam1)
dtparam=i2c_arm=on
```

## 3. System packages (apt: these are NOT on PyPI in a usable form)

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-libcamera libcamera-apps libcap-dev espeak-ng i2c-tools v4l-utils
# (v4l-utils gives media-ctl: how the app finds the ToF's /dev/videoN)
# Hailo-10H runtime: install h10-hailort (5.1.1 on the rig) per Hailo's instructions.
```

The user needs these groups (log out and in after adding):

```bash
sudo usermod -aG dialout,i2c,video,audio,gpio $USER
```

## 4. The app

```bash
cd ~
git clone https://github.com/The-Shappe/lidar.git
cd lidar
python3 -m venv --system-site-packages venv    # REQUIRED: picamera2 and hailo come from apt
./venv/bin/pip install --upgrade pip
./venv/bin/pip install -r requirements.txt     # pinned; do not add numpy==1.25 on top
./venv/bin/pip install --no-deps ArducamDepthCamera==0.1.24
                                               # ToF SDK. --no-deps: it asks for numpy<2 and
                                               # would replace numpy 2.2.4; it runs on 2.2.4
# The models (model/*.hef + model/metadata.yaml) come with the checkout today.
```

## 5. Run it

```bash
./start_lidar.sh                               # by hand, http://<pi-ip>:8000
./deploy/install-service.sh                    # or: as a service that starts at boot (running.md)
```

## 6. Hardware checks

```bash
python -m rplidar /dev/ttyUSB0 256000          # LiDAR (stop the app first: one owner per port)
./venv/bin/python -m app.sensors.radar_utils --check # radar boards present / measured
./venv/bin/python -m tools.device.tof_check    # ToF depth camera, step by step (stop the app first)
hailortcli scan                                # NPU
rpicam-hello --list-cameras                    # camera and its sensor modes
```

CamArray HAT channel mode: I2C bus 11 on the Pi 5, controller 0x24, register 0x24. (Older
notes said bus 10; that numbering was a Pi 4's. `i2cdetect -l` to check.) The HAT forgets
its mode on power-cycle; the app needs 4-in-1 (0x00).

```bash
sudo i2cset -y 11 0x24 0x24 0x02   # camera 0 only
sudo i2cset -y 11 0x24 0x24 0x12   # camera 1 only
sudo i2cset -y 11 0x24 0x24 0x22   # camera 2 only
sudo i2cset -y 11 0x24 0x24 0x32   # camera 3 only
sudo i2cset -y 11 0x24 0x24 0x00   # all 4 (2x2 grid)
```
