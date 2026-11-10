"""Configuration settings for Pi Camera, LiDAR, and Navigation systems"""

import math
import os

_APP_DIR = os.path.dirname(os.path.abspath(__file__))

# The release, as in CHANGELOG.md. The API reports it (/docs, /openapi.json), pyproject.toml
# reads it from here: one place to change.
APP_VERSION = "3.1.0"

HEF_PATH = os.path.join(_APP_DIR, "model", "yolo26m.hef")  # compiled YOLO detector

# ============== CAMERA CONFIGURATION ==============
# The 2x2 picture of all four cameras (the CamArray board's 4-in-1 mode), in the IMX519's
# full-view mode: 2328 x 1748 is the whole 4656 x 3496 sensor binned 2x2, so each camera
# gives its full 66 deg view at 1164 x 874 (2026-11-02, new cameras). The sensor runs at
# 30 fps; the board's 4-in-1 picture comes at half that: 15.7 a second, measured (the
# cropped 1920 x 1080 mode gave 30.7, at 56 deg and 960 x 540 — full view was chosen).
# Until then 1920 x 1080, cycled one camera at a time.
CAMERA_WIDTH = 2328
CAMERA_HEIGHT = 1748
JPEG_QUALITY = 80
TARGET_FPS = 30  # desired pipeline / display FPS
FRAME_INTERVAL = 1.0 / TARGET_FPS

CAMERA_CAPTURE_FPS = (
    TARGET_FPS  # the sensor's, the 2328 x 1748 mode's most (60 was for cycling)
)

STREAM_MAX_WIDTH = 480  # px per sector feed
STREAM_QUALITY = 50  # JPEG quality for the display stream
STREAM_FPS = (
    30  # the display stream's ceiling: every camera's picture is new each frame
)

STREAM_STALL_TIMEOUT_S = 5.0

CAMERA_FRAME_TIMEOUT_S = 5.0

CAMERA_STOP_TIMEOUT_S = 5.0

CAMERA_RETRY_BACKOFF_S = (2.0, 5.0, 10.0)

# The camera board (CamArray HAT): register 0x24 of controller 0x24 on I2C bus 11, set to
# 0x00 (all four cameras in the 2x2 picture) at every camera start — it forgets it on a
# power cycle. The old board's 4-in-1 picture came scrambled, and was worked round by
# cycling one camera at a time, checking every picture and falling back to the front
# camera alone; all removed with the new cameras (2026-11-02).
CAMERA_HAT_I2C_BUS = 11
CAMERA_HAT_ADDRESS = 0x24
# Lens focus of all four cameras, set straight on their lens driver (focus_absolute,
# 0..4095; higher is nearer) at every camera start: libcamera's LensPosition never reaches
# it on this board, so the lens sat at 0 — past infinity — and every camera blurred
# (2026-11-02: sharpness 2-3x up on the side cameras at 1000). 1000 was the sharpest on a
# ceiling ~2 m away, so ~1 m to far is sharp enough to detect. NEEDS VALIDATION outdoors.
# None leaves the lens alone.
CAMERA_FOCUS_CODE = 1000
CAMERA_LENS_DRIVER = "ak7375"  # its name in /sys/class/video4linux/v4l-subdev*/name

# Can each camera actually see? Checked on the detector's pictures, at most once per
# CAMERA_VIEW_CHECK_S per camera (2026-10-07: the right and back cameras gave black
# pictures for a whole road test while /health said the camera was fine).
CAMERA_VIEW_CHECK_S = 1.0
# Mean picture level (0-255) below which a camera sees only black — a covered lens, an
# unplugged sensor (measured 0.0-0.1 on 2026-10-07), or a truly dark place.
CAMERA_DARK_LEVEL = 12.0
CAMERA_DARK_CONFIRM_S = 3.0
# Variance of the picture's Laplacian (grey, every 4th pixel) below which it is blurred:
# a film, dirt or a loose lens. NEEDS VALIDATION on the device — a blank wall close up
# is smooth too, hence the long confirmation; it is shown on /health, never spoken.
CAMERA_BLUR_VAR = 20.0
CAMERA_BLUR_CONFIRM_S = 60.0
# No frame at all for this long: close and reopen the camera, which a stuck stream
# sometimes needs (2026-10-09: the capture froze and never recovered by itself). Each
# further reopen waits twice as long, up to CAMERA_STALL_REOPEN_MAX_S.
CAMERA_STALL_REOPEN_S = 15.0
CAMERA_STALL_REOPEN_MAX_S = 120.0

CAMERA_SIMULATION_ALLOWED = (
    os.environ.get("SHAPPE_SIMULATE_CAMERA", "") == "1"
)  # development only; simulated frames are never evidence

HAILO_RETRY_BACKOFF_S = (2.0, 5.0, 10.0, 30.0)
HAILO_INFER_FAIL_RESET_S = 5.0
# A detector that came up, broke, and then cannot be rebuilt this many times running is on
# a dead device: every NPU worker is stopped, the shared VDevice released, and fresh ones
# started on a new one (2026-10-21). OFF until tried on the bench: with the app running,
# `hailortcli fw-control reset --reset-type nn_core` (then `chip`) and watch it come back.
# Releasing a device that a native call still holds could crash the whole app, guidance
# included, so it is only released once every NPU worker has exited.
HAILO_VDEVICE_RESET = False
HAILO_VDEVICE_RESET_AFTER = 3
HAILO_VDEVICE_RESET_WAIT_S = 15.0
# After an engine broke mid-run, it is rebuilt in this process only once the NPU answers a
# separate one (`hailortcli fw-control identify`, within this). Bench test 2026-10-21: after
# a chip reset, rebuilding on the dead chip blocked inside HailoRT and froze the WHOLE app
# — guidance, voice, web server — for 28 s until the systemd watchdog killed it (with no
# watchdog, for good). The probe is a separate process: if it hangs, only it is killed.
HAILO_PROBE_TIMEOUT_S = 10.0
HAILO_DETECT_RUN_TIMEOUT_MS = 2000
HAILO_SEG_RUN_TIMEOUT_MS = 2000

# ============== DETECTION CONFIGURATION ==============
DETECTION_CONF = 0.5
DETECTION_IOU = 0.7

# ============== LIDAR CONFIGURATION ==============
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_PORT_BY_ID = (
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_"
    "01f49e0b5bd7b2478a89e10184961a3f-if00-port0"
)
LIDAR_PORT_CANDIDATES = tuple(
    p
    for p in (
        os.environ.get("SHAPPE_LIDAR_PORT", "").strip(),
        LIDAR_PORT_BY_ID,
        LIDAR_PORT,
    )
    if p
)
LIDAR_BAUD = 256000
LIDAR_MAX_DISTANCE = 12000  # mm
LIDAR_WARNING_DISTANCE = 500  # mm

LIDAR_MAX_RPM = 1380  # this unit's measured speed at full PWM (~23 Hz)
# Target motor speed: full PWM since 2026-11-05 (was 900 RPM / 15 Hz, the top of the
# A2M12's rated 5-15 Hz). Faster scans, but fewer points per turn at the same sample rate.
LIDAR_RPM = LIDAR_MAX_RPM

LIDAR_ANGLE_OFFSET = 0  # degrees to add to raw angles

LIDAR_MIRROR = (
    False  # verified on this rig; do not flip without a physical left/right check
)

LIDAR_CLUSTER_MIN_POINTS = 3
LIDAR_CLUSTER_WINDOW_MM = 150
LIDAR_PERSIST_DEG = 4.5
LIDAR_PERSIST_MAX_AGE_S = 0.5
# The wearer's own arm, strap or cane handle (2026-10-21): returns nearer than this are
# dropped until they have held in one 10-degree slice for LIDAR_SELF_PERSIST_S — a wall
# walked right up to holds, a swinging arm does not. PROVISIONAL: set from a recorded walk
# (arm swing through the cones). 0 turns it off.
LIDAR_SELF_RANGE_MM = 250.0
LIDAR_SELF_PERSIST_S = 0.5
# Something right at the LiDAR straight ahead (a hand, a person pressed close) hides the way
# from the laser, so the laser's "nothing ahead" is then no evidence of glass (2026-11-09:
# "Blocked by glass", with a hand near the LiDAR and a person ~1 m ahead). Right at it = a
# return under LIDAR_OCCLUDED_MM within LIDAR_FRONT_VIEW_DEG of front, in this scan,
# counted before the self filter drops it. Kept that narrow on purpose: the first version
# (2026-11-09 morning) also counted a front arc that returned little, and a held second,
# and a real pane with open space behind it then went unnamed on the rig.
LIDAR_FRONT_VIEW_DEG = 10.0
LIDAR_OCCLUDED_MM = 250.0
# The ToF sees something solid on the path at the radar's echo, within this, or nearer: the
# echo is that thing (a table under the laser's plane), not glass (2026-11-09). Tight on purpose:
# 400 mm (RADAR_AGREE_MM) unnamed a pane with a table or a person 1 ft behind it.
GLASS_TOF_SAME_MM = 150.0
# A glass claim from an echo this near needs that echo on this share of the front radar's
# frames over this long: a pane that close echoes nearly every frame, clutter does not
# (2026-11-09: -17 dB at 1.0 ft, 1 frame in 3-5, in front of a window blind, said "Blocked by
# glass"). It still blocks, as an obstacle. Too few frames to judge: glass, as before.
GLASS_NEAR_STEADY_FT = 2.0
GLASS_NEAR_STEADY_S = 1.5
GLASS_NEAR_STEADY_FRAC = 0.6

LIDAR_MAX_BACKLOG_S = 0.25
LIDAR_KEEP_BACKLOG_S = 0.1
LIDAR_MEAS_RATE_HZ = 3900.0

LIDAR_SCAN_MAX_GAP_DEG = 20.0

LIDAR_RETRY_BACKOFF_S = (1.0, 2.0, 5.0)

LIDAR_SIMULATION_ALLOWED = (
    os.environ.get("SHAPPE_SIMULATE_LIDAR", "") == "1"
)  # development only; simulated scans are never evidence

LIDAR_SECTORS = {
    "front": (337.5, 22.5),
    "front_right": (22.5, 67.5),
    "right": (67.5, 112.5),
    "back_right": (112.5, 157.5),
    "back": (157.5, 202.5),
    "back_left": (202.5, 247.5),
    "left": (247.5, 292.5),
    "front_left": (292.5, 337.5),
}

# Ordered clockwise from straight ahead — use this when iterating LiDAR sectors.
LIDAR_SECTOR_ORDER = (
    "front",
    "front_right",
    "right",
    "back_right",
    "back",
    "back_left",
    "left",
    "front_left",
)

GUIDANCE_SECTORS = ("front", "right", "back", "left")

GUIDANCE_CONE_DEG = 10.0
GUIDANCE_CONE_CENTRES = {"front": 0.0, "right": 90.0, "back": 180.0, "left": 270.0}

GUIDANCE_CORRIDOR_HALF_WIDTH_MM = 300.0
GUIDANCE_CORRIDOR_MAX_MM = 2500.0  # only applied within this range

CAMERA_SECTORS = ("front", "right", "back", "left")
LIDAR_TO_CAMERA = {
    "front": "front",
    "front_right": "right",
    "right": "right",
    "back_right": "back",
    "back": "back",
    "back_left": "left",
    "left": "left",
    "front_left": "front",
}

RADAR_ENABLED = True

RADAR_BEARINGS = {  # measured per board serial; never inferred from USB port order
    "5AEC021934": 30.0,  # sector  60-120  hub port 1  -> front
    "5AEC022273": 90.0,  # sector 120-180  hub port 6  -> right
    "5AEC021975": 150.0,  # sector 180-240  hub port 2  -> back
    "5AEC021929": 210.0,  # sector 240-300  hub port 3  -> back
    "5AEC021938": 270.0,  # sector 300-360  hub port 7  -> left
    "5AEC022220": 330.0,  # sector   0-60   hub port 5  -> front
}

RADAR_BEARING_STEP_DEG = 60.0

RADAR_BEARING_OFFSET_DEG = 30.0


def radar_board_key(bearing: float) -> str:
    """State-dict key for one physical board."""
    return f"deg{int(round(bearing)) % 360:03d}"


def radar_sector_start(bearing: float) -> float:
    """The SECTOR label a board is known by, from its bearing."""
    return (bearing + RADAR_BEARING_OFFSET_DEG) % 360.0


def radar_sector_label(bearing: float) -> str:
    """Human label for a board's sector, e.g."""
    start = radar_sector_start(bearing)
    return f"{start:.0f}\u2013{start + RADAR_BEARING_STEP_DEG:.0f}\u00b0"


def _radar_ang_diff(a: float, b: float) -> float:
    d = abs(a - b) % 360.0
    return min(d, 360.0 - d)


RADAR_PORT_EXCLUDE = LIDAR_PORT_CANDIDATES  # resolved with realpath (F13)

RADAR_BAUDRATE = 115200

RADAR_START_MM = (
    250  # where each board's measured range starts: nearer is not measured. The
)
# LiDAR covers that, except in its blind arc (LIDAR_BLIND_ARCS_DEG, behind-left)
RADAR_END_MM = 4000  # radar reach: 13 ft, so "Glass ahead" can start at SIMPLE_WALL_APPROACH_FT (was 3000, 2026-10-01)
RADAR_UPDATE_RATE = 6.0

RADAR_MAX_LAG_S = 0.5

RADAR_MIN_RATE_FRACTION = 0.6
RADAR_PROFILE = 5
RADAR_REFLECTOR_SHAPE = "PLANAR"  # GENERIC or PLANAR; PLANAR suits flat panes and walls (on trial 2026-10-05, was GENERIC)
RADAR_SIGNAL_QUALITY = 20.0  # dB; higher listens longer and hears weaker echoes (on trial 2026-10-05, library default 15)
RADAR_THRESHOLD_SENSITIVITY = (
    0.5  # 0-1, higher hears weaker echoes and more stray ones (library default 0.5)
)
RADAR_STALE_S = 1.5  # a sector older than this counts as offline

LIDAR_STALE_S = 1.0
# A scan whose valid returns are under this share of the packets swept — outside
# LIDAR_BLIND_ARCS_DEG, the wearer's body (2026-10-30; it counted them before) — is not a reading:
# the window is blinded (sun, rain, dirt) or failing, and "no return" would read as 40 ft
# of open space (audit NEW-043). PROVISIONAL (2026-10-21): open space also returns little,
# so set it from /lidar/reader's points_per_scan / packets_per_scan measured outdoors.
LIDAR_MIN_VALID_FRACTION = 0.10

CLOCK_FUTURE_TOLERANCE_S = 0.5

RADAR_READY_TIMEOUT_S = 140.0

RADAR_RESTART_WAIT_S = 30.0
# A board with a measured bearing that was missing at start-up is looked for this often,
# only on USB ports whose serial (read from sysfs, no port opened) is that board's, and
# started when it answers (2026-10-21). Until then it stayed missing until a full
# /radar/rediscover, which stops every board. 0 turns the search off.
RADAR_LATE_JOIN_S = 15.0

# A board session with no progress for this long is ended and reconnects. The
# exptool client's own 2 s link timeout runs on the wall clock, and it waits with
# no deadline for a result while a board sends anything else.
RADAR_STALL_S = 5.0  # streaming: no frame
RADAR_CALIBRATE_STALL_S = 30.0  # opening and calibrating; assumed, not timed

RADAR_CALIBRATE_MAX_ENTRIES = 32
RADAR_CALIBRATE_MAX_BODY_BYTES = 16384

RADAR_PRIORITY = False

RADAR_BEAM_H_DEG = 65.0


def radar_beam_deg(bearing: float) -> float:
    """How wide the board facing `bearing` hears (every board here is a ring board)."""
    return RADAR_BEAM_H_DEG


def radar_ring() -> tuple:
    """The ring boards' bearings from RADAR_BEARINGS, in bearing order (used by the live map)."""
    return tuple(sorted({b % 360.0 for b in RADAR_BEARINGS.values()}))


RADAR_CONE_TRUST_MM = GUIDANCE_CORRIDOR_HALF_WIDTH_MM / math.sin(
    math.radians(RADAR_BEARING_OFFSET_DEG + RADAR_BEAM_H_DEG / 2)
)

# Two readings within this margin are treated as the same physical object.
RADAR_AGREE_MM = 400
GLASS_DOOR_MAX_BEARING_DEG = 45.0  # laser returns matching a centred echo within this of ahead are the wall beside a pane; corridor side walls echo at ~62 deg
GLASS_DOOR_WALL_DEPTH_MM = 200.0  # a return farther round that still lies this close to the echo's distance straight ahead is the wall the pane sits in, not a side wall
GLASS_LASER_BEYOND_MM = 250.0  # a laser return farther than an echo by more than this cannot be what the radar heard (radar/laser timing, board mount offset)
GLASS_BOARD_VIEW_MARGIN_DEG = 10.0  # a board hears a little past the edge of its beam
GLASS_SAME_ECHO_MM = (
    60.0  # two front boards ranging within this of each other heard the same thing
)
GLASS_GRAZING_DEG = 75.0  # a surface the laser sees more edge-on than this sends the radar next to nothing back, so it cannot explain an echo
# A glass door at the end of a corridor (walk_policy._door_in_wall_ft, 2026-11-06): an echo
# both front boards hear is explained only by a surface the laser sees within this of
# square-on — what faces the sensor echoes; a corridor's walls seen obliquely do not. 40, not
# 45: a 1.2 m corridor's walls lie at 40-45 deg close to its door and broke the door's wall.
GLASS_FACING_DEG = 40.0
# A front board's FARTHER echoes count too, not only its nearest (2026-11-02): in a narrow
# corridor the nearest is the side wall at the beam's edge, which the laser explains, and the
# pane behind it was never looked at — "Walk straight" with glass 3 ft ahead. False: nearest only.
WALK_GLASS_ALL_ECHOES = True
# A farther echo at twice the range of one this board's laser explains, within this, is that
# surface's double bounce, not a pane.
GLASS_GHOST_TOL_MM = 150.0

RADAR_GLASS_GAP_MM = 600

RADAR_SECTORS = ("front", "right", "back", "left")

RADAR_CARDINAL_CENTRES = {"front": 0.0, "right": 90.0, "back": 180.0, "left": 270.0}


def _radar_cardinal_arc_sectors():
    step, half_beam = RADAR_BEARING_STEP_DEG, RADAR_BEAM_H_DEG / 2.0
    mount = [
        (RADAR_BEARING_OFFSET_DEG + i * step) % 360.0
        for i in range(int(round(360.0 / step)))
    ]
    out = {}
    for name, centre in RADAR_CARDINAL_CENTRES.items():
        boards = [b for b in mount if _radar_ang_diff(b, centre) < half_beam + 22.5]
        names = []
        for sec, (lo, hi) in LIDAR_SECTORS.items():
            width = (hi - lo) % 360.0 or 360.0
            steps = int(width * 4)
            covered = sum(
                1
                for k in range(steps)
                if any(
                    _radar_ang_diff((lo + k / 4.0) % 360.0, b) <= half_beam
                    for b in boards
                )
            )
            if covered / 4.0 >= width / 2.0:
                names.append(sec)
        out[name] = tuple(names)
    return out


RADAR_CARDINAL_ARC_SECTORS = _radar_cardinal_arc_sectors()


WALK_BLOCK_FT = (
    4.0  # front nearer than this = blocked (~1.2 m; was 6.5, decided 2026-10-01)
)
WALK_MIN_TURN_FT = 2.0  # a turn target must have at least this much room;
# below it there is nowhere to go and the answer is stop
WALK_FUSION_GLASS_OVERRIDE = True

LIDAR_NO_RETURN_FT = 40.0
RADAR_NO_RETURN_FT = round(
    RADAR_END_MM / 304.8, 1
)  # a silent radar counts as clear to its own reach

WALK_TURN_MARGIN_FT = 1.5

WALK_TURN_ORDER = ("left", "right", "back")

WALK_TURN_HYSTERESIS_FT = 0.8


WALK_TURN_HOLD_S = 4.0

# Room enough both ways (each side at least this): the turn goes AWAY from the side the laser
# measured the obstacle on (2026-11-02). It was the fixed order / the both-sensors side, so in
# open space the rig always said "Turn right", even with the obstacle on the right — and a
# wearer who veers rather than pivots veered into it. Not within WALK_TURN_AWAY_MIN_DEG of
# dead ahead, nor for a block only the radar measured (no bearing). A turn being held is kept.
WALK_TURN_AWAY_FROM_OBSTACLE = True
WALK_TURN_AMPLE_FT = 6.0
WALK_TURN_AWAY_MIN_DEG = 3.0

WALK_MEASURED_MEMORY_S = 3.0

WALK_GLASS_GAP_FT = 2.0  # LiDAR farther than radar by this much = glass (~0.6 m)

HEADING_ENABLED = True
HEADING_BIN_DEG = 1.0  # range profile resolution used for scan matching
HEADING_INTERP_MAX_GAP_DEG = (
    3.0  # never interpolate a bin across a wider gap in the scan
)
HEADING_INTERP_MAX_JUMP_MM = 300.0  # ...nor across a range step this big (two objects)
HEADING_FIT_MAX_RESEEDS = (
    4  # times the yaw + translation fit may restart from its own answer
)
HEADING_MAX_YAW_STEP_DEG = 25.0  # widest rotation searched between two scans
# (25 deg at 15 Hz = 375 deg/s — far beyond a walk)
HEADING_MATCH_MIN_BINS = 40  # too few overlapping bins = no honest estimate
HEADING_MATCH_MAX_RANGE_MM = 8000  # far returns are noisy; match on nearer structure
HEADING_WINDOW_S = 2.5  # smoothing window for yaw rate / speed
HEADING_NOMINAL_SPEED_MPS = 1.2  # assumed pace when odometry cannot measure one
HEADING_MAX_SPEED_MPS = 2.5  # anything faster is a bad fit, not a pedestrian
HEADING_MAX_LATERAL_MPS = 1.5
HEADING_MIN_MOVING_MPS = 0.25  # below this the user is standing still
HEADING_TRIM_FRACTION = 0.3  # worst residuals dropped before the velocity refit
# (people walking past must not become "my motion")
HEADING_MIN_MATCH_QUALITY = 0.15  # below this the scan match is not trusted
HEADING_SPEED_SE_LIMIT_MPS = 0.35

HEADING_MIN_FORWARD_SENS = 0.20

HEADING_SENS_SMOOTH_BINS = 9
HEADING_MAX_INCIDENCE_SEC = 3.0
HEADING_SENS_JUMP_FRACTION = 0.1
HEADING_STATE_STALE_S = LIDAR_STALE_S

GYRO_ENABLED = (
    False  # gyro unplugged 2026-10-23; set True when it is back on the SPI bus
)

GYRO_TRANSPORT = "spi"  # "spi" or "i2c"
GYRO_SPI_BUS = 0
GYRO_SPI_DEVICE = 0  # CE0 -> /dev/spidev0.0
GYRO_SPI_HZ = 1_000_000
GYRO_I2C_BUS = 1  # only used when GYRO_TRANSPORT == "i2c"
GYRO_I2C_ADDRESS = 0x68  # 0x69 if AD0 is pulled high

GYRO_RANGE_DPS = 500
GYRO_ACCEL_RANGE_G = 4

GYRO_MOUNTING_CONFIRMED = (
    False  # gyro yaw stays out of steering until axis and sign are commissioned
)

GYRO_YAW_AXIS = "z"  # "x", "y" or "z" — the axis pointing UP
GYRO_YAW_SIGN = -1.0  # flip to +1.0 if a right turn reads negative

GYRO_SAMPLE_HZ = 100.0
GYRO_BUFFER_S = 12.0
GYRO_MIN_COVERAGE = 0.75

GYRO_STALE_S = 0.5
GYRO_RECONNECT_S = 1.0  # wait between reconnect attempts after a drop
GYRO_MAX_READ_ERRORS = 15  # consecutive failures before the link is dead

GYRO_BIAS_GAIN = 0.02
GYRO_BIAS_MAX_DPS = 8.0  # a genuine zero-rate offset is under a few
GYRO_BIAS_MAX_ERR_DPS = 6.0
# Startup calibration: the average gyro reading while still IS the zero offset.
GYRO_CALIBRATION_SAMPLES = 400

GYRO_CAL_MAX_STD_DPS = 2.0

GYRO_TRUST_ERR_DPS = 0.75
GYRO_TRUST_SAMPLES = 150  # at ~15 Hz, about 10 s of steady agreement
GYRO_RECAL_ON_STILL = True
GYRO_RECAL_MIN_INTERVAL_S = 60.0
GYRO_RECAL_STILL_S = 2.0  # how long "still" must hold before re-zeroing
GYRO_RECAL_MAX_YAW_DPS = 0.5

GYRO_SEED_MOTION_FIT = True

GYRO_MAX_RATE_DPS = 400.0

GYRO_STEP_DETECT = True
GYRO_STEP_WINDOW_S = 1.5  # rolling window the |a| variation is measured over
GYRO_STEP_STD_G = 0.06  # standing still is well under this even handheld
GYRO_STEP_PEAK_G = (
    0.10  # |a| excursion above the resting mean that counts as a footfall
)
GYRO_STEP_REFRACTORY_S = 0.25  # no second step inside this (240 steps/min ceiling)

GYRO_STEP_UPWARD_ONLY = True

GYRO_STEP_REF_TAU_S = 6.0

GYRO_CADENCE_WINDOW_S = 5.0
GYRO_CADENCE_STALE_S = 2.5

GYRO_TILT_WARN_DEG = 12.0

GYRO_STRIDE_DEFAULT_M = 0.71  # typical adult walking stride; a seed, not a spec
GYRO_STRIDE_MIN_M = 0.35  # clamp: below this it is a shuffle or a miscount
GYRO_STRIDE_MAX_M = 1.10  # clamp: above this it is a run or a missed step
GYRO_STRIDE_LEARN_TAU_S = 30.0

GYRO_GAIT_CONFIRMED = (
    False  # no stride is learned until footfalls are counted on the worn harness
)


DRIFT_ENABLED = True

DRIFT_ANALYZE_HZ = 8.0

DRIFT_BODY_RADIUS_MM = 400.0

DRIFT_LOOKAHEAD_S = 6.0  # how far into the future the path is projected
DRIFT_STEP_S = 0.2  # projection time step (30 steps at 6 s)
DRIFT_MAX_RANGE_MM = 8000  # returns beyond this are not a near-term concern

DRIFT_ALERT_TTC_S = 5.0  # start nudging below this
DRIFT_URGENT_TTC_S = 2.5  # firm instruction below this
DRIFT_RELEASE_TTC_S = 7.0  # hysteresis: cue clears only above this, so a

DRIFT_FAN_DEG = 45.0  # search +/- this either side of current motion
DRIFT_FAN_STEP_DEG = 2.5
DRIFT_CLEAR_TTC_S = 7.0  # a candidate heading must be clear this long
DRIFT_IMPROVE_S = 2.0
DRIFT_SLIGHT_DEG = 8.0  # correction below this = "bear slightly"
DRIFT_BEAR_DEG = 20.0  # below this = "bear"; above = "turn"

DRIFT_WALL_NEAR_FT = 1.6  # closer than this on one side = too close (~0.5 m)
DRIFT_WALL_RELEASE_FT = 2.5  # hysteresis: alert clears only past this, so a
# user hovering at 1.6 ft is not told repeatedly
DRIFT_WALL_HORIZON_S = 1.5  # how far ahead the closing test looks
DRIFT_WALL_CLOSING_FT = 0.1  # perpendicular distance must shrink by this much
# over the horizon to count as "going toward it"
DRIFT_WALL_SIDE_MIN_DEG = 45.0  # side band: nearest point must lie between
DRIFT_WALL_SIDE_MAX_DEG = 135.0  # these bearings to count as left/right

DRIFT_WALL_ROOM_FT = 3.0  # opposite side needs at least this much room

# Open-space veer (case B).
DRIFT_VEER_DEG = 15.0  # cumulative heading change that counts as veering
DRIFT_VEER_WINDOW_S = 6.0  # over this rolling window
DRIFT_VEER_RATE_DPS = 4.0  # or a sustained turn rate this high

DRIFT_CONFIRM_SCANS = 3  # agreeing scans before a cue is issued
DRIFT_REPEAT_S = 3.0  # same cue not repeated inside this window
# (an ESCALATION in urgency always fires)

DRIFT_SURFACE_MIN_POINTS = 8
DRIFT_SURFACE_MIN_SPAN_MM = 600.0  # shorter than this is an object, not a wall
DRIFT_SURFACE_MAX_RESIDUAL_MM = 120.0  # RMS off the fitted line to still be flat
DRIFT_SURFACE_GAP_MM = 400.0  # range jump that splits two surfaces

# ============== NAVIGATION THRESHOLDS ==============
DANGER_THRESHOLD_MM = 800  # Dangerous proximity threshold
SAFE_THRESHOLD_MM = 1000  # Safe distance threshold
WALL_WARNING_DISTANCE = 2440  # Distance threshold for wall warning (~8 feet)

GUIDANCE_DANGER_MM = 800  # imminent — stop
GUIDANCE_WARNING_MM = 1500  # close — caution
GUIDANCE_SAFE_MM = 2000  # be aware; clear above this
GUIDANCE_AWARE_MM = 3000  # furthest distance worth mentioning at all

# ============== CRITICAL OBJECTS ==============
CRITICAL_OBJECTS = [
    "car",
    "truck",
    "bus",
    "bicycle",
    "motorcycle",
    "person",
    "pedestrian",
    "scooter",
]

SEG_ENABLED = True
SEG_HEF_PATH = os.path.join(_APP_DIR, "model", "best.hef")
SEG_TARGET_FPS = 10.0
SEG_ALWAYS = False  # segmentation runs only while its debug view is open
DETECTION_FRONT_EVERY_PASS = True
# A side camera's detection at most this often (all three share it, the one longest undetected
# first, so each is re-detected within ~1.3 s): only the front's detections reach the voice,
# and a side's ~40 ms of NPU ahead of it cost the front pictures (2026-10-30). 0: every pass.
DETECTION_SIDE_MIN_INTERVAL_S = 0.4
SEG_VIEW_FPS = 2
SEG_VIEW_PRIORITY = True

STARTUP_SETTLE_S = 2.0
STARTUP_OPTIONAL_WAIT_S = 15.0
STARTUP_LIDAR_ABSENT_S = 20.0

STARTUP_MIN_SAMPLES = 6

STARTUP_STABLE_FT = 1.5

STARTUP_STABLE_MAX_WAIT_S = 5.0

STARTUP_READY_CUE = "Ready."

# How long a sentence keeps the voice busy until pi_voice has timed the real ones
# (simple_guidance.measure_airtimes): BASE + PER_CHAR * letters + PER_COMMA * commas.
# Fitted 2026-10-23 to espeak-ng at 170 wpm, never short for any of the 309 sentences
# (mean +0.17 s); the 2.8 words/s it replaced was short for 308, by up to 0.83 s.
NOTIFY_AIRTIME_BASE_S = 0.71
NOTIFY_AIRTIME_PER_CHAR_S = 0.049
NOTIFY_AIRTIME_PER_COMMA_S = 0.26

BEV_ENABLED = True

BEV_RANGE_MM = 8000

BEV_CELL_MM = 50

BEV_L_HIT = 0.85
BEV_L_FREE = 0.40
BEV_L_MAX = 3.5
BEV_L_MIN = -2.0

BEV_OCC_THRESH = 0.5

BEV_DECAY = 0.93

BEV_MIN_MOTION_QUALITY = 0.35

BEV_MAX_STEP_MM = 400.0
BEV_MAX_STEP_DEG = 25.0

BEV_PROFILE_BIN_DEG = 1.0

BEV_PROFILE_MIN_POINTS = 1

BEV_RADAR_STAMP_ENABLED = True
BEV_RADAR_ARC_THICK_MM = 150  # drawn thickness of the stamped arc

BEV_MIN_CLUSTER_CELLS = 3

BEV_WALL_MIN_LEN_MM = 1000
BEV_WALL_MAX_THICK_MM = 250
BEV_POLE_MAX_MM = 250
BEV_BODY_MAX_MM = 800

BEV_GAP_MIN_MM = 700
BEV_GAP_MAX_MM = 1200

BEV_GAP_CLEAR_MM = 600

BEV_GAP_MAX_BEARING_DEG = 60.0

BEV_MIN_WEDGE_DEG = 15.0


BEV_SHAPE_IN_HAZARDS = True

BEV_SHAPE_MATCH_DEG = 20.0
BEV_SHAPE_MATCH_MM = 400.0

BEV_EXTRACT_HZ = 5.0

# ---- Moving things in the LiDAR scan (utils/lidar_movers.py, added 2026-10-07)
LIDAR_MOVERS_ENABLED = True
LIDAR_MOVERS_SPEAK = False  # False: shown and logged as "would say", not spoken
LIDAR_MOVERS_MIN_RANGE_MM = 300  # nearer is the wearer's own arm or cane
LIDAR_MOVERS_MAX_RANGE_MM = 8000  # Neighbouring returns farther apart than BREAK_MM + BREAK_PER_M x range are two things.
LIDAR_MOVERS_BREAK_MM = 200.0
LIDAR_MOVERS_BREAK_PER_M = 0.06
LIDAR_MOVERS_MAX_GAP_DEG = 3.0  # an angular gap wider than this also splits
LIDAR_MOVERS_MIN_POINTS = 3
LIDAR_MOVERS_MAX_WIDTH_M = 2.0  # wider is a wall: not followed
LIDAR_MOVERS_PERSON_MAX_WIDTH_M = 0.9  # wider, or faster than VEHICLE_SPEED: "Vehicle"
LIDAR_MOVERS_VEHICLE_SPEED_MPS = 3.0
LIDAR_MOVERS_GATE_M = 0.5  # a blob this far from where a track was heading is another
LIDAR_MOVERS_WINDOW_S = 1.2  # each track's speed is fitted over this
LIDAR_MOVERS_MIN_SAMPLES = 6
LIDAR_MOVERS_MIN_SPAN_S = 0.6
LIDAR_MOVERS_LOST_S = 0.5  # a track unseen this long is dropped
# Moving = its own speed (the user's taken off) is at least MIN_SPEED, raised by
# NOISE_PER_M per metre of range (scan jitter and turn error grow with range).
LIDAR_MOVERS_MIN_SPEED_MPS = 0.4
LIDAR_MOVERS_NOISE_PER_M = 0.07
LIDAR_MOVERS_CONFIRM_SCANS = 3
# The user's own motion comes from the still things around them: at least this many,
# agreeing within EGO_MAX_RMS. Without that, any walk up to WALK_MAX is allowed for.
LIDAR_MOVERS_EGO_MIN_TRACKS = 4
LIDAR_MOVERS_EGO_MAX_RMS_MPS = 0.2
LIDAR_MOVERS_WALK_MAX_MPS = 1.8
# Warn when it will pass within MISS_M (vehicles: SIMPLE_VEHICLE_MISS_M) inside HORIZON_S.
LIDAR_MOVERS_HORIZON_S = 5.0
LIDAR_MOVERS_MISS_M = 1.0
LIDAR_MOVERS_HEAD_ON_DEG = (
    30.0  # ahead and heading this straight at the user: "coming toward you"
)
LIDAR_MOVERS_CAMERA_MATCH_DEG = 10.0  # front camera box this close in bearing names it
LIDAR_MOVERS_REPEAT_S = 6.0  # the same "would say" is not logged again inside this

# ---- Moving things only the radar hears (utils/radar_movers.py, added 2026-10-07)
# Fills the LiDAR's gaps: its blind arc, and things above or below its plane. An echo
# the LiDAR also ranges (RADAR_AGREE_MM, in that board's beam) is left to the LiDAR.
RADAR_MOVERS_ENABLED = True
RADAR_MOVERS_GATE_M = 0.3  # an echo this far from where a track was heading is another
RADAR_MOVERS_WINDOW_S = 1.5  # closing speed is fitted over this
RADAR_MOVERS_MIN_SAMPLES = 5
RADAR_MOVERS_MIN_SPAN_S = (
    0.6  # with CONFIRM_FRAMES: ~1.1 s to a warning, inside a 4 m reach
)
RADAR_MOVERS_LOST_S = 0.7  # an echo unheard this long is dropped
RADAR_MOVERS_MIN_CLOSING_MPS = 0.4  # its own closing speed, the user's walk taken off
RADAR_MOVERS_CONFIRM_FRAMES = 2
RADAR_MOVERS_MAX_TURN_DPS = 10.0  # turning faster sweeps the beams across walls: paused

# ---- Moving things followed with a Kalman filter (utils/motion_tracker.py, added 2026-10-13)
# The LiDAR blobs lidar_movers.segment cuts cleanly, and each person or vehicle the front
# camera names (range from its box height, bearing from its place in the frame), join the
# track they most likely came from. A track is a constant-velocity Kalman filter on the
# ground: position, velocity and how sure of both. It feeds the route planner and the
# mover warning (spoken only with LIDAR_MOVERS_SPEAK, as before).
MOTION_TRACKER_ENABLED = True
MOTION_TRACKER_WARN = True  # its closest-approach warning stands in for lidar_movers'
MOTION_GATE_CHI2 = 9.21  # 99 % for two degrees of freedom
MOTION_GATE_M = 1.0  # and no farther than this from where a track should be...
MOTION_GATE_VEHICLE_M = 2.0  # ...or this for a vehicle (a camera's range is rough)
MOTION_GATE_NEW_EXTRA_M = (
    1.0  # a track with under CONFIRM_HITS sightings: this much more
)
MOTION_LOST_S = 0.8  # a track unseen this long is dropped
MOTION_ACCEL_PERSON = 1.2  # m/s^2 of random acceleration a person's model allows
MOTION_ACCEL_ALONG = 1.5  # a moving vehicle's, along its heading...
MOTION_ACCEL_ACROSS = 0.4  # ...and across it: a car brakes, it does not slide sideways
MOTION_CONFIRM_HITS = 6
MOTION_CONFIRM_S = 0.6
MOTION_SIGNIFICANCE = (
    3.0  # moving: speed at least this many sigma, in its own direction
)
MOTION_VEHICLE_STEADY_S = (
    0.8  # a vehicle's heading must hold within STEADY_DEG this long
)
MOTION_VEHICLE_STEADY_DEG = 20.0  # (a track that swapped between two cars swings)
MOTION_VEHICLE_MIN_TRAVEL_M = 1.0  # and it must have gone this far in the last second
MOTION_CAMERA_SECTORS = ("front",)  # the cameras whose people and vehicles are followed
MOTION_CAMERA_RANGE_FRAC = 0.15  # a box-size range is this far off (1 sigma)...
MOTION_CAMERA_BEARING_DEG = 2.0  # ...its bearing this
MOTION_CAMERA_MAX_AGE_S = 0.4  # a detector frame older than this is not used
MOTION_EGO_SD_MPS = 0.6  # the user's own pace when nothing measures it: this unsure
MOTION_PREDICT_DRIFT_PERSON = 0.1  # m/s: how fast doubt about a prediction grows
MOTION_PREDICT_DRIFT_VEHICLE = 0.3
MOTION_PREDICT_MAX_SD_M = 0.8  # past this a prediction says too little to act on

# ---- The route: the best way through what is around the user (utils/route_planner.py,
# added 2026-10-13). Everything sensed becomes points on the ground; for each direction
# from 90 deg left to 90 deg right the planner measures how far a lane as wide as the user
# stays clear, now and against where moving things will be, and picks the smallest turn
# that gives ROUTE_NEED_M — at walking pace, at half pace, or not at all (wait, or step
# aside from something coming at the user).
ROUTE_PLANNER_ENABLED = True
ROUTE_PLANNER_SPEAK = (
    False  # False: its turn side and steering are shown and logged as "would say"
)
ROUTE_LANE_HALF_M = 0.30  # half the lane: the user's half-width and a margin
ROUTE_LANE_TIGHT_M = 0.22  # the user's half-width alone, when nothing wider fits
ROUTE_REACH_M = 4.5
ROUTE_NEED_M = (
    2.4  # clear walking wanted at walking pace (half pace: half, at least 1.2)
)
ROUTE_STEP_DEG = 5
ROUTE_MAX_TURN_DEG = 90
ROUTE_STEER_MAX_DEG = 35  # a way this far off straight is "turn slightly"
ROUTE_AROUND_M = 1.0  # no way 1 m clear anywhere ahead: turn around
ROUTE_LOOKAHEAD_S = 4.0
ROUTE_STEP_ASIDE_M = 0.6  # room needed to step aside from something coming
ROUTE_HOLD_S = 3.0  # the way chosen is kept this long while it stays clear
# Arcs (clockwise degrees) the LiDAR cannot see from where it is worn: no return there
# is NOT open space. Measured on the rig as the back-left arc (tools/sim/movers_sim.BLIND);
# re-measure after moving the LiDAR (a head on the shoulder hides ~245-295).
LIDAR_BLIND_ARCS_DEG = ((170.0, 290.0),)

# ---- The ToF depth camera (Arducam, 240 x 180 distances) merged with the front camera
# (utils/tof_camera.py, utils/depth_fusion.py; added 2026-10-13, before the sensor is
# bought). Both are pinhole cameras with their own numbers, so each depth pixel is placed
# on the front camera's picture with its own distance:
#   1. P  = d * K_tof^-1 [u, v, 1]   the point the ToF pixel (u, v) measured
#   2. P' = R * P + t                that point as the camera sees it
#   3. [u', v'] = K_cam * P' / Z'    the camera pixel it lands on
# Then every front detection takes the distance of the nearest surface inside its box.
# Off: nothing starts and nothing changes. The ToF goes on the Pi 5's CAM0 port (the
# CamArray is on CAM1): /boot/firmware/config.txt needs dtoverlay=arducam-pivariety,cam0
# there instead of dtoverlay=imx519,cam0, and the SDK: pip install --no-deps
# ArducamDepthCamera==0.1.24 (its metadata asks for numpy<2; it runs on 2.2.4, 2026-11-03).
# On since 2026-11-03 (the sensor in CAM0 streams 31 fps: tools/device/tof_check.py). It runs in
# its own process; live view at /tof/live. Nothing the user hears changes until
# TOF_USE_DISTANCE, which waits for the bench calibration (tools/device/tof_calibrate.py).
TOF_ENABLED = True
TOF_USE_DISTANCE = (
    False  # False: the ToF's distance is only shown beside the box-size one
)
# The /dev/videoN the SDK opens (not the CSI port number). None: the node media-ctl links
# to the arducam-pivariety sensor, which moves with probe order once both ports are in use.
TOF_VIDEO_INDEX = None
TOF_RANGE_MM = (
    4000  # the SDK's range setting (2000 or 4000: beyond 2 m that one wraps to near)
)
TOF_FRAME_TIMEOUT_MS = 500
TOF_RETRY_BACKOFF_S = (2.0, 5.0, 10.0)
TOF_MIN_CONFIDENCE = (
    30.0  # depth pixels the sensor is less sure of than this are dropped
)
TOF_MIN_MM = 150.0
TOF_MAX_MM = 4000.0
# Is the SDK's distance along each pixel's ray (True) or along the lens axis? Bench check:
# tools/device/tof_calibrate.py wall, pointed square-on at a flat wall.
TOF_DEPTH_IS_RADIAL = True
TOF_PAIR_MAX_S = (
    0.06  # a camera picture uses the ToF frame captured nearest it, within this
)
TOF_BUFFER_FRAMES = 8
TOF_LIVE_MAX_FPS = 15  # /tof/live: at most this many frames a second (~170 KB each)
# Until tools/device/tof_calibrate.py writes TOF_CALIBRATION_FILE (checkerboard on the bench),
# both lenses are taken as ideal and placed as in the 3D design ("aimed apart").
TOF_CALIBRATION_FILE = os.path.join(_APP_DIR, "calibration", "tof_calibration.json")
TOF_CALIBRATION_PAIRS_DIR = os.path.join(_APP_DIR, "calibration", "tof_pairs")
TOF_SIZE_PX = (240, 180)
TOF_DIAGONAL_FOV_DEG = 70.0  # Arducam's figure; on the 4:3 sensor about 58.5 x 45.6 deg
# The detector's front picture: the IMX519's 66 deg across all 4656 px, the whole sensor
# (the 2328 x 1748 mode: no crop) halved twice to the 1164 x 874 quadrant: about 66 x 51 deg.
# Until 2026-11-02 a 3840 x 2160 centre crop (ScalerCrop, measured 2026-10-08): 56.3 x 33.5
# deg on a 960 x 540 quadrant. (SIMPLE_CAMERA_HFOV_DEG, below, is derived from these.)
FUSION_CAMERA_LENS_HFOV_DEG = 66.0
FUSION_CAMERA_SENSOR_PX = (4656, 3496)
FUSION_CAMERA_CROP_PX = (4656, 3496)
# Where the ToF sits against the camera, in the camera's frame (x right, y down, z ahead):
FUSION_TOF_OFFSET_M = (0.0, 0.038, 0.006)  # 3.8 cm below, 6 mm ahead
FUSION_TOF_DOWN_DEG = 20.0  # aimed this much further down (camera 10, ToF 30)
FUSION_CAMERA_DOWN_DEG = (
    10.0  # the camera below level: turns a 3D point into a distance along the ground
)
# A box's distance: the nearest surface inside it (not the wall behind a person). The box
# is first trimmed (sides, top, bottom: the ground at its feet), then the depth points are
# grouped where they are no more than GROUP_M apart; the nearest group holding at least
# MIN_SHARE of them (and MIN_POINTS) is the thing.
TOF_BOX_TRIM = (0.15, 0.10, 0.20)
TOF_BOX_GROUP_M = 0.30
TOF_BOX_MIN_POINTS = 6
TOF_BOX_MIN_SHARE = 0.20
# The depth picture aligned to the camera's (DepthFrame.depth_map, /tof/rgbd.jpg,
# /tof/aligned.npy): one distance per block of this many x this many camera pixels. A ToF
# pixel spans ~4.3 camera pixels (1164 px over 66 deg against 240 px over 58.5 deg), so 4
# keeps all its detail; a block between ones that got a reading takes the nearest of theirs.
TOF_ALIGNED_CELL_PX = 4
TOF_RANGE_SD_M = 0.05  # how far off a ToF distance is (1 sigma), plus...
TOF_RANGE_SD_FRAC = (
    0.02  # ...this much of it (the motion tracker's camera error ellipse)
)

# ---- Hazards on the ground ahead, from the ToF, confirmed by the front camera
# (utils/ground_hazards.py, 2026-11-06): stairs up and down, single steps (kerbs), drops,
# holes, poles, low obstacles under the LiDAR's plane and overhanging ones a cane passes
# under. Found and shown with TOF_HAZARDS_ENABLED (/tof/hazards, /tof/rgbd.jpg); SPOKEN
# only with TOF_HAZARDS_SPEAK — until a test walk has judged them they are logged as
# "would say", as the LiDAR movers are.
TOF_HAZARDS_ENABLED = True
TOF_HAZARDS_SPEAK = False  # False: shown and logged as "would say", not spoken
TOF_HAZARD_HZ = 10.0  # analyses a second (each on the newest ToF frame; ~15 ms each)
TOF_HAZARD_FRESH_S = 0.5  # the guidance uses an analysis no older than this
# The ground, fitted on every frame: these only steer the fit, it measures the rest.
TOF_MOUNT_HEIGHT_M = 1.45  # the front camera above the ground when worn
TOF_MOUNT_HEIGHT_TOL_M = (
    0.45  # a level surface further from that is not taken as the ground
)
TOF_GROUND_MAX_TILT_DEG = 15.0  # the body's sway and the mount's error, at most
TOF_GROUND_INLIER_M = 0.03  # a point this near the plane is on it
TOF_GROUND_MIN_POINTS = 300  # fewer on the plane: no ground seen
# The walking corridor: half the body's width and a margin, either side of straight ahead.
TOF_CORRIDOR_HALF_M = 0.40
# The ground's height along the corridor, every BIN ahead, out to PROFILE_MAX (the ToF reads
# to 4 m; past ~3.6 m the floor is at a grazing angle and thin).
TOF_PROFILE_BIN_M = 0.05
TOF_PROFILE_MIN_POINTS = 6
TOF_PROFILE_MAX_M = 3.6
TOF_PROFILE_JUMP_M = 0.035  # neighbouring bins further apart than this: a new surface
# A step: a sharp change in the ground's height of at least MIN and at most MAX (more is a
# drop down, or an obstacle up). Stairs: two or more steps of a stair's rise and run.
TOF_STEP_MIN_M = 0.07
TOF_STEP_MAX_M = 0.25
TOF_STAIR_RISE_M = (0.10, 0.23)
TOF_STAIR_RUN_M = (0.18, 0.60)
# Above the ground by this: part of an obstacle (5 cm cells, joined).
TOF_OBSTACLE_MIN_H_M = 0.08
TOF_OBSTACLE_MIN_POINTS = 20
TOF_LOW_MAX_H_M = 0.60  # an obstacle no taller: "low" (under the LiDAR's plane)
TOF_OVERHANG_MIN_H_M = (
    0.50  # nothing of it lower than this: overhanging (the cane passes under)
)
TOF_POLE_MAX_WIDTH_M = 0.30  # a pole: this narrow, from the ground up...
TOF_POLE_MIN_H_M = 0.80  # ...this high or out of the top of the ToF's view
# Said from WARN_M; a drop or stairs down within STOP_M is said as "..., stop" (level 2).
# The ToF sees the ground from ~1.1 m ahead (its view starts 7 deg below level, 30 deg
# down at its centre), so STOP_M is the last distance at which the edge is still seen.
TOF_HAZARD_WARN_M = 3.5
TOF_HAZARD_STOP_M = 1.6
TOF_HAZARD_CONFIRM = (3, 5)  # seen in 3 of the last 5 analyses (~0.3 s)
# A wall, by the ToF (utils/ground_hazards, 2026-11-10): the tall thing standing on the path
# is a wall when, seen from above, its surface runs flat across the path. Each frame turns
# the thing's points into a top-down line, one sample per few ToF columns (the median of
# every point in them, so the sensor's noise averages out), and grows it left and right from
# the walking path while each next sample stays within TOF_WALL_FLAT_M of the line so far:
# a corner of two walls is two such arms. A wall: at least TOF_WALL_MIN_WIDTH_M of it, or
# the ToF's whole view edge to edge and TOF_WALL_MIN_SPAN_M (close up, the view is ~1.1 m
# wide at 1 m), and taller than TOF_WALL_MIN_H_M or out of the top of the view. A chair, a
# bin, a person, a pillar, a box: narrower, or not flat. A wardrobe that wide reads as a
# wall: the same turn. Seen in TOF_WALL_CONFIRM[0] of the last [1] analyses.
# The guidance then names an unnamed block "Blocked by wall, turn ..." (and "Wall ahead"),
# when the ToF's wall is within TOF_WALL_MATCH_FT of the laser's front range and the camera
# names nothing there: a window blind was "Obstacle in front, turn left" (2026-11-10). Names
# only: the block, the turn and every distance are the laser's and the walk policy's.
# TOF_WALL_SPEAK: said (on since 2026-11-10, the tester's call: it renames only). False logs
# it as "would say" instead (tools/field/walk_replay.py --turn-on tof-wall shows what a walk would
# have heard); tools/field/label_report.py checks it against the testers' Wall labels.
TOF_WALL_SPEAK = True
TOF_WALL_FLAT_M = 0.05
TOF_WALL_MIN_WIDTH_M = 1.2
TOF_WALL_MIN_SPAN_M = 0.6
TOF_WALL_MIN_H_M = 1.2
TOF_WALL_CONFIRM = (2, 3)
TOF_WALL_MATCH_FT = 1.5
# With the camera looking and seeing nothing there, a fall (drop, stairs or step down,
# hole) at least this deep is still said, on the ToF alone; anything else is not.
TOF_FALL_ALONE_M = 0.15
# The camera's confirmation: its picture within MAX_AGE_S of the ToF frame, searched this
# far around where the hazard lands on it (the design calibration is ~26 px off).
TOF_CAMERA_MAX_AGE_S = 0.3
TOF_CAMERA_MARGIN_PX = 40
TOF_CAMERA_EDGE_DENSITY = (
    0.04  # edge pixels, as a share of the area: an outline is there
)

TRAFFIC_SIGN_STALE_S = 5.0

ENV_TRUST_ENABLED = False  # off: it called a lit office and a dead camera "fog"

ENV_ANALYZE_HZ = 0.2  # every 5 s

ENV_IDENTITY_MIN_VISIBILITY = 0.45

ENV_STALE_S = 30.0

SPEECH_LOG_ENABLED = True
SPEECH_LOG_TO_FILE = False  # True writes logs/speech_*.log for a field test
MARK_NOTE_MAX_CHARS = 200

SPEECH_LOG_DIR = "logs"

SPEECH_LOG_MAX_BYTES = 2_000_000

SPEECH_LOG_KEEP_LINES = 400

# Walk recording (utils/walk_recorder.py): every guidance tick's inputs and what was said,
# for tools/field/walk_replay.py. Off unless SHAPPE_RECORD_WALK=1 or POST /guidance/record.
WALK_RECORD = os.environ.get("SHAPPE_RECORD_WALK", "") == "1"
WALK_RECORD_DIR = os.path.join(_APP_DIR, "logs", "walks")
WALK_RECORD_MAX_MB = (
    500  # uncompressed; a recording stops itself here (~1 h of walking)
)
# Older recordings are deleted when a new one starts: the newest this many, this much in all
# (compressed, on the card).
WALK_RECORD_KEEP_FILES = 30
WALK_RECORD_KEEP_MB = 1000
# A tester's "Wrong" or "Missed" (/guidance/mark, the dashboard or the phone) saves a clip:
# the guidance ticks from WALK_CLIP_BEFORE_S before the mark to WALK_CLIP_AFTER_S after,
# in the recording's format, as logs/walks/clip-*.jsonl.gz — what the sensors saw, for
# tools/field/walk_replay.py, recording or not (2026-11-10: the 17:17 false glass could not be
# replayed, nothing was recorded). The last ticks are kept in memory for it, ~0.3 ms and
# ~10 kB each.
WALK_CLIP_ON_MARK = True
WALK_CLIP_BEFORE_S = 10.0
WALK_CLIP_AFTER_S = 2.0
WALK_CLIP_KEEP_FILES = 50
# What was really there (2026-11-10, Phase 1 of learning from walks): a tester's label button
# on the dashboard (/guidance/mark {"kind": "label", "label": ...}) names the thing the device
# just talked about — or what is ahead, if it named nothing — and saves a clip the same way,
# named clip-...-<label>.jsonl.gz. tools/field/label_report.py scores the current code against the
# labels and exports them for training on a laptop (never on the Pi). "nothing": nothing
# was there (a false alarm). Labelled clips are the training data: kept apart from the
# Wrong/Missed ones, WALK_LABEL_KEEP_FILES of them (~0.2 MB each).
MARK_LABELS = ("wall", "glass", "obstacle", "person", "nothing")
WALK_LABEL_KEEP_FILES = 3000

SPEECH_LOG_KEEP_RUNS = 200

# ============== SERVER CONFIGURATION ==============
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8000

ADMIN_TOKEN = os.environ.get("SHAPPE_ADMIN_TOKEN", "").strip()

CORS_ORIGINS = [
    o.strip() for o in os.environ.get("SHAPPE_CORS_ORIGINS", "").split(",") if o.strip()
]

ALLOWED_HOSTS_TEXT = os.environ.get("SHAPPE_ALLOWED_HOSTS", "")

# A required heartbeat older than this withholds systemd's watchdog ping (WatchdogSec=15 in
# deploy/): 5 s (was 10, 2026-10-30) — the guidance loop beats every 0.1 s.
SERVICE_HEARTBEAT_MAX_AGE_S = 5.0

SHUTDOWN_TIMEOUT_S = 45.0
SERVER_GRACEFUL_SHUTDOWN_S = 5.0
# The largest request body, and WebSocket message, the server reads. A full LiDAR scan as
# JSON is ~60 KB. Unlimited, a 43 MB POST stalled the other threads 0.6 s and took 380 MB,
# from any device on the Wi-Fi, read even before the admin-token check (scan 2026-10-23).
SERVER_MAX_BODY_BYTES = 1_048_576

FRAME_GRID_ROWS = 2
FRAME_GRID_COLS = 2

# The picture fractions below (SIMPLE_OBJECT_CENTRE_BAND, the *_HEIGHT_FRAC sizes, the steering
# and vehicle margins) were set on the old 960 x 540 picture of a 3840 px crop. Scaled by these
# they keep the same size in pixels at the same focal length — the same angle and distance —
# on whatever picture the camera gives (2026-11-02: 1164 x 874 of the whole sensor; the focal
# length, ~896 px, is the same, so a threshold is just a smaller share of a bigger picture).
_CAMERA_QUAD_PX = (CAMERA_WIDTH / FRAME_GRID_COLS, CAMERA_HEIGHT / FRAME_GRID_ROWS)
_LENS_TAN = math.tan(math.radians(FUSION_CAMERA_LENS_HFOV_DEG) / 2.0)
_CAMERA_FOCAL_PX = (_CAMERA_QUAD_PX[0] / 2.0) / (
    _LENS_TAN * FUSION_CAMERA_CROP_PX[0] / FUSION_CAMERA_SENSOR_PX[0]
)
_WAS_FOCAL_PX = 480.0 / (_LENS_TAN * 3840.0 / 4656.0)
_PER_W = 960.0 / _CAMERA_QUAD_PX[0] * _CAMERA_FOCAL_PX / _WAS_FOCAL_PX
_PER_H = 540.0 / _CAMERA_QUAD_PX[1] * _CAMERA_FOCAL_PX / _WAS_FOCAL_PX
FRAME_SECTOR_LAYOUT = {  # camera quadrant mapping; do not change without a physical check
    # Cameras moved on the rig 2026-11-09: the one that was "back" faces front, the old
    # "front" faces left, the old "left" faces back; "right" stays.
    "front": (1, 0),  # bottom-left quadrant   (bottom-right 2026-11-05 to 2026-11-09)
    "right": (0, 0),  # top-left quadrant      (unchanged)
    "back": (0, 1),  # top-right quadrant      (bottom-left 2026-11-05 to 2026-11-09)
    "left": (1, 1),  # bottom-right quadrant   (top-right 2026-11-05 to 2026-11-09)
}


SIMPLE_ALERT_CONFIRM_S = 0.2
# A block, and a wall, the LiDAR itself measures in this many DIFFERENT scans in a row is
# confirmed at once (~67 ms at 15 Hz), not after SIMPLE_ALERT_CONFIRM_S, which was ~250 ms of
# a ~320 ms reaction (2026-10-30). Radar-only (glass) and camera names keep the 0.2 s:
# their frames come slower and can repeat between ticks. 0: off. VALIDATE on a walk.
SIMPLE_LIDAR_CONFIRM_SCANS = 2
# The same for what the FRONT CAMERA names (a person, a vehicle in front) and a vehicle its
# tracker judges approaching: this many DIFFERENT detected frames in a row (~65-130 ms at
# the full-view camera's ~16 a second), not SIMPLE_ALERT_CONFIRM_S — "Person in front" waited ~0.25 s, and held a
# LiDAR-confirmed block back while it did (2026-10-30). 0: off. VALIDATE on a walk.
SIMPLE_CAMERA_CONFIRM_FRAMES = 2
SIMPLE_ALERT_GAP_S = 0.5
SIMPLE_CLEAR_CONFIRM_S = 1.0
SIMPLE_BAND_MARGIN_FT = 1.0

# The speech gate's waits, cut for a real-time voice on 2026-11-09 (were 1.0 s, 3.0 s and 6.0 s:
# a hazard's change of kind waited 6.3 s, a closed way out 1.1 s). A new sentence that is
# not more urgent is held SIMPLE_CONFIRM_S, so a flicker is not said; "Walk straight" is held
# SIMPLE_CLEAR_SAY_CONFIRM_S, since saying the way is clear too early is the dangerous
# direction. Between sentences: SIMPLE_MIN_GAP_S from the last one's start, and never before
# it has finished plus SIMPLE_AFTER_SPEECH_S (the old 3 s gap had covered that by itself).
SIMPLE_CONFIRM_S = 0.4
SIMPLE_CLEAR_SAY_CONFIRM_S = 1.0
SIMPLE_MIN_GAP_S = 1.5
SIMPLE_AFTER_SPEECH_S = 0.2

SIMPLE_REPEAT_S = 30.0
SIMPLE_REPEAT_MAX_S = 120.0
# Still blocked after turning: the block sentence comes back this soon (doubling to
# SIMPLE_REPEAT_MAX_S while it lasts), not after SIMPLE_REPEAT_S (2026-10-21).
SIMPLE_BLOCKED_REPEAT_S = 8.0
# "Guidance stopped, use your cane" repeats this often for as long as it lasts, no backoff.
SIMPLE_OFFLINE_REPEAT_S = 12.0

SIMPLE_WALL_APPROACH_FT = 12.0
# Something on the walking line that is not a wall, glass or a named object: "Obstacle
# ahead" inside this, before the WALK_BLOCK_FT turn (~1.9 s at 1.3 m/s; 2026-10-21).
SIMPLE_OBSTACLE_AHEAD_FT = 8.0

SIMPLE_ON_WALL_MM = 60.0
SIMPLE_WALL_MATCH_FT = 1.0
SIMPLE_CROSSING_HOLD_S = 5.0
# After a traffic light leaves the front camera, "Walk straight" stays withheld until the
# user has been measured walking this long — not on a timer, at the kerb (2026-10-21).
SIMPLE_CROSSING_WALKED_S = 4.0
# With no motion reading at all, the hold ends this long after the light was last seen.
SIMPLE_CROSSING_MAX_HOLD_S = 60.0
# A NEW crossing needs the light seen this long (gaps up to CONFIRM_GAP_S allowed): one
# frame of a lamp taken for a light held "Walk straight" back for up to a minute.
SIMPLE_CROSSING_CONFIRM_S = 0.8
SIMPLE_CROSSING_CONFIRM_GAP_S = 0.5

SIMPLE_DRIFT_NEAR_FT = 5.0  # drift said only inside 5 ft, ahead of the 4 ft near-wall band (was 2.5, decided 2026-10-01)
SIMPLE_DRIFT_MIN_DEG = 4.0
SIMPLE_DRIFT_AHEAD_FT = 20.0  # converging wall must meet the path within this (was 12; ~1.8 s earlier at 8 deg)
SIMPLE_DRIFT_WINDOW_S = 1.0  # trend window (was 1.5)
SIMPLE_DRIFT_MIN_SPAN_S = 0.6  # minimum trend span (was 1.0)
SIMPLE_DRIFT_CLOSING_FT_S = 0.2
SIMPLE_DRIFT_JUMP_MM = 300.0

SIMPLE_WALL_NEAR_FT = (
    4.0  # "Near left wall, move right" inside this (was 1.5, decided 2026-10-01)
)
SIMPLE_WALL_NEAR_MARGIN_FT = 0.5
SIMPLE_MOVE_ROOM_FT = 2.0
SIMPLE_GLASS_BESIDE_FT = 4.0  # "Glass on left, move right": glass only one front board hears, inside this (decided 2026-10-01)
SIMPLE_SIDE_WALL_EXTEND_MM = 600.0
SIMPLE_SIDE_WALL_HOLD_SPAN_MM = 400.0


SIMPLE_NAMED_OBJECTS = {
    # Vehicles (see SIMPLE_VEHICLE_HEIGHT_M).
    "car": "Car",
    "truck": "Truck",
    "bus": "Bus",
    "motorcycle": "Motorbike",
    "bicycle": "Bicycle",
    "train": "Train",
    "vehicle": "Vehicle",
    "sedan": "Car",
    "suv": "Car",
    "pickup": "Truck",
    "cyclist": "Bicycle",
    "forklift": "Forklift",
    "big_truck": "Truck",
    "mid_truck": "Truck",
    "small_truck": "Truck",
    "truck_": "Truck",
    "truck_l": "Truck",
    "truck_m": "Truck",
    "truck_xl": "Truck",
    "big_bu": "Bus",
    "bus_": "Bus",
    "bus_l": "Bus",
    "small_bu": "Bus",
    # People and animals big enough to knock someone over or bite.
    "person": "Person",
    "persona": "Person",
    "pessoa": "Person",
    "dog": "Dog",
    "horse": "Horse",
    "cow": "Cow",
    "sheep": "Sheep",
    "elephant": "Elephant",
    "bear": "Bear",
    "bufflow": "Buffalo",
    "animal": "Animal",
    "tree": "Tree",
    "branch": "Branch",
    "potted plant": "Plant",
    "pottedplant": "Plant",
    "simple_pole": "Pole",
    "street_pole": "Pole",
    "double_pole": "Pole",
    "pole_special": "Pole",
    "traffic light": "Pole",
    "traffic_light": "Pole",
    "stop sign": "Pole",
    "parking meter": "Pole",
    "traffic_sign": "Pole",
    "street_sign": "Pole",
    "delimitation_stake": "Post",
    "stopper": "Bollard",
    # Street furniture and ground hazards.
    "fire hydrant": "Hydrant",
    "fire_hydrant": "Hydrant",
    "bench": "Bench",
    "bus_stop": "Bus stop",
    "advertising_column": "Pillar",
    "waste_container": "Bin",
    "stairs": "Stairs",
    "pothole": "Pothole",
    "stone": "Stone",
}

# The detector reports ONLY these classes. Everything else a model knows (chairs,
# tables, laptops, bottles ...) is dropped inside the engine and reaches nothing:
# not the voice, the hazard list, the BEV labels or the dashboard boxes.
DETECTION_CLASSES = frozenset(SIMPLE_NAMED_OBJECTS)

SIMPLE_STEER_RANGE_M = 5.0
SIMPLE_STEER_GAP_M = 0.8
SIMPLE_STEER_STICKY = 0.10 * _PER_W  # 0.05 flipped sides on 1% box jitter, dead ahead
SIMPLE_STEER_DEAD_BAND = 0.03 * _PER_W
SIMPLE_STEER_SIDE_MIN_FT = 2.0
SIMPLE_OBJECT_HEIGHT_M = {
    "person": 1.7,
    "persona": 1.7,
    "pessoa": 1.7,
    "dog": 0.6,
    "horse": 1.6,
    "cow": 1.4,
    "sheep": 0.9,
    "elephant": 2.8,
    "bear": 1.2,
    "bufflow": 1.5,
    "animal": 1.0,
    "tree": 5.0,
    "branch": 1.0,
    "potted plant": 1.0,
    "pottedplant": 1.0,
    "simple_pole": 3.0,
    "street_pole": 6.0,
    "double_pole": 3.0,
    "pole_special": 3.0,
    "traffic light": 1.0,
    "traffic_light": 1.0,
    "stop sign": 0.75,
    "parking meter": 1.3,
    "traffic_sign": 0.75,
    "street_sign": 0.75,
    "delimitation_stake": 1.0,
    "stopper": 0.9,
    "fire hydrant": 0.8,
    "fire_hydrant": 0.8,
    "bench": 0.8,
    "bus_stop": 2.5,
    "advertising_column": 2.5,
    "waste_container": 1.1,
    "stairs": 1.0,
    "pothole": 0.3,
    "stone": 0.3,
}

# ---- How far away a detection is (sensor_fusion.detection_distance, the dashboard).
# The laser is searched across the bearings a box would cover for any field of view in this
# range: the camera's (FUSION_CAMERA_*, 66 deg since 2026-11-02, as SIMPLE_CAMERA_HFOV_DEG —
# the voice's figure), +-3 deg for the lens. Until 2026-10-30 an unmeasured 55-75, which put
# the dashboard's camera bearings ~4 deg further out than the voice's.
_CAMERA_HFOV_DEG = 2.0 * math.degrees(
    math.atan(
        math.tan(math.radians(FUSION_CAMERA_LENS_HFOV_DEG) / 2.0)
        * FUSION_CAMERA_CROP_PX[0]
        / FUSION_CAMERA_SENSOR_PX[0]
    )
)
DETECTION_DISTANCE_HFOV_DEG = (_CAMERA_HFOV_DEG - 3.0, _CAMERA_HFOV_DEG + 3.0)
# The laser's range is the thing's only within these multiples of the box-size estimate.
# A thing shorter than its class (sitting, half hidden) makes that estimate too FAR, so
# the laser may be much nearer; hardly ever much farther (that is the wall behind it).
DETECTION_DISTANCE_AGREE = (0.5, 1.4)
DETECTION_SEATED_ASPECT = (
    0.5  # a person box at least this wide for its height is sitting
)
DETECTION_SEATED_HEIGHT_M = 1.3

SIMPLE_CAMERA_FRESH_S = 2.0
SIMPLE_OBJECT_MIN_CONF = 0.5
SIMPLE_OBJECT_CENTRE_BAND = 0.5 * _PER_W  # the middle ~30 deg
SIMPLE_OBJECT_MIN_HEIGHT_FRAC = 0.25 * _PER_H

SIMPLE_VEHICLE_HEIGHT_M = {
    "car": 1.5,
    "truck": 3.0,
    "bus": 3.2,
    "motorcycle": 1.2,
    "bicycle": 1.0,
    "train": 3.8,
    "vehicle": 1.5,
    "sedan": 1.45,
    "suv": 1.8,
    "pickup": 1.8,
    "cyclist": 1.7,
    "forklift": 2.2,
    "big_truck": 3.5,
    "mid_truck": 3.0,
    "small_truck": 2.2,
    "truck_": 3.0,
    "truck_l": 3.5,
    "truck_m": 3.0,
    "truck_xl": 3.8,
    "big_bu": 3.2,
    "bus_": 3.2,
    "bus_l": 3.2,
    "small_bu": 2.6,
}
# Typical widths (m): where a vehicle will pass is judged from its nearest front corner and
# its width (simple_guidance._Vehicles, 2026-11-06). Classes not listed: 1.8.
SIMPLE_VEHICLE_WIDTH_M = {
    "car": 1.8,
    "sedan": 1.8,
    "suv": 1.9,
    "pickup": 1.9,
    "vehicle": 1.8,
    "truck": 2.5,
    "big_truck": 2.5,
    "mid_truck": 2.4,
    "small_truck": 2.0,
    "truck_": 2.5,
    "truck_l": 2.5,
    "truck_m": 2.4,
    "truck_xl": 2.55,
    "bus": 2.55,
    "big_bu": 2.55,
    "bus_": 2.55,
    "bus_l": 2.55,
    "small_bu": 2.3,
    "train": 3.0,
    "forklift": 1.2,
    "motorcycle": 0.8,
    "bicycle": 0.6,
    "cyclist": 0.6,
}
SIMPLE_VEHICLE_CLASSES = tuple(SIMPLE_VEHICLE_HEIGHT_M)
SIMPLE_VEHICLE_MIN_CONF = 0.45
SIMPLE_VEHICLE_MIN_HEIGHT_FRAC = 0.06 * _PER_H  # ~32 px: a car ~30 m out
SIMPLE_VEHICLE_WINDOW_S = 1.0
# 0.4 s (was 0.6, 2026-10-30): the front camera is detected ~16 times a second (full view,
# 2026-11-02; ~9 when cycled), so 0.4 s holds ~6 frames, more than 0.6 s did. VALIDATE on a walk: a parked car walked toward.
SIMPLE_VEHICLE_MIN_SPAN_S = 0.4
SIMPLE_VEHICLE_LOST_S = 1.0  # a track unseen this long is dropped
SIMPLE_VEHICLE_APPROACH_RATE = 0.12
SIMPLE_VEHICLE_CONFIRM_FRAMES = 3
SIMPLE_VEHICLE_EGO_MARGIN = 1.5
# The user's pace taken off a vehicle's approach when nothing can measure it (open space,
# no gait reading) — until 2026-10-21 that was 0, so walking toward a parked car read as
# the car approaching. The cost: a user who IS standing still there is warned of a slowly
# creeping vehicle later (it must close ~1.8 m/s faster). 0.0 restores the old behaviour.
SIMPLE_VEHICLE_PACE_UNKNOWN_MPS = HEADING_NOMINAL_SPEED_MPS
# Drives every camera range and bearing estimate. From the geometry above (FUSION_CAMERA_*:
# the IMX519's 66 deg across all 4656 px, uncropped since 2026-11-02; 56.3 deg on the old
# 3840 px crop, and 70 assumed until 2026-10-23, which put box-size ranges ~23% too near).
SIMPLE_CAMERA_HFOV_DEG = 2.0 * math.degrees(
    math.atan(
        math.tan(math.radians(FUSION_CAMERA_LENS_HFOV_DEG) / 2.0)
        * FUSION_CAMERA_CROP_PX[0]
        / FUSION_CAMERA_SENSOR_PX[0]
    )
)
SIMPLE_VEHICLE_MISS_M = 2.0
SIMPLE_VEHICLE_IDLE_RATE = 0.05
SIMPLE_VEHICLE_IDLE_SLIDE = 0.05 * _PER_W

SIMPLE_VEHICLE_CAREFUL_HEIGHT_FRAC = 0.12 * _PER_H
SIMPLE_VEHICLE_CAREFUL_MISS_M = 3.0
SIMPLE_VEHICLE_RANGE_MATCH = 2.0
SIMPLE_VEHICLE_SIDE_MARGIN = 0.05 * _PER_W

SIMPLE_SETTLE_S = (
    2.0  # a change about the SAME hazard that is not more urgent (6.0 until 2026-11-09)
)
# ...but the way out flipping left <-> right while the side given still has room keeps the old
# 6 s: the walk policy can alternate near a post just off centre, and the advice given is
# still good (turn review, 2026-11-09). A side that CLOSED is said at once (_advice_closed).
SIMPLE_FLIP_SETTLE_S = 6.0
SIMPLE_OBJECT_RELEASE_S = 3.0
SIMPLE_GLASS_RELEASE_S = 3.0
# "Guidance stopped, use your cane" once neither rangefinder has reported ahead for this long.
SIMPLE_OFFLINE_S = 3.0
# A LiDAR scan with almost no returns (LIDAR_MIN_VALID_FRACTION) is said as "Guidance stopped"
# only once such scans have lasted this long; a single one is held silently, like a stale
# scan (2026-11-09: one sparse scan cut in with "Guidance stopped", then held the block
# warning back ~3 s — three times on the rig at 09:57-10:05).
SIMPLE_SPARSE_CONFIRM_S = 1.0
# The guidance step failing tick after tick for this long is a fault: the wearer is told,
# and the watchdog heartbeat is withheld so systemd restarts the app (2026-10-21). Seconds,
# not ticks (3 at 5 Hz until 2026-10-30): ticks now follow the LiDAR's scans.
GUIDANCE_FAIL_S = 0.6
# The guidance loop finishing NO tick (a hang, not a failing step) for this long: the Pi voice
# says "Guidance stopped, use your cane" itself, every SIMPLE_OFFLINE_REPEAT_S. Until
# 2026-10-30 a hung loop was silent until systemd's watchdog restarted the app (~40 s).
GUIDANCE_STALL_SAY_S = 2.0

# The Pi itself (utils/pi_health, 2026-10-21): read every PI_HEALTH_POLL_S off the guidance
# path. "Hot" from PI_HOT_C (the Pi 5 firmware starts throttling near 80) or any throttle
# flag, until it is back under PI_HOT_CLEAR_C; "power low" for PI_UNDERVOLTAGE_HOLD_S after
# any under-voltage sample, as it comes in short dips under load.
PI_HEALTH_POLL_S = 2.0
PI_HOT_C = 80.0
PI_HOT_CLEAR_C = 75.0
PI_UNDERVOLTAGE_HOLD_S = 30.0
# "Power low" / "Device overheating", and their all-clear, said like "Glass sensor off/on".
SIMPLE_SYSTEM_LOSS_CONFIRM_S = 3.0
SIMPLE_SYSTEM_RECOVER_CONFIRM_S = 60.0
SIMPLE_SYSTEM_RECOVER_MAX_S = 600.0
SIMPLE_SYSTEM_STARTUP_GRACE_S = 20.0  # power dips while everything starts up

SIMPLE_GLASS_LOSS_CONFIRM_S = 3.0
SIMPLE_GLASS_RECOVER_CONFIRM_S = 10.0
SIMPLE_GLASS_RECOVER_MAX_S = 160.0
SIMPLE_GLASS_STARTUP_GRACE_S = 30.0  # assumed; set it from a timed cold boot

# "Camera off" / "Camera on": told once, the same way as the glass sensor, when the front
# camera can no longer name what is ahead (sensor_fusion.front_camera_coverage) — the
# LiDAR still guides, but nothing will be called a person, a car or a pole.
SIMPLE_CAMERA_LOSS_CONFIRM_S = 5.0  # longer than a corrupted-picture camera restart
SIMPLE_CAMERA_RECOVER_CONFIRM_S = 10.0
SIMPLE_CAMERA_RECOVER_MAX_S = 160.0
SIMPLE_CAMERA_STARTUP_GRACE_S = 45.0  # the detector takes a while to load; assumed
SIMPLE_LIDAR_FRESH_S = 2.0

CONSOLE_ENABLED = True
CONSOLE_STATUS_S = 2.0
CONSOLE_HEARTBEAT_S = 60.0

# False only on the bench: every sound off (Pi voice, sink probe, dashboard speech, the
# phone's AF_AUDIO_ENABLED); guidance still shown and logged, and /health says "down".
AUDIO_ENABLED = True
PI_VOICE_ENABLED = True
# The guidance loop ticks on every LiDAR scan as it lands (~15 Hz, 2026-10-30); this is
# the floor when no scan comes (was a 5 Hz timer). A tick takes ~4 ms on the Pi 5.
PI_VOICE_TICK_HZ = 10.0
# An urgent sentence ("Blocked…", "Vehicle approaching…", "Guidance stopped…") starts with a
# short tone the wearer can react to at once — the words take ~1 s to say (2026-10-30). Played
# through PipeWire, like espeak-ng's voice; it holds the words back ~0.17 s (measured on the
# Pi: 0.1 s tone + ~60 ms to start the player). False: words only.
PI_VOICE_URGENT_TONE = True
PI_VOICE_TONE_CMD = ["pw-play", "-"]  # the tone's WAV on stdin
PI_VOICE_TONE_HZ = 1000.0
PI_VOICE_TONE_S = 0.1
# /guidance/walk serves the loop's newest result; older than this, it says so.
GUIDANCE_RESULT_STALE_S = 2.0
PI_VOICE_REPORT_QUEUE_MAX = 256
PI_VOICE_REAP_WAIT_S = 2.0
PI_VOICE_ALLOW_HDMI = False
# ~170 words a minute; the gate times every sentence with this command at start-up.
PI_VOICE_CMD = ["espeak-ng", "-s", "170", "-v", "en-us"]
