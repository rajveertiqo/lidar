"""Configuration settings for Pi Camera, LiDAR, and Navigation systems"""

import math
import os

_APP_DIR = os.path.dirname(os.path.abspath(__file__))

HEF_PATH = os.path.join(_APP_DIR, "model", "yolo26m.hef")  # compiled YOLO detector

# ============== CAMERA CONFIGURATION ==============
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
JPEG_QUALITY = 80
TARGET_FPS = 30  # desired pipeline / display FPS
FRAME_INTERVAL = 1.0 / TARGET_FPS

CAMERA_CAPTURE_FPS = 2 * TARGET_FPS

STREAM_MAX_WIDTH = 480  # px per sector feed
STREAM_QUALITY = 50  # JPEG quality for the display stream
STREAM_FPS = 30  # smooth/live — matches the camera capture rate

STREAM_STALL_TIMEOUT_S = 5.0

CAMERA_FRAME_TIMEOUT_S = 5.0

CAMERA_STOP_TIMEOUT_S = 5.0

CAMERA_RETRY_BACKOFF_S = (2.0, 5.0, 10.0)

CAMERA_SIMULATION_ALLOWED = (
    os.environ.get("SHAPPE_SIMULATE_CAMERA", "") == "1"
)  # development only; simulated frames are never evidence

HAILO_RETRY_BACKOFF_S = (2.0, 5.0, 10.0, 30.0)
HAILO_INFER_FAIL_RESET_S = 5.0
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

LIDAR_RPM = 900  # Target motor speed in RPM (15 Hz, top of A2M12 range)
LIDAR_MAX_RPM = 1380  # this unit's measured speed at full PWM (see above)

LIDAR_ANGLE_OFFSET = 0  # degrees to add to raw angles

LIDAR_MIRROR = (
    False  # verified on this rig; do not flip without a physical left/right check
)

LIDAR_CLUSTER_MIN_POINTS = 3
LIDAR_CLUSTER_WINDOW_MM = 150
LIDAR_PERSIST_DEG = 4.5
LIDAR_PERSIST_MAX_AGE_S = 0.5

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

RADAR_START_MM = 250  # nearer than this the radar is blind; LiDAR covers it
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

CLOCK_FUTURE_TOLERANCE_S = 0.5

RADAR_READY_TIMEOUT_S = 140.0

RADAR_RESTART_WAIT_S = 30.0

RADAR_CALIBRATE_MAX_ENTRIES = 32
RADAR_CALIBRATE_MAX_BODY_BYTES = 16384

RADAR_PRIORITY = False

RADAR_BEAM_H_DEG = 65.0

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

WALK_MEASURED_MEMORY_S = 3.0

WALK_GLASS_GAP_FT = 2.0  # LiDAR farther than radar by this much = glass (~0.6 m)

HEADING_ENABLED = True
HEADING_BIN_DEG = 1.0  # range profile resolution used for scan matching
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

GYRO_ENABLED = True

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
SEG_VIEW_FPS = 2
SEG_VIEW_PRIORITY = True

STARTUP_SETTLE_S = 2.0
STARTUP_OPTIONAL_WAIT_S = 15.0
STARTUP_LIDAR_ABSENT_S = 20.0

STARTUP_MIN_SAMPLES = 6

STARTUP_STABLE_FT = 1.5

STARTUP_STABLE_MAX_WAIT_S = 5.0

STARTUP_READY_CUE = "Ready."

NOTIFY_WORDS_PER_SEC = 2.8  # measured speaking rate of the delivery voice

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

SPEECH_LOG_KEEP_RUNS = 200

# ============== SERVER CONFIGURATION ==============
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8000

ADMIN_TOKEN = os.environ.get("SHAPPE_ADMIN_TOKEN", "").strip()

CORS_ORIGINS = [
    o.strip() for o in os.environ.get("SHAPPE_CORS_ORIGINS", "").split(",") if o.strip()
]

ALLOWED_HOSTS_TEXT = os.environ.get("SHAPPE_ALLOWED_HOSTS", "")

SERVICE_HEARTBEAT_MAX_AGE_S = 10.0

SHUTDOWN_TIMEOUT_S = 45.0
SERVER_GRACEFUL_SHUTDOWN_S = 5.0

FRAME_GRID_ROWS = 2
FRAME_GRID_COLS = 2
FRAME_SECTOR_LAYOUT = {  # camera quadrant mapping; do not change without a physical check
    "front": (0, 1),  # top-right quadrant     (was bottom-right until 2026-09-22)
    "right": (0, 0),  # top-left quadrant      (unchanged)
    "back": (1, 1),  # bottom-right quadrant   (was top-right until 2026-09-22)
    "left": (1, 0),  # bottom-left quadrant    (unchanged)
}


SIMPLE_ALERT_CONFIRM_S = 0.2
SIMPLE_ALERT_GAP_S = 0.5
SIMPLE_CLEAR_CONFIRM_S = 1.0
SIMPLE_BAND_MARGIN_FT = 1.0

SIMPLE_CONFIRM_S = 1.0
SIMPLE_MIN_GAP_S = 3.0

SIMPLE_REPEAT_S = 30.0
SIMPLE_REPEAT_MAX_S = 120.0

SIMPLE_WALL_APPROACH_FT = 12.0

SIMPLE_ON_WALL_MM = 60.0
SIMPLE_WALL_MATCH_FT = 1.0
SIMPLE_CROSSING_HOLD_S = 5.0

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

SIMPLE_STEER_RANGE_M = 5.0
SIMPLE_STEER_GAP_M = 0.8
SIMPLE_STEER_STICKY = 0.10  # 0.05 flipped sides on 1% box jitter, dead ahead
SIMPLE_STEER_DEAD_BAND = 0.03
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

SIMPLE_CAMERA_FRESH_S = 2.0
SIMPLE_OBJECT_MIN_CONF = 0.5
SIMPLE_OBJECT_CENTRE_BAND = 0.5
SIMPLE_OBJECT_MIN_HEIGHT_FRAC = 0.25

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
SIMPLE_VEHICLE_CLASSES = tuple(SIMPLE_VEHICLE_HEIGHT_M)
SIMPLE_VEHICLE_MIN_CONF = 0.45
SIMPLE_VEHICLE_MIN_HEIGHT_FRAC = 0.06  # ~32 px: a car ~30 m out
SIMPLE_VEHICLE_WINDOW_S = 1.0
SIMPLE_VEHICLE_MIN_SPAN_S = 0.6
SIMPLE_VEHICLE_LOST_S = 1.0  # a track unseen this long is dropped
SIMPLE_VEHICLE_APPROACH_RATE = 0.12
SIMPLE_VEHICLE_CONFIRM_FRAMES = 3
SIMPLE_VEHICLE_EGO_MARGIN = 1.5
SIMPLE_CAMERA_HFOV_DEG = (
    70.0  # ASSUMED, not measured; drives every camera range estimate
)
SIMPLE_VEHICLE_MISS_M = 2.0
SIMPLE_VEHICLE_IDLE_RATE = 0.05
SIMPLE_VEHICLE_IDLE_SLIDE = 0.05

SIMPLE_VEHICLE_CAREFUL_HEIGHT_FRAC = 0.12
SIMPLE_VEHICLE_CAREFUL_MISS_M = 3.0
SIMPLE_VEHICLE_RANGE_MATCH = 2.0
SIMPLE_VEHICLE_SIDE_MARGIN = 0.05

SIMPLE_SETTLE_S = 6.0
SIMPLE_OBJECT_RELEASE_S = 3.0
SIMPLE_GLASS_RELEASE_S = 3.0
# "Sensors offline" once neither rangefinder has reported ahead for this long.
SIMPLE_OFFLINE_S = 3.0

SIMPLE_GLASS_LOSS_CONFIRM_S = 3.0
SIMPLE_GLASS_RECOVER_CONFIRM_S = 10.0
SIMPLE_GLASS_RECOVER_MAX_S = 160.0
SIMPLE_GLASS_STARTUP_GRACE_S = 30.0  # assumed; set it from a timed cold boot

SIMPLE_LIDAR_FRESH_S = 2.0

CONSOLE_ENABLED = True
CONSOLE_STATUS_S = 2.0
CONSOLE_HEARTBEAT_S = 60.0

AUDIO_ENABLED = False  # every sound off: Pi voice, sink probe, dashboard speech; guidance still shown and logged
PI_VOICE_ENABLED = True
PI_VOICE_TICK_HZ = 5.0
PI_VOICE_REPORT_QUEUE_MAX = 256
PI_VOICE_REAP_WAIT_S = 2.0
PI_VOICE_ALLOW_HDMI = False
# ~170 words a minute: the 2.8 words/s the gate's airtime estimate assumes.
PI_VOICE_CMD = ["espeak-ng", "-s", "170", "-v", "en-us"]
