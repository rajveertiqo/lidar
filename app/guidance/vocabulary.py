"""The closed vocabulary: every sentence the guidance can say (ALLOWED), what each is
about, and how urgent it is (_LEVEL). Moved out of simple_guidance on 2026-11-11, which
re-exports every name (sg.WALL_AHEAD still works). The phone's phrase list is made from
ALLOWED (ble_bridge/make_vocab.py)."""

from typing import Any, Dict, Tuple

from app.config import SIMPLE_NAMED_OBJECTS

WALK_STRAIGHT = "Walk straight"
WALL_AHEAD = "Wall ahead"
GLASS_AHEAD = "Glass ahead"
BLOCKED_WALL = "Blocked by wall"
BLOCKED_GLASS = "Blocked by glass"
TURN_LEFT = "Turn left"
TURN_RIGHT = "Turn right"
# The route planner's (app/perception/route_planner, 2026-10-13): past something no camera names, and
# no way ahead at all. Said only with ROUTE_PLANNER_SPEAK.
TURN_SLIGHT_LEFT = "Turn slightly left"
TURN_SLIGHT_RIGHT = "Turn slightly right"
TURN_AROUND = "Turn around"

DRIFT_LEFT = "Drifting toward left wall, turn slightly right"
DRIFT_RIGHT = "Drifting toward right wall, turn slightly left"

NEAR_LEFT = "Near left wall, move right"
NEAR_RIGHT = "Near right wall, move left"
_SIDE_WALL = (DRIFT_LEFT, DRIFT_RIGHT, NEAR_LEFT, NEAR_RIGHT)

# Glass only one front board hears, inside SIMPLE_GLASS_BESIDE_FT (decided 2026-10-01).
GLASS_LEFT = "Glass on left, move right"
GLASS_RIGHT = "Glass on right, move left"
_GLASS_BESIDE = (GLASS_LEFT, GLASS_RIGHT)

VEHICLE_FROM_LEFT = "Vehicle approaching, move right"
VEHICLE_FROM_RIGHT = "Vehicle approaching, move left"
VEHICLE_APPROACHING = "Vehicle approaching"
VEHICLE_AHEAD = "Vehicle in front, be careful"
_VEHICLE_APPROACH = (VEHICLE_FROM_LEFT, VEHICLE_FROM_RIGHT, VEHICLE_APPROACHING)
_VEHICLE_SENTENCES = _VEHICLE_APPROACH + (VEHICLE_AHEAD,)

# Something the LiDAR sees moving by itself, coming at the user (app/perception/lidar_movers,
# 2026-10-07), or that only the radar hears closing in where the LiDAR cannot see it
# (app/perception/radar_movers: "Something"). Crossing: stop and let it pass. Head-on or from
# behind: step aside, where the laser shows room. Spoken only with LIDAR_MOVERS_SPEAK.
_MOVER_WORDS = ("Person", "Vehicle", "Something")
MOVER_CROSSING = {
    (w, s): f"{w} approaching from {s}, stop"
    for w in _MOVER_WORDS
    for s in ("left", "right")
}
MOVER_TOWARD = {
    (w, m): f"{w} coming toward you, move {m}"
    for w in _MOVER_WORDS
    for m in ("left", "right")
}
MOVER_TOWARD_STOP = {w: f"{w} coming toward you, stop" for w in _MOVER_WORDS}
MOVER_BEHIND = {
    (w, m): f"{w} approaching from behind, move {m}"
    for w in _MOVER_WORDS
    for m in ("left", "right")
}
MOVER_BEHIND_ONLY = {w: f"{w} approaching from behind" for w in _MOVER_WORDS}
_MOVER_INFO: Dict[str, Dict[str, Any]] = {
    **{t: {"word": k[0], "move": None} for k, t in MOVER_CROSSING.items()},
    **{t: {"word": k[0], "move": k[1]} for k, t in MOVER_TOWARD.items()},
    **{t: {"word": w, "move": None} for w, t in MOVER_TOWARD_STOP.items()},
    **{t: {"word": k[0], "move": k[1]} for k, t in MOVER_BEHIND.items()},
    **{t: {"word": w, "move": None} for w, t in MOVER_BEHIND_ONLY.items()},
}
_MOVER_SENTENCES = tuple(_MOVER_INFO)
_APPROACHING = _VEHICLE_SENTENCES + _MOVER_SENTENCES

# Was "Sensors offline" until 2026-10-21: it sounded like the "Glass sensor off" advisory,
# and did not say what to do.
SENSORS_OFFLINE = "Guidance stopped, use your cane"

# On the walking line inside SIMPLE_OBSTACLE_AHEAD_FT, not a wall, glass or anything a
# camera names: said before the WALK_BLOCK_FT turn (2026-10-21).
OBSTACLE_AHEAD = "Obstacle ahead"
# The same thing inside WALK_BLOCK_FT: named, with the turn (2026-11-09). Until then it was
# "Turn left" alone, and the wearer was not told something stood in front (a post at
# 0.8 ft on the rig, 09:40): the camera names no plain pole. A wall beside the user that
# the front brushes is still "Turn left" — that is no obstacle in front.
_WAYS = ("left", "right", "around")
OBSTACLE_IN_FRONT = {s: f"Obstacle in front, turn {s}" for s in _WAYS}
_OBSTACLE_TURN = {t: s for s, t in OBSTACLE_IN_FRONT.items()}
_TURNS = (TURN_LEFT, TURN_RIGHT) + tuple(OBSTACLE_IN_FRONT.values())
# Blocked by a wall or a pane, AND the way out (2026-11-09): "Blocked by glass" alone left
# the wearer at the pane not knowing where to go. The turn is the walk policy's: toward more
# room, or around when neither side has WALK_MIN_TURN_FT. The bare sentences stay in the
# vocabulary (the phone's registry is append-only) but are no longer said.
BLOCKED_WALL_TURN = {s: f"Blocked by wall, turn {s}" for s in _WAYS}
BLOCKED_GLASS_TURN = {s: f"Blocked by glass, turn {s}" for s in _WAYS}
BLOCKED_WALL_ANY = (BLOCKED_WALL,) + tuple(BLOCKED_WALL_TURN.values())
BLOCKED_GLASS_ANY = (BLOCKED_GLASS,) + tuple(BLOCKED_GLASS_TURN.values())
# Every block sentence that carries its way out -> that way (left, right or around).
_WAY_OUT = {
    **_OBSTACLE_TURN,
    **{t: s for s, t in BLOCKED_WALL_TURN.items()},
    **{t: s for s, t in BLOCKED_GLASS_TURN.items()},
}
# The walk policy's "stop": blocked, and no direction has room to turn into (2026-10-21).
STOP_NO_WAY = "Stop, no clear way"
# A traffic light in the front camera: said once per crossing, in place of the "Walk
# straight" that is withheld there. Never "safe" (2026-10-21).
TRAFFIC_LIGHT_AHEAD = "Traffic light ahead"
# The start-up hold (app/guidance/startup_gate): said when it begins, and when guidance does.
STARTING = "Starting, please wait"
READY = "Ready"
_STARTUP_NOTICES = (STARTING, READY)

GLASS_SENSOR_OFF = "Glass sensor off"
GLASS_SENSOR_ON = "Glass sensor on"
_GLASS_NOTICES = (GLASS_SENSOR_OFF, GLASS_SENSOR_ON)

CAMERA_OFF = "Camera off"
CAMERA_ON = "Camera on"
_CAMERA_NOTICES = (CAMERA_OFF, CAMERA_ON)

# The Pi itself (app/core/pi_health, 2026-10-21): under-voltage means the battery or supply is
# failing and the device may switch off; heat means it is slowing down.
POWER_LOW = "Power low"
POWER_NORMAL = "Power normal"
DEVICE_HOT = "Device overheating"
DEVICE_COOL = "Device cooled down"
_SYSTEM_NOTICES = (POWER_LOW, POWER_NORMAL, DEVICE_HOT, DEVICE_COOL)
_STATUS_NOTICES = _GLASS_NOTICES + _CAMERA_NOTICES + _SYSTEM_NOTICES

_OBJECT_WORDS = {c.lower(): w for c, w in SIMPLE_NAMED_OBJECTS.items()}
_WORDS = sorted(set(_OBJECT_WORDS.values()))
OBJECT_AHEAD = {w: f"{w} ahead" for w in _WORDS}
OBJECT_IN_FRONT = {w: f"{w} in front" for w in _WORDS}
# A named block with the walk policy's way out when it is around (2026-11-09): a person in a
# 1 m corridor was "Person in front", where unnamed it was "Obstacle in front, turn around".
# Left and right are STEER_IN_FRONT's.
OBJECT_IN_FRONT_AROUND = {w: f"{w} in front, turn around" for w in _WORDS}
_WAY_OUT.update({t: "around" for t in OBJECT_IN_FRONT_AROUND.values()})
_OBJECT_OF = {
    t: w
    for table in (OBJECT_AHEAD, OBJECT_IN_FRONT, OBJECT_IN_FRONT_AROUND)
    for w, t in table.items()
}
_PATH_AHEAD = (WALL_AHEAD, GLASS_AHEAD, OBSTACLE_AHEAD) + tuple(OBJECT_AHEAD.values())

# Hazards on the ground ahead, from the ToF and confirmed by the front camera
# (app/perception/ground_hazards, 2026-11-06): what the shoulder LiDAR's one plane cannot see. A fall
# in front ("..., stop") is urgent; the rest are said once, as a wall ahead is. A pole, a
# pothole, or a low thing the camera names use the sentences that already exist ("Pole
# ahead", "Pothole ahead", "Bench ahead"); an unnamed tall one is "Obstacle ahead". Said
# only with TOF_HAZARDS_SPEAK, else logged as "would say".
STAIRS_UP_AHEAD = "Stairs up ahead"
STAIRS_DOWN_AHEAD = "Stairs down ahead"
STAIRS_DOWN_STOP = "Stairs down in front, stop"
STEP_UP_AHEAD = "Step up ahead"
STEP_DOWN_AHEAD = "Step down ahead"
DROP_AHEAD = "Drop ahead"
DROP_STOP = "Drop in front, stop"
LOW_OBSTACLE_AHEAD = "Low obstacle ahead"
OVERHANG_AHEAD = "Overhanging obstacle ahead"
_GROUND_STOP = (STAIRS_DOWN_STOP, DROP_STOP)
_GROUND_SENTENCES = (
    STAIRS_UP_AHEAD,
    STAIRS_DOWN_AHEAD,
    STAIRS_DOWN_STOP,
    STEP_UP_AHEAD,
    STEP_DOWN_AHEAD,
    DROP_AHEAD,
    DROP_STOP,
    LOW_OBSTACLE_AHEAD,
    OVERHANG_AHEAD,
)
# What each is about (guidance()'s "obstacle", the phone's registry).
_GROUND_OBSTACLE = {
    STAIRS_UP_AHEAD: "stairs",
    STAIRS_DOWN_AHEAD: "stairs",
    STAIRS_DOWN_STOP: "stairs",
    STEP_UP_AHEAD: "step",
    STEP_DOWN_AHEAD: "step",
    DROP_AHEAD: "drop",
    DROP_STOP: "drop",
    LOW_OBSTACLE_AHEAD: "low obstacle",
    OVERHANG_AHEAD: "overhang",
}
_GROUND_FALL_KINDS = ("drop", "stairs_down", "hole", "step_down")
# A fall ahead outranks a wall or glass ahead; the rest come after what the LiDAR names.
_GROUND_FALL_AHEAD = (DROP_AHEAD, STAIRS_DOWN_AHEAD, STEP_DOWN_AHEAD) + tuple(
    OBJECT_AHEAD[w] for w in ("Pothole",) if w in OBJECT_AHEAD
)

STEER_PAST: Dict[Tuple[str, str, str], str] = {}
for _w in _WORDS:
    STEER_PAST[(_w, "left", "right")] = f"{_w} on left, turn slightly right"
    STEER_PAST[(_w, "right", "left")] = f"{_w} on right, turn slightly left"
    STEER_PAST[(_w, "ahead", "left")] = f"{_w} ahead, turn slightly left"
    STEER_PAST[(_w, "ahead", "right")] = f"{_w} ahead, turn slightly right"
STEER_IN_FRONT = {
    (_w, _t): f"{_w} in front, turn {_t}" for _w in _WORDS for _t in ("left", "right")
}
BETWEEN_LEFT = "Walk between them, slightly left"
BETWEEN_RIGHT = "Walk between them, slightly right"
BETWEEN_STRAIGHT = "Walk straight between them"
_BETWEEN = (BETWEEN_LEFT, BETWEEN_RIGHT, BETWEEN_STRAIGHT)
_WALK_ON = (WALK_STRAIGHT,) + _BETWEEN
_STEER_INFO: Dict[str, Dict[str, Any]] = {
    **{t: {"turn": k[2], "word": k[0], "where": k[1]} for k, t in STEER_PAST.items()},
    **{
        t: {"turn": k[1], "word": k[0], "where": "in front"}
        for k, t in STEER_IN_FRONT.items()
    },
    BETWEEN_LEFT: {"turn": "left", "word": None, "where": "between"},
    BETWEEN_RIGHT: {"turn": "right", "word": None, "where": "between"},
    BETWEEN_STRAIGHT: {"turn": None, "word": None, "where": "between"},
    TURN_SLIGHT_LEFT: {"turn": "left", "word": None, "where": "ahead"},
    TURN_SLIGHT_RIGHT: {"turn": "right", "word": None, "where": "ahead"},
}

ALLOWED = (
    (
        WALK_STRAIGHT,
        WALL_AHEAD,
        GLASS_AHEAD,
        BLOCKED_WALL,
        BLOCKED_GLASS,
        TURN_LEFT,
        TURN_RIGHT,
        DRIFT_LEFT,
        DRIFT_RIGHT,
        NEAR_LEFT,
        NEAR_RIGHT,
        GLASS_LEFT,
        GLASS_RIGHT,
        SENSORS_OFFLINE,
        VEHICLE_FROM_LEFT,
        VEHICLE_FROM_RIGHT,
        VEHICLE_APPROACHING,
        VEHICLE_AHEAD,
    )
    + _GLASS_NOTICES
    + _MOVER_SENTENCES
    + tuple(OBJECT_AHEAD.values())
    + tuple(OBJECT_IN_FRONT.values())
    + tuple(STEER_PAST.values())
    + tuple(STEER_IN_FRONT.values())
    + _BETWEEN
    + _CAMERA_NOTICES
    + (TURN_SLIGHT_LEFT, TURN_SLIGHT_RIGHT, TURN_AROUND)
    + (OBSTACLE_AHEAD, STOP_NO_WAY, TRAFFIC_LIGHT_AHEAD)
    + _STARTUP_NOTICES
    + _SYSTEM_NOTICES
    + _GROUND_SENTENCES
    + tuple(OBSTACLE_IN_FRONT.values())
    + tuple(BLOCKED_WALL_TURN.values())
    + tuple(BLOCKED_GLASS_TURN.values())
    + tuple(OBJECT_IN_FRONT_AROUND.values())
)

_ACTION = {
    WALK_STRAIGHT: "walk_straight",
    WALL_AHEAD: "wall_ahead",
    GLASS_AHEAD: "glass_ahead",
    BLOCKED_WALL: "blocked_wall",
    BLOCKED_GLASS: "blocked_glass",
    TURN_LEFT: "turn_left",
    TURN_RIGHT: "turn_right",
    TURN_AROUND: "turn_around",
    DRIFT_LEFT: "drift_left",
    DRIFT_RIGHT: "drift_right",
    NEAR_LEFT: "near_wall_left",
    NEAR_RIGHT: "near_wall_right",
    GLASS_LEFT: "glass_beside_left",
    GLASS_RIGHT: "glass_beside_right",
    VEHICLE_FROM_LEFT: "vehicle_approaching_left",
    VEHICLE_FROM_RIGHT: "vehicle_approaching_right",
    VEHICLE_APPROACHING: "vehicle_approaching",
    VEHICLE_AHEAD: "vehicle_ahead",
    SENSORS_OFFLINE: "sensors_offline",
    GLASS_SENSOR_OFF: "glass_sensor_off",
    GLASS_SENSOR_ON: "glass_sensor_on",
    CAMERA_OFF: "camera_off",
    CAMERA_ON: "camera_on",
    OBSTACLE_AHEAD: "obstacle_ahead",
    STOP_NO_WAY: "stop",
    TRAFFIC_LIGHT_AHEAD: "traffic_light_ahead",
    STARTING: "starting",
    READY: "ready",
    POWER_LOW: "power_low",
    POWER_NORMAL: "power_normal",
    DEVICE_HOT: "device_hot",
    DEVICE_COOL: "device_cool",
    STAIRS_UP_AHEAD: "stairs_up_ahead",
    STAIRS_DOWN_AHEAD: "stairs_down_ahead",
    STAIRS_DOWN_STOP: "stairs_down_stop",
    STEP_UP_AHEAD: "step_up_ahead",
    STEP_DOWN_AHEAD: "step_down_ahead",
    DROP_AHEAD: "drop_ahead",
    DROP_STOP: "drop_stop",
    LOW_OBSTACLE_AHEAD: "low_obstacle_ahead",
    OVERHANG_AHEAD: "overhang_ahead",
    # A turn, as "Turn left" is: the heading reset (unified_guidance) and the phone's arrow.
    **{t: f"turn_{s}" for t, s in _WAY_OUT.items()},
    **{t: "object_ahead" for t in OBJECT_AHEAD.values()},
    **{t: "object_in_front" for t in OBJECT_IN_FRONT.values()},
    **{t: "steer_" + i["turn"] for t, i in _STEER_INFO.items() if i["turn"]},
    BETWEEN_STRAIGHT: "walk_between",
    **{
        t: (
            "mover_move_" + i["move"]
            if i["move"]
            else "mover_warn"
            if t in MOVER_BEHIND_ONLY.values()
            else "mover_stop"
        )
        for t, i in _MOVER_INFO.items()
    },
}

_LEVEL = {
    WALK_STRAIGHT: 0,
    WALL_AHEAD: 1,
    GLASS_AHEAD: 1,
    DRIFT_LEFT: 1,
    DRIFT_RIGHT: 1,
    NEAR_LEFT: 1,
    NEAR_RIGHT: 1,
    GLASS_LEFT: 1,
    GLASS_RIGHT: 1,
    BLOCKED_WALL: 2,
    BLOCKED_GLASS: 2,
    TURN_LEFT: 2,
    TURN_RIGHT: 2,
    **{t: 2 for t in _WAY_OUT},
    TURN_AROUND: 2,
    STOP_NO_WAY: 2,
    SENSORS_OFFLINE: 2,
    OBSTACLE_AHEAD: 1,
    GLASS_SENSOR_OFF: 1,
    GLASS_SENSOR_ON: 1,
    CAMERA_OFF: 1,
    CAMERA_ON: 1,
    TRAFFIC_LIGHT_AHEAD: 1,
    STARTING: 1,
    READY: 1,
    **{t: 1 for t in _SYSTEM_NOTICES},
    **{t: 1 for t in OBJECT_AHEAD.values()},
    **{t: 2 for t in OBJECT_IN_FRONT.values()},
    # A stopped vehicle, the user standing still: an advisory, said once.
    VEHICLE_AHEAD: 1,
    VEHICLE_FROM_LEFT: 3,
    VEHICLE_FROM_RIGHT: 3,
    VEHICLE_APPROACHING: 3,
    # Said the moment the sentence playing ends; the camera's vehicle cuts in.
    **{t: 2 for t in _MOVER_SENTENCES},
    **{t: 1 for t in STEER_PAST.values()},
    **{t: 1 for t in _BETWEEN},
    TURN_SLIGHT_LEFT: 1,
    TURN_SLIGHT_RIGHT: 1,
    **{t: 2 for t in STEER_IN_FRONT.values()},
    # A fall in front cuts in, with the urgent tone (unified_guidance.is_urgent).
    **{t: (2 if t in _GROUND_STOP else 1) for t in _GROUND_SENTENCES},
}
_BLOCKED = 2
_VEHICLE = 3
