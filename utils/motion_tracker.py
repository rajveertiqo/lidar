"""Moving things followed with a Kalman filter — where each is, how fast it goes, how sure
that is, and where it will be (added 2026-10-13, from the walk simulator).

Everything here is in the user's frame at the newest scan, in metres: x ahead, y to the
right, as lidar_movers. Velocities are each thing's OWN, over the ground: the user's turn
and step since the last scan are taken off before each scan is folded in (heading_tracker's
scan_yaw_deg and scan_fwd_mm / scan_right_mm; lidar_movers' still-things estimate when the
odometry has none, and of the turn the heading tracker missed).

Detections: the blobs lidar_movers.segment cuts cleanly (a blob whose end is hidden by
something nearer slides with the view, not by itself), and the people and vehicles the
cameras in MOTION_CAMERA_SECTORS name, placed by bearing (where the box is in the frame)
and range (its height against the class's typical height). A camera is sure of bearing and
unsure of range, so its error is an ellipse along the line of sight, not a circle.

A track is a constant-velocity Kalman filter: state (x, y, vx, vy) and covariance P. A
person's model allows MOTION_ACCEL_PERSON of random acceleration every way; a moving
vehicle's allows MOTION_ACCEL_ALONG along its heading and MOTION_ACCEL_ACROSS across it.
A detection joins the track it most likely came from (Mahalanobis distance inside the
99 % gate, within MOTION_GATE_M of where the track should be, of the same kind); the rest
start tracks. Moving means: followed MOTION_CONFIRM_S and MOTION_CONFIRM_HITS times, faster
than lidar_movers' threshold, and faster than MOTION_SIGNIFICANCE times its own uncertainty
in the direction it goes; a vehicle must also hold its heading and have actually travelled.
"""

import math
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from config import (
    LIDAR_MOVERS_HEAD_ON_DEG,
    LIDAR_MOVERS_HORIZON_S,
    LIDAR_MOVERS_MAX_WIDTH_M,
    LIDAR_MOVERS_MIN_SPEED_MPS,
    LIDAR_MOVERS_MISS_M,
    LIDAR_MOVERS_NOISE_PER_M,
    LIDAR_MOVERS_PERSON_MAX_WIDTH_M,
    LIDAR_MOVERS_VEHICLE_SPEED_MPS,
    LIDAR_STALE_S,
    MOTION_ACCEL_ACROSS,
    MOTION_ACCEL_ALONG,
    MOTION_ACCEL_PERSON,
    MOTION_CAMERA_BEARING_DEG,
    MOTION_CAMERA_MAX_AGE_S,
    MOTION_CAMERA_RANGE_FRAC,
    MOTION_CAMERA_SECTORS,
    MOTION_CONFIRM_HITS,
    MOTION_CONFIRM_S,
    MOTION_EGO_SD_MPS,
    MOTION_GATE_CHI2,
    MOTION_GATE_M,
    MOTION_GATE_NEW_EXTRA_M,
    MOTION_GATE_VEHICLE_M,
    MOTION_LOST_S,
    MOTION_PREDICT_DRIFT_PERSON,
    MOTION_PREDICT_DRIFT_VEHICLE,
    MOTION_PREDICT_MAX_SD_M,
    MOTION_SIGNIFICANCE,
    MOTION_VEHICLE_MIN_TRAVEL_M,
    MOTION_VEHICLE_STEADY_DEG,
    MOTION_VEHICLE_STEADY_S,
    SIMPLE_OBJECT_MIN_CONF,
    SIMPLE_VEHICLE_CLASSES,
    SIMPLE_VEHICLE_MIN_CONF,
    SIMPLE_VEHICLE_MIN_HEIGHT_FRAC,
    SIMPLE_VEHICLE_MISS_M,
    TOF_RANGE_SD_FRAC,
    TOF_RANGE_SD_M,
    TOF_USE_DISTANCE,
)

PERSON, VEHICLE = "Person", "Vehicle"
_SECTOR_BEARING = {"front": 0.0, "right": 90.0, "back": 180.0, "left": -90.0}
_BIG = (
    "truck",
    "bus",
    "train",
    "big_truck",
    "mid_truck",
    "truck_",
    "truck_l",
    "truck_m",
    "truck_xl",
    "big_bu",
    "bus_",
    "bus_l",
    "small_bu",
    "forklift",
)
_SMALL = ("motorcycle", "bicycle", "cyclist")
_LIDAR_SD_M = 0.05  # a blob's centre, scan to scan
_LIDAR_SD_PER_M = 0.02  # and its growth with range
_ALERT_SCANS = 3  # a warning holds this many scans before it counts (CONFIRM_SCANS)


def body(word: Optional[str], cls: Optional[str] = None) -> Tuple[float, float]:
    """(radius m, half-length m along its heading) of a thing's footprint."""
    if word != VEHICLE:
        return 0.3, 0.0
    if cls in _SMALL:
        return 0.45, 0.8
    if cls in _BIG:
        return 1.25, 4.0
    return 0.95, 2.15


def predicted_sd(track: Dict[str, Any], t: float) -> float:
    """How unsure (1 sigma, m) the place a track will be at in `t` s is: where it is now, its
    speed, and a drift for turns and stops; at most MOTION_PREDICT_MAX_SD_M."""
    drift = (
        MOTION_PREDICT_DRIFT_VEHICLE
        if track.get("word") == VEHICLE
        else MOTION_PREDICT_DRIFT_PERSON
    ) * t
    sd = math.sqrt(
        float(track["pos_var"]) + float(track["vel_var"]) * t * t + drift * drift
    )
    return min(MOTION_PREDICT_MAX_SD_M, sd)


def gap_at(track: Dict[str, Any], t: float, px: float, py: float) -> float:
    """Metres from the point (px, py) to the edge of the track's body `t` s from now."""
    cx, cy = track["x"] + track["vx"] * t, track["y"] + track["vy"] * t
    dx, dy = px - cx, py - cy
    half = float(track.get("half") or 0.0)
    speed = math.hypot(track["vx"], track["vy"])
    if half and speed > 0.5:
        ux, uy = track["vx"] / speed, track["vy"] / speed
        a = max(-half, min(half, dx * ux + dy * uy))
        dx, dy = dx - ux * a, dy - uy * a
    return math.hypot(dx, dy) - float(track.get("rad") or 0.3)


class MotionTracker:
    """Kalman tracks of everything the LiDAR and the cameras follow; which move, where to."""

    def __init__(self):
        self._lock = threading.Lock()
        self._tracks: List[Dict[str, Any]] = []
        self._next_id = 1
        self._last_t: Optional[float] = None
        self._cam_seen: Dict[str, float] = {}
        self._own = {
            "fwd_mps": 0.0,
            "right_mps": 0.0,
            "sd_mps": MOTION_EGO_SD_MPS,
            "source": "none",
        }
        self._snap: Dict[str, Any] = self._empty()

    @staticmethod
    def _empty() -> Dict[str, Any]:
        return {
            "t": None,
            "updated_mono": None,
            "tracks": [],
            "threat": None,
            "own": {
                "fwd_mps": 0.0,
                "right_mps": 0.0,
                "sd_mps": MOTION_EGO_SD_MPS,
                "source": "none",
            },
            "followed": 0,
        }

    def reset(self) -> None:
        with self._lock:
            self.__init__()

    # ---- one scan ---------------------------------------------------------

    def update(
        self,
        points,
        t: float,
        heading: Optional[Dict[str, Any]] = None,
        camera_state: Optional[Dict[str, Any]] = None,
        ego: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Fold one scan (and any new camera frame) in. `heading` is heading_tracker's state
        for this scan, `ego` lidar_movers' still-things estimate of the user's own motion."""
        from utils.lidar_movers import segment

        blobs = [b for b in segment(points) if b["clean"]]
        with self._lock:
            if self._last_t is not None and not (
                0.0 < t - self._last_t <= MOTION_LOST_S
            ):
                self._tracks = []  # a gap, or the clock went backwards
                self._last_t = None
                # And the cameras' last stamps with them: after a backward step they lie in
                # the future, and every new frame was refused as "already seen" for as long
                # as the step (fuzz test 2026-10-22).
                self._cam_seen = {}
            dt = 0.0 if self._last_t is None else t - self._last_t
            self._last_t = t
            fwd, right, sd, speed_sd, source = self._own_step(heading, ego, dt)
            if dt > 0:
                self._own = {
                    "fwd_mps": fwd / dt,
                    "right_mps": right / dt,
                    "sd_mps": speed_sd,
                    "source": source,
                }
            yaw_deg = (heading or {}).get("scan_yaw_deg")
            yaw = (
                math.radians(float(yaw_deg))
                if isinstance(yaw_deg, (int, float))
                else 0.0
            )
            # The turn the still things still show once the heading's is taken off (lidar_movers'
            # fit): the heading tracker's own error, which would swing every track sideways.
            if (
                ego
                and ego.get("valid")
                and isinstance(ego.get("turn_dps"), (int, float))
            ):
                yaw += math.radians(float(ego["turn_dps"])) * dt
            for tr in self._tracks:
                self._predict(tr, dt)
                self._carry(tr, fwd, right, sd, yaw)
            self._join([self._blob_det(b) for b in blobs], t)
            yaw_rate = yaw / dt if dt > 0 else 0.0  # rad/s, right: this scan's turn
            self._join(self._camera_dets(camera_state, t, yaw_rate), t)
            self._tracks = [
                tr for tr in self._tracks if t - tr["seen"] <= MOTION_LOST_S
            ]
            for tr in self._tracks:
                self._judge(tr, t)
            self._snap = self._snapshot(t)

    @staticmethod
    def _own_step(
        heading: Optional[Dict[str, Any]], ego: Optional[Dict[str, Any]], dt: float
    ) -> Tuple[float, float, float, float, str]:
        """(forward m, right m, that step's 1-sigma m, the user's speed 1-sigma m/s, where
        from) the user moved since the last scan. A step's scatter averages out over scans;
        the speed's doubt is what may make a still thing look as if it moved."""
        h = heading or {}
        fwd, right = h.get("scan_fwd_mm"), h.get("scan_right_mm")
        if isinstance(fwd, (int, float)) and not isinstance(fwd, bool):
            side = float(right) / 1000.0 if isinstance(right, (int, float)) else 0.0
            se = h.get("speed_error_mps")
            return (
                float(fwd) / 1000.0,
                side,
                0.03 + (0.0 if isinstance(right, (int, float)) else 0.3 * dt),
                float(se) if isinstance(se, (int, float)) else 0.1,
                "odometry",
            )
        if ego and ego.get("valid"):
            rms = max(0.05, float(ego.get("rms_mps") or 0.0))
            return (
                float(ego["fwd_mps"]) * dt,
                float(ego["right_mps"]) * dt,
                rms * dt,
                rms,
                "still things",
            )
        if h.get("speed_trusted"):
            return (
                float(h.get("speed_mps") or 0.0) * dt,
                0.0,
                0.2 * dt,
                0.2,
                str(h.get("speed_source")),
            )
        if h.get("confidently_still"):
            return 0.0, 0.0, 0.1 * dt, 0.1, "standing still"
        pace = float(h.get("speed_mps") or 0.0) if h.get("moving") else 0.0
        return pace * dt, 0.0, MOTION_EGO_SD_MPS * dt, MOTION_EGO_SD_MPS, "assumed"

    @staticmethod
    def _accel_cov(tr: Dict[str, Any]) -> np.ndarray:
        s = tr["s"]
        speed = math.hypot(s[2], s[3])
        if tr["word"] != VEHICLE or speed < 1.5:
            q = (
                MOTION_ACCEL_PERSON if tr["word"] != VEHICLE else MOTION_ACCEL_ALONG
            ) ** 2
            return np.eye(2) * q
        u = np.array([s[2], s[3]]) / speed
        n = np.array([-u[1], u[0]])
        return (MOTION_ACCEL_ALONG**2) * np.outer(u, u) + (
            MOTION_ACCEL_ACROSS**2
        ) * np.outer(n, n)

    def _predict(self, tr: Dict[str, Any], dt: float) -> None:
        if dt <= 0:
            return
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        g = np.array([[dt * dt / 2.0, 0.0], [0.0, dt * dt / 2.0], [dt, 0.0], [0.0, dt]])
        tr["s"] = f @ tr["s"]
        tr["P"] = f @ tr["P"] @ f.T + g @ self._accel_cov(tr) @ g.T

    @staticmethod
    def _carry(
        tr: Dict[str, Any], fwd: float, right: float, sd: float, yaw: float
    ) -> None:
        """Into the new scan's frame: the user stepped (fwd, right) and turned `yaw` right."""
        tr["s"][0] -= fwd
        tr["s"][1] -= right
        tr["P"][0, 0] += sd * sd
        tr["P"][1, 1] += sd * sd
        tr["trail"] = [(ts, x - fwd, y - right) for ts, x, y in tr["trail"]]
        if abs(yaw) < 1e-9:
            return
        c, s = math.cos(yaw), math.sin(yaw)
        r2 = np.array([[c, s], [-s, c]])
        r4 = np.zeros((4, 4))
        r4[:2, :2] = r4[2:, 2:] = r2
        tr["s"] = r4 @ tr["s"]
        tr["P"] = r4 @ tr["P"] @ r4.T
        tr["trail"] = [(ts, x * c + y * s, y * c - x * s) for ts, x, y in tr["trail"]]
        tr["hdg"] = [(ts, a - yaw) for ts, a in tr["hdg"]]

    # ---- detections -------------------------------------------------------

    @staticmethod
    def _blob_det(b: Dict[str, Any]) -> Dict[str, Any]:
        sd = _LIDAR_SD_M + _LIDAR_SD_PER_M * float(b["near_m"])
        return {
            "x": float(b["x"]),
            "y": float(b["y"]),
            "R": np.eye(2) * sd * sd,
            "width": float(b["width"]),
            "word": None,
            "cls": None,
            "src": "lidar",
        }

    def _camera_dets(
        self, camera_state: Optional[Dict[str, Any]], t: float, yaw_rate: float = 0.0
    ) -> List[Dict[str, Any]]:
        """The people and vehicles in each camera's newest detector frame, placed on the ground
        in THIS scan's frame: turned back by what the user turned (`yaw_rate`, rad/s right)
        since the picture — a side camera's is up to ~0.25 s old, 15 deg at 60 deg/s (scan
        2026-10-23)."""
        if not camera_state:
            return []
        from utils import simple_guidance as sg

        cam = camera_state
        focal = (sg._QUAD_W / 2.0) / math.tan(
            math.radians(sg.SIMPLE_CAMERA_HFOV_DEG) / 2.0
        )
        out: List[Dict[str, Any]] = []
        for sector in MOTION_CAMERA_SECTORS:
            view = cam.get(sector) or {}
            stamp = float(
                view.get("frame_time")
                or (cam.get("last_update") if sector == "front" else 0.0)
                or 0.0
            )
            if (
                not stamp
                or view.get("simulated")
                or stamp <= self._cam_seen.get(sector, 0.0)
                or abs(t - stamp) > MOTION_CAMERA_MAX_AGE_S
            ):
                continue
            if sector == "front" and not sg._camera_sees(cam, t):
                continue
            self._cam_seen[sector] = stamp
            offset = math.radians(_SECTOR_BEARING.get(sector, 0.0))
            offset -= yaw_rate * max(0.0, t - stamp)
            for det in view.get("detections") or []:
                cls = str(det.get("class_name", "")).lower()
                word = (
                    VEHICLE
                    if cls in SIMPLE_VEHICLE_CLASSES
                    else PERSON
                    if cls in sg._PERSON_CLASSES
                    else None
                )
                if word is None:
                    continue
                need = (
                    SIMPLE_VEHICLE_MIN_CONF
                    if word == VEHICLE
                    else SIMPLE_OBJECT_MIN_CONF
                )
                if float(det.get("confidence") or 0.0) < need:
                    continue
                try:
                    x1, y1, x2, y2 = (float(v) for v in det.get("bbox") or ())
                except (TypeError, ValueError):
                    continue
                if (
                    x2 <= x1
                    or y2 <= y1
                    or (y2 - y1) < SIMPLE_VEHICLE_MIN_HEIGHT_FRAC * sg._QUAD_H
                ):
                    continue
                depth = (
                    det.get("depth")
                    if (TOF_USE_DISTANCE and sector == "front")
                    else None
                )
                if isinstance(depth, dict) and isinstance(
                    depth.get("distance_m"), (int, float)
                ):
                    # Measured by the ToF (utils/depth_fusion.py): range and bearing of
                    # the nearest surface in the box, cut off by the frame or not.
                    rng = float(depth["distance_m"])
                    brg = offset + math.radians(float(depth.get("bearing_deg") or 0.0))
                    sr = TOF_RANGE_SD_M + TOF_RANGE_SD_FRAC * rng
                else:
                    rng, cut = sg._size_range(cls, (x1, y1, x2, y2))
                    if cut:
                        continue  # a box the frame cuts off: its range is a guess
                    brg = offset + math.atan(
                        ((x1 + x2) / 2.0 - sg._QUAD_W / 2.0) / focal
                    )
                    sr = 0.1 + MOTION_CAMERA_RANGE_FRAC * rng
                u = np.array([math.cos(brg), math.sin(brg)])
                n = np.array([-u[1], u[0]])
                sb = 0.05 + rng * math.radians(MOTION_CAMERA_BEARING_DEG)
                out.append(
                    {
                        "x": rng * u[0],
                        "y": rng * u[1],
                        "R": sr * sr * np.outer(u, u) + sb * sb * np.outer(n, n),
                        "width": None,
                        "word": word,
                        "cls": cls,
                        "src": "camera",
                    }
                )
        return out

    @staticmethod
    def _fits(d: Dict[str, Any], tr: Dict[str, Any]) -> bool:
        """May detection `d` be a sighting of track `tr` at all?"""
        if d["word"] and tr["word_src"] == "camera" and d["word"] != tr["word"]:
            return False  # a camera said it is the other kind
        if (
            d["word"] == VEHICLE
            and tr["word_src"] == "lidar"
            and tr["width"] is not None
            and tr["width"] < LIDAR_MOVERS_PERSON_MAX_WIDTH_M
        ):
            return False  # a person-sized blob is not a car
        if d["width"] is not None and tr["width"] is not None:
            if abs(d["width"] - tr["width"]) > max(0.4, 0.5 * tr["width"]):
                return False  # lidar_movers' rule: the same size
        gate = MOTION_GATE_VEHICLE_M if tr["word"] == VEHICLE else MOTION_GATE_M
        if tr["hits"] < MOTION_CONFIRM_HITS:
            gate += MOTION_GATE_NEW_EXTRA_M
        # The detection's own spread, once per detection rather than once per pair: it was
        # ~60% of the tracker's time in clutter (fuzz test 2026-10-22).
        spread = d.get("_gate_spread")
        if spread is None:
            spread = d["_gate_spread"] = 2.0 * math.sqrt(
                float(np.max(np.linalg.eigvalsh(d["R"])))
            )
        gate += spread
        return math.hypot(d["x"] - tr["s"][0], d["y"] - tr["s"][1]) <= gate

    def _join(self, dets: List[Dict[str, Any]], t: float) -> None:
        """Each detection to the track it most likely came from; the rest start tracks."""
        if not dets:
            return
        pairs = []
        for i, d in enumerate(dets):
            z = np.array([d["x"], d["y"]])
            for j, tr in enumerate(self._tracks):
                if not self._fits(d, tr):
                    continue
                y = z - tr["s"][:2]
                s = tr["P"][:2, :2] + d["R"]
                try:
                    m = float(y @ np.linalg.solve(s, y))
                except np.linalg.LinAlgError:
                    continue
                if m <= MOTION_GATE_CHI2:
                    pairs.append((m, i, j))
        used_d, used_t = set(), set()
        for _, i, j in sorted(pairs):
            if i in used_d or j in used_t:
                continue
            used_d.add(i)
            used_t.add(j)
            self._correct(self._tracks[j], dets[i], t)
        for i, d in enumerate(dets):
            if i not in used_d and (
                d["src"] == "camera" or d["width"] <= LIDAR_MOVERS_MAX_WIDTH_M
            ):
                self._tracks.append(self._start(d, t))

    def _start(self, d: Dict[str, Any], t: float) -> Dict[str, Any]:
        v0 = 5.0 if d["word"] == VEHICLE else 2.0
        p = np.zeros((4, 4))
        p[:2, :2] = d["R"]
        p[2, 2] = p[3, 3] = v0 * v0
        tr = {
            "id": self._next_id,
            "s": np.array([d["x"], d["y"], 0.0, 0.0]),
            "P": p,
            "born": t,
            "seen": t,
            "hits": 1,
            "word": d["word"] or PERSON,
            "word_src": d["src"] if d["word"] else "lidar",
            "cls": d["cls"],
            "width": d["width"],
            "trail": [(t, d["x"], d["y"])],
            "hdg": [],
            "moving": False,
            "sources": {d["src"]},
        }
        self._next_id += 1
        return tr

    @staticmethod
    def _correct(tr: Dict[str, Any], d: Dict[str, Any], t: float) -> None:
        """The Kalman update with one sighting."""
        h = np.zeros((2, 4))
        h[0, 0] = h[1, 1] = 1.0
        s = tr["P"][:2, :2] + d["R"]
        k = tr["P"] @ h.T @ np.linalg.inv(s)
        tr["s"] = tr["s"] + k @ (np.array([d["x"], d["y"]]) - tr["s"][:2])
        tr["P"] = (np.eye(4) - k @ h) @ tr["P"]
        tr["P"] = (tr["P"] + tr["P"].T) / 2.0
        tr["seen"] = t
        tr["hits"] += 1
        tr["sources"].add(d["src"])
        if d["word"] and (d["src"] == "camera" or tr["word_src"] != "camera"):
            tr["word"], tr["word_src"], tr["cls"] = (
                d["word"],
                d["src"],
                d["cls"] or tr["cls"],
            )
        if d["width"] is not None:
            tr["width"] = (
                d["width"]
                if tr["width"] is None
                else 0.7 * tr["width"] + 0.3 * d["width"]
            )

    # ---- judging ----------------------------------------------------------

    def _judge(self, tr: Dict[str, Any], t: float) -> None:
        """Is it moving by itself?"""
        s, p = tr["s"], tr["P"]
        speed = math.hypot(s[2], s[3])
        if speed > 1e-6:
            u = np.array([s[2], s[3]]) / speed
        else:
            u = np.array([1.0, 0.0])
        vel_sd_dir = math.sqrt(max(0.0, float(u @ p[2:, 2:] @ u)))
        tr["speed"], tr["vel_sd_dir"] = speed, vel_sd_dir
        tr["trail"].append((t, float(s[0]), float(s[1])))
        tr["trail"] = [e for e in tr["trail"] if t - e[0] <= 1.0]
        if speed > 1.0:
            tr["hdg"].append((t, math.atan2(s[3], s[2])))
        tr["hdg"] = [e for e in tr["hdg"] if t - e[0] <= MOTION_VEHICLE_STEADY_S + 0.4]
        if (
            tr["word_src"] != "camera"
            and tr["width"] is not None
            and (
                tr["width"] > LIDAR_MOVERS_PERSON_MAX_WIDTH_M
                or speed >= LIDAR_MOVERS_VEHICLE_SPEED_MPS
            )
        ):
            tr["word"] = VEHICLE
        r = math.hypot(s[0], s[1])
        walk = math.hypot(self._own["fwd_mps"], self._own["right_mps"])
        need = (
            max(LIDAR_MOVERS_MIN_SPEED_MPS, LIDAR_MOVERS_NOISE_PER_M * r)
            + self._own["sd_mps"]
            # Walking past something near shows a different face of it each scan (lidar_movers).
            + 0.5 * float(tr["width"] or 0.0) * walk / max(r, 0.3)
        )
        if tr["moving"]:
            need *= 0.7  # hysteresis: let go more slowly
        steady = True
        if tr["word"] == VEHICLE:
            h = tr["hdg"]
            last = h[-1][1] if h else 0.0
            steady = (
                len(h) > 2
                and t - h[0][0] >= MOTION_VEHICLE_STEADY_S - 1e-6
                and all(
                    abs((a - last + math.pi) % (2 * math.pi) - math.pi)
                    <= math.radians(MOTION_VEHICLE_STEADY_DEG)
                    for _, a in h
                )
            )
            o = tr["trail"][0]
            steady = (
                steady
                and t - o[0] >= MOTION_VEHICLE_STEADY_S - 1e-6
                and math.hypot(s[0] - o[1], s[1] - o[2])
                >= max(MOTION_VEHICLE_MIN_TRAVEL_M, 0.5 * speed)
            )
        tr["steady"] = steady
        tr["moving"] = bool(
            tr["hits"] >= MOTION_CONFIRM_HITS
            and t - tr["born"] >= MOTION_CONFIRM_S - 1e-6
            and speed >= need
            and speed >= MOTION_SIGNIFICANCE * vel_sd_dir
            and steady
            and (
                tr["word_src"] == "camera"
                or tr["width"] is None
                or tr["width"] <= LIDAR_MOVERS_PERSON_MAX_WIDTH_M
                or tr["word"] == VEHICLE
            )
        )

    def _view(self, tr: Dict[str, Any]) -> Dict[str, Any]:
        """A track as the planner and the dashboard read it (plain floats, metres)."""
        s, p = tr["s"], tr["P"]
        rad, half = body(tr["word"], tr["cls"])
        own_fwd, own_right = self._own["fwd_mps"], self._own["right_mps"]
        x, y = float(s[0]), float(s[1])
        return {
            "source": "kalman",
            "id": tr["id"],
            "word": tr["word"],
            "cls": tr["cls"],
            "x": x,
            "y": y,
            "vx": float(s[2]),
            "vy": float(s[3]),
            "pos_var": float((p[0, 0] + p[1, 1]) / 2.0),
            "vel_var": float((p[2, 2] + p[3, 3]) / 2.0),
            "rad": rad,
            "half": half,
            "moving": bool(tr["moving"]),
            "speed": round(float(tr["speed"]), 3),
            "speed_sd": round(float(tr["vel_sd_dir"]), 3),
            "width": None if tr["width"] is None else round(float(tr["width"]), 2),
            "sources": sorted(tr["sources"]),
            # lidar_movers' fields, so mover_sentence and the dashboard read it unchanged
            "x_m": round(x, 2),
            "y_m": round(y, 2),
            "distance_m": round(math.hypot(x, y), 2),
            "bearing_deg": round(math.degrees(math.atan2(y, x)), 1),
            "vx_mps": round(float(s[2]) - own_fwd, 2),
            "vy_mps": round(float(s[3]) - own_right, 2),
            "own_speed_mps": round(float(tr["speed"]), 2),
            "width_m": None if tr["width"] is None else round(float(tr["width"]), 2),
        }

    def _alert(self, v: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """lidar_movers' closest-approach warning, from the filtered speeds: the user keeps
        walking as now, and it passes within LIDAR_MOVERS_MISS_M (a vehicle
        SIMPLE_VEHICLE_MISS_M) inside LIDAR_MOVERS_HORIZON_S."""
        vehicle = v["word"] == VEHICLE
        x, y = v["x"], v["y"]
        vx, vy = v["vx_mps"], v["vy_mps"]  # relative to the walking user
        pv, vv = x * vx + y * vy, vx * vx + vy * vy
        if pv >= 0.0 or vv < 1e-6:
            return None  # not closing
        t_cpa = -pv / vv
        cx, cy = x + vx * t_cpa, y + vy * t_cpa
        miss = math.hypot(cx, cy)
        allow = SIMPLE_VEHICLE_MISS_M if vehicle else LIDAR_MOVERS_MISS_M
        if t_cpa > LIDAR_MOVERS_HORIZON_S or miss > allow:
            return None
        r = math.hypot(x, y)
        bearing = math.degrees(math.atan2(y, x))
        if abs(bearing) > 135.0:
            mode, side = "behind", "left" if bearing < 0 else "right"
        elif abs(bearing) <= 45.0:
            head_on = -pv / (math.sqrt(vv) * max(r, 1e-6))
            if head_on >= math.cos(math.radians(LIDAR_MOVERS_HEAD_ON_DEG)):
                mode, side = "toward", "left" if bearing < 0 else "right"
            else:  # crossing: from the side it moves away from
                mode, side = "crossing", "left" if v["vy"] > 0 else "right"
        else:
            mode, side = "crossing", "left" if bearing < 0 else "right"
        return {
            "mode": mode,
            "side": side,
            "pass_side": "right" if cy > 0.15 else "left" if cy < -0.15 else None,
            "ttc_s": round(t_cpa, 1),
            "miss_m": round(miss, 2),
        }

    def _snapshot(self, t: float) -> Dict[str, Any]:
        views = [self._view(tr) for tr in self._tracks]
        moving = [v for v in views if v["moving"]]
        by_id = {tr["id"]: tr for tr in self._tracks}
        for v in views:
            if not v["moving"]:  # a stop ends the run: moving again holds afresh
                by_id[v["id"]]["alert_scans"] = 0
        for v in moving:
            # Said only once it has held for ALERT_SCANS scans: a young track's heading is rough.
            a = self._alert(v)
            tr = by_id[v["id"]]
            tr["alert_scans"] = tr.get("alert_scans", 0) + 1 if a else 0
            v["alert"] = a if tr["alert_scans"] >= _ALERT_SCANS else None
        alerts = [v for v in moving if v["alert"]]
        threat = None
        if alerts:
            m = min(alerts, key=lambda a: a["alert"]["ttc_s"])
            threat = {**{k: val for k, val in m.items() if k != "alert"}, **m["alert"]}
        return {
            "t": t,
            "updated_mono": time.monotonic(),
            "tracks": moving,
            "threat": threat,
            "own": {
                k: (round(v, 3) if isinstance(v, float) else v)
                for k, v in self._own.items()
            },
            "followed": len(views),
        }

    # ---- read -------------------------------------------------------------

    def snapshot(self) -> Dict[str, Any]:
        """The newest scan's moving things; empty once it is older than LIDAR_STALE_S."""
        with self._lock:
            snap = self._snap
        mono = snap.get("updated_mono")
        if mono is None or time.monotonic() - mono > LIDAR_STALE_S:
            return {**self._empty(), "stale": True}
        return {**snap, "stale": False}


_tracker = MotionTracker()


def get_motion_tracker() -> MotionTracker:
    return _tracker
