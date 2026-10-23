"""BEV occupancy grid — obstacle SHAPE, from a 2D LiDAR."""

import math
import re
import threading
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from config import (
    BEV_BODY_MAX_MM,
    BEV_CELL_MM,
    BEV_DECAY,
    BEV_EXTRACT_HZ,
    LIDAR_STALE_S,
    BEV_GAP_CLEAR_MM,
    BEV_GAP_MAX_BEARING_DEG,
    BEV_GAP_MAX_MM,
    BEV_GAP_MIN_MM,
    BEV_L_FREE,
    BEV_L_HIT,
    BEV_L_MAX,
    BEV_L_MIN,
    BEV_MAX_STEP_DEG,
    BEV_MAX_STEP_MM,
    BEV_MIN_CLUSTER_CELLS,
    BEV_MIN_MOTION_QUALITY,
    BEV_MIN_WEDGE_DEG,
    BEV_OCC_THRESH,
    BEV_POLE_MAX_MM,
    BEV_PROFILE_BIN_DEG,
    BEV_PROFILE_MIN_POINTS,
    BEV_RADAR_ARC_THICK_MM,
    BEV_RADAR_STAMP_ENABLED,
    BEV_RANGE_MM,
    BEV_WALL_MAX_THICK_MM,
    BEV_WALL_MIN_LEN_MM,
    RADAR_BEAM_H_DEG,
    WALK_GLASS_GAP_FT,
)

# A per-board radar row: config.radar_board_key() of a measured bearing.
_BOARD_KEY = re.compile(r"^deg(\d{3})$")

try:
    import cv2

    _CV2_ERROR = None
except Exception as exc:  # pragma: no cover - exercised only on a broken install
    cv2 = None
    _CV2_ERROR = str(exc)

MM_PER_FT = 304.8

_CAM_SECTORS = ("front", "right", "back", "left")


def _bearing_to_camera_sector(bearing_deg: float) -> str:
    """Which camera quadrant covers this bearing (front = 0° ± 45°)."""
    idx = int(((bearing_deg % 360.0) + 45.0) % 360.0 // 90.0)
    return _CAM_SECTORS[idx]


def _angular_gap(a: float, b: float) -> float:
    """Smallest absolute angular difference between two bearings (degrees)."""
    return abs((a - b + 180.0) % 360.0 - 180.0)


class BEVGrid:
    """One ego-centred log-odds occupancy grid, updated scan by scan."""

    def __init__(self, range_mm: int = BEV_RANGE_MM, cell_mm: int = BEV_CELL_MM):
        self.cell_mm = float(cell_mm)
        self.range_mm = float(range_mm)
        self.half = int(round(range_mm / cell_mm))
        self.n = 2 * self.half + 1
        self.centre = self.half

        self.logodds = np.zeros((self.n, self.n), dtype=np.float32)
        self.frames = 0
        self.last_time: Optional[float] = None
        self.last_profile: Optional[np.ndarray] = None
        self.last_reset_reason: Optional[str] = "no scan yet"

        self._radar_arcs: List[Dict[str, Any]] = []
        self._radar_cells = np.zeros((self.n, self.n), dtype=bool)

        self._lock = threading.Lock()
        self._snapshot: Dict[str, Any] = {
            "available": cv2 is not None,
            "error": _CV2_ERROR,
            "objects": [],
            "gaps": [],
            "unknown_wedges": [],
            "last_update": 0.0,
        }
        self._last_extract: Optional[float] = None  # monotonic

    # ---- geometry ---------------------------------------------------------

    def _to_cells(
        self, bearing_deg: np.ndarray, dist_mm: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Bearings/ranges -> (row, col), forward drawn UP and right RIGHT."""
        th = np.radians(bearing_deg)
        r = dist_mm / self.cell_mm
        col = self.centre + r * np.sin(th)
        row = self.centre - r * np.cos(th)
        return row, col

    # ---- per-scan update --------------------------------------------------

    def update(
        self,
        points: Sequence[Tuple[float, float]],
        heading: Optional[Dict[str, Any]] = None,
        radar: Optional[Dict[str, Any]] = None,
        now: Optional[float] = None,
    ) -> None:
        """Fold one scan into the grid."""
        if cv2 is None or not points:
            return

        now = time.time() if now is None else now
        dt = None if self.last_time is None else (now - self.last_time)
        self.last_time = now

        self._fade_and_register(heading, dt)

        bearings = np.asarray([p[0] for p in points], dtype=np.float32)
        dists = np.asarray([p[1] for p in points], dtype=np.float32)
        inside = (dists > 0) & (dists <= self.range_mm)

        profile = self._build_profile(bearings, dists)
        self.last_profile = profile

        self._apply_free(profile)
        self._apply_hits(bearings[inside], dists[inside])
        self._radar_arcs = []
        self._radar_cells.fill(False)
        if BEV_RADAR_STAMP_ENABLED:
            self._stamp_radar(radar, profile)

        np.clip(self.logodds, BEV_L_MIN, BEV_L_MAX, out=self.logodds)
        self.frames += 1

    def _fade_and_register(
        self, heading: Optional[Dict[str, Any]], dt: Optional[float]
    ) -> None:
        """Age the accumulated evidence and move it under the user."""
        if self.frames == 0:
            self.last_reset_reason = None
            return

        self.logodds *= BEV_DECAY

        step = self._motion_step(heading, dt)
        if step is None:
            self.logodds.fill(0.0)
            return

        dyaw_deg, dfwd_mm, dright_mm = step
        self.last_reset_reason = None

        m = cv2.getRotationMatrix2D(
            (float(self.centre), float(self.centre)), float(dyaw_deg), 1.0
        )
        m[1, 2] += dfwd_mm / self.cell_mm
        m[0, 2] -= dright_mm / self.cell_mm

        self.logodds = cv2.warpAffine(
            self.logodds,
            m,
            (self.n, self.n),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0.0,
        )

    def _motion_step(
        self, heading: Optional[Dict[str, Any]], dt: Optional[float]
    ) -> Optional[Tuple[float, float, float]]:
        """How far the user moved since the last scan, or None if not trusted."""
        if not heading or dt is None or not (0.005 < dt < 1.0):
            self.last_reset_reason = "no usable inter-scan interval"
            return None

        yaw_q = float(heading.get("yaw_quality") or 0.0)
        from_gyro = heading.get("yaw_source") == "gyro"
        trans_q = float(heading.get("translation_quality") or 0.0)
        if not (from_gyro or yaw_q >= BEV_MIN_MOTION_QUALITY):
            self.last_reset_reason = f"yaw quality {yaw_q:.2f} below threshold"
            return None

        scan_yaw = heading.get("scan_yaw_deg")
        if scan_yaw is None:
            self.last_reset_reason = "no rotation measured for this scan"
            return None
        dyaw = float(scan_yaw)

        if trans_q >= BEV_MIN_MOTION_QUALITY and heading.get("speed_observable"):
            # heading_tracker always gives this scan's step (None: not measured); a
            # speed x dt fallback here could never run.
            dfwd = float(heading.get("scan_fwd_mm") or 0.0)
            dright = float(heading.get("scan_right_mm") or 0.0)
        else:
            dfwd = dright = 0.0

        if abs(dyaw) > BEV_MAX_STEP_DEG or math.hypot(dfwd, dright) > BEV_MAX_STEP_MM:
            self.last_reset_reason = "implausible step (failed match or clock stall)"
            return None
        return dyaw, dfwd, dright

    # ---- evidence ---------------------------------------------------------

    def _build_profile(self, bearings: np.ndarray, dists: np.ndarray) -> np.ndarray:
        """Nearest return per bearing bin; NaN where the bin saw nothing."""
        nbins = int(round(360.0 / BEV_PROFILE_BIN_DEG))
        prof = np.full(nbins, np.inf, dtype=np.float32)
        counts = np.zeros(nbins, dtype=np.int32)

        valid = dists > 0
        if not np.any(valid):
            return np.full(nbins, np.nan, dtype=np.float32)

        idx = (bearings[valid] / BEV_PROFILE_BIN_DEG).astype(np.int32) % nbins
        np.minimum.at(prof, idx, np.minimum(dists[valid], self.range_mm))
        np.add.at(counts, idx, 1)

        prof[counts < BEV_PROFILE_MIN_POINTS] = np.nan
        prof[np.isinf(prof)] = np.nan
        return prof

    def _apply_free(self, profile: np.ndarray) -> None:
        """Subtract evidence from every cell the beams passed through."""
        nbins = len(profile)
        th = np.radians(np.arange(nbins, dtype=np.float32) * BEV_PROFILE_BIN_DEG)
        r = np.where(np.isnan(profile), 0.0, profile) / self.cell_mm

        cols = np.clip(self.centre + r * np.sin(th), 0, self.n - 1)
        rows = np.clip(self.centre - r * np.cos(th), 0, self.n - 1)
        poly = np.stack([cols, rows], axis=1).astype(np.int32).reshape(-1, 1, 2)

        free = np.zeros((self.n, self.n), dtype=np.uint8)
        cv2.fillPoly(free, [poly], 1)
        self.logodds -= free.astype(np.float32) * BEV_L_FREE

    def _apply_hits(self, bearings: np.ndarray, dists: np.ndarray) -> None:
        """Add evidence at every cell a return actually landed in."""
        if bearings.size == 0:
            return
        rows, cols = self._to_cells(bearings, dists)
        rr = np.clip(np.rint(rows), 0, self.n - 1).astype(np.int32)
        cc = np.clip(np.rint(cols), 0, self.n - 1).astype(np.int32)
        np.add.at(self.logodds, (rr, cc), BEV_L_HIT)

    def _stamp_radar(
        self, radar: Optional[Dict[str, Any]], profile: np.ndarray
    ) -> None:
        """Draw the radar's echo as occupied, where the LiDAR saw through it."""
        sectors = (radar or {}).get("sectors") or {}
        gap_mm = WALK_GLASS_GAP_FT * MM_PER_FT

        for key, sec in sectors.items():
            board = _BOARD_KEY.match(str(key))
            if not board or not sec.get("online") or sec.get("degraded"):
                continue
            bearing = float(board.group(1)) % 360.0
            rmm = sec.get("min_dist")
            if not isinstance(rmm, (int, float)) or rmm <= 0 or rmm > self.range_mm:
                continue

            lidar_mm = self._profile_min(profile, bearing, RADAR_BEAM_H_DEG)
            if lidar_mm is not None and (lidar_mm - rmm) < gap_mm:
                continue

            self._draw_arc(bearing, float(rmm), RADAR_BEAM_H_DEG)
            self._radar_arcs.append(
                {
                    "board": key,
                    "bearing_deg": round(bearing, 1),
                    "range_mm": round(float(rmm), 1),
                    "lidar_mm": None if lidar_mm is None else round(lidar_mm, 1),
                    "beam_deg": RADAR_BEAM_H_DEG,
                }
            )

    def _profile_min(
        self, profile: np.ndarray, centre_deg: float, width_deg: float
    ) -> Optional[float]:
        """Nearest LiDAR return across an arc, or None if the arc was silent."""
        nbins = len(profile)
        half = width_deg / 2.0
        lo = int(round((centre_deg - half) / BEV_PROFILE_BIN_DEG))
        hi = int(round((centre_deg + half) / BEV_PROFILE_BIN_DEG))
        idx = np.arange(lo, hi + 1) % nbins
        vals = profile[idx]
        vals = vals[~np.isnan(vals)]
        return float(vals.min()) if vals.size else None

    def _draw_arc(self, centre_deg: float, radius_mm: float, width_deg: float) -> None:
        """Stamp an occupied arc."""
        radius_cells = int(round(radius_mm / self.cell_mm))
        if radius_cells <= 0:
            return
        thick = max(1, int(round(BEV_RADAR_ARC_THICK_MM / self.cell_mm)))

        start = -(centre_deg + width_deg / 2.0) + 90.0
        end = -(centre_deg - width_deg / 2.0) + 90.0

        arc = np.zeros((self.n, self.n), dtype=np.uint8)
        cv2.ellipse(
            arc,
            (self.centre, self.centre),
            (radius_cells, radius_cells),
            0.0,
            -end,
            -start,
            1,
            thickness=thick,
        )
        hit = arc > 0
        self.logodds[hit] = BEV_L_MAX
        self._radar_cells |= hit

    # ---- extraction -------------------------------------------------------

    def extract(self, camera: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Turn the current grid into named objects, openings and blind wedges."""
        t0 = time.perf_counter()
        occ = (self.logodds > BEV_OCC_THRESH).astype(np.uint8)

        kernel = np.ones((3, 3), np.uint8)
        occ = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, kernel)

        objects = self._clusters(occ, camera)
        gaps = self._gaps(objects, occ)
        wedges = self._unknown_wedges()

        return {
            "available": True,
            "error": None,
            "objects": objects,
            "gaps": gaps,
            "unknown_wedges": wedges,
            "radar_arcs": list(self._radar_arcs),
            "cells_occupied": int(occ.sum()),
            "frames": self.frames,
            "registered": self.last_reset_reason is None,
            "reset_reason": self.last_reset_reason,
            "compute_ms": round((time.perf_counter() - t0) * 1000.0, 2),
            "grid": {
                "cell_mm": self.cell_mm,
                "range_mm": self.range_mm,
                "size": self.n,
            },
            "last_update": time.time(),
        }

    def _clusters(
        self, occ: np.ndarray, camera: Optional[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """Label the occupied cells and describe each blob's shape."""
        count, labels = cv2.connectedComponents(occ, connectivity=8)
        if count <= 1:
            return []

        rows, cols = np.nonzero(occ)
        lab = labels[rows, cols]

        order = np.argsort(lab, kind="stable")
        rows, cols, lab = rows[order], cols[order], lab[order]
        bounds = np.searchsorted(lab, np.arange(1, count))
        bounds = np.append(bounds, len(lab))

        out: List[Dict[str, Any]] = []
        start = 0
        for end in bounds:
            if end - start >= BEV_MIN_CLUSTER_CELLS:
                obj = self._describe(rows[start:end], cols[start:end], camera)
                if obj is not None:
                    out.append(obj)
            start = end

        out.sort(key=lambda o: o["range_mm"])
        return out

    def _describe(
        self, rows: np.ndarray, cols: np.ndarray, camera: Optional[Dict[str, Any]]
    ) -> Optional[Dict[str, Any]]:
        """Shape, position and identity candidates for one cluster."""
        fwd = (self.centre - rows.astype(np.float32)) * self.cell_mm
        right = (cols.astype(np.float32) - self.centre) * self.cell_mm
        pts = np.stack([fwd, right], axis=1)

        dist = np.hypot(fwd, right)
        k = int(np.argmin(dist))
        near_mm = float(dist[k])
        near_bearing = float(np.degrees(np.arctan2(right[k], fwd[k])) % 360.0)

        centroid = pts.mean(axis=0)
        centred = pts - centroid
        try:
            cov = np.cov(centred.T)
            _, vecs = np.linalg.eigh(cov)
        except np.linalg.LinAlgError:
            return None
        major_ax, minor_ax = vecs[:, -1], vecs[:, 0]

        pm = centred @ major_ax
        pn = centred @ minor_ax
        length = float(pm.max() - pm.min()) + self.cell_mm
        width = float(pn.max() - pn.min()) + self.cell_mm

        kind = self._classify(length, width)

        end_a = (centroid + major_ax * float(pm.min())).tolist()
        end_b = (centroid + major_ax * float(pm.max())).tolist()

        cam_sector = _bearing_to_camera_sector(near_bearing)
        return {
            "kind": kind,
            "range_mm": round(near_mm, 1),
            "range_ft": round(near_mm / MM_PER_FT, 2),
            "bearing_deg": round(near_bearing, 1),
            "clock": self._clock(near_bearing),
            "length_mm": round(length, 1),
            "width_mm": round(width, 1),
            "cells": int(len(rows)),
            "centroid_mm": [round(float(centroid[0]), 1), round(float(centroid[1]), 1)],
            "ends_mm": [[round(v, 1) for v in end_a], [round(v, 1) for v in end_b]],
            "radar_confirmed": self._radar_backed(rows, cols),
            "camera_sector": cam_sector,
            "camera_candidates": self._camera_labels(camera, cam_sector),
        }

    @staticmethod
    def _classify(length_mm: float, width_mm: float) -> str:
        """Name a shape, or decline to."""
        if length_mm >= BEV_WALL_MIN_LEN_MM and width_mm <= BEV_WALL_MAX_THICK_MM:
            return "wall"
        if length_mm <= BEV_POLE_MAX_MM:
            return "pole"
        if length_mm <= BEV_BODY_MAX_MM:
            return "body"
        return "obstacle"

    @staticmethod
    def _clock(bearing_deg: float) -> int:
        """Bearing as a clock face position, the unit the spoken cues use."""
        hour = int(round((bearing_deg % 360.0) / 30.0)) % 12
        return 12 if hour == 0 else hour

    def _radar_backed(self, rows: np.ndarray, cols: np.ndarray) -> bool:
        """Does this cluster contain cells the radar stamped?"""
        return bool(self._radar_cells[rows, cols].any())

    @staticmethod
    def _camera_labels(camera: Optional[Dict[str, Any]], sector: str) -> List[str]:
        """Distinct class names the camera currently reports in that quadrant."""
        dets = ((camera or {}).get(sector) or {}).get("detections") or []
        seen: List[str] = []
        for d in dets:
            name = d.get("class_name")
            if name and name not in seen:
                seen.append(name)
        return seen

    def _gaps(
        self, objects: List[Dict[str, Any]], occ: np.ndarray
    ) -> List[Dict[str, Any]]:
        """Passable openings between the facing ends of two wall segments."""
        walls = [o for o in objects if o["kind"] == "wall"]
        out: List[Dict[str, Any]] = []

        for i in range(len(walls)):
            for j in range(i + 1, len(walls)):
                a, b = walls[i], walls[j]
                best = None
                for pa in a["ends_mm"]:
                    for pb in b["ends_mm"]:
                        d = math.dist(pa, pb)
                        if best is None or d < best[0]:
                            best = (d, pa, pb)
                if best is None:
                    continue

                span, pa, pb = best
                if not (BEV_GAP_MIN_MM <= span <= BEV_GAP_MAX_MM):
                    continue

                mid = ((pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0)
                bearing = math.degrees(math.atan2(mid[1], mid[0])) % 360.0
                if _angular_gap(bearing, 0.0) > BEV_GAP_MAX_BEARING_DEG:
                    continue
                if not self._clear_through(occ, mid, bearing):
                    continue

                rng = math.hypot(mid[0], mid[1])
                out.append(
                    {
                        "width_mm": round(span, 1),
                        "width_ft": round(span / MM_PER_FT, 2),
                        "bearing_deg": round(bearing, 1),
                        "clock": self._clock(bearing),
                        "range_mm": round(rng, 1),
                        "range_ft": round(rng / MM_PER_FT, 2),
                        "between": [a["range_mm"], b["range_mm"]],
                    }
                )

        out.sort(key=lambda g: g["range_mm"])
        return out

    def _clear_through(
        self, occ: np.ndarray, mid_mm: Tuple[float, float], bearing_deg: float
    ) -> bool:
        """Is there open ground BEYOND the opening, or is it an alcove?"""
        th = math.radians(bearing_deg)
        steps = max(1, int(round(BEV_GAP_CLEAR_MM / self.cell_mm)))
        for s in range(1, steps + 1):
            d = s * self.cell_mm
            fwd = mid_mm[0] + d * math.cos(th)
            right = mid_mm[1] + d * math.sin(th)
            r = int(round(self.centre - fwd / self.cell_mm))
            c = int(round(self.centre + right / self.cell_mm))
            if not (0 <= r < self.n and 0 <= c < self.n):
                return True
            if occ[r, c]:
                return False
        return True

    def _unknown_wedges(self) -> List[Dict[str, Any]]:
        """Arcs the LiDAR returned nothing across."""
        prof = self.last_profile
        if prof is None:
            return []

        nbins = len(prof)
        missing = np.isnan(prof)
        if not missing.any() or missing.all():
            return []

        first_seen = int(np.argmin(missing))
        rolled = np.roll(missing, -first_seen)

        out: List[Dict[str, Any]] = []
        run_start = None
        for i in range(nbins + 1):
            hot = bool(rolled[i]) if i < nbins else False
            if hot and run_start is None:
                run_start = i
            elif not hot and run_start is not None:
                width = (i - run_start) * BEV_PROFILE_BIN_DEG
                if width >= BEV_MIN_WEDGE_DEG:
                    start_deg = ((run_start + first_seen) * BEV_PROFILE_BIN_DEG) % 360.0
                    out.append(
                        {
                            "start_deg": round(start_deg, 1),
                            "width_deg": round(width, 1),
                            "centre_deg": round((start_deg + width / 2.0) % 360.0, 1),
                        }
                    )
                run_start = None
        return out

    # ---- publishing -------------------------------------------------------

    def maybe_extract(self, camera: Optional[Dict[str, Any]] = None) -> None:
        """Run extraction if its rate budget allows, and publish the result."""
        if cv2 is None:
            return
        # Monotonic: on the wall clock a backward step stopped extraction until the clock
        # caught up, and the last snapshot was served all the while (scan 2026-10-23).
        mono = time.monotonic()
        if self._last_extract is not None and (mono - self._last_extract) < (
            1.0 / max(1e-3, BEV_EXTRACT_HZ)
        ):
            return
        self._last_extract = mono
        snap = self.extract(camera)
        with self._lock:
            self._snapshot = snap

    def snapshot(self) -> Dict[str, Any]:
        """The most recent extraction, `stale` once it is older than LIDAR_STALE_S or three
        extraction periods. Safe to call from any thread."""
        with self._lock:
            out = dict(self._snapshot)
            made = self._last_extract
        out["stale"] = made is None or time.monotonic() - made > max(
            LIDAR_STALE_S, 3.0 / max(1e-3, BEV_EXTRACT_HZ)
        )
        return out

    def render_png(self, scale: int = 2) -> Optional[bytes]:
        """Top-down debug image: red occupied, dark free, grey unknown."""
        if cv2 is None:
            return None
        img = np.full((self.n, self.n, 3), 70, dtype=np.uint8)  # unknown
        img[self.logodds < -BEV_OCC_THRESH] = (35, 35, 35)  # free
        img[self.logodds > BEV_OCC_THRESH] = (60, 60, 230)  # occupied (BGR)

        # Ego marker and a forward tick, so up is unambiguously "ahead".
        cv2.circle(img, (self.centre, self.centre), 3, (0, 255, 255), -1)
        cv2.line(
            img,
            (self.centre, self.centre),
            (self.centre, self.centre - 12),
            (0, 255, 255),
            1,
        )

        if scale > 1:
            img = cv2.resize(
                img, (self.n * scale, self.n * scale), interpolation=cv2.INTER_NEAREST
            )
        ok, buf = cv2.imencode(".png", img)
        return buf.tobytes() if ok else None


_grid: Optional[BEVGrid] = None
_grid_lock = threading.Lock()


def get_grid() -> BEVGrid:
    global _grid
    with _grid_lock:
        if _grid is None:
            _grid = BEVGrid()
        return _grid


def reset_grid() -> None:
    """Drop the accumulated grid."""
    global _grid
    with _grid_lock:
        _grid = None
