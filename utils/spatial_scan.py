"""What KIND of place this is, and which ways lead out of it."""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence

MM_PER_FT = 304.8

_STRUCTURAL_WALL_MM = 1500.0

_PARALLEL_MIN_DOT = 0.9

_FACING_TOLERANCE_DEG = 40.0

_CORRIDOR_MAX_WIDTH_MM = 3000.0

_ENCLOSED_MIN_WALLS = 3

# A direction has to beat this to be offered as a way out at all, in mm.
_MIN_USEFUL_CLEARANCE_MM = 1200.0

_DOORWAY_MIN_MM = 700.0
_DOORWAY_MAX_MM = 1200.0

_DOORWAY_MAX_RANGE_MM = 6000.0


def _norm180(deg: float) -> float:
    """Bearing folded to (-180, 180]."""
    d = (float(deg) + 180.0) % 360.0 - 180.0
    return d if d != -180.0 else 180.0


def _angular_gap(a: float, b: float) -> float:
    """Smallest absolute angle between two bearings, 0-180."""
    return abs(_norm180(a - b))


def _structural_walls(surfaces: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """The walls long enough to describe the shape of the space."""
    return [
        s
        for s in surfaces
        if s.get("kind") == "wall"
        and float(s.get("span_mm") or 0.0) >= _STRUCTURAL_WALL_MM
    ]


def _offsets_about(a: Dict[str, Any], b: Dict[str, Any]) -> tuple:
    """Both walls' perpendicular offsets measured about ONE shared axis, in mm."""
    dx, dy = a.get("direction") or (1.0, 0.0)
    px, py = -float(dy), float(dx)  # perpendicular to a's run

    def off(w):
        cx, cy = w.get("centroid_mm") or (0.0, 0.0)
        return float(cx) * px + float(cy) * py

    return off(a), off(b)


def _parallel(a: Dict[str, Any], b: Dict[str, Any]) -> bool:
    """Do two fitted lines run the same way, either direction?"""
    ax, ay = a.get("direction") or (1.0, 0.0)
    bx, by = b.get("direction") or (1.0, 0.0)
    return abs(float(ax) * float(bx) + float(ay) * float(by)) >= _PARALLEL_MIN_DOT


def _wall_ends(surface: Dict[str, Any]) -> List[tuple]:
    """The two ends of a fitted wall, in mm."""
    cx, cy = surface.get("centroid_mm") or (0.0, 0.0)
    dx, dy = surface.get("direction") or (1.0, 0.0)
    lo, hi = surface.get("extent_mm") or (0.0, 0.0)
    return [
        (float(cx) + float(dx) * float(lo), float(cy) + float(dy) * float(lo)),
        (float(cx) + float(dx) * float(hi), float(cy) + float(dy) * float(hi)),
    ]


def doorways(surfaces: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Passable openings between the facing ends of two fitted walls."""
    walls = [s for s in surfaces if s.get("kind") == "wall"]
    out: List[Dict[str, Any]] = []

    for i in range(len(walls)):
        for j in range(i + 1, len(walls)):
            a, b = walls[i], walls[j]
            best = None
            for pa in _wall_ends(a):
                for pb in _wall_ends(b):
                    d = math.dist(pa, pb)
                    if best is None or d < best[0]:
                        best = (d, pa, pb)
            if best is None:
                continue
            span, pa, pb = best
            if not (_DOORWAY_MIN_MM <= span <= _DOORWAY_MAX_MM):
                continue

            mid = ((pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0)
            bearing = math.degrees(math.atan2(mid[1], mid[0])) % 360.0
            rng = math.hypot(mid[0], mid[1])
            if rng > _DOORWAY_MAX_RANGE_MM:
                continue
            out.append(
                {
                    "width_mm": round(span, 1),
                    "width_ft": round(span / MM_PER_FT, 2),
                    "bearing_deg": round(bearing, 1),
                    "range_mm": round(rng, 1),
                    "range_ft": round(rng / MM_PER_FT, 2),
                    "source": "fitted-surfaces",
                }
            )

    out.sort(key=lambda g: g["range_mm"])
    return out


def classify_place(snapshot: Dict[str, Any]) -> Dict[str, Any]:
    """What sort of space this is: corridor, room, open, or unknown."""
    if not snapshot or not snapshot.get("available"):
        return {"place": "unknown", "confidence": 0.0, "reason": "no BEV grid yet"}
    if not snapshot.get("registered", False):
        return {
            "place": "unknown",
            "confidence": 0.0,
            "reason": f"grid not registered ({snapshot.get('reset_reason') or 'unknown'})",
        }

    objects = snapshot.get("objects") or []
    walls = _structural_walls(snapshot.get("surfaces") or [])

    if len(walls) < 2:
        if len(objects) < _ENCLOSED_MIN_WALLS:
            return {
                "place": "open",
                "confidence": 0.5,
                "reason": f"only {len(objects)} things in range, none of them long walls",
            }
        return {
            "place": "unknown",
            "confidence": 0.2,
            "reason": f"{len(objects)} objects but fewer than 2 structural walls",
        }

    best_pair = None
    for i in range(len(walls)):
        for j in range(i + 1, len(walls)):
            a, b = walls[i], walls[j]
            if not _parallel(a, b):
                continue
            off_a, off_b = _offsets_about(a, b)
            if off_a * off_b >= 0:
                continue  # same side: one wall, not two
            width = abs(off_a) + abs(off_b)
            if best_pair is None or width < best_pair[0]:
                best_pair = (width, a, b)

    if best_pair and best_pair[0] <= _CORRIDOR_MAX_WIDTH_MM:
        width, a, b = best_pair
        dx, dy = a.get("direction") or (1.0, 0.0)
        axis = _norm180(math.degrees(math.atan2(float(dy), float(dx))))

        blocked = 0
        for end in (axis, _norm180(axis + 180.0)):
            for w in walls:
                if w is a or w is b:
                    continue
                brg = float(w.get("centre_bearing_deg") or 0.0)
                if _angular_gap(brg, end) > _FACING_TOLERANCE_DEG:
                    continue
                if float(w.get("nearest_mm") or 0.0) <= _CORRIDOR_MAX_WIDTH_MM:
                    blocked += 1
                    break
        if blocked >= 2:
            return {
                "place": "room",
                "confidence": 0.75,
                "reason": (
                    f"walls {width / MM_PER_FT:.1f} ft apart but closed at "
                    f"both ends — a small room, not a corridor"
                ),
            }

        return {
            "place": "corridor",
            "confidence": 0.8,
            "width_ft": round(width / MM_PER_FT, 1),
            "axis_deg": round(axis, 1),
            "reason": (
                f"two parallel walls {width / MM_PER_FT:.1f} ft apart with you "
                f"between them ({a.get('span_mm', 0) / 1000:.1f} m and "
                f"{b.get('span_mm', 0) / 1000:.1f} m of wall)"
            ),
        }

    if len(walls) >= _ENCLOSED_MIN_WALLS:
        return {
            "place": "room",
            "confidence": 0.7,
            "reason": f"{len(walls)} structural walls around, none of them a facing pair",
        }

    return {
        "place": "room",
        "confidence": 0.4,
        "reason": f"{len(walls)} structural walls, too few to be sure",
    }


def candidate_paths(
    snapshot: Dict[str, Any], clearances_ft: Optional[Dict[str, Any]] = None
) -> List[Dict[str, Any]]:
    """The ways out of here, best first."""
    paths: List[Dict[str, Any]] = []

    snapshot = snapshot or {}
    found = snapshot.get("gaps") or doorways(snapshot.get("surfaces") or [])
    for gap in found:
        width_mm = float(gap.get("width_mm") or 0.0)
        paths.append(
            {
                "kind": "doorway",
                "bearing_deg": gap.get("bearing_deg"),
                "clock": gap.get("clock"),
                "width_ft": gap.get("width_ft"),
                "source": gap.get("source", "bev-grid"),
                "range_ft": gap.get("range_ft"),
                "score": round(width_mm / 1000.0 + 2.0, 2),
                "why": f"doorway {gap.get('width_ft')} ft wide",
            }
        )

    for name, ft in (clearances_ft or {}).items():
        if not isinstance(ft, (int, float)):
            continue
        if ft * MM_PER_FT < _MIN_USEFUL_CLEARANCE_MM:
            continue
        paths.append(
            {
                "kind": "clearance",
                "direction": name,
                "bearing_deg": {
                    "front": 0.0,
                    "right": 90.0,
                    "back": 180.0,
                    "left": 270.0,
                }.get(name),
                "range_ft": round(float(ft), 1),
                "score": round(min(float(ft), 12.0) / 12.0, 2),
                "why": f"{ft:.1f} ft of room",
            }
        )

    paths.sort(key=lambda p: -p["score"])
    return paths


def preferred_side(snapshot: Dict[str, Any]) -> Optional[str]:
    """ "left" or "right" when the spatial picture has a genuine opinion, else None."""
    if not snapshot or not snapshot.get("available") or not snapshot.get("registered"):
        return None

    found = snapshot.get("gaps") or doorways(snapshot.get("surfaces") or [])

    left, right = [], []
    for gap in found:
        bearing = _norm180(float(gap.get("bearing_deg") or 0.0))
        if abs(bearing) < 20.0 or abs(bearing) > 150.0:
            continue  # effectively ahead or behind: not a side
        (right if bearing > 0 else left).append(gap)

    if bool(left) == bool(right):
        return None  # none either side, or one of each — no opinion

    chosen = left or right
    return "left" if chosen is left else "right"


def live_snapshot(lidar_state: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """The BEV grid's snapshot, with the scan's fitted walls as `surfaces`."""
    from utils.bev_grid import get_grid

    snapshot = dict(get_grid().snapshot() or {})
    try:
        from utils.drift_guidance import fit_surfaces, _side_and_forward_points

        if lidar_state is None:
            from utils.shared_state import get_lidar_state

            lidar_state = get_lidar_state()
        snapshot["surfaces"] = fit_surfaces(_side_and_forward_points(lidar_state))
    except Exception:
        snapshot["surfaces"] = []
    return snapshot


def scan(clearances_ft: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """The whole picture, for the API and the dashboard."""
    try:
        snapshot = live_snapshot()
    except Exception as exc:
        return {
            "available": False,
            "error": f"{type(exc).__name__}: {exc}",
            "place": {
                "place": "unknown",
                "confidence": 0.0,
                "reason": "BEV grid unavailable",
            },
            "paths": [],
            "preferred_side": None,
        }

    try:
        return {
            "available": bool(snapshot.get("available")),
            "error": snapshot.get("error"),
            "place": classify_place(snapshot),
            "paths": candidate_paths(snapshot, clearances_ft),
            "preferred_side": preferred_side(snapshot),
            "objects_seen": len(snapshot.get("objects") or []),
            "gaps_seen": len(snapshot.get("gaps") or []),
            "doorways_fitted": len(doorways(snapshot.get("surfaces") or [])),
            "surfaces_seen": len(snapshot.get("surfaces") or []),
            "surfaces": [
                {
                    "kind": sf.get("kind"),
                    "span_mm": round(float(sf.get("span_mm") or 0.0)),
                    "structural": (
                        sf.get("kind") == "wall"
                        and float(sf.get("span_mm") or 0.0) >= _STRUCTURAL_WALL_MM
                    ),
                    "bearing_deg": sf.get("centre_bearing_deg"),
                    "nearest_mm": round(float(sf.get("nearest_mm") or 0.0)),
                    "rms_mm": sf.get("rms_mm"),
                }
                for sf in sorted(
                    snapshot.get("surfaces") or [],
                    key=lambda x: -(float(x.get("span_mm") or 0.0)),
                )[:6]
            ],
            "structural_wall_min_mm": _STRUCTURAL_WALL_MM,
            "unknown_wedges": len(snapshot.get("unknown_wedges") or []),
            "registered": bool(snapshot.get("registered")),
            "frames": snapshot.get("frames"),
        }
    except Exception as exc:
        return {
            "available": False,
            "error": f"{type(exc).__name__}: {exc}",
            "place": {"place": "unknown", "confidence": 0.0, "reason": "scan failed"},
            "paths": [],
            "preferred_side": None,
        }
