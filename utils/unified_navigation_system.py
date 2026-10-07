"""Unified Navigation System - WITH ROAD CROSSING INTEGRATION Combines obstacle avoidance with traffic light awareness."""

import logging
import math
from typing import List, Dict, Any, Optional

from config import GUIDANCE_DANGER_MM, GUIDANCE_WARNING_MM, GUIDANCE_SAFE_MM
DANGER_MM = GUIDANCE_DANGER_MM     # ~2.6 ft
WARNING_MM = GUIDANCE_WARNING_MM   # ~5 ft
SAFE_MM = GUIDANCE_SAFE_MM         # ~6.5 ft

from config import (
    LIDAR_SECTOR_ORDER, GUIDANCE_SECTORS, CAMERA_SECTORS, LIDAR_MAX_DISTANCE,
)

# THREAT detection spans all 8 x 45° LiDAR sectors (finer hazard direction).
LIDAR_SECTORS_8 = list(LIDAR_SECTOR_ORDER)
# Cameras remain the 4 mosaic quadrants.
SECTORS = list(CAMERA_SECTORS)

# Single sector definition shared with the LiDAR scanner (was a duplicate copy).
from utils.lidar_utils import get_sector

TRAFFIC_LIGHT_KEYWORDS = [
    "traffic", "light", "signal", "red", "green", "yellow",
    "walk", "pedestrian", "crossing"
]


def mm_to_ft(mm) -> Optional[float]:
    """Convert mm to feet"""
    if mm is None or mm <= 0:
        return None
    try:
        f = float(mm)
        if math.isinf(f) or math.isnan(f):
            return None
        return round(f / 304.8, 1)
    except:
        return None


def safe_json(obj):
    """Make object JSON serializable"""
    if obj is None:
        return None
    if isinstance(obj, bool):
        return obj
    if isinstance(obj, int):
        return obj
    if isinstance(obj, float):
        if math.isinf(obj) or math.isnan(obj):
            return None
        return obj
    if isinstance(obj, dict):
        return {k: safe_json(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [safe_json(x) for x in obj]
    try:
        return safe_json(float(obj))
    except:
        return str(obj)


def detect_traffic_lights_from_detections(camera_data: Dict) -> Dict[str, Any]:
    """Extract traffic light information from camera detections"""
    traffic_lights = {
        "detected": False,
        "at_intersection": False,
        "directions": {}
    }
    
    for direction in SECTORS:
        detections = camera_data.get(direction, [])
        
        # Handle both formats: list or dict with detections key
        if isinstance(detections, dict):
            detections = detections.get("detections", [])
        if not isinstance(detections, list):
            detections = []
        
        traffic_lights["directions"][direction] = {
            "has_light": False,
            "state": "unknown",
            "pedestrian_signal": "unknown",
            "confidence": 0
        }
        
        for det in detections:
            if not isinstance(det, dict):
                continue
                
            class_name = (det.get("class_name") or "").lower().strip()
            confidence = det.get("confidence", 0)

            is_traffic_light = class_name == "traffic light"

            if is_traffic_light and confidence >= 0.4:  # Lower threshold for traffic lights
                logging.getLogger(__name__).debug(
                    "Traffic light detected: '%s' (%.0f%%) from %s",
                    class_name, confidence * 100, direction
                )
                traffic_lights["detected"] = True
                traffic_lights["at_intersection"] = True
                traffic_lights["directions"][direction]["has_light"] = True
                traffic_lights["directions"][direction]["confidence"] = confidence
    
    return traffic_lights


def get_crossing_guidance(traffic_lights: Dict) -> Dict[str, Any]:
    """Road crossing guidance: never "cross" — no light's colour can be read, so a light only means wait."""
    guidance = {
        "is_crossing_scenario": traffic_lights["detected"],
        "safe_to_cross": False,
        "action": "wait",
        "message": "",
        "urgency": "normal",
        "detected_direction": None
    }

    if not traffic_lights["detected"]:
        guidance["message"] = "No traffic lights detected"
        return guidance

    guidance["message"] = "Traffic light detected but state unclear"
    return guidance


def determine_intersection_type(traffic_lights: Dict) -> str:
    """Determine intersection type from detected lights"""
    directions_with_lights = [
        d for d in SECTORS 
        if traffic_lights["directions"][d]["has_light"]
    ]
    
    count = len(directions_with_lights)
    
    if count >= 3:
        return "4-way"
    elif count == 2:
        if set(directions_with_lights) in [{"front", "back"}, {"left", "right"}]:
            return "2-way"
        return "3-way"
    elif count == 1:
        return "2-way"
    return "unknown"


def decisionMaker(sample: Dict[str, Any]) -> Dict[str, Any]:
    """COMPREHENSIVE navigation decision maker Integrates obstacle avoidance with traffic light awareness"""
    scan_data = sample.get("scan_data") or []
    precomputed = sample.get("precomputed_sectors")

    if precomputed:
        sectors = {
            sec: {"min": precomputed.get(sec, {}).get("min_dist"),
                  "count": precomputed.get(sec, {}).get("count", 0)}
            for sec in LIDAR_SECTORS_8
        }
        lidar_point_count = sum(s["count"] for s in sectors.values())
    else:
        sectors = {sec: {"min": None, "count": 0} for sec in LIDAR_SECTORS_8}

        for pt in scan_data:
            try:
                if isinstance(pt, dict):
                    angle = float(pt.get("angle_deg", 0))
                    dist = float(pt.get("distance_mm", 0))
                elif isinstance(pt, (list, tuple)) and len(pt) >= 2:
                    angle, dist = float(pt[0]), float(pt[1])
                else:
                    continue

                if dist <= 0 or dist > 50000:
                    continue

                sec = get_sector(angle)
                sectors[sec]["count"] += 1

                if sectors[sec]["min"] is None or dist < sectors[sec]["min"]:
                    sectors[sec]["min"] = dist

            except:
                continue
        lidar_point_count = len(scan_data)
    
    # Build camera data dict - handle both list and dict formats
    camera_data = {}
    for sec in SECTORS:
        sec_data = sample.get(sec, [])
        if isinstance(sec_data, dict):
            camera_data[sec] = sec_data.get("detections", [])
        else:
            camera_data[sec] = sec_data if isinstance(sec_data, list) else []
    
    traffic_lights = detect_traffic_lights_from_detections(camera_data)

    crossing_guidance = None
    intersection_type = None
    if traffic_lights["detected"]:
        crossing_guidance = get_crossing_guidance(traffic_lights)
        intersection_type = determine_intersection_type(traffic_lights)
    
    dangerous = []
    warning = []
    safe = []
    
    # THREAT: classify all 8 LiDAR sectors (finer awareness of where danger is).
    for sec in LIDAR_SECTORS_8:
        min_d = sectors[sec]["min"]
        if min_d is not None:
            if min_d <= DANGER_MM:
                dangerous.append(sec)
            elif min_d <= WARNING_MM:
                warning.append(sec)
            else:
                safe.append(sec)
    
    cones = sample.get("precomputed_cones") or {}

    def _cone_min(name):
        c = cones.get(name) or {}
        d = c.get("min_dist")
        if d is None and not cones:
            return sectors.get(name, {}).get("min")
        return d

    front_min = _cone_min("front")
    front_blocked = front_min is not None and front_min <= DANGER_MM
    front_warn = (not front_blocked) and front_min is not None and front_min <= WARNING_MM
    
    # Check for critical objects (people, vehicles)
    critical = []
    for cam in SECTORS:
        dets = sample.get(cam, [])
        if isinstance(dets, dict):
            dets = dets.get("detections", [])
        if isinstance(dets, list):
            for d in dets:
                if isinstance(d, dict):
                    name = (d.get("class_name") or "").lower()
                    # Skip traffic lights from critical objects
                    if any(kw in name for kw in TRAFFIC_LIGHT_KEYWORDS):
                        continue
                    if any(x in name for x in ["person", "car", "truck", "bus", "bicycle", "motorcycle", "scooter"]):
                        critical.append({"name": name, "direction": cam})
    
    action = "go_forward"
    message = "Path clear"
    direction = "front"
    crossing_mode = False

    if front_blocked:
        front_ft = mm_to_ft(front_min)

        critical_front = [c for c in critical if c["direction"] == "front"]
        obj_detail = f" ({critical_front[0]['name']})" if critical_front else ""

        def _clearance(alt):
            """Usable clearance for a turn candidate, or None if unusable."""
            d = _cone_min(alt)
            if d is not None and d <= DANGER_MM:
                return None               # something right there
            return d if d else LIDAR_MAX_DISTANCE

        sides = [(a, c) for a in ("right", "left")
                 if (c := _clearance(a)) is not None]
        sides.sort(key=lambda t: -t[1])
        back = [(a, c) for a in ("back",) if (c := _clearance(a)) is not None]

        best_sec, best_dist = None, 0
        for pool in (sides, back):                      # sides first
            good = [t for t in pool if t[1] > WARNING_MM]
            if good:
                best_sec, best_dist = good[0]
                break
        if best_sec is None:                            # nothing roomy — take any
            fallback = sorted(sides + back, key=lambda t: -t[1])
            if fallback and fallback[0][1] > DANGER_MM:
                best_sec, best_dist = fallback[0]

        if best_sec and best_dist > DANGER_MM:
            action = f"turn_{best_sec}"
            message = f"STOP! Obstacle {front_ft}ft ahead{obj_detail}. Turn {best_sec.upper()} - {mm_to_ft(best_dist)}ft clear"
            direction = best_sec
        else:
            action = "stop"
            message = f"STOP! Obstacle {front_ft}ft ahead{obj_detail}. All directions blocked."
            direction = "stop"

    # Priority 2: LIDAR WARNING DISTANCE - proceed with extreme caution
    elif front_warn:
        front_ft = mm_to_ft(front_min)

        critical_front = [c for c in critical if c["direction"] == "front"]
        if critical_front:
            obj_name = critical_front[0]['name']
            # Vehicles require STOP, not caution
            if any(v in obj_name for v in ["car", "truck", "bus", "motorcycle"]):
                action = "stop"
                message = f"STOP! {obj_name.title()} detected {front_ft}ft ahead - too close!"
                direction = "stop"
            else:
                action = "proceed_cautiously"
                message = f"CAUTION: {obj_name.title()} {front_ft}ft ahead. Move slowly."
                direction = "front"
        else:
            action = "proceed_cautiously"
            message = f"CAUTION: Obstacle {front_ft}ft ahead. Move slowly."
            direction = "front"

    # Priority 3: Traffic light / road crossing scenario (only if path is physically clear)
    elif traffic_lights["detected"]:
        crossing_mode = True
        action = "wait_for_signal"          # the colour cannot be read: never "cross"
        message = crossing_guidance["message"]
        direction = "stop"

    # Priority 4: Camera-detected critical objects (when LiDAR shows safe distance)
    elif critical:
        vehicles = [c for c in critical if any(v in c["name"] for v in ["car", "truck", "bus", "motorcycle"])]
        people = [c for c in critical if "person" in c["name"]]

        critical_in_danger = False
        for obj in critical:
            obj_sector = obj["direction"]
            if obj_sector in dangerous:
                critical_in_danger = True
                break

        if vehicles:
            if critical_in_danger or vehicles[0]["direction"] == "front":
                action = "stop"
                message = f"STOP! Vehicle ({vehicles[0]['name']}) detected on {vehicles[0]['direction']} with obstacles nearby."
                direction = "stop"
            else:
                action = "caution_vehicle"
                message = f"Vehicle detected: {vehicles[0]['name']} on {vehicles[0]['direction']}. Proceed with caution."
                direction = "front"
        elif people:
            person_dir = people[0]["direction"]
            person_sector_dist = sectors[person_dir]["min"]

            # Only STOP if person is in a dangerous sector (confirmed by LiDAR)
            if critical_in_danger:
                closest_obstacle = min([sectors[s]['min'] for s in dangerous if sectors[s]['min'] is not None]) if dangerous else None
                obstacle_dist = mm_to_ft(closest_obstacle) if closest_obstacle else "unknown"
                action = "stop"
                message = f"STOP! Person detected on {person_dir} - obstacles at {obstacle_dist}ft."
                direction = "stop"
            elif person_dir == "front" and person_sector_dist and person_sector_dist > WARNING_MM:
                action = "caution_person"
                message = f"Person detected ahead at {mm_to_ft(person_sector_dist)}ft. Proceed carefully."
                direction = "front"
            elif person_dir == "front" and person_sector_dist and person_sector_dist > DANGER_MM:
                action = "proceed_cautiously"
                message = f"Person detected {mm_to_ft(person_sector_dist)}ft ahead. Move slowly."
                direction = "front"
            else:
                action = "caution_person"
                message = f"Person detected on {person_dir}. Proceed carefully."
                direction = "front"

    elif any((_cone_min(s) or 1e9) <= DANGER_MM for s in ("left", "right")):
        side = [(s, _cone_min(s)) for s in ("left", "right")
                if (_cone_min(s) or 1e9) <= DANGER_MM]
        closest_sector, closest_danger = min(side, key=lambda t: t[1])
        action = "caution_surroundings"
        message = (f"Forward is clear. CAUTION: {closest_sector.upper()} close at "
                   f"{mm_to_ft(closest_danger)}ft. Move slowly forward.")
        direction = "front"

    # Priority 6: Path clear - proceed normally
    else:
        if front_min:
            message = f"Path clear. Continue forward ({mm_to_ft(front_min)}ft clear)"
        else:
            message = (f"Path clear. Continue forward — nothing detected within "
                       f"{mm_to_ft(LIDAR_MAX_DISTANCE)}ft.")
    
    result = {
        "action": action,
        "urgent_message": message,
        "primary_direction": direction,
        "speed": {
            "speed": "stop" if "stop" in action or "wait" in action else 
                    ("slow" if "cautiously" in action or "caution" in action else "normal"),
            "multiplier": 0.0 if "stop" in action or "wait" in action else 
                         (0.5 if "cautiously" in action or "caution" in action else 1.0)
        },
        "dangerous_sectors": dangerous,
        "warning_sectors": warning,
        "safe_sectors": safe,
        "sector_distances": {
            sec: {
                "distance_mm": sectors[sec]["min"],
                "distance_ft": mm_to_ft(sectors[sec]["min"]),
                "count": sectors[sec]["count"],
                "is_safe": sec not in dangerous and sec not in warning
            }
            for sec in LIDAR_SECTORS_8
        },
        # Narrow steering cones (what the guidance actually decides on).
        "cone_distances": {
            name: {
                "distance_mm": _cone_min(name),
                "distance_ft": mm_to_ft(_cone_min(name)),
            }
            for name in GUIDANCE_SECTORS
        },
        "wall_detection": {
            sec: {
                "wall_detected": sectors[sec]["count"] >= 10 and sectors[sec]["min"] and sectors[sec]["min"] < 3000,
                "distance": sectors[sec]["min"] if sectors[sec]["count"] >= 10 else None,
                "warning": sectors[sec]["min"] and sectors[sec]["min"] < 2500 if sectors[sec]["count"] >= 10 else False
            }
            for sec in LIDAR_SECTORS_8
        },
        
        "crossing_mode": crossing_mode,
        "traffic_lights": {
            "detected": traffic_lights["detected"],
            "intersection_type": intersection_type,
            "front": traffic_lights["directions"]["front"],
            "left": traffic_lights["directions"]["left"],
            "right": traffic_lights["directions"]["right"],
            "back": traffic_lights["directions"]["back"],
            "guidance": crossing_guidance
        },
        
        "path": {
            "direction": direction,
            "clear_distance_ft": mm_to_ft(sectors[direction]["min"]) if direction in sectors else 10,
            "recommended": message
        },
        "cues": {
            "primary": _cue_for_action(action),
            "detail": message,
            "warning": message if dangerous or "stop" in action else None
        },
        "landmarks": [],
        "critical_objects": critical,
        "sensor_health": {
            "lidar_active": lidar_point_count > 0,
            "lidar_points": lidar_point_count,
            "cameras_active": sum(1 for s in SECTORS if sample.get(s))
        }
    }
    
    return safe_json(result)


def _cue_for_action(action: str) -> str:
    """Short instruction phrase for an action."""
    phrases = {
        "wait_for_signal": "Wait for signal",
        "caution_person": "Person nearby",
        "caution_vehicle": "Vehicle nearby",
        "caution_surroundings": "Watch surroundings",
        "stop": "Stop",
        "go_forward": "Go forward",
        "proceed_cautiously": "Proceed carefully",
        "turn_left": "Turn left",
        "turn_right": "Turn right",
        "turn_back": "Turn around"
    }
    return phrases.get(action, action.replace("_", " ").title())


def detect_walls_by_sector(scan_data: List) -> Dict:
    """Simple wall detection for API compatibility"""
    sample = {"scan_data": scan_data}
    result = decisionMaker(sample)
    return result.get("wall_detection", {})