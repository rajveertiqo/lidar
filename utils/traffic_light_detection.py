"""Traffic Light Detection and Road Crossing Assistance Module For blind assistance system - helps users safely cross roads"""

import time
import logging
from typing import Dict, Any, Optional, Tuple
from enum import Enum
from collections import deque
from dataclasses import dataclass, field

from config import SIMPLE_VEHICLE_CLASSES, TRAFFIC_SIGN_STALE_S


# ============== CONFIGURATION ==============
class TrafficConfig:
    # Detection confidence threshold
    MIN_CONFIDENCE = 0.5

    # Temporal filtering - how many frames to consider
    HISTORY_SIZE = 10
    MIN_CONSISTENT_FRAMES = 3

    # LiDAR vehicle detection
    VEHICLE_APPROACH_THRESHOLD = 5000  # mm - warn if vehicle within 5m


class TrafficLightState(Enum):
    """Traffic light states: no lamp's colour can be read, so UNKNOWN is the only one."""

    UNKNOWN = "unknown"


class PedestrianSignalState(Enum):
    """Pedestrian signal states"""

    UNKNOWN = "unknown"
    WALK = "walk"
    DONT_WALK = "dont_walk"
    FLASHING = "flashing"


class CrossingState(Enum):
    """Road crossing state: no light's colour can be read, so it never moves towards crossing."""

    IDLE = "idle"  # Not at crossing
    WAITING = "waiting"  # After an abort, until a reset


@dataclass
class DirectionStatus:
    """Traffic status for one direction"""

    traffic_light: TrafficLightState = TrafficLightState.UNKNOWN
    pedestrian_signal: PedestrianSignalState = PedestrianSignalState.UNKNOWN
    confidence: float = 0.0
    last_update: float = 0.0
    vehicle_detected: bool = False
    vehicle_distance: Optional[float] = None
    vehicle_seen: bool = False  # the camera's last frame this way showed a vehicle
    history: deque = field(
        default_factory=lambda: deque(maxlen=TrafficConfig.HISTORY_SIZE)
    )
    # The camera frame last counted, so a re-poll of the same frame is not.
    frame_time: float = 0.0


class TrafficLightDetector:
    """Processes camera detections to identify traffic light states and determine safe crossing conditions"""

    def __init__(self):
        self.directions: Dict[str, DirectionStatus] = {
            "front": DirectionStatus(),
            "right": DirectionStatus(),
            "left": DirectionStatus(),
            "back": DirectionStatus(),
        }

        self.crossing_state = CrossingState.IDLE
        self.intersection_type: Optional[str] = None

    def parse_traffic_light_class(
        self, class_name: str
    ) -> Tuple[TrafficLightState, PedestrianSignalState]:
        """Parse detection class name to determine light state"""
        name = class_name.lower().strip()

        traffic_state = TrafficLightState.UNKNOWN
        ped_state = PedestrianSignalState.UNKNOWN

        if "walk" in name:
            if "dont" in name or "don't" in name or "stop" in name or "no" in name:
                ped_state = PedestrianSignalState.DONT_WALK
            elif "flash" in name:
                ped_state = PedestrianSignalState.FLASHING
            else:
                ped_state = PedestrianSignalState.WALK

        return traffic_state, ped_state

    def process_camera_detections(
        self, camera_data: Dict[str, Any]
    ) -> Dict[str, DirectionStatus]:
        """Process detections from all cameras to update traffic light states"""
        current_time = time.time()

        for direction in ["front", "right", "left", "back"]:
            detections = camera_data.get(direction, [])

            frame_time = (
                detections.get("frame_time") if isinstance(detections, dict) else None
            )
            if frame_time:
                if frame_time == self.directions[direction].frame_time:
                    continue
                self.directions[direction].frame_time = frame_time
            seen_at = frame_time or current_time

            # Handle both formats: list of detections OR dict with detections key
            if isinstance(detections, dict):
                detections = detections.get("detections", [])

            if not isinstance(detections, list):
                detections = []

            traffic_detections = []
            vehicle_seen = False
            for det in detections:
                if not isinstance(det, dict):
                    continue

                class_name = det.get("class_name", "").lower().strip()
                confidence = det.get("confidence", 0)

                is_traffic = False
                is_pedestrian = False
                if (
                    class_name in SIMPLE_VEHICLE_CLASSES
                    and confidence >= TrafficConfig.MIN_CONFIDENCE
                ):
                    vehicle_seen = True

                if class_name.replace("_", " ") == "traffic light":  # either model's spelling
                    is_traffic = True
                    logging.getLogger(__name__).debug(
                        "[TRAFFIC] Found '%s' (%.0f%%) in %s camera",
                        class_name,
                        confidence * 100,
                        direction,
                    )

                if "walk" in class_name or "pedestrian" in class_name:
                    is_pedestrian = True

                if (
                    is_traffic or is_pedestrian
                ) and confidence >= TrafficConfig.MIN_CONFIDENCE:
                    traffic_detections.append(
                        {
                            "class_name": class_name,
                            "confidence": confidence,
                            "is_traffic": is_traffic,
                            "is_pedestrian": is_pedestrian,
                        }
                    )

            self.directions[direction].vehicle_seen = vehicle_seen
            if traffic_detections:
                best_det = max(traffic_detections, key=lambda x: x["confidence"])
                traffic_state, ped_state = self.parse_traffic_light_class(
                    best_det["class_name"]
                )

                self.directions[direction].history.append(
                    {"traffic": traffic_state, "pedestrian": ped_state, "time": seen_at}
                )

                filtered_traffic = self._filter_state_history(direction, "traffic")
                filtered_ped = self._filter_state_history(direction, "pedestrian")

                self.directions[direction].traffic_light = filtered_traffic
                self.directions[direction].pedestrian_signal = filtered_ped
                self.directions[direction].confidence = best_det["confidence"]
                self.directions[direction].last_update = seen_at

        return self.directions

    def _filter_state_history(self, direction: str, state_type: str) -> Any:
        """Apply temporal filtering to reduce noise"""
        history = list(self.directions[direction].history)
        if not history:
            if state_type == "traffic":
                return TrafficLightState.UNKNOWN
            return PedestrianSignalState.UNKNOWN

        state_counts = {}
        for entry in history[-TrafficConfig.HISTORY_SIZE :]:
            state = entry.get(state_type)
            if state:
                state_counts[state] = state_counts.get(state, 0) + 1

        if not state_counts:
            if state_type == "traffic":
                return TrafficLightState.UNKNOWN
            return PedestrianSignalState.UNKNOWN

        most_common = max(state_counts.items(), key=lambda x: x[1])

        if most_common[1] >= TrafficConfig.MIN_CONSISTENT_FRAMES:
            return most_common[0]

        if state_type == "traffic":
            return TrafficLightState.UNKNOWN
        return PedestrianSignalState.UNKNOWN

    def process_lidar_for_vehicles(self, lidar_sectors: Dict[str, Any]):
        """How far a vehicle the camera sees is (process_camera_detections first): the LiDAR
        ranges anything, so a wall within 5 m alone was a "vehicle" (scan 2026-10-23)."""
        for direction in ["front", "right", "left", "back"]:
            sector_data = lidar_sectors.get(direction, {})
            min_dist = sector_data.get("min_dist") or sector_data.get("distance_mm")

            if (
                self.directions[direction].vehicle_seen
                and isinstance(min_dist, (int, float))
                and min_dist > 0
            ):
                self.directions[direction].vehicle_distance = min_dist
                self.directions[direction].vehicle_detected = (
                    min_dist < TrafficConfig.VEHICLE_APPROACH_THRESHOLD
                )
            else:
                self.directions[direction].vehicle_distance = None
                self.directions[direction].vehicle_detected = False

    def detect_intersection_type(self) -> str:
        """Intersection type from the lights whose colour is known; none can be read, so "unknown"."""
        self.intersection_type = "unknown"
        return self.intersection_type

    def get_crossing_safety(self) -> Dict[str, Any]:
        """Is it safe to cross? Never "yes": no light's colour can be read, so this only says whether a light was seen."""
        now = time.time()
        seen = [
            d
            for d, st in self.directions.items()
            if st.confidence > 0 and (now - st.last_update) < TRAFFIC_SIGN_STALE_S
        ]
        if seen:
            message = (
                "Traffic light ahead, but I cannot tell which lamp "
                "is lit. Listen for traffic before you cross."
            )
        else:
            message = "No traffic lights detected. Look and listen carefully."

        return {
            "is_safe": False,
            "safety_level": "unknown",
            "message": message,
            "action": "wait",
            "intersection_type": self.intersection_type,
            "front_light": "unknown",
            "front_pedestrian": "unknown",
            "left_light": self.directions["left"].traffic_light.value,
            "right_light": self.directions["right"].traffic_light.value,
            "confidence": 0,
            "detected_direction": None,
        }

    def update_crossing_state(self, user_action: Optional[str] = None) -> CrossingState:
        """An abort parks the crossing state in WAITING until a reset; no light's colour can be read, so nothing moves it towards crossing."""
        if user_action == "abort":
            self.crossing_state = CrossingState.WAITING
        return self.crossing_state

    def crossing_message(self) -> Dict[str, str]:
        """Primary + detail wording for the current crossing state."""
        safety = self.get_crossing_safety()

        guidance = {"primary": "", "detail": "", "urgency": "normal"}

        state_messages = {
            CrossingState.IDLE: (
                "Approaching intersection",
                "Scanning for traffic signals",
            ),
            CrossingState.WAITING: ("Wait", safety["message"]),
        }

        primary, detail = state_messages.get(self.crossing_state, ("", ""))
        guidance["primary"] = primary
        guidance["detail"] = detail

        return guidance

    def get_full_status(self) -> Dict[str, Any]:
        """Get complete traffic light and crossing status"""
        safety = self.get_crossing_safety()

        return {
            "crossing_state": self.crossing_state.value,
            "intersection_type": self.detect_intersection_type(),
            "safety": safety,
            "directions": {
                direction: {
                    "traffic_light": status.traffic_light.value,
                    "pedestrian_signal": status.pedestrian_signal.value,
                    "confidence": status.confidence,
                    "vehicle_detected": status.vehicle_detected,
                    "vehicle_distance_ft": round(status.vehicle_distance / 304.8, 1)
                    if status.vehicle_distance
                    else None,
                    "last_update": status.last_update,
                }
                for direction, status in self.directions.items()
            },
            "guidance": self.crossing_message(),
            "crossing_time": None,
        }

    def reset(self):
        """Reset detector state"""
        for direction in self.directions:
            self.directions[direction] = DirectionStatus()
        self.crossing_state = CrossingState.IDLE
        self.intersection_type = None


traffic_detector = TrafficLightDetector()


def get_traffic_detector() -> TrafficLightDetector:
    """Get global traffic detector instance"""
    return traffic_detector
