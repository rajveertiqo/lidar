"""Environment Detection Module Detects weather conditions like rain, fog, low light from camera and LiDAR data"""

import threading
import time

import numpy as np
from typing import Dict, Any, List, Tuple, Optional
from collections import deque

try:
    import cv2

    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False
    print("[WARN] OpenCV not installed, using basic environment detection")


# ============== CONFIGURATION ==============
class EnvironmentConfig:
    # Fog detection thresholds
    FOG_CONTRAST_THRESHOLD = 0.35  # Low contrast indicates fog

    # Rain detection thresholds
    RAIN_STREAK_THRESHOLD = 0.15  # Vertical edge density for rain

    # Low light detection
    LOW_LIGHT_THRESHOLD = 50  # Average brightness threshold

    # History for temporal smoothing
    HISTORY_SIZE = 3  # short window = fast response to change


# ============== STATE ==============
environment_state = {
    "condition": "clear",  # clear, fog, rain, low_light, storm
    "visibility": 1.0,  # 0.0 (poor) to 1.0 (excellent)
    "confidence": 0.0,
    "details": {
        "fog_level": 0.0,
        "rain_intensity": 0.0,
        "brightness": 1.0,
        "contrast": 1.0,
        "lidar_quality": 1.0,
    },
    "warnings": [],
    "last_update": 0,
}

_history_lock = threading.Lock()
detection_history = {
    "fog": deque(maxlen=EnvironmentConfig.HISTORY_SIZE),
    "rain": deque(maxlen=EnvironmentConfig.HISTORY_SIZE),
    "brightness": deque(maxlen=EnvironmentConfig.HISTORY_SIZE),
    "lidar_quality": deque(maxlen=EnvironmentConfig.HISTORY_SIZE),
}


# ============== CAMERA-BASED DETECTION ==============


def analyze_image_for_fog(image: np.ndarray) -> Dict[str, float]:
    """Detect fog conditions from camera image."""
    if image is None or image.size == 0:
        return {"fog_detected": False, "fog_level": 0.0, "confidence": 0.0}

    try:
        if len(image.shape) == 2:
            gray = image
            hsv = None
        else:
            if HAS_OPENCV:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            else:
                gray = np.mean(image, axis=2).astype(np.uint8)
                hsv = None

        # 1. Contrast Analysis (fog reduces contrast)
        contrast = gray.std() / 128.0  # Normalize to 0-1 range
        contrast_score = 1.0 - min(1.0, contrast / 0.5)  # Lower contrast = higher fog

        dark_channel_score = None
        if HAS_OPENCV and len(image.shape) == 3:
            dark_channel = np.min(image, axis=2)
            # In fog, dark channel values are high (no true dark pixels)
            dark_mean = dark_channel.mean() / 255.0
            dark_channel_score = dark_mean  # Higher = more fog

        # 3. Saturation Analysis (fog desaturates colors)
        saturation_score = None
        if hsv is not None:
            saturation = hsv[:, :, 1].mean() / 255.0
            saturation_score = 1.0 - saturation  # Lower saturation = higher fog

        # 4. Brightness uniformity (fog creates uniform brightness)
        brightness_std = gray.std() / 255.0
        uniformity_score = 1.0 - min(1.0, brightness_std * 4)  # More uniform = more fog

        # 5. Edge density (fog reduces visible edges)
        edge_score = None
        if HAS_OPENCV:
            edges = cv2.Canny(gray, 50, 150)
            edge_density = edges.mean() / 255.0
            edge_score = 1.0 - min(1.0, edge_density * 10)  # Fewer edges = more fog

        fog_indicators = [
            contrast_score,
            dark_channel_score,
            saturation_score,
            uniformity_score,
            edge_score,
        ]
        fog_indicators = [s for s in fog_indicators if s is not None]

        if fog_indicators:
            fog_level = np.mean(fog_indicators)
            fog_detected = fog_level > EnvironmentConfig.FOG_CONTRAST_THRESHOLD
            confidence = min(1.0, len(fog_indicators) / 5.0 * fog_level)
        else:
            fog_level = 0.0
            fog_detected = False
            confidence = 0.0

        return {
            "fog_detected": fog_detected,
            "fog_level": float(fog_level),
            "confidence": float(confidence),
            "contrast": float(contrast),
            "dark_channel": float(dark_channel_score or 0.0),
            "saturation": (
                float(1.0 - saturation_score) if saturation_score is not None else 0.5
            ),
        }

    except Exception as e:
        print(f"[WARN] Fog detection error: {e}")
        return {"fog_detected": False, "fog_level": 0.0, "confidence": 0.0}


def analyze_image_for_rain(image: np.ndarray) -> Dict[str, float]:
    """Detect rain from camera image."""
    if image is None or image.size == 0:
        return {"rain_detected": False, "rain_intensity": 0.0, "confidence": 0.0}

    try:
        if len(image.shape) == 2:
            gray = image
        else:
            if HAS_OPENCV:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = np.mean(image, axis=2).astype(np.uint8)

        rain_score = 0.0
        confidence = 0.0

        if HAS_OPENCV:
            sobel_v = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
            sobel_h = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)

            vertical_energy = np.abs(sobel_v).mean()
            horizontal_energy = np.abs(sobel_h).mean()

            # Rain has more vertical than horizontal edges
            if horizontal_energy > 0:
                streak_ratio = vertical_energy / (horizontal_energy + 1e-6)
                streak_score = (
                    min(1.0, (streak_ratio - 1.0) / 2.0) if streak_ratio > 1 else 0
                )
            else:
                streak_score = 0.0

            # 2. Laplacian variance (sharpness - rain reduces it)
            laplacian = cv2.Laplacian(gray, cv2.CV_64F)
            sharpness = laplacian.var() / 1000.0  # Normalize
            blur_score = 1.0 - min(1.0, sharpness)

            # 3. High frequency noise (rain creates noise patterns)
            blur_diff = cv2.GaussianBlur(gray, (5, 5), 0).astype(float) - gray.astype(
                float
            )
            noise_level = np.abs(blur_diff).mean() / 30.0
            noise_score = min(1.0, noise_level)

            # 4. Local variance analysis (rain creates local intensity variations)
            local_var = (
                cv2.blur(gray.astype(float) ** 2, (15, 15))
                - cv2.blur(gray.astype(float), (15, 15)) ** 2
            )
            local_var_score = min(1.0, local_var.mean() / 500.0)

            rain_score = (
                streak_score * 0.4
                + blur_score * 0.2
                + noise_score * 0.2
                + local_var_score * 0.2
            )
            confidence = min(1.0, rain_score * 1.5)

        rain_detected = rain_score > EnvironmentConfig.RAIN_STREAK_THRESHOLD

        return {
            "rain_detected": rain_detected,
            "rain_intensity": float(rain_score),
            "confidence": float(confidence),
        }

    except Exception as e:
        print(f"[WARN] Rain detection error: {e}")
        return {"rain_detected": False, "rain_intensity": 0.0, "confidence": 0.0}


def analyze_lighting_conditions(image: np.ndarray) -> Dict[str, float]:
    """Analyze lighting conditions from camera image."""
    if image is None or image.size == 0:
        return {"brightness": 0.5, "is_low_light": False, "is_bright": False}

    try:
        if len(image.shape) == 2:
            gray = image
        else:
            if HAS_OPENCV:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = np.mean(image, axis=2).astype(np.uint8)

        brightness = gray.mean()
        brightness_normalized = brightness / 255.0

        is_low_light = brightness < EnvironmentConfig.LOW_LIGHT_THRESHOLD
        is_bright = brightness > 200

        h, w = gray.shape
        regions = [
            gray[: h // 2, :],  # Top half
            gray[h // 2 :, :],  # Bottom half
            gray[:, : w // 2],  # Left half
            gray[:, w // 2 :],  # Right half
        ]

        region_brightness = [r.mean() for r in regions]
        brightness_variance = np.std(region_brightness) / 255.0

        return {
            "brightness": float(brightness_normalized),
            "is_low_light": bool(is_low_light),
            "is_bright": bool(is_bright),
            "brightness_uniformity": float(1.0 - min(1.0, brightness_variance * 4)),
            "raw_brightness": float(brightness),
        }

    except Exception as e:
        print(f"[WARN] Lighting analysis error: {e}")
        return {"brightness": 0.5, "is_low_light": False, "is_bright": False}


# ============== LIDAR-BASED DETECTION ==============


def analyze_lidar_for_weather(
    points: List[Tuple[float, float]], expected_max_range: float = 12000
) -> Dict[str, float]:
    """Analyze LiDAR data for weather conditions."""
    if not points or len(points) < 10:
        return {"quality": 0.5, "weather_interference": 0.0, "confidence": 0.0}

    try:
        distances = [d for _, d in points if d > 0]

        if len(distances) < 10:
            return {"quality": 0.5, "weather_interference": 0.0, "confidence": 0.0}

        # 1. Range analysis (weather reduces max range)
        max_range = max(distances)
        range_ratio = max_range / expected_max_range
        range_score = min(1.0, range_ratio)

        # 2. Point density (weather reduces valid returns)
        expected_points = 360 / 2  # Assuming 2-degree resolution
        density_ratio = len(distances) / expected_points
        density_score = min(1.0, density_ratio)

        # 3. Noise analysis (weather increases variance in similar angles)
        angle_bins = {}
        for angle, dist in points:
            bin_idx = int(angle / 5) * 5
            if bin_idx not in angle_bins:
                angle_bins[bin_idx] = []
            angle_bins[bin_idx].append(dist)

        variances = []
        for bin_points in angle_bins.values():
            if len(bin_points) >= 3:
                mean_d = np.mean(bin_points)
                if mean_d > 0:
                    var_ratio = np.std(bin_points) / mean_d
                    variances.append(var_ratio)

        noise_level = np.mean(variances) if variances else 0.0
        noise_score = 1.0 - min(1.0, noise_level / 0.2)  # Higher noise = lower score

        # 4. Invalid return ratio
        total_expected = 360  # One point per degree expected
        valid_ratio = len(distances) / total_expected
        validity_score = min(1.0, valid_ratio)

        # 5. Distance distribution (weather creates more close returns from droplets)
        close_returns = sum(1 for d in distances if d < 500)
        close_ratio = close_returns / len(distances)
        scatter_score = 1.0 - min(1.0, close_ratio * 5)  # Many close returns = weather

        quality = (
            range_score * 0.2
            + density_score * 0.2
            + noise_score * 0.3
            + validity_score * 0.15
            + scatter_score * 0.15
        )

        weather_interference = 1.0 - quality

        return {
            "quality": float(quality),
            "weather_interference": float(weather_interference),
            "range_score": float(range_score),
            "density_score": float(density_score),
            "noise_level": float(noise_level),
            "confidence": float(min(1.0, len(distances) / 100)),
        }

    except Exception as e:
        print(f"[WARN] LiDAR weather analysis error: {e}")
        return {"quality": 0.5, "weather_interference": 0.0, "confidence": 0.0}


# ============== MAIN DETECTION FUNCTION ==============


def detect_environment_conditions(
    image: Optional[np.ndarray] = None,
    lidar_points: Optional[List[Tuple[float, float]]] = None,
    image_bytes: Optional[bytes] = None,
    record: bool = True,
) -> Dict[str, Any]:
    """Main function to detect environment conditions from camera and LiDAR. `record`: fold
    this reading into the shared history and state the camera trust reads (the detection
    thread). False: an answer only, from a copy (GET /environment/status, whose readings
    went into the history the guidance trusts, 2026-10-23)."""
    global environment_state

    results = {
        "fog": {"detected": False, "level": 0.0, "confidence": 0.0},
        "rain": {"detected": False, "intensity": 0.0, "confidence": 0.0},
        "lighting": {"brightness": 0.5, "is_low_light": False},
        "lidar": {"quality": 1.0, "interference": 0.0},
    }

    if image is None and image_bytes is not None and HAS_OPENCV:
        try:
            nparr = np.frombuffer(image_bytes, np.uint8)
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        except Exception as e:
            print(f"[WARN] Failed to decode image: {e}")

    if image is not None and HAS_OPENCV:
        try:
            target_width = 640
            h, w = image.shape[:2]
            if w > target_width:
                scale = target_width / float(w)
                image = cv2.resize(
                    image,
                    (target_width, int(round(h * scale))),
                    interpolation=cv2.INTER_AREA,
                )
        except Exception as e:
            print(f"[WARN] Failed to downscale image: {e}")

    if image is not None:
        fog_result = analyze_image_for_fog(image)
        rain_result = analyze_image_for_rain(image)
        lighting_result = analyze_lighting_conditions(image)

        results["fog"] = {
            "detected": fog_result.get("fog_detected", False),
            "level": fog_result.get("fog_level", 0.0),
            "confidence": fog_result.get("confidence", 0.0),
        }
        results["rain"] = {
            "detected": rain_result.get("rain_detected", False),
            "intensity": rain_result.get("rain_intensity", 0.0),
            "confidence": rain_result.get("confidence", 0.0),
        }
        results["lighting"] = lighting_result

    if lidar_points:
        lidar_result = analyze_lidar_for_weather(lidar_points)
        results["lidar"] = {
            "quality": lidar_result.get("quality", 1.0),
            "interference": lidar_result.get("weather_interference", 0.0),
        }

    # The history is shared by the detection thread and the web: one at a time (it was
    # changed and read with no lock), and only a recording caller changes it.
    with _history_lock:
        history = (
            detection_history
            if record
            else {k: deque(v, maxlen=v.maxlen) for k, v in detection_history.items()}
        )
        if image is not None:
            if history["fog"]:
                avg_fog = np.mean(list(history["fog"]))
                if (
                    abs(results["fog"]["level"] - avg_fog) > 0.4
                ):  # Significant change threshold
                    history["fog"].clear()

            if history["rain"]:
                avg_rain = np.mean(list(history["rain"]))
                if abs(results["rain"]["intensity"] - avg_rain) > 0.4:
                    history["rain"].clear()

            history["fog"].append(results["fog"]["level"])
            history["rain"].append(results["rain"]["intensity"])
            history["brightness"].append(results["lighting"]["brightness"])

        if lidar_points:
            history["lidar_quality"].append(results["lidar"]["quality"])

        if history["fog"]:
            fog_values = list(history["fog"])
            # Weight recent values more: [0.5, 0.3, 0.2] for 3 frames
            weights = np.array([0.5**i for i in range(len(fog_values) - 1, -1, -1)])
            weights = weights / weights.sum()
            results["fog"]["level"] = np.average(fog_values, weights=weights)

        if history["rain"]:
            rain_values = list(history["rain"])
            weights = np.array([0.5**i for i in range(len(rain_values) - 1, -1, -1)])
            weights = weights / weights.sum()
            results["rain"]["intensity"] = np.average(rain_values, weights=weights)

        if history["lidar_quality"]:
            lidar_values = list(history["lidar_quality"])
            weights = np.array([0.5**i for i in range(len(lidar_values) - 1, -1, -1)])
            weights = weights / weights.sum()
            results["lidar"]["quality"] = np.average(lidar_values, weights=weights)

    condition = "clear"
    visibility = 1.0
    warnings = []

    fog_level = results["fog"]["level"]
    rain_intensity = results["rain"]["intensity"]
    is_low_light = results["lighting"].get("is_low_light", False)
    lidar_quality = results["lidar"]["quality"]

    # Priority: Storm > Rain > Fog > Low Light > Clear
    if fog_level > 0.6 and rain_intensity > 0.3:
        condition = "storm"
        visibility = 0.2
        warnings.append("⛈️ Storm conditions - very poor visibility")
    elif rain_intensity > 0.4:
        condition = "rain"
        visibility = max(0.3, 1.0 - rain_intensity)
        if rain_intensity > 0.6:
            warnings.append("🌧️ Heavy rain - reduced visibility")
        else:
            warnings.append("🌦️ Rain detected - proceed with caution")
    elif fog_level > 0.4:
        condition = "fog"
        visibility = max(0.3, 1.0 - fog_level)
        if fog_level > 0.7:
            warnings.append("🌫️ Dense fog - very limited visibility")
        else:
            warnings.append("🌁 Fog detected - reduced visibility")
    elif is_low_light:
        condition = "low_light"
        visibility = 0.6
        warnings.append("🌙 Low light conditions")

    if lidar_quality < 0.5:
        warnings.append("📡 LiDAR quality degraded - possible weather interference")

    confidences = [
        results["fog"]["confidence"],
        results["rain"]["confidence"],
        results["lidar"].get("confidence", 0.5) if lidar_points else 0,
    ]
    overall_confidence = (
        np.mean([c for c in confidences if c > 0])
        if any(c > 0 for c in confidences)
        else 0.5
    )

    state = {
        "condition": condition,
        "visibility": float(visibility),
        "confidence": float(overall_confidence),
        "details": {
            "fog_level": float(fog_level),
            "rain_intensity": float(rain_intensity),
            "brightness": float(results["lighting"].get("brightness", 0.5)),
            "contrast": float(results["lighting"].get("brightness_uniformity", 0.5)),
            "lidar_quality": float(lidar_quality),
        },
        "warnings": warnings,
        "last_update": time.time(),
    }
    if record:
        environment_state = state
    return dict(state)


def get_environment_state() -> Dict[str, Any]:
    """Get current environment state"""
    return environment_state.copy()


def clear_detection_history():
    """Clear detection history for immediate reset (useful when conditions change rapidly)"""
    with _history_lock:
        for values in detection_history.values():
            values.clear()
    print("[ENV] Detection history cleared for fast reset")


def get_visibility_description(visibility: float) -> str:
    """Get human-readable visibility description"""
    if visibility >= 0.9:
        return "Excellent"
    elif visibility >= 0.7:
        return "Good"
    elif visibility >= 0.5:
        return "Moderate"
    elif visibility >= 0.3:
        return "Poor"
    else:
        return "Very Poor"


def get_condition_icon(condition: str) -> str:
    """Get icon for condition"""
    icons = {"clear": "☀️", "fog": "🌫️", "rain": "🌧️", "low_light": "🌙", "storm": "⛈️"}
    return icons.get(condition, "❓")
