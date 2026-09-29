"""Color and shape detector with robust close-range and perspective handling."""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Tuple, Union

import cv2
import numpy as np

from pathlib import Path
import sys

# Support running directly or as a module
_pkg_dir = Path(__file__).resolve().parent
_root_dir = _pkg_dir.parent
for _p in (str(_pkg_dir), str(_root_dir)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from .config import (
        COLORS,
        SHAPES,
        DEFAULT_DETECTION_CONFIG,
        CAMERA_CALIBRATION_720P,
        CAMERA_CALIBRATION_360P,
    )
except (ImportError, ValueError):
    from config import (
        COLORS,
        SHAPES,
        DEFAULT_DETECTION_CONFIG,
        CAMERA_CALIBRATION_720P,
        CAMERA_CALIBRATION_360P,
    )


@dataclass
class Detection:
    color: str
    shape: str
    area: float
    center: Tuple[int, int]
    contour: np.ndarray
    metrics: Dict[str, Union[float, int, bool]] = field(default_factory=dict)
    reasons: List[str] = field(default_factory=list)
    mode: str = "robust"


def get_camera_calibration(width: int, height: int) -> Tuple[np.ndarray, np.ndarray]:
    """Scale the 720p calibration matrix to the actual stream resolution."""
    if width <= 0 or height <= 0:
        raise ValueError("frame width and height must be positive")
    camera_matrix = CAMERA_CALIBRATION_720P["camera_matrix"].copy()
    camera_matrix[0, :] *= width / 1280.0
    camera_matrix[1, :] *= height / 720.0
    camera_matrix[2, :] = (0.0, 0.0, 1.0)
    return camera_matrix, CAMERA_CALIBRATION_720P["dist_coeffs"].copy()


def undistort_frame(frame: np.ndarray, camera_matrix: Optional[np.ndarray] = None,
                    dist_coeffs: Optional[np.ndarray] = None) -> np.ndarray:
    """Removes barrel distortion caused by wide-angle camera lens."""
    h, w = frame.shape[:2]
    if camera_matrix is None or dist_coeffs is None:
        camera_matrix, dist_coeffs = get_camera_calibration(w, h)
    return cv2.undistort(frame, camera_matrix, dist_coeffs)


def color_mask(hsv: np.ndarray, color: str, kernel_size: int = 3, iterations: int = 1,
               color_ranges: Optional[dict] = None) -> np.ndarray:
    """Build binary mask for target color using HSV ranges and morphology."""
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    ranges = COLORS[color]["ranges"] if color_ranges is None else color_ranges[color]
    for lower, upper in ranges:
        mask |= cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
    if kernel_size > 0:
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=iterations)
    return mask


def classify_shape_classic(contour: np.ndarray, config: dict = DEFAULT_DETECTION_CONFIG) -> Tuple[Optional[str], dict, List[str]]:
    """Original ColorLockingLAB shape classification algorithm."""
    metrics = {}
    reasons = []
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    metrics["area"] = area
    metrics["perimeter"] = perimeter

    min_area = config.get("min_area", 800)
    if area < min_area:
        reasons.append(f"area < {min_area}")
        return None, metrics, reasons
    if perimeter <= 0:
        reasons.append("perimeter <= 0")
        return None, metrics, reasons

    corners = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
    rotated = cv2.minAreaRect(contour)
    side_a, side_b = rotated[1]
    metrics["corners"] = len(corners)
    metrics["side_a"] = side_a
    metrics["side_b"] = side_b

    if side_a <= 0 or side_b <= 0:
        reasons.append("invalid bounding sides")
        return None, metrics, reasons

    ratio = min(side_a, side_b) / max(side_a, side_b)
    circularity = 4 * np.pi * area / (perimeter * perimeter)
    fill_ratio = area / (side_a * side_b)
    metrics["ratio"] = ratio
    metrics["circularity"] = circularity
    metrics["fill_ratio"] = fill_ratio

    # Circle check
    if circularity >= config.get("classic_circularity", 0.70) and ratio >= config.get("classic_ratio", 0.75) and len(corners) >= 7:
        return "circle", metrics, reasons

    # Convex and 4 corners check
    if len(corners) != 4:
        reasons.append(f"corners={len(corners)} != 4")
        return None, metrics, reasons
    if not cv2.isContourConvex(corners):
        reasons.append("not convex")
        return None, metrics, reasons

    # Fill ratio
    classic_fill = config.get("classic_fill_ratio", 0.78)
    if fill_ratio < classic_fill:
        reasons.append(f"fill={fill_ratio:.2f} < {classic_fill}")
        return None, metrics, reasons

    # Square vs Rectangle
    square_ratio = config.get("classic_square_ratio", 0.85)
    if ratio >= square_ratio:
        return "square", metrics, reasons

    box = cv2.boxPoints(rotated)
    edges = np.roll(box, -1, axis=0) - box
    long_edge = max(edges, key=lambda edge: np.dot(edge, edge))
    shape = "horizontal" if abs(long_edge[0]) >= abs(long_edge[1]) else "vertical"
    return shape, metrics, reasons


def _calculate_polygon_angles(pts: np.ndarray) -> List[float]:
    """Calculate interior angles in degrees for each vertex of a polygon."""
    angles = []
    n = len(pts)
    for i in range(n):
        p_prev = pts[i - 1][0]
        p_curr = pts[i][0]
        p_next = pts[(i + 1) % n][0]
        v1 = p_prev - p_curr
        v2 = p_next - p_curr
        len1 = math.hypot(v1[0], v1[1])
        len2 = math.hypot(v2[0], v2[1])
        if len1 == 0 or len2 == 0:
            angles.append(0.0)
            continue
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        cos_val = max(-1.0, min(1.0, dot / (len1 * len2)))
        angles.append(math.degrees(math.acos(cos_val)))
    return angles


def classify_shape_robust(contour: np.ndarray, config: dict = DEFAULT_DETECTION_CONFIG) -> Tuple[Optional[str], dict, List[str]]:
    """Robust shape classification designed for close-range and perspective distortion.

    Key improvements over classic:
    1. Ellipse Fitting for Circles: Under perspective slant, circles project to ellipses.
       Area / Fitted Ellipse Area stays ~1.0 (0.85-1.15) regardless of viewing angle.
    2. Convex Hull preprocessing: Eliminates edge notches, lighting reflections, and ripples.
    3. Adaptive Epsilon: Scans epsilon factors [0.02..0.05] so barrel-curved edges
       are simplified to 4 clean vertices without artificial splitting.
    4. Relaxed Quadrilateral Fill & Ratio: Recognizes trapezoids caused by perspective foreshortening.
    """
    metrics = {}
    reasons = []
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    metrics["area"] = area
    metrics["perimeter"] = perimeter

    min_area = config.get("min_area", 600)
    max_area = config.get("max_area", 250000)
    if area < min_area:
        reasons.append(f"area {area:.0f} < {min_area}")
        return None, metrics, reasons
    if area > max_area:
        reasons.append(f"area {area:.0f} > {max_area}")
        return None, metrics, reasons
    if perimeter <= 0:
        reasons.append("perimeter <= 0")
        return None, metrics, reasons

    # Pre-smooth with convex hull if enabled
    use_hull = config.get("use_convex_hull", True)
    hull = cv2.convexHull(contour) if use_hull else contour
    hull_perimeter = cv2.arcLength(hull, True)
    hull_area = cv2.contourArea(hull)

    # 1. Circle Check using Direct Least Squares Ellipse Fitting
    # Under perspective projection, a circle transforms into an ellipse!
    if len(contour) >= 5:
        try:
            ellipse = cv2.fitEllipse(contour)
            center, (d1, d2), angle = ellipse
            major_axis = max(d1, d2)
            minor_axis = min(d1, d2)
            if major_axis > 0 and minor_axis > 0:
                ellipse_area = (np.pi / 4.0) * major_axis * minor_axis
                ellipse_fit_ratio = area / ellipse_area
                axis_ratio = minor_axis / major_axis
                metrics["ellipse_fit_ratio"] = ellipse_fit_ratio
                metrics["ellipse_axis_ratio"] = axis_ratio
                metrics["ellipse_angle"] = angle

                # Check if it matches an ellipse (a circle viewed from an angle)
                min_fit = config.get("robust_ellipse_min_ratio", 0.85)
                max_fit = config.get("robust_ellipse_max_ratio", 1.15)
                if min_fit <= ellipse_fit_ratio <= max_fit and axis_ratio >= 0.45:
                    # Verify it does not have 4 sharp perpendicular corners
                    test_poly = cv2.approxPolyDP(hull, 0.03 * hull_perimeter, True)
                    if len(test_poly) != 4:
                        return "circle", metrics, reasons
                    # If 4 corners found, check if angles are close to 90 deg; if not, it's a circle
                    angles = _calculate_polygon_angles(test_poly)
                    sharp_corner_count = sum(1 for a in angles if 75 <= a <= 105)
                    if sharp_corner_count < 3:
                        return "circle", metrics, reasons
        except Exception:
            pass

    # 2. Quadrilateral (Square / Rectangle) Check with Adaptive Epsilon
    best_poly = None
    epsilons = config.get("adaptive_epsilons", [0.02, 0.025, 0.03, 0.035, 0.04, 0.05])
    for eps in epsilons:
        approx = cv2.approxPolyDP(hull, eps * hull_perimeter, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            best_poly = approx
            break

    if best_poly is None:
        # Fallback to direct contour approx
        for eps in [0.02, 0.03, 0.04]:
            approx = cv2.approxPolyDP(contour, eps * perimeter, True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                best_poly = approx
                break

    if best_poly is None:
        default_corners = len(cv2.approxPolyDP(contour, 0.02 * perimeter, True))
        reasons.append(f"no 4-corner quad found (approx {default_corners} corners)")
        return None, metrics, reasons

    # Analyze Quadrilateral Geometry
    angles = _calculate_polygon_angles(best_poly)
    metrics["angles"] = angles
    # Interior angles of perspective-distorted rectangle should stay in [55, 125]
    valid_angles = all(50 <= a <= 130 for a in angles)
    if not valid_angles:
        reasons.append(f"angles out of bounds: {[round(a, 1) for a in angles]}")
        return None, metrics, reasons

    rotated = cv2.minAreaRect(contour)
    side_a, side_b = rotated[1]
    if side_a <= 0 or side_b <= 0:
        reasons.append("invalid minAreaRect sides")
        return None, metrics, reasons

    fill_ratio = area / (side_a * side_b)
    min_fill = config.get("robust_fill_ratio", 0.70)
    metrics["fill_ratio"] = fill_ratio
    metrics["side_a"] = side_a
    metrics["side_b"] = side_b

    if fill_ratio < min_fill:
        reasons.append(f"fill {fill_ratio:.2f} < {min_fill}")
        return None, metrics, reasons

    # Calculate edge lengths directly from detected 4 points
    pts = best_poly.reshape(4, 2)
    edges = [math.hypot(pts[(i + 1) % 4][0] - pts[i][0], pts[(i + 1) % 4][1] - pts[i][1]) for i in range(4)]
    # Opposite edges in quadrilateral
    w_avg = (edges[0] + edges[2]) / 2.0
    h_avg = (edges[1] + edges[3]) / 2.0
    quad_ratio = min(w_avg, h_avg) / max(w_avg, h_avg)
    rotated_ratio = min(side_a, side_b) / max(side_a, side_b)
    effective_ratio = max(quad_ratio, rotated_ratio)
    metrics["ratio"] = effective_ratio
    metrics["quad_ratio"] = quad_ratio
    metrics["rotated_ratio"] = rotated_ratio

    square_ratio = config.get("robust_square_ratio", 0.80)
    if effective_ratio >= square_ratio:
        return "square", metrics, reasons

    # Determine horizontal vs vertical orientation
    box = cv2.boxPoints(rotated)
    box_edges = np.roll(box, -1, axis=0) - box
    long_edge = max(box_edges, key=lambda edge: np.dot(edge, edge))
    shape = "horizontal" if abs(long_edge[0]) >= abs(long_edge[1]) else "vertical"
    return shape, metrics, reasons


def detect(
    frame: np.ndarray,
    mode: str = "robust",
    config: Optional[dict] = None,
    undistort: bool = False,
    debug: bool = False,
    color_ranges: Optional[dict] = None,
) -> Tuple[List[Detection], Dict[str, np.ndarray]]:
    """Detect colored shapes in image frame."""
    if config is None:
        config = DEFAULT_DETECTION_CONFIG

    if undistort:
        frame = undistort_frame(frame)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    detections: List[Detection] = []
    masks: Dict[str, np.ndarray] = {}

    kernel_size = config.get("morph_kernel_size", 3)
    iterations = config.get("morph_iterations", 1)

    for color_name in COLORS:
        mask = color_mask(hsv, color_name, kernel_size, iterations, color_ranges)
        masks[color_name] = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if mode == "classic":
                shape, metrics, reasons = classify_shape_classic(contour, config)
            else:
                shape, metrics, reasons = classify_shape_robust(contour, config)

            if shape is None and not debug:
                continue

            moments = cv2.moments(contour)
            if moments["m00"] != 0:
                center = (int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"]))
            else:
                x, y, w, h = cv2.boundingRect(contour)
                center = (x + w // 2, y + h // 2)

            detections.append(Detection(
                color=color_name,
                shape=shape or "unknown",
                area=cv2.contourArea(contour),
                center=center,
                contour=contour,
                metrics=metrics,
                reasons=reasons,
                mode=mode,
            ))

    detections.sort(key=lambda d: d.area, reverse=True)
    return detections, masks


class ColorShapeDetector:
    """Reusable stateful detector with caching for camera maps."""

    def __init__(self, mode: str = "robust", config: Optional[dict] = None,
                 enable_undistort: bool = True, color_ranges: Optional[dict] = None):
        self.mode = mode
        self.config = config or DEFAULT_DETECTION_CONFIG.copy()
        self.enable_undistort = enable_undistort
        self.color_ranges = color_ranges
        self._map1 = None
        self._map2 = None
        self._last_shape = None

    def detect(self, frame: np.ndarray, debug: bool = False) -> Tuple[List[Detection], Dict[str, np.ndarray], np.ndarray]:
        """Process frame and return (detections, masks, processed_frame)."""
        proc_frame = frame
        if self.enable_undistort:
            h, w = frame.shape[:2]
            if self._last_shape != (w, h):
                cam_mat, dist = get_camera_calibration(w, h)
                self._map1, self._map2 = cv2.initUndistortRectifyMap(cam_mat, dist, None, cam_mat, (w, h), cv2.CV_32FC1)
                self._last_shape = (w, h)
            proc_frame = cv2.remap(frame, self._map1, self._map2, cv2.INTER_LINEAR)

        detections, masks = detect(proc_frame, mode=self.mode, config=self.config,
                                   undistort=False, debug=debug,
                                   color_ranges=self.color_ranges)
        return detections, masks, proc_frame

    @staticmethod
    def annotate(frame: np.ndarray, detections: List[Detection]) -> np.ndarray:
        """Return a copy with compact ASCII labels suitable for the live MJPEG feed."""
        annotated = frame.copy()
        for detection in detections:
            color_settings = COLORS.get(detection.color, {})
            bgr = color_settings.get("bgr", (255, 255, 255))
            cv2.drawContours(annotated, [detection.contour], -1, bgr, 2)
            cv2.circle(annotated, detection.center, 4, bgr, -1)
            x, y, _, _ = cv2.boundingRect(detection.contour)
            label = f"{detection.color.upper()} {detection.shape.upper()} {detection.area:.0f}px2"
            cv2.putText(annotated, label, (x, max(20, y - 7)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, bgr, 2, cv2.LINE_AA)
        return annotated
