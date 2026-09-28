"""Image processing package for RoboMaster EP target detection.

Specialized in color and shape detection at close range and under perspective distortion.
"""

from .config import (
    COLORS,
    SHAPES,
    DEFAULT_DETECTION_CONFIG,
    CAMERA_CALIBRATION_720P,
    CAMERA_CALIBRATION_360P,
)
from .detector import (
    Detection,
    ColorShapeDetector,
    detect,
    classify_shape_robust,
    classify_shape_classic,
    color_mask,
    undistort_frame,
)

__all__ = [
    "COLORS",
    "SHAPES",
    "DEFAULT_DETECTION_CONFIG",
    "CAMERA_CALIBRATION_720P",
    "CAMERA_CALIBRATION_360P",
    "Detection",
    "ColorShapeDetector",
    "detect",
    "classify_shape_robust",
    "classify_shape_classic",
    "color_mask",
    "undistort_frame",
]
