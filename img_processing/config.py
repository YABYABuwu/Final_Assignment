"""Configuration and default parameters for image processing."""

import numpy as np

# Color definitions with HSV ranges, display colors, and labels
COLORS = {
    "red": {
        "label": "แดง",
        "bgr": (40, 40, 240),
        "ranges": [
            ((0, 100, 60), (10, 255, 255)),
            ((165, 100, 60), (179, 255, 255)),
        ],
    },
    "green": {
        "label": "เขียว",
        "bgr": (50, 200, 50),
        "ranges": [
            ((35, 70, 25), (92, 255, 255)),
        ],
    },
    "yellow": {
        "label": "เหลือง",
        "bgr": (0, 220, 240),
        "ranges": [
            ((18, 90, 60), (35, 255, 255)),
        ],
    },
    "blue": {
        "label": "น้ำเงิน",
        "bgr": (240, 100, 30),
        "ranges": [
            ((95, 80, 40), (135, 255, 255)),
        ],
    },
}

SHAPES = {
    "circle": "วงกลม",
    "square": "จัตุรัส",
    "horizontal": "ผืนผ้าแนวนอน",
    "vertical": "ผืนผ้าแนวตั้ง",
}

# Detection thresholds
DEFAULT_DETECTION_CONFIG = {
    "min_area": 600,             # Minimum contour area in px^2
    "max_area": 250000,          # Maximum contour area in px^2 (large for close-range)
    "morph_kernel_size": 3,      # Morphology kernel size
    "morph_iterations": 1,       # Morphology iterations
    
    # Classic mode thresholds (strict, matches ColorLockingLAB)
    "classic_circularity": 0.70,
    "classic_ratio": 0.75,
    "classic_square_ratio": 0.85,
    "classic_fill_ratio": 0.78,
    
    # Robust mode thresholds (handles close-range and perspective distortion)
    "robust_ellipse_min_ratio": 0.85,  # contour_area / ellipse_area min
    "robust_ellipse_max_ratio": 1.15,  # contour_area / ellipse_area max
    "robust_square_ratio": 0.80,       # Relaxed square ratio threshold to tolerate perspective foreshortening
    "robust_fill_ratio": 0.70,         # Relaxed minAreaRect fill ratio (trapezoid fills ~72-78%)
    "adaptive_epsilons": [0.02, 0.025, 0.03, 0.035, 0.04, 0.05],
    "use_convex_hull": True,           # Smooth out edge notches before polygon approx
    "enable_undistort": True,          # Enable camera lens barrel undistortion
}

# Default RoboMaster EP Camera Calibration Parameters
# Measured for 1280x720 (720p) stream, FOV ~120 deg
CAMERA_CALIBRATION_720P = {
    "camera_matrix": np.array([
        [640.0, 0.0,   640.0],
        [0.0,   640.0, 360.0],
        [0.0,   0.0,   1.0]
    ], dtype=np.float32),
    # Typical barrel distortion coefficients [k1, k2, p1, p2, k3]
    "dist_coeffs": np.array([-0.30, 0.10, 0.0, 0.0, -0.01], dtype=np.float32),
}

# Scaled for 640x360 (360p) stream
CAMERA_CALIBRATION_360P = {
    "camera_matrix": np.array([
        [320.0, 0.0,   320.0],
        [0.0,   320.0, 180.0],
        [0.0,   0.0,   1.0]
    ], dtype=np.float32),
    "dist_coeffs": np.array([-0.30, 0.10, 0.0, 0.0, -0.01], dtype=np.float32),
}
