"""Unit tests for the img_processing package."""

import unittest
from copy import deepcopy
import numpy as np
import cv2

from img_processing.detector import (
    ColorShapeDetector,
    detect,
    get_camera_calibration,
    classify_shape_classic,
    classify_shape_robust,
    color_mask,
    undistort_frame,
)
from img_processing.config import COLORS, DEFAULT_DETECTION_CONFIG
from src.config_loader import load_config
from src.targets import COLORS as TARGET_COLORS, detect as detect_target


class TestImageProcessing(unittest.TestCase):
    def setUp(self):
        # 720p blank image
        self.frame_h, self.frame_w = 720, 1280
        self.img = np.zeros((self.frame_h, self.frame_w, 3), dtype=np.uint8)

    def test_color_mask(self):
        # Draw red, green, yellow, blue boxes
        hsv_test = np.zeros((100, 100, 3), dtype=np.uint8)
        # Fill with pure green in HSV: H=60, S=200, V=200
        hsv_test[:] = (60, 200, 200)
        mask = color_mask(hsv_test, "green")
        self.assertGreater(cv2.countNonZero(mask), 5000)

    def test_shared_hsv_ranges_and_runtime_override(self):
        settings_ranges = load_config()["color_ranges"]
        self.assertEqual(TARGET_COLORS, settings_ranges)
        for color in settings_ranges:
            self.assertEqual(COLORS[color]["ranges"], settings_ranges[color])

        frame = np.zeros((360, 640, 3), dtype=np.uint8)
        cv2.rectangle(frame, (240, 100), (400, 260), (0, 0, 255), -1)
        swapped = deepcopy(settings_ranges)
        swapped["red"], swapped["green"] = swapped["green"], swapped["red"]
        self.assertIn(("red", "square"), {(d.color, d.shape) for d in detect_target(frame)})
        self.assertIn(("green", "square"), {
            (d.color, d.shape) for d in detect_target(frame, color_ranges=swapped)})
        self.assertIn(("green", "square"), {
            (d.color, d.shape) for d in detect(frame, color_ranges=swapped)[0]})
        detector = ColorShapeDetector(enable_undistort=False, color_ranges=swapped)
        self.assertIn(("green", "square"), {
            (d.color, d.shape) for d in detector.detect(frame)[0]})

    def test_ideal_shapes_detection(self):
        # Red circle
        cv2.circle(self.img, (250, 360), 70, (40, 40, 240), -1)
        # Green square
        cv2.rectangle(self.img, (550, 290), (690, 430), (50, 200, 50), -1)
        # Yellow horizontal rectangle
        cv2.rectangle(self.img, (850, 320), (1050, 400), (0, 220, 240), -1)

        detections, _ = detect(self.img, mode="robust")
        shapes_found = {(d.color, d.shape) for d in detections}
        self.assertIn(("red", "circle"), shapes_found)
        self.assertIn(("green", "square"), shapes_found)
        self.assertIn(("yellow", "horizontal"), shapes_found)

    def test_perspective_circle_as_ellipse(self):
        # Circle viewed at angle becomes an ellipse
        cv2.ellipse(self.img, (300, 360), (100, 65), 30, 0, 360, (40, 40, 240), -1)
        
        # Robust mode should successfully identify it as a circle
        robust_dets, _ = detect(self.img, mode="robust")
        robust_shapes = {(d.color, d.shape) for d in robust_dets}
        self.assertIn(("red", "circle"), robust_shapes)

        # Classic mode rejects the ellipse because circularity and ratio are too low
        classic_dets, _ = detect(self.img, mode="classic")
        classic_shapes = {(d.color, d.shape) for d in classic_dets}
        self.assertNotIn(("red", "circle"), classic_shapes)

    def test_perspective_square_as_trapezoid(self):
        # Square viewed obliquely forms a trapezoid
        trap_pts = np.array([[550, 310], [720, 260], [735, 460], [550, 410]], dtype=np.int32)
        cv2.fillPoly(self.img, [trap_pts], (50, 200, 50))

        # Robust mode identifies it as square
        robust_dets, _ = detect(self.img, mode="robust")
        robust_shapes = {(d.color, d.shape) for d in robust_dets}
        self.assertIn(("green", "square"), robust_shapes)

    def test_undistort_function(self):
        out = undistort_frame(self.img)
        self.assertEqual(out.shape, self.img.shape)

    def test_camera_calibration_scales_for_dashboard_540p(self):
        camera_matrix, _ = get_camera_calibration(960, 540)
        self.assertAlmostEqual(camera_matrix[0, 0], 480.0)
        self.assertAlmostEqual(camera_matrix[1, 1], 480.0)
        self.assertAlmostEqual(camera_matrix[0, 2], 480.0)
        self.assertAlmostEqual(camera_matrix[1, 2], 270.0)

    def test_detector_can_annotate_detected_signs(self):
        cv2.circle(self.img, (250, 360), 70, (40, 40, 240), -1)
        detector = ColorShapeDetector(mode="robust", enable_undistort=False)
        detections, _, processed = detector.detect(self.img)
        annotated = detector.annotate(processed, detections)
        self.assertEqual(annotated.shape, self.img.shape)
        self.assertFalse(np.array_equal(annotated, processed))


if __name__ == "__main__":
    unittest.main()
