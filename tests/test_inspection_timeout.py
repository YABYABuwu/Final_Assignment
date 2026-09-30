import time
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

from src.config_loader import load_config
from src.mission_stop import MissionStop
from src.target_inspection import WallTargetInspector


class InspectionTimeoutTests(unittest.TestCase):
    def setUp(self):
        self.settings = load_config()["exploration"]
        self.settings["target_inspection"].update(wait_timeout_s=.15, gimbal_wait_timeout_s=.15, aim_settle_s=0)
        self.logger = Mock()
        self.chassis = Mock()
        self.gimbal = Mock()
        self.inspector = WallTargetInspector(
            self.gimbal, Mock(), SimpleNamespace(camera_error=None), self.logger,
            self.chassis, SimpleNamespace(status=lambda: {}), self.settings, "infrared")

    def test_missing_angles_exit_and_stop(self):
        self.logger.get_sample.return_value = None
        with self.assertRaisesRegex(MissionStop, "fresh gimbal"):
            self.inspector._angles()
        self.chassis.stop.assert_called()

    def test_stale_safety_data_rejected_even_if_logger_returns_it(self):
        self.logger.get_sample.return_value = ((0,) * 10, time.time() - 100)
        with self.assertRaisesRegex(MissionStop, "safety telemetry"):
            self.inspector._safe_status()

    def test_angles_must_be_newer_than_command(self):
        stamp = time.time()
        self.logger.get_sample.return_value = ((0, 0, 0), stamp)
        with self.assertRaises(MissionStop):
            self.inspector._angles(after=stamp)

    def test_frozen_action_exits_without_releasing_or_overlapping(self):
        self.logger.get_sample.side_effect = lambda *a, **k: ((0,) * 10, time.time())
        action = SimpleNamespace(state="action_running", has_succeeded=False)
        self.gimbal.moveto.return_value = action
        with self.assertRaises(MissionStop):
            self.inspector._point(0, 0)
        self.assertIs(self.inspector.active_action, action)
        self.assertIn("action_state=action_running", self.inspector.wait_error)
        self.assertIn("command=moveto", self.inspector.wait_error)
        self.assertIn("sample_after_command=True", self.inspector.wait_error)
        self.gimbal.recenter.assert_not_called()

    def test_wrong_angle_times_out_after_action_success(self):
        self.logger.get_sample.side_effect = lambda *a, **k: ((0,) * 10, time.time())
        wait = Mock(return_value=True)
        self.gimbal.moveto.return_value = SimpleNamespace(has_succeeded=True, wait_for_completed=wait)
        with self.assertRaises(MissionStop):
            self.inspector._point(-20, 90)
        self.assertLessEqual(wait.call_args.kwargs["timeout"], .151)
        self.assertIn("action_released=True", self.inspector.wait_error)
        self.assertIn("target_yaw_deg=90", self.inspector.wait_error)

    def test_fresh_aligned_success_returns(self):
        self.logger.get_sample.side_effect = lambda *a, **k: ((0,) * 10, time.time())
        self.gimbal.moveto.return_value = None
        self.assertEqual(self.inspector._point(0, 0), (0, 0))

    def test_invalid_timestamps_rejected(self):
        for stamp in (float("nan"), float("inf"), time.time() + 100, time.time() - 100):
            self.assertFalse(self.inspector._fresh_timestamp(stamp))

    def test_configured_deadlines_are_separate(self):
        self.inspector.settings = load_config()["exploration"]
        with patch("src.target_inspection.time.monotonic", return_value=100):
            self.assertEqual(self.inspector._deadline(), 105)
            self.assertEqual(self.inspector._deadline(gimbal=True), 108)

    def test_restore_timeout_identifies_command_and_stale_data(self):
        self.inspector.command_detail = {"command": "recenter", "phase": "restore",
                                         "pitch_frame": "chassis", "requested_at": time.time()}
        self.logger.get_sample.return_value = ((1, 2, 3), time.time() - 20)
        with self.assertRaises(MissionStop):
            self.inspector._check_deadline(0, "waiting")
        self.assertIn("command=recenter", self.inspector.wait_error)
        self.assertIn("phase=restore", self.inspector.wait_error)
        self.assertIn("sample_fresh=False", self.inspector.wait_error)
        self.assertIn("sample_after_command=False", self.inspector.wait_error)
