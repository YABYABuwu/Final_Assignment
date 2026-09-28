import csv
import io
import json
from queue import Empty
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

from main import _sdk_connection_type
from src.PID import PIDController
from src.chassis import ChassisController, angle_error
from src.config_loader import load_config
from src.dashboard import Dashboard
from src.logger import SensorLogger


class FakeModule:
    def __init__(self):
        self.callbacks = {}
        self.commands = []

    def sub_position(self, **options):
        self.callbacks["position"] = options["callback"]
        return True

    def unsub_position(self):
        self.callbacks.pop("position", None)

    def sub_attitude(self, **options):
        self.callbacks["attitude"] = options["callback"]
        return True

    def unsub_attitude(self):
        self.callbacks.pop("attitude", None)

    def drive_speed(self, **speed):
        self.commands.append(speed)


class TemplateTests(unittest.TestCase):
    def test_config_and_pid(self):
        config = load_config()
        self.assertFalse(config["mission"]["enabled"])
        pid = PIDController(1, 0, 0, max_output=0.3)
        self.assertEqual(pid.compute(2, 0.1), 0.3)
        self.assertEqual(pid.compute(-2, 0.1), -0.3)
        self.assertEqual(angle_error(-179, 179), 2)
        self.assertTrue(config["dashboard"]["enabled"])
        self.assertEqual(config["motion"]["max_lateral_accel_m_s2"], .30)
        self.assertEqual(config["exploration"]["sensor"]["tof_channel"], 0)
        self.assertGreater(config["exploration"]["wall_threshold_mm"], 0)
        self.assertEqual(config["exploration"]["tof_median_window"], 3)
        self.assertIsInstance(config["exploration"]["alignment"]["enabled"], bool)
        self.assertEqual(config["exploration"]["emergency_stop_distance_m"], .20)
        self.assertNotIn("min_range_m", config["exploration"]["map"])
        self.assertNotIn("max_range_m", config["exploration"]["map"])

    def test_sdk_connection_uses_constant_objects(self):
        constants = SimpleNamespace(
            CONNECTION_WIFI_AP=object(),
            CONNECTION_WIFI_STA=object(),
            CONNECTION_USB_RNDIS=object(),
        )
        for name, expected in (
            ("ap", constants.CONNECTION_WIFI_AP),
            ("sta", constants.CONNECTION_WIFI_STA),
            ("rndis", constants.CONNECTION_USB_RNDIS),
        ):
            self.assertIs(_sdk_connection_type(name, constants), expected)

    def test_dashboard_serves_page_and_latest_telemetry(self):
        logger = SimpleNamespace(
            stream_settings={"position": {"enabled": True}, "tof": {"enabled": False}},
            dropped_rows=2,
            get_latest=lambda name, max_age_s=None: (1, 2, 0),
            get_history_since=lambda after_id=0: {
                "cursor": 3, "streams": {"position": [(3, 1.0, (1, 2, 0))]}
            },
        )
        robot = SimpleNamespace(camera=SimpleNamespace())
        dashboard = Dashboard(robot, logger, load_config()["dashboard"])
        dashboard.mission_status = "Ready"
        handler_type = dashboard._handler_class()

        def request(path):
            handler = handler_type.__new__(handler_type)
            handler.path = path
            handler.wfile = io.BytesIO()
            handler.send_response = lambda code: None
            handler.send_header = lambda name, value: None
            handler.end_headers = lambda: None
            handler.do_GET()
            return handler.wfile.getvalue()

        page = request("/")
        self.assertIn(b"RoboMaster Dashboard", page)
        status = json.loads(request("/api/status"))
        self.assertEqual(status["streams"]["position"], [1, 2, 0])
        self.assertNotIn("tof", status["streams"])
        self.assertEqual(status["dropped_csv_rows"], 2)
        history = json.loads(request("/api/history?since=2"))
        self.assertEqual(history["cursor"], 3)
        self.assertEqual(history["streams"]["position"][0][2], [1, 2, 0])
        self.assertEqual(history["columns"]["position"], ["x_m", "y_m", "z_deg"])

    def test_dashboard_camera_keeps_only_latest_jpeg(self):
        calls = []

        def read_image(**options):
            calls.append(options)
            if len(calls) == 1:
                return object()
            raise OSError("camera disconnected")

        fake_cv2 = SimpleNamespace(
            IMWRITE_JPEG_QUALITY=1,
            imencode=lambda extension, image, options: (
                True, SimpleNamespace(tobytes=lambda: b"jpeg-bytes")
            ),
        )
        logger = SimpleNamespace(stream_settings={}, dropped_rows=0)
        robot = SimpleNamespace(camera=SimpleNamespace(read_cv2_image=read_image))
        settings = load_config()["dashboard"].copy()
        settings["max_fps"] = 1000
        dashboard = Dashboard(robot, logger, settings)
        dashboard.running.set()
        with patch.dict("sys.modules", {"cv2": fake_cv2}):
            dashboard._camera_loop()
        self.assertEqual(dashboard.latest_jpeg, b"jpeg-bytes")
        self.assertEqual(dashboard.frame_number, 1)
        self.assertEqual(calls[0]["strategy"], "newest")
        self.assertEqual(dashboard.camera_error, "camera disconnected")

    def test_dashboard_camera_retries_empty_sdk_frame_queue(self):
        calls = []

        def read_image(**options):
            calls.append(options)
            if len(calls) == 1:
                raise Empty()
            if len(calls) == 2:
                return object()
            raise OSError("camera disconnected")

        fake_cv2 = SimpleNamespace(
            IMWRITE_JPEG_QUALITY=1,
            imencode=lambda extension, image, options: (
                True, SimpleNamespace(tobytes=lambda: b"jpeg-bytes")
            ),
        )
        logger = SimpleNamespace(stream_settings={}, dropped_rows=0)
        robot = SimpleNamespace(camera=SimpleNamespace(read_cv2_image=read_image))
        dashboard = Dashboard(robot, logger, load_config()["dashboard"])
        dashboard.running.set()
        with patch.dict("sys.modules", {"cv2": fake_cv2}):
            dashboard._camera_loop()
        self.assertEqual(dashboard.frame_number, 1)
        self.assertEqual(dashboard.camera_error, "camera disconnected")

    def test_dashboard_boxes_targets_without_changing_inspection_frame(self):
        image = np.zeros((360, 640, 3), dtype=np.uint8)
        cv2.rectangle(image, (270, 130), (370, 230), (0, 0, 255), -1)
        calls = []

        def read_image(**options):
            calls.append(options)
            if len(calls) == 1:
                return image
            raise OSError("camera disconnected")

        config = load_config()
        dashboard = Dashboard(
            SimpleNamespace(camera=SimpleNamespace(read_cv2_image=read_image)),
            SimpleNamespace(stream_settings={}, dropped_rows=0), config["dashboard"],
            target_settings=config["exploration"]["target_inspection"])
        dashboard.running.set()
        dashboard._camera_loop()

        self.assertEqual(dashboard.camera_targets[0]["color"], "red")
        self.assertEqual(dashboard.camera_targets[0]["shape"], "square")
        self.assertTrue(np.array_equal(dashboard.latest_frame, image))
        original_jpeg = cv2.imencode(".jpg", image,
                                    [cv2.IMWRITE_JPEG_QUALITY,
                                     config["dashboard"]["jpeg_quality"]])[1].tobytes()
        self.assertNotEqual(dashboard.latest_jpeg, original_jpeg)

    def test_logger_selection_and_csv(self):
        module = FakeModule()
        robot = SimpleNamespace(chassis=module)
        with tempfile.TemporaryDirectory() as temp:
            settings = {
                "directory": Path(temp),
                "streams": {
                    "position": {"enabled": True, "save": True, "frequency_hz": 10},
                    "attitude": {"enabled": True, "save": False, "frequency_hz": 10},
                },
            }
            logger = SensorLogger(robot, settings)
            logger.start()
            module.callbacks["position"]((1, 2, 0))
            module.callbacks["attitude"]((45, 0, 0))
            self.assertEqual(logger.get_latest("position"), (1, 2, 0))
            self.assertEqual(logger.get_latest("attitude"), (45, 0, 0))
            logger.exploration_state = {"status": "completed", "cell_grid": {"cells": []}}
            logger.stop()
            with (logger.run_dir / "position.csv").open(newline="") as file:
                self.assertEqual(len(list(csv.reader(file))), 2)
            with (logger.run_dir / "run_summary.json").open(encoding="utf-8") as file:
                summary = json.load(file)
            self.assertEqual(summary["received_rows"]["position"], 1)
            self.assertEqual(summary["dropped_csv_rows"], 0)
            self.assertEqual(summary["exploration"], logger.exploration_state)
            self.assertFalse((logger.run_dir / "attitude.csv").exists())
            self.assertEqual(module.callbacks, {})

    def test_full_queue_drops_csv_rows_but_keeps_latest_position(self):
        module = FakeModule()
        robot = SimpleNamespace(chassis=module)
        with tempfile.TemporaryDirectory() as temp:
            settings = {
                "directory": Path(temp),
                "queue_max_rows": 2,
                "batch_size": 1,
                "flush_interval_s": 0.1,
                "streams": {
                    "position": {"enabled": True, "save": True, "frequency_hz": 10},
                },
            }
            logger = SensorLogger(robot, settings)
            logger.start()
            for number in range(100):
                module.callbacks["position"]((number, 0, 0))
            self.assertEqual(logger.get_latest("position"), (99, 0, 0))
            logger.stop()
            self.assertGreater(logger.dropped_rows, 0)
            with (logger.run_dir / "position.csv").open(newline="") as file:
                rows = list(csv.reader(file))
            self.assertEqual(len(rows) - 1, 100 - logger.dropped_rows)

    def test_history_is_bounded_and_copies_nested_sdk_values(self):
        robot = SimpleNamespace()
        settings = {
            "directory": "unused",
            "history_max_samples": 2,
            "streams": {"esc": {"enabled": True, "save": False, "frequency_hz": 10}},
        }
        logger = SensorLogger(robot, settings)
        speeds = [1, 2, 3, 4]
        logger._callback("esc", (speeds, [0] * 4, [0] * 4, [0] * 4))
        speeds[0] = 99
        self.assertEqual(logger.get_history_since()["streams"]["esc"][0][2][0][0], 1)
        logger._callback("esc", ([5] * 4, [0] * 4, [0] * 4, [0] * 4))
        logger._callback("esc", ([6] * 4, [0] * 4, [0] * 4, [0] * 4))
        history = logger.get_history_since(1)
        self.assertEqual(history["cursor"], 3)
        self.assertEqual(len(history["streams"]["esc"]), 2)
        self.assertEqual(history["streams"]["esc"][0][2][0][0], 5)

    def test_move_to_stops_at_target_and_on_missing_data(self):
        module = FakeModule()
        robot = SimpleNamespace(chassis=module)
        with tempfile.TemporaryDirectory() as temp:
            settings = {
                "directory": Path(temp),
                "streams": {
                    "position": {"enabled": True, "save": False, "frequency_hz": 10},
                    "attitude": {"enabled": True, "save": False, "frequency_hz": 10},
                },
            }
            logger = SensorLogger(robot, settings)
            logger.start()
            motion = load_config()["motion"]
            chassis = ChassisController(robot, logger, motion)
            with self.assertRaises(TimeoutError):
                chassis.move_to(1, 0, timeout_s=0.02)
            self.assertEqual(module.commands[-1], {"x": 0, "y": 0, "z": 0})
            module.callbacks["position"]((1, 0, 0))
            module.callbacks["attitude"]((0, 0, 0))
            self.assertEqual(chassis.move_to(1, 0, yaw=0), (1, 0, 0))
            logger.stop()

    def test_move_to_pauses_for_missing_pose_and_resumes(self):
        module = FakeModule()
        motion = load_config()["motion"]
        motion["control_period_s"] = 0.001
        chassis = ChassisController(SimpleNamespace(chassis=module), None, motion)
        poses = iter([None, None, (1, 0, 0)])
        chassis.get_pose = lambda: next(poses, (1, 0, 0))

        self.assertEqual(chassis.move_to(1, 0, yaw=0, disable_timeout=True), (1, 0, 0))
        self.assertGreaterEqual(len(module.commands), 3)
        self.assertTrue(all(command == {"x": 0, "y": 0, "z": 0}
                            for command in module.commands))

    def test_move_to_pauses_when_slam_telemetry_is_unsynchronized(self):
        module = FakeModule()
        motion = load_config()["motion"]
        motion["control_period_s"] = 0.001
        chassis = ChassisController(SimpleNamespace(chassis=module), None, motion)
        chassis.get_pose = lambda: (1, 0, 0)
        pauses = iter([True, True, False])

        pose = chassis.move_to(1, 0, yaw=0, disable_timeout=True,
                               pause_if=lambda: next(pauses, False))

        self.assertEqual(pose, (1, 0, 0))
        self.assertEqual(module.commands[:2], [{"x": 0, "y": 0, "z": 0}] * 2)

    def test_move_to_sends_bounded_speed_then_times_out(self):
        module = FakeModule()
        robot = SimpleNamespace(chassis=module)
        with tempfile.TemporaryDirectory() as temp:
            settings = {
                "directory": Path(temp),
                "streams": {
                    "position": {"enabled": True, "save": False, "frequency_hz": 10},
                    "attitude": {"enabled": True, "save": False, "frequency_hz": 10},
                },
            }
            logger = SensorLogger(robot, settings)
            logger.start()
            module.callbacks["position"]((0, 0, 0))
            module.callbacks["attitude"]((90, 0, 0))
            motion = load_config()["motion"]
            motion["control_period_s"] = 0.001
            chassis = ChassisController(robot, logger, motion)
            with self.assertRaises(TimeoutError):
                chassis.move_to(1, 0, yaw=None, timeout_s=0.01)
            first = module.commands[0]
            self.assertAlmostEqual(first["x"], 0, places=4)
            self.assertLess(first["y"], 0)
            self.assertEqual(first["z"], 0)
            self.assertLessEqual(abs(first["y"]), motion["max_speed_m_s"])
            self.assertEqual(module.commands[-1], {"x": 0, "y": 0, "z": 0})
            logger.stop()

    def test_move_to_ramps_only_lateral_speed_without_delaying_the_stop(self):
        module = FakeModule()
        motion = load_config()["motion"]
        motion["control_period_s"] = .01
        motion["max_lateral_accel_m_s2"] = .5
        chassis = ChassisController(SimpleNamespace(chassis=module), None, motion)
        chassis.get_pose = lambda: (0, 0, 0)

        with self.assertRaises(TimeoutError):
            chassis.move_to(1, 1, yaw=0, timeout_s=.08)

        moving = [command for command in module.commands if command["x"] or command["y"]]
        self.assertGreater(len(moving), 3)
        self.assertGreater(moving[0]["x"], .2)
        self.assertLess(abs(moving[0]["y"]), .01)
        self.assertGreater(abs(moving[-1]["y"]), abs(moving[0]["y"]))
        for old, new in zip(moving, moving[1:]):
            step = abs(new["y"] - old["y"])
            self.assertLessEqual(step, motion["max_lateral_accel_m_s2"] * motion["control_period_s"] + 1e-6)
        self.assertEqual(module.commands[-1], {"x": 0, "y": 0, "z": 0})

    def test_forward_and_backward_speed_are_not_ramped(self):
        motion = load_config()["motion"]
        motion["control_period_s"] = .001
        for destination, expected in ((1, motion["max_speed_m_s"]),
                                      (-1, -motion["max_speed_m_s"])):
            module = FakeModule()
            chassis = ChassisController(SimpleNamespace(chassis=module), None, motion)
            chassis.get_pose = lambda: (0, 0, 0)
            with self.assertRaises(TimeoutError):
                chassis.move_to(destination, 0, yaw=0, timeout_s=.01)
            self.assertAlmostEqual(module.commands[0]["x"], expected)
            self.assertEqual(module.commands[0]["y"], 0)

    def test_move_to_corrects_heading_drift_without_new_turn_target(self):
        module = FakeModule()
        motion = load_config()["motion"]
        motion["control_period_s"] = 0.001
        chassis = ChassisController(SimpleNamespace(chassis=module), None, motion)
        poses = iter([(0, 0, 0), (0, 0, 7)])
        chassis.get_pose = lambda: next(poses, (0, 0, 7))

        with self.assertRaises(TimeoutError):
            chassis.move_to(1, 0, timeout_s=0.02)

        self.assertEqual(module.commands[0]["z"], 0)
        self.assertTrue(any(command["z"] < 0 for command in module.commands[1:]))
        self.assertEqual(module.commands[-1], {"x": 0, "y": 0, "z": 0})

        module.commands.clear()
        chassis.get_pose = lambda: (0, 0, 7)
        with self.assertRaises(TimeoutError):
            chassis.move_to(1, 0, timeout_s=0.01)
        self.assertLess(module.commands[0]["z"], 0)  # The original heading survives a new waypoint.

        module.commands.clear()
        chassis.reset_heading()
        with self.assertRaises(TimeoutError):
            chassis.move_to(1, 0, timeout_s=0.01)
        self.assertEqual(module.commands[0]["z"], 0)

    def test_move_to_corrects_yaw_inside_arrival_tolerance_while_sliding(self):
        module = FakeModule()
        motion = load_config()["motion"]
        motion["control_period_s"] = 0.001
        chassis = ChassisController(SimpleNamespace(chassis=module), None, motion)
        chassis.get_pose = lambda: (0, 0, 0.5)

        with self.assertRaises(TimeoutError):
            chassis.move_to(0, 1, yaw=0, timeout_s=0.01)

        moving = [command for command in module.commands if command["y"] > 0]
        self.assertTrue(moving)
        self.assertTrue(all(command["z"] < 0 for command in moving))
        self.assertEqual(module.commands[-1], {"x": 0, "y": 0, "z": 0})

    def test_gimbal_heading_tracks_rotation_missing_from_chassis_attitude(self):
        motion = load_config()["motion"]
        motion["heading_source"] = "gimbal"
        samples = {"position": (0, 0, 0), "attitude": (0, 0, 0),
                   "gimbal": (0, 0, 0, 0)}
        logger = SimpleNamespace(
            get_latest=lambda name, max_age_s=None: samples.get(name),
            get_sample=lambda name, max_age_s=None: (samples[name], 1.0)
            if name in samples else None,
        )
        chassis = ChassisController(SimpleNamespace(chassis=FakeModule()), logger, motion)
        self.assertAlmostEqual(chassis.get_pose()[2], 0)

        # The relative and ground gimbal angles change by different amounts
        # when the chassis turns; attitude incorrectly remains at zero.
        samples["gimbal"] = (0, -90, 0, -85)
        self.assertAlmostEqual(chassis.get_pose()[2], 5)
        motion["control_period_s"] = .001
        with self.assertRaises(TimeoutError):
            chassis.move_to(1, 0, yaw=0, timeout_s=.01)
        self.assertTrue(any(command["z"] < 0 for command in chassis.chassis.commands))
        samples.pop("gimbal")
        self.assertIsNone(chassis.get_pose())

    def test_move_to_can_wait_for_dfs_arrival_without_deadline(self):
        module = FakeModule()
        motion = load_config()["motion"]
        motion["timeout_s"] = 0.005
        motion["control_period_s"] = 0.001
        chassis = ChassisController(SimpleNamespace(chassis=module), None, motion)
        poses = iter([(0, 0, 0)] * 30 + [(1, 0, 0)])
        chassis.get_pose = lambda: next(poses, (1, 0, 0))

        self.assertEqual(chassis.move_to(1, 0, disable_timeout=True), (1, 0, 0))
        self.assertGreaterEqual(len(module.commands), 31)
        self.assertEqual(module.commands[-1], {"x": 0, "y": 0, "z": 0})

    def test_move_to_stop_condition_returns_current_pose_and_stops_wheels(self):
        module = FakeModule()
        motion = load_config()["motion"]
        motion["control_period_s"] = .001
        chassis = ChassisController(SimpleNamespace(chassis=module), None, motion)
        poses = iter([(0, 0, 0), (.03, 0, 0)])
        chassis.get_pose = lambda: next(poses, (.03, 0, 0))

        pose = chassis.move_to(1, 0, yaw=0, timeout_s=.1,
                               stop_if=lambda current: current[0] >= .03)

        self.assertEqual(pose, (.03, 0, 0))
        self.assertGreater(module.commands[0]["x"], 0)
        self.assertEqual(module.commands[-1], {"x": 0, "y": 0, "z": 0})


if __name__ == "__main__":
    unittest.main()
