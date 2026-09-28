"""Rear IR routing and motion interlock without a physical robot."""

import math
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from src.chassis import ChassisController
from src.config_loader import load_config
from src.logger import SensorLogger
from src.mission_stop import MissionStop
from src.rear_ir import RearIRBumper, adapter_index, recovery_vector


class FakeAdapter:
    def __init__(self):
        self.callback = None
        self.commands = []

    def sub_adapter(self, **options):
        self.callback = options["callback"]
        return True

    def unsub_adapter(self):
        self.callback = None

    def drive_speed(self, **command):
        self.commands.append(command)


class RearIRTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        # Exercise active-low signals in fake callbacks on both sides.
        for side in ("right", "left"):
            self.config["rear_ir"][side]["active_io"] = 0
        self.adapter = FakeAdapter()
        self.robot = SimpleNamespace(sensor_adaptor=self.adapter, chassis=self.adapter)
        self.temp = tempfile.TemporaryDirectory()
        settings = {"directory": Path(self.temp.name), "streams": {
            "adapter": {"enabled": True, "save": False, "frequency_hz": 20}}}
        self.logger = SensorLogger(self.robot, settings)
        self.logger.start()
        self.bumper = RearIRBumper(self.logger, self.config["rear_ir"])

    def tearDown(self):
        self.logger.stop()
        self.temp.cleanup()

    def send_io(self, right, left):
        io = [1] * 12
        for side, value in (("right", right), ("left", left)):
            port = self.config["rear_ir"][side]
            io[adapter_index(port["id"], port["port"])] = value
        self.adapter.callback((io, [0] * 12))

    def test_id_port_mapping_and_side_selectivity(self):
        self.assertEqual((adapter_index(3, 1), adapter_index(4, 1)), (4, 6))
        self.send_io(0, 1)
        self.assertTrue(self.bumper.snapshot()["sides"]["right"]["detected"])
        self.assertFalse(self.bumper.snapshot()["sides"]["left"]["detected"])
        self.assertTrue(self.bumper.blocks_motion(-0.1, 0, 0))
        self.assertTrue(self.bumper.blocks_motion(0, 0.1, 0))
        self.assertFalse(self.bumper.blocks_motion(0, -0.1, 0))
        self.assertFalse(self.bumper.blocks_motion(0.1, 0, 0))
        self.send_io(1, 0)
        self.assertTrue(self.bumper.blocks_motion(0, -0.1, 0))
        self.assertFalse(self.bumper.blocks_motion(0, 0.1, 0))

    def test_configured_polarity_is_independent_per_side(self):
        settings = self.config["rear_ir"].copy()
        settings["right"] = dict(settings["right"], active_io=1)
        settings["left"] = dict(settings["left"], active_io=0)
        bumper = RearIRBumper(self.logger, settings)
        self.send_io(1, 0)
        self.assertTrue(bumper.snapshot()["sides"]["right"]["detected"])
        self.assertTrue(bumper.snapshot()["sides"]["left"]["detected"])
        self.send_io(0, 1)
        self.assertFalse(bumper.snapshot()["sides"]["right"]["detected"])
        self.assertFalse(bumper.snapshot()["sides"]["left"]["detected"])

    def test_recovery_vector_moves_away_from_each_detected_side(self):
        def sides(right, left):
            return {"right": {"detected": right}, "left": {"detected": left}}

        self.assertEqual(recovery_vector(sides(False, True), .08),
                         ("slide_right", 0.0, .08))
        self.assertEqual(recovery_vector(sides(True, False), .08),
                         ("slide_left", 0.0, -.08))
        self.assertEqual(recovery_vector(sides(True, True), .08),
                         ("forward", .08, 0.0))
        self.assertEqual(recovery_vector(sides(False, False), .08),
                         ("clear", 0.0, 0.0))
        self.assertEqual(recovery_vector(sides(False, True), .08, attempt=2),
                         ("forward", .08, 0.0))
        direction, x_speed, y_speed = recovery_vector(
            sides(False, True), .08, attempt=3)
        self.assertEqual(direction, "front_right")
        self.assertAlmostEqual(math.hypot(x_speed, y_speed), .08)

    def test_missing_adapter_data_blocks_reverse_and_rotation(self):
        self.assertTrue(self.bumper.blocks_motion(-0.1, 0, 0))
        self.assertEqual(self.bumper.last_block, "waiting_data")
        self.assertTrue(self.bumper.blocks_motion(0, 0, 10))
        self.assertFalse(self.bumper.blocks_motion(0.1, 0, 0))
        self.send_io(1, 1)
        with self.logger.lock:
            values, _ = self.logger.latest["adapter"]
            self.logger.latest["adapter"] = (values, time.time() - 2)
        self.assertTrue(self.bumper.blocks_motion(-0.1, 0, 0))
        self.assertEqual(self.bumper.snapshot()["state"], "waiting_data")

    def test_detected_side_takes_priority_over_missing_other_side(self):
        self.send_io(2, 0)
        self.assertTrue(self.bumper.blocks_motion(-0.1, 0, 0))
        self.assertEqual(self.bumper.last_block, "left")

    def test_controller_limits_recovery_and_escapes_right_sensor_to_the_left(self):
        motion = self.config["motion"].copy()
        motion["control_period_s"] = 0.001
        chassis = ChassisController(self.robot, self.logger, motion)
        chassis.rear_ir = self.bumper
        chassis.get_pose = lambda: (0, 0, 0)
        self.send_io(0, 1)
        with self.assertRaisesRegex(MissionStop, "IR recovery exceeded waypoint time"):
            chassis.move_to(-1, 0, yaw=0, timeout_s=0.01)
        self.assertTrue(any(command["y"] < 0 for command in self.adapter.commands))
        self.assertFalse(any(command["x"] < 0 or command["y"] > 0
                             for command in self.adapter.commands))

        self.adapter.commands.clear()
        with self.assertRaises(TimeoutError):
            chassis.move_to(1, 0, yaw=0, timeout_s=0.01)
        self.assertTrue(any(command["x"] > 0 for command in self.adapter.commands))

    def test_recovery_clears_then_resumes_original_target(self):
        motion = self.config["motion"].copy()
        motion["control_period_s"] = 0.001
        settings = self.config["rear_ir"].copy()
        settings["recovery_clear_samples"] = 2
        bumper = RearIRBumper(self.logger, settings)
        chassis = ChassisController(self.robot, self.logger, motion)
        chassis.rear_ir = bumper
        pose = [0.0, 0.0, 0.0]
        chassis.get_pose = lambda: tuple(pose)
        self.send_io(0, 1)
        original_drive = self.adapter.drive_speed
        original_get_sample = self.logger.get_sample
        recovery_steps = [0]

        def fresh_sample(name, max_age_s=None):
            if name == "adapter":
                self.send_io(1 if recovery_steps[0] >= 2 else 0, 1)
            return original_get_sample(name, max_age_s=max_age_s)

        def drive(**command):
            original_drive(**command)
            if command["y"] < 0:
                recovery_steps[0] += 1
                pose[1] -= 0.01

        self.logger.get_sample = fresh_sample
        self.adapter.drive_speed = drive
        with self.assertRaises(TimeoutError):
            chassis.move_to(-1, 0, yaw=0, timeout_s=0.2)
        self.assertTrue(any(command["y"] < 0 for command in self.adapter.commands))
        self.assertTrue(any(command["x"] < 0 for command in self.adapter.commands))
        self.assertEqual(bumper.events[0]["status"], "cleared")

    def test_both_rear_sensors_recover_forward_until_both_clear(self):
        motion = self.config["motion"].copy()
        motion["control_period_s"] = 0.001
        chassis = ChassisController(self.robot, self.logger, motion)
        chassis.rear_ir = self.bumper
        pose = [0.0, 0.0, 0.0]
        chassis.get_pose = lambda: tuple(pose)
        self.send_io(0, 0)
        self.assertTrue(self.bumper.blocks_motion(-0.1, 0, 0))
        self.assertEqual(self.bumper.last_block, "both")
        original_get_sample = self.logger.get_sample
        original_drive = self.adapter.drive_speed
        steps = [0]

        def fresh_sample(name, max_age_s=None):
            if name == "adapter":
                self.send_io(1 if steps[0] >= 2 else 0,
                             1 if steps[0] >= 3 else 0)
            return original_get_sample(name, max_age_s=max_age_s)

        def drive(**command):
            original_drive(**command)
            if command["x"] > 0 or command["y"] != 0:
                steps[0] += 1
                pose[0] += command["x"] * .05
                pose[1] += command["y"] * .05

        self.logger.get_sample = fresh_sample
        self.adapter.drive_speed = drive
        with self.assertRaises(TimeoutError):
            chassis.move_to(-1, 0, yaw=0, timeout_s=0.2)
        self.assertGreaterEqual(steps[0], 3)
        self.assertEqual(self.bumper.events[0]["status"], "cleared")
        self.assertTrue(any(command["x"] > 0 for command in self.adapter.commands))
        self.assertTrue(any(command["y"] > 0 for command in self.adapter.commands))
        self.assertTrue(any(command["x"] < 0 for command in self.adapter.commands))
        self.assertTrue(all(command["z"] == 0 for command in self.adapter.commands))

    def test_recovery_retries_three_times_then_stops_without_clear_signal(self):
        motion = self.config["motion"].copy()
        motion["control_period_s"] = 0.001
        settings = self.config["rear_ir"].copy()
        settings["recovery_max_m"] = 0.09
        bumper = RearIRBumper(self.logger, settings)
        chassis = ChassisController(self.robot, self.logger, motion)
        chassis.rear_ir = bumper
        pose = [0.0, 0.0, 0.0]
        chassis.get_pose = lambda: tuple(pose)
        self.send_io(0, 1)
        original_drive = self.adapter.drive_speed
        original_get_sample = self.logger.get_sample

        def fresh_sample(name, max_age_s=None):
            if name == "adapter":
                self.send_io(0, 1)
            return original_get_sample(name, max_age_s=max_age_s)

        def drive(**command):
            original_drive(**command)
            if command["y"] < 0:
                pose[1] -= 0.05

        self.logger.get_sample = fresh_sample
        self.adapter.drive_speed = drive
        with self.assertRaisesRegex(MissionStop, "after 3 recovery attempts"):
            chassis.move_to(-1, 0, yaw=0, disable_timeout=True)
        self.assertLessEqual(abs(pose[1]), 0.31)
        self.assertEqual(len(bumper.events), 3)
        self.assertTrue(all(event["status"] == "limit" for event in bumper.events))


if __name__ == "__main__":
    unittest.main()
