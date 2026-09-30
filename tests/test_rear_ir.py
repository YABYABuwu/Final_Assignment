"""Rear IR routing and motion interlock without a physical robot."""

import math
import threading
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import yaml

from src.chassis import ChassisController
from src.config_loader import load_config
from src.logger import SensorLogger
from src.mission_stop import MissionStop
from src.rear_ir import FrontIRBumper, RearIRBumper, adapter_index, recovery_vector


class FakeAdapter:
    def __init__(self):
        self.callback = None
        self.commands = []
        self.direct_io = {}

    def sub_adapter(self, **options):
        self.callback = options["callback"]
        return True

    def unsub_adapter(self):
        self.callback = None

    def drive_speed(self, **command):
        self.commands.append(command)

    def get_io(self, id, port):
        return self.direct_io.get((id, port), 1)


class RearIRTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        # These tests inject adapter callbacks, so use stream IO and fixed
        # polarity instead of the hardware-only direct read and calibration.
        for end in ("rear_ir", "front_ir"):
            self.config[end]["io_read_mode"] = "stream"
            self.config[end]["auto_calibrate_io"] = False
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

    def test_id_port_mapping_and_slide_checks_any_detected_side(self):
        self.assertEqual((adapter_index(3, 1), adapter_index(4, 1)), (4, 6))
        self.send_io(0, 1)
        self.assertTrue(self.bumper.snapshot()["sides"]["right"]["detected"])
        self.assertFalse(self.bumper.snapshot()["sides"]["left"]["detected"])
        self.assertTrue(self.bumper.blocks_motion(-0.1, 0, 0))
        self.assertTrue(self.bumper.blocks_motion(0, 0.1, 0))
        self.assertTrue(self.bumper.blocks_motion(0, -0.1, 0))
        self.assertFalse(self.bumper.blocks_motion(0.1, 0, 0))
        self.send_io(1, 0)
        self.assertTrue(self.bumper.blocks_motion(0, -0.1, 0))
        self.assertTrue(self.bumper.blocks_motion(0, 0.1, 0))

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

    def test_auto_mode_falls_back_to_direct_io_for_invalid_dds_stream(self):
        settings = dict(self.config["rear_ir"], io_read_mode="auto")
        bumper = RearIRBumper(self.logger, settings)
        self.adapter.callback(([0] * 12, [100] * 12))

        state = bumper.snapshot()

        self.assertEqual(state["io_source"], "direct_fallback")
        self.assertFalse(state["sides"]["right"]["detected"])
        self.assertFalse(state["sides"]["left"]["detected"])

    def test_auto_mode_confirms_partial_low_stream_with_direct_io(self):
        settings = dict(self.config["rear_ir"], io_read_mode="auto")
        bumper = RearIRBumper(self.logger, settings)
        partial_stream = [0] * 12
        partial_stream[0] = 1
        self.adapter.callback((partial_stream, [500] * 12))

        state = bumper.snapshot()

        self.assertEqual(state["io_source"], "direct_fallback")
        self.assertFalse(state["sides"]["right"]["detected"])
        self.assertFalse(state["sides"]["left"]["detected"])

    def test_default_config_enables_fallback_and_keeps_calibration_off(self):
        config = load_config()
        for name in ("front_ir", "rear_ir"):
            self.assertEqual(config[name]["io_read_mode"], "auto")
            self.assertFalse(config[name]["auto_calibrate_io"])

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

    def test_cardinal_escape_follows_current_travel_axis(self):
        left = {"right": {"detected": False}, "left": {"detected": True}}
        right = {"right": {"detected": True}, "left": {"detected": False}}
        both = {"right": {"detected": True}, "left": {"detected": True}}
        for end in ("front", "rear"):
            expected_longitudinal = (
                ("backward", -.08, 0.0) if end == "front" else
                ("forward", .08, 0.0))
            self.assertEqual(
                recovery_vector(left, .08, 1, end, "cardinal",
                                movement_axis="longitudinal"),
                ("slide_right", 0.0, .08))
            self.assertEqual(
                recovery_vector(right, .08, 1, end, "cardinal",
                                movement_axis="longitudinal"),
                ("slide_left", 0.0, -.08))
            for sides in (left, right):
                self.assertEqual(
                    recovery_vector(sides, .08, 2, end, "cardinal",
                                    movement_axis="longitudinal"),
                    expected_longitudinal)
                self.assertEqual(
                    recovery_vector(sides, .08, 1, end, "cardinal",
                                    movement_axis="lateral"),
                    expected_longitudinal)
            self.assertEqual(
                recovery_vector(left, .08, 2, end, "cardinal",
                                movement_axis="lateral"),
                ("slide_right", 0.0, .08))
            self.assertEqual(
                recovery_vector(right, .08, 2, end, "cardinal",
                                movement_axis="lateral"),
                ("slide_left", 0.0, -.08))
            for attempt in (1, 2):
                self.assertEqual(
                    recovery_vector(both, .08, attempt, end, "cardinal",
                                    movement_axis="longitudinal"),
                    expected_longitudinal)
            self.assertEqual(
                recovery_vector(left, .08, 2, end, "cardinal",
                                forward_clear=False,
                                movement_axis="longitudinal"),
                ("blocked", 0.0, 0.0))

    def test_slide_checks_both_sensors_at_each_end(self):
        io = [1] * 12
        logger = SimpleNamespace(get_sample=lambda name, max_age_s=None:
                                 (tuple(io), time.time()))
        bumpers = {}
        for end, bumper_type in (("front", FrontIRBumper), ("rear", RearIRBumper)):
            settings = dict(self.config[end + "_ir"], auto_calibrate_io=False,
                            io_read_mode="stream")
            settings["left"] = dict(settings["left"], active_io=0)
            settings["right"] = dict(settings["right"], active_io=0)
            bumpers[end] = bumper_type(logger, settings)
        front_left = bumpers["front"].settings["left"]
        io[adapter_index(front_left["id"], front_left["port"])] = 0
        self.assertTrue(bumpers["front"].blocks_motion(0, .1, 0))
        self.assertEqual(bumpers["front"].last_block, "left")
        self.assertFalse(bumpers["rear"].blocks_motion(0, .1, 0))
        io[adapter_index(front_left["id"], front_left["port"])] = 1
        rear_right = bumpers["rear"].settings["right"]
        io[adapter_index(rear_right["id"], rear_right["port"])] = 0
        self.assertTrue(bumpers["rear"].blocks_motion(0, -.1, 0))
        self.assertEqual(bumpers["rear"].last_block, "right")
        self.assertFalse(bumpers["front"].blocks_motion(0, -.1, 0))

    def test_chassis_selects_escape_axis_from_motion(self):
        motion = dict(self.config["motion"], control_period_s=0.001)
        chassis = ChassisController(self.robot, self.logger, motion)
        chassis.get_pose = lambda: (0.0, 0.0, 0.0)
        bumper = SimpleNamespace(end="front", last_block="left",
                                 settings={},
                                 blocks_motion=lambda x, y, z: True)
        chassis.front_ir = bumper
        selected_axes = []

        def capture_recovery(*args, **kwargs):
            selected_axes.append(kwargs["movement_axis"])
            raise MissionStop("axis captured")

        chassis._recover_from_ir = capture_recovery
        for target, axis in (((0.0, 1.0), "lateral"),
                             ((1.0, 0.0), "longitudinal")):
            with self.assertRaisesRegex(MissionStop, "axis captured"):
                chassis.move_to(*target, yaw=0.0, timeout_s=0.1)
            self.assertEqual(selected_axes[-1], axis)

    def test_slide_recovery_waits_for_opposite_ir_data(self):
        chassis = ChassisController(self.robot, self.logger, self.config["motion"])
        chassis.get_pose = lambda: (0.0, 0.0, 0.0)
        front_checks = [0]
        front = SimpleNamespace(end="front", last_block=None)

        def front_blocks(x, y, z):
            front_checks[0] += 1
            front.last_block = "waiting_data" if front_checks[0] == 1 else None
            return front_checks[0] == 1

        front.blocks_motion = front_blocks
        chassis.front_ir = front
        original_get_sample = self.logger.get_sample

        def get_sample(name, max_age_s=None):
            if name == "tof":
                return (300,), time.time()
            return original_get_sample(name, max_age_s=max_age_s)

        self.logger.get_sample = get_sample
        samples = [0]
        settings = dict(self.config["rear_ir"], recovery_clear_samples=2)
        rear = SimpleNamespace(end="rear", settings=settings, recovering=None,
                               last_block="right", finish_recovery=lambda *args: None)

        def snapshot():
            samples[0] += 1
            detected = samples[0] <= 2
            return {"sample_time": time.time() + samples[0],
                    "sides": {"right": {"detected": detected},
                              "left": {"detected": False}}}

        rear.snapshot = snapshot
        self.assertTrue(chassis._recover_from_ir(
            rear, "right", 1, 0.001, None, None, None, movement_axis="lateral"))
        self.assertGreaterEqual(front_checks[0], 2)
        self.assertTrue(any(command["x"] > 0 for command in self.adapter.commands))

    def test_diagonal_recovery_moves_forward_and_away(self):
        def sides(right, left):
            return {"right": {"detected": right}, "left": {"detected": left}}

        # Rear left detected -> forward and right (both moves forward and away from left)
        direction, vx, vy = recovery_vector(sides(False, True), .08, end="rear", mode="diagonal", forward_clear=True)
        self.assertEqual(direction, "front_right")
        self.assertGreater(vx, 0)
        self.assertGreater(vy, 0)

        # Rear right detected -> forward and left (both moves forward and away from right)
        direction, vx, vy = recovery_vector(sides(True, False), .08, end="rear", mode="diagonal", forward_clear=True)
        self.assertEqual(direction, "front_left")
        self.assertGreater(vx, 0)
        self.assertLess(vy, 0)

        # Both sides detected -> forward straight
        direction, vx, vy = recovery_vector(sides(True, True), .08, end="rear", mode="diagonal", forward_clear=True)
        self.assertEqual(direction, "forward")
        self.assertAlmostEqual(vx, .08)
        self.assertEqual(vy, 0.0)

        # When forward is blocked (e.g. wall in front), falls back to pure lateral slide
        direction, vx, vy = recovery_vector(sides(False, True), .08, end="rear", mode="diagonal", forward_clear=False)
        self.assertEqual(direction, "slide_right")
        self.assertEqual(vx, 0.0)
        self.assertGreater(vy, 0)

        direction, vx, vy = recovery_vector(sides(True, False), .08, end="rear", mode="diagonal", forward_clear=False)
        self.assertEqual(direction, "slide_left")
        self.assertEqual(vx, 0.0)
        self.assertLess(vy, 0)

    def test_forward_first_recovery_mode(self):
        def sides(right, left):
            return {"right": {"detected": right}, "left": {"detected": left}}

        # Attempt 1: straight forward
        direction, vx, vy = recovery_vector(sides(False, True), .08, attempt=1, end="rear", mode="forward_first", forward_clear=True)
        self.assertEqual(direction, "forward")
        self.assertAlmostEqual(vx, .08)
        self.assertEqual(vy, 0.0)

        # Attempt 2: diagonal escape
        direction, vx, vy = recovery_vector(sides(False, True), .08, attempt=2, end="rear", mode="forward_first", forward_clear=True)
        self.assertEqual(direction, "front_right")
        self.assertGreater(vx, 0)
        self.assertGreater(vy, 0)

    def test_front_ir_blocks_forward_and_recovery_moves_away(self):
        settings = self.config["front_ir"].copy()
        settings["right"] = {"id": 3, "port": 1, "active_io": 1}
        settings["left"] = dict(settings["left"], active_io=0)
        front = FrontIRBumper(self.logger, settings)
        self.assertTrue(front.blocks_motion(.1, 0, 0))
        self.assertEqual(front.last_block, "waiting_data")
        io = [1] * 12
        self.adapter.callback((io, [0] * 12))
        self.assertTrue(front.blocks_motion(.1, 0, 0))
        self.assertEqual(front.last_block, "right")
        self.assertFalse(front.blocks_motion(-.1, 0, 0))
        self.assertEqual(recovery_vector(front.snapshot()["sides"], .08, end="front"),
                         ("slide_left", 0.0, -.08))
        self.assertEqual(recovery_vector(front.snapshot()["sides"], .08, attempt=2,
                                         end="front"), ("backward", -.08, 0.0))

    def test_front_recovery_stops_when_rear_ir_blocks_escape(self):
        settings = self.config["front_ir"].copy()
        settings["right"] = {"id": 3, "port": 1, "active_io": 1}
        settings["left"] = dict(settings["left"], active_io=0)
        front = FrontIRBumper(self.logger, settings)
        motion = self.config["motion"].copy()
        motion["control_period_s"] = 0.001
        chassis = ChassisController(self.robot, self.logger, motion)
        chassis.front_ir = front
        chassis.rear_ir = self.bumper
        chassis.get_pose = lambda: (0, 0, 0)
        io = [1] * 12
        io[adapter_index(4, 1)] = 0
        self.adapter.callback((io, [0] * 12))
        with self.assertRaisesRegex(MissionStop, "recovery exhausted after 8 attempts"):
            chassis.move_to(1, 0, yaw=0, timeout_s=.3)
        self.assertFalse(any(command["x"] != 0 or command["y"] != 0
                             for command in self.adapter.commands))
        self.assertEqual([event["status"] for event in front.events],
                         ["blocked"] * 8)

    def test_config_rejects_same_adapter_port_for_front_and_rear(self):
        settings = self.config.copy()
        settings["front_ir"] = dict(settings["front_ir"], enabled=True,
                                    right=dict(settings["front_ir"]["right"], active_io=1),
                                    left=dict(settings["front_ir"]["left"], active_io=0))
        path = Path(self.temp.name) / "duplicate.yaml"
        path.write_text(yaml.safe_dump(settings), encoding="utf-8")
        self.assertTrue(load_config(path)["front_ir"]["enabled"])
        settings["rear_ir"] = dict(settings["rear_ir"],
                                   right=dict(settings["rear_ir"]["right"],
                                              id=settings["front_ir"]["right"]["id"],
                                              port=settings["front_ir"]["right"]["port"]))
        path.write_text(yaml.safe_dump(settings), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "shares an adapter port"):
            load_config(path)

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

    def test_rotation_is_not_exempt_while_translating_away(self):
        self.send_io(0, 1)
        self.assertTrue(self.bumper.blocks_motion(0.1, 0, 10))
        self.assertEqual(self.bumper.last_block, "right")

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

    def test_recovery_distance_limit_advances_to_the_next_attempt(self):
        chassis = ChassisController(self.robot, self.logger, self.config["motion"])
        chassis.get_pose = lambda: (0.0, 0.0, 0.0)
        events = []
        settings = dict(self.config["rear_ir"], recovery_max_m=0.0001)
        bumper = SimpleNamespace(
            end="rear", settings=settings, recovering=None, last_block="right",
            snapshot=lambda: {
                "sample_time": time.time(),
                "sides": {"right": {"detected": True},
                          "left": {"detected": False}},
            },
            finish_recovery=lambda *args: events.append(args),
        )

        recovered = chassis._recover_from_ir(
            bumper, "right", 1, 0.001, None, None, None)

        self.assertFalse(recovered)
        self.assertEqual(events[0][1], "limit")

    def test_missing_tof_stops_and_waits_before_forward_recovery(self):
        chassis = ChassisController(self.robot, self.logger, self.config["motion"])
        chassis.get_pose = lambda: (0.0, 0.0, 0.0)
        chassis.front_ir = SimpleNamespace(
            end="front", last_block=None,
            blocks_motion=lambda x, y, z: False)
        aborted = threading.Event()
        snapshots = [0]
        settings = dict(self.config["rear_ir"], recovery_max_m=1.0)

        def snapshot():
            snapshots[0] += 1
            if snapshots[0] >= 3:
                aborted.set()
            return {
                "sample_time": time.time() + snapshots[0],
                "sides": {"right": {"detected": True},
                          "left": {"detected": False}},
            }

        bumper = SimpleNamespace(
            end="rear", settings=settings, recovering=None, last_block="right",
            snapshot=snapshot, finish_recovery=lambda *args: None)

        with self.assertRaisesRegex(MissionStop, "telemetry failed"):
            chassis._recover_from_ir(
                bumper, "right", 1, 0.001, None, aborted, None,
                movement_axis="lateral")

        self.assertFalse(any(command["x"] > 0 for command in self.adapter.commands))

    def test_forward_recovery_requires_valid_tof_at_threshold(self):
        chassis = ChassisController(self.robot, self.logger, self.config["motion"])
        chassis.front_ir = SimpleNamespace(
            end="front", last_block=None,
            blocks_motion=lambda x, y, z: False)
        bumper = SimpleNamespace(end="rear", settings=self.config["rear_ir"])
        readings = iter((None, 0, 65535, float("nan"), 249, 250))

        def get_sample(name, max_age_s=None):
            value = next(readings)
            return None if value is None else ((value,), time.time())

        self.logger.get_sample = get_sample
        expected = (None, None, None, None, False, True)
        actual = tuple(
            chassis._longitudinal_ir_escape_clearance(bumper, 0.08)[0]
            for _ in expected)
        self.assertEqual(actual, expected)

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
            if name == "tof":
                return (300,), time.time()
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

    def test_failed_recovery_tries_two_directions_before_stopping(self):
        motion = dict(self.config["motion"], control_period_s=0.001)
        chassis = ChassisController(self.robot, self.logger, motion)
        chassis.get_pose = lambda: (0.0, 0.0, 0.0)
        bumper = SimpleNamespace(end="rear", last_block="right",
                                 settings={"recovery_max_attempts": 2},
                                 blocks_motion=lambda x, y, z: True)
        chassis.rear_ir = bumper
        attempts = []

        def recover(*args, **kwargs):
            attempts.append(args[2])
            return False

        chassis._recover_from_ir = recover
        with self.assertRaisesRegex(MissionStop, "exhausted after 2 attempts"):
            chassis.move_to(-1, 0, yaw=0, disable_timeout=True)
        self.assertEqual(attempts, [1, 2])

    def test_left_obstacle_on_both_ends_allows_lateral_right_recovery(self):
        settings_f = dict(self.config["front_ir"], auto_calibrate_io=False, io_read_mode="stream")
        settings_f["left"] = dict(settings_f["left"], active_io=0)
        settings_f["right"] = dict(settings_f["right"], active_io=0)
        front = FrontIRBumper(self.logger, settings_f)

        settings_r = dict(self.config["rear_ir"], auto_calibrate_io=False, io_read_mode="stream",
                          recovery_clear_samples=2)
        settings_r["left"] = dict(settings_r["left"], active_io=0)
        settings_r["right"] = dict(settings_r["right"], active_io=0)
        rear = RearIRBumper(self.logger, settings_r)

        motion = dict(self.config["motion"], control_period_s=0.001)
        chassis = ChassisController(self.robot, self.logger, motion)
        chassis.front_ir = front
        chassis.rear_ir = rear
        chassis.get_pose = lambda: (0.0, 0.0, 0.0)

        # Both Front-Left and Rear-Left detect obstacle (0)
        io = [1] * 12
        io[adapter_index(settings_f["left"]["id"], settings_f["left"]["port"])] = 0
        io[adapter_index(settings_r["left"]["id"], settings_r["left"]["port"])] = 0
        self.adapter.callback((io, [0] * 12))

        # Check that rear does not block front from escaping to the right
        opp_blocked, opp_side = chassis._opposite_blocks_recovery_path(rear, 0.0, 0.08, "front")
        self.assertFalse(opp_blocked)

        # Drive speed commands during recovery slide to the right
        steps = [0]
        original_get_sample = self.logger.get_sample

        def callback_sample(name, max_age_s=None):
            if name == "adapter":
                steps[0] += 1
                # After 2 recovery steps, clear both obstacles
                if steps[0] >= 3:
                    clear_io = [1] * 12
                    self.adapter.callback((clear_io, [0] * 12))
            return original_get_sample(name, max_age_s=max_age_s)

        self.logger.get_sample = callback_sample
        recovered = chassis._recover_from_ir(front, "left", 1, 0.001, None, None, None,
                                             movement_axis="longitudinal")
        self.assertTrue(recovered)
        self.assertTrue(any(command["y"] > 0 for command in self.adapter.commands))

    def test_cross_conflict_lateral_recovery_is_blocked_by_opposite(self):
        settings_f = dict(self.config["front_ir"], auto_calibrate_io=False, io_read_mode="stream")
        settings_f["left"] = dict(settings_f["left"], active_io=0)
        settings_f["right"] = dict(settings_f["right"], active_io=0)
        front = FrontIRBumper(self.logger, settings_f)

        settings_r = dict(self.config["rear_ir"], auto_calibrate_io=False, io_read_mode="stream")
        settings_r["left"] = dict(settings_r["left"], active_io=0)
        settings_r["right"] = dict(settings_r["right"], active_io=0)
        rear = RearIRBumper(self.logger, settings_r)

        motion = dict(self.config["motion"], control_period_s=0.001)
        chassis = ChassisController(self.robot, self.logger, motion)
        chassis.front_ir = front
        chassis.rear_ir = rear
        chassis.get_pose = lambda: (0.0, 0.0, 0.0)

        # Front-Left (wants to slide right) but Rear-Right is blocked!
        io = [1] * 12
        io[adapter_index(settings_f["left"]["id"], settings_f["left"]["port"])] = 0
        io[adapter_index(settings_r["right"]["id"], settings_r["right"]["port"])] = 0
        self.adapter.callback((io, [0] * 12))

        opp_blocked, opp_side = chassis._opposite_blocks_recovery_path(rear, 0.0, 0.08, "front")
        self.assertTrue(opp_blocked)
        self.assertEqual(opp_side, "right")


if __name__ == "__main__":
    unittest.main()
