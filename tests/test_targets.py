import copy
import time
import unittest
from types import SimpleNamespace

import cv2
import numpy as np

from src.config_loader import load_config
from src.mission_stop import MissionStop
from src.target_inspection import WallTargetInspector
from src.targets import TargetTracker, detect


def red_square(center=(320, 180), size=90):
    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    x, y = center
    cv2.rectangle(frame, (x - size // 2, y - size // 2),
                  (x + size // 2, y + size // 2), (0, 0, 255), -1)
    return frame


def two_targets(yaw=0):
    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    shift = round(yaw * 640 / 90)
    cv2.rectangle(frame, (175 - shift, 135), (265 - shift, 225), (0, 0, 255), -1)
    cv2.circle(frame, (430 - shift, 180), 48, (0, 255, 0), -1)
    return frame


def two_red_squares(yaw=0):
    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    shift = round(yaw * 640 / 90)
    for center_x in (200, 440):
        cv2.rectangle(frame, (center_x - 45 - shift, 135),
                      (center_x + 45 - shift, 225), (0, 0, 255), -1)
    return frame


class FakeLogger:
    def __init__(self):
        self.pitch = 0
        self.chassis_pitch = None
        self.yaw = 0
        self.timestamp = time.time()
        self.status = (0,) * 10

    def get_sample(self, name, max_age_s=None):
        if name == "gimbal":
            return ((self.pitch if self.chassis_pitch is None else self.chassis_pitch),
                    self.yaw, self.pitch, self.yaw), self.timestamp
        if name == "status":
            return self.status, time.time()
        if name in ("position", "attitude"):
            return (0, 0, 0), time.time()
        return None


class FakeGimbal:
    def __init__(self, logger):
        self.logger = logger
        self.commands = []
        self.active = False

    def moveto(self, pitch, yaw, pitch_speed, yaw_speed):
        if self.active:
            raise AssertionError("overlapping gimbal action")
        self.active = True
        self.commands.append((pitch, yaw))
        self.logger.pitch, self.logger.yaw = pitch, yaw
        if pitch == 0 and self.logger.chassis_pitch is not None:
            self.logger.chassis_pitch = pitch
        self.logger.timestamp = time.time() + .01

        def release():
            self.active = False
            return True

        return SimpleNamespace(has_succeeded=True, wait_for_completed=release)

    def recenter(self, pitch_speed, yaw_speed):
        return self.moveto(0, 0, pitch_speed, yaw_speed)


class FakeFrames:
    def __init__(self, frame):
        self.frame = frame
        self.number = 0

    def current_frame_number(self):
        return self.number

    def wait_for_frame(self, after_number=0, check_health=None):
        if check_health:
            check_health()
        self.number = after_number + 1
        return self.number, self.frame.copy()


class FakeWorker:
    def __init__(self):
        self.paused = False

    def status(self):
        return {"error": None}

    def pause_mapping(self):
        self.paused = True

    def resume_mapping(self):
        self.paused = False


class TargetTests(unittest.TestCase):
    def test_near_target_uses_frame_fraction_and_selection(self):
        items = detect(red_square())
        self.assertEqual([(item.color, item.shape) for item in items],
                         [("red", "square")])
        self.assertGreater(items[0].area / (640 * 360), .02)
        self.assertEqual(detect(red_square(size=25)), [])
        self.assertEqual(detect(red_square(), selected={("blue", "square")}), [])

    def test_all_shape_types_and_colors_are_available(self):
        frame = np.zeros((360, 640, 3), dtype=np.uint8)
        cv2.rectangle(frame, (30, 30), (125, 125), (0, 0, 255), -1)
        cv2.circle(frame, (255, 80), 52, (0, 255, 0), -1)
        cv2.rectangle(frame, (380, 30), (550, 100), (0, 255, 255), -1)
        cv2.rectangle(frame, (95, 185), (170, 335), (255, 0, 0), -1)
        pairs = {(item.color, item.shape) for item in detect(frame)}
        self.assertEqual(pairs, {("red", "square"), ("green", "circle"),
                                 ("yellow", "horizontal"), ("blue", "vertical")})

    def test_tracker_requires_distinct_frames(self):
        tracker = TargetTracker(.02)
        frame = red_square()
        for count in range(1, 4):
            visible = tracker.update(frame)
            self.assertEqual(tracker.tracks[1].consecutive, count)
            self.assertIn(1, visible)

    def make_inspector(self, fire_result=True):
        settings = copy.deepcopy(load_config()["exploration"])
        settings["target_inspection"].update({"pitch_deg": -15.0,
                                               "confirm_frames": 3,
                                               "lock_frames": 3,
                                               "fire_hold_s": .001,
                                               "max_targets_per_wall": 1})
        logger = FakeLogger()
        gimbal = FakeGimbal(logger)
        frames = FakeFrames(red_square())
        worker = FakeWorker()
        chassis = SimpleNamespace(stop=lambda: None)
        calls = []

        def fire(**kwargs):
            self.assertTrue(worker.paused)
            self.assertFalse(gimbal.active)
            calls.append(kwargs)
            return fire_result

        blaster = SimpleNamespace(fire=fire)
        inspector = WallTargetInspector(gimbal, blaster, frames, logger, chassis,
                                        worker, settings, fire_type="infrared")
        return inspector, gimbal, worker, calls

    def test_wall_inspection_fires_two_shots_after_lock_and_restores_scan_pitch(self):
        inspector, gimbal, worker, calls = self.make_inspector()
        inspector.settings["target_inspection"]["fire_hold_s"] = .03
        started = time.monotonic()
        result = inspector.inspect((0, 0), (1, 0), 0, 0)
        self.assertGreaterEqual(time.monotonic() - started, .03)
        self.assertEqual(result["status"], "targets_checked")
        self.assertEqual(result["targets"][0]["status"], "fire_command_accepted")
        self.assertEqual(calls, [{"fire_type": "infrared", "times": 2}])
        self.assertEqual(result["targets"][0]["aim_mode"], "centered")
        self.assertEqual(result["targets"][0]["fire_hold_s"], .03)
        self.assertEqual(gimbal.commands, [(-15.0, 0), (0.0, 0)])
        self.assertFalse(worker.paused)

    def test_rejected_fire_stops_without_retry_and_restores_mapping(self):
        inspector, gimbal, worker, calls = self.make_inspector(False)
        with self.assertRaisesRegex(MissionStop, "firing state unknown"):
            inspector.inspect((0, 0), (1, 0), 0, 0)
        self.assertEqual(len(calls), 1)
        self.assertEqual(gimbal.commands[-1], (0.0, 0))
        self.assertFalse(worker.paused)

    def test_aim_moves_toward_off_center_target_before_firing(self):
        inspector, gimbal, worker, calls = self.make_inspector()

        class MovingFrames(FakeFrames):
            def wait_for_frame(self, after_number=0, check_health=None):
                if check_health:
                    check_health()
                self.number = after_number + 1
                x = round(430 - gimbal.logger.yaw * 640 / 90)
                return self.number, red_square(center=(x, 180))

        inspector.camera_frames = MovingFrames(red_square())
        result = inspector.inspect((0, 0), (1, 0), 0, 0)
        self.assertEqual(result["targets"][0]["status"], "fire_command_accepted")
        self.assertGreater(len(gimbal.commands), 2)
        self.assertEqual(len(calls), 1)
        self.assertFalse(worker.paused)

    def test_wall_inspection_aims_at_two_targets_on_same_wall(self):
        inspector, gimbal, worker, calls = self.make_inspector()
        inspector.settings["target_inspection"]["max_targets_per_wall"] = 2

        class MovingFrames(FakeFrames):
            def wait_for_frame(self, after_number=0, check_health=None):
                if check_health:
                    check_health()
                self.number = after_number + 1
                return self.number, two_targets(gimbal.logger.yaw)

        inspector.camera_frames = MovingFrames(two_targets())
        result = inspector.inspect((0, 0), (1, 0), 0, 0)
        self.assertEqual(result["status"], "targets_checked")
        self.assertEqual({(item["color"], item["shape"]) for item in result["targets"]},
                         {("red", "square"), ("green", "circle")})
        self.assertEqual(calls, [{"fire_type": "infrared", "times": 2}] * 2)
        self.assertFalse(worker.paused)

    def test_pitch_limit_fires_two_infrared_shots_at_closest_reachable_point(self):
        inspector, gimbal, worker, calls = self.make_inspector()
        inspector.settings["target_inspection"]["pitch_deg"] = -20.0
        inspector.camera_frames = FakeFrames(red_square(center=(320, 270)))
        result = inspector.inspect((0, 0), (1, 0), 0, 0)
        self.assertEqual(calls, [{"fire_type": "infrared", "times": 2}])
        self.assertEqual(result["targets"][0]["shots_requested"], 2)
        self.assertEqual(result["targets"][0]["aim_mode"], "closest_reachable")
        self.assertFalse(worker.paused)

    def test_two_same_color_shapes_are_aimed_separately(self):
        inspector, gimbal, _, calls = self.make_inspector()
        inspector.settings["target_inspection"]["max_targets_per_wall"] = 2

        class MovingFrames(FakeFrames):
            def wait_for_frame(self, after_number=0, check_health=None):
                if check_health:
                    check_health()
                self.number = after_number + 1
                return self.number, two_red_squares(gimbal.logger.yaw)

        inspector.camera_frames = MovingFrames(two_red_squares())
        result = inspector.inspect((0, 0), (1, 0), 0, 0)
        self.assertEqual(len(result["targets"]), 2)
        self.assertTrue(all(item["status"] == "fire_command_accepted" for item in result["targets"]))
        self.assertEqual(len(calls), 2)

    def test_off_center_target_fires_at_best_observed_angle_after_aim_budget(self):
        inspector, gimbal, _, calls = self.make_inspector()
        inspector.settings["target_inspection"].update({"pitch_deg": -20.0,
                                                        "max_aim_steps": 4})
        inspector.camera_frames = FakeFrames(red_square(center=(480, 270)))
        result = inspector.inspect((0, 0), (1, 0), 0, 0)
        self.assertEqual(calls, [{"fire_type": "infrared", "times": 2}])
        self.assertEqual(result["targets"][0]["status"], "fire_command_accepted")
        self.assertEqual(result["targets"][0]["aim_mode"], "closest_reachable")
        self.assertEqual(gimbal.commands[-2], (-20.0, 0.0))

    def test_safety_flag_prevents_aiming_and_firing(self):
        inspector, gimbal, worker, calls = self.make_inspector()
        inspector.logger.status = (0, 0, 0, 0, 1, 0, 0, 0, 0, 0)
        with self.assertRaisesRegex(MissionStop, "safety status flag 4"):
            inspector.inspect((0, 0), (1, 0), 0, 0)
        self.assertEqual(calls, [])
        self.assertEqual(gimbal.commands, [])
        self.assertFalse(worker.paused)

    def test_gimbal_waits_for_slow_action_release(self):
        inspector, gimbal, _, _ = self.make_inspector()
        checks = []

        class SlowAction:
            state = "action_running"

            @property
            def has_succeeded(self):
                checks.append(1)
                return len(checks) >= 3

            def wait_for_completed(self):
                gimbal.active = False
                return True

        def delayed_move(pitch, yaw, pitch_speed, yaw_speed):
            self.assertFalse(gimbal.active)
            gimbal.active = True
            gimbal.commands.append((pitch, yaw))
            gimbal.logger.pitch, gimbal.logger.yaw = pitch, yaw
            gimbal.logger.timestamp = time.time() + .01
            return SlowAction()

        gimbal.moveto = delayed_move
        inspector._point(-15, 0)
        self.assertGreaterEqual(len(checks), 3)
        self.assertFalse(gimbal.active)

    def test_inspection_accepts_ground_pitch_when_chassis_pitch_drifts(self):
        inspector, gimbal, worker, calls = self.make_inspector()
        gimbal.logger.chassis_pitch = -23.6
        progress = []
        inspector.on_progress = progress.append
        result = inspector.inspect((0, 0), (0, -1), -90, 0)
        self.assertEqual(result["targets"][0]["status"], "fire_command_accepted")
        self.assertEqual(len(calls), 1)
        self.assertFalse(worker.paused)
        self.assertTrue(any(item["status"] == "waiting_camera_frame" for item in progress))
        self.assertTrue(any(item["status"] == "waiting_target_angle" and
                            item["pitch_frame"] == "ground" and
                            item["target_pitch_deg"] == -15 for item in progress))

    def test_camera_error_does_not_overlap_running_gimbal_action(self):
        inspector, gimbal, worker, calls = self.make_inspector()
        inspector.camera_frames.camera_error = None

        def unfinished_move(pitch, yaw, pitch_speed, yaw_speed):
            gimbal.commands.append((pitch, yaw))
            inspector.camera_frames.camera_error = "stream lost"
            return SimpleNamespace(state="action_running", has_succeeded=False)

        gimbal.moveto = unfinished_move
        with self.assertRaisesRegex(MissionStop, "camera stopped"):
            inspector.inspect((0, 0), (1, 0), 0, 0)
        self.assertEqual(gimbal.commands, [(-15.0, 0)])
        self.assertEqual(calls, [])
        self.assertFalse(worker.paused)

    def test_sdk_target_command_uses_ground_pitch_frame(self):
        from robomaster import gimbal as sdk_gimbal

        sent = []
        sdk = SimpleNamespace(_action_dispatcher=SimpleNamespace(send_action=sent.append))
        action = sdk_gimbal.Gimbal.moveto(sdk, pitch=-15, yaw=-90)
        self.assertIs(sent[0], action)
        self.assertEqual(action._coordinate, sdk_gimbal.COORDINATE_YCPN)

    def test_restoring_scan_angle_uses_chassis_pitch_frame(self):
        inspector, gimbal, _, _ = self.make_inspector()
        inspector.logger.chassis_pitch = -23.6
        scan_commands = []

        def scan_recenter(pitch_speed, yaw_speed):
            scan_commands.append("recenter")
            inspector.logger.chassis_pitch = 0
            inspector.logger.yaw = 0
            inspector.logger.timestamp = time.time() + .01
            return SimpleNamespace(has_succeeded=True, wait_for_completed=lambda: True)

        inspector.scan_gimbal = SimpleNamespace(recenter=scan_recenter)
        result = inspector.inspect((0, 0), (0, -1), -90, 0)
        self.assertEqual(result["status"], "targets_checked")
        self.assertEqual(scan_commands, ["recenter"])
        self.assertEqual(gimbal.commands[0], (-15.0, -90))

    def test_interrupted_restore_records_recenter_action_state(self):
        inspector, _, worker, _ = self.make_inspector()
        progress = []
        inspector.on_progress = progress.append

        def recenter(pitch_speed, yaw_speed):
            return SimpleNamespace(state="action_running", has_succeeded=False)

        inspector.scan_gimbal = SimpleNamespace(recenter=recenter)
        inspector.logger.get_sample_original = inspector.logger.get_sample

        def interrupted_sample(name, max_age_s=None):
            if name == "gimbal" and inspector.active_action is not None:
                raise KeyboardInterrupt()
            return inspector.logger.get_sample_original(name, max_age_s)

        inspector.logger.get_sample = interrupted_sample
        with self.assertRaises(KeyboardInterrupt):
            inspector.inspect((0, 0), (0, -1), -90, 0)
        self.assertFalse(worker.paused)
        self.assertEqual(progress[-1]["status"], "stopped")
        self.assertEqual(progress[-1]["interrupted_from"], "waiting_target_action")
        self.assertEqual(progress[-1]["action_state"], "action_running")

    def test_interrupted_inspection_records_the_waiting_step(self):
        inspector, _, worker, _ = self.make_inspector()
        progress = []
        inspector.on_progress = progress.append

        def interrupted_frame(*args, **kwargs):
            raise KeyboardInterrupt()

        inspector.camera_frames.wait_for_frame = interrupted_frame
        with self.assertRaises(KeyboardInterrupt):
            inspector.inspect((0, 0), (0, -1), -90, 0)
        self.assertFalse(worker.paused)
        self.assertEqual(progress[-1]["status"], "stopped")
        self.assertEqual(progress[-1]["interrupted_from"], "waiting_camera_frame")


if __name__ == "__main__":
    unittest.main()
