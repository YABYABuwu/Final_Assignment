import itertools
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from src.directional_recovery import escape_direction, recover, DirectionalToF
from src.mission_stop import MissionStop
from src.config_loader import load_config


class DirectionalRecoveryTests(unittest.TestCase):
    def test_all_sixteen_ir_patterns(self):
        expected = { (0,0,0,0): "clear", (1,0,0,0): "slide_right",
                     (0,0,1,0): "slide_right", (1,0,1,0): "slide_right",
                     (0,1,0,0): "slide_left", (0,0,0,1): "slide_left",
                     (0,1,0,1): "slide_left", (1,1,0,0): "backward",
                     (0,0,1,1): "forward"}
        for bits in itertools.product((False, True), repeat=4):
            self.assertEqual(escape_direction(bits[:2], bits[2:])[0], expected.get(bits, "blocked"))
        self.assertEqual(escape_direction((None, False), (False, False))[0], "waiting_data")

    def simulate(self, clear_at=None, conflict=False, margin=1., stalled=False):
        self.position = [0., 0., 0.]
        self.velocity = [0., 0.]
        self.commands = []
        self.travel = 0.
        self.stamp = 0
        def drive(x, y, z):
            self.velocity[:] = [x, y]
            if x or y:
                self.commands.append((x,y))
        def pose():
            self.stamp += 1
            self.controller.position_sample_time = self.stamp
            for i in (0,1):
                d = self.velocity[i] * .1 if not stalled else 0
                self.position[i] += d
                self.travel += abs(d)
            return tuple(self.position)
        def snapshot(front):
            clear = clear_at is not None and self.travel >= clear_at
            return {"sample_time": self.stamp,
                    "sides": {"left": {"detected": not front and not clear},
                              "right": {"detected": front and conflict}}}
        self.front, self.rear = Mock(), Mock()
        self.front.snapshot.side_effect = lambda: snapshot(True)
        self.rear.snapshot.side_effect = lambda: snapshot(False)
        self.rear.settings = {"recovery_clear_samples": 2, "recovery_speed_m_s": .08}
        self.guard = Mock()
        self.guard.clearance.return_value = margin
        self.guard.owner.settings = {"max_sample_age_s": .5}
        self.guard.owner._gimbal_sample.return_value = (0, 0, time.time())
        logger = Mock()
        logger.get_sample.return_value = None
        self.controller = SimpleNamespace(directional_tof=self.guard, logger=logger,
            front_ir=self.front, rear_ir=self.rear, get_pose=pose, position_sample_time=0,
            chassis=SimpleNamespace(drive_speed=drive), stop=lambda: drive(0,0,0))
        self.budget = {}
        with patch("src.directional_recovery.time.sleep"):
            return recover(self.controller, self.rear, "left", .05, None, None, None, self.budget)

    def test_left_rear_only_moves_right_and_retargets_from_actual_pose(self):
        self.assertTrue(self.simulate(clear_at=.035))
        self.assertTrue(all(x == 0 and y > 0 for x,y in self.commands))
        self.assertGreater(self.budget["cleared_pose"][1], .03)

    def test_checks_again_after_four_steps_and_can_clear(self):
        self.assertTrue(self.simulate(clear_at=.105))
        self.assertIn("reassessed_after_four=True", self.rear.finish_recovery.call_args.args[3])

    def test_exhaustion_is_bounded_across_both_ends(self):
        with self.assertRaisesRegex(MissionStop, "0.16 m|8 steps"):
            self.simulate()
        self.assertLess(self.travel, .17)

    def test_conflict_and_insufficient_tof_never_drive(self):
        for options in ({"conflict": True}, {"margin": .01}):
            with self.assertRaises(MissionStop):
                self.simulate(**options)
            self.assertEqual(self.commands, [])

    def test_no_progress_stops(self):
        with self.assertRaisesRegex(MissionStop, "no progress"):
            self.simulate(stalled=True)

    def test_tof_offsets_and_same_direction_reuses_head(self):
        for dx,dy,offset in ((1,0,0),(-1,0,0),(0,1,.05),(0,-1,.06)):
            import math
            yaw = math.degrees(math.atan2(dy,dx))
            owner = Mock()
            owner.settings = load_config()["exploration"]
            owner.slam_worker = None
            owner.base_pose = (0,0,0)
            owner.chassis.get_pose.return_value = (0,0,0)
            owner._sensor_offset.return_value = .075
            owner.logger.get_sample.side_effect = lambda name, **kw: (
                (200,) if name == "tof" else (0,yaw,0,yaw), time.time())
            guard = DirectionalToF(owner)
            with patch("src.directional_recovery.time.time", side_effect=itertools.count(100,.001)):
                first = guard.clearance(dx,dy)
                second = guard.clearance(dx,dy)
            self.assertAlmostEqual(first, .2+offset-.175)
            self.assertAlmostEqual(first, second)
            owner._scan_for_direction.assert_called_once()
