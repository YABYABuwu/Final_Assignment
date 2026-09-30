import itertools
import math
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

    def simulate(self, clear_at=None, conflict=False, margin=1., stalled=False,
                 wait_cycles=0, distance_per_sample=None):
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
                if distance_per_sample is not None and self.velocity[i]:
                    d = math.copysign(distance_per_sample(), self.velocity[i])
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
        self.guard.motion_interrupted = False
        self.waited_cycles = 0
        def clearance(dx, dy):
            if self.stamp >= 10 and self.waited_cycles < wait_cycles:
                self.waited_cycles += 1
                return None
            return margin
        self.guard.clearance.side_effect = clearance
        self.guard.owner.settings = {"max_sample_age_s": .5}
        self.guard.owner._gimbal_sample.return_value = (0, 0, time.time())
        logger = Mock()
        logger.get_sample.return_value = None
        self.controller = SimpleNamespace(directional_tof=self.guard, logger=logger,
            front_ir=self.front, rear_ir=self.rear, get_pose=pose, position_sample_time=0,
            chassis=SimpleNamespace(drive_speed=drive), stop=lambda: drive(0,0,0))
        self.budget = {}
        # Simulated position and command timestamps share one clock.
        with patch("src.directional_recovery.time.sleep"), \
                patch("src.directional_recovery.time.time", side_effect=lambda: self.stamp):
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
        with self.assertRaisesRegex(MissionStop, "20 fresh moving position samples"):
            self.simulate(stalled=True)
        self.assertEqual(len(self.commands), 21)

    def test_recovery_floor_and_ceiling_are_preserved(self):
        self.assertTrue(self.simulate(clear_at=.035))
        for x, y in self.commands:
            self.assertGreaterEqual(math.hypot(x, y), .03)
            self.assertLessEqual(math.hypot(x, y), .04)

    def test_waiting_clearance_samples_do_not_consume_progress_window(self):
        with self.assertRaisesRegex(MissionStop, "20 fresh moving position samples"):
            self.simulate(stalled=True, wait_cycles=25)
        self.assertEqual(self.waited_cycles, 25)
        self.assertGreaterEqual(len(self.commands), 21)

    def test_batched_odometry_progress_is_not_a_stall(self):
        # Batched updates survive the window even after more than eight
        # unchanged samples, which the old per-sample rule rejected.
        increments = itertools.cycle([0.] * 12 + [.003])
        self.assertTrue(self.simulate(clear_at=.009, distance_per_sample=lambda: next(increments)))

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
