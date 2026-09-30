"""Regression coverage for shared SDK dispatcher ownership (no robot required)."""

import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from src.gimbal_control import wait_for_gimbal_idle
from src.mission_stop import MissionStop


class GimbalActionLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.action = SimpleNamespace(target=4, state="action_running", has_succeeded=False)
        self.other = SimpleNamespace(target=3, state="action_running", has_succeeded=False)
        self.dispatcher = SimpleNamespace(
            _in_progress_mutex=threading.Lock(),
            _in_progress={"head": self.action, "chassis": self.other},
        )
        self.gimbal = SimpleNamespace(_host=4, _action_dispatcher=self.dispatcher)
        self.health = Mock()
        self.waiting = Mock()

    def release(self, timeout=None):
        # Completion callbacks must be able to acquire the dispatcher mutex.
        self.assertTrue(self.dispatcher._in_progress_mutex.acquire(blocking=False))
        try:
            self.assertIs(self.dispatcher._in_progress.pop("head"), self.action)
        finally:
            self.dispatcher._in_progress_mutex.release()
        return True

    def test_waits_for_real_success_and_release_without_touching_chassis(self):
        polls = []

        def complete_after_two_polls(delay):
            self.assertEqual(self.action.state, "action_running")
            self.assertIs(self.dispatcher._in_progress["head"], self.action)
            polls.append(delay)
            if len(polls) == 2:
                self.action.state = "action_succeeded"
                self.action.has_succeeded = True

        self.action.wait_for_completed = Mock(side_effect=self.release)
        with patch("src.gimbal_control.time.sleep", side_effect=complete_after_two_polls):
            wait_for_gimbal_idle(self.gimbal, self.health, self.waiting)

        self.assertEqual(len(polls), 2)
        self.waiting.assert_called_once_with(self.action)
        self.action.wait_for_completed.assert_called_once_with(timeout=None)
        self.assertEqual(self.dispatcher._in_progress, {"chassis": self.other})
        self.assertEqual(self.other.state, "action_running")

    def test_failed_action_is_preserved_and_stops_next_command(self):
        self.action.state = "action_failed"
        with self.assertRaisesRegex(MissionStop, "previous gimbal action failed"):
            wait_for_gimbal_idle(self.gimbal, self.health, self.waiting)
        self.assertIs(self.dispatcher._in_progress["head"], self.action)
        self.assertEqual(self.action.state, "action_failed")

    def test_inspection_completion_wait_uses_existing_deadline(self):
        self.action.state = "action_succeeded"
        self.action.has_succeeded = True
        self.action.wait_for_completed = Mock(side_effect=self.release)
        wait_for_gimbal_idle(self.gimbal, self.health, self.waiting,
                            remaining_timeout=lambda: 1.5)
        self.action.wait_for_completed.assert_called_once_with(timeout=1.5)

    def test_health_stop_does_not_fake_completion(self):
        self.health.side_effect = [None, MissionStop("inspection timeout")]
        with self.assertRaisesRegex(MissionStop, "inspection timeout"):
            wait_for_gimbal_idle(self.gimbal, self.health, self.waiting)
        self.assertIs(self.dispatcher._in_progress["head"], self.action)
        self.assertFalse(self.action.has_succeeded)
