"""Chassis-relative gimbal actions for DFS scans."""

import time

from robomaster import gimbal as sdk_gimbal, util

from src.mission_stop import MissionStop


def wait_for_gimbal_idle(gimbal, check_health, on_wait, remaining_timeout=None):
    """Wait for real SDK completion without changing dispatcher/action state."""
    raw_gimbal = getattr(gimbal, "gimbal", gimbal)
    dispatcher = getattr(raw_gimbal, "_action_dispatcher", None)
    target = getattr(raw_gimbal, "_host", None)
    if dispatcher is None or target is None:
        return

    while True:
        check_health()
        # The dispatcher is shared with other modules. Only inspect this head,
        # and release its mutex before completion callbacks take the same lock.
        with dispatcher._in_progress_mutex:
            pending = tuple(action for action in dispatcher._in_progress.values()
                            if action.target == target)
        if not pending:
            return
        on_wait(pending[0])
        for action in pending:
            while not action.has_succeeded:
                check_health()
                if action.state in ("action_failed", "action_rejected",
                                    "action_exception", "action_aborted"):
                    raise MissionStop("previous gimbal action failed: {}".format(action.state))
                time.sleep(0.03)
            check_health()
            timeout = remaining_timeout() if remaining_timeout is not None else None
            if not action.wait_for_completed(timeout=timeout):
                raise MissionStop("previous gimbal action was not released by SDK")


class ChassisRelativeGimbal:
    """Expose moveto with both axes in the chassis frame.

    RoboMaster's public moveto fixes pitch in the ground frame. The SDK action
    supports COORDINATE_CAR, so use the same dispatcher and completion flow.
    """

    def __init__(self, gimbal):
        self.gimbal = gimbal

    def moveto(self, pitch=0, yaw=0, pitch_speed=30, yaw_speed=30):
        action = sdk_gimbal.GimbalMoveAction(
            pitch=util.GIMBAL_PITCH_TARGET_CHECKER.val2proto(pitch),
            yaw=util.GIMBAL_YAW_TARGET_CHECKER.val2proto(yaw),
            pitch_speed=pitch_speed,
            yaw_speed=yaw_speed,
            coord=sdk_gimbal.COORDINATE_CAR,
        )
        self.gimbal._action_dispatcher.send_action(action)
        return action

    def recenter(self, pitch_speed=30, yaw_speed=60):
        """Use the SDK's physical center action before DFS starts."""
        return self.gimbal.recenter(pitch_speed=pitch_speed, yaw_speed=yaw_speed)
