"""Chassis-relative gimbal actions for DFS scans."""

from robomaster import gimbal as sdk_gimbal, util


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
