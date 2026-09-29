"""Simple position control for a RoboMaster EP chassis."""

import math
import time

from src.PID import PIDController
from src.mission_stop import MissionStop
from src.rear_ir import recovery_vector


def angle_error(target, current):
    """Shortest signed angle from current to target, in degrees."""
    return (target - current + 180) % 360 - 180


class ChassisController:
    def __init__(self, robot, logger, settings):
        self.chassis = robot.chassis
        self.logger = logger
        self.settings = settings
        pid = settings["pid"]
        self.pid_x = PIDController(**pid["x"], max_output=settings["max_speed_m_s"])
        self.pid_y = PIDController(**pid["y"], max_output=settings["max_speed_m_s"])
        self.pid_yaw = PIDController(**pid["yaw"], max_output=settings["max_turn_deg_s"])
        self.heading_reference = None
        self.gimbal_heading_offset = None
        self.heading_bias_deg = 0.0
        self.commanded_lateral_m_s = 0.0
        self.position_sample_time = None
        self.rear_ir = None
        self.front_ir = None

    def reset_heading(self):
        """Use the current heading as the reference on the next move_to call."""
        self.heading_reference = None

    def set_heading_bias(self, bias_deg):
        """Apply a wall-observed yaw correction to future pose readings."""
        self.heading_bias_deg = angle_error(float(bias_deg), 0)

    def stop(self):
        """Send zero speed to all three axes."""
        self.chassis.drive_speed(x=0, y=0, z=0)
        self.commanded_lateral_m_s = 0.0

    def get_pose(self):
        """Return (x metres, y metres, yaw degrees), or None if data is stale."""
        age = self.settings["sample_timeout_s"]
        sample_getter = getattr(self.logger, "get_sample", None)
        position_sample = (sample_getter("position", max_age_s=age)
                           if callable(sample_getter) else None)
        position = (position_sample[0] if position_sample is not None else
                    self.logger.get_latest("position", max_age_s=age))
        attitude = self.logger.get_latest("attitude", max_age_s=age)
        if position is None or attitude is None:
            return None
        self.position_sample_time = position_sample[1] if position_sample is not None else None
        yaw = float(attitude[0])
        if self.settings.get("heading_source", "attitude") == "gimbal":
            # Gimbal ground yaw and chassis-relative yaw come from one SDK
            # sample. Their difference tracks chassis rotation even when the
            # chassis attitude stream under-reports gradual physical yaw.
            sample = self.logger.get_sample("gimbal", max_age_s=age)
            if sample is None or len(sample[0]) < 4:
                return None
            angles = sample[0]
            try:
                valid = all(math.isfinite(float(value)) for value in
                            (angles[1], angles[3], yaw))
            except (TypeError, ValueError):
                return None
            if not valid:
                return None
            observed = angle_error(float(angles[3]), float(angles[1]))
            if self.gimbal_heading_offset is None:
                self.gimbal_heading_offset = angle_error(yaw, observed)
            yaw = angle_error(observed + self.gimbal_heading_offset, 0)
        return position[0], position[1], angle_error(yaw + self.heading_bias_deg, 0)

    def _recover_from_ir(self, bumper, side, attempt, period, deadline, abort_event,
                         pause_if, movement_axis="longitudinal"):
        """Move away from active IRs for one bounded recovery attempt."""
        settings = bumper.settings
        forward = settings["recovery_speed_m_s"]
        origin = None
        last_pose = None
        last_sample_time = None
        clear_count = 0
        commanded_m = 0.0
        status = "stopped"
        reason = None
        bumper.recovering = side
        try:
            while True:
                if abort_event is not None and abort_event.is_set():
                    raise MissionStop("IR recovery aborted because exploration telemetry failed")
                if deadline is not None and time.monotonic() >= deadline:
                    raise MissionStop("IR recovery exceeded waypoint time")
                if pause_if is not None and pause_if():
                    self.stop()
                    clear_count = 0
                    time.sleep(period)
                    continue
                state = bumper.snapshot()
                sensors = state["sides"]
                pose = self.get_pose()
                if (any(sensor["detected"] is None for sensor in sensors.values()) or
                        pose is None):
                    self.stop()
                    clear_count = 0
                    time.sleep(period)
                    continue
                if origin is None:
                    origin = pose
                last_pose = pose
                traveled_m = math.hypot(pose[0] - origin[0], pose[1] - origin[1])
                if (traveled_m >= settings["recovery_max_m"] or
                        commanded_m >= settings["recovery_max_m"]):
                    status = "limit"
                    reason = "{} IR {} still blocked after one recovery attempt".format(bumper.end, side)
                    return False
                sample_time = state["sample_time"]
                new_sample = sample_time != last_sample_time
                if new_sample:
                    both_clear = all(sensor["detected"] is False for sensor in sensors.values())
                    clear_count = clear_count + 1 if both_clear else 0
                    last_sample_time = sample_time
                if clear_count >= settings["recovery_clear_samples"]:
                    status = "cleared"
                    bumper.last_block = None
                    return True
                if not new_sample:
                    self.stop()
                    time.sleep(period)
                    continue
                forward_clear = True
                clear_threshold_mm = settings.get("forward_tof_clear_mm", 250.0)
                if bumper.end == "rear":
                    if self.front_ir is not None and self.front_ir.blocks_motion(forward, 0, 0):
                        if self.front_ir.last_block == "waiting_data":
                            self.stop()
                            clear_count = 0
                            time.sleep(period)
                            continue
                        forward_clear = False
                    elif self.logger is not None:
                        tof_sample = self.logger.get_sample("tof", max_age_s=0.5)
                        if tof_sample is not None and len(tof_sample[0]) > 0:
                            try:
                                tof_val = float(tof_sample[0][0])
                                if 0 < tof_val < clear_threshold_mm:
                                    forward_clear = False
                            except (TypeError, ValueError, IndexError):
                                pass
                else:
                    if self.rear_ir is not None and self.rear_ir.blocks_motion(-forward, 0, 0):
                        if self.rear_ir.last_block == "waiting_data":
                            self.stop()
                            clear_count = 0
                            time.sleep(period)
                            continue
                        forward_clear = False

                mode = settings.get("recovery_mode", "diagonal")
                direction, escape_x, escape_y = recovery_vector(
                    sensors, forward, attempt=attempt, end=bumper.end,
                    mode=mode, forward_clear=forward_clear,
                    movement_axis=movement_axis)
                if direction == "blocked":
                    raise MissionStop("{} IR {} has no clear longitudinal escape".format(
                        bumper.end, side))
                if direction == "clear":
                    # Hold still while confirming consecutive clear samples.
                    self.stop()
                    time.sleep(period)
                    continue
                opposite = self.front_ir if bumper.end == "rear" else self.rear_ir
                if opposite is not None and opposite.blocks_motion(escape_x, escape_y, 0):
                    self.stop()
                    if opposite.last_block == "waiting_data":
                        clear_count = 0
                        time.sleep(period)
                        continue
                    raise MissionStop("{} IR {} blocks {} recovery".format(
                        opposite.end, opposite.last_block, bumper.end))
                bumper.recovering = direction
                self.chassis.drive_speed(x=escape_x, y=escape_y, z=0)
                self.commanded_lateral_m_s = escape_y
                commanded_m += forward * period
                time.sleep(period)
        except MissionStop as error:
            reason = str(error)
            raise
        finally:
            self.stop()
            distance_m = (math.hypot(last_pose[0] - origin[0], last_pose[1] - origin[1])
                          if origin is not None and last_pose is not None else 0.0)
            bumper.finish_recovery(side, status, distance_m, reason)

    def move_to(self, x, y, yaw=None, timeout_s=None, abort_event=None,
                disable_timeout=False, stop_if=None, pause_if=None):
        """Drive toward an absolute (x, y) waypoint; optionally face yaw.

        x/y use the chassis position frame established at subscription.
        With yaw=None, hold the first move's heading across later waypoints.
        stop_if receives each fresh pose before the PID command; returning True
        ends the move at that pose and the finally block stops the wheels.
        Missing telemetry pauses the wheels until fresh data returns. A caller
        supplied deadline still raises TimeoutError when progress never finishes.
        """
        timeout = None if disable_timeout else (
            timeout_s if timeout_s is not None else self.settings["timeout_s"]
        )
        if timeout is not None and timeout <= 0:
            raise ValueError("timeout_s must be positive")
        for pid in (self.pid_x, self.pid_y, self.pid_yaw):
            pid.reset()

        period = self.settings["control_period_s"]
        deadline = None if timeout is None else time.monotonic() + timeout
        previous = time.monotonic()
        previous_position = None
        previous_position_time = None
        approach_speed = 0.0
        measured_speed = 0.0
        target_yaw = yaw
        recovery_attempts = {"front": 0, "rear": 0}
        if yaw is not None:
            self.heading_reference = yaw
        try:
            while deadline is None or time.monotonic() < deadline:
                if abort_event is not None and abort_event.is_set():
                    raise MissionStop("motion aborted because exploration telemetry failed")
                if pause_if is not None and pause_if():
                    self.stop()
                    previous_position = None
                    previous_position_time = None
                    approach_speed = 0.0
                    measured_speed = 0.0
                    for pid in (self.pid_x, self.pid_y, self.pid_yaw):
                        pid.reset()
                    previous = time.monotonic()
                    time.sleep(period)
                    continue
                pose = self.get_pose()
                if pose is None:
                    self.stop()
                    previous_position = None
                    previous_position_time = None
                    approach_speed = 0.0
                    measured_speed = 0.0
                    for pid in (self.pid_x, self.pid_y, self.pid_yaw):
                        pid.reset()
                    previous = time.monotonic()
                    time.sleep(period)
                    continue
                if stop_if is not None and stop_if(pose):
                    return pose
                if stop_if is not None:
                    pose = self.get_pose()
                    if pose is None:
                        self.stop()
                        previous_position = None
                        previous_position_time = None
                        approach_speed = 0.0
                        measured_speed = 0.0
                        for pid in (self.pid_x, self.pid_y, self.pid_yaw):
                            pid.reset()
                        previous = time.monotonic()
                        time.sleep(period)
                        continue
                current_x, current_y, current_yaw = pose
                if target_yaw is None and self.settings.get("hold_heading", True):
                    if self.heading_reference is None:
                        self.heading_reference = current_yaw
                    target_yaw = self.heading_reference
                error_x = x - current_x
                error_y = y - current_y
                distance = math.hypot(error_x, error_y)
                heading_error = 0 if target_yaw is None else angle_error(target_yaw, current_yaw)
                now = time.monotonic()
                dt = max(now - previous, 0.001)
                previous = now
                position_time = self.position_sample_time or now
                if previous_position_time is not None and position_time > previous_position_time:
                    sample_dt = position_time - previous_position_time
                    displacement = (current_x - previous_position[0],
                                    current_y - previous_position[1])
                    measured = math.hypot(*displacement) / sample_dt
                    measured_speed = min(measured, 2 * self.settings["max_speed_m_s"])
                    if distance > 0 and measured <= 2 * self.settings["max_speed_m_s"]:
                        measured = (displacement[0] * error_x + displacement[1] * error_y) / (sample_dt * distance)
                        approach_speed = measured
                if previous_position_time is None or position_time > previous_position_time:
                    previous_position = (current_x, current_y)
                    previous_position_time = position_time
                arrival_speed = math.sqrt(2 * self.settings["braking_decel_m_s2"] *
                                          self.settings["position_tolerance_m"])
                arrived = (distance <= self.settings["position_tolerance_m"] and
                           measured_speed <= arrival_speed)
                facing = target_yaw is None or abs(heading_error) <= self.settings["angle_tolerance_deg"]
                if arrived and facing:
                    return pose
                # Leave room for one control cycle of measured travel, then
                # cap the command by the speed that can stop at the waypoint.
                remaining = max(0.0, distance - max(0.0, approach_speed) * period)
                braking_limit = math.sqrt(2 * self.settings["braking_decel_m_s2"] * remaining)
                speed_limit = min(self.settings["max_speed_m_s"], braking_limit)
                # Compute speeds in the fixed position frame, then rotate into
                # the robot's forward/right frame used by drive_speed().
                if self.pid_x.previous_error is not None and error_x * self.pid_x.previous_error < 0:
                    self.pid_x.integral = 0.0
                if self.pid_y.previous_error is not None and error_y * self.pid_y.previous_error < 0:
                    self.pid_y.integral = 0.0
                integral_x, integral_y = self.pid_x.integral, self.pid_y.integral
                vx_world = self.pid_x.compute(error_x, dt, anti_windup=True)
                vy_world = self.pid_y.compute(error_y, dt, anti_windup=True)
                heading_rad = math.radians(current_yaw)
                vx_robot = vx_world * math.cos(heading_rad) + vy_world * math.sin(heading_rad)
                vy_robot = -vx_world * math.sin(heading_rad) + vy_world * math.cos(heading_rad)
                speed = math.hypot(vx_robot, vy_robot)
                if speed > speed_limit:
                    # A capped command cannot use additional integral effort.
                    self.pid_x.integral, self.pid_y.integral = integral_x, integral_y
                    vx_robot *= speed_limit / speed
                    vy_robot *= speed_limit / speed
                # Ramp only chassis-sideways speed. Forward/backward PID speed
                # remains immediate; braking and emergency stops stay immediate.
                old_y = self.commanded_lateral_m_s
                if vy_robot * old_y <= 0:
                    old_y = 0.0
                if abs(vy_robot) > abs(old_y):
                    max_change = self.settings["max_lateral_accel_m_s2"] * min(dt, period)
                    vy_robot = old_y + max(-max_change, min(max_change, vy_robot - old_y))
                # Keep correcting small heading errors while translating. The
                # arrival tolerance only decides when the waypoint is done;
                # using it as a deadband lets yaw drift during long slides.
                turn = (self.pid_yaw.compute(heading_error, dt)
                        if target_yaw is not None else 0)
                blocked = [bumper for bumper in (self.front_ir, self.rear_ir)
                           if bumper is not None and bumper.blocks_motion(vx_robot, vy_robot, turn)]
                if blocked:
                    self.stop()
                    bumper = next((item for item in blocked if item.last_block != "waiting_data"), None)
                    if bumper is not None:
                        side = bumper.last_block
                        recovery_attempts[bumper.end] += 1
                        self._recover_from_ir(
                            bumper, side, recovery_attempts[bumper.end], period,
                            deadline, abort_event, pause_if,
                            movement_axis=("lateral" if abs(vy_robot) > abs(vx_robot)
                                           else "longitudinal"))
                    for pid in (self.pid_x, self.pid_y, self.pid_yaw):
                        pid.reset()
                    previous_position = None
                    previous_position_time = None
                    approach_speed = 0.0
                    measured_speed = 0.0
                    previous = time.monotonic()
                    time.sleep(period)
                    continue
                self.chassis.drive_speed(x=vx_robot, y=vy_robot, z=turn)
                self.commanded_lateral_m_s = vy_robot
                time.sleep(period)
            raise TimeoutError(f"waypoint ({x}, {y}, {yaw}) not reached within {timeout} s")
        finally:
            self.stop()
