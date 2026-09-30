"""Simple position control for a RoboMaster EP chassis."""

import math
import time

from src.PID import PIDController
from src.mission_stop import MissionStop
from src.rear_ir import recovery_vector


def angle_error(target, current):
    """Shortest signed angle from current to target, in degrees."""
    return (target - current + 180) % 360 - 180


def grid_motion_settings(config, speed_override=None):
    """Use the same PID, heading source and default speed for both rounds."""
    settings = config["motion"].copy()
    speed = config["exploration"]["max_speed_m_s"] if speed_override is None else speed_override
    if type(speed) not in (int, float) or not math.isfinite(speed) or speed <= 0:
        raise ValueError("movement speed must be positive and finite")
    settings["max_speed_m_s"] = min(settings["max_speed_m_s"], speed)
    settings["heading_source"] = config["exploration"]["heading_source"]
    return settings


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
        self.directional_tof = None

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

    def _longitudinal_ir_escape_clearance(self, bumper, speed):
        """Return True/False/None for a longitudinal IR escape path.

        None means required safety telemetry is unavailable and recovery must
        stop and wait rather than treating the path as clear.
        """
        escape_x = speed if bumper.end == "rear" else -speed
        opposite = self.front_ir if bumper.end == "rear" else self.rear_ir
        if opposite is not None and opposite.blocks_motion(escape_x, 0, 0):
            if opposite.last_block == "waiting_data":
                return None, "{} IR data unavailable".format(opposite.end)
            return False, "{} IR {} detected".format(
                opposite.end, opposite.last_block)

        if bumper.end == "front":
            if opposite is None:
                return None, "rear IR unavailable"
            return True, None

        if self.logger is None:
            return None, "front ToF unavailable"
        tof_max_age_s = bumper.settings.get(
            "forward_tof_max_age_s", bumper.settings.get("max_age_s", 0.5))
        tof_sample = self.logger.get_sample("tof", max_age_s=tof_max_age_s)
        if tof_sample is None or not tof_sample[0]:
            return None, "front ToF data unavailable"
        try:
            tof_mm = float(tof_sample[0][0])
        except (TypeError, ValueError, IndexError):
            return None, "front ToF value invalid"
        if not math.isfinite(tof_mm) or tof_mm <= 0 or tof_mm == 65535:
            return None, "front ToF value invalid"
        threshold_mm = bumper.settings.get("forward_tof_clear_mm", 250.0)
        if tof_mm < threshold_mm:
            return False, "front ToF {:.0f} mm is below {:.0f} mm".format(
                tof_mm, threshold_mm)
        return True, None

    def _opposite_blocks_recovery_path(self, opposite, escape_x, escape_y, bumper_end):
        """Check if the escape direction puts the opposite bumper at risk.

        Only obstacles in the direction of the escape motion block recovery.
        For example, escaping laterally away from a left obstacle (escape_y > 0)
        is not blocked by an active left sensor on the opposite bumper.
        """
        if opposite is None:
            return False, None

        if hasattr(opposite, "snapshot"):
            opp_state = opposite.snapshot()
            if opp_state.get("state") == "waiting_data":
                return True, "waiting_data"
            opp_sides = opp_state.get("sides", {})

            # When escaping laterally, only the side in the direction of travel can collide
            if escape_y > 0:
                right_det = opp_sides.get("right", {}).get("detected")
                if right_det is None:
                    return True, "waiting_data"
                if right_det is True:
                    return True, "right"
            elif escape_y < 0:
                left_det = opp_sides.get("left", {}).get("detected")
                if left_det is None:
                    return True, "waiting_data"
                if left_det is True:
                    return True, "left"

            # When escaping longitudinally toward the opposite bumper, check its motion block
            if escape_x != 0:
                toward_opp = (escape_x > 0 if opposite.end == "front" else escape_x < 0)
                if toward_opp and opposite.blocks_motion(escape_x, 0, 0):
                    return True, opposite.last_block

            return False, None

        if opposite.blocks_motion(escape_x, escape_y, 0):
            return True, getattr(opposite, "last_block", "detected")
        return False, None

    def _adaptive_ir_escape(
        self,
        bumper,
        sensors,
        speed,
        attempt,
        movement_axis,
        longitudinal_clear,
        clearance_reason,
    ):
        """Try alternate translations using current IR/ToF checks."""
        preferred = recovery_vector(
            sensors,
            speed,
            attempt=attempt,
            end=bumper.end,
            mode="adaptive",
            forward_clear=longitudinal_clear is True,
            movement_axis=movement_axis,
        )

        if preferred[0] == "clear":
            return preferred, None

        candidates = [preferred]

        # Try both cardinal priorities before giving up.
        for candidate_attempt in (1, 2):
            candidates.append(
                recovery_vector(
                    sensors,
                    speed,
                    attempt=candidate_attempt,
                    end=bumper.end,
                    mode="cardinal",
                    forward_clear=longitudinal_clear is True,
                    movement_axis=movement_axis,
                )
            )

        # Diagonal escape requires longitudinal clearance.
        if longitudinal_clear is True:
            candidates.append(
                recovery_vector(
                    sensors,
                    speed,
                    attempt=1,
                    end=bumper.end,
                    mode="diagonal",
                    forward_clear=True,
                    movement_axis=movement_axis,
                )
            )

        opposite = (
            self.front_ir
            if bumper.end == "rear"
            else self.rear_ir
        )

        waiting = False
        rejected = []
        seen = set()

        for direction, vx, vy in candidates:
            candidate = (direction, vx, vy)
            if candidate in seen:
                continue
            seen.add(candidate)

            if direction == "blocked":
                waiting = (
                    waiting or longitudinal_clear is None
                )
                rejected.append(
                    clearance_reason
                    or "longitudinal path blocked"
                )
                continue

            if vx != 0 and longitudinal_clear is not True:
                waiting = (
                    waiting or longitudinal_clear is None
                )
                rejected.append(
                    clearance_reason
                    or "longitudinal path blocked"
                )
                continue

            destination_side = (
                "right"
                if vy > 0
                else "left"
                if vy < 0
                else None
            )

            both_sides = (sensors.get("right", {}).get("detected") is True and
                          sensors.get("left", {}).get("detected") is True)
            if (
                destination_side
                and not both_sides
                and (
                    sensors[destination_side]["detected"]
                    is not False
                )
            ):
                rejected.append(
                    "{} IR {} blocks lateral escape".format(
                        bumper.end,
                        destination_side,
                    )
                )
                continue

            if opposite is not None:
                blocked, side = (
                    self._opposite_blocks_recovery_path(
                        opposite,
                        vx,
                        vy,
                        bumper.end,
                    )
                )

                if blocked:
                    waiting = (
                        waiting or side == "waiting_data"
                    )
                    rejected.append(
                        "{} IR {} blocks {}".format(
                            opposite.end,
                            side,
                            direction,
                        )
                    )
                    continue

            return (direction, vx, vy), None

        return (
            (
                "waiting_clearance" if waiting else "blocked",
                0.0,
                0.0,
            ),
            "; ".join(rejected)
            or "no clear escape direction",
        )

    def _recover_from_ir(
        self,
        bumper,
        side,
        attempt,
        period,
        deadline,
        abort_event,
        pause_if,
        movement_axis="longitudinal",
        recovery_budget=None,
    ):
        """Recover using distance measured from position telemetry."""
        settings = bumper.settings
        if settings.get("recovery_mode") == "directional":
            from src.directional_recovery import recover
            return recover(self, bumper, side, period, deadline, abort_event, pause_if,
                           recovery_budget if recovery_budget is not None else {})
        forward = settings["recovery_speed_m_s"]

        if recovery_budget is None:
            recovery_budget = {"traveled_m": 0.0}

        total_max_m = settings.get(
            "recovery_total_max_m",
            settings["recovery_max_m"],
        )

        last_sample_time = None
        clear_count = 0
        traveled_m = 0.0
        status = "stopped"
        reason = None

        bumper.recovering = side

        try:
            while True:
                if (
                    abort_event is not None
                    and abort_event.is_set()
                ):
                    raise MissionStop(
                        "IR recovery aborted because "
                        "exploration telemetry failed"
                    )

                if (
                    deadline is not None
                    and time.monotonic() >= deadline
                ):
                    raise MissionStop(
                        "IR recovery exceeded waypoint time"
                    )

                if pause_if is not None and pause_if():
                    self.stop()
                    clear_count = 0
                    time.sleep(period)
                    continue

                state = bumper.snapshot()
                sensors = state["sides"]
                pose = self.get_pose()

                # Accumulate measured travel between position samples.
                # The budget retains the last position across attempts.
                if pose is not None:
                    position_time = self.position_sample_time

                    previous_pose = recovery_budget.get(
                        "last_pose"
                    )
                    previous_time = recovery_budget.get(
                        "last_position_time"
                    )

                    new_position = (
                        position_time is None
                        or previous_time is None
                        or position_time > previous_time
                    )

                    if new_position:
                        if previous_pose is not None:
                            distance_delta = math.hypot(
                                pose[0] - previous_pose[0],
                                pose[1] - previous_pose[1],
                            )

                            traveled_m += distance_delta
                            recovery_budget["traveled_m"] += (
                                distance_delta
                            )

                        recovery_budget["last_pose"] = tuple(
                            pose
                        )
                        recovery_budget["last_position_time"] = (
                            position_time
                        )

                if (
                    any(
                        sensor["detected"] is None
                        for sensor in sensors.values()
                    )
                    or pose is None
                ):
                    self.stop()
                    clear_count = 0
                    time.sleep(period)
                    continue

                reached_limit = (
                    traveled_m >= settings["recovery_max_m"]
                )

                reached_total_limit = (
                    recovery_budget["traveled_m"] >= total_max_m
                )

                sample_time = state["sample_time"]
                new_sample = (
                    sample_time != last_sample_time
                )

                both_clear = all(
                    sensor["detected"] is False
                    for sensor in sensors.values()
                )

                if new_sample:
                    clear_count = (
                        clear_count + 1 if both_clear else 0
                    )
                    last_sample_time = sample_time

                if (
                    clear_count
                    >= settings["recovery_clear_samples"]
                ):
                    status = "cleared"
                    bumper.last_block = None
                    recovery_budget["cleared_pose"] = tuple(pose)
                    return True

                if both_clear:
                    # Stay still while confirming fresh clear samples.
                    self.stop()
                    time.sleep(period)
                    continue

                if not new_sample:
                    self.stop()
                    time.sleep(period)
                    continue

                if reached_total_limit:
                    status = "total_limit"
                    reason = (
                        "{} IR {} still blocked after "
                        "{:.3f} m total recovery"
                    ).format(
                        bumper.end,
                        side,
                        recovery_budget["traveled_m"],
                    )
                    raise MissionStop(reason)

                if reached_limit:
                    status = "limit"
                    reason = (
                        "{} IR {} still blocked "
                        "after one recovery attempt"
                    ).format(
                        bumper.end,
                        side,
                    )
                    return False

                longitudinal_clear, clearance_reason = (
                    self._longitudinal_ir_escape_clearance(
                        bumper,
                        forward,
                    )
                )

                mode = settings.get(
                    "recovery_mode",
                    "diagonal",
                )

                direction, escape_x, escape_y = (
                    recovery_vector(
                        sensors,
                        forward,
                        attempt=attempt,
                        end=bumper.end,
                        mode=mode,
                        forward_clear=(
                            longitudinal_clear is True
                        ),
                        movement_axis=movement_axis,
                    )
                )

                if mode == "adaptive":
                    vector, clearance_reason = (
                        self._adaptive_ir_escape(
                            bumper,
                            sensors,
                            forward,
                            attempt,
                            movement_axis,
                            longitudinal_clear,
                            clearance_reason,
                        )
                    )
                    direction, escape_x, escape_y = vector

                if direction == "waiting_clearance":
                    self.stop()
                    bumper.recovering = "waiting_clearance"
                    clear_count = 0
                    time.sleep(period)
                    continue

                if direction == "blocked":
                    if longitudinal_clear is None:
                        self.stop()
                        bumper.recovering = "waiting_clearance"
                        clear_count = 0
                        time.sleep(period)
                        continue

                    status = "blocked"
                    reason = (
                        "{} IR {} has no clear escape: {}"
                    ).format(
                        bumper.end,
                        side,
                        clearance_reason,
                    )
                    return False

                if direction == "clear":
                    self.stop()
                    time.sleep(period)
                    continue

                if (
                    escape_x != 0
                    and longitudinal_clear is not True
                ):
                    if longitudinal_clear is None:
                        self.stop()
                        bumper.recovering = "waiting_clearance"
                        clear_count = 0
                        time.sleep(period)
                        continue

                    status = "blocked"
                    reason = (
                        "{} IR {} longitudinal recovery "
                        "blocked: {}"
                    ).format(
                        bumper.end,
                        side,
                        clearance_reason,
                    )
                    return False

                opposite = (
                    self.front_ir
                    if bumper.end == "rear"
                    else self.rear_ir
                )

                if opposite is not None:
                    opp_blocked, opp_block_side = (
                        self._opposite_blocks_recovery_path(
                            opposite,
                            escape_x,
                            escape_y,
                            bumper.end,
                        )
                    )

                    if opp_blocked:
                        self.stop()

                        if opp_block_side == "waiting_data":
                            clear_count = 0
                            time.sleep(period)
                            continue

                        status = "blocked"
                        reason = (
                            "{} IR {} blocks {} recovery"
                        ).format(
                            opposite.end,
                            opp_block_side,
                            bumper.end,
                        )
                        return False

                bumper.recovering = direction

                self.chassis.drive_speed(
                    x=escape_x,
                    y=escape_y,
                    z=0,
                )

                self.commanded_lateral_m_s = escape_y

                # No speed * time accounting here.
                # Only changes in position consume the budget.

                time.sleep(period)

        except MissionStop as error:
            reason = str(error)
            raise

        finally:
            self.stop()

            bumper.finish_recovery(
                side,
                status,
                traveled_m,
                reason,
            )

    def move_to(self, x, y, yaw=None, timeout_s=None, abort_event=None,
                disable_timeout=False, stop_if=None, pause_if=None,
                on_ir_recovered=None):
        """Drive toward an absolute (x, y) waypoint; optionally face yaw.

        x/y use the chassis position frame established at subscription.
        With yaw=None, hold the first move's heading across later waypoints.
        stop_if receives each fresh pose before the PID command; returning True
        ends the move at that pose and the finally block stops the wheels.
        Missing telemetry pauses the wheels until fresh data returns. A caller
        supplied deadline still raises TimeoutError when progress never finishes.
        on_ir_recovered receives the episode's start pose, fresh cleared pose,
        and current target. It may return a replacement (x, y) target.
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
        recovery_state = {
            "front": {
                "attempts": 0,
                "traveled_m": 0.0,
            },
            "rear": {
                "attempts": 0,
                "traveled_m": 0.0,
            },
        }
        if self.directional_tof is not None:
            self.directional_tof.invalidate()
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
                travel_margin = None
                if self.directional_tof is not None and stop_if is None and math.hypot(x-pose[0], y-pose[1]) > self.settings["position_tolerance_m"]:
                    a = math.radians(pose[2])
                    dx, dy = x-pose[0], y-pose[1]
                    margin = self.directional_tof.clearance(dx*math.cos(a)+dy*math.sin(a), -dx*math.sin(a)+dy*math.cos(a))
                    if margin is None:
                        self.stop()
                        time.sleep(period)
                        continue
                    travel_margin = margin
                    pose = self.get_pose()
                    if pose is None:
                        self.stop()
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
                if self.directional_tof is not None:
                    blocked = []
                    for bumper in (self.front_ir, self.rear_ir):
                        if bumper is None:
                            raise MissionStop("directional recovery requires both front and rear IR")
                        sides = bumper.snapshot()["sides"]
                        hits = [k for k in ("left", "right") if sides[k]["detected"] is True]
                        missing = any(sides[k]["detected"] is None for k in ("left", "right"))
                        if missing or hits:
                            bumper.last_block = "waiting_data" if missing else "both" if len(hits) == 2 else hits[0]
                            blocked.append(bumper)
                if blocked:
                    self.stop()
                    bumper = next((item for item in blocked if item.last_block != "waiting_data"), None)
                    if bumper is not None:
                        side = bumper.last_block
                        state = recovery_state[bumper.end]
                        state.setdefault("start_pose", tuple(pose))
                        state["attempts"] += 1
                        max_attempts = bumper.settings.get("recovery_max_attempts", 1)
                        if state["attempts"] > max_attempts:
                            raise MissionStop(
                                "{} IR {} recovery exhausted after {} attempts".format(
                                    bumper.end, side, max_attempts))
                        recovered = self._recover_from_ir(
                            bumper, side, state["attempts"], period,
                            deadline, abort_event, pause_if,
                            movement_axis=("lateral" if abs(vy_robot) > abs(vx_robot)
                                           else "longitudinal"),
                            recovery_budget=state)
                        if recovered:
                            if on_ir_recovered is not None:
                                replacement = on_ir_recovered(
                                    state["start_pose"], state["cleared_pose"], (x, y))
                                if replacement is not None:
                                    x, y = replacement
                            state["attempts"] = 0
                            state["traveled_m"] = 0.0
                            state.pop("start_pose", None)
                            state.pop("cleared_pose", None)

                            # Start the next recovery episode
                            # with a new position baseline.
                            state.pop("last_pose", None)
                            state.pop(
                                "last_position_time", None
                            )
                            if timeout is not None:
                                deadline = time.monotonic() + timeout

                        elif state["attempts"] >= max_attempts:
                            raise MissionStop(
                                "{} IR {} recovery exhausted "
                                "after {} attempts and {:.3f} m"
                                .format(
                                    bumper.end,
                                    side,
                                    state["attempts"],
                                    state["traveled_m"],
                                )
                            )
                    for pid in (self.pid_x, self.pid_y, self.pid_yaw):
                        pid.reset()
                    previous_position = None
                    previous_position_time = None
                    approach_speed = 0.0
                    measured_speed = 0.0
                    previous = time.monotonic()
                    time.sleep(period)
                    continue
                if travel_margin is not None and travel_margin <= 0:
                    raise MissionStop("directional ToF stopped motion at safe edge distance")
                self.chassis.drive_speed(x=vx_robot, y=vy_robot, z=turn)
                self.commanded_lateral_m_s = vy_robot
                time.sleep(period)
            raise TimeoutError(f"waypoint ({x}, {y}, {yaw}) not reached within {timeout} s")
        finally:
            self.stop()
