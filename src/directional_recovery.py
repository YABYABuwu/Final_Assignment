"""Four-sensor escape policy and shared, direction-checked ToF clearance."""
import math
import time

from src.mission_stop import MissionStop


def edge_margin(owner, reading_m, relative_yaw, body_yaw):
    angle = math.radians(relative_yaw)
    vx, vy = math.cos(angle), math.sin(angle)
    offset = .05 if vy > 0 and abs(vy) > abs(vx) else .06 if vy < 0 and abs(vy) > abs(vx) else 0
    safe_edge = max(0, owner.settings["emergency_stop_distance_m"] - owner._sensor_offset(body_yaw))
    return reading_m + offset - safe_edge


def escape_direction(front, rear):
    """SDK chassis axes: +x forward, +y right."""
    fl, fr = front
    rl, rr = rear
    if any(value is None for value in (fl, fr, rl, rr)):
        return "waiting_data", 0, 0
    if not any((fl, fr, rl, rr)):
        return "clear", 0, 0
    if rl and rr and not (fl or fr):
        return "forward", 1, 0
    if fl and fr and not (rl or rr):
        return "backward", -1, 0
    if (fl or rl) and not (fr or rr):
        return "slide_right", 0, 1
    if (fr or rr) and not (fl or rl):
        return "slide_left", 0, -1
    return "blocked", 0, 0


class DirectionalToF:
    """Reuse the owner's SDK completion/median scan, then keep the head there."""
    def __init__(self, owner):
        self.owner = owner
        self.world_yaw = None
        self.requested_at = 0

    def invalidate(self):
        self.world_yaw = None

    def clearance(self, vx, vy):
        owner = self.owner
        health = owner.slam_worker.status() if owner.slam_worker is not None else {}
        if health.get("error"):
            raise MissionStop(health["error"])
        if health.get("waiting_telemetry"):
            return None
        pose = owner.chassis.get_pose()
        if pose is None:
            return None
        relative = math.degrees(math.atan2(vy, vx))
        world = pose[2] + relative
        angles = owner.logger.get_sample("gimbal", max_age_s=owner.settings["max_sample_age_s"])
        aligned = (angles is not None and len(angles[0]) >= 2 and
                   abs((angles[0][1] - relative + 180) % 360 - 180) <=
                   owner.settings["gimbal"]["angle_tolerance_deg"] and
                   abs(angles[0][0] - owner.settings["gimbal"]["pitch_deg"]) <=
                   owner.settings["gimbal"]["pitch_tolerance_deg"])
        if (self.world_yaw is None or
                abs((world - self.world_yaw + 180) % 360 - 180) > 3 or not aligned):
            owner.chassis.stop()
            a = math.radians(world - owner.base_pose[2])
            self.requested_at = time.time()
            owner._scan_for_direction((math.cos(a), math.sin(a)))
            self.world_yaw = world
        tof = owner.logger.get_sample("tof", max_age_s=owner.settings["max_sample_age_s"])
        angles = owner.logger.get_sample("gimbal", max_age_s=owner.settings["max_sample_age_s"])
        if tof is None or angles is None:
            return None
        if any(not math.isfinite(s[1]) or not 0 <= time.time()-s[1] <= owner.settings["max_sample_age_s"] for s in (tof, angles)):
            return None
        if len(angles[0]) < 2 or abs(angles[0][0] - owner.settings["gimbal"]["pitch_deg"]) > owner.settings["gimbal"]["pitch_tolerance_deg"]:
            return None
        if (tof[1] <= self.requested_at or abs(tof[1] - angles[1]) > owner.settings["sample_skew_s"] or
                abs((angles[0][1] - relative + 180) % 360 - 180) > owner.settings["gimbal"]["angle_tolerance_deg"]):
            return None
        try:
            reading = float(tof[0][owner.settings["sensor"]["tof_channel"]]) / 1000
        except (IndexError, TypeError, ValueError):
            return None
        if not math.isfinite(reading) or reading <= 0 or reading == 65.535:
            return None
        # Calibrated distances from the robot edge, not the yaw pivot.
        return edge_margin(owner, reading, relative, pose[2])


def recover(controller, bumper, side, period, deadline, abort_event, pause_if, budget):
    """One episode shares eight steps / 16 cm across front and rear sensors."""
    guard = controller.directional_tof
    if guard is None:
        raise MissionStop("directional IR recovery needs a gimbal ToF controller")
    steps, traveled, step_distance, stagnant = 0, 0., 0., 0
    previous = None
    previous_stamp = None
    direction = None
    clear_samples = 0
    last_ir_stamp = None
    status, reason = "aborted", None
    moving = False
    original = controller.logger.get_sample("gimbal", max_age_s=guard.owner.settings["max_sample_age_s"])
    original_yaw = original[0][1] if original is not None else guard.owner._gimbal_sample()[0]
    guard.invalidate()
    try:
        while True:
            if abort_event is not None and abort_event.is_set():
                raise MissionStop("directional IR recovery aborted")
            if deadline is not None and time.monotonic() >= deadline:
                raise MissionStop("directional IR recovery exceeded waypoint time")
            if pause_if is not None and pause_if():
                controller.stop()
                clear_samples = 0
                moving = False
                time.sleep(period)
                continue
            states = [b.snapshot() if b is not None else None for b in (controller.front_ir, controller.rear_ir)]
            if any(s is None or any(s["sides"][k]["detected"] is None for k in ("left", "right")) for s in states):
                controller.stop()
                bumper.recovering = "waiting_data"
                clear_samples = 0
                moving = False
                time.sleep(period)
                continue
            selected, dx, dy = escape_direction(*[tuple(s["sides"][k]["detected"] for k in ("left", "right")) for s in states])
            pose = controller.get_pose()
            stamp = controller.position_sample_time
            if pose is None or stamp is None:
                controller.stop()
                clear_samples = 0
                moving = False
                time.sleep(period)
                continue
            if stamp != previous_stamp:
                if previous is not None:
                    distance = math.hypot(pose[0] - previous[0], pose[1] - previous[1])
                    traveled += distance
                    step_distance += distance
                    stagnant = stagnant + 1 if moving and distance < .0001 else 0
                previous, previous_stamp = pose, stamp
            budget["traveled_m"] = traveled
            ir_stamp = tuple(s.get("sample_time") for s in states)
            if selected == "clear":
                controller.stop()
                if all(t is not None for t in ir_stamp) and (last_ir_stamp is None or all(t > p for t,p in zip(ir_stamp,last_ir_stamp))):
                    clear_samples += 1
                    last_ir_stamp = ir_stamp
                if clear_samples >= bumper.settings["recovery_clear_samples"]:
                    budget["cleared_pose"] = tuple(pose)
                    for b in (controller.front_ir, controller.rear_ir):
                        b.last_block = None
                    if original_yaw is not None:
                        guard.invalidate()
                        angle = math.radians(original_yaw)
                        while guard.clearance(math.cos(angle), math.sin(angle)) is None:
                            controller.stop()
                            if abort_event is not None and abort_event.is_set():
                                raise MissionStop("IR recovery aborted while restoring travel direction")
                            if deadline is not None and time.monotonic() >= deadline:
                                raise MissionStop("IR recovery exceeded waypoint time while restoring travel direction")
                            time.sleep(period)
                    status = "cleared"
                    return True
                time.sleep(period)
                continue
            clear_samples = 0
            if selected == "blocked":
                raise MissionStop("IR conflict: crossed sensors or escape end also blocked")
            if traveled >= .16:
                raise MissionStop("IR recovery reached 0.16 m total measured travel")
            if stagnant >= 8:
                raise MissionStop("IR recovery made no progress over 8 fresh position samples")
            if direction != selected or step_distance >= .02:
                controller.stop()
                moving = False
                if steps >= 8:
                    raise MissionStop("IR recovery exhausted 8 steps")
                direction = selected
                step_distance = 0.
                steps += 1
                bumper.recovering = "recheck_after_four" if steps == 5 else direction
                # Every step, including extensions after four, requires new range.
                guard.requested_at = time.time()
            margin = guard.clearance(dx, dy)
            if margin is None:
                controller.stop()
                moving = False
                bumper.recovering = "waiting_clearance"
                time.sleep(period)
                continue
            if margin < max(0, .02 - step_distance):
                raise MissionStop("ToF has insufficient edge clearance for {} 2 cm escape".format(direction))
            bumper.recovering = direction
            speed = min(bumper.settings["recovery_speed_m_s"], .04,
                        max(.005, (.02 - step_distance) * 2))
            controller.chassis.drive_speed(x=dx * speed, y=dy * speed, z=0)
            moving = True
            controller.commanded_lateral_m_s = dy * speed
            time.sleep(period)
    except MissionStop as error:
        reason = str(error)
        raise
    finally:
        controller.stop()
        guard.invalidate()
        bumper.finish_recovery(side, status, traveled,
                               "steps={}; reassessed_after_four={}; {}".format(steps, steps > 4, reason or "all four IR clear"))
