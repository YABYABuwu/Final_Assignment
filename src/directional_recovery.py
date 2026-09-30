"""Four-sensor escape policy and shared, direction-checked ToF clearance."""
import math
import time
from collections import deque

from src.mission_stop import MissionStop


RECOVERY_FLOOR_SPEED_M_S = .03
PROGRESS_WINDOW_SAMPLES = 20
MIN_WINDOW_PROGRESS_M = .002


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


def tof_escape_direction(front, rear, guard, on_probe=None):
    """Disambiguate a one-sided IR hit using ToF at the contacted end."""
    selected = escape_direction(front, rear)
    checks = {}
    if selected[0] not in ("slide_left", "slide_right"):
        return selected, checks
    for end, hits, opposite, vx in (("front", front, rear, 1), ("rear", rear, front, -1)):
        if not any(hits):
            continue
        if on_probe is not None:
            on_probe(end)
        margin = guard.clearance(vx, 0)
        if margin is None:
            return ("waiting_clearance", 0, 0), checks
        checks[end + "_margin_m"] = round(margin, 4)
        if margin <= 0:
            # Never reverse toward an end already reporting an IR contact.
            if any(opposite):
                return ("blocked", 0, 0), checks
            return (("backward", -1, 0) if end == "front" else ("forward", 1, 0)), checks
    return selected, checks


class DirectionalToF:
    """Reuse the owner's SDK completion/median scan, then keep the head there."""
    def __init__(self, owner):
        self.owner = owner
        self.world_yaw = None
        self.requested_at = 0
        self.motion_interrupted = False

    def invalidate(self):
        self.world_yaw = None

    def clearance(self, vx, vy):
        self.motion_interrupted = False
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
            self.motion_interrupted = True
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
    steps, traveled, step_distance = 0, 0., 0.
    progress_distances = deque(maxlen=PROGRESS_WINDOW_SAMPLES)
    previous = None
    previous_stamp = None
    direction = None
    contact_pattern = None
    contact_decision = None
    bumper.recovery_contact_tof = None
    clear_samples = 0
    last_ir_stamp = None
    status, reason = "aborted", None
    moving = False
    moving_since = None
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
            pattern = tuple(tuple(s["sides"][k]["detected"] for k in ("left", "right")) for s in states)
            selected, dx, dy = escape_direction(*pattern)
            pose = controller.get_pose()
            stamp = controller.position_sample_time
            if pose is None or stamp is None:
                controller.stop()
                clear_samples = 0
                moving = False
                time.sleep(period)
                continue
            if previous_stamp is None or stamp > previous_stamp:
                if previous is not None:
                    distance = math.hypot(pose[0] - previous[0], pose[1] - previous[1])
                    traveled += distance
                    step_distance += distance
                    # Both endpoints must follow a real drive command. Pauses
                    # and head scans do not consume the progress window.
                    if moving and previous_stamp > moving_since:
                        progress_distances.append(distance)
                previous, previous_stamp = pose, stamp
            budget["traveled_m"] = traveled
            ir_stamp = tuple(s.get("sample_time") for s in states)
            if selected == "clear":
                controller.stop()
                moving = False
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
            if (len(progress_distances) == PROGRESS_WINDOW_SAMPLES and
                    sum(progress_distances) < MIN_WINDOW_PROGRESS_M):
                raise MissionStop(
                    "IR recovery made no progress over {} fresh moving position samples "
                    "(accumulated {:.1f} mm, minimum {:.1f} mm)".format(
                        PROGRESS_WINDOW_SAMPLES, sum(progress_distances) * 1000,
                        MIN_WINDOW_PROGRESS_M * 1000))
            if pattern != contact_pattern or (step_distance >= .02 and contact_decision is not None):
                contact_pattern = pattern
                contact_decision = None
                # Reclassify once per 2 cm step or IR-pattern change, not
                # every control tick (which would keep swinging the head).
                guard.invalidate()
            if contact_decision is None:
                controller.stop()
                moving = False
                decision, checks = tof_escape_direction(
                    *pattern, guard,
                    on_probe=lambda end: setattr(bumper, "recovering", "checking_" + end + "_tof"))
                bumper.recovery_contact_tof = {**checks, "decision": decision[0]}
                if decision[0] == "waiting_clearance":
                    bumper.recovering = "waiting_contact_tof"
                    time.sleep(period)
                    continue
                # Do not decide from an IR pattern that changed during the scan.
                checked_states = [b.snapshot() for b in (controller.front_ir, controller.rear_ir)]
                checked_pattern = tuple(tuple(s["sides"][k]["detected"] for k in ("left", "right"))
                                        for s in checked_states)
                if checked_pattern != pattern:
                    contact_pattern = None
                    time.sleep(period)
                    continue
                contact_decision = decision
            selected, dx, dy = contact_decision
            if selected == "blocked":
                raise MissionStop("ToF confirms an end obstacle but the opposite IR end is also blocked")
            if direction != selected or step_distance >= .02:
                controller.stop()
                moving = False
                if steps >= 8:
                    raise MissionStop("IR recovery exhausted 8 steps")
                if direction != selected:
                    progress_distances.clear()
                direction = selected
                step_distance = 0.
                steps += 1
                bumper.recovering = "recheck_after_four" if steps == 5 else direction
                # Every step, including extensions after four, requires new range.
                guard.requested_at = time.time()
            margin = guard.clearance(dx, dy)
            if getattr(guard, "motion_interrupted", False) is True:
                # clearance() may stop wheels while it blocks on a gimbal scan.
                moving = False
            if margin is None:
                controller.stop()
                moving = False
                bumper.recovering = "waiting_clearance"
                time.sleep(period)
                continue
            # The head scan can take time. Re-read shared IR before driving;
            # a changed or missing pattern must be classified again at rest.
            current_states = [b.snapshot() for b in (controller.front_ir, controller.rear_ir)]
            current_pattern = tuple(tuple(s["sides"][k]["detected"] for k in ("left", "right"))
                                    for s in current_states)
            if current_pattern != pattern:
                controller.stop()
                moving = False
                contact_pattern = None
                contact_decision = None
                time.sleep(period)
                continue
            if margin < max(0, .02 - step_distance):
                raise MissionStop("ToF has insufficient edge clearance for {} 2 cm escape".format(direction))
            bumper.recovering = direction
            speed = min(bumper.settings["recovery_speed_m_s"], .04,
                        max(RECOVERY_FLOOR_SPEED_M_S, (.02 - step_distance) * 2))
            if not moving:
                moving_since = time.time()
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
                               "steps={}; reassessed_after_four={}; contact_tof={}; {}".format(
                                   steps, steps > 4, getattr(bumper, "recovery_contact_tof", None),
                                   reason or "all four IR clear"))
