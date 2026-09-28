"""Stationary wall inspection and bounded infrared shots per confirmed target."""

import math
import time

from src.mission_stop import MissionStop
from src.slam import _wrap_degrees
from src.targets import TargetTracker


class WallTargetInspector:
    def __init__(self, gimbal, blaster, camera_frames, logger, chassis,
                 slam_worker, settings, fire_type, on_progress=None, scan_gimbal=None):
        self.gimbal = gimbal
        self.scan_gimbal = scan_gimbal if scan_gimbal is not None else gimbal
        self.blaster = blaster
        self.camera_frames = camera_frames
        self.logger = logger
        self.chassis = chassis
        self.slam_worker = slam_worker
        self.settings = settings
        self.fire_type = fire_type
        self.on_progress = on_progress
        self.commanded_angles = None
        self.active_cell = None
        self.active_delta = None
        self.restoring = False
        self.active_action = None
        self.last_progress_status = None

    def _progress(self, cell, delta, status, target=None, **detail):
        if status != "stopped":
            self.last_progress_status = status
        if self.on_progress is not None:
            self.on_progress({"cell": list(cell), "direction": list(delta),
                              "status": status, "target": target, **detail})

    def _health(self):
        error = self.slam_worker.status().get("error")
        if error:
            raise MissionStop(error)
        camera_error = getattr(self.camera_frames, "camera_error", None)
        if camera_error is not None and not self.restoring:
            raise MissionStop(f"camera stopped during target inspection: {camera_error or 'unknown error'}")

    def _angles(self, pitch_frame="ground"):
        while True:
            self._health()
            sample = self.logger.get_sample("gimbal", max_age_s=self.settings["max_sample_age_s"])
            if sample is not None and len(sample[0]) >= 3:
                try:
                    pitch_index = 2 if pitch_frame == "ground" else 0
                    pitch, yaw = float(sample[0][pitch_index]), float(sample[0][1])
                    if math.isfinite(pitch) and math.isfinite(yaw):
                        return pitch, yaw, sample[1]
                except (TypeError, ValueError):
                    pass
            time.sleep(.03)

    def _safe_status(self):
        """Wait for fresh safety telemetry while the wheels remain stopped."""
        while True:
            self._health()
            samples = [self.logger.get_sample(name, max_age_s=self.settings["max_sample_age_s"])
                       for name in ("position", "attitude", "status")]
            if (all(sample is not None for sample in samples) and
                    max(sample[1] for sample in samples) - min(sample[1] for sample in samples)
                    <= self.settings["sample_skew_s"] and len(samples[2][0]) >= 10):
                sample = samples[2]
                for index in (4, 5, 6, 7, 8, 9):
                    if sample[0][index] not in (0, False, None):
                        raise MissionStop(f"robot safety status flag {index} is active")
                return
            self.chassis.stop()
            time.sleep(.05)

    def _point(self, pitch, yaw, pitch_frame="ground", recenter=False):
        """Wait for both SDK action release and fresh angle telemetry."""
        if self.active_cell is not None:
            self._progress(self.active_cell, self.active_delta, "waiting_target_safety")
        self._safe_status()
        request_time = time.time()
        try:
            gimbal = self.gimbal if pitch_frame == "ground" else self.scan_gimbal
            if recenter:
                action = gimbal.recenter(
                    pitch_speed=30,
                    yaw_speed=self.settings["gimbal"]["yaw_speed_deg_s"])
            else:
                action = gimbal.moveto(
                    pitch=pitch, yaw=yaw, pitch_speed=30,
                    yaw_speed=self.settings["gimbal"]["yaw_speed_deg_s"])
        except Exception as error:
            raise MissionStop(f"target gimbal command failed: {error}") from error
        self.active_action = action
        released = action is None
        reported_state = None
        reported_angle_state = None
        while True:
            self._health()
            state = getattr(action, "state", None)
            if state in ("action_failed", "action_rejected", "action_exception", "action_aborted"):
                self.active_action = None
                raise MissionStop(f"target gimbal action failed: {state}")
            if not released and getattr(action, "has_succeeded", False):
                wait = getattr(action, "wait_for_completed", None)
                if callable(wait):
                    try:
                        completed = wait()
                    except Exception as error:
                        raise MissionStop(f"target gimbal action release failed: {error}") from error
                    if not completed:
                        raise MissionStop("target gimbal action was not released by SDK")
                released = True
                self.active_action = None
            waiting_state = "waiting_target_angle" if released else "waiting_target_action"
            if waiting_state != reported_state and self.active_cell is not None:
                self._progress(self.active_cell, self.active_delta, waiting_state,
                               pitch_frame=pitch_frame,
                               target_pitch_deg=pitch,
                               target_yaw_deg=yaw,
                               action_state=state)
                reported_state = waiting_state
            measured_pitch, measured_yaw, timestamp = self._angles(pitch_frame)
            if waiting_state != reported_angle_state and self.active_cell is not None:
                self._progress(self.active_cell, self.active_delta, waiting_state,
                               pitch_frame=pitch_frame,
                               target_pitch_deg=pitch,
                               actual_pitch_deg=round(measured_pitch, 2),
                               target_yaw_deg=yaw, actual_yaw_deg=round(measured_yaw, 2),
                               action_state=state)
                reported_angle_state = waiting_state
            if (released and timestamp > request_time and
                    abs(measured_pitch - pitch) <= self.settings["gimbal"]["pitch_tolerance_deg"] and
                    abs(_wrap_degrees(measured_yaw - yaw)) <=
                    self.settings["gimbal"]["angle_tolerance_deg"]):
                self.commanded_angles = (pitch, yaw)
                return measured_pitch, measured_yaw
            time.sleep(.03)

    def _frame(self):
        after = self.camera_frames.current_frame_number()
        if self.active_cell is not None:
            self._progress(self.active_cell, self.active_delta, "waiting_camera_frame",
                           after_frame=after)
        return self.camera_frames.wait_for_frame(after, check_health=self._health)

    def _fire(self, item, frame, aim_mode):
        self._safe_status()
        pitch, yaw, _ = self._angles()
        expected_pitch, expected_yaw = self.commanded_angles
        if (abs(pitch - expected_pitch) > self.settings["gimbal"]["pitch_tolerance_deg"] or
                abs(_wrap_degrees(yaw - expected_yaw)) >
                self.settings["gimbal"]["angle_tolerance_deg"]):
            raise MissionStop("target gimbal drifted before infrared fire")
        self._health()
        try:
            accepted = self.blaster.fire(fire_type=self.fire_type, times=2)
        except Exception as error:
            raise MissionStop(
                f"infrared fire command failed; firing state unknown: {error}") from error
        if not accepted:
            raise MissionStop("infrared fire command was not accepted; firing state unknown")
        # The SDK reports command acceptance, not the end of the two-shot burst.
        # Hold the chassis and gimbal for the configured firing interval.
        hold_s = self.settings["target_inspection"]["fire_hold_s"]
        self._progress(self.active_cell, self.active_delta, "waiting_infrared_fire",
                       target={"color": item.color, "shape": item.shape},
                       shots_requested=2, hold_s=hold_s)
        end = time.monotonic() + hold_s
        while time.monotonic() < end:
            self._health()
            self.chassis.stop()
            time.sleep(min(.05, max(0, end - time.monotonic())))
        height, width = frame.shape[:2]
        return {"status": "fire_command_accepted", "color": item.color,
                "shape": item.shape, "area_fraction": round(item.area / (width * height), 4),
                "center": list(item.center), "shots_requested": 2, "aim_mode": aim_mode,
                "fire_hold_s": hold_s}

    def _confirm_and_fire(self, track_id, tracker, frame, item, mode):
        for _ in range(self.settings["target_inspection"]["lock_frames"] - 1):
            _, frame = self._frame()
            item = tracker.update(frame).get(track_id)
            if item is None:
                return {"status": "target_lost"}
        return self._fire(item, frame, mode)

    def _aim(self, track_id, tracker, first_frame, first_item):
        config = self.settings["target_inspection"]
        frame = first_frame
        lost = 0
        moves = 0
        first = True
        best_error = math.inf
        best_angles = None
        while True:
            visible = ({track_id: first_item} if first else tracker.update(frame))
            first = False
            item = visible.get(track_id)
            if item is None:
                lost += 1
                if lost >= 3:
                    return {"status": "target_lost"}
                _, frame = self._frame()
                continue
            lost = 0
            height, width = frame.shape[:2]
            error_x = item.center[0] / width - (.5 + config["aim_offset_x_fraction"])
            error_y = (.5 + config["aim_offset_y_fraction"]) - item.center[1] / height
            pitch, yaw, _ = self._angles()
            error = math.hypot(error_x, error_y)
            if error < best_error:
                best_error, best_angles = error, (pitch, yaw)
            if error <= config["center_radius_fraction"]:
                return self._confirm_and_fire(track_id, tracker, frame, item, "centered")
            step = config["max_step_deg"]
            yaw_step = max(-step, min(step, error_x * config["camera_hfov_deg"]))
            pitch_step = max(-step, min(step, error_y * config["camera_vfov_deg"]))
            next_yaw = max(-250, min(250, yaw + yaw_step))
            next_pitch = max(-20, min(20, pitch + pitch_step))
            if (moves >= config["max_aim_steps"] or
                    (abs(next_yaw - yaw) < .1 and abs(next_pitch - pitch) < .1)):
                if (abs(best_angles[0] - pitch) >= .1 or
                        abs(_wrap_degrees(best_angles[1] - yaw)) >= .1):
                    self._point(*best_angles)
                    _, frame = self._frame()
                    item = tracker.update(frame).get(track_id)
                    if item is None:
                        return {"status": "target_lost"}
                return self._confirm_and_fire(track_id, tracker, frame, item,
                                              "closest_reachable")
            self._point(next_pitch, next_yaw)
            moves += 1
            _, frame = self._frame()

    def inspect(self, cell, delta, world_yaw, body_yaw):
        self.active_cell, self.active_delta = cell, delta
        config = self.settings["target_inspection"]
        selected = None if config["selected"] == "all" else {
            tuple(item.split(":")) for item in config["selected"]}
        result = {"cell": list(cell), "direction": list(delta),
                  "pitch_deg": config["pitch_deg"], "status": "checking",
                  "targets": [], "checked_at": time.time()}
        self.chassis.stop()
        _, original_yaw, _ = self._angles()
        target_yaw = _wrap_degrees(world_yaw - body_yaw)
        candidates = (target_yaw - 360, target_yaw, target_yaw + 360)
        reachable = [value for value in candidates if -250 <= value <= 250]
        if not reachable:
            raise MissionStop("wall camera cannot reach target yaw")
        inspection_yaw = min(reachable, key=lambda value: abs(value - original_yaw))
        self.slam_worker.pause_mapping()
        try:
            self._progress(cell, delta, "pointing")
            self._point(config["pitch_deg"], inspection_yaw)
            tracker = TargetTracker(config["min_area_fraction"], selected)
            attempted = []
            for _ in range(config["max_targets_per_wall"]):
                self._progress(cell, delta, "searching")
                visible = {}
                frame = None
                for _ in range(config["confirm_frames"]):
                    _, frame = self._frame()
                    visible = tracker.update(frame)
                height, width = frame.shape[:2]
                diagonal = math.hypot(width, height)
                ready = [(track_id, item) for track_id, item in visible.items()
                         if tracker.tracks[track_id].consecutive >= config["confirm_frames"] and
                         not any(item.color == color and item.shape == shape and
                                 math.hypot(item.center[0] - center[0],
                                            item.center[1] - center[1]) / diagonal <= .06
                                 for color, shape, center in attempted)]
                if not ready:
                    break
                track_id, item = max(ready, key=lambda pair: pair[1].area)
                attempted.append((item.color, item.shape, item.center))
                self._progress(cell, delta, "aiming", {"color": item.color, "shape": item.shape})
                outcome = self._aim(track_id, tracker, frame, item)
                outcome.setdefault("color", item.color)
                outcome.setdefault("shape", item.shape)
                result["targets"].append(outcome)
                if len(result["targets"]) < config["max_targets_per_wall"]:
                    self._point(config["pitch_deg"], inspection_yaw)
            result["status"] = "targets_checked" if result["targets"] else "no_target"
            return result
        finally:
            restored = False
            inspection_wait = self.last_progress_status
            try:
                self.restoring = True
                if self.active_action is None:
                    self._progress(cell, delta, "restoring_scan_angle")
                    # The SDK center action proved reliable before DFS scans.
                    # The next scan chooses its own yaw, so restoring the old
                    # wall-facing yaw is unnecessary.
                    self._point(0, 0, pitch_frame="chassis", recenter=True)
                    restored = True
            finally:
                self.restoring = False
                self.slam_worker.resume_mapping()
                self._progress(cell, delta, result["status"] if restored and
                               result["status"] != "checking" else "stopped",
                               interrupted_from=(self.last_progress_status if not restored else
                                                 inspection_wait if result["status"] == "checking" else None),
                               action_state=getattr(self.active_action, "state", None))
                self.active_cell = self.active_delta = None
