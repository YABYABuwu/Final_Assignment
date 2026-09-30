"""Stationary wall inspection and bounded infrared shots per confirmed target."""

import math
import time

from src.mission_stop import MissionStop
from src.slam import _wrap_degrees
from src.targets import TargetTracker


def _nearest_candidate(visible, color, shape, center):
    """Reacquire only the exact color and shape nearest its last position."""
    candidates = [
        (tid, detection) for tid, detection in visible.items()
        if detection.color == color and detection.shape == shape
    ]
    if not candidates:
        return None, None
    return min(
        candidates,
        key=lambda pair: (
            math.hypot(pair[1].center[0] - center[0], pair[1].center[1] - center[1]),
            -pair[1].area,
        ),
    )


class WallTargetInspector:
    def __init__(self, gimbal, blaster, camera_frames, logger, chassis,
                 slam_worker, settings, fire_type, on_progress=None, scan_gimbal=None,
                 color_ranges=None):
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
        self.color_ranges = color_ranges
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

    def _point(self, pitch, yaw, pitch_frame="ground", recenter=False, yaw_speed=None):
        """Wait for both SDK action release and fresh angle telemetry."""
        if self.active_cell is not None:
            self._progress(self.active_cell, self.active_delta, "waiting_target_safety")
        self._safe_status()
        request_time = time.time()
        try:
            gimbal = self.gimbal if pitch_frame == "ground" else self.scan_gimbal
            target_cfg = self.settings.get("target_inspection", {})
            if yaw_speed is None:
                yaw_speed = target_cfg.get(
                    "scan_yaw_speed_deg_s",
                    self.settings.get("gimbal", {}).get("yaw_speed_deg_s", 60)
                )
            if recenter:
                action = gimbal.recenter(
                    pitch_speed=30,
                    yaw_speed=yaw_speed)
            else:
                action = gimbal.moveto(
                    pitch=pitch, yaw=yaw, pitch_speed=30,
                    yaw_speed=yaw_speed)
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
                settle_s = self.settings.get("target_inspection", {}).get("aim_settle_s", 0.0)
                if settle_s > 0:
                    time.sleep(settle_s)
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
        config = self.settings.get("target_inspection", {})
        shots = config.get("shots_per_target", 2)
        self._health()
        try:
            accepted = self.blaster.fire(fire_type=self.fire_type, times=shots)
        except Exception as error:
            raise MissionStop(
                f"infrared fire command failed; firing state unknown: {error}") from error
        if not accepted:
            raise MissionStop("infrared fire command was not accepted; firing state unknown")

        # Hold gimbal steady and pause so all physical shots finish before moving the gimbal
        settle_s = config.get("fire_settle_s", 1.0)
        if settle_s > 0:
            time.sleep(settle_s)

        height, width = frame.shape[:2]
        return {"status": "fire_command_accepted", "color": item.color,
                "shape": item.shape, "area_fraction": round(item.area / (width * height), 4),
                "center": list(item.center), "shots_requested": shots, "aim_mode": aim_mode}

    def _confirm_and_fire(self, track_id, tracker, frame, item, mode):
        lock_frames = self.settings.get("target_inspection", {}).get("lock_frames", 3)
        max_misses = self.settings.get("target_inspection", {}).get("lock_max_misses", 3)
        target_color = item.color
        target_shape = item.shape
        confirmed_item = item
        exact_hits = 1
        misses = 0

        while exact_hits < lock_frames and misses <= max_misses:
            _, frame = self._frame()
            visible = tracker.update(frame)
            current_item = visible.get(track_id)
            if current_item is None:
                new_track_id, current_item = _nearest_candidate(
                    visible, target_color, target_shape, confirmed_item.center
                )
                if current_item is not None:
                    track_id = new_track_id
            if current_item is None:
                misses += 1
            else:
                confirmed_item = current_item
                exact_hits += 1

        if exact_hits < lock_frames:
            return {"status": "target_lost",
                    "reason": "exact color and shape not stable during fire lock",
                    "exact_hits": exact_hits, "required_hits": lock_frames}

        return self._fire(confirmed_item, frame, mode)

    def _retry_best_angle(self, track_id, tracker, best_angles, color, shape,
                          last_center, aim_speed):
        """Return to the best exact-shape view and reconfirm before firing."""
        if best_angles is None:
            return None
        self._point(*best_angles, yaw_speed=aim_speed)
        attempts = self.settings.get("target_inspection", {}).get("reacquire_frames", 6)
        for _ in range(attempts):
            _, frame = self._frame()
            visible = tracker.update(frame)
            item = visible.get(track_id)
            if item is None or item.color != color or item.shape != shape:
                track_id, item = _nearest_candidate(
                    visible, color, shape, last_center
                )
            if item is not None:
                return self._confirm_and_fire(
                    track_id, tracker, frame, item, "closest_reachable"
                )
        return None

    def _aim(self, track_id, tracker, first_frame, first_item):
        config = self.settings["target_inspection"]
        frame = first_frame
        lost = 0
        moves = 0
        first = True
        best_error = math.inf
        best_angles = None
        target_color = first_item.color
        target_shape = first_item.shape
        max_lost = config.get("target_lost_frames", 5)
        aim_speed = config.get("aim_yaw_speed_deg_s", 30)

        # Method 1: Lock-on & Blind Aim (Open-loop Fire)
        # Calculates firing angles directly from confirmed target detection,
        # points the gimbal, and fires without visual re-verification during/after tilt
        # (resolves physical occlusion where gimbal tip blocks target upon tilting up).
        if config.get("blind_aim", False):
            height, width = first_frame.shape[:2]
            error_x = first_item.center[0] / width - (.5 + config.get("aim_offset_x_fraction", 0.0))
            error_y = (.5 + config.get("aim_offset_y_fraction", 0.0)) - first_item.center[1] / height
            pitch, yaw, _ = self._angles()
            yaw_step = error_x * config.get("camera_hfov_deg", 90.0)
            pitch_step = error_y * config.get("camera_vfov_deg", 60.0)
            target_yaw = round(max(-250.0, min(250.0, yaw + yaw_step)), 2)
            target_pitch = round(max(-20.0, min(20.0, pitch + pitch_step)), 2)
            if abs(target_yaw - yaw) >= .1 or abs(target_pitch - pitch) >= .1:
                self._point(target_pitch, target_yaw, yaw_speed=aim_speed)
            return self._fire(first_item, first_frame, aim_mode="blind_aim")

        while True:
            visible = ({track_id: first_item} if first else tracker.update(frame))
            first = False
            item = visible.get(track_id)
            if item is None:
                track_id, item = _nearest_candidate(
                    visible, target_color, target_shape, first_item.center
                )
            if item is None:
                lost += 1
                if lost >= max_lost:
                    retry = self._retry_best_angle(
                        track_id, tracker, best_angles, target_color,
                        target_shape, first_item.center, aim_speed
                    )
                    if retry is not None:
                        return retry
                    return {"status": "target_lost",
                            "reason": "exact color and shape missing during aiming",
                            "lost_frames": lost}
                _, frame = self._frame()
                continue
            lost = 0
            first_item = item
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
                    self._point(*best_angles, yaw_speed=aim_speed)
                    _, frame = self._frame()
                    visible = tracker.update(frame)
                    item = visible.get(track_id)
                    if item is None:
                        track_id, item = _nearest_candidate(
                            visible, target_color, target_shape, first_item.center
                        )
                    if item is None:
                        return {"status": "target_lost",
                                "reason": "target missing at closest reachable angle"}
                return self._confirm_and_fire(track_id, tracker, frame, item,
                                              "closest_reachable")
            self._point(next_pitch, next_yaw, yaw_speed=aim_speed)
            moves += 1
            _, frame = self._frame()

    def inspect(self, cell, delta, world_yaw, body_yaw, range_mm=None, initial_pitch=None):
        self.active_cell, self.active_delta = cell, delta
        config = self.settings["target_inspection"]
        selected = None if config["selected"] == "all" else {
            tuple(item.split(":")) for item in config["selected"]}

        if initial_pitch is None:
            initial_pitch = config["pitch_deg"]
            if config.get("adaptive_pitch", True):
                if range_mm is None:
                    sample = self.logger.get_sample("tof", max_age_s=self.settings["max_sample_age_s"])
                    if sample is not None and len(sample[0]) > 0:
                        try:
                            range_mm = float(sample[0][0])
                        except (TypeError, ValueError, IndexError):
                            range_mm = None
                if range_mm is not None and 0 < range_mm <= config.get("close_range_threshold_mm", 180):
                    initial_pitch = config.get("close_pitch_deg", -10.0)

        result = {"cell": list(cell), "direction": list(delta),
                  "pitch_deg": initial_pitch, "status": "checking",
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
            scan_speed = config.get("scan_yaw_speed_deg_s", 50)
            self._point(initial_pitch, inspection_yaw, yaw_speed=scan_speed)
            tracker = TargetTracker(config["min_area_fraction"], selected,
                                    self.color_ranges)
            attempted = []
            confirm_frames = config["confirm_frames"]
            search_frames = config.get("search_frames", confirm_frames + 2)
            for _ in range(config["max_targets_per_wall"]):
                self._progress(cell, delta, "searching")
                visible = {}
                frame = None
                for _ in range(search_frames):
                    _, frame = self._frame()
                    visible = tracker.update(frame)
                    if any(t.consecutive >= confirm_frames for t in tracker.tracks.values()):
                        break
                if frame is None:
                    break
                height, width = frame.shape[:2]
                diagonal = math.hypot(width, height)
                ready = [(track_id, item) for track_id, item in visible.items()
                         if tracker.tracks[track_id].consecutive >= confirm_frames and
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
                    self._point(initial_pitch, inspection_yaw, yaw_speed=scan_speed)
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
