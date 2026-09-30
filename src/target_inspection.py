"""Stationary wall inspection and bounded infrared shots per confirmed target."""

import math
import time

from src.mission_stop import MissionStop
from src.slam import _wrap_degrees
from src.targets import TargetTracker
from src.gimbal_control import wait_for_gimbal_idle


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
            math.hypot(
                pair[1].center[0] - center[0],
                pair[1].center[1] - center[1],
            ),
            -pair[1].area,
        ),
    )


class WallTargetInspector:
    def __init__(
        self,
        gimbal,
        blaster,
        camera_frames,
        logger,
        chassis,
        slam_worker,
        settings,
        fire_type,
        on_progress=None,
        scan_gimbal=None,
        color_ranges=None,
    ):
        self.gimbal = gimbal
        self.scan_gimbal = (
            scan_gimbal if scan_gimbal is not None else gimbal
        )
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
        self.wait_error = None
        self.command_detail = None

    def _deadline(self, gimbal=False):
        config = self.settings.get("target_inspection", {})
        return time.monotonic() + config.get(
            "gimbal_wait_timeout_s" if gimbal else "wait_timeout_s", 8.0 if gimbal else 5.0)

    def _check_deadline(self, deadline, stage):
        if time.monotonic() >= deadline:
            self.chassis.stop()
            self.wait_error = "target inspection timeout: " + stage
            if self.command_detail is not None:
                detail = dict(self.command_detail)
                sample = self.logger.get_sample("gimbal")
                detail["action_state"] = getattr(self.active_action, "state", None)
                detail["action_released"] = self.active_action is None
                if sample is not None and len(sample[0]) >= 3:
                    stamp = sample[1]
                    detail.update(actual_pitch_deg=sample[0][2 if detail["pitch_frame"] == "ground" else 0],
                                  actual_yaw_deg=sample[0][1], sample_timestamp=stamp,
                                  sample_fresh=self._fresh_timestamp(stamp))
                    if isinstance(stamp, (int, float)) and math.isfinite(stamp):
                        detail["sample_age_s"] = round(time.time() - stamp, 3)
                        detail["sample_after_command"] = stamp > detail["requested_at"]
                self.command_detail = detail
                self.wait_error += "; " + ", ".join("{}={}".format(k, v) for k, v in detail.items())
            if self.active_cell is not None:
                self._progress(self.active_cell, self.active_delta, "stopped", error=self.wait_error)
            raise MissionStop(self.wait_error)

    def _fresh_timestamp(self, timestamp):
        return (isinstance(timestamp, (int, float)) and math.isfinite(timestamp)
                and 0 <= time.time() - timestamp <= self.settings["max_sample_age_s"])

    def _progress(self, cell, delta, status, target=None, **detail):
        if status != "stopped":
            self.last_progress_status = status

        if self.on_progress is not None:
            self.on_progress({
                "cell": list(cell),
                "direction": list(delta),
                "status": status,
                "target": target,
                **detail,
            })

    def _health(self):
        error = self.slam_worker.status().get("error")
        if error:
            raise MissionStop(error)

        camera_error = getattr(
            self.camera_frames, "camera_error", None
        )
        if camera_error is not None and not self.restoring:
            raise MissionStop(
                "camera stopped during target inspection: "
                f"{camera_error or 'unknown error'}"
            )

    def _angles(self, pitch_frame="ground", deadline=None, after=None):
        deadline = self._deadline() if deadline is None else deadline
        while True:
            self._check_deadline(deadline, "waiting for fresh gimbal angles")
            self._health()

            sample = self.logger.get_sample(
                "gimbal",
                max_age_s=self.settings["max_sample_age_s"],
            )

            if (sample is not None and len(sample[0]) >= 3
                    and self._fresh_timestamp(sample[1])
                    and (after is None or sample[1] > after)):
                try:
                    pitch_index = (
                        2 if pitch_frame == "ground" else 0
                    )
                    pitch = float(sample[0][pitch_index])
                    yaw = float(sample[0][1])

                    if math.isfinite(pitch) and math.isfinite(yaw):
                        return pitch, yaw, sample[1]
                except (TypeError, ValueError):
                    pass

            time.sleep(.03)

    def _safe_status(self):
        """Wait for fresh safety telemetry while wheels remain stopped."""
        self.command_detail = None
        deadline = self._deadline()
        while True:
            self._check_deadline(deadline, "waiting for fresh safety telemetry")
            self._health()

            samples = [
                self.logger.get_sample(
                    name,
                    max_age_s=self.settings["max_sample_age_s"],
                )
                for name in ("position", "attitude", "status")
            ]

            if (
                all(sample is not None and self._fresh_timestamp(sample[1]) for sample in samples)
                and (
                    max(sample[1] for sample in samples)
                    - min(sample[1] for sample in samples)
                    <= self.settings["sample_skew_s"]
                )
                and len(samples[2][0]) >= 10
            ):
                sample = samples[2]

                for index in (4, 5, 6, 7, 8, 9):
                    if sample[0][index] not in (0, False, None):
                        raise MissionStop(
                            f"robot safety status flag {index} is active"
                        )

                return

            self.chassis.stop()
            time.sleep(.05)

    def _wait_for_gimbal_idle(self, gimbal, deadline):
        def check_health():
            self._check_deadline(deadline, "waiting for previous gimbal action")
            self._health()

        def on_wait(action):
            self.active_action = action
            if self.active_cell is not None:
                self._progress(self.active_cell, self.active_delta,
                               "waiting_target_action", action_state=action.state)

        wait_for_gimbal_idle(
            gimbal, check_health, on_wait,
            remaining_timeout=lambda: max(.001, deadline - time.monotonic()),
        )
        self.active_action = None

    def _point(
        self,
        pitch,
        yaw,
        pitch_frame="ground",
        recenter=False,
        yaw_speed=None,
    ):
        """Wait for SDK action release and fresh angle telemetry."""
        if self.active_cell is not None:
            self._progress(
                self.active_cell,
                self.active_delta,
                "waiting_target_safety",
            )

        self._safe_status()
        deadline = self._deadline(gimbal=True)
        request_time = time.time()
        self.command_detail = {"command": "recenter" if recenter else "moveto",
                               "phase": "restore" if self.restoring else "inspection",
                               "pitch_frame": pitch_frame, "target_pitch_deg": pitch,
                               "target_yaw_deg": yaw, "requested_at": request_time}

        try:
            gimbal = (
                self.gimbal
                if pitch_frame == "ground"
                else self.scan_gimbal
            )

            target_cfg = self.settings.get(
                "target_inspection", {}
            )

            if yaw_speed is None:
                yaw_speed = target_cfg.get(
                    "scan_yaw_speed_deg_s",
                    self.settings.get("gimbal", {}).get(
                        "yaw_speed_deg_s", 60
                    ),
                )

            self._wait_for_gimbal_idle(gimbal, deadline)
            request_time = time.time()
            self.command_detail["requested_at"] = request_time
            if recenter:
                action = gimbal.recenter(
                    pitch_speed=30,
                    yaw_speed=yaw_speed,
                )
            else:
                action = gimbal.moveto(
                    pitch=pitch,
                    yaw=yaw,
                    pitch_speed=30,
                    yaw_speed=yaw_speed,
                )

        except Exception as error:
            raise MissionStop(
                f"target gimbal command failed: {error}"
            ) from error

        self.active_action = action
        released = action is None
        reported_state = None
        reported_angle_state = None

        while True:
            self._check_deadline(deadline, "waiting for gimbal action or target angle")
            self._health()
            state = getattr(action, "state", None)

            if state in (
                "action_failed",
                "action_rejected",
                "action_exception",
                "action_aborted",
            ):
                self.active_action = None
                raise MissionStop(
                    f"target gimbal action failed: {state}"
                )

            if (
                not released
                and getattr(action, "has_succeeded", False)
            ):
                wait = getattr(action, "wait_for_completed", None)

                if callable(wait):
                    try:
                        completed = wait(timeout=max(.001, deadline - time.monotonic()))
                    except Exception as error:
                        raise MissionStop(
                            "target gimbal action release failed: "
                            f"{error}"
                        ) from error

                    if not completed:
                        raise MissionStop(
                            "target gimbal action was not released by SDK"
                        )

                released = True
                self.active_action = None

            waiting_state = (
                "waiting_target_angle"
                if released
                else "waiting_target_action"
            )

            if (
                waiting_state != reported_state
                and self.active_cell is not None
            ):
                self._progress(
                    self.active_cell,
                    self.active_delta,
                    waiting_state,
                    pitch_frame=pitch_frame,
                    target_pitch_deg=pitch,
                    target_yaw_deg=yaw,
                    action_state=state,
                )
                reported_state = waiting_state

            measured_pitch, measured_yaw, timestamp = (
                self._angles(pitch_frame, deadline=deadline, after=request_time)
            )

            if (
                waiting_state != reported_angle_state
                and self.active_cell is not None
            ):
                self._progress(
                    self.active_cell,
                    self.active_delta,
                    waiting_state,
                    pitch_frame=pitch_frame,
                    target_pitch_deg=pitch,
                    actual_pitch_deg=round(measured_pitch, 2),
                    target_yaw_deg=yaw,
                    actual_yaw_deg=round(measured_yaw, 2),
                    action_state=state,
                )
                reported_angle_state = waiting_state

            angles_aligned = (
                timestamp > request_time
                and (
                    abs(measured_pitch - pitch)
                    <= self.settings["gimbal"]["pitch_tolerance_deg"]
                )
                and (
                    abs(_wrap_degrees(measured_yaw - yaw))
                    <= self.settings["gimbal"]["angle_tolerance_deg"]
                )
            )

            if (
                released
                and angles_aligned
            ):
                self.commanded_angles = (pitch, yaw)

                settle_s = self.settings.get(
                    "target_inspection", {}
                ).get("aim_settle_s", 0.0)

                if settle_s > 0:
                    time.sleep(settle_s)

                self.command_detail = None
                return measured_pitch, measured_yaw

            time.sleep(.03)

    def _frame(self):
        after = self.camera_frames.current_frame_number()

        if self.active_cell is not None:
            self._progress(
                self.active_cell,
                self.active_delta,
                "waiting_camera_frame",
                after_frame=after,
            )

        return self.camera_frames.wait_for_frame(
            after,
            check_health=self._health,
        )

    def _fire(self, item, frame, aim_mode):
        self._safe_status()

        pitch, yaw, _ = self._angles()
        expected_pitch, expected_yaw = self.commanded_angles

        if (
            abs(pitch - expected_pitch)
            > self.settings["gimbal"]["pitch_tolerance_deg"]
            or (
                abs(_wrap_degrees(yaw - expected_yaw))
                > self.settings["gimbal"]["angle_tolerance_deg"]
            )
        ):
            raise MissionStop(
                "target gimbal drifted before infrared fire"
            )

        config = self.settings.get("target_inspection", {})
        shots = config.get("shots_per_target", 2)

        self._health()

        try:
            accepted = self.blaster.fire(
                fire_type=self.fire_type,
                times=shots,
            )
        except Exception as error:
            raise MissionStop(
                "infrared fire command failed; "
                f"firing state unknown: {error}"
            ) from error

        if not accepted:
            raise MissionStop(
                "infrared fire command was not accepted; "
                "firing state unknown"
            )

        # Hold the gimbal still until the requested shots finish.
        settle_s = config.get("fire_settle_s", 1.0)
        if settle_s > 0:
            time.sleep(settle_s)

        height, width = frame.shape[:2]

        return {
            "status": "fire_command_accepted",
            "color": item.color,
            "shape": item.shape,
            "area_fraction": round(
                item.area / (width * height), 4
            ),
            "center": list(item.center),
            "shots_requested": shots,
            "aim_mode": aim_mode,
        }

    def _confirm_and_fire(
        self,
        track_id,
        tracker,
        frame,
        item,
        mode,
        aim_after_lock=False,
    ):
        config = self.settings.get("target_inspection", {})
        lock_frames = config.get("lock_frames", 3)
        max_misses = config.get("lock_max_misses", 3)

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
                    visible,
                    target_color,
                    target_shape,
                    confirmed_item.center,
                )

                if current_item is not None:
                    track_id = new_track_id

            if (
                current_item is None
                or current_item.color != target_color
                or current_item.shape != target_shape
            ):
                misses += 1
                exact_hits = 0
            else:
                confirmed_item = current_item
                exact_hits += 1

        if exact_hits < lock_frames:
            return {
                "status": "target_lost",
                "reason": (
                    "exact color and shape not stable during fire lock"
                ),
                "exact_hits": exact_hits,
                "required_hits": lock_frames,
            }

        if aim_after_lock:
            return self._fire_locked_target(
                confirmed_item, frame
            )

        return self._fire(confirmed_item, frame, mode)

    def _aim(self, track_id, tracker, first_frame, first_item):
        config = self.settings["target_inspection"]
        aim_speed = config.get("aim_yaw_speed_deg_s", 30)
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

        # Confirm the complete shape before moving the barrel.
        return self._confirm_and_fire(
            track_id,
            tracker,
            first_frame,
            first_item,
            "centered",
            aim_after_lock=True,
        )

    def _fire_locked_target(self, item, frame):
        """Project a stationary confirmed target into an IR aim angle."""
        config = self.settings["target_inspection"]
        height, width = frame.shape[:2]

        # Use the last confirmed position before moving the gimbal.
        error_x = (
            item.center[0] / width
            - (.5 + config["aim_offset_x_fraction"])
        )
        error_y = (
            (.5 + config["aim_offset_y_fraction"])
            - item.center[1] / height
        )

        pitch, yaw, _ = self._angles()

        desired_pitch = (
            pitch + error_y * config["camera_vfov_deg"]
        )
        desired_yaw = (
            yaw + error_x * config["camera_hfov_deg"]
        )

        target_pitch = max(-20, min(20, desired_pitch))
        target_yaw = max(-250, min(250, desired_yaw))

        mode = "centered"

        if (
            math.hypot(error_x, error_y)
            > config["center_radius_fraction"]
        ):
            mode = (
                "closest_reachable"
                if (
                    target_pitch != desired_pitch
                    or target_yaw != desired_yaw
                )
                else "locked_projection"
            )

            if self.active_cell is not None:
                self._progress(
                    self.active_cell,
                    self.active_delta,
                    "target_locked",
                    {
                        "color": item.color,
                        "shape": item.shape,
                    },
                    target_pitch_deg=target_pitch,
                    target_yaw_deg=target_yaw,
                )

            # Move in bounded steps and wait for each SDK action.
            # Do not require another visible shape after movement.
            for _ in range(config["max_aim_steps"]):
                dp = target_pitch - pitch
                dy = target_yaw - yaw

                if abs(dp) < .1 and abs(dy) < .1:
                    break

                step = config["max_step_deg"]

                self._point(
                    pitch + max(-step, min(step, dp)),
                    yaw + max(-step, min(step, dy)),
                    yaw_speed=config.get(
                        "aim_yaw_speed_deg_s", 30
                    ),
                )

                pitch, yaw, _ = self._angles()

            if (
                abs(target_pitch - pitch) >= .1
                or abs(target_yaw - yaw) >= .1
            ):
                return {
                    "status": "target_lost",
                    "reason": "locked aim exceeds movement budget",
                }

        # Fresh safety telemetry, gimbal angles and camera health
        # are still checked before sending the infrared command.
        result = self._fire(item, frame, mode)

        result.update({
            "lock_before_aim": True,
            "center_source": "pre_aim_lock",
            "locked_pitch_deg": pitch,
            "locked_yaw_deg": yaw,
        })

        return result

    def inspect(
        self,
        cell,
        delta,
        world_yaw,
        body_yaw,
        range_mm=None,
        initial_pitch=None,
    ):
        self.active_cell = cell
        self.active_delta = delta

        config = self.settings["target_inspection"]

        selected = (
            None
            if config["selected"] == "all"
            else {
                tuple(item.split(":"))
                for item in config["selected"]
            }
        )

        if initial_pitch is None:
            initial_pitch = config["pitch_deg"]

            if config.get("adaptive_pitch", True):
                if range_mm is None:
                    sample = self.logger.get_sample(
                        "tof",
                        max_age_s=self.settings["max_sample_age_s"],
                    )

                    if sample is not None and len(sample[0]) > 0:
                        try:
                            range_mm = float(sample[0][0])
                        except (
                            TypeError,
                            ValueError,
                            IndexError,
                        ):
                            range_mm = None

                if (
                    range_mm is not None
                    and (
                        0 < range_mm
                        <= config.get(
                            "close_range_threshold_mm", 180
                        )
                    )
                ):
                    initial_pitch = config.get(
                        "close_pitch_deg", -10.0
                    )

        result = {
            "cell": list(cell),
            "direction": list(delta),
            "pitch_deg": initial_pitch,
            "status": "checking",
            "targets": [],
            "checked_at": time.time(),
        }

        self.chassis.stop()
        _, original_yaw, _ = self._angles()

        target_yaw = _wrap_degrees(world_yaw - body_yaw)

        candidates = (
            target_yaw - 360,
            target_yaw,
            target_yaw + 360,
        )
        reachable = [
            value for value in candidates
            if -250 <= value <= 250
        ]

        if not reachable:
            raise MissionStop(
                "wall camera cannot reach target yaw"
            )

        if abs(abs(_wrap_degrees(target_yaw)) - 180.0) <= 30.0 and abs(original_yaw) < 80.0:
            neg_candidates = [a for a in reachable if a < 0]
            if neg_candidates:
                inspection_yaw = max(neg_candidates)
            else:
                inspection_yaw = min(
                    reachable,
                    key=lambda value: abs(value - original_yaw),
                )
        else:
            inspection_yaw = min(
                reachable,
                key=lambda value: abs(value - original_yaw),
            )

        self.slam_worker.pause_mapping()

        try:
            self._progress(cell, delta, "pointing")

            scan_speed = config.get(
                "scan_yaw_speed_deg_s", 50
            )

            self._point(
                initial_pitch,
                inspection_yaw,
                yaw_speed=scan_speed,
            )

            tracker = TargetTracker(
                config["min_area_fraction"],
                selected,
                self.color_ranges,
            )

            attempted = []
            confirm_frames = config["confirm_frames"]
            search_frames = config.get(
                "search_frames", confirm_frames + 2
            )

            for _ in range(config["max_targets_per_wall"]):
                self._progress(cell, delta, "searching")

                visible = {}
                frame = None

                for _ in range(search_frames):
                    _, frame = self._frame()
                    visible = tracker.update(frame)

                    if any(
                        t.consecutive >= confirm_frames
                        for t in tracker.tracks.values()
                    ):
                        break

                if frame is None:
                    break

                height, width = frame.shape[:2]
                diagonal = math.hypot(width, height)

                ready = [
                    (track_id, item)
                    for track_id, item in visible.items()
                    if (
                        tracker.tracks[track_id].consecutive
                        >= confirm_frames
                    )
                    and not any(
                        item.color == color
                        and item.shape == shape
                        and (
                            math.hypot(
                                item.center[0] - center[0],
                                item.center[1] - center[1],
                            ) / diagonal <= .06
                        )
                        for color, shape, center in attempted
                    )
                ]

                if not ready:
                    break

                track_id, item = max(
                    ready,
                    key=lambda pair: pair[1].area,
                )

                attempted.append((
                    item.color,
                    item.shape,
                    item.center,
                ))

                self._progress(
                    cell,
                    delta,
                    "aiming",
                    {
                        "color": item.color,
                        "shape": item.shape,
                    },
                )

                outcome = self._aim(
                    track_id,
                    tracker,
                    frame,
                    item,
                )

                outcome.setdefault("color", item.color)
                outcome.setdefault("shape", item.shape)
                result["targets"].append(outcome)

                if (
                    len(result["targets"])
                    < config["max_targets_per_wall"]
                ):
                    self._point(
                        initial_pitch,
                        inspection_yaw,
                        yaw_speed=scan_speed,
                    )

            result["status"] = (
                "targets_checked"
                if result["targets"]
                else "no_target"
            )

            return result

        finally:
            restored = False
            inspection_wait = self.last_progress_status

            try:
                self.restoring = True

                if self.active_action is None and self.wait_error is None:
                    self._progress(
                        cell,
                        delta,
                        "restoring_scan_angle",
                    )

                    # Return to chassis-relative center.
                    # The next scan selects its own yaw.
                    self._point(
                        0,
                        0,
                        pitch_frame="chassis",
                        recenter=True,
                    )

                    restored = True

            finally:
                self.restoring = False
                self.slam_worker.resume_mapping()

                self._progress(
                    cell,
                    delta,
                    (
                        result["status"]
                        if (
                            restored
                            and result["status"] != "checking"
                        )
                        else "stopped"
                    ),
                    interrupted_from=(
                        self.last_progress_status
                        if not restored
                        else (
                            inspection_wait
                            if result["status"] == "checking"
                            else None
                        )
                    ),
                    action_state=getattr(
                        self.active_action, "state", None
                    ),
                    error=self.wait_error,
                    command_detail=self.command_detail,
                )

                self.active_cell = None
                self.active_delta = None
