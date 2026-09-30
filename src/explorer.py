"""Depth-first frontier traversal over a SLAM occupancy grid."""

import heapq
import math
import statistics
import threading
import time

from src.slam import CellWallGrid, _wrap_degrees
from src.mission_stop import MissionStop
from src.ir_lane import prepare_lane, retarget_lane
from src.gimbal_control import wait_for_gimbal_idle


class DFSExplorer:
    """Traverse shared open cell borders observed with the gimbal ToF."""

    DIRECTIONS = ((1, 0), (0, -1), (-1, 0), (0, 1))

    def __init__(self, chassis, gimbal, logger, slam_map, settings, target_inspector=None):
        self.chassis = chassis
        self.gimbal = gimbal
        self.logger = logger
        self.map = slam_map
        self.settings = settings
        self.status = "ready"
        self.error = None
        self.visited = set()
        self.scanned_cells = set()
        self.stack = []
        self.route = []
        self.route_goal = None
        self.route_plans = 0
        self.route_blockages = 0
        self.moves = 0
        self.base_pose = None
        self.current_cell = (0, 0)
        self.cell_targets = {}
        self.ir_lanes = {}
        self.last_ir_lane = None
        self.alignments = {}
        self.last_alignment_move = None
        self.heading_alignments = {}
        self.last_heading_alignment = None
        self.last_applied_heading_alignment = None
        self.heading_measurement = None
        self.travel_heading_deg = None
        self.last_motion_heading = None
        self.last_motion_stop = None
        self.scan_alignment = None
        self.target_inspector = target_inspector
        self.wall_inspections = {}
        self.target_progress = None
        self.wall_grid = CellWallGrid(
            settings["step_m"], min(1000, math.ceil(math.hypot(
                settings["map"]["width_m"], settings["map"]["height_m"]
            ) / settings["step_m"]) + 1)
        )
        self.slam_worker = None
        self.lock = threading.RLock()

    def snapshot(self):
        with self.lock:
            return {
                "status": self.status,
                "error": self.error,
                "visited": [list(node) for node in sorted(self.visited)],
                "scanned_cells": [list(node) for node in sorted(self.scanned_cells)],
                "stack": [list(node) for node in self.stack],
                "route": [list(node) for node in self.route],
                "route_goal": list(self.route_goal) if self.route_goal is not None else None,
                "route_points": [list(self.cell_targets.get(node, self._to_map(node)))
                                 for node in self.route]
                if self.base_pose is not None else [],
                "route_plans": self.route_plans,
                "route_blockages": self.route_blockages,
                "visited_points": [list(self.cell_targets.get(node, self._to_map(node)))
                                   for node in sorted(self.visited)]
                if self.base_pose is not None else [],
                "stack_points": [list(self.cell_targets.get(node, self._to_map(node)))
                                 for node in self.stack]
                if self.base_pose is not None else [],
                "moves": self.moves,
                "ir_lanes": [dict(value) for _, value in sorted(self.ir_lanes.items())],
                "last_ir_lane": dict(self.last_ir_lane) if self.last_ir_lane else None,
                "map_bounds": {
                    "width_m": self.map.width * self.map.resolution,
                    "height_m": self.map.height * self.map.resolution,
                    "start_x_m": -self.map.origin_x,
                    "start_y_m": -self.map.origin_y,
                },
                "heading_source": self.settings["heading_source"],
                "travel_heading_deg": self.travel_heading_deg,
                "gimbal_pitch_frame": "chassis",
                "alignment_enabled": self.settings["alignment"]["enabled"],
                "alignment_interval_steps": self.settings["alignment"].get("interval_steps", 2),
                "last_alignment_move": self.last_alignment_move,
                "emergency_stop_distance_m": self.settings["emergency_stop_distance_m"],
                "last_motion_heading": self.last_motion_heading,
                "last_motion_stop": self.last_motion_stop,
                "scan_alignment": self.scan_alignment,
                "tof_median_window": self.settings["tof_median_window"],
                "cell_targets": {f"{node[0]},{node[1]}": list(point)
                                 for node, point in sorted(self.cell_targets.items())},
                "alignments": {f"{node[0]},{node[1]}": result
                                   for node, result in sorted(self.alignments.items())},
                "heading_alignment_enabled": self.settings["heading_alignment"]["enabled"],
                "heading_bias_deg": getattr(self.chassis, "heading_bias_deg", 0.0),
                "last_heading_alignment": self.last_heading_alignment,
                "last_applied_heading_alignment": self.last_applied_heading_alignment,
                "heading_measurement": self.heading_measurement,
                "heading_alignments": {f"{node[0]},{node[1]}": result
                                       for node, result in sorted(self.heading_alignments.items())},
                "target_inspection_enabled": self.settings["target_inspection"]["enabled"],
                "wall_inspections": [value for _, value in sorted(self.wall_inspections.items())],
                "target_progress": self.target_progress,
                "max_nodes": self.settings["max_nodes"],
                "cell_grid": self.wall_grid.snapshot(self.base_pose, self.current_cell),
            }

    def _set_status(self, status, error=None):
        with self.lock:
            self.status, self.error = status, error
            snapshot = self.snapshot()
        self.map.set_exploration_state(snapshot)

    def _to_map(self, node):
        step = self.settings["step_m"]
        u, v = node[0] * step, node[1] * step
        x0, y0, yaw = self.base_pose
        angle = math.radians(yaw)
        return (x0 + u * math.cos(angle) - v * math.sin(angle),
                y0 + u * math.sin(angle) + v * math.cos(angle))

    def _wait_for_fresh(self, read):
        """Hold the wheels while telemetry is unavailable, then resume on a fresh sample."""
        previous_status = self.status
        waiting = False
        while True:
            worker_status = self.slam_worker.status() if self.slam_worker is not None else None
            if worker_status is not None and worker_status.get("error"):
                raise MissionStop(worker_status["error"])
            value = None if (worker_status or {}).get("waiting_telemetry") else read()
            if value is not None:
                if waiting:
                    self._set_status(previous_status)
                return value
            if not waiting:
                self._set_status("waiting_telemetry")
                waiting = True
            stop = getattr(self.chassis, "stop", None)
            if callable(stop):
                stop()
            time.sleep(0.05)

    def _current_yaw(self):
        if (self.settings["heading_source"] == "gimbal" or
                self.settings["heading_alignment"]["enabled"]):
            getter = getattr(self.chassis, "get_pose", None)
            if callable(getter):
                def read_gimbal_yaw():
                    pose = getter()
                    if pose is None:
                        return None
                    try:
                        yaw = float(pose[2])
                    except (IndexError, TypeError, ValueError):
                        return None
                    return yaw if math.isfinite(yaw) else None
                return self._wait_for_fresh(read_gimbal_yaw)

        def read_attitude_yaw():
            sample = self.logger.get_sample("attitude", max_age_s=self.settings["max_sample_age_s"])
            if sample is None:
                return None
            try:
                yaw = float(sample[0][0])
            except (IndexError, TypeError, ValueError):
                return None
            return yaw if math.isfinite(yaw) else None
        return self._wait_for_fresh(read_attitude_yaw)

    def _drive_holding_current_yaw(self, x, y, kind, stop_if=None, on_ir_recovered=None):
        """Keep the initial DFS heading while checking fresh yaw before each move."""
        observed_yaw = self._current_yaw()
        with self.lock:
            if self.travel_heading_deg is None:
                self.travel_heading_deg = observed_yaw
            yaw = self.travel_heading_deg
            self.last_motion_heading = {
                "yaw_deg": yaw, "observed_yaw_deg": observed_yaw,
                "target_m": [x, y], "kind": kind,
                "source": self.settings["heading_source"],
            }
        self.map.set_exploration_state(self.snapshot())
        options = {"yaw": yaw, "abort_event": self.slam_worker.abort_event,
                   "disable_timeout": True,
                   "pause_if": lambda: self.slam_worker.status().get("waiting_telemetry", False)}
        if stop_if is not None:
            options["stop_if"] = stop_if
        if on_ir_recovered is not None:
            options["on_ir_recovered"] = on_ir_recovered
        return self.chassis.move_to(x, y, **options)

    def _motion_pose(self):
        """Read the chassis frame used by move_to, rather than scan-matched pose."""
        getter = getattr(self.chassis, "get_pose", None)
        def read_pose():
            pose = getter() if callable(getter) else self.map.pose
            if pose is None or len(pose) < 2:
                return None
            try:
                return pose if all(math.isfinite(float(v)) for v in pose[:2]) else None
            except (TypeError, ValueError):
                return None
        return self._wait_for_fresh(read_pose)

    def _sensor_offset(self, world_yaw):
        direction = math.radians(world_yaw)
        body_yaw = math.radians(self._current_yaw())
        sensor = self.settings["sensor"]
        return (sensor["pivot_x_m"] * math.cos(direction - body_yaw) +
                sensor["pivot_y_m"] * math.sin(direction - body_yaw) +
                sensor["offset_from_yaw_axis_m"] *
                math.cos(math.radians(sensor["offset_yaw_deg"])))

    def _gimbal_sample(self):
        def read_gimbal():
            sample = self.logger.get_sample("gimbal", max_age_s=self.settings["max_sample_age_s"])
            if sample is None or len(sample[0]) < 4:
                return None
            try:
                yaw, chassis_pitch = float(sample[0][1]), float(sample[0][0])
            except (TypeError, ValueError):
                return None
            if not math.isfinite(yaw) or not math.isfinite(chassis_pitch):
                return None
            # The DFS action uses COORDINATE_CAR for both axes.
            return yaw, chassis_pitch, float(sample[1])
        return self._wait_for_fresh(read_gimbal)

    @staticmethod
    def _command_yaw(target_relative_yaw, current_relative_yaw):
        """Choose the nearest equivalent chassis-relative SDK yaw target."""
        candidates = (target_relative_yaw - 360.0, target_relative_yaw,
                      target_relative_yaw + 360.0)
        reachable = [angle for angle in candidates if -250.0 <= angle <= 250.0]
        if not reachable:
            raise MissionStop("gimbal cannot reach the requested direction within its yaw limits")
        # When facing the rear (~180 deg) from near-center, prefer negative yaw (-180 deg)
        # because the RoboMaster EP cable harness has ample clockwise slack without binding.
        if abs(abs(_wrap_degrees(target_relative_yaw)) - 180.0) <= 30.0 and abs(current_relative_yaw) < 80.0:
            neg_candidates = [a for a in reachable if a < 0]
            if neg_candidates:
                return max(neg_candidates)
        return min(reachable, key=lambda angle: abs(angle - current_relative_yaw))

    def _wait_for_gimbal_idle(self):
        """Keep wheels stopped until an earlier head action is released."""
        def check_health():
            worker_status = self.slam_worker.status() if self.slam_worker is not None else {}
            if worker_status.get("error"):
                raise MissionStop(worker_status["error"])

        wait_for_gimbal_idle(
            self.gimbal, check_health,
            lambda action: self._set_status("waiting_gimbal_action"),
        )

    def _prepare_gimbal(self, force=False):
        """Physically center the head and wait for SDK release and fresh angles."""
        if not force and not self.settings["gimbal"]["auto_recenter"]:
            return
        stop = getattr(self.chassis, "stop", None)
        if callable(stop):
            stop()
        self._set_status("recenter_for_heading" if force else "centering_gimbal")
        self._wait_for_gimbal_idle()
        request_time = time.time()
        action = self.gimbal.recenter(
            pitch_speed=self.settings["gimbal"]["recenter_speed_deg_s"],
            yaw_speed=self.settings["gimbal"]["recenter_speed_deg_s"],
        )
        if action is not None:
            while not getattr(action, "has_succeeded", False):
                state = getattr(action, "state", None)
                if state in ("action_failed", "action_rejected", "action_exception", "action_aborted"):
                    raise MissionStop("gimbal recenter failed: {}".format(state))
                worker_status = self.slam_worker.status()
                if worker_status["error"]:
                    raise MissionStop(worker_status["error"])
                time.sleep(0.03)
            wait_for_completed = getattr(action, "wait_for_completed", None)
            if callable(wait_for_completed) and not wait_for_completed():
                raise MissionStop("gimbal SDK reported recenter success but did not release its action")
        while True:
            yaw, pitch, timestamp = self._gimbal_sample()
            if timestamp > request_time:
                break
            time.sleep(0.03)
        if (abs(_wrap_degrees(yaw)) > self.settings["gimbal"]["angle_tolerance_deg"] or
                abs(pitch) > self.settings["gimbal"]["pitch_tolerance_deg"]):
            raise MissionStop(
                "gimbal recenter did not reach center: yaw {:.1f}, pitch {:.1f}".format(
                    yaw, pitch))

    def _latest_alignment_scan(self):
        return (self.map.latest_scan_timestamp, self.map.latest_gimbal_yaw_deg,
                self.map.latest_range_mm)

    def _scan_for_direction(self, delta, yaw_offset_deg=0.0, median_window=None):
        """Point the single ToF toward a candidate cell and wait for its new scan."""
        stop = getattr(self.chassis, "stop", None)
        if callable(stop):
            stop()
        worker_status = self.slam_worker.status() if self.slam_worker is not None else None
        if worker_status is not None and worker_status["error"]:
            raise MissionStop(worker_status["error"])
        world_yaw = (self.base_pose[2] + math.degrees(math.atan2(delta[1], delta[0])) +
                     yaw_offset_deg)
        body_yaw = self._current_yaw()
        sensor = self.settings["sensor"]
        target_yaw = _wrap_degrees(
            world_yaw - body_yaw - float(sensor["yaw_offset_deg"])
        )
        current_yaw, _, _ = self._gimbal_sample()
        command_yaw = self._command_yaw(target_yaw, current_yaw)
        tolerance = self.settings["gimbal"]["angle_tolerance_deg"]
        pitch_tolerance = self.settings["gimbal"]["pitch_tolerance_deg"]
        # Center yaw without recenter(), which uses a separate SDK action and
        # previously remained active after angle telemetry had settled.
        if abs(target_yaw) <= tolerance:
            command_yaw = 0.0
        previous_status = self.status
        self._set_status("scanning")
        self._wait_for_gimbal_idle()
        request_time = time.time()
        action = self.gimbal.moveto(
            pitch=self.settings["gimbal"]["pitch_deg"],
            yaw=command_yaw,
            pitch_speed=30,
            yaw_speed=self.settings["gimbal"]["yaw_speed_deg_s"],
        )

        aligned = False
        measured_yaw = current_yaw
        action_confirmed = action is None
        median_window = (self.settings["tof_median_window"] if median_window is None
                         else median_window)
        samples = []
        last_sample_timestamp = request_time
        collection_after = request_time if action_confirmed else None
        completion_time = request_time if action_confirmed else None
        settled_count = 0
        settled_angle = None
        last_alignment_timestamp = request_time
        alignment_retries = 0
        while True:
            worker_status = self.slam_worker.status() if self.slam_worker is not None else None
            if worker_status is not None and worker_status["error"]:
                raise MissionStop(worker_status["error"])
            action_state = getattr(action, "state", None)
            if action_state in ("action_failed", "action_rejected", "action_exception", "action_aborted"):
                raise MissionStop(f"gimbal moveto failed: {action_state}")
            measured_yaw, measured_pitch, angle_timestamp = self._gimbal_sample()
            aligned = (angle_timestamp > request_time and
                       abs(_wrap_degrees(target_yaw - measured_yaw)) <= tolerance and
                       abs(self.settings["gimbal"]["pitch_deg"] - measured_pitch) <= pitch_tolerance)

            action_complete = action is None or getattr(action, "has_succeeded", False)
            if action_complete and not action_confirmed:
                # The SDK removes its dispatcher entry before signaling the
                # completion event. Only wait once it reports success, so the
                # SDK does not turn an in-progress action into an exception.
                wait_for_completed = getattr(action, "wait_for_completed", None)
                if callable(wait_for_completed) and not wait_for_completed():
                    raise MissionStop("gimbal SDK reported success but did not release its action")
                action_confirmed = True
                completion_time = time.time()
                if median_window > 1:
                    collection_after = time.time()
                else:
                    collection_after = request_time

            # A completed SDK action can still leave the measured head off target.
            # Retry once only after fresh, unchanged telemetry confirms it settled.
            if (action is not None and action_confirmed and not aligned and
                    angle_timestamp > completion_time and
                    angle_timestamp > last_alignment_timestamp):
                angles = (measured_yaw, measured_pitch)
                if (settled_angle is not None and
                        abs(_wrap_degrees(angles[0] - settled_angle[0])) <= 0.25 and
                        abs(angles[1] - settled_angle[1]) <= 0.25):
                    settled_count += 1
                else:
                    settled_count = 1
                settled_angle = angles
                last_alignment_timestamp = angle_timestamp
                if settled_count >= 3:
                    if alignment_retries:
                        raise MissionStop(
                            "gimbal alignment failed after retry: yaw target {:.1f}, actual {:.1f}; "
                            "pitch target {:.1f}, actual {:.1f}".format(
                                target_yaw, measured_yaw,
                                self.settings["gimbal"]["pitch_deg"], measured_pitch))
                    alignment_retries += 1
                    self._wait_for_gimbal_idle()
                    request_time = time.time()
                    command_yaw = self._command_yaw(target_yaw, measured_yaw)
                    action = self.gimbal.moveto(
                        pitch=self.settings["gimbal"]["pitch_deg"], yaw=command_yaw,
                        pitch_speed=30,
                        yaw_speed=self.settings["gimbal"]["yaw_speed_deg_s"],
                    )
                    action_confirmed = action is None
                    completion_time = request_time if action_confirmed else None
                    collection_after = request_time if action_confirmed else None
                    last_sample_timestamp = request_time
                    last_alignment_timestamp = request_time
                    settled_count = 0
                    settled_angle = None
                    samples.clear()
                    continue

            scan_timestamp, scan_yaw, scan_range = self._latest_alignment_scan()
            scan_ready = (aligned and scan_timestamp is not None and
                    scan_timestamp > request_time and
                    scan_timestamp > last_sample_timestamp and
                    scan_yaw is not None and
                    abs(_wrap_degrees(target_yaw - scan_yaw)) <= tolerance and
                    scan_range is not None and
                    time.time() - scan_timestamp <= self.settings["max_sample_age_s"] * 2)
            with self.lock:
                self.scan_alignment = {
                    "target_yaw_deg": round(target_yaw, 2),
                    "actual_yaw_deg": round(measured_yaw, 2),
                    "yaw_tolerance_deg": tolerance,
                    "target_pitch_deg": self.settings["gimbal"]["pitch_deg"],
                    "actual_pitch_deg": round(measured_pitch, 2),
                    "pitch_tolerance_deg": pitch_tolerance,
                    "aligned": aligned,
                    "action_confirmed": action_confirmed,
                    "alignment_retries": alignment_retries,
                }
            if not action_confirmed:
                scan_status = "waiting_gimbal_action"
            elif not aligned:
                scan_status = "waiting_gimbal_alignment"
            elif (worker_status or {}).get("tof_waiting", False):
                scan_status = "waiting_tof"
            else:
                scan_status = "scanning"
            if self.status != scan_status:
                self._set_status(scan_status)
            if (action_confirmed and scan_ready and
                    scan_timestamp > collection_after and
                    not (worker_status or {}).get("tof_waiting", False)):
                if samples and time.time() - samples[0][0] > self.settings["max_sample_age_s"] * 2:
                    samples.clear()
                samples.append((scan_timestamp, float(scan_range)))
                last_sample_timestamp = scan_timestamp
                if len(samples) >= median_window:
                    with self.lock:
                        self.scan_alignment = None
                        self.last_scan_relative_yaw_deg = measured_yaw
                    self._set_status(previous_status)
                    return float(statistics.median(value for _, value in samples)), world_yaw
            time.sleep(0.03)

    @staticmethod
    def _fit_wall_normal(points, facing_yaw_deg, minimum_span_m, maximum_residual_m):
        """Fit a wall in chassis coordinates and return its inward ray normal."""
        first, last = points[0], points[-1]
        dx, dy = last[0] - first[0], last[1] - first[1]
        span = math.hypot(dx, dy)
        if span < minimum_span_m:
            return None, span, None
        residual = max(abs(dx * (point[1] - first[1]) -
                           dy * (point[0] - first[0])) / span for point in points)
        if residual > maximum_residual_m:
            return None, span, residual
        normal_x, normal_y = dy / span, -dx / span
        facing = math.radians(facing_yaw_deg)
        if normal_x * math.cos(facing) + normal_y * math.sin(facing) < 0:
            normal_x, normal_y = -normal_x, -normal_y
        return math.degrees(math.atan2(normal_y, normal_x)), span, residual

    def _align_heading(self, node):
        """Recenter, fit a nearby wall from fresh fan scans, then correct yaw bias."""
        settings = self.settings["heading_alignment"]
        candidates = []
        for delta in self.DIRECTIONS:
            check = self.wall_grid.checks.get((tuple(node), delta), {})
            range_mm = check.get("range_mm")
            if (self.wall_grid.state(node, delta) == "wall" and
                    type(range_mm) in (int, float) and
                    math.isfinite(range_mm) and 0 < range_mm <=
                    self.settings["wall_threshold_mm"]):
                candidates.append((range_mm, delta))

        def record(result):
            with self.lock:
                self.heading_alignments[tuple(node)] = result
                self.last_heading_alignment = result
                if result["status"] == "applied":
                    self.last_applied_heading_alignment = result
                self.heading_measurement = None
            self.map.set_exploration_state(self.snapshot())
            return result

        if not candidates:
            return record({"status": "no_suitable_wall", "cell": list(node),
                           "reason": "no directly measured wall within the wall threshold"})
        _, delta = max(candidates)
        side = CellWallGrid.DELTA_TO_SIDE[delta]
        start_pose = self._motion_pose()
        self.chassis.stop()
        with self.lock:
            self.heading_measurement = {"cell": list(node), "side": side,
                                        "phase": "recenter", "point": 0,
                                        "total": len(settings["scan_offsets_deg"])}
        self._prepare_gimbal(force=True)
        self._set_status("measuring_wall_heading")
        sensor = self.settings["sensor"]
        points = []
        scans = []
        for index, offset_deg in enumerate(settings["scan_offsets_deg"], 1):
            with self.lock:
                self.heading_measurement = {"cell": list(node), "side": side,
                                            "phase": "scanning", "point": index,
                                            "total": len(settings["scan_offsets_deg"])}
            self.map.set_exploration_state(self.snapshot())
            reading_mm, _ = self._scan_for_direction(
                delta, yaw_offset_deg=offset_deg,
                median_window=settings["samples_per_angle"])
            current_pose = self._motion_pose()
            if math.hypot(current_pose[0] - start_pose[0],
                          current_pose[1] - start_pose[1]) > settings["max_stationary_shift_m"]:
                raise MissionStop("chassis moved during wall heading measurement")
            if (not math.isfinite(reading_mm) or reading_mm <= 0 or
                    reading_mm > self.settings["wall_threshold_mm"]):
                return record({"status": "skipped_range", "cell": list(node),
                               "side": side, "reason": "fan scan did not hit the same wall",
                               "rejected_offset_deg": offset_deg,
                               "rejected_range_mm": reading_mm, "scans": scans})
            yaw_deg = self.last_scan_relative_yaw_deg + sensor["yaw_offset_deg"]
            ray = math.radians(yaw_deg)
            offset = math.radians(yaw_deg + sensor["offset_yaw_deg"])
            distance_m = reading_mm / 1000.0
            px = (sensor["pivot_x_m"] +
                  sensor["offset_from_yaw_axis_m"] * math.cos(offset) +
                  distance_m * math.cos(ray))
            py = (sensor["pivot_y_m"] +
                  sensor["offset_from_yaw_axis_m"] * math.sin(offset) +
                  distance_m * math.sin(ray))
            points.append((px, py))
            scans.append({"offset_deg": offset_deg, "yaw_deg": round(yaw_deg, 2),
                          "range_mm": round(reading_mm, 1)})
        center_yaw = scans[len(scans) // 2]["yaw_deg"]
        normal, span, residual = self._fit_wall_normal(
            points, center_yaw, settings["min_span_m"], settings["max_residual_m"])
        result = {"cell": list(node), "side": side, "scans": scans,
                  "span_m": round(span, 4),
                  "residual_m": round(residual, 4) if residual is not None else None}
        if normal is None:
            return record({**result, "status": "skipped_geometry",
                           "reason": "wall points are too close or not collinear"})
        observed_yaw = self._current_yaw()
        expected_normal = (self.base_pose[2] +
                           math.degrees(math.atan2(delta[1], delta[0])))
        correction = _wrap_degrees(expected_normal - normal - observed_yaw)
        result.update({"wall_normal_relative_deg": round(normal, 2),
                       "observed_yaw_deg": round(observed_yaw, 2),
                       "correction_deg": round(correction, 2)})
        previous_bias = getattr(self.chassis, "heading_bias_deg", 0.0)
        new_bias = _wrap_degrees(previous_bias + correction)
        self.chassis.set_heading_bias(new_bias)
        self.slam_worker.set_heading_bias(new_bias)
        return record({**result, "status": "applied", "heading_bias_deg": round(new_bias, 2)})

    def _can_step(self, node, destination):
        delta = (destination[0] - node[0], destination[1] - node[1])
        with self.lock:
            self.wall_grid.checks.pop((tuple(node), delta), None)
        worker_status = self.slam_worker.status() if self.slam_worker is not None else None
        if worker_status is not None and worker_status["error"]:
            raise MissionStop(worker_status["error"])
        pose = self.map.pose
        if pose is None:
            return False
        measured_mm, world_yaw = self._scan_for_direction(delta)
        try:
            measured_m = measured_mm / 1000.0
        except (TypeError, ValueError):
            return False
        if not math.isfinite(measured_m) or measured_m <= 0:
            return False
        move_yaw = math.radians(world_yaw)
        sensor_offset = self._sensor_offset(world_yaw)
        pose = self.map.pose
        center = self._to_map(node)
        pose_along = ((pose[0] - center[0]) * math.cos(move_yaw) +
                      (pose[1] - center[1]) * math.sin(move_yaw))
        with self.lock:
            self.wall_grid.observe(
                node, delta, measured_m + sensor_offset + pose_along,
                measured_mm, self.settings["wall_threshold_mm"],
                self.map.latest_scan_timestamp,
            )
            allowed = self.wall_grid.can_cross(node, delta)
        self.map.set_exploration_state(self.snapshot())
        return allowed

    def _align_cell(self, node, target_offset=(0.0, 0.0)):
        """Measure selected walls once, then move to one bounded PID target."""
        def skip(status, reason, pose=None, steps=0, after=None):
            result = {"status": status, "reason": reason, "steps": steps,
                      "target_distance_m": self.settings["alignment"]["wall_distance_m"]}
            with self.lock:
                self.alignments[node] = result
            self.map.set_exploration_state(self.snapshot())

        desired = self.settings["alignment"]["wall_distance_m"]
        if (self.last_ir_lane and self.last_ir_lane["status"] == "confirmed" and
                node == self.current_cell and list(node) in self.last_ir_lane["cells"]):
            skip("retained_ir_lane", "Keep the confirmed local IR lane instead of recentering")
            return
        tolerance = self.settings["alignment"]["tolerance_m"]
        max_shift = self.settings["alignment"]["max_shift_m"]
        walls = {}
        for delta in self.DIRECTIONS:
            check = self.wall_grid.checks.get((node, delta))
            if check is None or check["range_mm"] > check["wall_threshold_mm"]:
                continue
            yaw = self.base_pose[2] + math.degrees(math.atan2(delta[1], delta[0]))
            distance = check["range_mm"] / 1000.0 + self._sensor_offset(yaw)
            if not math.isfinite(distance) or distance <= 0:
                skip("skipped_invalid_wall", f"invalid wall distance at {node}")
                return
            walls[delta] = distance

        selected = []
        for positive, negative in (((1, 0), (-1, 0)), ((0, 1), (0, -1))):
            selected.append(tuple(delta for delta in (positive, negative) if delta in walls))

        measured_at = []
        for sides in selected:
            for delta in sides:
                reading, yaw = self._scan_for_direction(delta)
                if reading > self.settings["wall_threshold_mm"]:
                    skip("skipped_wall_missing", f"selected wall at {node} is no longer detected")
                    return
                distance = reading / 1000.0 + self._sensor_offset(yaw)
                walls[delta] = distance
                measured_at.append(self.map.latest_scan_timestamp)

        worker_status = self.slam_worker.status() if self.slam_worker is not None else None
        if (worker_status or {}).get("error"):
            raise MissionStop(worker_status["error"])
        if ((worker_status or {}).get("waiting_telemetry") or
                any(timestamp is None or
                    time.time() - timestamp > self.settings["max_sample_age_s"]
                    for timestamp in measured_at)):
            skip("skipped_stale_scan", "selected wall scan became stale before alignment")
            return

        corrections = []
        for axis, sides in enumerate(selected):
            if not sides:
                correction = 0.0
            elif len(sides) == 2:
                correction = (walls[sides[0]] - walls[sides[1]]) / 2
            elif sides[0][0] > 0 or sides[0][1] > 0:
                correction = walls[sides[0]] - desired
            else:
                correction = desired - walls[sides[0]]
            corrections.append(correction + target_offset[axis] if sides else 0.0)

        shift = math.hypot(*corrections)
        result = {"walls": {name: round(walls.get(delta), 4) for name, delta in
                            zip(CellWallGrid.SIDE_NAMES, self.DIRECTIONS)
                            if delta in walls},
                  "target_distance_m": desired, "correction_m":
                  [round(value, 4) for value in corrections],
                  "selected_sides": [CellWallGrid.SIDE_NAMES[self.DIRECTIONS.index(delta)]
                                     for sides in selected for delta in sides],
                  "axis_modes": {name: ("between_walls" if len(sides) == 2 else "single_wall")
                                 for name, sides in zip(("x", "y"), selected) if sides},
                  "measurement_scans": len(measured_at)}
        if shift <= tolerance:
            result["status"] = "within_tolerance" if walls else "no_wall"
            result["steps"] = 0
            with self.lock:
                self.alignments[node] = result
            self.map.set_exploration_state(self.snapshot())
            return

        x, y = self._motion_pose()[:2]
        if any(timestamp is None or
               time.time() - timestamp > self.settings["max_sample_age_s"]
               for timestamp in measured_at):
            skip("skipped_stale_scan", "selected wall scan became stale while waiting for pose")
            return
        fraction = min(1.0, max_shift / shift)
        move_x, move_y = (value * fraction for value in corrections)
        angle = math.radians(self.base_pose[2])
        target = (x + move_x * math.cos(angle) - move_y * math.sin(angle),
                  y + move_x * math.sin(angle) + move_y * math.cos(angle))
        if not self.map.contains_world(*target):
            skip("skipped_map_boundary", f"alignment target for {node} is outside the SLAM map")
            return
        self._set_status("aligning")
        pose = self._drive_holding_current_yaw(*target, kind="alignment")
        actual_x, actual_y = pose[:2]
        displacement = (actual_x - x, actual_y - y)
        estimated_after = {}
        for delta, distance in walls.items():
            wall_yaw = angle + math.atan2(delta[1], delta[0])
            along = displacement[0] * math.cos(wall_yaw) + displacement[1] * math.sin(wall_yaw)
            estimated_after[CellWallGrid.DELTA_TO_SIDE[delta]] = round(distance - along, 4)
        with self.lock:
            self.cell_targets[node] = (actual_x, actual_y)
            result["status"] = "partial" if fraction < 1.0 else "moved"
            result["estimated_after_m"] = estimated_after
            result["applied_correction_m"] = [round(move_x, 4), round(move_y, 4)]
            result["steps"] = 1
            self.alignments[node] = result
        self.map.set_exploration_state(self.snapshot())

    def _scan_all_directions(self, node):
        """Scan a cell once; reuse its recorded edges on later visits."""
        if node in self.scanned_cells:
            with self.lock:
                return {delta: self.wall_grid.can_cross(node, delta)
                        for delta in self.DIRECTIONS}
        previous_status = self.status
        for index, delta in enumerate(self.DIRECTIONS, 1):
            self._set_status(f"scanning_{index}_of_{len(self.DIRECTIONS)}")
            neighbor = (node[0] + delta[0], node[1] + delta[1])
            self._can_step(node, neighbor)
        with self.lock:
            clear = {delta: self.wall_grid.can_cross(node, delta) for delta in self.DIRECTIONS}
            self.scanned_cells.add(node)
        self._set_status(previous_status)
        return clear

    def _inspect_walls(self, node):
        if not self.settings["target_inspection"]["enabled"]:
            return
        if self.target_inspector is None:
            raise MissionStop("target inspection needs the dashboard camera and infrared blaster")
        for delta in self.DIRECTIONS:
            key = (tuple(node), delta)
            with self.lock:
                edge = self.wall_grid.edges.get(CellWallGrid.edge_key(node, delta), {})
                if (edge.get("state") != "wall" or edge.get("source") != "direct" or
                        key in self.wall_inspections):
                    continue
                self.wall_inspections[key] = {
                    "cell": list(node), "direction": list(delta),
                    "status": "checking", "targets": []}
            self._set_status("inspecting_wall_target")
            try:
                neighbor = (node[0] + delta[0], node[1] + delta[1])
                self._can_step(node, neighbor)
                with self.lock:
                    fresh_check = self.wall_grid.checks.get((tuple(node), delta))
                    fresh_state = self.wall_grid.state(node, delta)
                if fresh_check is None:
                    raise MissionStop("wall target inspection needs a fresh ToF wall check")
                if fresh_state != "wall":
                    with self.lock:
                        self.wall_inspections[key]["status"] = "wall_no_longer_present"
                    self.map.set_exploration_state(self.snapshot())
                    continue
                body_yaw = self._current_yaw()
                world_yaw = self.base_pose[2] + math.degrees(math.atan2(delta[1], delta[0]))
                range_mm = fresh_check.get("range_mm")
                try:
                    result = self.target_inspector.inspect(node, delta, world_yaw, body_yaw, range_mm=range_mm)
                except TypeError:
                    result = self.target_inspector.inspect(node, delta, world_yaw, body_yaw)
            except MissionStop as error:
                with self.lock:
                    self.wall_inspections[key]["status"] = "stopped"
                    self.wall_inspections[key]["reason"] = str(error)
                self.map.set_exploration_state(self.snapshot())
                raise
            with self.lock:
                self.wall_inspections[key] = result
            self.map.set_exploration_state(self.snapshot())
            for target_res in result.get("targets", []):
                if target_res.get("status") == "fire_command_accepted":
                    self._save_target_record(node, delta, target_res)
        self._set_status("exploring")

    def _save_target_record(self, cell, delta, target_res):
        try:
            from src.target_marker import (
                load_target_document, next_target_id, build_target_record, save_target_record
            )
            from pathlib import Path
            project_dir = Path(__file__).resolve().parent.parent
            targets_file = project_dir / "data" / "targets.json"
            doc = load_target_document(targets_file)
            tid = next_target_id(doc["targets"])
            detection_dict = {
                "color": target_res.get("color"),
                "shape": target_res.get("shape"),
                "center_px": target_res.get("center", [0, 0]),
                "center_offset_norm": [0.0, 0.0],
                "area_px2": target_res.get("area_fraction", 0.0) * (640 * 360),
                "stability_hits": self.settings.get("target_inspection", {}).get("confirm_frames", 3),
                "stability_required": self.settings.get("target_inspection", {}).get("confirm_frames", 3),
            }
            pose = self._motion_pose()
            gimbal_sample = self._gimbal_sample()
            gimbal = (gimbal_sample[1], gimbal_sample[0], 0, 0) if gimbal_sample else (0, 0, 0, 0)
            # In RoboMaster NED frame: delta (0, -1) is -90 deg (left wall = y+),
            # and delta (0, 1) is +90 deg (right wall = y-).
            delta_to_side = {(1, 0): "x+", (0, -1): "y+", (-1, 0): "x-", (0, 1): "y-"}
            side = delta_to_side.get(tuple(delta))
            rec = build_target_record(
                tid, cell, detection_dict, pose=pose, gimbal=gimbal, side=side,
                fired=True, fire_times=target_res.get("shots_requested", 2),
            )
            save_target_record(targets_file, rec, replace=False)
            print(f"[explorer] Auto-marked and saved target {tid} ({rec['color']} {rec['shape']}) at cell {cell} to {targets_file}")
        except Exception as e:
            import sys
            print(f"[explorer] Warning: failed to auto-save target: {e}", file=sys.stderr)

    def _set_target_progress(self, progress):
        with self.lock:
            self.target_progress = progress
        self.map.set_exploration_state(self.snapshot())

    def _can_return(self, child, parent):
        """Check the shared map edge without moving the gimbal again."""
        worker_status = self.slam_worker.status() if self.slam_worker is not None else None
        if worker_status is not None and worker_status["error"]:
            raise MissionStop(worker_status["error"])
        back = (parent[0] - child[0], parent[1] - child[1])
        with self.lock:
            return self.wall_grid.state(child, back) == "open"

    def _frontier_cells(self):
        """Visited cells with a confirmed open edge into an unvisited cell."""
        with self.lock:
            visited = set(self.visited)
            scanned = set(self.scanned_cells)
            return {node for node in visited & scanned
                    if any((node[0] + delta[0], node[1] + delta[1]) not in visited
                           and self.wall_grid.can_cross(node, delta)
                           for delta in self.DIRECTIONS)}

    def _shortest_route(self, goals):
        """A* over scanned, visited cells; prefer fewer turns among equal lengths."""
        with self.lock:
            start = self.current_cell
            visited = set(self.visited)
            scanned = set(self.scanned_cells)
            goals = set(goals) & visited & scanned
            if not goals or start not in visited:
                return None
            if start in goals:
                return [start]

            def heuristic(node):
                return min(abs(node[0] - goal[0]) + abs(node[1] - goal[1])
                           for goal in goals)

            queue = [(heuristic(start), 0, 0, start, None, [start])]
            best = {(start, None): (0, 0)}
            while queue:
                _, steps, turns, node, previous_delta, path = heapq.heappop(queue)
                if (steps, turns) != best.get((node, previous_delta)):
                    continue
                if node in goals:
                    return path
                for delta in self.DIRECTIONS:
                    neighbor = (node[0] + delta[0], node[1] + delta[1])
                    if (neighbor not in visited or neighbor not in scanned or
                            not self.wall_grid.can_cross(node, delta)):
                        continue
                    cost = (steps + 1, turns + int(previous_delta is not None
                                                  and previous_delta != delta))
                    key = (neighbor, delta)
                    if cost < best.get(key, (float("inf"), float("inf"))):
                        best[key] = cost
                        heapq.heappush(queue, (cost[0] + heuristic(neighbor),
                                               cost[0], cost[1], neighbor, delta,
                                               path + [neighbor]))
            return None

    def _record_step(self, destination):
        """Keep the displayed trail ending at the robot after any grid step."""
        with self.lock:
            if destination in self.stack:
                self.stack = self.stack[:self.stack.index(destination) + 1]
            else:
                self.stack.append(destination)

    def _assess_stopped_center(self, cell, pose, wall_distance_m=None):
        """Keep the planned center separate from a partial emergency-stop pose."""
        planned = self.cell_targets.get(cell, self._to_map(cell))
        offset = (pose[0] - planned[0], pose[1] - planned[1])
        gap = math.hypot(*offset)
        tolerance = self.settings["alignment"]["tolerance_m"]
        confirmation = "planned_pose" if gap <= tolerance else None
        wall_error = None
        if wall_distance_m is not None:
            wall_error = wall_distance_m - self.settings["alignment"]["wall_distance_m"]
        result = {
            "center_confirmed": confirmation is not None,
            "center_confirmation": confirmation,
            "planned_center_m": [round(planned[0], 4), round(planned[1], 4)],
            "offset_from_planned_center_m": [round(value, 4) for value in offset],
            "planned_center_error_m": round(gap, 4),
            "wall_target_error_m": round(wall_error, 4) if wall_error is not None else None,
        }
        return result

    def _move(self, destination):
        with self.lock:
            source = self.current_cell
            target = self.cell_targets.get(tuple(destination))
            if target is None:
                origin = self.cell_targets.get(source, self._to_map(source))
                step = self.settings["step_m"]
                du = (destination[0] - source[0]) * step
                dv = (destination[1] - source[1]) * step
                angle = math.radians(self.base_pose[2])
                target = (origin[0] + du * math.cos(angle) - dv * math.sin(angle),
                          origin[1] + du * math.sin(angle) + dv * math.cos(angle))
                self.cell_targets[tuple(destination)] = target
        delta = (destination[0] - source[0], destination[1] - source[1])
        if delta not in self.DIRECTIONS:
            raise ValueError("grid movement needs an adjacent destination")
        lane_settings = self.settings.get("ir_lane", {})
        lane_enabled = lane_settings.get("enabled", False)
        edge = tuple(sorted((source, tuple(destination))))
        saved_lane = self.ir_lanes.get(edge) if lane_enabled else None
        # Anchors belong to this edge only. Never translate cell_targets or the
        # grid origin, which would propagate a local correction to other edges.
        _, lane, normal, destination_index, lane_target = prepare_lane(
            source, destination,
            {node: self.cell_targets.get(node, self._to_map(node)) for node in edge},
            self.base_pose[2], saved_lane)
        if saved_lane:
            target = lane_target
        candidate_changed = False

        def retarget_after_ir(before, after, old_target):
            nonlocal x, y, candidate_changed, lane
            correction = retarget_lane(lane, normal, destination_index, before, after,
                                       lane_settings, self.map.contains_world)
            if correction is None:
                return None
            lane, shifted = correction
            candidate_changed = True
            x, y = shifted
            with self.lock:
                self.last_ir_lane = {**lane, "status": "candidate", "target_m": [x, y]}
                self.last_motion_heading["target_m"] = [x, y]
            self.map.set_exploration_state(self.snapshot())
            return shifted

        x, y = target
        if lane_enabled:
            with self.lock:
                self.last_ir_lane = {**lane, "status": "reused" if saved_lane else "original",
                                     "target_m": [x, y]}
        if not self.map.contains_world(x, y):
            raise MissionStop(f"grid target for {destination} is outside the SLAM map")
        # A return to a visited cell needs one fresh scan in the travel
        # direction for the emergency monitor, but not a four-sided scan.
        measured_mm, world_yaw = self._scan_for_direction(delta)
        start_pose = self._motion_pose()
        move_yaw = math.radians(world_yaw)
        center = self._to_map(source)
        pose_along = ((start_pose[0] - center[0]) * math.cos(move_yaw) +
                      (start_pose[1] - center[1]) * math.sin(move_yaw))
        distance = measured_mm / 1000.0 + self._sensor_offset(world_yaw) + pose_along
        emergency_distance = self.settings["emergency_stop_distance_m"]
        if measured_mm <= self.settings["wall_threshold_mm"]:
            with self.lock:
                if lane_enabled:
                    self.ir_lanes.pop(edge, None)
                    self.last_ir_lane = {**self.last_ir_lane, "status": "blocked"}
                center_result = self._assess_stopped_center(source, start_pose)
                wall_confirmed = center_result["center_confirmed"]
                if wall_confirmed:
                    self.wall_grid.observe(source, delta, distance, measured_mm,
                                           self.settings["wall_threshold_mm"],
                                           self.map.latest_scan_timestamp)
                if tuple(destination) not in self.visited:
                    self.cell_targets.pop(tuple(destination), None)
                self.last_motion_stop = {
                    "status": "blocked_before_move",
                    "wall_confirmed": wall_confirmed,
                    "source_cell": list(source), "destination_cell": list(destination),
                    "center_cell": list(source), "actual_pose": list(start_pose),
                    "target_m": [x, y], "entered_destination": False,
                    "range_mm": measured_mm, "center_distance_m": round(distance, 4),
                    **center_result,
                }
            self.map.set_exploration_state(self.snapshot())
            return False

        stopped = {}
        sensor_yaw_offset = float(self.settings["sensor"]["yaw_offset_deg"])
        alignment_misses = 0
        alignment_last_timestamp = None

        def stop_if(pose):
            nonlocal alignment_misses, alignment_last_timestamp
            previous_status = self.status
            waiting = False
            while True:
                worker_status = self.slam_worker.status()
                if worker_status["error"]:
                    raise MissionStop(worker_status["error"])
                tof = self.logger.get_sample("tof", max_age_s=self.settings["max_sample_age_s"])
                if not worker_status.get("waiting_telemetry") and tof is not None:
                    scan_yaw, scan_pitch, angle_timestamp = self._gimbal_sample()
                    if abs(tof[1] - angle_timestamp) <= self.settings["sample_skew_s"]:
                        getter = getattr(self.chassis, "get_pose", None)
                        live_pose = getter() if callable(getter) else pose
                        if live_pose is not None:
                            pose = live_pose
                            break
                if not waiting:
                    self._set_status("waiting_telemetry")
                    waiting = True
                self.chassis.stop()
                time.sleep(0.05)
            if waiting:
                self._set_status(previous_status)
            target_yaw = _wrap_degrees(world_yaw - pose[2] - sensor_yaw_offset)
            tolerance = self.settings["gimbal"].get(
                "movement_angle_tolerance_deg",
                self.settings["gimbal"]["angle_tolerance_deg"],
            )
            pitch_tolerance = self.settings["gimbal"]["pitch_tolerance_deg"]
            yaw_error = abs(_wrap_degrees(target_yaw - scan_yaw))
            pitch_error = abs(self.settings["gimbal"]["pitch_deg"] - scan_pitch)
            if yaw_error > tolerance or pitch_error > pitch_tolerance:
                # Count only fresh telemetry, not repeated reads of one sample.
                if angle_timestamp != alignment_last_timestamp:
                    alignment_misses += 1
                    alignment_last_timestamp = angle_timestamp
                required_misses = self.settings["gimbal"].get(
                    "movement_alignment_miss_samples", 3
                )
                if alignment_misses >= required_misses:
                    raise MissionStop(
                        "movement ToF alignment lost for {} samples: "
                        "yaw target {:.1f}, actual {:.1f}, error {:.2f}; "
                        "pitch target {:.1f}, actual {:.1f}, error {:.2f}".format(
                            alignment_misses, target_yaw, scan_yaw, yaw_error,
                            self.settings["gimbal"]["pitch_deg"], scan_pitch,
                            pitch_error))
                # Ignore a short telemetry spike; a later aligned sample resets it.
                return False
            alignment_misses = 0
            alignment_last_timestamp = angle_timestamp
            scan_distance = self.map._range_value(tof[0])
            if scan_distance is None:
                return False
            center_distance = scan_distance + self._sensor_offset(world_yaw)
            too_close = center_distance <= emergency_distance
            if getattr(self.chassis, "directional_tof", None) is not None:
                from src.directional_recovery import edge_margin
                too_close = edge_margin(self, scan_distance, world_yaw - pose[2], pose[2]) <= 0
            if too_close:
                stopped.update({"range_mm": scan_distance * 1000.0,
                                "center_distance_m": round(center_distance, 4),
                                "timestamp": tof[1]})
                return True
            return False

        previous_scan = self.map.latest_scan_timestamp or 0.0
        previous_status = self.status
        try:
            pose = self._drive_holding_current_yaw(
                x, y, kind="grid_step", stop_if=stop_if,
                on_ir_recovered=retarget_after_ir if lane_enabled else None)
        except Exception as error:
            if lane_enabled:
                with self.lock:
                    self.ir_lanes.pop(edge, None)
                    self.last_ir_lane = {**self.last_ir_lane, "status": "aborted"}
                    if isinstance(error, MissionStop):
                        self.last_ir_lane["reason"] = str(error)
                self.map.set_exploration_state(self.snapshot())
            raise
        entered = True
        if stopped:
            path_x, path_y = x - start_pose[0], y - start_pose[1]
            path_length = math.hypot(path_x, path_y)
            progress = ((pose[0] - start_pose[0]) * path_x +
                        (pose[1] - start_pose[1]) * path_y) / max(path_length, 1e-9)
            entered = progress >= path_length / 2.0
            center_cell = tuple(destination) if entered else source
            # Emergency range stops wheels immediately; only a stationary,
            # correctly centered remeasurement may change map topology.
            verified_mm, verified_yaw = self._scan_for_direction(delta)
            with self.lock:
                if lane_enabled:
                    self.ir_lanes.pop(edge, None)
                    self.last_ir_lane = {**self.last_ir_lane, "status": "blocked"}
                center_result = self._assess_stopped_center(
                    center_cell, pose, stopped["center_distance_m"])
                if not entered and tuple(destination) not in self.visited:
                    self.cell_targets.pop(tuple(destination), None)
                planned = center_result["planned_center_m"]
                center_offset_along = ((pose[0] - planned[0]) * math.cos(move_yaw) +
                                       (pose[1] - planned[1]) * math.sin(move_yaw))
                wall_confirmed = (center_result["center_confirmed"] and
                                  verified_mm <= self.settings["wall_threshold_mm"])
                if wall_confirmed:
                    self.wall_grid.observe(center_cell, delta,
                                           verified_mm / 1000 + self._sensor_offset(verified_yaw) + center_offset_along,
                                           verified_mm, self.settings["wall_threshold_mm"],
                                           self.map.latest_scan_timestamp)
                self.last_motion_stop = {
                    "status": ("emergency_stop_centered" if center_result["center_confirmed"]
                               else "emergency_stop_off_center"),
                    "source_cell": list(source), "destination_cell": list(destination),
                    "center_cell": list(center_cell), "actual_pose": list(pose),
                    "target_m": [x, y], "entered_destination": entered,
                    "range_mm": stopped["range_mm"],
                    "wall_confirmed": wall_confirmed,
                    "verified_range_mm": verified_mm,
                    "center_distance_m": stopped["center_distance_m"],
                    **center_result,
                }
            self.map.set_exploration_state(self.snapshot())
            if not entered:
                return False
        with self.lock:
            if lane_enabled and not stopped and (candidate_changed or saved_lane):
                self.ir_lanes[edge] = dict(lane)
                self.last_ir_lane = {**lane, "status": "confirmed", "target_m": [x, y]}
            self.moves += 1
            self.current_cell = tuple(destination)
        self._set_status("waiting_slam_scan")
        while True:
            if (self.map.latest_scan_timestamp or 0.0) > previous_scan:
                self._set_status(previous_status)
                return True
            worker_status = self.slam_worker.status()
            if worker_status["error"]:
                raise MissionStop(worker_status["error"])
            time.sleep(0.05)

    def run(self, slam_worker):
        """Explore new cells, routing to the nearest remaining frontier."""
        self.slam_worker = slam_worker
        try:
            self._set_status("starting")
            if self.settings.get("target_inspection", {}).get("enabled", True):
                try:
                    from src.target_marker import init_target_document
                    from pathlib import Path
                    targets_file = Path(__file__).resolve().parent.parent / "data" / "targets.json"
                    init_target_document(targets_file, backup=True)
                except Exception as _e:
                    print(f"[explorer] Notice: targets file reset skipped: {_e}")
            slam_worker.wait_ready()
            self._prepare_gimbal()
            pose = self.map.pose
            if pose is None:
                raise MissionStop("SLAM has no initial pose")
            initial_heading = self._current_yaw()
            with self.lock:
                self.base_pose = tuple(pose)
                self.travel_heading_deg = initial_heading
                self.wall_grid.cells.add((0, 0))
                self.cell_targets[(0, 0)] = tuple(self._motion_pose()[:2])
            root = (0, 0)
            with self.lock:
                self.stack = [root]
                self.route = []
                self.route_goal = None
                self.route_plans = 0
                self.route_blockages = 0
                self.last_alignment_move = None
                self.visited = {root}
            self._set_status("exploring")
            while self.stack:
                with self.lock:
                    current = self.stack[-1]
                    node_count = len(self.visited)
                if node_count >= self.settings["max_nodes"]:
                    # Keep the robot at the last explored cell. Backtracking is
                    # still used only while DFS needs to reach another branch.
                    self._set_status("node_limit_reached")
                    return self.snapshot()

                clear = self._scan_all_directions(current)
                interval = self.settings["alignment"].get("interval_steps", 2)
                alignment_due = (self.last_alignment_move is None or
                                 self.moves - self.last_alignment_move >= interval)
                if alignment_due:
                    if self.settings["heading_alignment"]["enabled"]:
                        self._align_heading(current)
                        self._set_status("exploring")
                    if self.settings["alignment"]["enabled"]:
                        self._align_cell(current)
                        self._set_status("exploring")
                    if (self.settings["heading_alignment"]["enabled"] or
                            self.settings["alignment"]["enabled"]):
                        with self.lock:
                            self.last_alignment_move = self.moves
                    clear = {delta: self.wall_grid.can_cross(current, delta)
                             for delta in self.DIRECTIONS}
                self._inspect_walls(current)
                clear = {delta: self.wall_grid.can_cross(current, delta)
                         for delta in self.DIRECTIONS}
                moved_to = None
                for delta in self.DIRECTIONS:
                    neighbor = (current[0] + delta[0], current[1] + delta[1])
                    if neighbor in self.visited:
                        continue
                    if clear[delta]:
                        self._set_status(f"moving_to_{neighbor[0]}_{neighbor[1]}")
                        entered = self._move(neighbor)
                        if entered:
                            with self.lock:
                                self.visited.add(neighbor)
                            self._record_step(neighbor)
                        if not entered:
                            continue
                        moved_to = neighbor
                        break
                if moved_to is not None:
                    self._set_status("exploring")
                    continue

                if self.moves == 0 and current == root:
                    self._set_status(
                        "no_safe_direction",
                        "กริดยังไม่มีขอบทางเปิดที่ ToF เกินเกณฑ์กำแพง",
                    )
                    return self.snapshot()

                frontiers = self._frontier_cells()
                if not frontiers and current == root:
                    break
                goals = frontiers if frontiers else {root}
                route = self._shortest_route(goals)
                if route is None:
                    raise MissionStop("No confirmed open route to a remaining frontier or start cell")
                with self.lock:
                    self.route = route
                    self.route_goal = route[-1]
                    self.route_plans += 1
                for destination in route[1:]:
                    source = self.current_cell
                    delta = (destination[0] - source[0],
                             destination[1] - source[1])
                    if not self.wall_grid.can_cross(source, delta):
                        with self.lock:
                            self.route_blockages += 1
                        break
                    self._set_status(f"routing_to_{destination[0]}_{destination[1]}")
                    if not self._move(destination):
                        with self.lock:
                            self.route_blockages += 1
                        break
                    self._record_step(destination)
                    with self.lock:
                        self.route = self.route[1:]
                with self.lock:
                    self.route = []
                    self.route_goal = None
                self._set_status("exploring")

            self._set_status("completed")
            return self.snapshot()
        except MissionStop as error:
            self._set_status("stopped", str(error))
            raise
        except Exception as error:
            self._set_status("failed", str(error))
            raise
