"""Depth-first frontier traversal over a SLAM occupancy grid."""

import math
import statistics
import threading
import time

from src.slam import CellWallGrid, _wrap_degrees
from src.mission_stop import MissionStop


class DFSExplorer:
    """Traverse shared open cell borders observed with the gimbal ToF."""

    DIRECTIONS = ((1, 0), (0, -1), (-1, 0), (0, 1))

    def __init__(self, chassis, gimbal, logger, slam_map, settings):
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
        self.moves = 0
        self.base_pose = None
        self.current_cell = (0, 0)
        self.cell_targets = {}
        self.alignments = {}
        self.last_motion_heading = None
        self.last_motion_stop = None
        self.scan_alignment = None
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
                "visited_points": [list(self.cell_targets.get(node, self._to_map(node)))
                                   for node in sorted(self.visited)]
                if self.base_pose is not None else [],
                "stack_points": [list(self.cell_targets.get(node, self._to_map(node)))
                                 for node in self.stack]
                if self.base_pose is not None else [],
                "moves": self.moves,
                "heading_source": self.settings["heading_source"],
                "gimbal_pitch_frame": "chassis",
                "alignment_enabled": self.settings["alignment"]["enabled"],
                "emergency_stop_distance_m": self.settings["emergency_stop_distance_m"],
                "last_motion_heading": self.last_motion_heading,
                "last_motion_stop": self.last_motion_stop,
                "scan_alignment": self.scan_alignment,
                "tof_median_window": self.settings["tof_median_window"],
                "cell_targets": {f"{node[0]},{node[1]}": list(point)
                                 for node, point in sorted(self.cell_targets.items())},
                "alignments": {f"{node[0]},{node[1]}": result
                               for node, result in sorted(self.alignments.items())},
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
        if self.settings["heading_source"] == "gimbal":
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

    def _drive_holding_current_yaw(self, x, y, kind, stop_if=None):
        """Capture the live chassis heading immediately before this motion."""
        yaw = self._current_yaw()
        with self.lock:
            self.last_motion_heading = {"yaw_deg": yaw, "target_m": [x, y],
                                        "kind": kind, "source": self.settings["heading_source"]}
        self.map.set_exploration_state(self.snapshot())
        options = {"yaw": yaw, "abort_event": self.slam_worker.abort_event,
                   "disable_timeout": True,
                   "pause_if": lambda: self.slam_worker.status().get("waiting_telemetry", False)}
        if stop_if is not None:
            options["stop_if"] = stop_if
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
        return min(reachable, key=lambda angle: abs(angle - current_relative_yaw))

    def _prepare_gimbal(self):
        """Physically center the head once and wait for SDK completion."""
        if not self.settings["gimbal"]["auto_recenter"]:
            return
        self._set_status("centering_gimbal")
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

    def _scan_for_direction(self, delta):
        """Point the single ToF toward a candidate cell and wait for its new scan."""
        worker_status = self.slam_worker.status() if self.slam_worker is not None else None
        if worker_status is not None and worker_status["error"]:
            raise MissionStop(worker_status["error"])
        world_yaw = self.base_pose[2] + math.degrees(math.atan2(delta[1], delta[0]))
        body_yaw = self._current_yaw()
        sensor = self.settings["sensor"]
        target_yaw = _wrap_degrees(
            world_yaw - body_yaw - float(sensor["yaw_offset_deg"])
        )
        current_yaw, _, _ = self._gimbal_sample()
        command_yaw = self._command_yaw(target_yaw, current_yaw)
        request_time = time.time()
        tolerance = self.settings["gimbal"]["angle_tolerance_deg"]
        pitch_tolerance = self.settings["gimbal"]["pitch_tolerance_deg"]
        # Center yaw without recenter(), which uses a separate SDK action and
        # previously remained active after angle telemetry had settled.
        if abs(target_yaw) <= tolerance:
            command_yaw = 0.0
        previous_status = self.status
        self._set_status("scanning")
        action = self.gimbal.moveto(
            pitch=self.settings["gimbal"]["pitch_deg"],
            yaw=command_yaw,
            pitch_speed=30,
            yaw_speed=self.settings["gimbal"]["yaw_speed_deg_s"],
        )

        aligned = False
        measured_yaw = current_yaw
        action_confirmed = action is None
        median_window = self.settings["tof_median_window"]
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
                    request_time = time.time()
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

            scan_timestamp = self.map.latest_scan_timestamp
            scan_yaw = self.map.latest_gimbal_yaw_deg
            scan_range = self.map.latest_range_mm
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
                    self._set_status(previous_status)
                    return float(statistics.median(value for _, value in samples)), world_yaw
            time.sleep(0.03)

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

    def _align_cell(self, node):
        """Center between two walls, or use the configured distance to one wall."""
        def skip(status, reason, pose=None, steps=0, after=None):
            result = {"status": status, "reason": reason, "steps": steps,
                      "target_distance_m": self.settings["alignment"]["wall_distance_m"]}
            if after is not None:
                result["after_m"] = after
            with self.lock:
                if pose is not None:
                    self.cell_targets[node] = tuple(pose)
                self.alignments[node] = result
            self.map.set_exploration_state(self.snapshot())

        desired = self.settings["alignment"]["wall_distance_m"]
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

        for sides in selected:
            for delta in sides:
                reading, yaw = self._scan_for_direction(delta)
                if reading > self.settings["wall_threshold_mm"]:
                    skip("skipped_wall_missing", f"selected wall at {node} is no longer detected")
                    return
                walls[delta] = reading / 1000.0 + self._sensor_offset(yaw)

        corrections = []
        for sides in selected:
            if not sides:
                correction = 0.0
            elif len(sides) == 2:
                correction = (walls[sides[0]] - walls[sides[1]]) / 2
            elif sides[0][0] > 0 or sides[0][1] > 0:
                correction = walls[sides[0]] - desired
            else:
                correction = desired - walls[sides[0]]
            corrections.append(correction)

        shift = math.hypot(*corrections)
        result = {"walls": {name: round(walls.get(delta), 4) for name, delta in
                            zip(CellWallGrid.SIDE_NAMES, self.DIRECTIONS)
                            if delta in walls},
                  "target_distance_m": desired, "correction_m":
                  [round(value, 4) for value in corrections],
                  "selected_sides": [CellWallGrid.SIDE_NAMES[self.DIRECTIONS.index(delta)]
                                     for sides in selected for delta in sides],
                  "axis_modes": {name: ("between_walls" if len(sides) == 2 else "single_wall")
                                 for name, sides in zip(("x", "y"), selected) if sides}}
        if shift <= tolerance:
            result["status"] = "within_tolerance" if walls else "no_wall"
            result["steps"] = 0
            with self.lock:
                self.alignments[node] = result
            self.map.set_exploration_state(self.snapshot())
            return

        x, y = self._motion_pose()[:2]
        angle = math.radians(self.base_pose[2])
        self._set_status("aligning")
        verified = {}
        steps = 0
        for axis, sides in enumerate(selected):
            if not sides:
                continue
            while True:
                current = {}
                for delta in sides:
                    reading, yaw = self._scan_for_direction(delta)
                    if reading > self.settings["wall_threshold_mm"]:
                        skip("skipped_wall_missing", f"selected wall at {node} is no longer detected",
                             (x, y), steps, verified)
                        return
                    distance = reading / 1000.0 + self._sensor_offset(yaw)
                    current[delta] = (distance, reading, yaw)
                    name = CellWallGrid.SIDE_NAMES[self.DIRECTIONS.index(delta)]
                    verified[name] = round(distance, 4)
                if len(sides) == 2:
                    correction = (current[sides[0]][0] - current[sides[1]][0]) / 2
                elif sides[0][axis] > 0:
                    correction = current[sides[0]][0] - desired
                else:
                    correction = desired - current[sides[0]][0]
                if abs(correction) <= tolerance:
                    break
                # max_shift_m limits each PID move. A large correction is
                # measured again after the first move instead of failing.
                step = max(-max_shift, min(max_shift, correction))
                step_x = step * (math.cos(angle) if axis == 0 else -math.sin(angle))
                step_y = step * (math.sin(angle) if axis == 0 else math.cos(angle))
                target = (x + step_x, y + step_y)
                if not self.map.contains_world(*target):
                    skip("skipped_map_boundary", f"alignment target for {node} is outside the SLAM map",
                         (x, y), steps, verified)
                    return
                pose = self._drive_holding_current_yaw(*target, kind="alignment")
                x, y = pose[:2]
                steps += 1
                actual = {}
                for delta in sides:
                    reading, yaw = self._scan_for_direction(delta)
                    if reading > self.settings["wall_threshold_mm"]:
                        skip("skipped_wall_missing", f"selected wall at {node} is no longer detected",
                             (x, y), steps, verified)
                        return
                    distance = reading / 1000.0 + self._sensor_offset(yaw)
                    actual[delta] = distance
                    name = CellWallGrid.SIDE_NAMES[self.DIRECTIONS.index(delta)]
                    verified[name] = round(distance, 4)
                if len(sides) == 2:
                    remaining = (actual[sides[0]] - actual[sides[1]]) / 2
                elif sides[0][axis] > 0:
                    remaining = actual[sides[0]] - desired
                else:
                    remaining = desired - actual[sides[0]]
                if (abs(remaining) > tolerance and
                        abs(remaining) >= abs(correction) - .005):
                    skip("stalled", f"remaining correction {remaining:.3f} m at {node}",
                         (x, y), steps, verified)
                    return
        with self.lock:
            self.cell_targets[node] = (x, y)
            result["status"] = "moved"
            result["after_m"] = verified
            result["steps"] = steps
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

    def _can_return(self, child, parent):
        """Check the shared map edge without moving the gimbal again."""
        worker_status = self.slam_worker.status() if self.slam_worker is not None else None
        if worker_status is not None and worker_status["error"]:
            raise MissionStop(worker_status["error"])
        back = (parent[0] - child[0], parent[1] - child[1])
        with self.lock:
            return self.wall_grid.state(child, back) == "open"

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
        x, y = target
        if not self.map.contains_world(x, y):
            raise MissionStop(f"grid target for {destination} is outside the SLAM map")
        # A return to a visited cell needs one fresh scan in the travel
        # direction for the emergency monitor, but not a four-sided scan.
        measured_mm, world_yaw = self._scan_for_direction(delta)
        distance = measured_mm / 1000.0 + self._sensor_offset(world_yaw)
        start_pose = self._motion_pose()
        emergency_distance = self.settings["emergency_stop_distance_m"]
        if measured_mm <= self.settings["wall_threshold_mm"]:
            with self.lock:
                self.wall_grid.observe(source, delta, distance, measured_mm,
                                       self.settings["wall_threshold_mm"],
                                       self.map.latest_scan_timestamp)
                self.cell_targets[source] = tuple(start_pose[:2])
                if tuple(destination) not in self.visited:
                    self.cell_targets.pop(tuple(destination), None)
                self.last_motion_stop = {
                    "status": ("emergency_stop_centered" if distance <= emergency_distance
                               else "blocked_before_move"),
                    "source_cell": list(source), "destination_cell": list(destination),
                    "center_cell": list(source), "actual_pose": list(start_pose),
                    "target_m": [x, y], "entered_destination": False,
                    "range_mm": measured_mm, "center_distance_m": round(distance, 4),
                }
            self.map.set_exploration_state(self.snapshot())
            return False

        stopped = {}
        sensor_yaw_offset = float(self.settings["sensor"]["yaw_offset_deg"])

        def stop_if(pose):
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
            tolerance = self.settings["gimbal"]["angle_tolerance_deg"]
            pitch_tolerance = self.settings["gimbal"]["pitch_tolerance_deg"]
            if (abs(_wrap_degrees(target_yaw - scan_yaw)) > tolerance or
                    abs(self.settings["gimbal"]["pitch_deg"] - scan_pitch) > pitch_tolerance):
                raise MissionStop(
                    "movement ToF alignment lost: yaw target {:.1f}, actual {:.1f}; "
                    "pitch target {:.1f}, actual {:.1f}".format(
                        target_yaw, scan_yaw,
                        self.settings["gimbal"]["pitch_deg"], scan_pitch))
            scan_distance = self.map._range_value(tof[0])
            if scan_distance is None:
                return False
            center_distance = scan_distance + self._sensor_offset(world_yaw)
            if center_distance <= emergency_distance:
                stopped.update({"range_mm": scan_distance * 1000.0,
                                "center_distance_m": round(center_distance, 4),
                                "timestamp": tof[1]})
                return True
            return False

        previous_scan = self.map.latest_scan_timestamp or 0.0
        previous_status = self.status
        pose = self._drive_holding_current_yaw(x, y, kind="grid_step", stop_if=stop_if)
        entered = True
        if stopped:
            path_x, path_y = x - start_pose[0], y - start_pose[1]
            path_length = math.hypot(path_x, path_y)
            progress = ((pose[0] - start_pose[0]) * path_x +
                        (pose[1] - start_pose[1]) * path_y) / max(path_length, 1e-9)
            entered = progress >= path_length / 2.0
            center_cell = tuple(destination) if entered else source
            with self.lock:
                self.cell_targets[center_cell] = tuple(pose[:2])
                if not entered and tuple(destination) not in self.visited:
                    self.cell_targets.pop(tuple(destination), None)
                self.wall_grid.observe(center_cell, delta,
                                       stopped["center_distance_m"],
                                       stopped["range_mm"],
                                       self.settings["wall_threshold_mm"],
                                       stopped["timestamp"])
                self.last_motion_stop = {
                    "status": "emergency_stop_centered",
                    "source_cell": list(source), "destination_cell": list(destination),
                    "center_cell": list(center_cell), "actual_pose": list(pose),
                    "target_m": [x, y], "entered_destination": entered,
                    "range_mm": stopped["range_mm"],
                    "center_distance_m": stopped["center_distance_m"],
                }
            self.map.set_exploration_state(self.snapshot())
            if not entered:
                return False
        with self.lock:
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
        """Explore new cells with fresh scans, then return over known open edges."""
        self.slam_worker = slam_worker
        try:
            self._set_status("starting")
            slam_worker.wait_ready()
            self._prepare_gimbal()
            pose = self.map.pose
            if pose is None:
                raise MissionStop("SLAM has no initial pose")
            with self.lock:
                self.base_pose = tuple(pose)
                self.wall_grid.cells.add((0, 0))
                self.cell_targets[(0, 0)] = tuple(self._motion_pose()[:2])
            root = (0, 0)
            with self.lock:
                self.stack = [root]
                self.visited = {root}
            self._set_status("exploring")
            while self.stack:
                with self.lock:
                    current = self.stack[-1]
                    node_count = len(self.visited)
                if node_count >= self.settings["max_nodes"]:
                    self._set_status("node_limit_returning")
                    while len(self.stack) > 1:
                        with self.lock:
                            child, parent = self.stack[-1], self.stack[-2]
                        if not self._can_return(child, parent):
                            raise MissionStop("DFS cannot safely return to the start cell")
                        self._set_status("returning_to_start")
                        if not self._move(parent):
                            raise MissionStop("DFS cannot safely return to the start cell")
                        with self.lock:
                            self.stack.pop()
                    self._set_status("node_limit_returned")
                    return self.snapshot()

                newly_scanned = current not in self.scanned_cells
                clear = self._scan_all_directions(current)
                if newly_scanned and self.settings["alignment"]["enabled"]:
                    self._align_cell(current)
                    clear = {delta: self.wall_grid.can_cross(current, delta)
                             for delta in self.DIRECTIONS}
                    self._set_status("exploring")
                next_node = None
                for delta in self.DIRECTIONS:
                    neighbor = (current[0] + delta[0], current[1] + delta[1])
                    if neighbor in self.visited:
                        continue
                    if clear[delta] and self._can_step(current, neighbor):
                        next_node = neighbor
                        break

                if next_node is not None:
                    self._set_status(f"moving_to_{next_node[0]}_{next_node[1]}")
                    if self._move(next_node):
                        with self.lock:
                            self.stack.append(next_node)
                            self.visited.add(next_node)
                    self._set_status("exploring")
                    continue

                if self.moves == 0 and len(self.stack) == 1:
                    self._set_status(
                        "no_safe_direction",
                        "กริดยังไม่มีขอบทางเปิดที่ ToF เกินเกณฑ์กำแพง",
                    )
                    return self.snapshot()

                with self.lock:
                    finished = self.stack.pop()
                    parent = self.stack[-1] if self.stack else None
                if parent is not None:
                    if not self._can_return(finished, parent):
                        raise MissionStop("DFS backtrack path is no longer clear")
                    self._set_status(f"backtracking_to_{parent[0]}_{parent[1]}")
                    if not self._move(parent):
                        raise MissionStop("DFS cannot safely backtrack to the parent cell")
                    self._set_status("exploring")

            self._set_status("completed")
            return self.snapshot()
        except MissionStop as error:
            self._set_status("stopped", str(error))
            raise
        except Exception as error:
            self._set_status("failed", str(error))
            raise
