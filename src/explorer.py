"""Depth-first frontier traversal over a SLAM occupancy grid."""

import math
import statistics
import threading
import time

from src.slam import CellWallGrid, _wrap_degrees


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
                "alignment_enabled": self.settings["alignment"]["enabled"],
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

    def _current_yaw(self):
        if self.settings["heading_source"] == "gimbal":
            getter = getattr(self.chassis, "get_pose", None)
            if callable(getter):
                pose = getter()
                if pose is None:
                    raise TimeoutError("gimbal-derived chassis yaw is missing or stale")
                return float(pose[2])
        sample = self.logger.get_sample("attitude", max_age_s=self.settings["max_sample_age_s"])
        if sample is None:
            raise TimeoutError("attitude data is missing or stale during exploration")
        return float(sample[0][0])

    def _motion_pose(self):
        """Read the chassis frame used by move_to, rather than scan-matched pose."""
        getter = getattr(self.chassis, "get_pose", None)
        pose = getter() if callable(getter) else self.map.pose
        if pose is None or len(pose) < 2 or not all(math.isfinite(v) for v in pose[:2]):
            raise TimeoutError("position data is missing or stale during alignment")
        return pose

    def _sensor_offset(self, world_yaw):
        direction = math.radians(world_yaw)
        body_yaw = math.radians(self._current_yaw())
        sensor = self.settings["sensor"]
        return (sensor["pivot_x_m"] * math.cos(direction - body_yaw) +
                sensor["pivot_y_m"] * math.sin(direction - body_yaw) +
                sensor["offset_from_yaw_axis_m"] *
                math.cos(math.radians(sensor["offset_yaw_deg"])))

    def _gimbal_sample(self):
        sample = self.logger.get_sample("gimbal", max_age_s=self.settings["max_sample_age_s"])
        if sample is None or len(sample[0]) < 3:
            raise TimeoutError("gimbal angle data is missing or stale during exploration")
        return float(sample[0][1]), float(sample[0][2]), float(sample[1])

    @staticmethod
    def _command_yaw(target_relative_yaw, current_relative_yaw):
        """Choose the nearest equivalent chassis-relative SDK yaw target."""
        candidates = (target_relative_yaw - 360.0, target_relative_yaw,
                      target_relative_yaw + 360.0)
        reachable = [angle for angle in candidates if -250.0 <= angle <= 250.0]
        if not reachable:
            raise RuntimeError("gimbal cannot reach the requested direction within its yaw limits")
        return min(reachable, key=lambda angle: abs(angle - current_relative_yaw))

    def _scan_for_direction(self, delta):
        """Point the single ToF toward a candidate cell and wait for its new scan."""
        worker_status = self.slam_worker.status() if self.slam_worker is not None else None
        if worker_status is not None and worker_status["error"]:
            raise RuntimeError(worker_status["error"])
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
        # recenter() also controls chassis-relative pitch. On the actual robot its
        # yaw and ToF settled while the SDK kept that action running. An absolute
        # moveto(yaw=0) centers yaw without invoking the recenter action.
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
        waiting_for_action = False
        action_confirmed = action is None
        median_window = self.settings["tof_median_window"]
        samples = []
        last_sample_timestamp = request_time
        collection_after = request_time if action_confirmed else None
        while True:
            worker_status = self.slam_worker.status() if self.slam_worker is not None else None
            if worker_status is not None and worker_status["error"]:
                raise RuntimeError(worker_status["error"])
            action_state = getattr(action, "state", None)
            if action_state in ("action_failed", "action_rejected", "action_exception", "action_aborted"):
                raise RuntimeError(f"gimbal moveto failed: {action_state}")
            measured_yaw, measured_pitch, angle_timestamp = self._gimbal_sample()
            aligned = (angle_timestamp > request_time and
                       abs(_wrap_degrees(target_yaw - measured_yaw)) <= tolerance and
                       abs(self.settings["gimbal"]["pitch_deg"] - measured_pitch) <= tolerance)

            action_complete = action is None or getattr(action, "has_succeeded", False)
            if action_complete and not action_confirmed:
                # The SDK removes its dispatcher entry before signaling the
                # completion event. Only wait once it reports success, so the
                # SDK does not turn an in-progress action into an exception.
                wait_for_completed = getattr(action, "wait_for_completed", None)
                if callable(wait_for_completed) and not wait_for_completed():
                    raise RuntimeError("gimbal SDK reported success but did not release its action")
                action_confirmed = True
                if median_window > 1:
                    collection_after = time.time()
                else:
                    collection_after = request_time

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
            if scan_ready and not action_confirmed and not waiting_for_action:
                self._set_status("waiting_gimbal_action")
                waiting_for_action = True
            elif action_confirmed and waiting_for_action:
                self._set_status(previous_status)
                waiting_for_action = False
            if (action_confirmed and scan_ready and
                    scan_timestamp > collection_after and
                    not (worker_status or {}).get("tof_waiting", False)):
                if samples and time.time() - samples[0][0] > self.settings["max_sample_age_s"] * 2:
                    samples.clear()
                samples.append((scan_timestamp, float(scan_range)))
                last_sample_timestamp = scan_timestamp
                if len(samples) >= median_window:
                    self._set_status(previous_status)
                    return float(statistics.median(value for _, value in samples)), world_yaw
            time.sleep(0.03)

    def _can_step(self, node, destination):
        delta = (destination[0] - node[0], destination[1] - node[1])
        with self.lock:
            self.wall_grid.checks.pop((tuple(node), delta), None)
        worker_status = self.slam_worker.status() if self.slam_worker is not None else None
        if worker_status is not None and worker_status["error"]:
            raise RuntimeError(worker_status["error"])
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
        """Use the four settled scans from a new cell to center between walls."""
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
                raise RuntimeError(f"invalid wall distance for alignment at {node}")
            walls[delta] = distance

        corrections = []
        for positive, negative in (((1, 0), (-1, 0)), ((0, 1), (0, -1))):
            plus, minus = walls.get(positive), walls.get(negative)
            if plus is not None and minus is not None:
                width = plus + minus
                if width < 2 * desired:
                    raise RuntimeError(
                        f"walls at {node} are only {width:.3f} m apart; "
                        f"cannot stay {desired:.3f} m from both")
                correction = (plus - minus) / 2.0
            elif plus is not None:
                correction = plus - desired
            elif minus is not None:
                correction = desired - minus
            else:
                correction = 0.0
            corrections.append(correction)

        shift = math.hypot(*corrections)
        result = {"walls": {name: round(walls.get(delta), 4) for name, delta in
                            zip(CellWallGrid.SIDE_NAMES, self.DIRECTIONS)
                            if delta in walls},
                  "target_distance_m": desired, "correction_m":
                  [round(value, 4) for value in corrections]}
        if shift > max_shift:
            raise RuntimeError(f"alignment at {node} needs {shift:.3f} m, "
                               f"above configured limit {max_shift:.3f} m")
        if shift <= tolerance:
            result["status"] = "within_tolerance" if walls else "no_wall"
            with self.lock:
                self.alignments[node] = result
            self.map.set_exploration_state(self.snapshot())
            return

        x, y = self._motion_pose()[:2]
        angle = math.radians(self.base_pose[2])
        dx, dy = corrections
        target = (x + dx * math.cos(angle) - dy * math.sin(angle),
                  y + dx * math.sin(angle) + dy * math.cos(angle))
        if not self.map.contains_world(*target):
            raise RuntimeError(f"alignment target for {node} is outside the SLAM map")
        self._set_status("aligning")
        previous_scan = self.map.latest_scan_timestamp or 0.0
        self.chassis.move_to(*target, yaw=self.base_pose[2],
                             abort_event=self.slam_worker.abort_event,
                             disable_timeout=True)
        while (self.map.latest_scan_timestamp or 0.0) <= previous_scan:
            worker_status = self.slam_worker.status()
            if worker_status["error"]:
                raise RuntimeError(worker_status["error"])
            time.sleep(0.05)
        verified = {}
        for delta in self.DIRECTIONS:
            if delta not in walls:
                continue
            reading, yaw = self._scan_for_direction(delta)
            actual = reading / 1000.0 + self._sensor_offset(yaw)
            name = CellWallGrid.SIDE_NAMES[self.DIRECTIONS.index(delta)]
            verified[name] = round(actual, 4)
            expected = walls[delta] - dx * delta[0] - dy * delta[1]
            if actual < desired - tolerance or abs(actual - expected) > 2 * tolerance:
                raise RuntimeError(
                    f"alignment at {node} did not reach a safe wall distance: "
                    f"{name} {actual:.3f} m, expected {expected:.3f} m")
        with self.lock:
            self.cell_targets[node] = target
            result["status"] = "moved"
            result["after_m"] = verified
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
            raise RuntimeError(worker_status["error"])
        back = (parent[0] - child[0], parent[1] - child[1])
        with self.lock:
            return self.wall_grid.state(child, back) == "open"

    def _move(self, destination):
        with self.lock:
            target = self.cell_targets.get(tuple(destination))
            if target is None:
                current = self.current_cell
                origin = self.cell_targets.get(current, self._to_map(current))
                step = self.settings["step_m"]
                du = (destination[0] - current[0]) * step
                dv = (destination[1] - current[1]) * step
                angle = math.radians(self.base_pose[2])
                target = (origin[0] + du * math.cos(angle) - dv * math.sin(angle),
                          origin[1] + du * math.sin(angle) + dv * math.cos(angle))
                self.cell_targets[tuple(destination)] = target
        x, y = target
        previous_scan = self.map.latest_scan_timestamp or 0.0
        previous_status = self.status
        self.chassis.move_to(x, y, yaw=self.base_pose[2],
                             abort_event=self.slam_worker.abort_event,
                             disable_timeout=True)
        with self.lock:
            self.moves += 1
            self.current_cell = tuple(destination)
        self._set_status("waiting_slam_scan")
        while True:
            if (self.map.latest_scan_timestamp or 0.0) > previous_scan:
                self._set_status(previous_status)
                return
            worker_status = self.slam_worker.status()
            if worker_status["error"]:
                raise RuntimeError(worker_status["error"])
            time.sleep(0.05)

    def run(self, slam_worker):
        """Explore new cells with fresh scans, then return over known open edges."""
        self.slam_worker = slam_worker
        try:
            self._set_status("starting")
            slam_worker.wait_ready()
            pose = self.map.pose
            if pose is None:
                raise RuntimeError("SLAM has no initial pose")
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
                            raise RuntimeError("DFS cannot safely return to the start cell")
                        self._set_status("returning_to_start")
                        self._move(parent)
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
                    self._move(next_node)
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
                        raise RuntimeError("DFS backtrack path is no longer clear")
                    self._set_status(f"backtracking_to_{parent[0]}_{parent[1]}")
                    self._move(parent)
                    self._set_status("exploring")

            self._set_status("completed")
            return self.snapshot()
        except Exception as error:
            self._set_status("failed", str(error))
            raise
