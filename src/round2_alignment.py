"""Use DFS alignment against mapped walls without running exploration or SLAM."""

import copy
import math
import threading

from src.explorer import DFSExplorer


class AlignmentTelemetry:
    """Read existing subscriptions; never start another sensor worker."""

    def __init__(self, logger, settings):
        self.logger, self.settings = logger, settings
        self.abort_event = threading.Event()

    def status(self):
        samples = [self.logger.get_sample(name, max_age_s=self.settings["max_sample_age_s"])
                   for name in ("position", "attitude", "status")]
        waiting = (any(sample is None for sample in samples) or
                   max((sample[1] for sample in samples if sample), default=0) -
                   min((sample[1] for sample in samples if sample), default=0) >
                   self.settings["sample_skew_s"])
        error = None
        if not waiting:
            flags = samples[2][0]
            waiting = len(flags) < 10
            if not waiting and any(flags[i] not in (0, False, None) for i in range(4, 10)):
                error = "robot safety flag active during Round 2 alignment"
                self.abort_event.set()
        return {"error": error, "waiting_telemetry": waiting, "tof_waiting": False}

    def set_heading_bias(self, bias):
        # DFS has already set ChassisController's bias; no SLAM worker exists here.
        pass


class AlignmentView:
    def __init__(self, owner):
        self.owner = owner
        self.latest_scan_timestamp = None
        self.latest_range_mm = None
        self.latest_gimbal_yaw_deg = None

    @property
    def pose(self):
        return self.owner.chassis.get_pose()

    def contains_world(self, x, y):
        center = self.owner.cell_point(self.owner.current_cell)
        a = math.radians(self.owner.base_pose[2])
        dx, dy = x - center[0], y - center[1]
        half = self.owner.grid.cell_size_m / 2
        return (abs(dx * math.cos(a) + dy * math.sin(a)) <= half and
                abs(-dx * math.sin(a) + dy * math.cos(a)) <= half)

    def set_exploration_state(self, state):
        self.owner.publish()


class Round2Aligner(DFSExplorer):
    def __init__(self, chassis, gimbal, logger, grid_map, settings, translation,
                 heading_deg, on_change=None):
        self.grid = grid_map
        self.translation = tuple(translation)
        self.on_change = on_change
        self.phase = None
        self.events = []
        self.mapped_sides = []
        settings = copy.deepcopy(settings)
        settings["step_m"] = grid_map.cell_size_m
        super().__init__(chassis, gimbal, logger, AlignmentView(self), settings)
        self.base_pose = (grid_map.base_pose[0] + translation[0],
                          grid_map.base_pose[1] + translation[1], grid_map.base_pose[2])
        self.travel_heading_deg = heading_deg
        self.slam_worker = AlignmentTelemetry(logger, settings)

    def cell_point(self, node):
        point = self.grid.cell_to_world(node)
        return tuple(point[i] + self.translation[i] for i in (0, 1))

    def publish(self):
        if self.on_change:
            self.on_change(copy.deepcopy({
                "status": self.status, "phase": self.phase,
                "cell": list(self.current_cell), "mapped_sides": self.mapped_sides,
                "heading": self.last_heading_alignment,
                "position": self.alignments.get(self.current_cell),
                "scan_alignment": self.scan_alignment, "error": self.error,
                "events": self.events}))

    def snapshot(self):
        # DFS publishes through its map interface; Round 2 has no occupancy map.
        return {"status": self.status, "error": self.error}

    def _latest_alignment_scan(self):
        age = self.settings["max_sample_age_s"]
        tof = self.logger.get_sample("tof", max_age_s=age)
        angles = self.logger.get_sample("gimbal", max_age_s=age)
        position = self.logger.get_sample("position", max_age_s=age)
        if tof is None or angles is None or position is None or len(angles[0]) < 2:
            return None, None, None
        stamps = [sample[1] for sample in (tof, angles, position)]
        if max(stamps) - min(stamps) > self.settings["sample_skew_s"]:
            return None, None, None
        channel = self.settings["sensor"]["tof_channel"]
        try:
            reading = float(tof[0][channel])
        except (IndexError, TypeError, ValueError):
            return None, None, None
        if not math.isfinite(reading) or reading <= 0 or reading == 65535:
            return None, None, None
        self.map.latest_scan_timestamp = tof[1]
        self.map.latest_range_mm = reading
        self.map.latest_gimbal_yaw_deg = angles[0][1]
        return tof[1], angles[0][1], reading

    def align(self, cell, phase, target_offset_world=(0.0, 0.0), last_ir_lane=None):
        self.current_cell = tuple(cell)
        self.phase = phase
        self.cell_targets[self.current_cell] = self.cell_point(self.current_cell)
        self.last_ir_lane = last_ir_lane
        self.last_heading_alignment = None
        self.alignments.pop(self.current_cell, None)
        directions = [delta for delta in self.DIRECTIONS
                      if self.grid.cells.get(self.current_cell, {}).get(
                          self.grid.DELTA_TO_SIDE[delta]) == "wall"]
        self.mapped_sides = [self.grid.DELTA_TO_SIDE[delta] for delta in directions]
        self.chassis.stop()
        try:
            self._set_status("aligning_mapped_walls")
            self.wall_grid.checks.clear()
            self._prepare_gimbal(force=True)
            if not directions:
                self._set_status("skipped_no_mapped_wall")
                return self._motion_pose()
            if not (self.settings["alignment"]["enabled"] or
                    self.settings["heading_alignment"]["enabled"]):
                self._set_status("disabled")
                return self._motion_pose()
            # Topology chooses directions. Only fresh ranges can select or move.
            for delta in directions:
                reading, yaw = self._scan_for_direction(delta)
                if reading <= self.settings["wall_threshold_mm"]:
                    self.wall_grid.observe(self.current_cell, delta,
                                           reading / 1000 + self._sensor_offset(yaw), reading,
                                           self.settings["wall_threshold_mm"],
                                           self.map.latest_scan_timestamp)
            if self.settings["heading_alignment"]["enabled"]:
                result = self._align_heading(self.current_cell)
                if result["status"] == "applied":
                    pose = self._motion_pose()
                    self._drive_holding_current_yaw(*pose[:2], kind="heading_alignment")
            if self.settings["alignment"]["enabled"]:
                a = math.radians(self.base_pose[2])
                dx, dy = target_offset_world
                offset = (dx * math.cos(a) + dy * math.sin(a),
                          -dx * math.sin(a) + dy * math.cos(a))
                self._align_cell(self.current_cell, target_offset=offset)
            self._prepare_gimbal(force=True)
            self._set_status("aligned")
            return self._motion_pose()
        except BaseException as error:
            self._set_status("stopped", str(error))
            raise
        finally:
            self.chassis.stop()
            self.events.append({"cell": list(self.current_cell), "phase": phase,
                                "status": self.status, "mapped_sides": list(self.mapped_sides),
                                "heading": self.last_heading_alignment,
                                "position": self.alignments.get(self.current_cell),
                                "error": self.error})
            self.publish()
