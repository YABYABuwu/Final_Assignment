"""Round 2 waypoint execution with per-edge IR lane memory."""

import copy
import math

from src.ir_lane import prepare_lane, retarget_lane
from src.mission_stop import MissionStop


class Round2Navigator:
    def __init__(self, chassis, grid_map, settings, translation, on_change=None):
        self.chassis = chassis
        self.grid = grid_map
        self.settings = settings
        self.translation = tuple(translation)
        self.on_change = on_change
        self.lanes = {}
        self.last_lane = None
        if settings.get("enabled", False) and settings["max_offset_m"] >= grid_map.cell_size_m / 2:
            raise ValueError("IR lane max_offset_m must be below half the loaded map cell size")
        self._publish()

    def snapshot(self):
        return copy.deepcopy({"enabled": self.settings.get("enabled", False),
                              "ir_lanes": [value for _, value in sorted(self.lanes.items())],
                              "last_ir_lane": self.last_lane})

    def _publish(self):
        if self.on_change is not None:
            self.on_change(self.snapshot())

    def _cell_point(self, cell):
        point = self.grid.cell_to_world(cell)
        return tuple(point[i] + self.translation[i] for i in (0, 1))

    def move_to(self, source, destination, target):
        destination = tuple(destination)
        source = tuple(source) if source is not None else None
        if source is not None and source != destination:
            if not self.grid.is_passable(source, destination):
                self.chassis.stop()
                previous = self.lanes.pop(tuple(sorted((source, destination))), None)
                self.last_lane = {**(previous or {"cells": [list(source), list(destination)],
                                                 "offset_m": [0.0, 0.0]}),
                                  "status": "aborted", "target_m": list(target)}
                self._publish()
                raise MissionStop("Round 2 route edge is not open in the loaded map")
        if not self.settings.get("enabled", False) or source is None or source == destination:
            return self.chassis.move_to(*target, yaw=None)

        edge = tuple(sorted((source, destination)))
        saved = self.lanes.get(edge)
        targets = {source: self._cell_point(source), destination: tuple(target)}
        edge, lane, normal, index, lane_target = prepare_lane(
            source, destination, targets, self.grid.base_pose[2], saved)
        current_target = lane_target if saved else tuple(target)
        changed = False
        self.last_lane = {**lane, "status": "reused" if saved else "original",
                          "target_m": list(current_target)}
        self._publish()

        def contains_world(x, y):
            angle = math.radians(self.grid.base_pose[2])
            half = self.grid.cell_size_m / 2
            for node in edge:
                center = self._cell_point(node)
                dx, dy = x - center[0], y - center[1]
                along = dx * math.cos(angle) + dy * math.sin(angle)
                across = -dx * math.sin(angle) + dy * math.cos(angle)
                if abs(along) <= half and abs(across) <= half:
                    return True
            return False

        def recovered(before, after, old_target):
            nonlocal lane, current_target, changed
            correction = retarget_lane(lane, normal, index, before, after,
                                       self.settings, contains_world)
            if correction is None:
                return None
            lane, current_target = correction
            changed = True
            self.last_lane = {**lane, "status": "candidate", "target_m": list(current_target)}
            self._publish()
            return current_target

        try:
            if not contains_world(*current_target):
                raise MissionStop("Round 2 IR lane target is outside the route cells")
            pose = self.chassis.move_to(*current_target, yaw=None, on_ir_recovered=recovered)
            # Only completed traversal can teach a lane, never a partial stop.
            if (pose is None or not all(math.isfinite(value) for value in pose[:2]) or
                    math.hypot(pose[0] - current_target[0], pose[1] - current_target[1]) >
                    self.chassis.settings["position_tolerance_m"]):
                raise MissionStop("Round 2 stopped before reaching its IR lane target")
        except BaseException:
            self.chassis.stop()
            self.lanes.pop(edge, None)
            self.last_lane = {**self.last_lane, "status": "aborted"}
            self._publish()
            raise
        if changed or saved:
            self.lanes[edge] = lane
            self.last_lane = {**lane, "status": "confirmed", "target_m": list(current_target)}
            self._publish()
        return pose
