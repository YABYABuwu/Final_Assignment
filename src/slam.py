"""Odometry-backed 2D occupancy-grid SLAM from RoboMaster EP ToF scans."""

import json
import math
import struct
import threading
import time
import zlib
from pathlib import Path
from src.mission_stop import MissionStop


MAP_FORMAT = "robomaster-occupancy-grid"
MAP_VERSION = 1


def _encode_png(width, height, rows, color_type):
    """Encode already filtered 8-bit image rows without an image dependency."""
    def chunk(kind, payload):
        return (struct.pack(">I", len(payload)) + kind + payload +
                struct.pack(">I", zlib.crc32(kind + payload) & 0xffffffff))

    return (b"\x89PNG\r\n\x1a\n" +
            chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8,
                                        color_type, 0, 0, 0)) +
            chunk(b"IDAT", zlib.compress(bytes(rows))) +
            chunk(b"IEND", b""))


def _wrap_degrees(angle):
    return (angle + 180.0) % 360.0 - 180.0


def _line_cells(x0, y0, x1, y1):
    """Yield integer grid cells on an inclusive Bresenham line."""
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        doubled = 2 * error
        if doubled > -dy:
            error -= dy
            x0 += sx
        if doubled < dx:
            error += dx
            y0 += sy


class CellWallGrid:
    """Discrete occupancy on shared cell borders, fed only by settled scans.

    The current border uses a direct ToF wall threshold. Farther borders
    retain the ray's snapped wall position for map visualization.
    """

    DIRECTIONS = ((1, 0), (0, -1), (-1, 0), (0, 1))
    SIDE_NAMES = ("x+", "y-", "x-", "y+")
    DELTA_TO_SIDE = {(1, 0): "x+", (0, -1): "y-", (-1, 0): "x-", (0, 1): "y+"}

    def __init__(self, cell_size_m, max_ray_cells):
        self.cell_size_m = float(cell_size_m)
        self.max_ray_cells = max(1, int(max_ray_cells))
        self.edges = {}
        self.cells = set()
        self.checks = {}

    @staticmethod
    def edge_key(node, delta):
        neighbor = (node[0] + delta[0], node[1] + delta[1])
        return tuple(sorted((tuple(node), neighbor)))

    def state(self, node, delta):
        return self.edges.get(self.edge_key(node, delta), {}).get("state", "unknown")

    def observe(self, node, delta, hit_distance_m, range_mm, wall_threshold_mm, timestamp):
        if delta not in self.DIRECTIONS or not math.isfinite(hit_distance_m):
            raise ValueError("wall grid observation needs a cardinal, finite ray")
        self.cells.add(tuple(node))
        wall_index = max(0, int(math.floor(hit_distance_m / self.cell_size_m)))
        ray_cells = 1 if range_mm <= wall_threshold_mm else min(wall_index + 1, self.max_ray_cells)
        for index in range(ray_cells):
            current = (node[0] + index * delta[0], node[1] + index * delta[1])
            key = self.edge_key(current, delta)
            self.cells.update(key)
            if index == 0:
                self.edges[key] = {
                    "state": "wall" if range_mm <= wall_threshold_mm else "open",
                    "source": "direct",
                    "measured_from": list(node),
                    "range_mm": range_mm,
                    "wall_threshold_mm": wall_threshold_mm,
                    "observed_at": timestamp,
                }
            elif self.edges.get(key, {}).get("source") != "direct":
                self.edges[key] = {
                    "state": "wall" if index == wall_index else "open",
                    "source": "inferred",
                    "observed_at": timestamp,
                }
        self.checks[(tuple(node), delta)] = {
            "range_mm": range_mm,
            "wall_threshold_mm": wall_threshold_mm,
            "observed_at": timestamp,
        }

    def can_cross(self, node, delta):
        return (self.state(node, delta) == "open" and
                self.checks.get((tuple(node), delta), {}).get("range_mm", 0) >
                self.checks.get((tuple(node), delta), {}).get("wall_threshold_mm", float("inf")))

    def snapshot(self, base_pose, current):
        return {
            "version": 1, "cell_size_m": self.cell_size_m,
            "base_pose": list(base_pose) if base_pose is not None else None,
            "current": list(current) if current is not None else None,
            "cells": [{"index": list(node), "sides": {
                name: {**self.edges.get(self.edge_key(node, delta), {"state": "unknown"}),
                       "checked_from_here": (node, delta) in self.checks}
                for name, delta in zip(self.SIDE_NAMES, self.DIRECTIONS)
            }} for node in sorted(self.cells)],
        }

    @classmethod
    def validate_document(cls, grid):
        if grid is None:
            return
        if (not isinstance(grid, dict) or grid.get("version") != 1 or
                type(grid.get("cell_size_m")) not in (int, float) or
                not math.isfinite(grid["cell_size_m"]) or grid["cell_size_m"] <= 0 or
                not isinstance(grid.get("cells"), list) or len(grid["cells"]) > 500000):
            raise ValueError("invalid cell wall grid metadata")
        def pair(value):
            return (isinstance(value, list) and len(value) == 2 and
                    all(type(v) is int and abs(v) <= 1000000 for v in value))
        base = grid.get("base_pose")
        if base is not None and (not isinstance(base, list) or len(base) != 3 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in base)):
            raise ValueError("invalid cell wall grid base pose")
        if grid.get("current") is not None and not pair(grid["current"]):
            raise ValueError("invalid cell wall grid current cell")
        seen, edges = set(), {}
        for cell in grid["cells"]:
            if not isinstance(cell, dict) or not pair(cell.get("index")):
                raise ValueError("invalid cell wall grid cell")
            node = tuple(cell["index"])
            if node in seen or not isinstance(cell.get("sides"), dict):
                raise ValueError("duplicate or invalid cell wall grid cell")
            seen.add(node)
            for name, delta in zip(cls.SIDE_NAMES, cls.DIRECTIONS):
                side = cell["sides"].get(name)
                if not isinstance(side, dict) or side.get("state") not in ("unknown", "open", "wall"):
                    raise ValueError("invalid cell wall grid side")
                if ("source" in side and side["source"] not in ("direct", "inferred")):
                    raise ValueError("invalid cell wall grid source")
                if ("checked_from_here" in side and type(side["checked_from_here"]) is not bool):
                    raise ValueError("invalid cell wall grid check flag")
                if ("measured_from" in side and not pair(side["measured_from"])):
                    raise ValueError("invalid cell wall grid measurement origin")
                for field in ("range_mm", "wall_threshold_mm", "observed_at"):
                    if field in side and (type(side[field]) not in (int, float) or
                                          not math.isfinite(side[field]) or
                                          (field != "observed_at" and side[field] <= 0)):
                        raise ValueError(f"invalid cell wall grid {field}")
                key = cls.edge_key(node, delta)
                if key in edges and edges[key] != side["state"]:
                    raise ValueError("inconsistent shared cell wall")
                edges[key] = side["state"]


class OccupancyGridSLAM:
    """Build and locally scan-match a 2D occupancy grid.

    The RoboMaster SDK supplies the pose prior. A single ToF mounted on the
    gimbal supplies one ray at the current gimbal yaw. Its configured offset is
    measured from the gimbal yaw axis along the optical axis. A single ray is
    integrated into the map but is not enough for scan matching.
    """

    def __init__(self, settings):
        self.settings = settings
        map_settings = settings["map"]
        self.resolution = float(map_settings["resolution_m"])
        self.width = int(round(map_settings["width_m"] / self.resolution))
        self.height = int(round(map_settings["height_m"] / self.resolution))
        if self.width < 2 or self.height < 2 or self.width * self.height > 500_000:
            raise ValueError("SLAM grid must contain between 4 and 500000 cells")
        self.log_odds = [0.0] * (self.width * self.height)
        self.origin_x = -self.width * self.resolution / 2.0
        self.origin_y = -self.height * self.resolution / 2.0
        self.anchor_pose = None
        self.pose = None
        self.trajectory = []
        self.latest_ranges_mm = None
        self.latest_range_mm = None
        self.latest_gimbal_yaw_deg = None
        self.latest_scan_timestamp = None
        self.scan_count = 0
        self.has_map = False
        self.localization_score = None
        self.exploration_state = {"status": "disabled", "visited": [], "stack": []}
        self.last_error = None
        self.lock = threading.RLock()

    def _inside(self, ix, iy):
        return 0 <= ix < self.width and 0 <= iy < self.height

    def world_to_cell(self, x, y):
        if self.origin_x is None:
            return None
        return (math.floor((x - self.origin_x) / self.resolution),
                math.floor((y - self.origin_y) / self.resolution))

    def contains_world(self, x, y):
        cell = self.world_to_cell(x, y)
        return cell is not None and self._inside(*cell)

    def cell_to_world(self, ix, iy, center=True):
        shift = 0.5 if center else 0.0
        return (self.origin_x + (ix + shift) * self.resolution,
                self.origin_y + (iy + shift) * self.resolution)

    def _index(self, ix, iy):
        return iy * self.width + ix

    def _cell_odds(self, ix, iy):
        if not self._inside(ix, iy):
            return 0.0
        return self.log_odds[self._index(ix, iy)]

    def _add_odds(self, ix, iy, delta):
        if self._inside(ix, iy):
            index = self._index(ix, iy)
            self.log_odds[index] = max(-4.0, min(4.0, self.log_odds[index] + delta))

    def _range_value(self, reading_mm):
        """Accept one range or select the configured zero-based SDK channel."""
        if isinstance(reading_mm, (list, tuple)):
            channel = int(self.settings["sensor"]["tof_channel"])
            if len(reading_mm) <= channel:
                return None
            reading_mm = reading_mm[channel]
        try:
            measured = float(reading_mm) / 1000.0
        except (TypeError, ValueError):
            return None
        if not math.isfinite(measured) or measured <= 0 or measured == 65.535:
            return None
        return measured

    def _beam_geometry(self, pose, reading_mm, gimbal_yaw_deg=0.0):
        x, y, yaw = pose
        yaw_rad = math.radians(yaw)
        cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)
        sensor = self.settings["sensor"]
        measured = self._range_value(reading_mm)
        try:
            gimbal_yaw_deg = float(gimbal_yaw_deg)
        except (TypeError, ValueError):
            return []
        if measured is None or not math.isfinite(gimbal_yaw_deg):
            return []
        beam_yaw = yaw + gimbal_yaw_deg + float(sensor["yaw_offset_deg"])
        beam_rad = math.radians(beam_yaw)
        offset_rad = beam_rad + math.radians(float(sensor["offset_yaw_deg"]))
        pivot_x, pivot_y = float(sensor["pivot_x_m"]), float(sensor["pivot_y_m"])
        offset = float(sensor["offset_from_yaw_axis_m"])
        sx = (x + pivot_x * cos_yaw - pivot_y * sin_yaw +
              offset * math.cos(offset_rad))
        sy = (y + pivot_x * sin_yaw + pivot_y * cos_yaw +
              offset * math.sin(offset_rad))
        ex = sx + measured * math.cos(beam_rad)
        ey = sy + measured * math.sin(beam_rad)
        return [(sx, sy, ex, ey, True)]

    def scan_is_valid(self, reading_mm, gimbal_yaw_deg=0.0):
        return len(self._beam_geometry((0.0, 0.0, 0.0), reading_mm,
                                       gimbal_yaw_deg)) == 1

    def _endpoint_score(self, beams):
        """Score hit endpoints against nearby occupied cells in the old map."""
        scores = []
        radius = max(1, int(math.ceil(0.10 / self.resolution)))
        for sx, sy, ex, ey, hit in beams:
            if not hit:
                continue
            cell = self.world_to_cell(ex, ey)
            if cell is None:
                continue
            ix, iy = cell
            best = 0.0
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if math.hypot(dx, dy) > radius + 0.25:
                        continue
                    best = max(best, self._cell_odds(ix + dx, iy + dy))
            scores.append(best / 4.0)
        return sum(scores) / len(scores) if scores else 0.0

    def _scan_match(self, prior, reading_mm, gimbal_yaw_deg):
        beams = self._beam_geometry(prior, reading_mm, gimbal_yaw_deg)
        hit_count = sum(1 for beam in beams if beam[4])
        if hit_count < 2 or self.scan_count < 3:
            self.localization_score = None
            return prior

        baseline = self._endpoint_score(beams)
        best_pose, best_score = prior, baseline
        radius = float(self.settings["scan_match"]["translation_m"])
        translation_step = max(self.resolution, radius / 2.0)
        angle_radius = float(self.settings["scan_match"]["angle_deg"])
        angle_step = float(self.settings["scan_match"]["angle_step_deg"])
        count = int(math.floor(radius / translation_step))
        angle_count = int(math.floor(angle_radius / angle_step))
        for dx_step in range(-count, count + 1):
            dx = dx_step * translation_step
            for dy_step in range(-count, count + 1):
                dy = dy_step * translation_step
                for angle_step_index in range(-angle_count, angle_count + 1):
                    d_yaw = angle_step_index * angle_step
                    candidate = (prior[0] + dx, prior[1] + dy,
                                 prior[2] + d_yaw)
                    candidate_beams = self._beam_geometry(
                        candidate, reading_mm, gimbal_yaw_deg
                    )
                    score = self._endpoint_score(candidate_beams)
                    if score > best_score:
                        best_pose, best_score = candidate, score
        self.localization_score = round(best_score, 3)
        minimum_score = float(self.settings["scan_match"]["minimum_score"])
        improvement = float(self.settings["scan_match"]["minimum_improvement"])
        if best_score >= minimum_score and best_score >= baseline + improvement:
            return best_pose
        return prior

    def _integrate_beam(self, beam):
        sx, sy, ex, ey, hit = beam
        dx, dy = ex - sx, ey - sy
        length = math.hypot(dx, dy)
        skip = min(float(self.settings["map"]["robot_clearance_m"]), length)
        if length <= skip:
            return
        sx += dx * skip / length
        sy += dy * skip / length
        start = self.world_to_cell(sx, sy)
        if start is None or not self._inside(*start):
            return
        end = self.world_to_cell(ex, ey)
        if end is None:
            return
        if not self._inside(*end):
            # Limit work to the finite grid. A hit beyond its edge is not a wall.
            ray_x, ray_y = ex - sx, ey - sy
            x_max = self.origin_x + self.width * self.resolution - self.resolution * 1e-6
            y_max = self.origin_y + self.height * self.resolution - self.resolution * 1e-6
            travel = 1.0
            if ray_x > 0:
                travel = min(travel, (x_max - sx) / ray_x)
            elif ray_x < 0:
                travel = min(travel, (self.origin_x - sx) / ray_x)
            if ray_y > 0:
                travel = min(travel, (y_max - sy) / ray_y)
            elif ray_y < 0:
                travel = min(travel, (self.origin_y - sy) / ray_y)
            ex, ey = sx + ray_x * travel, sy + ray_y * travel
            end = self.world_to_cell(ex, ey)
            hit = False
            if end is None or not self._inside(*end):
                return
        cells = list(_line_cells(*start, *end))
        if not cells:
            return
        free_delta = float(self.settings["map"]["free_update"])
        occupied_delta = float(self.settings["map"]["occupied_update"])
        for ix, iy in cells[:-1] if hit else cells:
            self._add_odds(ix, iy, free_delta)
        if hit:
            self._add_odds(*cells[-1], occupied_delta)

    def update(self, odom_pose, reading_mm, timestamp=None, gimbal_yaw_deg=0.0):
        """Integrate one pose, selected ToF distance and relative gimbal yaw."""
        if len(odom_pose) < 3:
            raise ValueError("SLAM needs (x, y, yaw)")
        odom_pose = tuple(float(value) for value in odom_pose[:3])
        if not all(math.isfinite(value) for value in odom_pose):
            raise ValueError("pose contains a non-finite value")
        timestamp = time.time() if timestamp is None else float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError("scan timestamp must be finite")
        try:
            gimbal_yaw_deg = float(gimbal_yaw_deg)
        except (TypeError, ValueError) as error:
            raise ValueError("gimbal yaw must be finite") from error
        if not math.isfinite(gimbal_yaw_deg) or not self.scan_is_valid(
                reading_mm, gimbal_yaw_deg):
            raise ValueError("scan needs a valid ToF distance and gimbal yaw")
        with self.lock:
            if self.anchor_pose is None:
                self.anchor_pose = list(odom_pose)
            pose = self._scan_match(odom_pose, reading_mm, gimbal_yaw_deg)
            beams = self._beam_geometry(pose, reading_mm, gimbal_yaw_deg)
            for beam in beams:
                self._integrate_beam(beam)
            self.pose = list(pose)
            if isinstance(reading_mm, (list, tuple)):
                self.latest_ranges_mm = []
                for value in reading_mm:
                    try:
                        self.latest_ranges_mm.append(float(value))
                    except (TypeError, ValueError):
                        self.latest_ranges_mm.append(None)
                self.latest_range_mm = float(
                    reading_mm[int(self.settings["sensor"]["tof_channel"])]
                )
            else:
                self.latest_range_mm = float(reading_mm)
                self.latest_ranges_mm = [self.latest_range_mm]
            self.latest_gimbal_yaw_deg = gimbal_yaw_deg
            self.latest_scan_timestamp = timestamp
            self.scan_count += 1
            self.has_map = True
            if (not self.trajectory or
                    math.hypot(pose[0] - self.trajectory[-1][0],
                               pose[1] - self.trajectory[-1][1]) >= self.resolution * 2 or
                    abs(_wrap_degrees(pose[2] - self.trajectory[-1][2])) >= 2.0):
                self.trajectory.append([round(value, 4) for value in pose])
                if len(self.trajectory) > 6_000:
                    del self.trajectory[:len(self.trajectory) - 6_000]
            self.last_error = None
            return list(self.pose)

    def path_has_obstacle(self, start_xy, end_xy, clearance_m=0.0):
        """Return true when a mapped occupied cell blocks a line segment."""
        with self.lock:
            start = self.world_to_cell(*start_xy)
            end = self.world_to_cell(*end_xy)
            if (start is None or end is None or not self._inside(*start) or
                    not self._inside(*end)):
                return True
            cells = _line_cells(*start, *end)
            radius = int(math.ceil(clearance_m / self.resolution))
            for ix, iy in cells:
                for dy in range(-radius, radius + 1):
                    for dx in range(-radius, radius + 1):
                        if math.hypot(dx, dy) > radius + 0.25:
                            continue
                        if not self._inside(ix + dx, iy + dy):
                            return True
                        if self._cell_odds(ix + dx, iy + dy) >= 0.8:
                            return True
            return False

    def set_exploration_state(self, state):
        with self.lock:
            self.exploration_state = json.loads(json.dumps(state))

    def _occupancy_data(self):
        data = []
        known, free, occupied = 0, 0, 0
        for odds in self.log_odds:
            if odds >= 0.62:
                data.append(100)
                occupied += 1
                known += 1
            elif odds <= -0.62:
                data.append(0)
                free += 1
                known += 1
            else:
                data.append(-1)
        return data, {"known_cells": known, "free_cells": free,
                      "occupied_cells": occupied,
                      "unknown_cells": self.width * self.height - known}

    def to_dict(self):
        """Return a JSON-serializable map document; rows are y-major from bottom."""
        with self.lock:
            data, counts = self._occupancy_data()
            return {
                "format": MAP_FORMAT,
                "version": MAP_VERSION,
                "frame": "map",
                "axis_convention": "SDK position coordinates in metres; grid rows increase with +y",
                "resolution_m": self.resolution,
                "width": self.width,
                "height": self.height,
                "origin": [self.origin_x, self.origin_y, 0.0],
                "anchor_pose": list(self.anchor_pose) if self.anchor_pose is not None else None,
                "pose": list(self.pose) if self.pose is not None else None,
                "trajectory": [list(point) for point in self.trajectory],
                "data": data,
                "counts": counts,
                "scan_count": self.scan_count,
                "has_map": self.has_map,
                "sensor_model": {
                    "type": "single_gimbal_tof",
                    "tof_channel": int(self.settings["sensor"]["tof_channel"]),
                    "offset_from_yaw_axis_m": float(
                        self.settings["sensor"]["offset_from_yaw_axis_m"]
                    ),
                    "robot_clearance_m": float(self.settings["map"]["robot_clearance_m"]),
                    "pivot_x_m": float(self.settings["sensor"]["pivot_x_m"]),
                    "pivot_y_m": float(self.settings["sensor"]["pivot_y_m"]),
                    "yaw_offset_deg": float(self.settings["sensor"]["yaw_offset_deg"]),
                    "offset_yaw_deg": float(self.settings["sensor"]["offset_yaw_deg"]),
                    "latest_gimbal_yaw_deg": self.latest_gimbal_yaw_deg,
                },
                "localization_score": self.localization_score,
                "latest_scan_timestamp": self.latest_scan_timestamp,
                "exploration": json.loads(json.dumps(self.exploration_state)),
            }

    def load_dict(self, document):
        """Replace map state from a JSON export generated by this module."""
        if not isinstance(document, dict) or document.get("format") != MAP_FORMAT:
            raise ValueError("unsupported map file format")
        if type(document.get("version")) is not int or document["version"] != MAP_VERSION:
            raise ValueError("unsupported map file version")
        width, height = document.get("width"), document.get("height")
        resolution = document.get("resolution_m")
        origin = document.get("origin")
        data = document.get("data")
        if (type(width) is not int or type(height) is not int or width < 2 or height < 2 or
                width * height > 500_000 or not isinstance(resolution, (int, float)) or
                type(resolution) not in (int, float) or not math.isfinite(resolution) or resolution <= 0 or
                not isinstance(origin, list) or len(origin) < 2 or
                not all(type(v) in (int, float) and math.isfinite(v) for v in origin[:2]) or
                not isinstance(data, list) or len(data) != width * height):
            raise ValueError("map file has invalid grid metadata")
        if any(type(value) is not int or (value != -1 and not 0 <= value <= 100)
               for value in data):
            raise ValueError("map file occupancy values must be -1 or integers from 0 to 100")
        def checked_pose(value, name):
            if value is None:
                return None
            if (not isinstance(value, list) or len(value) != 3 or
                    any(type(item) not in (int, float) or not math.isfinite(item)
                        for item in value)):
                raise ValueError(f"map file {name} must be null or a finite x/y/yaw list")
            return [float(item) for item in value]

        anchor_pose = checked_pose(document.get("anchor_pose"), "anchor_pose")
        pose = checked_pose(document.get("pose"), "pose")
        trajectory = document.get("trajectory", [])
        if (not isinstance(trajectory, list) or len(trajectory) > 6000 or
                any(checked_pose(point, "trajectory point") is None for point in trajectory)):
            raise ValueError("map file trajectory must contain at most 6000 finite poses")
        exploration_state = document.get("exploration", {
            "status": "loaded", "visited": [], "stack": []})
        if not isinstance(exploration_state, dict):
            raise ValueError("map file exploration state must be an object")
        CellWallGrid.validate_document(exploration_state.get("cell_grid"))
        try:
            json.dumps(exploration_state, allow_nan=False)
        except (TypeError, ValueError) as error:
            raise ValueError("map file exploration state must contain only finite JSON values") from error
        scan_count = document.get("scan_count", 0)
        if type(scan_count) is not int or scan_count < 0:
            raise ValueError("map file scan_count must be a nonnegative integer")
        localization_score = document.get("localization_score")
        if (localization_score is not None and
                (type(localization_score) not in (int, float) or not math.isfinite(localization_score))):
            raise ValueError("map file localization_score must be finite or null")
        scan_timestamp = document.get("latest_scan_timestamp")
        if (scan_timestamp is not None and
                (type(scan_timestamp) not in (int, float) or not math.isfinite(scan_timestamp))):
            raise ValueError("map file latest_scan_timestamp must be finite or null")
        sensor_model = document.get("sensor_model", {})
        if not isinstance(sensor_model, dict):
            raise ValueError("map file sensor_model must be an object")
        latest_gimbal_yaw = sensor_model.get("latest_gimbal_yaw_deg")
        if (latest_gimbal_yaw is not None and
                (type(latest_gimbal_yaw) not in (int, float) or
                 not math.isfinite(latest_gimbal_yaw))):
            raise ValueError("map file latest gimbal yaw must be finite or null")
        with self.lock:
            self.width, self.height = width, height
            self.resolution = float(resolution)
            self.origin_x, self.origin_y = float(origin[0]), float(origin[1])
            self.log_odds = [0.0] * len(data)
            for index, value in enumerate(data):
                if value == -1:
                    continue
                probability = max(0.05, min(0.95, value / 100.0))
                self.log_odds[index] = math.log(probability / (1.0 - probability))
            self.anchor_pose = anchor_pose
            self.pose = pose
            self.trajectory = [[float(value) for value in point] for point in trajectory]
            self.scan_count = scan_count
            self.has_map = True
            self.localization_score = localization_score
            self.latest_scan_timestamp = scan_timestamp
            self.latest_range_mm = None
            self.latest_ranges_mm = None
            self.latest_gimbal_yaw_deg = latest_gimbal_yaw
            self.exploration_state = document.get("exploration", {
                "status": "loaded", "visited": [], "stack": []})

    def load_file(self, path):
        with Path(path).open(encoding="utf-8") as file:
            document = json.load(file)
        self.load_dict(document)

    def save_file(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        with temporary.open("w", encoding="utf-8") as file:
            json.dump(self.to_dict(), file, ensure_ascii=False, separators=(",", ":"))
        temporary.replace(path)

    def png_bytes(self, scale=4):
        """Render occupancy as PNG, with +X up and +Y right like the dashboard."""
        if type(scale) is not int or scale < 1 or scale > 16:
            raise ValueError("PNG scale must be an integer from 1 to 16")
        document = self.to_dict()
        data = document["data"]
        grid_width, grid_height = document["width"], document["height"]
        width, height = grid_height * scale, grid_width * scale
        rows = bytearray()
        for ix in range(grid_width - 1, -1, -1):
            line = bytes(205 if data[iy * grid_width + ix] == -1 else
                         0 if data[iy * grid_width + ix] == 100 else 255
                         for iy in range(grid_height))
            expanded = bytes(pixel for pixel in line for _ in range(scale))
            for _ in range(scale):
                rows.append(0)  # PNG filter type: none
                rows.extend(expanded)

        return _encode_png(width, height, rows, 0)

    def save_png(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_bytes(self.png_bytes())
        temporary.replace(path)

    def grid_png_bytes(self):
        """Render the DFS cell grid with wall, open and unknown borders."""
        exploration = self.to_dict().get("exploration", {})
        grid = exploration.get("cell_grid")
        if not isinstance(grid, dict) or not grid.get("cells"):
            raise ValueError("DFS cell grid is unavailable")
        cells = grid["cells"]
        xs = [cell["index"][0] for cell in cells]
        ys = [cell["index"][1] for cell in cells]
        min_x, max_x, min_y, max_y = min(xs), max(xs), min(ys), max(ys)
        span_x, span_y = max_x - min_x + 3, max_y - min_y + 3
        cell_px = min(64, 4096 // max(span_x, span_y))
        if cell_px < 2:
            raise ValueError("DFS cell grid is too large for PNG export")
        width, height = span_y * cell_px, span_x * cell_px
        pixels = bytearray(bytes((9, 20, 29)) * (width * height))

        def pixel(px, py, color):
            if 0 <= px < width and 0 <= py < height:
                offset = (py * width + px) * 3
                pixels[offset:offset + 3] = bytes(color)

        def line(x0, y0, x1, y1, color, thickness=1, dashed=False):
            steps = max(abs(x1 - x0), abs(y1 - y0))
            for step in range(steps + 1):
                if dashed and (step // 8) % 2:
                    continue
                px = round(x0 + (x1 - x0) * step / max(steps, 1))
                py = round(y0 + (y1 - y0) * step / max(steps, 1))
                for dy in range(-(thickness // 2), (thickness + 1) // 2):
                    for dx in range(-(thickness // 2), (thickness + 1) // 2):
                        pixel(px + dx, py + dy, color)

        def bounds(index):
            ix, iy = index
            left = (iy - min_y + 1) * cell_px
            top = (max_x - ix + 1) * cell_px
            return left, top, left + cell_px, top + cell_px

        visited = {tuple(node) for node in exploration.get("visited", [])}
        current = tuple(grid["current"]) if grid.get("current") is not None else None
        for cell in cells:
            index = tuple(cell["index"])
            left, top, right, bottom = bounds(index)
            fill = ((25, 68, 55) if index == current else
                    (23, 55, 70) if index in visited else (18, 42, 55))
            row = bytes(fill) * cell_px
            for py in range(top, bottom):
                offset = (py * width + left) * 3
                pixels[offset:offset + len(row)] = row

        stack = exploration.get("stack", [])
        for first, second in zip(stack, stack[1:]):
            left_a, top_a, _, _ = bounds(first)
            left_b, top_b, _, _ = bounds(second)
            half = cell_px // 2
            line(left_a + half, top_a + half, left_b + half, top_b + half,
                 (105, 217, 255), 3)

        for cell in cells:
            left, top, right, bottom = bounds(cell["index"])
            ends = {"x+": (left, top, right, top),
                    "x-": (left, bottom, right, bottom),
                    "y-": (left, top, left, bottom),
                    "y+": (right, top, right, bottom)}
            for side, points in ends.items():
                info = cell["sides"][side]
                state = info["state"]
                color = ((255, 184, 104) if state == "wall" else
                         (140, 241, 210) if state == "open" else (113, 133, 148))
                line(*points, color, 4 if state == "wall" else 2,
                     state == "unknown" or info.get("source") == "inferred")

        glyphs = {
            "0": ("111", "101", "101", "101", "111"),
            "1": ("010", "110", "010", "010", "111"),
            "2": ("111", "001", "111", "100", "111"),
            "3": ("111", "001", "111", "001", "111"),
            "4": ("101", "101", "111", "001", "001"),
            "5": ("111", "100", "111", "001", "111"),
            "6": ("111", "100", "111", "101", "111"),
            "7": ("111", "001", "010", "010", "010"),
            "8": ("111", "101", "111", "101", "111"),
            "9": ("111", "101", "111", "001", "111"),
            "-": ("000", "000", "111", "000", "000"),
            ",": ("000", "000", "000", "010", "100"),
        }
        if cell_px >= 24:
            text_scale = 2 if cell_px >= 48 else 1
            for cell in cells:
                left, top, _, _ = bounds(cell["index"])
                label = f"{cell['index'][0]},{cell['index'][1]}"
                text_width = (len(label) * 4 - 1) * text_scale
                if text_width > cell_px - 8:
                    continue
                start_x = left + (cell_px - text_width) // 2
                start_y = top + (cell_px - 5 * text_scale) // 2
                for char_index, char in enumerate(label):
                    for row_index, row in enumerate(glyphs[char]):
                        for column_index, bit in enumerate(row):
                            if bit == "1":
                                for sy in range(text_scale):
                                    for sx in range(text_scale):
                                        pixel(start_x + (char_index * 4 + column_index) *
                                              text_scale + sx,
                                              start_y + row_index * text_scale + sy,
                                              (169, 191, 203))

        rows = bytearray()
        stride = width * 3
        for py in range(height):
            rows.append(0)
            rows.extend(pixels[py * stride:(py + 1) * stride])
        return _encode_png(width, height, rows, 2)

    def save_grid_png(self, path):
        path = Path(path)
        try:
            content = self.grid_png_bytes()
        except ValueError as error:
            if str(error) != "DFS cell grid is unavailable":
                raise
            path.unlink(missing_ok=True)
            return False
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_bytes(content)
        temporary.replace(path)
        return True

    def ros_map_archive(self):
        """Build a ZIP containing ROS map_server YAML/PGM and the source JSON."""
        import io
        import zipfile

        document = self.to_dict()
        data = document["data"]
        width, height = document["width"], document["height"]
        pixels = bytearray()
        # PGM is top-to-bottom; OccupancyGrid JSON is bottom-to-top.
        for row in range(height - 1, -1, -1):
            for value in data[row * width:(row + 1) * width]:
                pixels.append(205 if value == -1 else 0 if value >= 65 else 254 if value <= 19 else 128)
        pgm = f"P5\n{width} {height}\n255\n".encode("ascii") + bytes(pixels)
        yaml_text = (
            "image: map.pgm\n"
            f"resolution: {document['resolution_m']}\n"
            f"origin: [{document['origin'][0]}, {document['origin'][1]}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n"
        )
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as archive:
            archive.writestr("map.yaml", yaml_text)
            archive.writestr("map.pgm", pgm)
            archive.writestr("slam.json", json.dumps(document, ensure_ascii=False))
        return buffer.getvalue()


class SlamWorker:
    """Sample synchronized logger telemetry and update the shared SLAM map."""

    def __init__(self, logger, slam_map, settings):
        self.logger = logger
        self.map = slam_map
        self.settings = settings
        self.stop_event = threading.Event()
        self.thread = None
        self.is_running = False
        self.ready = threading.Event()
        self.last_tof_timestamp = None
        self.error = None
        self.waiting_telemetry = False
        self.mapping_paused = False
        self.lock = threading.Lock()
        self.abort_event = threading.Event()
        self.started_at = None
        self.last_good_scan_monotonic = None

    def start(self):
        if self.is_running:
            raise MissionStop("SLAM worker is already running")
        self.stop_event.clear()
        self.abort_event.clear()
        self.ready.clear()
        self.started_at = time.monotonic()
        self.last_good_scan_monotonic = None
        self.last_tof_timestamp = None
        self.error = None
        self.waiting_telemetry = False
        self.mapping_paused = False
        self.is_running = True
        self.thread = threading.Thread(target=self._run, name="slam-map-updater", daemon=True)
        self.thread.start()

    def _run(self):
        period = 1.0 / self.settings["update_hz"]
        max_age = self.settings["max_sample_age_s"]
        while not self.stop_event.is_set():
            started = time.monotonic()
            try:
                position = self.logger.get_sample("position", max_age_s=max_age)
                attitude = self.logger.get_sample("attitude", max_age_s=max_age)
                tof = self.logger.get_sample("tof", max_age_s=max_age)
                gimbal = self.logger.get_sample("gimbal", max_age_s=max_age)
                status = self.logger.get_sample("status", max_age_s=max_age)
                if (position is not None and attitude is not None and tof is not None and
                        gimbal is not None and status is not None):
                    values, timestamp = tof
                    gimbal_values, gimbal_timestamp = gimbal
                    status_values = status[0]
                    if len(status_values) < 10:
                        raise ValueError("exploration needs the complete RoboMaster chassis status sample")
                    for index in (4, 5, 6, 7, 8, 9):
                        if index < len(status_values) and status_values[index] not in (0, False, None):
                            raise MissionStop(f"robot safety status flag {index} is active")
                    channel = int(self.settings["sensor"]["tof_channel"])
                    if len(values) <= channel:
                        raise ValueError("ToF callback did not include the configured channel")
                    if len(gimbal_values) < 2:
                        raise ValueError("gimbal callback needs pitch and relative yaw angles")
                    reading_mm = values[channel]
                    gimbal_yaw_deg = gimbal_values[1]
                    timestamps = (position[1], attitude[1], timestamp,
                                  gimbal_timestamp, status[1])
                    synchronized = max(timestamps) - min(timestamps) <= self.settings["sample_skew_s"]
                    with self.lock:
                        mapping_paused = self.mapping_paused
                    try:
                        pitch_aligned = (abs(float(gimbal_values[0]) -
                                             self.settings["gimbal"]["pitch_deg"]) <=
                                         self.settings["gimbal"]["pitch_tolerance_deg"])
                    except (TypeError, ValueError):
                        pitch_aligned = False
                    if not synchronized or mapping_paused or not pitch_aligned:
                        with self.lock:
                            self.waiting_telemetry = True
                    elif timestamp != self.last_tof_timestamp:
                        if self.map.scan_is_valid(reading_mm, gimbal_yaw_deg):
                            pose = (position[0][0], position[0][1], attitude[0][0])
                            self.map.update(pose, reading_mm, timestamp=timestamp,
                                            gimbal_yaw_deg=gimbal_yaw_deg)
                            self.last_tof_timestamp = timestamp
                            self.last_good_scan_monotonic = time.monotonic()
                            self.ready.set()
                            with self.lock:
                                self.error = None
                                self.waiting_telemetry = False
                        # A fresh callback can contain an invalid range (for
                        # example 0 mm). Skip it and wait for a usable scan;
                        # repeated invalid values are not a worker failure.
                    elif self.ready.is_set() and self._scan_stale():
                        with self.lock:
                            self.waiting_telemetry = True
                elif self.ready.is_set() and self._scan_stale():
                    with self.lock:
                        self.waiting_telemetry = True
            except Exception as error:
                self._fail(str(error))
            remaining = period - (time.monotonic() - started)
            self.stop_event.wait(max(0.01, remaining))
        self.is_running = False

    def _scan_stale(self):
        reference = self.last_good_scan_monotonic or self.started_at or time.monotonic()
        return time.monotonic() - reference > self.settings["max_sample_age_s"]

    def _fail(self, message):
        with self.lock:
            self.error = message
        with self.map.lock:
            self.map.last_error = message
        self.abort_event.set()
        self.stop_event.set()

    def pause_mapping(self):
        with self.lock:
            self.mapping_paused = True
            self.waiting_telemetry = True

    def resume_mapping(self):
        with self.lock:
            self.mapping_paused = False

    def wait_ready(self, timeout_s=None):
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        while not self.ready.is_set():
            with self.lock:
                error = self.error
            if error:
                raise MissionStop(error)
            if self.stop_event.is_set():
                raise MissionStop("SLAM worker stopped before the first valid scan")
            if deadline is not None and time.monotonic() >= deadline:
                raise TimeoutError("SLAM did not receive synchronized pose, gimbal, status and ToF data")
            self.ready.wait(0.1)

    def stop(self, save_path=None, run_dir=None):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join()
            self.thread = None
        self.is_running = False
        if save_path:
            self.map.save_file(save_path)
            self.map.save_png(Path(save_path).with_suffix(".png"))
            self.map.save_grid_png(Path(save_path).with_name(Path(save_path).stem + "-grid.png"))
        if run_dir:
            self.map.save_png(Path(run_dir) / "map.png")
            self.map.save_grid_png(Path(run_dir) / "map-grid.png")

    def status(self):
        with self.lock:
            error = self.error
            waiting_telemetry = self.waiting_telemetry
            mapping_paused = self.mapping_paused
        tof = self.logger.get_sample("tof", max_age_s=self.settings["max_sample_age_s"])
        channel = int(self.settings["sensor"]["tof_channel"])
        tof_waiting = (tof is None or len(tof[0]) <= channel or
                       self.map._range_value(tof[0][channel]) is None)
        return {"running": self.is_running, "ready": self.ready.is_set(),
                "error": error, "tof_waiting": tof_waiting,
                "waiting_telemetry": waiting_telemetry,
                "mapping_paused": mapping_paused}
