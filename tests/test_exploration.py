import io
import copy
import csv
import json
import math
import struct
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import zipfile
import zlib
from pathlib import Path
from types import SimpleNamespace

from src.config_loader import load_config
from src.dashboard import Dashboard
from src.explorer import DFSExplorer
from src.gimbal_control import ChassisRelativeGimbal
from src.logger import SensorLogger
from src.mission_stop import MissionStop
from src.slam import CellWallGrid, OccupancyGridSLAM, SlamWorker


class FakeLogger:
    def __init__(self):
        self.samples = {}

    def set(self, name, values, timestamp=None):
        self.samples[name] = (tuple(values), time.time() if timestamp is None else timestamp)

    def get_sample(self, name, max_age_s=None):
        return self.samples.get(name)


class FakeSDKModule:
    def __init__(self):
        self.callbacks = {}

    def __getattr__(self, name):
        if name.startswith("sub_"):
            return lambda **options: self._subscribe(name, options)
        if name.startswith("unsub_"):
            return lambda: self.callbacks.pop(name.replace("unsub_", "sub_"), None)
        raise AttributeError(name)

    def _subscribe(self, name, options):
        self.callbacks[name] = options["callback"]
        return True


class FakeSlamWorker:
    def __init__(self):
        self.abort_event = threading.Event()

    def wait_ready(self, timeout_s=5.0):
        return None

    def status(self):
        return {"running": True, "ready": True, "error": None}


class SimulatedGimbal:
    def __init__(self, slam_map, logger, range_provider):
        self.slam_map = slam_map
        self.logger = logger
        self.range_provider = range_provider
        self.yaw = 0.0
        self.pitch = 0.0
        self.scan_time = time.time()
        self.commands = []
        self.recenter_calls = 0
        self.logger.set("gimbal", (0, 0, 0, 0), self.scan_time)

    def _range(self, pose):
        return (self.range_provider(pose, self.yaw)
                if callable(self.range_provider) else self.range_provider)

    def moveto(self, pitch, yaw, pitch_speed, yaw_speed):
        self.pitch, self.yaw = float(pitch), float(yaw)
        self.commands.append((self.pitch, self.yaw))
        self.scan_time = max(time.time(), self.scan_time) + 0.002
        self.logger.set("gimbal", (self.pitch, self.yaw, self.pitch, self.yaw),
                        self.scan_time)
        pose = tuple(self.slam_map.pose)
        reading = self._range(pose)
        self.logger.set("tof", reading if isinstance(reading, (list, tuple)) else (reading,),
                        self.scan_time)
        self.slam_map.update(pose, reading, timestamp=self.scan_time,
                             gimbal_yaw_deg=self.yaw)

    def recenter(self, pitch_speed, yaw_speed):
        self.recenter_calls += 1
        return self.moveto(pitch=0, yaw=0, pitch_speed=pitch_speed, yaw_speed=yaw_speed)


class SimulatedChassis:
    def __init__(self, slam_map, logger, gimbal, ranges):
        self.slam_map = slam_map
        self.logger = logger
        self.gimbal = gimbal
        self.ranges = ranges
        self.scan_time = time.time()
        self.commands = []

    def move_to(self, x, y, yaw=None, abort_event=None, disable_timeout=False,
                stop_if=None, pause_if=None):
        if abort_event is not None and abort_event.is_set():
            raise MissionStop("simulated motion aborted")
        if stop_if is not None and stop_if(tuple(self.slam_map.pose)):
            return tuple(self.slam_map.pose)
        pose = (x, y, yaw or 0.0)
        self.commands.append(pose)
        self.scan_time = max(time.time(), self.scan_time,
                             self.slam_map.latest_scan_timestamp or 0) + 0.01
        self.logger.set("attitude", (pose[2], 0, 0), self.scan_time)
        readings = (self.ranges(pose, self.gimbal.yaw)
                    if callable(self.ranges) else self.ranges)
        self.logger.set("tof", readings if isinstance(readings, (list, tuple)) else (readings,),
                        self.scan_time)
        self.slam_map.update(pose, readings, timestamp=self.scan_time,
                             gimbal_yaw_deg=self.gimbal.yaw)
        return pose


class ExplorationTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.settings = copy.deepcopy(self.config["exploration"])
        # Keep the simulated geometry stable when the hardware settings change.
        self.settings["wall_threshold_mm"] = 300
        self.settings["tof_median_window"] = 1
        self.settings["alignment"].update({"enabled": True, "wall_distance_m": .25,
                                           "tolerance_m": .03, "max_shift_m": .20})
        self.settings["target_inspection"]["enabled"] = False
        self.ranges = [2000, 2000, 2000, 2000]

    def make_map(self):
        slam_map = OccupancyGridSLAM(self.settings)
        timestamp = time.time()
        slam_map.update((0, 0, 0), self.ranges, timestamp=timestamp)
        slam_map.update((0, 0, 0), self.ranges, timestamp=timestamp + 0.01)
        return slam_map

    def room_range(self, pose, gimbal_yaw, half_extent=1.6):
        x, y, yaw = pose
        yaw_rad = math.radians(yaw)
        sensor = self.settings["sensor"]
        beam = math.radians(yaw + gimbal_yaw + sensor["yaw_offset_deg"])
        sx = (x + sensor["pivot_x_m"] * math.cos(yaw_rad) -
              sensor["pivot_y_m"] * math.sin(yaw_rad) +
              sensor["offset_from_yaw_axis_m"] * math.cos(beam))
        sy = (y + sensor["pivot_x_m"] * math.sin(yaw_rad) +
              sensor["pivot_y_m"] * math.cos(yaw_rad) +
              sensor["offset_from_yaw_axis_m"] * math.sin(beam))
        dx, dy = math.cos(beam), math.sin(beam)
        intersections = []
        if dx > 1e-9:
            intersections.append((half_extent - sx) / dx)
        elif dx < -1e-9:
            intersections.append((-half_extent - sx) / dx)
        if dy > 1e-9:
            intersections.append((half_extent - sy) / dy)
        elif dy < -1e-9:
            intersections.append((-half_extent - sy) / dy)
        return round(min(distance for distance in intersections if distance > 0) * 1000)

    def make_dashboard(self, slam_map):
        logger = SimpleNamespace(stream_settings={}, dropped_rows=0)
        robot = SimpleNamespace(camera=SimpleNamespace())
        return Dashboard(robot, logger, self.config["dashboard"], slam_map=slam_map)

    def request(self, dashboard, method, path, body=b""):
        handler_type = dashboard._handler_class()
        handler = handler_type.__new__(handler_type)
        handler.path = path
        handler.wfile = io.BytesIO()
        handler.rfile = io.BytesIO(body)
        handler.headers = {"Content-Length": str(len(body))}
        handler.send_response = lambda code: None
        handler.send_header = lambda name, value: None
        handler.end_headers = lambda: None
        handler.send_error = lambda code, message=None: self.fail(
            f"dashboard returned HTTP {code}: {message}"
        )
        getattr(handler, method)()
        return handler.wfile.getvalue()

    def test_dashboard_map_json_round_trip_and_ros_export(self):
        slam_map = self.make_map()
        dashboard = self.make_dashboard(slam_map)

        api_map = json.loads(self.request(dashboard, "do_GET", "/api/map"))
        self.assertEqual(api_map["format"], "robomaster-occupancy-grid")
        self.assertTrue(api_map["has_map"])
        self.assertEqual(api_map["scan_count"], 2)
        self.assertGreater(api_map["counts"]["free_cells"], 0)
        self.assertGreater(api_map["counts"]["occupied_cells"], 0)
        self.assertTrue(api_map["trajectory"])
        self.assertEqual(api_map["sensor_model"]["type"], "single_gimbal_tof")
        self.assertAlmostEqual(api_map["sensor_model"]["offset_from_yaw_axis_m"], 0.075)
        self.assertAlmostEqual(api_map["sensor_model"]["robot_clearance_m"], 0.20)
        self.assertEqual(len(api_map["data"]), api_map["width"] * api_map["height"])

        exported = json.loads(self.request(
            dashboard, "do_GET", "/api/map/export?format=json"
        ))
        png = self.request(dashboard, "do_GET", "/api/map/export?format=png")
        self.assertTrue(png.startswith(b"\x89PNG\r\n\x1a\n"))
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "map.json"
            path.write_text(json.dumps(exported), encoding="utf-8")
            restored = OccupancyGridSLAM(self.settings)
            restored.load_file(path)
            self.assertEqual(restored.to_dict()["data"], exported["data"])
            self.assertEqual(restored.to_dict()["pose"], exported["pose"])
            self.assertEqual(restored.to_dict()["trajectory"], exported["trajectory"])
            self.assertEqual(restored.to_dict()["sensor_model"], exported["sensor_model"])

        self.request(dashboard, "do_POST", "/api/map/import", json.dumps(exported).encode())
        self.assertEqual(dashboard.map_snapshot()["data"], exported["data"])

        archive_bytes = self.request(dashboard, "do_GET", "/api/map/export?format=ros")
        with zipfile.ZipFile(io.BytesIO(archive_bytes)) as archive:
            self.assertEqual(set(archive.namelist()), {"map.yaml", "map.pgm", "slam.json"})
            self.assertIn(b"resolution:", archive.read("map.yaml"))
            self.assertTrue(archive.read("map.pgm").startswith(b"P5\n"))
            self.assertEqual(json.loads(archive.read("slam.json"))["data"], exported["data"])

    def test_stop_saves_png_with_dashboard_axis_orientation(self):
        settings = copy.deepcopy(self.settings)
        settings["map"]["width_m"] = .1
        settings["map"]["height_m"] = .1
        slam_map = OccupancyGridSLAM(settings)
        slam_map.log_odds = [1, -1, 0, 0]  # y=0: occupied at x=0, free at x=1
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "latest.json"
            SlamWorker(FakeLogger(), slam_map, settings).stop(path, Path(temp) / "run")
            png = path.with_suffix(".png").read_bytes()
            self.assertTrue(path.exists())
            self.assertEqual((Path(temp) / "run" / "map.png").read_bytes(), png)
            self.assertTrue(png.startswith(b"\x89PNG\r\n\x1a\n"))
            self.assertEqual(struct.unpack(">II", png[16:24]), (8, 8))
            offset, image_data = 8, bytearray()
            while offset < len(png):
                length = struct.unpack(">I", png[offset:offset + 4])[0]
                kind = png[offset + 4:offset + 8]
                if kind == b"IDAT":
                    image_data.extend(png[offset + 8:offset + 8 + length])
                offset += 12 + length
            rows = zlib.decompress(image_data)
            self.assertEqual(rows[1:9], bytes([255] * 4 + [205] * 4))
            self.assertEqual(rows[-8:], bytes([0] * 4 + [205] * 4))
            self.assertFalse(path.with_name("latest-grid.png").exists())

    def test_grid_png_marks_wall_open_unknown_and_current_cell(self):
        slam_map = self.make_map()
        wall_grid = CellWallGrid(.6, 10)
        now = time.time()
        wall_grid.observe((0, 0), (1, 0), .175, 100, 300, now)
        wall_grid.observe((0, 0), (0, 1), .675, 600, 300, now)
        slam_map.set_exploration_state({
            "status": "exploring", "visited": [[0, 0]], "stack": [[0, 0]],
            "cell_grid": wall_grid.snapshot((0, 0, 0), (0, 0)),
        })
        png = slam_map.grid_png_bytes()
        self.assertTrue(png.startswith(b"\x89PNG\r\n\x1a\n"))
        width, height = struct.unpack(">II", png[16:24])
        payload, offset = bytearray(), 8
        while offset < len(png):
            length = struct.unpack(">I", png[offset:offset + 4])[0]
            if png[offset + 4:offset + 8] == b"IDAT":
                payload.extend(png[offset + 8:offset + 8 + length])
            offset += length + 12
        raw = zlib.decompress(payload)

        def color(x, y):
            start = y * (1 + width * 3) + 1 + x * 3
            return tuple(raw[start:start + 3])

        indices = [cell["index"] for cell in wall_grid.snapshot((0, 0, 0), (0, 0))["cells"]]
        min_y = min(index[1] for index in indices)
        max_x = max(index[0] for index in indices)
        left, top = (0 - min_y + 1) * 64, (max_x - 0 + 1) * 64
        self.assertEqual(color(left + 32, top), (255, 184, 104))
        self.assertEqual(color(left + 64, top + 32), (140, 241, 210))
        self.assertEqual(color(left + 32, top + 64), (113, 133, 148))
        self.assertEqual(color(left + 8, top + 8), (25, 68, 55))

        dashboard = self.make_dashboard(slam_map)
        self.assertTrue(self.request(dashboard, "do_GET", "/api/map/export?format=grid-png")
                        .startswith(b"\x89PNG\r\n\x1a\n"))
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "latest.json"
            SlamWorker(FakeLogger(), slam_map, self.settings).stop(path, Path(temp) / "run")
            self.assertEqual(path.with_name("latest-grid.png").read_bytes(), png)
            self.assertEqual((Path(temp) / "run" / "map-grid.png").read_bytes(), png)

    def test_gimbal_tof_beam_uses_yaw_axis_offset_and_selected_channel(self):
        settings = copy.deepcopy(self.settings)
        zero_based_map = OccupancyGridSLAM(settings)
        self.assertEqual(zero_based_map._range_value([111, 222, 333, 444]), 0.111)
        settings["sensor"]["tof_channel"] = 1
        slam_map = OccupancyGridSLAM(settings)
        beam, = slam_map._beam_geometry((1.0, 2.0, 90.0),
                                        [111, 222, 333, 444], -90.0)
        sx, sy, ex, ey, hit = beam
        self.assertAlmostEqual(sx, 1.075)
        self.assertAlmostEqual(sy, 2.0)
        self.assertAlmostEqual(ex, 1.297)
        self.assertAlmostEqual(ey, 2.0)
        self.assertTrue(hit)
        self.assertTrue(slam_map.scan_is_valid([111, 222, 333, 444], -90))

        settings["sensor"]["tof_channel"] = 3
        self.assertEqual(OccupancyGridSLAM(settings)._range_value([111, 222, 333, 444]), 0.444)

        settings["sensor"]["offset_yaw_deg"] = 90.0
        lateral_map = OccupancyGridSLAM(settings)
        lateral_beam, = lateral_map._beam_geometry((0, 0, 0), 222, 0)
        self.assertAlmostEqual(lateral_beam[0], 0.0)
        self.assertAlmostEqual(lateral_beam[1], 0.075)
        self.assertAlmostEqual(lateral_beam[2], 0.222)
        self.assertAlmostEqual(lateral_beam[3], 0.075)

    def test_short_and_distant_tof_scans_do_not_create_false_walls(self):
        slam_map = OccupancyGridSLAM(self.settings)
        self.assertTrue(slam_map.scan_is_valid(83))
        slam_map.update((0, 0, 0), 83)
        self.assertEqual(slam_map.latest_range_mm, 83)
        self.assertEqual(slam_map.to_dict()["counts"]["occupied_cells"], 0)

        self.assertTrue(slam_map.scan_is_valid(20000))
        slam_map.update((0, 0, 0), 20000)
        slam_map.update((0, 0, 0), 20000)
        counts = slam_map.to_dict()["counts"]
        self.assertGreater(counts["free_cells"], 0)
        self.assertEqual(counts["occupied_cells"], 0)
        self.assertFalse(slam_map.scan_is_valid(0))
        self.assertFalse(slam_map.scan_is_valid(float("inf")))

    def test_dfs_visits_nodes_and_returns_to_start(self):
        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, self.ranges)
        chassis = SimulatedChassis(slam_map, logger, gimbal, self.ranges)
        worker = FakeSlamWorker()
        settings = dict(self.settings)
        settings["max_nodes"] = 4
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, settings)

        result = explorer.run(worker)

        self.assertEqual(result["status"], "node_limit_returned")
        self.assertEqual(len(result["visited"]), 4)
        self.assertEqual(result["stack"], [[0, 0]])
        self.assertEqual(result["moves"], 6)
        self.assertAlmostEqual(slam_map.pose[0], 0.0)
        self.assertAlmostEqual(slam_map.pose[1], 0.0)
        self.assertEqual(slam_map.exploration_state["status"], "node_limit_returned")

    def test_dfs_scans_new_cell_before_forward_move_but_not_return(self):
        class CheckingChassis(SimulatedChassis):
            def __init__(self, slam_map, logger, gimbal, ranges):
                super().__init__(slam_map, logger, gimbal, ranges)
                self.previous_scan_count = 0

            def move_to(self, x, y, yaw=None, abort_event=None,
                        disable_timeout=False, stop_if=None, pause_if=None):
                new_scans = self.gimbal.commands[self.previous_scan_count:]
                if not self.commands:
                    self_test.assertEqual([round(command[1]) for command in new_scans[1:5]],
                                          [0, -90, -180, 90])
                    self_test.assertEqual(len(new_scans), 6)  # Recenter + four sides + one travel check.
                else:
                    self_test.assertEqual(len(new_scans), 1)
                self.previous_scan_count = len(self.gimbal.commands)
                return super().move_to(x, y, yaw=yaw, abort_event=abort_event,
                                       disable_timeout=disable_timeout,
                                       stop_if=stop_if, pause_if=pause_if)

        self_test = self
        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, self.ranges)
        chassis = CheckingChassis(slam_map, logger, gimbal, self.ranges)
        settings = dict(self.settings)
        settings["max_nodes"] = 2
        result = DFSExplorer(chassis, gimbal, logger, slam_map, settings).run(FakeSlamWorker())

        self.assertEqual(result["status"], "node_limit_returned")
        self.assertEqual(len(chassis.commands), 2)

    def test_fresh_travel_scan_rejects_edge_that_closed_after_four_side_scan(self):
        settings = copy.deepcopy(self.settings)
        settings["alignment"]["enabled"] = False
        settings["max_nodes"] = 2
        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        front_scans = [0]

        def ranges(pose, yaw):
            if abs(((yaw + 180) % 360) - 180) < 10 and abs(pose[0]) < .01:
                front_scans[0] += 1
                return 2000 if front_scans[0] == 1 else 100
            return 2000

        gimbal = SimulatedGimbal(slam_map, logger, ranges)
        chassis = SimulatedChassis(slam_map, logger, gimbal, ranges)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, settings)

        result = explorer.run(FakeSlamWorker())

        self.assertEqual(result["status"], "node_limit_returned")
        self.assertGreaterEqual(front_scans[0], 2)
        self.assertEqual(chassis.commands[0][:2], (0.0, -.6))
        self.assertEqual(explorer.wall_grid.state((0, 0), (1, 0)), "wall")

    def test_grid_motion_captures_fresh_yaw_for_each_step_and_return(self):
        self.settings["heading_source"] = "gimbal"
        class HeadingChassis(SimulatedChassis):
            heading = 12.0

            def __init__(self, *args):
                super().__init__(*args)
                self.commanded_yaws = []

            def get_pose(self):
                return (self.slam_map.pose[0], self.slam_map.pose[1], self.heading)

            def move_to(self, x, y, yaw=None, **kwargs):
                self.commanded_yaws.append(yaw)
                stop_if = kwargs.get("stop_if")
                if stop_if is not None:
                    kwargs["stop_if"] = lambda pose: stop_if((pose[0], pose[1], self.heading))
                return super().move_to(x, y, yaw=yaw, **kwargs)

        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, self.ranges)
        chassis = HeadingChassis(slam_map, logger, gimbal, self.ranges)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, self.settings)
        explorer.base_pose = (0, 0, 0)
        explorer.cell_targets = {(0, 0): (0, 0), (1, 0): (.6, 0)}
        explorer.slam_worker = FakeSlamWorker()

        explorer._move((1, 0))
        chassis.heading = -7.0
        explorer._move((0, 0))

        self.assertEqual(chassis.commanded_yaws, [12.0, -7.0])
        self.assertEqual(explorer.snapshot()["last_motion_heading"], {
            "yaw_deg": -7.0, "target_m": [0, 0], "kind": "grid_step",
            "source": "gimbal"})

    def test_dfs_does_not_move_when_all_sensor_ranges_are_blocked(self):
        blocked_ranges = [200, 200, 200, 200]
        slam_map = OccupancyGridSLAM(self.settings)
        slam_map.update((0, 0, 0), blocked_ranges)
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, blocked_ranges)
        chassis = SimulatedChassis(slam_map, logger, gimbal, blocked_ranges)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, self.settings)

        result = explorer.run(FakeSlamWorker())

        self.assertEqual(result["status"], "no_safe_direction")
        self.assertEqual(result["visited"], [[0, 0]])
        self.assertEqual(result["moves"], 0)
        self.assertEqual(chassis.commands, [])
        current = next(cell for cell in result["cell_grid"]["cells"] if cell["index"] == [0, 0])
        self.assertTrue(all(side["state"] == "wall" for side in current["sides"].values()))

    def test_wall_target_inspection_checks_each_wall_face_once(self):
        settings = copy.deepcopy(self.settings)
        settings["target_inspection"]["enabled"] = True
        checked = []

        def inspect(node, delta, world_yaw, body_yaw):
            checked.append((node, delta))
            return {"cell": list(node), "direction": list(delta),
                    "status": "no_target", "targets": []}

        explorer = DFSExplorer(None, None, FakeLogger(), self.make_map(), settings,
                               target_inspector=SimpleNamespace(inspect=inspect))
        explorer.base_pose = (0, 0, 0)
        explorer.wall_grid.observe((0, 0), (1, 0), .2, 125, 300, time.time())
        def fresh_wall(node, neighbor):
            delta = (neighbor[0] - node[0], neighbor[1] - node[1])
            explorer.wall_grid.observe(node, delta, .2, 125, 300, time.time())
            return False

        with patch.object(explorer, "_current_yaw", return_value=0), \
                patch.object(explorer, "_can_step", side_effect=fresh_wall):
            explorer._inspect_walls((0, 0))
            explorer._inspect_walls((0, 0))
            explorer._inspect_walls((1, 0))
        self.assertEqual(checked, [((0, 0), (1, 0)), ((1, 0), (-1, 0))])
        self.assertEqual(explorer.snapshot()["wall_inspections"][0]["status"], "no_target")

    def test_wall_target_inspection_skips_wall_that_reopened(self):
        settings = copy.deepcopy(self.settings)
        settings["target_inspection"]["enabled"] = True
        calls = []
        explorer = DFSExplorer(None, None, FakeLogger(), self.make_map(), settings,
                               target_inspector=SimpleNamespace(
                                   inspect=lambda *args: calls.append(args)))
        explorer.base_pose = (0, 0, 0)
        explorer.wall_grid.observe((0, 0), (1, 0), .2, 125, 300, time.time())

        def reopen(node, neighbor):
            explorer.wall_grid.observe(node, (1, 0), 2, 1925, 300, time.time())
            return True

        with patch.object(explorer, "_can_step", side_effect=reopen):
            explorer._inspect_walls((0, 0))
        self.assertEqual(calls, [])
        self.assertEqual(explorer.snapshot()["wall_inspections"][0]["status"],
                         "wall_no_longer_present")

    def test_alignment_uses_center_distance_and_keeps_corrected_return_target(self):
        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        # A wall 12.5 cm ahead of the ToF is 20 cm from the chassis center.
        def wall_range(pose, yaw):
            if abs(((yaw + 180) % 360) - 180) > 45:
                return 2000
            return round((.2 - pose[0] - .075) * 1000)
        gimbal = SimulatedGimbal(slam_map, logger, wall_range)
        chassis = SimulatedChassis(slam_map, logger, gimbal, wall_range)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, self.settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()
        explorer.cell_targets[(0, 0)] = (0, 0)
        explorer.wall_grid.observe((0, 0), (1, 0), .2, 125, 300, time.time())

        explorer._align_cell((0, 0))

        self.assertAlmostEqual(chassis.commands[0][0], -.05)
        self.assertAlmostEqual(explorer.cell_targets[(0, 0)][0], -.05)
        self.assertEqual(explorer.alignments[(0, 0)]["walls"], {"x+": .2})
        self.assertEqual(explorer.alignments[(0, 0)]["status"], "moved")
        self.assertAlmostEqual(explorer.alignments[(0, 0)]["after_m"]["x+"], .25)
        explorer._move((0, 1))
        self.assertAlmostEqual(chassis.commands[-1][0], -.05)
        self.assertAlmostEqual(chassis.commands[-1][1], .6)
        explorer._move((0, 0))
        self.assertAlmostEqual(chassis.commands[-1][0], -.05)

    def test_alignment_moves_away_from_close_wall_without_emergency_success(self):
        class AlignmentChassis(SimulatedChassis):
            def move_to(self, x, y, stop_if=None, **kwargs):
                if stop_if is not None:
                    raise AssertionError("alignment must not use the movement emergency stop")
                return super().move_to(x, y, **kwargs)

        settings = copy.deepcopy(self.settings)
        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))

        def wall_range(pose, yaw):
            return round((.175 - pose[0] - .075) * 1000)

        gimbal = SimulatedGimbal(slam_map, logger, wall_range)
        chassis = AlignmentChassis(slam_map, logger, gimbal, wall_range)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()
        explorer.wall_grid.observe((0, 0), (1, 0), .175, 100, 300, time.time())

        explorer._align_cell((0, 0))
        self.assertEqual(explorer.alignments[(0, 0)]["status"], "moved")
        self.assertAlmostEqual(explorer.cell_targets[(0, 0)][0], -.075)
        self.assertEqual(len(chassis.commands), 1)

    def test_alignment_rescans_between_bounded_moves(self):
        settings = copy.deepcopy(self.settings)
        settings["wall_threshold_mm"] = 500
        settings["alignment"]["max_shift_m"] = .10
        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))

        def wall_range(pose, yaw):
            return round((.407 + pose[1] - .075) * 1000)

        gimbal = SimulatedGimbal(slam_map, logger, wall_range)
        chassis = SimulatedChassis(slam_map, logger, gimbal, wall_range)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()
        explorer.cell_targets[(0, 0)] = (0, 0)
        explorer.wall_grid.observe((0, 0), (0, -1), .407, 332, 500, time.time())

        explorer._align_cell((0, 0))

        self.assertEqual(len(chassis.commands), 2)
        self.assertAlmostEqual(chassis.commands[0][1], -.10)
        self.assertAlmostEqual(chassis.commands[1][1], -.157)
        self.assertEqual(explorer.alignments[(0, 0)]["steps"], 2)
        self.assertAlmostEqual(explorer.alignments[(0, 0)]["after_m"]["y-"], .25)
        self.assertEqual(len(gimbal.commands), 3)  # One initial scan, then one after each move.

    def test_alignment_refreshes_scan_after_waiting_for_pose(self):
        settings = copy.deepcopy(self.settings)
        settings["max_sample_age_s"] = .02
        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))

        def wall_range(pose, yaw):
            return round((.30 - pose[0] - .075) * 1000)

        gimbal = SimulatedGimbal(slam_map, logger, wall_range)
        chassis = SimulatedChassis(slam_map, logger, gimbal, wall_range)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()
        explorer.wall_grid.observe((0, 0), (1, 0), .30, 225, 300, time.time())
        read_pose = explorer._motion_pose

        def delayed_pose():
            time.sleep(.08)
            return read_pose()

        explorer._motion_pose = delayed_pose
        explorer._align_cell((0, 0))

        self.assertEqual(len(chassis.commands), 1)
        self.assertGreaterEqual(len(gimbal.commands), 3)  # Refresh after the delayed pose.

    def test_alignment_stops_if_wall_distance_does_not_improve(self):
        settings = copy.deepcopy(self.settings)
        settings["wall_threshold_mm"] = 500
        settings["alignment"]["max_shift_m"] = .10
        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, 332)
        chassis = SimulatedChassis(slam_map, logger, gimbal, 332)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()
        explorer.wall_grid.observe((0, 0), (0, -1), .407, 332, 500, time.time())

        explorer._align_cell((0, 0))
        self.assertEqual(len(chassis.commands), 1)
        self.assertEqual(explorer.alignments[(0, 0)]["status"], "stalled")
        self.assertIn("remaining correction", explorer.alignments[(0, 0)]["reason"])

    def test_alignment_reverses_after_overshoot_instead_of_failing(self):
        settings = copy.deepcopy(self.settings)
        settings["wall_threshold_mm"] = 500
        settings["alignment"]["max_shift_m"] = .10
        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))

        # The wall range changes more than odometry predicts, as in the run
        # where 10 cm of commanded motion reduced the measured gap by 17 cm.
        def wall_range(pose, yaw):
            return round((.379 + 1.68 * pose[0] - .075) * 1000)

        gimbal = SimulatedGimbal(slam_map, logger, wall_range)
        chassis = SimulatedChassis(slam_map, logger, gimbal, wall_range)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()
        explorer.wall_grid.observe((0, 0), (-1, 0), .379, 304, 500, time.time())

        explorer._align_cell((0, 0))

        self.assertEqual(len(chassis.commands), 2)
        self.assertAlmostEqual(chassis.commands[0][0], -.10)
        self.assertGreater(chassis.commands[1][0], chassis.commands[0][0])
        self.assertEqual(explorer.alignments[(0, 0)]["status"], "moved")
        self.assertLessEqual(abs(explorer.alignments[(0, 0)]["after_m"]["x-"] - .25),
                             settings["alignment"]["tolerance_m"])

    def test_disabled_alignment_keeps_wall_scan_and_skips_alignment(self):
        settings = copy.deepcopy(self.settings)
        settings["alignment"]["enabled"] = False
        blocked_ranges = [300, 300, 300, 300]
        slam_map = OccupancyGridSLAM(settings)
        slam_map.update((0, 0, 0), blocked_ranges)
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, blocked_ranges)
        chassis = SimulatedChassis(slam_map, logger, gimbal, blocked_ranges)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, settings)

        with patch.object(explorer, "_align_cell", side_effect=AssertionError("alignment called")):
            result = explorer.run(FakeSlamWorker())

        self.assertEqual(result["status"], "no_safe_direction")
        self.assertEqual(len(gimbal.commands), 5)  # Recenter + four directions.
        self.assertEqual(gimbal.recenter_calls, 1)
        self.assertFalse(result["alignment_enabled"])
        self.assertEqual(result["alignments"], {})
        self.assertEqual(chassis.commands, [])

    def test_alignment_centers_between_two_opposing_walls(self):
        settings = copy.deepcopy(self.settings)
        settings["wall_threshold_mm"] = 500
        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        def wall_range(pose, yaw):
            if abs(((yaw + 180) % 360) - 180) < 90:
                return round((.30 - pose[0] - .075) * 1000)
            return round((pose[0] + .40 - .075) * 1000)
        gimbal = SimulatedGimbal(slam_map, logger, wall_range)
        chassis = SimulatedChassis(slam_map, logger, gimbal, wall_range)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()
        explorer.cell_targets[(0, 0)] = (0, 0)
        explorer.wall_grid.observe((0, 0), (1, 0), .30, 225, 500, time.time())
        explorer.wall_grid.observe((0, 0), (-1, 0), .40, 325, 500, time.time())

        explorer._align_cell((0, 0))

        self.assertAlmostEqual(chassis.commands[0][0], -.05)
        self.assertAlmostEqual(chassis.commands[0][1], 0)
        self.assertEqual(explorer.alignments[(0, 0)]["walls"],
                         {"x+": .30, "x-": .40})
        self.assertEqual(explorer.alignments[(0, 0)]["selected_sides"], ["x+", "x-"])
        self.assertEqual(explorer.alignments[(0, 0)]["axis_modes"], {"x": "between_walls"})
        self.assertAlmostEqual(explorer.alignments[(0, 0)]["after_m"]["x+"], .35)
        self.assertAlmostEqual(explorer.alignments[(0, 0)]["after_m"]["x-"], .35)

    def test_alignment_centers_between_y_walls_in_bounded_steps(self):
        settings = copy.deepcopy(self.settings)
        settings["wall_threshold_mm"] = 500
        settings["alignment"]["max_shift_m"] = .05
        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))

        def wall_range(pose, yaw):
            if 0 < ((yaw + 180) % 360) - 180:
                return round((.45 - pose[1] - .075) * 1000)
            return round((pose[1] + .25 - .075) * 1000)

        gimbal = SimulatedGimbal(slam_map, logger, wall_range)
        chassis = SimulatedChassis(slam_map, logger, gimbal, wall_range)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()
        explorer.wall_grid.observe((0, 0), (0, 1), .45, 375, 500, time.time())
        explorer.wall_grid.observe((0, 0), (0, -1), .25, 175, 500, time.time())

        explorer._align_cell((0, 0))

        self.assertEqual(len(chassis.commands), 2)
        self.assertAlmostEqual(chassis.commands[0][1], .05)
        self.assertAlmostEqual(chassis.commands[1][1], .10)
        self.assertAlmostEqual(explorer.alignments[(0, 0)]["after_m"]["y+"], .35)
        self.assertAlmostEqual(explorer.alignments[(0, 0)]["after_m"]["y-"], .35)

    def test_grid_emergency_stop_after_midpoint_keeps_planned_center_when_unconfirmed(self):
        class EmergencyChassis(SimulatedChassis):
            def move_to(self, x, y, yaw=None, stop_if=None, **kwargs):
                self.commands.append((x, y, yaw))
                timestamp = max(time.time(), self.slam_map.latest_scan_timestamp or 0) + .01
                pose = (.45, 0, 0)
                self.slam_map.update(pose, 110,
                                     timestamp=timestamp,
                                     gimbal_yaw_deg=self.gimbal.yaw)
                self.logger.set("tof", (110,), timestamp)
                self.logger.set("gimbal", (0, self.gimbal.yaw, 0, self.gimbal.yaw), timestamp)
                if stop_if is None or not stop_if(pose):
                    raise AssertionError("grid movement did not stop on the close wall")
                return pose

        settings = copy.deepcopy(self.settings)
        settings["alignment"]["enabled"] = False
        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, 2000)
        chassis = EmergencyChassis(slam_map, logger, gimbal, 2000)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()
        explorer.cell_targets[(0, 0)] = (0, 0)

        self.assertTrue(explorer._move((1, 0)))

        result = explorer.last_motion_stop
        self.assertEqual(result["status"], "emergency_stop_off_center")
        self.assertEqual(result["range_mm"], 110)
        self.assertAlmostEqual(result["center_distance_m"], .185)
        self.assertEqual(result["center_cell"], [1, 0])
        self.assertAlmostEqual(explorer.cell_targets[(1, 0)][0], .60)
        self.assertEqual(result["actual_pose"][:2], [.45, 0])
        self.assertAlmostEqual(result["planned_center_error_m"], .15)
        self.assertFalse(result["center_confirmed"])
        self.assertEqual(explorer.current_cell, (1, 0))
        explorer._move((2, 0))
        self.assertEqual(len(chassis.commands), 2)

    def test_single_wall_distance_cannot_confirm_cell_center(self):
        explorer = DFSExplorer(None, None, FakeLogger(), self.make_map(), self.settings)
        explorer.base_pose = (0, 0, 0)
        explorer.cell_targets[(1, 0)] = (.6, 0)

        rejected = explorer._assess_stopped_center((1, 0), (.45, 0, 0), .25)
        self.assertFalse(rejected["center_confirmed"])
        self.assertAlmostEqual(rejected["wall_target_error_m"], 0)
        self.assertEqual(explorer.cell_targets[(1, 0)], (.6, 0))
        accepted = explorer._assess_stopped_center((1, 0), (.59, 0, 0), .25)
        self.assertTrue(accepted["center_confirmed"])
        self.assertEqual(accepted["center_confirmation"], "planned_pose")
        self.assertEqual(explorer.cell_targets[(1, 0)], (.6, 0))

    def test_dfs_continues_after_off_center_emergency_stop(self):
        settings = copy.deepcopy(self.settings)
        settings["alignment"]["enabled"] = False
        settings["max_nodes"] = 2
        explorer = DFSExplorer(None, None, FakeLogger(), self.make_map(), settings)
        attempts = []

        def move(destination):
            attempts.append(destination)
            explorer.current_cell = destination
            explorer.last_motion_stop = {"status": "emergency_stop_off_center",
                                         "center_confirmed": False,
                                         "planned_center_error_m": .15}
            return True

        with patch.object(explorer, "_prepare_gimbal"), \
                patch.object(explorer, "_motion_pose", return_value=(0, 0, 0)), \
                patch.object(explorer, "_scan_all_directions", return_value={
                    delta: delta == (1, 0) for delta in explorer.DIRECTIONS}), \
                patch.object(explorer.wall_grid, "can_cross",
                             side_effect=lambda node, delta: delta == (1, 0)), \
                patch.object(explorer, "_can_return", return_value=True), \
                patch.object(explorer, "_move", side_effect=move):
            result = explorer.run(FakeSlamWorker())

        self.assertEqual(result["status"], "node_limit_returned")
        self.assertEqual(attempts, [(1, 0), (0, 0)])
        self.assertEqual(explorer.status, "node_limit_returned")
        self.assertEqual(explorer.stack, [(0, 0)])

    def test_grid_motion_accepts_measured_pitch_offset_within_calibrated_tolerance(self):
        class OffsetPitchChassis(SimulatedChassis):
            def move_to(self, x, y, yaw=None, stop_if=None, **kwargs):
                pose = (x, y, yaw or 0.0)
                self.commands.append(pose)
                timestamp = max(time.time(), self.slam_map.latest_scan_timestamp or 0) + .01
                self.slam_map.update(pose, 2000, timestamp=timestamp,
                                     gimbal_yaw_deg=self.gimbal.yaw)
                self.logger.set("tof", (2000,), timestamp)
                self.logger.set("gimbal", (7.1, self.gimbal.yaw, 0, self.gimbal.yaw),
                                timestamp)
                if stop_if is not None and stop_if(pose):
                    raise AssertionError("safe pitch offset incorrectly stopped grid motion")
                return pose

        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, 2000)
        chassis = OffsetPitchChassis(slam_map, logger, gimbal, 2000)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, self.settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()
        explorer.cell_targets[(0, 0)] = (0, 0)

        self.assertTrue(explorer._move((1, 0)))
        self.assertEqual(explorer.current_cell, (1, 0))

    def test_grid_emergency_stop_before_midpoint_keeps_source_cell(self):
        class EmergencyChassis(SimulatedChassis):
            def move_to(self, x, y, yaw=None, stop_if=None, **kwargs):
                timestamp = max(time.time(), self.slam_map.latest_scan_timestamp or 0) + .01
                pose = (.10, 0, 0)
                self.slam_map.update(pose, 125, timestamp=timestamp,
                                     gimbal_yaw_deg=self.gimbal.yaw)
                self.logger.set("tof", (125,), timestamp)
                self.logger.set("gimbal", (0, self.gimbal.yaw, 0, self.gimbal.yaw), timestamp)
                if stop_if is None or not stop_if(pose):
                    raise AssertionError("grid movement did not stop on the close wall")
                return pose

        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, 2000)
        chassis = EmergencyChassis(slam_map, logger, gimbal, 2000)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, self.settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()
        explorer.cell_targets[(0, 0)] = (0, 0)

        self.assertFalse(explorer._move((1, 0)))
        self.assertEqual(explorer.current_cell, (0, 0))
        self.assertAlmostEqual(explorer.cell_targets[(0, 0)][0], 0)
        self.assertAlmostEqual(explorer.last_motion_stop["planned_center_error_m"], .10)
        self.assertFalse(explorer.last_motion_stop["center_confirmed"])
        self.assertEqual(explorer.wall_grid.state((0, 0), (1, 0)), "wall")

    def test_grid_emergency_stop_on_return_uses_return_direction(self):
        class ReturnChassis(SimulatedChassis):
            def move_to(self, x, y, yaw=None, stop_if=None, **kwargs):
                if not self.commands:
                    return super().move_to(x, y, yaw=yaw, stop_if=stop_if, **kwargs)
                timestamp = max(time.time(), self.slam_map.latest_scan_timestamp or 0) + .01
                pose = (.15, 0, 0)
                self.slam_map.update(pose, 110, timestamp=timestamp,
                                     gimbal_yaw_deg=self.gimbal.yaw)
                self.logger.set("tof", (110,), timestamp)
                self.logger.set("gimbal", (0, self.gimbal.yaw, 0, self.gimbal.yaw), timestamp)
                if stop_if is None or not stop_if(pose):
                    raise AssertionError("return movement did not stop on the close wall")
                return pose

        slam_map = self.make_map()
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, 2000)
        chassis = ReturnChassis(slam_map, logger, gimbal, 2000)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, self.settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()
        explorer.cell_targets[(0, 0)] = (0, 0)

        self.assertTrue(explorer._move((1, 0)))
        self.assertTrue(explorer._move((0, 0)))
        self.assertAlmostEqual(explorer.cell_targets[(0, 0)][0], 0)
        self.assertFalse(explorer.last_motion_stop["center_confirmed"])
        self.assertEqual(explorer.last_motion_stop["center_cell"], [0, 0])
        self.assertAlmostEqual(abs(gimbal.yaw), 180)

    def test_gimbal_chooses_nearest_relative_yaw_at_half_turn(self):
        slam_map = OccupancyGridSLAM(self.settings)
        slam_map.update((0, 0, 0), 2000, gimbal_yaw_deg=90)
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, 2000)
        gimbal.yaw = 90
        logger.set("gimbal", (0, 90, 0, 90))
        explorer = DFSExplorer(None, gimbal, logger, slam_map, self.settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()

        measured_mm, world_yaw = explorer._scan_for_direction((-1, 0))

        self.assertEqual(measured_mm, 2000)
        self.assertEqual(world_yaw, 180)
        self.assertAlmostEqual(gimbal.commands[-1][1], 180)
        self.assertAlmostEqual(explorer._command_yaw(-30, 0), -30)
        self.assertAlmostEqual(explorer._command_yaw(0, -180), 0)

    def test_scan_median_uses_three_new_scans_in_target_direction(self):
        settings = copy.deepcopy(self.settings)
        settings["tof_median_window"] = 3
        slam_map = OccupancyGridSLAM(settings)
        slam_map.update((0, 0, 0), 2000)
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, 200)
        explorer = DFSExplorer(None, gimbal, logger, slam_map, settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()

        def later_scans():
            for yaw, distance in ((90, 100), (0, 2000), (0, 2100)):
                time.sleep(.08)
                slam_map.update((0, 0, 0), distance,
                                timestamp=time.time(), gimbal_yaw_deg=yaw)

        producer = threading.Thread(target=later_scans)
        producer.start()
        try:
            measured_mm, _ = explorer._scan_for_direction((1, 0))
        finally:
            producer.join(timeout=2)
        self.assertEqual(measured_mm, 2000)
        self.assertFalse(producer.is_alive())

    def test_tof_65535_is_not_a_valid_scan(self):
        slam_map = OccupancyGridSLAM(self.settings)
        self.assertFalse(slam_map.scan_is_valid(65535, 0))

    def test_gimbal_front_scan_ignores_startup_frame_yaw(self):
        slam_map = OccupancyGridSLAM(self.settings)
        slam_map.update((0, 0, 0), 2000, gimbal_yaw_deg=-180)
        logger = FakeLogger()
        logger.set("attitude", (179, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, 2000)
        gimbal.yaw = -180
        logger.set("gimbal", (0, -180, 0, 0))
        explorer = DFSExplorer(None, gimbal, logger, slam_map, self.settings)
        explorer.base_pose = (0, 0, 179)
        explorer.slam_worker = FakeSlamWorker()

        measured_mm, world_yaw = explorer._scan_for_direction((1, 0))

        self.assertEqual(measured_mm, 2000)
        self.assertEqual(world_yaw, 179)
        self.assertAlmostEqual(gimbal.commands[-1][1], 0)
        self.assertEqual(gimbal.recenter_calls, 0)

    def test_gimbal_front_scan_preserves_nonzero_configured_pitch(self):
        settings = copy.deepcopy(self.settings)
        settings["gimbal"]["pitch_deg"] = 5
        slam_map = OccupancyGridSLAM(settings)
        slam_map.update((0, 0, 0), 2000)
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, 2000)
        explorer = DFSExplorer(None, gimbal, logger, slam_map, settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()

        explorer._scan_for_direction((1, 0))

        self.assertEqual(gimbal.recenter_calls, 0)
        self.assertAlmostEqual(gimbal.commands[-1][0], 5)

    def test_scan_checks_pitch_relative_to_chassis(self):
        class ChassisLevelGimbal(SimulatedGimbal):
            def moveto(self, pitch, yaw, pitch_speed, yaw_speed):
                super().moveto(pitch, yaw, pitch_speed, yaw_speed)
                # Ground pitch can drift with chassis tilt while the gimbal
                # remains at the requested chassis-relative pitch.
                self.logger.set("gimbal", (0, self.yaw, -5, self.yaw),
                                self.scan_time + .001)

        slam_map = OccupancyGridSLAM(self.settings)
        slam_map.update((0, 0, 0), 2000)
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = ChassisLevelGimbal(slam_map, logger, 2000)
        explorer = DFSExplorer(None, gimbal, logger, slam_map, self.settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()

        self.assertEqual(explorer._scan_for_direction((1, 0))[0], 2000)

    def test_scan_accepts_observed_pitch_offset_without_relaxing_yaw(self):
        class OffsetPitchGimbal(SimulatedGimbal):
            def moveto(self, pitch, yaw, pitch_speed, yaw_speed):
                action = super().moveto(pitch, yaw, pitch_speed, yaw_speed)
                self.logger.set("gimbal", (7.1, self.yaw, 0, self.yaw),
                                self.scan_time + .001)
                return action

        slam_map = OccupancyGridSLAM(self.settings)
        slam_map.update((0, 0, 0), 2000)
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = OffsetPitchGimbal(slam_map, logger, 2000)
        explorer = DFSExplorer(None, gimbal, logger, slam_map, self.settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()

        self.assertEqual(explorer._scan_for_direction((1, 0))[0], 2000)
        self.assertEqual(self.settings["gimbal"]["angle_tolerance_deg"], 3)
        self.assertEqual(self.settings["gimbal"]["pitch_tolerance_deg"], 8)

    def test_completed_gimbal_action_retries_once_when_yaw_is_stuck(self):
        class FreshGimbalLogger(FakeLogger):
            def get_sample(self, name, max_age_s=None):
                sample = super().get_sample(name, max_age_s)
                return ((sample[0], time.time()) if name == "gimbal" and sample else sample)

        class CompletedAction:
            has_succeeded = True
            state = "action_succeeded"

            def wait_for_completed(self):
                return True

        class StalledOnceGimbal(SimulatedGimbal):
            def moveto(self, pitch, yaw, pitch_speed, yaw_speed):
                super().moveto(pitch, yaw, pitch_speed, yaw_speed)
                if len(self.commands) == 1:
                    self.yaw = -28.6
                    self.logger.set("gimbal", (pitch, self.yaw, pitch, self.yaw))
                return CompletedAction()

        slam_map = OccupancyGridSLAM(self.settings)
        slam_map.update((0, 0, 0), 2000)
        logger = FreshGimbalLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = StalledOnceGimbal(slam_map, logger, 2000)
        explorer = DFSExplorer(None, gimbal, logger, slam_map, self.settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()

        self.assertEqual(explorer._scan_for_direction((0, -1))[0], 2000)
        self.assertEqual(len(gimbal.commands), 2)

    def test_completed_gimbal_action_stops_after_failed_retry(self):
        class FreshGimbalLogger(FakeLogger):
            def get_sample(self, name, max_age_s=None):
                sample = super().get_sample(name, max_age_s)
                return ((sample[0], time.time()) if name == "gimbal" and sample else sample)

        class CompletedAction:
            has_succeeded = True
            state = "action_succeeded"

            def wait_for_completed(self):
                return True

        class StuckGimbal(SimulatedGimbal):
            def moveto(self, pitch, yaw, pitch_speed, yaw_speed):
                super().moveto(pitch, yaw, pitch_speed, yaw_speed)
                self.yaw = -28.6
                self.logger.set("gimbal", (pitch, self.yaw, pitch, self.yaw))
                return CompletedAction()

        slam_map = OccupancyGridSLAM(self.settings)
        slam_map.update((0, 0, 0), 2000)
        logger = FreshGimbalLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = StuckGimbal(slam_map, logger, 2000)
        explorer = DFSExplorer(None, gimbal, logger, slam_map, self.settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()

        with self.assertRaisesRegex(MissionStop, "gimbal alignment failed after retry"):
            explorer._scan_for_direction((0, -1))
        self.assertEqual(len(gimbal.commands), 2)

    def test_completed_gimbal_action_retries_when_pitch_is_stuck(self):
        class FreshGimbalLogger(FakeLogger):
            def get_sample(self, name, max_age_s=None):
                sample = super().get_sample(name, max_age_s)
                return ((sample[0], time.time()) if name == "gimbal" and sample else sample)

        class CompletedAction:
            has_succeeded = True
            state = "action_succeeded"

            def wait_for_completed(self):
                return True

        class StalledPitchOnceGimbal(SimulatedGimbal):
            def moveto(self, pitch, yaw, pitch_speed, yaw_speed):
                super().moveto(pitch, yaw, pitch_speed, yaw_speed)
                if len(self.commands) == 1:
                    self.logger.set("gimbal", (-9, self.yaw, -9, self.yaw))
                return CompletedAction()

        slam_map = OccupancyGridSLAM(self.settings)
        slam_map.update((0, 0, 0), 2000)
        logger = FreshGimbalLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = StalledPitchOnceGimbal(slam_map, logger, 2000)
        explorer = DFSExplorer(None, gimbal, logger, slam_map, self.settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()

        self.assertEqual(explorer._scan_for_direction((1, 0))[0], 2000)
        self.assertEqual(len(gimbal.commands), 2)

    def test_gimbal_scan_waits_for_sdk_action_before_next_command(self):
        class PendingAction:
            def __init__(self):
                self.complete_at = time.monotonic() + 0.56
                self.completion_confirmed = False

            @property
            def has_succeeded(self):
                return time.monotonic() >= self.complete_at

            @property
            def state(self):
                return "action_succeeded" if self.has_succeeded else "action_running"

            def wait_for_completed(self, timeout=None):
                if not self.has_succeeded:
                    raise AssertionError("SDK completion was awaited before success")
                self.completion_confirmed = True
                return True

        class BusyGimbal(SimulatedGimbal):
            def moveto(self, pitch, yaw, pitch_speed, yaw_speed):
                if getattr(self, "pending_action", None) is not None and not self.pending_action.has_succeeded:
                    raise MissionStop("overlapping gimbal action")
                super().moveto(pitch, yaw, pitch_speed, yaw_speed)
                self.pending_action = PendingAction()
                return self.pending_action

        slam_map = OccupancyGridSLAM(self.settings)
        slam_map.update((0, 0, 0), 2000)
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = BusyGimbal(slam_map, logger, 2000)
        explorer = DFSExplorer(None, gimbal, logger, slam_map, self.settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()

        explorer._scan_for_direction((1, 0))
        self.assertTrue(gimbal.pending_action.completion_confirmed)
        explorer._scan_for_direction((0, 1))
        self.assertTrue(gimbal.pending_action.completion_confirmed)
        self.assertEqual(len(gimbal.commands), 2)

    def test_gimbal_action_failure_stops_scan(self):
        class FailedAction:
            has_succeeded = False

            @property
            def state(self):
                return "action_failed"

        class FailedGimbal(SimulatedGimbal):
            def moveto(self, pitch, yaw, pitch_speed, yaw_speed):
                super().moveto(pitch, yaw, pitch_speed, yaw_speed)
                return FailedAction()

        slam_map = OccupancyGridSLAM(self.settings)
        slam_map.update((0, 0, 0), 2000)
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = FailedGimbal(slam_map, logger, 2000)
        explorer = DFSExplorer(None, gimbal, logger, slam_map, self.settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()

        with self.assertRaisesRegex(MissionStop, "gimbal moveto failed: action_failed"):
            explorer._scan_for_direction((1, 0))

    def test_dfs_updates_wall_grid_and_requires_fresh_threshold_scan(self):
        settings = copy.deepcopy(self.settings)
        settings["step_m"] = .6
        slam_map = OccupancyGridSLAM(settings)
        slam_map.update((0, 0, 0), 2000)
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, 1480)
        explorer = DFSExplorer(None, gimbal, logger, slam_map, settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()

        def unexpected_map_check(*args, **kwargs):
            raise AssertionError("DFS must not use occupancy cells to veto a ToF-clear step")

        slam_map.path_has_obstacle = unexpected_map_check
        slam_map.contains_world = unexpected_map_check
        self.assertTrue(explorer._can_step((0, 0), (1, 0)))
        self.assertEqual(explorer.wall_grid.state((0, 0), (1, 0)), "open")
        gimbal.range_provider = 300
        self.assertFalse(explorer._can_step((0, 0), (1, 0)))
        gimbal.range_provider = 301
        self.assertTrue(explorer._can_step((0, 0), (1, 0)))
        explorer._scan_for_direction = lambda delta: (0, 0)
        self.assertFalse(explorer._can_step((0, 0), (1, 0)))
        self.assertFalse(explorer.wall_grid.can_cross((0, 0), (1, 0)))

    def test_dfs_gimbal_action_uses_chassis_frame_for_both_axes(self):
        from robomaster.gimbal import COORDINATE_CAR

        sent = []
        sdk_gimbal = SimpleNamespace(
            _action_dispatcher=SimpleNamespace(send_action=sent.append))
        action = ChassisRelativeGimbal(sdk_gimbal).moveto(
            pitch=5, yaw=90, pitch_speed=30, yaw_speed=60)

        self.assertIs(sent[0], action)
        self.assertEqual(action._coordinate, COORDINATE_CAR)
        self.assertEqual(action.encode()._coordinate, COORDINATE_CAR)

    def test_dfs_gimbal_recenter_delegates_to_sdk(self):
        calls = []
        sdk_gimbal = SimpleNamespace(
            recenter=lambda **kwargs: calls.append(kwargs) or "recenter-action")

        result = ChassisRelativeGimbal(sdk_gimbal).recenter(
            pitch_speed=40, yaw_speed=50)

        self.assertEqual(result, "recenter-action")
        self.assertEqual(calls, [{"pitch_speed": 40, "yaw_speed": 50}])

    def test_dfs_explores_and_backtracks_inside_a_simulated_room(self):
        range_provider = lambda pose, yaw: self.room_range(pose, yaw, half_extent=2.0)
        slam_map = OccupancyGridSLAM(self.settings)
        slam_map.update((0, 0, 0), range_provider((0, 0, 0), 0))
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, range_provider)
        chassis = SimulatedChassis(slam_map, logger, gimbal, range_provider)
        settings = dict(self.settings)
        settings["max_nodes"] = 8
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, settings)

        result = explorer.run(FakeSlamWorker())

        self.assertEqual(result["status"], "node_limit_returned")
        self.assertEqual(len(result["visited"]), 8)
        self.assertGreater(result["moves"], len(result["visited"]) - 1)
        self.assertEqual(result["stack"], [[0, 0]])
        self.assertAlmostEqual(slam_map.pose[0], 0.0)
        self.assertAlmostEqual(slam_map.pose[1], 0.0)
        self.assertGreater(slam_map.to_dict()["counts"]["occupied_cells"], 0)

    def test_return_to_visited_cell_only_points_gimbal_along_travel(self):
        settings = dict(self.settings)
        settings["max_nodes"] = 2
        slam_map = OccupancyGridSLAM(settings)
        slam_map.update((0, 0, 0), 2000)
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, 2000)
        chassis = SimulatedChassis(slam_map, logger, gimbal, 2000)
        explorer = DFSExplorer(chassis, gimbal, logger, slam_map, settings)

        result = explorer.run(FakeSlamWorker())

        self.assertEqual(result["status"], "node_limit_returned")
        self.assertEqual(result["moves"], 2)
        self.assertEqual(len(gimbal.commands), 7)  # Recenter, four directions, travel and return scans.
        self.assertEqual(explorer.scanned_cells, {(0, 0)})

    def test_revisiting_scanned_cell_reuses_four_recorded_sides(self):
        slam_map = OccupancyGridSLAM(self.settings)
        slam_map.update((0, 0, 0), 2000)
        logger = FakeLogger()
        logger.set("attitude", (0, 0, 0))
        gimbal = SimulatedGimbal(slam_map, logger, 2000)
        explorer = DFSExplorer(None, gimbal, logger, slam_map, self.settings)
        explorer.base_pose = (0, 0, 0)
        explorer.slam_worker = FakeSlamWorker()

        first = explorer._scan_all_directions((0, 0))
        repeated = explorer._scan_all_directions((0, 0))

        self.assertEqual(first, repeated)
        self.assertEqual(len(gimbal.commands), 4)

    def test_return_rejects_a_now_blocked_shared_edge_without_scanning(self):
        settings = dict(self.settings)
        slam_map = OccupancyGridSLAM(settings)
        logger = FakeLogger()
        gimbal = SimulatedGimbal(slam_map, logger, 2000)
        explorer = DFSExplorer(None, gimbal, logger, slam_map, settings)
        explorer.slam_worker = FakeSlamWorker()
        explorer.wall_grid.observe((0, 0), (1, 0), .675, 600, 300, time.time())
        self.assertTrue(explorer._can_return((1, 0), (0, 0)))
        explorer.wall_grid.observe((1, 0), (-1, 0), .175, 100, 300, time.time())
        self.assertFalse(explorer._can_return((1, 0), (0, 0)))
        self.assertEqual(gimbal.commands, [])

    def test_slam_worker_uses_logger_streams_and_rejects_unsafe_status(self):
        timestamp = time.time()
        logger = FakeLogger()
        logger.set("position", (0, 0, 0), timestamp)
        logger.set("attitude", (0, 0, 0), timestamp)
        logger.set("tof", self.ranges, timestamp)
        logger.set("gimbal", (0, 0, 0, 0), timestamp)
        logger.set("status", (0,) * 10, timestamp)
        slam_map = OccupancyGridSLAM(self.settings)
        worker = SlamWorker(logger, slam_map, self.settings)
        worker.start()
        try:
            worker.wait_ready(timeout_s=2)
            self.assertTrue(worker.status()["ready"])
            self.assertEqual(slam_map.scan_count, 1)
            self.assertEqual(slam_map.latest_gimbal_yaw_deg, 0)
        finally:
            worker.stop()

        close_logger = FakeLogger()
        close_logger.set("position", (0, 0, 0), timestamp)
        close_logger.set("attitude", (0, 0, 0), timestamp)
        close_logger.set("tof", (83, 0, 0, 0), timestamp)
        close_logger.set("gimbal", (0, 0, 0, 0), timestamp)
        close_logger.set("status", (0,) * 10, timestamp)
        close_map = OccupancyGridSLAM(self.settings)
        close_worker = SlamWorker(close_logger, close_map, self.settings)
        close_worker.start()
        try:
            close_worker.wait_ready(timeout_s=2)
            self.assertEqual(close_map.latest_range_mm, 83)
        finally:
            close_worker.stop()

        channel_settings = copy.deepcopy(self.settings)
        channel_settings["sensor"]["tof_channel"] = 2
        channel_logger = FakeLogger()
        channel_logger.set("position", (0, 0, 0), timestamp)
        channel_logger.set("attitude", (0, 0, 0), timestamp)
        channel_logger.set("tof", (111, 222, 333, 444), timestamp)
        channel_logger.set("gimbal", (0, 45, 0, 45), timestamp)
        channel_logger.set("status", (0,) * 10, timestamp)
        channel_map = OccupancyGridSLAM(channel_settings)
        channel_worker = SlamWorker(channel_logger, channel_map, channel_settings)
        channel_worker.start()
        try:
            channel_worker.wait_ready(timeout_s=2)
            self.assertEqual(channel_map.latest_range_mm, 333)
            self.assertEqual(channel_map.latest_gimbal_yaw_deg, 45)
        finally:
            channel_worker.stop()

        invalid_settings = copy.deepcopy(self.settings)
        invalid_settings["sensor"]["tof_channel"] = 1
        invalid_logger = FakeLogger()
        invalid_map = OccupancyGridSLAM(invalid_settings)
        invalid_worker = SlamWorker(invalid_logger, invalid_map, invalid_settings)

        def set_invalid_tof_inputs(reading):
            sample_time = time.time()
            invalid_logger.set("position", (0, 0, 0), sample_time)
            invalid_logger.set("attitude", (0, 0, 0), sample_time)
            invalid_logger.set("tof", (83, reading, 0, 0), sample_time)
            invalid_logger.set("gimbal", (0, 0, 0, 0), sample_time)
            invalid_logger.set("status", (0,) * 10, sample_time)

        set_invalid_tof_inputs(0)
        invalid_worker.start()
        try:
            time.sleep(.25)
            self.assertFalse(invalid_worker.status()["ready"])
            self.assertTrue(invalid_worker.status()["tof_waiting"])
            self.assertFalse(invalid_worker.abort_event.is_set())
            set_invalid_tof_inputs(321)
            invalid_worker.wait_ready(timeout_s=2)
            self.assertEqual(invalid_map.latest_range_mm, 321)
            set_invalid_tof_inputs(0)
            time.sleep(.25)
            self.assertTrue(invalid_worker.status()["tof_waiting"])
            self.assertEqual(invalid_map.scan_count, 1)
            self.assertFalse(invalid_worker.abort_event.is_set())
            set_invalid_tof_inputs(456)
            for _ in range(20):
                if invalid_map.scan_count == 2:
                    break
                time.sleep(.05)
            self.assertEqual(invalid_map.latest_range_mm, 456)
            self.assertFalse(invalid_worker.status()["tof_waiting"])
        finally:
            invalid_worker.stop()

        unsafe_logger = FakeLogger()
        unsafe_logger.set("position", (0, 0, 0), timestamp)
        unsafe_logger.set("attitude", (0, 0, 0), timestamp)
        unsafe_logger.set("tof", self.ranges, timestamp)
        unsafe_logger.set("gimbal", (0, 0, 0, 0), timestamp)
        unsafe_status = [0] * 10
        unsafe_status[5] = 1
        unsafe_logger.set("status", unsafe_status, timestamp)
        unsafe_worker = SlamWorker(
            unsafe_logger, OccupancyGridSLAM(self.settings), self.settings
        )
        unsafe_worker.start()
        try:
            with self.assertRaisesRegex(MissionStop, "safety status flag 5"):
                unsafe_worker.wait_ready(timeout_s=2)
            self.assertTrue(unsafe_worker.abort_event.is_set())
        finally:
            unsafe_worker.stop()

    def test_persistent_zero_tof_waits_without_aborting_exploration(self):
        settings = copy.deepcopy(self.settings)
        settings["max_sample_age_s"] = .3
        settings["update_hz"] = 20
        logger = FakeLogger()

        def set_samples(reading):
            sample_time = time.time()
            logger.set("position", (0, 0, 0), sample_time)
            logger.set("attitude", (0, 0, 0), sample_time)
            logger.set("tof", (reading, 0, 0, 0), sample_time)
            logger.set("gimbal", (0, 0, 0, 0), sample_time)
            logger.set("status", (0,) * 10, sample_time)

        set_samples(500)
        slam_map = OccupancyGridSLAM(settings)
        worker = SlamWorker(logger, slam_map, settings)
        worker.start()
        try:
            worker.wait_ready(timeout_s=2)
            for _ in range(18):
                set_samples(0)
                time.sleep(.025)
            self.assertIsNone(worker.status()["error"])
            self.assertTrue(worker.status()["tof_waiting"])
            self.assertFalse(worker.abort_event.is_set())
            self.assertEqual(slam_map.latest_range_mm, 500)
            set_samples(700)
            for _ in range(30):
                if slam_map.latest_range_mm == 700:
                    break
                time.sleep(.025)
            self.assertEqual(slam_map.latest_range_mm, 700)
        finally:
            worker.stop()

    def test_slam_worker_waits_through_telemetry_gap_and_resumes(self):
        settings = copy.deepcopy(self.settings)
        settings["max_sample_age_s"] = .15
        settings["update_hz"] = 30

        class AgingLogger(FakeLogger):
            def get_sample(self, name, max_age_s=None):
                sample = super().get_sample(name)
                if sample is not None and max_age_s is not None:
                    if time.time() - sample[1] > max_age_s:
                        return None
                return sample

        logger = AgingLogger()

        def publish():
            timestamp = time.time()
            for name, value in (("position", (0, 0, 0)), ("attitude", (0, 0, 0)),
                                ("tof", self.ranges), ("gimbal", (0, 0, 0, 0)),
                                ("status", (0,) * 10)):
                logger.set(name, value, timestamp)

        publish()
        worker = SlamWorker(logger, OccupancyGridSLAM(settings), settings)
        worker.start()
        try:
            worker.wait_ready(timeout_s=2)
            time.sleep(.36)
            self.assertTrue(worker.status()["waiting_telemetry"])
            self.assertIsNone(worker.status()["error"])
            self.assertFalse(worker.abort_event.is_set())
            publish()
            for _ in range(30):
                if not worker.status()["waiting_telemetry"]:
                    break
                time.sleep(.02)
            self.assertFalse(worker.status()["waiting_telemetry"])
            self.assertGreaterEqual(worker.map.scan_count, 2)
        finally:
            worker.stop()

    def test_slam_skips_downward_tof_and_resumes_horizontal_mapping(self):
        settings = copy.deepcopy(self.settings)
        settings["update_hz"] = 30
        logger = FakeLogger()
        slam_map = OccupancyGridSLAM(settings)
        worker = SlamWorker(logger, slam_map, settings)

        def publish(pitch, distance):
            timestamp = time.time()
            for name, value in (("position", (0, 0, 0)), ("attitude", (0, 0, 0)),
                                ("tof", (distance,)), ("gimbal", (pitch, 0, pitch, 0)),
                                ("status", (0,) * 10)):
                logger.set(name, value, timestamp)

        publish(0, 900)
        worker.start()
        try:
            worker.wait_ready(timeout_s=2)
            initial_scans = slam_map.scan_count
            worker.pause_mapping()
            publish(-15, 100)
            time.sleep(.15)
            self.assertEqual(slam_map.scan_count, initial_scans)
            self.assertTrue(worker.status()["mapping_paused"])
            worker.resume_mapping()
            time.sleep(.1)
            self.assertEqual(slam_map.scan_count, initial_scans)
            publish(0, 800)
            for _ in range(30):
                if slam_map.scan_count > initial_scans:
                    break
                time.sleep(.02)
            self.assertGreater(slam_map.scan_count, initial_scans)
            self.assertEqual(slam_map.latest_range_mm, 800)
        finally:
            worker.stop()

    def test_gimbal_sample_waits_for_fresh_data_without_failing(self):
        class AgingLogger(FakeLogger):
            def get_sample(self, name, max_age_s=None):
                sample = super().get_sample(name)
                if sample is not None and max_age_s is not None:
                    if time.time() - sample[1] > max_age_s:
                        return None
                return sample

        class StoppableChassis:
            stops = 0

            def stop(self):
                self.stops += 1

        logger = AgingLogger()
        chassis = StoppableChassis()
        explorer = DFSExplorer(chassis, None, logger, self.make_map(), self.settings)
        explorer.slam_worker = FakeSlamWorker()
        logger.set("gimbal", (0, 0, 0, 0), time.time() - 10)
        publisher = threading.Thread(target=lambda: (time.sleep(.1), logger.set("gimbal", (0, 90, 0, 90))))
        publisher.start()
        try:
            yaw, pitch, _ = explorer._gimbal_sample()
            self.assertEqual((yaw, pitch), (90, 0))
            self.assertGreater(chassis.stops, 0)
            self.assertEqual(explorer.status, "ready")
        finally:
            publisher.join()

    def test_real_logger_streams_feed_slam_and_preserve_four_tof_csv_columns(self):
        chassis_module = FakeSDKModule()
        sensor_module = FakeSDKModule()
        gimbal_module = FakeSDKModule()
        robot = SimpleNamespace(chassis=chassis_module, sensor=sensor_module,
                                gimbal=gimbal_module)
        streams = {
            "position": {"enabled": True, "save": False, "frequency_hz": 10},
            "attitude": {"enabled": True, "save": False, "frequency_hz": 10},
            "status": {"enabled": True, "save": False, "frequency_hz": 5},
            "tof": {"enabled": True, "save": True, "frequency_hz": 5},
            "gimbal": {"enabled": True, "save": False, "frequency_hz": 5},
        }
        with tempfile.TemporaryDirectory() as temp:
            logger = SensorLogger(robot, {"directory": temp, "streams": streams})
            logger.start()
            try:
                chassis_module.callbacks["sub_position"]((0, 0, 0))
                chassis_module.callbacks["sub_attitude"]((0, 0, 0))
                chassis_module.callbacks["sub_status"]((0,) * 10)
                sensor_module.callbacks["sub_distance"](tuple(self.ranges))
                gimbal_module.callbacks["sub_angle"]((0, 0, 0, 0))
                slam_map = OccupancyGridSLAM(self.settings)
                worker = SlamWorker(logger, slam_map, self.settings)
                worker.start()
                try:
                    worker.wait_ready(timeout_s=2)
                    self.assertEqual(slam_map.scan_count, 1)
                    self.assertEqual(slam_map.latest_range_mm, self.ranges[0])
                    self.assertEqual(logger.get_latest("tof"), tuple(self.ranges))
                    history = logger.get_history_since()
                    self.assertEqual(history["streams"]["tof"][0][2], tuple(self.ranges))
                finally:
                    worker.stop()
            finally:
                logger.stop()

            with (logger.run_dir / "tof.csv").open(newline="", encoding="utf-8") as file:
                rows = list(csv.reader(file))
            self.assertEqual(rows[0], [
                "timestamp", "elapsed_s", "tof_0_mm", "tof_1_mm", "tof_2_mm", "tof_3_mm"
            ])
            self.assertEqual(len(rows[1]), 6)

    def test_map_import_rejects_invalid_schema_and_live_replacement(self):
        slam_map = self.make_map()
        valid_document = slam_map.to_dict()
        target = OccupancyGridSLAM(self.settings)

        boolean_version = copy.deepcopy(valid_document)
        boolean_version["version"] = True
        with self.assertRaisesRegex(ValueError, "unsupported map file version"):
            target.load_dict(boolean_version)

        truncated_grid = copy.deepcopy(valid_document)
        truncated_grid["data"].pop()
        with self.assertRaisesRegex(ValueError, "invalid grid metadata"):
            target.load_dict(truncated_grid)

        live_worker = SimpleNamespace(is_running=True)
        dashboard = Dashboard(
            SimpleNamespace(camera=SimpleNamespace()),
            SimpleNamespace(stream_settings={}, dropped_rows=0),
            self.config["dashboard"], slam_map=target, slam_worker=live_worker,
        )
        with self.assertRaisesRegex(ValueError, "stop exploration"):
            dashboard.import_map(valid_document)


if __name__ == "__main__":
    unittest.main()
