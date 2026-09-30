import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from src.dashboard import Dashboard
from src.chassis import grid_motion_settings
from src.config_loader import load_config
from src.logger import SensorLogger
from src.mission_stop import MissionStop
from src.planner import GridMap
from src.round2_navigation import Round2Navigator
from src.run_review import RunStore


class FakeChassis:
    settings = {"position_tolerance_m": .01}

    def __init__(self):
        self.recoveries = []
        self.commands = []
        self.failure = None
        self.partial = False
        self.stops = 0
        self.motion_options = []

    def stop(self):
        self.stops += 1

    def move_to(self, x, y, yaw=None, on_ir_recovered=None, disable_timeout=False):
        self.motion_options.append((yaw, disable_timeout))
        for before, after in self.recoveries:
            if on_ir_recovered:
                replacement = on_ir_recovered(before, after, (x, y))
                if replacement is not None:
                    x, y = replacement
        if self.failure:
            raise self.failure
        self.commands.append((x, y))
        return (x - .1 if self.partial else x, y, 0)


class Round2NavigationTests(unittest.TestCase):
    def test_round2_uses_grid_pid_settings_and_configured_heading_source(self):
        config = load_config()
        config["exploration"]["heading_source"] = "gimbal"
        config["exploration"]["max_speed_m_s"] = .17
        settings = grid_motion_settings(config)
        self.assertEqual(settings["pid"], config["motion"]["pid"])
        self.assertEqual(settings["heading_source"], "gimbal")
        self.assertEqual(settings["max_speed_m_s"], .17)
        self.assertEqual(settings["braking_decel_m_s2"], config["motion"]["braking_decel_m_s2"])
        self.assertEqual(grid_motion_settings(config, .12)["max_speed_m_s"], .12)
        self.assertEqual(grid_motion_settings(config, 10)["max_speed_m_s"],
                         config["motion"]["max_speed_m_s"])
        for invalid in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                grid_motion_settings(config, invalid)

    def test_initial_heading_is_held_for_first_move_recovery_and_return(self):
        self.nav.heading_deg = 37
        self.nav.move_to(None, (0, 0), (0, 0))
        self.teach()
        self.nav.move_to((1, 0), (0, 0), (0, 0))
        self.assertEqual(self.chassis.motion_options, [(37, True)] * 3)
        self.assertEqual(self.chassis.commands[-1], (0, .03))
        self.assertEqual(self.nav.snapshot()["travel_heading_deg"], 37)

    def setUp(self):
        self.grid = GridMap()
        self.grid.cells = {(0, 0): {"x+": "open"},
                           (1, 0): {"x-": "open", "y+": "open"},
                           (1, 1): {"y-": "open"}}
        self.chassis = FakeChassis()
        self.states = []
        self.settings = {"enabled": True, "min_shift_m": .005, "max_offset_m": .1}
        self.nav = Round2Navigator(self.chassis, self.grid, self.settings, (0, 0),
                                   self.states.append)

    def teach(self):
        self.chassis.recoveries = [((.2, 0, 0), (.2, .03, 0))]
        self.nav.move_to((0, 0), (1, 0), (.6, 0))
        self.chassis.recoveries = []

    def test_retargets_and_reuses_lane_in_reverse_without_affecting_other_edge(self):
        self.teach()
        self.assertEqual(self.chassis.commands[-1], (.6, .03))
        self.assertIn("candidate", [s["last_ir_lane"]["status"] for s in self.states
                                    if s["last_ir_lane"]])
        self.nav.move_to((1, 0), (0, 0), (0, 0))
        self.assertEqual(self.chassis.commands[-1], (0, .03))
        self.nav.move_to((0, 0), (1, 0), (.6, 0))
        self.nav.move_to((1, 0), (1, 1), (.6, .6))
        self.assertEqual(self.chassis.commands[-1], (.6, .6))
        self.assertEqual(len(self.nav.lanes), 1)

    def test_shooting_standoff_is_preserved_per_visit(self):
        self.chassis.recoveries = [((.2, 0, 0), (.2, .03, 0))]
        self.nav.move_to((0, 0), (1, 0), (.4, 0))
        self.assertEqual(self.chassis.commands[-1], (.4, .03))
        self.chassis.recoveries = []
        self.nav.move_to((1, 0), (0, 0), (.15, 0))
        self.assertEqual(self.chassis.commands[-1], (.15, .03))
        self.nav.move_to((0, 0), (1, 0), (.6, 0))
        self.assertEqual(self.chassis.commands[-1], (.6, .03))

    def test_rotated_translated_map_uses_odometry_coordinates(self):
        self.grid.base_pose = [4, 5, 90]
        self.nav = Round2Navigator(self.chassis, self.grid, self.settings, (-3, -3))
        self.chassis.recoveries = [((1, 2.2, 90), (.97, 2.2, 90))]
        self.nav.move_to((0, 0), (1, 0), (1, 2.6))
        self.assertAlmostEqual(self.chassis.commands[-1][0], .97)
        self.assertAlmostEqual(self.chassis.commands[-1][1], 2.6)
        self.chassis.recoveries = []
        self.nav.move_to((1, 0), (0, 0), (1, 2))
        self.assertAlmostEqual(self.chassis.commands[-1][0], .97)
        self.assertAlmostEqual(self.chassis.commands[-1][1], 2)

    def test_longitudinal_recovery_does_not_shorten_waypoint(self):
        self.chassis.recoveries = [((.2, 0, 0), (.17, .002, 0))]
        self.nav.move_to((0, 0), (1, 0), (.6, 0))
        self.assertEqual(self.chassis.commands[-1], (.6, 0))
        self.assertFalse(self.nav.lanes)

    def test_failed_partial_or_interrupted_move_does_not_keep_lane(self):
        for failure in (TimeoutError("timeout"), MissionStop("blocked"), KeyboardInterrupt(), None):
            with self.subTest(failure=failure):
                self.setUp()
                self.teach()
                self.chassis.failure = failure
                self.chassis.partial = failure is None
                with self.assertRaises(type(failure) if failure is not None else MissionStop):
                    self.nav.move_to((1, 0), (0, 0), (0, 0))
                self.assertFalse(self.nav.lanes)
                self.assertEqual(self.nav.last_lane["status"], "aborted")
                self.assertGreater(self.chassis.stops, 0)

    def test_offset_budget_is_total_and_blocks_excessive_correction(self):
        self.teach()
        self.chassis.recoveries = [((.2, .03, 0), (.2, .11, 0))]
        with self.assertRaisesRegex(MissionStop, "offset or map limit"):
            self.nav.move_to((1, 0), (0, 0), (0, 0))
        self.assertFalse(self.nav.lanes)

    def test_closed_edge_or_outside_target_does_not_move(self):
        with self.assertRaises(MissionStop):
            self.nav.move_to((0, 0), (1, 1), (.6, .6))
        with self.assertRaises(MissionStop):
            self.nav.move_to((0, 0), (1, 0), (.6, .4))
        self.assertFalse(self.chassis.commands)

    def test_disabled_or_initial_positioning_keeps_original_waypoint(self):
        self.chassis.recoveries = [((.2, 0, 0), (.2, .03, 0))]
        self.nav.move_to(None, (0, 0), (0, 0))
        self.settings["enabled"] = False
        self.nav.move_to((0, 0), (1, 0), (.6, 0))
        self.assertEqual(self.chassis.commands, [(0, 0), (.6, 0)])
        self.assertFalse(self.nav.lanes)

    def test_loaded_map_size_limits_offsets(self):
        self.grid.cell_size_m = .1
        with self.assertRaises(ValueError):
            Round2Navigator(self.chassis, self.grid, self.settings, (0, 0))

    def test_live_status_and_saved_review_include_round2_lane(self):
        self.teach()
        state = self.nav.snapshot()
        # Snapshots do not mutate the navigator when consumed by UI/logging.
        state["ir_lanes"][0]["offset_m"][1] = 99
        self.assertAlmostEqual(self.nav.snapshot()["ir_lanes"][0]["offset_m"][1], .03)
        with tempfile.TemporaryDirectory() as temp:
            robot = SimpleNamespace(battery=SimpleNamespace(
                sub_battery_info=lambda **kwargs: True, unsub_battery_info=lambda: True))
            logger = SensorLogger(robot, {"directory": Path(temp), "streams": {
                "battery": {"enabled": True, "save": True, "frequency_hz": 1}}})
            logger.start()
            logger.round2_navigation = self.nav.snapshot()
            dashboard = Dashboard(SimpleNamespace(camera=None), logger,
                                  {"enabled": False})
            self.assertEqual(dashboard.snapshot()["round2_navigation"], self.nav.snapshot())
            logger.stop()
            summary = json.loads((logger.run_dir / "run_summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary["round2_navigation"], self.nav.snapshot())
            result = RunStore(temp).load_run(logger.run_dir.name)
            self.assertEqual(result["summary"]["round2_navigation"], self.nav.snapshot())
