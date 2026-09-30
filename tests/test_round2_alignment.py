import math
import time
import unittest
from unittest.mock import Mock

from src.config_loader import load_config
from src.planner import GridMap
from src.round2_alignment import Round2Aligner, AlignmentTelemetry
from src.mission_stop import MissionStop


class Round2AlignmentTests(unittest.TestCase):
    def setUp(self):
        self.settings = load_config()["exploration"]
        self.settings["heading_alignment"]["enabled"] = False
        self.settings["alignment"].update(enabled=True, wall_distance_m=.28)
        self.grid = GridMap()
        self.grid.cells[(0, 0)] = {"x+": "wall", "y+": "open", "x-": "unknown"}
        self.pose = [0., 0., 0.]
        self.chassis = Mock()
        self.chassis.get_pose.side_effect = lambda: tuple(self.pose)
        self.logger = Mock()
        self.states = []
        self.aligner = Round2Aligner(self.chassis, Mock(), self.logger, self.grid,
                                     self.settings, (0, 0), 0, self.states.append)
        self.aligner.slam_worker.status = Mock(return_value={})
        self.aligner._prepare_gimbal = Mock()
        self.aligner._sensor_offset = Mock(return_value=0)
        self.scans = []

        def scan(delta, **kwargs):
            self.scans.append(delta)
            self.aligner.map.latest_scan_timestamp = time.time()
            a = math.radians(self.grid.base_pose[2])
            along = self.pose[0] * math.cos(a) + self.pose[1] * math.sin(a)
            return (.35 - along) * 1000, 0

        def drive(x, y, **kwargs):
            self.pose[:2] = [x, y]
            return tuple(self.pose)

        self.aligner._scan_for_direction = Mock(side_effect=scan)
        self.aligner._drive_holding_current_yaw = Mock(side_effect=drive)

    def test_centers_using_only_known_wall_and_publishes_event(self):
        self.aligner.align((0, 0), "startup")
        self.assertAlmostEqual(self.pose[0], .07)
        self.assertEqual(set(self.scans), {(1, 0)})
        self.assertEqual(self.states[-1]["events"][-1]["position"]["status"], "moved")
        self.assertEqual(self.grid.cells[(0, 0)]["x-"], "unknown")

    def test_preserves_shooting_standoff(self):
        self.aligner.align((0, 0), "before_shooting", (-.10, 0))
        self.assertAlmostEqual(self.pose[0], -.03)

    def test_rotated_grid_preserves_world_standoff(self):
        self.grid.base_pose[2] = 90
        self.aligner.base_pose = (0, 0, 90)
        self.aligner.align((0, 0), "before_shooting", (0, -.10))
        self.assertAlmostEqual(self.pose[0], 0)
        self.assertAlmostEqual(self.pose[1], -.03)

    def test_confirmed_ir_lane_is_not_recentered(self):
        self.aligner.align((0, 0), "before_shooting", last_ir_lane={
            "status": "confirmed", "cells": [[0, 0], [1, 0]]})
        self.aligner._drive_holding_current_yaw.assert_not_called()
        self.assertEqual(self.states[-1]["position"]["status"], "retained_ir_lane")

    def test_no_known_walls_does_not_search(self):
        self.grid.cells[(0, 0)] = {"x+": "open"}
        self.aligner.align((0, 0), "startup")
        self.assertEqual(self.states[-1]["status"], "skipped_no_mapped_wall")
        self.assertEqual(self.scans, [])
        self.aligner._drive_holding_current_yaw.assert_not_called()

    def test_missing_mapped_wall_does_not_move(self):
        self.aligner._scan_for_direction.side_effect = lambda delta: (2000, 0)
        self.aligner.align((0, 0), "startup")
        self.aligner._drive_holding_current_yaw.assert_not_called()
        self.assertEqual(self.states[-1]["position"]["status"], "no_wall")

    def test_heading_correction_precedes_centering(self):
        self.aligner.settings["heading_alignment"]["enabled"] = True
        self.aligner._align_heading = Mock(return_value={"status": "applied"})
        self.aligner.align((0, 0), "startup")
        calls = self.aligner._drive_holding_current_yaw.call_args_list
        self.assertEqual(calls[0].kwargs["kind"], "heading_alignment")
        self.assertEqual(calls[1].kwargs["kind"], "alignment")

    def test_failure_stops_and_records_reason(self):
        self.aligner._scan_for_direction.side_effect = MissionStop("gimbal failed")
        with self.assertRaises(MissionStop):
            self.aligner.align((0, 0), "startup")
        self.assertEqual(self.states[-1]["events"][-1]["error"], "gimbal failed")
        self.chassis.stop.assert_called()

    def test_scan_rejects_missing_invalid_and_unsynchronized_data(self):
        samples = {"tof": ((350,), 10), "gimbal": ((0, 15), 10),
                   "position": ((0, 0, 0), 10)}
        self.logger.get_sample.side_effect = lambda name, **kw: samples.get(name)
        self.assertEqual(self.aligner._latest_alignment_scan(), (10, 15, 350))
        for reading in (0, 65535, float("nan")):
            samples["tof"] = ((reading,), 10)
            self.assertEqual(self.aligner._latest_alignment_scan(), (None,) * 3)
        samples["tof"] = ((350,), 1)
        self.assertEqual(self.aligner._latest_alignment_scan(), (None,) * 3)
        del samples["tof"]
        self.assertEqual(self.aligner._latest_alignment_scan(), (None,) * 3)

    def test_telemetry_requires_complete_fresh_safe_samples(self):
        telemetry = AlignmentTelemetry(self.logger, self.settings)
        samples = {}
        self.logger.get_sample.side_effect = lambda name, **kw: samples.get(name)
        self.assertTrue(telemetry.status()["waiting_telemetry"])
        samples.update(position=((0, 0, 0), 10), attitude=((0, 0, 0), 10),
                       status=((0,) * 10, 10))
        self.assertFalse(telemetry.status()["waiting_telemetry"])
        samples["status"] = ((0,) * 4 + (1,) + (0,) * 5, 10)
        self.assertTrue(telemetry.status()["error"])
        self.assertTrue(telemetry.abort_event.is_set())

    def test_shared_scan_collects_fresh_logger_samples_without_slam(self):
        del self.aligner._scan_for_direction
        self.aligner.slam_worker.status.return_value = {"error": None}
        pitch = self.settings["gimbal"]["pitch_deg"]
        values = {"tof": (350,), "gimbal": (pitch, 0, pitch, 0),
                  "position": (0, 0, 0), "attitude": (0, 0, 0)}
        self.logger.get_sample.side_effect = lambda name, **kw: (values[name], time.time())
        self.aligner.gimbal.moveto.return_value = None
        reading, yaw = self.aligner._scan_for_direction((1, 0))
        self.assertEqual((reading, yaw), (350, 0))
        self.aligner.gimbal.moveto.assert_called_once()
