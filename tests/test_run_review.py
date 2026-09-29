import csv
import json
import tempfile
import unittest
from pathlib import Path

from src.run_review import RunStore, parse_value


def write_csv(path, columns, rows):
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["timestamp", "elapsed_s", *columns])
        writer.writerows(rows)


class ReviewTests(unittest.TestCase):
    def test_parse_value_supports_sdk_arrays(self):
        self.assertEqual(parse_value("(1, 2, 3)"), [1.0, 2.0, 3.0])
        self.assertEqual(parse_value("True"), 1)
        self.assertIsNone(parse_value(""))

    def test_run_summary_gaps_events_and_downsampling(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = Path(temp) / "20260924_120000"
            run_dir.mkdir()
            write_csv(run_dir / "position.csv", ["x_m", "y_m", "z_deg"], [
                [1, 0.0, 0, 0, 0],
                [2, 0.1, 0.1, 0, 0],
                [3, 2.1, 0.2, 0, 0],
                [4, 2.2, 0.3, 0, 0],
            ])
            write_csv(run_dir / "status.csv", ["slip", "impact_x"], [
                [1, 0.0, 0, 0], [2, 0.1, 1, 0], [3, 0.2, 1, 0], [4, 0.3, 0, 0],
            ])
            write_csv(run_dir / "tof.csv", ["tof_1_mm"], [
                [1, 0.0, 500], [2, 0.1, 150], [3, 0.2, 140],
                [4, 0.3, 600], [5, 0.4, 180],
            ])
            (run_dir / "run_summary.json").write_text(json.dumps({
                "status": "failed", "error": "waypoint timeout", "dropped_csv_rows": 2,
                "stream_settings": {"position": {"frequency_hz": 10}},
            }), encoding="utf-8")

            store = RunStore(temp, max_points=2)
            self.assertEqual(store.list_runs()[0]["name"], run_dir.name)
            result = store.load_run(run_dir.name)
            self.assertEqual(result["streams"]["position"]["displayed_rows"], 2)
            self.assertEqual(result["streams"]["position"]["total_rows"], 4)
            self.assertEqual(result["streams"]["position"]["gap_count"], 1)
            self.assertEqual(result["duration_s"], 2.2)
            messages = " ".join(issue["message"] for issue in result["issues"])
            self.assertIn("waypoint timeout", messages)
            self.assertIn("dropped 2 rows", messages)
            self.assertIn("no samples for 2.00 s", messages)
            self.assertEqual(messages.count("status: slip detected"), 1)
            self.assertEqual(messages.count("ToF #0 below 200 mm"), 2)
            self.assertEqual(store.csv_path(run_dir.name, "position"), run_dir / "position.csv")
            with self.assertRaises(ValueError):
                store.load_run("../outside")
            with self.assertRaises(ValueError):
                store.csv_path(run_dir.name, "../position")

    def test_old_run_without_summary_still_opens(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = Path(temp) / "run1"
            run_dir.mkdir()
            write_csv(run_dir / "battery.csv", ["percent"], [[1, 0, 88]])
            result = RunStore(temp).load_run("run1")
            self.assertEqual(result["summary"], {})
            self.assertEqual(result["streams"]["battery"]["samples"][0][2], [88.0])

    def test_no_safe_direction_is_reported_in_review(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = Path(temp) / "blocked"
            run_dir.mkdir()
            (run_dir / "run_summary.json").write_text(json.dumps({
                "status": "no_safe_direction", "error": "ไม่มีทิศที่ผ่านระยะเผื่อ",
            }), encoding="utf-8")
            result = RunStore(temp).load_run("blocked")
            self.assertEqual(result["issues"][0]["level"], "warning")
            self.assertIn("ไม่มีทิศ", result["issues"][0]["message"])

    def test_stopped_run_and_stalled_alignment_are_visible_in_review(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = Path(temp) / "alignment_stop"
            run_dir.mkdir()
            (run_dir / "run_summary.json").write_text(json.dumps({
                "status": "stopped", "error": "movement ToF data is missing or stale",
                "exploration": {"alignments": {"2,0": {
                    "status": "stalled", "reason": "remaining correction 0.059 m at (2, 0)",
                }}},
            }), encoding="utf-8")
            issues = RunStore(temp).load_run(run_dir.name)["issues"]
            self.assertTrue(any("movement ToF" in issue["message"] for issue in issues))
            self.assertTrue(any("0.059 m" in issue["message"] for issue in issues))

    def test_unconfirmed_cell_center_is_visible_in_review(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = Path(temp) / "off_center"
            run_dir.mkdir()
            (run_dir / "run_summary.json").write_text(json.dumps({
                "status": "stopped", "error": "DFS stopped off cell center",
                "exploration": {"last_motion_stop": {
                    "status": "emergency_stop_off_center", "center_cell": [1, 0],
                    "center_confirmed": False, "planned_center_error_m": .15,
                }},
            }), encoding="utf-8")
            issues = RunStore(temp).load_run(run_dir.name)["issues"]
            self.assertTrue(any("0.150 m from planned center" in issue["message"] and
                                issue["level"] == "warning" for issue in issues))

    def test_target_inspection_failure_is_visible_in_review(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = Path(temp) / "target_stop"
            run_dir.mkdir()
            (run_dir / "run_summary.json").write_text(json.dumps({
                "status": "stopped", "error": "camera stopped",
                "exploration": {"wall_inspections": [{
                    "cell": [0, 0], "direction": [1, 0],
                    "status": "stopped", "reason": "camera stopped", "targets": []},
                    {"cell": [0, 0], "direction": [0, 1],
                     "status": "targets_checked", "targets": [{
                         "color": "red", "shape": "square", "status": "aim_limit"}]}]},
            }), encoding="utf-8")
            issues = RunStore(temp).load_run(run_dir.name)["issues"]
            self.assertTrue(any("Target inspection stopped" in issue["message"]
                                for issue in issues))
            self.assertTrue(any("Target red / square" in issue["message"]
                                for issue in issues))

    def test_zero_tof_is_reported_once_per_invalid_streak(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = Path(temp) / "zeros"
            run_dir.mkdir()
            write_csv(run_dir / "tof.csv", ["tof_0_mm"], [
                [1, 0.0, 0], [2, 0.1, 0], [3, 0.2, 350], [4, 0.3, 0],
            ])
            result = RunStore(temp).load_run("zeros")
            messages = [issue["message"] for issue in result["issues"]]
            self.assertEqual(messages.count("ToF #0 returned 0 mm; scan skipped"), 2)

    def test_rear_ir_events_follow_saved_polarity_and_port_mapping(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = Path(temp) / "ir_run"
            run_dir.mkdir()
            (run_dir / "run_summary.json").write_text(json.dumps({
                "rear_ir_settings": {"right": {"id": 3, "port": 1},
                                     "left": {"id": 4, "port": 1}, "active_io": 0},
                "rear_ir_recoveries": [{"side": "right", "status": "cleared",
                                        "elapsed_s": 0.25, "distance_m": 0.04,
                                        "reason": None}],
            }), encoding="utf-8")
            write_csv(run_dir / "adapter.csv", ["io_5", "io_7"], [
                [1, 0.0, 1, 1], [2, 0.1, 0, 1], [3, 0.2, 0, 0],
                [4, 0.3, 0, 0], [5, 0.4, 1, 1], [6, 0.5, 0, 1],
            ])
            issues = RunStore(temp).load_run("ir_run")["issues"]
            messages = [issue["message"] for issue in issues]
            self.assertEqual(messages.count("Rear IR right detected (IO 0)"), 2)
            self.assertEqual(messages.count("Rear IR left detected (IO 0)"), 1)
            self.assertIn("Rear IR right recovery cleared (0.040 m)", messages)

    def test_ir_recovery_direction_appears_in_review(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = Path(temp) / "direction"
            run_dir.mkdir()
            (run_dir / "run_summary.json").write_text(json.dumps({
                "front_ir_recoveries": [{"side": "left", "status": "cleared",
                                         "direction": "backward", "elapsed_s": 0.5,
                                         "distance_m": 0.03, "reason": None}],
            }), encoding="utf-8")
            issues = RunStore(temp).load_run("direction")["issues"]
            self.assertIn("Front IR left recovery cleared (0.030 m) via backward",
                          [issue["message"] for issue in issues])

    def test_rear_ir_review_uses_each_sides_polarity(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = Path(temp) / "mixed_ir"
            run_dir.mkdir()
            (run_dir / "run_summary.json").write_text(json.dumps({
                "rear_ir_settings": {
                    "right": {"id": 3, "port": 1, "active_io": 1},
                    "left": {"id": 4, "port": 1, "active_io": 0},
                },
            }), encoding="utf-8")
            write_csv(run_dir / "adapter.csv", ["io_5", "io_7"], [
                [1, 0.0, 0, 1], [2, 0.1, 1, 0], [3, 0.2, 1, 0],
            ])
            messages = [issue["message"] for issue in RunStore(temp).load_run("mixed_ir")["issues"]]
            self.assertEqual(messages.count("Rear IR right detected (IO 1)"), 1)
            self.assertEqual(messages.count("Rear IR left detected (IO 0)"), 1)

    def test_front_ir_detection_and_recovery_appear_in_review(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = Path(temp) / "front_ir"
            run_dir.mkdir()
            (run_dir / "run_summary.json").write_text(json.dumps({
                "front_ir_settings": {
                    "right": {"id": 3, "port": 2, "active_io": 1},
                    "left": {"id": 4, "port": 2, "active_io": 0},
                },
                "front_ir_recoveries": [{"side": "left", "status": "stopped",
                                         "elapsed_s": .2, "distance_m": .01,
                                         "reason": "rear IR blocks escape"}],
            }), encoding="utf-8")
            write_csv(run_dir / "adapter.csv", ["io_6", "io_8"], [
                [1, 0.0, 0, 1], [2, 0.1, 1, 1], [3, 0.2, 0, 0],
            ])
            issues = RunStore(temp).load_run(run_dir.name)["issues"]
            messages = [issue["message"] for issue in issues]
            self.assertIn("Front IR right detected (IO 1)", messages)
            self.assertIn("Front IR left detected (IO 0)", messages)
            self.assertTrue(any("Front IR left recovery stopped" in message for message in messages))


if __name__ == "__main__":
    unittest.main()
