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


if __name__ == "__main__":
    unittest.main()
