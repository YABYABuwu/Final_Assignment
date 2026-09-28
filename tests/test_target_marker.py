import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from src.planner import GridMap, MultiTargetPlanner
from src.target_marker import (
    build_target_record,
    fire_infrared,
    load_target_document,
    next_target_id,
    parse_cell,
    save_target_record,
    select_confirmed_detection,
)


class TargetMarkerTests(unittest.TestCase):
    def detection(self, **overrides):
        value = {
            "color": "red",
            "shape": "circle",
            "area_px2": 1200.0,
            "center_px": [320, 180],
            "center_offset_norm": [0.02, -0.01],
            "stability_hits": 3,
            "stability_required": 3,
            "confirmed": True,
        }
        value.update(overrides)
        return value

    def test_parse_cell(self):
        self.assertEqual(parse_cell("3, -2"), (3, -2))
        with self.assertRaisesRegex(ValueError, "exactly two"):
            parse_cell("3,2,1")

    def test_selects_only_fresh_confirmed_centered_detection(self):
        status = {
            "enabled": True,
            "error": None,
            "age_ms": 80,
            "detections": [
                self.detection(color="blue", area_px2=5000),
                self.detection(center_offset_norm=[0.5, 0.0]),
                self.detection(confirmed=False),
                self.detection(area_px2=1800),
            ],
        }
        selected, error = select_confirmed_detection(
            status, color="red", shape="circle", max_center_offset=.22,
        )
        self.assertIsNone(error)
        self.assertEqual(selected["area_px2"], 1800)

        status["age_ms"] = 800
        selected, error = select_confirmed_detection(status, max_age_ms=500)
        self.assertIsNone(selected)
        self.assertIn("stale", error)

    def test_fire_infrared_never_uses_water_default(self):
        calls = []
        device = SimpleNamespace(fire=lambda **kwargs: calls.append(kwargs) or True)
        module = SimpleNamespace(INFRARED_FIRE="ir")
        self.assertTrue(fire_infrared(module, device, times=2))
        self.assertEqual(calls, [{"fire_type": "ir", "times": 2}])
        with self.assertRaises(ValueError):
            fire_infrared(module, device, times=6)

    def test_saved_target_is_round2_compatible_and_atomic(self):
        detection = self.detection()
        record = build_target_record(
            "T1", (3, 2), detection, pose=(1.2, .6, 2.5),
            gimbal=(0, 90, 0, 92.5), side="y+", fired=True, fire_times=1,
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "targets.json"
            document = save_target_record(path, record)
            self.assertEqual(document["targets"][0]["pos"], [3, 2])
            self.assertEqual(document["targets"][0]["side"], "y+")
            self.assertTrue(document["targets"][0]["infrared_test"]["fired"])
            self.assertFalse(list(path.parent.glob("*.tmp")))

            loaded = load_target_document(path)
            self.assertEqual(loaded["targets"][0]["color"], "red")
            self.assertEqual(next_target_id(loaded["targets"]), "T2")
            with self.assertRaisesRegex(ValueError, "already exists"):
                save_target_record(path, record)

            replacement = dict(record, color="blue")
            save_target_record(path, replacement, replace=True)
            with path.open(encoding="utf-8") as file:
                self.assertEqual(json.load(file)["targets"][0]["color"], "blue")

    def test_record_can_be_planned_directly_by_round2_planner(self):
        record = build_target_record(
            "T1", (1, 0), self.detection(), pose=(0, 0, 0),
            gimbal=(0, 0, 0, 0), fired=True, fire_times=1,
        )
        grid = GridMap(cell_size_m=.6)
        grid.cells[(0, 0)] = {"x+": "open", "x-": "wall", "y+": "wall", "y-": "wall"}
        grid.cells[(1, 0)] = {"x+": "wall", "x-": "open", "y+": "wall", "y-": "wall"}
        plan = MultiTargetPlanner(grid, max_shooting_dist=1).plan((0, 0), [record])
        self.assertTrue(plan["success"])
        self.assertEqual(plan["target_order"], ["T1"])


if __name__ == "__main__":
    unittest.main()
