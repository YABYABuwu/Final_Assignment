import copy
import unittest

from src.config_loader import load_config
from src.slam import CellWallGrid, OccupancyGridSLAM


class CellWallGridTests(unittest.TestCase):
    def test_ray_opens_crossed_edges_and_wall_is_shared(self):
        grid = CellWallGrid(0.6, 20)
        self.assertFalse(grid.can_cross((0, 0), (1, 0)))
        grid.observe((0, 0), (1, 0), 1.555, 1480, 300, 1.0)
        self.assertTrue(grid.can_cross((0, 0), (1, 0)))
        self.assertEqual(grid.state((1, 0), (1, 0)), "open")
        self.assertEqual(grid.state((2, 0), (1, 0)), "wall")
        self.assertEqual(grid.state((3, 0), (-1, 0)), "wall")
        # A reciprocal open edge alone does not grant a fresh ToF check.
        self.assertFalse(grid.can_cross((1, 0), (-1, 0)))
        grid.observe((1, 0), (-1, 0), .16, 85, 300, 2.0)
        self.assertEqual(grid.state((0, 0), (1, 0)), "wall")
        self.assertFalse(grid.can_cross((0, 0), (1, 0)))

    def test_current_edge_uses_300_mm_threshold_inclusively(self):
        grid = CellWallGrid(.6, 20)
        grid.observe((0, 0), (1, 0), .375, 300, 300, 1.0)
        self.assertFalse(grid.can_cross((0, 0), (1, 0)))
        grid.observe((0, 0), (0, -1), .376, 301, 300, 1.0)
        self.assertEqual(grid.state((0, 0), (0, -1)), "open")
        self.assertTrue(grid.can_cross((0, 0), (0, -1)))
        self.assertFalse(grid.can_cross((0, -1), (0, 1)))
        self.assertEqual(grid.snapshot((0, 0, 0), (0, 0))["cells"][1]["sides"]["y-"].get("wall_threshold_mm"), 300)

    def test_far_ray_cannot_overwrite_direct_open_edge_from_either_side(self):
        grid = CellWallGrid(.6, 20)
        grid.observe((0, 0), (1, 0), 1.7, 1625, 300, 1.0)
        grid.observe((1, 0), (-1, 0), .91, 835, 300, 2.0)
        grid.observe((4, 0), (-1, 0), 2.309, 2234, 300, 3.0)

        self.assertEqual(grid.state((0, 0), (1, 0)), "open")
        self.assertTrue(grid.can_cross((0, 0), (1, 0)))
        self.assertTrue(grid.can_cross((1, 0), (-1, 0)))
        sides = {tuple(cell["index"]): cell["sides"] for cell in
                 grid.snapshot((0, 0, 0), (0, 0))["cells"]}
        self.assertEqual(sides[(0, 0)]["x+"], sides[(1, 0)]["x-"])
        self.assertEqual(sides[(0, 0)]["x+"]["source"], "direct")
        self.assertEqual(sides[(0, 0)]["x+"]["range_mm"], 835)

    def test_direct_wall_replaces_inference_and_survives_later_far_ray(self):
        grid = CellWallGrid(.6, 20)
        grid.observe((3, 0), (-1, 0), 1.4, 1325, 300, 1.0)
        self.assertEqual(grid.state((1, 0), (-1, 0)), "wall")
        grid.observe((1, 0), (-1, 0), .175, 100, 300, 2.0)
        grid.observe((4, 0), (-1, 0), 3.1, 3025, 300, 3.0)
        self.assertEqual(grid.state((1, 0), (-1, 0)), "wall")
        grid.observe((0, 0), (1, 0), .875, 800, 300, 4.0)
        self.assertEqual(grid.state((1, 0), (-1, 0)), "open")

    def test_ray_budget_does_not_invent_a_wall(self):
        grid = CellWallGrid(.6, 3)
        grid.observe((0, 0), (1, 0), 65.0, 64925, 300, 1.0)
        self.assertEqual(grid.state((2, 0), (1, 0)), "open")
        self.assertEqual(grid.state((3, 0), (1, 0)), "unknown")

    def test_wall_grid_round_trip_and_invalid_shared_edge(self):
        settings = load_config()["exploration"]
        grid = CellWallGrid(.6, 20)
        grid.observe((0, 0), (0, 1), .16, 85, 300, 1.0)
        slam = OccupancyGridSLAM(settings)
        slam.update((0, 0, 0), 1480)
        slam.set_exploration_state({"cell_grid": grid.snapshot((0, 0, 0), (0, 0))})
        document = slam.to_dict()
        restored = OccupancyGridSLAM(settings)
        restored.load_dict(document)
        self.assertEqual(restored.to_dict()["exploration"], document["exploration"])
        invalid = copy.deepcopy(document)
        invalid["exploration"]["cell_grid"]["cells"][0]["sides"]["y+"]["state"] = "open"
        with self.assertRaisesRegex(ValueError, "inconsistent"):
            restored.load_dict(invalid)
        self.assertEqual(restored.to_dict()["exploration"], document["exploration"])
        invalid_source = copy.deepcopy(document)
        invalid_source["exploration"]["cell_grid"]["cells"][0]["sides"]["y+"]["source"] = "guess"
        with self.assertRaisesRegex(ValueError, "source"):
            restored.load_dict(invalid_source)


if __name__ == "__main__":
    unittest.main()
