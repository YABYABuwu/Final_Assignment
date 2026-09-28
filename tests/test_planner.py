import json
import unittest
from pathlib import Path

from src.planner import GridMap, find_path_bfs, find_path_astar, MultiTargetPlanner


class PlannerTests(unittest.TestCase):
    def setUp(self):
        # Build a synthetic 3x3 grid map
        # (0,0) -open- (1,0) -open- (2,0)
        #   |            |            |
        #  wall         open         wall
        #   |            |            |
        # (0,1) -wall- (1,1) -open- (2,1)
        self.gmap = GridMap(cell_size_m=0.6, base_pose=[0.0, 0.0, 0.0])
        self.gmap.cells = {
            (0, 0): {"x+": "open", "x-": "wall", "y+": "wall", "y-": "wall"},
            (1, 0): {"x+": "open", "x-": "open", "y+": "open", "y-": "wall"},
            (2, 0): {"x+": "wall", "x-": "open", "y+": "wall", "y-": "wall"},
            (0, 1): {"x+": "wall", "x-": "wall", "y+": "wall", "y-": "wall"},
            (1, 1): {"x+": "open", "x-": "wall", "y+": "wall", "y-": "open"},
            (2, 1): {"x+": "wall", "x-": "open", "y+": "wall", "y-": "wall"},
        }
        self.gmap.visited = {(0, 0), (1, 0), (2, 0), (1, 1), (2, 1)}

    def test_line_of_sight_direct_neighbor(self):
        # (0,0) -> (1,0) is open
        self.assertTrue(self.gmap.has_line_of_sight((0, 0), (1, 0), max_distance=2))
        # (0,0) -> (0,1) is wall
        self.assertFalse(self.gmap.has_line_of_sight((0, 0), (0, 1), max_distance=2))

    def test_line_of_sight_two_tiles(self):
        # (0,0) -> (2,0) via open edges
        self.assertTrue(self.gmap.has_line_of_sight((0, 0), (2, 0), max_distance=2))
        # Exceeds max distance
        self.assertFalse(self.gmap.has_line_of_sight((0, 0), (2, 0), max_distance=1))
        # Blocked path: (0,1) -> (2,1) wall between 0,1 and 1,1
        self.assertFalse(self.gmap.has_line_of_sight((0, 1), (2, 1), max_distance=2))

    def test_bfs_shortest_path(self):
        # From (0,0) to (2,1)
        # Valid path: (0,0) -> (1,0) -> (1,1) -> (2,1) (3 steps)
        path = find_path_bfs(self.gmap, (0, 0), (2, 1))
        self.assertEqual(path, [(0, 0), (1, 0), (1, 1), (2, 1)])

    def test_astar_shortest_path(self):
        path = find_path_astar(self.gmap, (0, 0), (2, 1))
        self.assertEqual(path, [(0, 0), (1, 0), (1, 1), (2, 1)])

    def test_shooting_standpoints(self):
        # Target at (2,1). Standpoints could be (1,1) (dist 1)
        standpoints = self.gmap.find_shooting_standpoints((2, 1), max_distance=2)
        sp_nodes = [s["standpoint"] for s in standpoints]
        self.assertIn((1, 1), sp_nodes)

        # Check gimbal angle: from (1,1) looking east at (2,1) is 0 deg
        for s in standpoints:
            if s["standpoint"] == (1, 1):
                self.assertEqual(s["gimbal_yaw_deg"], 0.0)

    def test_shooting_standpoints_same_cell(self):
        # Target at (2,1). With allow_same_cell=True, (2,1) itself should be valid
        standpoints = self.gmap.find_shooting_standpoints((2, 1), max_distance=2, allow_same_cell=True)
        sp_nodes = [s["standpoint"] for s in standpoints]
        self.assertIn((2, 1), sp_nodes)
        same_cell_sp = next(s for s in standpoints if s["standpoint"] == (2, 1))
        self.assertEqual(same_cell_sp["distance_cells"], 0)

    def test_multi_target_planner(self):
        targets = [(2, 0), (2, 1)]
        planner = MultiTargetPlanner(self.gmap, max_shooting_dist=2, path_algorithm="bfs")
        plan = planner.plan((0, 0), targets, return_to_start=False)

        self.assertTrue(plan["success"])
        self.assertEqual(len(plan["shooting_plan"]), 2)
        self.assertLessEqual(plan["total_steps"], 4)

    def test_planner_with_real_map(self):
        map_path = Path(__file__).resolve().parent.parent / "data" / "maps" / "latest.json"
        if not map_path.exists():
            self.skipTest("latest.json not found")

        gmap = GridMap.from_file(str(map_path))
        target_cells = [(3, 2), (2, 1)]
        available = [c for c in target_cells if c in gmap.cells]
        if len(available) < 2:
            self.skipTest(
                f"target cells {target_cells} not all in map (only {sorted(gmap.cells.keys())})"
            )

        planner = MultiTargetPlanner(gmap, max_shooting_dist=2)
        plan = planner.plan((0, 0), available, return_to_start=False)

        self.assertTrue(plan["success"], msg=plan.get("error"))
        self.assertEqual(len(plan["shooting_plan"]), len(available))
        self.assertGreater(plan["total_steps"], 0)
        self.assertAlmostEqual(
            plan["total_distance_m"],
            plan["total_steps"] * gmap.cell_size_m, places=5
        )


if __name__ == "__main__":
    unittest.main()
