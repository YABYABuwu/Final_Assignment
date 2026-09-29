"""Shortest path planning and multi-target mission planner for RoboMaster EP maze navigation.

Features:
- GridMap abstraction over CellWallGrid / latest.json map data.
- BFS & A* shortest path algorithms with direction-continuity tie-breaking (for omnidirectional sliding).
- Line-of-sight & shooting standpoint calculation (<= 2 tiles distance).
- Optimal multi-target sequencing (TSP solver) for Round 2.
- Waypoint generation for ChassisController.move_to.
- Visualization plot generator for reports and presentations.
"""

from collections import deque
import heapq
import itertools
import json
import math
from pathlib import Path


class GridMap:
    """Graph representation of the maze based on CellWallGrid data."""

    DELTA_TO_SIDE = {
        (1, 0): "x+",
        (-1, 0): "x-",
        (0, 1): "y+",
        (0, -1): "y-",
    }
    SIDE_TO_DELTA = {v: k for k, v in DELTA_TO_SIDE.items()}

    def __init__(self, cell_size_m=0.60, base_pose=None):
        self.cell_size_m = float(cell_size_m)
        self.base_pose = list(base_pose) if base_pose is not None else [0.0, 0.0, 0.0]
        self.cells = {}  # (x, y) -> {"x+": state, "x-": state, "y+": state, "y-": state}
        self.visited = set()
        self.cell_targets = {}  # (x, y) -> [world_x, world_y]
        self.detected_targets = []

    @classmethod
    def from_dict(cls, data):
        """Construct GridMap from latest.json dictionary or exploration dictionary."""
        exploration = data.get("exploration", data)
        cg = exploration.get("cell_grid", {})

        cell_size = cg.get("cell_size_m", 0.60)
        base_pose = cg.get("base_pose", [0.0, 0.0, 0.0])

        grid_map = cls(cell_size_m=cell_size, base_pose=base_pose)

        for cell in cg.get("cells", []):
            idx = tuple(cell["index"])
            sides = {side: info.get("state", "unknown") for side, info in cell.get("sides", {}).items()}
            grid_map.cells[idx] = sides

        grid_map.visited = {tuple(p) for p in exploration.get("visited", [])}

        grid_map.detected_targets = (
            data.get("targets")
            or exploration.get("targets")
            or exploration.get("detected_targets")
            or []
        )

        targets_dict = exploration.get("cell_targets", {})
        for k, v in targets_dict.items():
            try:
                coords = tuple(int(c.strip()) for c in k.split(","))
                grid_map.cell_targets[coords] = list(v)
            except Exception:
                pass

        return grid_map

    @classmethod
    def from_file(cls, filepath):
        """Load GridMap from a JSON file."""
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        return cls.from_dict(data)

    def is_passable(self, u, v):
        """Return True if move from cell u to neighboring cell v is unobstructed ('open')."""
        if u not in self.cells or v not in self.cells:
            return False
        delta = (v[0] - u[0], v[1] - u[1])
        if delta not in self.DELTA_TO_SIDE:
            return False
        side = self.DELTA_TO_SIDE[delta]
        return self.cells[u].get(side) == "open"

    def get_neighbors(self, node):
        """Return all adjacent cells reachable via 'open' borders."""
        neighbors = []
        if node not in self.cells:
            return neighbors
        for delta, side in self.DELTA_TO_SIDE.items():
            if self.cells[node].get(side) == "open":
                nxt = (node[0] + delta[0], node[1] + delta[1])
                if nxt in self.cells:
                    neighbors.append(nxt)
        return neighbors

    def cell_to_world(self, node):
        """Convert grid cell (gx, gy) to world coordinates (X, Y) in metres."""
        if node in self.cell_targets:
            return tuple(self.cell_targets[node])
        step = self.cell_size_m
        u = node[0] * step
        v = node[1] * step
        x0, y0, yaw_deg = self.base_pose
        angle = math.radians(yaw_deg)
        return (
            x0 + u * math.cos(angle) - v * math.sin(angle),
            y0 + u * math.sin(angle) + v * math.cos(angle),
        )

    def has_line_of_sight(self, from_node, to_node, max_distance=2, allow_same_cell=False):
        """Check if from_node has an unobstructed orthogonal line of sight to to_node.

        Both cells must share the same row or column. Distance in grid cells must be
        between 1 and max_distance (or 0 if allow_same_cell is True), and all intermediary borders must be 'open'.
        """
        if from_node == to_node:
            return allow_same_cell

        dx = to_node[0] - from_node[0]
        dy = to_node[1] - from_node[1]

        # Must be orthogonal and within max_distance
        if dx != 0 and dy != 0:
            return False
        dist = abs(dx) + abs(dy)
        if dist > max_distance:
            return False

        step_x = 0 if dx == 0 else (1 if dx > 0 else -1)
        step_y = 0 if dy == 0 else (1 if dy > 0 else -1)
        step_delta = (step_x, step_y)
        side_name = self.DELTA_TO_SIDE[step_delta]

        curr = from_node
        for _ in range(dist):
            if curr not in self.cells or self.cells[curr].get(side_name) != "open":
                return False
            curr = (curr[0] + step_x, curr[1] + step_y)

        return True

    def find_shooting_standpoints(self, target_node, max_distance=2, allow_same_cell=False,
                                  default_same_cell_yaw=0.0, target_side=None):
        """Find valid accessible cells within <= max_distance with line of sight to target.

        If allow_same_cell is True, the target's own cell is also a valid standpoint (distance 0).
        If target_side is specified ('x+', 'x-', 'y+', 'y-'), standpoints must face directly
        towards that target's wall rather than an orthogonal wall.

        Returns list of dicts:
            {
                'standpoint': (x, y),
                'target': (tx, ty),
                'distance_cells': int,
                'gimbal_yaw_deg': float (relative to robot chassis in RoboMaster frame)
            }
        """
        standpoints = []

        # Distance 0: Standing in the same tile
        if allow_same_cell and target_node in self.cells:
            standpoints.append({
                "standpoint": target_node,
                "target": target_node,
                "distance_cells": 0,
                "gimbal_yaw_deg": default_same_cell_yaw,
            })

        # Only look at standpoints that directly face the target's wall
        side_to_cardinal = {
            "x+": [(1, 0)],
            "x-": [(-1, 0)],
            "y+": [(0, 1)],
            "y-": [(0, -1)],
        }
        cardinals = side_to_cardinal.get(target_side, [(1, 0), (-1, 0), (0, 1), (0, -1)])

        for dx, dy in cardinals:
            for dist in range(1, max_distance + 1):
                # Candidate standpoint looking towards target
                sp = (target_node[0] - dx * dist, target_node[1] - dy * dist)
                if sp in self.cells and self.has_line_of_sight(sp, target_node, max_distance=max_distance, allow_same_cell=allow_same_cell):
                    # Direction vector from standpoint to target
                    vec_x = target_node[0] - sp[0]
                    vec_y = target_node[1] - sp[1]
                    # RoboMaster Gimbal is Clockwise Positive: negate atan2 so +y (left) is -90 deg
                    yaw_deg = -math.degrees(math.atan2(vec_y, vec_x))

                    standpoints.append({
                        "standpoint": sp,
                        "target": target_node,
                        "distance_cells": dist,
                        "gimbal_yaw_deg": yaw_deg,  # grid-frame angle; add base_pose[2] for world frame
                    })

        return standpoints


def find_path_bfs(grid_map, start, goal, prefer_straight=True):
    """Find shortest path from start to goal using Breadth-First Search (BFS).

    If prefer_straight is True, breaks ties by preferring to continue in the
    same direction as the previous move, minimizing lateral/forward axis switching.
    """
    if start == goal:
        return [start]
    if start not in grid_map.cells or goal not in grid_map.cells:
        return None

    # Queue item: (current_node, path, last_direction)
    queue = deque([(start, [start], None)])
    visited = {start: 0}

    best_path = None
    best_direction_changes = float("inf")
    min_length = None

    while queue:
        current, path, last_dir = queue.popleft()

        if min_length is not None and len(path) > min_length:
            break

        if current == goal:
            if min_length is None:
                min_length = len(path)
            # Count direction changes
            changes = 0
            for i in range(1, len(path) - 1):
                d1 = (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
                d2 = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
                if d1 != d2:
                    changes += 1

            if changes < best_direction_changes:
                best_direction_changes = changes
                best_path = path

            if not prefer_straight:
                return path
            continue

        for neighbor in grid_map.get_neighbors(current):
            curr_dir = (neighbor[0] - current[0], neighbor[1] - current[1])
            new_cost = len(path)

            if neighbor not in visited or visited[neighbor] >= new_cost:
                visited[neighbor] = new_cost
                # To prefer straight motion during traversal, prioritize same direction
                if prefer_straight and last_dir is not None and curr_dir == last_dir:
                    queue.appendleft((neighbor, path + [neighbor], curr_dir))
                else:
                    queue.append((neighbor, path + [neighbor], curr_dir))

    return best_path


def find_path_astar(grid_map, start, goal, turn_penalty=0.05):
    """Find shortest path using A* Search.

    turn_penalty: slight cost added when changing direction to prefer straight paths.
    """
    if start == goal:
        return [start]
    if start not in grid_map.cells or goal not in grid_map.cells:
        return None

    def heuristic(node):
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    # Priority queue: (f_score, g_score, node, path, last_dir)
    frontier = []
    heapq.heappush(frontier, (heuristic(start), 0, start, [start], None))
    cost_so_far = {(start, None): 0}

    while frontier:
        _, g_score, current, path, last_dir = heapq.heappop(frontier)

        if current == goal:
            return path

        for neighbor in grid_map.get_neighbors(current):
            direction = (neighbor[0] - current[0], neighbor[1] - current[1])
            turn_cost = turn_penalty if (last_dir is not None and direction != last_dir) else 0.0
            new_g = g_score + 1.0 + turn_cost

            state_key = (neighbor, direction)
            if state_key not in cost_so_far or new_g < cost_so_far[state_key]:
                cost_so_far[state_key] = new_g
                f_score = new_g + heuristic(neighbor)
                heapq.heappush(frontier, (f_score, new_g, neighbor, path + [neighbor], direction))

    return None


class MultiTargetPlanner:
    """Plans optimal tour to visit and shoot multiple targets within the maze."""

    def __init__(self, grid_map, max_shooting_dist=2, allow_same_cell=False,
                 same_cell_standoff_m=0.20, path_algorithm="bfs"):
        self.grid_map = grid_map
        self.max_shooting_dist = max_shooting_dist
        self.allow_same_cell = allow_same_cell
        self.same_cell_standoff_m = float(same_cell_standoff_m)
        self.path_algorithm = path_algorithm

    def find_path(self, start, goal):
        if self.path_algorithm == "astar":
            return find_path_astar(self.grid_map, start, goal)
        return find_path_bfs(self.grid_map, start, goal)

    def plan(self, start, targets, return_to_start=False):
        """Plan optimal sequence to shoot all targets.

        Args:
            start: tuple (x, y) start cell.
            targets: list of (tx, ty) tuples or list of dicts with 'pos', 'id', 'color'.
            return_to_start: whether the robot must return to start after shooting.

        Returns:
            dict containing:
                'success': bool,
                'target_order': list of target IDs/indices,
                'shooting_plan': list of action dicts (standpoint, target, gimbal_yaw, distance),
                'full_path': list of cells from start to finish,
                'waypoints': list of (x, y) world coordinates,
                'total_distance_m': float,
                'total_steps': int
        """
        parsed_targets = []
        for i, t in enumerate(targets):
            if isinstance(t, dict):
                pos = tuple(t.get("pos", t.get("index")))
                target_id = t.get("id", f"T{i + 1}")
                color = t.get("color", "red")
                side = t.get("side")
            else:
                pos = tuple(t)
                target_id = f"T{i + 1}"
                color = "red"
                side = None

            # RoboMaster Gimbal convention: 0 is front (x+), 180 is back (x-),
            # -90 is left (y+), +90 is right (y-).
            side_map = {
                "x+": 0.0, "front": 0.0, "forward": 0.0,
                "x-": 180.0, "back": 180.0, "backward": 180.0,
                "y+": -90.0, "left": -90.0,
                "y-": 90.0, "right": 90.0,
            }
            target_yaw = side_map.get(side, 0.0) if side else 0.0
            parsed_targets.append({
                "id": target_id,
                "pos": pos,
                "color": color,
                "shape": t.get("shape", "circle") if isinstance(t, dict) else "circle",
                "side": side,
                "yaw_deg": target_yaw,
                "observation": t.get("observation", {}) if isinstance(t, dict) else {},
            })

        # Collect candidate shooting standpoints for each target
        target_candidates = {}
        for t in parsed_targets:
            standpoints = self.grid_map.find_shooting_standpoints(
                t["pos"], max_distance=self.max_shooting_dist,
                allow_same_cell=self.allow_same_cell,
                default_same_cell_yaw=t["yaw_deg"],
                target_side=t.get("side"),
            )
            if not standpoints:
                # Target not visible from any reachable open cell
                return {
                    "success": False,
                    "error": f"Target {t['id']} at {t['pos']} has no valid shooting standpoint within {self.max_shooting_dist} tiles.",
                }
            target_candidates[t["id"]] = standpoints

        # Cache shortest paths between key nodes to make TSP search fast
        path_cache = {}

        def get_cached_path(p1, p2):
            key = (p1, p2)
            if key not in path_cache:
                path_cache[key] = self.find_path(p1, p2)
            return path_cache[key]

        best_cost = float("inf")
        best_sequence = None
        best_standpoints = None

        # Search optimal permutation of targets
        if len(parsed_targets) <= 7:
            target_permutations = list(itertools.permutations(parsed_targets))
            for perm in target_permutations:
                # For each permutation, enumerate all combinations of standpoints
                # (one per target) and pick the globally optimal combination.
                candidate_lists = [target_candidates[t["id"]] for t in perm]
                best_combo_cost = float("inf")
                best_combo_sp = None

                for combo in itertools.product(*candidate_lists):
                    current_cost = 0
                    current_pos = start
                    valid_combo = True
                    for t, sp_info in zip(perm, combo):
                        path = get_cached_path(current_pos, sp_info["standpoint"])
                        if path is None:
                            valid_combo = False
                            break
                        current_cost += len(path) - 1
                        # Small fractional penalty for shooting distance: prefer standpoints
                        # closer to the target wall (distance 0 < 1 < 2). This breaks ties
                        # so a 1-step walk to a 1-tile standpoint is preferred over staying
                        # at a 2-tile standpoint for free, ensuring the camera can see the target.
                        current_cost += sp_info["distance_cells"] * 0.1
                        current_pos = sp_info["standpoint"]
                    if not valid_combo:
                        continue
                    if return_to_start:
                        return_path = get_cached_path(current_pos, start)
                        if return_path is None:
                            continue
                        current_cost += len(return_path) - 1
                    if current_cost < best_combo_cost:
                        best_combo_cost = current_cost
                        best_combo_sp = [{**t, **sp_info} for t, sp_info in zip(perm, combo)]

                if best_combo_sp is None:
                    continue

                if best_combo_cost < best_cost:
                    best_cost = best_combo_cost
                    best_sequence = perm
                    best_standpoints = best_combo_sp
        else:
            # Scalable greedy nearest-neighbor solver for large target sets (>7 targets)
            for first_idx in range(min(5, len(parsed_targets))):
                remaining = list(parsed_targets)
                curr_target = remaining.pop(first_idx)
                # Pick best initial standpoint from start
                best_init_sp = None
                best_init_cost = float("inf")
                for sp_info in target_candidates[curr_target["id"]]:
                    p = get_cached_path(start, sp_info["standpoint"])
                    if p is not None and (len(p) - 1) < best_init_cost:
                        best_init_cost = len(p) - 1
                        best_init_sp = sp_info

                if best_init_sp is None:
                    continue

                current_cost = best_init_cost
                current_pos = best_init_sp["standpoint"]
                selected_sp = [{**curr_target, **best_init_sp}]
                valid_tour = True

                while remaining:
                    best_next = None
                    best_next_sp = None
                    best_next_cost = float("inf")
                    for cand in remaining:
                        for sp_info in target_candidates[cand["id"]]:
                            p = get_cached_path(current_pos, sp_info["standpoint"])
                            if p is not None:
                                cost = (len(p) - 1) + sp_info["distance_cells"] * 0.1
                                if cost < best_next_cost:
                                    best_next_cost = cost
                                    best_next = cand
                                    best_next_sp = sp_info
                    if best_next is None or best_next_sp is None:
                        valid_tour = False
                        break
                    remaining.remove(best_next)
                    current_cost += best_next_cost
                    current_pos = best_next_sp["standpoint"]
                    selected_sp.append({**best_next, **best_next_sp})

                if not valid_tour:
                    continue

                if return_to_start:
                    ret_path = get_cached_path(current_pos, start)
                    if ret_path is None:
                        continue
                    current_cost += len(ret_path) - 1

                if current_cost < best_cost:
                    best_cost = current_cost
                    best_sequence = tuple(selected_sp)
                    best_standpoints = selected_sp

        if best_sequence is None:
            return {"success": False, "error": "No valid route found connecting all targets."}

        # Construct full path and shooting actions
        full_path = [start]
        current_pos = start
        shooting_actions = []

        for item in best_standpoints:
            sp = item["standpoint"]
            sub_path = get_cached_path(current_pos, sp)
            if sub_path and len(sub_path) > 1:
                full_path.extend(sub_path[1:])

            shooting_actions.append({
                "target_id": item["id"],
                "target_pos": item["pos"],
                "target_color": item.get("color", "red"),
                "target_shape": item.get("shape", "circle"),
                "side": item.get("side"),
                "standpoint": sp,
                "gimbal_yaw_deg": item["gimbal_yaw_deg"],
                "distance_cells": item["distance_cells"],
                "path_step_index": len(full_path) - 1,
                "target_pitch_deg": item.get("observation", {}).get("gimbal_pitch_deg", 0.0),
            })
            current_pos = sp

        if return_to_start:
            return_path = get_cached_path(current_pos, start)
            if return_path and len(return_path) > 1:
                full_path.extend(return_path[1:])

        # Generate waypoints in metric world coordinates with standoff adjustment for same-cell targets
        waypoints = []
        for i, cell in enumerate(full_path):
            base_world = self.grid_map.cell_to_world(cell)
            action = next((a for a in shooting_actions if a["path_step_index"] == i and a["distance_cells"] == 0), None)
            # Only apply forward standoff if the target is directly in front along the approach path
            if action is not None and i > 0 and self.same_cell_standoff_m > 0 and abs(action.get("gimbal_yaw_deg", 0.0)) <= 45:
                prev_cell = full_path[i - 1]
                dx = cell[0] - prev_cell[0]
                dy = cell[1] - prev_cell[1]
                dist = math.hypot(dx, dy)
                if dist > 0:
                    ux, uy = dx / dist, dy / dist
                    yaw_rad = math.radians(self.grid_map.base_pose[2])
                    world_ux = ux * math.cos(yaw_rad) - uy * math.sin(yaw_rad)
                    world_uy = ux * math.sin(yaw_rad) + uy * math.cos(yaw_rad)
                    adjusted_pt = (
                        base_world[0] - world_ux * self.same_cell_standoff_m,
                        base_world[1] - world_uy * self.same_cell_standoff_m,
                    )
                    waypoints.append(adjusted_pt)
                    action["standpoint_world"] = adjusted_pt
                    continue
            waypoints.append(base_world)

        for a in shooting_actions:
            if "standpoint_world" not in a:
                a["standpoint_world"] = waypoints[a["path_step_index"]]

        total_steps = len(full_path) - 1
        total_dist_m = total_steps * self.grid_map.cell_size_m

        return {
            "success": True,
            "target_order": [t["id"] if isinstance(t, dict) else t for t in best_sequence],
            "shooting_plan": shooting_actions,
            "full_path": full_path,
            "waypoints": waypoints,
            "total_steps": total_steps,
            "total_distance_m": total_dist_m,
        }


def plot_mission_map(grid_map, plan, output_path=None, title="Round 2 Shortest Path & Target Mission"):
    """Render a visual map with maze walls, planned path, shooting standpoints, and targets."""
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle, FancyArrowPatch

    fig, ax = plt.subplots(figsize=(10, 10), dpi=150)
    ax.set_aspect("equal")

    # Determine bounds from grid cells
    # Oriented so Forward (+X) is UP and Lateral (+Y) is RIGHT, matching exploration map
    xs = [c[0] for c in grid_map.cells]
    ys = [c[1] for c in grid_map.cells]
    min_x, max_x = min(xs, default=0), max(xs, default=5)
    min_y, max_y = min(ys, default=0), max(ys, default=5)

    # Set grid view margins: Horizontal is Y, Vertical is X
    margin = 1
    ax.set_xlim(min_y - margin, max_y + margin + 1)
    ax.set_ylim(min_x - margin, max_x + margin + 1)

    # Draw cell tiles (plot_x = gy, plot_y = gx)
    for (gx, gy), sides in grid_map.cells.items():
        is_visited = (gx, gy) in grid_map.visited
        bg_color = "#e8f4f8" if is_visited else "#f5f5f5"
        rect = Rectangle((gy, gx), 1, 1, facecolor=bg_color, edgecolor="#e0e0e0", linewidth=0.5)
        ax.add_patch(rect)
        ax.text(gy + 0.1, gx + 0.1, f"({gx},{gy})", fontsize=7, color="#888888")

        # Draw walls:
        # x+: top edge (gx+1) from (gy, gx+1) to (gy+1, gx+1) - Forward wall
        # x-: bottom edge (gx) from (gy, gx) to (gy+1, gx) - Back wall
        # y+: right edge (gy+1) from (gy+1, gx) to (gy+1, gx+1) - Right wall
        # y-: left edge (gy) from (gy, gx) to (gy, gx+1) - Left wall
        wall_color = "#111111"
        wall_width = 3.0
        open_color = "#70c070"
        open_width = 1.0

        borders = [
            ("x+", [(gy, gx + 1), (gy + 1, gx + 1)]),
            ("x-", [(gy, gx), (gy + 1, gx)]),
            ("y+", [(gy + 1, gx), (gy + 1, gx + 1)]),
            ("y-", [(gy, gx), (gy, gx + 1)]),
        ]

        for side_name, line_pts in borders:
            state = sides.get(side_name)
            if state == "wall":
                ax.plot([line_pts[0][0], line_pts[1][0]], [line_pts[0][1], line_pts[1][1]],
                        color=wall_color, linewidth=wall_width, solid_capstyle="round")
            elif state == "open":
                ax.plot([line_pts[0][0], line_pts[1][0]], [line_pts[0][1], line_pts[1][1]],
                        color=open_color, linewidth=open_width, linestyle=":")

    # Draw path (plot_x = gy + 0.5, plot_y = gx + 0.5)
    if plan and plan.get("success") and plan.get("full_path"):
        full_path = plan["full_path"]
        path_xs = [c[1] + 0.5 for c in full_path]
        path_ys = [c[0] + 0.5 for c in full_path]

        ax.plot(path_xs, path_ys, color="#0066cc", linewidth=2.5, linestyle="-", label="Planned Path", zorder=3)

        # Draw arrows for each step
        for i in range(len(full_path) - 1):
            x1, y1 = path_xs[i], path_ys[i]
            x2, y2 = path_xs[i + 1], path_ys[i + 1]
            arrow = FancyArrowPatch((x1, y1), (x2, y2),
                                    arrowstyle="-|>", mutation_scale=12,
                                    color="#004499", zorder=4)
            ax.add_patch(arrow)

        # Mark Start
        start = full_path[0]
        ax.plot(start[1] + 0.5, start[0] + 0.5, marker="o", markersize=14, color="#00aa00", label="Start", zorder=5)
        ax.text(start[1] + 0.5, start[0] + 0.5, "START", color="white", fontsize=7,
                ha="center", va="center", weight="bold", zorder=6)

        # Track targets per (cell, side) to apply small offset if multiple
        targets_at_wall = {}
        for sp_info in plan.get("shooting_plan", []):
            tp_key = tuple(sp_info["target_pos"])
            side_key = sp_info.get("side", "none")
            key = (tp_key, side_key)
            targets_at_wall[key] = targets_at_wall.get(key, 0) + 1

        placed_at_wall = {}
        color_map = {
            "red": "#e60000",
            "blue": "#0066ff",
            "yellow": "#e6b800",
            "green": "#00aa33",
        }

        # Mark Shooting Standpoints and Aim Lines
        for sp_info in plan.get("shooting_plan", []):
            sp = sp_info["standpoint"]
            tp = tuple(sp_info["target_pos"])
            tid = sp_info["target_id"]
            dist = sp_info["distance_cells"]
            side = sp_info.get("side")
            shape = sp_info.get("target_shape", "circle")
            target_color = color_map.get(sp_info.get("target_color", "red"), "#e60000")

            # Standpoint marker in (plot_x=gy, plot_y=gx)
            sp_plot_x, sp_plot_y = sp[1] + 0.5, sp[0] + 0.5
            ax.plot(sp_plot_x, sp_plot_y, marker="s", markersize=12, color="#ff9900", zorder=5)
            ax.text(sp_plot_x, sp_plot_y, "FIRE", color="black", fontsize=6,
                    ha="center", va="center", weight="bold", zorder=6)

            # Determine wall coordinate for the target (plot_x=gy, plot_y=gx)
            c_plot_x, c_plot_y = tp[1] + 0.5, tp[0] + 0.5
            wall_dist = 0.38
            key = (tp, side)
            idx = placed_at_wall.get(key, 0)
            placed_at_wall[key] = idx + 1
            total_here = targets_at_wall.get(key, 1)
            # Offset along the wall if multiple targets on same wall
            jitter = (idx - (total_here - 1) / 2.0) * 0.22 if total_here > 1 else 0.0

            if side == "x+":
                # Front wall (Top)
                tx, ty = c_plot_x + jitter, c_plot_y + wall_dist
            elif side == "x-":
                # Back wall (Bottom)
                tx, ty = c_plot_x + jitter, c_plot_y - wall_dist
            elif side == "y+":
                # Left wall
                tx, ty = c_plot_x - wall_dist, c_plot_y + jitter
            elif side == "y-":
                # Right wall
                tx, ty = c_plot_x + wall_dist, c_plot_y + jitter
            else:
                tx, ty = c_plot_x, c_plot_y + jitter

            # Shooting ray / line of sight (dashed line with arrow)
            ax.annotate("",
                        xy=(tx, ty),
                        xytext=(sp_plot_x, sp_plot_y),
                        arrowprops=dict(arrowstyle="->", color=target_color, lw=1.8, ls="--"),
                        zorder=7)

            # Draw target with actual shape and color
            if shape == "circle":
                patch = Circle((tx, ty), radius=0.10, facecolor=target_color, edgecolor="#111111", linewidth=1.5, zorder=8)
                ax.add_patch(patch)
            elif shape == "square":
                patch = Rectangle((tx - 0.10, ty - 0.10), 0.20, 0.20, facecolor=target_color, edgecolor="#111111", linewidth=1.5, zorder=8)
                ax.add_patch(patch)
            elif shape == "vertical":
                patch = Rectangle((tx - 0.06, ty - 0.13), 0.12, 0.26, facecolor=target_color, edgecolor="#111111", linewidth=1.5, zorder=8)
                ax.add_patch(patch)
            elif shape == "horizontal":
                patch = Rectangle((tx - 0.13, ty - 0.06), 0.26, 0.12, facecolor=target_color, edgecolor="#111111", linewidth=1.5, zorder=8)
                ax.add_patch(patch)
            else:
                ax.plot(tx, ty, marker="o", markersize=14, color=target_color, markeredgecolor="#111111", zorder=8)

            label_color = "white" if target_color in ("#e60000", "#0066ff", "#00aa33") else "black"
            ax.text(tx, ty, f"{tid}", color=label_color, fontsize=6.5,
                    ha="center", va="center", weight="bold", zorder=9)
            # Label with shape & distance outside the wall
            label_offset_y = 0.18 if (side == "x+" or (side not in ("x-", "x+") and ty >= c_plot_y)) else -0.18
            ax.text(tx, ty + label_offset_y, f"{shape} ({dist}t)", color="#333333",
                    fontsize=6, ha="center", va="center", zorder=9)

    ax.set_title(title, fontsize=14, fontweight="bold")
    ax.set_xlabel("Y (Lateral / Side Tiles)")
    ax.set_ylabel("X (Forward / Heading Tiles) ▲ FORWARD")
    ax.grid(False)

    summary_text = (
        f"Total Steps: {plan.get('total_steps', 0)} tiles\n"
        f"Distance: {plan.get('total_distance_m', 0.0):.2f} m\n"
        f"Targets: {len(plan.get('shooting_plan', []))}"
    )
    plt.gcf().text(0.15, 0.02, summary_text, fontsize=10,
                   bbox=dict(boxstyle="round,pad=0.5", facecolor="#f0f0f0", edgecolor="#cccccc"))

    if output_path:
        plt.savefig(output_path, bbox_inches="tight")
        plt.close()
    else:
        plt.show()


if __name__ == "__main__":
    import argparse
    import ast

    parser = argparse.ArgumentParser(description="Shortest path and target mission planner.")
    parser.add_argument("--map", default="Final_Assignment/data/maps/latest.json", help="Path to map JSON file")
    parser.add_argument("--start", default="(0,0)", help="Start cell tuple e.g. (0,0)")
    parser.add_argument("--targets", default="[(3,2), (2,1)]", help="List of target cells e.g. '[(3,2), (2,1)]'")
    parser.add_argument("--algorithm", default="bfs", choices=["bfs", "astar"], help="Path algorithm (bfs/astar)")
    parser.add_argument("--max-distance", type=int, default=2, help="Max shooting distance in tiles (<= 2)")
    parser.add_argument("--allow-same-cell", action="store_true", help="Allow shooting from the same tile as the target (distance 0)")
    parser.add_argument("--return-to-start", action="store_true", help="Return to start cell after shooting")
    parser.add_argument("--output", default="round2_plan.png", help="Output PNG path for path visualization")

    args = parser.parse_args()

    start_cell = ast.literal_eval(args.start)
    target_cells = ast.literal_eval(args.targets)

    print(f"Loading map: {args.map}")
    gmap = GridMap.from_file(args.map)
    print(f"Total explored cells: {len(gmap.cells)}")

    planner = MultiTargetPlanner(gmap, max_shooting_dist=args.max_distance, allow_same_cell=args.allow_same_cell, path_algorithm=args.algorithm)
    plan = planner.plan(start_cell, target_cells, return_to_start=args.return_to_start)

    if not plan["success"]:
        print(f"Failed to plan mission: {plan['error']}")
    else:
        print("\n--- Mission Plan Found ---")
        print(f"Target Visit Order: {plan['target_order']}")
        print(f"Total Steps: {plan['total_steps']} tiles ({plan['total_distance_m']:.2f} m)")
        print(f"Full Path: {plan['full_path']}")
        print("\nShooting Actions:")
        for action in plan["shooting_plan"]:
            print(f" - At Standpoint {action['standpoint']}: Aim Gimbal to {action['gimbal_yaw_deg']:.1f} deg "
                  f"to shoot Target {action['target_id']} at {action['target_pos']} ({action['distance_cells']} tile(s) away)")

        print(f"\nWaypoints for ChassisController:")
        for i, wp in enumerate(plan["waypoints"]):
            print(f"  Step {i:02d}: x={wp[0]:.3f} m, y={wp[1]:.3f} m")

        if args.output:
            plot_mission_map(gmap, plan, output_path=args.output)
            print(f"\nSaved mission visualization to: {args.output}")
