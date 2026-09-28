"""Round 2 infrared execution script for RoboMaster EP Assignment 2.

Navigates to shoot all targets in the shortest path/time (< 5 mins)
using the map generated in Round 1. Moves by sliding (hold heading)
and aims the gimbal at each target from <= 2 tiles away.

Usage:
  python run_round2.py --dry-run --targets "[(3,2), (2,1)]"
  python run_round2.py --targets "[(3,2), (2,1)]"
"""

import argparse
import ast
import json
from pathlib import Path
import time

from src.config_loader import load_config
from src.planner import GridMap, MultiTargetPlanner, plot_mission_map, find_path_bfs, find_path_astar
from src.mission_stop import MissionStop


def parse_targets_arg(target_str):
    try:
        targets = ast.literal_eval(target_str)
        if isinstance(targets, list):
            return targets
        raise ValueError
    except Exception as e:
        raise argparse.ArgumentTypeError(f"Invalid targets format: {target_str}. Example: '[(3,2), (2,1)]'") from e


def main():
    parser = argparse.ArgumentParser(description="Round 2 Shortest Path Target Shooting Runner")
    parser.add_argument("--map", default="data/maps/latest.json", help="Path to map JSON generated from Round 1")
    parser.add_argument("--start", default="(0,0)", help="Starting cell coordinate, e.g. '(0,0)'")
    parser.add_argument("--targets", default=None, type=parse_targets_arg,
                        help="List of target cell coordinates, e.g. '[(3,2), (2,1)]'. If omitted, automatically loaded from Round 1.")
    parser.add_argument("--targets-file", default="data/targets.json", help="Path to dedicated targets JSON file if separate from map")
    parser.add_argument("--max-distance", type=int, default=2, help="Max shooting distance in tiles (<= 2)")
    parser.add_argument("--allow-same-cell", action="store_true", default=True,
                        help="Allow shooting from the same tile as the target (distance 0, default True)")
    parser.add_argument("--no-allow-same-cell", action="store_false", dest="allow_same_cell",
                        help="Disallow shooting from the same tile as the target")
    parser.add_argument("--standoff", type=float, default=0.20, help="Standoff distance in meters from cell center when shooting in same cell (accounts for 20cm gimbal length)")
    parser.add_argument("--speed", type=float, default=0.25,
                        help="Movement speed in m/s (default 0.25 m/s, slightly faster than exploration 0.20 m/s)")
    parser.add_argument("--return-to-start", action="store_true", help="Return to starting cell after shooting")
    parser.add_argument("--algorithm", default="bfs", choices=["bfs", "astar"], help="Search algorithm")
    parser.add_argument("--no-skip-unreachable", action="store_false", dest="skip_unreachable", default=True,
                        help="Do not skip unreachable targets; fail planning instead")
    parser.add_argument("--output", default="data/maps/round2_plan.png", help="Output PNG path for map and route")
    parser.add_argument("--dry-run", action="store_true", help="Plan and visualize without connecting to physical robot")
    args = parser.parse_args()

    project_dir = Path(__file__).resolve().parent
    map_file = project_dir / args.map
    if not map_file.exists():
        print(f"Error: Map file not found at {map_file}. Please ensure Round 1 exploration has completed.")
        return

    start_cell = ast.literal_eval(args.start)
    print(f"=== Round 2: Fast Target Mission ===")
    print(f"Loading map: {map_file}")
    grid_map = GridMap.from_file(map_file)
    print(f"Loaded {len(grid_map.cells)} cells from map.")
    print(f"Start Cell: {start_cell}")
    start_cell = tuple(start_cell) if not isinstance(start_cell, tuple) else start_cell
    if start_cell not in grid_map.cells:
        print(f"Error: start cell {start_cell} is not in the loaded map. Available cells: {sorted(grid_map.cells.keys())[:10]}...")
        return

    # Resolve targets (CLI argument > marker file > targets embedded in the map)
    target_list = args.targets
    if not target_list:
        targets_file = project_dir / args.targets_file
        if targets_file.exists():
            try:
                with open(targets_file, "r", encoding="utf-8") as f:
                    file_data = json.load(f)
                    if isinstance(file_data, list):
                        target_list = file_data
                    elif isinstance(file_data, dict):
                        target_list = file_data.get("targets", [])
                    if target_list:
                        print(f"Auto-loaded targets from {args.targets_file}.")
            except Exception as e:
                print(f"Warning: Failed to load targets file {targets_file}: {e}")
        if not target_list and grid_map.detected_targets:
            print("Auto-loaded targets from map file.")
            target_list = grid_map.detected_targets

    if not target_list:
        print("Error: No targets specified via --targets and no detected targets found in map or data/targets.json.")
        print("Usage example: python run_round2.py --targets \"[(3,2), (2,1)]\"")
        return

    # Filter out hostages / non-targets and unreachable targets
    valid_targets = []
    path_func = find_path_astar if args.algorithm == "astar" else find_path_bfs
    for item in target_list:
        if isinstance(item, dict):
            # Check if flagged as hostage or should_shoot is False
            if item.get("type") == "hostage" or item.get("should_shoot") is False:
                print(f"  [Skip] Non-target or Hostage ignored: {item.get('id', item)}")
                continue

            target_pos = tuple(item.get("pos", item.get("index", ())))
            if args.skip_unreachable:
                side = item.get("side")
                side_map = {
                    "x+": 0.0, "front": 0.0, "forward": 0.0,
                    "x-": 180.0, "back": 180.0, "backward": 180.0,
                    "y+": 90.0, "left": 90.0,
                    "y-": -90.0, "right": -90.0,
                }
                yaw = side_map.get(side, 0.0) if side else 0.0
                standpoints = grid_map.find_shooting_standpoints(
                    target_pos, max_distance=args.max_distance,
                    allow_same_cell=args.allow_same_cell,
                    default_same_cell_yaw=yaw
                )
                reachable_sp = [sp for sp in standpoints if path_func(grid_map, start_cell, sp["standpoint"]) is not None]
                if not reachable_sp:
                    print(f"  [Skip] Unreachable target: {item.get('id', item)} at {target_pos} (no line-of-sight standpoint reachable on map)")
                    continue
            valid_targets.append(item)
        else:
            target_pos = tuple(item)
            if args.skip_unreachable:
                standpoints = grid_map.find_shooting_standpoints(
                    target_pos, max_distance=args.max_distance,
                    allow_same_cell=args.allow_same_cell
                )
                reachable_sp = [sp for sp in standpoints if path_func(grid_map, start_cell, sp["standpoint"]) is not None]
                if not reachable_sp:
                    print(f"  [Skip] Unreachable target at {target_pos} (no line-of-sight standpoint reachable on map)")
                    continue
            valid_targets.append(item)

    if not valid_targets:
        print("Error: All found targets are flagged as hostages/non-targets or are unreachable on the current map.")
        return

    # Deduplicate targets discovered multiple times at the same cell (keeping latest observation)
    unique_targets = []
    seen_target_keys = set()
    for item in reversed(valid_targets):
        if isinstance(item, dict):
            key = (tuple(item.get("pos", ())), item.get("color"), item.get("shape"), item.get("side"))
            if key in seen_target_keys:
                continue
            seen_target_keys.add(key)
            unique_targets.append(item)
        else:
            unique_targets.append(item)
    unique_targets.reverse()
    valid_targets = unique_targets

    print(f"Active Targets to Shoot ({len(valid_targets)}): {[t.get('id', t) if isinstance(t, dict) else t for t in valid_targets]}")

    planner = MultiTargetPlanner(grid_map, max_shooting_dist=args.max_distance,
                                 allow_same_cell=args.allow_same_cell,
                                 same_cell_standoff_m=args.standoff,
                                 path_algorithm=args.algorithm)
    plan = planner.plan(start_cell, valid_targets, return_to_start=args.return_to_start)

    if not plan["success"]:
        print(f"Failed to plan mission: {plan['error']}")
        return

    print(f"\nOptimal Route Planned:")
    print(f"  Target Visit Order: {plan['target_order']}")
    print(f"  Total Steps: {plan['total_steps']} tiles ({plan['total_distance_m']:.2f} m)")
    print(f"  Cell Path: {' -> '.join(str(c) for c in plan['full_path'])}")
    print("\nPlanned Shooting Actions:")
    for action in plan["shooting_plan"]:
        print(f"  - Stand at {action['standpoint']} -> Aim Gimbal {action['gimbal_yaw_deg']:.1f} deg -> "
              f"Shoot Target {action['target_id']} at {action['target_pos']} ({action['distance_cells']} tile(s) away)")

    output_path = project_dir / args.output
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plot_mission_map(grid_map, plan, output_path=str(output_path))
    print(f"\nMission path visualization saved to: {output_path}")

    if args.dry_run:
        print("\n[Dry Run Completed] Robot connection skipped.")
        return

    # Physical Robot Execution
    print("\nConnecting to RoboMaster EP...")
    config = load_config(project_dir / "config" / "settings.yaml")
    log_settings = config["logging"].copy()
    log_settings["directory"] = project_dir / log_settings["directory"]
    if "exploration" in config and "position_coordinate_system" in config["exploration"]:
        log_settings["position_cs"] = config["exploration"]["position_coordinate_system"]

    from robomaster import blaster, conn, robot
    from src.chassis import ChassisController
    from src.gimbal_control import ChassisRelativeGimbal
    from src.logger import SensorLogger
    from src.rear_ir import RearIRBumper, FrontIRBumper
    from src.target_marker import fire_infrared

    def sdk_conn_type(name):
        return {
            "ap": conn.CONNECTION_WIFI_AP,
            "sta": conn.CONNECTION_WIFI_STA,
            "rndis": conn.CONNECTION_USB_RNDIS,
        }[name]

    ep_robot = robot.Robot()
    logger = None
    chassis = None
    dashboard = None
    connected = False

    try:
        ep_robot.initialize(conn_type=sdk_conn_type(config["connection"]["type"]))
        connected = True
        logger = SensorLogger(ep_robot, log_settings)
        logger.start()

        motion_settings = config["motion"].copy()
        motion_settings["max_speed_m_s"] = args.speed
        chassis = ChassisController(ep_robot, logger, motion_settings)
        if config["rear_ir"]["enabled"]:
            chassis.rear_ir = RearIRBumper(logger, config["rear_ir"])
        if config["front_ir"]["enabled"]:
            chassis.front_ir = FrontIRBumper(logger, config["front_ir"])

        gimbal = ChassisRelativeGimbal(ep_robot.gimbal)
        _recenter_action = gimbal.recenter()
        if _recenter_action is not None:
            _recenter_action.wait_for_completed(timeout=5.0)

        print("Waiting for telemetry sensors...")
        logger.wait_for("position", 5.0)
        logger.wait_for("attitude", 5.0)
        logger.wait_for("gimbal", 5.0)
        logger.wait_for("tof", 5.0)
        logger.wait_for("status", 5.0)

        # Open camera and dashboard for visual target lock
        from src.dashboard import Dashboard
        from src.target_inspection import WallTargetInspector
        import math as _math

        dashboard = Dashboard(
            ep_robot, logger, config["dashboard"],
            target_settings=config["exploration"]["target_inspection"],
        )
        dashboard.start()
        host = config["dashboard"]["host"]
        port = config["dashboard"]["port"]
        print(f"Dashboard: http://{host}:{port}")

        # Index shooting actions by the waypoint step index (supporting multiple targets per standpoint)
        actions_by_step = {}
        for act in plan["shooting_plan"]:
            actions_by_step.setdefault(act["path_step_index"], []).append(act)

        # Minimal slam_worker stub: round2 does not do SLAM; inspector only needs pause/resume and error check
        class _NoOpSlamWorker:
            abort_event = __import__("threading").Event()
            def status(self): return {"error": None, "waiting_telemetry": False, "tof_waiting": False}
            def pause_mapping(self): pass
            def resume_mapping(self): pass

        slam_stub = _NoOpSlamWorker()
        fire_type_val = blaster.INFRARED_FIRE
        inspector = WallTargetInspector(
            ep_robot.gimbal,
            ep_robot.blaster,
            dashboard,
            logger,
            chassis,
            slam_stub,
            config["exploration"],
            fire_type_val,
            on_progress=lambda *args: print(f"  [inspection] {args}"),
            scan_gimbal=gimbal,
        )

        start_time = time.time()
        print("\n=== Executing Round 2 Mission ===")

        start_map_x, start_map_y = grid_map.cell_to_world(start_cell)
        init_pose = chassis.get_pose()
        robot_origin_x = float(init_pose[0]) if init_pose else 0.0
        robot_origin_y = float(init_pose[1]) if init_pose else 0.0

        body_yaw = 0.0  # heading held throughout (yaw=None in move_to latches to initial heading)
        for step_idx, cell in enumerate(plan["full_path"]):
            world_x, world_y = plan["waypoints"][step_idx]
            # Map waypoints relative to the starting cell position on the floor
            target_x = robot_origin_x + (world_x - start_map_x)
            target_y = robot_origin_y + (world_y - start_map_y)
            print(f"\nStep {step_idx + 1}/{len(plan['full_path'])}: Moving to {cell} (x={target_x:.2f} m, y={target_y:.2f} m)")

            curr_pose = chassis.get_pose()
            dist_to_target = _math.hypot(curr_pose[0] - target_x, curr_pose[1] - target_y) if curr_pose else 999.0
            if step_idx == 0 and dist_to_target < 0.10:
                print(f"  -> Already at start standpoint {cell}; skipping initial motion.")
                pose = curr_pose
            else:
                pose = chassis.move_to(target_x, target_y, yaw=None)
            if pose is not None and len(pose) >= 3:
                body_yaw = float(pose[2])

            if step_idx in actions_by_step:
                for action in actions_by_step[step_idx]:
                    target_id = action["target_id"]
                    target_pos = action["target_pos"]
                    target_color = action.get("target_color", "")
                    target_shape = action.get("target_shape", "")
                    target_pitch = config["exploration"]["target_inspection"].get("pitch_deg", -20.0)
                    grid_yaw = action["gimbal_yaw_deg"]  # degrees, grid frame
                    base_yaw = grid_map.base_pose[2]     # map rotation in world frame
                    world_yaw = _math.fmod(grid_yaw + base_yaw + 540.0, 360.0) - 180.0
                    sp = action["standpoint"]
                    direction_delta = (target_pos[0] - sp[0], target_pos[1] - sp[1])
                    if direction_delta == (0, 0):
                        side = action.get("side", "y+")
                        side_deltas = {"x+": (1, 0), "x-": (-1, 0), "y+": (0, -1), "y-": (0, 1)}
                        direction_delta = side_deltas.get(side, (0, -1))

                    print(f"  -> Standpoint for {target_id} ({target_color} {target_shape}) at {target_pos}, world_yaw={world_yaw:.1f} deg, pitch={target_pitch:.1f} deg")
                    result = inspector.inspect(
                        cell=sp,
                        delta=direction_delta,
                        world_yaw=world_yaw,
                        body_yaw=body_yaw,
                        initial_pitch=target_pitch,
                    )
                    print(f"  -> Inspection result for {target_id}: {result.get('status')} targets={result.get('targets')}")

        elapsed = time.time() - start_time
        print(f"\n=== Round 2 Complete! Elapsed Time: {elapsed:.1f} s ({elapsed/60:.2f} min) ===")
        if elapsed <= 300:
            print("Successfully completed within the 5-minute requirement!")
        else:
            print("Warning: Exceeded the 5-minute limit.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except MissionStop as err:
        print(f"\nMission stopped: {err}")
    finally:
        try:
            if 'dashboard' in dir() and dashboard is not None:
                dashboard.stop()
        finally:
            try:
                if chassis is not None:
                    chassis.stop()
            finally:
                try:
                    if logger is not None:
                        logger.stop()
                finally:
                    if connected:
                        ep_robot.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
