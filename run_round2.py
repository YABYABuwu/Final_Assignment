"""Round 2 Execution Script for RoboMaster EP Assignment 2.

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
from src.planner import GridMap, MultiTargetPlanner, plot_mission_map
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
    parser.add_argument("--allow-same-cell", action="store_true", help="Allow shooting from the same tile as the target (distance 0)")
    parser.add_argument("--standoff", type=float, default=0.20, help="Standoff distance in meters from cell center when shooting in same cell (accounts for 20cm gimbal length)")
    parser.add_argument("--return-to-start", action="store_true", help="Return to starting cell after shooting")
    parser.add_argument("--algorithm", default="bfs", choices=["bfs", "astar"], help="Search algorithm")
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

    # Resolve targets (CLI argument > map detected_targets > targets.json)
    target_list = args.targets
    if not target_list:
        if grid_map.detected_targets:
            print("Auto-loaded targets from map file.")
            target_list = grid_map.detected_targets
        else:
            targets_file = project_dir / args.targets_file
            if targets_file.exists():
                try:
                    with open(targets_file, "r", encoding="utf-8") as f:
                        file_data = json.load(f)
                        if isinstance(file_data, list):
                            target_list = file_data
                        elif isinstance(file_data, dict):
                            target_list = file_data.get("targets", [])
                        print(f"Auto-loaded targets from {args.targets_file}.")
                except Exception as e:
                    print(f"Warning: Failed to load targets file {targets_file}: {e}")

    if not target_list:
        print("Error: No targets specified via --targets and no detected targets found in map or data/targets.json.")
        print("Usage example: python run_round2.py --targets \"[(3,2), (2,1)]\"")
        return

    # Filter out hostages / non-targets if target objects have metadata
    valid_targets = []
    for item in target_list:
        if isinstance(item, dict):
            # Check if flagged as hostage or should_shoot is False
            if item.get("type") == "hostage" or item.get("should_shoot") is False:
                print(f"  [Skip] Non-target or Hostage ignored: {item.get('id', item)}")
                continue
            valid_targets.append(item)
        else:
            valid_targets.append(item)

    if not valid_targets:
        print("Error: All found targets are flagged as hostages/non-targets. Nothing to shoot.")
        return

    print(f"Active Targets to Shoot: {valid_targets}")

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

    from robomaster import conn, robot
    from src.chassis import ChassisController
    from src.gimbal_control import ChassisRelativeGimbal
    from src.logger import SensorLogger
    from src.rear_ir import RearIRBumper, FrontIRBumper

    def sdk_conn_type(name):
        return {
            "ap": conn.CONNECTION_WIFI_AP,
            "sta": conn.CONNECTION_WIFI_STA,
            "rndis": conn.CONNECTION_USB_RNDIS,
        }[name]

    ep_robot = robot.Robot()
    logger = None
    chassis = None
    connected = False

    try:
        ep_robot.initialize(conn_type=sdk_conn_type(config["connection"]["type"]))
        connected = True
        logger = SensorLogger(ep_robot, log_settings)
        logger.start()

        motion_settings = config["motion"].copy()
        chassis = ChassisController(ep_robot, logger, motion_settings)
        if config["rear_ir"]["enabled"]:
            chassis.rear_ir = RearIRBumper(logger, config["rear_ir"])
        if config["front_ir"]["enabled"]:
            chassis.front_ir = FrontIRBumper(logger, config["front_ir"])

        gimbal = ChassisRelativeGimbal(ep_robot.gimbal)
        gimbal.recenter()

        print("Waiting for telemetry sensors...")
        logger.wait_for("position", 5.0)
        logger.wait_for("attitude", 5.0)

        # Index shooting actions by the waypoint step index
        actions_by_step = {act["path_step_index"]: act for act in plan["shooting_plan"]}

        start_time = time.time()
        print("\n=== Executing Round 2 Mission ===")

        for step_idx, cell in enumerate(plan["full_path"]):
            world_x, world_y = plan["waypoints"][step_idx]
            print(f"\nStep {step_idx + 1}/{len(plan['full_path'])}: Moving to {cell} (x={world_x:.2f} m, y={world_y:.2f} m)")
            
            # Slide to cell holding heading
            chassis.move_to(world_x, world_y, yaw=None)

            # Check if this cell is a shooting standpoint
            if step_idx in actions_by_step:
                action = actions_by_step[step_idx]
                target_id = action["target_id"]
                target_pos = action["target_pos"]
                yaw_deg = action["gimbal_yaw_deg"]
                print(f"  -> Shooting Standpoint reached for {target_id} at {target_pos}!")
                print(f"  -> Aiming gimbal to {yaw_deg:.1f} deg...")
                gimbal_action = gimbal.moveto(pitch=0, yaw=yaw_deg, yaw_speed=90)
                if gimbal_action is not None:
                    gimbal_action.wait_for_completed(timeout=3.0)
                time.sleep(0.5)

                # Fire blaster (if available)
                print(f"  -> FIRING at Target {target_id}!")
                try:
                    if hasattr(ep_robot, "blaster"):
                        ep_robot.blaster.fire(times=1)
                except Exception as e:
                    print(f"  (Blaster notice: {e})")
                time.sleep(0.5)

                # Return gimbal to center
                gimbal.moveto(pitch=0, yaw=0, yaw_speed=90)
                time.sleep(0.3)

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
        if chassis is not None:
            chassis.stop()
        if logger is not None:
            logger.stop()
        if connected:
            ep_robot.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
