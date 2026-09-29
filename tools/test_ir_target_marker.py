"""Interactively test infrared firing and mark one target for run_round2.py.

Examples:
  py -3.8 test_ir_target_marker.py --target-cell 3,2
  py -3.8 test_ir_target_marker.py --target-cell 3,2 --color red --shape circle --fire
  py -3.8 test_ir_target_marker.py --target-cell 3,2 --dry-run
"""

import argparse
import json
from pathlib import Path

from main import _sdk_connection_type
from src.config_loader import load_config
from src.dashboard import Dashboard
from src.logger import SensorLogger
from src.mission_stop import MissionStop
from src.target_marker import (
    build_target_record,
    fire_infrared,
    load_target_document,
    next_target_id,
    parse_cell,
    save_target_record,
    select_confirmed_detection,
)


COLORS = ("red", "green", "yellow", "blue")
SHAPES = ("circle", "square", "horizontal", "vertical")
SIDES = ("x+", "x-", "y+", "y-")


def _parser():
    parser = argparse.ArgumentParser(
        description="Confirm a colored sign, optionally fire infrared, and save it for Round 2."
    )
    parser.add_argument("--target-cell", required=True, type=parse_cell,
                        help="Grid cell to mark as x,y, for example 3,2")
    parser.add_argument("--target-id", help="Target ID; omitted selects the next T-number")
    parser.add_argument("--side", choices=SIDES,
                        help="Optional target-facing side used by same-cell Round 2 shots")
    parser.add_argument("--color", choices=COLORS, help="Only accept this color")
    parser.add_argument("--shape", choices=SHAPES, help="Only accept this shape")
    parser.add_argument("--targets-file", default="data/targets.json",
                        help="Output consumed by run_round2.py")
    parser.add_argument("--map", default="data/maps/latest.json",
                        help="Round 1 map used to validate the target cell")
    parser.add_argument("--fire", action="store_true",
                        help="Enable infrared firing after an interactive Enter confirmation")
    parser.add_argument("--times", type=int, choices=range(1, 6), default=1,
                        help="Infrared pulses, 1 to 5")
    parser.add_argument("--max-center-offset", type=float, default=0.22,
                        help="Maximum normalized distance from the image crosshair")
    parser.add_argument("--max-result-age-ms", type=int, default=500,
                        help="Maximum detection age accepted when Enter is pressed")
    parser.add_argument("--replace", action="store_true",
                        help="Replace an existing target with the same ID")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print an example record without connecting, firing, or writing")
    return parser


def _dry_run_record(args, target_id):
    detection = {
        "color": args.color or "red",
        "shape": args.shape or "circle",
        "center_px": [320, 180],
        "center_offset_norm": [0.0, 0.0],
        "area_px2": 1000.0,
        "stability_hits": 3,
        "stability_required": 3,
    }
    return build_target_record(
        target_id, args.target_cell, detection,
        pose=(0, 0, 0), gimbal=(0, 0, 0, 0), side=args.side,
        fired=False, fire_times=0,
    )


def main(argv=None):
    args = _parser().parse_args(argv)
    if args.max_center_offset <= 0:
        print("Error: --max-center-offset must be positive")
        return 2
    if args.max_result_age_ms <= 0:
        print("Error: --max-result-age-ms must be positive")
        return 2

    project_dir = Path(__file__).resolve().parent
    target_path = project_dir / args.targets_file
    try:
        existing = load_target_document(target_path)
        target_id = args.target_id or next_target_id(existing["targets"])
        duplicate = any(
            isinstance(item, dict) and item.get("id") == target_id
            for item in existing["targets"]
        )
        if duplicate and not args.replace:
            print(f"Error: target id {target_id} already exists; use --replace to update it")
            return 2
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"Error: {error}")
        return 2

    if args.dry_run:
        print(json.dumps(_dry_run_record(args, target_id), ensure_ascii=False, indent=2))
        print("Dry run: did not connect, fire, or write a target file.")
        return 0

    from src.planner import GridMap

    map_path = project_dir / args.map
    try:
        grid_map = GridMap.from_file(map_path)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"Error loading map {map_path}: {error}")
        return 2
    if args.target_cell not in grid_map.cells:
        print(f"Error: target cell {args.target_cell} is not present in {map_path}")
        return 2

    config = load_config(project_dir / "config" / "settings.yaml")
    for stream in ("position", "attitude", "gimbal"):
        if not config["logging"]["streams"].get(stream, {}).get("enabled"):
            print(f"Error: logging.streams.{stream}.enabled must be true")
            return 2
    if not config["dashboard"]["enabled"] or not config["image_processing"]["enabled"]:
        print("Error: dashboard.enabled and image_processing.enabled must both be true")
        return 2

    from robomaster import blaster, conn, robot

    log_settings = config["logging"].copy()
    log_settings["directory"] = project_dir / log_settings["directory"]
    ep_robot = robot.Robot()
    logger = None
    dashboard = None
    connected = False
    gimbal_suspended = False
    try:
        ep_robot.initialize(conn_type=_sdk_connection_type(config["connection"]["type"], conn))
        connected = True
        logger = SensorLogger(ep_robot, log_settings)
        logger.start()
        dashboard = Dashboard(
            ep_robot, logger, config["dashboard"],
            target_settings=config.get("exploration", {}).get("target_inspection"),
        )
        dashboard.start()
        logger.wait_for("position")
        logger.wait_for("attitude")
        logger.wait_for("gimbal")
        gimbal_suspended = ep_robot.gimbal.suspend() is not False

        host = config["dashboard"]["host"]
        port = config["dashboard"]["port"]
        print(f"Dashboard: http://{host}:{port}")
        print("Gimbal suspended: rotate it by hand and place the target at the crosshair.")
        print("Enter = verify latest detection, then fire/mark | r = show status | q = cancel")

        while True:
            command = input("> ").strip().lower()
            if command == "q":
                print("Cancelled; no target was saved.")
                logger.run_status = "interrupted"
                return 0
            status = dashboard.snapshot()["target_detection"]
            candidate, reason = select_confirmed_detection(
                status, color=args.color, shape=args.shape,
                max_center_offset=args.max_center_offset,
                max_age_ms=args.max_result_age_ms,
            )
            if command == "r":
                print(json.dumps(status, ensure_ascii=False, indent=2))
                continue
            if command:
                print("Unknown command: press Enter to verify, r for status, or q to cancel")
                continue
            if candidate is None:
                print(f"Not ready: {reason}")
                continue

            pose = logger.get_latest("position", config["motion"]["sample_timeout_s"])
            attitude = logger.get_latest("attitude", config["motion"]["sample_timeout_s"])
            gimbal = logger.get_latest("gimbal", config["motion"]["sample_timeout_s"])
            if pose is None or attitude is None or gimbal is None:
                print("Not ready: position, attitude, or gimbal telemetry is stale")
                continue
            observed_pose = (pose[0], pose[1], attitude[0])

            fired = False
            if args.fire:
                result = fire_infrared(blaster, ep_robot.blaster, args.times)
                if result is False:
                    print("Infrared fire command failed; target was not saved.")
                    continue
                fired = True
                print(f"Infrared fired {args.times} time(s).")

            record = build_target_record(
                target_id, args.target_cell, candidate, observed_pose, gimbal,
                side=args.side, fired=fired, fire_times=args.times if fired else 0,
            )
            save_target_record(target_path, record, replace=args.replace)
            logger.run_status = "completed"
            print(f"Saved {target_id} at cell {args.target_cell} to {target_path}")
            print("run_round2.py can now load this file through --targets-file.")
            return 0
    except KeyboardInterrupt:
        print("\nStopped by user; no incomplete target was saved.")
        if logger is not None:
            logger.run_status = "interrupted"
        return 130
    except (MissionStop, OSError, ValueError, json.JSONDecodeError) as error:
        print(f"Stopped: {error}")
        if logger is not None:
            logger.run_status = "stopped"
            logger.run_error = str(error)
        return 1
    finally:
        try:
            if gimbal_suspended:
                ep_robot.gimbal.resume()
        finally:
            try:
                if dashboard is not None:
                    dashboard.stop()
            finally:
                try:
                    if logger is not None:
                        logger.stop()
                finally:
                    if connected:
                        ep_robot.close()


if __name__ == "__main__":
    raise SystemExit(main())
