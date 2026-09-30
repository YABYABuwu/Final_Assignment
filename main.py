"""Connect the robot and run telemetry, waypoint missions, or DFS exploration."""

from pathlib import Path
import time

from src.chassis import ChassisController, grid_motion_settings
from src.config_loader import load_config
from src.dashboard import Dashboard
from src.explorer import DFSExplorer
from src.gimbal_control import ChassisRelativeGimbal
from src.logger import SensorLogger
from src.mission_stop import MissionStop
from src.rear_ir import FrontIRBumper, RearIRBumper
from src.slam import OccupancyGridSLAM, SlamWorker
from src.sound_player import play_startup_sound


def _sdk_connection_type(name, sdk_conn):
    """Pass the SDK's own constants: its connection code compares by identity."""
    return {
        "ap": sdk_conn.CONNECTION_WIFI_AP,
        "sta": sdk_conn.CONNECTION_WIFI_STA,
        "rndis": sdk_conn.CONNECTION_USB_RNDIS,
    }[name]


def main():
    config = load_config()
    project_dir = Path(__file__).resolve().parent
    log_settings = config["logging"].copy()
    log_settings["directory"] = project_dir / log_settings["directory"]
    exploration_settings = config["exploration"]
    map_settings = exploration_settings["map"]
    if map_settings["save_path"]:
        map_settings["save_path"] = project_dir / map_settings["save_path"]
    if map_settings["load_path"]:
        map_settings["load_path"] = project_dir / map_settings["load_path"]
    slam_map = OccupancyGridSLAM(exploration_settings)
    if map_settings["load_path"]:
        slam_map.load_file(map_settings["load_path"])
    if exploration_settings["enabled"]:
        log_settings["position_cs"] = exploration_settings["position_coordinate_system"]

    # Import here so config errors are shown before any SDK connection attempt.
    from robomaster import conn, robot

    ep_robot = robot.Robot()
    logger = None
    chassis = None
    dashboard = None
    slam_worker = None
    explorer = None
    connected = False
    try:
        ep_robot.initialize(conn_type=_sdk_connection_type(
            config["connection"]["type"], conn
        ))
        connected = True
        play_startup_sound(ep_robot)
        logger = SensorLogger(ep_robot, log_settings)
        logger.start()
        motion_settings = config["motion"].copy()
        if exploration_settings["enabled"]:
            motion_settings = grid_motion_settings(config)
        logger.motion_settings = {"max_speed_m_s": motion_settings["max_speed_m_s"],
                                  "braking_decel_m_s2": motion_settings["braking_decel_m_s2"],
                                  "max_lateral_accel_m_s2": motion_settings["max_lateral_accel_m_s2"]}
        chassis = ChassisController(ep_robot, logger, motion_settings)
        if config["rear_ir"]["enabled"]:
            chassis.rear_ir = RearIRBumper(logger, config["rear_ir"])
            logger.rear_ir_settings = config["rear_ir"]
            logger.rear_ir_recoveries = chassis.rear_ir.events
        if config["front_ir"]["enabled"]:
            chassis.front_ir = FrontIRBumper(logger, config["front_ir"])
            logger.front_ir_settings = config["front_ir"]
            logger.front_ir_recoveries = chassis.front_ir.events
        if exploration_settings["enabled"]:
            targets_file = project_dir / "data" / "targets.json"
            from src.target_marker import init_target_document
            init_target_document(targets_file, backup=True)
            slam_worker = SlamWorker(logger, slam_map, exploration_settings)
            explorer = DFSExplorer(chassis, ChassisRelativeGimbal(ep_robot.gimbal), logger, slam_map,
                                    exploration_settings)
            if config["front_ir"].get("recovery_mode") == "directional":
                from src.directional_recovery import DirectionalToF
                chassis.directional_tof = DirectionalToF(explorer)
            slam_worker.start()
        print("Connected. Logs:", logger.run_dir or "disabled")
        if config["dashboard"]["enabled"]:
            dashboard = Dashboard(ep_robot, logger, config["dashboard"],
                                  slam_map=slam_map, slam_worker=slam_worker,
                                  explorer=explorer, motion_settings=logger.motion_settings,
                                  rear_ir=chassis.rear_ir, front_ir=chassis.front_ir,
                                  target_settings=(exploration_settings["target_inspection"]
                                                   if explorer is not None else None),
                                  color_ranges=config["color_ranges"],
                                  mission_start_required=(exploration_settings["enabled"] or config["mission"]["enabled"]))
            dashboard.start()
            if explorer is not None and exploration_settings["target_inspection"]["enabled"]:
                from robomaster import blaster as sdk_blaster
                from src.target_inspection import WallTargetInspector
                explorer.target_inspector = WallTargetInspector(
                    ep_robot.gimbal, ep_robot.blaster, dashboard, logger, chassis,
                    slam_worker, exploration_settings, sdk_blaster.INFRARED_FIRE,
                    on_progress=explorer._set_target_progress, scan_gimbal=explorer.gimbal,
                    color_ranges=config["color_ranges"])
            host = config["dashboard"]["host"]
            port = config["dashboard"]["port"]
            print(f"Dashboard: http://{host}:{port}")

        if dashboard is not None:
            dashboard.wait_for_mission_start(chassis)

        if exploration_settings["enabled"]:
            if dashboard is not None:
                dashboard.mission_status = "SLAM readying"
            logger.wait_for("position")
            logger.wait_for("attitude")
            logger.wait_for("tof")
            logger.wait_for("gimbal")
            logger.wait_for("status")
            slam_worker.wait_ready()
            explorer.run(slam_worker)
            slam_worker.stop(map_settings["save_path"], logger.run_dir)
            if dashboard is not None:
                dashboard.mission_status = f"Exploration {explorer.status}"
            if explorer.status == "no_safe_direction":
                logger.run_status = explorer.status
                logger.run_error = explorer.error
            else:
                logger.run_status = "completed"
        elif config["mission"]["enabled"]:
            logger.wait_for("position", config["motion"]["sample_timeout_s"])
            logger.wait_for("attitude", config["motion"]["sample_timeout_s"])
            for point in config["mission"]["waypoints"]:
                if dashboard is not None:
                    dashboard.mission_status = f"Moving to {point}"
                pose = chassis.move_to(point["x"], point["y"], point.get("yaw"))
                print("Reached:", pose)
            if dashboard is not None:
                dashboard.mission_status = "Mission complete"
            logger.run_status = "completed"
        else:
            if dashboard is None:
                duration = config["logging"]["preview_duration_s"]
                print(f"Mission disabled. Recording telemetry for {duration} s.")
                time.sleep(duration)
                print("Position:", logger.get_latest("position"))
                print("Attitude:", logger.get_latest("attitude"))
            logger.run_status = "completed"

        if dashboard is not None:
            print("Dashboard is open. Press Ctrl+C to stop.")
            while True:
                if dashboard.camera_error is not None:
                    raise MissionStop(f"camera error: {dashboard.camera_error}")
                time.sleep(0.5)
    except KeyboardInterrupt:
        print("Stopped by user")
        if logger is not None and logger.run_status == "running":
            logger.run_status = "interrupted"
    except MissionStop as error:
        print(f"Mission stopped: {error}")
        if logger is not None:
            logger.run_status = "stopped"
            logger.run_error = str(error)
        if dashboard is not None:
            dashboard.mission_status = f"Stopped: {error}"
    except Exception as error:
        if logger is not None:
            logger.run_status = "failed"
            logger.run_error = str(error)
        raise
    finally:
        try:
            if chassis is not None:
                chassis.stop()
        finally:
            try:
                if slam_worker is not None:
                    slam_worker.stop(map_settings["save_path"],
                                     logger.run_dir if logger is not None else None)
            finally:
                if logger is not None and explorer is not None:
                    logger.exploration_state = explorer.snapshot()
                try:
                    if dashboard is not None:
                        dashboard.stop()
                finally:
                    try:
                        if logger is not None:
                            logger.stop()
                            if logger.dropped_rows:
                                print(f"Warning: skipped {logger.dropped_rows} CSV rows (queue full)")
                    finally:
                        if connected:
                            ep_robot.close()


if __name__ == "__main__":
    main()
