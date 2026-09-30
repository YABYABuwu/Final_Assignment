from pathlib import Path
import math
from statistics import mode

import yaml


DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "settings.yaml"


def load_config(path=DEFAULT_CONFIG):
    """Read YAML and check the fields needed before connecting to a robot."""
    with open(path, encoding="utf-8") as file:
        config = yaml.safe_load(file)

    if not isinstance(config, dict):
        raise ValueError("settings.yaml must contain a mapping")
    for section in ("connection", "motion", "logging", "dashboard", "image_processing",
                    "review", "mission", "exploration", "rear_ir", "front_ir"):
        if not isinstance(config.get(section), dict):
            raise ValueError(f"missing config section: {section}")
    color_ranges = config.get("color_ranges")
    expected_colors = {"red", "green", "yellow", "blue"}
    if not isinstance(color_ranges, dict) or set(color_ranges) != expected_colors:
        raise ValueError("color_ranges must contain red, green, yellow and blue")
    for color, ranges in color_ranges.items():
        if not isinstance(ranges, list) or not ranges:
            raise ValueError(f"color_ranges.{color} must contain at least one HSV range")
        for index, endpoints in enumerate(ranges):
            key = f"color_ranges.{color}[{index}]"
            if not isinstance(endpoints, list) or len(endpoints) != 2:
                raise ValueError(f"{key} must contain lower and upper HSV triplets")
            lower, upper = endpoints
            for triplet in (lower, upper):
                if (not isinstance(triplet, list) or len(triplet) != 3 or
                        any(type(value) is not int or value < 0 or
                            value > (179 if axis == 0 else 255)
                            for axis, value in enumerate(triplet))):
                    raise ValueError(f"{key} HSV values must be integers within H 0..179, S/V 0..255")
            if any(low > high for low, high in zip(lower, upper)):
                raise ValueError(f"{key} lower HSV values must not exceed upper values")
    if config["connection"].get("type") not in ("ap", "sta", "rndis"):
        raise ValueError("connection.type must be ap, sta or rndis")

    dashboard = config["dashboard"]
    if not isinstance(dashboard.get("enabled"), bool):
        raise ValueError("dashboard.enabled must be true or false")
    if not isinstance(dashboard.get("host"), str) or not dashboard["host"]:
        raise ValueError("dashboard.host must be a nonempty string")
    if type(dashboard.get("port")) is not int or not 1 <= dashboard["port"] <= 65535:
        raise ValueError("dashboard.port must be between 1 and 65535")
    if dashboard.get("resolution") not in ("360p", "540p", "720p"):
        raise ValueError("dashboard.resolution must be 360p, 540p or 720p")
    if not isinstance(dashboard.get("max_fps"), (int, float)) or dashboard["max_fps"] <= 0:
        raise ValueError("dashboard.max_fps must be positive")
    quality = dashboard.get("jpeg_quality")
    if type(quality) is not int or not 1 <= quality <= 100:
        raise ValueError("dashboard.jpeg_quality must be between 1 and 100")

    image_processing = config["image_processing"]
    if not isinstance(image_processing.get("enabled"), bool):
        raise ValueError("image_processing.enabled must be true or false")
    if image_processing.get("mode") not in ("robust", "classic"):
        raise ValueError("image_processing.mode must be robust or classic")
    detection_fps = image_processing.get("detection_fps")
    if (type(detection_fps) not in (int, float) or not math.isfinite(detection_fps) or
            detection_fps <= 0 or detection_fps > dashboard["max_fps"]):
        raise ValueError("image_processing.detection_fps must be positive and no greater than dashboard.max_fps")
    stability_window = image_processing.get("stability_window")
    stability_min_hits = image_processing.get("stability_min_hits")
    if type(stability_window) is not int or not 1 <= stability_window <= 30:
        raise ValueError("image_processing.stability_window must be an integer from 1 to 30")
    if (type(stability_min_hits) is not int or
            not 1 <= stability_min_hits <= stability_window):
        raise ValueError("image_processing.stability_min_hits must be between 1 and stability_window")
    if not isinstance(image_processing.get("enable_undistort"), bool):
        raise ValueError("image_processing.enable_undistort must be true or false")
    for name in ("min_area", "max_area"):
        value = image_processing.get(name)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"image_processing.{name} must be a positive number")
    if image_processing["max_area"] <= image_processing["min_area"]:
        raise ValueError("image_processing.max_area must be greater than min_area")
    kernel_size = image_processing.get("morph_kernel_size")
    if type(kernel_size) is not int or kernel_size < 0 or (kernel_size > 0 and kernel_size % 2 == 0):
        raise ValueError("image_processing.morph_kernel_size must be zero or a positive odd integer")
    iterations = image_processing.get("morph_iterations")
    if type(iterations) is not int or iterations <= 0:
        raise ValueError("image_processing.morph_iterations must be a positive integer")
    if image_processing["enabled"] and not dashboard["enabled"]:
        raise ValueError("image_processing needs dashboard.enabled: true for the camera stream")

    review = config["review"]
    if not isinstance(review.get("host"), str) or not review["host"]:
        raise ValueError("review.host must be a nonempty string")
    if type(review.get("port")) is not int or not 1 <= review["port"] <= 65535:
        raise ValueError("review.port must be between 1 and 65535")
    points = review.get("max_points_per_stream")
    if type(points) is not int or points <= 0:
        raise ValueError("review.max_points_per_stream must be a positive integer")
    close_tof = review.get("close_tof_mm")
    if not isinstance(close_tof, (int, float)) or close_tof <= 0:
        raise ValueError("review.close_tof_mm must be positive")

    motion = config["motion"]
    if not isinstance(motion.get("hold_heading"), bool):
        raise ValueError("motion.hold_heading must be true or false")
    for name in ("position_tolerance_m", "angle_tolerance_deg", "timeout_s",
                 "sample_timeout_s", "control_period_s", "max_speed_m_s", "braking_decel_m_s2", "max_lateral_accel_m_s2",
                 "max_turn_deg_s"):
        if (not isinstance(motion.get(name), (int, float)) or
                not math.isfinite(motion[name]) or motion[name] <= 0):
            raise ValueError(f"motion.{name} must be a positive number")

    streams = config["logging"].get("streams")
    preview = config["logging"].get("preview_duration_s")
    if not isinstance(preview, (int, float)) or preview < 0:
        raise ValueError("logging.preview_duration_s must be zero or positive")
    for name in ("queue_max_rows", "history_max_samples", "batch_size"):
        value = config["logging"].get(name)
        if type(value) is not int or value <= 0:
            raise ValueError(f"logging.{name} must be a positive integer")
    interval = config["logging"].get("flush_interval_s")
    if not isinstance(interval, (int, float)) or interval <= 0:
        raise ValueError("logging.flush_interval_s must be a positive number")
    if not isinstance(streams, dict):
        raise ValueError("logging.streams must be a mapping")
    from src.logger import STREAMS
    for name, settings in streams.items():
        if name not in STREAMS:
            raise ValueError(f"unknown stream: {name}")
        if not isinstance(settings, dict) or not isinstance(settings.get("enabled"), bool):
            raise ValueError(f"logging.streams.{name}.enabled must be true or false")
        if not isinstance(settings.get("save"), bool):
            raise ValueError(f"logging.streams.{name}.save must be true or false")
        if settings.get("frequency_hz") not in (1, 5, 10, 20, 50):
            raise ValueError(f"logging.streams.{name}.frequency_hz must be 1, 5, 10, 20 or 50")
        if name == "battery" and settings["frequency_hz"] not in (1, 5, 10):
            raise ValueError("battery frequency_hz must be 1, 5 or 10")
        if settings["save"] and not settings["enabled"]:
            raise ValueError(f"logging.streams.{name} cannot save when disabled")

    used_ir_ports = {}
    for end in ("rear", "front"):
        key = end + "_ir"
        ir = config[key]
        if not isinstance(ir.get("enabled"), bool):
            raise ValueError(f"{key}.enabled must be true or false")
        for side in ("right", "left"):
            port = ir.get(side)
            if (not isinstance(port, dict) or type(port.get("id")) is not int or
                    not 1 <= port["id"] <= 6 or type(port.get("port")) is not int or
                    port["port"] not in (1, 2)):
                raise ValueError(f"{key}.{side} needs id 1..6 and port 1 or 2")
            active_io = port.get("active_io", ir.get("active_io"))
            if (ir["enabled"] or active_io is not None) and (
                    type(active_io) is not int or active_io not in (0, 1)):
                raise ValueError(f"{key}.{side}.active_io must be 0 or 1")
            if ir["enabled"]:
                address = (port["id"], port["port"])
                if address in used_ir_ports:
                    raise ValueError(f"{key}.{side} shares an adapter port with {used_ir_ports[address]}")
                used_ir_ports[address] = f"{key}.{side}"
        if (type(ir.get("max_age_s")) not in (int, float) or
                not math.isfinite(ir["max_age_s"]) or ir["max_age_s"] <= 0):
            raise ValueError(f"{key}.max_age_s must be positive")
        if ir["enabled"] and not streams.get("adapter", {}).get("enabled"):
            raise ValueError(f"{key} needs logging.streams.adapter.enabled: true")
        for name in ("recovery_speed_m_s", "recovery_max_m"):
            value = ir.get(name)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key}.{name} must be a positive number")
        max_attempts = ir.get("recovery_max_attempts", 1)
        if type(max_attempts) is not int or not 1 <= max_attempts <= 10:
            raise ValueError(f"{key}.recovery_max_attempts must be an integer from 1 to 10")
        total_max_m = ir.get("recovery_total_max_m", ir["recovery_max_m"])
        if (type(total_max_m) not in (int, float) or
                not math.isfinite(total_max_m) or
                total_max_m < ir["recovery_max_m"]):
            raise ValueError(
                f"{key}.recovery_total_max_m must be at least recovery_max_m"
            )
        if ir["recovery_speed_m_s"] > motion["max_speed_m_s"]:
            raise ValueError(f"{key}.recovery_speed_m_s cannot exceed motion.max_speed_m_s")
        exploration_speed = config["exploration"].get("max_speed_m_s")
        if (config["exploration"].get("enabled") and
                type(exploration_speed) in (int, float) and math.isfinite(exploration_speed) and
                ir["recovery_speed_m_s"] > exploration_speed):
            raise ValueError(f"{key}.recovery_speed_m_s cannot exceed exploration.max_speed_m_s")
        step_m = config["exploration"].get("step_m")
        if (
            type(step_m) in (int, float)
            and math.isfinite(step_m)
            and total_max_m >= step_m
        ):
            raise ValueError(
                f"{key}.recovery_total_max_m "
                "must be below one exploration step"
            )
        clear_samples = ir.get("recovery_clear_samples")
        if type(clear_samples) is not int or not 1 <= clear_samples <= 20:
            raise ValueError(f"{key}.recovery_clear_samples must be an integer from 1 to 20")

        mode = ir.get("recovery_mode")

        allowed_recovery_modes = (
            "directional",
            "adaptive",
            "cardinal",
            "diagonal",
            "forward_first",
            "staged",
        )

        if (
            mode is not None
            and mode not in allowed_recovery_modes
        ):
            raise ValueError(
                f"{key}.recovery_mode must be adaptive, cardinal, "
                "diagonal, forward_first, or staged"
            )

        tof_clear = ir.get("forward_tof_clear_mm")
        if tof_clear is not None and (type(tof_clear) not in (int, float) or not math.isfinite(tof_clear) or tof_clear <= 0):
            raise ValueError(f"{key}.forward_tof_clear_mm must be a positive number")
        tof_max_age = ir.get("forward_tof_max_age_s")
        if (tof_max_age is not None and
                (type(tof_max_age) not in (int, float) or
                 not math.isfinite(tof_max_age) or tof_max_age <= 0)):
            raise ValueError(f"{key}.forward_tof_max_age_s must be a positive number")
        if not isinstance(ir.get("direct_io_fallback", True), bool):
            raise ValueError(f"{key}.direct_io_fallback must be true or false")
        if ir.get("io_read_mode", "auto") not in ("auto", "direct", "stream"):
            raise ValueError(f"{key}.io_read_mode must be auto, direct, or stream")
        direct_cache_s = ir.get("direct_fallback_cache_s", 0.2)
        if (type(direct_cache_s) not in (int, float) or
                not math.isfinite(direct_cache_s) or not 0.05 <= direct_cache_s <= 2.0):
            raise ValueError(
                f"{key}.direct_fallback_cache_s must be between 0.05 and 2.0 seconds"
            )
        if not isinstance(ir.get("auto_calibrate_io", False), bool):
            raise ValueError(f"{key}.auto_calibrate_io must be true or false")
        calibration_samples = ir.get("calibration_samples", 10)
        if type(calibration_samples) is not int or not 3 <= calibration_samples <= 100:
            raise ValueError(f"{key}.calibration_samples must be an integer from 3 to 100")
        calibration_consistency = ir.get("calibration_min_consistency", 0.8)
        if (type(calibration_consistency) not in (int, float) or
                not math.isfinite(calibration_consistency) or
                not 0.5 < calibration_consistency <= 1.0):
            raise ValueError(
                f"{key}.calibration_min_consistency must be greater than 0.5 and at most 1.0"
            )

    if config["mission"].get("enabled"):
        for name in ("position", "attitude"):
            if not streams.get(name, {}).get("enabled"):
                raise ValueError(f"mission needs logging.streams.{name}.enabled: true")

    directional = [config[k].get("recovery_mode") == "directional" for k in ("front_ir", "rear_ir")]
    if any(directional) and (not all(directional) or not all(config[k]["enabled"] for k in ("front_ir", "rear_ir"))):
        raise ValueError("directional recovery requires enabled front_ir and rear_ir with the same mode")
    exploration = config["exploration"]
    if not isinstance(exploration.get("enabled"), bool):
        raise ValueError("exploration.enabled must be true or false")
    if type(exploration.get("position_coordinate_system")) is not int or exploration["position_coordinate_system"] not in (0, 1):
        raise ValueError("exploration.position_coordinate_system must be 0 or 1")
    for name in ("step_m", "max_speed_m_s", "wall_threshold_mm",
                 "max_sample_age_s", "sample_skew_s", "update_hz"):
        value = exploration.get(name)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"exploration.{name} must be a positive number")
    if exploration["sample_skew_s"] > exploration["max_sample_age_s"]:
        raise ValueError("exploration.sample_skew_s cannot exceed max_sample_age_s")
    lane = exploration.setdefault("ir_lane", {
        "enabled": False, "min_shift_m": 0.005, "max_offset_m": 0.10})
    if not isinstance(lane, dict) or type(lane.get("enabled")) is not bool:
        raise ValueError("exploration.ir_lane.enabled must be true or false")
    for name in ("min_shift_m", "max_offset_m"):
        value = lane.get(name)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError("exploration.ir_lane.{} must be positive and finite".format(name))
    if not lane["min_shift_m"] < lane["max_offset_m"] < exploration["step_m"] / 2:
        raise ValueError("IR lane needs min_shift_m < max_offset_m < half a grid step")
    if exploration["update_hz"] > 50:
        raise ValueError("exploration.update_hz cannot exceed 50 Hz")
    median_window = exploration.get("tof_median_window")
    if type(median_window) is not int or median_window < 1 or median_window > 9 or median_window % 2 != 1:
        raise ValueError("exploration.tof_median_window must be an odd integer from 1 to 9")
    if exploration["max_speed_m_s"] > motion["max_speed_m_s"]:
        raise ValueError("exploration.max_speed_m_s cannot exceed motion.max_speed_m_s")
    if exploration.get("heading_source") not in ("gimbal", "attitude"):
        raise ValueError("exploration.heading_source must be gimbal or attitude")
    alignment = exploration.get("alignment")
    if not isinstance(alignment, dict):
        raise ValueError("exploration.alignment must be a mapping")
    if not isinstance(alignment.get("enabled"), bool):
        raise ValueError("exploration.alignment.enabled must be true or false")
    interval_steps = alignment.setdefault("interval_steps", 2)
    if type(interval_steps) is not int or interval_steps < 1:
        raise ValueError("exploration.alignment.interval_steps must be a positive integer")
    for name in ("wall_distance_m", "tolerance_m", "max_shift_m"):
        value = alignment.get(name)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"exploration.alignment.{name} must be a positive number")
    if alignment["tolerance_m"] >= alignment["wall_distance_m"]:
        raise ValueError("exploration.alignment.tolerance_m must be below wall_distance_m")
    if alignment["max_shift_m"] >= exploration["step_m"] / 2:
        raise ValueError("exploration.alignment.max_shift_m must be below half a grid step")
    heading_alignment = exploration.get("heading_alignment")
    if not isinstance(heading_alignment, dict):
        raise ValueError("exploration.heading_alignment must be a mapping")
    if not isinstance(heading_alignment.get("enabled"), bool):
        raise ValueError("exploration.heading_alignment.enabled must be true or false")
    offsets = heading_alignment.get("scan_offsets_deg")
    if (not isinstance(offsets, list) or len(offsets) < 3 or len(offsets) > 9 or
            len(offsets) % 2 != 1 or
            any(type(value) not in (int, float) or not math.isfinite(value) or
                abs(value) > 40 for value in offsets) or
            offsets != sorted(offsets) or len(set(offsets)) != len(offsets) or
            offsets[len(offsets) // 2] != 0):
        raise ValueError("exploration.heading_alignment.scan_offsets_deg needs sorted distinct angles around 0")
    samples_per_angle = heading_alignment.get("samples_per_angle")
    if (type(samples_per_angle) is not int or samples_per_angle < 1 or
            samples_per_angle > 9 or samples_per_angle % 2 != 1):
        raise ValueError("exploration.heading_alignment.samples_per_angle must be an odd integer from 1 to 9")
    for name in ("min_span_m", "max_residual_m",
                 "max_stationary_shift_m"):
        value = heading_alignment.get(name)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"exploration.heading_alignment.{name} must be positive")
    emergency_distance = exploration.get("emergency_stop_distance_m")
    if (type(emergency_distance) not in (int, float) or
            not math.isfinite(emergency_distance) or emergency_distance <= 0):
        raise ValueError("exploration.emergency_stop_distance_m must be a positive number")
    max_nodes = exploration.get("max_nodes")
    if type(max_nodes) is not int or not 1 <= max_nodes <= 10000:
        raise ValueError("exploration.max_nodes must be an integer from 1 to 10000")
    map_settings = exploration.get("map")
    scan_match = exploration.get("scan_match")
    if not isinstance(map_settings, dict) or not isinstance(scan_match, dict):
        raise ValueError("exploration.map and exploration.scan_match must be mappings")
    for name in ("resolution_m", "width_m", "height_m", "robot_clearance_m"):
        value = map_settings.get(name)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"exploration.map.{name} must be a positive number")
    cells_x = round(map_settings["width_m"] / map_settings["resolution_m"])
    cells_y = round(map_settings["height_m"] / map_settings["resolution_m"])
    if cells_x < 2 or cells_y < 2 or cells_x * cells_y > 500000:
        raise ValueError("exploration map must contain between 4 and 500000 cells")
    for name, cells in (("start_x_m", cells_x), ("start_y_m", cells_y)):
        start = map_settings.get(name)
        if start is not None and (type(start) not in (int, float) or
                                  not math.isfinite(start) or
                                  not 0 <= start < cells * map_settings["resolution_m"]):
            raise ValueError(
                f"exploration.map.{name} must be null or within the map extent")
    for name in ("free_update", "occupied_update"):
        value = map_settings.get(name)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value == 0:
            raise ValueError(f"exploration.map.{name} must be a nonzero number")
    if map_settings["free_update"] >= 0 or map_settings["occupied_update"] <= 0:
        raise ValueError("exploration map updates must lower free odds and raise occupied odds")
    for path_name in ("save_path", "load_path"):
        path = map_settings.get(path_name)
        if path_name == "save_path" and (not isinstance(path, str) or not path):
            raise ValueError("exploration.map.save_path must be a nonempty path")
        if path_name == "load_path" and path is not None and (not isinstance(path, str) or not path):
            raise ValueError("exploration.map.load_path must be null or a nonempty path")
    for name in ("translation_m", "angle_deg", "angle_step_deg", "minimum_score", "minimum_improvement"):
        value = scan_match.get(name)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError(f"exploration.scan_match.{name} must be a nonnegative number")
    if scan_match["angle_step_deg"] <= 0 or scan_match["angle_step_deg"] > scan_match["angle_deg"]:
        raise ValueError("exploration.scan_match.angle_step_deg must be positive and no larger than angle_deg")
    sensor = exploration.get("sensor")
    gimbal = exploration.get("gimbal")
    if not isinstance(sensor, dict) or not isinstance(gimbal, dict):
        raise ValueError("exploration.sensor and exploration.gimbal must be mappings")
    if not isinstance(gimbal.get("auto_recenter"), bool):
        raise ValueError("exploration.gimbal.auto_recenter must be true or false")
    channel = sensor.get("tof_channel")
    if type(channel) is not int or channel not in (0, 1, 2, 3):
        raise ValueError("exploration.sensor.tof_channel must be an integer from 0 to 3")
    for name in ("offset_from_yaw_axis_m", "offset_yaw_deg", "pivot_x_m",
                 "pivot_y_m", "yaw_offset_deg"):
        value = sensor.get(name)
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"exploration.sensor.{name} must be a finite number")
    if sensor["offset_from_yaw_axis_m"] < 0:
        raise ValueError("exploration.sensor.offset_from_yaw_axis_m must be nonnegative")
    for name in ("yaw_speed_deg_s", "recenter_speed_deg_s",
                 "angle_tolerance_deg", "pitch_tolerance_deg",
                 "movement_angle_tolerance_deg"):
        value = gimbal.get(name)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"exploration.gimbal.{name} must be a positive number")
    if gimbal["angle_tolerance_deg"] >= 45:
        raise ValueError("exploration.gimbal.angle_tolerance_deg must be less than 45")
    if gimbal["pitch_tolerance_deg"] >= 45:
        raise ValueError("exploration.gimbal.pitch_tolerance_deg must be less than 45")
    if gimbal["movement_angle_tolerance_deg"] >= 45:
        raise ValueError("exploration.gimbal.movement_angle_tolerance_deg must be less than 45")
    movement_misses = gimbal.get("movement_alignment_miss_samples")
    if type(movement_misses) is not int or not 1 <= movement_misses <= 20:
        raise ValueError(
            "exploration.gimbal.movement_alignment_miss_samples must be an integer from 1 to 20"
        )
    pitch = gimbal.get("pitch_deg")
    if not isinstance(pitch, (int, float)) or not math.isfinite(pitch):
        raise ValueError("exploration.gimbal.pitch_deg must be a finite number")
    target = exploration.get("target_inspection")
    if not isinstance(target, dict) or not isinstance(target.get("enabled"), bool):
        raise ValueError("exploration.target_inspection.enabled must be true or false")
    selected = target.get("selected")
    colors = set(color_ranges)
    shapes = {"circle", "square", "horizontal", "vertical"}
    if selected != "all":
        if (not isinstance(selected, list) or not selected or
                any(not isinstance(item, str) or len(item.split(":")) != 2 or
                    item.split(":")[0] not in colors or item.split(":")[1] not in shapes
                    for item in selected)):
            raise ValueError("exploration.target_inspection.selected must be all or COLOR:SHAPE list")
    for name in ("pitch_deg", "min_area_fraction", "center_radius_fraction",
                 "aim_offset_x_fraction", "aim_offset_y_fraction",
                 "max_step_deg", "camera_hfov_deg", "camera_vfov_deg"):
        value = target.get(name)
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f"exploration.target_inspection.{name} must be finite")
    if not -20 <= target["pitch_deg"] <= 20:
        raise ValueError("exploration.target_inspection.pitch_deg must be within -20..20")
    if not 0 < target["min_area_fraction"] < .5:
        raise ValueError("exploration.target_inspection.min_area_fraction must be within 0..0.5")
    if not 0 < target["center_radius_fraction"] <= .2:
        raise ValueError("exploration.target_inspection.center_radius_fraction must be within 0..0.2")
    for name in ("aim_offset_x_fraction", "aim_offset_y_fraction"):
        if not -.25 <= target[name] <= .25:
            raise ValueError(f"exploration.target_inspection.{name} must be within -0.25..0.25")
    for name in ("max_step_deg", "camera_hfov_deg", "camera_vfov_deg"):
        if not 0 < target[name] <= (10 if name == "max_step_deg" else 180):
            raise ValueError(f"exploration.target_inspection.{name} is outside its range")
    for name in ("confirm_frames", "lock_frames", "lock_max_misses",
                 "target_lost_frames", "reacquire_frames",
                 "max_aim_steps", "max_targets_per_wall"):
        if type(target.get(name)) is not int or not 1 <= target[name] <= 30:
            raise ValueError(f"exploration.target_inspection.{name} must be an integer from 1 to 30")
    if target["max_aim_steps"] < target["lock_frames"]:
        raise ValueError("exploration.target_inspection.max_aim_steps must cover lock_frames")
    shots = target.get("shots_per_target", 2)
    if type(shots) is not int or not 1 <= shots <= 5:
        raise ValueError("exploration.target_inspection.shots_per_target must be an integer from 1 to 5")
    settle = target.get("fire_settle_s", 1.0)
    if type(settle) not in (int, float) or not math.isfinite(settle) or settle < 0:
        raise ValueError("exploration.target_inspection.fire_settle_s must be a non-negative number")
    aim_settle = target.get("aim_settle_s", 0.15)
    if type(aim_settle) not in (int, float) or not math.isfinite(aim_settle) or aim_settle < 0:
        raise ValueError("exploration.target_inspection.aim_settle_s must be a non-negative number")
    for name in ("scan_yaw_speed_deg_s", "aim_yaw_speed_deg_s", "wait_timeout_s", "gimbal_wait_timeout_s"):
        if name in target:
            val = target[name]
            if type(val) not in (int, float) or not math.isfinite(val) or val <= 0:
                raise ValueError(f"exploration.target_inspection.{name} must be a positive number")
    if "search_frames" in target:
        sf = target["search_frames"]
        if type(sf) is not int or sf < target["confirm_frames"]:
            raise ValueError("exploration.target_inspection.search_frames must be an integer >= confirm_frames")
    if target.get("fire_mode") != "infrared":
        raise ValueError("exploration.target_inspection.fire_mode must be infrared")
    if exploration["enabled"]:
        if config["mission"].get("enabled"):
            raise ValueError("mission and exploration cannot both be enabled")
        if not dashboard["enabled"]:
            raise ValueError("exploration needs dashboard.enabled: true to show the SLAM map")
        for name in ("position", "attitude", "tof", "status", "gimbal"):
            if not streams.get(name, {}).get("enabled"):
                raise ValueError(f"exploration needs logging.streams.{name}.enabled: true")
    return config
