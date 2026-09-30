"""Rear diagonal IR bumper using the shared SensorLogger adapter stream."""

import math
import threading
import time


def adapter_index(sensor_id, port):
    """SDK adapter order: ID 1 port 1/2, ID 2 port 1/2, ..."""
    return (sensor_id - 1) * 2 + (port - 1)


def recovery_vector(sides, speed, attempt=1, end="rear", mode="staged",
                    forward_clear=True, movement_axis="longitudinal"):
    """Choose a staged escape away from a detected front or rear IR.

    Parameters
    ----------
    sides : dict
        {"right": {"detected": bool}, "left": {"detected": bool}}
    speed : float
        Magnitude of recovery escape speed in m/s.
    attempt : int, optional
        Recovery retry attempt number (1, 2, 3, ...).
    end : str, optional
        "rear" or "front".
    mode : str, optional
        "cardinal" (choose an escape perpendicular to the current travel axis),
        "diagonal" (moves away longitudinally and laterally simultaneously),
        "forward_first" (longitudinal first if clear, then diagonal),
        or "staged" (legacy: lateral then longitudinal then diagonal).
    forward_clear : bool, optional
        Whether the longitudinal escape direction is clear of obstacles
        (e.g., front ToF >= threshold when escaping rear obstacle). A blocked
        cardinal slide escape returns "blocked"; diagonal modes slide sideways.
    movement_axis : str, optional
        "longitudinal" for forward/backward travel or "lateral" for a slide.
    """
    longitudinal = speed if end == "rear" else -speed
    prefix = "front" if end == "rear" else "back"
    right = sides["right"]["detected"] is True
    left = sides["left"]["detected"] is True

    if not right and not left:
        return "clear", 0.0, 0.0

    diag = speed / math.sqrt(2.0)

    if mode == "adaptive":
        # Every third attempt prefers a diagonal escape when clear.
        # ChassisController still checks each candidate direction.
        if attempt % 3 == 0 and forward_clear:
            mode = "diagonal"
            attempt = 1
        else:
            mode = "cardinal"

    # Both sides blocked: escape purely longitudinally if clear, or fall back to lateral slide
    if right and left:
        if forward_clear:
            return ("forward" if end == "rear" else "backward"), longitudinal, 0.0
        if attempt % 2 == 0:
            return "slide_right", 0.0, speed
        return "slide_left", 0.0, -speed

    # Alternate cardinal axes between attempts so a blocked first escape does
    # not repeat forever in the same direction.
    if mode == "cardinal":
        longitudinal_attempt = ((movement_axis == "lateral" and attempt % 2 == 1) or
                                (movement_axis != "lateral" and attempt % 2 == 0))
        if longitudinal_attempt:
            if not forward_clear:
                return "blocked", 0.0, 0.0
            return ("forward" if end == "rear" else "backward"), longitudinal, 0.0
        if left:
            return "slide_right", 0.0, speed
        return "slide_left", 0.0, -speed

    # 3. Mode: "diagonal" (compound escape: moves away longitudinally and laterally)
    if mode == "diagonal":
        if forward_clear:
            if attempt == 2:
                # Attempt 2: pull straight longitudinally
                return ("forward" if end == "rear" else "backward"), longitudinal, 0.0
            # Attempt 1 or >= 3: diagonal compound motion
            if left:
                return (prefix + "_right", diag if end == "rear" else -diag, diag)
            else:
                return (prefix + "_left", diag if end == "rear" else -diag, -diag)
        else:
            # Longitudinal path blocked by wall: slide laterally
            if left:
                return "slide_right", 0.0, speed
            else:
                return "slide_left", 0.0, -speed

    # 4. Mode: "forward_first" (longitudinal first if clear, then diagonal)
    if mode == "forward_first":
        if forward_clear and attempt == 1:
            return ("forward" if end == "rear" else "backward"), longitudinal, 0.0
        if forward_clear and attempt >= 2:
            if left:
                return (prefix + "_right", diag if end == "rear" else -diag, diag)
            else:
                return (prefix + "_left", diag if end == "rear" else -diag, -diag)
        if left:
            return "slide_right", 0.0, speed
        else:
            return "slide_left", 0.0, -speed

    # 5. Mode: "staged" (legacy staged behavior)
    if attempt == 2:
        return ("forward" if end == "rear" else "backward"), longitudinal, 0.0
    if attempt >= 3:
        return (prefix + "_right", diag if end == "rear" else -diag, diag) if left else (
            prefix + "_left", diag if end == "rear" else -diag, -diag)
    if left:
        return "slide_right", 0.0, speed
    if right:
        return "slide_left", 0.0, -speed
    return "clear", 0.0, 0.0


class IRBumper:
    def __init__(self, logger, settings, end):
        self.logger = logger
        self.settings = settings
        self.end = end
        self.last_block = None
        self.recovering = None
        self.events = []
        self._direct_lock = threading.Lock()
        self._direct_cache = None
        self._direct_cache_time = 0.0
        self._calibration_counts = {
            side: {0: 0, 1: 0} for side in ("right", "left")
        }
        self._calibration_seen = 0
        self._calibration_last_timestamp = None
        self._calibrated_active_io = {}
        self._calibration_complete = not self.settings.get("auto_calibrate_io", False)

    @staticmethod
    def _stream_io_is_invalid(values):
        """Detect the post-power-cycle DDS failure: all or almost all IO=0 while ADC is alive."""
        if len(values) < 24:
            return False
        io_values = values[:12]
        adc_values = values[12:24]
        live_adcs = sum(1 for value in adc_values if isinstance(value, (int, float)) and value > 0)
        if live_adcs == 0:
            return False
        zero_ios = sum(1 for value in io_values if type(value) is int and value == 0)
        return zero_ios == 12 or (zero_ios >= 10 and live_adcs >= 4)

    def _stream_reports_detection(self, values):
        """Return True when DDS claims any configured bumper is active.

        A partially recovered DDS stream can contain one valid high bit while
        leaving the remaining IO values falsely low. Confirm every reported
        contact through synchronous get_io() before blocking motion.
        """
        for side in ("right", "left"):
            port = self.settings[side]
            index = adapter_index(port["id"], port["port"])
            active_io = port.get("active_io", self.settings.get("active_io"))
            if (index < len(values) and active_io in (0, 1) and
                    type(values[index]) is int and values[index] == active_io):
                return True
        return False

    def _read_direct_io(self):
        """Read configured ports synchronously, with a short shared cache."""
        cache_s = float(self.settings.get("direct_fallback_cache_s", 0.2))
        now = time.time()
        with self._direct_lock:
            if self._direct_cache is not None and now - self._direct_cache_time < cache_s:
                return self._direct_cache, self._direct_cache_time

            adapter = getattr(getattr(self.logger, "robot", None), "sensor_adaptor", None)
            readings = {}
            for side in ("right", "left"):
                port = self.settings[side]
                try:
                    value = adapter.get_io(id=port["id"], port=port["port"])
                except Exception:
                    value = None
                readings[side] = value if type(value) is int and value in (0, 1) else None

            self._direct_cache = readings
            self._direct_cache_time = now
            return readings, now

    def snapshot(self):
        sample = self.logger.get_sample("adapter", max_age_s=self.settings["max_age_s"])
        result = {"enabled": True, "state": "ready", "blocked": self.last_block,
                  "recovering": self.recovering,
                  "sides": {}}
        if sample is None:
            result["state"] = "waiting_data"
            for side in ("right", "left"):
                result["sides"][side] = {"io": None, "detected": None}
            return result

        values, timestamp = sample
        direct_values = None
        io_source = "stream"
        read_mode = self.settings.get("io_read_mode", "auto")
        use_direct = (read_mode == "direct" or
                      (read_mode == "auto" and
                       self.settings.get("direct_io_fallback", True) and
                       (self._stream_io_is_invalid(values) or
                        self._stream_reports_detection(values))))
        if use_direct:
            direct_values, timestamp = self._read_direct_io()
            io_source = "direct" if read_mode == "direct" else "direct_fallback"
        result["io_source"] = io_source
        result["sample_time"] = timestamp
        raw_values = {}
        for side in ("right", "left"):
            port = self.settings[side]
            index = adapter_index(port["id"], port["port"])
            value = (direct_values.get(side) if direct_values is not None else
                     values[index] if index < len(values) else None)
            raw_values[side] = value

        if (not self._calibration_complete and
                timestamp != self._calibration_last_timestamp and
                all(type(value) is int and value in (0, 1)
                    for value in raw_values.values())):
            for side, value in raw_values.items():
                self._calibration_counts[side][value] += 1
            self._calibration_seen += 1
            self._calibration_last_timestamp = timestamp

            required = int(self.settings.get("calibration_samples", 10))
            consistency = float(self.settings.get("calibration_min_consistency", 0.8))
            stable = self._calibration_seen >= required and all(
                max(counts.values()) / self._calibration_seen >= consistency
                for counts in self._calibration_counts.values()
            )
            if stable:
                for side, counts in self._calibration_counts.items():
                    clear_io = max((0, 1), key=lambda value: counts[value])
                    self._calibrated_active_io[side] = 1 - clear_io
                self._calibration_complete = True

        if not self._calibration_complete:
            result["state"] = "calibrating"
        result["calibration"] = {
            "enabled": self.settings.get("auto_calibrate_io", False),
            "complete": self._calibration_complete,
            "samples": self._calibration_seen,
            "required_samples": int(self.settings.get("calibration_samples", 10)),
        }

        for side in ("right", "left"):
            port = self.settings[side]
            value = raw_values[side]
            valid = type(value) is int and value in (0, 1)
            active_io = self._calibrated_active_io.get(
                side, port.get("active_io", self.settings.get("active_io"))
            )
            detected = (value == active_io if valid and active_io in (0, 1) and
                        self._calibration_complete else None)
            result["sides"][side] = {
                "io": value if valid else None,
                "active_io": active_io if self._calibration_complete else None,
                "detected": detected,
                "source": io_source,
            }
            if not valid or active_io not in (0, 1):
                result["state"] = "waiting_data"
        if (self.last_block == "waiting_data" and result["state"] == "ready") or (
                self.last_block in ("right", "left") and
                result["sides"][self.last_block]["detected"] is False) or (
                self.last_block == "both" and
                all(sensor["detected"] is False for sensor in result["sides"].values())):
            result["blocked"] = None
        return result

    def blocks_motion(self, x, y, z):
        """Guard motion toward this bumper, sideways travel and yaw."""
        # Do not trap the robot against an obstacle when it translates directly
        # away. Rotation is never exempt because it can sweep a corner inward.
        sliding = abs(y) > abs(x)
        away_speed = x if self.end == "rear" else -x
        # Any yaw can sweep the contacted corner farther into the obstacle, so
        # only bypass this bumper for a purely translating escape.
        if not sliding and away_speed > 0 and away_speed >= abs(y) and z == 0:
            self.last_block = None
            return False

        state = self.snapshot()
        toward = x < 0 if self.end == "rear" else x > 0
        relevant = (("right", sliding or toward or y > 0 or z != 0),
                    ("left", sliding or toward or y < 0 or z != 0))
        detected = [side for side, hazardous in relevant
                    if hazardous and state["sides"][side]["detected"] is True]
        if detected:
            self.last_block = "both" if len(detected) == 2 else detected[0]
            return True
        if any(hazardous and state["sides"][side]["detected"] is None
               for side, hazardous in relevant):
            self.last_block = "waiting_data"
            return True
        self.last_block = None
        return False

    def finish_recovery(self, side, status, distance_m, reason=None):
        """Expose the outcome to the live dashboard and saved run summary."""
        direction = self.recovering
        self.recovering = None
        self.events.append({
            "end": self.end,
            "side": side, "status": status,
            "direction": direction if direction != side else None,
            "elapsed_s": round(time.time() - self.logger.start_time, 3),
            "distance_m": round(distance_m, 3), "reason": reason,
        })


class RearIRBumper(IRBumper):
    def __init__(self, logger, settings):
        super().__init__(logger, settings, "rear")


class FrontIRBumper(IRBumper):
    def __init__(self, logger, settings):
        super().__init__(logger, settings, "front")
