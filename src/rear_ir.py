"""Rear diagonal IR bumper using the shared SensorLogger adapter stream."""

import math
import time


def adapter_index(sensor_id, port):
    """SDK adapter order: ID 1 port 1/2, ID 2 port 1/2, ..."""
    return (sensor_id - 1) * 2 + (port - 1)


def recovery_vector(sides, speed, attempt=1, end="rear"):
    """Choose a staged escape away from a detected front or rear IR."""
    longitudinal = speed if end == "rear" else -speed
    prefix = "front" if end == "rear" else "back"
    right = sides["right"]["detected"] is True
    left = sides["left"]["detected"] is True
    if right and left:
        return ("forward" if end == "rear" else "backward"), longitudinal, 0.0
    if (right or left) and attempt == 2:
        return ("forward" if end == "rear" else "backward"), longitudinal, 0.0
    if (right or left) and attempt >= 3:
        diagonal = speed / math.sqrt(2.0)
        return (prefix + "_right", diagonal if end == "rear" else -diagonal, diagonal) if left else (
            prefix + "_left", diagonal if end == "rear" else -diagonal, -diagonal)
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
        result["sample_time"] = timestamp
        for side in ("right", "left"):
            port = self.settings[side]
            index = adapter_index(port["id"], port["port"])
            value = values[index] if index < len(values) else None
            valid = type(value) is int and value in (0, 1)
            active_io = port.get("active_io", self.settings.get("active_io"))
            result["sides"][side] = {
                "io": value if valid else None,
                "detected": value == active_io if valid and active_io in (0, 1) else None,
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
        state = self.snapshot()
        toward = x < 0 if self.end == "rear" else x > 0
        relevant = (("right", toward or y > 0 or z != 0),
                    ("left", toward or y < 0 or z != 0))
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
        self.recovering = None
        self.events.append({
            "end": self.end,
            "side": side, "status": status,
            "elapsed_s": round(time.time() - self.logger.start_time, 3),
            "distance_m": round(distance_m, 3), "reason": reason,
        })


class RearIRBumper(IRBumper):
    def __init__(self, logger, settings):
        super().__init__(logger, settings, "rear")


class FrontIRBumper(IRBumper):
    def __init__(self, logger, settings):
        super().__init__(logger, settings, "front")
