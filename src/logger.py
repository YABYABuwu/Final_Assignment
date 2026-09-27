"""Subscribe to basic RoboMaster EP telemetry and optionally save CSV files."""

import csv
import json
import queue
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path


# name: (robot module, subscribe method, unsubscribe method, CSV column names)
STREAMS = {
    "position": ("chassis", "sub_position", "unsub_position", ("x_m", "y_m", "z_deg")),
    "attitude": ("chassis", "sub_attitude", "unsub_attitude", ("yaw_deg", "pitch_deg", "roll_deg")),
    "imu": ("chassis", "sub_imu", "unsub_imu", ("acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z")),
    "esc": ("chassis", "sub_esc", "unsub_esc", ("speed", "angle", "esc_time", "state")),
    "status": ("chassis", "sub_status", "unsub_status", ("static", "up_hill", "down_hill", "on_slope", "picked_up", "slip", "impact_x", "impact_y", "impact_z", "roll_over", "hill_static")),
    "tof": ("sensor", "sub_distance", "unsub_distance", ("tof_0_mm", "tof_1_mm", "tof_2_mm", "tof_3_mm")),
    "adapter": ("sensor_adaptor", "sub_adapter", "unsub_adapter", tuple(f"io_{n}" for n in range(1, 13)) + tuple(f"adc_{n}" for n in range(1, 13))),
    "battery": ("battery", "sub_battery_info", "unsub_battery_info", ("percent",)),
    "gimbal": ("gimbal", "sub_angle", "unsub_angle", ("pitch_deg", "yaw_deg", "pitch_ground_deg", "yaw_ground_deg")),
}


class SensorLogger:
    """Keep the latest readings; write only streams whose save setting is true."""

    def __init__(self, robot, settings):
        self.robot = robot
        self.stream_settings = settings["streams"]
        self.position_cs = settings.get("position_cs", 0)
        self.output_dir = Path(settings["directory"])
        self.history_max_samples = settings.get("history_max_samples", 2000)
        self.lock = threading.Lock()
        self.latest = {}
        self.history = {name: deque(maxlen=self.history_max_samples)
                        for name in self.stream_settings}
        self.sample_id = 0
        self.active = []
        self.start_time = time.time()
        self.run_dir = None
        self.pending = queue.Queue(maxsize=settings.get("queue_max_rows", 1000))
        self.batch_size = settings.get("batch_size", 50)
        self.flush_interval_s = settings.get("flush_interval_s", 0.25)
        self.writer_thread = None
        self.stop_writer = threading.Event()
        self.accepting = False
        self.dropped_rows = 0
        self.write_error = None
        self.run_status = "running"
        self.run_error = None
        self.exploration_state = None
        self.received_rows = {name: 0 for name in self.stream_settings}

    def _callback(self, name, data):
        timestamp = time.time()
        if name == "adapter":
            # SDK sends (IO values, ADC values); copy both lists immediately.
            values = tuple(data[0]) + tuple(data[1])
        elif isinstance(data, (list, tuple)):
            # Copy nested SDK arrays before the decoder reuses them.
            values = tuple(tuple(item) if isinstance(item, (list, tuple)) else item
                           for item in data)
        else:
            values = (data,)

        with self.lock:
            self.latest[name] = (values, timestamp)
            self.sample_id += 1
            elapsed = round(timestamp - self.start_time, 3)
            self.history[name].append((self.sample_id, elapsed, values))
            self.received_rows[name] += 1
            if self.accepting and self.stream_settings[name]["save"]:
                row = [round(timestamp, 3), elapsed, *values]
                try:
                    self.pending.put_nowait((name, row))
                except queue.Full:
                    self.dropped_rows += 1

    def _write_batch(self, batch):
        """Write a group of samples, opening each CSV at most once."""
        grouped = {}
        for name, row in batch:
            grouped.setdefault(name, []).append(row)
        for name, rows in grouped.items():
            path = self.run_dir / f"{name}.csv"
            with path.open("a", newline="", encoding="utf-8") as file:
                csv.writer(file).writerows(rows)

    def _write_loop(self):
        """Drain queued rows in a separate thread, including on shutdown."""
        while not self.stop_writer.is_set() or not self.pending.empty():
            try:
                first = self.pending.get(timeout=self.flush_interval_s)
            except queue.Empty:
                continue
            batch = [first]
            deadline = time.monotonic() + self.flush_interval_s
            while len(batch) < self.batch_size:
                try:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0 or self.stop_writer.is_set():
                        break
                    batch.append(self.pending.get(timeout=remaining))
                except queue.Empty:
                    break
            try:
                if self.write_error is None:
                    self._write_batch(batch)
            except Exception as error:
                self.write_error = error

    def start(self):
        """Start selected subscriptions. Call once after robot.initialize()."""
        if self.active:
            raise RuntimeError("logger is already started")
        self.run_dir = None
        if any(s["enabled"] and s["save"] for s in self.stream_settings.values()):
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            self.run_dir = self.output_dir / stamp
            self.run_dir.mkdir(parents=True, exist_ok=False)

        self.start_time = time.time()
        with self.lock:
            self.latest.clear()
            self.sample_id = 0
            for samples in self.history.values():
                samples.clear()
        self.dropped_rows = 0
        self.write_error = None
        self.run_status = "running"
        self.run_error = None
        self.exploration_state = None
        self.received_rows = {name: 0 for name in self.stream_settings}
        self.stop_writer.clear()
        self.accepting = True
        if self.run_dir is not None:
            self.writer_thread = threading.Thread(target=self._write_loop, name="csv-writer")
            self.writer_thread.start()
        try:
            for name, settings in self.stream_settings.items():
                if not settings["enabled"]:
                    continue
                module_name, subscribe, _, columns = STREAMS[name]
                module = getattr(self.robot, module_name)
                callback = lambda data, stream=name: self._callback(stream, data)
                options = {"freq": settings["frequency_hz"], "callback": callback}
                if name == "position":
                    options["cs"] = self.position_cs
                if settings["save"]:
                    path = self.run_dir / f"{name}.csv"
                    with path.open("w", newline="", encoding="utf-8") as file:
                        csv.writer(file).writerow(["timestamp", "elapsed_s", *columns])
                result = getattr(module, subscribe)(**options)
                if result is False:
                    raise RuntimeError(f"could not subscribe to {name}")
                self.active.append(name)
        except Exception:
            self.stop()
            raise

    def get_latest(self, name, max_age_s=None):
        """Return a tuple, or None when no fresh sample is available."""
        sample = self.get_sample(name, max_age_s=max_age_s)
        return None if sample is None else sample[0]

    def get_sample(self, name, max_age_s=None):
        """Return ``(values, wall_clock_timestamp)`` for a fresh sample."""
        if name not in STREAMS:
            raise ValueError(f"unknown stream: {name}")
        with self.lock:
            sample = self.latest.get(name)
        if sample is None:
            return None
        values, timestamp = sample
        if max_age_s is not None and time.time() - timestamp > max_age_s:
            return None
        return values, timestamp

    def get_history_since(self, after_id=0):
        """Return recent samples newer than an ID for live dashboard graphs."""
        if type(after_id) is not int or after_id < 0:
            raise ValueError("after_id must be a nonnegative integer")
        with self.lock:
            streams = {
                name: [sample for sample in samples if sample[0] > after_id]
                for name, samples in self.history.items()
                if self.stream_settings[name]["enabled"]
            }
            cursor = self.sample_id
        return {"cursor": cursor, "streams": streams}

    def wait_for(self, name, timeout_s=None):
        """Wait for the first sample; an explicit timeout_s bounds the wait."""
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        while deadline is None or time.monotonic() < deadline:
            if not self.accepting:
                raise RuntimeError("sensor logger stopped before receiving data")
            sample = self.get_latest(name)
            if sample is not None:
                return sample
            time.sleep(0.02)
        raise TimeoutError(f"no {name} data within {timeout_s} s")

    def stop(self):
        """Unsubscribe, then write every queued row before returning."""
        errors = []
        with self.lock:
            self.accepting = False
        for name in reversed(self.active):
            module_name, _, unsubscribe, _ = STREAMS[name]
            try:
                getattr(getattr(self.robot, module_name), unsubscribe)()
            except Exception as error:
                errors.append(f"{name}: {error}")
        self.active.clear()
        self.stop_writer.set()
        if self.writer_thread is not None:
            self.writer_thread.join()
            self.writer_thread = None
        if self.write_error is not None:
            errors.append(f"CSV write: {self.write_error}")
        if self.run_dir is not None:
            summary = {
                "status": self.run_status,
                "error": self.run_error,
                "started_at": self.start_time,
                "ended_at": time.time(),
                "dropped_csv_rows": self.dropped_rows,
                "received_rows": self.received_rows,
                "stream_settings": self.stream_settings,
                "logger_errors": errors,
                "exploration": self.exploration_state,
            }
            temporary_path = self.run_dir / "run_summary.tmp"
            with temporary_path.open("w", encoding="utf-8") as file:
                json.dump(summary, file, ensure_ascii=False, indent=2)
            temporary_path.replace(self.run_dir / "run_summary.json")
        if errors:
            raise RuntimeError("logger stop failed: " + "; ".join(errors))
