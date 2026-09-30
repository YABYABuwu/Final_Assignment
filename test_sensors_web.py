"""Interactive Sensor Test and Calibration Web Server for RoboMaster EP.

Zero motor movement: wheels are unpowered so the robot can be pushed/turned by hand.
Streams camera feed with live target detection, real-time ToF distance gauge,
and IR obstacle bumper (Front & Rear, Left & Right) indicators.

Usage:
  py -3.8 test_sensors_web.py
  py -3.8 test_sensors_web.py --port 8000
  py -3.8 test_sensors_web.py --mock   (for offline UI preview without physical robot)
"""

import argparse
import json
import math
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import cv2
import numpy as np

from src.config_loader import load_config
from src.logger import SensorLogger
from src.rear_ir import FrontIRBumper, RearIRBumper, adapter_index
from src.targets import detect

HTML_FILE = Path(__file__).resolve().parent / "dashboard" / "sensor_test.html"


class SensorTestApp:
    def __init__(self, config, host="0.0.0.0", port=8000, conn_type="ap", mock=False):
        self.config = config
        self.host = host
        self.port = port
        self.conn_type = conn_type
        self.mock = mock

        self.running = threading.Event()
        self.running.set()
        self.frame_condition = threading.Condition()

        self.latest_jpeg = None
        self.latest_frame = None
        self.camera_targets = []
        self.frame_number = 0
        self.fps = 0.0
        self._last_fps_time = time.time()
        self._fps_count = 0

        self.robot = None
        self.logger = None
        self.front_ir = None
        self.rear_ir = None
        self.gimbal_ctrl = None
        self.server = None

        self.camera_error = None
        self.camera_thread = None
        self.server_thread = None

    def start(self):
        if not self.mock:
            self._connect_robot()
        else:
            print("[MOCK MODE] Running without physical robot connection.")

        self._start_web_server()
        self.camera_thread = threading.Thread(target=self._camera_loop, name="camera-thread", daemon=True)
        self.camera_thread.start()

    def _connect_robot(self):
        from robomaster import conn, robot
        from src.gimbal_control import ChassisRelativeGimbal

        conn_map = {
            "ap": conn.CONNECTION_WIFI_AP,
            "sta": conn.CONNECTION_WIFI_STA,
            "rndis": conn.CONNECTION_USB_RNDIS,
        }

        print(f"Connecting to RoboMaster EP via {self.conn_type.upper()}...")
        self.robot = robot.Robot()
        self.robot.initialize(conn_type=conn_map[self.conn_type])

        # Set robot mode to FREE: completely uncouples chassis from gimbal so wheels don't fight rotation
        try:
            self.robot.set_robot_mode(mode=robot.FREE)
            print("Robot mode set to FREE (wheels unlocked, manual rotation enabled).")
        except Exception as e:
            print("Notice setting FREE mode:", e)

        log_settings = {
            "directory": Path(__file__).resolve().parent / "data" / "raw",
            "queue_max_rows": 500,
            "history_max_samples": 500,
            "batch_size": 20,
            "flush_interval_s": 0.5,
            "streams": {
                "position": {"enabled": True, "save": False, "frequency_hz": 20},
                "attitude": {"enabled": True, "save": False, "frequency_hz": 20},
                "tof": {"enabled": True, "save": False, "frequency_hz": 20},
                "adapter": {"enabled": True, "save": False, "frequency_hz": 20},
                "battery": {"enabled": True, "save": False, "frequency_hz": 5},
                "gimbal": {"enabled": True, "save": False, "frequency_hz": 10},
                "status": {"enabled": True, "save": False, "frequency_hz": 5},
            },
        }

        self.logger = SensorLogger(self.robot, log_settings)
        self.logger.start()

        # Front & Rear IR Bumpers
        self.front_ir = FrontIRBumper(self.logger, self.config["front_ir"])
        self.rear_ir = RearIRBumper(self.logger, self.config["rear_ir"])

        # Chassis Relative Gimbal Controller
        self.gimbal_ctrl = ChassisRelativeGimbal(self.robot.gimbal)

        # Start Camera Stream
        res = self.config["dashboard"].get("resolution", "360p")
        print(f"Starting camera stream (resolution: {res})...")
        self.robot.camera.start_video_stream(display=False, resolution=res)
        print("Robot initialized successfully!")

    def _camera_loop(self):
        target_cfg = self.config["exploration"]["target_inspection"]
        color_ranges = self.config["color_ranges"]
        min_area_frac = target_cfg.get("min_area_fraction", 0.02)
        aim_off_x = target_cfg.get("aim_offset_x_fraction", 0.0)
        aim_off_y = target_cfg.get("aim_offset_y_fraction", 0.20)
        jpeg_quality = self.config["dashboard"].get("jpeg_quality", 75)

        box_colors = {
            "red": (55, 55, 255),
            "green": (80, 240, 80),
            "yellow": (60, 220, 255),
            "blue": (255, 140, 70),
        }

        mock_angle = 0.0

        while self.running.is_set():
            started = time.monotonic()
            image = None

            if self.mock:
                # Generate mock frame with demo colored shapes
                h, w = 360, 640
                image = np.zeros((h, w, 3), dtype=np.uint8)
                image[:] = (20, 25, 30)

                mock_angle += 0.04
                cx1 = int(240 + 70 * math.cos(mock_angle))
                cy1 = int(180 + 40 * math.sin(mock_angle))
                cv2.circle(image, (cx1, cy1), 40, (0, 0, 230), -1)  # Red Circle

                cx2 = int(450 - 50 * math.cos(mock_angle))
                cy2 = int(170 + 50 * math.sin(mock_angle))
                cv2.rectangle(image, (cx2 - 35, cy2 - 35), (cx2 + 35, cy2 + 35), (40, 210, 40), -1)  # Green Square

                cv2.putText(image, "MOCK CAMERA STREAM - FOR TESTING", (120, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            else:
                try:
                    image = self.robot.camera.read_cv2_image(timeout=1, strategy="newest")
                except Exception as e:
                    self.camera_error = str(e)
                    time.sleep(0.1)
                    continue

            if image is not None:
                h, w = image.shape[:2]
                annotated = image.copy()
                targets = []

                # Target Detection
                try:
                    detections = detect(image, min_area_frac, selected=None, color_ranges=color_ranges)
                    for item in detections:
                        x, y, bw, bh = cv2.boundingRect(item.contour)
                        bcolor = box_colors.get(item.color, (255, 255, 255))
                        cv2.rectangle(annotated, (x, y), (x + bw, y + bh), bcolor, 2)
                        cv2.putText(
                            annotated,
                            f"{item.color} {item.shape}",
                            (x, max(18, y - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            bcolor,
                            1,
                            cv2.LINE_AA,
                        )
                        cv2.circle(annotated, item.center, 4, bcolor, -1)

                        norm_x = (item.center[0] - (w / 2.0)) / (w / 2.0)
                        norm_y = (item.center[1] - (h / 2.0)) / (h / 2.0)
                        targets.append({
                            "color": item.color,
                            "shape": item.shape,
                            "box": [x, y, bw, bh],
                            "center": list(item.center),
                            "area_px2": float(item.area),
                            "area_fraction": round(item.area / (w * h), 4),
                            "offset_x_norm": round(norm_x, 3),
                            "offset_y_norm": round(norm_y, 3),
                        })
                except Exception:
                    pass

                # Draw Aiming Crosshair with configured offset
                aim_px_x = int(w / 2.0 + aim_off_x * (w / 2.0))
                aim_px_y = int(h / 2.0 + aim_off_y * (h / 2.0))
                cv2.drawMarker(
                    annotated,
                    (aim_px_x, aim_px_y),
                    (0, 255, 200),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=30,
                    thickness=1,
                )
                cv2.circle(annotated, (aim_px_x, aim_px_y), 8, (0, 255, 200), 1)

                ok, encoded = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
                if ok:
                    with self.frame_condition:
                        self.latest_jpeg = encoded.tobytes()
                        self.latest_frame = annotated
                        self.camera_targets = targets
                        self.frame_number += 1
                        self._fps_count += 1
                        now = time.time()
                        if now - self._last_fps_time >= 1.0:
                            self.fps = round(self._fps_count / (now - self._last_fps_time), 1)
                            self._fps_count = 0
                            self._last_fps_time = now
                        self.frame_condition.notify_all()

            elapsed = time.monotonic() - started
            delay = max(0.005, (1.0 / 25.0) - elapsed)
            time.sleep(delay)

    def _get_sensor_snapshot(self):
        exp_cfg = self.config["exploration"]
        wall_thresh = exp_cfg.get("wall_threshold_mm", 500)
        emg_stop_m = exp_cfg.get("emergency_stop_distance_m", 0.25)
        align_dist_m = exp_cfg["alignment"].get("wall_distance_m", 0.28)
        clearance_m = exp_cfg["map"].get("robot_clearance_m", 0.20)
        tof_chan = exp_cfg["sensor"].get("tof_channel", 0)

        if self.mock:
            # Mock sensor readings
            t = time.time()
            mock_dist = 380 + int(150 * math.sin(t * 0.8))
            mock_io_front_r = 1 if math.sin(t * 1.5) > 0.5 else 0
            mock_io_rear_l = 0 if math.cos(t * 1.2) > 0.4 else 1

            return {
                "tof": {
                    "distance_mm": mock_dist,
                    "channels": [mock_dist, mock_dist + 40, None, None],
                    "active_channel": 0,
                    "wall_threshold_mm": wall_thresh,
                    "emergency_stop_distance_mm": emg_stop_m * 1000,
                    "alignment_distance_mm": align_dist_m * 1000,
                    "robot_clearance_mm": clearance_m * 1000,
                },
                "ir": {
                    "front": {
                        "left": {"detected": False, "io": 1, "active_io": 0, "adc": 850, "id": 4, "port": 2},
                        "right": {"detected": (mock_io_front_r == 1), "io": mock_io_front_r, "active_io": 1, "adc": 120, "id": 1, "port": 1},
                    },
                    "rear": {
                        "left": {"detected": (mock_io_rear_l == 0), "io": mock_io_rear_l, "active_io": 0, "adc": 90, "id": 4, "port": 1},
                        "right": {"detected": False, "io": 0, "active_io": 1, "adc": 820, "id": 3, "port": 2},
                    },
                },
                "gimbal": {
                    "yaw_chassis_deg": 0.0,
                    "pitch_chassis_deg": 0.0,
                    "yaw_ground_deg": 0.0,
                    "pitch_ground_deg": 0.0,
                },
                "odometry": {
                    "x_m": round(0.15 * math.sin(t * 0.2), 3),
                    "y_m": round(0.08 * math.cos(t * 0.2), 3),
                    "yaw_deg": round(12.5 * math.sin(t * 0.1), 1),
                    "pitch_deg": 0.4,
                    "roll_deg": -0.2,
                },
                "battery": {"percent": 88},
                "camera": {
                    "fps": self.fps or 20.0,
                    "targets": self.camera_targets,
                },
                "config_refs": {
                    "wall_threshold_mm": wall_thresh,
                    "emergency_stop_distance_m": emg_stop_m,
                    "alignment_wall_distance_m": align_dist_m,
                },
            }

        # Real Robot Sensor Readings
        tof_sample = self.logger.get_latest("tof")
        tof_val = None
        tof_channels = [None, None, None, None]
        if tof_sample:
            tof_channels = list(tof_sample)
            if tof_chan < len(tof_channels):
                tof_val = tof_channels[tof_chan]

        # Front IR
        front_snap = self.front_ir.snapshot() if self.front_ir else {}
        rear_snap = self.rear_ir.snapshot() if self.rear_ir else {}

        adapter_sample = self.logger.get_latest("adapter")

        def parse_ir_side(snap, end, side):
            cfg = self.config[f"{end}_ir"][side]
            idx = adapter_index(cfg["id"], cfg["port"])
            io_val = None
            adc_val = None
            if adapter_sample:
                if idx < 12 and idx < len(adapter_sample):
                    io_val = adapter_sample[idx]
                if 12 + idx < len(adapter_sample):
                    adc_val = adapter_sample[12 + idx]

            side_data = snap.get("sides", {}).get(side, {})
            return {
                "detected": side_data.get("detected"),
                "io": side_data.get("io") if side_data.get("io") is not None else io_val,
                "active_io": side_data.get("active_io", cfg.get("active_io")),
                "adc": adc_val,
                "id": cfg["id"],
                "port": cfg["port"],
            }

        front_left = parse_ir_side(front_snap, "front", "left")
        front_right = parse_ir_side(front_snap, "front", "right")
        rear_left = parse_ir_side(rear_snap, "rear", "left")
        rear_right = parse_ir_side(rear_snap, "rear", "right")

        # Gimbal
        gimbal_sample = self.logger.get_latest("gimbal")
        pitch_chassis = float(gimbal_sample[0]) if gimbal_sample else 0.0
        yaw_chassis = float(gimbal_sample[1]) if gimbal_sample else 0.0
        pitch_ground = float(gimbal_sample[2]) if gimbal_sample and len(gimbal_sample) > 2 else 0.0
        yaw_ground = float(gimbal_sample[3]) if gimbal_sample and len(gimbal_sample) > 3 else 0.0

        # Odometry / Hand Motion
        pos_sample = self.logger.get_latest("position")
        x_m = float(pos_sample[0]) if pos_sample else 0.0
        y_m = float(pos_sample[1]) if pos_sample else 0.0

        att_sample = self.logger.get_latest("attitude")
        yaw_deg = float(att_sample[0]) if att_sample else 0.0
        pitch_deg = float(att_sample[1]) if att_sample else 0.0
        roll_deg = float(att_sample[2]) if att_sample else 0.0

        # Battery
        bat_sample = self.logger.get_latest("battery")
        bat_pct = int(bat_sample[0]) if bat_sample else 100

        return {
            "tof": {
                "distance_mm": tof_val,
                "channels": tof_channels,
                "active_channel": tof_chan,
                "wall_threshold_mm": wall_thresh,
                "emergency_stop_distance_mm": emg_stop_m * 1000,
                "alignment_distance_mm": align_dist_m * 1000,
                "robot_clearance_mm": clearance_m * 1000,
            },
            "ir": {
                "front": {"left": front_left, "right": front_right},
                "rear": {"left": rear_left, "right": rear_right},
            },
            "gimbal": {
                "yaw_chassis_deg": yaw_chassis,
                "pitch_chassis_deg": pitch_chassis,
                "yaw_ground_deg": yaw_ground,
                "pitch_ground_deg": pitch_ground,
            },
            "odometry": {
                "x_m": x_m,
                "y_m": y_m,
                "yaw_deg": yaw_deg,
                "pitch_deg": pitch_deg,
                "roll_deg": roll_deg,
            },
            "battery": {"percent": bat_pct},
            "camera": {
                "fps": self.fps,
                "targets": self.camera_targets,
            },
            "config_refs": {
                "wall_threshold_mm": wall_thresh,
                "emergency_stop_distance_m": emg_stop_m,
                "alignment_wall_distance_m": align_dist_m,
            },
        }

    def _start_web_server(self):
        app = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                route = urlsplit(self.path).path
                if route == "/":
                    content = HTML_FILE.read_bytes()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(content)))
                    self.end_headers()
                    self.wfile.write(content)
                elif route == "/api/sensors":
                    data = app._get_sensor_snapshot()
                    body = json.dumps(data, ensure_ascii=False).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(body)
                elif route == "/video":
                    self._video_stream()
                else:
                    self.send_error(404)

            def do_POST(self):
                route = urlsplit(self.path).path
                if route == "/api/gimbal/move":
                    length = int(self.headers.get("Content-Length", "0"))
                    payload = json.loads(self.rfile.read(length).decode("utf-8"))
                    pitch = float(payload.get("pitch", 0.0))
                    yaw = float(payload.get("yaw", 0.0))
                    if app.gimbal_ctrl and not app.mock:
                        app.gimbal_ctrl.moveto(pitch=pitch, yaw=yaw, pitch_speed=35, yaw_speed=45)
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(b'{"ok":true}')
                elif route == "/api/gimbal/recenter":
                    if app.gimbal_ctrl and not app.mock:
                        app.gimbal_ctrl.recenter()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(b'{"ok":true}')
                elif route == "/api/robot/unlock":
                    if app.robot and not app.mock:
                        from robomaster import robot
                        app.robot.set_robot_mode(mode=robot.FREE)
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(b'{"ok":true,"mode":"free"}')
                elif route == "/api/gimbal/suspend":
                    if app.robot and not app.mock:
                        app.robot.gimbal.suspend()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(b'{"ok":true,"gimbal":"suspended"}')
                elif route == "/api/gimbal/resume":
                    if app.robot and not app.mock:
                        app.robot.gimbal.resume()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(b'{"ok":true,"gimbal":"resumed"}')
                else:
                    self.send_error(404)

            def _video_stream(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last_frame = 0
                try:
                    while app.running.is_set():
                        with app.frame_condition:
                            app.frame_condition.wait_for(
                                lambda: app.frame_number != last_frame or not app.running.is_set(),
                                timeout=0.5,
                            )
                            if app.frame_number == last_frame:
                                continue
                            jpeg = app.latest_jpeg
                            last_frame = app.frame_number

                        if jpeg:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                            self.wfile.write(jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, format, *args):
                pass

        self.server = ThreadingHTTPServer((self.host, self.port), Handler)
        self.server.daemon_threads = True
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        print(f"Sensor Dashboard running at: http://127.0.0.1:{self.port} (and http://{self.host}:{self.port})")

    def stop(self):
        print("\nStopping Sensor Test Dashboard...")
        self.running.clear()
        with self.frame_condition:
            self.frame_condition.notify_all()

        if self.server:
            self.server.shutdown()
            self.server.server_close()

        if self.robot and not self.mock:
            try:
                self.robot.camera.stop_video_stream()
            except Exception:
                pass
            if self.logger:
                self.logger.stop()
            try:
                self.robot.close()
            except Exception:
                pass
        print("Done. Goodbye!")


def main():
    parser = argparse.ArgumentParser(description="RoboMaster EP Sensor Test & Calibration Dashboard")
    parser.add_argument("--port", type=int, default=8000, help="Web dashboard port (default: 8000)")
    parser.add_argument("--host", default="0.0.0.0", help="Web dashboard host (default: 0.0.0.0)")
    parser.add_argument("--connection", choices=["ap", "sta", "rndis"], default=None,
                        help="Connection type: ap (Wi-Fi AP), sta (Router), or rndis (USB). Defaults to settings.yaml")
    parser.add_argument("--mock", action="store_true",
                        help="Simulate sensors without connecting to physical robot (for offline UI test)")
    args = parser.parse_args()

    config = load_config()
    conn_type = args.connection or config["connection"]["type"]

    app = SensorTestApp(
        config=config,
        host=args.host,
        port=args.port,
        conn_type=conn_type,
        mock=args.mock,
    )

    app.start()

    print("\n" + "=" * 65)
    print("  RoboMaster EP Sensor Test & Calibration Dashboard is LIVE!")
    print(f"  -> Open in your browser: http://localhost:{args.port}")
    print("  * Robot wheels are IDLE (safe to push and move by hand)")
    print("  * Press Ctrl+C in this terminal to exit safely")
    print("=" * 65 + "\n")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        app.stop()


if __name__ == "__main__":
    main()
