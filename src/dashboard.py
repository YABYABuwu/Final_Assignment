"""Local dashboard for live robot telemetry and camera images."""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from src.logger import STREAMS
from src.mission_stop import MissionStop


PAGE = Path(__file__).resolve().parent.parent / "dashboard" / "index.html"


class Dashboard:
    def __init__(self, robot, logger, settings, slam_map=None, slam_worker=None,
                 explorer=None, motion_settings=None, rear_ir=None, front_ir=None):
        self.camera = robot.camera
        self.logger = logger
        self.settings = settings
        self.slam_map = slam_map
        self.slam_worker = slam_worker
        self.explorer = explorer
        self.motion_settings = motion_settings
        self.rear_ir = rear_ir
        self.front_ir = front_ir
        self.running = threading.Event()
        self.frame_changed = threading.Condition()
        self.latest_jpeg = None
        self.latest_frame = None
        self.frame_number = 0
        self.camera_error = None
        self.mission_status = "Ready"
        self.camera_thread = None
        self.server_thread = None
        self.server = None
        self.camera_started = False

    def _camera_loop(self):
        """Only this thread reads frames from the SDK camera."""
        import cv2

        period = 1 / self.settings["max_fps"]
        while self.running.is_set():
            started = time.monotonic()
            try:
                image = self.camera.read_cv2_image(timeout=1, strategy="newest")
                if image is not None:
                    ok, encoded = cv2.imencode(
                        ".jpg", image,
                        [cv2.IMWRITE_JPEG_QUALITY, self.settings["jpeg_quality"]],
                    )
                    if ok:
                        with self.frame_changed:
                            self.latest_jpeg = encoded.tobytes()
                            self.latest_frame = image.copy() if hasattr(image, "copy") else image
                            self.frame_number += 1
                            self.frame_changed.notify_all()
            except Exception as error:
                self.camera_error = str(error)
                self.running.clear()
                with self.frame_changed:
                    self.frame_changed.notify_all()
                break
            remaining = period - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)

    def wait_for_frame(self, after_number=0, check_health=None):
        """Return a fresh BGR frame without starting a second SDK camera reader."""
        with self.frame_changed:
            while self.running.is_set() and self.camera_error is None:
                if check_health is not None:
                    check_health()
                if self.frame_number > after_number and self.latest_frame is not None:
                    frame = self.latest_frame
                    return self.frame_number, frame.copy() if hasattr(frame, "copy") else frame
                self.frame_changed.wait(0.1)
        raise MissionStop(self.camera_error or "camera stream stopped during target inspection")

    def current_frame_number(self):
        with self.frame_changed:
            return self.frame_number

    def snapshot(self):
        """Build a JSON friendly snapshot without touching the camera or disk."""
        streams = {}
        for name in STREAMS:
            if self.logger.stream_settings.get(name, {}).get("enabled"):
                streams[name] = self.logger.get_latest(name, max_age_s=2)
        return {
            "streams": streams,
            "dropped_csv_rows": self.logger.dropped_rows,
            "camera_ready": self.latest_jpeg is not None,
            "camera_error": self.camera_error,
            "mission_status": self.mission_status,
            "motion_settings": self.motion_settings,
            "rear_ir": self.rear_ir.snapshot() if self.rear_ir is not None else {"enabled": False},
            "front_ir": self.front_ir.snapshot() if self.front_ir is not None else {"enabled": False},
            "slam": self.slam_worker.status() if self.slam_worker is not None else None,
            "exploration": self.explorer.snapshot() if self.explorer is not None else None,
        }

    def map_snapshot(self):
        if self.slam_map is None:
            return {"format": "robomaster-occupancy-grid", "version": 1,
                    "width": 0, "height": 0, "data": [], "pose": None, "has_map": False,
                    "trajectory": [], "exploration": {"status": "disabled"}}
        return self.slam_map.to_dict()

    def map_export(self, format_name):
        if self.slam_map is None:
            raise ValueError("SLAM map is unavailable")
        if format_name == "json":
            content = json.dumps(self.slam_map.to_dict(), ensure_ascii=False).encode("utf-8")
            return "application/json; charset=utf-8", content, "robomaster-map.json"
        if format_name == "ros":
            return "application/zip", self.slam_map.ros_map_archive(), "robomaster-ros-map.zip"
        if format_name == "png":
            return "image/png", self.slam_map.png_bytes(), "robomaster-map.png"
        if format_name == "grid-png":
            return "image/png", self.slam_map.grid_png_bytes(), "robomaster-grid-map.png"
        raise ValueError("format must be json, png, grid-png or ros")

    def import_map(self, document):
        if self.slam_map is None:
            raise ValueError("SLAM map is unavailable")
        if self.slam_worker is not None and self.slam_worker.is_running:
            raise ValueError("stop exploration before replacing the live map")
        self.slam_map.load_dict(document)

    def history(self, after_id=0):
        """Return new plot points and labels for each enabled stream."""
        result = self.logger.get_history_since(after_id)
        result["columns"] = {
            name: STREAMS[name][3] for name in result["streams"]
        }
        return result

    def _handler_class(self):
        dashboard = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                route = urlsplit(self.path).path
                if route == "/":
                    content = PAGE.read_bytes()
                    self._send_content("text/html; charset=utf-8", content)
                elif route == "/api/status":
                    content = json.dumps(dashboard.snapshot()).encode("utf-8")
                    self._send_content("application/json; charset=utf-8", content)
                elif route == "/api/history":
                    query = parse_qs(urlsplit(self.path).query)
                    try:
                        after_id = int(query.get("since", ["0"])[0])
                        content = json.dumps(dashboard.history(after_id)).encode("utf-8")
                    except ValueError:
                        self.send_error(400, "since must be a nonnegative integer")
                        return
                    self._send_content("application/json; charset=utf-8", content)
                elif route == "/api/map":
                    content = json.dumps(dashboard.map_snapshot()).encode("utf-8")
                    self._send_content("application/json; charset=utf-8", content)
                elif route == "/api/map/export":
                    query = parse_qs(urlsplit(self.path).query)
                    try:
                        content_type, content, filename = dashboard.map_export(
                            query.get("format", ["json"])[0]
                        )
                    except ValueError as error:
                        self.send_error(400, str(error))
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", content_type)
                    self.send_header("Content-Length", str(len(content)))
                    self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(content)
                elif route == "/video":
                    self._video_stream()
                else:
                    self.send_error(404)

            def do_POST(self):
                if urlsplit(self.path).path != "/api/map/import":
                    self.send_error(404)
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if length <= 0 or length > 10_000_000:
                        self.send_error(413, "map JSON must be between 1 byte and 10 MB")
                        return
                    document = json.loads(self.rfile.read(length).decode("utf-8"))
                    dashboard.import_map(document)
                except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
                    self.send_error(400, str(error))
                    return
                self._send_content("application/json; charset=utf-8", b'{"loaded":true}')

            def _send_content(self, content_type, content):
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(content)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(content)

            def _video_stream(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last_number = 0
                try:
                    while dashboard.running.is_set():
                        with dashboard.frame_changed:
                            dashboard.frame_changed.wait_for(
                                lambda: dashboard.frame_number != last_number or not dashboard.running.is_set(),
                                timeout=1,
                            )
                            if dashboard.frame_number == last_number:
                                continue
                            image = dashboard.latest_jpeg
                            last_number = dashboard.frame_number
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(image)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(image + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, format, *args):
                pass

        return Handler

    def start(self):
        """Start camera capture and a local web server."""
        if self.running.is_set():
            raise MissionStop("dashboard is already running")
        import cv2  # Fail before opening the camera if OpenCV is unavailable.

        try:
            result = self.camera.start_video_stream(
                display=False, resolution=self.settings["resolution"]
            )
            if result is False:
                raise MissionStop("could not start camera video stream")
            self.camera_started = True
            self.server = ThreadingHTTPServer(
                (self.settings["host"], self.settings["port"]), self._handler_class()
            )
            self.server.daemon_threads = True
            self.running.set()
            self.camera_thread = threading.Thread(target=self._camera_loop, name="camera-reader")
            self.server_thread = threading.Thread(target=self.server.serve_forever, name="dashboard-web")
            self.camera_thread.start()
            self.server_thread.start()
        except Exception:
            self.stop()
            raise

    def stop(self):
        """Stop the web server, camera reader and SDK video stream."""
        self.running.clear()
        with self.frame_changed:
            self.frame_changed.notify_all()
        if self.server_thread is not None:
            self.server.shutdown()
            self.server_thread.join()
            self.server_thread = None
        if self.server is not None:
            self.server.server_close()
            self.server = None
        if self.camera_thread is not None:
            self.camera_thread.join(timeout=2)
        if self.camera_started:
            self.camera.stop_video_stream()
            self.camera_started = False
        if self.camera_thread is not None:
            self.camera_thread.join(timeout=2)
            self.camera_thread = None
