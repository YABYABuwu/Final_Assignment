"""Local dashboard for live robot telemetry and camera images."""

import json
from queue import Empty
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
                 explorer=None, motion_settings=None, rear_ir=None, front_ir=None,
                 target_settings=None, color_ranges=None, mission_start_required=False):
        self.camera = robot.camera
        self.logger = logger
        self.settings = settings
        self.slam_map = slam_map
        self.slam_worker = slam_worker
        self.explorer = explorer
        self.motion_settings = motion_settings
        self.rear_ir = rear_ir
        self.front_ir = front_ir
        self.target_settings = target_settings
        self.target_policy_lock = threading.Lock()
        self.target_policy_revision = 0
        self.mission_start_required = mission_start_required
        self.mission_started = threading.Event()
        self.mission_start_lock = threading.Lock()
        self.target_policy = {
            "selected": list(target_settings["selected"]) if target_settings and
                        target_settings["selected"] != "all" else "all",
            "fire_mode": target_settings.get("fire_mode", "infrared") if target_settings else "infrared",
        }
        self.color_ranges = color_ranges
        self.running = threading.Event()
        self.frame_changed = threading.Condition()
        self.latest_jpeg = None
        self.latest_frame = None
        self.latest_frame_monotonic = None
        self.latest_frame_generation = None
        self.supports_frame_metadata = True
        self.camera_targets = []
        self.frame_number = 0
        self.camera_error = None
        self.camera_health = {
            "state": "waiting_frame",
            "skipped_reads": 0,
            "recovered_gaps": 0,
            "last_frame_time": None,
            "last_skip_reason": None,
        }
        self.logger.camera_health = self.camera_health
        self.mission_status = "รอเลือกเป้าและกดเริ่มภารกิจ" if mission_start_required else "Ready"
        self.camera_thread = None
        self.server_thread = None
        self.server = None
        self.camera_started = False

    def target_policy_snapshot(self):
        """Return one consistent policy; inspection freezes it for each wall."""
        with self.target_policy_lock:
            selected = self.target_policy["selected"]
            return {"selected": list(selected) if selected != "all" else "all",
                    "fire_mode": self.target_policy["fire_mode"],
                    "revision": self.target_policy_revision,
                    "enabled": bool(self.target_settings and self.target_settings["enabled"])}

    def update_target_policy(self, document):
        if not self.target_settings or not self.target_settings["enabled"]:
            raise ValueError("target inspection is disabled in config")
        if not isinstance(document, dict):
            raise ValueError("target settings must be a JSON object")
        selected, mode = document.get("selected"), document.get("fire_mode")
        pairs = {color + ":" + shape for color in ("red", "green", "yellow", "blue")
                 for shape in ("circle", "square", "horizontal", "vertical")}
        if selected != "all" and (not isinstance(selected, list) or
                                   any(not isinstance(pair, str) or pair not in pairs for pair in selected)):
            raise ValueError("selected must be all or a list of supported COLOR:SHAPE pairs")
        if mode not in ("infrared", "gel"):
            raise ValueError("fire_mode must be infrared or gel")
        with self.target_policy_lock:
            self.target_policy = {"selected": sorted(set(selected)) if selected != "all" else "all",
                                  "fire_mode": mode}
            self.target_policy_revision += 1
        return self.target_policy_snapshot()

    def start_mission(self, document):
        """Save the displayed selection before releasing the one-shot start gate."""
        if not isinstance(document, dict):
            raise ValueError("mission start settings must be a JSON object")
        with self.mission_start_lock:
            if not self.mission_start_required or self.mission_started.is_set():
                raise ValueError("mission start is unavailable or already requested")
            if not self.running.is_set():
                raise ValueError("dashboard is not running")
            if self.camera_error is not None:
                raise ValueError(self.camera_error)
            if self.target_settings and self.target_settings["enabled"]:
                self.update_target_policy(document)
            self.mission_status = "กำลังเริ่มภารกิจ"
            self.mission_started.set()
        return {"started": True, "target_policy": self.target_policy_snapshot()}

    def wait_for_mission_start(self, chassis):
        """Keep wheels stopped without starting navigation or target inspection."""
        if not self.mission_start_required:
            return
        chassis.stop()
        while not self.mission_started.wait(.1):
            if not self.running.is_set() or self.camera_error is not None:
                raise MissionStop(self.camera_error or "dashboard stopped before mission start")

    def _skip_camera_frame(self, reason):
        """Discard a missing/broken frame without stopping the dashboard."""
        with self.frame_changed:
            self.camera_health["state"] = "waiting_frame"
            self.camera_health["skipped_reads"] += 1
            self.camera_health["last_skip_reason"] = reason

            # Do not give an old inspection image to the target detector.
            self.latest_frame = None
            self.camera_targets = []

            # Keep the last JPEG for display while waiting for a new image.
            self.frame_changed.notify_all()

    def _sync_decoder_health(self):
        snapshot = getattr(self.camera, "decoder_snapshot", None)
        if callable(snapshot):
            health = snapshot()
            with self.frame_changed:
                self.camera_health.update(health)

    def is_frame_current(self, metadata):
        """Reject incoming images from an earlier decoder or an expired frame."""
        if metadata is None or metadata.get("timestamp") is None:
            return False
        if time.monotonic() - metadata["timestamp"] > self.settings.get("ffmpeg", {}).get("max_frame_age_s", .5):
            return False
        snapshot = getattr(self.camera, "decoder_snapshot", None)
        if callable(snapshot):
            health = snapshot()
            return (health["decoder_state"] == "ready" and
                    metadata.get("generation") == health["decoder_generation"])
        return True

    def is_camera_current(self, generation):
        """Check the live stream independently of the pre-aim lock image."""
        with self.frame_changed:
            return (self.latest_frame is not None and
                    self.latest_frame_generation == generation and
                    self.is_frame_current({"timestamp": self.latest_frame_monotonic,
                                           "generation": self.latest_frame_generation}))

    def wait_for_camera_health(self, generation, check_health=None):
        """Wait in place for live images; only a real restart invalidates a lock."""
        from src.ffmpeg_camera import CameraRestarted

        with self.frame_changed:
            while self.running.is_set() and self.camera_error is None:
                if check_health is not None:
                    check_health()
                snapshot = getattr(self.camera, "decoder_snapshot", None)
                if callable(snapshot):
                    health = snapshot()
                    if health["decoder_generation"] != generation:
                        raise CameraRestarted("decoder restarted before infrared fire")
                    if health["decoder_state"] == "error":
                        raise MissionStop(health["last_decoder_error"] or "camera decoder failed")
                if self.is_camera_current(generation):
                    return
                self.frame_changed.wait(.1)
        raise MissionStop(self.camera_error or "camera stopped before infrared fire")

    def _camera_loop(self):
        """Only this thread reads frames from the SDK camera."""
        import cv2

        period = 1 / self.settings["max_fps"]

        while self.running.is_set():
            started = time.monotonic()

            try:
                image = self.camera.read_cv2_image(
                    timeout=min(period, .02),
                    strategy="newest",
                )
                self._sync_decoder_health()
                metadata = {
                    "timestamp": getattr(self.camera, "last_read_timestamp", None) or time.monotonic(),
                    "generation": getattr(self.camera, "last_read_generation", None),
                }

                if image is None:
                    self._skip_camera_frame(
                        "no decoded frame available"
                    )
                else:
                    shape = getattr(image, "shape", None)

                    if shape is not None and (
                        len(shape) != 3
                        or shape[2] != 3
                        or shape[0] <= 0
                        or shape[1] <= 0
                    ):
                        raise cv2.error(
                            "invalid decoded BGR frame"
                        )

                    annotated = image
                    camera_targets = []

                    if (
                        self.target_settings is not None
                        and self.target_settings["enabled"]
                    ):
                        from src.targets import detect

                        selected = self.target_policy_snapshot()["selected"]
                        selected = (
                            None
                            if selected == "all"
                            else {
                                tuple(pair.split(":"))
                                for pair in selected
                            }
                        )

                        detections = detect(
                            image,
                            self.target_settings["min_area_fraction"],
                            selected,
                            self.color_ranges,
                        )

                        annotated = image.copy()
                        height, width = image.shape[:2]

                        box_colors = {
                            "red": (55, 55, 255),
                            "green": (80, 240, 80),
                            "yellow": (60, 220, 255),
                            "blue": (255, 140, 70),
                        }

                        for item in detections:
                            x, y, w, h = cv2.boundingRect(
                                item.contour
                            )
                            color = box_colors[item.color]

                            cv2.rectangle(
                                annotated,
                                (x, y),
                                (x + w, y + h),
                                color,
                                2,
                            )

                            cv2.putText(
                                annotated,
                                f"{item.color} {item.shape}",
                                (x, max(15, y - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                .5,
                                color,
                                1,
                                cv2.LINE_AA,
                            )

                            camera_targets.append({
                                "color": item.color,
                                "shape": item.shape,
                                "center": list(item.center),
                                "box": [x, y, w, h],
                                "area_fraction": round(
                                    item.area / (width * height),
                                    4,
                                ),
                            })

                    ok, encoded = cv2.imencode(
                        ".jpg",
                        annotated,
                        [
                            cv2.IMWRITE_JPEG_QUALITY,
                            self.settings["jpeg_quality"],
                        ],
                    )

                    if ok and self.is_frame_current(metadata):
                        with self.frame_changed:
                            if (
                                self.camera_health["state"]
                                == "waiting_frame"
                                and (
                                    self.camera_health["last_frame_time"]
                                    is not None
                                )
                            ):
                                self.camera_health["recovered_gaps"] += 1

                            self.camera_health["state"] = "ready"
                            self.camera_health["last_frame_time"] = (
                                time.time()
                            )

                            self.latest_jpeg = encoded.tobytes()
                            self.latest_frame = (
                                image.copy()
                                if hasattr(image, "copy")
                                else image
                            )
                            self.latest_frame_monotonic = metadata["timestamp"]
                            self.latest_frame_generation = metadata["generation"]
                            self.camera_targets = camera_targets
                            self.frame_number += 1
                            self.frame_changed.notify_all()
                    else:
                        self._skip_camera_frame(
                            "JPEG encoding rejected the frame"
                        )

            except (Empty, OSError, cv2.error) as error:
                self._sync_decoder_health()
                # Temporary frame/network/decoding failure:
                # skip this read and request the newest frame next.
                self._skip_camera_frame(
                    str(error) or type(error).__name__
                )

            except Exception as error:
                # Unexpected programming errors remain visible.
                self._sync_decoder_health()
                self.camera_error = str(error)
                self.camera_health["state"] = "error"
                self.running.clear()

                with self.frame_changed:
                    self.frame_changed.notify_all()

                break

            remaining = period - (
                time.monotonic() - started
            )
            if remaining > 0:
                time.sleep(remaining)
                
    def wait_for_frame(self, after_number=0, check_health=None, include_metadata=False):
        """Return a fresh BGR frame without starting a second SDK camera reader."""
        with self.frame_changed:
            while self.running.is_set() and self.camera_error is None:
                if check_health is not None:
                    check_health()
                metadata = {"timestamp": self.latest_frame_monotonic,
                            "generation": self.latest_frame_generation}
                if (self.frame_number > after_number and self.latest_frame is not None and
                        self.is_frame_current(metadata)):
                    frame = self.latest_frame
                    result = (self.frame_number, frame.copy() if hasattr(frame, "copy") else frame)
                    return result + (metadata,) if include_metadata else result
                self.frame_changed.wait(0.1)
        raise MissionStop(self.camera_error or "camera stream stopped during target inspection")

    def current_frame_number(self):
        with self.frame_changed:
            return self.frame_number

    def snapshot(self):
        """Build a JSON-friendly snapshot without camera or disk reads."""
        self._sync_decoder_health()
        with self.frame_changed:
            camera_health = dict(self.camera_health)
            camera_ready = (
                camera_health["state"] == "ready"
                and self.latest_frame is not None
                and self.is_frame_current({"timestamp": self.latest_frame_monotonic,
                                           "generation": self.latest_frame_generation})
            )
            camera_targets = list(self.camera_targets)

        streams = {}
        for name in STREAMS:
            if self.logger.stream_settings.get(
                name, {}
            ).get("enabled"):
                streams[name] = self.logger.get_latest(
                    name,
                    max_age_s=2,
                )

        last_frame_time = camera_health["last_frame_time"]

        target_detection = {
            "enabled": bool(self.target_settings),
            "detections": [
                {
                    "color": t.get("color"),
                    "shape": t.get("shape"),
                    "center_px": t.get(
                        "center", [320, 180]
                    ),
                    "center_offset_norm": [
                        round(
                            (
                                t.get("center", [320, 180])[0]
                                - 320
                            ) / 320.0,
                            4,
                        ),
                        round(
                            (
                                t.get("center", [320, 180])[1]
                                - 180
                            ) / 180.0,
                            4,
                        ),
                    ],
                    "area_px2": (
                        t.get("area_fraction", 0.0)
                        * (640 * 360)
                    ),
                    "stability_hits": 3,
                    "stability_required": 3,
                    "confirmed": True,
                }
                for t in camera_targets
            ],
            "age_ms": (
                round(
                    max(0, time.time() - last_frame_time)
                    * 1000
                )
                if last_frame_time is not None
                else None
            ),
            "error": self.camera_error,
        }

        return {
            "streams": streams,
            "dropped_csv_rows": self.logger.dropped_rows,
            "camera_ready": camera_ready,
            "camera_targets": camera_targets,
            "target_detection": target_detection,
            "target_policy": self.target_policy_snapshot(),
            "camera_error": self.camera_error,
            "camera_health": camera_health,
            "mission_status": self.mission_status,
            "mission_control": {"requires_start": self.mission_start_required,
                                "started": self.mission_started.is_set()},
            "round2_navigation": getattr(self.logger, "round2_navigation", None),
            "round2_alignment": getattr(self.logger, "round2_alignment", None),
            "motion_settings": self.motion_settings,
            "rear_ir": (
                self.rear_ir.snapshot()
                if self.rear_ir is not None
                else {"enabled": False}
            ),
            "front_ir": (
                self.front_ir.snapshot()
                if self.front_ir is not None
                else {"enabled": False}
            ),
            "slam": (
                self.slam_worker.status()
                if self.slam_worker is not None
                else None
            ),
            "exploration": (
                self.explorer.snapshot()
                if self.explorer is not None
                else None
            ),
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
                if urlsplit(self.path).path == "/api/mission/start":
                    try:
                        length = int(self.headers.get("Content-Length", "0"))
                        if not 0 < length <= 4096:
                            raise ValueError("start JSON must be between 1 and 4096 bytes")
                        document = json.loads(self.rfile.read(length).decode("utf-8"))
                        result = dashboard.start_mission(document)
                    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
                        self.send_error(400, str(error))
                        return
                    self._send_content("application/json; charset=utf-8", json.dumps(result).encode("utf-8"))
                    return
                if urlsplit(self.path).path == "/api/targets":
                    try:
                        length = int(self.headers.get("Content-Length", "0"))
                        if not 0 < length <= 4096:
                            raise ValueError("target JSON must be between 1 and 4096 bytes")
                        document = json.loads(self.rfile.read(length).decode("utf-8"))
                        policy = dashboard.update_target_policy(document)
                    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
                        self.send_error(400, str(error))
                        return
                    self._send_content("application/json; charset=utf-8", json.dumps(policy).encode("utf-8"))
                    return
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
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", content_type)
                    self.send_header("Content-Length", str(len(content)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(content)
                except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError):
                    pass

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
            if self.settings.get("camera_backend", "sdk") == "ffmpeg":
                from src.ffmpeg_camera import FFmpegCamera
                if not isinstance(self.camera, FFmpegCamera):
                    self.camera = FFmpegCamera(self.camera, self.settings)
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
            self._sync_decoder_health()
        if self.camera_thread is not None:
            self.camera_thread.join(timeout=2)
            self.camera_thread = None
