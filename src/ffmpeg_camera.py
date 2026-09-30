"""Decode RoboMaster H.264 outside the robot-control process."""

from collections import deque
from pathlib import Path
from queue import Empty
import shutil
import subprocess
import threading
import time

import numpy as np

from src.mission_stop import MissionStop


class CameraRestarted(Exception):
    """Discard an inspection lock when the decoder generation changes."""


class FFmpegCamera:
    SIZES = {"360p": (640, 360), "540p": (960, 540), "720p": (1280, 720)}

    def __init__(self, camera, settings):
        self.camera = camera
        self.settings = settings
        self.config = settings.get("ffmpeg", {})
        self.executable = self._find_executable()
        self.changed = threading.Condition()
        self.stopping = threading.Event()
        self.thread = None
        self.process = None
        self.latest = None
        self.sequence = 0
        self.consumed = 0
        self.stream_enabled = False
        self.health = {"backend": "ffmpeg", "decoder_state": "stopped",
                       "decoder_generation": 0, "decoder_restarts": 0,
                       "last_decoder_error": None, "decoder_events": []}
        self.last_read_timestamp = None
        self.last_read_generation = None

    def _find_executable(self):
        project = Path(__file__).resolve().parent.parent
        configured = self.config.get("path", "")
        if configured:
            candidate = Path(configured)
            if not candidate.is_absolute():
                candidate = project / candidate
            if candidate.is_file():
                return str(candidate)
            raise MissionStop("FFmpeg executable not found: {}".format(candidate))
        bundled = project / "tools" / "ffmpeg" / "ffmpeg.exe"
        executable = str(bundled) if bundled.is_file() else shutil.which("ffmpeg")
        if not executable:
            raise MissionStop("FFmpeg not found; set dashboard.ffmpeg.path before running")
        return executable

    def decoder_snapshot(self):
        with self.changed:
            return {**self.health, "decoder_events": list(self.health["decoder_events"])}

    def _state(self, state, error=None):
        with self.changed:
            self.health["decoder_state"] = state
            if error:
                self.health["last_decoder_error"] = error
            if state != "ready":
                self.latest = None
            self.changed.notify_all()

    def start_video_stream(self, display=False, resolution="360p"):
        if self.thread is not None:
            raise MissionStop("FFmpeg camera is already running")
        if self.camera.conf.video_stream_proto != "tcp":
            raise MissionStop("FFmpeg backend currently requires RoboMaster EP TCP video")
        if getattr(self.camera._liveview, "_video_streaming", False):
            raise MissionStop("SDK decoder is already running; stop it before FFmpeg starts")
        self.width, self.height = self.SIZES[resolution]
        try:
            # These SDK commands enable transmission only. Never call public
            # start_video_stream(), which starts libmedia_codec decoding.
            if not self.camera._stream_sdk(1, resolution):
                raise MissionStop("could not enable SDK video transmission")
            self.stream_enabled = True
            if not self.camera._video_stream(1, resolution):
                raise MissionStop("could not enable camera video stream")
            self.camera._video_enable = True
            self.stopping.clear()
            self._state("starting")
            self.thread = threading.Thread(target=self._supervise, name="ffmpeg-camera", daemon=True)
            self.thread.start()
            return True
        except Exception:
            self.stop_video_stream()
            raise

    def _command(self):
        host, port = self.camera.video_stream_addr
        timeout_us = int(self.config.get("stall_timeout_s", 5.0) * 1000000)
        return [self.executable, "-hide_banner", "-loglevel", "warning", "-nostdin",
                "-rw_timeout", str(timeout_us), "-fflags", "+discardcorrupt",
                "-flags", "low_delay", "-probesize", "32768", "-analyzeduration", "0",
                "-f", "h264", "-i", "tcp://{}:{}".format(host, port), "-an",
                "-vf", "scale={}:{},fps={}".format(self.width, self.height, self.settings["max_fps"]),
                "-pix_fmt", "bgr24", "-c:v", "rawvideo", "-threads", "1",
                "-f", "rawvideo", "pipe:1"]

    def _read_frames(self, process, generation, ended):
        size = self.width * self.height * 3
        try:
            while not self.stopping.is_set():
                buffer = bytearray(size)
                view = memoryview(buffer)
                filled = 0
                while filled < size:
                    count = process.stdout.readinto(view[filled:])
                    if not count:
                        return  # Never publish a partial raw frame.
                    filled += count
                stamp = time.monotonic()
                image = np.frombuffer(buffer, dtype=np.uint8).reshape(self.height, self.width, 3)
                with self.changed:
                    if self.stopping.is_set():
                        return
                    self.sequence += 1
                    self.latest = (self.sequence, stamp, generation, image)
                    self.health["decoder_state"] = "ready"
                    self.changed.notify_all()
        except (OSError, ValueError):
            pass  # The supervisor restarts the decoder on EOF/read failure.
        finally:
            ended.set()
            self._state("reconnecting")

    @staticmethod
    def _read_errors(process, errors):
        try:
            for line in iter(process.stderr.readline, b""):
                errors.append(line.decode("utf-8", errors="replace").strip()[:500])
        except (OSError, ValueError):
            pass

    @staticmethod
    def _terminate(process):
        if process.poll() is None:
            try:
                process.terminate()
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            except OSError:
                pass

    def _supervise(self):
        while not self.stopping.is_set():
            process = None
            reader = error_reader = None
            errors = deque(maxlen=8)
            try:
                with self.changed:
                    self.health["decoder_generation"] += 1
                    generation = self.health["decoder_generation"]
                process = subprocess.Popen(
                    self._command(), stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE, bufsize=0,
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                )
                with self.changed:
                    self.process = process
                ended = threading.Event()
                reader = threading.Thread(target=self._read_frames, args=(process, generation, ended), daemon=True)
                error_reader = threading.Thread(target=self._read_errors, args=(process, errors), daemon=True)
                reader.start()
                error_reader.start()
                last_frame_at = time.monotonic()
                while not self.stopping.wait(.1) and process.poll() is None and not ended.is_set():
                    with self.changed:
                        if self.latest is not None:
                            last_frame_at = self.latest[1]
                    if time.monotonic() - last_frame_at > self.config.get("stall_timeout_s", 5.0):
                        errors.append("decoder produced no fresh frames within camera stall limit")
                        break
                self._terminate(process)
                reader.join()
                error_reader.join()
                error = "FFmpeg exited {}: {}".format(process.returncode, "; ".join(errors) or "video disconnected")
            except Exception as exc:
                error = "FFmpeg could not start: {}".format(exc)
                self._state("error", error)
                return  # Invalid executable/configuration requires operator action.
            finally:
                if process is not None:
                    self._terminate(process)
                    for worker in (reader, error_reader):
                        if worker is not None and worker.is_alive():
                            worker.join()
                    process.stdout.close()
                    process.stderr.close()
                with self.changed:
                    self.process = None
            if not self.stopping.is_set():
                with self.changed:
                    self.health["decoder_restarts"] += 1
                    events = self.health["decoder_events"]
                    events.append({"timestamp": time.time(), "generation": generation, "reason": error})
                    del events[:-50]
                self._state("reconnecting", error)
                self.stopping.wait(self.config.get("restart_delay_s", .5))
        self._state("stopped")

    def read_cv2_image(self, timeout=.02, strategy="newest"):
        deadline = time.monotonic() + timeout
        max_age = self.config.get("max_frame_age_s", .5)
        with self.changed:
            while not self.stopping.is_set():
                if self.health["decoder_state"] == "error":
                    raise MissionStop(self.health["last_decoder_error"])
                if self.latest is not None:
                    sequence, stamp, generation, image = self.latest
                    if sequence > self.consumed and time.monotonic() - stamp <= max_age:
                        self.consumed = sequence
                        self.last_read_timestamp = stamp
                        self.last_read_generation = generation
                        return image
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise Empty("waiting for a fresh FFmpeg camera frame")
                self.changed.wait(remaining)
        raise Empty("FFmpeg camera stopped")

    def stop_video_stream(self):
        self.stopping.set()
        with self.changed:
            process = self.process
            self.changed.notify_all()
        if process is not None:
            self._terminate(process)
        if self.thread is not None:
            self.thread.join()
            self.thread = None
        if self.stream_enabled:
            try:
                self.camera._video_stream(0)
            finally:
                self.camera._video_enable = False
                if not self.camera._audio_enable:
                    self.camera._stream_sdk(0)
                self.stream_enabled = False
        self._state("stopped")
