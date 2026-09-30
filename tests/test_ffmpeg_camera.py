"""Camera isolation and stale-lock regressions; no live robot or FFmpeg run."""

import io
import sys
import threading
import time
import unittest
from queue import Empty
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from src.ffmpeg_camera import CameraRestarted, FFmpegCamera
from src.dashboard import Dashboard
from src.target_inspection import WallTargetInspector


class FFmpegCameraTests(unittest.TestCase):
    def setUp(self):
        self.raw = SimpleNamespace(
            conf=SimpleNamespace(video_stream_proto="tcp"),
            _liveview=SimpleNamespace(_video_streaming=False, start_video_stream=Mock()),
            _stream_sdk=Mock(return_value=True), _video_stream=Mock(return_value=True),
            _video_enable=False, _audio_enable=False, video_stream_addr=("192.168.2.1", 40921),
            start_video_stream=Mock(),
        )
        self.camera = FFmpegCamera(self.raw, {
            "max_fps": 10, "ffmpeg": {"path": sys.executable, "max_frame_age_s": .5},
        })

    def test_start_enables_stream_without_starting_sdk_decoder(self):
        with patch("src.ffmpeg_camera.threading.Thread"):
            self.assertTrue(self.camera.start_video_stream(resolution="360p"))
            self.raw._stream_sdk.assert_called_once_with(1, "360p")
            self.raw._video_stream.assert_called_once_with(1, "360p")
            self.raw.start_video_stream.assert_not_called()
            self.raw._liveview.start_video_stream.assert_not_called()
            command = self.camera._command()
            self.assertIn("tcp://192.168.2.1:40921", command)
            self.assertIn("bgr24", command)
            self.camera.stop_video_stream()
        self.assertFalse(self.raw._video_enable)

    def test_latest_frame_is_consumed_once_and_expired_frame_is_rejected(self):
        self.camera.health.update(decoder_state="ready", decoder_generation=1)
        image = np.zeros((1, 2, 3), dtype=np.uint8)
        self.camera.latest = (2, time.monotonic(), 1, image)
        self.assertIs(self.camera.read_cv2_image(timeout=0), image)
        with self.assertRaises(Empty):
            self.camera.read_cv2_image(timeout=0)
        self.camera.latest = (3, time.monotonic() - 1, 1, image)
        with self.assertRaises(Empty):
            self.camera.read_cv2_image(timeout=0)

    def test_truncated_frame_and_decoder_eof_invalidate_old_image(self):
        self.camera.width, self.camera.height = 2, 1
        self.camera.latest = (1, time.monotonic(), 1, np.zeros((1, 2, 3)))
        ended = threading.Event()
        self.camera._read_frames(SimpleNamespace(stdout=io.BytesIO(b"123")), 1, ended)
        self.assertTrue(ended.is_set())
        self.assertEqual(self.camera.sequence, 0)
        self.assertIsNone(self.camera.latest)
        self.assertEqual(self.camera.decoder_snapshot()["decoder_state"], "reconnecting")

    def test_inspection_retries_same_wall_and_keeps_finished_outcomes(self):
        inspector = WallTargetInspector.__new__(WallTargetInspector)
        inspector.chassis = SimpleNamespace(stop=Mock())
        inspector._progress = Mock()
        outcome = {"status": "fire_command_accepted", "color": "red", "shape": "circle"}
        calls = []

        def inspect_once(cell, delta, world_yaw, body_yaw, range_mm, initial_pitch, completed, attempted):
            calls.append((cell, delta))
            if len(calls) == 1:
                completed.append(outcome)
                attempted.append(("red", "circle", (100, 100)))
                raise CameraRestarted()
            self.assertEqual(attempted, [("red", "circle", (100, 100))])
            return {"targets": completed}

        inspector._inspect_once = inspect_once
        result = inspector.inspect((1, 1), (0, 1), 90, 0)
        self.assertEqual(calls, [((1, 1), (0, 1))] * 2)
        self.assertEqual(result["targets"], [outcome])

    def test_decoder_disconnect_before_fire_never_sends_a_shot(self):
        inspector = WallTargetInspector.__new__(WallTargetInspector)
        inspector._safe_status = Mock()
        inspector._health = Mock()
        inspector._angles = Mock(return_value=(0, 0, time.time()))
        inspector.commanded_angles = (0, 0)
        inspector.settings = {"gimbal": {"pitch_tolerance_deg": 8, "angle_tolerance_deg": 3}}
        inspector.chassis = SimpleNamespace(stop=Mock())
        inspector.active_cell = None
        inspector.camera_frames = SimpleNamespace(
            supports_frame_metadata=True, is_camera_current=Mock(return_value=False),
            wait_for_camera_health=Mock(side_effect=CameraRestarted("decoder restarted")))
        inspector.camera_frame_metadata = {"generation": 1, "timestamp": time.monotonic()}
        inspector.blaster = SimpleNamespace(fire=Mock())
        with self.assertRaises(CameraRestarted):
            inspector._fire(None, None, "center")
        inspector.blaster.fire.assert_not_called()

    def test_old_pre_aim_lock_can_fire_with_fresh_same_generation_stream(self):
        inspector = WallTargetInspector.__new__(WallTargetInspector)
        inspector._safe_status = Mock()
        inspector._health = Mock()
        inspector._angles = Mock(return_value=(0, 0, time.time()))
        inspector.commanded_angles = (0, 0)
        inspector.settings = {"gimbal": {"pitch_tolerance_deg": 8, "angle_tolerance_deg": 3},
                              "target_inspection": {"fire_settle_s": 0}}
        inspector.camera_frames = SimpleNamespace(
            supports_frame_metadata=True, is_camera_current=Mock(return_value=True),
            wait_for_camera_health=Mock(), is_frame_current=Mock(return_value=False))
        inspector.camera_frame_metadata = {"generation": 1, "timestamp": time.monotonic() - 20}
        inspector.blaster = SimpleNamespace(fire=Mock(return_value=True))
        inspector.fire_type = "infrared"
        target = SimpleNamespace(color="green", shape="vertical", area=1, center=(1, 1))
        outcome = inspector._fire(target, np.zeros((2, 2, 3)), "blind_aim")
        self.assertEqual(outcome["status"], "fire_command_accepted")
        inspector.blaster.fire.assert_called_once_with(fire_type="infrared", times=2)
        inspector.camera_frames.is_frame_current.assert_not_called()

    def test_live_health_uses_latest_image_and_rejects_changed_generation(self):
        self.camera.health.update(decoder_state="ready", decoder_generation=1)
        dashboard = Dashboard(SimpleNamespace(camera=self.camera), SimpleNamespace(),
                              {"ffmpeg": {"max_frame_age_s": .5}})
        dashboard.running.set()
        dashboard.latest_frame = np.zeros((1, 2, 3))
        dashboard.latest_frame_monotonic = time.monotonic()
        dashboard.latest_frame_generation = 1
        self.assertTrue(dashboard.is_camera_current(1))
        dashboard.wait_for_camera_health(1)
        self.camera.health["decoder_generation"] = 2
        self.assertFalse(dashboard.is_camera_current(1))
        with self.assertRaises(CameraRestarted):
            dashboard.wait_for_camera_health(1)
