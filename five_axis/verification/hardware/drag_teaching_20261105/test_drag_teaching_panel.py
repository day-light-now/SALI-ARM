"""拖动示教面板级回归：身份绑定票根、无帧诊断记录、源帧去重。

需要已 source 的 ROS 环境（导入 production PanelBackend）；不启 ROS spin、
不 Popen、不打开串口。
"""
import json
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
sys.path.insert(0, str(ROOT / "verification/hardware/control_panel_20261104"))
sys.path.insert(0, str(HERE))

import drag_teaching as teach  # noqa: E402
import test_drag_teaching as fixtures  # noqa: E402
from control_panel import PanelBackend  # noqa: E402
from control_panel_model import PanelError  # noqa: E402
from test_control_panel_callbacks import CONFIRMATIONS, FakeNode, axes  # noqa: E402


class TeachingPanelCase(unittest.TestCase):
    def setUp(self):
        self._directory = tempfile.TemporaryDirectory(prefix="drag-panel-")
        self.backend = PanelBackend(FakeNode(), self._directory.name, 60, 3, "teaching-token")
        self.backend._store("dynamic", {"at": time.monotonic(), "axes": axes(stage=0)})
        self.backend._store("joints", {"at": time.monotonic(), "positions": [0.05] * 4})
        self.backend._store("identity", {"at": time.monotonic(),
                                         "payload": dict(fixtures.IDENTITY)})
        self.backend._ff_view = lambda: {"gravity_float": True}
        self.backend.heartbeat()

    def tearDown(self):
        self._directory.cleanup()

    def _save_teaching(self):
        capture = fixtures.capture_of()
        return self.backend.teachings.save(capture, "review")

    def test_identity_change_after_preview_blocks_launch(self):
        teaching = self._save_teaching()
        preview = self.backend.preview_teaching(teaching["id"], 0.5)
        # 预览后驱动实际运行的 mapping 发生变化（同一消息源被替换）。
        self.backend._store("identity", {"at": time.monotonic(),
                                         "payload": dict(fixtures.IDENTITY,
                                                         mapping_sha256="b" * 64)})
        with patch.object(self.backend, "_launch", return_value={"launch_reached": True}) as launch:
            with self.assertRaises(PanelError) as caught:
                self.backend.execute_teaching(preview["ticket"]["id"], CONFIRMATIONS)
        self.assertEqual(caught.exception.code, "IDENTITY_CHANGED")
        self.assertFalse(launch.called, "identity change after preview must not reach the launcher")

    def test_unchanged_identity_reaches_launcher(self):
        teaching = self._save_teaching()
        preview = self.backend.preview_teaching(teaching["id"], 0.5)
        with patch.object(self.backend, "_launch", return_value={"launch_reached": True}) as launch:
            response = self.backend.execute_teaching(preview["ticket"]["id"], CONFIRMATIONS)
        self.assertTrue(launch.called)
        self.assertTrue(response.get("launch_reached"))
        args = launch.call_args[0]
        self.assertEqual(args[0], "teaching_playback")
        self.assertEqual(args[1]["identity_sha256"], teach.identity_fingerprint(fixtures.IDENTITY))

    def test_stop_before_first_frame_keeps_incomplete_summary(self):
        recorder = teach.TeachingRecorder(fixtures.BOUNDS, fixtures.LIMITS_DEG)
        ramp = fixtures.frames(10)
        for row in ramp:
            row["frame"] = dict(row["frame"], hold_ready=False, float=False)
        fixtures.feed(recorder, ramp)
        self.backend._teaching_recorder = recorder
        self.backend._finalize_teaching_record(
            SimpleNamespace(id="no-frames", stop_reason="拖动示教结束（停止优先）"),
            {"stop_observations_confirmed": True})
        last = self.backend._teaching_last
        self.assertIsNotNone(last, self.backend._teaching_last_error)
        self.assertFalse(last["complete"])
        self.assertIsNone(self.backend._teaching_last_error)
        self.assertEqual(len(list(self.backend.teachings.raw_dir.glob("*.json"))), 1)
        status = self.backend._teaching_display()
        self.assertEqual(status["last"]["recording_id"], last["recording_id"])
        self.assertEqual(status["phase"], "incomplete")

    def test_repeated_source_frame_is_deduplicated_in_callback(self):
        self.backend._teaching_recorder = teach.TeachingRecorder(
            fixtures.BOUNDS, fixtures.LIMITS_DEG)
        names = list(axes())
        row = {"position": 0.1, "velocity": 0.0, "feedback_age": 0.005,
               "feedback_valid": 1.0, "motor_state": 1.0, "control_state": 5.0,
               "command_id": 7.0, "hold_ready": 1.0}
        message = SimpleNamespace(
            joint_names=names,
            interface_values=[SimpleNamespace(interface_names=list(row),
                                              values=list(row.values())) for _ in names],
            header=SimpleNamespace(stamp=SimpleNamespace(sec=123, nanosec=0)))
        for index in range(60):
            with patch("control_panel.time.monotonic", return_value=100.0 + index * 0.01):
                self.backend._dynamic_callback(message)
        recorder = self.backend._teaching_recorder
        self.assertEqual(len(recorder.frames), 1,
                         "60 deliveries of one source frame must not become 60 samples")
        self.assertEqual(recorder.duplicate_frames, 59)
        self.assertEqual(recorder.display()["duration_s"], 0.0)
        self.assertEqual(recorder.frames[0]["source_stamp"], 123.0)

    def test_nonfinite_discrete_feedback_invalidates_recording_without_exception(self):
        for field in ("motor_state", "control_state", "command_id"):
            for value in (float("nan"), float("inf")):
                with self.subTest(field=field, value=value):
                    recorder = teach.TeachingRecorder(fixtures.BOUNDS, fixtures.LIMITS_DEG)
                    fixtures.feed(recorder, fixtures.frames(60, start=time.monotonic() - 0.6))
                    self.backend._teaching_recorder = recorder
                    self.backend._dynamic_sequence = 60
                    self.backend._teaching_last_error = None
                    row = {"position": 0.1, "velocity": 0.0, "feedback_age": 0.005,
                           "feedback_valid": 0.0, "motor_state": 1.0, "control_state": 5.0,
                           "command_id": 7.0, "hold_ready": 1.0}
                    row[field] = value
                    message = SimpleNamespace(
                        joint_names=list(axes()),
                        interface_values=[SimpleNamespace(interface_names=list(row),
                                                          values=list(row.values())) for _ in range(4)])
                    self.backend._dynamic_callback(message)
                    self.assertIsNone(self.backend._teaching_last_error)
                    self.assertIs(recorder.window_open, False)
                    capture = recorder.finalize(True, None, "invalid-feedback", 1.0)
                    self.assertFalse(capture["complete"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
