"""当前位置静态采样：面板级 ack/预览/启动门禁反例（真实 PanelBackend，无 ROS spin）。"""
import json
import math
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

import control_panel as panel_module  # noqa: E402
import drag_teaching as dt  # noqa: E402
import supported_capture as sc  # noqa: E402
from control_panel import Operation, PanelBackend  # noqa: E402
from control_panel_model import PanelError  # noqa: E402

HASH = "a" * 64
IDENTITY = {"schema": "slaihand_runtime_identity/1", "boot_id": "boot-A", "started_at": 1.0,
            "mapping_sha256": HASH, "model_sha256": HASH, "policy_sha256": HASH,
            "mass_profile_sha256": HASH, "policy_profile": "p", "load_id": "no_end_load",
            "gravity_base_m_s2": [0.0, 9.80665, 0.0], "j3_offset_rad": math.pi,
            "kp": 160.0, "kd": 5.0, "reference_speed_rad_s": 1.0,
            "gravity_ff_alpha": 0.0, "gravity_ff_ramp_s": 1.0}
CONFIRMATIONS = {"supported": True, "clearance_and_local_power_cut": True,
                 "native_output_not_hardware_clamp": True,
                 "supported_static_capture": True}
NAMES = ["slaihand_joint%d" % index for index in range(1, 5)]


def axes(stage=0, motor=0, hold_ready=0, command_id=0, position=0.05, valid=1.0):
    return {name: {"control_state": float(stage), "feedback_valid": valid,
                   "position": position, "velocity": 0.0, "effort": 0.0,
                   "feedback_age": 0.005, "driver_temperature": 30.0,
                   "motor_temperature": 25.0, "motor_state": float(motor),
                   "hold_ready": float(hold_ready), "command_id": float(command_id),
                   "disabled_observed": 0.0, "command_owner": 1.0, "audit_ok": 1.0}
            for name in NAMES}


class FakeNode:
    def create_subscription(self, *args, **kwargs):
        return None

    def create_client(self, *args, **kwargs):
        return None


class CapturePanelCase(unittest.TestCase):
    def setUp(self):
        self._directory = tempfile.TemporaryDirectory(prefix="capture-panel-")
        self.backend = PanelBackend(FakeNode(), self._directory.name, 60, 3, "capture-token")
        self.backend._ff_view = lambda: {"connected": True, "alpha": 0.0, "ramp": 1.0,
                                         "active": False, "stage": 0, "command_id": 0,
                                         "float_available": True, "gravity_float": False}
        self.store(stage=0, motor=0, hold_ready=0, command_id=0)
        self.backend._store("identity", {"at": time.monotonic(), "payload": dict(IDENTITY)})
        self.backend.heartbeat()

    def tearDown(self):
        self._directory.cleanup()

    def store(self, stage, motor, hold_ready, command_id, at=None):
        moment = time.monotonic() if at is None else at
        self.backend._store("dynamic", {"at": moment, "axes": axes(stage, motor, hold_ready, command_id)})
        self.backend._store("joints", {"at": moment, "positions": [0.05] * 4})

    def _ready_operation(self):
        operation = Operation("supported_capture", "op-capture-1",
                              Path(self._directory.name) / "operations" / "op-capture-1")
        operation.directory.mkdir(parents=True, exist_ok=True)
        operation.proc = SimpleNamespace(pid=12345, poll=lambda: None)
        operation.started_at = time.time()
        operation.capture_ack_path = operation.directory / sc.ACK_FILENAME
        with self.backend._lock:
            operation.capture_stop_generation = self.backend._stop_generation
        with self.backend._lock:
            self.backend._operation = operation
        self.store(stage=5, motor=1, hold_ready=1, command_id=7)
        self.backend._check_supported_capture(time.monotonic())
        return operation

    # -- preview / start --------------------------------------------------
    def test_preview_requires_disarmed_and_binds_identity(self):
        self.store(stage=8, motor=0, hold_ready=0, command_id=0)
        with self.assertRaises(PanelError) as caught:
            self.backend.preview_supported_capture()
        self.assertEqual(caught.exception.code, "NOT_DISARMED")
        self.store(stage=0, motor=0, hold_ready=0, command_id=0)
        response = self.backend.preview_supported_capture()
        preview = response["preview"]
        self.assertTrue(preview["baseline"]["satisfied"])
        self.assertEqual(preview["start_deg"], [2.865, 2.865, 2.865, 2.865])
        self.assertEqual(response["ticket"]["mode"], "supported_capture")
        self.assertEqual(response["ticket"]["identity_sha256"],
                         dt.identity_fingerprint(IDENTITY))

    def test_start_gating_matrix(self):
        self.store(stage=0, motor=0, hold_ready=0, command_id=0)
        ticket = self.backend.preview_supported_capture()["ticket"]
        with self.assertRaises(PanelError) as caught:
            self.backend.start_supported_capture(ticket["id"], {"supported": True})
        self.assertEqual(caught.exception.code, "CONFIRMATION")
        # identity changed after preview
        self.backend._store("identity", {"at": time.monotonic(),
                                         "payload": dict(IDENTITY, mapping_sha256="b" * 64)})
        with self.assertRaises(PanelError) as caught:
            self.backend.start_supported_capture(ticket["id"], CONFIRMATIONS)
        self.assertEqual(caught.exception.code, "IDENTITY_CHANGED")
        self.backend._store("identity", {"at": time.monotonic(), "payload": dict(IDENTITY)})
        # pose drift after preview
        self.backend._store("joints", {"at": time.monotonic(), "positions": [0.2] * 4})
        with self.assertRaises(PanelError) as caught:
            self.backend.start_supported_capture(ticket["id"], CONFIRMATIONS)
        self.assertEqual(caught.exception.code, "POSE_DRIFT")
        self.backend._store("joints", {"at": time.monotonic(), "positions": [0.05] * 4})
        # success path reaches the launcher with the fixed capture argv
        with patch.object(self.backend, "_launch", return_value={"launched": True}) as launch:
            response = self.backend.start_supported_capture(ticket["id"], CONFIRMATIONS)
        self.assertTrue(launch.called)
        kind, used_ticket, confirmations, baseline = launch.call_args[0]
        self.assertEqual(kind, "supported_capture")
        self.assertEqual(used_ticket["id"], ticket["id"])
        self.assertEqual(used_ticket["mode"], "supported_capture")
        self.assertEqual(used_ticket["identity_sha256"], dt.identity_fingerprint(IDENTITY))

    # -- ack --------------------------------------------------------------
    def test_ack_rejections_and_success(self):
        # no operation
        with self.assertRaises(PanelError) as caught:
            self.backend.ack_supported_capture("op", {"operator_disengaged": True})
        self.assertEqual(caught.exception.code, "NO_CAPTURE")
        operation = self._ready_operation()
        # missing operator confirmation
        with self.assertRaises(PanelError) as caught:
            self.backend.ack_supported_capture(operation.id, {})
        self.assertEqual(caught.exception.code, "DISENGAGE_ACK")
        # wrong operation id
        with self.assertRaises(PanelError) as caught:
            self.backend.ack_supported_capture("op-other", {"operator_disengaged": True})
        self.assertEqual(caught.exception.code, "STALE_OPERATION")
        # not ready: stage changes back to HOLD but not ready
        self.store(stage=5, motor=1, hold_ready=0, command_id=7)
        self.backend._check_supported_capture(time.monotonic())
        with self.assertRaises(PanelError) as caught:
            self.backend.ack_supported_capture(operation.id, {"operator_disengaged": True})
        self.assertEqual(caught.exception.code, "NOT_READY")
        # command_id changed
        self.store(stage=5, motor=1, hold_ready=1, command_id=8)
        self.backend._check_supported_capture(time.monotonic())
        self.assertEqual(operation.capture_command_id, 7)
        with self.assertRaises(PanelError) as caught:
            self.backend.ack_supported_capture(operation.id, {"operator_disengaged": True})
        self.assertEqual(caught.exception.code, "COMMAND_CHANGED")
        # identity changed
        self.store(stage=5, motor=1, hold_ready=1, command_id=7)
        self.backend._check_supported_capture(time.monotonic())
        self.backend._store("identity", {"at": time.monotonic(),
                                         "payload": dict(IDENTITY, policy_sha256="c" * 64)})
        with self.assertRaises(PanelError) as caught:
            self.backend.ack_supported_capture(operation.id, {"operator_disengaged": True})
        self.assertEqual(caught.exception.code, "IDENTITY_CHANGED")
        self.backend._store("identity", {"at": time.monotonic(), "payload": dict(IDENTITY)})
        # success
        response = self.backend.ack_supported_capture(operation.id, {"operator_disengaged": True})
        self.assertTrue(response["ok"])
        self.assertTrue(operation.capture_ack_path.is_file())
        ack = json.loads(operation.capture_ack_path.read_text(encoding="utf-8"))
        self.assertEqual(ack["command_id"], 7)
        self.assertEqual(ack["identity_sha256"], dt.identity_fingerprint(IDENTITY))
        self.assertEqual(ack["panel_stop_generation"], operation.capture_stop_generation)
        self.assertIsNotNone(operation.capture_ack_sent_at)
        # duplicate
        with self.assertRaises(PanelError) as caught:
            self.backend.ack_supported_capture(operation.id, {"operator_disengaged": True})
        self.assertEqual(caught.exception.code, "DUPLICATE_ACK")

    def test_ack_after_finish_or_stop_epoch_change_rejected(self):
        operation = self._ready_operation()
        operation.finished = True
        with self.assertRaises(PanelError) as caught:
            self.backend.ack_supported_capture(operation.id, {"operator_disengaged": True})
        self.assertEqual(caught.exception.code, "NO_CAPTURE")
        operation.finished = False
        with self.backend._lock:
            self.backend._stop_generation += 1
        with self.assertRaises(PanelError) as caught:
            self.backend.ack_supported_capture(operation.id, {"operator_disengaged": True})
        self.assertEqual(caught.exception.code, "STOP_EPOCH_CHANGED")

    def test_stale_feedback_blocks_ack(self):
        operation = self._ready_operation()
        self.store(stage=5, motor=1, hold_ready=1, command_id=7,
                   at=time.monotonic() - 2.0)
        self.backend._check_supported_capture(time.monotonic())
        with self.assertRaises(PanelError) as caught:
            self.backend.ack_supported_capture(operation.id, {"operator_disengaged": True})
        self.assertEqual(caught.exception.code, "NOT_READY")

    def test_baseline_alpha_display(self):
        self.backend._ff_view = lambda: {"connected": True, "alpha": 0.1, "ramp": 2.0,
                                         "active": True, "stage": 0, "command_id": 0,
                                         "float_available": True, "gravity_float": False}
        self.store(stage=0, motor=0, hold_ready=0, command_id=0)
        display = self.backend._capture_display(self.backend._core(), None, None, False, None)
        self.assertFalse(display["baseline"]["satisfied"])
        response = self.backend.preview_supported_capture()
        self.assertFalse(response["preview"]["baseline"]["satisfied"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
