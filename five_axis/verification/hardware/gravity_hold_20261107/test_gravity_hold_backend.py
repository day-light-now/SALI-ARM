"""Backend-only regression for the in-session gravity-hold mode switch.

No ROS spin, no serial, no driver. Uses the real PanelBackend with a fake node
and fake observations so lease/stop/identity/duplicate/teaching gates can be
counterexampled deterministically.
"""
import math
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "slaihand_bringup" / "scripts"))

from control_panel import PanelBackend, PanelError  # noqa: E402
from calibration_capture import prearm_diagnostic  # noqa: E402
from calibration_session import group_document  # noqa: E402
from commission_supported_trial import mark_unknown  # noqa: E402

TOKEN = "<redacted-panel-token>"
CONFIRMATIONS = {"supported": True, "clearance_and_local_power_cut": True,
                 "native_output_not_hardware_clamp": True}


class FakeNode:
    def create_subscription(self, *args, **kwargs):
        return None

    def create_client(self, *args, **kwargs):
        return FakeClient()


class FakeClient:
    def __init__(self):
        self.requests = []

    def call_async(self, request):
        self.requests.append(request)
        future = SimpleNamespace(done=lambda: True)
        future.add_done_callback = lambda callback: callback(future)
        future.result = lambda: SimpleNamespace(result=SimpleNamespace(
            successful=True, reason="accepted only"))
        return future


class HoldCase(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="gravity-hold-")
        self.backend = PanelBackend(FakeNode(), self.directory.name, 60, 3, TOKEN)
        self.backend.heartbeat()
        self.operation = SimpleNamespace(id="op-1", kind="gravity_float", finished=False,
                                         cancel_requested=False, stop_reason=None,
                                         gravity_mode=None)
        self.backend._operation = self.operation
        self.core = {"command_id": 7, "hold_ready": True}
        self.driver = {"connected": True, "command_id": 7, "gravity_float": True,
                       "gravity_hold": False, "lock_state": 0, "hold_ready": True,
                       "lock_reason_code": 0, "float_kd": 5.0, "hold_gain_scale": 0.0,
                       "remaining_s": 10.0, "hold_target": [0.0] * 4,
                       "hold_error": [0.0] * 4, "alpha": 1.0, "active": True,
                       "reason": None}
        self.identity = {"fresh": True, "payload": {"boot_id": "boot-1", "kp": 160, "kd": 5}}
        self.backend._core = lambda: dict(self.core)
        self.backend._ff_view = lambda: dict(self.driver)
        self.backend._identity_view = lambda: dict(self.identity)
        self.captured = []
        self.lock_client = self.backend._gravity_lock_client
        self.resume_client = self.backend._gravity_resume_client
        self.backend._call_service = lambda key, client, request: self.captured.append(
            (key, client, request))

    def tearDown(self):
        self.directory.cleanup()

    def _lock(self):
        return self.backend.lock_current_pose()

    def _resume(self, confirmations):
        return self.backend.resume_gravity_float(confirmations)

    def test_lock_queues_driver_bound_request_and_rejects_duplicate(self):
        accepted = self._lock()
        self.assertTrue(accepted["accepted"])
        self.assertEqual(accepted["action"], "lock")
        with self.assertRaises(PanelError) as context:
            self._lock()
        self.assertEqual(context.exception.code, "BUSY_MODE")
        self.backend._drain_queues()
        self.assertEqual(len(self.lock_client.requests), 1)
        request = self.lock_client.requests[0]
        names = [parameter.name for parameter in request.parameters]
        self.assertEqual(names, ["command_id", "boot_id"])
        self.assertEqual(request.parameters[0].value.double_value, 7.0)
        self.assertEqual(request.parameters[1].value.string_value, "boot-1")
        # The normal immediate fake success must be accepted by the real
        # dispatch callback, not left in "sent" with a stale pending entry.
        with self.backend._lock:
            self.assertEqual(self.backend._gravity_mode_state["phase"], "accepted")
            self.assertNotIn("gravity_lock_current_pose",
                             self.backend._pending_mode_futures)

    def test_lock_failure_receipt_marks_rejected_via_real_dispatch(self):
        class FailingClient:
            def call_async(self, request):
                future = SimpleNamespace(done=lambda: True)
                future.add_done_callback = lambda callback: callback(future)
                future.result = lambda: SimpleNamespace(result=SimpleNamespace(
                    successful=False, reason="driver refused"))
                return future

        self.backend._gravity_lock_client = FailingClient()
        self._lock()
        self.backend._drain_queues()
        with self.backend._lock:
            self.assertEqual(self.backend._gravity_mode_state["phase"], "rejected")
            self.assertEqual(self.backend._gravity_mode_state["message"], "driver refused")
            self.assertNotIn("gravity_lock_current_pose",
                             self.backend._pending_mode_futures)

    def test_stop_generation_rejects_queued_lock_without_service_call(self):
        self._lock()
        with self.backend._lock:
            self.backend._stop_generation += 1
            self.backend._stop_locked = True
        self.backend._drain_queues()
        self.assertEqual(self.captured, [])
        self.assertEqual(self.backend._gravity_mode_state["phase"], "rejected")

    def test_concurrent_stop_after_initial_check_prevents_mode_send(self):
        original = self.backend._ff_view
        calls = {"n": 0}

        def ff_view():
            calls["n"] += 1
            if calls["n"] == 2:
                with self.backend._lock:
                    self.backend._stop_generation += 1
                    self.backend._stop_locked = True
            return original()

        self.backend._ff_view = ff_view
        self.assertTrue(self._lock()["accepted"])
        self.backend._drain_queues()
        self.assertEqual(self.captured, [])
        self.assertEqual(self.lock_client.requests, [])
        self.assertEqual(self.backend._gravity_mode_state["phase"], "rejected")

    def test_real_dispatch_overwrite_old_future_is_ignored(self):
        class DeferredFuture:
            def __init__(self):
                self.callbacks = []
                self.success = None

            def done(self):
                return self.success is not None

            def add_done_callback(self, callback):
                self.callbacks.append(callback)

            def result(self):
                return SimpleNamespace(result=SimpleNamespace(
                    successful=self.success, reason="deferred"))

            def resolve(self, success):
                self.success = success
                for callback in self.callbacks:
                    callback(self)

        class DeferredClient:
            def __init__(self):
                self.futures = []

            def call_async(self, request):
                future = DeferredFuture()
                self.futures.append(future)
                return future

        client = DeferredClient()
        self.backend._gravity_lock_client = client
        context1 = {"request_id": "req-1", "operation_id": "op-1", "generation": 0,
                    "action": "lock", "command_id": 7, "boot_id": "boot-1",
                    "at": time.monotonic()}
        context2 = dict(context1, request_id="req-2")
        with self.backend._lock:
            self.backend._gravity_mode_state = dict(
                context1, phase="queued", message="queued")
        self.backend._dispatch_gravity_mode(context1)
        with self.backend._lock:
            token1 = self.backend._pending_mode_futures["gravity_lock_current_pose"]
            self.backend._gravity_mode_state = dict(context2, phase="queued", message="queued")
        self.backend._dispatch_gravity_mode(context2)
        with self.backend._lock:
            self.assertEqual(self.backend._gravity_mode_state["request_id"], "req-2")
            self.assertEqual(self.backend._gravity_mode_state["phase"], "sent")
        # Old future resolves after the real dispatch replaced the mapping.
        client.futures[0].resolve(True)
        with self.backend._lock:
            self.assertEqual(self.backend._gravity_mode_state["request_id"], "req-2")
            self.assertEqual(self.backend._gravity_mode_state["phase"], "sent")
        # New future still owns and accepts its own request.
        client.futures[1].resolve(True)
        with self.backend._lock:
            self.assertEqual(self.backend._gravity_mode_state["phase"], "accepted")
            self.assertNotIn("gravity_lock_current_pose",
                             self.backend._pending_mode_futures)
        self.assertEqual(len(client.futures), 2)

    def test_old_mode_receipt_cannot_rewrite_new_request(self):
        class Future:
            def __init__(self, reason):
                self.reason = reason

            def result(self):
                return SimpleNamespace(result=SimpleNamespace(successful=True, reason=self.reason))

        old_future, new_future = Future("old accepted"), Future("new accepted")
        old_token = {"future": old_future, "request_id": "old-request",
                     "operation_id": "op-1", "generation": 0}
        new_token = {"future": new_future, "request_id": "new-request",
                     "operation_id": "op-1", "generation": 0}
        with self.backend._lock:
            self.backend._gravity_mode_state.update(
                {"request_id": "new-request", "action": "resume", "phase": "sent"})
            self.backend._pending["gravity_resume_float"] = (new_future, time.monotonic() + 3)
            self.backend._pending_mode_futures["gravity_resume_float"] = new_token
        # OLD same-name future arrives after the mapping was overwritten.
        self.backend._service_done("gravity_resume_float", old_future, old_token)
        with self.backend._lock:
            state = dict(self.backend._gravity_mode_state)
            pending_mode = self.backend._pending_mode_futures.get("gravity_resume_float")
            pending_future = self.backend._pending.get("gravity_resume_float")
        self.assertEqual(state["request_id"], "new-request")
        self.assertEqual(state["phase"], "sent")
        self.assertIs(pending_mode, new_token)
        self.assertIs(pending_future[0], new_future)
        # The real new future is still allowed to finish its own request.
        self.backend._service_done("gravity_resume_float", new_future, new_token)
        with self.backend._lock:
            self.assertEqual(self.backend._gravity_mode_state["phase"], "accepted")

    def test_expired_request_is_rejected_before_service(self):
        context = {"action": "lock", "operation_id": "op-1", "generation": 0,
                   "command_id": 7, "boot_id": "boot-1", "at": time.monotonic() - 5.0,
                   "request_id": "expired"}
        with self.backend._lock:
            self.backend._gravity_mode_state.update(
                {"request_id": "expired", "action": "lock", "phase": "queued"})
        self.backend._gravity_mode_queue.put(context)
        self.backend._drain_queues()
        self.assertEqual(self.captured, [])
        self.assertEqual(self.backend._gravity_mode_state["phase"], "rejected")

    def test_lease_timeout_and_command_identity_change_block_lock(self):
        self.backend._lease_last = time.monotonic() - 4.0
        with self.assertRaises(PanelError) as context:
            self._lock()
        self.assertEqual(context.exception.code, "LEASE")
        self.backend._lease_last = time.monotonic()
        self.driver["command_id"] = 8
        with self.assertRaises(PanelError) as context:
            self._lock()
        self.assertEqual(context.exception.code, "COMMAND_CHANGED")
        self.driver["command_id"] = 7
        # Context only rejects a missing/stale identity; boot binding is enforced
        # again in _dispatch_gravity_mode before the service request.
        accepted = self._lock()
        self.assertTrue(accepted["accepted"])
        self.identity = {"fresh": True, "payload": {"boot_id": "other", "kp": 160, "kd": 5}}
        self.backend._drain_queues()
        self.assertEqual(self.captured, [])
        self.assertEqual(self.backend._gravity_mode_state["phase"], "rejected")

    def test_teaching_recording_blocks_lock(self):
        self.backend._teaching_recorder = object()
        with self.assertRaises(PanelError) as context:
            self._lock()
        self.assertEqual(context.exception.code, "TEACHING_ACTIVE")

    def test_limit_or_motor_mismatch_blocks_lock_at_panel_gate(self):
        self.driver["lock_state"] = 5
        with self.assertRaises(PanelError) as context:
            self._lock()
        self.assertEqual(context.exception.code, "LOCK_STATE")
        self.driver["lock_state"] = 0
        self.core["hold_ready"] = False
        with self.assertRaises(PanelError) as context:
            self._lock()
        self.assertEqual(context.exception.code, "NOT_READY")

    def test_resume_requires_weight_ack_and_rejects_repeat(self):
        with self.assertRaises(PanelError) as context:
            self._resume({})
        self.assertEqual(context.exception.code, "RELEASE_ACK")
        self.driver.update(lock_state=2, gravity_hold=True)
        accepted = self._resume({"release_weight": True})
        self.assertTrue(accepted["accepted"])
        with self.assertRaises(PanelError) as context:
            self._resume({"release_weight": True})
        self.assertEqual(context.exception.code, "BUSY_MODE")
        self.backend._drain_queues()
        self.assertEqual(len(self.resume_client.requests), 1)
        request = self.resume_client.requests[0]
        self.assertEqual([p.name for p in request.parameters], ["command_id", "boot_id"])

    def test_calibration_preflight_reports_alpha_identity_and_old_model(self):
        valid_identity = {"schema": "slaihand_runtime_identity/1", "boot_id": "boot-1",
                          "started_at": 1.0, "mapping_sha256": "a" * 64,
                          "model_sha256": "b" * 64, "policy_sha256": "c" * 64,
                          "mass_profile_sha256": "d" * 64, "policy_profile": "p",
                          "load_id": "no_end_load", "gravity_base_m_s2": [0.0, 9.8, 0.0],
                          "j3_offset_rad": 3.14, "kp": 160.0, "kd": 5.0,
                          "reference_speed_rad_s": 1.0, "gravity_ff_alpha": 0.0,
                          "gravity_ff_ramp_s": 1.0}
        self.identity = {"fresh": True, "payload": valid_identity}
        self.driver["alpha"] = 0.1
        result = self.backend._calibration_preflight(group_document("S01"))
        self.assertFalse(result["ok"])
        self.assertEqual(result["observations"]["group"], "S01")
        self.assertEqual(result["observations"]["alpha"], 0.1)
        self.assertTrue(any(r.startswith("ALPHA_NONZERO:") for r in result["reasons"]),
                        result["reasons"])
        self.assertTrue(any(r.startswith("MODEL_UNQUALIFIED:") for r in result["reasons"]),
                        result["reasons"])
        self.driver["alpha"] = 0.0
        result = self.backend._calibration_preflight(group_document("S01"))
        self.assertFalse(any(r.startswith("ALPHA_NONZERO:") for r in result["reasons"]),
                         result["reasons"])
        self.assertTrue(any(r.startswith("MODEL_UNQUALIFIED:") for r in result["reasons"]),
                        result["reasons"])
        self.identity = {"fresh": False, "reason": "没有运行身份"}
        result = self.backend._calibration_preflight(group_document("S01"))
        self.assertTrue(any(r.startswith("IDENTITY_NOT_READY:") for r in result["reasons"]),
                        result["reasons"])

    def test_qualified_new_load_profile_has_a_pass_path(self):
        import calibration_session as session
        spec = {"waypoints": [{"degrees": [0.0, 10.0, 0.0, 0.0],
                               "move_s": 7.5, "dwell_s": 6.0}],
                "load_id": "new_load_3p5kg_j5", "mass_profile_sha256": "e" * 64,
                "model_sha256": "b" * 64, "mapping_sha256": "a" * 64,
                "j5_geometry_sha256": "f" * 64,
                "j5_geometry": True, "model_profile": "new_load_3p5kg_j5.json"}
        session.QUALIFIED_GROUPS["N01"] = spec
        try:
            self.driver["alpha"] = 0.0
            self.identity = {"fresh": True, "payload": {
                "schema": "slaihand_runtime_identity/1", "boot_id": "boot-1",
                "started_at": 1.0, "mapping_sha256": "a" * 64,
                "model_sha256": "b" * 64, "policy_sha256": "c" * 64,
                "mass_profile_sha256": "e" * 64, "policy_profile": "p",
                "j5_geometry_sha256": "f" * 64,
                "load_id": "new_load_3p5kg_j5", "gravity_base_m_s2": [0.0, 9.8, 0.0],
                "j3_offset_rad": 3.14, "kp": 160.0, "kd": 5.0,
                "reference_speed_rad_s": 1.0, "gravity_ff_alpha": 0.0,
                "gravity_ff_ramp_s": 1.0}}
            document = session.group_document("N01")
            result = self.backend._calibration_preflight(document, "N01")
            self.assertTrue(result["ok"], result["reasons"])
            self.assertTrue(result["observations"]["group_qualified"])
            self.assertEqual(result["observations"]["load_id"], "new_load_3p5kg_j5")
            self.identity["payload"]["load_id"] = "new_load_3p5kg_j5"
            self.identity["payload"]["model_sha256"] = "0" * 64
            result = self.backend._calibration_preflight(document, "N01")
            self.assertFalse(result["ok"], result["reasons"])
            self.assertTrue(any(r.startswith("MODEL_MISMATCH:") for r in result["reasons"]))
            self.identity["payload"]["model_sha256"] = "b" * 64
            self.identity["payload"]["mapping_sha256"] = "f" * 64
            result = self.backend._calibration_preflight(document, "N01")
            self.assertTrue(any(r.startswith("MAPPING_MISMATCH:") for r in result["reasons"]))
            self.identity["payload"]["mapping_sha256"] = "a" * 64
            self.identity["payload"]["j5_geometry_sha256"] = "0" * 64
            result = self.backend._calibration_preflight(document, "N01")
            self.assertTrue(any(r.startswith("J5_GEOMETRY_MISMATCH:") for r in result["reasons"]))
            self.identity["payload"]["j5_geometry_sha256"] = "f" * 64
            self.identity["payload"]["load_id"] = "no_end_load"
            result = self.backend._calibration_preflight(document, "N01")
            self.assertFalse(result["ok"], result["reasons"])
            self.assertTrue(any(r.startswith("LOAD_MISMATCH:") for r in result["reasons"]))
        finally:
            session.QUALIFIED_GROUPS.pop("N01", None)

    def test_unknown_during_stop_wait_is_preserved(self):
        result = {"unknown_observed": False}
        self.assertFalse(mark_unknown(result, {"j1": {"control_state": 8}}))
        self.assertFalse(result["unknown_observed"])
        self.assertTrue(mark_unknown(result, {"j4": {"control_state": 9}}))
        self.assertTrue(result["unknown_observed"])

    def test_prearm_diagnostic_lists_each_reason_and_observations(self):
        missing = prearm_diagnostic(None)
        self.assertTrue(missing["reasons"])
        sample = {"stage": 0, "motor_state": [0, 0, 0, 0],
                  "runtime": {"verified": False, "reason": "身份未就绪"},
                  "driver_ff": {"alpha": 0.1, "age_s": 2.0}}
        result = prearm_diagnostic(sample)
        self.assertTrue(any(r.startswith("RUNTIME_NOT_VERIFIED:") for r in result["reasons"]))
        self.assertTrue(any(r.startswith("FF_STALE:") for r in result["reasons"]))
        self.assertTrue(any(r.startswith("ALPHA_NONZERO:") for r in result["reasons"]))
        self.assertEqual(result["observations"]["alpha"], 0.1)
        sample["runtime"] = {"verified": True, "identity": {
            "load_id": "no_end_load", "model_sha256": "b" * 64,
            "mapping_sha256": "a" * 64, "j5_geometry_sha256": "f" * 64}}
        sample["driver_ff"] = {"alpha": 0.0, "age_s": 0.1}
        self.assertEqual(prearm_diagnostic(sample)["reasons"], [])
        mismatch = prearm_diagnostic(sample, expected_identity={
            "load_id": "new_load_3p5kg_j5", "model_sha256": "0" * 64,
            "mapping_sha256": "a" * 64, "j5_geometry_sha256": "f" * 64})
        self.assertTrue(any(r.startswith("LOAD_ID_MISMATCH:") for r in mismatch["reasons"]))
        self.assertTrue(any(r.startswith("MODEL_FINGERPRINT_MISMATCH:") for r in mismatch["reasons"]))

    def test_ff_extension_has_lock_target_error_gain_and_reason(self):
        values = [1.0, 3.0, 1.0, 5.0, 7.0] + [0.0] * 16 + [1.0, 5.0, 20.0]
        values += [2.0, 0.5, 9.0] + [0.1, 0.2, 0.3, 0.4] + [0.01, 0.02, 0.03, 0.04] + [3.0, 7.0]
        self.backend._ff_callback(SimpleNamespace(data=values))
        self.backend._ff_view = lambda: PanelBackend._ff_view(self.backend)
        self.operation.gravity_mode = {"action": "lock", "command_id": 7, "operation_id": "op-1",
                                       "phase": "accepted"}
        view = self.backend._ff_view()
        self.assertTrue(view["connected"])
        self.assertTrue(view["gravity_hold"])
        self.assertEqual(view["lock_state"], 2)
        self.assertEqual(view["hold_target"], [0.1, 0.2, 0.3, 0.4])
        self.assertEqual(view["hold_error"], [0.01, 0.02, 0.03, 0.04])
        self.assertEqual(view["lock_reason_code"], 3)
        self.assertAlmostEqual(view["hold_gain_scale"], 0.5)
        mode = self.backend._gravity_mode_display(
            {"positions": [0.1, 0.2, 0.3, 0.4]}, self.operation)
        self.assertEqual(mode["phase"], "locked")
        self.assertTrue(mode["locked_confirmed"])
        self.assertEqual(mode["lock_reason_code"], 3)
        self.assertAlmostEqual(mode["kp"], 80.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
