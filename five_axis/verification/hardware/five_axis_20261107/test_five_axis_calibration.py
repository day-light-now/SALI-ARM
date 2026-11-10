"""Unified selectable-joint calibration workflow tests (backend + HTTP + UI).

No ROS, no serial, no CAN: the ROS module names are replaced with inert stubs
exactly like ``test_five_axis_panel_backend``, then the real production
``PanelBackend`` is driven with injected fake feedback/identity state.  The
workflow is non-formal by construction: it must only record manual readings,
never enable, never write a motor zero/mapping/URDF, and it must invalidate the
sequence on stop / identity change / stale feedback / UNKNOWN / abort.

Run:
    python verification/hardware/five_axis_20261107/test_five_axis_calibration.py
"""
import hashlib
import http.client
import json
import math
import re
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SCRIPTS = ROOT / "src" / "slaihand_bringup" / "scripts"
STATIC = ROOT / "src" / "slaihand_bringup" / "web" / "control_panel"
MAPPING = ROOT / "src" / "slaihand_bringup" / "config" / "hardware_mapping.yaml"
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(HERE))

import test_five_axis_panel_backend as stubs  # installs the ROS stubs # noqa: E402

control_panel = stubs.control_panel
import control_panel_model as model  # noqa: E402
from control_panel_http import PanelServer  # noqa: E402

NAMES5 = ["slaihand_joint%d" % index for index in range(1, 6)]
NAMES4 = ["slaihand_joint%d" % index for index in range(1, 5)]
ACK = {"axis_calibration_supported": True, "axis_calibration_clearance": True}
TOKEN = "<redacted-panel-token>"
PHASES = ["zero", "positive", "lower", "upper"]


class Base(unittest.TestCase):
    joint_count = 5

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="axis-calibration-")
        self.addCleanup(self.tmp.cleanup)
        stubs.SHARE["slaihand_bringup"] = str(ROOT / "src" / "slaihand_bringup")
        stubs.SHARE["slaihand_description"] = str(stubs.DESCRIPTION)
        self.backend = control_panel.PanelBackend(
            stubs.FakeNode(), self.tmp.name, 60, 30, "token",
            gravity_enabled=False, joint_count=self.joint_count)
        self.names = list(NAMES5 if self.joint_count == 5 else NAMES4)
        self.identity = stubs.runtime_identity("a" * 64)
        self.session_id = None

    def inject(self, positions=None, stage=0):
        stubs.inject(self.backend, self.names, stage=stage, positions=positions,
                     identity=self.identity)

    def set_positions(self, positions, stage=0):
        now = time.monotonic()
        axes = stubs.axes_for(self.names, stage)
        for name, position in zip(self.names, positions):
            axes[name]["position"] = position
        self.backend._dynamic = {"at": now, "axes": axes}
        self.backend._joints = {"at": now, "positions": list(positions)}

    def start(self, joint=None, confirmations=None):
        response = self.backend.start_axis_calibration(
            joint or self.names[-1], dict(ACK if confirmations is None else confirmations))
        self.session_id = response["session_id"]
        return response

    def capture(self, phase, positions=None, session_id=None, confirmations=None):
        if positions is not None:
            self.set_positions(positions)
        return self.backend.capture_axis_calibration(
            session_id if session_id is not None else self.session_id,
            phase, dict(ACK if confirmations is None else confirmations))

    def display(self):
        return self.backend.status()["axis_calibration"]

    def assert_error(self, code, func):
        with self.assertRaises(control_panel.PanelError) as caught:
            func()
        self.assertEqual(caught.exception.code, code, caught.exception.message)
        return caught.exception

    def audit_path(self, session_id=None):
        session_id = session_id or self.session_id
        return Path(self.tmp.name) / "axis_calibration" / (session_id + ".json")

    def read_audit(self, session_id=None):
        path = self.audit_path(session_id)
        self.assertTrue(path.is_file(), "audit JSON missing: %s" % path)
        return json.loads(path.read_text(encoding="utf-8"))


class StartRules(Base):
    def test_start_requires_support_and_clearance_confirmation(self):
        self.inject()
        self.assert_error("AXIS_CAL_CONFIRMATION",
                          lambda: self.backend.start_axis_calibration(self.names[-1], {}))
        self.assert_error(
            "AXIS_CAL_CONFIRMATION",
            lambda: self.backend.start_axis_calibration(
                self.names[-1], {"axis_calibration_supported": True}))
        response = self.start()
        self.assertTrue(response["ok"])
        self.assertFalse(response["completed"])
        self.assertTrue(response["session_id"])
        self.assertEqual(self.backend._axis_calibration["state"], "active")

    def test_start_requires_a_controlled_joint(self):
        self.inject()
        for joint in ("", None, "slaihand_joint9", "J5"):
            with self.subTest(joint=joint):
                self.assert_error("AXIS_CAL_JOINT",
                                  lambda joint=joint: self.backend.start_axis_calibration(
                                      joint, dict(ACK)))

    def test_start_requires_fresh_runtime_identity(self):
        self.inject()
        self.backend._identity["at"] = time.monotonic() - 10.0
        self.assert_error("RUNTIME_IDENTITY", lambda: self.start())
        self.backend._identity = {"at": time.monotonic(),
                                  "payload": {"schema": "slaihand_runtime_identity/1"}}
        self.assert_error("RUNTIME_IDENTITY", lambda: self.start())

    def test_start_requires_fresh_feedback_and_all_axes_disarmed(self):
        self.inject(stage=5)
        self.assert_error("NOT_DISARMED", lambda: self.start())
        self.inject(stage=9)
        self.assert_error("UNKNOWN", lambda: self.start())
        self.inject()
        self.backend._dynamic["at"] = time.monotonic() - 10.0
        self.assert_error("FEEDBACK_STALE", lambda: self.start())
        self.inject()
        for name in self.names:
            self.backend._dynamic["axes"][name]["feedback_valid"] = 0
        self.assert_error("FEEDBACK_INVALID", lambda: self.start())
        self.inject()
        for name in self.names:
            self.backend._dynamic["axes"][name].pop("position", None)
        self.backend._joints = {"at": time.monotonic(), "positions": None}
        self.assert_error("FEEDBACK_INVALID", lambda: self.start())

    def test_start_requires_every_axis_actually_motor_disabled(self):
        for value in (1, 0.5, float("nan"), None):
            self.inject()
            self.backend._dynamic["axes"][self.names[0]]["motor_state"] = value
            self.assert_error("NOT_DISARMED", lambda: self.start())
        self.inject()
        self.backend._dynamic["axes"][self.names[-1]].pop("motor_state", None)
        self.assert_error("NOT_DISARMED", lambda: self.start())

    def test_start_requires_selected_axis_feedback(self):
        self.inject()
        joint = self.names[-1]
        self.backend._dynamic["axes"][joint]["feedback_age"] = 1.5
        error = self.assert_error("AXIS_FEEDBACK", lambda: self.start())
        self.assertIn("AXIS_FEEDBACK_STALE", " ".join(error.extra["feedback"]["reasons"]))

    def test_start_records_identity_and_writes_a_non_formal_audit(self):
        before = hashlib.sha256(MAPPING.read_bytes()).hexdigest()
        self.inject()
        response = self.start()
        display = response["axis_calibration"]
        self.assertFalse(display["formal"])
        self.assertEqual(display["selected_joint"], self.names[-1])
        self.assertEqual(display["selected_label"], "J5")
        self.assertEqual(display["phase"], "zero")
        self.assertEqual(display["pending"],
                         [model.AXIS_CAL_PHASE_LABELS[name] for name in PHASES])
        session = self.backend._axis_calibration
        self.assertEqual(session["formal"], False)
        self.assertEqual(session["boot_id"], self.identity["boot_id"])
        self.assertEqual(session["identity"]["mapping_sha256"],
                         self.identity["mapping_sha256"])
        self.assertEqual(session["identity"]["model_sha256"],
                         self.identity["model_sha256"])
        self.assertTrue(session["identity"]["identity_sha256"])
        audit = self.read_audit()
        self.assertEqual(audit["schema"], model.AXIS_CALIBRATION_SCHEMA)
        self.assertFalse(audit["formal"])
        self.assertEqual(audit["session"]["transitions"][0]["action"], "start")
        self.assertFalse(audit["session"]["motor_command_sent"])
        self.assertTrue(any("轴标定会话开始" in event["message"]
                            for event in self.backend._events))
        after = hashlib.sha256(MAPPING.read_bytes()).hexdigest()
        self.assertEqual(before, after, "workflow must not write hardware_mapping.yaml")
        self.assert_error("CAL_ACTIVE", lambda: self.start())
        self.backend.abort_axis_calibration("test cleanup")
        self.backend._operation = control_panel.Operation(
            "hold", "test-operation", Path(self.tmp.name) / "operation")
        self.assert_error("BUSY", lambda: self.start())


class CaptureSequence(Base):
    def setUp(self):
        super().setUp()
        self.inject(positions=[0.0, 0.0, 0.0, 0.0, 0.1])
        self.start()

    def complete_flow(self):
        self.capture("zero")
        self.capture("positive", positions=[0.0, 0.0, 0.0, 0.0, 0.1 + math.radians(10.0)])
        self.capture("lower", positions=[0.0, 0.0, 0.0, 0.0, math.radians(-20.0)])
        return self.capture("upper", positions=[0.0, 0.0, 0.0, 0.0, math.radians(40.0)])

    def test_phase_order_same_session_and_duplicates(self):
        self.assert_error("CAL_PHASE_ORDER", lambda: self.capture("positive"))
        self.assert_error("CAL_SESSION", lambda: self.capture("zero", session_id="deadbeef"))
        response = self.capture("zero")
        self.assertFalse(response["completed"])
        self.assertEqual(response["next_phase"], "positive")
        self.assert_error("CAL_PHASE_ORDER", lambda: self.capture("zero"))
        self.assert_error("CAL_PHASE_ORDER", lambda: self.capture("lower"))
        self.assert_error("CAL_PHASE_ORDER", lambda: self.capture("upper"))

    def test_every_capture_requires_the_support_ack(self):
        self.assert_error("AXIS_CAL_CONFIRMATION",
                          lambda: self.capture("zero", confirmations={}))
        self.assert_error("AXIS_CAL_CONFIRMATION",
                          lambda: self.capture("zero",
                                               confirmations={"axis_calibration_supported": True}))
        self.assertEqual(self.backend._axis_calibration["captures"], {})

    def test_positive_requires_a_real_manual_move_and_yields_only_a_candidate(self):
        self.capture("zero")
        self.assert_error("CAL_DIRECTION_AMBIGUOUS", lambda: self.capture("positive"))
        self.set_positions([0.0, 0.0, 0.0, 0.0, math.radians(300.0)])
        self.assert_error("CAL_SINGLE_TURN_JUMP", lambda: self.capture("positive"))
        response = self.capture("positive",
                                positions=[0.0, 0.0, 0.0, 0.0, 0.1 + math.radians(10.0)])
        candidate = response["capture"]["candidate"]
        self.assertEqual(candidate["candidate_sign"], 1)
        self.assertAlmostEqual(candidate["delta_deg"], 10.0, places=6)
        self.assertTrue(candidate["provisional"])
        self.assertFalse(candidate["formal_mapping_written"])
        session = self.backend._axis_calibration
        self.assertTrue(session["positive_candidate"]["provisional"])
        self.assertFalse(session["positive_candidate"]["formal_mapping_written"])
        self.assertFalse(session["motor_command_sent"])

    def test_negative_manual_move_yields_negative_candidate_sign(self):
        self.capture("zero")
        response = self.capture("positive",
                                positions=[0.0, 0.0, 0.0, 0.0, 0.1 - math.radians(10.0)])
        self.assertEqual(response["capture"]["candidate"]["candidate_sign"], -1)

    def test_limits_must_be_finite_and_strictly_ordered_then_complete(self):
        self.capture("zero")
        self.capture("positive", positions=[0.0, 0.0, 0.0, 0.0, 0.1 + math.radians(10.0)])
        self.capture("lower", positions=[0.0, 0.0, 0.0, 0.0, math.radians(-20.0)])
        self.assert_error("CAL_LIMIT_ORDER",
                          lambda: self.capture("upper",
                                               positions=[0.0, 0.0, 0.0, 0.0,
                                                          math.radians(-30.0)]))
        self.assert_error("CAL_LIMIT_ORDER",
                          lambda: self.capture("upper",
                                               positions=[0.0, 0.0, 0.0, 0.0,
                                                          math.radians(-20.0)]))
        response = self.capture("upper",
                                positions=[0.0, 0.0, 0.0, 0.0, math.radians(40.0)])
        self.assertTrue(response["completed"])
        self.assertIsNone(response["next_phase"])
        limits = response["capture"]["limits_candidate"]
        self.assertAlmostEqual(limits["lower_rad"], math.radians(-20.0), places=9)
        self.assertAlmostEqual(limits["upper_rad"], math.radians(40.0), places=9)
        self.assertTrue(limits["zero_inside"])
        self.assertTrue(limits["provisional"])
        self.assertFalse(limits["formal_mapping_written"])
        display = self.display()
        self.assertEqual(display["phase"], "complete")
        self.assertEqual(display["pending"], [])
        self.assertEqual(display["candidate"]["direction"]["candidate_sign"], 1)
        self.assertFalse(display["formal_writes"]["motor_zero"])
        self.assertFalse(display["formal_writes"]["mapping"])
        self.assertFalse(display["formal_writes"]["urdf"])
        self.assert_error("CAL_INVALIDATED", lambda: self.capture("upper"))

    def test_captures_are_read_only_and_never_a_formal_record(self):
        before_mapping = hashlib.sha256(MAPPING.read_bytes()).hexdigest()
        generation = self.backend._stop_generation
        response = self.complete_flow()
        self.assertTrue(response["completed"])
        self.assertEqual(self.backend._stop_generation, generation)
        self.assertIsNone(self.backend._operation)
        self.assertEqual(self.backend._service,
                         {"stop_all": {}, "reset_disabled": {}, "switch_controller": {}})
        session = self.backend._axis_calibration
        self.assertEqual(session["state"], "complete")
        for phase in PHASES:
            capture = session["captures"][phase]
            self.assertFalse(capture["motor_command_sent"])
            self.assertFalse(capture["motor_zero_written"])
            self.assertFalse(capture["mapping_written"])
            self.assertFalse(capture["formal"])
            self.assertEqual(capture["support_ack"], ACK)
            self.assertEqual(capture["boot_id"], self.identity["boot_id"])
            self.assertEqual(capture["identity_sha256"],
                             session["identity"]["identity_sha256"])
        self.assertFalse(session["formal"])
        audit = self.read_audit()
        self.assertFalse(audit["formal"])
        self.assertEqual(audit["session"]["state"], "complete")
        self.assertEqual(sorted(audit["session"]["captures"]), sorted(PHASES))
        self.assertEqual([item["action"] for item in audit["session"]["transitions"]],
                         ["start", "capture", "capture", "capture", "capture", "complete"])
        self.assertTrue(all(item["motor_command_sent"] is False
                            for item in audit["session"]["transitions"]))
        self.assertTrue(any("轴标定" in event["message"] for event in self.backend._events))
        after_mapping = hashlib.sha256(MAPPING.read_bytes()).hexdigest()
        self.assertEqual(before_mapping, after_mapping)


class Invalidation(Base):
    def setUp(self):
        super().setUp()
        self.inject(positions=[0.0, 0.0, 0.0, 0.0, 0.1])
        self.start()

    def test_identity_change_invalidates_the_sequence(self):
        self.backend._identity = {"at": time.monotonic(),
                                  "payload": dict(self.identity, boot_id="other-boot")}
        self.assert_error("CAL_INVALIDATED", lambda: self.capture("zero"))
        display = self.display()
        self.assertEqual(display["phase"], "invalidated")
        self.assertIn("DEVICE_CHANGED", " ".join(display["reasons"]))
        audit = self.read_audit()
        self.assertEqual(audit["session"]["state"], "invalidated")
        self.assertIn("DEVICE_CHANGED",
                      " ".join(audit["session"]["terminal_reasons"]))
        self.assert_error("CAL_INVALIDATED", lambda: self.capture("zero"))

    def test_stop_invalidates_the_sequence(self):
        self.backend.stop("test stop")
        self.assert_error("CAL_INVALIDATED", lambda: self.capture("zero"))
        display = self.display()
        self.assertEqual(display["phase"], "invalidated")
        reasons = " ".join(display["reasons"])
        self.assertTrue("STOP_LOCKED" in reasons or "STOP_EPOCH_CHANGED" in reasons, reasons)

    def test_feedback_loss_invalidates_the_sequence(self):
        self.backend._dynamic["at"] = time.monotonic() - 10.0
        self.assert_error("CAL_INVALIDATED", lambda: self.capture("zero"))
        self.assertIn("FEEDBACK_STALE", " ".join(self.display()["reasons"]))

    def test_non_disarmed_stage_blocks_capture_without_a_reading(self):
        self.inject(stage=5)
        self.assert_error("CAL_INVALIDATED", lambda: self.capture("zero"))
        self.assertEqual(self.backend._axis_calibration["state"], "invalidated")
        self.assertEqual(self.backend._axis_calibration["captures"], {})

    def test_unknown_invalidates_the_sequence(self):
        self.inject(stage=9)
        self.assert_error("CAL_INVALIDATED", lambda: self.capture("zero"))
        display = self.display()
        self.assertEqual(display["phase"], "invalidated")
        self.assertIn("UNKNOWN", " ".join(display["reasons"]))

    def _dynamic_message(self, motor_state):
        axes = stubs.axes_for(self.names)
        joint_names = []
        interface_values = []
        for name in self.names:
            axes[name]["motor_state"] = motor_state
            joint_names.append(name)
            interface_values.append(SimpleNamespace(interface_names=list(axes[name]),
                                                    values=list(axes[name].values())))
        return SimpleNamespace(joint_names=joint_names, interface_values=interface_values,
                               header=SimpleNamespace(stamp=SimpleNamespace(sec=1,
                                                                            nanosec=0)))

    def test_feedback_update_inside_final_commit_is_rejected(self):
        self.inject()
        self.capture("zero", [0, 0, 0, 0, 0])
        self.capture("positive", [0, 0, 0, 0, .1])
        self.capture("lower", [0, 0, 0, 0, -.3])
        original = self.backend._axis_calibration_axis_feedback
        calls = {"n": 0}

        def update_during_final(core, joint):
            value = original(core, joint)
            calls["n"] += 1
            if calls["n"] == 2:
                self.backend._dynamic_callback(self._dynamic_message(1))
            return value

        self.backend._axis_calibration_axis_feedback = update_during_final
        self.assert_error("CAL_INVALIDATED", lambda: self.capture("upper", [0, 0, 0, 0, .3]))
        self.assertNotIn("upper", self.backend._axis_calibration["captures"])
        self.assertEqual(self.backend._axis_calibration["state"], "active")

    def test_real_thread_feedback_update_before_final_commit_is_rejected(self):
        import threading
        self.inject()
        self.capture("zero", [0, 0, 0, 0, 0])
        self.capture("positive", [0, 0, 0, 0, .1])
        self.capture("lower", [0, 0, 0, 0, -.3])
        original = self.backend._axis_calibration_axis_feedback
        calls = {"n": 0}

        def update_in_thread(core, joint):
            value = original(core, joint)
            calls["n"] += 1
            if calls["n"] == 1:
                thread = threading.Thread(
                    target=lambda: self.backend._dynamic_callback(self._dynamic_message(1)))
                thread.start()
                thread.join(timeout=2)
            return value

        self.backend._axis_calibration_axis_feedback = update_in_thread
        with self.assertRaises(control_panel.PanelError) as caught:
            self.capture("upper", [0, 0, 0, 0, .3])
        self.assertIn(caught.exception.code, ("AXIS_FEEDBACK", "CAL_INVALIDATED", "NOT_DISARMED"))
        self.assertNotIn("upper", self.backend._axis_calibration["captures"])

    def test_feedback_callback_waits_for_calibration_commit_lock(self):
        import threading
        acquired = threading.Event()
        released = threading.Event()
        updated = threading.Event()
        self.backend._calibration_commit_lock.acquire()
        try:
            def update():
                acquired.set()
                self.backend._dynamic_callback(self._dynamic_message(1))
                updated.set()
            thread = threading.Thread(target=update)
            thread.start()
            acquired.wait(1)
            self.assertFalse(updated.wait(0.1), "feedback callback bypassed commit lock")
        finally:
            self.backend._calibration_commit_lock.release()
            released.set()
        thread.join(timeout=2)
        self.assertTrue(updated.is_set())
        self.assertEqual(self.backend._dynamic["axes"][self.names[0]]["motor_state"], 1)

    def test_abort_between_upper_read_and_commit_cannot_complete(self):
        self.inject()
        self.capture("zero", [0, 0, 0, 0, 0])
        self.capture("positive", [0, 0, 0, 0, .1])
        self.capture("lower", [0, 0, 0, 0, -.3])
        original = self.backend._axis_calibration_axis_feedback

        def interleave(core, joint):
            value = original(core, joint)
            self.backend.abort_axis_calibration("injected concurrent abort")
            return value

        self.backend._axis_calibration_axis_feedback = interleave
        self.assert_error("CAL_INVALIDATED", lambda: self.capture("upper", [0, 0, 0, 0, .3]))
        session = self.backend._axis_calibration
        self.assertEqual(session["state"], "aborted")
        self.assertNotIn("upper", session["captures"])
        self.assertEqual([item["action"] for item in session["transitions"]],
                         ["start", "capture", "capture", "capture", "aborted"])

    def test_duplicate_phase_concurrently_cannot_overwrite(self):
        self.inject()
        self.capture("zero", [0, 0, 0, 0, 0])
        self.assert_error("CAL_PHASE_ORDER",
                          lambda: self.capture("zero", [0, 0, 0, 0, 0]))

    def test_abort_invalidates_the_sequence(self):
        response = self.backend.abort_axis_calibration("operator abort")
        self.assertTrue(response["accepted"])
        self.assertEqual(self.display()["phase"], "aborted")
        self.assert_error("CAL_INVALIDATED", lambda: self.capture("zero"))
        audit = self.read_audit()
        self.assertEqual(audit["session"]["state"], "aborted")
        self.assertIn("ABORTED", " ".join(audit["session"]["terminal_reasons"]))

    def test_watchdog_invalidates_on_its_own(self):
        self.backend._dynamic["at"] = time.monotonic() - 10.0
        self.backend._check_axis_calibration(time.monotonic())
        self.assertEqual(self.backend._axis_calibration["state"], "invalidated")

    def test_panel_closing_invalidates_the_sequence(self):
        self.backend._closing.set()
        display = self.display()
        self.assertEqual(display["phase"], "invalidated")
        self.assertIn("PANEL_CLOSING", " ".join(display["reasons"]))


class FourAxisWorkflow(Base):
    joint_count = 4

    def test_four_axis_mode_runs_the_same_manual_workflow(self):
        positions = [0.0, 0.0, 0.0, 0.0]
        self.inject(positions=positions)
        response = self.backend.start_axis_calibration(self.names[0], dict(ACK))
        self.session_id = response["session_id"]
        self.assertEqual(response["axis_calibration"]["joints"][0]["label"], "J1")
        self.capture("zero")
        positive = self.capture("positive", positions=[math.radians(10.0), 0.0, 0.0, 0.0])
        self.assertEqual(positive["capture"]["candidate"]["candidate_sign"], 1)
        self.capture("lower", positions=[math.radians(-20.0), 0.0, 0.0, 0.0])
        response = self.capture("upper", positions=[math.radians(40.0), 0.0, 0.0, 0.0])
        self.assertTrue(response["completed"])
        self.assertEqual(response["capture"]["limits_candidate"]["lower_rad"],
                         math.radians(-20.0))


class HttpRoutes(Base):
    def setUp(self):
        super().setUp()
        self.server = PanelServer(self.backend, TOKEN, self.backend.static_dir, 0)
        self.server.start()
        self.addCleanup(self.server.stop)
        self.port = self.server.port
        self.origin = "http://127.0.0.1:%d" % self.port
        self.host = "127.0.0.1:%d" % self.port

    def request(self, method, path, body=None, *, token=True, origin=True, host=None):
        headers = {"Host": host or self.host}
        if token:
            headers["X-Panel-Token"] = TOKEN
        payload = None
        if body is not None:
            payload = json.dumps(body).encode("utf-8")
            headers["Content-Type"] = "application/json"
            headers["Content-Length"] = str(len(payload))
        if method == "POST" and origin is not False:
            headers["Origin"] = self.origin if origin is True else str(origin)
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        try:
            connection.request(method, path, body=payload, headers=headers)
            response = connection.getresponse()
            return response.status, response.read(), dict(response.getheaders())
        finally:
            connection.close()

    def test_status_and_detail_routes_expose_the_non_formal_workflow(self):
        status, body, _ = self.request("GET", "/api/status")
        self.assertEqual(status, 200)
        payload = json.loads(body)
        display = payload["axis_calibration"]
        self.assertEqual(display["schema"], model.AXIS_CALIBRATION_SCHEMA)
        self.assertFalse(display["formal"])
        self.assertEqual(len(display["joints"]), 5)
        self.assertEqual([item["name"] for item in display["phases"]], PHASES)
        self.assertIn("capabilities", display)
        status, body, _ = self.request("GET", "/api/calibration/axis")
        self.assertEqual(status, 200)
        detail = json.loads(body)
        self.assertTrue(detail["ok"])
        self.assertEqual(detail["axis_calibration"]["schema"],
                         model.AXIS_CALIBRATION_SCHEMA)

    def test_start_capture_abort_routes_drive_the_real_backend(self):
        self.inject(positions=[0.0, 0.0, 0.0, 0.0, 0.1])
        status, body, _ = self.request("POST", "/api/calibration/axis/start",
                                       {"joint": self.names[-1], "confirmations": ACK})
        self.assertEqual(status, 202, body)
        started = json.loads(body)
        self.assertTrue(started["ok"])
        session_id = started["session_id"]
        self.assertTrue(session_id)
        status, body, _ = self.request("POST", "/api/calibration/axis/capture",
                                       {"session_id": session_id, "phase": "positive",
                                        "confirmations": ACK})
        self.assertEqual(status, 409)
        self.assertEqual(json.loads(body)["code"], "CAL_PHASE_ORDER")
        status, body, _ = self.request("POST", "/api/calibration/axis/capture",
                                       {"phase": "zero", "confirmations": ACK})
        self.assertEqual(status, 400)
        self.assertEqual(json.loads(body)["code"], "REQUEST")
        status, body, _ = self.request("POST", "/api/calibration/axis/capture",
                                       {"session_id": session_id, "phase": "zero",
                                        "confirmations": ACK})
        self.assertEqual(status, 202, body)
        capture = json.loads(body)
        self.assertEqual(capture["phase"], "zero")
        self.assertAlmostEqual(capture["capture"]["position_deg"], math.degrees(0.1), places=6)
        status, body, _ = self.request("GET", "/api/status")
        active = json.loads(body)["axis_calibration"]
        self.assertEqual(active["next_phase"], "positive")
        self.assertEqual(active["session_id"], session_id)
        status, body, _ = self.request("POST", "/api/calibration/axis/abort",
                                       {"reason": "http test abort"})
        self.assertEqual(status, 202, body)
        aborted = json.loads(body)
        self.assertTrue(aborted["completed"])
        self.assertEqual(aborted["axis_calibration"]["phase"], "aborted")

    def test_routes_enforce_existing_token_origin_host_and_csp(self):
        start_body = {"joint": self.names[-1], "confirmations": ACK}
        status, body, _ = self.request("POST", "/api/calibration/axis/start",
                                       start_body, token=False)
        self.assertEqual(status, 403)
        self.assertEqual(json.loads(body)["code"], "TOKEN")
        status, body, _ = self.request("GET", "/api/status", token=False)
        self.assertEqual(status, 403)
        self.assertEqual(json.loads(body)["code"], "TOKEN")
        status, body, _ = self.request("POST", "/api/calibration/axis/start",
                                       start_body, origin=False)
        self.assertEqual(status, 403)
        self.assertEqual(json.loads(body)["code"], "ORIGIN")
        status, body, _ = self.request("POST", "/api/calibration/axis/start",
                                       start_body, origin="http://evil.example")
        self.assertEqual(status, 403)
        self.assertEqual(json.loads(body)["code"], "ORIGIN")
        status, body, _ = self.request("GET", "/api/status", host="evil.example:1")
        self.assertEqual(status, 403)
        self.assertEqual(json.loads(body)["code"], "HOST")
        status, body, headers = self.request("GET", "/?token=" + TOKEN)
        self.assertEqual(status, 200)
        self.assertIn("default-src 'none'",
                      headers.get("Content-Security-Policy", ""))
        # A rejected sequence start did not mutate state.
        self.assertIsNone(self.backend._axis_calibration)


class UiStatic(unittest.TestCase):
    def test_calibration_card_is_inside_the_existing_teaching_sampling_tab(self):
        html = (STATIC / "index.html").read_text(encoding="utf-8")
        self.assertIn('id="section-axis-calibration"', html)
        self.assertRegex(html, re.compile(
            r'<section class="card" id="section-axis-calibration" data-tab="calibration">'))
        for element in ("axis-cal-joint", "btn-axis-cal-start", "btn-axis-cal-abort",
                        "btn-axis-cal-zero", "btn-axis-cal-positive",
                        "btn-axis-cal-lower", "btn-axis-cal-upper",
                        "ack-axis-cal-supported", "ack-axis-cal-clearance",
                        "axis-cal-flags", "axis-cal-captures", "cap-axis-cal"):
            self.assertIn('id="%s"' % element, html)
        self.assertIn("零位 → 正方向 → 下限 → 上限", html)
        self.assertIn("不使能", html)
        self.assertIn("不写电机零位", html)
        self.assertNotIn("<script>", html)
        self.assertNotIn("onclick=", html)
        self.assertNotIn("style=", html)
        # The five existing workspaces are preserved.
        for tab in ("overview", "plan", "gravity", "calibration", "operation"):
            self.assertIn('data-tab="%s"' % tab, html)
        self.assertGreaterEqual(html.count('class="tab'), 5)

    def test_panel_js_wires_start_capture_abort_without_fake_values(self):
        js = (STATIC / "panel.js").read_text(encoding="utf-8")
        for fragment in ("renderAxisCalibration", "updateAxisCalButtons",
                         "axisCalConfirmations", "/api/calibration/axis/start",
                         "/api/calibration/axis/capture", "/api/calibration/axis/abort",
                         "axis_calibration_supported", "axis_calibration_clearance",
                         "AXIS_CAL_PHASES", "候选，非正式", "未生成（不写零位"):
            self.assertIn(fragment, js)
        self.assertIn("const AXIS_COUNT = 5;", js)
        self.assertNotIn("/api/j5", js)
        self.assertNotIn("renderJ5", js)

    def test_panel_js_keeps_the_five_axis_plan_and_register_gate(self):
        source = (SCRIPTS / "control_panel.py").read_text(encoding="utf-8")
        # Another owner may wire the five-axis client later; this workflow must
        # not remove or weaken the existing client-axis gate.
        self.assertIn('"CLIENT_AXES"', source)
        self.assertIn("client_axes != FIVE_AXES", source)
        self.assertIn("validate_axis_calibration_confirmations", source)
        self.assertIn("不写", source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
