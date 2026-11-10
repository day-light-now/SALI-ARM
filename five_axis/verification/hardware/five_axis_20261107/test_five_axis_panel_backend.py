"""Backend-level five-axis/four-axis compatibility tests with ROS stubs.

No ROS installation, no driver, no serial: the ROS module names are replaced
with inert stubs only for this test process, then the production PanelBackend
logic is driven with injected state.  This pins the unified gate, the single
J1-J5 table and the absence of a second J5 owner.
"""
import hashlib
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SCRIPTS = ROOT / "src" / "slaihand_bringup" / "scripts"
DESCRIPTION = ROOT / "src" / "slaihand_description"
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(HERE))

import test_five_axis_plan as five  # noqa: E402


# ---------------------------------------------------------------- ROS stubs
def _module(name, **attributes):
    module = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(module, key, value)
    sys.modules[name] = module
    return module


class _QoS:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class _Message:
    def __init__(self, *args, **kwargs):
        pass


class _TriggerRequest:
    pass


class _SwitchRequest:
    pass


class _SignalOptions:
    NO = object()


class _Reliability:
    RELIABLE = object()
    BEST_EFFORT = object()


class _Durability:
    TRANSIENT_LOCAL = object()
    VOLATILE = object()


SHARE = {}

_module("rclpy")
_module("rclpy.qos", DurabilityPolicy=_Durability, ReliabilityPolicy=_Reliability, QoSProfile=_QoS)
_module("rclpy.signals", SignalHandlerOptions=_SignalOptions)
_module("ament_index_python")
_module("ament_index_python.packages",
        get_package_prefix=lambda name: str(HERE / "_missing_prefix_" / name),
        get_package_share_directory=lambda name: SHARE.get(name, str(HERE / "_missing_share_" / name)))
_module("control_msgs")
_module("control_msgs.msg", DynamicJointState=_Message, JointTrajectoryControllerState=_Message)
_module("controller_manager_msgs")
_module("controller_manager_msgs.srv", SwitchController=type("SwitchController", (), {"Request": _SwitchRequest}))
_module("rcl_interfaces")
_param_type = type("ParameterType", (), {"PARAMETER_DOUBLE": 1, "PARAMETER_STRING": 2})
_module("rcl_interfaces.msg", Parameter=_Message, ParameterType=_param_type,
        ParameterValue=_Message)
_module("rcl_interfaces.srv", SetParametersAtomically=type("SetParametersAtomically", (), {}))
_module("sensor_msgs")
_module("sensor_msgs.msg", JointState=_Message)
_module("std_srvs")
_module("std_srvs.srv", Trigger=type("Trigger", (), {"Request": _TriggerRequest}))
_module("std_msgs")
_module("std_msgs.msg", Float64MultiArray=_Message, String=_Message)

import control_panel  # noqa: E402


class FakeNode:
    def create_subscription(self, *args, **kwargs):
        return None

    def create_client(self, *args, **kwargs):
        return None


def runtime_identity(mapping_sha, load_id="j5_3p5kg_candidate"):
    return {
        "schema": "slaihand_runtime_identity/1", "boot_id": "test-boot",
        "started_at": 1.0,
        "mapping_sha256": mapping_sha, "model_sha256": "a" * 64,
        "policy_sha256": "b" * 64, "mass_profile_sha256": "c" * 64,
        "policy_profile": "test", "load_id": load_id,
        "gravity_base_m_s2": [0.0, 9.80665, 0.0], "j3_offset_rad": 3.141592653589793,
        "kp": 160.0, "kd": 5.0, "reference_speed_rad_s": 1.0,
        "gravity_ff_alpha": 0.0, "gravity_ff_ramp_s": 1.0,
        "active_axes": 5, "j5_motion_qualified": True,
        "j5_geometry": True, "j5_geometry_sha256": "d" * 64,
    }


def axes_for(names, stage=0):
    return {
        name: {"position": 0.0, "velocity": 0.0, "feedback_age": 0.01,
               "feedback_valid": 1, "control_state": stage,
               "hold_ready": 1, "disabled_observed": 1 if stage in (0, 8) else 0,
               "command_owner": 0, "audit_ok": 1, "command_id": 1,
               "motor_state": 0 if stage in (0, 8) else 1, "effort": 0.0,
               "driver_temperature": 30.0, "motor_temperature": 30.0}
        for name in names
    }


def inject(backend, names, *, stage=0, positions=None, identity=None):
    now = time.monotonic()
    backend._dynamic = {"at": now, "axes": axes_for(names, stage)}
    backend._joints = {"at": now,
                       "positions": [0.0] * len(names) if positions is None else positions}
    backend._controller = None
    if identity is not None:
        backend._identity = {"at": now, "payload": identity}


class Base(unittest.TestCase):
    joint_count = 5
    qualified_mapping = False

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="five-axis-panel-")
        self.addCleanup(self.tmp.cleanup)
        if self.qualified_mapping:
            bringup = Path(self.tmp.name) / "bringup"
            config = bringup / "config"
            config.mkdir(parents=True)
            (config / "hardware_mapping.yaml").write_text(
                five.qualified_mapping_text(), encoding="utf-8")
            SHARE["slaihand_bringup"] = str(bringup)
        else:
            SHARE["slaihand_bringup"] = str(ROOT / "src" / "slaihand_bringup")
        SHARE["slaihand_description"] = str(DESCRIPTION)
        self.backend = control_panel.PanelBackend(
            FakeNode(), self.tmp.name, 60, 30, "token", gravity_enabled=False,
            joint_count=self.joint_count)


class FiveAxisGate(Base):
    joint_count = 5

    def test_single_table_has_five_rows_and_j5_pending(self):
        status = self.backend.status()
        self.assertEqual(len(status["axes"]), 5)
        self.assertEqual([row["label"] for row in status["axes"]],
                         ["J1", "J2", "J3", "J4", "J5"])
        self.assertEqual(status["server"]["joint_count"], 5)
        self.assertEqual(status["unified"]["axes"], 5)
        self.assertNotIn("j5", status)  # no separate read-only card/status object
        self.assertFalse(status["capabilities"]["preview"]["ok"])
        self.assertTrue(status["unified"]["preflight"]["reasons"])
        self.assertIn("J5_IDENTITY_NOT_READY",
                      " ".join(status["unified"]["preflight"]["reasons"]))
        self.assertIn("待定", status["axes"][4]["qualification"])
        self.assertIsNone(status["axes"][4]["actual_deg"])

    def test_missing_j5_feedback_refuses_motion_with_j5_preflight(self):
        # Fresh J1-J4 feedback but the same frame has no J5, and no J5 position.
        names4 = ["slaihand_joint%d" % index for index in range(1, 5)]
        now = time.monotonic()
        self.backend._dynamic = {"at": now, "axes": axes_for(names4)}
        self.backend._joints = {"at": now, "positions": None}
        with self.assertRaises(control_panel.PanelError) as caught:
            self.backend.preview(five.five_document())
        self.assertEqual(caught.exception.code, "J5_PREFLIGHT")
        reasons = caught.exception.extra["j5_preflight"]["reasons"]
        self.assertTrue(any(reason.startswith("J5_FEEDBACK_MISSING") for reason in reasons))
        self.assertFalse(any(reason.startswith("J5_TIMEOUT_INVALID") for reason in reasons))

    def test_hold_and_gravity_float_are_refused_by_the_same_gate(self):
        confirmations = {"supported": True, "clearance_and_local_power_cut": True,
                         "native_output_not_hardware_clamp": True}
        for action in (lambda: self.backend.hold(dict(confirmations)),
                       lambda: self.backend.hold(dict(confirmations), gravity_float=True)):
            with self.assertRaises(control_panel.PanelError) as caught:
                action()
            self.assertIn(caught.exception.code,
                          ("J5_PREFLIGHT", "FEEDBACK_STALE", "FLOAT_UNAVAILABLE"))

    def test_stop_covers_all_five_axes(self):
        response = self.backend.stop("test stop")
        names = ["slaihand_joint%d" % index for index in range(1, 6)]
        self.assertEqual(response["axes"], names)
        self.assertEqual(response["stop_gate"]["covers_axes"], names)
        self.assertEqual(self.backend.status()["unified"]["stop_axes"], names)

    def test_reconnect_cannot_start_a_second_owner_or_four_axis_launch(self):
        with self.assertRaises(control_panel.PanelError) as caught:
            self.backend.reconnect(ack_power_cycle=True)
        self.assertEqual(caught.exception.code, "CONNECTION_AXES")

    def test_old_four_axis_document_never_becomes_five_axis(self):
        legacy = five.five_document(degrees=(1, 2, 3, 4), schema=None)
        inject(self.backend, ["slaihand_joint%d" % index for index in range(1, 6)])
        saved = self.backend.save_plan("legacy", legacy)
        self.assertEqual(saved["plan"]["document"]["schema"],
                         five.model.LEGACY_PLAN_SCHEMA)
        self.assertEqual(len(saved["plan"]["document"]["waypoints"][0]["degrees"]), 4)
        loaded = self.backend.load_plan(saved["plan"]["id"])
        self.assertEqual(loaded["document"]["schema"], five.model.LEGACY_PLAN_SCHEMA)
        # It is refused by the unified five-axis execution gate.
        with self.assertRaises(control_panel.PanelError) as caught:
            self.backend.preview(loaded["document"])
        self.assertIn(caught.exception.code, ("J5_PREFLIGHT", "PLAN_INVALID"))
        self.assertTrue(caught.exception.message)


class QualifiedFiveAxisGate(Base):
    joint_count = 5
    qualified_mapping = True

    def test_qualified_five_axis_plan_is_accepted_with_one_entry(self):
        names = ["slaihand_joint%d" % index for index in range(1, 6)]
        mapping = (Path(SHARE["slaihand_bringup"]) / "config" / "hardware_mapping.yaml").read_bytes()
        sha = hashlib.sha256(mapping).hexdigest()
        inject(self.backend, names, identity=runtime_identity(sha))
        status = self.backend.status()
        self.assertTrue(status["unified"]["preflight"]["ok"], status["unified"]["preflight"])
        self.assertTrue(status["unified"]["j5"]["qualified"])
        self.assertEqual(status["unified"]["gravity_identity"]["status"], "QUALIFIED")
        response = self.backend.preview(five.five_document())
        self.assertTrue(response["ok"])
        self.assertEqual(response["preview"]["axes"], 5)
        self.assertEqual(response["preview"]["waypoints"][0]["target_deg"],
                         [1.0, 2.0, -3.0, 4.0, 5.0])
        self.assertEqual(response["preview"]["frozen"]["schema"], five.model.PLAN_SCHEMA)
        # A fully qualified simulation may hand a five-axis hold to the real
        # protected client.  Popen is faked so this test never starts a ROS node.
        confirmations = {"supported": True, "clearance_and_local_power_cut": True,
                         "native_output_not_hardware_clamp": True}
        fake = mock.MagicMock()
        fake.pid = 4242
        fake.poll.return_value = None
        fake.stdout = iter(())
        with mock.patch.object(control_panel.subprocess, "Popen", return_value=fake) as popen:
            held = self.backend.hold(dict(confirmations))
        self.assertTrue(held["ok"])
        argv = popen.call_args.args[0]
        self.assertIn("--joint-count", argv)
        self.assertEqual(argv[argv.index("--joint-count") + 1], "5")
        self.assertIn("--j5-limits-deg", argv)
        self.assertIn("--expected-identity-sha256", argv)
        client = Path(self.backend.client_script).read_text(encoding="utf-8")
        self.assertIn("--joint-count", client)
        self.assertEqual(self.backend.client_axes, 5)

    def test_preview_identity_is_frozen_until_launch(self):
        names = ["slaihand_joint%d" % index for index in range(1, 6)]
        mapping = (Path(SHARE["slaihand_bringup"]) / "config" / "hardware_mapping.yaml").read_bytes()
        sha = hashlib.sha256(mapping).hexdigest()
        inject(self.backend, names, identity=runtime_identity(sha))
        preview = self.backend.preview(five.five_document())
        changed = dict(runtime_identity(sha), boot_id="changed-boot", policy_sha256="f" * 64)
        inject(self.backend, names, identity=changed)
        fake = mock.MagicMock()
        with mock.patch.object(control_panel.subprocess, "Popen", return_value=fake) as popen:
            with self.assertRaises(control_panel.PanelError) as caught:
                self.backend.execute(
                    preview["ticket"]["id"], five.five_document(),
                    {"supported": True, "clearance_and_local_power_cut": True,
                     "native_output_not_hardware_clamp": True})
        self.assertEqual(caught.exception.code, "IDENTITY_CHANGED")
        self.assertFalse(popen.called)
        self.assertIsNone(self.backend._operation)

    def test_wrong_dimension_is_refused_even_when_j5_is_qualified(self):
        names = ["slaihand_joint%d" % index for index in range(1, 6)]
        mapping = (Path(SHARE["slaihand_bringup"]) / "config" / "hardware_mapping.yaml").read_bytes()
        sha = hashlib.sha256(mapping).hexdigest()
        inject(self.backend, names, identity=runtime_identity(sha))
        with self.assertRaises(control_panel.PanelError) as caught:
            self.backend.preview(five.five_document(degrees=(1, 2, 3, 4, 5, 6)))
        self.assertEqual(caught.exception.code, "PLAN_INVALID")


class FourAxisCompatibility(Base):
    joint_count = 4

    def test_four_axis_mode_keeps_one_table_and_legacy_plan(self):
        names = ["slaihand_joint%d" % index for index in range(1, 5)]
        inject(self.backend, names)
        status = self.backend.status()
        self.assertEqual(len(status["axes"]), 4)
        self.assertEqual(status["unified"]["axes"], 4)
        self.assertTrue(status["unified"]["j5"]["required"] is False)
        legacy = five.five_document(degrees=(1, 2, 3, 4), schema=None)
        response = self.backend.preview(legacy)
        self.assertTrue(response["ok"])
        self.assertEqual(response["preview"]["axes"], 4)
        self.assertEqual(response["preview"]["schema"], five.model.LEGACY_PLAN_SCHEMA)
        with self.assertRaises(control_panel.PanelError) as caught:
            self.backend.preview(five.five_document())
        self.assertEqual(caught.exception.code, "PLAN_INVALID")


if __name__ == "__main__":
    unittest.main(verbosity=2)
