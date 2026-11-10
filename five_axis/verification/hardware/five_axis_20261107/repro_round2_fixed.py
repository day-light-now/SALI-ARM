"""Round-two counterexample checks after the minimal fixes (no ROS/hardware)."""
import ast
import hashlib
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import test_five_axis_calibration as t  # noqa: E402

results = []


def capture_error(call):
    try:
        return {"accepted": True, "result": call()}
    except Exception as error:
        return {"accepted": False, "code": getattr(error, "code", type(error).__name__),
                "message": str(error)[:240]}


for axis in ("slaihand_joint1", "slaihand_joint5"):
    c = t.Base(); c.setUp()
    try:
        c.inject(); c.backend._dynamic["axes"][axis]["motor_state"] = 1
        results.append({"case": "enabled_motor_calibration", "axis": axis,
                        **capture_error(c.start)})
    finally:
        c.doCleanups()

c = t.Base(); c.setUp()
try:
    c.inject(); c.start()
    c.capture("zero", [0, 0, 0, 0, 0])
    c.capture("positive", [0, 0, 0, 0, .1])
    c.capture("lower", [0, 0, 0, 0, -.3])
    original = c.backend._axis_calibration_axis_feedback

    def interleave(core, joint):
        value = original(core, joint)
        c.backend.abort_axis_calibration("injected concurrent abort")
        return value

    c.backend._axis_calibration_axis_feedback = interleave
    outcome = capture_error(lambda: c.capture("upper", [0, 0, 0, 0, .3]))
    results.append({"case": "abort_before_upper_commit", **outcome,
                    "state": c.backend._axis_calibration["state"],
                    "transitions": [v["action"] for v in c.backend._axis_calibration["transitions"]]})
finally:
    c.doCleanups()

s = t.stubs
c = s.QualifiedFiveAxisGate(); c.setUp()
try:
    b = c.backend
    sha = hashlib.sha256((Path(s.SHARE["slaihand_bringup"]) / "config/hardware_mapping.yaml").read_bytes()).hexdigest()
    identity = s.runtime_identity(sha)
    names = list(b.joint_names)
    s.inject(b, names, identity=identity)
    doc = s.five.five_document(); preview = b.preview(doc)
    s.inject(b, names, identity=dict(identity, boot_id="changed-boot", policy_sha256="f" * 64))
    calls = []
    b._launch = lambda *args, **kwargs: calls.append(args) or {"ok": True}
    results.append({"case": "identity_changed_after_preview",
                    **capture_error(lambda: b.execute(
                        preview["ticket"]["id"], doc,
                        {"supported": True, "clearance_and_local_power_cut": True,
                         "native_output_not_hardware_clamp": True})),
                    "launch_calls": len(calls)})
finally:
    c.doCleanups()

module = ast.parse((ROOT / "src/slaihand_bringup/scripts/commission_supported_trial.py").read_text(encoding="utf-8-sig"))
function = next(node for node in ast.walk(module)
                if isinstance(node, ast.FunctionDef) and node.name == "float_settings_confirmed")


def float_case(length, row, joint_count, layout=None, expected_error=None):
    scope = {"seen": {"float_state": row, "float_state_at": time.monotonic(),
                      "float_state_layout": layout},
             "time": time, "result": {"command_id": 7, "joint_count": joint_count},
             "a": SimpleNamespace(float_kd=2.0, hold_seconds=10.0)}
    exec(compile(ast.Module(body=[function], type_ignores=[]), "client", "exec"), scope)
    accepted = scope["float_settings_confirmed"]()
    return {"length": length, "joint_count": joint_count, "accepted": accepted,
            "error": scope["result"].get("float_settings_error"),
            "expected_error": expected_error}


row43 = [1., 3., 1., 5., 7.] + [0.] * 20 + [1., 2., 10.] + [0., 0., 100.] + [0.] * 10 + [0., 7.]
results.append({"case": "five_axis_valid_float_config",
                **float_case(43, row43, 5)})
results.append({"case": "wrong_axis_float_config",
                **float_case(43, row43, 4)})

layout_banana = SimpleNamespace(dim=[SimpleNamespace(label="slaihand_gravity_ff_state/banana", size=5),
                                     SimpleNamespace(label="float", size=1)])
state, error = t.model.parse_gravity_ff_state([0.] * 43, joint_count=5, layout=layout_banana)
results.append({"case": "malformed_explicit_ff_version", "accepted": state is not None,
                "error": error})
layout_float2 = SimpleNamespace(dim=[SimpleNamespace(label="slaihand_gravity_ff_state/2", size=5),
                                     SimpleNamespace(label="float", size=2)])
state, error = t.model.parse_gravity_ff_state([0.] * 43, joint_count=5, layout=layout_float2)
results.append({"case": "malformed_explicit_float_dimension", "accepted": state is not None,
                "error": error})
state, error = t.model.parse_gravity_ff_state([0.] * 43, joint_count=5)
results.append({"case": "unmarked_v2_payload", "accepted": state is not None,
                "error": error, "axes": None if state is None else state["axes"]})

Path(__file__).with_suffix(".json").write_text(json.dumps(results, ensure_ascii=False, indent=2),
                                               encoding="utf-8")
print(json.dumps(results, ensure_ascii=False, indent=2))
