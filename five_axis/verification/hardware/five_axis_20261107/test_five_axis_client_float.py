"""Regression test for the protected client's five-axis gravity_float receipt."""
import ast
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "src/slaihand_bringup/scripts"
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(Path(__file__).resolve().parent))


def load_function():
    module = ast.parse((SCRIPTS / "commission_supported_trial.py").read_text(encoding="utf-8-sig"))
    return next(node for node in ast.walk(module)
                if isinstance(node, ast.FunctionDef) and node.name == "float_settings_confirmed")


def state_for(length, joint_count, values, layout=None):
    seen = {"float_state": values, "float_state_at": time.monotonic(),
            "float_state_layout": layout}
    result = {"command_id": 7, "joint_count": joint_count}
    scope = {"seen": seen, "time": time, "result": result,
             "a": SimpleNamespace(float_kd=2.0, hold_seconds=10.0)}
    code = compile(ast.Module(body=[load_function()], type_ignores=[]),
                   "client-float", "exec")
    exec(code, scope)
    return scope["float_settings_confirmed"](), result, scope


def row(length):
    values = [0.0] * length
    values[:5] = [1.0, 3.0, 1.0, 5.0, 7.0]
    if length == 43:
        values[25:28] = [1.0, 2.0, 10.0]
    if length == 24:
        values[21:24] = [1.0, 2.0, 10.0]
    return values


class ClientFloat(unittest.TestCase):
    def test_five_axis_43_field_receipt_is_accepted(self):
        accepted, result, _ = state_for(43, 5, row(43))
        self.assertTrue(accepted)
        self.assertEqual(result["float_settings_protocol"], "slaihand_gravity_ff_state/2")

    def test_five_axis_client_rejects_four_axis_payload(self):
        accepted, result, _ = state_for(37, 5, row(37))
        self.assertFalse(accepted)
        self.assertEqual(result["float_settings_error"]["code"], "FF_AXIS_MISMATCH")

    def test_four_axis_24_field_receipt_remains_compatible(self):
        accepted, _, _ = state_for(24, 4, row(24))
        self.assertTrue(accepted)

    def test_wrong_axis_count_for_valid_five_axis_payload_rejected(self):
        accepted, result, _ = state_for(43, 4, row(43))
        self.assertFalse(accepted)
        self.assertEqual(result["float_settings_error"]["code"], "FF_AXIS_MISMATCH")


if __name__ == "__main__":
    unittest.main(verbosity=2)
