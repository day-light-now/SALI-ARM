"""Regression for the production gravity model / coordinate map / shadow estimate."""
import json
import math
import random
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SCRIPTS = ROOT / "src/slaihand_bringup/scripts"
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(ROOT / "verification"))

import control_panel_gravity as gravity  # noqa: E402
from gravity_model import GravityModel as ReferenceGravityModel  # noqa: E402

URDF = ROOT / "src/slaihand_description/urdf/slaihand_four_axis.urdf"
PROFILE = ROOT / "src/slaihand_bringup/config/gravity_provisional.json"
MAPPING = ROOT / "src/slaihand_bringup/config/hardware_mapping.yaml"
POLICY = ROOT / "src/slaihand_bringup/config/canusb_motion_policy.yaml"
OFFSETS = {"slaihand_joint3": math.pi}
GRAVITY = (0.0, 9.80665, 0.0)


def synthetic_pendulum(directory, mass=2.0, length=0.5):
    """base -> link1, revolute about +y, COM at (length,0,0)."""
    urdf = Path(directory) / "pendulum.urdf"
    urdf.write_text("""<?xml version="1.0"?>
<robot name="pend">
  <link name="base"><visual><geometry><box size="0.01 0.01 0.01"/></geometry></visual></link>
  <link name="link1"><visual><geometry><box size="0.01 0.01 0.01"/></geometry></visual></link>
  <joint name="pivot" type="revolute">
    <parent link="base"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.3" upper="3.3" effort="10" velocity="1"/>
  </joint>
</robot>
""", encoding="utf-8")
    profile = Path(directory) / "pendulum_mass.json"
    profile.write_text(json.dumps({"metadata": {"status": "CALIBRATED_TEST"},
                                   "links": {"link1": {"mass_kg": mass,
                                                       "com_link_m": [length, 0.0, 0.0]}}}),
                       encoding="utf-8")
    return urdf, profile


class PendulumCase(unittest.TestCase):
    def test_analytic_pendulum_holding_torque(self):
        with tempfile.TemporaryDirectory() as directory:
            urdf, profile = synthetic_pendulum(directory, mass=2.0, length=0.5)
            model = gravity.FourAxisGravityModel(urdf, profile, gravity_base=(0, 0, -9.80665),
                                                 joint_names=("pivot",))
            # p = L*(cos q, 0, -sin q); U = +m g L sin q; tau = m g L cos q
            for q in (0.0, math.pi / 6, math.pi / 2, math.pi):
                torque = model.compute([q])["tau_model_nm"][0]
                expected = 2.0 * 9.80665 * 0.5 * (-math.cos(q))
                self.assertAlmostEqual(torque, expected, places=12)

    def test_energy_finite_difference(self):
        with tempfile.TemporaryDirectory() as directory:
            urdf, profile = synthetic_pendulum(directory)
            model = gravity.FourAxisGravityModel(urdf, profile, gravity_base=(0, 0, -9.80665),
                                                 joint_names=("pivot",))
            q = 0.7
            h = 1e-6
            finite = (model.potential_energy([q + h]) - model.potential_energy([q - h])) / (2 * h)
            self.assertAlmostEqual(model.compute([q])["tau_model_nm"][0], finite, places=8)


class ExistingArmCase(unittest.TestCase):
    def setUp(self):
        self.model = gravity.FourAxisGravityModel(URDF, PROFILE, gravity_base=GRAVITY,
                                                  fixed_joint_offsets=OFFSETS)

    def random_pose(self, rng):
        return [rng.uniform(low, high) for low, high in self.model.limits]

    def test_metadata_and_hashes(self):
        metadata = self.model.metadata
        self.assertEqual(metadata["joint_names"], list(gravity.JOINT_NAMES))
        self.assertEqual(len(metadata["urdf_sha256"]), 64)
        self.assertEqual(len(metadata["mass_profile_sha256"]), 64)
        self.assertEqual(metadata["mass_profile_status"], "ESTIMATE_ONLY_NOT_FOR_HARDWARE")
        expected_total = sum(v["mass_kg"] for v in
                             json.loads(PROFILE.read_text(encoding="utf-8"))["links"].values())
        self.assertAlmostEqual(metadata["total_mass_kg"], round(expected_total, 6), places=9)
        self.assertAlmostEqual(expected_total, 6.3424, places=3)
        self.assertEqual(metadata["gravity_base_m_s2"], list(GRAVITY))

    def test_matches_existing_reference_model(self):
        reference = ReferenceGravityModel(URDF, json.loads(PROFILE.read_text(encoding="utf-8"))["links"],
                                          GRAVITY, fixed_joint_offsets=OFFSETS)
        rng = random.Random(20261105)
        for _ in range(40):
            q = self.random_pose(rng)
            expected, _ = reference.evaluate(q)
            got = self.model.compute(q)["tau_model_nm"]
            for a, b in zip(expected, got):
                self.assertLess(abs(a - b), 1e-10)

    def test_energy_finite_difference_real_arm(self):
        rng = random.Random(7)
        h = 1e-6
        for _ in range(5):
            q = self.random_pose(rng)
            torque = self.model.compute(q)["tau_model_nm"]
            for index in range(4):
                forward = list(q)
                backward = list(q)
                forward[index] += h
                backward[index] -= h
                finite = (self.model.potential_energy(forward)
                          - self.model.potential_energy(backward)) / (2 * h)
                self.assertLess(abs(torque[index] - finite), 1e-6)

    def test_j3_offset_is_applied_exactly_once(self):
        without = gravity.FourAxisGravityModel(URDF, PROFILE, gravity_base=GRAVITY)
        doubled = gravity.FourAxisGravityModel(URDF, PROFILE, gravity_base=GRAVITY,
                                               fixed_joint_offsets={"slaihand_joint3": 2 * math.pi})
        q = [0.3, 0.5, 0.4, 0.2]
        once = self.model.compute(q)["tau_model_nm"]
        self.assertNotEqual([round(v, 6) for v in once], [round(v, 6) for v in without.compute(q)["tau_model_nm"]])
        # 2*pi about the joint axis is identity: applying "twice" must equal no offset.
        for a, b in zip(doubled.compute(q)["tau_model_nm"], without.compute(q)["tau_model_nm"]):
            self.assertLess(abs(a - b), 1e-9)

    def test_zero_gravity_gives_zero_torque(self):
        model = gravity.FourAxisGravityModel(URDF, PROFILE, gravity_base=(0, 0, 0),
                                             fixed_joint_offsets=OFFSETS)
        torque = model.compute([0.3, 0.5, 0.4, 0.2])["tau_model_nm"]
        self.assertTrue(all(abs(v) < 1e-12 for v in torque))

    def test_mass_scaling_is_linear(self):
        profile = json.loads(PROFILE.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as directory:
            scaled = {"metadata": profile["metadata"],
                      "links": {k: {"mass_kg": v["mass_kg"] * 2.0, "com_link_m": v["com_link_m"]}
                                for k, v in profile["links"].items()}}
            path = Path(directory) / "scaled.json"
            path.write_text(json.dumps(scaled), encoding="utf-8")
            model = gravity.FourAxisGravityModel(URDF, path, gravity_base=GRAVITY,
                                                 fixed_joint_offsets=OFFSETS)
            q = [0.3, 0.5, 0.4, 0.2]
            for a, b in zip(model.compute(q)["tau_model_nm"], self.model.compute(q)["tau_model_nm"]):
                self.assertAlmostEqual(a, 2.0 * b, places=10)

    def test_out_of_limit_is_rejected(self):
        with self.assertRaises(gravity.GravityModelError):
            self.model.compute([3.5, 0, 0, 0])
        with self.assertRaises(gravity.GravityModelError):
            self.model.compute([0, 0, 0])


class CoordinateCase(unittest.TestCase):
    def test_j3_canonical_reference_restored(self):
        coords = gravity.CoordinateMap(MAPPING)
        raw_j3 = 3.3834973678187232 - 2*math.pi
        raw = list(coords.zeros)
        raw[2] = raw_j3
        model = coords.to_model_angle(raw)
        self.assertAlmostEqual(coords.zeros[2], -2.5423437857633324, places=12)
        self.assertAlmostEqual(math.degrees(model[2]), -20.474311834813136, places=8)
        self.assertAlmostEqual(coords.to_protocol_angle(model)[2], raw_j3, places=12)
        for q in (-math.pi/2, 0., math.pi/2):
            model[2] = q
            self.assertLess(abs(coords.to_protocol_angle(model)[2]), 12.5)
        # A future return to the old raw branch remains outside the operating
        # interval: this explicit reference does not silently modulo feedback.
        raw[2] -= 2*math.pi
        self.assertLess(coords.to_model_angle(raw)[2], -math.pi/2)

    def test_mapping_and_torque_sign(self):
        mapping = gravity.CoordinateMap(MAPPING)
        self.assertEqual(mapping.signs, [1, 1, 1, -1])
        raw = [0.1, -0.2, 0.3, -0.4]
        model = mapping.to_model_angle(raw)
        for a, b in zip(mapping.to_protocol_angle(model), raw):
            self.assertAlmostEqual(a, b, places=12)
        # J4 uses sign -1: model torque flips into protocol frame
        protocol = mapping.to_protocol_torque([1.0, 2.0, 3.0, 4.0])
        self.assertEqual(protocol, [1.0, 2.0, 3.0, -4.0])
        self.assertEqual(mapping.to_model_torque(protocol), [1.0, 2.0, 3.0, 4.0])


class ShadowCase(unittest.TestCase):
    def setUp(self):
        self.shadow = gravity.GravityShadow(URDF, PROFILE, MAPPING, POLICY,
                                            gravity_base=GRAVITY, fixed_joint_offsets=OFFSETS)

    def test_rows_and_safety_flags(self):
        estimate = self.shadow.estimate([-0.0717, 0.0019, 0.0279, 0.0141],
                                        [0.0, 0.0, 0.0, 0.0],
                                        [-0.0717, 0.0019, 0.0279, 0.0141])
        self.assertTrue(estimate["available"])
        self.assertEqual(estimate["mode"], "shadow")
        self.assertFalse(estimate["ff_applied"])
        self.assertEqual(estimate["alpha"], 0.0)
        self.assertEqual(len(estimate["rows"]), 4)
        self.assertEqual([row["joint"] for row in estimate["rows"]], list(gravity.JOINT_NAMES))
        for row in estimate["rows"]:
            self.assertAlmostEqual(row["tau_protocol_nm"],
                                   self.shadow.coords.signs[gravity.JOINT_NAMES.index(row["joint"])]
                                   * row["tau_model_nm"], places=4)
            self.assertEqual(row["guard_nm"], 0.0)
            self.assertFalse(row["guard_enabled"])
            # No driver feedforward received: applied headroom is unknown, not zero.
            self.assertIsNone(row["applied_ff_nm"])
            self.assertIsNone(row["applied_headroom_nm"])
            self.assertIsNotNone(row["p_term_nm"])
            self.assertIsNone(row["predicted_headroom_nm"])
        self.assertTrue(any("暂定" in warning for warning in estimate["warnings"]))

    def test_applied_headroom_uses_driver_values(self):
        self.shadow.guards = [8., 10., 2., 2.]  # Explicit optional guard configuration.
        estimate = self.shadow.estimate([0.0] * 4, [0.0] * 4, [0.0] * 4,
                                        applied_ff_motor_nm=[0.1, -0.2, 0.05, -0.05])
        for index, row in enumerate(estimate["rows"]):
            self.assertAlmostEqual(row["applied_ff_nm"], [0.1, -0.2, 0.05, -0.05][index], places=4)
            expected_envelope = abs(row["p_term_nm"]) + abs(row["d_term_nm"]) + abs(row["applied_ff_nm"])
            self.assertAlmostEqual(row["applied_envelope_nm"], round(expected_envelope, 4), places=3)
            self.assertAlmostEqual(row["applied_headroom_nm"],
                                   round(row["guard_nm"] - expected_envelope, 4), places=3)

    def test_guard_headroom_uses_envelope(self):
        self.shadow.guards = [8., 10., 2., 2.]  # Formula remains valid when explicitly enabled.
        estimate = self.shadow.estimate([0.0] * 4, [0.25] * 4, [0.0] * 4)
        for row in estimate["rows"]:
            self.assertAlmostEqual(row["d_term_nm"], round(-self.shadow.kd * 0.25, 4), places=3)
            predicted_envelope = abs(row["p_term_nm"]) + abs(row["d_term_nm"]) + abs(row["predicted_ff_nm"])
            self.assertAlmostEqual(row["predicted_envelope_nm"], round(predicted_envelope, 4), places=3)
            self.assertAlmostEqual(row["predicted_headroom_nm"],
                                   round(row["guard_nm"] - predicted_envelope, 4), places=3)

    def test_stale_reference_is_flagged(self):
        estimate = self.shadow.estimate([0.0] * 4, [0.3, 0.0, 0.0, 0.0], None)
        self.assertFalse(estimate["reference_known"])
        self.assertTrue(estimate["reference_stale"])
        self.assertTrue(any("参考" in warning for warning in estimate["warnings"]))
        for row in estimate["rows"]:
            self.assertIsNone(row["p_term_nm"])
            self.assertIsNone(row["predicted_headroom_nm"])
            self.assertIsNone(row["applied_headroom_nm"])
            self.assertIsNotNone(row["d_term_nm"])
            self.assertIsNotNone(row["predicted_ff_nm"])

    def test_policy_values(self):
        self.assertEqual(self.shadow.kp, 160.0)
        self.assertEqual(self.shadow.kd, 5.0)
        self.assertEqual(self.shadow.guards, [0.0, 0.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
