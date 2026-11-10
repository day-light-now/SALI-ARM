"""Fake-data counterexamples for the calibration core (no ROS, no hardware)."""
import copy
import json
import math
import sys
import tempfile
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import calibration_core as core  # noqa: E402

MODEL = {"urdf_sha256": "a" * 64, "mass_profile_sha256": "b" * 64,
         "mass_profile_status": "ESTIMATE_ONLY_NOT_FOR_HARDWARE",
         "mapping_sha256": "c" * 64, "policy_sha256": "d" * 64,
         "j3_offset_rad": math.pi, "gravity_base_m_s2": [0.0, 9.80665, 0.0]}
LOAD_ID = "no_end_load"
IDENTITY = {"mapping_sha256": MODEL["mapping_sha256"], "model_sha256": MODEL["urdf_sha256"],
            "policy_sha256": MODEL["policy_sha256"],
            "mass_profile_sha256": MODEL["mass_profile_sha256"],
            "gravity_base_m_s2": MODEL["gravity_base_m_s2"],
            "j3_offset_rad": MODEL["j3_offset_rad"], "load_id": LOAD_ID}
RUNTIME = {"verified": True, "source": "unit_test_only", "identity": IDENTITY}
WINDOW = {"samples": 10, "span_s": 2.0, "max_gap_s": 0.21, "max_speed_rad_s": 0.001,
          "max_position_delta_rad": 0.001, "stage": 5, "motor_state": [1, 1, 1, 1],
          "all_enabled": True, "fresh_all": True, "ff_all_zero": True, "ff_all_fresh": True,
          "command_ids": [7], "valid": True, "reasons": []}
FF = {"alpha": 0.0, "active": False, "applied": [0.0] * 4, "age_s": 0.2,
      "stage": 5, "command_id": 7}


def gravity(positions):
    return [0.5 * math.cos(positions[0]), 0.8 * math.sin(positions[1]),
            0.1 * math.cos(positions[2]), 0.2 * math.sin(positions[3])]


def make_sample(positions, scale=1.0, noise=0.0, support="free", contact="none",
                motion="static", age=0.01, schema=core.SCHEMA_SAMPLE, index=0,
                stage=5, motor_state=(1, 1, 1, 1), window=None, runtime=None,
                load_id=LOAD_ID, model=None, effort=None, ff=None):
    expected = [scale * value for value in gravity(positions)]
    measured = effort if effort is not None else \
        [value + noise * (1 if index % 2 else -1) for value in expected]
    local = 0.01
    totals = [age + local] * 4
    return {"schema": schema, "positions_rad": list(positions),
            "velocities_rad_s": [0.0] * 4, "effort_estimate_nm": measured,
            "driver_temperature_c": [30.0] * 4, "motor_temperature_c": [30.0] * 4,
            "feedback_age_s": [age] * 4, "local_receipt_age_s": local,
            "total_age_s": totals, "capture_age_s": max(totals),
            "feedback_valid": [True] * 4, "stage": stage, "motor_state": list(motor_state),
            "support": support, "contact": contact, "motion": motion,
            "load_id": load_id, "model": dict(model or MODEL),
            "runtime": copy.deepcopy(runtime or RUNTIME),
            "static_window": copy.deepcopy(window or WINDOW),
            "driver_ff": copy.deepcopy(ff or FF)}


def diverse_samples(count=12, scale=1.0, noise=0.0):
    samples = []
    for index in range(count):
        first = math.radians(-20 + 7 * index)
        second = math.radians(10 + 5 * index)
        third = math.radians(-15 + 4 * index)
        fourth = math.radians(-20 + 3 * index)
        samples.append(make_sample([first, second, third, fourth], scale=scale,
                                   noise=noise, index=index))
    return samples


class ValidationCase(unittest.TestCase):
    def test_valid_free_sample(self):
        ok, reasons = core.validate_sample(make_sample([0, 0, 0, 0]))
        self.assertTrue(ok, reasons)

    def test_stale_driver_feedback_is_invalid(self):
        ok, reasons = core.validate_sample(make_sample([0, 0, 0, 0], age=0.1))
        self.assertFalse(ok)
        self.assertTrue(any("年龄" in reason or "时效" in reason for reason in reasons))

    def test_age_fields_must_agree(self):
        sample = make_sample([0, 0, 0, 0])
        sample["capture_age_s"] = 0.5
        ok, reasons = core.validate_sample(sample)
        self.assertFalse(ok)
        self.assertTrue(any("不一致" in reason for reason in reasons))

    def test_missing_local_receipt_age_is_invalid(self):
        sample = make_sample([0, 0, 0, 0])
        sample.pop("local_receipt_age_s")
        ok, _ = core.validate_sample(sample)
        self.assertFalse(ok)

    def test_missing_stage_or_motor_state_is_invalid(self):
        sample = make_sample([0, 0, 0, 0])
        sample.pop("stage")
        ok, _ = core.validate_sample(sample)
        self.assertFalse(ok)
        sample = make_sample([0, 0, 0, 0])
        sample.pop("motor_state")
        ok, _ = core.validate_sample(sample)
        self.assertFalse(ok)

    def test_unstable_window_is_invalid(self):
        ok, reasons = core.validate_sample(
            make_sample([0, 0, 0, 0], window=dict(WINDOW, max_speed_rad_s=0.5)))
        self.assertFalse(ok)
        self.assertTrue(any("速度" in reason for reason in reasons))
        ok, reasons = core.validate_sample(
            make_sample([0, 0, 0, 0], window=dict(WINDOW, max_gap_s=1.0)))
        self.assertFalse(ok)
        self.assertTrue(any("间隙" in reason for reason in reasons))

    def test_window_first_frame_disabled_is_invalid(self):
        window = dict(WINDOW, all_enabled=False, motor_state=[0, 0, 0, 0],
                      stage=8, valid=False)
        ok, reasons = core.validate_sample(make_sample([0, 0, 0, 0], window=window))
        self.assertFalse(ok)
        self.assertTrue(any("使能" in reason for reason in reasons))

    def test_runtime_identity_mismatch_is_invalid(self):
        runtime = copy.deepcopy(RUNTIME)
        runtime["identity"]["mapping_sha256"] = "e" * 64
        ok, reasons = core.validate_sample(make_sample([0, 0, 0, 0], runtime=runtime))
        self.assertFalse(ok)
        self.assertTrue(any("运行时身份" in reason for reason in reasons))

    def test_short_window_is_invalid(self):
        window = dict(WINDOW, samples=2, span_s=0.2, valid=False)
        ok, reasons = core.validate_sample(make_sample([0, 0, 0, 0], window=window))
        self.assertFalse(ok)
        self.assertTrue(any("样本数" in reason or "跨度" in reason for reason in reasons))

    def test_missing_support_or_load_is_invalid(self):
        sample = make_sample([0, 0, 0, 0])
        sample["support"] = None
        ok, reasons = core.validate_sample(sample)
        self.assertFalse(ok)
        self.assertTrue(any("support" in reason for reason in reasons))
        ok, reasons = core.validate_sample(make_sample([0, 0, 0, 0], load_id=""))
        self.assertFalse(ok)
        self.assertTrue(any("负载" in reason for reason in reasons))

    def test_schema_mismatch_is_invalid(self):
        ok, reasons = core.validate_sample(make_sample([0, 0, 0, 0], schema="wrong/0"))
        self.assertFalse(ok)
        self.assertTrue(any("schema" in reason for reason in reasons))


class QualificationCase(unittest.TestCase):
    def test_moving_sample_is_excluded(self):
        sample = make_sample([0, 0, 0, 0], motion="up")
        self.assertIn("运动来向", core.exclusion_reason(sample))
        samples = diverse_samples()
        for sample in samples:
            sample["motion"] = "up"
            sample["velocities_rad_s"] = [0.8] * 4
        report = core.analyze(samples, gravity)
        self.assertEqual(report["status"], "INSUFFICIENT")
        self.assertIsNone(report["fit"])

    def test_disabled_sample_is_geometry_only(self):
        sample = make_sample([0, 0, 0, 0], stage=8, motor_state=(0, 0, 0, 0),
                             effort=[0.0] * 4)
        reason = core.exclusion_reason(sample)
        self.assertIn("失能", reason)
        report = core.analyze([sample] * 12, gravity)
        self.assertEqual(report["status"], "INSUFFICIENT")
        self.assertTrue(any(row.get("geometry_only") for row in report["exclusions"]))

    def test_ff_active_sample_is_excluded(self):
        sample = make_sample([0, 0, 0, 0], ff=dict(FF, alpha=0.3))
        self.assertIn("alpha", core.exclusion_reason(sample))

    def test_stale_ff_is_excluded(self):
        sample = make_sample([0, 0, 0, 0], ff=dict(FF, age_s=120.0))
        self.assertIn("过期", core.exclusion_reason(sample))
        samples = diverse_samples()
        for sample in samples:
            sample["driver_ff"]["age_s"] = 120.0
        report = core.analyze(samples, gravity)
        self.assertEqual(report["status"], "INSUFFICIENT")
        self.assertTrue(any("过期" in row["reason"] for row in report["exclusions"]))

    def test_ff_stage_or_command_mismatch_is_excluded(self):
        self.assertIn("阶段", core.exclusion_reason(make_sample([0, 0, 0, 0], ff=dict(FF, stage=8))))
        self.assertIn("命令 ID", core.exclusion_reason(make_sample([0, 0, 0, 0], ff=dict(FF, command_id=9))))
        window = dict(WINDOW, command_ids=[7, 9], valid=False)
        self.assertIn("命令 ID", core.exclusion_reason(make_sample([0, 0, 0, 0], window=window)))

    def test_unverified_runtime_is_excluded(self):
        sample = make_sample([0, 0, 0, 0], runtime={"verified": False, "source": "unknown"})
        self.assertIn("运行中配置身份未验证", core.exclusion_reason(sample))
        unverified = [dict(row, runtime={"verified": False, "source": "unknown"})
                      for row in diverse_samples()]
        report = core.analyze(unverified, gravity)
        self.assertEqual(report["status"], "INSUFFICIENT")


class AnalysisCase(unittest.TestCase):
    def test_fit_recovers_global_scale(self):
        report = core.analyze(diverse_samples(scale=0.9), gravity)
        self.assertEqual(report["status"], "PROVISIONAL_SCALE", report["reasons"])
        self.assertAlmostEqual(report["fit"]["scale"], 0.9, places=3)
        self.assertIsNotNone(report["holdout"])

    def test_mixed_versions_are_refused(self):
        samples = diverse_samples()
        for index, sample in enumerate(samples):
            if index % 2:
                sample["model"]["mapping_sha256"] = "e" * 64
                sample["runtime"]["identity"]["mapping_sha256"] = "e" * 64
        report = core.analyze(samples, gravity)
        self.assertEqual(report["status"], "INSUFFICIENT")
        self.assertTrue(any("不一致" in reason for reason in report["reasons"]))
        self.assertIsNone(report["fit"])

    def test_identity_mismatch_with_analysis_model_is_refused(self):
        expected = dict(IDENTITY, mapping_sha256="f" * 64)
        report = core.analyze(diverse_samples(), gravity, expected_identity=expected)
        self.assertEqual(report["status"], "INSUFFICIENT")
        self.assertTrue(any("当前分析模型" in reason for reason in report["reasons"]))
        self.assertTrue(report['diagnostics']['diagnostic_only'])

    def test_negative_scale_is_refused(self):
        samples = diverse_samples()
        for sample in samples:
            sample["effort_estimate_nm"] = [-value for value in sample["effort_estimate_nm"]]
        report = core.analyze(samples, gravity)
        self.assertEqual(report["status"], "INSUFFICIENT")
        self.assertTrue(any("非正" in reason for reason in report["reasons"]))

    def test_insufficient_samples_or_diversity(self):
        report = core.analyze(diverse_samples(count=3), gravity)
        self.assertEqual(report["status"], "INSUFFICIENT")
        same = [make_sample([0.1, 0.2, 0.1, 0.0]) for _ in range(12)]
        report = core.analyze(same, gravity)
        self.assertEqual(report["status"], "INSUFFICIENT")
        self.assertTrue(any("姿态多样性" in reason for reason in report["reasons"]))

    def test_status_never_claims_calibrated(self):
        for samples in (diverse_samples(), diverse_samples(count=2),
                        [make_sample([0, 0, 0, 0], support="supported")]):
            report = core.analyze(samples, gravity)
            self.assertIn(report["status"], ("PROVISIONAL_SCALE", "INSUFFICIENT"))

    def test_diagnostics_kept_when_refused(self):
        samples = diverse_samples()
        for sample in samples:
            sample["support"] = "supported"
        report = core.analyze(samples, gravity)
        self.assertEqual(report["status"], "INSUFFICIENT")
        self.assertIsNotNone(report["diagnostics"])
        self.assertTrue(report["diagnostics"]["diagnostic_only"])


class RecorderCase(unittest.TestCase):
    def _recorder(self, at, include_position=True, feedback_age=0.01):
        from calibration_capture import Recorder, NAMES
        recorder = Recorder.__new__(Recorder)
        axes = {}
        for name in NAMES:
            interface = {"velocity": 0.0, "effort": 0.0, "feedback_age": feedback_age,
                         "driver_temperature": 30.0, "motor_temperature": 30.0,
                         "feedback_valid": 1.0, "motor_state": 1.0, "control_state": 5.0}
            if include_position:
                interface["position"] = 0.0
            axes[name] = interface
        recorder.joint = None
        recorder.dynamic = {"at": at, "axes": axes}
        recorder.dynamic_sequence = 1
        recorder.reference = None
        recorder.reference_at = None
        recorder.ff = None
        recorder.runtime_identity = None
        recorder.runtime_identity_at = None
        recorder.local_identity = None
        return recorder

    def test_cached_snapshot_is_rejected(self):
        recorder = self._recorder(time.monotonic() - 0.20)
        self.assertIsNone(recorder._build())

    def test_age_is_summed_not_max(self):
        recorder = self._recorder(time.monotonic() - 0.035, feedback_age=0.035)
        self.assertIsNone(recorder._build())
        recorder = self._recorder(time.monotonic() - 0.01, feedback_age=0.01)
        snapshot = recorder._build()
        self.assertIsNotNone(snapshot)
        self.assertAlmostEqual(snapshot["capture_age_s"], 0.02, places=2)
        self.assertGreater(snapshot["capture_age_s"], max(snapshot["feedback_age_s"]))

    def test_missing_dynamic_position_is_rejected(self):
        recorder = self._recorder(time.monotonic() - 0.01, include_position=False)
        self.assertIsNone(recorder._build())


class CollectStaticCase(unittest.TestCase):
    class FakeRecorder:
        def __init__(self, snapshots):
            self.snapshots = list(snapshots)
            self.index = 0

        def snapshot(self, timeout=1.0):
            item = self.snapshots[min(self.index, len(self.snapshots) - 1)]
            self.index += 1
            return copy.deepcopy(item)

    def _snapshot(self, at, sequence, stage=5, motor=(1, 1, 1, 1), command_id=7):
        return {"positions_rad": [0.1, 0.2, 0.1, 0.0], "velocities_rad_s": [0.0] * 4,
                "effort_estimate_nm": [0.1, 0.2, 0.1, 0.0],
                "driver_temperature_c": [30.0] * 4, "motor_temperature_c": [30.0] * 4,
                "feedback_age_s": [0.01] * 4, "local_receipt_age_s": 0.01,
                "total_age_s": [0.02] * 4, "capture_age_s": 0.02,
                "feedback_valid": [True] * 4, "motor_state": list(motor), "stage": stage,
                "dynamic_sequence": sequence, "joint_delta_rad": 0.0,
                "controller_reference_rad": None, "controller_reference_age_s": None,
                "driver_ff": {"alpha": 0.0, "active": False, "applied": [0.0] * 4,
                              "age_s": 0.2, "stage": stage, "command_id": command_id},
                "runtime": dict(RUNTIME), "_monotonic": at}

    def test_repeated_receipts_are_not_counted(self):
        from calibration_capture import collect_static
        start = time.monotonic()
        snapshots = [self._snapshot(start + index * 0.25, 0) for index in range(20)]
        self.assertIsNone(collect_static(self.FakeRecorder(snapshots), 1.0, 5, "auto", 1.0))

    def test_first_frame_disabled_invalidates_window(self):
        from calibration_capture import _window_aggregate
        rows = [self._snapshot(float(i), i) for i in range(5)]
        rows[0]["stage"] = 8
        rows[0]["motor_state"] = [0, 0, 0, 0]
        window = _window_aggregate(rows)
        self.assertFalse(window["all_enabled"])
        self.assertEqual(window["motor_state"], [0, 0, 0, 0])
        self.assertFalse(window["valid"])

    def test_command_change_resets_window(self):
        from calibration_capture import collect_static
        start = time.monotonic()
        snapshots = [self._snapshot(start + index * 0.25, index,
                                    command_id=7 if index < 3 else 9) for index in range(10)]
        result = collect_static(self.FakeRecorder(snapshots), 1.0, 8, "auto", 3.0)
        # A command change resets the window: the result may be a fresh window for
        # the new command, but never a window spliced across both commands.
        if result is not None:
            self.assertEqual(result["static_window"]["command_ids"], [9])

    def test_static_window_is_collected(self):
        from calibration_capture import collect_static
        start = time.monotonic()
        snapshots = [self._snapshot(start + index * 0.25, index) for index in range(20)]
        result = collect_static(self.FakeRecorder(snapshots), 1.0, 5, "auto", 5.0)
        self.assertIsNotNone(result)
        self.assertTrue(result["static_window"]["all_enabled"])

    def test_protocol_velocity_noise_still_qualifies(self):
        from calibration_capture import _window_aggregate
        start = time.monotonic()
        rows = []
        for index in range(12):
            row = self._snapshot(start + index * 0.2, index)
            # Real drive velocity noise is +/-2-3 feedback codes while the
            # position is truly stationary; position drift stays the real gate.
            row["velocities_rad_s"] = [0.05, -0.05, 0.033, -0.033]
            rows.append(row)
        window = _window_aggregate(rows)
        self.assertTrue(window["valid"], window["reasons"])
        self.assertLess(window["max_speed_rad_s"], core.STATIC_SPEED_RAD_S)

    def test_moving_window_still_rejected(self):
        from calibration_capture import _window_aggregate
        start = time.monotonic()
        rows = []
        for index in range(12):
            row = self._snapshot(start + index * 0.2, index)
            row["velocities_rad_s"] = [0.2, 0.2, 0.2, 0.2]
            row["positions_rad"] = [0.1 + index * 0.02, 0.2, 0.1, 0.0]
            rows.append(row)
        window = _window_aggregate(rows)
        self.assertFalse(window["valid"])


class RawFileCase(unittest.TestCase):
    def write_capture(self, directory, samples, header_schema=core.SCHEMA_CAPTURE):
        path = Path(directory) / "capture.jsonl"
        header = {"schema": header_schema, "tool": "test", "created": "now",
                  "model": MODEL, "load_id": LOAD_ID, "runtime": RUNTIME}
        lines = [json.dumps(header)] + [json.dumps(sample) for sample in samples]
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        path.chmod(0o444)
        return path

    def test_load_raw_and_preview(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_capture(directory, diverse_samples())
            payload = core.load_raw(path)
            self.assertEqual(len(payload["samples"]), 12)
            report = core.analyze(payload["samples"], gravity)
            self.assertIn(report["status"], ("PROVISIONAL_SCALE", "INSUFFICIENT"))
            lines = core.preview_lines(report)
            self.assertTrue(any("状态" in line for line in lines))

    def test_wrong_header_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_capture(directory, diverse_samples(), header_schema="wrong/0")
            with self.assertRaises(core.CalibrationError):
                core.load_raw(path)

    def test_bad_json_line_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "broken.jsonl"
            path.write_text(json.dumps({"schema": core.SCHEMA_CAPTURE}) + "\nnot json\n",
                            encoding="utf-8")
            with self.assertRaises(core.CalibrationError):
                core.load_raw(path)


if __name__ == "__main__":
    unittest.main(verbosity=2)
