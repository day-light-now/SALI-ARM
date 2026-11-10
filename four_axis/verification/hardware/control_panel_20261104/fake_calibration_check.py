"""Make a fake capture file, then exercise import/preview/report CLI end to end."""
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[2] / "src/slaihand_bringup/scripts"))
import calibration_core as core  # noqa: E402
import calibration_capture as capture  # noqa: E402

LOAD_ID = "no_end_load"
MODEL = {"urdf_sha256": capture._sha256(capture.URDF),
         "mass_profile_sha256": capture._sha256(capture.PROFILE),
         "mass_profile_status": "ESTIMATE_ONLY_NOT_FOR_HARDWARE",
         "mapping_sha256": capture._sha256(capture.MAPPING),
         "policy_sha256": capture._sha256(capture.POLICY),
         "j3_offset_rad": math.pi, "gravity_base_m_s2": [0.0, 9.80665, 0.0],
         "load_id": LOAD_ID, "units": {"position": "rad", "velocity": "rad/s", "torque": "N*m"}}
IDENTITY = {"schema": "slaihand_runtime_identity/1", "boot_id": "unit-test",
            "mapping_sha256": MODEL["mapping_sha256"], "model_sha256": MODEL["urdf_sha256"],
            "policy_sha256": MODEL["policy_sha256"],
            "mass_profile_sha256": MODEL["mass_profile_sha256"],
            "gravity_base_m_s2": MODEL["gravity_base_m_s2"],
            "j3_offset_rad": MODEL["j3_offset_rad"], "load_id": LOAD_ID}
RUNTIME = {"verified": True, "source": "fake_check_test_only", "identity": IDENTITY}
WINDOW = {"samples": 10, "span_s": 2.0, "max_gap_s": 0.21, "max_speed_rad_s": 0.001,
          "max_position_delta_rad": 0.001, "stage": 5, "motor_state": [1, 1, 1, 1],
          "all_enabled": True, "fresh_all": True, "ff_all_zero": True, "ff_all_fresh": True,
          "command_ids": [7], "valid": True, "reasons": []}
FF = {"alpha": 0.0, "active": False, "applied": [0.0] * 4, "age_s": 0.2,
      "stage": 5, "command_id": 7}
model = capture._model_fn()
out = Path("/tmp/fake_calibration.jsonl")
lines = [json.dumps({"schema": core.SCHEMA_CAPTURE, "tool": "fake", "created": "2026-11-05",
                     "support": "free", "contact": "none", "motion": "static",
                     "model": MODEL, "load_id": LOAD_ID, "runtime": RUNTIME})]
for index in range(12):
    pose = [math.radians(-10 + 5 * index), math.radians(15 + 4 * index),
            math.radians(-12 + 3 * index), math.radians(-15 + 2.5 * index)]
    tau = model(pose)
    lines.append(json.dumps({"schema": core.SCHEMA_SAMPLE, "positions_rad": pose,
                             "velocities_rad_s": [0.0] * 4, "effort_estimate_nm": tau,
                             "driver_temperature_c": [30.0] * 4, "motor_temperature_c": [30.0] * 4,
                             "feedback_age_s": [0.01] * 4, "local_receipt_age_s": 0.01,
                             "total_age_s": [0.02] * 4, "capture_age_s": 0.02,
                             "feedback_valid": [True] * 4, "stage": 5,
                             "motor_state": [1, 1, 1, 1],
                             "support": "free", "contact": "none", "motion": "static",
                             "load_id": LOAD_ID, "model": MODEL, "runtime": RUNTIME,
                             "static_window": WINDOW, "driver_ff": FF}))
out.write_text("\n".join(lines) + "\n", encoding="utf-8")
report_path = Path("/tmp/fake_calibration_report.json")
code = capture.main(["--import", str(out)])
code |= capture.main(["--preview", str(out), "--save-report", str(report_path)])
payload = json.loads(report_path.read_text(encoding="utf-8"))
assert payload["schema"] == "slaihand_calibration_report/2"
assert payload["analysis"]["status"] == "PROVISIONAL_SCALE", payload["analysis"]["reasons"]
assert payload["raw_sha256"] == capture._sha256(out)
assert payload["analysis"]["counts"]["usable"] == 12
print("CLI_REPORT_OK", report_path)

# Stale feedforward age must refuse the fit even when alpha is zero.
stale = Path("/tmp/fake_calibration_stale_ff.jsonl")
stale_lines = [lines[0]]
for line in lines[1:]:
    sample = json.loads(line)
    sample["driver_ff"] = dict(FF, age_s=120.0)
    stale_lines.append(json.dumps(sample))
stale.write_text("\n".join(stale_lines) + "\n", encoding="utf-8")
report = capture._build_report(stale)
assert report["status"] == "INSUFFICIENT", report["status"]
assert any("过期" in row["reason"] for row in report["exclusions"]), report["exclusions"]

# UNKNOWN runtime identity: diagnostic only.
unknown = Path("/tmp/fake_calibration_unknown_runtime.jsonl")
unknown_lines = [json.dumps(dict(json.loads(lines[0]), runtime={"verified": False, "source": "unknown"}))]
for line in lines[1:]:
    sample = json.loads(line)
    sample["runtime"] = {"verified": False, "source": "unknown"}
    unknown_lines.append(json.dumps(sample))
unknown.write_text("\n".join(unknown_lines) + "\n", encoding="utf-8")
report = capture._build_report(unknown)
assert report["status"] == "INSUFFICIENT"

# Report path must not be the raw file itself (including a symlink alias).
assert capture.main(["--preview", str(out), "--save-report", str(out)]) == 2
alias = Path("/tmp/fake_calibration_alias.jsonl")
if alias.exists() or alias.is_symlink():
    alias.unlink()
alias.symlink_to(out)
assert capture.main(["--preview", str(out), "--save-report", str(alias)]) == 2
alias.unlink()

# Raw capture files are created exclusively with unique names.
first_path, first_stream = capture._open_raw("/tmp/fake_calibration_raw", "unit")
first_stream.close()
second_path, second_stream = capture._open_raw("/tmp/fake_calibration_raw", "unit")
second_stream.close()
assert first_path != second_path
print("SAME_PATH_AND_UNIQUE_RAW_OK")
sys.exit(code)
