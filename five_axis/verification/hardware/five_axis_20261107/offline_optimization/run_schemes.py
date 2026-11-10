"""Run the offline five-axis control matrix, with evidence categories.

Corrections from the independent review:

* ``max_error_deg`` is an unsigned dwell-end peak, not a signed static error.
  Therefore no signed gravity-feedforward model is produced or ranked here.
* The six poses are one synchronized path; one-path LOO is not workspace
  generalization.
* The measured high-gain result file is used, and predicted vs measured values
  are shown separately.
* BLOCKED / IDEALIZED entries have null metrics and never participate in a
  performance ranking.
* This module never imports ROS, opens a serial port, or commands a motor.
  ``existing_command_path`` is not commissioning or acceptance.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
import five_axis_control_lab as lab  # noqa: E402

AXES = ("J1", "J2", "J3", "J4", "J5")
BASELINE = HERE.parents[0] / "five_axis_2deg_diagnostics/results/BASELINE_0_40_160_5.json"
GAINED = HERE.parents[0] / "five_axis_2deg_diagnostics/results/GAIN_0_40_280_240_160_200_160.json"
HIGH_GAINS = {"J1": 280.0, "J2": 240.0, "J3": 160.0, "J4": 200.0, "J5": 160.0}
SCHEDULE = {"low": 280.0, "high": 500.0, "full_angle_deg": 30.0}


def _max(values):
    values = [abs(float(value)) for value in values if value is not None]
    return max(values) if values else None


def _result(name, category, status, summary, rows, notes, *,
            existing_command_path=False, signed_data=False, dynamic_data=False,
            blocked_reasons=None):
    return {
        "scheme": name,
        "category": category,
        "status": status,
        "existing_command_path": bool(existing_command_path),
        "commissioned": False,
        "acceptance_passed": False,
        "signed_static_data": bool(signed_data),
        "dynamic_data": bool(dynamic_data),
        "simulated": category in ("STATIC_PREDICTION", "IDEALIZED"),
        "summary": summary,
        "rows": rows,
        "blocked_reasons": list(blocked_reasons or []),
        "notes": notes,
    }


def constant():
    observations = lab.load_observations(BASELINE)
    rows = {}
    for axis in AXES:
        rows[axis] = [{
            "angle_deg": item.angle_deg,
            "measured_error_magnitude_deg": round(item.error_magnitude_deg, 4),
            "source": "BASELINE_0_40_160_5.json",
            "measurement_definition": "max(abs(actual-target)) in final ~0.3 s",
            "signed_direction": None,
        } for item in observations[axis]]
    return _result(
        "constant_160_5", "MEASURED", "MEASURED_BASELINE",
        {"max_error_magnitude_deg": {axis: _max(
            row["measured_error_magnitude_deg"] for row in rows[axis]) for axis in AXES},
         "signed_direction_available": False},
        rows, "Real 160/5 baseline; unsigned dwell-end peak. Existing scalar MIT "
              "path only; no accuracy or margin acceptance.",
        existing_command_path=True)


def constant_high():
    baseline = lab.load_observations(BASELINE)
    gained = lab.load_observations(GAINED)
    rows = {}
    for axis in AXES:
        high_by_angle = {round(item.angle_deg, 6): item for item in gained[axis]}
        rows[axis] = []
        for item in baseline[axis]:
            high = high_by_angle.get(round(item.angle_deg, 6))
            if high is None:
                raise ValueError("high-gain result missing angle %.3f for %s"
                                 % (item.angle_deg, axis))
            predicted = math.degrees(
                item.unsigned_torque_magnitude_nm / HIGH_GAINS[axis])
            improvement = ((item.error_magnitude_deg - high.error_magnitude_deg)
                           / item.error_magnitude_deg * 100.0)
            rows[axis].append({
                "angle_deg": item.angle_deg,
                "measured_baseline_error_magnitude_deg": round(
                    item.error_magnitude_deg, 4),
                "measured_high_gain_error_magnitude_deg": round(
                    high.error_magnitude_deg, 4),
                "predicted_high_gain_magnitude_deg": round(predicted, 4),
                "measured_improvement_pct": round(improvement, 2),
                "prediction_error_deg": round(predicted - high.error_magnitude_deg, 4),
                "signed_direction_available": False,
            })
    summary = {"gains": HIGH_GAINS,
               "max_error_magnitude_deg": {
                   axis: {"baseline": _max(row["measured_baseline_error_magnitude_deg"]
                                           for row in rows[axis]),
                          "high_measured": _max(row["measured_high_gain_error_magnitude_deg"]
                                                for row in rows[axis]),
                          "high_predicted": _max(row["predicted_high_gain_magnitude_deg"]
                                                 for row in rows[axis])}
                   for axis in AXES},
               "measured_improvement_pct_range": {
                   axis: [round(min(row["measured_improvement_pct"] for row in rows[axis]), 2),
                          round(max(row["measured_improvement_pct"] for row in rows[axis]), 2)]
                   for axis in AXES},
               "ranking_performed": False}
    return _result(
        "constant_per_axis_high", "MEASURED_COMPARISON", "MEASURED_HIGH_GAIN",
        summary, rows,
        "Measured baseline and measured high-gain files are compared at matching "
        "relative targets. Prediction is unsigned 1/Kp scaling, not a physical "
        "model. The two runs had different absolute start poses; only relative "
        "targets and the same 0.5-degree magnitude criterion are comparable. "
        "Existing command path only; no commissioning/acceptance claim.",
        existing_command_path=True)


def angle_schedule():
    observations = lab.load_observations(BASELINE)
    rows = {}
    for axis in AXES:
        rows[axis] = []
        for item in observations[axis]:
            kp = lab.schedule_kp(axis, item.angle_deg, **SCHEDULE)
            rows[axis].append({
                "angle_deg": item.angle_deg,
                "schedule_kp": round(kp, 3),
                "predicted_magnitude_deg": round(
                    math.degrees(item.unsigned_torque_magnitude_nm / kp), 4),
                "signed_direction_available": False,
            })
    return _result(
        "angle_schedule_kp", "STATIC_PREDICTION", "UNSIGNED_MAGNITUDE_CONDITIONAL",
        {"schedule": dict(SCHEDULE),
         "max_predicted_magnitude_deg": {
             axis: _max(row["predicted_magnitude_deg"] for row in rows[axis])
             for axis in AXES},
         "not_a_dynamic_stability_result": True},
        rows, "Conditional static magnitude prediction only. It does not show that "
              "Kp=500 is mechanically safe or stable; no angle-scheduled C++ "
              "implementation exists. Not hardware-ready.",
        signed_data=False)


def gravity_ff():
    observations = lab.load_observations(BASELINE)
    fit = lab.fit_unsigned_magnitude(observations)
    loo = lab.leave_one_out_magnitude(observations)
    return _result(
        "gravity_feedforward", "BLOCKED", "BLOCKED_SIGNED_STATIC_DATA_MISSING",
        {"metrics": None,
         "unsigned_path_magnitude_fit_only": True,
         "unsigned_path_fit_degree2": {axis: [round(value, 6) for value in fit[axis]]
                                        for axis in AXES},
         "unsigned_path_loo_max_torque_error_nm": {
             axis: (round(loo[axis], 4) if loo[axis] is not None else None)
             for axis in AXES},
         "deployable_feedforward": False},
        {axis: [] for axis in AXES},
        "BLOCKED: the original files only contain unsigned max(abs) errors. "
        "A signed torque feedforward with joint coordinates, sign, zero, gear "
        "ratio, and Nm convention cannot be produced from them. The unsigned "
        "one-path fit is explanatory only. No performance metric is provided.",
        blocked_reasons=["signed_static_time_series_missing",
                         "multi_pose_identification_missing",
                         "current_bus_torque_measurement_missing"])


def integral():
    return _result(
        "integral_trim", "IDEALIZED", "NOT_EVALUATED",
        {"metrics": None, "ideal_model_only": True},
        {axis: [] for axis in AXES},
        "An ideal static integral model can drive a constant error to zero, but "
        "anti-windup, saturation, leakage, fault reset and actuator limits are "
        "not implemented or tested. Not ranked and not hardware-ready.",
        blocked_reasons=["anti_windup_design_missing", "current_limits_missing"])


def disturbance_observer():
    return _result(
        "disturbance_observer", "BLOCKED", "NOT_EVALUATED",
        {"metrics": None, "ideal_zero_not_evidence": True},
        {axis: [] for axis in AXES},
        "No current, bus, torque or signed velocity data exists; a hard-coded "
        "zero residual must not be ranked as performance. Not hardware-ready.",
        blocked_reasons=["current_bus_torque_data_missing",
                         "observer_bandwidth_noise_study_missing"])


def model_gravity():
    return _result(
        "model_based_gravity", "BLOCKED", "NOT_EVALUATED",
        {"metrics": None,
         "missing": ["J5/flange geometry", "new-load mass/COM",
                     "verified five-axis inertial model", "current/voltage limits"]},
        {axis: [] for axis in AXES},
        "No valid current-assembly gravity vector exists. The old four-axis "
        "no_end_load model must not be reused. Not hardware-ready.",
        blocked_reasons=["load_geometry_missing", "multi_pose_inertial_identification_missing"])


SCHEMES = {
    "constant": constant,
    "constant_high": constant_high,
    "angle_schedule": angle_schedule,
    "gravity_ff": gravity_ff,
    "integral": integral,
    "disturbance_observer": disturbance_observer,
    "model_gravity": model_gravity,
}


def main():
    parser = argparse.ArgumentParser(description="Run offline five-axis schemes")
    parser.add_argument("--scheme", default="all", choices=["all", *SCHEMES])
    parser.add_argument("--output", default=str(HERE / "scheme_results.json"))
    args = parser.parse_args()
    selected = SCHEMES if args.scheme == "all" else {
        args.scheme: SCHEMES[args.scheme]}
    results = []
    for name, factory in selected.items():
        result = factory()
        results.append(result)
        print("%-28s %-16s existing_command_path=%-5s metrics=%s" % (
            result["scheme"], result["status"], result["existing_command_path"],
            result["summary"].get("metrics", "n/a")))
    payload = {
        "schema": "slaihand_offline_scheme_matrix/2",
        "scope": "offline evidence only; no ROS/hardware/driver/panel actions",
        "data_definition": "max_error_deg is max(abs(actual-target)) over final ~0.3 s; unsigned",
        "signed_static_data_available": False,
        "workspace_generalization": False,
        "ranking_performed": False,
        "freshness_policy": "default 1800 s for any future admission; no 86400 override",
        "commissioning_or_acceptance": False,
        "schemes": results,
    }
    Path(args.output).write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                                 encoding="utf-8")
    print("written", args.output)


if __name__ == "__main__":
    main()
