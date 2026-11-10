"""Offline five-axis control study (no ROS, no hardware).

Important correction from the independent review:

* ``max_error_deg`` in the original client/JTC results is the maximum of
  ``abs(actual - target)`` over the final ~0.3 s of a dwell.  It is an unsigned
  quality metric, not a signed static offset.
* Therefore this module does **not** produce a deployable signed gravity
  feedforward model.  It may only build an explicitly unsigned, path-local
  magnitude study from the available results.
* A single synchronized path (q_start + s*[1,1,1,1,1]) cannot establish a
  five-dimensional gravity model or general workspace validity.
* No measured current, bus, external torque, or signed time series is present.

Run:
    python3 verification/.../offline_optimization/five_axis_control_lab.py
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
RESULTS = ROOT / "verification/hardware/five_axis_20261107/five_axis_2deg_diagnostics/results"
BASELINE = RESULTS / "BASELINE_0_40_160_5.json"
GAINED = RESULTS / "GAIN_0_40_280_240_160_200_160.json"
AXES = ("J1", "J2", "J3", "J4", "J5")
BASE_KP = 160.0
BASE_KD = 5.0
PROTOCOL_KP_MAX = 500.0
EXPECTED_POSITIVE_ANGLES = (5.0, 10.0, 20.0, 30.0, 35.0, 40.0)


@dataclass(frozen=True)
class Observation:
    angle_deg: float
    pose_deg: tuple
    error_magnitude_deg: float
    unsigned_torque_magnitude_nm: float


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) \
        and math.isfinite(float(value))


def _relative_pose(target, initial):
    if len(target) != 5 or len(initial) != 5:
        raise ValueError("five-axis target/initial length must be 5")
    values = [math.degrees(float(target[index]) - float(initial[index]))
              for index in range(5)]
    if not all(_finite(value) for value in values):
        raise ValueError("non-finite relative pose")
    return tuple(values)


def load_observations(path):
    """Load the positive +5..+40 synchronized poses, fail closed.

    The source is the real client result file.  Missing axes, missing results,
    non-finite values, or an incomplete six-point positive set are errors, not
    silently skipped rows.
    """
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    initial = document.get("initial_model_rad")
    if not isinstance(initial, list) or len(initial) != 5:
        raise ValueError("initial_model_rad must contain five axes")
    if not all(_finite(value) for value in initial):
        raise ValueError("non-finite initial_model_rad")
    rows = document.get("waypoint_results")
    if not isinstance(rows, list):
        raise ValueError("waypoint_results missing")
    observations = {axis: [] for axis in AXES}
    for index, row in enumerate(rows):
        target = row.get("target_rad")
        errors = row.get("max_error_deg")
        if not isinstance(target, list) or len(target) != 5:
            raise ValueError("waypoint %d target_rad must be five axes" % index)
        if not isinstance(errors, list) or len(errors) != 5:
            raise ValueError("waypoint %d max_error_deg must be five axes" % index)
        pose = _relative_pose(target, initial)
        angle = round(pose[0], 6)
        if angle <= 0.0 or angle > 45.0:
            continue  # fit only the measured positive synchronized path
        if not all(_finite(value) for value in errors):
            raise ValueError("waypoint %d contains a non-finite error" % index)
        for axis_index, axis in enumerate(AXES):
            magnitude = abs(float(errors[axis_index]))
            observations[axis].append(Observation(
                angle, pose, magnitude,
                BASE_KP * math.radians(magnitude)))
    for axis in AXES:
        if len(observations[axis]) < len(EXPECTED_POSITIVE_ANGLES):
            raise ValueError("%s has only %d positive poses; six required" %
                             (axis, len(observations[axis])))
    return observations


def _solve(matrix, vector):
    size = len(vector)
    augmented = [list(matrix[row]) + [vector[row]] for row in range(size)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise ValueError("singular polynomial fit")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        augmented[column] = [value / scale for value in augmented[column]]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [a - factor * b for a, b in zip(augmented[row], augmented[column])]
    return [augmented[row][-1] for row in range(size)]


def polyfit(points, degree=2):
    size = degree + 1
    matrix = [[0.0] * size for _ in range(size)]
    vector = [0.0] * size
    for x, y in points:
        powers = [x ** power for power in range(size)]
        for row in range(size):
            vector[row] += powers[row] * y
            for column in range(size):
                matrix[row][column] += powers[row] * powers[column]
    return _solve(matrix, vector)


def polyval(coefficients, x):
    return sum(value * x ** power for power, value in enumerate(coefficients))


def fit_unsigned_magnitude(observations):
    """Path-local unsigned magnitude fit; not a signed gravity model."""
    return {
        axis: polyfit([(item.angle_deg, item.unsigned_torque_magnitude_nm)
                       for item in observations[axis]], 2)
        for axis in AXES
    }


def leave_one_out_magnitude(observations, degree=2):
    """LOO on one path only; not a multi-pose generalization test."""
    result = {}
    for axis in AXES:
        points = [(item.angle_deg, item.unsigned_torque_magnitude_nm)
                  for item in observations[axis]]
        errors = []
        for index, (angle, torque) in enumerate(points):
            training = points[:index] + points[index + 1:]
            if len(training) < degree + 1:
                continue
            model = polyfit(training, degree)
            errors.append(abs(polyval(model, angle) - torque))
        result[axis] = max(errors) if errors else None
    return result


def schedule_kp(axis, angle_deg, low=280.0, high=PROTOCOL_KP_MAX,
                full_angle_deg=30.0):
    """Static feasibility bound only; not a validated mechanical limit."""
    if axis in ("J3", "J5"):
        return BASE_KP
    phase = min(max(float(angle_deg), 0.0), float(full_angle_deg))
    return float(low) + (float(high) - float(low)) * phase / float(full_angle_deg)


def compare(observations):
    fit = fit_unsigned_magnitude(observations)
    loo = leave_one_out_magnitude(observations)
    report = {
        "schema": "slaihand_offline_control_study/2",
        "data_kind": "UNSIGNED_MAGNITUDE_ONLY_PATH_LOCAL",
        "signed_static_data_available": False,
        "signed_time_series_available": False,
        "validity_domain": "one synchronized path q_start + s*[1,1,1,1,1], positive 5..40 deg",
        "deployable_gravity_feedforward": False,
        "note": "max_error_deg is max(abs(actual-target)) over the last ~0.3 s; "
                "direction is unavailable. This is not a signed gravity model, "
                "not a dynamic stability result, and not hardware evidence.",
        "baseline_kp": BASE_KP, "baseline_kd": BASE_KD,
        "protocol_kp_max_not_capability": PROTOCOL_KP_MAX,
        "axes": {},
    }
    for axis in AXES:
        points = observations[axis]
        items = []
        for item in points:
            items.append({
                "angle_deg": round(item.angle_deg, 3),
                "pose_deg": [round(value, 3) for value in item.pose_deg],
                "measured_baseline_error_magnitude_deg": round(item.error_magnitude_deg, 3),
                "unsigned_implied_torque_magnitude_nm": round(
                    item.unsigned_torque_magnitude_nm, 3),
                "constant_high_gain_predicted_magnitude_deg": round(
                    math.degrees(item.unsigned_torque_magnitude_nm /
                                 (280.0 if axis == "J1" else
                                  240.0 if axis == "J2" else
                                  200.0 if axis == "J4" else 160.0)), 3),
                "angle_schedule_predicted_magnitude_deg": round(
                    math.degrees(item.unsigned_torque_magnitude_nm /
                                 schedule_kp(axis, item.angle_deg)), 3),
            })
        report["axes"][axis] = {
            "max_positive_angle_deg": max(item.angle_deg for item in points),
            "unsigned_path_magnitude_fit_degree2": [round(value, 6)
                                                     for value in fit[axis]],
            "unsigned_path_loo_max_torque_error_nm": (
                round(loo[axis], 4) if loo[axis] is not None else None),
            "schedule_kp_at_max_deg": round(schedule_kp(axis, points[-1].angle_deg), 2),
            "rows": items,
        }
    return report


def main():
    report = compare(load_observations(BASELINE))
    output = HERE / "offline_control_study.json"
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                      encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
