"""Exploratory coupled linear fit of per-joint torque estimates.

No ROS/hardware and NOT a deployable feedforward model. Inputs are the 20 deg
and 40 deg multi-pose torque proxies. The purpose is to test whether a simple
linear pose model is accurate enough to justify a later FF design.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
JOINTS = ("slaihand_joint1", "slaihand_joint2", "slaihand_joint3",
          "slaihand_joint4", "slaihand_joint5")


def solve(matrix, vector):
    size = len(vector)
    augmented = [list(matrix[row]) + [vector[row]] for row in range(size)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise ValueError("singular model")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        augmented[column] = [value / scale for value in augmented[column]]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [a - factor * b for a, b in zip(augmented[row], augmented[column])]
    return [augmented[row][-1] for row in range(size)]


def fit(points):
    size = 6
    matrix = [[0.0] * size for _ in range(size)]
    vector = [0.0] * size
    for angles, torque in points:
        row = [1.0] + [math.radians(value) for value in angles]
        for i in range(size):
            vector[i] += row[i] * torque
            for j in range(size):
                matrix[i][j] += row[i] * row[j]
    return solve(matrix, vector)


def predict(coefficients, angles):
    row = [1.0] + [math.radians(value) for value in angles]
    return sum(a * b for a, b in zip(coefficients, row))


def collect():
    samples = {axis: [] for axis in JOINTS}
    for stem in ("multi_pose_20", "multi_pose_40"):
        summary = json.loads((HERE / (stem + "_summary_20261109.json")).read_text(encoding="utf-8"))
        torque = json.loads((HERE / (stem + "_torque_20261109.json")).read_text(encoding="utf-8"))
        for segment, torque_segment in zip(summary["segments"], torque["segments"]):
            angles = segment["reference_deg"]
            for axis in JOINTS:
                samples[axis].append((angles, torque_segment["axes"][axis]["effort_nm"]["mean"]))
    return samples


def build():
    samples = collect()
    axes = {}
    for axis, points in samples.items():
        coefficients = fit(points)
        loo = []
        for index, (angles, expected) in enumerate(points):
            training = points[:index] + points[index + 1:]
            loo.append(abs(predict(fit(training), angles) - expected))
        axes[axis] = {
            "coefficients_intercept_and_rad": coefficients,
            "sample_count": len(points),
            "leave_one_out_max_error_nm": max(loo),
            "leave_one_out_mean_error_nm": sum(loo) / len(loo),
            "deployable": False,
        }
    return {
        "schema": "slaihand_coupled_linear_torque_fit/1",
        "input": "multi_pose_20/40 torque estimates",
        "target": "DAMIAO effort estimate, not measured current",
        "validity": "path-local exploratory; no multi-pose holdout beyond LOO",
        "axes": axes,
    }


def main():
    report = build()
    output = HERE / "coupled_linear_torque_fit_20261109.json"
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print("written", output)


if __name__ == "__main__":
    main()
