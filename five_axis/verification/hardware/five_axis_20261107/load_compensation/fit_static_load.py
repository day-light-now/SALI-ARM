"""Offline path-local load-proxy fit; no ROS, no hardware, no deployable FF.

Input is the corrected 0-40 synchronized baseline summary.  The "load proxy"
is the PD request inferred from ``-Kp*(feedback-reference)``.  It is not
measured current or a gravity model, and it is only valid on the sampled path.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
SUMMARY = HERE.parents[0] / "signed_acquisition/corrected_sweep160_summary_20261109.json"
JOINTS = ("slaihand_joint1", "slaihand_joint2", "slaihand_joint3",
          "slaihand_joint4", "slaihand_joint5")


def _solve(matrix, vector):
    size = len(vector)
    augmented = [list(matrix[row]) + [vector[row]] for row in range(size)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise ValueError("singular fit")
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


def load_points(path=SUMMARY):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    points = {axis: [] for axis in JOINTS}
    seen = set()
    for segment in data.get("segments", []):
        key = tuple(round(value, 4) for value in segment["relative_deg"])
        if key in seen:
            continue
        seen.add(key)
        angle = segment["relative_deg"][0]
        for axis in JOINTS:
            proxy = segment["axes"][axis]["pd_request_estimate_nm"]["mean"]
            points[axis].append((angle, float(proxy)))
    return points


def fit(points, degree=2):
    points = sorted(points)
    if len(points) < degree + 2:
        raise ValueError("not enough independent points")
    coefficients = polyfit(points, degree)
    loo = []
    for index, (angle, expected) in enumerate(points):
        training = points[:index] + points[index + 1:]
        model = polyfit(training, degree)
        loo.append(abs(polyval(model, angle) - expected))
    angles = [angle for angle, _ in points]
    return {
        "degree": degree,
        "coefficients": coefficients,
        "points": points,
        "validity_domain_deg": [min(angles), max(angles)],
        "leave_one_out_max_error_nm": max(loo),
        "deployable": False,
    }


def build(path=SUMMARY):
    points = load_points(path)
    axes = {}
    for axis in JOINTS:
        try:
            axes[axis] = fit(points[axis])
        except ValueError as error:
            axes[axis] = {"error": str(error), "deployable": False}
    return {
        "schema": "slaihand_path_load_proxy/1",
        "source": str(Path(path).name),
        "definition": "path-local PD request proxy = -Kp*(feedback-reference)",
        "not_current": True,
        "not_gravity_model": True,
        "not_five_axis_coupled": True,
        "validity": "one synchronized q_start+s*[1,1,1,1,1] path, baseline 160/5",
        "axes": axes,
    }


def main():
    report = build()
    output = HERE / "path_load_proxy_20261109.json"
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print("written", output)


if __name__ == "__main__":
    main()
