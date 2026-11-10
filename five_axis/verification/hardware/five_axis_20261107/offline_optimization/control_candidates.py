"""Legacy helper functions for the early unsigned-magnitude study.

WARNING (2026-11-09 review): the original result files contain
``max(abs(actual-target))`` over a dwell-end window, not signed static error.
The helper names below are retained for the historical offline tests only.  The
``implied_torque`` value is an unsigned magnitude, NOT a signed gravity torque,
and must not be used to build a deployable feedforward model.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

BASE_KP = 160.0
BASE_KD = 5.0
PROTOCOL_KP_MAX = 500.0
AXES = ("J1", "J2", "J3", "J4", "J5")


@dataclass(frozen=True)
class Observation:
    angle_deg: float
    error_deg: float
    torque_nm: float


def implied_torque(error_deg):
    return BASE_KP * math.radians(error_deg)


def make_observations(errors_by_axis):
    """errors_by_axis: {axis: [(angle_deg, error_deg), ...]}"""
    return {axis: [Observation(angle, error, implied_torque(error))
                   for angle, error in points]
            for axis, points in errors_by_axis.items()}


def constant_gain_residual(observations, axis, kp):
    return [math.degrees(item.torque_nm / kp) for item in observations[axis]]


def scheduled_gain(axis, angle_deg, low=BASE_KP, high=PROTOCOL_KP_MAX,
                   full_angle_deg=30.0):
    if axis in ("J3", "J5"):
        return BASE_KP
    phase = min(max(float(angle_deg), 0.0), float(full_angle_deg))
    return low + (high - low) * phase / float(full_angle_deg)


def scheduled_gain_residual(observations, axis, **kwargs):
    return [math.degrees(item.torque_nm / scheduled_gain(axis, item.angle_deg, **kwargs))
            for item in observations[axis]]


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


def feedforward_fit(observations, degree=2):
    return {axis: polyfit([(item.angle_deg, item.torque_nm) for item in observations[axis]],
                          degree)
            for axis in observations}


def leave_one_out_torque_error(observations, degree=2):
    result = {}
    for axis in observations:
        points = [(item.angle_deg, item.torque_nm) for item in observations[axis]]
        worst = 0.0
        for index, (angle, torque) in enumerate(points):
            training = points[:index] + points[index + 1:]
            if len(training) < degree + 1:
                continue
            model = polyfit(training, degree)
            worst = max(worst, abs(polyval(model, angle) - torque))
        result[axis] = worst
    return result


def feedforward_residual(observations, fit, kp=BASE_KP):
    return {axis: [math.degrees((item.torque_nm - polyval(fit[axis], item.angle_deg)) / kp)
                   for item in points]
            for axis, points in observations.items()}


def integral_final_error(initial_error_deg, ki, kp=BASE_KP, duration_s=10.0, dt=0.01):
    """Very small ideal integral-trim model, not a controller stability proof."""
    error = math.radians(initial_error_deg)
    trim = 0.0
    for _ in range(int(duration_s / dt)):
        trim += ki * error * dt
        error = (implied_torque(initial_error_deg) - trim) / kp
    return math.degrees(error), trim
