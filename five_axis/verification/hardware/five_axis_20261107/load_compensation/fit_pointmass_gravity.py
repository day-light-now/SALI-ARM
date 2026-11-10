"""Feasibility check: point-mass gravity fit from the multi-pose torque proxy.

No ROS/hardware. J5 is not a physical gravity joint; it is a fixed end mass on
J4. The URDF has visual geometry only (no inertial blocks), so this script fits
four lumped point masses at the J1-J4 link origins. It is a feasibility screen,
not a deliverable gravity model.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
GRAVITY = (0.0, 0.0, -9.81)

# Fixed joint transforms copied from slaihand_four_axis.urdf.
# (xyz, rpy, axis)
JOINTS = (
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
    ((0.039, 0.0068669849, 0.1351999710), (1.5707963268, 0.0, 1.5707963268), (0.0, 0.0, 1.0)),
    ((0.0797365276843, -0.000108104601912, -0.0365), (3.14023651841, -1.57079632679, 0.0), (0.0, 0.0, -1.0)),
    ((0.04125, 1.25982557719e-13, 0.1395), (-3.14159265359, -1.57079632679, 0.0), (0.0, 0.0, -1.0)),
)


def matmul(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def xyz_rpy(xyz, rpy):
    matrix = [[1, 0, 0, xyz[0]], [0, 1, 0, xyz[1]], [0, 0, 1, xyz[2]], [0, 0, 0, 1]]
    return matmul(matmul(rot_z(rpy[2]), rot_y(rpy[1])), matmul(rot_x(rpy[0]), matrix))


def axis_angle(axis, angle):
    x, y, z = axis
    c, s, v = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [[c + x*x*v, x*y*v - z*s, x*z*v + y*s, 0],
            [y*x*v + z*s, c + y*y*v, y*z*v - x*s, 0],
            [z*x*v - y*s, z*y*v + x*s, c + z*z*v, 0],
            [0, 0, 0, 1]]


def link_origins(q):
    transform = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    origins = []
    for (xyz, rpy, axis), angle in zip(JOINTS, q):
        transform = matmul(transform, matmul(xyz_rpy(xyz, rpy), axis_angle(axis, angle)))
        origins.append((transform[0][3], transform[1][3], transform[2][3]))
    return origins


def regressor(q):
    """d(link_i origin)/d(q_j) columns for point masses at link origins."""
    origins = link_origins(q)
    base = list(origins)
    columns = []
    for j in range(4):
        step = 1e-4
        plus, minus = list(q), list(q)
        plus[j] += step
        minus[j] -= step
        d_plus, d_minus = link_origins(plus), link_origins(minus)
        dp = [tuple((d_plus[i][k] - d_minus[i][k]) / (2 * step) for k in range(3))
              for i in range(4)]
        columns.append(dp)
    return origins, columns


def gravity_torque(q, masses):
    origins, columns = regressor(q)
    torques = []
    for j in range(4):
        value = 0.0
        for i in range(4):
            value += masses[i] * sum(GRAVITY[k] * columns[j][i][k] for k in range(3))
        torques.append(value)
    return torques


def solve(matrix, vector):
    size = len(vector)
    a = [list(matrix[i]) + [vector[i]] for i in range(size)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(a[row][column]))
        if abs(a[pivot][column]) < 1e-12:
            raise ValueError("singular")
        a[column], a[pivot] = a[pivot], a[column]
        scale = a[column][column]
        a[column] = [value / scale for value in a[column]]
        for row in range(size):
            if row == column:
                continue
            factor = a[row][column]
            a[row] = [x - factor * y for x, y in zip(a[row], a[column])]
    return [a[i][-1] for i in range(size)]


def fit(poses):
    size = 4
    matrix = [[0.0] * size for _ in range(size)]
    vector = [0.0] * size
    for q, tau in poses:
        _, columns = regressor(q)
        # One row per joint j; unknown m_i.
        for j in range(4):
            for i in range(size):
                row = sum(GRAVITY[k] * columns[j][i][k] for k in range(3))
                vector[i] += row * tau[j]
                for k in range(size):
                    other = sum(GRAVITY[d] * columns[j][k][d] for d in range(3))
                    matrix[i][k] += row * other
    return solve(matrix, vector)


def predict(masses, q):
    return gravity_torque(q, masses)


def collect():
    poses = []
    for stem in ("multi_pose_20", "multi_pose_40"):
        summary = json.loads((HERE / (stem + "_summary_20261109.json")).read_text(encoding="utf-8"))
        torque = json.loads((HERE / (stem + "_torque_20261109.json")).read_text(encoding="utf-8"))
        for segment, torque_segment in zip(summary["segments"], torque["segments"]):
            q = [math.radians(value) for value in segment["reference_deg"][:4]]
            tau = [torque_segment["axes"]["slaihand_joint%d" % (i + 1)]["effort_nm"]["mean"]
                   for i in range(4)]
            poses.append((q, tau))
    return poses


def main():
    poses = collect()
    try:
        masses = fit(poses)
    except ValueError as error:
        output = {
            "schema": "slaihand_pointmass_gravity_feasibility/1",
            "source": "multi_pose_20/40 torque estimates",
            "assumption": "point masses at J1-J4 link origins; J5 is a fixed end mass, not a gravity joint",
            "sample_count": len(poses),
            "singular": True,
            "error": str(error),
            "deployable": False,
            "note": "The four-origin point-mass regressor is rank deficient. URDF has no inertial blocks; J5 is a fixed payload on J4 and cannot make the missing link COM parameters identifiable.",
        }
        print(json.dumps(output, ensure_ascii=False, indent=2))
        (HERE / "pointmass_gravity_feasibility_20261109.json").write_text(
            json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return
    loo = []
    for index, (q, tau) in enumerate(poses):
        model = fit(poses[:index] + poses[index + 1:])
        predicted = predict(model, q)
        loo.append(max(abs(predicted[j] - tau[j]) for j in range(4)))
    output = {
        "schema": "slaihand_pointmass_gravity_feasibility/1",
        "source": "multi_pose_20/40 torque estimates",
        "assumption": "point masses at J1-J4 link origins; J5 is a fixed end mass, not a gravity joint",
        "masses_kg": masses,
        "sample_count": len(poses),
        "loo_max_joint_error_nm": max(loo),
        "loo_mean_joint_error_nm": sum(loo) / len(loo),
        "deployable": False,
        "note": "URDF has no inertial blocks; actual link COMs and J5 payload COM are not modeled.",
    }
    print(json.dumps(output, ensure_ascii=False, indent=2))
    (HERE / "pointmass_gravity_feasibility_20261109.json").write_text(
        json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
