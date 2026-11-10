"""Offline gravity-torque probe for the existing four-axis arm (read-only).

Uses the arm's canonical URDF plus the existing provisional mass audit, the
documented default base gravity (+Y down, pending field confirmation) and the
same fixed J3 placement offset the existing MuJoCo model uses.
No ROS, CAN, serial or motor output.
"""
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "verification"))
from gravity_model import GravityModel  # noqa: E402

URDF = ROOT / "src/slaihand_description/urdf/slaihand_four_axis.urdf"
AUDIT = ROOT / "verification/gravity_preparation/provisional_gravity.json"
NAMES = ["slaihand_joint%d" % i for i in range(1, 5)]
GUARDS = [8.0, 10.0, 2.0, 2.0]          # policy estimate_guard_nm
KP = 160.0
GRAVITY = (0.0, 9.80665, 0.0)            # documented default: base +Y down


def limits():
    import xml.etree.ElementTree as ET
    root = ET.parse(URDF).getroot()
    out = []
    for name in NAMES:
        limit = root.find("joint[@name='%s']/limit" % name)
        out.append((float(limit.get("lower")), float(limit.get("upper"))))
    return out


def main():
    audit = json.loads(AUDIT.read_text(encoding="utf-8"))
    if audit.get("status") != "ESTIMATE_ONLY_NOT_FOR_HARDWARE":
        raise SystemExit("unexpected audit status")
    model = GravityModel(URDF, audit["links"], GRAVITY,
                         fixed_joint_offsets={"slaihand_joint3": math.pi})
    bounds = limits()
    print("limits_deg:", [[round(math.degrees(a), 2), round(math.degrees(b), 2)] for a, b in bounds])
    print("guards_Nm:", GUARDS, " hard-coded policy estimate guard; KP:", KP)

    for label, pose_deg in (("zero", [0, 0, 0, 0]),
                            ("live_start", [-4.109, 0.109, 1.596, 0.809]),
                            ("live_outbound", [5.891, 10.109, 11.596, 10.809])):
        torque, energy = model.evaluate(np.deg2rad(pose_deg))
        print("%-14s deg=%s  g_Nm=%s  guard_ratio=%s" % (
            label, pose_deg, [round(v, 3) for v in torque],
            [round(abs(v) / g, 2) for v, g in zip(torque, GUARDS)]))

    axes = [[lo, hi] for lo, hi in bounds]
    grid = [np.linspace(lo, hi, 7) for lo, hi in axes]
    worst = np.zeros(4)
    worst_pose = [None] * 4
    over = 0
    total = 0
    for q in itertools.product(*grid):
        torque, _ = model.evaluate(np.array(q))
        total += 1
        for index, value in enumerate(torque):
            if abs(value) > worst[index]:
                worst[index] = abs(value)
                worst_pose[index] = [round(math.degrees(v), 1) for v in q]
        if any(abs(v) > g for v, g in zip(torque, GUARDS)):
            over += 1
    print("grid poses:", total, "poses exceeding any guard:", over)
    for index in range(4):
        print("J%d max|g|=%.3f Nm (%.0f%% of guard %.1f) at %s" % (
            index + 1, worst[index], 100 * worst[index] / GUARDS[index], GUARDS[index], worst_pose[index]))

    # Request-estimate headroom: kp * settling error + |g| versus the guard.
    for error_deg in (0.0, 0.1, 0.3, 0.5):
        rows = []
        for index in range(4):
            estimate = KP * math.radians(error_deg) + worst[index]
            rows.append("%.2f/%.1f" % (estimate, GUARDS[index]))
        print("settling_error=%.1f deg -> kp*err+g / guard:", error_deg, rows)


if __name__ == "__main__":
    main()
