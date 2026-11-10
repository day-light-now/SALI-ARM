"""Four-axis gravity feedforward adapted to the existing SLAIHand arm — offline only.

Inputs are the arm's canonical URDF, the existing provisional mass audit
(ESTIMATE_ONLY_NOT_FOR_HARDWARE), the documented default base gravity vector
(base +Y down, still pending a field check) and the same fixed J3 placement
offset the existing MuJoCo model uses.  Outputs the URDF holding torque and the
motor-frame torque (hardware_sign * tau_urdf) that a future native feedforward
path would carry.

Rules kept on purpose:
- refuses to run without ``--allow-provisional`` because the masses are not
  calibrated for hardware;
- never writes ROS parameters, CAN frames or Flash; no serial device is opened;
- reports the remaining estimate-guard headroom so a feedforward value can be
  compared with the protected request-estimate guard.

Usage:
    python3 gravity_ff.py --allow-provisional --degrees -4.1 0.1 1.6 0.8
    python3 gravity_ff.py --allow-provisional --grid
"""
import argparse
import itertools
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "verification"))
from gravity_model import GravityModel  # noqa: E402

URDF = ROOT / "src/slaihand_description/urdf/slaihand_four_axis.urdf"
MASSES = ROOT / "verification/gravity_preparation/provisional_gravity.json"
MAPPING = ROOT / "src/slaihand_bringup/config/hardware_mapping.yaml"
NAMES = ["slaihand_joint%d" % i for i in range(1, 5)]
# Documented default from the shoulder-installation screenshot; still unverified in the field.
DEFAULT_GRAVITY_BASE = (0.0, 9.80665, 0.0)
# Same fixed downstream-body placement the existing MuJoCo model applies.
DEFAULT_FIXED_OFFSETS = {"slaihand_joint3": math.pi}
KP_DEFAULT = 160.0
# policy estimate_guard_nm, indexed J1..J4
ESTIMATE_GUARDS = (8.0, 10.0, 2.0, 2.0)


def hardware_signs(path=MAPPING):
    """q_model = sign * (encoder - zero); torque maps with the same sign."""
    signs = {}
    current = None
    for raw in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if line.startswith("  slaihand_joint") and line.endswith(":"):
            current = line.strip()[:-1]
        elif current and line.strip().startswith("hardware_sign:"):
            value = line.split(":", 1)[1].strip()
            if value in ("1", "+1", "-1"):
                signs[current] = int(value)
            current = None
    for name in NAMES:
        if signs.get(name) not in (-1, 1):
            raise ValueError("hardware_sign missing/invalid for %s" % name)
    return {name: signs[name] for name in NAMES}


class FourAxisGravityFeedforward:
    def __init__(self, urdf=URDF, masses=MASSES, gravity_base=DEFAULT_GRAVITY_BASE,
                 fixed_offsets=None, signs=None):
        audit = json.loads(Path(masses).read_text(encoding="utf-8"))
        if audit.get("status") != "ESTIMATE_ONLY_NOT_FOR_HARDWARE":
            raise ValueError("unexpected mass audit status: %r" % audit.get("status"))
        self.audit_status = audit["status"]
        self.model = GravityModel(urdf, audit["links"], gravity_base,
                                  fixed_joint_offsets=DEFAULT_FIXED_OFFSETS if fixed_offsets is None
                                  else fixed_offsets)
        if list(self.model.names) != NAMES:
            raise ValueError("unexpected joint order: %s" % self.model.names)
        self.signs = hardware_signs() if signs is None else {n: int(signs[n]) for n in NAMES}
        root = ET.parse(urdf).getroot()
        self.limits = []
        for name in NAMES:
            limit = root.find("joint[@name='%s']/limit" % name)
            self.limits.append((float(limit.get("lower")), float(limit.get("upper"))))

    def torques(self, degrees):
        values = [float(v) for v in degrees]
        if len(values) != 4 or any(not math.isfinite(v) for v in values):
            raise ValueError("需要四个有限角度")
        q = np.deg2rad(values)
        for value, (low, high) in zip(q, self.limits):
            if not low <= value <= high:
                raise ValueError("角度超出 URDF 限位；不裁剪")
        urdf, energy = self.model.evaluate(q)
        motor = np.array([self.signs[name] * urdf[index] for index, name in enumerate(NAMES)])
        return {"degrees": values, "tau_urdf_nm": [float(v) for v in urdf],
                "tau_motor_nm": [float(v) for v in motor],
                "potential_energy_j": float(energy)}

    def guard_headroom(self, kp=KP_DEFAULT):
        """Worst-case |tau| over the workspace and the allowed position error."""
        worst = np.zeros(4)
        worst_pose = [None] * 4
        for q in itertools.product(*[np.linspace(lo, hi, 7) for lo, hi in self.limits]):
            urdf, _ = self.model.evaluate(np.array(q))
            motor = np.abs(np.array([self.signs[name] * urdf[i] for i, name in enumerate(NAMES)]))
            for index, value in enumerate(motor):
                if value > worst[index]:
                    worst[index] = value
                    worst_pose[index] = [round(math.degrees(v), 1) for v in q]
        rows = []
        for index, name in enumerate(NAMES):
            guard = ESTIMATE_GUARDS[index]
            headroom = guard - worst[index]
            rows.append({"joint": name, "max_abs_tau_motor_nm": round(float(worst[index]), 3),
                         "guard_nm": guard,
                         "headroom_nm": round(float(headroom), 3),
                         "allowed_error_deg_at_kp": round(math.degrees(max(headroom, 0.0) / kp), 3),
                         "worst_pose_deg": worst_pose[index]})
        return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--degrees", type=float, nargs=4)
    parser.add_argument("--grid", action="store_true", help="workspace worst case and guard headroom")
    parser.add_argument("--allow-provisional", action="store_true",
                        help="acknowledge uncalibrated CAD masses (offline review only)")
    args = parser.parse_args()
    if not args.allow_provisional:
        parser.error("CAD 质量未标定，必须显式 --allow-provisional；不得直接用于硬件")
    feedforward = FourAxisGravityFeedforward()
    if args.grid:
        print(json.dumps({"status": "PROVISIONAL_OFFLINE_ONLY", "mass_audit": feedforward.audit_status,
                          "guard_headroom": feedforward.guard_headroom()}, ensure_ascii=False, indent=2))
    if args.degrees:
        print(json.dumps({"status": "PROVISIONAL_OFFLINE_ONLY", **feedforward.torques(args.degrees)},
                         ensure_ascii=False, indent=2))
    if not args.grid and not args.degrees:
        parser.error("需要 --degrees 或 --grid")


if __name__ == "__main__":
    main()
