"""J3 fixed-offset proof: unified URDF / MuJoCo placement chain and pose table.

Offline only. What this can prove:
  1. where the extra +pi sits in the URDF -> MuJoCo -> gravity-model chain, and
     that the three descriptions share exactly one placement rotation;
  2. that the deployed URDF (repo and expanded) contains no extra pi beyond the
     J3 origin, so the software offset is not double-counting;
  3. the effect on predicted gravity torques at specific poses, including poses
     where J4 rotation flips the sign of the J2 prediction (field discriminator).
What it cannot prove: that the physical J3 body really is rotated 180 degrees.
That is the user-confirmed placement claim (2026-11-01) and needs the field
measurement in the pose table.
"""
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
import control_panel_gravity as gravity  # noqa: E402

URDF = ROOT / "src/slaihand_description/urdf/slaihand_four_axis.urdf"
EXPANDED = ROOT / "verification/hardware/canusb_motion_20261104/expanded_motion.urdf"
MJCF = ROOT / "verification/impedance_demo/shoulder_dynamics.xml"
PROFILE = ROOT / "src/slaihand_bringup/config/gravity_provisional.json"
POLICY = ROOT / "src/slaihand_bringup/config/canusb_motion_policy.yaml"
OUT_JSON = Path(__file__).resolve().parent / "J3偏置核验.json"
OUT_MD = Path(__file__).resolve().parent / "J3偏置核验.md"

J3_NAME = "slaihand_joint3"
PI = math.pi


# ------------------------------------------------------------------ helpers
def mat_to_quat(rotation):
    """(w, x, y, z) from a rotation matrix."""
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        w = 0.25 * scale
        x = (rotation[2][1] - rotation[1][2]) / scale
        y = (rotation[0][2] - rotation[2][0]) / scale
        z = (rotation[1][0] - rotation[0][1]) / scale
    elif rotation[0][0] > rotation[1][1] and rotation[0][0] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2
        w = (rotation[2][1] - rotation[1][2]) / scale
        x = 0.25 * scale
        y = (rotation[0][1] + rotation[1][0]) / scale
        z = (rotation[0][2] + rotation[2][0]) / scale
    elif rotation[1][1] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2
        w = (rotation[0][2] - rotation[2][0]) / scale
        x = (rotation[0][1] + rotation[1][0]) / scale
        y = 0.25 * scale
        z = (rotation[1][2] + rotation[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2
        w = (rotation[1][0] - rotation[0][1]) / scale
        x = (rotation[0][2] + rotation[2][0]) / scale
        y = (rotation[1][2] + rotation[2][1]) / scale
        z = 0.25 * scale
    return (w, x, y, z)


def quat_to_mat(quat):
    w, x, y, z = quat
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def angle_between(first, second):
    """Rotation angle between two rotation matrices, degrees."""
    product = gravity._mat_mul([[first[j][i] for j in range(3)] for i in range(3)], second)
    trace = product[0][0] + product[1][1] + product[2][2]
    cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.degrees(math.acos(cosine))


def origin_of(joint):
    origin = joint.find("origin")
    xyz = [float(v) for v in origin.get("xyz").split()] if origin is not None else [0.0, 0.0, 0.0]
    rpy = [float(v) for v in origin.get("rpy").split()] if origin is not None else [0.0, 0.0, 0.0]
    axis = [float(v) for v in joint.find("axis").get("xyz").split()]
    return xyz, rpy, axis


def chain_for(urdf_path, extra_j3_offset=0.0):
    """Compose body rotations at q=0; returns {link: rotation} and joint data."""
    root = ET.parse(urdf_path).getroot()
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    children = {joint.find("child").get("link") for joint in joints.values()}
    links = [link.get("name") for link in root.findall("link")]
    base = next(link for link in links if link not in children)
    rotations = {base: gravity._mat_identity()}
    for name in ("slaihand_joint1", "slaihand_joint2", "slaihand_joint3", "slaihand_joint4"):
        joint = joints[name]
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        _, rpy, axis = origin_of(joint)
        rotation = gravity._mat_mul(rotations[parent], gravity._rot_xyz(rpy))
        if name == J3_NAME and extra_j3_offset:
            rotation = gravity._mat_mul(rotation, gravity._rot_axis(axis, extra_j3_offset))
        rotations[child] = rotation
    return rotations, joints, base


def mjcf_bodies():
    root = ET.parse(MJCF).getroot()
    bodies = {}
    for body in root.iter("body"):
        quat = body.get("quat")
        if quat:
            values = [float(v) for v in quat.split()]
            bodies[body.get("name")] = quat_to_mat(values)
    return bodies


def main():
    checks = {"status": "PASS", "checks": [], "warnings": []}

    def check(name, condition, detail=None):
        checks["checks"].append({"name": name, "ok": bool(condition), "detail": detail})
        if not condition:
            checks["status"] = "FAIL"

    # 1. URDF chain: the origin already carries a pi-like rotation.
    rotations_plain, joints, base = chain_for(URDF)
    j3 = joints[J3_NAME]
    _, j3_rpy, j3_axis = origin_of(j3)
    check("URDF link base is base_link", base == "base_link", base)
    check("J3 origin rpy contains a pi rotation",
          abs(abs(j3_rpy[0]) - PI) < 5e-2 and abs(abs(j3_rpy[1]) - PI / 2) < 5e-2, j3_rpy)

    # 2. MuJoCo model: bodies are relative placements. Every body equals the URDF
    # joint origin rpy; only link3 (J3) additionally carries the documented +pi
    # placement rotation. This is the same single placement the gravity model uses.
    mujoco = mjcf_bodies()
    if mujoco:
        base_rotation = gravity._rot_xyz([-PI / 2, 0, 0])
        if "base_link" in mujoco:
            check("MuJoCo base_link is the documented base Rx(-pi/2)",
                  angle_between(mujoco["base_link"], base_rotation) < 1e-3,
                  "%.6f deg" % angle_between(mujoco["base_link"], base_rotation))
        for name, link in (("slaihand_joint1", "link1"), ("slaihand_joint2", "link2"),
                           ("slaihand_joint3", "link3"), ("slaihand_joint4", "link4")):
            joint = joints[name]
            _, rpy, axis = origin_of(joint)
            expected_relative = gravity._rot_xyz(rpy)
            if name == J3_NAME:
                expected_relative = gravity._mat_mul(expected_relative,
                                                     gravity._rot_axis(axis, PI))
            if link in mujoco:
                check("MuJoCo relative placement matches URDF%s (%s)" % (
                    " + one J3 pi" if name == J3_NAME else "", link),
                      angle_between(mujoco[link], expected_relative) < 1e-3,
                      "%.6f deg" % angle_between(mujoco[link], expected_relative))
        if "link3" in mujoco:
            _, j3_rpy_local, j3_axis_local = origin_of(joints[J3_NAME])
            without = gravity._rot_xyz(j3_rpy_local)
            check("MuJoCo link3 differs from the plain URDF origin by exactly pi",
                  abs(angle_between(without, mujoco["link3"]) - 180.0) < 1e-3,
                  "%.6f deg" % angle_between(without, mujoco["link3"]))
    else:
        checks["warnings"].append("MuJoCo XML not found; placement cross-check skipped")

    # 3. The extra pi is exactly one half turn about the J3 axis, applied once.
    with_offset, _, _ = chain_for(URDF, extra_j3_offset=PI)
    twice, _, _ = chain_for(URDF, extra_j3_offset=2 * PI)
    for link in ("link3", "link4"):
        check("extra pi rotates %s by one half turn" % link,
              abs(angle_between(rotations_plain[link], with_offset[link]) - 180.0) < 1e-3,
              "%.6f deg" % angle_between(rotations_plain[link], with_offset[link]))
        check("2*pi would cancel the placement (%s)" % link,
              angle_between(twice[link], rotations_plain[link]) < 1e-3,
              "%.6f deg" % angle_between(twice[link], rotations_plain[link]))

    # 4. Deployed URDFs contain no extra pi beyond the origin.
    if EXPANDED.is_file():
        expanded_rotation, expanded_joints, _ = chain_for(EXPANDED)
        check("deployed expanded URDF equals repo URDF chain",
              all(angle_between(expanded_rotation[link], rotations_plain[link]) < 1e-3
                  for link in ("link3", "link4")),
              "expanded_motion.urdf")
        _, expanded_rpy, _ = origin_of(expanded_joints[J3_NAME])
        check("expanded URDF keeps the same J3 origin", expanded_rpy == j3_rpy, expanded_rpy)
    else:
        checks["warnings"].append("expanded_motion.urdf missing; deployment check skipped")

    # 5. Gravity model applies the placement exactly once and in one place.
    model_plain = gravity.FourAxisGravityModel(URDF, PROFILE, gravity_base=(0.0, 9.80665, 0.0))
    model_offset = gravity.FourAxisGravityModel(
        URDF, PROFILE, gravity_base=(0.0, 9.80665, 0.0), fixed_joint_offsets={J3_NAME: PI})
    model_twice = gravity.FourAxisGravityModel(
        URDF, PROFILE, gravity_base=(0.0, 9.80665, 0.0), fixed_joint_offsets={J3_NAME: 2 * PI})
    check("gravity model offset equals the shared placement",
          json.dumps(model_offset.metadata["fixed_joint_offsets_rad"]) ==
          json.dumps({J3_NAME: round(PI, 9)}),
          model_offset.metadata["fixed_joint_offsets_rad"])
    pose = [0.0, 0.0, 0.0, 0.0]
    plain_torque = model_plain.compute(pose)["tau_model_nm"]
    twice_torque = model_twice.compute(pose)["tau_model_nm"]
    check("2*pi model equals the plain chain",
          all(abs(a - b) < 1e-9 for a, b in zip(plain_torque, twice_torque)))

    # 6. Installation claim versus current configuration.
    policy = {}
    for raw in POLICY.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if ":" in line:
            key, value = line.split(":", 1)
            policy[key.strip()] = value.strip()
    claim = {"base_gravity": [0.0, 9.80665, 0.0], "j3_physical_turn_deg": 180.0,
             "extra_end_load": "none", "source": "user confirmed 2026-11-01"}
    configured = {
        "base_gravity": [float(policy.get("gravity_base_x", "nan")),
                         float(policy.get("gravity_base_y", "nan")),
                         float(policy.get("gravity_base_z", "nan"))],
        "j3_offset_rad": float(policy.get("gravity_j3_offset_rad", "nan")),
    }
    check("configured base gravity matches the claimed +Y down",
          configured["base_gravity"] == claim["base_gravity"], configured["base_gravity"])
    check("configured J3 offset matches the claimed 180 degree turn",
          abs(configured["j3_offset_rad"] - PI) < 1e-12, configured["j3_offset_rad"])

    # 7. Field discriminator: J4 rotation flips the J2 prediction sign.
    table = []
    for j4_deg in (-30.0, 0.0, 30.0):
        pose = [0.0, 0.0, 0.0, math.radians(j4_deg)]
        plain = model_plain.compute(pose)["tau_model_nm"]
        offset = model_offset.compute(pose)["tau_model_nm"]
        table.append({"j4_deg": j4_deg,
                      "plain_tau_model_nm": [round(v, 4) for v in plain],
                      "offset_tau_model_nm": [round(v, 4) for v in offset],
                      "delta_nm": [round(b - a, 4) for a, b in zip(plain, offset)]})
    flips = [row for row in table if row["plain_tau_model_nm"][1] * row["offset_tau_model_nm"][1] < 0
             and row["j4_deg"] != 0.0]
    check("J4 rotation yields sign-discriminating J2 predictions (field test)", bool(flips),
          [row["j4_deg"] for row in flips])
    checks["pose_table"] = table
    checks["claim"] = claim
    checks["configured"] = configured

    OUT_JSON.write_text(json.dumps(checks, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    lines = ["# J3 固定偏置核验（离线）", "",
             "结论：%s。这是离线坐标链证据；物理 180° 安装需按第 5 节现场判别，不能由本文件替代。" % checks["status"],
             "", "## 1. 坐标链证据"]
    for item in checks["checks"]:
        lines.append("- [%s] %s%s" % ("OK" if item["ok"] else "FAIL", item["name"],
                                      "" if item["detail"] is None else "：`%s`" % item["detail"]))
    lines += ["", "## 2. 现场判别姿态（J4 转动使 J2 预测反号）",
              "", "| J4 ° | 无偏置 J1/J2/J3/J4 N·m | 有偏置 J1/J2/J3/J4 N·m | Δ N·m |",
              "|---:|---|---|---|"]
    for row in table:
        lines.append("| %g | %s | %s | %s |" % (
            row["j4_deg"], row["plain_tau_model_nm"], row["offset_tau_model_nm"], row["delta_nm"]))
    lines += ["", "## 3. 安装声明", "", "- 声明：%s" % claim,
              "- 当前配置：%s" % configured,
              "- 说明：软件偏置只表达该声明一次；URDF origin 已含的 π 与它相加会变成 2π，不是同一件事。",
              "- 现场步骤：J1=J2=0、J3=0，J4 分别取 ±30°，每次静止保持后读取 J2 的 `effort` 估计与位置误差；"
              "若有支撑/接触须记录，不能把支撑力当作重力。",
              "- 数据不足或噪声大于约 0.2 N·m 时判为不可辨识，需增加姿态或提供质量信息。"]
    if checks["warnings"]:
        lines += ["", "## 4. 警告", ""] + ["- " + warning for warning in checks["warnings"]]
    OUT_MD.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"status": checks["status"],
                      "checks": [(item["name"], item["ok"]) for item in checks["checks"]],
                      "pose_table": table}, ensure_ascii=False, indent=2))
    return 0 if checks["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
