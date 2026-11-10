"""四轴重力模型、坐标映射与 shadow 估算（纯标准库，不依赖 ROS/numpy）。

设计约束（Codex《审阅_重力补偿》）：
- 唯一坐标转换：q_model = sign*(raw-zero)，v_model = sign*v，tau_protocol = sign*tau_model；
- 重力由实测姿态计算，本阶段只做 shadow（只显示，不进入命令）；
- 模型/质量档案/重力方向/固定偏置全部显式，带来源与哈希；暂定质量不得冒充已标定；
- 越限拒绝不裁剪；不访问串口/CAN，不写任何文件。

Python 侧同步实现，便于面板在无 numpy/scipy 的环境加载；测试用仓库现有
verification/gravity_model.py（numpy/scipy 解析模型）交叉核对。
"""
from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

JOINT_NAMES = ("slaihand_joint1", "slaihand_joint2", "slaihand_joint3", "slaihand_joint4")
BACKEND = "analytic-urdf"
FORMAT = "four_axis_gravity/1"
DEFAULT_GRAVITY_BASE = (0.0, 9.80665, 0.0)
# 现有 MuJoCo 模型采用的固定放置修正；正式模型必须显式选择且只应用一次。
KNOWN_J3_PLACEMENT_OFFSET = math.pi


class GravityModelError(ValueError):
    """模型/配置/输入不合法。"""


# ----------------------------------------------------------------- 3D helpers
def _mat_identity():
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def _mat_mul(a, b):
    return [[a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
             for j in range(3)] for i in range(3)]


def _mat_vec(a, v):
    return [_dot(a[i], v) for i in range(3)]


def _rot_xyz(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # URDF rpy: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _rot_axis(axis, angle):
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm <= 0.0 or not math.isfinite(norm):
        raise GravityModelError("关节轴无效")
    x, y, z = x / norm, y / norm, z / norm
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def _sha256(path):
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        return None


def _finite(values):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
               for v in values)


# ------------------------------------------------------------------- model
class FourAxisGravityModel:
    """由现有 URDF + 质量档案计算四轴重力矩（URDF 关节坐标，∂U/∂q）。"""

    def __init__(self, urdf_path, mass_profile_path, gravity_base=DEFAULT_GRAVITY_BASE,
                 fixed_joint_offsets=None, joint_names=JOINT_NAMES):
        self.urdf_path = Path(urdf_path)
        self.mass_profile_path = Path(mass_profile_path)
        if not self.urdf_path.is_file():
            raise GravityModelError("URDF 不存在：%s" % self.urdf_path)
        if not self.mass_profile_path.is_file():
            raise GravityModelError("质量档案不存在：%s" % self.mass_profile_path)
        if not _finite(gravity_base) or len(gravity_base) != 3:
            raise GravityModelError("重力向量无效")
        self.gravity_base = [float(v) for v in gravity_base]
        self.fixed_joint_offsets = dict(fixed_joint_offsets or {})
        for name, value in self.fixed_joint_offsets.items():
            if name not in joint_names or not math.isfinite(value):
                raise GravityModelError("固定偏置无效：%s" % name)

        profile = json.loads(self.mass_profile_path.read_text(encoding="utf-8"))
        self.first_moments = {}
        self.parameterization = 'mass_com'
        if profile.get('schema') == 'slaihand_identified_gravity_base/1':
            if any(name != 'slaihand_joint3' and abs(value) > 1e-12
                   for name,value in self.fixed_joint_offsets.items()):
                raise GravityModelError('识别参数不支持额外关节偏置')
            identity = profile.get('identity') or {}
            if identity.get('urdf_sha256') != _sha256(self.urdf_path):
                raise GravityModelError('识别参数与当前URDF版本不一致')
            if list(identity.get('gravity_base_m_s2') or []) != self.gravity_base:
                raise GravityModelError('识别参数重力方向不一致')
            offset = self.fixed_joint_offsets.get('slaihand_joint3', 0.)
            if not _finite([identity.get('j3_offset_rad')]) or abs(identity['j3_offset_rad']-offset) > 1e-9:
                raise GravityModelError('识别参数J3偏置不一致')
            names = profile.get('parameter_slots')
            values = profile.get('effective_coefficients')
            if not isinstance(names, list) or not isinstance(values, list) or len(names) != len(values) or not _finite(values):
                raise GravityModelError('识别基参数字段无效')
            entries = {}
            for name, value in zip(names, values):
                if not isinstance(name, str) or name.count(':') != 1:
                    raise GravityModelError('识别基参数名称无效')
                link, field = name.split(':')
                if field not in ('mass', 'hx', 'hy', 'hz') or field in entries.setdefault(link, {}):
                    raise GravityModelError('识别基参数重复或字段无效')
                entries[link][field] = float(value)
            for link, entry in entries.items():
                if set(entry) != {'mass', 'hx', 'hy', 'hz'}:
                    raise GravityModelError('识别基参数不完整')
                self.first_moments[link] = [entry[field] for field in ('hx', 'hy', 'hz')]
            profile = dict(profile, links={link: {'mass_kg':entry['mass'], 'com_link_m':[0.,0.,0.]}
                                           for link,entry in entries.items()})
            self.parameterization = 'identified_base'
        links = profile.get("links")
        if not isinstance(links, dict):
            raise GravityModelError("质量档案缺少 links")
        self.mass_status = (profile.get("metadata") or {}).get("status") or profile.get("status")
        self.masses = {}
        total = 0.0
        for link, entry in links.items():
            mass = entry.get("mass_kg")
            com = entry.get("com_link_m")
            if not isinstance(mass, (int, float)) or not math.isfinite(mass) or (mass <= 0 and self.parameterization == 'mass_com'):
                raise GravityModelError("质量无效：%s" % link)
            if not isinstance(com, list) or len(com) != 3 or not _finite(com):
                raise GravityModelError("重心无效：%s" % link)
            self.masses[link] = (float(mass), [float(v) for v in com])
            total += float(mass)
        self.total_mass_kg = total if self.parameterization == 'mass_com' else None

        root = ET.parse(self.urdf_path).getroot()
        self.links = [link.get("name") for link in root.findall("link")]
        self.joints = []
        for joint in root.findall("joint"):
            kind = joint.get("type")
            if kind not in ("revolute", "fixed"):
                raise GravityModelError("仅支持 revolute/fixed 关节：%s" % joint.get("name"))
            name = joint.get("name")
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            origin = joint.find("origin")
            xyz = [float(v) for v in (origin.get("xyz", "0 0 0").split() if origin is not None else "0 0 0".split())]
            rpy = [float(v) for v in (origin.get("rpy", "0 0 0").split() if origin is not None else "0 0 0".split())]
            axis = None
            limits = None
            if kind == "revolute":
                axis_node = joint.find("axis")
                axis = [float(v) for v in axis_node.get("xyz").split()] if axis_node is not None else [0, 0, 1]
                norm = math.sqrt(_dot(axis, axis))
                if not math.isfinite(norm) or norm <= 0:
                    raise GravityModelError("关节轴无效：%s" % name)
                axis = [v / norm for v in axis]
                limit = joint.find("limit")
                if limit is None:
                    raise GravityModelError("缺少限位：%s" % name)
                limits = (float(limit.get("lower")), float(limit.get("upper")))
                if not _finite(limits) or limits[0] >= limits[1]:
                    raise GravityModelError("限位无效：%s" % name)
            self.joints.append({"name": name, "kind": kind, "parent": parent, "child": child,
                                "xyz": xyz, "rpy": rpy, "axis": axis, "limits": limits})
        for joint in self.joints:
            if joint["child"] not in self.masses and joint["kind"] == "fixed":
                raise GravityModelError("固定关节子连杆缺少质量：%s" % joint["child"])

        movable = [j["name"] for j in self.joints if j["kind"] == "revolute"]
        if set(movable) != set(joint_names):
            raise GravityModelError("可动关节集合不符：%s" % movable)
        self.joint_names = list(joint_names)
        self.name_index = {name: i for i, name in enumerate(self.joint_names)}
        self.limits = []
        for name in self.joint_names:
            joint = next(j for j in self.joints if j["name"] == name)
            self.limits.append(joint["limits"])

        children = {j["child"] for j in self.joints}
        roots = [link for link in self.links if link not in children]
        if len(roots) != 1:
            raise GravityModelError("运动树必须只有一个根")
        self.root = roots[0]
        if self.parameterization == 'identified_base':
            expected_links = set(self.links) - {self.root}
            if set(self.first_moments) != expected_links:
                raise GravityModelError('识别基参数的连杆集合与URDF不一致')
        self.metadata = {
            "backend": BACKEND,
            "format": FORMAT,
            "urdf": str(self.urdf_path),
            "urdf_sha256": _sha256(self.urdf_path),
            "mass_profile": str(self.mass_profile_path),
            "mass_profile_sha256": _sha256(self.mass_profile_path),
            "mass_profile_status": self.mass_status,
            "total_mass_kg": round(total, 6) if self.total_mass_kg is not None else None,
            "parameterization": self.parameterization,
            "gravity_base_m_s2": list(self.gravity_base),
            "gravity_source": "explicit_config",
            "fixed_joint_offsets_rad": {k: round(v, 9) for k, v in self.fixed_joint_offsets.items()},
            "joint_names": list(self.joint_names),
            "limits_rad": [[round(lo, 9), round(hi, 9)] for lo, hi in self.limits],
            "units": {"mass": "kg", "position": "m", "torque": "N*m", "angle": "rad"},
        }

    # -- evaluation ----------------------------------------------------
    def _evaluate(self, q_model):
        values = [float(v) for v in q_model]
        if len(values) != len(self.joint_names) or not _finite(values):
            raise GravityModelError("需要四个有限关节角")
        for value, (low, high) in zip(values, self.limits):
            if not low <= value <= high:
                raise GravityModelError("关节角超出 URDF 限位；不裁剪")
        angles = {name: values[self.name_index[name]] for name in self.joint_names}
        frames = {self.root: ([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [0.0, 0.0, 0.0])}
        chains = {self.root: []}
        torque = [0.0] * len(self.joint_names)
        energy = 0.0
        pending = list(self.joints)
        while pending:
            progressed = False
            for joint in list(pending):
                if joint["parent"] not in frames:
                    continue
                parent_rot, parent_pos = frames[joint["parent"]]
                local_rot = _mat_mul(parent_rot, _rot_xyz(joint["rpy"]))
                local_pos = _add(parent_pos, _mat_vec(parent_rot, joint["xyz"]))
                chain = list(chains[joint["parent"]])
                if joint["kind"] == "revolute":
                    name = joint["name"]
                    fixed = _rot_axis(joint["axis"], self.fixed_joint_offsets.get(name, 0.0))
                    local_rot = _mat_mul(local_rot, fixed)
                    chain.append((self.name_index[name], list(local_pos), _mat_vec(local_rot, joint["axis"])))
                    motion = _rot_axis(joint["axis"], angles[name])
                    local_rot = _mat_mul(local_rot, motion)
                frames[joint["child"]] = (local_rot, local_pos)
                chains[joint["child"]] = chain
                entry = self.masses.get(joint["child"])
                if entry is not None:
                    mass, com = entry
                    if joint['child'] in self.first_moments:
                        rotated = _mat_vec(local_rot, self.first_moments[joint['child']])
                        weighted_center = [mass*p+h for p,h in zip(local_pos,rotated)]
                        energy -= _dot(self.gravity_base, weighted_center)
                        for index, pivot, axis in chain:
                            weighted_arm = [center-mass*p for center,p in zip(weighted_center,pivot)]
                            torque[index] -= _dot(axis, _cross(weighted_arm,self.gravity_base))
                        pending.remove(joint)
                        progressed = True
                        continue
                    center = _add(local_pos, _mat_vec(local_rot, com))
                    force = [mass * g for g in self.gravity_base]
                    energy -= _dot(force, center)
                    for index, pivot, axis in chain:
                        torque[index] -= _dot(axis, _cross(_sub(center, pivot), force))
                pending.remove(joint)
                progressed = True
            if not progressed:
                raise GravityModelError("运动树不连通或存在环")
        return torque, energy

    def compute(self, q_model):
        torque, energy = self._evaluate(q_model)
        return {"joint_names": list(self.joint_names),
                "tau_model_nm": torque,
                "potential_energy_j": energy,
                "metadata": self.metadata}

    def potential_energy(self, q_model):
        return self._evaluate(q_model)[1]


# ------------------------------------------------------------- coordinates
class CoordinateMap:
    """q_model = sign*(raw-zero)；力矩按虚功一致 tau_protocol = sign*tau_model。"""

    def __init__(self, mapping_path, joint_names=JOINT_NAMES):
        mapping_path = Path(mapping_path)
        if not mapping_path.is_file():
            raise GravityModelError("hardware_mapping.yaml 不存在：%s" % mapping_path)
        signs, zeros = {}, {}
        current = None
        for raw in mapping_path.read_text(encoding="utf-8").splitlines():
            line = raw.split("#", 1)[0].rstrip()
            if line.startswith("  slaihand_joint") and line.rstrip().endswith(":"):
                current = line.strip()[:-1]
            elif current and line.strip().startswith("hardware_sign:"):
                value = line.split(":", 1)[1].strip()
                if value in ("1", "+1", "-1"):
                    signs[current] = int(value)
            elif current and line.strip().startswith("zero_offset_rad:"):
                value = line.split(":", 1)[1].strip()
                try:
                    zeros[current] = float(value)
                except ValueError:
                    pass
        for name in joint_names:
            if name not in signs or name not in zeros:
                raise GravityModelError("映射缺少 %s 的 hardware_sign/zero_offset_rad" % name)
        self.joint_names = list(joint_names)
        self.signs = [signs[name] for name in self.joint_names]
        self.zeros = [zeros[name] for name in self.joint_names]
        self.metadata = {"mapping": str(mapping_path), "mapping_sha256": _sha256(mapping_path),
                         "signs": dict(zip(self.joint_names, self.signs)),
                         "zeros_rad": dict(zip(self.joint_names, self.zeros))}

    def _check(self, values, label):
        values = [float(v) for v in values]
        if len(values) != len(self.joint_names) or not _finite(values):
            raise GravityModelError("%s 需要四个有限值" % label)
        return values

    def to_model_angle(self, raw):
        raw = self._check(raw, "协议角")
        return [sign * (value - zero) for sign, zero, value in zip(self.signs, self.zeros, raw)]

    def to_protocol_angle(self, model):
        model = self._check(model, "模型角")
        return [zero + sign * value for sign, zero, value in zip(self.signs, self.zeros, model)]

    def to_model_velocity(self, velocity_protocol):
        velocity = self._check(velocity_protocol, "协议速度")
        return [sign * value for sign, value in zip(self.signs, velocity)]

    def to_protocol_torque(self, torque_model):
        torque = self._check(torque_model, "模型力矩")
        return [sign * value for sign, value in zip(self.signs, torque)]

    def to_model_torque(self, torque_protocol):
        torque = self._check(torque_protocol, "协议力矩")
        return [sign * value for sign, value in zip(self.signs, torque)]


# ----------------------------------------------------------------- policy
def load_motion_policy(policy_path):
    """只读取 shadow 显示需要的标量；生效值仍以驱动启动时使用的 URDF 参数为准。"""
    policy_path = Path(policy_path)
    if not policy_path.is_file():
        raise GravityModelError("policy 不存在：%s" % policy_path)
    values, guards = {}, None
    for raw in policy_path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key, value = key.strip(), value.strip()
        if key == "estimate_guard_nm":
            guards = [float(v) for v in value.strip("[]").split(",")]
        elif key in ("kp", "kd", "reference_speed_rad_s", "feedback_speed_rad_s",
                     "following_error_deg", "temperature_c"):
            values[key] = float(value)
    if guards is None or len(guards) != 4:
        raise GravityModelError("policy 缺少 estimate_guard_nm")
    values["estimate_guard_nm"] = guards
    values["policy"] = str(policy_path)
    values["policy_sha256"] = _sha256(policy_path)
    return values


def gravity_profile_filename(policy_path):
    for raw in Path(policy_path).read_text(encoding='utf-8').splitlines():
        line = raw.split('#', 1)[0].strip()
        if line.startswith('gravity_profile:'):
            name = line.split(':', 1)[1].strip().strip('\"\'')
            if not name or Path(name).name != name or '/' in name or '\\' in name:
                raise GravityModelError('重力参数档案须为config目录内的文件名')
            return name
    return 'gravity_provisional.json'


# ----------------------------------------------------------------- shadow
class GravityShadow:
    """只计算并展示重力估算；ff 系数 alpha 恒为 0，绝不进入命令。"""

    def __init__(self, urdf_path, mass_profile_path, mapping_path, policy_path,
                 gravity_base=DEFAULT_GRAVITY_BASE, fixed_joint_offsets=None):
        self.model = FourAxisGravityModel(urdf_path, mass_profile_path, gravity_base,
                                          fixed_joint_offsets=fixed_joint_offsets)
        self.coords = CoordinateMap(mapping_path)
        if self.model.parameterization == 'identified_base':
            profile = json.loads(Path(mass_profile_path).read_text(encoding='utf-8'))
            if profile['identity']['mapping_sha256'] != _sha256(mapping_path):
                raise GravityModelError('识别参数与当前坐标映射不一致')
        self.policy = load_motion_policy(policy_path)
        self.kp = self.policy["kp"]
        self.kd = self.policy["kd"]
        self.guards = self.policy["estimate_guard_nm"]
        self.alpha = 0.0
        self.shadow_alpha = 1.0
        self.metadata = dict(self.model.metadata)
        self.metadata.update({
            "coordinate": self.coords.metadata,
            "kp": self.kp, "kd": self.kd, "guards_nm": self.guards,
            "alpha": self.alpha, "shadow_alpha": self.shadow_alpha, "beta": 0.0,
            "mode": "shadow", "ff_applied": False,
            "policy_sha256": self.policy["policy_sha256"],
        })

    def estimate(self, positions_model, velocities_model=None, reference_model=None,
                 applied_ff_motor_nm=None, gravity_float=False):
        """Shadow estimate. Three separate quantities are reported:

        - p/d terms and "applied headroom": computed from the real controller
          reference and the feedforward values the driver actually wrote;
        - "predicted" headroom: what the guard would read if the hypothetical
          compensation (shadow_alpha * g) were applied; explicitly a model
          prediction, not an applied value;
        - when the reference is unknown every P term and every total headroom is
          reported as None (unknown), never as zero-error arithmetic.
        """
        positions = [float(v) for v in positions_model]
        gravity = self.model.compute(positions)
        tau_model = gravity["tau_model_nm"]
        tau_protocol = self.coords.to_protocol_torque(tau_model)
        velocity = [0.0] * 4 if velocities_model is None else [float(v) for v in velocities_model]
        if len(velocity) != 4 or not _finite(velocity):
            raise GravityModelError("速度需要四个有限值")
        reference = None if reference_model is None else [float(v) for v in reference_model]
        if reference is not None and (len(reference) != 4 or not _finite(reference)):
            raise GravityModelError("参考角需要四个有限值")
        applied = None if applied_ff_motor_nm is None else [float(v) for v in applied_ff_motor_nm]
        if applied is not None and (len(applied) != 4 or not _finite(applied)):
            raise GravityModelError("已应用前馈需要四个有限值")
        rows = []
        kp, kd = (0., 5.) if gravity_float else (self.kp, self.kd)
        warnings = []
        if reference is None:
            warnings.append("控制器参考未知：P 项与总余量显示为未知，不以零误差假设填充")
        for index, name in enumerate(self.model.joint_names):
            guard = self.guards[index]
            d_term = -kd * velocity[index]
            predicted_ff = self.coords.signs[index] * (self.shadow_alpha * tau_model[index])
            row = {
                "joint": name,
                "q_deg": round(math.degrees(positions[index]), 3),
                "tau_model_nm": round(tau_model[index], 4),
                "tau_protocol_nm": round(tau_protocol[index], 4),
                "d_term_nm": round(d_term, 4),
                "predicted_ff_nm": round(predicted_ff, 4),
                "guard_nm": guard,
                "guard_enabled": guard > 0,
                "p_term_nm": None,
                "applied_ff_nm": None if applied is None else round(applied[index], 4),
                "applied_envelope_nm": None,
                "applied_headroom_nm": None,
                "applied_headroom_deg_at_kp": None,
                "predicted_envelope_nm": None,
                "predicted_headroom_nm": None,
                "predicted_headroom_deg_at_kp": None,
            }
            if reference is not None:
                error = reference[index] - positions[index]
                p_term = kp * error
                row["p_term_nm"] = round(p_term, 4)
                if applied is not None:
                    applied_envelope = abs(p_term) + abs(d_term) + abs(applied[index])
                    applied_headroom = guard - applied_envelope
                    row["applied_envelope_nm"] = round(applied_envelope, 4)
                    if guard > 0:
                        row["applied_headroom_nm"] = round(applied_headroom, 4)
                        if kp > 0:
                            row["applied_headroom_deg_at_kp"] = round(math.degrees(max(0., applied_headroom) / kp), 3)
                predicted_envelope = abs(p_term) + abs(d_term) + abs(predicted_ff)
                predicted_headroom = guard - predicted_envelope
                row["predicted_envelope_nm"] = round(predicted_envelope, 4)
                if guard > 0:
                    row["predicted_headroom_nm"] = round(predicted_headroom, 4)
                    if kp > 0:
                        row["predicted_headroom_deg_at_kp"] = round(math.degrees(max(0., predicted_headroom) / kp), 3)
            rows.append(row)
        if applied is None:
            warnings.append("未收到驱动活动前馈：已应用余量不可计算")
        if self.model.parameterization == 'identified_base':
            warnings.append('实测重力基参数候选，独立验证覆盖本次采集姿态')
        elif self.model.mass_status != "CALIBRATED":
            warnings.append("质量档案为暂定模型（%s），不得当作已标定" % (self.model.mass_status or "unknown"))
        if self.model.fixed_joint_offsets:
            warnings.append("已应用固定关节偏置 %s（需实机多姿态核验，且只应用一次）"
                            % self.model.fixed_joint_offsets)
        return {"available": True, "mode": "shadow", "ff_applied": False,
                "alpha": self.alpha, "beta": 0.0, "shadow_alpha": self.shadow_alpha,
                "reference_known": reference is not None,
                "rows": rows, "warnings": warnings,
                "potential_energy_j": round(gravity["potential_energy_j"], 4),
                "reference_stale": reference is None,
                "metadata": dict(self.metadata, kp=kp, kd=kd)}
