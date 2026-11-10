# J5/五轴重力辨识静态激励轨迹（candidate）

这 15 组是**离线静态姿态候选轨迹**，用于采集保持力矩，供后续重力/负载模型辨识使用。
它们不是重力前馈补偿本身，也不启用 `gravity_ff_alpha`/float。

- 5 组 `GRAV_*` 是早期 J1–J4 组：J5 固定相对 0°。
- 4 组 `J5_GRAV_*` 是 J5 已有限位/TIMEOUT 后的 J5 参与组，J5 在 ±10°/±15° 内双向
  变化，J1–J4 只做小幅相对偏移。
- 当前真实 J5 限位 ±90°、TIMEOUT2000 已实测；但新负载只有 3.5 kg 估计，几何/COM
  未知，所以这些轨迹只用于静态采样，不能据此做正式重力拟合或 FF。
- 执行前必须确认支撑/净空/切电；标准 arm 会同时使能 J1–J5。
- 预览会按当前起点检查完整限位，超出即拒绝，不裁剪。

## 轨迹

| 文件 | 主要激励 | J5 |
|---|---|---|
| `GRAV_A_J1_J2.json` | J1/J2 两个方向 | 固定 0° |
| `GRAV_B_J2_J3.json` | J2/J3 组合 | 固定 0° |
| `GRAV_C_J3_J4.json` | J3/J4 交叉 | 固定 0° |
| `GRAV_D_J1_J4.json` | J1/J4 大幅组合 | 固定 0° |
| `GRAV_E_MIXED.json` | 四轴混合姿态 | 固定 0° |
| `J5_GRAV_A.json` | J5 +10°，含 J1 +5° | +10° |
| `J5_GRAV_B.json` | J5 −10° ↔ +10° 往返 | ±10° |
| `J5_GRAV_C.json` | J5 +15°，J3/J4 交叉 | ±15° |
| `J5_GRAV_D.json` | J5 −15°，J3/J4 反向交叉 | ±15° |
| `WIDE_J5_60.json` | J5 ±60° 宽幅 | ±60° |
| `WIDE_J5_80.json` | J5 ±80° 宽幅 | ±80° |
| `WIDE_J1_J2_J5.json` | J1/J2 大幅 + J5 ±45° | ±45° |
| `WIDE_J3_J4_J5.json` | J3 ±60°/J4 −30~60° + J5 ±60° | ±60° |
| `WIDE_MIXED_GRID.json` | J1–J5 多轴宽幅网格 | ±60° |
| `WIDE_J5_COMBINED.json` | J5 ±75° + J1–J4 组合 | ±75° |

格式：`slaihand_joint_plan/2`，`relative_start_deg`，从当前姿态偏移；每个静态点停留
6 s，移动 2–2.5 s（首点 0.5 s），参考速度低于 0.5 rad/s。

## 面板下拉

6 组 `WIDE_*` 已加入“规划与执行”面板的组下拉，位于：

```text
五轴宽幅位置轨迹（普通轨迹模式，非正式重力采样）
```

选择后点击“载入”会把计划作为**普通轨迹**载入，并自动切到“轨迹跟随”模式；
不会进入 `CALIBRATION_PREFLIGHT` 正式重力采样门禁。第一行 `0` 是相对当前起点，
不是绝对零度。不要再把 `WIDE_*` 放进“重力采样（α=0）”模式。

## 使用

1. 先确认当前为 disarmed、STOP 可用、支撑/净空；
2. 在“规划与执行”选择 WIDE 组载入，或直接导入 JSON，先 preview；
3. 一次只执行一组，观察反馈与保持力矩；
4. 不要把这些组当作正式重力模型/FF 的证据。

运行离线校验：

```bash
python3 -m unittest verification.hardware.five_axis_20261107.test_gravity_trajectory_library
```

该测试只做离线 shape/限位/时间校验；不启动 ROS、不执行运动。
