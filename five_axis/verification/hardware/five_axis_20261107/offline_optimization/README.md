# 五轴离线控制研究（2026-11-09 修订版）

## 0. 证据边界

本轮只有离线计算，**没有启动 ROS、driver、panel，没有打开串口/CAN，没有使能或运动**。
本报告不构成上机准入、动态稳定性结论、1.5× 裕量或精度验收。

数据定义来自 `joint_waypoint_plan.py:assess_waypoints`：

```text
max_error_deg = max(abs(actual - target)) over the final ~0.3 s of a dwell
```

因此它是**无符号的窗口峰值**。原始结果里没有 target-actual 的有符号时序、
完整 actual 五轴姿态、速度、时间戳和反馈有效性，所以：

- 不能据此生成有符号重力前馈；
- 不能把 `Kp * max_abs_error` 当作真实重力矩；
- 不能用单条同步路径的拟合残差证明多关节姿态泛化。

## 1. 数据来源与可比性

| 文件 | 用途 |
|---|---|
| `results/BASELINE_0_40_160_5.json` | 真实 160/5 baseline |
| `results/GAIN_0_40_280_240_160_200_160.json` | 真实 J1/J2/J4=280/240/200，J3/J5=160 |

两次运行的**绝对起点不同**，只能按相对目标（+5/+10/+20/+30/+35/+40°）和
同样的 `max(abs(...))` 判据比较；不能直接比较绝对编码器位置。

## 2. 常值增益：实测 baseline vs 实测 high

真实测量（不是 1/Kp 预测）：

| 40° | J1 | J2 | J4 |
|---|---:|---:|---:|
| baseline 160/5 | 3.564° | 3.805° | 2.843° |
| high measured | 2.013° | 2.275° | 2.100° |
| 1/Kp prediction | 2.037° | 2.537° | 2.275° |
| measured improvement | 43.5% | 40.2% | 26.1% |

J1/J2 的实测改善约为 20–44%；**J4 在 20–40° 只有约 10–26%**，不能写成
“J1/J2/J4 全部 30–45%”。高增益仍不满足 0.5°。
`constant_high` 现在实际读取了 `GAIN_0_40_...json`，并同时输出
measured baseline / measured high / predicted high / improvement。

## 3. 角度调度 Kp(q)

当前矩阵参数是显式统一的：

```text
J1/J2/J4: Kp = 280 at 0° -> 500 at 30° -> 500
J3/J5:    Kp = 160
```

在“无符号峰值、准静态、1/Kp”条件下，40° 预测仍是：

```text
J1 ~1.14°, J2 ~1.22°, J4 ~0.91°
```

这**不是**“Kp=500 实机一定不达标”的证明，也不是动态稳定性证明，只是该
静态模型的条件下预测。并且当前没有实现 C++ 的 Kp(q) 平滑插值、有效域、
身份/审计和限流，因此**不是 hardware-ready**。

## 4. 方案分类（不做性能排名）

| 方案 | 类别 | 指标 | 可下发 |
|---|---|---|---|
| `constant_160_5` | MEASURED | 有 | 现有常值 MIT 路径 |
| `constant_high` | MEASURED_COMPARISON | 有 | 实验常值 MIT 路径 |
| `angle_schedule_kp` | STATIC_PREDICTION | 有符号缺失，无 | 否 |
| `gravity_feedforward` | BLOCKED | **null** | 否 |
| `integral_trim` | IDEALIZED / 未评价 | **null** | 否 |
| `disturbance_observer` | BLOCKED | **null** | 否 |
| `model_based_gravity` | BLOCKED | **null** | 否 |

`gravity_feedforward` 和旧的 `hybrid` 不再作为两个“不同控制器”并列排名：
在相同拟合和相同 Kp=160 下它们本来就会重复；`hybrid` 已从矩阵移除，待有
有符号数据、不同真实闭环配置和动态数据后再评价。

`disturbance_observer` 不再用硬编码 0 参与排名；`model_gravity` 不再用旧
四轴模型外推；`integral` 只保留“理想静态模型”说明，指标为 null。

## 5. 有符号静止窗口与独立姿态

目前原始数据**不支持**有符号静止窗口分析。若后续获准采集，必须保存：

- 每周期 target、actual 五轴完整位置、速度、时间戳、反馈有效性；
- 电机坐标/模型坐标、`hardware_sign`、零偏、减速比、Nm 约定；
- 筛选真正静止、无故障、无饱和、反馈新鲜的窗口；
- 分别报告 signed mean/median、波动、max_abs。

拟合/验证必须按**整次运行或整组独立姿态**拆分；不能随机拆相邻采样点。
计划见 `NEXT_SAMPLING_PLAN_20261109.md`。采集至少应包含：

- 其他轴固定的单轴变化；
- 少量混合姿态；
- 正/反向接近同一姿态；
- 重复保持与跨次运行验证；
- 当前路径附近的局部拟合，并明确拒绝外推。

## 6. 静态 vs 动态

本研究的 `1/Kp` 模型没有惯量、时延、柔性、电流饱和、摩擦、噪声和温升。
因此：

- 不能由静态误差预测动态稳定性；
- 不能由 Kp=500 的静态预测决定实机安全；
- Kp/Kd 必须由实测运动峰值误差、过冲、速度/振荡、保持末尾 signed bias/波动、
  温升和停止结果选择。

FF 若未来实现，必须带 Nm 限幅、变化率限制、平滑启停、身份/有效域绑定，
并在反馈无效、STOP、失能、故障时清理；先做软件/PTY 验证。

## 7. 准入与新鲜度

- 本离线仓库不启动 driver；即使后续启动，也必须使用**默认 1800 s** 的
  artifact/evidence 新鲜度，**不得**沿用 86400 s；
- 掉电/重启后必须重新读取 J5 TIMEOUT 和当前证据，不能沿用旧 RAM 状态；
- `existing_command_path` 只表示现有 MIT 接口可下发，不等于 `commissioned`
  或 `acceptance_passed`；
- 1.5× 裕量仍需要每轴真实允许连续/峰值输出、持续时间、温度/供电条件与
  需求证据；协议 TMAX 和 `Kp*error` 不是裕量证明。

## 8. 运行与测试

```bash
cd /mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain

python3 verification/hardware/five_axis_20261107/offline_optimization/five_axis_control_lab.py
python3 verification/hardware/five_axis_20261107/offline_optimization/run_schemes.py --scheme all

python3 -m unittest \
  verification.hardware.five_axis_20261107.offline_optimization.test_control_candidates \
  verification.hardware.five_axis_20261107.offline_optimization.test_scheme_matrix
```

产物：

- `offline_control_study.json`: unsigned/path-local study;
- `scheme_results.json`: categories, measured comparison, null blocked metrics;
- `test_scheme_matrix.py`: 数据完整性、实测文件被读取、J4改善区间、BLOCKED
  null、无符号 FF 不可下发、调度参数边界、空数据/NaN/缺轴拒绝。
