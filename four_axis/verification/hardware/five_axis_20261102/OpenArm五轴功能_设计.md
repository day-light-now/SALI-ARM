# OpenArm 风格五轴功能：设计与边界（2026-11-02）

## 1. 参照的 OpenArm 控制方法

官方 `enactic/openarm_can` commit `f340d4b808fb177e1f297af54eb55fd51c6c7c10`，`openarm_simple_hardware.cpp`：

- `on_activate`：`enable_all()`，`recv_all()`，`return_to_zero()`；
- `read()`：逐轴读取位置/速度/力矩；
- `write()`：对每个臂轴调用 `mit_control_all({kp[i], kd[i], pos_cmd[i], vel_cmd[i], tau_cmd[i]})`，驱动内部执行 MIT；
- 控制器：forward position、forward velocity、joint trajectory（JTC 插值 none）；
- 官方 `update_rate: 750`，CAN-FD；每轴 Kp/Kd 来自硬件参数。

Codex 的 J4 原生适配（`native_mit_console.py` / `native_actuation.py` / `openarm_mit.py`）已验证：五字段 MIT 编码、截断量化、严格拒绝越界、坐标映射、J4 单轴、100Hz 主机调度、1s 增益斜坡、当前位置接管、2Nm“观测点估计”护栏（不是驱动内部硬限幅）、四轴 FD。**它不是五轴整臂验收。**

## 2. 本交付的“五轴功能”范围

目标是用户五轴臂（J1–J5）的功能等价，不是 OpenArm 7 轴/夹爪/MoveIt 的整机移植。

| 功能 | 本交付 |
|---|---|
| J1–J5 只读状态（位置/速度/力矩/温度/状态/参数） | 是，独立五轴只读监视器 |
| J5 手动标定门（方向/零位/安全行程） | 是；**未标定前 J5 只监视，绝不下发** |
| 五轴原生 MIT 当前位置保持 | 是，逐轴顺序使能 + 1s 增益斜坡 |
| 五轴多段关节轨迹 | 是，minjerk、逐轴 0.02rad/s、到点误差、超时 |
| 五轴速度模式 | 是，kp=0、kd 驱动内部速度环，显式 velocity_ack、限时 |
| 全轴故障停止 | 是，FD J1–J5（仅已激活轴），新反馈确认；UNKNOWN 锁定 |
| 逐轴行程/速度/误差/温度/反馈年龄/主机周期 | 是，配置化 |
| 驱动内部电流/力矩硬限幅 | **未验证**；主机护栏只是观测点估计，不是硬限幅 |
| 重力补偿 / 动力学前馈 | **未实现**；不声称 |
| ROS2/MoveIt/750Hz/CAN-FD/夹爪/自动回零 | **不在本交付**；不声称等价 |

## 3. J5 的硬阻塞（必须先解决）

`src/slaihand_bringup/config/hardware_mapping.yaml` 的 J5：

```yaml
virtual_placeholder: true
calibration_status: can_id_saved_power_cycle_verified_direction_zero_limits_pending
hardware_sign: null
zero_offset_rad: null
```

`J5_识别记录_20261030.md` 明确：方向、软件零位、机械限位、小幅运动、力矩输出、重力补偿均未验证。因此：

1. **不得猜 J5 的 sign/zero/limits**，也不得用 OpenArm 的关节 5 零位替代。
2. 本交付提供**手动、只读、不使能电机**的 J5 标定流程：用户手动把 J5 放到零点、正向、两个安全端点，软件只读编码器并记录；方向由两次读数差推断；行程由两端点加安全余量得到；用户逐项确认后才写 `j5_calibration.json`。
3. 没有有效 `j5_calibration.json` 时，五轴控制器拒绝激活 J5；J1–J4 可按四轴运行，J5 只显示原始角度/速度/状态。
4. J5 的 P/V/T 上限从当次设备寄存器读取，不写死。

## 4. 逐轴配置（操作员确认，不是自动调参）

配置模板：`five_axis_config.json`。每个轴：

- `travel_min_deg` / `travel_max_deg`：J1–J4 以 `core.LIMITS` 为外边界，配置文件给更保守的局部行程；J5 由标定记录给；
- `estimate_guard_nm`：当前反馈点的驱动请求估计中止阈值，**不是**连续力矩硬限幅；
- `kp` / `kd`：驱动内部 MIT 增益；默认与 Codex J4 原生一致 `32/2`，逐轴可改；
- `speed_rad_s`：目标速度上限，默认 0.02；
- `following_error_deg`：默认 4°；
- `feedback_speed_rad_s`：J1–J3/J5 模板 0.2；J4 用已测 1.0；
- `temperature_c`：默认 50；
- `arrival_error_deg`：默认 0.5。

`user_confirmed: false` 的模板**不能使能**；操作员必须逐轴复核（尤其 J5 行程和每轴估计护栏）并改为 `true`。默认值刻意保守，可能不足以支撑整臂重力；若估计护栏触发，软件停止并报告，不自动升增益/升护栏。

## 5. 控制与安全

- 主机 100Hz（`period=10ms`），反馈年龄 ≤60ms，主机间隔 >40ms 故障；不是 750Hz。
- 先只读预检：五轴模式 1、TIMEOUT2000、状态 0、参数齐全、映射/标定一致；任何一项不符拒绝接管。
- 当前位置接管；**不自动回零/不写零位/不写参数/不写 Flash**。
- 逐轴顺序使能：预置 MIT（kp=kd=0）→ FC → 状态 1 → 1s 增益斜坡；任一轴失败/超时 → 全轴 FD。
- 保持/轨迹/速度都按逐轴包络；目标速度编码后仍 ≤0.02rad/s。
- 故障（反馈速度 >0.2、跟随误差 >4°、温度 >50、反馈过期、主机间隔、状态异常、请求估计 >guard、反馈力矩 >2×guard）→ 取消计划、FD 全部已激活轴、等待新反馈 state0；超时 UNKNOWN，不自动重连/重试。
- 停止后只读观察沿用 Codex 的可见帧/CC 只在 DISABLED 后/UNKNOWN 不查询策略；五轴版本对 J1–J5 做同样的可见性标注。
- 不发送 MIT 到未激活轴；不查询参数以外的寄存器；不查询 J6/J7/夹爪。

## 6. 明确不做

- 不使用官方 `enable_all`/`return_to_zero` 直接照搬；
- 不声称 2Nm/guard 是驱动内部硬限幅；
- 不声称已做重力补偿、负载保持、整臂同步或 OpenArm 整机等价；
- 不做 ROS2/MoveIt/夹爪/750Hz/CAN-FD；
- 不在本轮操作实机、串口、电机参数或零位。

## 7. 验收路径

1. 操作员复核 `five_axis_config.json` 并确认；
2. 用五轴只读监视器手动标定 J5（不使能），生成 `j5_calibration.json`；
3. Codex 离线验收五轴核心/监视器/标定测试；
4. 实机先单轴、低目标速度、支撑状态，逐轴确认；再五轴保持；最后小角度多轴轨迹。任何一步失败停止，不连续重试。

## 8. 实现状态（2026-11-02）

已完成离线实现：`five_axis_monitor.py`、`j5_calibration.py`、`five_axis_config.py`、`five_axis_mit.py`、`five_axis_session.py`、`five_axis_console.py`；新增 34 项测试；全量 179 项离线通过。

未完成/未验证：J5 尚未手动标定（mapping 仍为 null）；`five_axis_config.json` 的 `user_confirmed` 仍为 false；未操作实机；未验证驱动侧连续力矩/电流硬限幅；未实现重力补偿/ROS2/MoveIt/750Hz。详细边界见 `OpenArm五轴功能_交付_20261102.md`。
