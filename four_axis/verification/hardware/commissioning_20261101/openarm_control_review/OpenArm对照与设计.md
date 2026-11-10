# OpenArm 对照与设计

本文回答任务书 A：官方原生 MIT 与本项目主机 PD 的真实数据流差异、适合借鉴与不适合照搬的部分。所有官方代码只读，不运行、不移植、不声称已实机验证 OpenArm。

## 1. 参考与版本

| 项目 | 地址 | 本地快照 | commit |
|---|---|---|---|
| OpenArm ROS2 | https://github.com/enactic/openarm_ros2 | `../tmp/openarm_ros2_latest`（干净） | `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` |
| OpenArm CAN | https://github.com/enactic/openarm_can | `../tmp/openarm_can_latest`（干净） | `f340d4b808fb177e1f297af54eb55fd51c6c7c10` |

本任务目录 `official_reference/` 是只读副本：

- `openarm_simple_hardware.cpp`（ROS2 硬件接口 `write()`、`on_activate()`、`return_to_zero()`）
- `dm_motor_control.cpp`（MIT 参数打包/解包、限幅、量化）
- `openarm_bimanual_controllers.yaml`（`update_rate: 750`、`interpolation_method: "none"`）

## 2. 官方原生 MIT 数据流

1. ROS2 controller manager 以 `update_rate: 750` Hz 运行，`joint_trajectory_controller` 关闭插值（`interpolation_method: "none"`），由上游高频下发参考。
2. `OpenArmHW::write()`（`official_reference/openarm_simple_hardware.cpp:285-303`）把每个关节的 `{kp_[i], kd_[i], pos_commands_[i], vel_commands_[i], tau_commands_[i]}` 组成 `openarm::damiao_motor::MITParam`，调用 `openarm_->get_arm().mit_control_all(arm_params)`。
3. `DMDeviceCollection::mit_control_all()` → `mit_control_one()`（`../tmp/openarm_can_latest/src/openarm/damiao_motor/dm_motor_device_collection.cpp:116-131`）先检查 `ControlMode::MIT`，再调 `create_mit_control_command()`。
4. `CanPacketEncoder::pack_mit_control_data()`（`official_reference/dm_motor_control.cpp:136-156`）把 kp(0..500)、kd(0..5)、q(±pMax)、dq(±vMax)、tau(±tMax) 分别量化成 12/12/16/12/12 位，8 字节；`double_to_uint()`（同文件 226-231）先 `limit_min_max` 再量化，字段越界是**静默截断**，没有“总力矩上限”字段。
5. 驱动器收到帧后，在**驱动内部**按 `kp*(q_des−q)+kd*(dq_des−dq)+tau_ff` 计算总力矩，并受驱动器自己的 TMAX、OC 电流限制等内部限制。主机看到的 `tau` 只是前馈项；MIT 报文里没有“总输出限幅”。

## 3. 本项目主机 PD 数据流

1. `USB2CANEndpoint` 只发 ID1–4 的 MIT/FC/FD；`actuation_io.ControlSession` 约 100Hz 调用 `cycle()`，读 8 字节反馈并构 `Snapshot`。
2. `FourAxisController._torque_frame()` 在主机（模型空间）算
   `requested = kp*(target−q) + kd*(v_ref − sign*v_fb)`，
   再按所选档位的 `torque_cap` 选 12 位编码值；驱动 kp/kd 字节写 0、目标速度写 0，驱动只执行 `tau` 前馈。
3. 因此总力矩闭环在主机，**主机能对总编码力矩做限幅**；这是 1Nm/2Nm 候选能守住命令边界的实际原因。反馈力矩是驱动估计值，不是轴端测力。
4. 时钟/链路：Windows + USB2CAN，主机约 100Hz，反馈年龄上限 60ms，主机周期上限 40ms；没有 Linux/CAN-FD 的 750Hz 或实时保证。

## 4. 差异表

| 维度 | 官方 OpenArm | 本项目 |
|---|---|---|
| 闭环位置 | 驱动器内部 MIT | 主机 PD，驱动 kp/kd=0 |
| 总力矩限幅 | 驱动器内部 TMAX/OC；MIT 报文无总限幅字段 | 主机按档位对总编码命令限幅（≤0.30/1/2Nm） |
| 控制频率 | yaml 示例 750Hz | 约 100Hz，Windows/USB |
| 参考轨迹 | ROS2 控制器高频参考，插值 none | 主机最小跃度 10°/18s，每程18s，整次45s |
| 使能/零位 | `on_activate()` `enable_all()` + `return_to_zero()` | 不使能全部、不回零；J4 当前位置接管 |
| 轴数 | 7 轴/双臂 + 夹爪 | 仅 J4 可候选使能；J1–J3 只监视；J5 不查询 |
| 总线 | CAN-FD | USB2CAN 串口（921600） |
| 身份/参数 | URDF/硬件参数 | 运行时预检 MIT/TIMEOUT2000/状态/新鲜度 |

## 5. 适合借鉴

1. **MIT 字段布局与量化**：官方 `pack_mit_control_data()` 与本项目 `commissioning.encode_shadow()` 的位宽/范围一致（kp 12位 0..500、kd 12位 0..5、q 16位、dq 12位、tau 12位）。本轮把它作为只读对照，没有改变本项目编码。
2. **有界、显式的参考移动**：官方 `return_to_zero()`（`openarm_simple_hardware.cpp:305-353`）用 200 步 × 10ms 的线性插值从当前位到零，说明“位置参考要受时间和速率约束”。本项目借鉴的是**有界起动窗口 + 冻结票根**的思想，不复制回零动作，也不复制 200 步/10ms 的具体参数。
3. **字段级限幅 + 显式状态**：官方 `double_to_uint()` 的字段截断提醒我们：只看单个字段不能推断总输出；本项目保留主机总编码限幅并在日志里区分 request/encoded。
4. **激活前状态检查**：官方 `on_activate()` 先设置回调、使能、再读状态；本项目用预检 + `_check`（MIT/TIMEOUT/状态/新鲜度/温度）在**任何写帧前**拒绝，且不照搬 `enable_all`。

## 6. 不适合照搬

1. **驱动增益直接用于生产**：官方把 kp/kd 放进 MIT 字段，由驱动器闭环。若在本项目直接把 Kp=32/Kd=0.5 交给驱动，总输出由驱动内部 TMAX/OC 决定，主机无法在不改电机参数的前提下保证 2Nm 命令边界；这违反“总编码命令绝对值≤所选档”的保留项。因此本轮生产入口仍是主机 PD + 总编码限幅。
2. **750Hz/实时**：本项目的 Windows/USB2CAN 链路不能照抄 750Hz，也不能声称实时保证；反馈年龄/周期上限仍是 60ms/40ms。
3. **`enable_all`/`return_to_zero`**：会违反“仅 J4 使能、当前位置接管、不回零”的边界；本轮明确不复制。
4. **七轴 ROS2 移植**：本轮不做全臂/多轴联动/重力补偿；J5 仍不进入传输。
5. **把 URDF 当物理包络**：官方/本项目的模型限位都不是已验收物理包络；本轮仍用 `LIMITS` 只作模型边界，实际路径余量/支撑由用户确认。

## 7. 原生 MIT 的离线评估原型

`slai_console/j4_control.py::native_mit_estimate()` 只做离线估算，不进入生产：

```text
native_total = kp*(q_des−q) + kd*(dq_des−dq) + tau_ff
```

例：Kp=32、误差 4°（0.0698rad）、Kd=0.5、速度误差 0.2rad/s，则
`32*0.0698 + 0.5*0.2 = 2.334Nm`（不含 tau_ff 已超过 2Nm）。
主机即使把 `tau_ff` 限制在 2Nm 内，也只能约束这一项；P/D 项由驱动器内部算，MIT 帧没有总限幅字段。若不改驱动器的 TMAX/OC，主机无法证明总输出守住 2Nm。

因此：

- 原生 MIT 仅作离线对照/原型；没有写入生产路径，没有改驱动增益；
- 生产入口仍为“主机 PD + 总编码命令限幅”，并在 `TRIAL_STARTED`/`TRIAL_METRICS` 中明确记录 `drive_kp=0, drive_kd=0`、`candidate_host_kp/kd`、`encoded_cap_nm`；
- 若未来要用原生 MIT，需要先证明驱动器内部总力矩边界（不改参数）并单独验收，本轮不声称。

## 8. 仍未验证/剩余差异

- 驱动器内部 MIT 的真实总输出、内部 OC/TMAX 行为、MIT 状态解包时序：**未测量**。
- 750Hz 下的实时性、CAN-FD 与 USB2CAN 的时序差异：**未测量**；本项目不声称等价。
- 驱动反馈力矩是估计值，不是轴端力/力矩传感器；本轮所有“反馈力矩”读数都按估计值处理。
- 新档位的实机行为：**未验证**；用户尚未运行，等待 Codex 验收。
- 官方代码只读对照，未编译、未运行、未在 OpenArm 硬件上验证；不能称“已照抄/已实机验证 OpenArm”。
