# 接入核查说明（integration_notes）— 四轴重力补偿

- 依据：Codex《审阅_重力补偿》第 2 节要求；本文件只列**实际路径、函数、行号与证据**，不把来信数字当作已复验结论。
- 基线：仓库 `D:/CodexData/Projects/机械臂/slaihand_ros2_ws`，git HEAD `81c9449`，本轮前脏工作区 46 项；未覆盖无关改动。
- 环境：WSL Ubuntu 24.04，ROS 2 Jazzy，Python 3.12.3，GCC 13.3；numpy 1.26.4 / scipy 1.11.4；**无 pinocchio / mujoco**（未新增软件）。实机链路：`/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00`（WSL USB CDC），当前工作空间 `/home/xuyu/slaihand_speed05_ws_20261104`。

## 1. 控制主链路（唯一写权限）

```text
面板 control_panel.py
  └─(固定 argv 子进程) commission_supported_trial.py
       └─ FollowJointTrajectory action → controller_manager(100 Hz)
            └─ slaihand_joint_trajectory_controller（position 接口）
                 └─ CanUsbSystem::write()  ← src/slaihand_hardware/src/canusb_system.cpp:221
                      └─ MotionSupervisor::tick() 生成 Batch   (motion_supervisor.cpp:118)
                      └─ CanUsbSystem::send()    (canusb_system.cpp:108)
                           └─ ReadOnlyPort::send_motor()  ← src/slaihand_transport/src/canusb_readonly.cpp
                                └─ /dev/serial/by-id/... → DM-J10010L/J8009P ×4
```

- 唯一硬件写入口：`CanUsbSystem::send()`（`send_motor` / `send_read` 二选一），由 `write()` 单周期驱动；其他进程只读。
- 面板自身不打开串口/CAN（`control_panel.py` 只订阅话题、调 stop/reset/switch 服务、起受保护客户端子进程）。
- `controller_manager` 更新率 100 Hz（`canusb_motion_controllers.yaml:3`）；WSL 下 RT/FIFO 不可用（日志 `Could not enable FIFO RT scheduling policy`），即普通 10 ms 周期线程。

## 2. MIT 报文（`native_command`）

- 定义：`src/slaihand_hardware/include/slaihand_hardware/native_codec.hpp:35`：
  `native_command(axis, map, q, kp, kd)` → `create_mit_control_command(motor, {kp, kd, raw, 0., 0.})`（第 43 行）。
- `MITParam` 字段顺序：**`{kp, kd, q, dq, tau}`**（`src/openarm_can/include/openarm/damiao_motor/dm_motor_control.hpp:53`）；编码器 `create_mit_control_command` 同文件 :81。
- 单位：`q` rad（电机侧原始角）、`dq` rad/s、`kp` 0–500、`kd` 0–5、`tau` N·m（电机侧，按电机 `tMax` 缩放）；`raw = zero + sign*q_model`（native_codec.hpp:38）。
- 当前实参：`q=raw`、`dq=0`、`tau=0`。**力矩字段存在于协议与编码器中，但恒为零；这是本轮要接的前馈入口。**
- 缩放（`dm_motor_constants.hpp` `MOTOR_LIMIT_PARAMS`）：DM10010L（J1/J2）`pMax 12.5 / vMax 25 / tMax 200 N·m`；DM8009（J3/J4）`pMax 12.5 / vMax 45 / tMax 54 N·m`。力矩量化步长 = `2·tMax/4095` ≈ **0.0977 N·m（J1/J2）**、**0.0264 N·m（J3/J4）**。

## 3. 速度目标 `v_des`

- 当前恒为 0（`native_command` 第 43 行传入 `dq=0.`），驱动器内部 D 项因此是 `kd·(0 − v_measured) = −kd·v`。
- 轨迹速度只进入 JTC 位置插值，不进入 MIT 报文；`v_des` 不能从现有链路直接获得（若要，需要显式传入，属本轮命令接口设计内容）。

## 4. 力矩反馈 `effort_estimate`

- 来源：**DM 驱动器反馈帧的 torque 字段**，不是由 `kp·e+kd·e_v+ff` 重构。
  `src/slaihand_transport/src/canusb_readonly.cpp:115`：
  `torque = (((p[4]&15)<<8)|p[5]) * (2*tmax)/4095 − tmax`；:116–117 组 `Feedback{sign*(position−zero), sign*velocity, sign*torque, driver_temperature, motor_temperature, motor_state(p[0]>>4), received_at}`。
- 符号：已乘 `mapping.sign`，为模型侧；`CanUsbSystem::publish()` 把它放入 state interface `effort`（`canusb_system.cpp:139`，`v[2]=f.effort_estimate`）。
- 它是驱动内部由电流推算的力矩估计（轴端、未乘外部传感器），**是独立于命令的反馈量，可用于摩擦残差拟合**；但不是外部力矩传感器，精度/温漂未知。

## 5. 坐标映射

- 映射：`q_model = hardware_sign·(raw − zero_offset)`，`v_model = sign·v_motor`，`τ_model = sign·τ_motor`（虚功一致，`A=diag(sign)`，无减速比项）。
  - 符号/零点来自 `src/slaihand_bringup/config/hardware_mapping.yaml`：J1/J2/J3 `hardware_sign: 1`，**J4 `hardware_sign: -1`**；零点分别为 1.4574273289082171 / −1.795033188372626 / −2.5423437857633324 / −0.40951400015259054。
  - 生效位置：`canusb_system.cpp:52–54` 读入 `mappings_[i]={can_id, sign, zero}` → `native_command` 编码、`ReadonlyState::feed` 解码（两处都只应用一次）。
- 关节顺序：URDF/参数文件按 `slaihand_joint1..4`，`on_init` 强制按名字校验（`canusb_system.cpp:49`），不依赖文件顺序。
- **J3 的 `+π`**：只存在于**现有 MuJoCo 仿真模型**（`verification/impedance_demo/model.py:24` `J3_PLACEMENT_OFFSET_RAD=np.pi`，注释“fixed placement offset, not qpos target or hardware encoder offset”）。URDF 的 J3 origin rpy 本身已含 ≈π（`slaihand_four_axis.urdf` J3：`rpy="3.14023651841 −1.57079632679 0"`）。**结论：这是模型放置修正，不在反馈映射/编码器零点中；正式重力模型必须显式决定是否采用，且只能加一次。** 待实机多姿态核验；不许重复加。
- 单位：算法内部 rad、rad/s、N·m；界面显示角度时转换。

## 6. 现有门禁与停止行为

- 请求估计门禁（`motion_supervisor.cpp:83–87`）：
  ```text
  d = decode_native(frame)                              # 实际编码后的 kp/kd/q/dq/tau
  motor_q = zero + sign·position,  motor_v = sign·velocity
  estimate = d.kp·(d.q − motor_q) + d.kd·(d.dq − motor_v) + d.tau
  if |estimate| > policy_.guard[i-1] → OBSERVED_STATE_REQUEST_GUARD_NOT_HARD_CLAMP（全部停止）
  ```
  现状 `d.dq=0`、`d.tau=0`，即 `estimate = kp·(q_des−q) − kd·v`；**它是编码后命令的合成估计（不是绝对值包络）**。加入前馈后 `d.tau = τ_ff_encoded`，量化误差按第 2 节。
- 反馈门禁（`check_observations`，`motion_supervisor.cpp:40–59`）：温度 50°C、`|velocity|>feedback_speed(1.0)`、位置出 URDF 限位、跟随误差 4°、`|effort_estimate|>guard[i]`（`FEEDBACK_ESTIMATE_GUARD_NOT_HARD_CLAMP`）、寄存器 ID/超时 2000 校验、状态年龄。
- 人为阈值来源：`canusb_motion_policy.yaml`（kp 160、kd 5、reference 0.5、feedback 1.0、following 4°、temperature 50、ramp 1、stagger 0.03、age 0.06、gap 0.05、session 45、guard [8,10,2,2]）；C++ 侧另有上限（`motion_supervisor.cpp:10–14`：kp≤160、kd≤5、reference≤0.5、feedback≤1、error≤4°、age≤0.06、gap≤0.05、runtime≤180、guard maxima {8,10,2,2}）。**两处都要改才能放宽 guard；本轮按 Codex 授权可改，但必须逐项记录旧值/新值/依据/回滚。**
- 停止：`stop_all` 服务置 `stop_request_`（`canusb_system.cpp:78–80`）→ `write()` 中 `supervisor_->stop("OPERATOR_GLOBAL_STOP", ..., true)`（:224）→ STOPPING 发 4×FD + 4×READ，0.3 秒内四轴 `motor_state==0` 且新反馈两次则 DISABLED+confirmed，否则 **UNKNOWN 锁定**（`motion_supervisor.cpp:92–108`，UNKNOWN 需现场处理/重启进程）。`hold_ready` = 速度≤0.05 持续 0.5 秒（:147–150）。`reset_disabled` 只允许 DISABLED→DISARMED 且四轴 `motor_state==0`。
- 所有权：JTC 激活时 `owned_=true`，只有 HOLD+hold_ready 才能切换（`prepare/perform_command_mode_switch`，`canusb_system.cpp:204–214`）。

## 7. 本轮对来信 Q3 的修正（基于实际公式）

- 来信 `Kd·v=1.25 N·m` 的算术成立，但含义是：`v_des=0` → 驱动内部 D 项确实输出 `−kd·v`，**是真实命令力矩**，不是仅保守包络；门禁 `estimate` 与它同源（编码后命令）。
- 重算 J3/J4 余量（guard 2 N·m）：
  - 静止、重力前馈最坏 0.39 N·m → 剩 1.61 N·m（≈0.58° 位置误差 @kp160）；
  - 0.25 rad/s 时 `kd·v=1.25 N·m`，加前馈 0.39 → 剩 0.36 N·m（≈0.13°）；
  - 即：**0.25 rad/s 的 J3/J4 扫描与“重力前馈 + 正常跟踪误差”叠加时，现有 2 N·m 门禁确实很紧**；是否调整门禁/速度由后续实测与 Codex 决定，本轮不先预设新阈值。
- 门禁上限来自 C++ `maxima{8,10,2,2}` 与 YAML/URDF 两处；修改需同步并记录。

## 8. 模型与质量现状

- URDF **不含惯性/质量**（5 个 link 只有 visual；`grader` 注释 “No collision or inertial data is included”）。
- 质量/重心来自 `verification/gravity_preparation/provisional_gravity.json`（`status: ESTIMATE_ONLY_NOT_FOR_HARDWARE`）：base_link 1.7819、link1 1.7417、link2 0.2279、link3 2.1051、link4 0.4858 kg，合计 6.3424 kg；来源为 CAD + 电机官方整机质量缩放，转子/定子、轴承、打印填充、末端负载未实测（`质量材料审计.md`）。
- 基础系重力：现有默认 `+Y` 向下 9.80665 m/s²（来自肩部安装截图，**现场未确认**）；正式实现改为显式配置 `R_world_base` 与 `g_base`。
- J3 放置 +π：见第 5 节，MuJoCo 模型采用，URDF 未含；正式重力模型必须显式选择并证明只加一次。

## 9. 下一阶段实施（按 Codex 顺序）

- **A（进行中）**：坐标转换模块 + 重力模型生产化（可脱离 ROS 调用，`load_model/compute_gravity`，模型元数据/哈希）+ shadow 模式 + 面板显示 + 单摆解析/势能差分/符号测试；开关关闭时命令与基线逐位一致（mock）。
- **B**：驱动最小补丁——`native_command` 增加 `tau` 参数并在 `controls()` 填 `sign·τ_model`（仅 `canusb_system.*`/`native_codec.*`/`motion_supervisor.*` 必要字段）；PTY/mock 解码验证非零前馈、v_des、增益；前馈斜坡/限幅/拒绝原因；默认关闭。
- **C**：质量/重心修正与多点姿态重力核验（用户实机，受支撑、小比例、多姿态）。
- **D**：有界单轴恒速扫描客户端（0.05/0.12/0.25，先 J1/J2）+ 摩擦拟合 + 第二阶段补偿。

## 9.1 测试环境与运行库一致性（2026-11-05 实测）

- 本机存在多个工作空间（`slaihand_canusb_motion_ws_20261104_kpMgtx`、`slaihand_session180_ws_20261104`、`slaihand_ros2_ws`、`slaihand_speed05_ws_20261104`、候选空间），各自安装过 `libslaihand_hardware.so`。pluginlib/class_loader 按 `LD_LIBRARY_PATH` 搜索库名，实测隔离域 179 的启动加载了 `slaihand_speed05_ws_20261104/install/slaihand_hardware/lib/libslaihand_hardware.so`，而其内容为旧版（`Motion policy widens B1 scope`，0.5 策略被拒）。
- 因此候选测试空间改为自包含构建四个项目包（bringup/description/transport/hardware），`test_env.sh` 清除继承的 `AMENT_PREFIX_PATH`/`LD_LIBRARY_PATH`/`PYTHONPATH` 等后只 source ROS + 候选；E2E 119 项、浏览器 37 项恢复通过。
- 对实机的建议：启动驱动时固定单一工作空间，并用 `/proc/<pid>/maps` 核对该进程实际加载的 `libslaihand_hardware.so` 路径；不同工作空间的同一包不可混用。

## 9.2 B 阶段驱动实现（2026-11-05）

- `native_codec.hpp`：`native_command(axis, map, q, kp, kd, tau=0)`；校验有限且 `|tau| ≤ tMax`；MIT 字段 `{kp,kd,raw,0,tau}`（`dq` 仍为 0）。
- `gravity_ff.hpp/.cpp`：四轴重力模型（G/L/J 紧凑文本），纯 C++17；测试 `test/gravity_ff_test.cpp` 覆盖解析、单摆解析解、势能差分、固定偏置只应用一次、错误拒绝。
- `motion_supervisor`：`MotionPolicy.gravity_alpha∈[0,1]`、`gravity_ramp≥0`；`controls()` 用 `ReadOnlyState` 的 **模型系** 反馈角（勿二次映射，实测踩坑）计算 g(q)；`tau_ff_req=ramp_scale·alpha·g`；`headroom=guard−|PD|` 饱和；编码后仍走原 `estimate` 门禁；`ff_requested/ff_applied` 只读暴露。
- `canusb_system`：硬件参数可选 `gravity_ff_alpha`/`gravity_ff_ramp_s`/`gravity_model`；无模型或 α=0 时与原路径等价；`/slaihand/gravity_ff_state` 发布 `[alpha, ramp, req×4, applied×4, stage, command_id]`。
- launch/xacro：`build_gravity_model_text()` 从 URDF + 质量档案 + policy 生成模型文本；launch 参数 `gravity_ff_alpha` 可覆盖 policy 值（供隔离测试）。
- **协议量化**：12 位对称力矩字段无精确零码，0 编码为 2048、解码为 +半步（J1/J2 0.0488 N·m、J3/J4 0.0132 N·m）；E2E 以“≤半步”判定 α=0 情形。
- 验证入口：`run_verification.sh` 依次运行 C++ 模型单测、C++↔Python 交叉核对、生产回调、端到端、驱动前馈 PTY 场景；最近一次结果 `PASS (callbacks=0 e2e=0 model=0 cross=0 ff=0)`。

## 9.3 B 阶段返工修正（2026-11-05）

- 前馈重编码使用边界修正后的 `command_target`；最终报文二次校验位置与完整门禁。
- 独立前馈斜坡 `gravity_ff_ramp`：从使能确认起算，0 表示立即；`gain_ramp_s` 行为不变。
- 前馈状态四层：requested / candidate / encoded / applied；`applied` 只按 MIT 帧 written 回执更新，`stop/reset/arm` 清零；`gravity_active()` 受阶段约束；话题 21 值（含 last_written）。
- **量化方向修正**：`double_to_uint` 向 `-tMax` 截断，`tau=0` 解码为 -0.04884 N·m（J1/J2）/-0.01319（J3/J4），不是此前写的“+半步”。前馈发送前加半码（tMax/4095）以取最近可表示值；α=0 时字节不变。
- 面板字段：驱动前馈活动/α/斜坡、已写入值、历史最后写入、状态时效；失能后不再显示活动非零前馈。

## 9.4 A 遗留与 C 阶段工具（2026-11-05）

- shadow：参考未知时 P 项/已应用余量/预测余量均为未知；“已应用余量”用驱动实际写入前馈，“假设补偿预测”用 α_pred=1 的模型值，两者分列显示。
- J3 偏置：`j3_offset_proof.py` 证明 URDF origin 已含 π、MuJoCo link3 只多一次 π、部署 URDF 无额外 π、2π 等价无偏置；现场判别姿态为 J4=±30°（J2 预测反号），物理结论待实测。
- 标定：`calibration_core.py`/`calibration_capture.py` 只读采集与导入预览；原始数据只读；报告含 provenance、留出与可辨识性；状态只到 `PROVISIONAL_SCALE`/`INSUFFICIENT`。

## 9.5 C 阶段采集门槛（2026-11-05 返工）

- 拟合资格：使能保持（stage HOLD/TRACK、motor_state=1）+ 自由无接触 + 稳定持续窗口 + 前馈 α=0 + 运行中配置身份已验证；失能样本仅几何检查。
- 新鲜度：同一 DynamicJointState 帧、总年龄 = max(本地接收, 驱动 age) ≤0.06 s；缓存/重复回执不计入窗口。
- 版本：逐样本与当前分析模型核对哈希/重力/J3/负载；混合版本拒绝；运行时身份 UNKNOWN 的批次只作诊断。
- 证据：raw 独占唯一命名、写后只读；报告与 raw 路径必须不同（含符号链接）。

## 9.6 运行身份接口与采集边界（2026-11-05）

- `/slaihand/runtime_identity`（std_msgs/String, JSON, RELIABLE+TRANSIENT_LOCAL，2 s 周期重发同 boot_id）由驱动从实际接受的硬件参数生成：mapping/model/policy/mass-profile 哈希、policy_profile、load_id、gravity_base、J3 偏置、kp/kd、alpha/ramp。
- 采集工具仅在该消息新鲜（≤3 s）且与驱动值核对通过时记 verified；过期/缺失/不匹配一律 UNKNOWN → 该批只作诊断，不进入可信拟合。
- 年龄：每轴 driver_age + local_receipt_age，任一 >0.06 s 拒绝；窗口逐帧检查使能/新鲜/前馈 α=0/命令 ID，命令变化丢弃窗口。

## 10. 仍需用户/硬件侧提供

1. 实际安装方向/肩部姿态确认（决定 `g_base`），以及是否采用 J3 +π 的实物核验。
2. 称重与重心可信数据（至少总质量、可拆件、末端工具）；线缆/负载归属。
3. `effort_estimate` 的物理标定依据（驱动手册：反馈 torque 是电流推算还是实测轴端），用于摩擦拟合的置信度。
4. 各电机连续/短时力矩能力与制动/支撑条件（决定新门禁上限，不凭旧值推断）。
