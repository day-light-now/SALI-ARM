# 重力补偿“锁定当前位置 / 位置保持”实现与验证

日期：2026-11-07  
目标仓库：`D:/CodexData/Projects/机械臂/slaihand_ros2_ws`（主工作树 HEAD `a954f8f`）  
独立工作树：`D:/CodexData/Projects/机械臂/slaihand_gravity_hold_ws`  
分支：`feature/gravity-hold-20261107`  
实现提交：**`6f6bbbc`** `feat(gravity): add in-session lock-current-pose and resume-float`  
验证/证据提交：**`897e0c6`** `test(gravity): add PTY, backend and browser evidence for gravity hold`

## 1. 实际 Ponytail 副本与复用

完整读取的实际副本：

- `/mnt/d/CodexData/Projects/机械臂/slaihand_ros2_ws/tmp/ponytail_skill_source/skills/ponytail/SKILL.md`（52 行）
- `/mnt/d/CodexData/Projects/机械臂/slaihand_ros2_ws/tmp/ponytail_skill_source/AGENTS.md`（32 行）

按 **full** 级别执行：先读 touched code/callers/tests；优先复用；只做本任务需要的模式切换，不新增控制进程、不新增底层 MIT 通道、不新增部署参数。

复用的既有实现：

- `control_panel.py`：单一操作所有者、`_stop_generation` 停止纪元、心跳租约、`_operation` 生命周期、原有 `/api/gravity-float` 自由拖动会话、`_drain_queues()` 单 ROS 线程服务调用模式。
- `commission_supported_trial.py`：自由拖动会话仍由既有保护客户端启动和计时；锁定/恢复只切换该会话内的驱动模式，不重新 arm、不另起进程。
- `motion_supervisor`：位置保持（HOLD）、`Kp/Kd` 增益斜坡、`hold_ready`、URDF 限位/反馈速度/跟随误差检查、重力前馈模型、最终报文门禁、STOP 优先和 UNKNOWN/DISABLED 处理。
- `gravity_ff_state`：扩展尾部状态，而不是新起控制状态源。
- 现有 UI 的重力补偿区域、轮询、心跳、`/api/stop` 和确认组件。

## 2. 原仓库 / 部署候选 / 源代码核对

主工作树基底：`a954f8f`。部署候选：

`/home/xuyu/slaihand_control_panel_ws_gravitycal_deploy_20261106`

对以下源文件做 SHA256 对比（主仓库 vs 部署候选）：

| 文件 | 结果 |
|---|---|
| `control_panel.py` | SAME |
| `control_panel_http.py` | SAME |
| `commission_supported_trial.py` | SAME |
| `motion_supervisor.cpp` | SAME |
| `canusb_system.cpp` | SAME |
| `motion_supervisor.hpp` | SAME |
| `canusb_system.hpp` | SAME |
| `canusb_motion_policy.yaml` | **DIFF**：部署候选为 `gravity_ff_alpha: 0.1`、`gravity_ff_ramp_s: 2.0`，并含 `gravity_profile: gravity_identified_candidate.json`、`gravity_float_enabled: true`；主仓库仍为 `alpha: 0.0`、`ramp: 1.0`、未启用 float |

结论：本功能在当前部署候选的“已启用自由拖动”policy 下可直接使用；主仓库默认 policy 未开启 float，因此源码默认不显示可用的自由拖动/锁定流程。未修改主仓库或部署候选的 policy，未改动任何现有部署参数。

主工作树原有三项 untracked 历史文件保持不动：

- `desktop_entries/03_桌面上位机/`
- `verification/hardware/commissioning_20261101/manual_extracts/catalog_p22.png`
- `verification/hardware/commissioning_20261101/manual_extracts/catalog_p24.png`

## 3. 实现行为

### 3.1 驱动/服务

新增两个标准 `rcl_interfaces/srv/SetParametersAtomically` 服务（参数 `command_id` double、`boot_id` string）：

- `/slaihand/gravity_lock_current_pose`
- `/slaihand/gravity_resume_float`

服务只做 **binding + 排队**：校验驱动已激活、HOLD、未接管、无 pending stop、`boot_id` 与 `command_id` 与当前会话一致，然后 CAS 一个请求。实际锁存发生在驱动的下一个 `write()` 控制周期：

- `MotionSupervisor::tick()` 在同一周期、同一批 `ReadOnlyState` 上锁存四轴 `position`；
- `v_des = 0`（不改写速度前馈）；
- 目标 `target_` / `hold_target_` 固定，后续反馈不重写；
- Kp 从 0 按 policy `gain_ramp_s` 渐入到已验证的 `kp`（实际运行 policy，常用 160）；Kd 从自由拖动 `float_kd` 渐入到 policy `kd`（常用 5）；
- 重力前馈保持 `alpha=1`、`ramp=3`，不切回普通位置模式 `alpha=0.1/0`，不调用 `clear_feedforward`；
- 如请求时四轴速度超过 0.2 rad/s，进入 `LOCKING`，继续用既有自由拖动阻尼减速；速度/限位/电机状态全部满足后才用新鲜反馈锁存，避免推拉中突然硬刹；
- 锁定后目标恒定；实际角、目标角、误差、增益比例、驱动锁定状态、剩余时间通过 `gravity_ff_state` 尾部发布；`lock_reason_code` 给出限位/电机/command 等具体拒绝原因；
- 恢复自由拖动：先进入 `RELEASING`，Kp/Kd 平滑降到 0/float_kd，再回到 `floating`（Kp=0，重力保持）；必须有 `release_weight=true` 确认；
- 会话计时完全复用 `started_/float_seconds_`，锁定/恢复不重置，不延长租约；
- `stop()` 优先：`write()` 先处理 stop 并清空 mode 请求；STOP 后迟到的 lock/resume 由 panel 纪元/驱动阶段拒绝；
- 位置模式安全门禁保留：URDF 实际限位、反馈速度、跟随误差、电机状态、温度、反馈新鲜性；自由拖动“越软限位不立即停”的行为不带到 hold。

### 3.2 面板/HTTP/UI

在重力补偿卡片内新增：

- 状态：`自由拖动 / 锁定中 / 已锁定 / 恢复自由拖动中`；
- `锁定当前位置` 按钮；
- 四轴 `实际 / 保持目标 / 误差` 表；
- 当前 Kp/Kd、增益渐入比例、驱动锁定状态、本次会话剩余、结束动作；
- `恢复自由拖动` 按钮 + “已承接重量”确认；
- 只有驱动 `gravity_ff_state` 实际报告 `lock_state=2`、`gravity_hold=true`、`command_id` 与请求绑定一致，且面板仍有该运行会话时，才显示“已锁定”。

HTTP：

- `POST /api/gravity/lock`
- `POST /api/gravity/float`（必须 `confirmations.release_weight=true`）

所有 ROS 服务调用仍由 panel 的单一 ROS 线程 `_drain_queues()` 发出；HTTP 线程只入队，不直接调用 rclpy。请求带 2 秒 TTL、重复请求拒绝、停止纪元变化拒绝、`command_id/boot_id` 改变拒绝、心跳过期拒绝。拖动示教记录中拒绝锁定，提示先结束记录，避免锁定帧被标成自由拖动示教帧。

## 4. 验证

仅 mock/PTY 和隔离 domain 176；未触碰真实串口/CAN、USB attach、domain 0、实机驱动/桌面入口或部署参数。

### 4.1 C++ 监督器

```bash
cd /mnt/d/CodexData/Projects/机械臂/slaihand_gravity_hold_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./build/slaihand_hardware/motion_supervisor_test
# PASS ... CXX=0
```

新增覆盖：错误 command_id 拒绝、快速拖动先 LOCKING、同一批反馈锁存、锁定目标不随外部移动漂移、Kp 非瞬跳/最终到 160、Kd 到 5、剩余时间递减不重置、限位外不锁存、释放先 RELEASING 再回自由拖动、重复 lock/release 拒绝、重力 alpha 连续性、最终报文 Kp/Kd 在门禁内。

### 4.2 后端门禁

```bash
source install/setup.bash
python3 verification/hardware/gravity_hold_20261107/test_gravity_hold_backend.py -q
# Ran 8 tests ... OK
```

覆盖：lease 过期、stop generation、过期请求、command/boot 改变、重复请求、teaching-active 拒绝、面板限位/unknown lock state、release 重量确认、ff 尾部解析和目标/误差/增益/原因。

### 4.3 PTY + ROS + HTTP + MIT 写回执

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1
python3 verification/hardware/gravity_hold_20261107/verify_gravity_hold_pty.py
# PASS
```

结果（`evidence/gravity_hold_pty.json`）：

- locked target：`[14.994, 25.004, -10.01, 5.005]` deg；
- `alpha_locked=1`、`resume_alpha=1`；
- 1840 个 MIT TX_WRITTEN 报文，`max_kp=159.951`、`max_kd=5.0`；
- 544 个 0<Kp<160 的渐入报文，确认不是 float→position 瞬跳；
- 运行中扰动后目标不漂移；释放后 `resume_kp=0`；
- STOP 后迟到锁定被拒，停止确认成功；
- 未打开物理串口。

### 4.4 浏览器实际点击

Windows headless Chrome 通过 CDP 实际点击（WSL 隔离 fixture，fake PTY，panel 18888）：

```bash
python verification/hardware/gravity_hold_20261107/browser_gravity_hold.py
# PASS
```

实际点击：`btn-gravity-float` → `btn-gravity-lock` → `btn-gravity-resume` → `btn-stop`。证据：

- `evidence/gravity_hold_locked.png`：UI 显示“已锁定”、四轴实际/目标/误差、Kp/Kd、增益比例、会话剩余 35.5 s；
- `evidence/gravity_hold_resumed.png`：显示恢复自由拖动、Kp=0、alpha=1；
- `evidence/gravity_hold_before.png`、`evidence/gravity_hold_stopped.png`；
- `evidence/gravity_hold_browser.json`：`locked_phase=locked`、`resume_phase=float`、`resume_kp=0`、`stop_confirmed=true`。

说明：浏览器 fixture 为验证 float 会临时在安装 policy 中启用 `gravity_float_enabled`，脚本退出时恢复；源代码 policy 未改动。首次 Chrome 启动偶发 WSL localhost 转发未就绪，重试一次即可，报告不把它当作功能失败。

## 5. 复验 P1 修复：提前释放不增加刚度

复验反例发现：锁定渐入约 110 ms 后恢复，旧实现把释放斜坡从满增益重新开始，Kp 从 17.58 跳到 159.95。现已记录释放开始时的实际增益，释放从该实际值平滑降到 0；新增 C++ 回归“部分增益后提前恢复”，首帧 Kp 不得超过实际已达到的增益。

## 6. 语义边界与未解决项

1. **不是绝对不动**：抗推能力受有限 Kp/Kd、可用力矩、电机/机械承载和重力模型误差限制；UI 明确写“不保证任意外力下绝对不动”。
2. **外力超过既有 4° 跟随误差门禁会停止/失能**，这是保留的安全门禁，不是继续跟随外力；现场验收应缓慢施力。
3. **实机未验证**：只做 PTY/fake 和隔离 domain 176；未打开真实串口/CAN，未在实物上验证锁定刚度、温升、实际限位和断电/失联行为。
4. **主仓库默认 policy 未启用 float**；部署候选 policy 已启用。本次未改任何 policy/部署参数，交付的是源码与测试；正式部署需使用已确认的候选 policy。
5. **拖动示教录制中锁定被明确拒绝**，操作员需先停止记录；未实现“自动结束录制并标帧”的额外流程。
6. 会话结束仍走原有全轴停止/失能确认；驱动侧保留既有 `float_seconds+7s` 故障余量，UI 显示按用户设定的 float 时长计算的剩余时间。

## 7. Ponytail 收尾

- 跳过：实机动作/刚度标定、真实 PICO/后台部署、教学录制自动结束标记、UI 视觉细节重做、J5/其他任务。
- 风险：锁定保持只证明源码/PTY/浏览器路径；真实抗推能力和机械承载必须实机单独验收，当前不得宣称“外力下绝对不动”或“已实机验收”。
