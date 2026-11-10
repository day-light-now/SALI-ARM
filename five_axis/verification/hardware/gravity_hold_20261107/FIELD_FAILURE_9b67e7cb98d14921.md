# 现场失败与校准资格诊断报告

日期：2026-11-07  
现场操作 ID：`9b67e7cb98d14921`（旧 S01 四轴标定计划）  
目标仓库：`D:/CodexData/Projects/机械臂/slaihand_ros2_ws`；独立工作树 `/mnt/d/CodexData/Projects/机械臂/slaihand_gravity_hold_ws`，分支 `feature/gravity-hold-20261107`。  
本轮诊断提交：**`ea4527a`** `fix(calibration): reject old four-axis groups before arm and persist pre-arm diagnostics`  
复验反例修复提交：**`ca3ec85`** `fix(gravity): start release ramp at actual gain and harden stop/receipt races`  
复验三项残留修复：**`a90d817`** `fix(gravity): freeze mode callbacks and bind qualification identity end to end`  
锁定按钮任务与其实现在另一份报告中分别说明。

## 1. 只读核对结果

`/home/xuyu/.ros/slaihand/control_panel/operations/9b67e7cb98d14921/panel.json`：

- `kind=execute`、`phase=failed`、`exit_code=1`；
- `argv` 包含 `--calibrate --ack-free-static --plan ...`；
- `result_available=false`、`result_error="客户端未产生结果文件（未执行或提前退出）"`；
- 无 `client/*.json` 结果文件；
- `command_id` 字段未产生 arm receipt 接管结果；
- stdout 最后为 `等待超时，当前状态：{...}`，状态中四轴 `control_state=0`、`motor_state=0`、`command_id=0`；
- 从 `started_at` 到 `finished_at` 约 6.95 秒。

Codex 已核对的实时面板：`connected=true`、`stage=8`（已失能）、`stage_fresh=true`、`feedback_valid_all=true`；`baseline.alpha=0.1`、`satisfied=false`。

部署 candidate policy：`gravity_ff_alpha: 0.1`、`load_id: no_end_load`；这不是掉线或 UNKNOWN 状态。

## 2. 根因

1. **Panel 的旧校准组执行入口缺少预检查。** `preview/execute` 在 `mode=calibration` 时只校验计划形状/姿态/预算，没有在启动客户端前检查：
   - 驱动普通位置保持基线 `alpha` 必须为 0；
   - 运行身份必须新鲜且可解析；
   - 计划所属负载/模型/J5 资格。
2. **客户端预使能等待把 alpha 错误当成普通等待。** 原来 `wait(calibration_ready,5)` 统一等待 5 秒，超时后只抛出最后 `status()` 大字典；没有把 alpha、身份、stage、motor、ff 时限逐项列出。
3. **客户端预使能异常不落结果文件。** 结果写入在 `if attempted:` 内，而 `attempted` 在 arm 请求之前仍为 false；所以 alpha 失败时客户端退出 code1，但没有 `client/*.json`，面板只能显示“未产生结果文件”。
4. **新硬件的模型资格没有表达。** 现场已安装 J5、末端电机盒、约 3.5 kg 新增负载；旧组是 `no_end_load` 四轴档案，不含 J5 几何和新负载质量档案。不能沿用旧组继续拟合，也不能通过把 alpha 改成 0 来绕过。

## 3. 修复

### 3.1 Panel 预览/启动前资格门禁

`control_panel.py` 新增 `_calibration_preflight(document)`，在 `preview(mode=calibration)` 和 `execute(mode=calibration)` 启动客户端前调用：

- 读取实际 `_ff_view()` 的 `alpha`、连接/新鲜度；
- 读取新鲜运行身份、`load_id`、模型指纹；
- 按既有 `calibration_session.group_document()` 的计划哈希识别旧组（S01/S02/S03/S04/V01/G01-G06/L01-L04/V02）；
- 逐项产生稳定原因码：
  - `FF_NOT_READY` / `ALPHA_UNKNOWN` / `ALPHA_NONZERO`；
  - `IDENTITY_NOT_READY` / `IDENTITY_INVALID` / `LOAD_ID_MISSING`；
  - `PLAN_UNQUALIFIED`（计划不匹配已发布组，缺少 load_id/模型资格）；
  - `LOAD_MISMATCH`；
  - `MODEL_UNQUALIFIED`（旧四轴 `no_end_load` 组无 J5/3.5 kg 新负载档案资格）。
- 返回 HTTP `409 CALIBRATION_PREFLIGHT`，响应体包含 `calibration_preflight.reasons` 和 `calibration_preflight.observations`（alpha、group、load_id、mass_profile_sha256、identity freshness 等）；
- 在预览/启动前拒绝，不创建票根，不启动客户端；不等待 5 秒。

因此旧 S01 计划在当前部署 policy 下会立即显示：

`ALPHA_NONZERO ... 当前观察 alpha=0.1000`  
`MODEL_UNQUALIFIED ... 旧四轴组 load_id=no_end_load、无 J5 几何/3.5kg 新负载档案...`

页面展示已改为可换行显示逐项原因和观察值，不再只显示最后状态字典。

### 3.2 客户端 pre-arm 诊断与结果文件

`commission_supported_trial.py`：

- 新增 `PreArmError`，区分 `prearm_timeout`；
- 预使能资格由 `calibration_capture.prearm_diagnostic(sample)` 逐项产生：
  - `SAMPLE_UNAVAILABLE`（无同帧新鲜反馈）
  - `RUNTIME_NOT_VERIFIED`
  - `STAGE_NOT_DISARMED`
  - `MOTOR_NOT_DISABLED`
  - `FF_UNAVAILABLE`
  - `FF_STALE`
  - `ALPHA_UNKNOWN`
  - `ALPHA_NONZERO`
- 已知 `ALPHA_NONZERO` **立即拒绝**，不再等待 5 秒；
- 其余条件在 5 秒窗口内持续观察，超时也生成结构化诊断；
- 结果文件在 `finally` **无条件写入**（即使 `attempted=false`），包含：
  - `phase`、`phase_result`、`arm_requested`、`trajectory_requested`、`trajectory_started`、`stop_requested`、`stop_result`、`stop_observations_confirmed`；
  - `prearm_reasons`、`prearm_observations`、`prearm_timeout`；
  - `unknown_observed`；
- 明确区分：
  - 未请求使能：`phase_result=未请求使能`，`stop_result=未请求停止`；
  - 已请求使能但未开始轨迹：`arm_requested=true, trajectory_requested=false`，终于停止流程；
  - 真实停止未确认：`stop_requested=true, stop_observations_confirmed=false`，保留 `stop_error`，不伪装成正常结束；
  - UNKNOWN 时保留 `unknown_observed=true`，提示保持支撑并现场切电，不重复使能。

### 3.3 页面

- 预览拒绝消息中显示逐项校准原因和观察值，CSS 支持长文本自动换行；
- 操作结果面板新增：阶段判定、使能请求、轨迹请求、轨迹已开始、停止请求、停止判定、预检查原因、UNKNOWN 观测；
- 预使能失败不再显示“客户端未产生结果文件”而没有解释。

## 4. 回归与证据

- Backend 门禁：`10 tests OK`
  - `_calibration_preflight` alpha .1：`ALPHA_NONZERO` + `MODEL_UNQUALIFIED`，观察 `group=S01`、`alpha=0.1`；
  - identity 不新鲜：`IDENTITY_NOT_READY`；
  - `prearm_diagnostic` 同时列出 runtime/ff stale/alpha 原因；
  - 原有锁按钮 stop/lease/身份/重复请求测试继续通过。
- 预使能结果文件：`test_calibration_prearm_result.py` PASS
  - 客户端在对本机隔离 fake 栈、alpha=.1 的预检查中约 2.2 秒退出；
  - 生成结果文件；
  - `phase=pre_arm`、`phase_result=未请求使能`、`stop_result=未请求停止`；
  - `prearm_reasons=[ALPHA_NONZERO ...]`，观察 `alpha=0.1`、stage0、motor_state0、runtime_verified=true。
- 浏览器页面：`browser_calibration_preflight.py` PASS
  - 页面加载 S01，点击“服务器预览”；
  - 消息显示 `CALIBRATION_PREFLIGHT`、`ALPHA_NONZERO`、`MODEL_UNQUALIFIED` 和观察值；
  - 证据：`evidence/calibration_preflight_error.png`、`evidence/calibration_preflight_browser.json`。
- 证据 JSON：`evidence/calibration_prearm_result.json`。

命令：

```bash
cd /mnt/d/CodexData/Projects/机械臂/slaihand_gravity_hold_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1
python3 verification/hardware/gravity_hold_20261107/test_gravity_hold_backend.py -q
python3 verification/hardware/gravity_hold_20261107/test_calibration_prearm_result.py
python verification/hardware/gravity_hold_20261107/browser_calibration_preflight.py   # Windows side
```

## 5. 边界

- 仅修改候选源码与隔离测试；**没有替换/重启现场 42663 面板，没有替换驱动，没有操作真实串口/CAN**。
- 没有自动修改现场 alpha、没有自动改成 alpha=0、没有自动改 load_id/质量档案/J5。
- 新 3.5 kg 负载必须建立新的 `load_id`、新的质量/重力档案与 J5 几何；旧 `no_end_load` 四轴组被明确拒绝用于新负载拟合。新负载接入是后续独立工作。
- 当前 `MODEL_UNQUALIFIED` 是基于现场事实（J5 + 3.5 kg）的产品资格边界，不是对实际负载质量的测量。
- 本次修复只覆盖诊断、预览/启动门禁、pre-arm 结果与页面呈现；未进行实机动作、未做真实负载标定。

## 6. 复验反例修复

### [P1] 提前恢复自由拖动不再跳回满刚度

`MotionSupervisor` 记录释放开始时的实际 `hold_gain_scale_`（`release_gain_start_`）。释放斜坡现在从该实际增益开始下降：

- 早期锁定约 110 ms 后恢复，不再从 0.11 跳到 1.0（Kp 约 17.58 → 159.95）；
- 释放首帧 Kp 不超过实际已达到增益；
- C++ 回归新增“部分增益后提前恢复”检查。

### [P2] 停止后的旧模式请求

`_dispatch_gravity_mode` 的最终停止门禁、请求状态检查和 `call_async` 现在在同一个 panel 锁内完成。初始检查之后发生的并发停止会使最终检查拒绝发送；不再出现“停止门禁已锁但仍发送一次模式服务”。

### [P2] 旧回执不能改写新请求

`_service_done` 现在按 mode 请求的 `request_id` 绑定回执：

- 旧锁定回执到达时，如果当前是新恢复请求，不会把新请求标为 accepted/rejected；
- 旧回调只记服务事件，并报告“忽略旧重力模式服务回执”；
- 超时处理同样只修改对应 request_id。

### [P2] 停止等待期间出现 UNKNOWN

客户端在停止等待结束后再次观察四轴状态；若 UNKNOWN 首次出现在停止等待期间，仍设置 `unknown_observed=true` 并提示保持支撑/现场切电，不会保存 false。

### 校准资格通过路径

新增 `calibration_qualification.json` 注册入口和 `calibration_session.QUALIFIED_GROUPS`：

- 未来新负载档案可以登记新的组、路点、`load_id`、`mass_profile_sha256`、`j5_geometry`；
- panel 预检查只有在该组已登记、计划路点匹配、当前运行身份 `load_id`/质量档案哈希匹配且 J5 几何为 true 时通过；
- 旧 `no_end_load` 组仍拒绝；新增 backend 测试验证了“新负载资格满足时通过、load_id 不匹配时拒绝”的正反路径。

### 复验结果

- C++ supervisor：PASS（包含提前释放实际增益检查）。
- Backend：**14 tests OK**（alpha/identity/old model、P1 门禁、stop 并发、旧回执、UNKNOWN 标记、新负载通过路径）。
- 浏览器 calibration preflight 回归：PASS；新版页面仍显示逐项原因和观察值。
- 现场 42663 面板、驱动、串口/CAN 和部署参数未改动。

## 7. 复验三项残留修复（`a90d817`）

1. **旧同名 future 不能接受新请求**：`_service_done` 回调闭包冻结 `future/request_id/operation_id/generation`，在任何 `_pending`/状态 pop/update 前核对 token；新的测试真实覆盖“new future 已覆盖 key 后 old future 到达”，并验证 new future 仍能完成自己的请求。
2. **资格绑定实际模型/映射/J5 几何**：资格组必须声明并匹配 `load_id`、`mass_profile_sha256`、`model_sha256`、`mapping_sha256`、`j5_geometry_sha256`，J5 布尔不再是唯一条件；backend 正反路径覆盖 model/mapping/j5 指纹不匹配。
3. **资格身份贯通客户端/raw**：`commission_supported_trial.py` 增加 `--load-id/--calibration-group/--model-sha256/--mapping-sha256/--j5-geometry-sha256`，不再硬编码 `no_end_load`；Recorder/provenance/CalibrationSession/raw record 使用冻结的 load_id；prearm 诊断核对期望身份；panel ticket/launch 传递资格字段。

回归：C++ PASS；backend **14 tests OK**；页面/PTY 证据沿用前节；现场 42663 未改。

Ponytail 收尾：跳过真实硬件动作与 J5/新负载软件接入本身；风险是新组计划尚未生成时旧组会被拒绝，现场必须按新 load_id/档案/J5 几何重新生成后再采样。
