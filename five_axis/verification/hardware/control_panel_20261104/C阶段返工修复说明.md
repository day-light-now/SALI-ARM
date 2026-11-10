# C 阶段返工修复说明（2026-11-05）

依据 `Codex验收_C阶段返工.md`。本轮只修采集/分析门槛、新鲜度与版本/来源校验，以及操作单与原始证据保护；未改驱动、增益、alpha（保持 0），未安排现场采集。

## 复现（原样运行 `codex_c_repro.py`）

```text
{'moving': {'status': 'INSUFFICIENT', 'usable': 0, 'scale': None},
 'disabled': {'status': 'INSUFFICIENT', 'usable': 0, 'scale': None},
 'mixed_versions': {'status': 'INSUFFICIENT', 'usable': 12, 'scale': None},
 'negative_scale': {'status': 'INSUFFICIENT', 'usable': 12, 'scale': None}}
cached_snapshot_age_s=0.20; returned_feedback_age= None
```

反例输出中的 `usable/scale` 只来自诊断区（`diagnostics.diagnostic_only=true`），不构成拟合结论；`fit` 字段只在全部门槛通过时才存在。

## 1. P1：纯静态资格

- schema 升到 `/2`，新增必填：`stage`、四轴 `motor_state`、`static_window`（窗口样本数、跨度、最大速度、最大位置变化、窗口阶段与电机状态）、`capture_age_s`、`load_id`、`runtime`、`driver_ff`。
- 拟合资格（`exclusion_reason`）依次要求：自由无接触 → `motion=static` → 阶段为 HOLD/TRACK → 四轴使能（`motor_state==1`）→ 窗口内速度/位置稳定、阶段使能 → 驱动前馈 `alpha==0` → 运行时配置身份已验证。
- **失能样本**（stage 8 等）保留原始并标 `geometry_only`，只能做几何检查，绝不进入力矩拟合。
- 非静态/未知状态/带支撑/带接触/前馈非零/身份未验证的样本都写明排除原因，不再“悄悄可用”。
- 负/零尺度、符号一致率 < 0.8、留出 RMSE > 0.2 N·m 或 > 3× 训练 RMSE 均判 `INSUFFICIENT` 并给出原因，数值保留在诊断区。
- 可辨识性措辞改为“本工具主动只拟合全局尺度（范围选择）”，不再把“静态数据只能拟合全局尺度”当作一般定理。

## 2. P1：新鲜度与非同步位置

- `Recorder._build()` 只接受**同一帧** `DynamicJointState` 的 position/velocity/effort/temps/motor_state/control_state；本地接收年龄 `capture_age_s` 与驱动 `feedback_age` 取最大值，必须 ≤ 0.06 s，否则返回 `None`。
- 反例中 0.20 s 缓存的快照现在返回 `None`（见上）。
- 如果同时收到新鲜的 `JointState`，只做交叉核对（四轴位置差 > 0.01 rad 拒绝），不再混合两个话题拼样本。
- 静止窗口采集要求每条样本来自**新的** dynamic 序列号，同一缓存/回执不能重复累计；窗口跨度 < 0.8 s、样本 < 5、速度/位置超阈值均拒绝。
- 新增 Recorder 回调测试：缓存 0.20 s 拒绝、缺 position 拒绝、新鲜快照返回真实总年龄、重复回执不计入窗口。

## 3. P1：版本一致性与来源

- 每条样本记录完整身份：URDF/质量档案/mapping/policy 哈希、质量档案状态、重力向量、J3 偏置、`load_id`、`runtime`。
- `analyze` 先做身份一致性：样本间任一字段不同 → `INSUFFICIENT`（拒绝混合拟合）；与“当前分析模型”不一致 → `INSUFFICIENT`；身份不完整 → `INSUFFICIENT`。`report.identity` 不再是空对象。
- `runtime` 目前记录为 `{"verified": false, "source": "unknown"}`：驱动没有可验证的运行时配置身份接口。这样的批次**只能诊断**，状态固定为 `INSUFFICIENT`，不会成为可信驱动标定；预览会明确写出该限制。取得可验证运行时身份后再放开。

## 4. 操作单与原始证据

- 标题改为“工具不发运动；姿态调整由用户独立执行”；A 段要求使能稳定保持、自由无接触，明确失能手掰不能取重力保持力矩；单条 ~0.2 N·m 符号证据不足以证明物理安装。
- 采集改为**逐姿态显式触发**（`--trigger enter`，每次摆好后按 Enter 采一个持续窗口），不再用 `--count 12` 自动计时；未知支撑不允许默认 free；沿用的安装声明不再重复确认，只有真实矛盾才询问。
- 原始文件用独占模式 `open(...,"x")` + 纳秒级唯一文件名创建，异常退出也 close/chmod 0444；`--save-report` 拒绝与 raw 相同路径（含符号链接）；文档明确 `chmod 0444` 只是权限设置，不是不可篡改证明。

## 5. 证据

- `test_calibration_core.py` **35/35**（含运动/失能/混版本/负尺度/未验证身份/窗口不稳/缓存过期/重复回执等反例）。
- `fake_calibration_check.py`：验证运行时的假数据得到 `PROVISIONAL_SCALE` + 报告；UNKNOWN 运行时批次得到 `INSUFFICIENT`（仅诊断）；同路径/符号链接写报告返回 2；原始文件唯一名创建。
- 全套 `run_verification.sh`：见同目录最新日志（j3/calib/cli 状态位）。
- `alpha` 保持 0；未采集现场数据；现场操作单已更新，等 Codex 复核后再引导用户采集。
