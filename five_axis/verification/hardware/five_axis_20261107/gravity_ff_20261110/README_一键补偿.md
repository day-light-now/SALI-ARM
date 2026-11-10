# 一键静态重力补偿（拖拽版）

把任意 `slaihand_joint_plan/2` 计划 JSON **拖到 `run_compensation.cmd` 上**即可。
默认走 **hybrid**：离线模型补偿 → 单次采集 → 按实测残差再修正一次 → 再采集 →
对比报告 → 停驱动，并在开始前要求输入 `RUN` 确认现场安全。（旧的三轮迭代路径
仍保留在 `run_compensation_iter.cmd`。）

## 三种策略

| 入口 | 策略 | 采集轮数 | 适用 |
|---|---|---:|---|
| `run_compensation.cmd` | **hybrid（默认）** | 2 | 快速达到较好精度（模型粗补偿 + 一次实测修正） |
| `run_compensation_iter.cmd` | iter（旧路径） | 3 | 基线 + 迭代 1/2，最高精度；无模型、或域外姿态 |
| `run_compensation_planonly.cmd` | plan-only | 0 | **离线**：只校验并生成模型补偿计划，不启动驱动/不运动 |
| `run_compensation_dry.cmd` | dry | 0 | 只验证准入/artifact/驱动，不运动 |

模型补偿由 `compensate_plan.py` 完成，带训练域/补偿幅值/限位/端点保护；域外计划
会被拒绝，此时请改用 `run_compensation_iter.cmd`。

## 文件

| 文件 | 用途 |
|---|---|
| `run_compensation.cmd` | **拖拽入口（真机，默认 hybrid）** |
| `run_compensation_iter.cmd` | 拖拽入口（旧 iter：基准+迭代 1/2） |
| `run_compensation_planonly.cmd` | 拖拽入口（离线生成模型补偿计划） |
| `run_compensation_dry.cmd` | 拖拽入口（dry-run：只到驱动就绪） |
| `one_click_compensation.ps1` | PowerShell 实现（路径转换、确认、策略/模式） |
| `compensate.sh` | WSL 侧完整流水线（run/dry/plan × hybrid/model/iter） |
| `compensate_plan.py` | 模型补偿计划转换器（含安全域保护） |
| `compare_plan_runs.py` | 无基线时按原始计划计算残差 |
| `compare_any.py` / `summarize_comp.py` | 旧 iter 路径的对比与报告 |
| `summarize_model_comp.py` | hybrid/model 路径报告 |
| `make_hybrid_iter.py` | hybrid 的一次实测修正（以**原始目标**为基准，避免双重补偿） |
| `make_iter_comp.py` | 旧 iter 路径的按实测残差生成修正计划 |
| `validate_plan.py` / `plan_duration.py` | 计划校验与录制时长 |
| `parse_verify.py` / `check_windows.py` | 准入证据解析、静态窗口充足性检查 |

## 计划要求（拖拽前自动检查）

- schema `slaihand_joint_plan/2`，`coordinates: absolute_deg`；
- 1–12 个路点；**首、末路点必须是绝对零 `[0,0,0,0,0]`**；
- 每路点 `move_s`、`dwell_s` > 0；各轴在映射限位内；
- 客户端时间预算 178 s：例如 11 路点 × (6 s + 6 s) = 132 s 可过；
- 模型补偿还要求姿态在 T1a/T1b 训练域内（±2°）且预测补偿 ≤3°，否则拒绝。

## 运行与安全

- 开始前弹窗要求输入 `RUN`，请确认机械臂**已支撑、运动区域净空、可随时切电**；
- 全程不启用重力 FF：`gravity_ff_alpha` 保持 0；4° 跟随误差、STOP/UNKNOWN、
  限位、反馈新鲜度/准入、J5 watchdog 2000 均不变；
- 每轮计划首末绝对零；每轮之间软件复位/重启候选驱动，保证 fresh DISARMED；
- 每轮需要：只读 TIMEOUT 回验（若断电回 0 才写一次 0→2000）、30 min artifact、
  采集、静态窗口提取；任一轮失败（如 USB 写长尾）脚本立即停止并保留日志，
  **不会自动重试、不会强制回零**；
- 实现的是**位置前馈静态补偿**，不是转矩 FF；转矩 FF/自由拖拽仍需外部测量条件。

## 输出

- hybrid：`plans/<tag>_base_20261110.json`、`plans/<tag>_modelcomp_20261110.json`、
  `plans/<tag>_iter1_20261110.json`；
- 窗口：`windows_<tag>_model.json`、`windows_<tag>_iter1.json`；
- 报告：`runs/<tag>/<tag>_REPORT.md`、`<tag>_model_compare.json`、
  `<tag>_iter1_compare.json`；
- iter：`plans/<tag>_base|_iter1|_iter2`、`windows_<tag>_base|_iter1|_iter2`、
  `runs/<tag>/<tag>_REPORT.md`；
- 日志：`logs/<tag>_*.log`。

## 失败处理

- 输出 `ABORT: ...` 时按提示看 `logs/<tag>_*.log`；驱动会被自动停止。
- 若驱动/串口处于不确定状态：保持支撑并现场检查；不要连续拖拽重试。
- 隔夜或断电后第一次运行：脚本会自动检测 TIMEOUT=0 并做一次写设置。

## 当前状态（2026-11-10 晚）

- 离线 `plan` 模式已验证：`run_compensation_planonly.cmd` 可生成
  `<tag>_modelcomp` 计划，不启动驱动；域外计划会被拒绝。
- 真机 hybrid 全链路尚未验证（今天没有空闲硬件）；旧 iter 路径的 dry-run 已通过，
  真机全链路曾被 `SERIAL_OR_AUDIT_WRITE_FAILURE` 打断 3 次，均为 USB 写长尾，
  wrapper 均安全 `ABORT`。下一步真机优先跑 hybrid + 测试套件（见
  `README_测试准备_20261110.md`）。
- 在链路稳定前请勿连续拖拽重试；出现 `ABORT` 后先检查日志与现场状态。
