# 一键静态重力补偿（拖拽版）

把任意 `slaihand_joint_plan/2` 计划 JSON **拖到 `run_compensation.cmd` 上**即可。
脚本会自动完成"只读准入 → artifact → 启动候选驱动 → 基准采集 → 迭代 1 →
迭代 2 → 对比报告 → 停驱动"，并在开始前要求输入 `RUN` 确认现场安全。

## 文件

| 文件 | 用途 |
|---|---|
| `run_compensation.cmd` | **拖拽入口（真机运动）**：确认后执行 3 轮采集 |
| `run_compensation_dry.cmd` | 拖拽入口（dry-run）：只验证准入/artifact/驱动，不运动 |
| `one_click_compensation.ps1` | 拖拽入口的 PowerShell 实现（路径转换、确认、调用 WSL） |
| `compensate.sh` | WSL 侧完整流水线 |
| `validate_plan.py` / `plan_duration.py` | 计划校验（1–12 路点、首末绝对零、限位）与录制时长 |
| `make_iter_comp.py` / `make_iter2.py` | 按实测残差生成迭代 1/2 补偿计划 |
| `compare_any.py` / `summarize_comp.py` | 残差对比与报告 |
| `parse_verify.py` / `check_windows.py` | 准入证据解析、静态窗口充足性检查 |

## 计划要求（拖拽前自动检查）

- schema `slaihand_joint_plan/2`，`coordinates: absolute_deg`；
- 1–12 个路点；**首、末路点必须是绝对零 `[0,0,0,0,0]`**；
- 每路点 `move_s`、`dwell_s` > 0；各轴在映射限位内；
- 客户端时间预算 178 s：例如 11 路点 × (6 s + 6 s) = 132 s 可过。

## 运行与安全

- 开始前弹窗要求输入 `RUN`，请确认机械臂**已支撑、运动区域净空、可随时切电**；
- 全程不启用重力 FF：`gravity_ff_alpha` 保持 0；4° 跟随误差、STOP/UNKNOWN、
  限位、反馈新鲜度/准入、J5 watchdog 2000 均不变；
- 每轮计划首末绝对零；每轮之间重启候选驱动，保证下一轮从 fresh DISARMED 开始；
- 每轮需要：只读 TIMEOUT 回验（若断电回 0 才写一次 0→2000）、30 min artifact、
  采集、静态窗口提取；任一轮失败（如 USB 写长尾）脚本立即停止并保留日志，
  **不会自动重试、不会强制回零**；
- 本次实现的是**位置前馈静态补偿**（提升稳态姿态精度），不是转矩 FF；
  转矩 FF 仍需 J5 几何/effort 标定与准入升级，当前禁止。

## 输出

- 计划：`plans/<tag>_base|_iter1|_iter2_20261110.json`
- 原始记录与客户端：`raw/<tag>_*/`、`clients/<tag>_*/`
- 静态窗口：`windows_<tag>_*.json`
- 报告：`runs/<tag>/<tag>_REPORT.md`、`<tag>_summary.json`、
  `iter1_compare.json`、`iter2_compare.json`
- 日志：`logs/<tag>_*.log`

## 失败处理

- 输出 `ABORT: ...` 时按提示看 `logs/<tag>_*.log`；驱动会被自动停止。
- 若驱动/串口处于不确定状态：保持支撑并现场检查；不要连续拖拽重试。
- 隔夜或断电后第一次运行：脚本会自动检测 TIMEOUT=0 并做一次写设置。

## 当前状态（2026-11-10）

- **dry-run 已验证通过**（`run_compensation_dry.cmd` 或 `-DryRun`）：计划校验、
  只读 TIMEOUT 准入、artifact、候选驱动、J5_QUALIFIED 均正常，无运动。
- **真机全链路冒烟未完成**：连续 3 次尝试都在采集阶段被已知的
  `SERIAL_OR_AUDIT_WRITE_FAILURE`（CANUSB incomplete serial packet write）
  打断；wrapper 每次都在检测到驱动停止/窗口不足后 `ABORT` 并停驱动，
  没有异常运动。该故障与脚本逻辑无关，是今天反复出现的 USB 写长尾。
- 采集/补偿动作本身今天已手工验证完整跑通（JZ 基准、迭代 1、迭代 2 均完成），
  一键脚本只是把同一序列自动化；待 USB 链路稳定（检查线缆/端口/供电或换直连）
  后再做一次真机全链路验证即可。
- 在链路稳定前请勿连续拖拽重试；出现 `ABORT` 后先检查日志与现场状态。

