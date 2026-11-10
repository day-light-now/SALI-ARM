# B 阶段三项返工修复说明（2026-11-05）

依据 `Codex验收_B阶段返工.md`。本轮只修三项 P1 与必要断言，未扩大范围；`alpha` 默认仍为 0；未操作实机，未改保护阈值。

## 修复 1：非零前馈丢失编码边界修正

- 根因：`controls()` 先为完整 URDF 边界选可表示位置码；`candidate!=0` 分支又用原始 `target_` 重新编码，丢弃修正。
- 修复：用 `command_target` 保存修正后的模型目标，前馈重编码复用它；最终报文重新解码并再次验证位置在完整 URDF 区间内，同时按含 `tau` 的最终命令做完整估计门禁。
- 覆盖：四轴 × 上/下限 × 双向 sign × `alpha=0.5`；新增单测断言 `crossings==0`。

## 修复 2：gravity_ff_ramp_s 未生效

- 根因：前馈误用增益斜坡 `p_.ramp`；`p_.gravity_ramp` 只校验/发布。
- 修复：独立 `ff_scale = clamp((at-confirmed_at)/gravity_ramp, 0, 1)`，每轴从使能确认起算；`0` 表示使能确认后立即生效（policy 注释与本文档写明）；增益斜坡原行为不变；停止路径仍直接 FD，不等待斜坡。
- 复验：2.2 s 时 ramp=1 → -0.2442 N·m；ramp=10 → -0.04884 N·m（不同曲线）。

## 修复 3：已应用状态不代表实际写入

- 四层状态分离：`requested`（模型请求）→ `candidate`（按 `guard−|PD|` 限幅）→ `encoded`（最终报文解码值）→ `applied`（串口写成功回执后更新）；`applied` 只在 `written(MIT)` 时更新，历史最后写入单列并带时间。
- 活动判定：`active` 仅在 PRELOAD/ENABLING/RAMP/HOLD/TRACK、模型存在且 α>0 时为真；`stop()/reset()/arm()` 清除当前状态；STOPPING/DISABLED/UNKNOWN 与新 `command_id` 不再显示旧值。
- 话题 `/slaihand/gravity_ff_state` 扩为 21 值：`[alpha, ramp, active, stage, command_id, requested×4, encoded×4, applied×4, last_written×4]`，best-effort；面板按 `active` 与消息时效显示“活动 / 未活动 / 历史最后写入”。
- 回归：部分写失败（跳过一轴 `written`）不更新该轴 `applied`；stop 后 `active=0`、`applied` 全 0；reset/re-arm 同样清零；命令整体拒绝时不会展示未写出的值。

## 附加修正（与上一项直接相关）

OpenArm codec 的 `double_to_uint` 向 `-tMax` 截断：`tau=0` 编码为 2047、解码为 **-0.04884 N·m**（J1/J2；J3/J4 为 -0.01319），小前馈会被整体下偏最多一个码。修复：发送前加半码（`tMax/4095`），使报文取最近可表示力矩；`alpha=0` 时字节不变（仍为 2047）。A 阶段说明里“+半步”的符号错误已改正为负半步。

## 复验证据

- 加载库（候选运行环境）：`/home/xuyu/slaihand_control_panel_ws_20261104/install/slaihand_hardware/lib/libslaihand_hardware.so`，sha256 `3d6ef15d8ecdd3692863112c07b274b681f4272f942dda18a1042739de336c09`（2026-11-05 09:07）。
- Codex 反例原样复跑：

```text
configured_ff_ramp=1 ff_after_2.2s=-0.2442
after_stop stage=8 active=0 ff_applied=0
configured_ff_ramp=10 ff_after_2.2s=-0.04884
after_stop stage=8 active=0 ff_applied=0
boundary_crossings=0
```

- 全套离线：C++ supervisor（含新增前馈斜坡/边界/写入回执断言）通过；重力模型单测 7/7；C++↔Python 交叉核对通过；生产回调 11/11；端到端 119/119（含“失能后驱动不再报告活动前馈”）；浏览器 38/38；驱动前馈 PTY α=0.55：解码 `[0.928, 1.221, -0.013, -0.119]`，与模型差不超过一个量化码，停止后 `active=0/applied=0`。
- 命令：`bash verification/hardware/control_panel_20261104/run_verification.sh` 与 `python3 run_codex_b_repro.py`（后者需先 source `test_env.sh`）。

## 仍未处理（按 Codex 顺序留待后续）

- A 阶段遗留：J3 偏置的物理一致性核验、shadow 余量在“参考未知”时的展示语义。
- C 阶段质量/姿态核验、D 阶段恒速扫描客户端、Kp/Kd 调参页。
- `alpha` 保持 0，未用于实机。
