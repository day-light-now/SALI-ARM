# J5 大幅轨迹中断分析请求（给 Codex，2026-11-08）

## 结论先行

这次不是扭矩超限中断。日志明确是 **JTC 轨迹跟随误差超限**：

```text
WARN [tolerances]: State tolerances failed for joint 1:
ERROR Position Error: 0.184485, Position Tolerance: 0.174533
WARN Aborted due to state tolerance violation
WARN canusb_system: Control output disabled; reason=FOLLOWING_ERROR
```

- 0.184485 rad = **10.57°**
- 当时容差已放大到 **10°**（0.174533 rad）
- 同一轮更早一次：Position Error 0.186465 = **10.68°**，也 > 10°
- 之前 8° 容差时：0.153788 = 8.81°，0.158533 = 9.08°，均 > 8°
- 更早 4° 容差时：0.071503 = 4.10°

容差从 4→8→10° 都没有解决；**放慢计划到 move_s=15 s 也没有解决**，所以不要再只靠放大容差。

## 最新失败操作

目录：

```text
/home/xuyu/.ros/slaihand/five_axis_panel_20261108_adopted/operations/480484e3cd0147fb/
```

文件：

- `source.json`：`relative_start_deg`
- `plan.json`：面板冻结后的 `absolute_deg`
- `client/20261108_150751_bbd492eb.json`：客户端结果
- `panel.json`：面板结果
- `ticket.json`

源计划（相对起点）：

```json
[
  [0, 0, 0, 0, 0],
  [30, 50, 0, 0, 45],
  [0, 70, 0, 0, 0],
  [-20, 20, 0, 0, -45],
  [0, 0, 0, 0, 0]
]
```

用户已经把每段 `move_s` 从 5 改成 **15 s**，仍有失败。

冻结后的第一点等于当前实际角度：

```json
[0.24042632739906283, 0.13114163312676155,
 -0.1529985719812218, -0.1529985719812218, 0.34971102167136414]
```

所以这次**不是**之前那个“第一点绝对 0”的错误；相对起点转换是正确的。

时间线：

```text
1794121654.59  action goal accepted
1794121671.52  FOLLOWING_ERROR / control output disabled
1794121671.74  state tolerance failed (joint 1)
```

即开始后约 17 s：首段 0.5 s + 6 s dwell 后，进入 15 s 大段约 10.5 s 时失败。

## 结果里的实际位置（abort 后）

`client/20261108_150751_bbd492eb.json` 的 `last_observed_status`：

```text
J1 position 0.30556191348134654 rad  motor_state=0 timeout=2000 control_state=7
J2 position 0.5001144426642252  rad  motor_state=0 timeout=2000 control_state=7
J3 position 0.02746623941405346 rad
J4 position -0.08659494926375189 rad
J5 position 0.6301976043335618 rad
```

`waypoint_results` 的第二点 target 是：

```text
[0.5278, 0.8750, -0.0027, -0.0027, 0.7915] rad
```

即目标约 J1 30.24°、J2 50.13°、J5 45.35°。abort 后实际 J1 约 17.5°、J2 约 28.7°，说明 **J1/J2 都明显滞后**，日志只先报了 joint1。

## 扭矩/控制器证据

驱动 audit：

```text
/home/xuyu/.ros/slaihand/canusb_five_axis_candidate/canusb_motion_QbFl1r.jsonl
```

最后阶段的 `CONTROL_CYCLE`：

```text
target = [0.4238, 0.7016, -0.0027, -0.0027, 0.6355] rad
request_estimate_nm = [6.724, 11.117, -0.074, -0.623, 0.493] Nm
control_state = 8, command_id = 14
```

- J1 request ≈ 6.72 Nm，J2 ≈ 11.12 Nm
- J5 profile `tmax_nm = 28.0`
- 没有“扭矩超限”错误；故障是 FOLLOWING_ERROR
- 需要 Codex 判断：`request_estimate_nm` 是否真实等于最终 MIT 力矩命令，还是只是估算；J1/J2 是否实际进入电流/力矩饱和但仍被记录成跟随误差。

## 关键可疑点

1. **降低速度无效**：15 s/段仍失败，J1/J2 滞后约 10–20°，更像控制权限/重力负载/增益问题，不是轨迹速度。
2. **旧 URDF effort 限制**：
   `src/slaihand_description/urdf/slaihand_four_axis.urdf` 的 J1–J4 都是
   `effort="1" velocity="1.0"`。
   用户提示“之前没有小臂，现在有了”。需要确认 `effort=1` 是否仍被
   `CanUsbSystem`/JTC/command limit 使用；当前控制器启动日志却说
   `Enforcing command limits is disabled. Command limits from URDF will be ignored.`
   如果实际仍参与限制，这是明显过期参数。
3. **没有重力前馈**：当前 `gravity_ff_alpha=0`、`gravity_float_enabled=false`，
   J5/3.5 kg 的几何/COM 未测。大幅 J1/J2 姿态的静态重力负载可能导致稳态跟随误差；
   但 audit 中 J1 request 6.7 Nm、J2 11.1 Nm，低于 tmax 28 Nm，是否足够需要实测确认。
4. **JTC 当前配置**：
   `src/slaihand_bringup/config/canusb_five_axis_controllers.yaml`
   J1–J5 trajectory tolerance 已改为 10°（0.17453292519943295），goal 仍 1°。
   继续放大容差会掩盖真实的跟踪能力问题。
5. 用户已有两组“只动 J5”的成功数据（保留未动）；大幅 J1/J2 失败。

## 需要 Codex 分析/回答

1. 根据 audit 中 target、实际位置、`request_estimate_nm`、J1/J2 电机反馈，
   判断是 **力矩/电流饱和**、**控制增益不足**、**重力负载**，还是 **JTC 状态估计/单位问题**。
2. 确认 `slaihand_four_axis.urdf` 的 `effort="1" velocity="1.0"` 是否在五轴
   `CanUsbSystem` 路径上仍生效；如果生效，给出应更新的真实 effort/velocity 和依据。
3. 评估 `kp=160/kd=5`、MIT `tmax=28`、PMAX/VMAX 在当前小臂+3.5 kg 负载下是否合理；
   建议是否提高 kp、更新 effort、或先加静态重力补偿/前馈。
4. 给出下一组安全测试建议：例如 J1/J2 幅度降到多少、move_s 多少、是否先只做单关节 J1/J2，
   而不是继续放大 JTC 容差。
5. 若需要，请给 DSH 一个明确的代码/配置修改清单；不要直接建议去掉 FOLLOWING_ERROR/STOP 保护。

## 当前状态

- 提交：`9310b19`（10°容差）
- 面板下拉已有 `WIDE_*` 组；本次已用相对起点正确转换
- 本次失败后驱动已 FOLLOWING_ERROR 断开，需要 reset；当前未再发运动
- 两组只动 J5 的成功结果保留
