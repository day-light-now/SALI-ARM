# 有符号五轴采集结果（2026-11-09）

本轮真实设备采集；没有放宽 artifact/evidence 新鲜度，没有改保护参数。
driver 为 baseline **Kp=160/Kd=5**，JTC 路径容差仍是已有 10°；本轮没有调增益。

## 准入

- USB 串口：`/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00` → `/dev/ttyACM0`
- TIMEOUT：实际读回 0，写入 2000，再次读回 2000；证据
  `j5_timeout_setup_evidence_20261109.json`
- artifact：按当前 policy/load 重新生成，使用默认新鲜度，未使用 86400 放宽
- 全五轴 `motor_state=0`、`feedback_valid=1`、`timeout_count=2000` 后才启动

## 采集 1：同步 0–40° 路径

计划：`WORKSPACE_0_40_ACTUALSTART.json`，8 个路点，6 s 移动 / 4 s 停留。  
采集：`raw/signed_sweep_20261109.jsonl.gz`，controller_state 7501 帧。

客户端结果：

```text
trajectory_error_code = 0
Goal successfully reached!
reason = WAYPOINT_FEEDBACK_CRITERION_NOT_MET
```

JTC 完成动作，但 0.5° 无符号逐点判据未通过。稳定窗口 signed mean（°，
feedback - reference；负值表示实际低于目标）：

| 近似相对角 | J1 | J2 | J3 | J4 | J5 |
|---:|---:|---:|---:|---:|---:|
| 0° | -0.358 | -0.391 | -0.247 | -0.155 | -0.252 |
| +5° | -1.095 | -0.973 | -0.251 | -0.631 | -0.332 |
| +10° | -1.655 | -1.436 | -0.306 | -1.048 | -0.207 |
| +20° | -2.989 | -2.370 | -0.222 | -1.709 | -0.197 |
| +30° | -3.552 | -3.237 | -0.147 | -2.416 | -0.126 |
| +35° | -3.669 | -3.515 | -0.127 | -2.617 | -0.135 |
| +40° | -3.669 | -3.757 | +0.046 | -2.662 | -0.063 |
| 回0° | +0.110 | +0.000 | +0.000 | +0.307 | +0.376 |

## 采集 2：独立单轴/混合小姿态

计划：`INDEPENDENT_5AXIS_SMALL.json`，12 个路点：J1–J5 单轴 +5/−5°、
混合 `[+5,+5,0,−5,+5]`，3–4 s 移动 / 4 s 停留。  
采集：`raw/signed_independent_20261109.jsonl.gz`，controller_state 帧完整。  
客户端结果同样是 JTC `Goal successfully reached!`、`WAYPOINT...NOT_MET`。

signed mean（°）：

| 姿态 | J1 | J2 | J3 | J4 | J5 |
|---|---:|---:|---:|---:|---:|
| J1 +5 | **-0.833** | 0 | 0 | 0 | 0 |
| J1 −5 | **+0.561** | -0.034 | 0 | +0.022 | 0 |
| J2 +5 | -0.382 | **-1.073** | 0 | +0.017 | 0 |
| J2 −5 | -0.379 | **+0.680** | -0.035 | +0.019 | 0 |
| J3 +5 | -0.372 | -0.262 | **-0.221** | +0.020 | 0 |
| J3 −5 | -0.376 | -0.262 | **+0.155** | -0.015 | 0 |
| J4 +5 | -0.393 | -0.262 | -0.153 | **-0.220** | 0 |
| J4 −5 | -0.131 | -0.262 | -0.153 | **+0.457** | -0.034 |
| J5 +5 | -0.223 | -0.355 | -0.153 | +0.093 | **-0.366** |
| 混合 | -0.733 | -0.860 | -0.152 | +0.122 | -0.267 |
| 回0 | +0.110 | +0.001 | -0.153 | -0.132 | +0.329 |

## 采集 3：高增益独立姿态对比

同一 `INDEPENDENT_5AXIS_SMALL` 姿态、同一 0.5° 判据；把 J1/J2/J4
从 160 提高到 280/240/200（J3/J5 保持 160）：

| 姿态 | baseline signed mean | high signed mean | 改善 |
|---|---:|---:|---:|
| J1 +5 | -0.833 | **-0.508** | 39.0% |
| J1 −5 | +0.561 | **+0.414** | 26.2% |
| J2 +5 | -1.073 | **-0.533** | 50.3% |
| J2 −5 | +0.680 | **+0.376** | 44.7% |
| J4 +5 | -0.220 | **-0.178** | 19.1% |
| J4 −5 | +0.457 | **+0.326** | 28.7% |

客户端 0.5° 判据仍有 J1 +5 max_abs 0.541°、J2 +5 0.563° 等未通过；
部分单轴姿态通过。signed mean 与方向改善是真实的，但还没有动态/电流证据。

原始高增益数据：`raw/signed_independent_high_20261109.jsonl.gz`；  
signed 统计：`independent_high_summary_20261109.json`。

## 结论

1. 有符号方向已获得；正角度时 J1/J2/J4 主要为负偏差，负角度时方向反转。
2. 同一姿态的正/反向响应不同：J1 +5 为 −0.833，而 J1 −5 为 +0.561；
   J2 +5 为 −1.073，J2 −5 为 +0.680。不能只用一条符号曲线。
3. 单轴/混合姿态的误差与同步路径不同，说明存在**姿态耦合/路径历史**；
   单路径拟合不能当多姿态重力模型。
4. 这些是 `Kp*(reference-feedback)` 的**控制力矩估算**，不是实测电流/扭矩；
   没有电流、母线、外部扭矩、动态峰值/温升数据。
5. 不能据此生成可下发的重力 FF，也不能做 1.5× 裕量结论。

## 数据文件

- `signed_static_summary_20261109.json`：同步路径 signed 统计；
- `independent_summary_20261109.json`：独立姿态 signed 统计；
- `signed_acquisition/recorder.py`：只读 controller_state/dynamic 记录；
- `signed_acquisition/analyze_signed.py`：稳定窗口 signed mean/median/std/max_abs；
- `raw/*.jsonl.gz`：原始时序；
- `client_result_*.json`、`independent_client_result_*.json`：客户端结果；
- `test_signed_analysis.py`：2 项离线测试。

## 下一步

- 用有符号多姿态数据做**整组独立验证**，不是随机拆采样；
- 补真实电流/母线/外部扭矩，确认 `Kp*(ref-feedback)` 与真实保持力矩一致；
- 再评估受限 FF、角度调度、积分/DOB；
- 动态实验需单独报告峰值、过冲、振荡、温升、停止；
- 在得到真实 allowed output/demand 前，不做 1.5× PASS。

## 采集 4：独立 ±10° baseline

12 点，包含 J1–J5 单轴 ±10°、一个 10° 混合姿态和回零。JTC 均
`Goal successfully reached!`，但 0.5° 逐点判据部分失败：

| 目标 | J1 max_abs | J2 max_abs | J3 max_abs | J4 max_abs | J5 max_abs |
|---|---:|---:|---:|---:|---:|
| J1 ±10 | 1.388 / 0.733 | — | — | — | — |
| J2 ±10 | — | 1.257 / 1.454 | — | — | — |
| J3 ±10 | — | — | 0.164 / 0.099 | — | — |
| J4 ±10 | — | — | — | 0.755 / 0.623 | — |
| J5 +10 | — | — | — | — | 0.317 |
| 混合 10° | 1.257 | 1.367 | 0.000 | 0.296 | 0.208 |

原始：`raw/signed_independent10_20261109.jsonl.gz`；统计：`independent10_summary_20261109.json`。

## 采集 5：大角度独立姿态，失败中止

计划 `INDEPENDENT_5AXIS_LARGE.json`（单轴 ±20°、混合 20°/30°），
计划已通过限位检查并实际使能。第一点 J1 +20° 完成：

```text
J1 signed mean -2.521°
J4 signed mean -0.318°
```

随后在 J1 −20° 运动段：

```text
driver: SERIAL_OR_AUDIT_WRITE_FAILURE
client: 控制端已全局停止或UNKNOWN
```

这不是增益、跟随误差或限位拒绝；是串口/审计写入路径的故障。测试**未完成**，
已停止 driver，没有重试。证据：`large_abort_evidence_20261109.json`、
`raw/signed_independent_large_20261109.jsonl.gz`。

## 采集 6：真实电流验证

当前驱动只提供 `effort` 估算，没有真实电流/母线传感器，也没有
`bus_current`/`bus_voltage` 状态接口。因此本轮**没有真实电流数据**；
不能用控制力矩估算或 effort 估算代替真实电流/扭矩。

如需继续，必须外接分流器/功率分析仪或增加总线电压电流采样，并先核对
单位、标定、量程和有效性；在此之前不能做 1.5× 裕量或真实负载能力结论。


## 采集 7：独立 40° 大角度

计划 `INDEPENDENT_5AXIS_40DEG.json`：单轴 ±40°、混合 ±40°。
计划已通过限位检查并实际使能。

第一点 J1 +40° 完成：

```text
J1 signed mean -3.422°, max_abs 3.455°
J4 signed mean -0.960°, max_abs 0.962°
```

随后 J1 从 +40° 向 −40° 运动时 JTC：

```text
Aborted due to path tolerance violation
```

本轮**未完成**，不是串口/审计失败，而是路径跟随容差；未重试。
原始：`raw/signed_independent_40deg_20261109.jsonl.gz`；
统计：`independent_40deg_summary_20261109.json`。


## 采集 8：后方遮挡安全停止

慢速 J1/J4 40° 计划中，J1 +40° 与回中完成；在 J1 −40° 向后段
路径容差中止。随后操作者报告**后方有遮挡**。

处理：立即停止 driver，没有重试，没有恢复动作；在操作者确认遮挡清除前，
禁止任何负向/向后运动。记录见 `OBSTRUCTION_STOP_20261109.*`。


## 采集 9：向前 J1 +10° 试探

操作者确认向前试探。计划 `FORWARD_J1_10.json` 只有当前位和 J1 +10°，
10 s 移动、8 s 停留，无向后段。

实际 J1 从约 −12.15° 向前到 −2.15°，**没有发生后方遮挡接触**。
J1 稳态 signed mean ≈ −0.864°，max_abs ≈ 0.864°。

但 JTC 以：

```text
Aborted due to goal_time_tolerance exceeding by 2.002 s
```

结束。向前运动本身可行；精度/收敛仍未通过。driver 随后停止，未继续 40°。
证据：`raw/forward_j1_10_20261109.jsonl.gz`、`forward_j1_10_summary_20261109.json`。
