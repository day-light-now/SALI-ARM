# J4 主动运动不动：受限控制软件修正交付

状态：**软件修正与离线回归已完成；等待 Codex 验收，用户尚未执行新档位实机试验。**

本轮只读取已有日志、编辑本项目代码并运行离线测试。没有打开 COM9 或任何物理串口，没有查询/使能/试动电机，没有写参数、清故障、改零位/URDF/原始日志，没有运行桌面 BAT，没有安装依赖。`0.30Nm` 总编码力矩上限和原保护全部保留，没有自动加力。

---

## 1. 真实日志复核（只读）

复核对象（未修改）：

- `tmp/live_trial/20261101_140405_3f5951.json` 与 `.jsonl`（失败的 J4 +10° 往返）
- `tmp/four_axis_readonly/20261101_140359_334500.jsonl`（同次往返的原始总线反馈）
- `tmp/live_trial/20261101_134732_a325bf.json`（此前 2 秒当前位置保持）
- `tmp/watchdog_check/20261101_131809_aee5b0.json`（看门狗报告）

复核脚本（仅读日志，不属于产品代码）：`tmp/j4_analysis/analyze_logs.py`、`reconstruct.py`、`replay_real_failure.py`。

### 1.1 可复核数值

| 项目 | 记录值 |
|---|---|
| J4 MIT 控制帧数 | 719 |
| 指令力矩峰值（编码后，幅值） | **0.145055 Nm** = 0.30 上限的 **48.35%**，编码可达上限 0.276923 的 **52.38%** |
| 指令请求峰值（量化前，按 Kp=4/Kd=0.5 由记录误差反算） | 0.141082 Nm |
| 请求被 0.30Nm 上限裁剪的周期 | 0 / 719 |
| 到达编码力矩上限的周期 / 时间 | **0 / 719（0.00%，0.00s）** |
| J4 位置码（16 位编码器） | 全程唯一值 **0x7AAA**，变化 **0** 次 |
| 编码器分辨率 | 0.0218569° |
| 实际行程 | **0.0000°**（相对起点；没有发生运动） |
| 反馈速度 | −0.032967 … +0.010989 rad/s（低于 0.10 rad/s 中止阈值） |
| 反馈力矩 | −0.171429 … +0.013187 Nm；幅值峰值 0.171429 Nm = 上限的 57.1% |
| 停止时跟随误差 | 2.0074°（2° 门槛） |
| 10° 出程用时 | **5.888 s / 18 s = 32.7%**，未完成出程 |
| 出程目标位置 | 8.38103°（起点 6.38223°，实际前进 1.9988° = 10° 的 20.0%） |
| 停止结果 | `reason=Following error`，四轴 FD，新反馈确认 `DISABLED` |

### 1.2 事实与推断分开

**已证实的事实：**

1. **这次没有用满输出预算。** 719 帧的峰值只有 0.145055 Nm，占 0.30 上限 48.35%；0 帧到达编码上限；0 帧请求被 0.30 裁剪。按 Kp=4，要达到 0.30Nm 请求需要 **4.30°** 跟随误差，但 2° 就中止，所以 Kp=4 在这次结构上**不可能**用满 0.30Nm。
2. **J4 确实没有动。** 原始反馈的 J4 位置码在出程窗口内始终是 0x7AAA，唯一值，0 个码值变化。不是“动了一下又回来”，也不是“反馈丢失/角度分支异常”。
3. **控制请求确实到达了驱动。** 使能状态变为 0x1，反馈力矩随指令从约 +0.013 变到 −0.171 Nm；驱动侧没有报故障（最终状态是正常失能 0）。所以不是“命令没发出/没被接受”。
4. **停止是设计内保护。** 目标−实际超过 2° 触发全局停止、逐轴失能，并由新反馈确认 DISABLED；不是通信、看门狗或温度故障。

**不能由本次日志推出的结论：**

- 不能证明“0.30Nm 不够”——本次只到 0.145Nm。
- 也不能证明“提高到 0.30Nm 就会动”——支撑夹死、静摩擦、负载、传动/抱闸都可能仍在门槛之上。
- 反馈力矩 0.171429 Nm 是驱动自己的估计值，不是外部负载的测量；`0.022°` 的保持误差也不是主动运动证据。

**量化说明（影响读数）：** J4 的 TMAX=54Nm、力矩字段 12 位，单个量化步长 `2×54/4095 = 0.026374 Nm`。因此 0.30Nm 上限下可编码的最大幅值是 `10.5 × 108/4095 = 0.276923 Nm`，不是 0.30。请求 0.141082 Nm 被就近编码成 0.145055 Nm，是量化向上取整（仍在 0.30 内）；报告指标同时给出“请求值”和“编码值”，避免把量化差当成可用输出。

---

## 2. 代码改动

只改了 `slai_console/actuation.py`、`slai_console/live_trial.py` 和三个相关测试文件。`actuation_io.py` **未改**（无必要：会话/独占/快照/停止确认路径原样够用）。`hardware_mapping.yaml`、URDF、`provisional_gravity.json`、已有实机日志和成功报告均未改。

### 2.1 `slai_console/actuation.py`

- **显式主机 PD 档位表**：新增 `HostPdTier`、`PD_TIERS`。
  - `kp4_verified`：Kp=4 / Kd=0.5，**默认**，与既有实机记录一致，`hardware_validated=True`。
  - `kp12_candidate`：Kp=12 / Kd=0.5，**较硬候选档，未实机验证**，`hardware_validated=False`。
  - 硬上限 `MAX_CANDIDATE_KP=12.`、`MAX_CANDIDATE_KD=.5`，与档位表分离，后续加档也不能绕过。
- **票根绑定档位**：`LimitedJ4Trial` 新增 `pd_tier` 字段（默认已验证档），`validate()` 解析档位；`LimitedJ4RoundTrip.hold_report` 改为带默认值的关键字参数。档位是冻结票根的一部分，改增益必须新票据。
- **控制器强制“档位=包络”**：`FourAxisController` 对限定 J4 试验要求 J4 包络的 `kp/kd` 与该票根档位**精确相等**；未知档位、kp>12、kd>0.5、torque>0.3、speed>0.02、following_error>2°、行程>14°/4°、feedback_speed>0.1、arrival>0.5° 全部拒绝。没有“改一下 kp 继续跑”的路径。
- **量化边界函数**：`torque_raw_bounds(cap,tmax)` 与 `encoded_cap_torque_nm(cap,tmax)`，统一“编码后仍在 cap 内”的上下限；0.30/54 下给出 0.276923 Nm。
- **输出记录**：`_torque_frame` 用上述边界选码，并把本周期真实构建的 `requested_nm/limited_nm/encoded_nm/cap_nm/encoded_cap_nm/raw/raw_low/raw_high/request_clipped/at_encoded_cap/error` 记入 `controller.output[joint]`。记录的是“构建了什么”，不是驱动后来回报的估计力矩。
- **结果分类** `motion_summary()`：从生产采样计算是否移动、是否停滞、行程、请求/编码峰值、占上限比例、到达/接近编码上限的周期与时长、请求被裁剪周期、位置码集合、跟随误差，并给出 `assessment`：
  - `MOVED`
  - `COMMAND_BELOW_CAP_NO_MOTION`（命令没要求足够输出）
  - `OUTPUT_AT_ENCODED_CAP_NO_MOTION`（已到编码上限仍不动）
  - `NO_MOTION_REQUIRED`（保持试验）
  - `NO_OUTPUT_RECORDED` / `NO_ENABLED_SAMPLES`
- **原保护逐项保留**：当前位置接管、错峰使能、新反馈确认、2 秒保持、冻结 10° 往返每程 18 秒/整次 45 秒、2° 跟随误差、0.02rad/s 目标限速、0.10rad/s 反馈中止、温度/时效/周期/写入异常、总力矩量化后限幅、停止后新反馈确认、J5 不进入传输。没有改 TMAX、没有写 KT/OC/Flash、没有提高 0.30 上限、没有新增重试。

### 2.2 `slai_console/live_trial.py`

- `TrialRequest` 新增 `tier`（默认 `kp4_verified`）和 `tier_ack`（默认 False）。
- `build_session()`：解析档位；**非默认档必须 `tier_ack is True`**，否则在任何使能前报错；随后写日志事件
  `PD_TIER_SELECTED`（档位名、kp、kd、hardware_validated、explicit_tier_ack、is_default、0.30 上限），再按档位构造 J4 包络（J1–J3 仍 kp=4/kd=.5，且从不使能）。
- `run_trial()`：
  - `TRIAL_STARTED` 现在记录**实际** `candidate_host_kp/kd`、`pd_tier`、`pd_tier_hardware_validated`、`pd_tier_explicitly_selected`、`encoded_cap_nm`（不再硬编码 4/.5）。
  - 每个控制周期记录 J4 的 `output`（仅当该周期真的构建并写出帧；故障周期不会把上一周期力矩算到新误差上）和 16 位位置码。
  - 结束后附加 `motion_metrics`、`stop_reason_code`、`stop_summary`（中文一句话）、`next_step_hint`（下一步机械/负载检查建议）、`torque_cap_nm`、`encoded_torque_cap_nm`、`pd_tier/host_kp/host_kd`。
  - 写 `TRIAL_METRICS` 事件（结构化指标）和一条 `PHASE` 文本摘要；正常路径外若中途异常，`finally` 里在已有采样时补算一次，避免只剩一个异常字符串。
  - `reason` 保持既有稳定字符串（如 `Following error`），以免破坏已冻结的判据/日志消费者；分类信息放在 `stop_reason_code`/`stop_summary`/`motion_metrics`。例如本次真实日志会得到 `FOLLOWING_ERROR_BELOW_CAP_NO_MOTION`，摘要明确写“指令峰值 0.145Nm、占编码上限 52%、没有用满输出”。
- 界面：新增明确的主机 PD 档位单选组（默认“已验证档 Kp=4” / “较硬候选档 Kp=12（未实机验证）”），较硬档另有单独勾选确认；每次试验结束后自动回到默认已验证档并清空确认，防止档位/确认粘连；试验期间单选与确认禁用。标题说明更新为“总编码力矩≤0.30Nm（量化后实际可达 0.2769Nm）”。

### 2.3 测试

- `slai_console/test_actuation.py`：+6 项（档位/包络精确匹配、上限与档位不可扩张、Kp4 2° 小力矩、Kp12 更早输出并在 0.30 内饱和、全码值范围不超限、停止后无升档入口）。
- `slai_console/test_round_trip.py`：+5 项（静摩擦卡死 Kp4 报“未用满输出”、静摩擦高于上限 Kp12 报“到编码上限仍不动”、Kp12 真实移动往返仍须 ≥9.5°/≤0.5°、未确认较硬档不使能、未知档位不使能）。
- `slai_console/test_live_trial.py`：+2 项（默认档位在结果与生产回调日志中为 Kp4；较硬档先拒绝、确认后保持成功且只使能 J4）。
- 所有新回归走真实生产路径：`Reader`→`ControlSession`→`FourAxisController`→`make_event_writer` 真回调写 `StringIO`，并解析日志断言，不只测 lambda 替身。

### 2.4 明确未改

- `slai_console/actuation_io.py`、`src/slaihand_bringup/config/hardware_mapping.yaml`、URDF、`provisional_gravity.json`、已有 `tmp/` 实机日志、既有成功报告、桌面入口与 BAT、外部凭据：均未改。
- 未安装依赖、未改权限、未提交 git。

---

## 3. 离线测试命令与结果

任务书指定命令（原样）：

```text
C:/Users/ASUS/AppData/Local/Programs/Python/Python312/python.exe -m unittest slai_console.test_round_trip slai_console.test_live_trial slai_console.test_watchdog_check slai_console.test_actuation slai_console.test_actuation_io slai_console.test_commissioning slai_console.test_console slai_console.test_timing -q
```

结果：**Ran 90 tests … OK**（原基线 77 项 + 本轮新增 13 项；无失败/错误）。测试只用假串口/合成电机状态，没有打开物理端口。

环境说明：本会话的 DSH 工作区沙箱会拒绝写入 `tempfile.mkdtemp()` 创建的 0700 目录（既有 `test_live_trial`/`test_round_trip` 用它放合成报告），因此直接在沙箱内跑上述命令会在临时目录处 `PermissionError`，不是测试失败。我用两种方式复核了同一批模块：

1. 对上述**原命令**一次性放宽沙箱执行，得到 `Ran 90 tests … OK`；
2. 反复本地运行时使用 `tmp/j4_analysis/run_tests.py`，它只在创建临时目录时强制 0777，随后运行**完全相同的 8 个模块**。

Codex 在常规终端执行上面的原命令即可。本轮测试产生的空临时目录已清理，`tmp/live_trial`、`tmp/watchdog_check` 下的既有日志文件未改动。

---

## 4. 新增回归与任务 F 对照

| 任务 F 要求 | 覆盖测试 |
|---|---|
| 复现 Kp4 下 2° 误差只产生小力矩 | `test_actuation.test_kp4_at_two_degrees_requests_less_than_half_the_cap`（请求 0.139626Nm、编码 0.145055Nm、0 次到上限） |
| 新档位在相同 0.30 上限下更早给出足够请求、编码值不超限 | `test_stiff_tier_asks_more_early_and_saturates_without_exceeding_cap`；`test_encoded_torque_never_exceeds_cap_across_full_raw_range` |
| 静摩擦/卡住的假关节失败并报告正确原因 | `test_round_trip.test_kp4_stuck_joint_reports_below_cap_not_only_generic_following_error`；`test_stiff_tier_stuck_joint_reaches_encoded_cap_and_still_reports_no_motion` |
| 静止不能算往返成功 | `test_round_trip.test_stationary_support_cannot_count_as_successful_round_trip`（并断言 `moved=False`、位置码 0 变化、`stop_reason_code`） |
| 保持/拒绝 J5/取消/断线/故障/参数变化路径仍通过 | 原有 `test_actuation`/`test_actuation_io`/`test_live_trial`/`test_watchdog_check`/`test_round_trip` 全部保留并通过 |
| 生产回调而非 lambda 替身 | 新增与既有测试均解析 `make_event_writer` 写出的真实日志（`PD_TIER_SELECTED`、`TRIAL_STARTED`、`TRIAL_METRICS`） |
| 档位必须显式选择、不能自动升档 | `test_limited_ticket_tier_must_match_envelope_and_ceiling`；`test_stiff_tier_requires_ack_then_hold_records_kp12`；`test_stiffer_tier_requires_separate_ack_before_any_enable`；`test_no_method_escalates_gains_after_a_stop` |

真实失败日志的分类复核（只读重放，生产 `motion_summary`）：

```text
moved=false, stalled=true
position_codes=[0x7AAA], position_code_changes=0
peak_requested_nm=0.141082, peak_encoded_nm=0.145055
peak_fraction_of_cap=0.4835, peak_fraction_of_encoded_cap=0.5238
cycles_at_encoded_cap=0
max_error_deg=2.0074
assessment=COMMAND_BELOW_CAP_NO_MOTION
stop_reason_code=FOLLOWING_ERROR_BELOW_CAP_NO_MOTION
```

---

## 5. 风险与未决

1. **Kp=12 是未实机验证的候选档。** 按代码和冻结轨迹推算：静止卡死时它会在约 **1.32°** 误差（约 **4.96s**）达到编码上限 0.276923Nm，约 **1.43°**（约 5.12s）开始请求 0.30Nm 并被裁剪；之后保持上限约 **0.9s**，到 2° 仍会按原保护停止。因为跟随误差是“目标−实际”，停止时刻与增益无关；加硬档改变的是力矩，不是中止时间。
2. **0.30Nm 仍可能不够。** 本次 0.145Nm 不动，不能排除 0.30 也不动；若新档位仍“到编码上限不动”，下一步应是机械/负载检查，而不是继续升力或提高上限。软件不会自动升档/加力。
3. **指标分类是事实归类，不是根因证明。** `COMMAND_BELOW_CAP_NO_MOTION` 只说明“本次没有要求满输出”；`OUTPUT_AT_ENCODED_CAP_NO_MOTION` 只说明“到编码上限仍无编码器位移”。两者都不测外部负载。
4. **合成静摩擦值不是实机测量。** 测试里的 0.20Nm/0.40Nm 是夹具参数，仅用于验证分类与保护，不代表本机实际静摩擦或负载。
5. **量化与估计力矩。** 0.30 上限下可编码峰值 0.276923Nm；驱动反馈力矩是估计值，0.171429Nm 不能当作外部负载。报告同时列出请求值与编码值。
6. **保持路径也可选较硬档**（同样受 0.30 上限和 2° 保护），但它同样是未实机验证组合；本轮没有、也不会代替用户运行。
7. **工作区既有未提交改动**（`src/slaihand_bringup/config/hardware_mapping.yaml`、`verification/impedance_demo/model.py`）不是本轮产生的，未回滚、未修改。

---

## 6. 交付状态

- 软件：档位、日志、指标、离线回归已完成（90 项通过）。
- 硬件：**未接触**。没有打开 COM9/任何串口，没有查询/使能/试动电机，没有改参数/零位/URDF/日志。
- 真实 140405 失败仍判为失败：`round_trip_criterion_met=false`、`moved=false`，没有包装成运动成功。
- **等待 Codex 验收，用户尚未执行新档位实机试验。** 验收通过前，请不要据此运行较硬候选档；实机操作与支撑/切电条件由用户另行决定。
