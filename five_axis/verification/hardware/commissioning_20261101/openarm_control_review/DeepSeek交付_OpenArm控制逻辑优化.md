# DeepSeek 交付：OpenArm 参考与 J4 起动/制动优化

状态：**软件实现与离线验证完成；Codex 第一轮返工的三项必修已修正（见 `Codex返工修正说明.md`），等待 Codex 第二轮验收；用户尚未执行新的有界起动/阻尼候选档实机试验。**

> 标注 `[返工修正]` 的段落是 2026-11-01 第一轮返工后的更正；未标注处保留第一轮历史事实。

本轮只读官方参考与已有日志、编辑本项目允许的源码、运行离线测试。没有打开任何物理串口，没有查询/使能/试动电机，没有修改 RAM/Flash/TMAX/零位/模式/TIMEOUT，没有运行 BAT，没有改 git 索引或提交。原有 1Nm/2Nm 候选档保留，新档位默认不选、需独立确认。

设计决策记录在 [设计节点.md](设计节点.md)，官方对照在 [OpenArm对照与设计.md](OpenArm对照与设计.md)。

---

## 1. 真实日志复核（只读，不改日志）

复核对象：`tmp/live_trial/20261101_154949_b50554`（1Nm/0.1）、`155439_df9c6a`（1Nm/0.2）、`155927_e9ad73`（2Nm/0.2）及对应 `tmp/four_axis_readonly/*.jsonl`；原始数据未修改。

| 运行 | 峰值编码力矩 | 触发 | 停止前最后位移 | 保护类型 | 停止后首帧 |
|---|---|---|---|---|---|
| 1Nm/0.1 | 0.4352Nm（未到 0.989 上限） | 计划后 4.16s 触发（使能采样 6.12s），反馈速度 −0.1648rad/s | 4 个码值 = 0.0656°（OBSERVABLE_START） | 0.10rad/s 速度中止 | +10ms 速度 −0.2308rad/s，位置码 31701→31695 |
| 1Nm/0.2 | 0.9890Nm（到 1Nm 编码上限，21 周期/0.29s） | 计划后 5.94s（使能采样 7.92s），2.005° 跟随误差 | 3 个码值 = 0.0437°（**CODE_JITTER**） | 2° 跟随误差 | 停在 31693/31695，无明显后续位移 |
| 2Nm/0.2 | 1.2000Nm（未到 1.991 上限） | 计划后 6.19s 触发（使能采样 8.16s），反馈速度 −0.2088rad/s | 6 个码值 = 0.1311°（OBSERVABLE_START） | 0.20rad/s 速度中止 | +9ms 速度 −0.4725rad/s、位置码 31671；+47ms 31663，停止后再走约 0.44° |

可复核事实：

1. **三例都不是“命令没发出”**：使能状态为 1，反馈力矩随命令改变；但位置码只变化 2–6 个码值，最多 0.13°，远没到 10°。1Nm/0.2 的 0.044° 只能算量化抖动，旧指标却给了 `MOVED`。
2. **释放发生在 10ms 帧之间**：位置码一帧跳 6–9 个，同时反馈速度从 ±0.011/0.033 跳到 0.16–0.21；主机在这一帧才第一次看到超速，随即按原保护 FD。**因此“提前 0.2 边界内刹车”对这几例不成立**，除非降低释放前的力矩或改变起动策略。
3. **FD 后仍有运动**：2Nm 例停止后首帧 0.4725rad/s、再走约 0.44°；1Nm/0.1 例首帧 0.2308rad/s。停机后的运动是惯量/摩擦/负载的结果，本轮把它如实记录，不再只报保护前的 0.13°。
4. **对称斜坡的减力延迟在真实记录里没有被直接观测到**：三例在释放帧就 FD，记录里没有“释放后继续给力矩再减力”的命令序列。我们只能从最后记录的 1.2Nm 状态做**反事实回放**：旧对称 1Nm/s 从 1.2Nm 减到 0 需 120 个周期（1.2s），新有界快速减力需 12 个周期（0.12s）。这是算法性质，不是实机已发生的时序。
5. 合成 plant 还显示：**单纯把下降斜坡改快可能让力矩掉到动摩擦以下，造成粘滑反复、峰值速度反而更大**。所以本轮不把“对称斜坡延迟”说成唯一根因，也不靠单一快速下降通过。

解释边界：驱动反馈力矩是估计值，不是轴端测力；用户确认断电可手动转动，不能排除支撑、静摩擦、负载、弹性或机械问题。0.2rad/s 边界内的真实运动能力仍未证明。

---

## 2. 控制结构与代码改动

**PD 执行位置不变**：仍在主机（Kp=32/Kd=0.5 候选档），驱动 kp/kd=0，总编码力矩按所选档限幅。没有把原生 MIT 增益用于生产。

### 2.1 新增 `slai_console/j4_control.py`（纯函数，可离线复算）

- `CandidateControl`：冻结的有界控制参数——`slew_up_nm_s`、`slew_down_nm_s`、`startup_cap_nm`、`startup_window_s`、`motion_confirm_codes`、`brake_gain_nm_per_rad_s`、`brake_cap_nm`、`brake_deadband_rad_s`；全部有上下界校验。
- `shape_torque()`：每周期顺序为
  1. 总命令先限到档位 `torque_cap`；
  2. 起动相（跟踪中且未确认运动）限到 `startup_cap`；
  3. 有界速度误差阻尼：`|v|−max(|v_ref|,deadband)` 超 0 时，从命令中减去 `min(brake_cap, brake_gain*excess)`，方向与测量运动相反；只耗散，不增加沿运动方向的能量；
  4. 非对称斜坡：同侧增力按 `slew_up`，同侧减力/减到零按 `slew_down`；**过零后的反向建力按 `slew_up`**（不能借快减率反向加力）[返工修正]；
  5. 起动相每一步（请求、阻尼后、斜坡后）都限到 `startup_cap`，返回 `limit_nm`；确认运动后才用档位上限 [返工修正]。
  - 反向幅度不超过 `slew_up*(dt-|previous|/slew_down)`；返回 `zero_crossed/reverse/release_time_s/build_time_s`。

- `classify_motion()`：`NO_CODE_DATA / NO_CODE_CHANGE / CODE_JITTER / OBSERVABLE_START / PARTIAL_TRACKING / EFFECTIVE_TRACKING`；分辨率用现场 PMAX 计算（J4 0.02186°/码）；少于 3 个码值间隔（0.0656°）归为抖动。
- `native_mit_estimate()`：仅离线对照，计算驱动器内部 MIT 的总力矩估计；不进入生产。
- `position_code()`、`symmetric_slew()`：反馈码/旧对称斜坡，供生产与回放共用。

### 2.2 `slai_console/actuation.py`

- `HostPdTier` 增加 `control: CandidateControl | None`；新增两个**明确命名、冻结、非默认**候选档：
  - `kp32_1nm_brake_candidate`：1Nm，Kp=32/Kd=0.5，速度中止 0.10rad/s，跟随误差 2°，起动上限 0.5Nm/6s，快减 10Nm/s，阻尼 2Nm/(rad/s)/0.30Nm，死区 0.05rad/s；
  - `kp32_2nm_brake_candidate`：2Nm，Kp=32/Kd=0.5，速度中止 0.20rad/s，跟随误差 4°，起动上限 1.0Nm/6s，快减 10Nm/s，阻尼 2Nm/(rad/s)/0.40Nm，死区 0.05rad/s。
- 既有 `kp4_verified`、`kp12_candidate`、`kp32_1nm_candidate`、`kp32_2nm_candidate` **原样保留**（`slew_nm_s` 对称斜坡路径不变）。
- 控制器：`_torque_frame()` 对有 `control` 的档位走新 shaper；起动相用 `limit=min(tier cap,startup_cap)` 选 12 位编码并校验 `abs(encoded)<=limit`，阻尼和量化都不能突破起动边界 [返工修正]。`_motion_confirmed()` 按该段轨迹方向累计位移，达到 `motion_confirm_codes` 个码值才解除起动相；**结果按每程锁存**，位置回抖不会重新打开起动相，`start()`/每次 `trajectory()` 重置 [返工修正]。`tick()` 在轨迹仍有效且超过 `startup_window_s` 仍无确认位移时，抛 `Startup no breakaway ...`，走原全局停止/FD/新反馈确认路径；不自动延长/升档/重试。
- `_check()` 的速度/状态/温度/时效保护在 shaper 之前执行；**硬性超速立即 STOPPING+FD，不被阻尼或快速减力延迟**。
- `motion_summary()` 增加 `motion_class`、`output_class`；`moved` 只对 `OBSERVABLE_START` 及以上为真；0.044°/3 码归为 `CODE_JITTER`；成功判据不变。

### 2.3 `slai_console/live_trial.py`

- `TRIAL_STARTED` 记录实际档位/Kp/Kd/上/下斜坡/起动上限/窗口/确认码数/阻尼参数；`candidate_control` 写入结果。
- 触发保护帧 `trigger_frame`：状态、位置码、模型角、原始/模型速度、跟随误差、当时在用的编码/限幅命令、缓起/阻尼标记、原因。`_check` 类故障优先用**reader 当前故障帧**（`c.last_snapshot` 可能早一周期），速度中止的触发值因此是该帧真实读数。
- 停止后新反馈 `post_stop` [返工修正]：主循环 `session.cycle()` 后**立即**采集可见帧（不再等 finally），按 reader receipt 去重；字段含 state/code/角/原始与模型速度/力矩/来源，并标注 `first_state0_guaranteed=False`、`may_miss_frames=True`、`visibility=visible_only`、`state0_count`、`coverage_s`、`sources`。**仅当四轴新反馈确认 DISABLED** 后，才在 0.30s/最多 32 次窗口内用 `usb_packet(4)`（0x7FF，`04 00 CC`）只读轮询 J4，记录 `queries`(TX)/`tx`/`polls`/`cc_new_frames`；不发 MIT/FC、不写参数、不查 J5、不改 `actuation_io.py`。UNKNOWN 时只被动采集，`active_query=False`、`unknown_lock=True`，不主动查询。无新 state0 时 `insufficient/no_new_feedback`，`travel_deg/peak_speed=None`，摘要明确“不能判断滑行、不能声称0滑行”；单帧不足不伪造 0。窗口末尾用最后一个新 state0 更新最终位置；若证明返回位置 >0.5°，撤销 `round_trip_criterion_met` 并记录 revoked。
- `stop_reason_code` 增加 `STARTUP_NO_BREAKAWAY`、`FEEDBACK_SPEED_ABORT`、`FEEDBACK_TORQUE_ABORT`、`TRAJECTORY_ARRIVAL_TIMEOUT`、`USB_WRITE_DEADLINE`；`Following error` 按运动分类细分 `FOLLOWING_ERROR_CODE_JITTER` / `..._OBSERVABLE_START` / `..._AT_ENCODED_CAP_NO_MOTION` / `..._BELOW_CAP_NO_MOTION`。
- 中文摘要/下一步提示按运动分类改写：码值抖动、可观察起动、部分/有效跟随分别表述；起动窗口停止明确写“受控停止、不自动延长或升力”。
- UI：新增两个档位单选；说明改写；较硬档独立确认；每次结束恢复默认 `kp4_verified` 并清空确认。

### 2.4 明确未改

`slai_console/actuation_io.py`、`commissioning.py`、`core.py`、`four_axis_monitor.py`、`hardware_mapping.yaml`、URDF/MuJoCo、`provisional_gravity.json`、原始日志/结果、历史报告、桌面 BAT、外部凭据：均未改。未安装依赖、未提交/推送。

---

## 3. 保留保护逐项核对

| 任务书保留项 | 状态 |
|---|---|
| 仅 J4 候选可使能；J5 不查询/不使能/不控制 | 保持；`active_axes=(4,)`；`ACTIVE=(1,2,3,4)` |
| 首次目标=新鲜当前位置 | 保持；`start()` 冻结当前 `q` |
| 输出总编码命令绝对值≤所选档，最高 2Nm | 保持；`torque_raw_bounds()` + 档位 `torque_cap`，最高 2 |
| J4 速度中止最高 0.20rad/s | 保持；新档 0.10/0.20，不超 0.20 |
| 2Nm 档跟随误差最高 4°，其余档不扩大 | 保持；2Nm 4°，1Nm 2°，0.3/0.12 不变 |
| 目标速度≤0.02rad/s | 保持；`Envelope.speed=.02` |
| 每程 10°/18s、整次 45s | 保持；`trajectory()` 冻结目标/18s，`max_runtime=45` |
| 支撑/净空确认 | 保持；`build_session` 三确认 + 档位独立确认 |
| 温度 50°C、反馈 60ms、主机周期 40ms、TIMEOUT2000 预检、故障全停 | 保持；原 `_check`/预检未改 |
| FD 四轴/新反馈确认/未知状态锁定 | 保持；`stop()`/`observe_stop()` 未改 |
| 不得靠加速度阈值/连续多帧判定忽略真实超速 | 保持；速度 `_check` 单帧硬中止；shaper 在其后，不参与超速判定 |
| 减力/FD 不被加力上限阻滞 | 同侧减力/减到零用 `slew_down`（有界 10Nm/s），FD 完全绕过 shaper；**过零反向建力按 `slew_up`**，不再用快减率反向加力 [返工修正] |
| 不自动选档/升档/加力/延长 | 默认 kp4；UI 复位；`build_session` 要求明确档位+确认；起动窗口到点停止 |

---

## 4. 离线验证

### 4.1 命令与结果

[返工修正] 原命令与 shim 严格区分：Codex 第一轮原样 9 模块为 **114 通过 + 2 个 `FileExistsError`**；此前“116 通过”依赖文件名/时间 shim，不能算原命令通过；第一轮历史的 8 模块 104、9 模块 116 均依赖该 shim，作为历史事实保留。返工后新增测试层 `test_reader_fixture.py::make_reader()`（真实短 sleep 重试 Reader 构造，**不替换时钟**），并移除了本地 runner 的时间 shim；下列原命令已无 FileExistsError。

基线命令（任务书原样，8 个模块）：

```text
C:/Users/ASUS/AppData/Local/Programs/Python/Python312/python.exe -m unittest slai_console.test_round_trip slai_console.test_live_trial slai_console.test_watchdog_check slai_console.test_actuation slai_console.test_actuation_io slai_console.test_commissioning slai_console.test_console slai_console.test_timing -q
```

结果：**Ran 112 tests … OK**（原 96 + 本轮/返工在既有模块内新增 16；无失败/错误、无 FileExistsError）[返工修正]。

```text
C:/Users/ASUS/AppData/Local/Programs/Python/Python312/python.exe -m unittest slai_console.test_round_trip slai_console.test_live_trial slai_console.test_watchdog_check slai_console.test_actuation slai_console.test_actuation_io slai_console.test_commissioning slai_console.test_console slai_console.test_timing slai_console.test_j4_control -q
```

结果：**Ran 126 tests … OK**（112 + 新增模块 14 项；无 FileExistsError）[返工修正]。

**Windows Reader 文件名说明（不跳过）** [返工修正]：本机 `time.time_ns()` 分辨率约 15.6ms，`four_axis_monitor.Reader` 用“秒 + 微秒%1e6”命名日志，同一 tick 内多个 Reader 会偶发 `FileExistsError`（既有只读监视器的命名缺陷，保护代码未改）。现在由测试层 `test_reader_fixture.py::make_reader()` 用真实短 sleep 重试构造（不替换时钟、不改控制时序），原样 9 模块命令 126 项通过；不再使用时间 shim。`test_round_trip`/`test_live_trial` 的 `TemporaryDirectory` 仅加 `ignore_cleanup_errors=True` 容错 Windows 清理占用，不跳过断言。

### 4.2 新增测试（14 项 `test_j4_control` + 16 项既有模块）

- **斜坡/阻尼**：旧对称 1Nm/s 从 1.2→0 需 120 周期（1.2s）；新有界 10Nm/s 需 12 周期（0.12s）；2 周期下降 0.2Nm vs 0.02Nm；减力/制动只耗散，输出始终 ≤ 档位上限；起动上限只在未确认运动时生效；参数上下界不可绕过。**返工新增**：过零后反向建力按 `slew_up`（+0.01→-2 反例为 -0.009 而非 -0.09，`rate=1`）；起动相阻尼复合/量化编码不越 1Nm（反例 `pre_slew` 从 -1.28 改为 -1.0）；`motion_confirmed` 每程锁存/重置。
- **真实日志回放**：读 `155927_e9ad73` + `155917_340800.jsonl`，确认最后命令 1.2Nm、停止前反馈 −0.2088rad/s、停止后 state-0 首帧 |v|≥0.4rad/s（实测 0.4725）、相邻码值变化 8–20；用同一最后状态反事实回放旧/新减力周期 120 vs 12；用 `155439` 的 0.044°/3 码确认新分类是 `CODE_JITTER` 而非有效运动。
- **第一轮返工反例** [返工修正]：过零反向（+0.01→-2 得 -0.009、rate=1；旧 -0.09/rate=10）、双向/恰好过零/`dt` 小于等于大于释放时间、生产 `_torque_frame` 路径；起动相阻尼复合/编码不越 1Nm（旧 pre_slew=-1.28/torque=-1.04，现 -1.0/-1.0）、`motion_confirmed` 每程锁存/重置；停止后 `QueryOnlyCoastBus`（按 CC 查询才回离散滑行）、`CacheOverwriteBus`（首帧被覆盖）、`NoDisableBus`（UNKNOWN 不主动查询）、无 state0（insufficient/no 0 coast）、`DriftOnQueryPlant`（可见偏移 >0.5° 撤销往返判据）。
- **合成 plant**（粘滑 + 惯量 + 16 位位置/12 位速度量化 + 2 周期反馈延迟 + 有界抖动，非平滑理想 plant）：
  - 高静摩擦 fs=1.2Nm、J=0.15：旧档约 6.2s 释放、速度中止峰值 0.2088rad/s、位移 0.66°、停止后再走 0.20°；新档在 6s 起动窗口内未突破 1.0Nm，受控停止，峰值 0.033rad/s（量化底噪）、位移 0、停止后位移 0；报告 `STARTUP_NO_BREAKAWAY`。
  - 中等静摩擦 fs=0.9Nm、J=0.15：旧档速度中止、峰值 0.2088rad/s、位移 0.78°、停止后再走 0.41°；新档完成 18s、位移 8.72°、峰值 0.1648rad/s、停止后位移 0。
  - 低惯量 J=0.05 的同一 plant：新档仍在 0.2 边界触发中止——**报告为未稳健完成**，不自动放宽；机械/惯量待确认。
- **硬性超速**：新档在 |v|=0.201 时立即 STOPPING + 四轴 FD，故障帧不发 MIT；阻尼/快速减力不延误。
- **生产路径**：新档 hold 走 2s 保持/新反馈失能；round trip 平滑 plant 完成 10°/返回；起动窗口停止路径写 `trigger_frame`/`post_stop`/`STARTUP_NO_BREAKAWAY`；速度中止注入测试写 `FEEDBACK_SPEED_ABORT`、触发帧 0.25rad/s、停止后 state-0 帧；`PD_TIER_SELECTED`/`TRIAL_STARTED`/`TRIAL_METRICS`/`POST_STOP_FEEDBACK` 均由真实 `make_event_writer` 回调写出并被解析。
- **原路径**：保持、拒绝 J5、取消、断线、旧反馈、故障、参数变化、无确认不发运动、默认 Kp=4、编码输出 ≤ 档位上限、静止/抖动不算往返成功——全部保留通过。

### 4.3 原生 MIT 对比（离线）

`native_mit_estimate(kp=32, kd=.5, 误差 4°, 速度误差 0.2rad/s)` → 总估计 **2.33Nm > 2Nm**；把 `tau_ff` 限在 2Nm 内只约束前馈项，P/D 项由驱动器内部算，MIT 报文没有总限幅字段。结论：生产继续主机 PD + 总编码限幅；原生 MIT 只做评估，不改驱动增益。详见 [OpenArm对照与设计.md](OpenArm对照与设计.md)。

---

## 5. 风险、限制与未验证

1. **合成 plant 只是算法证据，不是硬件保证**；J=0.15、fs=0.9/1.2 是合成参数，不代表本机真实惯量/静摩擦/负载。
2. **新档位很保守**：2Nm 档起动上限 1.0Nm 低于 155927 观测到的约 1.2Nm 释放点；1Nm 档 0.5Nm 低于 155439 观测到的约 0.989Nm。它们可能只给出“受控停止 + 明确报告”，而不是完成 10°；这是刻意的安全取舍，不是运动成功。用户/Codex 若要继续，需要机械检查或另选已有 1Nm/2Nm 档（本轮未改）。
3. **1Nm 档的 2° 误差停止（约 5.9s）早于 6s 起动窗口**，所以 1Nm 卡死时更可能报 `Following error`/`BELOW_CAP_NO_MOTION`，而不是 `STARTUP_NO_BREAKAWAY`；两种都按“未建立有效运动”报告。
4. **真实记录没有直接观测到释放后的减力延迟**；120 vs 12 周期是最后状态的反事实回放，不能当作实机已发生的减力时序。
5. **0.2rad/s 边界内未在所有合成参数下完成**：低惯量 J=0.05 仍会中止；报告为机械/惯量待确认，未放宽阈值。
6. **驱动反馈力矩是估计值**，不是轴端测力；所有“反馈力矩”读数不能当外部负载。
7. **原生 MIT / 750Hz / CAN-FD** 未在本项目验证；不声称等价。URDF/模型限位不是已验收物理包络；看门狗实际失能力矩/时延仍未验证。
8. **Windows 测试环境**：Reader 日志命名碰撞由测试层 `test_reader_fixture.py::make_reader()` 用真实短 sleep 重试处理（不替换时钟、不改控制时序）；`TemporaryDirectory(ignore_cleanup_errors=True)` 只容错清理占用，不跳过断言。本地 `tmp/j4_analysis/run_tests.py` 仅保留 DSH 沙箱 tempfile 0777 辅助，不作为交付结果。产品代码未改。
9. **第一轮返工的三项取舍** [返工修正]：过零反向建力受 `slew_up` 限制，反向制动分量会比“快减到零”慢；这是不能借快减率反向加力的直接代价。`post_stop` 只是**可见帧**（不保证首帧、可能丢帧）；只有四轴新反馈确认 DISABLED 后才做有限 J4 CC 轮询，UNKNOWN 不主动查询；无新 state0 时只报 `insufficient`，不报 0 滑行；只有新可见 state0 证明返回位置 >0.5° 时才撤销往返判据，没有新证据时保留原判据但明确“未重新确认”。

---

## 6. Codex 验收要点

- `slai_console/j4_control.py` 的 shaper 顺序、减力/制动只耗散、边界；`CandidateControl` 校验；**过零反向按 `slew_up`**、起动相复合/编码不越 `startup_cap` [返工修正]。
- `actuation.py` 的两个新档位是否冻结、非默认、不能绕过上限；既有 1Nm/2Nm 行为是否保持；`tick()` 起动窗口是否只在轨迹仍有效时触发（本轮修过“轨迹恰好到达同一周期误触发”的缺陷）；`motion_confirmed` 每程锁存/重置 [返工修正]。
- `live_trial.py` 的触发帧/停止后帧是否真实、是否只读采集、`stop_reason_code`/`motion_class` 是否把 0.044° 正确归为抖动、成功判据是否未放宽；**停止后可见帧/CC 轮询/UNKNOWN 不查询/insufficient/窗口末尾撤销判据** [返工修正]。
- 是否保持仅 J4、J5 排除、0.30/1/2Nm 与 0.1/0.2rad/s、2°/4°、10°/18s/45s、FD/新反馈确认。
- 是否未改保护代码/日志/模型/映射；是否没有自动升档、自动加力、自动延长窗口。

**等待 Codex 第二轮验收；用户尚未执行新档位实机试验。** 验收通过前，不要据此运行新的有界起动/阻尼候选档；实机支撑、局部净空与现场切电条件由用户另行确认。
