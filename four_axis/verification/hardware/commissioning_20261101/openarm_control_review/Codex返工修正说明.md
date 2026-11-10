# Codex 第一轮返工修正说明

状态：**三项必修已修正并补反例回归；等待 Codex 第二轮验收。用户尚未执行新档位实机试验。** 本轮未打开串口、未使能/试动电机、未改参数/零位/模型/原始日志，未提交 git。

对应验收文件：`Codex返工_第一轮.md`。原交付报告：`DeepSeek交付_OpenArm控制逻辑优化.md`（其不再成立的声明已在该报告中标 `[返工修正]` 并保留第一轮历史事实）。

本地只读复算脚本 `Codex返工反例复算.py` 用旧分支与当前 `shape_torque` 对比两个反例，输出与 Codex 给定的旧值一致：过零旧 `torque=-0.09, rate=10, release=true`、新 `-0.009, rate=1, reverse=true`；起动旧 `pre_slew=-1.28, torque=-1.04`、新 `-1.0, -1.0, limit=1.0`。脚本只读、无串口。

---

## 1. 必修1：反向过零不能借快减率反向加力

### 返工前（Codex 反例）

`j4_control.shape_torque(previous=+0.01, requested=-2, dt=.01, velocity=0, tracking=true, motion_confirmed=true)`：

- 旧实现把“过零/反向”整体当作 release，用 `slew_down=10Nm/s`；
- 结果 `torque=-0.09`、`rate_nm_s=10`、`release=true`；
- 从 +0.01 退出只需 0.001s，剩下的 0.009s 本应按 `slew_up=1Nm/s` 反向建力，却继续用 10Nm/s 反向加力。

### 修正

`j4_control.py` 新增 `asymmetric_slew(previous, target, dt, up, down)`，按段处理：

1. 同侧增大：`up`；同侧减小到零：`down`；
2. 异号：先用 `down` 用 `|previous|/down` 时间减到零；
3. 过零后：
   - 若 `dt <= |previous|/down`：只减到零，不反向；
   - 若 `dt >` 释放时间：剩余时间 `dt - |previous|/down` 内按 `up` 建立反向力矩，且不超过目标幅度；
4. 最终仍限到当前相上限（起动相为 `startup_cap`，确认后为档位上限）。

`shape_torque()` 现在返回 `rate_nm_s`（最后一段的速率）、`zero_crossed`、`reverse`、`release_time_s`、`build_time_s`，不再用 `release` 掩盖反向建力。生产路径 `actuation._torque_frame()` 调用的就是这个 `shape_torque()`（同一函数，无副本）；FD 仍直接绕过。

### 修正后

同一反例：

- `torque = -0.009`（1ms 减到零 + 9ms×1Nm/s）；
- `rate_nm_s = 1`、`zero_crossed=true`、`reverse=true`、`release=false`；
- 反向幅度不超过 `up*(dt-|previous|/down)`。

### 回归

`test_j4_control.py`：
- `test_zero_crossing_does_not_use_fast_release_for_reverse_torque`：双向过零、恰好到零、`dt` 小于/等于/大于释放时间、同侧增/减；断言各自速率与反向幅度；
- `test_production_torque_frame_reverse_is_up_limited`（`test_actuation.py`）：通过真实 `_torque_frame()` 路径复现 +0.01→约-2Nm 的反向请求，断言 `rate_nm_s=1`、输出在 -0.01 与 0 之间、编码 ≤ 档位上限。
- 既有 `test_release_is_much_faster_than_legacy_symmetric_slew` 仍通过：从 1.2Nm 减到 0 仍是 12 周期（0.12s），旧对称 120 周期（1.2s）；“减到零”仍快，“反向建力”单独受限。

---

## 2. 必修2：起动限幅不能被后置阻尼突破

### 返工前（Codex 反例）

profile=2Nm brake、`requested=-1, previous=-1, dt=.04, motion_confirmed=false, tracking=true, model_velocity=.19, reference_velocity=0`：

- 旧顺序：先起动限幅到 -1，再减阻尼 0.28，得到 `pre_slew=-1.28`；随后斜坡得到 `torque=-1.04`；
- 界面声称起动 ≤1Nm，但合法低于 0.2rad/s 中止阈值的输入使复合输出超过 1Nm；量化编码也没有按起动边界选码。

### 修正

- `shape_torque()`：起动相内先算 `limit=min(cap, startup_cap)`，请求限到 `±limit`，阻尼后再限到 `±limit`，斜坡后再限到 `±limit`；`pre_slew`、`torque`、`limit_nm` 都在起动边界内；确认运动后 `limit=cap`。
- `actuation._torque_frame()`：起动相用同一 `limit` 选 12 位编码值（`torque_raw_bounds(limit, tmax)`），并检查 `abs(encoded)<=limit`；编码后也不会越过起动边界。输出记录增加 `tier_cap_nm`、`startup_limit_nm`、`startup_capped`、`rate_nm_s`、`zero_crossed`、`reverse`。
- **motion_confirmed 改为每程锁存**：`self.motion_confirmed_leg` 在 `start()` 和每次 `trajectory()`（出程/返回各一次）重置；一旦某程达到 `motion_confirm_codes`，该程保持确认，位置回抖不会重新打开起动相。`MOTION_CONFIRMED` 事件记录一次。这样避免“起动相反复开关导致输出被反复夹到 1Nm”的另一种跳变。
- 总 2Nm 上限、0.20rad/s、4°/2°、0.02rad/s 等边界不变。

### 修正后

同一反例：

- `startup_capped=true`、`limit_nm=1.0`；
- `pre_slew=-1.0`、`torque=-1.0`、编码 ≤1.0（不再 -1.28/-1.04）；
- 双向、速度与位置误差反向、确认后可用到 2Nm 但不超过 2Nm。

### 回归

- `test_j4_control.py::test_startup_cap_bounds_brake_composite_in_both_directions`：上述反例、镜像方向、位置/速度反向、确认后 2Nm；
- `test_actuation.py::test_startup_phase_encoded_output_with_brake_is_within_cap`：真实 `tick`/`_torque_frame` 路径，模型速度 +0.19rad/s、未确认运动，断言 `pre_slew/limited/encoded` 均 ≤1Nm、`startup_limit_nm=1.0`；
- `test_actuation.py::test_motion_confirmation_latches_per_leg_and_resets`：确认后位置回抖不重新打开；新一程（返回）重置；
- 既有 `test_brake_tier_startup_cap_and_bounded_no_breakaway_stop` 仍通过：未突破 1Nm、6s 受控停止、四轴 FD。

---

## 3. 必修3：停止后采集不再把缓存叫首帧/缺帧当零滑行

### 返工前问题

- 旧 finally 才在 stop 确认后被动 `_drain()`；主 while 在 STOPPING 期间的 `session.cycle()` 已消费/覆盖第一帧 state0；
- 被动 drain 在请求-回复总线上不产生查询，所以不会自行持续有新帧；
- `post_stop['first']` 可能是缓存的最新帧而不是首帧；无新帧时 `travel_deg` 可能被算成 0，被误读为“零滑行”。

### 修正

1. **主循环内尽早采集**：每次 `session.cycle()` 后、`c.stop_time` 已设置且状态为 `STOPPING/DISABLED` 时，立即 `capture_post_frame(...,'stop_cycle')`；finally 再补 `stop_cycle_final`。所有可见帧按 `reader.state.at[4]` 去重并保留 receipt。
2. **可见性标注**：`post_stop['first_state0_guaranteed']=False`、`may_miss_frames=True`、`visibility='visible_only'`、`sources`、`state0_count`、`coverage_s`；明确“一次 drain 可覆盖同批更早帧，被动总线可能完全没有帧”。
3. **仅 DISABLED 后有限 CC 轮询 J4**：当且仅当 `c.state=='DISABLED'`（四轴新反馈确认失能）时，在主循环结束后 `POST_STOP_WINDOW_S=.30`、最多 32 次，用现有 `usb_packet(4)`（0x7FF，payload `04 00 CC 00 00 00 00 00`）只读查询 J4；每次写记录 `queries`（TX receipt）、`reader.event` 和 `reader.tx`，随后 `_drain()` 读回并 `capture_post_frame(...,'cc_poll')`。不发 MIT/FC，不写参数，不查询 J5，不改 `actuation_io.py`/Reader。
4. **UNKNOWN 不主动查询**：`c.state=='UNKNOWN'` 时只 `capture_post_frame(...,'passive_unknown')`，`active_query=False`、`unknown_lock=True`，避免用主动请求刷新可能仍使能电机的 watchdog；保持未知锁定和现场检查提示。
5. **缺帧/无新样本显式**：
   - 无可见帧：`insufficient=True`、`insufficient_reason='no_new_feedback'`；摘要“无可见新帧……不能判断滑行，不能声称0滑行”；
   - 只有 state1 无 state0：`no_new_state0_feedback`；
   - state0 少于 2 帧：`too_few_state0_frames`，`travel_deg=None`、`peak_speed_rad_s=None`（单帧不构成滑行证据）；
   - 覆盖小于 0.01s：`short_coverage`；
   - 不再把“没有新样本”算成 0 滑行。
6. **窗口末尾更新最终位置/判据**：对往返试验，用最后一个**新的 state0 可见帧**更新 `post_stop_final_error_deg`/`final_error_from_start_deg`；若其相对 anchor 超过 0.5°，`post_stop_invalidates_return=True` 且 `round_trip_criterion_met=False`，`stop_summary`/`POST_STOP_FEEDBACK` 记录“round-trip criterion revoked”。无 state0 时 `post_stop_final_error_deg=None` 并记录“未重新确认”，不伪造 0。
7. **测试 fake 生产路径**（`test_round_trip.py`）：
   - `QueryOnlyCoastBus`：停止后只有 J4 `0x7FF 04 00 CC` 查询才回一帧，且每次查询推进一个离散滑行码；证明只靠静态 state0/被动缓存过不了；
   - `CacheOverwriteBus`：一次回复两条 state 帧，Reader 只保留最后一条；断言 `first_state0_guaranteed=False` 且首可见不是被覆盖的第一条；
   - `NoDisableBus`：FD 后 J4 不回到 state0，stop 确认超时 UNKNOWN；断言 `observation.tx=0`、`unknown_lock=True`、没有主动 CC；
   - `QueryOnlyCoastBus(states0=False)`：无新 state0；断言 `insufficient`、`travel_deg=None`、`peak_speed_rad_s=None`、摘要“不能判断滑行/不能据此声称0滑行”；
   - `DriftOnQueryPlant`：往返成功、STOPPING 确认帧仍在前一位置；DISABLED 后 CC 轮询出现 >0.5° 的可见偏移，断言 `post_stop_invalidates_return`、`round_trip_criterion_met=False`、`post_stop_final_error_deg>0.5`、`post_stop_criterion_note` 含 revoked。
   - 另：`test_post_stop_query_only_discrete_coast_is_visible` 断言 `sources` 含 `cc_poll`、`queries[0].kind='cc_j4'`、payload 为 `04 00 cc...`、`state0_count>=2`。

### 保留

- 停止确认仍是四轴新反馈 state0 + `at>stop_time` + 新鲜度；UNKNOWN 锁定/现场检查提示不变；
- `actuation_io.py`/Reader 未改；
- CC 查询只在 DISABLED 后、只在 J4、只读、限时/限次数。

---

## 4. 原命令与 shim 结果（严格区分）

### 第一轮事实（保留）

Codex 第一轮运行原样 9 模块命令：**114 通过 + 2 个 `FileExistsError`（Reader 日志命名碰撞）**。此前交付报告写的“116 通过”依赖 `tmp/j4_analysis/run_tests.py` 的文件名/时间 shim；不能算原命令通过。

### 本轮修正

- 新增测试层 `slai_console/test_reader_fixture.py::make_reader()`：Reader 构造遇 `FileExistsError` 时用**真实短 sleep** 重试，直到拿到新的秒+微秒文件名；不改生产 Reader、不替换 `time.time_ns`、不改任何控制时效。
- `test_live_trial.py`、`test_round_trip.py`、`test_watchdog_check.py`、`test_actuation_io.py` 的 Reader 构造改用该 fixture。
- `tmp/j4_analysis/run_tests.py` 不再含任何时间 shim（仅保留 DSH 沙箱 tempfile 目录 0777 的本地辅助；不是交付命令）。

### 本轮结果（原命令，无时间 shim）

```text
C:/Users/ASUS/AppData/Local/Programs/Python/Python312/python.exe -m unittest slai_console.test_round_trip slai_console.test_live_trial slai_console.test_watchdog_check slai_console.test_actuation slai_console.test_actuation_io slai_console.test_commissioning slai_console.test_console slai_console.test_timing -q
```

→ **Ran 112 tests … OK**（原 96 + 本轮/返工在既有模块内新增 16；无 FileExistsError）。

```text
C:/Users/ASUS/AppData/Local/Programs/Python/Python312/python.exe -m unittest slai_console.test_round_trip slai_console.test_live_trial slai_console.test_watchdog_check slai_console.test_actuation slai_console.test_actuation_io slai_console.test_commissioning slai_console.test_console slai_console.test_timing slai_console.test_j4_control -q
```

→ **Ran 126 tests … OK**（112 + test_j4_control 14；无 FileExistsError）。

说明：两次原命令均在本会话当前环境直接运行；`test_reader_fixture` 是测试层重试，不是时间 shim，也不改变控制时序。`TemporaryDirectory(ignore_cleanup_errors=True)` 仅容错 Windows 清理时的文件占用，不跳过断言。

---

## 5. 保留边界与未改变项

| 项 | 状态 |
|---|---|
| 仅 J4 使能/J5 不查询 | 保持 |
| 总编码 | 保持 ≤0.30/1/2Nm；起动相额外 ≤0.5/1.0Nm 且编码后也 ≤ 该值 |
| 速度中止 | 保持 0.10/0.20rad/s；硬性超速单帧立即 STOPPING+四轴 FD，不被 shaper 延迟 |
| 跟随误差 | 2Nm 4°，1Nm 2°，其余不扩大 |
| 目标速度 | ≤0.02rad/s |
| 每程 10°/18s、整次 45s | 保持 |
| 温度/反馈年龄/主机周期/TIMEOUT2000/停止确认 | 保持 |
| Kp/Kd、档位命名/冻结票据/独立确认/结束恢复默认 | 保持；新增的只是两个 brake 候选档 |
| URDF/模型/原始证据/历史报告 | 未改 |
| 实机 | 未操作；等待 Codex 验收，用户尚未执行新档位试验 |

---

## 6. 风险与限制

1. 反向建力受 `slew_up` 限制，**制动的反向分量会比“快减到零”慢**；这是 Codex 要求的“不能用快减率反向加力”的直接代价。合成 plant 的既有结果（中静摩擦仍完成、较高静摩擦起动窗口停止）未变，但不等于实机可完成。
2. `post_stop` 的“可见帧”仍可能缺失同批更早帧；`first_state0_guaranteed=False`、`may_miss_frames=True`。没有新 state0 时只能报 `insufficient`，不能报 0 滑行。
3. CC 轮询只在 DISABLED 后；若 UNKNOWN 或串口写失败，不补发、不重试，报告 `tx_error`/`unknown_lock`。
4. 最终往返判据只在新可见 state0 证明 >0.5° 时撤销；没有新 state0 时保留原判据但明确“未重新确认”，不伪造。
5. 原生 MIT/750Hz 仍未用于生产；驱动反馈力矩是估计值。
6. 新测试依赖本机 `tmp/live_trial`/`tmp/four_axis_readonly` 的真实日志做只读回放；日志缺失时该回放测试会 skip（不伪造数据）。

**等待 Codex 第二轮验收；用户不要直接执行新档位。**
