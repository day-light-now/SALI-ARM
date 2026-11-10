# J5 阶段一：只读识别与手动软件标定

日期：2026-11-07  
分支：`feature/j5-integration-20261107`  
工作树：`D:/CodexData/Projects/机械臂/slaihand_j5_ws`  
基底：`e9bfb20`（包含 gravity-hold 锁定功能及三项复验修复）  
实现提交：**`b1ee05f`** `feat(j5): add read-only discovery and manual calibration web path`  
五项复验修复：**`8c0137c`** `fix(j5): enforce exclusive serial, boundary rejection, generation and identity binding`  
只读长连接故障修复：**`5cfdf57`** `fix(j5): release faulty serial and expose explicit read-only reconnect`  
生命周期/失败回执修复：**`a4755a4`** `fix(j5): serialize close/reconnect lifecycle and fail failed reconnects`  
完整提交帧写修复：**`2eaa973`** `fix(j5): write frames with complete-submission semantics and bounded EAGAIN`  
复验补充修复：**`888b678`** `fix(j5): bound positive partial writes and retain deferred discovery`  
模式回执同步修复：**`184740d`** `fix(gravity): reuse one mode callback token for dispatch and receipt`  
范围：**只读编码器/寄存器发现 + 网页手动标定链路**。未使能 J5、未发送 MIT/零位写入、未运动。

## 1. 现场事实与未决项

- J5 CAN ID **5** 已由 `verification/hardware/J5_识别记录_20261030.md` 记录保存并经断电保持验证；
- 电机型号项目记录为 **DM-J4340-2EC**，但型号寄存器尚未独立读取确认；
- PMAX/VMAX/TMAX、Master ID、mode、timeout 尚未由本轮真实 J5 反馈独立确认；
- 方向、软件零位、机械/线缆限位为空；
- 当前真实驱动 PID 2155 仍是四轴栈，只有 J1-J4 codec/反馈/数组，最近审计 OvR3Qa 无 J5 RX 帧；
- 因此阶段一不把任何 fake PTY 数值写入真实记录，也不冒充现场标定结果。

## 2. 代码复用

- `slai_console/j5_calibration.py`：原样复用 J5 手动标定记录数据类、校验和原子保存逻辑，复制到 bringup scripts 并安装。
- `slai_console/five_axis_monitor.py`：复用其 J5 寄存器集合（7/8/9/10/21/22/23）、USB2CAN 30 字节请求 framing、16 字节反馈流解析、重复寄存器/重复 CAN ID 冲突处理思路；在 `j5_readonly.py` 中裁剪为 J5-only 只读实现。
- `verification/hardware/feedback_decode.py`：复用 Damiao 8 字节反馈的 PMAX/VMAX/TMAX 编码语义（本地实现）。
- `slai_console/five_axis_config.py` / `five_axis_mit.py`：只作为后续运动实现的边界参考；**未**启用其 2 Nm 旧门槛、旧 J4 范围或 MIT 写入。
- 原四轴 control panel：复用 panel HTTP/token/lease/status/UI 轮询结构；J5 只作为可选只读控制器，未动 PID/四轴驱动。

## 3. 已实现

### 3.1 `src/slaihand_bringup/scripts/j5_readonly.py`

- J5-only 请求帧：只允许 **ID 5** 和只读寄存器 7/8/9/10/21/22/23；可构造的帧只有 `0x33` 寄存器读和 `0xCC` 反馈查询，没有 MIT/使能/零位写入函数。
- 16 字节反馈流解析；8 字节反馈 raw 位置/速度/力矩/状态/温度。
- 寄存器重复值冲突、反馈 CAN ID 与 master_id 不一致、非 J5 ID 忽略等诊断。
- 状态明确：
  - `model_declared="DM-J4340-2EC"`、`model_verified=false`；
  - `pmax/vmax/tmax` 只作为实测寄存器值；
  - `hardware_sign=null`、`zero_offset_rad=null`、`limits=null`；
  - `encoder_representation="single-turn raw motor position; no wrap expansion"`，不自动做整圈展开；
  - `authorized=false`、`motion_authorized=false`。
- `J5ManualCalibration` 严格顺序：
  1. `capture_zero`：确认驱动失能 + 机械支撑 + 新鲜反馈；
  2. `capture_direction`：零位后手动轻转约 10°（至少 5°），确认方向，推导 sign；
  3. `capture_limit(min/max)`：确认避开机械止挡/线缆限制；
  4. `save`：三个确认齐全才调用既有 `build_j5_calibration` 保存；
- 每次操作写审计 JSON；每次步骤的 sample 包含反馈 payload、时间、寄存器/profile、反馈时效、零位/方向/限位状态；没有零位/参数写命令。

### 3.2 网页

原四轴 control panel 新增 **J5 只读识别与手动标定** 卡片：

- 展示 CAN ID、声明型号（标为未独立核验）、PMAX/VMAX/TMAX、Master ID/mode/timeout、原始编码器/速度/力矩、状态/温度、反馈时效、零位/方向/限位状态、标定文件路径；
- 明确显示“J5 授权运动：否（阶段一只读）”、编码器表示和冲突；
- 按钮：采当前零位、确认正向、记录最小/最大端、保存标定；每一步有独立确认；
- 操作后只显示“已记录（无使能、无运动、无零位写入）”，直到保存完成才显示标定文件。

HTTP：

- `GET /api/j5/status`
- `POST /api/j5/calibration/zero`
- `POST /api/j5/calibration/direction`
- `POST /api/j5/calibration/limit`（`which=min|max`）
- `POST /api/j5/calibration/save`

仅当启动 panel 时显式传入 `--j5-port` 才打开 J5 只读串口；默认不打开第二串口，不并发真实总线。

## 4. 复验五项反例修复

1. **P1 串口排他**：`J5Reader` 先解析 `os.path.realpath`，打开后、清缓冲/发送前同时获取设备 fd 的 `flock(LOCK_EX|LOCK_NB)` 和内核 tty `TIOCEXCL`；占用失败立即关闭且不 reset/发送；同进程第二 reader、symlink 别名、native owner 先占/后占均被拒绝。新增 `test_j5_exclusive.py` 双 reader/native owner/别名反例。
2. **P1 单圈跨界**：`capture_direction` 对超过 60° 的 raw delta 返回 `SINGLE_TURN_BOUNDARY_OR_JUMP`，不把 6.25→0.14135 的 -6.10865 rad 当负方向；要求回到同一单圈表示分支重新采零/方向，不自动展开整圈。
3. **P2 重采依赖**：`capture_zero` 清空 direction/limits 并递增 generation；`capture_direction` 清空 limits；save 拒绝旧 generation direction；UI/status 的 saved 只对当前 generation 为真。HTTP 测试覆盖重采零后 direction/limits 为 null、saved=false。
4. **P2 身份冻结**：首次 zero 冻结 canonical 设备、session_id、CAN/master、mode/timeout、PMAX/VMAX/TMAX、首个 payload、样本时序；direction/limits 每步比对；断连/重启/寄存器变更或样本时序倒退均拒绝。最终记录新增 `calibration_generation`、`device_identity`、`decode_profile`，审计事件同 generation。
5. **P2 正常回执误判**：`control_panel._dispatch_gravity_mode` 只创建一个 token，保存与回调是同一对象；正常 immediate success、driver 失败、真实 dispatch 覆盖后的 old future 三组测试均走实际 dispatch 路径。J5 分支独立提交 `184740d`，gravity-hold 分支 `69ff94a`。

## 5. 只读长连接故障诊断与修复

现场 Codex 管理的只读页面出现两次 pyserial 写超时：

- 第一次约 4005 个请求后 `SerialTimeoutException: Write timeout`，RX/TX 停住；页面仍显示 connected=true / fresh=false，保留约 38 秒旧角度；
- 第二次 9 个请求后再次写超时；USB 仍 Attached，旧串口仍由页面进程持有；
- 重新打开串口短 probe 能读出新角度，说明设备仍在，坏的是会话/传输；
- 新增 baseline trace：70.76 s、1191 对 TX/RX；失败前最后一笔 `os.write` 返回 30/30（outq=1280），随后 pyserial 内部 select 49.84 ms 请求、50.059 ms 实际等待超时；
- 改用 `/tmp` 审计后原 `.05` 设置完整跑过 240 s（4531 TX/4530 RX，max_age .0443s）；USBmon 同轮 4531 次输出完成中位 1.5 ms、最大 43.467 ms、无 USB 错误。中途扩展坞拔插导致 14:44 整条 USB 消失，不能与 14:38 的超时混为同一事件。

根因（已证实的代码层机制）：

1. pyserial 3.5 `serial.write` 在最后一笔 `os.write` 已接受 30/30 后，仍执行一次可写 `select`；该 post-write 等待超时会抛 `SerialTimeoutException`，**不等于本帧未提交**；
2. 旧 `J5Controller._loop` 捕获任意异常后只 `_stop.set()` 并退出，**没有关闭 reader/serial**；
3. `J5Reader.snapshot()` 的 connected 仍来自最后一次反馈，因此页面保留旧 connected/角度，造成“仍在线”的假象。

修复（`8c0137c` 后的 transport 提交，以及当前 `J5FrameWriter`）：

- `J5Reader` 不再依赖 pyserial `write`，改为直接 `os.write` 的 `J5FrameWriter`：完整 30/30 立即返回，不做 post-write select；
- 短写继续同一帧剩余字节，最多到 5 ms deadline；无法完成时记 `PARTIAL_WRITE` 并关闭串口；
- EAGAIN 零字节接受只做有界等待，记 `eagain`；连续 3 次才 `WRITE_BACKPRESSURE`，不重发可能已发出的帧；
- 任何传输故障关闭串口、释放 flock/TIOCEXCL 和审计文件，`snapshot` 强制 connected=false、fresh=false、feedback=null、age=null；
- 任何传输故障关闭串口、释放 flock/TIOCEXCL 和审计文件，`snapshot` 强制 connected=false、fresh=false、feedback=null、age=null；
- `J5Controller` 故障时停止轮询并调用 `calibration.invalidate()`，生成新的 generation，清空零位/方向/限位，明确提示必须重采零位；
- 新增显式 `POST /api/j5/reconnect`，只恢复只读，不使能、不运动、不发送 MIT/参数写；重连使用新的 reader session/device identity，旧标定不延续；
- 页面新增“重连 J5 只读”按钮，并显示连接状态/会话/fault/失效原因；故障时不再展示旧角度为实时值。

PTY 注入验证：写超时、短写（7/30 字节）、读断开/恢复、设备仍存在、长时无硬件压力（10 秒 >150 帧，另 4005 请求计数压力）、故障后端口可重新打开、重连后 zero=false。证据：

- `evidence/j5_panel_transport_fault.json`
- `evidence/j5_browser_transport_fault.json`
- `evidence/j5_transport_fault.png`
- `evidence/j5_transport_reconnected.png`

现场只读事实（Codex 管理，仅记录，不构成本候选提交的自动化测试）：

- 实测 ID5/master0/mode1/TIMEOUT0/PMAX12.5/VMAX10/TMAX28，state0；
- 用户从负载侧看顺时针手转后，读数 `2.5389105058 → 2.9493781949 rad`（145.4688565→168.9869228deg），delta +23.518066deg，趋势+1；
- 当前页面/Codex 尚未保存正式完整标定；用户当前摆放想作零位，但本轮**没有真实编码器零位样本写入记录**，也不冒称已保存/已标定；
- 按用户指令暂停继续手动限位采集，先解决只读持续反馈。

## 6. 关闭/重连生命周期与失败回执修复

现场只读页面暂未更新，以下仅是候选代码修复。

- **关闭与重连并发**：新增 `J5Controller._lifecycle`，`close()` 与整个 `reconnect()`（关闭旧 reader、join 旧线程、打开新 reader、启动新线程）共用同一把生命周期锁。close 返回后不会再出现“关闭后又被重连重新打开串口/启动线程”。
- **两个重连并发**：第二个 reconnect 在生命周期锁后看到第一个已恢复的 reader，返回 `reconnected=false`，不会关闭/停止第一个连接；不存在“后一个失败拖停前一个但串口仍占用”。
- **重连失败回执**：`PanelBackend.j5_reconnect` 不再把 `ok=false` 覆盖成 HTTP 202/ok=true；失败返回 `409 J5_RECONNECT_FAILED`。网页 `j5Action` 同时检查响应 `ok`，失败显示错误，不再只按 HTTP 成功提示“已重连”。
- 测试：
  - `test_j5_lifecycle.py`：close 阻塞在 reconnect 打开阶段、两个并发 reconnect、close 后 reconnect 拒绝；
  - `test_j5_panel_transport.py`：注入 reopen 失败，断言 HTTP 409、ok=false；
  - `browser_j5_transport.py`：实际点击失败重连，断言提示错误且不含“已重连”，第二次成功后才显示重连并提示重采零。

### 现场 USB 写阻塞触发原因

离线解析 `~/.ros/slaihand/j5_readonly_20261107/readonly.jsonl`（不执行 probe、不碰串口）：

- 第一会话：4005 TX / 4005 RX，217.84 秒，约 18.4 请求/秒；最大事件间隔约 0.18 秒，失败前 TX/RX 仍成对、周期稳定；
- 第二会话：9 TX / 9 RX，0.532 秒后停止；
- 两段日志的最后事件都是成功 RX，**下一次写之前没有审计记录**；旧代码未记录异常，不能从日志区分“USB CDC 写端点停接收”与“驱动/主机 pyserial 超时”。

因此可以确定的是：**不是反馈读取先饿死导致的写阻塞**；旧代码把写超时当成线程结束而不释放串口，造成旧连接长期占用和假 online。USB/CDC 侧的最终触发原因仍需 Codex 在新候选页面上采集 `WRITE_TIMEOUT` 故障事件（新实现已记录 `frames_sent`、`last_rx_age`、`last_tx_age`、异常类型）与主机 USB 日志后才能定论，不能凭本次 PTY 注入冒充已查明。

## 7. pyserial 完整提交语义与帧写修复

### 已证实的 pyserial 行为

本机 pyserial 3.5 的 `serialposix.Serial.write` 在 `os.write` 返回后，即使数据已全部接受，仍会执行：

```python
abort, ready, _ = select.select([abort_pipe], [fd], [], timeout.time_left())
if not ready:
    raise SerialTimeoutException('Write timeout')
```

之后才执行 `d = d[n:]; tx_len -= n`。因此**最后一笔 `os.write` 已经返回 30/30 后，仍可能因等待下一次可写而抛 `SerialTimeoutException`**。

现场 20261107_145137 baseline trace 正是这一形态：

- 70.76 s、1191 对 TX/RX 后失败；
- 失败帧 `os.write n=30`，`outq=1280`；
- 紧接着 pyserial 内部 `select` 请求 49.84 ms，实际等待 50.059 ms，未 ready，抛 `Write timeout`。
- 这说明**不能仅凭该超时断言电机没有收到该帧**；完整提交后的等待失败不等于写入失败。

既有 C++ `ReadOnlyPort::write_serial`（`src/slaihand_transport/src/canusb_readonly.cpp:171`）是正确对照：5 ms 总 deadline，`sent == 30` 立即 `break` 返回，不等待 TX 队列 drain。

**仍未确诊的是 USB 侧触发原因**：为什么在该时刻输出队列保持非可写（outq=1280）尚无法从现有日志判断；可能涉及 USB CDC/驱动/调度。14:38 的写等待超时与 14:44 扩展坞拔插导致整条 USB 消失是两次独立事件，不能合并归因。改用 `/tmp` 审计后原 `.05` 配置跑过 240 s（4531 TX/4530 RX，max_age .0443s），USBmon 同轮中位输出完成 1.5 ms、最大 43.467 ms、无 USB 错误；非阻塞对照 48.8 s 出现反馈年龄 .2958 s，因此**不能宣称仅改 `write_timeout` 就修好**。本候选也不使用 pyserial 的写等待，改用完整提交即返回的帧写器；根因仍需新页面现场 USB 日志确认。

### 候选修复

新增 `J5FrameWriter`（`j5_readonly.py`）：

- 直接对 serial fd 做 `os.write`，**完整 30/30 立即返回**，不再调用 pyserial 的 post-write `select`；
- 短写：续写同一帧前检查**总 deadline**；正数短写不能让写机会无限续期。若到 deadline 仍未完成，返回 `partial`，准确报告 `written`/`waited_s` 并关闭串口；完整提交后不再等待；
- 零字节 EAGAIN：未提交的发现请求保留在 `setup` 中并移到队尾，让其他寄存器可继续发现；只有完整提交才消费该请求，部分提交绝不重发整帧；连续背压仍按有界阈值故障；
- EAGAIN、零字节接受：最多等待到 5 ms deadline，返回 `eagain`；不把“未提交帧”当已发送帧，也不重发旧帧；
- 连续 3 次零字节 EAGAIN 才进入 `WRITE_BACKPRESSURE` 故障，释放串口并要求显式只读重连；
- 不使用 `write_timeout=0` 的 pyserial 写循环（其 EAGAIN 路径可能忙循环）；`J5FrameWriter` 的 deadline 保证有界退出；
- 保留 250 ms 反馈时效门禁；反馈陈旧只标记 stale，不伪装成写故障；
- 不放宽故障释放、标定失效、只读重连语义。

### 测试

- `test_j5_frame_writer.py`：完整 30/30 返回后不调用 select；短写 7/30；**每次 1 字节、每次时钟 +4 ms 的连续正数短写在第 2 次后续写前被 deadline 截断（partial/2/0.008s，而非 30 次/120ms）**；EAGAIN 零字节有界；select 一直 ready 但 write 一直 EAGAIN 也在 5 ms 内退出。
- `test_j5_transport.py`：新增 `nonconsecutive_eagain_keeps_rid7_discovery`：rid7 的两次非连续 EAGAIN 被保留/重排，其他寄存器继续，最终 `master_id=16`、ready=true、setup 为空；另覆盖 `feedback_age_250ms_stale`、EAGAIN backpressure、短写、读取断开/恢复；长时 10 s/150+ 帧与 4005 请求压力保留。
- Codex 反例脚本 `repro_frame_writer_review.py` 现在第一例输出 partial/2/0.008s；第二例的 DeferMaster **对 rid7 每次均 EAGAIN**，因此按设计在保留请求后进入有界 `WRITE_BACKPRESSURE`，不再出现 setup 丢失后假 online；非连续两次 EAGAIN 的 ready 回归由上述 PTY 测试覆盖。
- 证据：`evidence/j5_frame_writer.json`、`evidence/j5_transport_injection.json`。

## 8. 验证（仅候选/隔离 PTY）

- J5 frame writer：PASS（完整 30/30 不等待、短写、EAGAIN 有界）
- J5 pure codec/register/conflict/五项反例测试：PASS
  - 单圈跨界负方向拒绝；
  - 重采零清依赖、重采方向清 limits；
  - PMAX/设备身份变化拒绝；
  - 最终记录含 generation/device_identity/decode_profile。
- J5 exclusive double-reader/native-owner/alias：PASS
  - 证据：`evidence/j5_exclusive.json`
- J5 frame writer：PASS
  - 完整 30/30 返回不等待；短写；EAGAIN/select ready 仍 5 ms 有界退出。
- J5 transport fault injection：PASS
  - EAGAIN backpressure/短写/读断开/设备仍存在/250ms 反馈陈旧/长时 normal/4005 请求压力；
  - 完整提交后不可写、短写 7/30、EAGAIN/select-ready 有界退出均有反例；
  - 故障后端口释放、可重开；
  - 证据命令：`python3 verification/hardware/j5_20261107/test_j5_transport.py`
- J5 lifecycle close/reconnect：PASS
  - 关闭不被并发 reconnect 重新打开；两个 reconnect 只有一个新 reader；
  - 证据：`evidence/j5_lifecycle.json`
- J5 panel HTTP transport fault：PASS
  - `GET /api/j5/status` 故障时 connected=false、fresh=false、feedback=null；
  - `POST /api/j5/reconnect` 后恢复只读，zero=false 需重采；
  - 注入 reopen 失败返回 HTTP 409/ok=false，不伪装成功；
  - 证据：`evidence/j5_panel_transport_fault.json`
- J5 PTY read-only/手动标定测试：PASS
  - fake PTY 反馈后 profile/raw 可见；
  - 零位、方向、限位、保存顺序；
  - 所有写入 payload 仅为 `0x33/0xCC` 读请求；
  - 证据：`evidence/j5_readonly_pty.json`
- J5 panel HTTP 测试：PASS
  - 真实 PanelBackend/PanelServer + fake PTY；
  - `GET /api/j5/status`、zero/direction/limit/save 均成功；
  - 重采零后 direction/limits 清空、saved=false；
  - 证据：`evidence/j5_panel_http.json`
- J5 浏览器实际点击：PASS
  - 页面加载 J5 卡片，原始编码器/声明型号/未授权状态可见；
  - 实际点击 zero/direction/min/max/save；
  - 证据：`evidence/j5_calibration_card.png`、`evidence/j5_browser.json`
- J5 浏览器 transport 故障/失败重连/重连：PASS
  - 故障提示、旧角度隐藏、重连按钮；
  - 失败重连显示错误且不含“已重连”；
  - 成功重连后零位待定；
  - 证据：`evidence/j5_browser_transport_fault.json`、
    `evidence/j5_transport_fault.png`、`evidence/j5_transport_reconnected.png`
- 既有 4-axis panel backend 回归：`11 tests OK`。
- gravity-hold 正常/失败/旧 future 真实 dispatch：`16 tests OK`（J5 分支与 gravity 分支各自独立提交）。

测试中 fake 使用的 PMAX/VMAX/TMAX 只是合成数据，不是实机值，也没有写入真实标定文件或现场参数。

## 9. 五轴运动仍待完成

本阶段**不包含**五轴运动控制。后续必须在现场/CodEx 确认以下数据后另行实现并验收：

1. J5 真实只读反馈帧、型号/PMAX/VMAX/TMAX、Master ID/mode/timeout；
2. 手动零位、正向、可用转角范围（含线缆）；
3. J5 法兰几何、安装变换、link5 质量/质心；
4. 新 3.5 kg 负载的 `load_id`、PROVISIONAL 质量档案、J5 几何和新重力模型；
5. transport/官方协议/driver/supervisor/ROS controller/limits/planner/feedback/runtime identity/锁定/示教/记录 schema 的五轴全链路；
6. 未核实前的 J5 MIT 写到总力矩包络、速度/限位、STOP/UNKNOWN 的完整测试。

在此之前 J5 保持 `motion_authorized=false`，不把 placeholder 五轴 URDF 当已验证几何，不借用 J3/J4 的电机/扭矩参数。

## 10. 边界

- 仅候选源码、隔离 domain/PTY、fake J5；未打开真实 J5 串口，未发送任何真实 CAN/MIT/零位写命令。
- 未修改真实驱动 PID2155、四轴部署、domain 0、现有部署参数、桌面入口。
- 现场真实 J5 位置当前无法从 PID2155 反馈保存；Codex 后续现场切换/连接由现场处理。
- 未做真实负载/重力拟合；旧 J1-J4 四轴重力模型不当作已适配 3.5 kg/J5 负载。

Ponytail 收尾：跳过真实 J5 读写、五轴运动与实机标定；风险是阶段一记录仍须现场 J5 反馈和人工确认才能产生，当前网页/PTY 证据不能替代现场标定。
