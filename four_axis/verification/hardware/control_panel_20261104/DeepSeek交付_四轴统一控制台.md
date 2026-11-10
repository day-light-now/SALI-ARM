# 四轴 ROS 统一控制台 · DeepSeek 交付（2026-11-04）

状态：**软件实现、离线测试与隔离域真实 ROS/JTC 验证已完成；首轮验收返工已修正；等待 Codex 第二轮验收。验收前请勿据此操作实机。**
本轮未打开任何真实串口/CAN、未操作 ROS 域 0、未改受保护文件、未提交 git。真实驱动（WSL 内已运行 2 小时 14 分的 `/dev/serial/by-id/...` 栈）全程未被触碰。

## 0. 首轮验收返工（P1 + 两项测试问题）

Codex 首轮发现三项，逐条修正如下；**未放宽任何保护/阈值，未吞掉 UNKNOWN，未修改受保护文件**。

1. **P1 停止纪元未落实**：`_request_stop()` 现同时置停止门禁 `stop_locked`；`preview/execute/hold` 一进入即返回 409 `STOP_LOCKED`（与缓存是否为 stage 0 无关）；`_launch()` 在 Popen 前同一临界区再次检查门禁与停止纪元；停止路径不等待文件 I/O 或启动锁。解锁必须同时满足：手动 `reset()` 已受理（记录 R）**且**随后收到 `dynamic_at > R`、四轴一致 `control_state==0`、`feedback_valid=1` 的新反馈。仅 DISARMED、仅 reset 服务接受回复、复位前旧快照、停用状态都不能解锁；UNKNOWN 时门禁永久锁定且复位被拒。
   - Codex 的原复现脚本 `codex_stop_race_repro.py` **未改一行**，现为 GREEN：`stop=202, hold=409, spawn_count=0, ROS 回调 0`（`codex_stop_race_result.json`；原 RED 证据保留为 `codex_first_stop_race_result.json`）。
   - 新增生产回调反例 `test_control_panel_callbacks.py`（真实 `PanelBackend`+`PanelServer`，Popen 打桩，无 ROS spin）11 项：stop→hold/execute/preview 全部 409 且 Popen=0、复位回复不解锁、陈旧/不一致/无效四轴不解锁、新鲜一致才解锁、UNKNOWN 不可复位、Popen 启动窗口内停止会取消并锁定、二次停止重新锁定。
2. **原样端到端 100 checks FAIL**：定位为**测试夹具污染**，不是面板或驱动阈值问题。失败的旧 fake 栈会留下孤儿 `robot_state_publisher`，它以 TRANSIENT_LOCAL 在隔离域上继续发布**旧的 URDF（含已关闭的 PTY 路径）**；新 `controller_manager` 先收到陈旧描述，驱动去打开已消失的 `/dev/pts/N` → `Cannot open CANUSB serial port` → 驱动 abort → 夹具启动失败。修正：每个场景独立新建 PTY/fake 栈/面板；进程带唯一 `SLAIHAND_PANEL_TEST_SESSION` 标记，回收只按本实例 PID/PGID/标记；新场景启动前等待域内无 `/slaihand/robot_description` 发布者；测试进程显式关闭子进程 stdout 管道，避免 GC 迟到关闭复用 fd。**未改驱动、未改阈值、未把 UNKNOWN 当通过**：停止场景如实接受 `confirmed` 或 `unknown`，真 UNKNOWN 时验证门禁锁定且复位被拒，其余场景用全新 fake 栈继续执行。
3. **测试脚本广泛杀进程**：`run_browser_fixture.sh`/`stop_browser_fixture.sh` 已去掉 `pkill`；fixture 自写 PID/PGID 状态文件，停止脚本校验 `/proc/<pid>/cmdline` 含 `browser_fixture.py`、环境含 `ROS_DOMAIN_ID=176`、PGID 匹配后才终止本实例。`smoke_panel.sh` 改为只杀自己 `$!` 的 PID。清理脚本只处理隔离域 176/177 上的 ROS 进程（先核对 `/proc/<pid>/environ` 与命令行），并打印真实串口栈仍存活。

本轮重跑结果（全部在隔离域 176/177 + 新建 PTY）：离线 23/23（Windows 与 WSL 各一次）、原规划 12/12、生产回调反例 11/11、端到端 **119/119 PASS**（5 个独立场景会话，**连续两次 PASS**；第二次无夹具重试、全程 84 秒）、浏览器 **33/33 PASS**（含 UI 停止门禁与复位解锁）。`codex_first_*` 原始证据未改动。

## 0.1 第三轮：0.5 rad/s 基线、UI 诊断与调参提案（2026-11-04 15:2x）

1. **“无样式/状态不加载”未能在部署页面上复现**：真实运行的面板 `/`、`/panel.css`、`/panel.js` 均 200，MIME 分别为 `text/html`、`text/css`、`text/javascript`，内容正确；Edge（headless + CDP）实测 1 个样式表/72 条 CSS 规则、`body` 背景已应用、停止按钮 168px、四轴行 4、`反馈：在线`/`阶段：未使能`/`心跳：正常`、无未捕获异常与 CSP/脚本错误，截图保留在 `evidence/edge_ui_diagnose.png`。结论：部署页面本身正常；用户看到的无样式页面应来自“非直连 URL”的查看方式（如把 HTML 存成文件/预览器不加载子资源，或未带 `?token=` 时得到 403 JSON）。需要用户提供当时地址栏的完整地址以定位查看器，而不是改页面。
2. **速度提示改为跟随受保护规划器**：面板原先把 “0.02 rad/s” 写死在 HTML，0.5 基线切换后必然失真。现由 `control_panel_model.REFERENCE_SPEED_RAD_S`（从 `joint_waypoint_plan` 读取）经 `/api/status` 下发，页面动态显示（本轮 0.5）；新增浏览器断言“页面峰值速度上限与规划器一致”。
3. **测试下层切到 0.5 工作空间**：旧下层 `slaihand_canusb_motion_ws_20261104_kpMgtx` 的硬件库把 `reference_speed` 上限校验为 0.02，0.5 策略会被驱动以 `Motion policy widens B1 scope` 拒绝（实测）。`stage_workspace.sh`、`run_verification.sh`、`run_browser_fixture.sh`、`smoke_panel.sh` 的下层改为 `/home/xuyu/slaihand_speed05_ws_20261104/install/setup.bash`，候选仍只构建 `slaihand_bringup`。重跑：生产回调 11/11、端到端 **119/119 PASS**（5 个独立会话）、浏览器 **34/34 PASS**（含 UI 停止门禁与速度上限断言）。
4. **手动调参页的驱动接口尚不存在（阻塞项）**：驱动只创建 arm/stop/reset 三个服务，Kp/Kd 在 `on_init` 由 URDF 硬件参数读入后运行期不可变，没有节点参数或增益话题；`src/slaihand_hardware/` 属受保护文件，本轮未改。按“必须接入真实控制链路、不能只保存表单”的要求，已写 `调参接口_设计节点.md` 提出最小契约（推荐 `std_msgs/Float64MultiArray` 请求/确认话题，或 Codex 更倾向的自定义 `SetGains` 服务），并写明门禁（仅四轴新反馈失能且静止时可应用、运动中拒绝、范围 0<Kp≤160、0<Kd≤5、不写 Flash、只改内存增益、审计记录）。**待 Codex 答复契约形式与驱动侧实现归属后**，面板侧 UI/校验/存档/应用记录与三类测试即可完成；在此之前不提交只能保存表单的“假调参”。

## 0.2 第四轮：重力补偿 A 阶段（模型 + shadow）— 2026-11-05

Codex 审阅（`Codex审阅_重力补偿.md`）接受“重力前馈 + 摩擦补偿”路线并要求先核查链路、再做模型/shadow。本节为 A 阶段结果。

1. **接入核查**：`integration_notes.md` 按 Codex 第 2 节逐项列出实际路径/函数/证据：唯一写入口 `CanUsbSystem::send()`；MIT 字段 `{kp,kd,q,dq,tau}` 且 `tau` 恒 0；`v_des` 恒 0 使内部 D 项为 `-kd·v`；`effort_estimate` 来自驱动反馈帧；力矩量化 J1/J2 0.0977、J3/J4 0.0264 N·m/码；J3 `+π` 只在 MuJoCo 模型、URDF origin 已含 π；门禁公式与停止行为；速率 100 Hz、WSL 无 RT。
2. **生产模型**：新增 `src/slaihand_bringup/scripts/control_panel_gravity.py`（纯标准库，不依赖 ROS/numpy）：四轴解析重力模型（URDF + 质量档案 + 显式重力向量 + 显式 J3 固定偏置）、唯一坐标映射（`q_model=sign·(raw−zero)`、`τ_protocol=sign·τ_model`）、策略读取、shadow 估算（含包络/门禁余量/警告）；越限拒绝不裁剪。
3. **配置**：新增 `src/slaihand_bringup/config/gravity_provisional.json`（保留原审计文件不动，附来源、状态、单位、未标定说明），随 config 目录安装。
4. **面板**：新增“重力补偿（shadow 估算）”卡片：逐轴模型重力矩、电机侧力矩、PD 分项估计、包络、门禁、余量；标注“模式 shadow（只显示）/前馈已应用：否/未写入命令”；显示后端、URDF 哈希、质量档案状态、总质量、重力方向、J3 偏置、kp/kd 与门禁值。影子计算失败只禁用显示，不阻塞面板。
5. **验证**：新增 `test_gravity_model.py` 14 项（单摆解析、势能差分、与现有解析模型逐位一致、J3 偏置只应用一次、质量线性、零重力、坐标/力矩映射、shadow 字段与余量、CLI 保护）全部通过；离线 23、生产回调 11、端到端 119、浏览器 37（新增 3 项 gravity 断言）全部通过。
6. **测试环境修正（重要）**：发现本机多个工作空间的 `libslaihand_hardware.so` 通过 `LD_LIBRARY_PATH` 互相抢占，`slaihand_speed05_ws_20261104` 里安装的硬件库实际是旧版（仍拒绝 0.5 策略，报 `Motion policy widens B1 scope`）。已把候选工作空间改为**自包含**（从仓库源码构建 bringup/description/transport/hardware 四个包），并新增 `test_env.sh` 清除继承的工作空间变量后只 source ROS + 候选；`stage_workspace.sh`/`run_verification.sh`/`run_browser_fixture.sh`/`smoke_panel.sh` 同步。重跑全部通过。**这也提示实机启动应固定单一工作空间，避免运行库与工作空间不一致。**
7. 本阶段没有任何命令写入：面板不发布电机命令，驱动未改，实机未动。A 阶段下一步（B）是驱动 `tau` 前馈最小补丁与 mock/PTY 验证。

## 0.3 第五轮：重力前馈 B 阶段（驱动接入）— 2026-11-05

按 Codex 审阅的 B 阶段要求实施，`alpha` 默认 0；未启用时对外行为与旧版一致。

1. **驱动前馈通路**：`native_codec.hpp` 的 `native_command(..., tau=0)` 增加力矩字段（校验 `|tau| ≤ tMax`）；`motion_supervisor` 每周期用实测模型角计算 `tau_ff_req = ramp_scale·alpha·g(q)`，按 `guard − |PD 估计|` 饱和后写回 MIT 帧，`tau_protocol = sign·tau_model`；既有请求/反馈估计门禁保持原公式与阈值。
2. **C++ 重力模型**：`gravity_ff.hpp/.cpp` 从硬件参数里的紧凑文本解析 G/L/J 记录，纯 C++17、无 ROS/无文件访问；launch 由 URDF + `gravity_provisional.json` + policy 的重力向量/J3 偏置生成文本，经 xacro 注入。C++ 与 Python 生产模型在 30 姿态 × 4 配置下交叉核对，最大差 1.8e-15。
3. **参数与发布**：`MotionPolicy` 增加 `gravity_alpha`（0..1）与 `gravity_ramp`（≥0）；硬件参数可选 `gravity_ff_alpha`（缺省 0）、`gravity_ff_ramp_s`、`gravity_model`；驱动发布 `/slaihand/gravity_ff_state`（`[alpha, ramp, requested×4, applied×4, stage, command_id]`，best-effort，1 Hz 心跳/α>0 时每周期）；面板订阅并显示驱动侧 α 与应用值。
4. **配置**：policy 增加 `gravity_ff_alpha: 0.0`、`gravity_ff_ramp_s: 1.0`、`gravity_base_{x,y,z}`、`gravity_j3_offset_rad`（现值仍为文档默认 +Y 向下与 +π，待实机核验）。
5. **两个实测发现（重要）**：
   - `ReadOnlyState` 的反馈已在 URDF 模型系（`sign·(raw−zero)`），前馈计算**不能**再次映射；此错误曾导致 J3 报“关节超限”并触发停止，已修正并纳入测试覆盖。
   - 12 位对称力矩字段无精确零码：0 编码为 2048、解码为 +半步（J1/J2 0.0488 N·m、J3/J4 0.0132 N·m）。这是既有 OpenArm 编码量化，不是前馈值；E2E 断言按“≤半步”判定。
6. **验证（全绿）**：C++ 重力模型单测（解析/单摆/势能差分/偏置仅一次/错误拒绝）；C++↔Python 交叉核对；生产回调 11；端到端 119（α=0，含前馈为零断言）；浏览器 38（新增驱动侧前馈字段断言）；驱动前馈 PTY 场景 α=0.55：解码力矩 `[0.830, 1.123, −0.040, −0.145] N·m`，与模型差 ≤0.027（一个量化步），客户端 exit 0、无 PTY 编码错误。
7. **未做/待定**：Kp/Kd 运行期调参接口仍未实现（需与 Codex 定的增益/前馈接口）；质量/重心标定、恒速摩擦扫描客户端、0.25 rad/s 下 J3/J4 门禁余量评审属 C/D 阶段。实机未动，`alpha` 保持 0。

## 0.4 第五轮返工：B 阶段三项 P1 修复（2026-11-05）

Codex 独立复验发现三项 P1，已全部修复并用其反例原样复跑确认（详见 `B阶段返工修复说明.md`）：

1. **前馈重编码丢失边界修正**：改用 `command_target` 保存修正后的模型目标，前馈重编码复用它；最终报文再次验证位置码在完整 URDF 区间内并做含 tau 的完整门禁。反例 `boundary_crossings=0`，单测覆盖四轴上下限/双向 sign/非零与饱和前馈。
2. **前馈斜坡未生效**：改用独立的 `gravity_ff_ramp`（从使能确认起算，0=立即），增益斜坡行为不变；反例 2.2 s 时 ramp=1 → -0.2442、ramp=10 → -0.04884。
3. **已应用状态不代表实际写入**：分离 requested/candidate/encoded/applied，`applied` 仅按每轴 `written` 回执更新；stop/reset/arm 清零；`active` 只在 PRELOAD..TRACK 且 α>0 时为真；话题扩为 21 值并带历史最后写入；面板按活动状态与时效显示。反例 `after_stop stage=8 active=0 ff_applied=0`。
4. **附带的量化修正**：发现 OpenArm codec 的 `double_to_uint` 向 `-tMax` 截断（tau=0 → 2047 → -0.04884 N·m），前馈发送前加半码取最近可表示值；α=0 字节不变。

复验：C++ supervisor（含新增断言）、重力模型 7/7、交叉核对、回调 11、端到端 119、浏览器 38、前馈 PTY α=0.55 全部通过；加载库 sha256 `3d6ef15d…`。`alpha` 仍为 0，未用于实机。

## 0.5 第六轮：A 遗留修正与 C 阶段只读标定工具（2026-11-05）

依据 `Codex复验_B阶段与C阶段任务.md`，本轮先清 A 阶段两项，再交 C 阶段软件与预览；`alpha` 保持 0，未操作实机。

1. **shadow 余量语义**：控制器参考未知时，P 项、已应用余量、预测余量全部显示“未知”，不再按零误差填充；把“已应用余量”（P+D+驱动实际写入前馈）与“假设补偿预测”（P+D+α_pred·模型重力矩，α_pred=1，明确标注未写入命令）分开命名与展示；面板新增 P 项/D 项/已写入前馈/已应用余量/假设补偿预测/预测余量列。`test_gravity_model.py` 15 项通过。
2. **J3 额外 π 的统一坐标链证明**：`j3_offset_proof.py` 全部 PASS——URDF J3 origin rpy 已含 π（部署 expanded URDF 相同、无额外 π）；MuJoCo 每个 body 为相对放置，`link1/2/4` 与 URDF origin 完全一致，`link3` 恰为 URDF origin 再乘一次 `R_axis(π)`（相差 180.000000°）；2π 与无偏置等价；重力模型偏置与放置声明同源；安装声明（+Y 向下、J3 180°、无末端负载）与 policy 配置一致。给出可判别姿态表：J1=J2=0、J3=0、J4=±30° 时 J2 预测反号（Δ≈0.37/0.40 N·m），作为现场判别。物理 180° 仍需现场测量，离线不能替代。
3. **C 阶段只读工具**：`calibration_core.py`（schema/校验/排除/残差分析/全局尺度/留出/可辨识性，状态仅 `PROVISIONAL_SCALE`/`INSUFFICIENT`，永不输出 `CALIBRATED`）；`calibration_capture.py`（`--record` 只读订阅采样、`--import` 校验、`--preview` + `--save-report` 分析报告与原始数据分开；原始 JSONL 写后只读）；`现场步骤_C阶段.md`（A 判别 2 条 → B 自由静态 ≥12 条 → 导入/预览 → 交 Codex，明确有支撑/接触单独标、不撞限位、不改 alpha）。
4. **假数据反例**：`test_calibration_core.py` 19 项（无效/时效/反馈无效/支撑接触排除/样本不足/多样性不足/模型异常/尺度回收/留出/可辨识性/`CALIBRATED` 永不出现/坏行拒绝）；`fake_calibration_check.py` 全链路 import→preview→报告；`j3_offset_proof.py` 链与姿态表。`run_verification.sh` 新增 `j3/calib/cli` 状态位。
5. **边界**：未生成质量档案/`CALIBRATED`，未改 `alpha`、Kp/Kd；面板暂无标定页（预览为 CLI 只读）；点到点旧 2°/6 秒项仍是独立未完成项。

## 0.6 第六轮返工：C 阶段采集/拟合门槛（2026-11-05）

依据 `Codex验收_C阶段返工.md`，修复三项 P1 与操作单/证据保护；Codex 反例原样复跑后所有情形均为 `INSUFFICIENT`、缓存快照返回 `None`（详见 `C阶段返工修复说明.md`）。

1. **纯静态资格**：schema 升版，必填 stage/四轴 motor_state/持续静止窗口（样本数、跨度、最大速度、位置变化、使能状态）/capture_age_s/load_id/runtime/driver_ff；只有“使能保持 + 自由无接触 + 窗口稳定 + 前馈 α=0 + 运行时身份已验证”才进入拟合；失能样本标几何检查、永不进入力矩拟合；负/零尺度、符号一致率低、留出失败均 `INSUFFICIENT` 并写原因。可辨识性措辞改为“本工具主动只做全局尺度（范围选择）”。
2. **新鲜度**：只用同一帧 DynamicJointState 取位置/速度/力矩；`capture_age_s` 与驱动 age 取最大值 ≤0.06 s，缓存 0.20 s 返回 `None`；JointState 仅做交叉核对（>0.01 rad 拒绝）；重复回执不重复计数。
3. **版本/来源**：样本间与“当前分析模型”的哈希/重力/J3/负载身份必须一致，混合版本拒绝；运行时身份当前为 UNKNOWN（驱动无该接口），该批只作诊断、不作为可信驱动标定。
4. **操作单/证据**：逐姿态 Enter 触发窗口采集（不再自动计时）；失能手掰不能取保持力矩；未知支撑不得默认 free；原始文件独占创建、唯一命名、异常也收尾为 0444；报告路径不得与 raw 相同（含符号链接）；明确 chmod 0444 不是篡改证明。
5. **验证**：`test_calibration_core.py` 35/35；CLI 假数据（验证身份→PROVISIONAL_SCALE、UNKNOWN 身份→INSUFFICIENT、同路径报告拒绝、唯一 raw）；全套 `PASS (callbacks=0 e2e=0 model=0 cross=0 ff=0 j3=0 calib=0 cli=0)`；`alpha` 保持 0，未采集现场数据。

## 0.7 第七轮：C 阶段剩余边界与运行身份接口（2026-11-05）

依据 `Codex复验_C阶段与运行身份接口.md`，三项边界修复并实现驱动运行身份：

1. **年龄相加**：每轴 `total_age_s = feedback_age + local_receipt_age`，任一超 0.06 s 或非有限即拒绝；`capture_age_s` 取每轴总年龄最大值并校验一致，三个字段分开保存；传输时间不可知时注明为下界。反例 .035+.035 现在被拒。
2. **窗口逐帧检查**：`_window_aggregate` 对每帧检查使能 stage/motor_state、新鲜度、前馈 α=0 与新鲜、命令 ID 一致，并记录 `all_enabled/fresh_all/ff_all_zero/ff_all_fresh/command_ids/max_gap_s/valid/reasons`；`motor_state` 取逐轴最小值；命令 ID 变化丢弃窗口重采；采集间隙 >0.3 s 拒绝。
3. **前馈时效/阶段/命令**：`driver_ff` 记录 alpha/ramp/active/applied/stage/command_id/age_s；拟合要求存在、age≤1.5 s（α=0 时 1 Hz 刷新留余量）、alpha=0、stage 与采样一致、command_id 属于窗口唯一命令、逐帧一致；过期反例（120 s）现在 INSUFFICIENT。
4. **运行身份接口**：驱动从实际接受参数生成 JSON 身份（schema/boot_id/started_at/mapping/model/policy/mass-profile 哈希/policy_profile/load_id/gravity/J3/kp/kd/alpha/ramp），发布 `/slaihand/runtime_identity`（RELIABLE+TRANSIENT_LOCAL）并每 2 s 重发同 boot_id 证明活性；xacro/launch/policy 增加 mass_profile_sha256、load_id、gravity/J3 参数；采集工具核对新鲜度与内容，verified 仅表示数据与运行配置匹配（不改变质量状态）。
5. **验证**：标定测试 34/34；CLI 假数据（验证身份→PROVISIONAL、前馈过期/UNKNOWN→INSUFFICIENT、同路径拒绝、raw 唯一名）；新增 `verify_runtime_identity.py` 在隔离域确认 latched、哈希/负载/重力/J3/kp/kd/alpha/ramp 一致与周期刷新（PASS）；全套新增 `identity` 状态位。

## 1. 交付物与改动文件

新增：
- `src/slaihand_bringup/scripts/control_panel.py`（入口：ROS 后端、状态快照、**停止门禁**、心跳租约、受保护客户端子进程、生命周期）
- `src/slaihand_bringup/scripts/control_panel_model.py`（纯逻辑：计划校验/冻结、预算、票根、限位、结果投影）
- `src/slaihand_bringup/scripts/control_panel_http.py`（生产 HTTP 回调：token/同源/Host/大小/路由/响应语义）
- `src/slaihand_bringup/web/control_panel/{index.html,panel.css,panel.js}`（中文单页，无外网字体/CDN，CSP 无内联脚本）
- `verification/hardware/control_panel_20261104/`：`设计节点.md`、本报告、`test_control_panel_offline.py`、`test_control_panel_callbacks.py`、`verify_control_panel.py`、`browser_check.py`、`browser_fixture.py`、`stage_workspace.sh`、`run_verification.sh`、`run_browser_fixture.sh`、`stop_browser_fixture.sh`、`cleanup_test_stacks.sh`、`smoke_panel.sh`、`check_deployed.sh`、`panel_result.json`、`panel_verify.log`、`panel_callbacks.log`、`panel_ros.log`、`codex_stop_race_result.json`（GREEN）、`codex_first_stop_race_result.json`（保留的 RED 证据）、`evidence/`（含 3 张截图、browser_result.json、会话与操作存档）。

修改：
- `src/slaihand_bringup/CMakeLists.txt`：仅新增安装 `web/`、`control_panel.py` 与两个新模块。`4ba838fd90ea0206…`
- `package.xml`：**未改**（无新依赖；与 before.json 基线逐字节一致）。

未改（已按 before.json 复核 11 个受保护文件哈希全部一致）：`commission_supported_trial.py`、`joint_waypoint_plan.py`、`offline_panel.py`、`canusb_motion_controllers.yaml`、`canusb_motion_policy.yaml`、`hardware_mapping.yaml`、四轴 URDF/xacro、`canusb_system.cpp`、`motion_supervisor.hpp`、`slaihand_transport`。未抽取接口，也不需要改这两个复用文件。

## 2. 关键设计（详见 `设计节点.md`）

- **两条通道**：只读观察走既有话题（joint/dynamic/controller_state）；运动只经固定 argv 子进程调用受保护 `commission_supported_trial.py`（`--plan`+三项 `--ack-*`+`--hold-seconds 1`），面板自己不 arm、不发轨迹、不开串口。
- **预览票根**：预览用实时反馈作起点、URDF 全限位作界调用 `compile_plan`，成功后冻结为 `absolute_deg` 绝对计划；票根绑定冻结计划、源文档规范化 SHA256、预览起点、反馈时刻、60 秒有效期、一次性使用。执行请求携带**当前编辑器文档**，服务端重新规范化后与票根源哈希比对（比前端自算哈希更稳），并复核反馈新鲜、未使能、起点漂移 ≤0.5°、心跳新鲜、时间预算。
- **状态所有权**：电机状态/保护/45 秒会话归驱动；轨迹与会话归受保护客户端；票根与确认归面板。目标角度只显示“用户/动作明确目标”（运行时为 JTC 参考，空闲为预览终点/上次计划终点），**不把反馈当目标**；无反馈一律 null，前端显示“—/失联”。
- **停止**：`POST /api/stop` 不取执行锁、不需确认、立即 202；同时（a）`SimpleQueue` 通知 ROS 线程调 `/slaihand/stop_all`、（b）HTTP 线程直接对子进程组 SIGTERM→2s→SIGKILL、（c）停止纪元递增。状态明确区分 `requested/confirmed/unknown`；停止后必须重新观测到 `control_state=0` 才能再预览/执行，杜绝迟到使能。
- **断连**：页面 1 秒心跳 + `pagehide` keepalive 放弃；仅在执行/保持中强制 3 秒租约，超时终止子进程并请求停止。面板 SIGINT/SIGTERM 同样终止在途客户端、请求停止、有界等待新失能后退出。
- **安全边界**：仅 127.0.0.1、会话随机 token、页面必须带 `?token=`、POST 必须精确 Origin、Host 白名单、Body ≤64KB、静态路径白名单、严格 CSP、单任务互斥、日志一律 textContent、计划存档仅写任务目录且 ID 服务端生成。

## 3. 实测命令与结果（第二轮，含首轮返工后重跑）

WSL Ubuntu 24.04 / ROS 2 Jazzy；隔离 `ROS_DOMAIN_ID=176`（无驱动域用 177）+ `ROS_LOCALHOST_ONLY=1`；假驱动为每个场景新建的 `/dev/pts/*`。

```bash
# 1) 离线（Windows 与 WSL 各跑一次）
python  verification/hardware/control_panel_20261104/test_control_panel_offline.py
# -> 23 tests OK（真实 HTTP handler + 真实 compile_plan + 票根/预算/投影）

# 2) 原有规划/采样判据回归（WSL）
python3 verification/hardware/canusb_motion_20261104/test_waypoint_plan.py
# -> 12 tests OK（未改该文件）

# 3) Codex 首轮 P1 复现脚本（未改动）
python3 verification/hardware/control_panel_20261104/codex_stop_race_repro.py
# -> {"stop_status": 202, "hold_status": 409, "spawn_count": 0, "ros_callbacks_after_stop": 0}
#    exit 0（GREEN）

# 4) 生产回调反例（真实 PanelBackend + HTTP，Popen 打桩，无 ROS spin）
python3 verification/hardware/control_panel_20261104/test_control_panel_callbacks.py
# -> 11 tests OK

# 5) 隔离域真实 ROS/JTC 端到端（WSL；run_verification.sh 先跑回调反例再跑端到端）
bash verification/hardware/control_panel_20261104/stage_workspace.sh
bash verification/hardware/control_panel_20261104/run_verification.sh
# -> RESULT: PASS (callbacks=0 e2e=0)
#    {"status": "PASS", "checks": 119, "failures": [],
#     "sessions": [core, stop, lease, exit, stale]}（每个场景独立 PTY/fake 栈/面板）

# 6) 浏览器实际打开候选页面（Windows Chrome headless + CDP）
bash verification/hardware/control_panel_20261104/run_browser_fixture.sh
python verification/hardware/control_panel_20261104/browser_check.py
bash verification/hardware/control_panel_20261104/stop_browser_fixture.sh
# -> {"status": "PASS", "checks": 33, "failures": []}
#    evidence/panel_desktop.png（1440×900）、panel_narrow.png（420×900）、panel_full.png（整页）
```

119 项端到端检查覆盖：打开/预览/导入/存档载入不使能（`fc` 全 0）；单点与多点预览与 `compile_plan` 结果逐值一致；超限/超速/13 点/停留过短/未知键/含糊模式全部 400；无确认、确认不全、错误文档、未知票根、过期票根、重复票根、并发执行、已失能预览全部被拒；真实正负路点执行（J3 负向 −1.00°）逐点通过、新失能确认、`fd` 增加；跟随偏差 0.6° 时 JTC 以 **−5** 中止并保留原始消息 `Aborted due to goal_time_tolerance exceeding by 2.00…`、逐点未通过、`overall_ok=false`；**停止受理即 409 `STOP_LOCKED` 拒绝紧接的 execute/hold/preview**；停止后 1.5 秒无迟到使能；停止状态与原因如实保留；复位并观测新 DISARMED 后解锁并可重新预览；心跳丢失 3 秒有界停止且同样锁定门禁；面板 SIGTERM 0.36–0.57 秒退出、子进程 PID 已消失、停止确认、写出会话记录；无驱动域全部 null 且拒绝预览/执行；SIGSTOP 冻结驱动后 `dynamic_age=1.20s`、界面不显示 0、预览被拒。

33 项浏览器检查覆盖：标题/阶段/心跳 chip、界面实际角度与 API 逐值一致、无目标显示“—”、停止按钮可见可用（168×53）、**轮询 3 秒不覆盖操作员输入**、编辑后提示重新预览、预览显示服务器绝对目标与预算、**在浏览器内勾选三项确认并真实执行**（退出码 0、逐点“通过”、停止确认“已确认”、原始输出可见、阶段变“已失能”）、**点击停止后横幅提示门禁锁定且预览按钮禁用**、**手动复位并观测 DISARMED 后横幅消失、预览恢复可用**、事件日志/ROS 明细可读、窄屏 420px 无整页横向溢出、停止入口仍可见、无未捕获异常与脚本错误。

## 4. 隔离部署与启动

- 候选工作空间：`/home/xuyu/slaihand_control_panel_ws_20261104`（仅构建 `slaihand_bringup`）。下层保持 `/home/xuyu/slaihand_canusb_motion_ws_20261104_kpMgtx/install/setup.bash`，**未覆盖**当前已安装工作空间。
- 启动（面板打印带一次性随机 token 的 URL，端口 0 自动分配）：

```bash
source /opt/ros/jazzy/setup.bash
source /home/xuyu/slaihand_canusb_motion_ws_20261104_kpMgtx/install/setup.bash
source /home/xuyu/slaihand_control_panel_ws_20261104/install/setup.bash
python3 /home/xuyu/slaihand_control_panel_ws_20261104/install/slaihand_bringup/lib/slaihand_bringup/control_panel.py --port 0
```

- 启动 URL 形式：`http://127.0.0.1:<端口>/?token=<会话token>`（本轮浏览器验证用的是 `http://127.0.0.1:18888/?token=…`，该测试实例已停止）。Windows 浏览器经 WSL localhost 转发可直接打开。
- 可选参数：`--archive-dir`（默认 `~/.ros/slaihand/control_panel`）、`--preview-ttl-seconds`（1–300，默认 60）、`--heartbeat-timeout-seconds`（1–30，默认 3）。只允许 `--host 127.0.0.1`。

## 5. 验证中发现并修正的问题

1. 页面被当 `application/json` 发送（浏览器显示源码）→ 修 `_send_page` 的 MIME，并加入离线断言。
2. 窄屏整页横向溢出（grid 子项 min-width 与长路径不可断行）→ `minmax(0,1fr)`、`.card{min-width:0}`、`overflow-wrap:anywhere`，并加窄屏滑动提示。
3. 心跳 chip 永远显示“过期”→ 状态补 `lease.fresh`（此前只用于服务端判定）。
4. 逐点“通过”单元格把 CSS 类当文本渲染 → 修正 `el()` 参数顺序。
5. 端口占用时 ROS 线程未收尾导致 core dump → 构造失败走完整 shutdown 并返回码 2。
6. 被拒 POST 未读走 body，客户端得到连接重置而非 JSON 403 → 先排空受限 body 再回错误。
7. `/favicon.ico` 404 噪声 → 204。
8. 浏览器 fixture 重启竞态导致半死栈（节点重名、控制器加载失败）→ 启动/停止脚本改为等待硬件激活后再起面板。
9. **[首轮返工 P1]** 停止受理后缓存中的旧 stage 0 仍能启动 hold/execute → 新增停止门禁（见第 0 节），`preview/execute/hold` 与 `_launch` 双重检查，解锁需“手动复位 + 因果更新的四轴一致 DISARMED”。
10. **[首轮返工]** 旧 fake 栈孤儿 `robot_state_publisher` 以 TRANSIENT_LOCAL 保留旧 URDF（已关闭的 PTY），令新驱动配置失败 → 每场景独立夹具 + 唯一会话标记 + 按标记/PID/PGID 回收 + 启动前等待域内无描述发布者；测试进程显式关闭子进程 stdout 管道，避免 GC 迟到关闭命中复用 fd。
11. **[首轮返工]** 测试脚本按名字 `pkill` 可能误伤其他面板/假栈 → 改为 PID/PGID + `/proc` 身份校验（cmdline、ROS_DOMAIN_ID、PGID 一致）后才终止本实例；清理脚本只处理隔离域 176/177 的 ROS 进程。

以上均为本次新增代码或测试夹具的问题，未改任何受保护文件或阈值。

## 6. 未验证项与限制

- **精度/重力未验证**：PTY 是理想跟随，不能证明实际重力补偿、摩擦、回差或 0.5° 到位精度；首次独立路点 J1 返回 0.503° 超判据的事实不因本面板改变。动作成功仍不等于精度通过，界面分栏显示。
- **实机操作未执行**，本轮也不建议在 Codex 验收前用候选面板控制实机；真实支撑、净空与现场切电条件由用户另行确认。
- 40 段预算与 45 秒会话上限、0.02rad/s、0.5° 容差、门禁全部沿用，未加任何放宽。
- “当前位置保持”仍是既有**有限 5 秒**流程，不宣称持续保持；计划完成即失能，不会在终点保持。
- 空闲时点“全轴停止”也会让驱动进入已失能、需要手动复位（界面已提示）；这是驱动 `stop_all` 的既有语义。
- 浏览器后台标签页若被系统完全冻结超过 3 秒，租约会把“冻结”当“断连”触发有界停止（安全侧误停，未做区分）。
- 传感器/总线抖动：负载下偶见 `dynamic_age` 约 0.38s（chip 显示“延迟”），仍低于 0.5s 门禁；未观察到因此误拒。
- 面板依赖同包安装的 `commission_supported_trial.py`（启动时校验其含 `--plan` 并显示 SHA256）；下层旧安装的客户端不支持时会被拒绝而不是降级。
- 浏览器验证使用 headless Chrome（1440×900 与 420×900）与截图人工核对；未覆盖 Firefox/Safari、触摸与屏幕阅读器。
- Windows `/mnt/d` 偶发文件锁导致写证据失败，测试脚本已加写重试与 `/tmp` 兜底；本轮最终证据均成功落盘。
- **真 UNKNOWN 仍可能出现**：驱动 `stopping()` 的 0.3 秒确认窗口与 60ms 反馈时效在负载下可能超时，此时面板如实报 UNKNOWN、门禁锁定、复位被拒（本轮端到端在隔离域上得到 confirmed；首轮 Codex 运行曾得到 unknown，两者都被测试接受并分别验证）。真实 UNKNOWN 需要人工处理或重启进程，面板不重试、不降级。
- 端到端每个场景独立启动 fake 栈（约 8–10 秒/次），5 个场景使总时长增至约 4 分钟；夹具启动失败会重建一次并记录为检查明细（不是静默通过），二次失败仍记 FAIL。
- 浏览器检查要求 fixture 是**新启动**的（面板/驱动状态不复位），`run_browser_fixture.sh` 会先停止已验证的旧实例；重复对同一 fixture 运行会因残留操作状态而失败，这是检查前提而非产品缺陷。

## 7. 边界遵守

- 未打开任何 `/dev/ttyACM*`、`/dev/serial/by-id/*`、COM9；未绑定/分离 USB；未杀真实 ROS 进程；未发真实 ROS 动作；未运行实机 launch/BAT。
- 所有测试仅用隔离域 176/177、`ROS_LOCALHOST_ONLY=1` 与本轮程序新建的 PTY；真实栈 PID 8260/8264 全程存活（2h14m）。
- 测试进程清理只按本实例 PID/PGID、唯一 `SLAIHAND_PANEL_TEST_SESSION` 标记与隔离域 176/177 的 `/proc` 环境判定，不按名字 `pkill`；清理脚本每次运行都打印真实串口栈仍在。
- 未改硬件参数/零位/符号/限位/增益/容差/门禁/模式/URDF/policy/controller 配置；未提交、未推送、未回滚他人文件；脏工作区保持原样（`git status` 的 M 列表与开工时一致，仅新增本交付文件）。

## 8. Codex 验收要点

1. `test_control_panel_offline.py`（23）、`test_waypoint_plan.py`（12）、`test_control_panel_callbacks.py`（11）原样重跑；`codex_stop_race_repro.py` 必须 GREEN（hold=409、spawn=0）。
2. `stage_workspace.sh` 后 `run_verification.sh` 应输出 `RESULT: PASS (callbacks=0 e2e=0)`；`panel_result.json` 应得 119/119 PASS，`sessions` 含 core/stop/lease/exit/stale 五个独立会话与各自 PTY。
3. 重点审查 `control_panel.py`：`_require_stop_gate`/`_check_stop_gate`/`_launch` 的三处门禁，`_request_stop` 在锁内设置门禁，`reset()` 只记录复位时间而不解锁，`_maybe_cleanup_after_operation` 不覆盖操作员停止记录；确认停止路径不等待文件 I/O 或启动锁。
4. 审查 `control_panel_http.py` 的鉴权顺序，确认 execute/hold 三项确认仅在服务端生效、票根一次性、执行文档哈希绑定。
5. 复核测试夹具：`verify_control_panel.py` 的会话标记/回收、`wait_domain_clean` 前置条件、`browser_fixture.py` 的 PID/PGID 状态文件与 `stop_browser_fixture.sh` 的身份校验；确认没有任何按名字 `pkill` 的清理路径。
6. 验收通过前，请勿让用户用该候选面板运行实机控制；`evidence/` 内截图可作 UI 验收参考。
