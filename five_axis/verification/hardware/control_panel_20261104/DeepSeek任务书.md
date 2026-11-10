# 四轴 ROS 统一控制台

根目录：`D:/CodexData/Projects/机械臂/slaihand_ros2_ws`。DeepSeek实现；Codex独立验收；用户执行所有实机操作。本轮用户已明确授权联合，软件离线工作连续完成即可。

## 目标与入口

提供本地中文网页控制台，替代反复输入ROS命令。沿用现有 ROS + OpenArm MIT + JointTrajectoryController，不重写控制律或硬件后端。

必读：
- `src/slaihand_bringup/scripts/commission_supported_trial.py`（有限会话、显式确认、预览、执行和停止）
- `src/slaihand_bringup/scripts/joint_waypoint_plan.py`（唯一规划/验收逻辑）
- `src/slaihand_bringup/config/canusb_motion_controllers.yaml` / `canusb_motion_policy.yaml`
- `verification/hardware/canusb_motion_20261104/四轴独立路点交付.md`
- `verification/hardware/canusb_motion_20261104/独立路点首次实机中止诊断.md`
- `verification/hardware/canusb_motion_20261104/verify_waypoint_client.py` 和 `verify_ros_motion.py`（已验证PTY夹具）

`offline_panel.py` 仅能用于界面参考，它特意只接受fake插件，不能解除该限制后拿来控制实机。新增独立入口。

已知：实机四轴 +10°往返已完成；首次独立路点实际也移动了，但J1返回误差0.503°略超新0.5°判据，JTC abort转保持产生参考跳变，backend REFERENCE_SPEED全停。当前客户端已修正，保留可用的动作原始错误和逐点误差。不能把这描述为“完全未运动”，不能掩盖未通过精度验收。理想PTY测试不能验证重力和实际精度。

## 产品交付

1. 状态：J1～J4实际角度、目标角度、误差、速度、反馈估计力矩、温度、反馈时效、控制阶段、动作/错误原因。无反馈用“—/失联”，不能用0填充。目标是明确的用户/动作目标，不把实际反馈伪装成目标。显示连接、未使能/保持/轨迹/停止/UNKNOWN。
2. 四轴目标输入（单位°），可一键读取当前位置填目标，但不运动；输入/拖动/打开页面不会使能。支持软件绝对角度和相对本次起点，含义清晰。
3. 多路点可增删、每点移动时间/停留、JSON导入/导出/保存后重放。沿用 `compile_plan` 校验全部段、URDF限位和时间预算。单点也是同一计划接口；不做额外单轴控制链路。
4. 工作流：编辑→服务器验证的预览（显示实际绝对目标和时间）→明确执行。编辑、过期、反馈失效/姿态明显变化后旧预览失效。推荐把预览相对计划冻结为绝对计划，并绑定起点、内容、短期一次性票根，防止预览与执行不一致。时间预算仍从执行时实际位置重新核验。
5. 执行前显式确认支撑、路径净空与可现场切电、原生MIT反馈门禁不是硬件限流。所有execute/hold调用服务端强制检查确认，不能仅前端禁按钮。初次接管目标仍为当前位置，不能使能即去输入目标或零位。
6. “当前位置保持”可用现有有限5秒保持流程，并明确有限时长；不要宣称持续保持。手动软件复位独立按钮，不自动复位/重试/重新使能。执行完成自动停、后台会话45秒上限保留。支持多次人工重放，每次重新预览、复位、明确执行；不做无人值守无限循环。
7. 全轴停止入口始终易见可用，不能被执行线程/锁阻塞；通过ROS stop_all，必要时先中断操作客户端，区分“停止已请求/新失能已确认/未知”。不是物理急停。启动、断连、停止竞态都不能触发迟到的使能。浏览器控制连接丢失时有有界停止机制（如操作页心跳租约）；backend退出也应终止在途客户端并请求stop，保留驱动原有有限会话。
8. 动作结果保存/显示原始错误码、消息、逐点反馈判据与stop_observations_confirmed；动作成功不能替代精度通过。预览/执行/复位/停止的HTTP响应不能混淆请求接受和操作完成。

## 技术边界与文件所有权

可新增：
- `src/slaihand_bringup/scripts/control_panel.py`、同目录 `control_panel_*.py`（必要模块）
- `src/slaihand_bringup/web/control_panel/` 静态网页/CSS/JS
- `verification/hardware/control_panel_20261104/` 设计、测试、证据、交付说明
可改：`src/slaihand_bringup/CMakeLists.txt` 和 package.xml，仅安装新入口、静态资源/必要依赖。
已存在的 `commission_supported_trial.py` / `joint_waypoint_plan.py` 为受保护可复用模块；优先调用其固定CLI/函数。若必须抽取接口，先写明理由和最小改法到设计记录，并通知Codex，未经答复不改这两个文件，其余独立工作可继续。

保护：`src/slaihand_hardware/`、`src/slaihand_transport/`、URDF、hardware_mapping、policy、controller配置、旧fake面板、原始日志与现有证据、零位/符号/模式/TIMEOUT/增益/限位/容差/门禁。禁止自动回零、仿真质量用于重力补偿、J5查询/控制、绕过保护。

只通过既有ROS话题/服务/CLI交互，面板本身不打开串口/CAN。由单独的ROS工作线程处理订阅和服务响应，HTTP任务不得阻塞ROS spin或停止。可通过有限客户端子进程执行计划，固定argv、不用shell拼接，不接受任意命令/任意文件写入路径。

本地HTTP仅监听127.0.0.1；使用会话随机token、同源校验、请求大小限制、单任务互斥。拒绝无token/跨源执行/重用过期预览/并发执行。导入JSON是数据；不执行其指令。日志展示用textContent。计划存档在任务专用目录，保留用户输入和结果，不读私人文件。

真实适配器正在WSL使用，严禁打开任何 `/dev/ttyACM*` / `/dev/serial/by-id/*` / COM9，严禁绑定/分离USB、杀真实ROS进程、发真实ROS动作/arm/stop/reset、改硬件参数、运行实机BAT或launch。测试可在隔离ROS_DOMAIN_ID（比如176）使用本轮程序新建的 `/dev/pts/*` 和PTY假驱动。不使用ROS域0测试。不能改系统或代理/防火墙设置；不安装新软件、不发布/提交/推送。

保留现有脏工作区，不回滚别人文件。运行常规编译/离线测试已授权。允许建立新的隔离WSL工作空间部署候选客户端并构建bringup，下层使用 `/home/xuyu/slaihand_canusb_motion_ws_20261104_kpMgtx/install/setup.bash`；不覆盖当前已安装workspace。LF/shebang须能在Linux执行。

## 节点与验收

先写 `设计节点.md`：架构、计划票根/状态所有权/停止竞态、浏览器断连、同源/token边界、真实回调测试方式。写完通知Codex，可继续边界内实现，不必等待每个小修改。

必须实际验证：
- 打开/预览/编辑/导入不使能；单点与多点的真实生产HTTP回调调用同一计划校验器。
- 新生成PTY上完整ROS/JTC状态显示、独立正负路点执行、结束新失能确认。
- HTTP并发执行拒绝；token/Origin拒绝；修改/过期/重复票根拒绝；无确认拒绝；反馈陈旧/UNKNOWN拒绝。
- 正在执行时UI停止、浏览器心跳丢失、面板正常退出的有界停止；不发生stop之后迟到使能。
- 模拟跟随偏差造成JTC终点失败，GUI/HTTP仍保留原始错误/逐点未通过，不显示成功。
- 原 `test_waypoint_plan.py` 12项仍通过。补测试应验证真实生产回调，而非另写一套实现副本。
- 用浏览器实际打开候选fake页面，核对输入/显示/预览/日志可读、目标不随轮询被重置、停止入口可见。桌面优先，窄屏可用，无需外网字体/CDN。

若环境缺少浏览器自动化，用实际可用工具或明确报告未执行项，不编造。若有网络/服务失败，轻量恢复一次仍失败就报告，不无限重发。

## 交付

源码写回上述允许路径；紧凑报告：`verification/hardware/control_panel_20261104/DeepSeek交付_四轴统一控制台.md`。列修改文件、关键设计、实际测试命令和结果、隔离部署路径、启动URL、未验证项及限制。报告给Codex验收；验收前不要引导用户运行候选实机控制。
