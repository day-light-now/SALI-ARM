# OpenArm参考与J4起动/制动优化：DeepSeek实现，Codex验收

根目录：D:/CodexData/Projects/机械臂/slaihand_ros2_ws。用户明确调用ctod-han，授权软件优化；实机操作由用户保留。请先读任务书及真实代码，独立判断，不把本文件假设当结论。

## 已核实官方参考
官方ROS2 https://github.com/enactic/openarm_ros2 ，本地干净快照 ../tmp/openarm_ros2_latest ，commit 4e837e1d0dae692ff67b560b69d8d281d7a8d4ed。
官方CAN https://github.com/enactic/openarm_can ，本地干净快照 ../tmp/openarm_can_latest ，commit f340d4b808fb177e1f297af54eb55fd51c6c7c10。
本任务official_reference目录有只读参考副本。ROS2硬件write()将{kp,kd,pos,vel,tau}送入MITParam、mit_control_all。controller yaml示例update_rate=750Hz。当前我们的驱动Kp/Kd=0，主机约100Hz算PD并发力矩；Windows/USB链路不能照抄750Hz或声称实时保证。官方on_activate()含enable_all/return_to_zero，不得照搬。本任务不是七轴ROS2移植或全臂使能。

## 症状与证据（不得篡改）
1Nm Kp32 Kd0.5原始日志 tmp/four_axis_readonly/20261101_154828_918900.jsonl；结果 tmp/live_trial/20261101_154949_b50554.json。命令峰值0.435Nm，真实位置变化支持速度突跳；0.1rad/s保护后失能下一帧约0.2308rad/s。
1Nm速度阈值0.2结果 tmp/live_trial/20261101_155439_df9c6a.json：约0.989Nm编码饱和但仅0.044°变化，跟随误差2°停止。
2Nm候选原始 tmp/four_axis_readonly/20261101_155917_340800.jsonl，结果 tmp/live_trial/20261101_155927_e9ad73.json/.jsonl：Kp32/Kd0.5，误差边界4°，连续主机双向力矩斜坡1Nm/s，速度中止0.2，目标速度0.02rad/s。命令峰值1.2Nm、未饱和；位置由31691到31689再31683，速度由-0.033到-0.121到-0.209rad/s；停止后新帧state0、pos31671、速度-0.4725，位置差分也支持真实加速。用户确认断电手动可转，不能据此排除负载、弹性、支撑或机械问题。
模型J4正方向sign=-1，勿把负反馈速度误判方向反了。

## 任务
A. 写清官方原生MIT与本项目主机PD的真实数据流差异，包含引用commit/文件/函数，以及适合借鉴与不适合照搬部分。不能称已照抄/已实机验证OpenArm。
B. 优化起动与制动逻辑，重点核查双向对称斜坡是否延迟减力/阻尼、静摩擦释放后目标误差积累与阻尼不足、量化/反馈时序、停机后仍运动。以源码和日志验证，不提前保证这是唯一根因。可以新增明确选择的阻尼候选档及有界算法，不能自动选档或自适应增力。
C. 原生电机MIT闭环可以给出评估/离线原型，但未经证明的总力矩限制不能靠FF字段限幅宣称实现。若原生PD总输出无法在不改电机参数下守住2Nm命令边界，本轮生产入口仍保留有总命令限幅的主机PD，并明确差异，不能悄悄改为原生驱动增益。
D. 改进诊断，少量码值变化不可被表述为有效运动；成功仍须真实10°出程/返回/新反馈失能。应区分码值抖动、可观察起动、有效轨迹跟随，报告包含触发保护帧和停止后新反馈位置/速度，避免只报保护前0.13°掩盖整个事件。驱动力矩反馈不是轴端测力。

## 可改与不可改
可改：slai_console/actuation.py、live_trial.py及对应test_actuation.py/test_live_trial.py/test_round_trip.py；可新增slai_console/j4_control*.py和相应test_j4_control*.py；本任务目录可写报告、新离线回放脚本及合成数据。baseline/和official_reference/只读。保护其他代码（尤其actuation_io.py、commissioning.py/core.py/four_axis_monitor.py）、mapping、URDF/MuJoCo、原始日志/结果、历史报告和skills。若必须改保护代码先说明并停该改动，不绕行。
不打开串口，不导入启动实机入口，不查询/使能/运动电机，不改RAM/Flash/TMAX/零位/模式/TIMEOUT，不打开任何bat，不控制电脑去实机操作。不改git索引，不提交/推送，不安装依赖，不改其他工作区。保留现有未提交修改。
严格保留：仅J4候选可使能；J5不查询/不使能/不控制；首次目标=新鲜当前位置；输出总编码命令绝对值<=所选档，最高2Nm；J4速度中止最高0.20rad/s；2Nm档跟随误差最高4°，其余档不扩大；目标速度<=0.02；每程10°18秒、最多45秒；支持/净空确认；温度50°C、反馈60ms、主机周期40ms、TIMEOUT2000预检和故障全停、FD四轴/新反馈确认/未知状态锁定。不得靠加速度阈值、连续多帧判定等方式悄悄忽略真实超速。
减力/制动路径可以不沿用原对称斜坡，但必须有理由和离线边界测试；不得使用加力上限来阻滞紧急减力或FD。Kp/Kd调整须命名、冻结票据绑定、独立确认、每次结束恢复默认，日志/UI准确记录；默认不得自动切新候选。
不将URDF当已验收物理包络，不宣称看门狗实际失能力矩/时延已验证。不重力补偿、不多轴联动。

## 验收
环境Python：C:/Users/ASUS/AppData/Local/Programs/Python/Python312/python.exe。
基线命令：python -m unittest slai_console.test_round_trip slai_console.test_live_trial slai_console.test_watchdog_check slai_console.test_actuation slai_console.test_actuation_io slai_console.test_commissioning slai_console.test_console slai_console.test_timing -q（96项通过；Windows偶发临时文件被占用，须说明而非跳过）
新增测试：真实日志只读回放，能重现旧算法的制动减力延迟或证明不存在；同样状态/时序下新算法减力不被旧斜坡拖延；静摩擦/释放/惯性/量化/抖动合成plant展示可复核改善，不能只用平滑理想plant过关；硬性超速仍立即全停；断连/旧反馈/异常回调/取消路径不复活；有界编码输出、同一周期数据身份/符号正确、默认不升级、无确认不发运动、J5排除；静止与抖动不算往返成功。
测试假plant是算法证据，不是硬件保证。若在0.2rad/s边界内无法稳健完成模拟，报告失败和机械待确认项，不自动放宽。

## 交付
报告：本目录/DeepSeek交付_OpenArm控制逻辑优化.md。另写OpenArm对照与设计.md，列参考链接/commit/代码位置、实际借鉴、仍有差异、关键决策与未验证项。源码直接写回允许路径。
先在本目录/设计节点.md记录选择的控制结构、斜坡/阻尼设计、是否改变PD执行位置、需要的新参数；然后在上述边界内连续实现并离线测试，无需等每个小改动确认。简洁报告改动文件、测试真实命令结果和限制。完成后停，等Codex验收，不让用户直接运行新候选。
