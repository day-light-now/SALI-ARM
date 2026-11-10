# B：CANUSB 四轴原生 MIT 接管与标准轨迹（2026-11-04）

本轮交付控制后端、启动入口及操作客户端。软件已经在真实 ROS 2 节点和自动生成的虚拟串口上验证；没有由代理连接实机、使能或运动电机。实机 B 阶段仍待用户执行。

## 用户最新限位指示

使用本机按 OpenArm 设计并确认的完整 URDF 角度范围，不再额外缩小：

| 关节 | 下限 ° | 上限 ° |
|---|---:|---:|
| J1 | −80 | 200 |
| J2 | −10 | 190 |
| J3 | −90 | 90 |
| J4 | −90 | 50 |

唯一位置限位源仍是 `src/slaihand_description/urdf/slaihand_four_axis.urdf`。xacro 不复制 min/max，插件直接使用 Jazzy 解析的 `HardwareInfo.limits`，Controller Manager 同时启用命令限位。没有默认2°内缩，没有相对接管点±2°窗口。公共 Python 默认工作范围也改为完整 URDF；旧五轴运行配置 J1–J4 的端点改为 null，以继承同一来源。J5 保持关闭。

标定 sign/zero 未改：J1 +1、零位1.4574273289082171 rad；J2/J3 +1、J4 −1。用户指定的 J1 采集路线±70°是测试路线，不是全工作范围。原始日志和历史参数快照不重写。

驱动16位位置量化可能使“正好在端点”的请求落到范围外一个码值；本实现仍用官方编码器，选择范围内最近的可表示码，最终检查编码后目标。最多一个位置码的表示差异不等同于另加角度安全窗口。纯离线回归覆盖四轴上下端点、J1 +170°及 J2 −9°，防止旧扇区和隐含内缩重现。

## 采用的控制链

`标准 JointTrajectoryController → position 命令接口 → CanUsbSystem → 官方 OpenArm MIT 编码 → 现有 CANUSB → 四轴共享ID0反馈分发`。

MIT 的 Kp/Kd 在驱动内部运行；没有重写主机 PD，没有使用仿真重量，没有启用重力补偿。运行前实读的 PMAX/VMAX/TMAX 必须匹配当前官方 DM10010L / DM8009 编码配置；不匹配即拒绝，绝不自动修改电机参数。

新实现：`native_codec.hpp`、`motion_supervisor.*`、`motion_audit.*`、`canusb_system.*`。共同传输封装保留现有只读能力；新增受限的 MotionPort 只允许 J1–J4 MIT/FC/FD，拒绝 J5、PV/速度偏移ID、参数写入、设零和清故障。

## 启动、接管、轨迹与停止

- launch 只启动状态采集；不会使能，不回零。标准轨迹控制器加载为 inactive。
- 用户单独请求 `arm_supported_current_pose` 后，重新读取四轴配置及新鲜反馈。必须 MIT、TIMEOUT2000、正常失能且起始静止；以这一次的当前位置冻结目标。
- 四轴先预装零增益帧，随后错峰使能并分别观察 state=1，再做1秒增益斜坡。请求 tau_ff=0，但有限位编码不等于严格零力矩，不能拿它当停止方案。
- 增益斜坡结束、低速稳定窗口完成后允许标准 JTC 取得四轴 position 接口。HOLD/hold_ready 表示控制阶段和稳定观察，不代表达到某个位置精度或已能独立承载自重。
- 一轴故障、超时、超速、越过完整URDF边界、跟随误差、估计门禁、循环超时或审计失败，均转入四轴 FD 停止；即使一个 FD 写入失败，也尝试其余轴。
- 每轴必须在本轴 FD 写入之后出现两次新的正常失能观测。缺少确认在300ms后进入 UNKNOWN，停止主动控制/查询，拒绝 reset/arm；必须现场检查并重新启动流程。软件观测不证明物理切力矩。
- 手动 reset 只在新鲜四轴失能状态下返回 DISARMED，不使能；不能清驱动故障。轨迹控制器的接口要先释放，不能绕过资源仲裁。

控制状态数值：0 DISARMED、1 PREFLIGHT、2 PRELOAD、3 ENABLING、4 RAMP、5 HOLD、6 TRACK、7 STOPPING、8 DISABLED、9 UNKNOWN。**8 表示控制输出停用，只有再结合 `disabled_observed=1` 和各轴 state=0，才是本轮主机失能观测确认。**

## 本轮调试参数与证据边界

| 项 | 采用值与含义 |
|---|---|
| Kp/Kd | 160 / 5，沿用已存在的原生 MIT 台架设置，不称为四轴实机已验证增益 |
| 估计门禁 J1/J2/J3/J4 | 8 / 10 / 2 / 2 Nm；检查当前反馈状态下的请求估计及驱动反馈估计，**不是驱动持续硬限流** |
| 参考速度 | ≤0.02 rad/s；与完整角度工作范围分开 |
| 反馈速度中止 | 1 rad/s，沿用用户已选阈值 |
| 跟随误差 | ≤4°；不是额外的目标角度范围 |
| 温度 | ≤50℃ |
| 控制反馈时效 / 循环间隔 | 60ms / 50ms；不放大TIMEOUT、不改固件保护 |
| 有限会话 | 45秒，超时全停；不是位置限位 |

每个控制周期批量下发四轴帧，不沿用 A 阶段“一次轮询一轴”的调度。100Hz仍是目标，真实四轴控制周期和 USB 延迟必须通过实机日志测量，不能拿虚拟设备结果当实测带宽。

审计使用有界队列，JSON格式化和文件写入移到后台；队列溢出或I/O失败触发全局停止，不静默忽略。日志分 `TX_SUBMITTED`、`TX_WRITTEN`、`RX` 和阶段事件，明确写入不是驱动执行确认。四轴同一周期及同一 command_id 可关联；操作客户端也记录收到的 command_id、后台审计文件路径、当次确认和参数。

## 验证

- 4个 CTest 通过，包括官方编码/映射、唯一URDF限位、只读协议、四轴监督反例。端点量化、无自动回零、原生增益斜坡、完整工作范围、全局故障停止、旧缓存不能冒充新失能、UNKNOWN锁定均有覆盖。
- 真正启动 ROS / Controller Manager / pluginlib / JointStateBroadcaster / 标准 JTC，设备仅为脚本新建的 `/dev/pts/*`。验证启动0使能，接管前拒绝JTC，接管后接受标准动作并完成四轴+1°往返；反馈行程约0.984°/0.984°/0.984°/1.005°。
- 再次接管后注入J3故障：四轴均收到FD，持续故障无法产生正常失能确认时进入UNKNOWN，reset和arm均拒绝。
- 实际安装的操作客户端也在同一虚拟设备上执行保持、停止与结果保存，确认客户端记录与后台 command_id 对应。见 `operator_client.log`、`operator_*.json`。
- 公共角度范围配置回归28项通过。首次 ROS 联调曾因测试脚本把 DISABLED 的数值写成7而超时；实测后台已正确进入8且失能确认，修正的是测试断言，没有放宽硬件保护。
- 最终以部署副本的 `deployed_ctest.log`、`ros_motion_result.json`、`readonly_compat.log`、`deployment.json` 和 `fake_audit/` 为证据；这些都是软件/虚拟证据。旧只读兼容性及 URDF 解析另行验证。

## 用户下一步

已部署至 `/home/xuyu/slaihand_canusb_motion_ws_20261104_kpMgtx`。先在旧 A **只读 launch** 窗口 Ctrl+C，释放串口及同名 ROS 节点；不要只停止 topic echo 窗口。随后由用户启动：

```bash
source /home/xuyu/slaihand_canusb_motion_ws_20261104_kpMgtx/install/setup.bash
ros2 launch slaihand_bringup slaihand_canusb_motion.launch.py port:=/dev/ttyACM0
```

此时仍是 DISARMED，只读采集，JTC inactive，不发使能。

另一个 Ubuntu 终端 source 同一个环境后，可先运行纯预览：

```bash
ros2 run slaihand_bringup commission_supported_trial.py
```

首轮运动操作由用户在确认**当次可靠支撑、完整净空和现场断电手段**后执行。确认原生 MIT 估计门禁不是持续硬限流；默认5秒四轴当前位置保持，结束自动请求四轴停止：

```bash
ros2 run slaihand_bringup commission_supported_trial.py --execute --ack-supported --ack-clearance --ack-native-output
```

该5秒实机记录检查后，再使用同一客户端的 `--round-trip` 调用标准轨迹动作；默认+1°是首轮测试轨迹，不是工作限位。结果保留实际行程与返回误差，动作服务成功但未观察到请求行程时不算往返通过，也不自动加力。Ctrl+C由客户端尝试请求全停；客户端被强制杀死等情况不等于驱动失联，后端45秒会话时限仍独立存在。

原始 A 实机日志保留，J5/参数/零位/URDF轴未改；本轮没有实机运动验收，没有质量/质心辨识、补偿、碰撞或末端任务验收，旧桌面按钮尚未接到此 ROS 后端。

## 首次实机B启动观察：尚未完成配置

用户11:06启动B后停在configure。进程7624的线程7662为D状态，wchan=`usb_kill_urb`；审计`canusb_motion_NEfvY3.jsonl`仅1114字节SESSION头，没有TX/RX记录。只读探针`probe_startup.py 7624 <audit>`得到`STARTUP_USB_KERNEL_STALL`。未打开第二个串口、未发使能。具体阻塞的系统调用/内核栈读取权限不足，尚不能判定是open、termios还是flush的哪一步；不能声称已定位到某行应用代码或已修复。下一步由用户退出启动并重新建立USB透传，再检查配置是否完成。
