# 现有 CANUSB 接入 ROS：软件交付（2026-11-04）

用户已确认使用原先 COM9、921600、能够读取四轴的那只 CANUSB。沿用现有 30 字节 TX / 16 字节 RX 封装；不要求换 DMMC02，不假定它等于 SocketCAN 或支持 SLCAN。协议名 `slai_canusb_tx30_rx16_v1` 表示这套已记录的报文格式，不表示产品型号。

本轮完成传输层、真实只读硬件插件和启动配置，并在虚拟串口上跑通真正的 ROS 节点与状态广播链。没有连接物理适配器，没有验收实机反馈或整臂运动。

## 实现

- `src/slaihand_transport/`：有界串口分帧、按 payload 轴号分发共享 CAN ID 0 的回复、运行时读取 PMAX/VMAX/TMAX。Linux 串口以 921600/8N1 配置，通过文件锁及 TIOCEXCL 独占；不存在设备型号自动探测或协议自动切换。
- `src/slaihand_hardware/.../canusb_readonly.*`：`CanUsbReadOnly` 插件，通过正式 pluginlib 名称加载。仅接受 J1–J4，拒绝任何 command interface；发送接口只提供 CC 状态查询和 RID 7/8/9/10/21/22/23 的读取。没有使能、运动、参数写入、清故障、设零、Flash 保存或 J5 查询路径。
- `slaihand_canusb_readonly.urdf.xacro`：包含同一份四轴 URDF，sign/zero/CAN ID 从现行 `hardware_mapping.yaml` 读取；不复制标定常量。
- `slaihand_canusb_readonly.launch.py`：只启动状态链路及 JointStateBroadcaster，不启动轨迹控制器；使用 `/slaihand` 命名空间，避免混入旧 fake 会话的全局模型和关节状态。未填写真实 Linux 串口路径时直接拒绝，不自动打开 COM9 或猜测 `/dev/ttyUSB0`。
- 原有 fake 模型、插件和启动入口仍保留，供离线轨迹验证。

发布位置、速度、驱动反馈力矩估计、温度、状态码、反馈接收时效、有效标记，以及本连接读取的模式/TIMEOUT。`effort` 是驱动估计值经 sign 换算后的量，不是标定过的轴端测力。

每个轴独立计接收时效；超过 250 ms 后位置/速度/力矩等置 NaN、`feedback_valid=0`，不持续展示旧角度为有效测量。这是主机接收时效，不是实测驱动 watchdog 延迟。寄存器冲突、无效量化范围或串口错误会使本只读会话失败；没有自动改参数或恢复运动。

配置阶段最多等待 5 秒收齐四轴寄存器及反馈。运行目标为每 10 ms 一个读取请求，四轴反馈各自轮询；不能称为四轴 100 Hz 闭环。实际周期、反馈速率与 USB 延迟尚待实测。

## 记录与生命周期

配置前先创建独占 JSONL 审计文件；记录每个完整读取写入和已解析接收帧、主机时间、协议及标定。正式 launch 还记录模型与 mapping 的 SHA256。串口完整写入记录为 `TX_READ_WRITTEN`，明确 `drive_execution_ack=false`；不会把写入当成驱动执行确认。

launch 默认日志目录 `~/.ros/slaihand/canusb_readonly/`；路径由启动日志打印，也可通过 `log_directory` 指定。日志失败则结束会话。

ROS activate 只激活读取和发布，不使能电机。deactivate 停止读取，保留串口持有；cleanup/shutdown/卸载释放串口。关闭此只读程序不发送 FD，也不是急停；不会改变接入前电机的使能状态。

## 已验证

1. WSL ROS 2 Jazzy 隔离编译 transport、hardware、description、bringup 四个包成功。
2. 3 项 CTest 通过：新 CANUSB 协议检查，以及既有映射/官方 MIT 编码、唯一 URDF 限位回归。新协议检查覆盖分包、噪声、坏帧、扩展 ID 拒绝、J5 拒绝、共享 ID0 分轴、实读范围、标定和冲突。
3. 通过 pluginlib 真正加载新插件，在测试程序自己新建的 `/dev/pts/*` 虚拟总线上完成配置、四轴读取、独占冲突、数据过期、释放串口及拒绝运动接口。测试会替换模型中的端口，只允许它自己生成的 PTY，不能传真实硬件端口运行这项测试。
4. C++ 的全部 32 种读取帧与现有 Python `usb_packet()` 逐字节一致；从原始只读日志提取前 200 条 RX 回放，四轴 q/dq/力矩估计/温度/状态/时间与 Python 结果一致。仅对回放使用当前 canonical mapping，不声称改写了历史标定。原日志只读、未修改。
5. 虚拟总线审计逐条复核：只有允许的四轴读取，没有运动/使能/参数写入。证据是 `ctest.log`、`fake_bus.log`、`fake_audit/`、`parity_result.json` 与回放夹具；这些都是离线或虚拟证据。
6. 实际运行 namespaced launch、Controller Manager、pluginlib 插件和 JointStateBroadcaster，端口只指向测试自己新建的 PTY。订阅确认四轴角度与标定一致，`/slaihand/joint_states` 和 `/slaihand/dynamic_joint_states` 均发布，四轴有效标记均为 1；见 `ros_graph_result.json`。首次完整启动发现显式 `interfaces: []` 被本机 Jazzy 当作无类型参数，已通过省略该项、使用广播器自身类型默认值修复；失败日志保留为 `ros_graph_initial_parameter_failure.log`。没有将“编译成功”直接写成广播链路通过。

真实 ROS 的虚拟设备测试也保留了 WSL 调度/跨文件系统审计写入的周期超时提示，以及测试结束时上游节点的中断/统计线程消息。未据虚拟测试宣称达到 100 Hz 或证明实机停止；后续运动控制必须单独量测循环时延，并避免在控制周期内阻塞写日志。

复核脚本：本目录 `verify_offline.sh`。该脚本隔离构建，仅运行生成的 PTY 测试，不运行实机 launch。Jazzy 构建仍有旧式生命周期接口的弃用提示，不影响本次编译；没有将提示压制后伪装成无警告构建。

## 现场操作的下一步

本轮只读枚举没有找到 COM9，WSL 也没有 `/dev/ttyUSB*`、`/dev/ttyACM*` 或 `/dev/serial/by-id` 设备。没有自动安装软件、转发 USB 或修改系统设备配置。下一次由用户接回这只 CANUSB，再确认 ROS 所在 Linux/WSL 能看到的实际串口路径；Windows 的 COM9 不能直接填进 POSIX 驱动。

在真实设备路径确认、关闭其他占用软件并完成构建后，由用户执行以下 ROS 命令（替换占位路径）：

```bash
ros2 launch slaihand_bringup slaihand_canusb_readonly.launch.py port:=/dev/<实际设备>
```

验收 `/slaihand/joint_states` 的四轴位置/速度/effort，以及 `/slaihand/dynamic_joint_states` 的状态、温度和有效标记，并保留 JSONL。只读阶段有读取 TX，验收口径是 **0 使能 / 0 运动命令 / 0 参数写入**，不是 0 TX。

只读实机链路验收后，继续在同一 owner 内接标准整臂控制器和显式运动生命周期；不另外新增一个抢占串口的控制窗口。本插件当前不导出命令接口，所以不能用它使能或运动。
