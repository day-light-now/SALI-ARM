# J3 启动圈数分支修复（2026-11-05）

现象：实物未转动整圈，重连后 J3 显示 −360.809°。此前固定加 2π 的软件零位只适合上一种原始角度分支。

恢复 canonical zero = −2.5423437857633324 rad，sign=+1。J3 采用 startup_unique_urdf：configure 时在新鲜、失能、静止反馈上，寻找落入完整 URDF ±90° 的唯一等价角度；选定有效零位后重新采集反馈，本连接内锁定。接收和 MIT 发送共用有效映射，不在运行中取模。无唯一分支或完整范围无法协议编码时拒绝配置。没有写电机零位、Flash 或参数。

运行身份新增 branch_ready、分支模式、圈数及有效零位；审计 STAGE 保存有效零位，采集工具要求分支 ready。

验证：C++ CTest 5/5；重力模型 16 项通过；verify_position_branches.py 使用新建 PTY 和真实 ROS 驱动，对 −1/0/+1 圈均通过，保持期间原始目标漂移分别 0.0001002/−0.0001829/−0.0004659 rad，没有整圈命令。

实际只读部署：/home/xuyu/slaihand_control_panel_ws_branch_20261105。四轴角度 [0.612, 0.219, −0.809, 8.284]°，均 motor_state=0、control_state=0；J3 model_turns=0、effective_zero 恢复 canonical，branch_ready=true。证据见 evidence/j3_branch_real_readonly.json。alpha=0；原用户已移除的四轴主机力矩阈值仍保持关闭，其他保护未调整。本轮未执行实机使能或轨迹。

已更新 test_env.sh 默认工作空间、运行网页及桌面快捷方式；后续网页重连沿用新版分支逻辑。硬件库 SHA256：2a77cff2a8174da752cb7fc102917d8a2291c6e98ecf1d8648a0a963276a49cc。
