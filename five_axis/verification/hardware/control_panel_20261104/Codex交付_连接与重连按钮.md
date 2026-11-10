# 网页连接与尝试重连（2026-11-05）

用户要求增加连接按钮以尝试重连。已在四轴状态卡片增加检查连接/尝试重连、进度/错误说明、故障恢复现场确认。GET /api/connection只检查设备/进程/反馈；POST /api/connection/reconnect沿用token、Origin、Host验证，异步执行，不返回虚假的已连接。

只操作当前ROS域、完整包名/launch文件名及明确串口路径全部匹配的唯一旧launch；退出和串口所有权均确认后才能新启动。无宽泛pkill，不杀其他域/进程，不启动第二条串口持有者。USBIP重连根据唯一2e88:4603适配器识别当前busid，不硬编码5-2。命令固定argv，不执行用户输入shell。过程可被全轴停止取消，未完成恢复不启动运动。

UNKNOWN/反馈未知必须明确确认已支撑并完成断电检查，用户恢复供电后再尝试；活动控制或在途操作拒绝恢复。清除旧缓存和预览，以新驱动3次新鲜四轴一致DISARMED/motor_state0确认连接。完成仍锁住动作入口；用户点击软件复位确认后，还需随后的新鲜未使能反馈解锁。没有自动复位、使能、续跑或电机参数写入。

恢复入口不等于通信根因修复。本次真实故障为SERIAL_OR_AUDIT_WRITE_FAILURE；API audit_ok=true，故障窗口没有RX、大量TX没有写入成功回执，随后.3s未确认失能进入UNKNOWN。已在用户确认支撑断电后停止旧驱动。USBIP取消记录与故障尚未时间对齐，不能据此声称唯一根因。未通过延长等待或解除UNKNOWN掩盖故障。

验证：test_panel_connection.py19项（含原停止纪元11项）通过；HTTP/模型23项通过。真实网页headless浏览器确认按钮渲染、检查连接GET响应、未勾选现场确认的UNKNOWN恢复被拒绝；没有发起已确认的真实重连或电机动作。截图evidence/panel_connection_controls.png已目视检查。实际用户点击后的完整USB重连/驱动恢复仍需现场供电验证。

部署到/home/xuyu/slaihand_control_panel_ws_torqueoff_20261105 bringup，硬件库不变，四轴主机力矩门槛仍关闭、alpha0。当前面板42663运行，桌面机械臂网页上位机.url已更新到新token。用户步骤：恢复供电→勾选断电检查确认→尝试重连→看到四轴未使能后点击软件复位→重新预览。
