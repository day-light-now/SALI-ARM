# J3 整圈表示修复（2026-11-05）

用户确认 J3 实物仍在昨天约 -20°姿态附近，没有转动整圈。新反馈原始角3.3834973678187232rad，经原软件零点-2.5423437857633324得339.525688°；在完整J3±90°区间内唯一整圈等价角为-20.474312°。只读证据见J3整圈表示只读核查.json。

实施为显式软件参考分支选择，不是反馈自动modulo：hardware_mapping.yaml 的 J3 zero_offset_rad 改为3.740841521416254（原值+2*pi）；保留 original_zero_offset_rad、software_reference_turns=1和证据路径。RX与MIT目标TX原本都消费同一mapping，因此同时使用等价参考，当前姿态反算目标仍是原始3.383497rad，不引入整圈运动目标。没有改变hardware_sign、物理零位、URDF、J3模型额外pi、增益、alpha或电机寄存器/Flash。

原映射全文备份J3整圈修正前_mapping.yaml。若未来反馈换回旧原始分支，将再次触发出限保护，不能自动恢复旧映射或自动回零；须先确认实物/原始反馈。历史数据按其原mapping哈希解释，不能用新映射重解释旧原始角。

验证：test_gravity_model.py 16/16，新增当前原始反馈映射、逆映射同一原始角、全J3范围协议可表示、旧分支不会自动折返的回归。

在新鲜stage0/motor_state全0确认后，停止旧驱动；只更新候选工作空间src和install的同一映射文件，再以alpha0、未使能方式启动。只读核对结果：四轴有效、stage0、motor_state全0；J3=-.3573441535975306rad；运行身份mapping hash与当前源码匹配。没有发送使能或运动命令。

另在实际订阅身份路径发现calibration_capture.py::_runtime_view引用未定义_finite，定点改为core._finite，实际身份检查现可成功。此处是Codex的局部修复，未接手DeepSeek其余C阶段实现。
