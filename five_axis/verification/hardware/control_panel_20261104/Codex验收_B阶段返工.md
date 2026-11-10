# B 阶段独立验收：需返工

2026-11-05。DeepSeek 继续负责实现，Codex 只写离线审阅反例和本文，没有修改生产驱动，没有操作实机。

现有候选 CTest 4/4 通过，但新增反例复现下列三个问题。请优先集中修复，再推进 C 阶段；Kp/Kd 面板与 D 阶段稍后处理。保留此前已通过内容。

## 重现命令

在 WSL source 本目录 test_env.sh，然后执行：

```bash
python3 /mnt/d/CodexData/Projects/机械臂/slaihand_ros2_ws/verification/hardware/control_panel_20261104/run_codex_b_repro.py
```

反例复用真实 MotionSupervisor、既有 Fixture、候选构建记录的编译/链接参数；仅在 /tmp 编译可执行文件，无串口/CAN。

原样输出关键值：

```text
configured_ff_ramp=1 ff_after_2.2s=-0.234055
after_stop stage=8 active=1 ff_applied=-0.234055
configured_ff_ramp=10 ff_after_2.2s=-0.234055
after_stop stage=8 active=1 ff_applied=-0.234055
OUTSIDE_ENCODED_LIMIT axis=1 q=-1.39639
boundary_crossings=1
```

## P1：非零前馈丢失编码边界修正

motion_supervisor.cpp::controls() 先为完整 URDF 边界选择可表示的内侧位置码；随后 applied!=0 分支重新调用 native_command(...target_[i-1]...)，把原始边界目标重新编码，丢弃上述修正。最终只查力矩估计，未复查最终位置码。

反例在 J1 下限 -80°附近、alpha=.5 时发出 q=-1.39639 rad 的越界位置码；alpha=0 原边界回归不会暴露此问题。

修复要求：位置边界修正必须保留到最终含 tau 的报文；对最终编码结果验证位置及完整命令门禁。覆盖四轴上下限、双向 sign、非零 FF 及饱和 FF。不得通过缩小用户确认的 URDF 范围掩盖问题。

## P1：gravity_ff_ramp_s 不生效

controls() 的 scale 使用 p_.ramp（gain_ramp_s）；请求为 scale*p_.gravity_alpha*gravity，p_.gravity_ramp 只校验/发布，没有参与计算。因此配置 10 秒前馈斜坡实际随 1 秒增益斜坡达到输出。

修复要求：明确独立前馈斜坡的起点和每轴规则，落实 gravity_ff_ramp_s；若拒绝零时长则校验与文档同步。保持 gain ramp 原行为。用生产路径验证 1 秒/10 秒不同曲线、停止直接 FD、不以斜坡延迟停止。

## P1：已应用状态不代表实际写入/活动输出

gravity_active() 只看模型存在及配置 alpha>0，与阶段无关；ff_applied_ 在 controls() 中先写入尚未编码的请求值，未在 stop/reset/arm 清除，也未等待 Batch 全部通过及 send/written 成功。因此 DISABLED 仍发布 alpha>0 和旧非零应用值；命令整体拒绝或串口写失败时也可能展示未发出的值。

修复要求：区分模型请求、限幅后候选、最终编码值、串口已写入值；依照逐轴 written 回执更新，明确这仍不是驱动实际力矩确认。STOPPING/DISABLED/UNKNOWN 与新 command_id 不得被标作当前活动非零前馈；历史最后写入值如需保留应单列并有时间/有效性。面板按状态和数据时效显示，不只看 alpha。加入拒绝、部分写失败、停止、reset、重新 arm 回归。

## 后续交付

修正后原样运行反例并加明确断言，不以移除检查或修改真实输入制造通过；扩展现有模拟测试覆盖上述生产路径。提供紧凑交付、当前加载库路径/哈希、实际命令与结果。保留 alpha=0 金标兼容和 provisional 质量来源限制。

另外，A 阶段关于 J3 偏置物理一致性、shadow 余量和未知参考的意见仍需落实。3 分钟会话是有意配置，不恢复 45 秒旧值。点到点原 2°暂停/6 秒预算问题未因此解决，不能标为完成。
