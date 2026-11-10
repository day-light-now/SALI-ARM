# 校准工作空间的安全门禁变更记录（2026-11-05，DeepSeek 代记录）

对象：`/home/xuyu/slaihand_control_panel_ws_calibration_20261105`（当前实机连接的运行工作空间，ROS_DOMAIN_ID=0，串口 `/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00`）。
说明：以下变更由 Codex 在自动采集联调中实施；DeepSeek 在接管状态检查时发现此前没有对应的变更记录，故补记。**行为未改动**，仅记录现状、理由与回滚方法。

## 1. 变更前后值

| 项 | 仓库/旧基线 | 校准工作空间当前 | 位置 |
|---|---|---|---|
| `estimate_guard_nm`（请求/反馈合成估计门禁） | `[8.0, 10.0, 2.0, 2.0]` N·m | `[0.0, 0.0, 0.0, 0.0]`（0 表示关闭） | `src/slaihand_bringup/config/canusb_motion_policy.yaml` |
| 驱动 `motion_supervisor` 门禁用法 | 固定比较，超限即全停 | `guard[i-1] > 0` 时才比较；0 跳过并令 FF 限幅余量为 0 | `src/slaihand_hardware/src/motion_supervisor.cpp` |
| ros2_control 通用命令限幅 | `enforce_command_limits` 默认 true | `false`（日志：`Enforcing command limits is disabled. Command limits from URDF will be ignored.`） | `src/slaihand_bringup/config/canusb_motion_controllers.yaml` |

## 2. 变更理由（Codex 联调结论）

- 通用位置限幅器把“静止保持时的目标−实测静差”当作每周期参考位移来限速，会改写保持目标，导致参考跳变并触发驱动的 `REFERENCE_SPEED` 停止；
- 自动采集的静态样本存在约 0.8° 稳定偏差，`kp×误差` 在 J3/J4（2 N·m 门禁）会被合成估计门禁判超限，无法采到保持力矩；
- 因此在校准专用空间内关闭这两个“通用/合成估计”层，由驱动原生检查兜底。

## 3. 仍然生效的保护（核对自驱动源码与运行日志）

- 参考步长：每周期 `|ref−target| ≤ reference_speed(0.5 rad/s) × dt`，超限 `REFERENCE_SPEED` 全停；
- 实测反馈速度 `>1 rad/s` 拒绝；跟随误差 `>4°` 拒绝；反馈温度 `>50 ℃` 拒绝；四轴状态/超时/冲突校验保留；
- URDF 完整限位：最终编码位置码必须落在 `[-80,200]×[-10,190]×[-90,90]×[-90,50]°`，不可表示即 `NO_REPRESENTABLE_URDF_TARGET`；
- 45/178 秒会话看门狗、停止门禁、失能确认、审计、唯一写入口与 `alpha=0` 默认不变；
- 参考峰值速度 0.5 rad/s 仍由 `compile_plan` 独立校验。

## 4. 风险与使用范围

- **风险**：关闭合成估计门禁后，PD 静差、外部接触、堵转等不再由该层直接全停；只剩跟随误差 4°、参考步长、URDF 限位与反馈速度等保护。**仅允许在受支撑、无接触、有人看护的校准采集中使用**，不得作为正常运行策略。
- 当前 `alpha=0`，前馈未启用；校准只做静态保持采样，不做高速轨迹。
- 校准结束后必须恢复到 `[8,10,2,2]` 与 `enforce_command_limits: true`，并重新构建/重启驱动。

## 5. 回滚方法

```bash
WS=/home/xuyu/slaihand_control_panel_ws_calibration_20261105
# 1) 恢复门禁值（示例：改回 8/10/2/2）
sed -i 's/estimate_guard_nm: \[0.0, 0.0, 0.0, 0.0\]/estimate_guard_nm: [8.0, 10.0, 2.0, 2.0]/' \
  $WS/src/slaihand_bringup/config/canusb_motion_policy.yaml
# 2) 恢复通用命令限幅
sed -i 's/enforce_command_limits: false/enforce_command_limits: true/' \
  $WS/src/slaihand_bringup/config/canusb_motion_controllers.yaml
# 3) 重新构建并重启（由用户执行，面板“检查连接/尝试重连”走同一流程）
cd $WS && colcon build --packages-select slaihand_bringup slaihand_hardware
```

注意：`slaihand_hardware` 里 `guard[i-1] > 0` 的判断本身是“0=关闭”的语义扩展，恢复 `[8,10,2,2]` 后即回到原行为，无需改回源码。

## 6. 待办

- Codex 复验时确认上述变更是其有意设计，并补上变更批准记录；
- 现场采集完成后按第 5 节回滚；
- DeepSeek 未改动当前实机配置、未使能、未运动。
