# 串口/审计写故障日志分析（离线）

- 扫描根：/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain/verification, /home/xuyu/.ros/slaihand
- 关键词计数：{"serial_write_fault": 33, "process_died": 6, "disabled_or_stop": 11, "write_supervision_failure": 19}

## 按日期

- 2026-11-05: 11
- 2026-11-06: 2
- 2026-11-07: 7
- 2026-11-09: 4
- 2026-11-10: 28

## 按小时

- 09 时: 3
- 10 时: 23
- 11 时: 1
- 12 时: 5
- 13 时: 3
- 14 时: 12
- 15 时: 5

## 按文件前缀/tag

- driver: 57
- panel: 2
- F4fr: 1
- gravitycal: 1
- J5A: 1
- J5B: 1
- JVc: 1
- smoke1014: 1
- smoke1015: 1
- smoke1016: 1
- T1a: 1
- V1c: 1

## 涉及文件（前 15）

- /home/xuyu/.ros/slaihand/control_panel/connection/driver_20261105_130908_818637.log: 10
- verification/hardware/five_axis_20261107/gravity_ff_20261110/logs/driver_launch_20261110_101541.log: 9
- verification/hardware/five_axis_20261107/gravity_ff_20261110/logs/driver_launch_20261110_095416.log: 5
- verification/hardware/five_axis_20261107/five_dof_refit_gc05_08_20261109/execution_20261109/logs/driver_launch_session1.log: 4
- verification/hardware/five_axis_20261107/gravity_ff_20261110/logs/driver_launch_20261110_105452.log: 4
- verification/hardware/five_axis_20261107/gravity_ff_20261110/logs/driver_launch_20261110_105653.log: 4
- /home/xuyu/.ros/slaihand/control_panel/connection/driver_20261107_115813_94765.log: 4
- verification/hardware/five_axis_20261107/gravity_ff_20261110/logs/driver_launch_20261110_104907.log: 3
- verification/hardware/five_axis_20261107/gravity_ff_20261110/logs/driver_launch_20261110_105209.log: 3
- /home/xuyu/.ros/slaihand/control_panel/connection/driver_20261105_153629_997996.log: 3
- verification/hardware/five_axis_20261107/gravity_ff_20261110/logs/driver_launch_20261110_122755.log: 2
- /home/xuyu/.ros/slaihand/control_panel/connection/driver_20261107_131252_371877.log: 2
- /home/xuyu/.ros/slaihand/control_panel/connection/driver_20261106_144449_816197.log: 2
- verification/hardware/five_axis_20261107/gravity_ff_20261110/logs/driver_launch_20261110_105002.log: 1
- verification/hardware/five_axis_20261107/gravity_ff_20261110/logs/F4fr_20261110.log: 1

## 最近 10 条命中

- [2026-11-05 13:59:23] /home/xuyu/.ros/slaihand/control_panel/connection/driver_20261105_130908_818637.log:185 `[ros2_control_node-2] [WARN] [1793858363.049310781] [canusb_system]: Control output disabled; host disabled observations=true; reason=WRITE_SUPERVISION_FAILURE`
- [2026-11-05 14:11:24] /home/xuyu/.ros/slaihand/control_panel/connection/driver_20261105_130908_818637.log:218 `[ros2_control_node-2] [WARN] [1793859084.699065785] [canusb_system]: Control output disabled; host disabled observations=true; reason=WRITE_SUPERVISION_FAILURE`
- [2026-11-05 14:31:00] /home/xuyu/.ros/slaihand/control_panel/connection/driver_20261105_130908_818637.log:229 `[ros2_control_node-2] [WARN] [1793860260.309150598] [canusb_system]: Control output disabled; host disabled observations=true; reason=WRITE_SUPERVISION_FAILURE`
- [2026-11-05 14:47:47] /home/xuyu/.ros/slaihand/control_panel/connection/driver_20261105_130908_818637.log:280 `[ros2_control_node-2] [WARN] [1793861267.646160537] [canusb_system]: Control output disabled; host disabled observations=true; reason=WRITE_SUPERVISION_FAILURE`
- [2026-11-05 14:47:47] /home/xuyu/.ros/slaihand/control_panel/connection/driver_20261105_130908_818637.log:281 `[ros2_control_node-2] [WARN] [1793861267.706134909] [canusb_system]: Control output disabled; host disabled observations=true; reason=WRITE_SUPERVISION_FAILURE`
- [2026-11-05 14:47:47] /home/xuyu/.ros/slaihand/control_panel/connection/driver_20261105_130908_818637.log:282 `[ros2_control_node-2] [WARN] [1793861267.876250775] [canusb_system]: Control output disabled; host disabled observations=true; reason=WRITE_SUPERVISION_FAILURE`
- [2026-11-05 14:47:48] /home/xuyu/.ros/slaihand/control_panel/connection/driver_20261105_130908_818637.log:283 `[ros2_control_node-2] [WARN] [1793861268.506151664] [canusb_system]: Control output disabled; host disabled observations=true; reason=WRITE_SUPERVISION_FAILURE`
- [2026-11-05 14:47:48] /home/xuyu/.ros/slaihand/control_panel/connection/driver_20261105_130908_818637.log:286 `[ros2_control_node-2] [WARN] [1793861268.686171449] [canusb_system]: Control output disabled; host disabled observations=true; reason=WRITE_SUPERVISION_FAILURE`
- [2026-11-05 14:47:48] /home/xuyu/.ros/slaihand/control_panel/connection/driver_20261105_130908_818637.log:287 `[ros2_control_node-2] [WARN] [1793861268.786156764] [canusb_system]: Control output disabled; host disabled observations=true; reason=WRITE_SUPERVISION_FAILURE`
- [2026-11-05 14:47:50] /home/xuyu/.ros/slaihand/control_panel/connection/driver_20261105_130908_818637.log:290 `[ros2_control_node-2] [ERROR] [1793861270.766120827] [canusb_system]: UNKNOWN: four-axis disabled observations NOT confirmed; keep physical support and cut moto`

## 初步结论

- 命中集中在 CANUSB 写串口包不完整（`incomplete serial packet write`）及其上层 `SERIAL_OR_AUDIT_WRITE_FAILURE`；驱动随后 DISABLED，客户端停止。
- 需要结合具体 tag 判断是否与运动/重启相关；处理原则仍是停止→确认失能→面板复位→最多重试一次，不连续重试。
