# 12:58:15 GC01提前停止：记录与界面修正

操作 `5e166c677b9446ba`；command 2；audit `canusb_motion_Y1dKMx.jsonl`。

## 已确认的停止原因

驱动先以 `SERIAL_OR_AUDIT_WRITE_FAILURE` 进入STOPPING，客户端随后报告全局停止/UNKNOWN。未看到这次先触发FOLLOWING_ERROR；冻结first_fault为空，首条停止原因来自STAGE审计，不能冒称有完整轴故障快照。

关键顺序（steady秒）：

- 3062.288102：TRACK。
- 3064.487824：cycle35370的J4 MIT帧TX_SUBMITTED，之后没有该帧TX_WRITTEN。
- 3064.493225：已进入STOPPING，开始提交各轴FD；距J4提交约5.401ms。
- 3064.504176：STAGE持久记录 `SERIAL_OR_AUDIT_WRITE_FAILURE`。
- 3064.527697：DISABLED，随后客户端发起停止复核，五轴新失能反馈已确认。

审计在事件之后持续产生正常记录，dynamic中audit_ok=1；审计队列失败会永久标failed并停止接受后续记录。这更支持串口写入未完成，时间尺度与5ms写入预算吻合，而非审计队列耗尽。底层是USB/IP延迟、串口背压、设备或其他I/O错误，现有泛化异常没有保留足够errno/写入计数来唯一确定，不能宣称已根治。

故障发生于轨迹开始约2.2秒后，第一个验收窗口到达时间8秒、结束14秒，因此九个路点样本均为0是提前停止的结果，不是九个姿态都发生精度超差。

## 另一个独立问题：这次实际提交为相对坐标

操作source.json的coordinates为 `relative_start_deg`，末点[0,0,0,0,0]因此被转换为当时起点约 `[-0.066,1.989,-2.208,-2.055,1.158]°`。这不符合用户要求的绝对零终点。

当前服务器四份GC01–GC04存档读回仍全部为 `absolute_deg`，最终目标精确五轴0。代码隔离测试也确认载入绝对GC文件时会勾选绝对模式。因此现有证据只能确定“存档与提交间坐标模式改变”，不能确定具体是哪个点击/编辑过程，不把它归咎于用户或声称已找到载入丢坐标的根因。

## 本次实际修正

只修改 `web/control_panel/panel.js`：

1. 从GC01–GC04名称的文件/存档载入时，保留绝对零终点要求。若之后切到relative或把末点编辑为非零，planFromEditor返回错误，预览/执行按钮流程不能提交该文档。普通自定义相对轨迹保持原行为。这是界面误操作检查，不是新增硬件安全认证或后端访问控制。
2. samples=0的结果行显示“未采样”；所有行都无样本时，汇总显示“未采到验收窗口”。底层operation仍为失败，不显示通过。

当前面板静态文件位于dsh_sched安装目录；已先比较HTTP服务文件与目标文件哈希，备份原JS，再原子替换**这一份静态资源**。HTTP读回与源码完全一致。备份 `panel.before.js`、前后哈希 `static_update_receipt.json`。没有重启driver或panel，没有修改串口deadline/EAGAIN、限位或跟随误差保护。

页面刷新后载入GC01–GC04才启用该界面检查；不会自动替换用户当前编辑计划，不会重试运动。

## 验证

`node verification/hardware/five_axis_20261107/test_panel_zero_contract.js`：修改前在“GC相对模式应被拒绝”断言失败；修改后通过。运行真实applyPlan/planFromEditor/renderOperationBody函数，DOM原语由内存节点提供，不使用ROS/设备。

`node --check src/slaihand_bringup/web/control_panel/panel.js`：通过。

`python -B verification/hardware/five_axis_20261107/test_five_axis_panel_http.py`：7/7通过。

`git diff --check`：通过。

串口故障**尚未修复、未通过实机复验**；没有自动reset、使能或重试。此次数据不能用于质心拟合、补偿效果比较或任何裕量验收。
