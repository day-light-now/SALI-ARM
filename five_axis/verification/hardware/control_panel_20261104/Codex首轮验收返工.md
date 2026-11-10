# Codex首轮验收：需要定向修正

范围核对通过：manifest 的 src 范围仅新增规定控制台文件和改 CMakeLists.txt，无越界变化。Codex独立运行23项离线HTTP/模型、12项原规划测试均通过。

## 1. P1 停止纪元没有落实“新复位观测后才能再启动”

生产 `control_panel.py` 的 `_request_stop()` 递增 `_stop_generation`，但新的 `hold()/execute()` 在停止后获取到的是**新**baseline。`_require_disarmed()`只查缓存stage==0，不检查该快照是否早于停止，也没有等待手动复位的新观测。`_launch()`仅比较baseline是否又变化，因此这个顺序仍能启动客户端：

缓存新鲜stage0 → HTTP stop受理 → ROS尚未处理stop/更新状态 → HTTP hold → 202，Popen一次。

Codex已运行真实生产HTTP handler + PanelBackend的无硬件复现：

```bash
source /home/xuyu/slaihand_control_panel_ws_20261104/install/setup.bash
python3 /mnt/d/CodexData/Projects/机械臂/slaihand_ros2_ws/verification/hardware/control_panel_20261104/codex_stop_race_repro.py
```

当前 RED：stop_status=202, hold_status=202, spawn_count=1, ros_callbacks_after_stop=0。Popen mock、无ROS spin/服务/串口，不能声称这已经导致实机使能；但违反任务书和设计承诺的面板门禁，驱动兜底不能替代它。

修正完成判据：停止受理后立即锁定preview/execute/hold，保持直到明确手动复位成功且收到**因果顺序上新鲜且四轴一致**的DISARMED观测；停用/旧缓存/仅一次服务接受回复不能解锁。`_launch`最终启动关口也检查同一停止纪元门禁，关闭准备→Popen之间的取消竞态。停止不能等待文件I/O/启动锁。新增生产回调反例测试覆盖stop紧接hold/execute、启动途中stop、STOP后新preview尝试、reset缓存/服务回复不足以解锁。原复现脚本须转GREEN，不要删/改反例以凑通过。

## 2. 原样端到端未复现114 PASS

Codex原样运行run_verification.sh：最终100 checks / FAIL，停止场景进入UNKNOWN，其后心跳、退出、陈旧反馈三个场景跳过并记失败。原始结果与ROS日志已保留为 `codex_first_panel_result.json`、`codex_first_panel_verify.log`、`codex_first_panel_ros.log`，不能覆盖。ROS明确记录OPERATOR_GLOBAL_STOP后未确认新失能。假驱动errors=[]，FC=[3,3,3,3]、FD=[40,40,40,40]。需要定位是停止协议/读取竞争还是fixture/调度问题，保留真UNKNOWN和锁定，不能把UNKNOWN当通过；各场景应独立建fake实例或明确恢复，让前一失败不遮蔽其余必测。只在新增控制台/本任务测试内修，不改保护源码/阈值。

## 3. 测试脚本不得广泛杀同名进程

`run_browser_fixture.sh` / `stop_browser_fixture.sh` 按 `pkill -f port:=/dev/pts/` 和 `control_panel.py --port 18888` 清理，未绑定本任务创建的PID/进程组/ROS域；后者甚至不能证明是fake面板。改为记录本fixture的PID/PGID，并核验进程身份后只停止自己创建的实例。不要误伤别的离线或真实控制面板。

继续由DeepSeek拥有实现文件；Codex不同时重写。保持原功能/保护边界。修完按原流程交付，保留历史测试口径，增加这轮反例与实际运行结果。
