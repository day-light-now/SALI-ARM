# Codex第二轮独立验收，2026-11-04

结论：本地四轴关节控制台软件验收通过，可由用户启动面板进行实机验证。未操作实机、未发送真实ROS控制请求。

## 独立复核

- `codex_second_changes.json`：src范围只有允许的新控制台文件与CMake安装变更，无范围外变更；受保护驱动、模型、标定、策略、客户端均保持基线内容。
- 已安装的五个客户端/模块文件、三个静态页面文件与当前源码SHA256相同。
- Codex原始停止反例未改，复跑GREEN：停止202、保持409、Popen=0、停止后ROS回调=0。修正是同步锁定停止门禁，并在手动复位之后收到四轴一致、新鲜、有效DISARMED观察才解除。
- 23项离线HTTP/模型 + 12项原规划测试通过。
- 原样 `run_verification.sh`：11项生产回调反例通过；5个独立新PTY/ROS会话的端到端119项通过，输出 `RESULT: PASS (callbacks=0 e2e=0)`。保留 `codex_second_callbacks.log`、`codex_second_panel_result.json`。
- 原样启动隔离浏览器fixture，重跑browser_check.py：33项通过，包含真实页面执行、停止门禁、手动复位、目标不被轮询重置、独立路点结果展示与窄屏。保留 `codex_second_browser.log` / `codex_second_browser_result.json`。已目视检查桌面/窄屏截图。
- 之后用PID/PGID核验脚本关闭本轮fixture；真实栈8260/8264仍存活，未改其状态。

## 收尾

独立复核额外收窄了未调用的备用 `cleanup_test_stacks.sh`：由域号前缀匹配改为精确176/177，并要求32位本控制台测试会话标记。仅执行bash语法检查，未执行其清理动作。生产源码不因此改变。

## 启动及限制

保留现有驱动窗口，在另一个Ubuntu窗口：

```bash
source /home/xuyu/slaihand_control_panel_ws_20261104/install/setup.bash
ROS_DOMAIN_ID=0 ros2 run slaihand_bringup control_panel.py --port 0
```

在Windows浏览器打开打印的完整带token地址。打开仅只读观察；如阶段为已失能，可显式软件复位，再预览。执行需要当前有效预览及三项人工确认。每次动作有限，完成自动失能；“当前位置保持”为5秒。全轴停止是软件停止请求，确认以新反馈为准；UNKNOWN时保持支撑并现场切电。

本报告通过的是软件验收。模型质量、重力补偿、实机全姿态精度、总线长期稳定和物理急停不在本轮验证范围。之前J1返回0.503°略超0.5°的实机事实不因界面更新而改变。动作执行/精度通过/失能确认分开显示。运行期间保持页面活跃，心跳丢失3秒会触发停止。
