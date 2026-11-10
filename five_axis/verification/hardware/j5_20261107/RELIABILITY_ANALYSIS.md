# J5 只读实机可靠性复核

日期：2026-11-07  
候选：`54ddb5c`（已由 Codex 部署为 `/home/xuyu/slaihand_j5_readonly_54ddb5c`，42664/domain177）  
本报告仅做离线证据分析：未打开串口、未 attach/detach USB、未重启页面/驱动、未操作 domain 0，未改动 frame writer、背压次数、5 ms 帧写 deadline 或 250 ms 反馈门禁。

## 1. 证据清单

路径：`D:/CodexData/Projects/机械臂/slaihand_ros2_ws/tmp/j5_connect_20261107/`

- `windows_direct_54ddb5c.json`
- `wsl_comparison_54ddb5c.json`
- `j5_fault_54ddb5c.json`
- `j5_observation_54ddb5c.json`
- `j5_usbmon_54ddb5c_target_only.txt`
- 对应 `windows_direct_probe.py`、`wsl_direct_comparison.py`（只读查看，未由本轮执行）

离线摘要：`evidence/j5_reliability_analysis.json`，生成脚本 `analyze_reliability_evidence.py`。

## 2. 实测结果

### 2.1 部署页面

- 421 TX / 421 RX 后，连续 3 次零字节 EAGAIN；
- `WRITE_BACKPRESSURE` 正确触发，串口释放；
- `connected=false`、`fresh=false`、`feedback=null`；
- `last_rx_age_s=.0614`、`last_tx_age_s=.1676`，最后一次 EAGAIN `waited_s=.005194`；
- 这不是旧 software regression 的“沉默旧角度/假 online”；旧 regression 已修复，本次是真实链路背压被正确上报。
- 一次显式只读重连后 120.16 s、2364 TX / 2363 RX，connected/fresh，last_write complete 30/30。

### 2.2 Windows 原生对照

`windows_direct_54ddb5c.json`：

- Windows native COM9，同一适配器/J5/协议；
- 60.847 s、1207 次请求全部成功；
- max roundtrip **7.203 ms**，>50 ms **0 次**；
- state 0，寄存器 7=0/8=5/9=0/10=1/21=12.5/22=10/23=28。

### 2.3 WSL USBIP 对照

`wsl_comparison_54ddb5c.json`：

- WSL serial over USBIP，重新 attach 后同样 stop-and-wait 脚本；
- **第 3 次查询即 `SerialTimeoutException: Write timeout`**；
- 仅 2 个寄存器完成，耗时 0.271 s，无反馈；
- 同一时刻 Windows 原生路径 60 s 无一次 >50 ms。

### 2.4 USBmon

目标过滤 `VID 2e88 / PID 4603`、`Bo:1:008:5`：

- Codex 严格窗口统计：**2695 次完成，中位 1.992 ms，max 804.133 ms，331 次 >50 ms，无 USB 错误码**；
- 本报告对目标文件独立重算：2777 次完成，中位 1.899 ms，p90 66.654 ms，p95 176.398 ms，p99 502.380 ms，max 804.133 ms，>50 ms 331 次、>100 ms 214 次、>500 ms 30 次，USB 错误 0；
- 完成数差异很可能来自 USBmon 文件边界包含严格页面窗口之外的 completion，outlier 计数与 max 一致；结论不受影响。

## 3. 已证实与未证实

### 已证实

1. `54ddb5c` 的故障处理契约正常：真实 EAGAIN 被分类、串口释放、旧角度隐藏、重连后重采零提示；不是原来的假 online 回归。
2. Windows 原生路径稳定（1207/1207，max 7.203 ms，0 次 >50 ms），同适配器同 J5。
3. WSL/USBIP 路径在同一脚本上第 3 次查询即超时，仅 2 个寄存器。
4. USBmon 目标端点存在真实 50 ms+、500 ms+ 完成延迟，且无 USB 错误码；这是链路完成延迟，不只是旧 pyserial post-write 等待机制。
5. 一次重连后 120 s 通过，说明设备仍可读；不能把单次失败归因于电机/驱动使能。

### 未证实

- WSL/USBIP 转发路径的具体子原因：`usbipd`/`vhci_hcd`、虚拟网络/NAT、宿主调度/DPC/IRQ、过滤驱动等均未定论；
- 不能绝对排除物理适配器、host USB 栈或扩展坞事件；
- 不能宣称可靠性已修复，也不能宣称当前页面可以开始零位/限位采集；
- `WRITE_BACKPRESSURE` 是真实链路背压的正确诊断，不是软件修复失败。

## 4. 下一步最小定位建议（只读，不由本轮执行）

1. **同适配器本地直连 WSL 对照**：由 Codex 在安全窗口把同一 J5 适配器直接 attach 给 WSL（不经 USBIP），跑同一 stop-and-wait 只读脚本；若本地直连稳定而 USBIP 不稳定，可将问题缩小到转发路径。
2. **双侧 USBmon + usbip/vhci 日志**：同时采集 Windows host 与 WSL guest 的 USBmon、`usbipd`/`vhci_hcd` 状态与队列计数，把 50 ms+/804 ms completion 对应到具体 URB 和转发事件。
3. **不改配置的调度观测**：记录 WSL vCPU 调度/软中断/宿主 CPU 与 USBIP 进程延迟，确认长 completion 是否与 guest/host 调度相关；暂不调整网络、VPN、OS 安全或任何驱动参数。
4. **保持 Windows COM9 作为稳定对照**；若 WSL 本地直连也出现长尾，再检查 host USB/适配器；若本地直连稳定，优先排查 USBIP 转发。
5. **不调整** 5 ms 帧写 deadline、3 次背压阈值、250 ms 反馈门禁；不恢复零位/限位采集，不自动部署，不发展五轴运动。

## 5. 边界

- 本轮仅新增离线分析脚本/报告和派生证据，未修改候选 frame writer 或保护参数。
- 未使用真实串口/USB/页面/驱动/domain 0；未操作 8-3 的另一只 CAN 适配器。
- 现场新版只读页面由 Codex 管理；本报告不替代实机可靠性验收。
