# SALI-ARM

四轴与五轴机械臂 ROS 2 控制/标定代码整理快照（2026-11-10）。

本仓库把两套工作区的**源码、配置、文档和关键验证摘要**放在一起，便于查阅与复现：

- `four_axis/`：早期四轴（J1–J4）控制栈、网页控制面板、拖动示教/重力浮动实验与重力标定资料。
- `five_axis/`：五轴（J1–J5，含 J5 电机盒/3.5 kg 负载盒）控制栈、J5 准入与看门狗、静态位置前馈重力补偿、重力/摩擦标定实验记录。

> 本仓库是**工程快照**，不是量产发布，也不是硬件验收报告。原始日志、原始采样（`.gz`/`.jsonl`）、崩溃转储、构建产物和厂商手册已排除；网页面板的本地 token 已做脱敏处理。

## 目录结构

```text
four_axis/
  README.md                       # 四轴工作区原始说明
  OpenArm四轴适配说明.md
  OpenArm五轴适配说明.md
  安装与测试结果.md
  软件与硬件并行任务.md
  build_workspace.sh / setup_ros2.sh / ros_profile.sh
  panel_windows.py, test_*.py, verify_*.py
  src/
    openarm_can/ slaihand_bringup/ slaihand_description/ slaihand_hardware/ slaihand_transport/
  slai_console/                   # 离线仿真/可视化控制台
  desktop_entries/
  verification/                   # 验证文档、JSON 结果与脚本（不含原始数据）

five_axis/
  README.md                       # 五轴工作区原始说明
  HANDOFF_20261109.md             # 五轴交接记录（含 GC09–GC12、静态补偿、J5、摩擦实验）
  OpenArm五轴适配说明.md
  安装与测试结果.md
  build_workspace.sh / setup_ros2.sh / ros_profile.sh
  panel_windows.py, test_*.py, verify_*.py
  src/                            # 五轴版 ROS 2 包（五轴 URDF、J5 模型/准入、驱动、面板）
  skills/slaihand-five-axis-handoff/   # 交接 skill
  slai_console/
  desktop_entries/
  verification/                   # 验证文档、JSON 结果与脚本（不含原始数据）
```

## 两套硬件配置

| 项目 | 四轴 | 五轴（含电机盒） |
|---|---|---|
| 关节 | J1–J4 | J1–J5 |
| 主要模型 | `slaihand_four_axis.urdf` | `slaihand_five_axis.urdf`（J5 为未 CAD 标定的占位几何） |
| 负载 | 无末端负载（`no_end_load`） | 3.5 kg 电机盒（`load_3p5kg_provisional.yaml`）；后期实验曾拆盒，对应 `no_end_load` |
| 当前用途 | 控制面板/拖动示教/重力浮动实验 | 五轴运动、J5 准入、静态重力补偿标定 |

## 状态与限制（务必阅读）

**五轴：**

- 已实现并验证的是**位置前馈静态重力补偿**（指令位置减去预测下垂量），不是转矩前馈。
- 转矩 FF、自由拖拽/零力示教**尚未 release**：当前测量链（`effort` 为 PD 重建估计、无独立力矩/电流传感器）和机构摩擦（实测约 0.55–1.1 Nm）使静态重力模型残差约 0.65 Nm，达不到安全浮动所需精度。
- 安全不变量不得放宽：4° 跟随误差、STOP/UNKNOWN、URDF 限位、反馈新鲜度/会话准入、J5 watchdog 2000。
- 临时负载（provisional load）路径禁止 `gravity_ff_alpha>0` 与 `gravity_float_enabled`；FF alpha 默认 0。

**四轴：**

- 控制面板、拖动示教和重力浮动均为**本地候选**，实机验证有限；浮动会话需要操作者托住机械臂并在结束时承接重量，不能保证任意姿态悬停。
- 最新部署面板支持可调 Kd（0–5）与 1–120 s 浮动会话，但单次时长有硬上限，不是长时连续拖拽。

> 任何实机运动前请确认支撑、净空和可切电；验证 JSON 与文档不代表量产或安全认证。

## 构建与运行

环境：Ubuntu 24.04（WSL2 可用）、ROS 2 Jazzy、colcon、USB-CAN 适配器。

```bash
source /opt/ros/jazzy/setup.bash
# 进入某个子目录的 src 工作区后
colcon build --symlink-install
source install/setup.bash
```

各子目录的 `README.md`、`安装与测试结果.md`、`HANDOFF_20261109.md` 和 `verification/` 下的文档说明了具体启动方式、配置文件和实验步骤。硬件映射、限位、零位和临时负载声明都在 `src/slaihand_bringup/config/` 中，使用前请逐项核对。

## 数据与来源

- 四轴快照来自 `slaihand_ros2_ws` 的 `codex/control-panel-20261105` 分支（a954f8f）。
- 五轴快照来自 `slaihand_five_axis_ws_gain` 的 `feature/five-axis-gain-test-20261108` 分支（9b4267d）。
- 已排除：`build/`、`install/`、`log/`、`tmp/`、原始 `.jsonl/.gz` 采样、`.dmp` 崩溃转储、`.obj/.STL` 验证网格、厂商手册与个人目录；网页面板 token 已替换为 `<redacted-panel-token>`。
- 所有权与使用许可：内部工程资料，未附开源许可证；对外使用请联系仓库所有者。
