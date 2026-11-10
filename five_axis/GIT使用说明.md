# SLAI 机械臂代码版本管理

当前仓库位于 `D:/CodexData/Projects/机械臂/slaihand_ros2_ws`。

2026 年 11 月 5 日的检查点保存在分支 `codex/control-panel-20261105`，标签为 `snapshot/2026-11-06-control-panel`。原 `main` 分支和 `baseline-2026-10-09` 标签保留历史基线。当前是本地 Git 仓库，远程地址待配置。

检查点包含 CANUSB 驱动、网页面板、重力模型和采集工具、自由拖动、J3 启动分支、1 rad/s 参考速度、J4 −50°～+90°限位及 D01 展示计划。提交身份为仓库本地的 `Codex <codex@local.invalid>`，用于记录自动化整理的源码检查点。自己提交前可配置个人姓名和邮箱。

## 管理范围

- `src/`：正式 URDF、当前 CAD 网格、ROS 2 驱动/控制与启动配置、上游 OpenArm CAN 源码及原许可证。
- `verification/`：重力估算、仿真联动、肩部安装验证、MuJoCo 阻抗演示、质量审计和测试。
- `desktop_entries/`：桌面 `SLAI机械臂控制` 中启动脚本与说明的版本快照；不是实时链接，修改桌面入口后需重新复制并检查差异。
- 根目录：构建脚本、Windows 面板与项目说明。

SolidWorks 源装配、旧 mmhand 项目、实机固件和外部依赖位于各自的工程目录。正式网格与整理后的验证证据纳入版本。构建目录、临时部署脚本、带 token 的网址、串口原始流和生成日志由 `.gitignore` 排除，并继续保存在本机。

带运行身份指纹的 URDF、映射、策略和重力参数由 `.gitattributes` 保留原始字节，使克隆后的 SHA256 与提交前一致。修改这些文件时需同步验证其来源身份。

## 日常操作

在 PowerShell 中进入仓库。

```powershell
cd 'D:/CodexData/Projects/机械臂/slaihand_ros2_ws'
git status
git diff
git log --oneline -5
```

首次自己提交前设置身份，把下面占位内容换成你自己的：

```powershell
git config user.name "你的姓名"
git config user.email "你的邮箱"
```

改完先跑相关测试，检查差异，再明确添加文件：

```powershell
git add verification/impedance_demo/app.py
git diff --cached
git commit -m "说明本次修改"
```

新功能可先开分支：`git switch -c feature/功能名`。不要使用 `git reset --hard` 或强制清理来找回旧代码；需要恢复时先检查是否有未提交修改。

## 运行与依赖

源码默认策略的重力前馈 alpha 为 0。实机候选工作空间为 `/home/xuyu/slaihand_control_panel_ws_speed1_20261105`，其普通位置模式 alpha=0.1，自由拖动功能显式开放。构建与验证入口位于 `verification/hardware/control_panel_20261104/`，依次使用 `stage_workspace.sh`、`test_env.sh` 和 `run_verification.sh`。`prepare_float_candidate.py` 生成候选部署配置。

切换 Git 版本影响源文件，已经安装并运行的 ROS 工作空间使用自己的构建副本。更换电脑时需要安装 ROS Jazzy 和依赖，并调整脚本中的本机路径。实机原始采集数据位于本机 `~/.ros/slaihand/`，模型候选、来源指纹和整理后的验证报告已纳入版本。

桌面入口快照同步（仅复制已知启动目录；先确认无正在编辑的入口文件）：

```powershell
Copy-Item -LiteralPath 'C:/Users/ASUS/Desktop/SLAI机械臂控制/01_四轴仿真' -Destination './desktop_entries' -Recurse -Force
Copy-Item -LiteralPath 'C:/Users/ASUS/Desktop/SLAI机械臂控制/使用说明.md' -Destination './desktop_entries/使用说明.md' -Force
git diff -- desktop_entries
```

## 本地完整备份

备份文件位于 `D:/CodexData/Projects/机械臂/git_backups/slaihand_ros2_ws_20261106.bundle`，包含提交、分支和标签。可以克隆到另一个目录供同事查看。

```powershell
git bundle verify '../git_backups/slaihand_ros2_ws_20261106.bundle'
git clone '../git_backups/slaihand_ros2_ws_20261106.bundle' '../slaihand_from_backup'
```

后续生成新的备份。

```powershell
git bundle create '../git_backups/slaihand_ros2_ws_next.bundle' --all
```

## 远程协作

确定公司 GitLab、Gitee 或 GitHub 的仓库地址后，可添加地址并推送。

```powershell
git remote add origin '替换为公司的仓库地址'
git push -u origin codex/control-panel-20261105
git push origin snapshot/2026-11-06-control-panel
```
