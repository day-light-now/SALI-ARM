# C 阶段现场操作单（短版，2026-11-05 返工版）

## 网页自动采集入口（2026-11-05 更新）

桌面打开“机械臂网页上位机”。本入口保持 α=0、原有 Kp/Kd，不修改质量档案。

1. 确认四轴反馈在线；若为已失能，手动软件复位，等待新鲜未使能状态。UNKNOWN 不可复位或继续执行。
2. 在“重力采样组”选择“自动采集 G01～G06（12姿态）”，点击“载入自动采集计划”。这里只载入数据，不运动。
3. 点击“服务器预览”：完整绝对姿态、整段路径和实际时间均需检查。训练批次每段移动7.5秒、停留6秒，总计划162秒，落在既有180秒驱动会话内。单组默认每段移动10秒、停留6秒；服务器拒绝越限/过快计划。
4. 勾选原有现场确认，以及“自由承重、无手扶或外部接触；防坠支撑不承担重量”，由用户点击执行。保持页面前台和心跳有效。
5. 程序自动等待到位参考和至少2秒持续静态窗口，采集四轴实际姿态/速度/估计力矩及运行身份，逐点独占写入原始 JSONL。任一点缺失/不稳/反馈失效会中止，不自动重试、复位或重新使能。
6. 等待结果显示“合格12/12”和新失能确认；然后由用户手动复位，选择 G07，再预览并显式执行。G07 两点仅作独立验证，记录 role=holdout，不混入训练尺度。
7. “采样合格”与“位置精度通过”分开显示。0.5°精度判据保留；采样按实际姿态记录，JTC采样动作的操作到位容差为4°，驱动仍逐周期执行原来的4°跟随误差、0.5rad/s参考速度、1rad/s反馈速度、完整URDF限位等保护。原生通路不使用会按静态误差改写保持目标的通用位置速度限幅器。

动作结果和 raw 记录位于该操作的 client/calibration/raw 子目录，网页显示具体位置。仅“动作结束”或用户回复“OK”不能代替采样合格结果。以下命令行工具保留供离线核验，现场优先使用网页，不需要手动计时/按Enter采样。

前提：`alpha=0`、Kp/Kd 不改、无额外末端负载、驱动在线。**工具本身不发运动、不使能**；姿态调整由用户独立完成。采样期间不要手动扶压、不要加减负载。

## A. 静态几何方向核验（工具不发运动；姿态由用户摆好）

在 `J3偏置核验.md` 的判别姿态上各采一条（工具只读；**需要使能稳定保持，不能用手掰着读**）：

```bash
cd /mnt/d/CodexData/Projects/机械臂/slaihand_ros2_ws/verification/hardware/control_panel_20261104
python3 calibration_capture.py --record --trigger enter --support free --contact none \
  --motion static --count 1 --tag j3_discriminator --load-id no_end_load \
  --note "J1=0 J2=0 J3=0 J4=-30" --out ~/.ros/slaihand/calibration
```

- 姿态 1：J1=0°，J2=0°，J3=0°，J4=−30°；姿态 2：J4=+30°。每次用正常使能流程保持稳定后按 Enter 采集。
- 判别的是 **J2** 力矩符号（两种模型预测相反，Δ≈0.37/0.40 N·m）；J3 自身几乎不变，不能拿 J3 力矩判别。
- 两姿态只是**暂定模型候选判别**：单条约 0.2 N·m 的符号证据不足以证明物理安装；需结合实际角度、PD 误差、反馈量化与摩擦不确定性，由 Codex 复核。
- 失能（未使能）反馈只能做几何位置检查，不能当作静态保持力矩。

## B. 静态重力样本（自由支撑，用户确认安全后）

每个姿态保持静止，按 Enter 触发一个持续窗口（默认 2 秒、10 个新反馈样本；**不允许在自动计时中边掰动边被当静态采样**）：

```bash
python3 calibration_capture.py --record --trigger enter --support free --contact none \
  --motion static --count 1 --tag free_static --load-id no_end_load \
  --note "自由支撑、无接触、无末端负载" --out ~/.ros/slaihand/calibration
```

- 重复执行；建议 J1 至少 3 个不同角度（跨度 ≥ 20°）、J2 至少 3 个（≥ 20°）、J3/J4 各至少 3 个（≥ 15°），都在完整 URDF 限位内，不要为凑数据撞限位。
- 有支撑或外部接触的样本单独采：`--support supported --contact table`（会保留原始但**不参与拟合**）。
- 未知支撑不能默认 free：`--support` 必须明确；`unknown` 的样本只会被排除。

## C. 导入与只读预览

```bash
python3 calibration_capture.py --import ~/.ros/slaihand/calibration/raw/<文件>.jsonl
python3 calibration_capture.py --preview ~/.ros/slaihand/calibration/raw/<文件>.jsonl \
  --save-report ~/.ros/slaihand/calibration/reports/<文件>.json
```

- 预览只读：不写质量档案、不改 alpha、不生成 `CALIBRATED`；报告路径不得与原始文件相同（含符号链接，工具会拒绝）。
- 状态只可能是 `PROVISIONAL_SCALE` 或 `INSUFFICIENT`。采集时工具会订阅驱动发布的 `/slaihand/runtime_identity`（latched + 每 2 s 刷新）并核对哈希/重力/J3/负载；**未收到、过期或与本地文件不一致**时该批只作诊断，不能作为可信驱动标定——先检查驱动是否运行同一版本。
- 原始 JSONL 写入后自动 `chmod 0444`；这只是权限设置，不是篡改证明，请保留原始文件并另存哈希。

## D. 判读规则（Codex 核验用）

- 仅“使能保持 + 自由无接触 + 持续静止窗口 + 版本一致 + 运行时身份已验证”的样本进入拟合；
- 负/零尺度、符号一致率过低、留出 RMSE 明显偏大一律 `INSUFFICIENT` 并写明原因；
- 本工具主动只拟合一个全局重力尺度（范围选择，不是“静态数据只能如此”的定理）；
- 只有 Codex 复核现场数据后，才讨论质量档案与 `alpha`；当前 `alpha=0`。

## E. 需要用户完成的最小测量

1. 沿用已有安装声明（+Y 向下、J3 实物转 180°、无末端负载）；**不再重复确认**，只有真实矛盾时再询问；
2. 按 A 采 2 条判别姿态（使能保持、自由无接触）；
3. 按 B 采 ≥12 条自由静态姿态（或提供可比质量/质心资料）；
4. 把 raw 文件、`--import/--preview` 输出与报告交给 Codex。
