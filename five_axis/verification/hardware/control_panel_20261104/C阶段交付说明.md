# C 阶段交付说明（2026-11-05）

依据 `Codex复验_B阶段与C阶段任务.md`。本轮交付：A 阶段遗留两项修正、C 阶段只读采集/导入/预览工具、假数据反例、现场短操作单。`alpha` 保持 0，未操作实机，未改 Kp/Kd，未提交质量档案或 `CALIBRATED`。

## 1. A 阶段遗留修正

### 1.1 shadow 余量语义（参考未知不再伪造）

- 控制器参考未知时：每轴 `p_term_nm = None`、`applied_headroom_nm = None`、`predicted_headroom_nm = None`，不按零误差填充；返回 `reference_known=false` 与明确警告。
- 两套量分开命名并分别展示：
  - **已应用余量** = 门禁 − (|实际 P| + |D| + |驱动已写入前馈|)，需要参考新鲜且驱动报告活动前馈，否则为“未知”；
  - **假设补偿预测** = 门禁 − (|实际 P| + |D| + |α_pred·模型重力矩|)，`α_pred` 默认 1.0，明确标注“仅模型预测，未写入命令”。
- 面板表格改为：P 项 / D 项 / 已写入前馈 / 已应用余量 / 假设补偿预测 / 预测余量，并新增“驱动前馈活动/α/斜坡/历史最后写入/时效”行；未知项显示“未知”而不是数字。
- 测试：`test_gravity_model.py` 15 项（新增参考未知、已应用余量需驱动值、预测与应用的区分）。

### 1.2 J3 额外 π 的必要性（统一坐标链 + 具体姿态）

`j3_offset_proof.py`（离线，只读）产出 `J3偏置核验.md/.json`，全部检查 PASS：

- URDF J3 origin rpy = `[3.14023651841, -1.57079632679, 0]`，已含 π 分量；部署的 `expanded_motion.urdf` J3 origin 完全相同，**没有额外 π**。
- MuJoCo `shoulder_dynamics.xml` 的每个 body 都是相对放置：`link1/2/4` 与 URDF origin 逐位一致（0.000000°），只有 `link3` 等于 URDF origin 再乘一次 `R_axis(π)`（与 URDF 相差 180.000000°）。
- 应用一次 π 使下游 link3/link4 整体转 180°；应用 2π 与无偏置完全等价（0.000000°）——说明软件偏置只在同一处加一次，URDF origin 的 π 与它不是同一件事。
- 重力模型 `fixed_joint_offsets={slaihand_joint3: π}` 与上述放置声明一致；`2π` 模型与无偏置模型逐位相等。
- 安装声明核对：当前 policy `gravity_base=[0, 9.80665, 0]`、`gravity_j3_offset_rad=π`，与用户声明“+Y 向下、J3 实物转 180°、无额外末端负载”一致（记录为 `claim`）。
- **可判别姿态**：J1=J2=0、J3=0，J4 取 ±30° 时，两种模型对 **J2** 预测反号（Δ≈0.37/0.40 N·m），而 J3 自身几乎不变——现场应测 J2，不能拿 J3 力矩做判别。表在 `J3偏置核验.md`。
- 边界：离线只能证明坐标链与软件偏置只加一次，**不能证明实物真的转了 180°**；物理结论需现场按该表测量，噪声 >0.2 N·m 记为不可辨识。

## 2. C 阶段只读工具

- `calibration_core.py`（纯 Python，无 ROS）
  - 记录 schema：四轴位置/速度/反馈力矩估计/驱动与电机温度/反馈时效与有效位/阶段/控制器参考及年龄/驱动前馈状态/支撑/接触/运动来向/操作者与备注/模型与映射与 policy 与质量档案哈希/J3 偏置版本；
  - 校验：缺项、NaN、非四轴、反馈时效 >0.06 s、反馈无效、support/contact/motion 缺失、模型哈希缺失均判无效；
  - 排除：`support≠free` 或 `contact≠none` 的样本保留原始但**不参与纯重力拟合**；
  - 分析：每轴残差均值/标准差/符号一致率、全局重力尺度最小二乘、奇偶留出验证、可辨识性结论（静态数据不能分辨单连杆质量，需可比质量/质心或跨负载测量）、样本/姿态多样性不足时 `INSUFFICIENT`；状态只会是 `PROVISIONAL_SCALE` 或 `INSUFFICIENT`，**不会**输出 `CALIBRATED`。
- `calibration_capture.py`
  - `--record`：只读订阅 `/slaihand/joint_states`、`/slaihand/dynamic_joint_states`、控制器状态与 `/slaihand/gravity_ff_state`，等待四轴新鲜样本后按条写入 JSONL；**不发布命令、不使能、不写参数/Flash**；写完后 `chmod 0444` 作为只读原始证据。
  - `--import`：只读校验并统计无效/排除原因。
  - `--preview`：计算模型重力残差与尺度，只打印；`--save-report` 把分析报告写到独立文件（与原始数据分开），含 raw sha256 与 provenance。
- `现场步骤_C阶段.md`：最小现场步骤（A 判别姿态 2 条 → B 自由静态 ≥12 条 → 导入/预览 → 交 Codex），明确“有支撑/接触单独标”“不撞限位”“不自行改 alpha”。

## 3. 假数据反例与证据

- `test_calibration_core.py` 19 项：有效/失效 schema、时效、反馈无效、支撑/接触样本保留但排除、样本不足、姿态多样性不足、模型异常、全局尺度回收（0.9→0.900）、留出、可辨识性、`CALIBRATED` 永不出现、raw 头/坏行拒绝、报告生成。
- `fake_calibration_check.py`：生成 12 条假样本 → `--import` → `--preview --save-report`，断言报告 schema、状态 `PROVISIONAL_SCALE`、raw sha256 一致、可用样本 12。
- `j3_offset_proof.py`：上述 J3 链与姿态表，全部 PASS。
- 全套命令：`bash run_verification.sh`（新增 j3/calib/cli 三项状态位）。

## 4. 未做 / 边界

- 未生成质量档案或任何 `CALIBRATED`；未改 policy 的 `alpha`；未改当前 Kp/Kd（仍为 160/5）；未做 Kp/Kd 调参页、D 阶段扫描；点到点旧 2°/6 秒项仍是独立未完成项。
- 面板暂无标定页（预览是 CLI 只读输出）；如需要可后续加只读展示。
- 现场数据必须由用户采集并交 Codex 复核后才讨论质量档案或 alpha；当前 `alpha=0`。
