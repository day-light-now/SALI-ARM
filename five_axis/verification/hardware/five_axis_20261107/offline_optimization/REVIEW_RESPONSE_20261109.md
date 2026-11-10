# 对 Codex 评审的修正记录（2026-11-09）

评审文件：`D:/CodexData/Projects/机械臂/CODEX_DSH_GAIN_REVIEW_20261109.md`

本轮只修改离线代码/文档，未启动 driver/panel、未使能/运动、未放宽保护、
未覆盖主工作区未提交改动。

| 评审点 | 修正 |
|---|---|
| 1. `max_abs` 无方向 | `five_axis_control_lab.py` 改为显式 `UNSIGNED_MAGNITUDE_ONLY_PATH_LOCAL`；`signed_static_data_available=false`；FF 标记 BLOCKED，不再输出可下发系数；报告写明只有末尾0.3s峰值、无符号、无时序。 |
| 2. 单路径不能泛化 | 报告明确 valid domain 仅为 `q_start+s*[1,1,1,1,1]` 正5–40°；增加独立姿态/反向/跨次运行采样计划 `NEXT_SAMPLING_PLAN_20261109.md`；不允许外推。 |
| 3. 高增益实测未读取 | `run_schemes.py` 实际读取 `GAIN_0_40_...json`，`constant_high` 输出 measured baseline / measured high / predicted high / improvement；J4 20–40°实测约9.9–26.1%，不再写30–45%。 |
| 4. 重复/理想排名 | 移除 `hybrid`；`gravity_ff` 指标 null 且 BLOCKED；`disturbance_observer` 不再硬编码0，指标 null；`integral` 归 IDEALIZED/NOT_EVALUATED；矩阵 `ranking_performed=false`。 |
| 5. 静态 vs 动态 | 报告明确1/Kp模型无惯量/时延/柔性/饱和；Kp500只是静态条件预测；动态实验需峰值/过冲/振荡/温升/停止数据。 |
| 6. 新鲜度/1.5× | 报告明确后续准入必须默认1800s、掉电后重读TIMEOUT；`existing_command_path` 不等于commissioned/acceptance；1.5×仍缺真实allowed/demand证据。 |
| 7. 测试有效性 | 新增空数据/缺轴/NaN/Inf拒绝、实测文件读取、J4改善区间、BLOCKED null、无符号FF不可下发、调度参数边界、去重/去恒真测试。 |
| 测试 | `test_control_candidates` + `test_scheme_matrix` 共 12 项，最后重跑 12/12 通过。 |

## 仍然不能做

- 不能从现有结果生成有符号重力前馈；
- 不能声称多关节工作区泛化；
- 不能把 BLOCKED/IDEALIZED 的 null 当性能；
- 不能用本次离线结果做上机准入或1.5×裕量；
- 不能据此启动任何硬件流程。
