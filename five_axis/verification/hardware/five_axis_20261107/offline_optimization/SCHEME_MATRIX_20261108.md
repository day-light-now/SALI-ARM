# 离线方案矩阵（2026-11-09 修订）

本表是**证据分类**，不是性能排名。没有任何方案被上机准入或验收。

| 方案 | 类别 | 实测指标 | 有符号数据 | 可下发 | 说明 |
|---|---|---|---|---|---|
| `constant_160_5` | MEASURED | 有 | 无 | 是（现有常值路径） | 真实 baseline |
| `constant_high` | MEASURED_COMPARISON | 有（measured/predicted 分开） | 无 | 是（实验常值路径） | J4 20–40°仅约10–26%改善 |
| `angle_schedule_kp` | STATIC_PREDICTION | 条件预测 | 无 | 否 | 280→500；无C++实现 |
| `gravity_feedforward` | BLOCKED | null | **缺** | 否 | max_abs 无方向，不能拟合有符号FF |
| `integral_trim` | IDEALIZED / NOT_EVALUATED | null | 无 | 否 | 缺anti-windup/电流/限幅 |
| `disturbance_observer` | BLOCKED | null | 无 | 否 | 缺真实电流/速度/母线；不能硬编码0排名 |
| `model_based_gravity` | BLOCKED | null | 无 | 否 | 缺J5/新负载几何、质量、COM |

## 关键数值

真实 40° 最大无符号误差：

```text
J1: baseline 3.564° -> high measured 2.013°
J2: baseline 3.805° -> high measured 2.275°
J4: baseline 2.843° -> high measured 2.100°  (改善26.1%)
```

J4 在20/30/35/40°的实测改善约为9.9/12.2/17.6/26.1%，不是30–45%。

## 命令

```bash
cd /mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain
python3 verification/hardware/five_axis_20261107/offline_optimization/run_schemes.py --scheme all
```

单独运行：

```bash
python3 .../run_schemes.py --scheme constant
python3 .../run_schemes.py --scheme constant_high
python3 .../run_schemes.py --scheme angle_schedule
python3 .../run_schemes.py --scheme gravity_ff
python3 .../run_schemes.py --scheme integral
python3 .../run_schemes.py --scheme disturbance_observer
python3 .../run_schemes.py --scheme model_gravity
```

## 禁止

- 不把 `gravity_ff` 的 unsigned 多项式当有符号重力前馈；
- 不把 `hybrid` 重复项当独立方案；
- 不把 BLOCKED/IDEALIZED 的 null 指标当0或参与排名；
- 不据本表启动 driver/panel、使能、运动或放宽 artifact 新鲜度；
- 不用协议 TMAX、`Kp*error` 或本表做 1.5× 裕量结论。
