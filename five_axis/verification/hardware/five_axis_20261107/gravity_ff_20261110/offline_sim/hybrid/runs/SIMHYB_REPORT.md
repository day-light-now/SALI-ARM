# 一键模型补偿报告（hybrid/model）

- tag: `SIMHYB`
- 方法：离线模型补偿（`compensate_plan.py`）→ 单次采集 → 按实测残差修正一次 → 再采集
- 位置前馈，非转矩 FF；FF alpha=0；安全门限未改；每轮首末绝对零。

| 指标 | 模型补偿 | 模型+一次实测修正 |
|---|---:|---:|
| 总体 RMS（deg） | 0.0804 | 0.0074 |
| 总体最大（deg） | 0.1913 | 0.0199 |
| J1 最大（deg） | 0.1773 | 0.0199 |
| J2 最大（deg） | 0.1913 | 0.0123 |
| J3 最大（deg） | 0.0893 | 0.0050 |
| J4 最大（deg） | 0.1150 | 0.0113 |
| J5 最大（deg） | 0.1037 | 0.0014 |

- 计划：`plans/SIMHYB_modelcomp_20261110.json`
- 原始窗口：`windows_SIMHYB_model.json`、`windows_SIMHYB_iter1.json`；驱动日志：`logs/SIMHYB_*`。
- 注意：模型补偿是域内粗补偿；高精度以模型+实测修正（或原 iter 模式）为准。
