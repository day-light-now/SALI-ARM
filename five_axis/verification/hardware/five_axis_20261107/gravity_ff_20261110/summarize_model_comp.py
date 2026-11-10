"""Write the one-click model-compensation report (no baseline run needed)."""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
tag = sys.argv[1]
work = HERE / 'runs' / tag


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


model = read(work / ('%s_model_compare.json' % tag))
iter1_path = work / ('%s_iter1_compare.json' % tag)
iter1 = read(iter1_path) if iter1_path.is_file() else None
model_stats = next(iter(model['runs'].values()))
iter_stats = next(iter(iter1['runs'].values())) if iter1 else None

lines = [
    '# 一键模型补偿报告（hybrid/model）',
    '',
    '- tag: `%s`' % tag,
    '- 方法：离线模型补偿（`compensate_plan.py`）→ 单次采集'
    + (' → 按实测残差修正一次 → 再采集' if iter_stats else ''),
    '- 位置前馈，非转矩 FF；FF alpha=0；安全门限未改；每轮首末绝对零。',
    '',
    '| 指标 | 模型补偿 | 模型+一次实测修正 |',
    '|---|---:|---:|',
    '| 总体 RMS（deg） | %.4f | %s |' % (
        model_stats['overall_rms'],
        '%.4f' % iter_stats['overall_rms'] if iter_stats else '—'),
    '| 总体最大（deg） | %.4f | %s |' % (
        model_stats['overall_max'],
        '%.4f' % iter_stats['overall_max'] if iter_stats else '—'),
]
for joint in range(5):
    lines.append('| J%d 最大（deg） | %.4f | %s |' % (
        joint + 1, model_stats['per_joint_max'][joint],
        '%.4f' % iter_stats['per_joint_max'][joint] if iter_stats else '—'))
lines += [
    '',
    '- 计划：`plans/%s_modelcomp_20261110.json`' % tag,
    '- 原始窗口：`windows_%s_model.json`、`windows_%s_iter1.json`；'
    '驱动日志：`logs/%s_*`。' % (tag, tag, tag),
    '- 注意：模型补偿是域内粗补偿；高精度以模型+实测修正（或原 iter 模式）为准。',
]
report = work / ('%s_REPORT.md' % tag)
report.write_text('\n'.join(lines) + '\n', encoding='utf-8')
summary = {'schema': 'slaihand_model_compensation_pipeline/1', 'tag': tag,
           'model': model_stats, 'iter1': iter_stats, 'report': str(report)}
(work / ('%s_summary.json' % tag)).write_text(
    json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print('RESULT model_rms=%.4f%s max=%.4f' % (
    model_stats['overall_rms'],
    ' iter1_rms=%.4f' % iter_stats['overall_rms'] if iter_stats else '',
    iter_stats['overall_max'] if iter_stats else model_stats['overall_max']))
print('REPORT:', report)
