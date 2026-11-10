"""Write the one-click compensation report for a tag."""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
tag = sys.argv[1]
work = HERE / 'runs' / tag


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


iter1 = read(work / f'{tag}_iter1_compare.json')
iter2 = read(work / f'{tag}_iter2_compare.json')
baseline = iter2['baseline']
comp1 = iter1['compensated']
comp2 = iter2['compensated']

lines = [
    '# 一键静态重力补偿报告',
    '',
    '- tag: `%s`' % tag,
    '- 方法：基准采集 → 按实测残差修正 → 迭代 1 → 再修正 → 迭代 2（位置前馈，'
    '非转矩 FF；安全门限未改，FF alpha=0）',
    '- 每轮计划首末均为绝对零；每轮之间重启驱动以保证 DISARMED 准入。',
    '',
    '| 指标 | 基准 | 迭代 1 | 迭代 2 |',
    '|---|---:|---:|---:|',
    '| 总体 RMS（deg） | %.4f | %.4f | %.4f |' % (
        baseline['overall_rms'], comp1['overall_rms'], comp2['overall_rms']),
    '| 总体最大（deg） | %.4f | %.4f | %.4f |' % (
        baseline['overall_max'], comp1['overall_max'], comp2['overall_max']),
]
for joint in range(5):
    lines.append('| J%d 最大（deg） | %.4f | %.4f | %.4f |' % (
        joint + 1,
        baseline['per_joint_max'][joint],
        comp1['per_joint_max'][joint],
        comp2['per_joint_max'][joint]))
lines += [
    '',
    '- 原始窗口与计划：`plans/%s_*_20261110.json`、`windows_%s_*.json`。' % (tag, tag),
    '- 驱动日志：`logs/%s_*.log`；客户端结果：`clients/%s_*/`。' % (tag, tag),
    '- 若任一轮采集失败（如 USB 写故障），脚本会停止并保留部分日志，不会自动重试。',
]
report = work / f'{tag}_REPORT.md'
report.write_text('\n'.join(lines) + '\n', encoding='utf-8')
summary = {'schema': 'slaihand_one_click_compensation/1', 'tag': tag,
           'baseline': baseline, 'iter1': comp1, 'iter2': comp2,
           'report': str(report)}
(work / f'{tag}_summary.json').write_text(
    json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print('RESULT baseline_rms=%.4f iter1_rms=%.4f iter2_rms=%.4f max=%.4f' % (
    baseline['overall_rms'], comp1['overall_rms'], comp2['overall_rms'],
    comp2['overall_max']))
