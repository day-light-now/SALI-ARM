"""Scan recorded logs for serial/audit write faults and summarize their context.

Offline only. Searches *.log under the five-axis verification tree and the
~/.ros/slaihand logs (if present), extracts matching lines with timestamps and
nearby context, and writes a JSON + Markdown characterization.
"""
import json
import os
import re
import time
from collections import Counter, defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SEARCH_ROOTS = [ROOT / 'verification', Path.home() / '.ros/slaihand']
KEYWORDS = {
    'serial_write_fault': [r'incomplete serial packet write', r'SERIAL_OR_AUDIT_WRITE_FAILURE'],
    'write_supervision_failure': [r'WRITE_SUPERVISION_FAILURE'],
    'process_died': [r'process has died', r'Aborted \(Signal sent by tkill'],
    'disabled_or_stop': [r'DISABLED', r'停止', r'STOP'],
}
TIME_RE = re.compile(r'(\d{4}-\d{2}-\d{2}[T ]\d{2}:\d{2}:\d{2}(?:\.\d+)?)')
EPOCH_RE = re.compile(r'\[(\d{10,}(?:\.\d+)?)\]')
FILE_DATE_RE = re.compile(r'(20\d{6})')


def timestamp_of(line):
    match = TIME_RE.search(line)
    if match:
        return match.group(1)
    match = EPOCH_RE.search(line)
    if match:
        try:
            return time.strftime('%Y-%m-%d %H:%M:%S',
                                 time.localtime(float(match.group(1))))
        except (ValueError, OSError):
            return None
    return None


def iter_logs():
    seen = set()
    for root in SEARCH_ROOTS:
        if not root.exists():
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in dirnames if d not in ('raw', 'clients', 'build', 'install')]
            for name in filenames:
                if not name.endswith('.log'):
                    continue
                path = Path(dirpath) / name
                if path in seen:
                    continue
                if path.stat().st_size > 8 * 1024 * 1024:
                    continue
                seen.add(path)
                yield path


def main():
    hits = []
    counts = Counter()
    per_file = Counter()
    per_tag = Counter()
    hours = Counter()
    dates = Counter()
    for path in iter_logs():
        try:
            lines = path.read_text(encoding='utf-8', errors='replace').splitlines()
        except OSError:
            continue
        file_date = FILE_DATE_RE.search(path.name)
        tag = path.stem.split('_')[0]
        for index, line in enumerate(lines):
            for kind, patterns in KEYWORDS.items():
                if not any(re.search(pattern, line) for pattern in patterns):
                    continue
                if kind == 'disabled_or_stop' and 'DISABLED' not in line and '停止' not in line:
                    continue
                stamp = timestamp_of(line)
                context = lines[max(0, index - 2): min(len(lines), index + 3)]
                hit = {'kind': kind, 'file': str(path.relative_to(ROOT)) if str(path).startswith(str(ROOT)) else str(path),
                       'line': index + 1, 'timestamp': stamp, 'tag': tag,
                       'file_date': file_date.group(1) if file_date else None,
                       'text': line.strip()[:300], 'context': [item.strip()[:240] for item in context]}
                hits.append(hit)
                counts[kind] += 1
                per_file[str(hit['file'])] += 1
                per_tag[tag] += 1
                if stamp:
                    dates[stamp[:10]] += 1
                    hours[stamp[11:13]] += 1
                break
    report = {'schema': 'slaihand_serial_fault_analysis/1',
              'scanned_roots': [str(root) for root in SEARCH_ROOTS],
              'counts': dict(counts), 'per_file': dict(per_file.most_common(20)),
              'per_tag': dict(per_tag.most_common(20)),
              'dates': dict(sorted(dates.items())), 'hours': dict(sorted(hours.items())),
              'hits': hits}
    (HERE / 'serial_fault_analysis_20261110.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    lines = ['# 串口/审计写故障日志分析（离线）', '',
             '- 扫描根：%s' % ', '.join(str(r) for r in SEARCH_ROOTS),
             '- 关键词计数：%s' % json.dumps(dict(counts), ensure_ascii=False),
             '', '## 按日期', '']
    for date, count in sorted(dates.items()):
        lines.append('- %s: %d' % (date, count))
    lines += ['', '## 按小时', '']
    for hour, count in sorted(hours.items()):
        lines.append('- %s 时: %d' % (hour, count))
    lines += ['', '## 按文件前缀/tag', '']
    for name, count in per_tag.most_common(15):
        lines.append('- %s: %d' % (name, count))
    lines += ['', '## 涉及文件（前 15）', '']
    for name, count in per_file.most_common(15):
        lines.append('- %s: %d' % (name, count))
    lines += ['', '## 最近 10 条命中', '']
    for hit in hits[-10:]:
        lines.append('- [%s] %s:%s `%s`' % (hit['timestamp'], hit['file'], hit['line'],
                                            hit['text'][:160]))
    lines += ['', '## 初步结论', '',
              '- 命中集中在 CANUSB 写串口包不完整（`incomplete serial packet write`）'
              '及其上层 `SERIAL_OR_AUDIT_WRITE_FAILURE`；驱动随后 DISABLED，'
              '客户端停止。',
              '- 需要结合具体 tag 判断是否与运动/重启相关；处理原则仍是停止→确认失能→'
              '面板复位→最多重试一次，不连续重试。']
    (HERE / 'serial_fault_analysis_20261110.md').write_text('\n'.join(lines) + '\n',
                                                           encoding='utf-8')
    print('hits', len(hits), 'counts', dict(counts))
    print('dates', dict(sorted(dates.items())))
    print('wrote serial_fault_analysis_20261110.json/.md')


if __name__ == '__main__':
    main()
