"""Import GC09-GC13 plans into the running panel; verify readback. No motion."""
from __future__ import annotations

import hashlib
import json
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
PLANS = HERE.parent / 'next_sampling_candidates'
TOKEN = '<redacted-panel-token>'
BASE = 'http://127.0.0.1:42664'
NAMES = {
    'GC09_J1_PAIRED_REPEAT_ZERO': 'GC09 后续采集 J1双向重复 · 绝对零终点',
    'GC10_J4_PAIRED_REPEAT_ZERO': 'GC10 后续采集 J4双向重复 · 绝对零终点',
    'GC11_J2_PAIRED_REPEAT_ZERO': 'GC11 后续采集 J2双向重复 · 绝对零终点',
    'GC12_J3_PAIRED_REPEAT_ZERO': 'GC12 后续采集 J3双向重复 · 绝对零终点',
    'GC13_NEW_MIXED_J5_HOLDOUT_ZERO': 'GC13 后续采集 混合姿态J5新留出 · 绝对零终点',
}


def request(method, path, body=None):
    data = None if body is None else json.dumps(body, ensure_ascii=False).encode('utf-8')
    req = urllib.request.Request(BASE + path, data=data, method=method)
    req.add_header('X-Panel-Token', TOKEN)
    req.add_header('Origin', BASE)
    if data is not None:
        req.add_header('Content-Type', 'application/json; charset=utf-8')
    with urllib.request.urlopen(req, timeout=10) as response:
        return json.loads(response.read().decode('utf-8'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    manifest = json.loads((PLANS / 'PLAN_MANIFEST.json').read_text(encoding='utf-8'))
    by_name = {plan['name']: plan for plan in manifest['plans']}
    results = []
    for path in sorted(PLANS.glob('GC*.json'), key=lambda p: p.name):
        key = path.stem
        if key not in NAMES or key not in by_name:
            continue
        document = json.loads(path.read_text(encoding='utf-8'))
        file_sha = sha(path)
        try:
            saved = request('POST', '/api/plans/save',
                            {'name': NAMES[key], 'document': document})
            plan_id = saved['plan']['id']
            time.sleep(0.15)
            read = request('GET', '/api/plans/' + plan_id)
            readback = read.get('document') or {}
            waypoints_same = readback.get('waypoints') == document['waypoints']
            coordinates_same = readback.get('coordinates') == document['coordinates']
            schema_same = readback.get('schema') == document['schema']
            results.append({
                'name': key,
                'display_name': NAMES[key],
                'file': path.name,
                'file_sha256': file_sha,
                'manifest_sha_match': file_sha == by_name[key]['sha256'],
                'panel_id': plan_id,
                'waypoints': len(readback.get('waypoints') or []),
                'readback_document_matches': waypoints_same and coordinates_same and schema_same,
                'final_absolute_deg': (readback.get('waypoints') or [{}])[-1].get('degrees'),
            })
        except urllib.error.HTTPError as error:
            results.append({'name': key, 'file': path.name, 'file_sha256': file_sha,
                            'error': 'HTTP %s: %s' % (error.code, error.read().decode('utf-8', 'replace'))})
        except Exception as error:  # noqa: BLE001 - report import failure verbatim
            results.append({'name': key, 'file': path.name, 'error': repr(error)})
    receipt = {
        'schema': 'slaihand_panel_sampling_import/3',
        'imported_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'panel_origin': BASE,
        'plans': results,
        'hardware_executed': False,
        'ff_enabled': False,
    }
    out = HERE / 'PANEL_IMPORT_RECEIPT.json'
    out.write_text(json.dumps(receipt, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    for item in results:
        print(item['name'], 'sha_match', item.get('manifest_sha_match'),
              'panel_id', item.get('panel_id'), 'wp', item.get('waypoints'),
              'readback', item.get('readback_document_matches'), item.get('error', ''))
    print('written', out)


if __name__ == '__main__':
    main()
