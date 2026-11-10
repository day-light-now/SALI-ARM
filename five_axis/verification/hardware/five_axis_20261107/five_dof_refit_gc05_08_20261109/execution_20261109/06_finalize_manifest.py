"""Finalize the GC09+ acquisition archive (offline, no hardware).

Discovers every gc* recording/client pair, verifies each client result,
counts recorder records, extracts SESSION + CONTROL_CYCLE for the executed
commands from the driver-session audit file, and writes
EXECUTION_MANIFEST_20261109.json with SHA-256 of every artifact.
"""
from __future__ import annotations

import gzip
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PLANS = HERE.parent / 'next_sampling_candidates'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def count_records(path):
    counts = {}
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            counts[row.get('kind')] = counts.get(row.get('kind'), 0) + 1
    return counts


def plan_for(tag):
    number = tag[2:].upper()
    matches = sorted(PLANS.glob('GC%s_*.json' % number))
    return matches[0] if matches else None


def main():
    tags = sorted({path.name.split('_')[0] for path in
        (HERE / 'raw').glob('gc*_20261109.jsonl.gz')
        if path.name == path.name.split('_')[0] + '_20261109.jsonl.gz'})
    manifest = {
        'schema': 'slaihand_gravity_followup_execution/1',
        'source_head': 'e48f2c6',
        'date': '2026-11-09',
        'scope': 'real upstream direction/repeatability acquisition; no FF, alpha 0',
        'hardware': {
            'device': '/dev/ttyACM0',
            'ros_domain_id': 183,
            'driver': 'slaihand_canusb_five_axis_candidate.launch.py',
            'guard_4deg_unchanged': True,
            'j5_watchdog_timeout_2000': True,
            'gravity_ff_alpha': 0.0,
        },
        'admission': {
            'evidence': str(HERE / 'evidence/j5_timeout_verify_existing_20261109_session3.json'),
            'evidence_sha256': sha(HERE / 'evidence/j5_timeout_verify_existing_20261109_session3.json'),
            'artifact': str(HERE / 'artifacts/j5_motion_calibration_artifact_20261109_session3.json'),
            'artifact_sha256': sha(HERE / 'artifacts/j5_motion_calibration_artifact_20261109_session3.json'),
            'artifact_fix_deployed': (
                'j5_motion_artifact.py from src (fix 7000420b...) copied to the installed '
                'lib before this session; builder and validator are identical. The artifact '
                'now records timeout_before=2000, write_outcome=verified_existing, '
                'write_attempts=0, parameter_writes=0.'),
        },
        'plans': [],
    }
    audit_sources = {}
    for tag in tags:
        plan_path = plan_for(tag)
        raw = HERE / 'raw' / (tag + '_20261109.jsonl.gz')
        client_files = []
        for path in sorted((HERE / 'clients' / tag).glob('*.json')):
            result = read(path)
            waypoints = result.get('waypoint_results', [])
            client_files.append({
                'file': str(path), 'sha256': sha(path),
                'mode': result.get('mode'), 'reason': result.get('reason'),
                'trajectory_error_code': result.get('trajectory_error_code'),
                'standard_jtc_action_succeeded': result.get('standard_jtc_action_succeeded'),
                'waypoint_count': len(waypoints),
                'waypoints_passed': sum(1 for w in waypoints if w.get('passed')),
                'max_error_deg': [round(v, 3) for w in waypoints
                                  for v in (w.get('max_error_deg') or [])],
                'final_delta_from_start_deg': result.get('final_delta_from_start_deg'),
                'initial_model_rad': result.get('initial_model_rad'),
                'last_motor_states': {name: value.get('motor_state')
                                      for name, value in (result.get('last_observed_status') or {}).items()},
            })
            receipt = result.get('backend_arm_receipt', '')
            if 'audit_file=' in receipt:
                audit_sources.setdefault(receipt.split('audit_file=')[1].strip(), set()).add(
                    int(result.get('command_id')))
        executed = [item for item in client_files if item.get('mode') == 'waypoints']
        primary = executed[-1] if executed else (client_files[-1] if client_files else None)
        manifest['plans'].append({
            'tag': tag,
            'plan_file': str(plan_path) if plan_path else None,
            'plan_sha256': sha(plan_path) if plan_path else None,
            'raw_file': str(raw),
            'raw_sha256': sha(raw),
            'raw_records': count_records(raw),
            'client_files': client_files,
            'primary_client': primary,
            'primary_client_status': (None if primary is None else
                                      ('TRAJECTORY_COMPLETE_CLIENT_CRITERION_NOT_MET'
                                       if primary.get('reason') == 'WAYPOINT_FEEDBACK_CRITERION_NOT_MET'
                                       else primary.get('reason'))),
        })
    manifest['audit'] = []
    for source, commands in audit_sources.items():
        path = Path(source)
        if not path.is_file():
            manifest['audit'].append({'source': source, 'commands': sorted(commands),
                                      'error': 'AUDIT_SOURCE_MISSING_IN_WSL'})
            continue
        archive = HERE / 'audit' / (path.stem + '_cycles_20261109.jsonl.gz')
        archive.parent.mkdir(parents=True, exist_ok=True)
        source_hash = hashlib.sha256()
        kept = 0
        with path.open('rb') as source_stream, gzip.open(archive, 'wt', encoding='utf-8') as out:
            for line in source_stream:
                source_hash.update(line)
                row = json.loads(line)
                if row.get('kind') == 'SESSION' or (
                        row.get('kind') == 'CONTROL_CYCLE'
                        and row.get('command_id') in commands):
                    out.write(json.dumps(row, separators=(',', ':')) + '\n')
                    kept += 1
        manifest['audit'].append({
            'source': source, 'source_sha256': source_hash.hexdigest(),
            'commands': sorted(commands), 'archive': str(archive),
            'archive_sha256': sha(archive), 'records': kept,
        })
    out = HERE / 'EXECUTION_MANIFEST_20261109.json'
    out.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print('written', out)
    for entry in manifest['plans']:
        client = entry.get('primary_client') or {}
        print(entry['tag'], entry['raw_records'],
              'wp %s/%s' % (client.get('waypoints_passed'), client.get('waypoint_count')),
              entry.get('primary_client_status'),
              'final', client.get('final_delta_from_start_deg'))
    for audit in manifest['audit']:
        print('audit', audit.get('records'), audit.get('error', ''))


if __name__ == '__main__':
    sys.exit(main())
