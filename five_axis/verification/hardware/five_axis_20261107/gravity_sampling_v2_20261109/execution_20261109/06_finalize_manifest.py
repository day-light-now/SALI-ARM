"""Finalize the GC05-GC08 acquisition archive (offline, no hardware).

After the four protected-client runs:
  * verify each client result (waypoints passed, trajectory error 0, final zero);
  * count recorder records per plan and gzip already done by the runner;
  * extract SESSION + CONTROL_CYCLE records for the four command ids from the
    driver-session audit file into local compressed archives;
  * write EXECUTION_MANIFEST_20261109.json with SHA-256 of every artifact.

Nothing here commands the robot or reads a device.
"""
from __future__ import annotations

import gzip
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path('/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain')
HERE = Path(__file__).resolve().parent
PLANS = ROOT / 'verification/hardware/five_axis_20261107/gravity_sampling_v2_20261109'

TAGS = ['gc05', 'gc06', 'gc07', 'gc08']
PLAN_FILES = {
    'gc05': 'GC05_J5_ZERO_PAIRED_ZERO.json',
    'gc06': 'GC06_J5_J2_20_PAIRED_ZERO.json',
    'gc07': 'GC07_J5_J3_J4_20_PAIRED_ZERO.json',
    'gc08': 'GC08_J5_MIXED_HOLDOUT_ZERO.json',
}


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


def main():
    manifest = {
        'schema': 'slaihand_gravity_sampling_execution/1',
        'source_head': '0ddfb08',
        'date': '2026-11-09',
        'scope': 'real five-axis acquisition; no FF, no gravity compensation, alpha 0',
        'hardware': {
            'device': '/dev/ttyACM0',
            'port_argument': '/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00',
            'ros_domain_id': 183,
            'driver': 'slaihand_canusb_five_axis_candidate.launch.py',
            'guard_4deg_unchanged': True,
            'j5_watchdog_timeout_2000': True,
            'gravity_ff_alpha': 0.0,
        },
        'admission': {
            'evidence': str((HERE / 'evidence/j5_timeout_verify_existing_20261109_session2.json')),
            'evidence_sha256': sha(HERE / 'evidence/j5_timeout_verify_existing_20261109_session2.json'),
            'artifact': str(HERE / 'artifacts/j5_motion_calibration_artifact_20261109_session2.json'),
            'artifact_sha256': sha(HERE / 'artifacts/j5_motion_calibration_artifact_20261109_session2.json'),
            'artifact_note': (
                'j5_motion_artifact.py build normalizes the timeout block to '
                'timeout_before=0/write_outcome=complete even for readback-only '
                'verified_existing evidence; the real evidence file is bound by '
                'SHA-256 and records timeout_before=timeout_after=2000, '
                'write_attempts=0, parameter_writes=0. The launch validates the '
                'evidence file hash, timeout_after, session_id, boot_id, device. '
                'This is a known truthful-metadata defect to fix later.'),
        },
        'plans': [],
    }
    audit_sources = {}
    for tag in TAGS:
        plan_path = PLANS / PLAN_FILES[tag]
        client_dir = HERE / 'clients' / tag
        client_files = sorted(client_dir.glob('*.json'))
        raw = HERE / 'raw' / f'{tag}_20261109.jsonl.gz'
        entry = {
            'tag': tag,
            'plan_file': str(plan_path),
            'plan_sha256': sha(plan_path),
            'raw_file': str(raw),
            'raw_sha256': sha(raw) if raw.exists() else None,
            'raw_records': count_records(raw) if raw.exists() else None,
            'client_files': [],
            'log_file': str(HERE / 'logs' / f'{tag}.log'),
        }
        for path in client_files:
            result = read(path)
            waypoints = result.get('waypoint_results', [])
            entry['client_files'].append({
                'file': str(path),
                'sha256': sha(path),
                'mode': result.get('mode'),
                'reason': result.get('reason'),
                'trajectory_error_code': result.get('trajectory_error_code'),
                'standard_jtc_action_succeeded': result.get('standard_jtc_action_succeeded'),
                'waypoint_count': len(waypoints),
                'waypoints_all_passed': all(bool(w.get('passed')) for w in waypoints),
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
                source = receipt.split('audit_file=')[1].strip()
                audit_sources.setdefault(source, set()).add(int(result.get('command_id')))
        executed = [item for item in entry['client_files'] if item.get('mode') == 'waypoints']
        entry['primary_client'] = executed[-1] if executed else (
            entry['client_files'][-1] if entry['client_files'] else None)
        if entry['primary_client'] is not None:
            entry['primary_client_status'] = (
                'TRAJECTORY_COMPLETE_CLIENT_CRITERION_NOT_MET'
                if entry['primary_client'].get('reason') == 'WAYPOINT_FEEDBACK_CRITERION_NOT_MET'
                else entry['primary_client'].get('reason'))
        manifest['plans'].append(entry)

    # Extract audit slices for the driver session(s) used by these commands.
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
            'source': source,
            'source_sha256': source_hash.hexdigest(),
            'commands': sorted(commands),
            'archive': str(archive),
            'archive_sha256': sha(archive),
            'records': kept,
        })

    out = HERE / 'EXECUTION_MANIFEST_20261109.json'
    out.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print('written', out)
    for entry in manifest['plans']:
        client = entry.get('primary_client') or {}
        print(entry['tag'], 'raw_records', entry['raw_records'],
              'status', entry.get('primary_client_status'),
              'wp', f"{client.get('waypoints_passed')}/{client.get('waypoint_count')}",
              'final', client.get('final_delta_from_start_deg'))
    for audit in manifest['audit']:
        print('audit', audit.get('source'), audit.get('records'), audit.get('error', ''))


if __name__ == '__main__':
    sys.exit(main())
