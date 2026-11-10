"""Read archived evidence only. This is not a process-crash reproducer."""
from collections import Counter
import gzip
import json
from analyze import HERE, EXEC, BASE, read, save, sha


def inspect():
    audit=EXEC/'audit/canusb_motion_waNPuU_crash_20261109.jsonl.gz'
    records=[]; invalid=0
    with gzip.open(audit,'rt',encoding='utf-8') as stream:
        for line in stream:
            try:records.append(json.loads(line))
            except ValueError:invalid+=1
    stages=[r for r in records if r['kind']=='STAGE']
    cycles=[r for r in records if r['kind']=='CONTROL_CYCLE']
    last=cycles[-1]
    written=[r for r in records if r['kind']=='TX_WRITTEN' and r['cycle_id']==last['cycle_id']]
    before_path=EXEC/'evidence/j5_timeout_safetycheck_20261109.json'
    after_path=EXEC/'evidence/j5_timeout_safetycheck_after_cut_20261109.json'
    before=read(before_path);after=read(after_path)
    client_path=EXEC/'clients/gc12/20261109_153833_9ed1f207.json'
    client=read(client_path)
    manual=BASE.parent/'commissioning_20261101/manual_extracts/J10010L_manual.txt'
    # BASE is .../verification/hardware/five_axis_20261107; manual is its sibling.
    sources=[audit,before_path,after_path,client_path,manual]
    result={'schema':'slaihand_gc12_archived_evidence_review/1','no_hardware_access':True,
        'process_crash_reproduced':False,'root_cause_determined':False,
        'source_sha256':{p.relative_to(BASE.parents[2]).as_posix():sha(p) for p in sources},
        'audit_record_counts':dict(Counter(r['kind'] for r in records)),
        'invalid_json_lines':invalid,
        'last_stage':{k:stages[-1][k] for k in ('cycle_id','steady_s','control_state','reason')},
        'last_control_cycle':{k:last[k] for k in ('cycle_id','steady_s','command_id','control_state')},
        'last_cycle_all_written_can_ids':sorted(r['can_id'] for r in written),
        'last_cycle_written_can_ids':sorted(r['can_id'] for r in written if r.get('role')==1 and 1<=r['can_id']<=5),
        'last_record_kind':records[-1]['kind'],
        'serial_write_is_not_drive_ack':all(r.get('drive_execution_ack') is False for r in written),
        'client_last_published_motor_states':{k:v['motor_state'] for k,v in client['last_observed_status'].items()},
        'client_confirmed_fresh_disable':client['stop_observations_confirmed'],
        'later_readback_observed_at_utc':before['observed_at'],
        'later_readback_states':{r['axis']:r['motor_state'] for r in before['axes']},
        'later_readback_payload_decoded_states':{r['axis']:int(r['feedback_payload'].split()[0],16)>>4 for r in before['axes']},
        'later_readback_feedback_valid':all(r['valid'] for r in before['axes']),
        'later_readback_state_meaning':'13 / 0xD = communication lost per local DAMIAO ERR tables; not state 1 enabled or state 0 disabled',
        'physical_torque_off_proven_by_state_code':False,
        'after_cut_observed_at_utc':after['observed_at'],
        'after_cut_no_feedback':all(not r['valid'] for r in after['axes']),
        'conclusion':'Earlier client state 1 and later fresh state 13 must be distinguished. Process disappearance is unresolved. No observation here certifies physical zero torque.',
        'reproduction_limit':'No exit code/signal/stack/core or deterministic exit reproducer is archived; this script verifies evidence interpretation only.'}
    save('gc12_incident_review.json',result)
    return result


if __name__=='__main__':
    r=inspect()
    print('Fresh states:',r['later_readback_states'])
    print('Last cycle:',r['last_control_cycle']['cycle_id'],'written axes:',r['last_cycle_written_can_ids'])
    print('Crash root cause determined:',r['root_cause_determined'])
