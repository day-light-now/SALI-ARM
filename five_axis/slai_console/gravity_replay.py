"""Replay native J2 logs into bounded gravity candidates; never connects CAN.

python -m slai_console.gravity_replay --result <native-result.json> --output <dir>
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import struct
import re

from .commissioning import Profile
from .core import ROOT
from .gravity_feedforward import GravityBounds, GravityCandidate, candidate_mit_packet, provisional_model
from .openarm_mit import decode_mit_control_data, estimate_drive_request


def read_mapping(path):
    # Only the three numeric fields from this repository's known schema.
    text=Path(path).read_text(encoding='utf-8')
    mapping={}
    for i in range(1,5):
        match=re.search(rf'^  slaihand_joint{i}:\s*\n(.*?)(?=^  slaihand_joint|\Z)',text,re.M|re.S)
        if not match:raise ValueError(f'Missing J{i} mapping')
        values={}
        for key in ('can_id','hardware_sign','zero_offset_rad'):
            field=re.search(rf'^    {key}:\s*([^#\n]+)',match[1],re.M)
            if not field:raise ValueError(f'Missing J{i} {key}')
            values[key]=float(field[1].strip())
        if values['can_id']!=i or values['hardware_sign'] not in (-1,1) or not math.isfinite(values['zero_offset_rad']):
            raise ValueError('Invalid gravity software mapping')
        mapping[i]=values
    return mapping


def replay(result_path, output):
    result_path=Path(result_path);output=Path(output)
    result=json.loads(result_path.read_text(encoding='utf-8'))
    if result['joint']!=2 or result['backend']!='drive_mit':
        raise ValueError('Only native J2 replay supported')
    mapping_path=ROOT/'src/slaihand_bringup/config/hardware_mapping.yaml'
    mapping=read_mapping(mapping_path)
    source=Path(result['readonly_log'])
    audit=result_path.with_suffix('.jsonl')
    events=[]
    for line in source.read_text(encoding='utf-8').splitlines():
        row=json.loads(line)
        if row.get('direction')=='rx':events.append((row['time'],0,row))
    for line in audit.read_text(encoding='utf-8').splitlines():
        row=json.loads(line)
        if row['kind']=='SERIAL_WRITTEN' and row['joint']==2 and row['frame_kind']=='mit':
            events.append((row['unix_s'],1,row))
    events.sort(key=lambda e:(e[0],e[1]))
    model,evidence=provisional_model();bounds=GravityBounds()
    candidate=GravityCandidate(model,bounds);regs={i:{} for i in range(1,5)}
    feedback={};rows=[];missing=0
    for now,kind,row in events:
        b=bytes.fromhex(row['payload'])
        if not kind:
            if len(b)!=8:continue
            joint=b[0]&15
            if joint not in regs:continue
            if b[0]==joint and b[1]==0 and b[2]==0x33 and b[3] in (7,8,9,10,21,22,23):
                if b[3] in (21,22,23):
                    value=struct.unpack('<f',b[4:])[0]
                    if b[3] in regs[joint] and regs[joint][b[3]]!=value:
                        raise ValueError('Conflicting live protocol ranges in source log')
                    regs[joint][b[3]]=value
            elif len(regs[joint])==3:
                feedback[joint]=(b,now)
            continue
        if len(feedback)!=4 or any(len(r)!=3 for r in regs.values()):
            missing+=1;continue
        q=[];received=[];profiles={};velocities={}
        for i in range(1,5):
            m=mapping[i];r=regs[i];payload,at=feedback[i]
            p=Profile(i,1,0,2000,r[21],r[22],r[23],m['hardware_sign'],m['zero_offset_rad'])
            position=((payload[1]<<8)|payload[2])*2*p.pmax/65535-p.pmax
            velocity=((payload[3]<<4)|(payload[4]>>4))*2*p.vmax/4095-p.vmax
            q.append(p.sign*(position-p.zero));received.append(at)
            profiles[i]=p;velocities[i]=velocity
        c=candidate.step(q,now,received)
        p=profiles[2];original=bytes.fromhex(row['payload'])
        packet=candidate_mit_packet(p,original,c['model_ff_nm'],bounds.cap_nm)
        estimate=estimate_drive_request(p,packet,
            position=p.zero+p.sign*q[1],velocity=velocities[2])
        baseline=estimate_drive_request(p,original,
            position=p.zero+p.sign*q[1],velocity=velocities[2])
        rows.append(dict(unix_s=now,q_rad=q,**c,
            baseline_estimate_motor_nm=baseline,candidate_estimate_motor_nm=estimate,
            encoded_ff_model_nm=p.sign*decode_mit_control_data(p,packet).tau,
            candidate_payload=packet.hex(' '),reference_payload=original.hex(' '),
            would_exceed_estimate_guard=abs(estimate)>result['observed_state_estimate_guard_nm']))
    if not rows:raise ValueError('No complete fresh J2 samples in replay')
    summary=dict(status='PROVISIONAL_OFFLINE_ONLY',hardware_authorized=False,
        actual_serial_writes=0,source_result=str(result_path),source_audit=str(audit),
        source_readonly=str(source),bounds=asdict(bounds),model=evidence,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in (result_path,audit,source,mapping_path)},
        sample_count=len(rows),incomplete_samples_skipped=missing,
        raw_gravity_j2_range_nm=[min(r['raw_gravity_nm'] for r in rows),max(r['raw_gravity_nm'] for r in rows)],
        peak_candidate_ff_nm=max(abs(r['model_ff_nm']) for r in rows),
        encoded_ff_quantum_nm=2*profiles[2].tmax/4095,
        peak_candidate_estimate_nm=max(abs(r['candidate_estimate_motor_nm']) for r in rows),
        would_exceed_guard_count=sum(r['would_exceed_estimate_guard'] for r in rows),
        note='Counterfactual fields at recorded states; not closed-loop simulation, not a prediction of tracking or load capacity')
    output.mkdir(parents=True,exist_ok=True)
    with (output/'candidate_frames.jsonl').open('x',encoding='utf-8') as handle:
        for row in rows:handle.write(json.dumps(row,ensure_ascii=False)+'\n')
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(replay(args.result,args.output),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
