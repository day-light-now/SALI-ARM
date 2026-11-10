"""Archive local evidence for five-DOF analysis. No device/ROS access."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent
BASE=HERE.parent
OLD=BASE/'identification_codex_20261109'
ACQ=BASE/'acquisition_20261109'
EXEC=BASE/'gravity_compensation_trajectories_20261109/execution_20261109'


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    initial=read(ACQ/'ACQUISITION_MANIFEST_20261109.json')
    sessions=[]
    for old in initial['sessions']:
        s=dict(old)
        for key in ('state','plan','client'):s[key]='acquisition_20261109/'+s[key]
        sessions.append(s)
    extra=[
        ('GC01',EXEC/'raw/gc01_success_20261109.jsonl.gz',EXEC/'clients/gc01f/20261109_133627_2d97c20e.json'),
        ('GC02',EXEC/'raw/gc02_20261109.jsonl.gz',EXEC/'clients/gc02/20261109_133915_2361e5fa.json'),
        ('GC03',EXEC/'raw/gc03_20261109.jsonl.gz',EXEC/'clients/gc03/20261109_134157_adcb6df9.json'),
        ('GC04',EXEC/'raw/gc04_20261109.jsonl.gz',EXEC/'clients/gc04/20261109_134444_3322dcce.json'),
        ('J5',ACQ/'raw/j5_excitation_20261109.jsonl.gz',ACQ/'clients/j5_excitation/20261109_141027_321d3798.json')]
    for name,state,client in extra:
        plan=HERE/(name+'_executed_plan.json')
        plan.write_text(json.dumps(read(client)['plan'],indent=2)+'\n',encoding='utf-8')
        sessions.append({'name':name,'state':state.relative_to(BASE).as_posix(),
            'state_sha256':sha(state),'client':client.relative_to(BASE).as_posix(),
            'plan':plan.relative_to(BASE).as_posix(),'status':'completed run; check each active static window independently'})
    manifest={'schema':'slaihand_five_dof_analysis_sources/1','source_commit':'5cebc3f',
        'scope':'local offline analysis only','admission':{
            'artifact':'acquisition_20261109/artifacts/artifact_l.json',
            'artifact_sha256':sha(ACQ/'artifacts/artifact_l.json'),
            'evidence':'acquisition_20261109/artifacts/evidence_l_verify_existing.json',
            'evidence_sha256':sha(ACQ/'artifacts/evidence_l_verify_existing.json')},
        'sessions':sessions,'mass_box_kg':3.5,'motor_mass_added_again':False,
        'box_placement':'downstream of rotating J5 output; motor housing remains on link4',
        'j5_spatial_axis_measured':False}
    (HERE/'SOURCE_MANIFEST.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    oldindex=read(OLD/'audit_sources.json')
    for entry in oldindex:
        shutil.copyfile(OLD/entry['archive'],HERE/entry['archive'])
    sources={}
    for s in sessions:
        c=read(BASE/s['client']);source=c['backend_arm_receipt'].split('audit_file=')[1]
        sources.setdefault(source,set()).add(c['command_id'])
    index=list(oldindex)
    for source,commands in sources.items():
        if any(e['source']==source and set(e['commands'])>=commands for e in oldindex):continue
        path=Path('//wsl.localhost/Ubuntu-24.04'+source) if os.name=='nt' else Path(source)
        output=HERE/(path.stem+'_cycles.jsonl.gz');h=hashlib.sha256();count=0
        with path.open('rb') as inp,gzip.open(output,'wt',encoding='utf-8') as out:
            for line in inp:
                h.update(line);r=json.loads(line)
                if r['kind']=='SESSION' or (r['kind']=='CONTROL_CYCLE' and r['command_id'] in commands):
                    out.write(json.dumps(r,separators=(',',':'))+'\n');count+=1
        index.append({'source':source,'source_sha256':h.hexdigest(),'commands':sorted(commands),
            'archive':output.name,'archive_sha256':sha(output),'records':count})
    (HERE/'audit_sources.json').write_text(json.dumps(index,indent=2)+'\n',encoding='utf-8')
    evidence=read(ACQ/'artifacts/evidence_l_verify_existing.json')
    for k,v in {'write_outcome':'verified_existing','timeout_before':2000,'timeout_after':2000,'write_attempts':0,'parameter_writes':0}.items():
        if evidence[k]!=v:raise ValueError('read-only watchdog evidence differs')
    print('Prepared',len(sessions),'sessions and',len(index),'audit archives; read-only watchdog evidence verified')


if __name__=='__main__':main()
