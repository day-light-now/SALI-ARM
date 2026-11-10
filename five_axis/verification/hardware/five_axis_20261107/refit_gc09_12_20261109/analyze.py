"""Offline upstream-pair/session refit. GC13 remains uncollected and unused.

Unknown approach history is excluded from fitting, not silently assigned zero
friction. Session offsets are penalized diagnostics, not deployable FF terms.
"""
from __future__ import annotations
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys

import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
PREV = BASE/'five_dof_refit_gc05_08_20261109'
EXEC = PREV/'execution_20261109'
sys.path.insert(0, str(PREV))
import refit as previous
from fit_five_dof import FiveDofPotential, rigid_parameters, legacy


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(name, value):
    (HERE/name).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                     allow_nan=False)+'\n', encoding='utf-8')


def prepare():
    old = read(PREV/'static_windows.json')
    for name, expected in old['source_hashes'].items():
        if sha(BASE/name) != expected:
            raise ValueError('old source changed: '+name)
    manifest = read(PREV/'SOURCE_MANIFEST.json')
    executed = read(EXEC/'EXECUTION_MANIFEST_20261109.json')
    for kind in ('evidence', 'artifact'):
        if sha(previous.local(executed['admission'][kind])) != executed['admission'][kind+'_sha256']:
            raise ValueError('new admission changed')
    manifest['followup_admission'] = executed['admission']
    manifest['source_commit'] = '75807e2'
    for plan in executed['plans']:
        if plan['tag'] == 'gc13':
            raise ValueError('GC13 must remain unused')
        primary = plan['primary_client']
        for name, expected in ((plan['raw_file'], plan['raw_sha256']),
                               (plan['plan_file'], plan['plan_sha256']),
                               (primary['file'], primary['sha256'])):
            if sha(previous.local(name)) != expected:
                raise ValueError('new source changed: '+name)
        if primary['trajectory_error_code'] != 0:
            raise ValueError('primary client did not complete')
        manifest['sessions'].append({'name': plan['tag'].upper(),
            'state': previous.local(plan['raw_file']).relative_to(BASE).as_posix(),
            'state_sha256': plan['raw_sha256'],
            'plan': previous.local(plan['plan_file']).relative_to(BASE).as_posix(),
            'client': previous.local(primary['file']).relative_to(BASE).as_posix()})
    # The pre-fault GC11 attempt is retained as a separate diagnostic, never mixed
    # into the clean training run or counted as a complete trajectory.
    aborted = EXEC/'clients/gc11/20261109_153509_f74da167.json'
    bound = next(c for p in executed['plans'] if p['tag']=='gc11'
                 for c in p['client_files'] if c['file'].endswith(aborted.name))
    if sha(aborted) != bound['sha256']:
        raise ValueError('aborted client changed')
    raw = EXEC/'raw/gc11_aborted_20261109.jsonl.gz'
    manifest['sessions'].append({'name': 'GC11_ABORTED',
        'state': raw.relative_to(BASE).as_posix(), 'state_sha256': sha(raw),
        'client': aborted.relative_to(BASE).as_posix(),
        'plan': next(s['plan'] for s in manifest['sessions'] if s['name']=='GC11'),
        'use': 'pre-fault windows only, diagnostic, not training',
        'raw_hash_provenance': 'computed from archived aborted raw; original manifest has no bound aborted-raw SHA'})
    audits = read(PREV/'audit_sources.json')
    for entry in audits:
        if sha(PREV/entry['archive']) != entry['archive_sha256']:
            raise ValueError('old audit changed')
        shutil.copyfile(PREV/entry['archive'], HERE/entry['archive'])
    for original in executed['audit']:
        entry = dict(original)
        path = previous.local(entry['archive'])
        if sha(path) != entry['archive_sha256']:
            raise ValueError('new audit changed')
        entry['archive'] = path.name
        shutil.copyfile(path, HERE/path.name)
        audits.append(entry)
    manifest['parent_manifest_sha256'] = sha(PREV/'SOURCE_MANIFEST.json')
    manifest['execution_manifest_sha256'] = sha(EXEC/'EXECUTION_MANIFEST_20261109.json')
    save('SOURCE_MANIFEST.json', manifest)
    save('audit_sources.json', audits)
    data = legacy.extract(data_dir=BASE, output_dir=HERE, manifest_path=HERE/'SOURCE_MANIFEST.json')
    previous.attach_directions(data['poses'], manifest)
    sources = {s['name']: read(BASE/s['client'])['backend_arm_receipt'].split('audit_file=')[1]
               for s in manifest['sessions']}
    for p in data['poses']:
        p['driver_session'] = sources[p['session']]
    save('static_windows.json', data)
    cross = []
    independent = read(EXEC/'gc_windows_20261109.json')
    for plan in independent['plans']:
        for w in plan.get('windows', []):
            if 'excluded' in w:
                continue
            identifier = f"{plan['tag'].upper()}_{w['index']}"
            p = next(p for p in data['poses'] if p['id'] == identifier)
            cross.append({'id': identifier,
                'max_q_difference_deg': float(np.max(np.abs(np.degrees(p['q_rad'])-w['q_deg']))),
                'max_effort_difference_nm': float(np.max(np.abs(np.array(p['effort_mean_nm'])-w['effort_mean_nm'])))})
    save('extraction_crosscheck.json', cross)
    print('Extracted', len(data['poses']), 'windows; independent crosscheck', len(cross), flush=True)


def split(poses):
    train, history, old_gc08 = previous.splits(poses)
    aborted = np.array([p['session']=='GC11_ABORTED' for p in poses])
    return train & ~aborted, history, old_gc08, aborted


def observed(poses):
    direction = np.array([p['last_planned_approach_sign'] for p in poses])
    mask = direction != 0
    mask[:, 4] &= np.array([p['session'] in ('J5','GC05','GC06','GC07','GC08') for p in poses])
    return direction, mask


def session_design(poses, training):
    sessions = sorted({p['driver_session'] for p, k in zip(poses, training) if k})
    design = np.zeros((len(poses), 5, len(sessions)*5))
    for i, p in enumerate(poses):
        if p['driver_session'] in sessions:
            k = sessions.index(p['driver_session'])
            for j in range(5):
                design[i,j,k*5+j] = 1.
    return design, sessions


def fit_model(x, base, y, direction, rowmask, session, *, shape='rigid', sigma=None):
    """Variable projection: only the seven orbit parameters are nonlinear.

    Offset penalty sigma is an exploratory regularization scale, not a measured
    sensor standard deviation. Holdout y values never enter the solve.
    """
    d = previous.direction_design(direction, 5)
    nuisance = d if sigma is None else np.concatenate([d, session], axis=2)
    a = nuisance[rowmask]
    penalty = np.zeros((0, a.shape[1])) if sigma is None else np.c_[
        np.zeros((session.shape[2],5)), np.eye(session.shape[2])/sigma]
    augmented = np.vstack([a, penalty])
    pinv = np.linalg.pinv(augmented, rcond=1e-9)
    def profile(h):
        target = np.r_[(y-base-x@h)[rowmask], np.zeros(len(penalty))]
        z = pinv@target
        return augmented@z-target, z
    if shape in ('fixed', 'free'):
        k = 3 if shape=='fixed' else 9
        design = np.c_[np.vstack([x[:,:,:k][rowmask], np.zeros((len(penalty),k))]), augmented]
        target = np.r_[(y-base)[rowmask], np.zeros(len(penalty))]
        z, _, rank, sig = np.linalg.lstsq(design, target, rcond=1e-9)
        h = np.r_[z[:k], np.zeros(9-k)]
        residual, coefficients = profile(h)
        full_rank = rank == design.shape[1]
        extra = {'rank': int(rank), 'columns':design.shape[1], 'rank_deficient':not full_rank,
                 'condition_number': float(sig[0]/sig[-1]) if full_rank else None,
                 'identifiable_subspace_condition_number':float(sig[0]/sig[rank-1]) if rank else None,
                 'axis_unverified': None,
                 'optimizer_success': True}
    else:
        solutions = []
        for theta in np.linspace(-math.pi, math.pi, 6, endpoint=False):
            for phi in (-.8, 0., .8):
                initial = [.1,.01,-.05,.03,.01,theta,phi]
                solutions.append(least_squares(lambda z: profile(rigid_parameters(z)[0])[0], initial,
                    max_nfev=700, ftol=1e-9, xtol=1e-9, gtol=1e-9))
        best = min(solutions, key=lambda r:r.cost)
        h, axis = rigid_parameters(best.x)
        residual, coefficients = profile(h)
        extra = {'rank': int(np.linalg.matrix_rank(best.jac)),
                 'condition_number': float(np.linalg.cond(best.jac)),
                 'axis_unverified': axis.tolist(), 'optimizer_success': bool(best.success)}
    gravity = base+x@h
    fresh = gravity+d@coefficients[:5]
    conditioned = gravity+nuisance@coefficients
    moments = h.reshape(3,3)
    return {**extra, 'shape': shape, 'session_penalty_scale_nm': sigma,
        'h_kg_m': h.tolist(), 'direction_coefficients_nm': coefficients[:5].tolist(),
        'session_offsets_nm': [] if sigma is None else coefficients[5:].reshape(-1,5).tolist(),
        'COM_q5_zero_link4_mm': ((moments[0]+moments[1])/3.5*1000).tolist(),
        'orbit_radii_mm': (np.linalg.norm(moments[1:],axis=1)/3.5*1000).tolist(),
        'training_penalized_cost': float(residual@residual/2),
        'deployable': False}, gravity, fresh, conditioned


def pairs(poses, gravity, conditioned):
    output = []
    specs = [(s,4,[(1,3),(5,7),(4,8)] if s=='GC05' else [(2,4),(6,8),(5,9)])
             for s in ('GC05','GC06','GC07','GC08')]
    specs += [(s,j,[(1,3),(5,7)]) for s,j in
              (('GC09',0),('GC10',3),('GC11',1),('GC12',2),('GC11_ABORTED',1))]
    for session, axis, indices in specs:
        for a,b in indices:
            ia = next(i for i,p in enumerate(poses) if p['id']==f'{session}_{a}')
            ib = next(i for i,p in enumerate(poses) if p['id']==f'{session}_{b}')
            truth = np.array([poses[k]['effort_mean_nm'] for k in (ia,ib)])
            pm = truth.mean(axis=0)
            gp = (gravity[ia]+gravity[ib])/2
            output.append({'session': session, 'reversed_axis_one_based': axis+1,
                'ids': [poses[ia]['id'],poses[ib]['id']],
                'actual_q_deg': np.degrees([poses[k]['q_rad'] for k in (ia,ib)]).tolist(),
                'observed_mean_nm': pm.tolist(), 'observed_half_difference_nm': ((truth[0]-truth[1])/2).tolist(),
                'gravity_mean_nm': gp.tolist(), 'gravity_mean_residual_nm': (gp-pm).tolist(),
                'conditioned_mean_residual_nm': ((conditioned[ia]+conditioned[ib])/2-pm).tolist(),
                'gravity_corrected_direction_half_nm': float(((truth[0]-truth[1]-gravity[ia]+gravity[ib])/2)[axis])})
    return output


def metrics(pred, truth, selection, mask):
    out = []
    for j in range(5):
        k = selection & mask[:,j]
        err = pred[k,j]-truth[k,j]
        out.append({'axis':j+1, 'window_count':int(sum(k)),
                    'rmse_nm':float(np.sqrt(np.mean(err**2))) if len(err) else None,
                    'max_abs_nm':float(np.max(abs(err))) if len(err) else None})
    return out


def run():
    data = read(HERE/'static_windows.json'); poses=data['poses']
    if len(poses) != 159:
        raise ValueError('expected 151 clean + 8 pre-fault diagnostic windows')
    direction, mask = observed(poses)
    train, history, old_gc08, aborted = split(poses)
    session, names = session_design(poses, train)
    model=FiveDofPotential()
    x=np.stack([model.orbit_regressor(p['q_rad']) for p in poses])
    base=np.stack([model.base(p['q_rad']) for p in poses])
    y=np.array([p['effort_mean_nm'] for p in poses])
    variants=[]; predictions={}
    for shape, sigma in (('fixed',None),('free',None),('rigid',None),
                         ('rigid',.05),('rigid',.15),('rigid',.30)):
        label=f'{shape}_known_direction_session_{sigma}'
        r,g,f,c=fit_model(x,base,y,direction,mask & train[:,None],session,shape=shape,sigma=sigma)
        r['name']=label
        for labelset, selection in (('training',train),('historical_validation',history),
                                    ('GC08_previously_seen_diagnostic',old_gc08),('GC11_aborted_diagnostic',aborted)):
            r[labelset]={'gravity_vs_effort':metrics(g,y,selection,mask),
                        'fresh_session_offset_zero':metrics(f,y,selection,mask),
                        'conditioned_on_known_training_session':metrics(c,y,selection,mask)}
        r['pairs']=pairs(poses,g,c)
        predictions[label]={'gravity_nm':g.tolist(),'fresh_session_total_nm':f.tolist(),
                            'conditioned_total_nm':c.tolist()}
        variants.append(r)
        print(label, 'cost',round(r['training_penalized_cost'],3),
              'COM',np.round(r['COM_q5_zero_link4_mm'],2),
              'friction',np.round(r['direction_coefficients_nm'],3),flush=True)
    # Fit only the actually reversed axis of each pair. Unchanged-axis pair
    # means do NOT cancel that axis's friction and must not enter this loss.
    pair_spec = variants[0]['pairs']
    pair_x=[]; pair_base=[]; pair_y=[]; pair_session=[]
    pair_rows=np.zeros((len(pair_spec),5),dtype=bool)
    pair_train=[]
    for k,p in enumerate(pair_spec):
        indices=[next(i for i,q in enumerate(poses) if q['id']==name) for name in p['ids']]
        pair_x.append(x[indices].mean(axis=0));pair_base.append(base[indices].mean(axis=0))
        pair_y.append(y[indices].mean(axis=0));pair_session.append(session[indices].mean(axis=0))
        pair_train.append(p['session'] not in ('GC08','GC11_ABORTED'))
        pair_rows[k,p['reversed_axis_one_based']-1]=True
    pair_x=np.array(pair_x);pair_base=np.array(pair_base);pair_y=np.array(pair_y)
    pair_session=np.array(pair_session);pair_train=np.array(pair_train)
    paired_models=[]
    for shape,sigma in (('fixed',None),('rigid',None),('rigid',.15)):
        r,_,_,_=fit_model(pair_x,pair_base,pair_y,np.zeros((len(pair_spec),5)),
            pair_rows & pair_train[:,None],pair_session,shape=shape,sigma=sigma)
        r['name']=f'pair_means_{shape}_session_{sigma}'
        g=base+x@np.array(r['h_kg_m'])
        offsets=np.zeros_like(base) if sigma is None else session@np.array(r['session_offsets_nm']).ravel()
        r['pairs']=pairs(poses,g,g+offsets)
        r['training_pair_count']=int(sum(pair_train))
        r['scope']='only reversed-axis paired means; no constant-friction law needed, symmetric cancellation still an assumption'
        paired_models.append(r)
        print(r['name'],'cost',round(r['training_penalized_cost'],4),
              'COM',np.round(r['COM_q5_zero_link4_mm'],2),'cond',r['condition_number'],flush=True)
    report={'schema':'slaihand_gc09_12_session_refit/1','source_commit':'75807e2',
        'deployable':False,'selected_for_deployment':None,'parameters_applied':False,
        'no_hardware_actions':True,'GC13_used':False,'new_independent_validation_available':False,
        'primary_diagnostic_model':'rigid_known_direction_session_0.15',
        'session_grouping':'actual audit file/driver session, not trajectory tag',
        'training_driver_sessions':names,
        'offset_regularization':'0.05/0.15/0.30 Nm sensitivity scales, not a measured standard deviation or certified bound',
        'unknown_approach_rule':'exclude axis/window from fitting and direction-conditioned metrics; do not assume zero static friction',
        'session_counts':{s:sum(p['session']==s for p in poses) for s in dict.fromkeys(p['session'] for p in poses)},
        'train_ids':[p['id'] for p,k in zip(poses,train) if k],
        'historical_validation_ids':[p['id'] for p,k in zip(poses,history) if k],
        'GC08_diagnostic_ids':[p['id'] for p,k in zip(poses,old_gc08) if k],
        'GC11_aborted_diagnostic_ids':[p['id'] for p,k in zip(poses,aborted) if k],
        'training_rows_per_axis':np.sum(mask & train[:,None],axis=0).tolist(),
        'convention':read(PREV/'refit_result.json')['convention'],
        'excluded':data['excluded'],'models':variants,'paired_only_models':paired_models,'predictions':predictions,
        'source_hashes':data['source_hashes'],'prior_geometry_provenance':legacy.prior_parameters()[2]}
    save('session_refit_result.json',report)
    save('offline_candidates.json',{'deployable':False,'selected_for_deployment':None,
        'result_sha256':sha(HERE/'session_refit_result.json'),
        'reason':'Session offsets and static friction remain confounded with physical gravity; J5 axis/pivot unmeasured; GC12 process exit unresolved.'})


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--extract',action='store_true')
    args=parser.parse_args()
    if args.extract:prepare()
    run()
