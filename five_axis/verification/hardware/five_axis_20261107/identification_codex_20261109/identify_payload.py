"""Reproducible, offline-only payload identification from immutable recordings.

URDF positions are model radians. Gravity holding torque is dU/dq. The box is
fixed to link4; q5 never enters the gravity model. No hardware/ROS imports.
"""
from __future__ import annotations
import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
DATA = HERE.parent / 'acquisition_20261109'
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]
URDF = ROOT / 'src/slaihand_description/urdf/slaihand_four_axis.urdf'
G = np.array([0., 9.80665, 0.])


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def records(path):
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        return [json.loads(line) for line in stream]


def ordered(payload, field):
    values = dict(zip(payload['joint_names'], payload[field]))
    a = np.array([values[n] for n in NAMES], dtype=float)
    if not np.isfinite(a).all():
        raise ValueError('nonfinite controller data')
    return a


class Geometry:
    def __init__(self, j3_offset=math.pi):
        self.joints = []
        for joint in ET.parse(URDF).getroot().findall('joint'):
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz'), sep=' ')
            rpy = np.fromstring(origin.get('rpy'), sep=' ')
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            axis /= np.linalg.norm(axis)
            t = np.eye(4)
            t[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
            t[:3, 3] = xyz  # translation is in the parent frame, never rotated
            self.joints.append((t, axis, j3_offset if joint.get('name') == NAMES[2] else 0.))

    def frames(self, q):
        world = np.eye(4)
        frames, pivots, axes = [], [], []
        for value, (origin, axis, offset) in zip(q, self.joints):
            world = world @ origin
            pivots.append(world[:3, 3].copy())
            axes.append(world[:3, :3] @ axis)
            motion = np.eye(4)
            motion[:3, :3] = Rotation.from_rotvec(axis * (value + offset)).as_matrix()
            world = world @ motion
            frames.append(world.copy())
        return frames, pivots, axes

    def regressor(self, q):
        frames, pivots, axes = self.frames(q)
        result = np.zeros((4, 16))
        for link, frame in enumerate(frames):
            p, r = frame[:3, 3], frame[:3, :3]
            for joint in range(link + 1):
                result[joint, link * 4] = -axes[joint] @ np.cross(p - pivots[joint], G)
                for k in range(3):
                    result[joint, link * 4 + k + 1] = -axes[joint] @ np.cross(r[:, k], G)
        return result

    def energy(self, q, parameters):
        frames, _, _ = self.frames(q)
        return -sum(G @ (parameters[4*i] * f[:3, 3] + f[:3, :3] @ parameters[4*i+1:4*i+4])
                    for i, f in enumerate(frames))


def contiguous(times, mask):
    hits = np.flatnonzero(mask)
    if not len(hits):
        return []
    return np.split(hits, np.flatnonzero((np.diff(hits) > 1) | (np.diff(times[hits]) > .04)) + 1)


def audit_for(session, client, index, output_dir=None):
    source = client['backend_arm_receipt'].split('audit_file=')[1]
    entry = next(e for e in index if e['source'] == source)
    path = Path(output_dir or HERE) / entry['archive']
    if sha(path) != entry['archive_sha256']:
        raise ValueError('audit archive hash mismatch')
    rows = records(path)
    metadata = rows[0]
    cycles = [r for r in rows[1:] if r['command_id'] == client['command_id']]
    return metadata, cycles


def extract(window_s=2., *, data_dir=None, output_dir=None, manifest_path=None):
    data_dir=Path(data_dir or DATA)
    output_dir=Path(output_dir or HERE)
    manifest = read(manifest_path or data_dir / 'ACQUISITION_MANIFEST_20261109.json')
    index = read(output_dir / 'audit_sources.json')
    verified = {}
    for k in ('artifact', 'evidence'):
        name = manifest['admission'][k]
        actual = sha(data_dir / name)
        if actual != manifest['admission'][k + '_sha256']:
            raise ValueError(k + ' hash mismatch')
        verified[name] = actual
    poses, excluded, alignment = [], [], []
    for session in manifest['sessions']:
        path = data_dir / session['state']
        if sha(path) != session['state_sha256']:
            raise ValueError('raw hash mismatch')
        verified[session['state']] = sha(path)
        client = read(data_dir / session['client'])
        verified[session['client']] = sha(data_dir / session['client'])
        verified[session['plan']] = sha(data_dir / session['plan'])
        if client['plan'] != read(data_dir / session['plan']):
            raise ValueError('plan/client mismatch')
        raw = records(path)
        c = []
        invalid_controller = 0
        for r in raw:
            if r['kind'] != 'controller_state':
                continue
            try:
                for field in ('reference_positions','feedback_positions','reference_velocities'):
                    ordered(r['payload'],field)
                c.append(r)
            except (ValueError,KeyError,TypeError):
                invalid_controller += 1
        d = [r for r in raw if r['kind'] == 'dynamic']
        t = np.array([r['t'] for r in c])
        dt = np.array([r['t'] for r in d])
        refs = np.array([ordered(r['payload'], 'reference_positions') for r in c])
        fb = np.array([ordered(r['payload'], 'feedback_positions') for r in c])
        rv = np.array([ordered(r['payload'], 'reference_velocities') for r in c])
        metadata, audit = audit_for(session, client, index, output_dir)
        at = np.array([r['steady_s'] for r in audit])
        ar = np.array([r['target'] for r in audit])
        # Find absolute trajectory-clock offset from interior exact-target plateaus.
        # Full plateau duration, not a chain of small successive reference changes.
        candidates, offsets = [], []
        cursor = -np.inf
        for i, w in enumerate(client['waypoint_results']):
            target = np.array(w['target_rad'])
            mask = (np.max(np.abs(refs-target), axis=1) < 1e-7) & (np.max(np.abs(rv), axis=1)<1e-7)
            groups = [g for g in contiguous(t, mask) if t[g[-1]]-t[g[0]] >= .6 and t[g[0]] > cursor]
            if not w['samples'] or not groups:
                excluded.append({'id': f'{session["name"]}_{i}', 'reason': 'no client samples or no exact-target dwell'})
                candidates.append(None)
                continue
            g = groups[0]
            cursor = t[g[-1]]
            candidates.append(g)
            if i > 0:
                offsets.append(t[g[0]] - w['arrival_s'])
        if not offsets:
            raise ValueError('cannot align plan')
        plan_offset = float(np.median(offsets))
        # Audit and recorder clocks are distinct; align matching plateau arrivals.
        audit_offsets = []
        acursor = -np.inf
        for i, (w, cg) in enumerate(zip(client['waypoint_results'], candidates)):
            if cg is None or i == 0:
                continue
            mask = (np.max(np.abs(ar-w['target_rad']), axis=1)<1e-7) & (at>acursor)
            groups = [g for g in contiguous(at, mask) if at[g[-1]]-at[g[0]]>=.6]
            if groups:
                ag = groups[0]; acursor = at[ag[-1]]
                audit_offsets.append(t[cg[0]] - at[ag[0]])
        ao = float(np.median(audit_offsets))
        alignment.append({'session':session['name'],'trajectory_start_recorder_s':plan_offset,
                          'nonfinite_or_incomplete_controller_records_excluded':invalid_controller,
                          'plan_arrival_offset_spread_s':float(np.ptp(offsets)),
                          'recorder_minus_audit_s':ao,'audit_offset_spread_s':float(np.ptp(audit_offsets)),
                          'audit_metadata':metadata})
        for i, (w, cg) in enumerate(zip(client['waypoint_results'], candidates)):
            if cg is None:
                continue
            # Interior of actual plan dwell; stop/next motion cannot enter by construction.
            end = min(plan_offset+w['end_s']-.15, t[cg[-1]]-.05)
            start = max(end-window_s, plan_offset+w['arrival_s']+.5, t[cg[0]]+.1)
            selected = cg[(t[cg]>=start)&(t[cg]<=end)]
            accepted, dynrows, gainrows = [], [], []
            for k in selected:
                j = int(np.searchsorted(dt,t[k]))
                j = min((max(0,j-1),min(j,len(dt)-1)), key=lambda n:abs(dt[n]-t[k]))
                a = int(np.searchsorted(at,t[k]-ao))
                a = min((max(0,a-1),min(a,len(at)-1)),key=lambda n:abs(at[n]+ao-t[k]))
                axes = [d[j]['payload']['interfaces'][n] for n in NAMES]
                if abs(dt[j]-t[k])>.025 or abs(at[a]+ao-t[k])>.025:
                    continue
                if any(v['command_id']!=client['command_id'] or v['motor_state']!=1 or
                       v['control_state'] not in (5,6) or v['feedback_valid']!=1 or v['audit_ok']!=1 or
                       v['configuration_conflict']!=0 or v['feedback_age']>.06 or
                       v['timeout_count']!=2000 or abs(v['velocity'])>.035 for v in axes):
                    continue
                if audit[a]['control_state'] not in (5,6) or any(v is None for v in audit[a].get('kp_encoded',[])):
                    continue
                if audit[a].get('kp_output_state')!='WRITTEN_CURRENT' or audit[a].get('request_estimate_state')!='CURRENT':
                    continue
                stamps=audit[a].get('kp_last_written_at',[])
                if len(stamps)!=5 or any(v is None or not 0<=at[a]-v<=.05 for v in stamps):
                    continue
                if np.max(np.abs(np.array(audit[a]['target'])-w['target_rad']))>1e-6:
                    continue
                accepted.append(k);dynrows.append(axes);gainrows.append(audit[a])
            if len(accepted)<50 or t[accepted[-1]]-t[accepted[0]]<.6:
                excluded.append({'id':f'{session["name"]}_{i}','reason':'insufficient fresh stationary active samples','samples':len(accepted)})
                continue
            q = fb[accepted]
            vel = np.array([[v['velocity'] for v in axes] for axes in dynrows])
            effort = np.array([[v['effort'] for v in axes] for axes in dynrows])
            kp = np.array([r['kp_encoded'] for r in gainrows])
            # Independently retain host request estimates (includes codec PD details).
            request = np.array([r['request_estimate_nm'] for r in gainrows])
            span = np.ptp(np.degrees(q),axis=0)
            if max(span)>.4:
                excluded.append({'id':f'{session["name"]}_{i}','reason':'position drift >0.4 deg','span_deg':span.tolist()})
                continue
            item = {'id':f'{session["name"]}_{i}','session':session['name'],'waypoint':i,
                    'q_rad':np.mean(q,axis=0).tolist(),'target_rad':w['target_rad'],
                    'q_std_rad':np.std(q,axis=0).tolist(),'q_span_deg':span.tolist(),
                    'effort_mean_nm':np.mean(effort,axis=0).tolist(),
                    'effort_median_nm':np.median(effort,axis=0).tolist(),
                    'effort_std_nm':np.std(effort,axis=0).tolist(),
                    'request_motor_mean_nm':np.mean(request,axis=0).tolist(),
                    'request_mean_nm':(np.mean(request,axis=0)*np.array([j['sign'] for j in metadata['joint_policy']])).tolist(),
                    'kp_encoded_mean':np.mean(kp,axis=0).tolist(),
                    'velocity_mean_rad_s':np.mean(vel,axis=0).tolist(),
                    'samples':len(q),'recorder_window_s':[float(t[accepted[0]]),float(t[accepted[-1]])],
                    'max_dynamic_join_s':max(abs(dt[int(np.argmin(abs(dt-t[k])))]-t[k]) for k in accepted),
                    'signed_error_mean_deg':np.mean(np.degrees(q-refs[accepted]),axis=0).tolist()}
            poses.append(item)
    result = {'schema':'slaihand_payload_static_windows/1','window_s':window_s,
              'source_hashes':verified,'alignment':alignment,'poses':poses,'excluded':excluded}
    (output_dir/'static_windows.json').write_text(json.dumps(result,indent=2)+'\n')
    with (output_dir/'static_windows.csv').open('w',newline='',encoding='utf-8-sig') as stream:
        writer=csv.writer(stream);writer.writerow(['id','samples',*[f'q{i}_deg' for i in range(1,6)],*[f'effort{i}_Nm' for i in range(1,6)],*[f'PD_request{i}_Nm' for i in range(1,6)]])
        for p in poses:
            writer.writerow([p['id'],p['samples'],*np.degrees(p['q_rad']),*p['effort_mean_nm'],*p['request_mean_nm']])
    return result


def metrics(prediction, target):
    residual = prediction-target
    return {'rmse_nm':float(np.sqrt(np.mean(residual**2))),
            'per_axis_rmse_nm':np.sqrt(np.mean(residual**2,axis=0)).tolist(),
            'per_axis_max_abs_nm':np.max(np.abs(residual),axis=0).tolist()}


def prior_parameters():
    path = ROOT/'src/slaihand_bringup/config/gravity_identified_candidate.json'
    profile = read(path)
    expected = [f'link{i}:{f}' for i in range(1,5) for f in ('mass','hx','hy','hz')]
    if profile['parameter_slots'] != expected:
        raise ValueError('unexpected prior parameter order')
    if not np.allclose(profile['identity']['gravity_base_m_s2'],G,atol=1e-12) or abs(profile['identity']['j3_offset_rad']-math.pi)>1e-12:
        raise ValueError('prior convention changed')
    compatibility = profile.get('compatibility_update',{})
    if sha(URDF) not in (profile['identity']['urdf_sha256'],compatibility.get('target_urdf_sha256')):
        raise ValueError('prior URDF compatibility not established')
    cadpath = ROOT/'verification/gravity_preparation/provisional_gravity.json'
    cad = read(cadpath)
    parameters=[]
    for k in range(1,5):
        link = cad['links'][f'link{k}']; m=link['mass_kg']
        parameters.extend([m,*(m*np.array(link['com_link_m']))])
    return np.array(profile['effective_coefficients']),np.array(parameters),{
        'identified_prior_sha256':sha(path),'cad_profile_sha256':sha(cadpath),
        'urdf_sha256':sha(URDF),'prior_holdout':profile['holdout'],
        'motor_already_in_cad_profile_kg':.362,
        'motor_not_added_again':True,
        'mass_definition':'3.5 kg added box/load EXCLUDING existing J5 motor; user clarified 2026-11-09'}


def fit_model(x,y,prior,mass,selected,bias=False,robust=False):
    base=x@prior+mass*x[:,:,12]
    h=x[:,:,13:16]
    design=np.concatenate([h,np.broadcast_to(np.eye(4),(len(x),4,4))],axis=2) if bias else h
    d=design[selected].reshape(-1,design.shape[-1]); target=(y-base)[selected].ravel()
    params=np.linalg.lstsq(d,target,rcond=1e-6)[0]
    if robust:
        params=least_squares(lambda z:d@z-target,params,loss='soft_l1',f_scale=.5).x
    return params,base+design@params,design


def fit(extracted):
    poses=extracted['poses'];geometry=Geometry()
    x=np.stack([geometry.regressor(p['q_rad'][:4]) for p in poses])
    prior,cad,provenance=prior_parameters()
    train=np.array([p['session']!='C' for p in poses]);hold=~train
    allrows=np.ones(len(poses),dtype=bool)
    if sum(train)!=22 or sum(hold)!=8:
        raise ValueError('expected frozen split A/A2/B2=22, C=8; review extraction before fitting')
    y=np.array([p['effort_mean_nm'][:4] for p in poses])
    request=np.array([p['request_mean_nm'][:4] for p in poses])
    gain=np.array([p['kp_encoded_mean'][:4] for p in poses])
    trained,pred,_=fit_model(x,y,prior,3.5,train)
    final,finalpred,_=fit_model(x,y,prior,3.5,allrows)
    u,s,v=np.linalg.svd(x.reshape(-1,16),full_matrices=False)
    rank=int(np.sum(s>s[0]*1e-6));null=v[rank:].T
    # A structural non-uniqueness witness: change link4 hz by 0.35 kg m
    # and compensate upstream inertial slots, leaving ALL static torques unchanged.
    witness=np.zeros(16)
    witness[15]=.35
    witness[9:12]=-geometry.joints[3][0][:3,2]*.35
    rng=np.random.default_rng(20261109)
    bootstrap=[]
    for _ in range(2000):
        pick=rng.integers(0,len(poses),len(poses))
        # Whole dwell windows are resampled; never treat 100Hz samples as independent poses.
        d=x[pick,:,13:16].reshape(-1,3)
        z=(y-x@prior-3.5*x[:,:,12])[pick].ravel()
        if np.linalg.matrix_rank(d)==3:
            bootstrap.append(np.linalg.lstsq(d,z,rcond=1e-6)[0]/3.5)
    sensitivity=[]
    def add(name,signal,base,mass,rows,bias=False,robust=False):
        par,prediction,_=fit_model(x,signal,base,mass,rows,bias,robust)
        sensitivity.append({'name':name,'com_link4_mm':(par[:3]/mass*1000).tolist(),
                            'constant_bias_nm':par[3:].tolist(),
                            'all_pose_metrics':metrics(prediction,signal)})
    add('identified_prior_all_poses',y,prior,3.5,allrows)
    add('CAD_prior_all_poses',y,cad,3.5,allrows)
    add('identified_prior_plus_axis_bias',y,prior,3.5,allrows,bias=True)
    add('identified_prior_robust_soft_l1',y,prior,3.5,allrows,robust=True)
    add('host_PD_proxy_same_model_frame',request,prior,3.5,allrows)
    for mass in (3.,4.):
        add(f'mass_assumed_{mass:g}_kg',y,prior,mass,allrows)
    for session in ('A','A2','B2','C'):
        add('single_session_'+session,y,prior,3.5,np.array([p['session']==session for p in poses]))
    pairs=[]
    for i in range(len(poses)):
        for j in range(i):
            distance=float(np.max(np.abs(np.degrees(np.array(poses[i]['q_rad'][:4])-poses[j]['q_rad'][:4]))))
            difference=y[i]-y[j]
            if distance<.65 and max(abs(difference))>1.:
                pairs.append({'poses':[poses[j]['id'],poses[i]['id']],'max_pose_difference_deg':distance,
                              'effort_difference_nm':difference.tolist()})
    effective=np.linalg.lstsq(x[train].reshape(-1,16),y[train].ravel(),rcond=1e-6)[0]
    residual_proxy_deg=np.degrees((pred[hold]-y[hold])/gain[hold])
    model={
        'schema':'slaihand_payload_identification_offline/1','status':'CONDITIONAL_ESTIMATE_VALIDATION_INSUFFICIENT',
        'deployable':False,'parameters_applied':False,'gravity_ff_alpha_changed':False,
        'provenance':dict(provenance,source_commit='99b28e9ef67ccf83e24c7e47f425ac83d43b84a1'),
        'coordinates':{'com_frame':'link4, canonical URDF axes; same fixed J3 pi placement as previous compensation',
                       'gravity_base_m_s2':G.tolist(),'j3_fixed_offset_rad':math.pi,
                       'joint_order':NAMES[:4],'q5_gravity_term':False,
                       'torque_sign':'holding torque dU/dq in model coordinates; motor sign [1,1,1,-1]'},
        'payload_mass_kg':3.5,'mass_measured':False,
        'fit_target':'DAMIAO effort estimate (not current/torque measurement)',
        'fit_equation':'tau(q) = Y(q)*theta_old + 3.5*Y_link4_mass(q) + H_link4(q)*h_box',
        'split_policy':'Session C excluded from parameter fitting in the primary validation model; full-data refit is a separate provisional artifact',
        'frozen_training_pose_ids':[p['id'] for p in poses if p['session']!='C'],
        'holdout_pose_ids':[p['id'] for p in poses if p['session']=='C'],
        'training_first_moment_kg_m':trained.tolist(),'training_com_link4_m':(trained/3.5).tolist(),
        'training_metrics':metrics(pred[train],y[train]),'holdout_metrics':metrics(pred[hold],y[hold]),
        'holdout_no_compensation_proxy':metrics(np.zeros_like(y[hold]),y[hold]),
        'holdout_residual_angle_proxy_deg':{'meaning':'torque-model residual / recorded encoded Kp; NOT a replay or physical accuracy test',
             'per_axis_rmse':np.sqrt(np.mean(residual_proxy_deg**2,axis=0)).tolist(),
             'per_axis_max_abs':np.max(abs(residual_proxy_deg),axis=0).tolist(),
             'all_axes_max_under_0p5_deg':bool(np.max(abs(residual_proxy_deg))<=.5)},
        'final_refit_all_30_poses':{'first_moment_kg_m':final.tolist(),'com_link4_m':(final/3.5).tolist(),
                                  'metrics':metrics(finalpred,y),'not_an_independent_validation':True},
        'conditional_window_bootstrap_95_percentile_m':np.percentile(bootstrap,[2.5,97.5],axis=0).tolist(),
        'bootstrap_scope':'Conditional sampling variation only; does NOT include mass, friction, old-model error or torque scale uncertainty',
        'identifiability':{'payload_only_rank':int(np.linalg.matrix_rank(x[:,:,13:16].reshape(-1,3))),
              'full_arm_rank':rank,'full_arm_columns':16,'full_arm_singular_values':s.tolist(),
              'nullspace_energy_for_link4_hxyz':np.diag(null[13:16]@null[13:16].T).tolist(),
              'nullspace_witness_parameter_delta':witness.tolist(),
              'witness_h4z_change_kg_m':float(witness[15]),
              'witness_keeps_all_link_masses_fixed':True,
              'witness_max_torque_change_nm':float(np.max(abs(x@witness))),
              'interpretation':'Full link4 hx/hy observable in ideal gravity model; hz confounded with upstream parameters. Box COM needs original link4 subtraction and known mass.'},
        'sensitivity':sensitivity,'near_repeat_hysteresis_examples':pairs,
        'all_arm_effective_fit_holdout':metrics(x[hold]@effective,y[hold]),
        'prediction_rows':[{'id':p['id'],'role':'holdout' if p['session']=='C' else 'training',
                            'measured_estimate_nm':y[k].tolist(),'prediction_nm':pred[k].tolist(),
                            'residual_nm':(pred[k]-y[k]).tolist()} for k,p in enumerate(poses)],
    }
    (HERE/'identification_result.json').write_text(json.dumps(model,indent=2)+'\n')
    combined=prior.copy();combined[12]+=3.5;combined[13:16]+=final
    candidate={'schema':'slaihand_offline_payload_ff_candidate/1','deployable':False,
               'reason':'holdout precision and physical COM validation insufficient; no production schema',
               'coordinates':model['coordinates'],'theta_total_kg_and_kg_m':combined.tolist(),
               'payload_first_moment_kg_m':final.tolist(),'payload_mass_assumed_kg':3.5,
               'observed_q_bounds_rad':np.array([np.min([p['q_rad'][:4] for p in poses],axis=0),np.max([p['q_rad'][:4] for p in poses],axis=0)]).T.tolist(),
               'source_result_sha256':sha(HERE/'identification_result.json'),'motor_sign':[1,1,1,-1],
               'output':'model torque = Geometry.regressor(q) @ theta; motor torque = signs * model torque',
               'allowed_hardware_torque_nm':None,'feedforward_enabled':False}
    (HERE/'offline_ff_candidate.json').write_text(json.dumps(candidate,indent=2)+'\n')
    make_plot(model)
    return model


def make_plot(result):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows=[r for r in result['prediction_rows'] if r['role']=='holdout']
    measured=np.array([r['measured_estimate_nm'] for r in rows]);predicted=np.array([r['prediction_nm'] for r in rows])
    fig,axes=plt.subplots(2,3,figsize=(14,8),layout='constrained')
    for j,ax in enumerate(axes.flat[:4]):
        ax.plot(range(8),measured[:,j],'o-',label='Recorded torque estimate')
        ax.plot(range(8),predicted[:,j],'s--',label='Frozen fit prediction')
        ax.set(title=f'J{j+1}: session C held out',xlabel='C waypoint',ylabel='Model torque [Nm]')
        ax.grid(alpha=.25)
    axes[0,0].legend(fontsize=8)
    values=np.array([v['com_link4_mm'] for v in result['sensitivity'][:5]])
    ax=axes[1,1]
    for j,label in enumerate(('x','y','z')):
        ax.plot(range(len(values)),values[:,j],'o-',label=label)
    ax.set_xticks(range(len(values)),['Old model','CAD','+bias','Robust','PD proxy'],rotation=20)
    ax.set(title='COM sensitivity: fixed 3.5 kg assumption',ylabel='COM in link4 [mm]');ax.legend();ax.grid(alpha=.25)
    ax=axes[1,2];ax.axis('off')
    com=np.array(result['final_refit_all_30_poses']['com_link4_m'])*1000
    ax.text(.02,.9,'CONDITIONAL ESTIMATE\nNOT DEPLOYABLE',fontsize=15,weight='bold',color='#a83232',va='top')
    ax.text(.02,.62,f'30 static windows\nBox mass: 3.5 kg (motor excluded)\nCOM: {com[0]:.1f}, {com[1]:.1f}, {com[2]:.1f} mm\n\nHeld-out RMSE: {result["holdout_metrics"]["rmse_nm"]:.3f} Nm\nTorque estimate, not measured current\nAxial COM depends on old-model prior',fontsize=11,va='top')
    fig.suptitle('Added box gravity identification — offline evidence, 2026-11-09',fontsize=17)
    fig.savefig(HERE/'identification_diagnostics.png',dpi=170)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--window-s',type=float,default=2.)
    args=parser.parse_args()
    extracted=extract(args.window_s)
    result=fit(extracted)
    print('Accepted',len(extracted['poses']),'excluded',extracted['excluded'])
    print('Conditional all-pose COM mm:',np.array(result['final_refit_all_30_poses']['com_link4_m'])*1000)
    print('Frozen C holdout:',result['holdout_metrics'])
    print('95% bootstrap, conditional only, mm:',np.array(result['conditional_window_bootstrap_95_percentile_m'])*1000)


if __name__=='__main__':
    main()
