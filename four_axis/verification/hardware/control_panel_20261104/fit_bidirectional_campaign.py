"""Offline paired gravity identification with an untouched V01 holdout."""
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT/'src/slaihand_bringup/scripts'))
import calibration_core as core
import calibration_capture as capture
from control_panel_gravity import FourAxisGravityModel

OPERATIONS={'S01':'a6e15593c30a4bc1','S02':'77dcfafb8417467e','S03':'c4b67a21c50e4eab','S04':'aa7a855983eb4264','V01':'5e5f47ea815c45a1'}


def load_campaign():
    pairs=[]
    sources=[]
    expected_identity=None
    for group,operation in OPERATIONS.items():
        directory=Path.home()/'.ros/slaihand/control_panel/operations'/operation/'client'
        result=json.loads(next(directory.glob('*.json')).read_text())
        summary=result['calibration']
        assert summary['group']==group and summary['campaign_complete']
        assert result['trajectory_error_code']==0 and result['stop_observations_confirmed']
        raw=Path(summary['raw_file'])
        payload=core.load_raw(raw)
        rows=payload['samples']
        assert len(rows)==(8 if group=='V01' else 12)
        role='holdout' if group=='V01' else 'training'
        for row in rows:
            ok,reasons=core.validate_sample(row)
            assert ok,reasons
            assert core.exclusion_reason(row) is None
            assert row['role']==role
            identity=core._identity(row)
            if expected_identity is None:
                expected_identity=identity
            assert identity==expected_identity,'mixed configuration identity'
        for index in range(0,len(rows),4):
            positive,negative=rows[index+1],rows[index+3]
            assert positive['pair_id']==negative['pair_id']
            assert positive['approach']=='positive' and negative['approach']=='negative'
            assert positive['target_rad']==negative['target_rad']
            pairs.append({'pair_id':positive['pair_id'],'role':role,
                'positive_q_rad':positive['positions_rad'],'negative_q_rad':negative['positions_rad'],
                'mean_estimate_nm':((np.array(positive['effort_estimate_nm'])+negative['effort_estimate_nm'])/2).tolist(),
                'half_difference_nm':((np.array(positive['effort_estimate_nm'])-negative['effort_estimate_nm'])/2).tolist(),
                'pose_difference_deg':np.degrees(np.array(positive['positions_rad'])-negative['positions_rad']).tolist()})
        sources.append({'group':group,'operation_id':operation,'raw_file':str(raw),
            'sha256':hashlib.sha256(raw.read_bytes()).hexdigest(),'qualified_points':len(rows)})
    current_identity = capture._expected_identity(capture.provenance('no_end_load'))
    assert core.identity_matches(expected_identity, current_identity), 'analysis resources differ from capture identity'
    return pairs,sources,expected_identity


def geometry_regressor(model,q):
    """Coefficients for link mass and first moments, independent of prior masses."""
    columns=[]
    original=model.masses
    try:
        for link in original:
            if link==model.root:
                continue
            model.masses={link:(1.,[0.,0.,0.])}
            origin=np.array(model.compute(q)['tau_model_nm'])
            columns.append(origin)
            for axis in range(3):
                center=[0.,0.,0.]
                center[axis]=1.
                model.masses={link:(1.,center)}
                columns.append(np.array(model.compute(q)['tau_model_nm'])-origin)
    finally:
        model.masses=original
    return np.array(columns).T


def metrics(prediction,measured):
    error=prediction-measured
    return {'rmse_nm':float(np.sqrt(np.mean(error**2))),
            'per_axis_rmse_nm':np.sqrt(np.mean(error**2,axis=0)).tolist(),
            'per_axis_max_abs_error_nm':np.max(np.abs(error),axis=0).tolist(),
            'prediction_nm':prediction.tolist(),'measured_nm':measured.tolist(),'error_nm':error.tolist()}


def main():
    pairs,sources,identity=load_campaign()
    policy=capture._policy_values()
    model=FourAxisGravityModel(capture.URDF,capture.PROFILE,
        gravity_base=[float(policy['gravity_base_'+axis]) for axis in 'xyz'],
        fixed_joint_offsets={'slaihand_joint3':float(policy['gravity_j3_offset_rad'])})
    train=[p for p in pairs if p['role']=='training']
    holdout=[p for p in pairs if p['role']=='holdout']
    assert len(train)==12 and len(holdout)==2
    # Construct and freeze the training fit before evaluating V01.
    def matrices(rows):
        return np.stack([(geometry_regressor(model,p['positive_q_rad'])+geometry_regressor(model,p['negative_q_rad']))/2 for p in rows])
    xtrain=matrices(train)
    ytrain=np.array([p['mean_estimate_nm'] for p in train])
    design=xtrain.reshape(-1,xtrain.shape[-1])
    coefficients,residuals,rank,singular=np.linalg.lstsq(design,ytrain.ravel(),rcond=1e-6)
    _, _, right_vectors = np.linalg.svd(design, full_matrices=False)
    provisional_train=np.array([(np.array(model.compute(p['positive_q_rad'])['tau_model_nm'])+model.compute(p['negative_q_rad'])['tau_model_nm'])/2 for p in train])
    scale=float(np.sum(provisional_train*ytrain)/np.sum(provisional_train**2))
    # Check the regressor algebra against the existing analytic evaluator.
    parameters=[]
    names=[]
    for link,(mass,com) in model.masses.items():
        if link==model.root:continue
        parameters.extend([mass,*[mass*x for x in com]])
        names.extend([link+':mass',link+':hx',link+':hy',link+':hz'])
    algebra_error=float(np.max(np.abs(xtrain@np.array(parameters)-provisional_train)))
    assert algebra_error<1e-10,algebra_error
    xhold=matrices(holdout)
    yhold=np.array([p['mean_estimate_nm'] for p in holdout])
    provisional_hold=np.array([(np.array(model.compute(p['positive_q_rad'])['tau_model_nm'])+model.compute(p['negative_q_rad'])['tau_model_nm'])/2 for p in holdout])
    physical={'kind':'URDF_linear_gravity_base_parameters','rank':int(rank),'columns':len(names),
        'rcond':1e-6,'singular_values':singular.tolist(),
        'minimum_norm_parameters':dict(zip(names,coefficients.tolist())),
        'train':metrics(xtrain@coefficients,ytrain),'holdout':metrics(xhold@coefficients,yhold)}
    baseline={'kind':'provisional_model_global_scale','scale':scale,
        'train':metrics(scale*provisional_train,ytrain),'holdout':metrics(scale*provisional_hold,yhold)}
    report={'schema':'paired_gravity_offline_review/1','sources':sources,'identity':identity,
        'training_pairs':12,'independent_validation_pairs':2,'pairs':pairs,
        'regressor_algebra_error_nm':algebra_error,'baseline':baseline,'geometry_fit':physical,
        'acceptance_rmse_nm':core.HOLDOUT_RMSE_MAX_NM,
        'geometry_holdout_rmse_passed':physical['holdout']['rmse_nm']<=core.HOLDOUT_RMSE_MAX_NM,
        'parameters_applied':False,'mass_profile_written':False,'alpha_changed':False}
    report['geometry_parameters_physically_validated']=False
    candidate = {
        'schema': 'slaihand_identified_gravity_base/1',
        'status': 'LOCAL_OFFLINE_CANDIDATE',
        'identity': identity,
        'sources': sources,
        'training_pair_ids': [pair['pair_id'] for pair in train],
        'validation_pair_ids': [pair['pair_id'] for pair in holdout],
        'parameter_slots': names,
        'effective_coefficients': coefficients.tolist(),
        'identifiable_rank': int(rank),
        'identifiable_basis': right_vectors[:rank].tolist(),
        'base_coefficients': (right_vectors[:rank] @ coefficients).tolist(),
        'observed_training_bounds_rad': np.stack([
            np.min([q for pair in train for q in (pair['positive_q_rad'], pair['negative_q_rad'])], axis=0),
            np.max([q for pair in train for q in (pair['positive_q_rad'], pair['negative_q_rad'])], axis=0)
        ]).T.tolist(),
        'prior_mass_values_used_in_fit': False,
        'individual_link_masses_identified': False,
        'holdout': physical['holdout'],
        'parameters_applied': False,
    }
    gains = set()
    for source in sources:
        header = core.load_raw(source['raw_file'])['header']
        runtime = header['runtime']['identity']
        gains.add((runtime['kp'], runtime['kd']))
    assert len(gains) == 1, 'capture gains changed within campaign'
    kp, kd = next(iter(gains))
    candidate['training_gains'] = {'kp':kp, 'kd':kd}
    candidate_path = HERE / 'evidence/identified_gravity_candidate.json'
    candidate_path.write_text(json.dumps(candidate, ensure_ascii=False, indent=2))
    report['candidate_file'] = str(candidate_path)
    output=HERE/'evidence/paired_gravity_campaign_fit.json'
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({'sources':sources,'training_pairs':12,'holdout_pairs':2,'rank':int(rank),'columns':len(names),
        'baseline_train':baseline['train'],'baseline_holdout':baseline['holdout'],
        'geometry_train':physical['train'],'geometry_holdout':physical['holdout'],'report':str(output)},ensure_ascii=False))

if __name__=='__main__':main()
