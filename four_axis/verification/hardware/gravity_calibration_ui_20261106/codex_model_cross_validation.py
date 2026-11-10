from pathlib import Path
import json,hashlib,sys,math
import numpy as np
root=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(root/'src/slaihand_bringup/scripts'))
from control_panel_gravity import FourAxisGravityModel
urdf=root/'src/slaihand_description/urdf/slaihand_four_axis.urdf'
oldpath=root/'src/slaihand_bringup/config/gravity_identified_candidate.json'
newpath=root/'verification/hardware/gravity_calibration_ui_20261106/evidence/gravity_large_angle_candidate.json'
models={name:FourAxisGravityModel(urdf,path,[0,9.80665,0],fixed_joint_offsets={'slaihand_joint3':math.pi}) for name,path in [('old',oldpath),('new',newpath)]}
results={}
for campaign,profile in [('original',oldpath),('large_angle',newpath)]:
    sets={'training':[],'holdout':[]}
    for source in json.loads(profile.read_text())['sources']:
        path=Path(source['raw_file'])
        assert hashlib.sha256(path.read_bytes()).hexdigest()==source['sha256'],path
        rows=[json.loads(line) for line in path.open()]
        samples=[row for row in rows if 'positions_rad' in row]
        by_pair={}
        for row in samples:
            if row.get('approach') in ('positive','negative'):
                by_pair.setdefault(row['pair_id'],{})[row['approach']]=row
        for pair in by_pair.values():
            a,b=pair['positive'],pair['negative']
            role='holdout' if a.get('role')=='holdout' else 'training'
            measured=(np.array(a['effort_estimate_nm'])+np.array(b['effort_estimate_nm']))/2
            sets[role].append((a['positions_rad'],b['positions_rad'],measured))
    results[campaign]={}
    for role,rows in sets.items():
        metrics={}
        for name,model in models.items():
            residuals=[]
            for qa,qb,observed in rows:
                prediction=(np.array(model.compute(qa)['tau_model_nm'])+np.array(model.compute(qb)['tau_model_nm']))/2
                residuals.append(prediction-observed)
            errors=np.array(residuals)
            metrics[name]={'rmse_nm':float(np.sqrt(np.mean(errors**2))),'axis_rmse_nm':np.sqrt(np.mean(errors**2,axis=0)).tolist(),'axis_max_nm':np.max(abs(errors),axis=0).tolist()}
        results[campaign][role]={'pairs':len(rows),'metrics':metrics}
print(json.dumps(results,indent=2))
(root/'verification/hardware/gravity_calibration_ui_20261106/evidence/codex_model_cross_validation.json').write_text(json.dumps(results,indent=2))
