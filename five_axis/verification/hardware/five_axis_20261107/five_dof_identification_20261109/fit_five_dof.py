"""Offline five-coordinate potential identification; no invented J5 mount.

The point-mass orbit is represented in link4 coordinates as
  r(q5) = b + u*cos(q5) + v*sin(q5).
A real rigid revolute orbit additionally requires u.v=0 and |u|=|v|.
This representation does not assume the placeholder URDF J5 axis is physical.
"""
import json
import math
from pathlib import Path
import sys
import numpy as np
from scipy.optimize import least_squares

HERE=Path(__file__).resolve().parent
BASE=HERE.parent
sys.path.insert(0,str(BASE/'identification_codex_20261109'))
import identify_payload as legacy


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))


class FiveDofPotential:
    def __init__(self):
        self.geometry=legacy.Geometry()
        self.prior=legacy.prior_parameters()[0]

    def orbit_regressor(self,q):
        q=np.asarray(q,dtype=float)
        if q.shape!=(5,) or not np.isfinite(q).all():raise ValueError('five finite model radians required')
        four=self.geometry.regressor(q[:4]);H=four[:,13:16]
        rotation=self.geometry.frames(q[:4])[0][-1][:3,:3]
        local_g=rotation.T@legacy.G
        c,s=math.cos(q[4]),math.sin(q[4])
        out=np.zeros((5,9));out[:4]=np.concatenate([H,H*c,H*s],axis=1)
        out[4,3:6]=local_g*s
        out[4,6:9]=-local_g*c
        return out

    def base(self,q):
        four=self.geometry.regressor(q[:4])
        return np.r_[four@self.prior+3.5*four[:,12],0.]

    def energy(self,q,parameters):
        frame=self.geometry.frames(q[:4])[0][-1]
        hg=np.array(parameters).reshape(3,3)
        moment=hg[0]+hg[1]*math.cos(q[4])+hg[2]*math.sin(q[4])
        return self.geometry.energy(q[:4],self.prior)-legacy.G@(3.5*frame[:3,3]+frame[:3,:3]@moment)


def metrics(pred,truth):
    return legacy.metrics(np.asarray(pred),np.asarray(truth))


def rigid_parameters(z):
    """Seven orbit DOFs; translation is NOT an asserted measured J5 pivot."""
    theta,phi=z[5:7]
    axis=np.array([math.cos(theta)*math.cos(phi),math.sin(theta)*math.cos(phi),math.sin(phi)])
    e1=np.array([-math.sin(theta),math.cos(theta),0.])
    e2=np.cross(axis,e1)
    u=z[3]*e1+z[4]*e2
    v=np.cross(axis,u)
    return 3.5*np.r_[z[:3],u,v],axis


def fit_rigid_variants(poses,x,base,y,train_rows,train,isj5,direction):
    hold=np.array([p['session'] in ('C','GC03','GC04') for p in poses])
    jhold=isj5&~train
    history={};full_direction=[]
    for p in poses:
        last=history.setdefault(p['session'],np.zeros(5))
        changes=np.array(p['planned_approach_sign'])
        last=np.where(changes!=0,changes,last);history[p['session']]=last
        full_direction.append(last.copy())
    full_direction=np.array(full_direction)
    results=[]
    for kind in ('gravity_only','J5_approach_nuisance','all_axes_approach_nuisance'):
        n=0 if kind=='gravity_only' else 1 if kind=='J5_approach_nuisance' else 5
        def prediction(z):
            h,_=rigid_parameters(z)
            nuisance=0 if n==0 else direction[:,:,0]*z[7] if n==1 else full_direction*z[7:]
            return base+x@h+nuisance
        def residual(z):return (prediction(z)-y)[train_rows]
        solutions=[]
        for theta in np.linspace(-math.pi,math.pi,6,endpoint=False):
            for phi in (-.8,0.,.8):
                initial=[.1,.01,-.05,.03,.01,theta,phi]+[.8]*n
                solved=least_squares(residual,initial,max_nfev=700,ftol=1e-9,xtol=1e-9,gtol=1e-9)
                solutions.append(solved)
        best=min(solutions,key=lambda s:s.cost)
        h,axis=rigid_parameters(best.x)
        prediction_total=prediction(best.x);gravity=base+x@h
        results.append({'name':'rigid_circle_'+kind,'mass_assumed_kg':3.5,
            'training_cost':float(best.cost),'optimizer_success':bool(best.success),
            'jacobian_condition_number':float(np.linalg.cond(best.jac)),
            'near_best_multistart_count':int(sum(s.cost<=best.cost*1.02 for s in solutions)),
            'orbit_moment_b_u_v_kg_m':h.reshape(3,3).tolist(),
            'COM_at_J5_zero_in_link4_m':((h[:3]+h[3:6])/3.5).tolist(),
            'COM_orbit_radius_m':float(np.linalg.norm(h[3:6])/3.5),
            'fitted_axis_direction_link4_unverified':axis.tolist(),
            'nuisance_coefficients_nm':best.x[7:].tolist(),
            'upstream_heldout_total_with_nuisance':metrics(prediction_total[hold,:4],y[hold,:4]),
            'upstream_heldout_gravity_only_vs_effort':metrics(gravity[hold,:4],y[hold,:4]),
            'J5_pm20_heldout_rmse_total_nm':float(np.sqrt(np.mean((prediction_total[jhold,4]-y[jhold,4])**2))),
            'J5_pm20_heldout_prediction_total_nm':prediction_total[jhold,4].tolist(),
            'J5_all_gravity_component_nm':gravity[isj5,4].tolist(),
            'J5_all_prediction_with_nuisance_nm':prediction_total[isj5,4].tolist(),
            'deployable':False,
            'scope':'mathematically rigid orbit; spatial axis/pivot and nuisance law are unmeasured. Nuisance term is not gravity FF.'})
    return results


def main():
    extracted=legacy.extract(data_dir=BASE,output_dir=HERE,manifest_path=HERE/'SOURCE_MANIFEST.json')
    poses=extracted['poses']
    manifest=read(HERE/'SOURCE_MANIFEST.json')
    clients={s['name']:read(BASE/s['client']) for s in manifest['sessions']}
    for p in poses:
        c=clients[p['session']];i=p['waypoint'];previous=c['initial_model_rad'] if i==0 else c['waypoint_results'][i-1]['target_rad']
        delta=np.array(p['target_rad'])-previous
        p['planned_approach_sign']=[int(np.sign(v)) if abs(v)>1e-5 else 0 for v in delta]
    (HERE/'static_windows.json').write_text(json.dumps(extracted,indent=2)+'\n',encoding='utf-8')
    model=FiveDofPotential()
    x=np.stack([model.orbit_regressor(p['q_rad']) for p in poses])
    base=np.stack([model.base(p['q_rad']) for p in poses])
    y=np.array([p['effort_mean_nm'] for p in poses])
    isj5=np.array([p['session']=='J5' for p in poses])
    train=np.array([p['session'] not in ('C','GC03','GC04') and p['id'] not in ('J5_3','J5_4') for p in poses])
    observed=np.ones((len(poses),5),dtype=bool)
    observed[~isj5,4]=False # old constant-q5 holdings do not independently identify J5 gravity
    train_rows=observed&train[:,None]
    direction=np.zeros((len(poses),5,1))
    for i,p in enumerate(poses):
        if isj5[i]:direction[i,4,0]=p['planned_approach_sign'][4]
    variants=[]
    predictions={}
    for use_friction in (False,True):
        design=np.concatenate([x,direction],axis=2) if use_friction else x
        d=design[train_rows];target=(y-base)[train_rows]
        par,_,rank,singular=np.linalg.lstsq(d,target,rcond=1e-6)
        prediction=base+design@par
        pred_gravity=base+x@par[:9]
        hu,hv=par[3:6],par[6:9];a,b=np.linalg.norm(hu),np.linalg.norm(hv)
        cos=float(np.dot(hu,hv)/(a*b)) if a*b>1e-12 else None
        mismatch=float(abs(a-b)/max(a,b)) if max(a,b)>1e-12 else None
        # This gate tests a necessary algebraic orbit condition, not safety.
        circle=cos is not None and abs(cos)<.05 and mismatch<.05
        up_hold=np.array([p['session'] in ('C','GC03','GC04') for p in poses])
        j5_hold=isj5&~train
        label='potential_plus_J5_approach_nuisance' if use_friction else 'potential_only'
        result={'name':label,'rank':int(rank),'columns':design.shape[-1],
            'condition_number':float(singular[0]/singular[-1]),'singular_values':singular.tolist(),
            'moment_b_u_v_kg_m':par[:9].reshape(3,3).tolist(),
            'box_orbit_b_u_v_link4_m':(par[:9].reshape(3,3)/3.5).tolist(),
            'box_position_at_q5_zero_link4_m':((par[:3]+par[3:6])/3.5).tolist(),
            'J5_approach_nuisance_nm':float(par[9]) if use_friction else None,
            'orbit_diagnostics':{'u_radius_m':float(a/3.5),'v_radius_m':float(b/3.5),
                'u_v_cosine':cos,'relative_radius_mismatch':mismatch,'necessary_rigid_circle_condition_passed':circle},
            'upstream_training':metrics(prediction[train,:4],y[train,:4]),
            'upstream_independent_sessions':metrics(prediction[up_hold,:4],y[up_hold,:4]),
            'J5_train_rmse_including_nuisance_nm':float(np.sqrt(np.mean((prediction[isj5&train,4]-y[isj5&train,4])**2))),
            'J5_heldout_pm20_rmse_including_nuisance_nm':float(np.sqrt(np.mean((prediction[j5_hold,4]-y[j5_hold,4])**2))),
            'J5_heldout_prediction_nm':prediction[j5_hold,4].tolist(),
            'J5_heldout_measured_estimate_nm':y[j5_hold,4].tolist(),
            'J5_all_gravity_component_nm':pred_gravity[isj5,4].tolist(),
            'deployable':False}
        variants.append(result);predictions[label]=prediction
    j5=[p for p in poses if p['session']=='J5'];first,last=j5[0],j5[-1]
    t0,t1=first['effort_mean_nm'][4],last['effort_mean_nm'][4]
    qj=np.array([p['q_rad'][4] for p in j5]);tj=np.array([p['effort_mean_nm'][4] for p in j5])
    approach=np.array([p['planned_approach_sign'][4] for p in j5])
    local=[]
    for with_direction in (False,True):
        local_design=np.c_[np.sin(qj),np.cos(qj),np.ones(len(qj))]
        if with_direction:local_design=np.c_[local_design,approach]
        ti=np.array([i not in (3,4) for i in range(6)])
        par,_,rank,sig=np.linalg.lstsq(local_design[ti],tj[ti],rcond=1e-6)
        pred=local_design@par
        local.append({'model':'a*sin(q5)+b*cos(q5)+offset'+('+direction_nuisance' if with_direction else ''),
            'coefficients':par.tolist(),'rank':int(rank),'condition_number':float(sig[0]/sig[-1]),
            'heldout_pm20_rmse_nm':float(np.sqrt(np.mean((pred[~ti]-tj[~ti])**2))),
            'gravity_amplitude_nm':float(np.hypot(par[0],par[1])),
            'meaning':'local fixed-upstream empirical diagnostic; not spatial COM or a five-axis controller'})
    rigid=fit_rigid_variants(poses,x,base,y,train_rows,train,isj5,direction)
    report={'schema':'slaihand_five_dof_identification/1','source_commit':'5cebc3f',
        'status':'SPATIAL_J5_GEOMETRY_UNVERIFIED_AND_GRAVITY_FRICTION_NOT_SEPARATED',
        'deployable':False,'parameters_applied':False,'motor_mass_added_again':False,
        'convention':{'gravity_base_m_s2':legacy.G.tolist(),'fixed_J3_offset_rad':math.pi,
            'box_mass_kg':3.5,'mass_excludes_motor':True,'q5_input':'measured calibrated model joint angle',
            'box_COM_orbit_frame':'link4','physical_J5_pivot_link4_m':None,'physical_J5_axis_link4':None},
        'model_equations':{'orbit':'r4(q5)=b+u*cos(q5)+v*sin(q5)',
            'potential':'U=U_old-m*g.p4-g.R4*(h_b+h_u*cos(q5)+h_v*sin(q5))',
            'holding_torque':'tau_i=partial U/partial q_i, i=1..5',
            'physical_circle_constraints':'dot(u,v)=0 and norm(u)=norm(v); J5 pivot is still not separately known'},
        'accepted_pose_counts':{s:sum(p['session']==s for p in poses) for s in clients},
        'excluded':extracted['excluded'],
        'train_pose_ids':[p['id'] for p,k in zip(poses,train) if k],
        'holdout_pose_ids':[p['id'] for p,k in zip(poses,train) if not k],
        'J5_zero_pair':{'first_actual_deg':math.degrees(first['q_rad'][4]),
            'last_actual_deg':math.degrees(last['q_rad'][4]),'first_effort_estimate_nm':t0,
            'last_effort_estimate_nm':t1,'mean_nm':(t0+t1)/2,'half_difference_nm':(t1-t0)/2,
            'scope':'path-dependence diagnostic, not a measured Coulomb coefficient or exact paired gravity because actual poses differ'},
        'variants':variants,'rigid_circle_variants':rigid,'local_J5_diagnostics':local,
        'old_fixed_load_candidate_superseded_for_J5_motion':True,
        'no_hardware_actions':True}
    (HERE/'five_dof_result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    candidate={'schema':'slaihand_five_dof_offline_potential_candidates/1','deployable':False,
        'selected_for_deployment':None,'physical_J5_axis_measured':False,'physical_J5_pivot_measured':False,
        'coordinates':report['convention'],'models':rigid,
        'reason':'independent upstream error remains large; fitted axis changes strongly with nuisance assumption',
        'source_result_sha256':legacy.sha(HERE/'five_dof_result.json')}
    (HERE/'five_dof_offline_candidates.json').write_text(json.dumps(candidate,indent=2)+'\n',encoding='utf-8')
    make_plot(j5,report)
    print('Accepted:',report['accepted_pose_counts'],'excluded:',report['excluded'])
    print('J5 zero pair:',report['J5_zero_pair'])
    for v in variants:print(v['name'],'rank',v['rank'],'condition',v['condition_number'],'orbit',v['orbit_diagnostics'],'J5 heldout RMSE',v['J5_heldout_pm20_rmse_including_nuisance_nm'],'upstream holdout RMSE',v['upstream_independent_sessions']['rmse_nm'])
    print('Local diagnostics:',local)
    for v in rigid:print(v['name'],'COM_mm',np.array(v['COM_at_J5_zero_in_link4_m'])*1000,'axis',v['fitted_axis_direction_link4_unverified'],'J5hold',v['J5_pm20_heldout_rmse_total_nm'],'upstream hold',v['upstream_heldout_total_with_nuisance']['rmse_nm'])


def make_plot(j5,report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    q=np.degrees([p['q_rad'][4] for p in j5]);tau=np.array([p['effort_mean_nm'][4] for p in j5])
    fig,ax=plt.subplots(1,2,figsize=(12,4.8),layout='constrained')
    ax[0].plot(q,tau,'o-',label='Recorded sequence')
    for k,(x,y) in enumerate(zip(q,tau)):ax[0].annotate(str(k+1),(x,y),xytext=(5,5),textcoords='offset points')
    ax[0].set(xlabel='Actual J5 angle [deg]',ylabel='J5 torque estimate [Nm]',title='Same zero target, different approach history')
    ax[0].grid(alpha=.25)
    for variant in report['rigid_circle_variants'][:2]:
        ax[1].plot(range(6),variant['J5_all_gravity_component_nm'],'o--',label=variant['name'].replace('rigid_circle_','gravity: '))
    ax[1].plot(range(6),tau,'ks-',label='Recorded torque estimate')
    ax[1].set(xlabel='Window index',ylabel='Torque [Nm]',title='Five-DOF gravity component vs recorded estimate');ax[1].grid(alpha=.25);ax[1].legend(fontsize=8)
    fig.suptitle('J5 output-load identification: offline only, not deployable',fontsize=14)
    fig.savefig(HERE/'J5_identification_diagnostics.png',dpi=170);plt.close(fig)


if __name__=='__main__':main()
