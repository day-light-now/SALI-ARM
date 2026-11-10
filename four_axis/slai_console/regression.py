"""Deterministic, physics-only acceptance baseline. Never opens a serial port.

Run from workspace: python -m slai_console.regression --output tmp/console_release
The historical 0.073 deg UI event is a reference observation, not a tolerance.
"""
import argparse
import csv
import json
from pathlib import Path
from unittest.mock import patch
import numpy as np
from .core import CommandGateway, Context, PreviewSlider, ROOT


def run(output):
    from .adapters import SimAdapter
    output = Path(output)
    output.mkdir(parents=True,exist_ok=True)
    a = SimAdapter()
    records, stages = [], []
    preview = PreviewSlider()
    events = []
    gateway = CommandGateway(preview,lambda *args,**kw:events.append(dict(level=args[0],message=args[1],**kw)))
    ctx = Context(mode='sim',connected=True,fresh=True,enabled=True)

    def sample(stage):
        records.append([stage,float(a.sim.data.time),*np.rad2deg(a.sim.data.qpos).tolist(),
                        *np.rad2deg(a.sim.target).tolist(),*a.sim.gravity_torque().tolist()])

    def advance(stage,steps):
        for _ in range(steps//20):
            a.step(20)
            sample(stage)

    def require(condition, message):
        if not condition:
            raise AssertionError(message)

    def settle(stage):
        for _ in range(1000):  # <=20 s simulation, including the planned movement
            a.step(20)
            sample(stage)
            error = float(np.max(np.abs(np.rad2deg(a.sim.data.qpos-a.sim.target))))
            if not a.busy and error<.3:
                return error
        raise AssertionError(stage+' did not reach 0.3 deg tolerance')

    try:
        zero_error=float(np.max(np.abs(a.sim.data.qpos)))
        require(zero_error<=1e-12,'Startup is not confirmed model zero')
        sample('zero')
        stages.append(dict(stage='zero',max_abs_q_rad=zero_error,criterion='<=1e-12 rad',passed=True))

        ticket=preview.edit(2,45.)
        require(np.array_equal(a.sim.target,np.zeros(4)),'Preview changed commanded target')
        require(np.array_equal(a.sim.data.qpos,np.zeros(4)),'Preview moved physics')
        a.motion(gateway.commit(ticket,ctx),ticket.duration)
        single_error=settle('single_j2_45')
        stages.append(dict(stage='single_j2_45',actual_deg=np.rad2deg(a.sim.data.qpos).tolist(),
            first_completion_error_deg=single_error,criterion='all axes <0.3 deg after 3 s planned move',
            historical_ui_reference_deg=.073,historical_reference_is_tolerance=False,passed=True))

        ticket=preview.trajectory([[10.,30.,5.,-10.],[0.,0.,0.,0.]],3.)
        gateway.commit(ticket,ctx)
        a.trajectory(ticket.plan,ticket.duration)
        trajectory_error=settle('trajectory_two_waypoints')
        require(len(events[-1]['waypoints'])==2,'Audit does not bind complete trajectory')
        stages.append(dict(stage='trajectory_two_waypoints',waypoints=ticket.plan,
            duration_per_segment_s=ticket.duration,completion_error_deg=trajectory_error,
            criterion='all axes <0.3 deg at final zero, complete plan in audit',passed=True))

        a.stop()
        a.sim.enter_demo()  # documented comparison pose; does not redefine model zero
        a.sim.set_mode('float')
        start=a.sim.data.qpos.copy()
        advance('gravity_float',2000)
        drift=float(np.max(np.abs(a.sim.data.qpos-start)))
        require(drift<1e-6,'Gravity float drifts in the current provisional model')
        a.stop()
        a.sim.push()
        advance('spring_push',400)
        disturbed=float(np.linalg.norm(a.sim.data.qpos-start))
        advance('spring_recovery',5000)
        recovered=float(np.linalg.norm(a.sim.data.qpos-start))
        require(disturbed>.01 and recovered<disturbed*.15,'Spring disturbance/recovery changed')
        stages.append(dict(stage='dynamics',comparison_pose_deg=np.rad2deg(start).tolist(),
            float_drift_rad=drift,disturbance_norm_rad=disturbed,recovery_norm_rad=recovered,
            criterion='float drift <1e-6 rad / 2 s; 8 N push for .4 s; 5 s recovery <15% disturbance',passed=True))
        result=dict(schema=1,simulation_only=True,hardware_opened=False,
                    integrator_step_s=.001,sample_interval_s=.020,stages=stages,events=events)
        (output/'regression.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        with (output/'regression.csv').open('w',encoding='utf-8-sig',newline='') as f:
            writer=csv.writer(f)
            writer.writerow(['stage','sim_time_s',*[f'actual_J{i}_deg' for i in range(1,5)],
                             *[f'target_J{i}_deg' for i in range(1,5)],*[f'gravity_J{i}_Nm' for i in range(1,5)]])
            writer.writerows(records)
        return result
    finally:
        a.stop()
        a.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'tmp/console_release')
    args=parser.parse_args()
    from .provenance import fingerprint, now, sha, write_json
    before=fingerprint()
    with patch('serial.Serial',side_effect=AssertionError('Offline baseline forbids real serial')):
        result=run(args.output)
    if fingerprint()!=before:
        raise RuntimeError('Runtime source/model/environment changed during baseline')
    write_json(args.output/'regression_provenance.json',dict(at=now(),runtime_fingerprint=before,
        inputs_unchanged=True,real_serial_forbidden=True,
        results={name:sha(args.output/name) for name in ('regression.json','regression.csv')}))
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
