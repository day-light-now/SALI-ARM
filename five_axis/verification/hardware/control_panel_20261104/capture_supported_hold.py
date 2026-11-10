"""Operator entry: existing finite current-pose hold client plus read-only capture.

Default is preview only. Execution needs the same explicit acknowledgements as
the existing protected client. No trajectories, automatic reset or gain changes.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

import calibration_core as core
from calibration_capture import (Recorder, _local_identity, provenance, collect_static,
                                 _open_raw, _model_fn)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('execute','ack-supported','ack-clearance','ack-native-output'):
        parser.add_argument('--'+key,action='store_true')
    parser.add_argument('--support',choices=core.SUPPORTS,default='unknown')
    parser.add_argument('--contact',choices=core.CONTACTS,default='none')
    parser.add_argument('--target-deg',type=float,nargs=4,metavar=('J1','J2','J3','J4'),
                        help='Optional absolute pose: existing protected plan, 10s move + 6s dwell')
    parser.add_argument('--out',type=Path,default=Path.home()/'.ros/slaihand/calibration')
    args=parser.parse_args()
    if args.execute and not all((args.ack_supported,args.ack_clearance,args.ack_native_output)):
        parser.error('执行需现有客户端的三项现场确认')
    if args.execute and (args.support!='free' or args.contact!='none'):
        parser.error('本入口只采自由、无接触的保持样本；其他情况用只读采集工具明确标记')
    import rclpy
    from ament_index_python.packages import get_package_prefix
    rclpy.init(args=None)
    recorder=Recorder();recorder.local_identity=_local_identity('no_end_load')
    child=None
    try:
        deadline=time.monotonic()+6
        sample=None
        while time.monotonic()<deadline:
            sample=recorder.snapshot(.25)
            if sample and sample['runtime'].get('verified') and sample.get('driver_ff') \
                    and sample['driver_ff'].get('age_s',99)<=core.FF_MAX_AGE_S:
                break
        if not sample or not sample['runtime'].get('verified'):
            raise RuntimeError('没有新鲜四轴反馈或运行身份不匹配')
        if sample['stage']!=0 or sample['motor_state']!=[0]*4:
            raise RuntimeError('需要未使能状态；不自动复位或重试使能')
        if not sample.get('driver_ff') or sample['driver_ff'].get('alpha')!=0:
            raise RuntimeError('基线采集要求驱动alpha=0')
        print('当前角度°：'+str([round(math.degrees(q),3) for q in sample['positions_rad']]),flush=True)
        plan_path=None
        if args.target_deg is not None:
            import xml.etree.ElementTree as ET
            from calibration_capture import URDF
            from joint_waypoint_plan import compile_plan
            robot=ET.parse(URDF).getroot()
            bounds=[tuple(float(robot.find(f"joint[@name='slaihand_joint{i}']/limit").get(k))
                          for k in ('lower','upper')) for i in range(1,5)]
            plan={'coordinates':'absolute_deg','waypoints':[{'degrees':args.target_deg,'move_s':10.,'dwell_s':6.}]}
            compile_plan(plan,sample['positions_rad'],bounds)
            print('运行身份匹配；绝对目标°：'+str(args.target_deg)+'，10秒移动+6秒停留，采样后失能。',flush=True)
        else:
            print('运行身份匹配；计划：当前位置保持10秒，静止窗口采一条，结束失能。',flush=True)
        if not args.execute:
            print('仅预览，未请求使能。');return 0
        model=provenance('no_end_load')
        client=Path(get_package_prefix('slaihand_bringup'))/'lib/slaihand_bringup/commission_supported_trial.py'
        client_out=args.out/'hold_client';client_out.mkdir(parents=True,exist_ok=True)
        argv=[sys.executable,str(client),'--execute','--ack-supported',
            '--ack-clearance','--ack-native-output','--hold-seconds',
            '1' if args.target_deg is not None else '10','--output-directory',str(client_out)]
        if args.target_deg is not None:
            import tempfile
            with tempfile.NamedTemporaryFile(mode='w',prefix='capture_pose_',suffix='.json',
                                             dir=client_out,delete=False) as stream:
                json.dump(plan,stream);plan_path=Path(stream.name)
            argv+=['--plan',str(plan_path)]
        child=subprocess.Popen(argv)
        deadline=time.monotonic()+(24 if args.target_deg is not None else 12)
        ready=False
        while time.monotonic()<deadline and child.poll() is None:
            item=recorder.snapshot(.15)
            axes=(recorder.dynamic or {}).get('axes',{})
            ff=(item or {}).get('driver_ff') or {}
            if args.target_deg is None:
                ready=bool(item and item['stage']==5 and ff.get('stage')==5 and ff.get('alpha')==0
                    and all(axis.get('hold_ready')==1 for axis in axes.values()))
            else:
                ready=bool(item and item['stage']==6 and ff.get('stage')==6 and ff.get('alpha')==0
                    and recorder.reference is not None
                    and max(abs(a-math.radians(b)) for a,b in zip(recorder.reference,args.target_deg))<1e-6
                    and max(abs(a-math.radians(b)) for a,b in zip(item['positions_rad'],args.target_deg))<=math.radians(.5))
            if ready:break
        if not ready:raise RuntimeError('未观测稳定保持，停止采集')
        aggregate=collect_static(recorder,2.,10,'auto',4.)
        if aggregate is None:raise RuntimeError('静止窗口未通过，未生成合格记录')
        record={'schema':core.SCHEMA_SAMPLE,'recorded_at':time.time(),
                'support':'free','contact':'none','motion':'static','load_id':'no_end_load',
                'model':model,'note':'user-executed finite protected hold/pose, alpha=0',
                'requested_target_deg':args.target_deg,**aggregate}
        ok,reasons=core.validate_sample(record)
        excluded=core.exclusion_reason(record) if ok else None
        if not ok or excluded:raise RuntimeError('样本不合格：'+str(reasons or excluded))
        rc=child.wait(timeout=16)
        if rc!=0:raise RuntimeError('保持客户端失败，退出码'+str(rc))
        path,stream=_open_raw(args.out,'current_hold')
        try:
            stream.write(json.dumps({'schema':core.SCHEMA_CAPTURE,'tool':Path(__file__).name,
                'recorded_at':time.time(),'model':model,'load_id':'no_end_load',
                'runtime':record['runtime'],'requested_count':1},ensure_ascii=False)+'\n')
            stream.write(json.dumps(record,ensure_ascii=False)+'\n');stream.flush()
            __import__('os').fsync(stream.fileno())
        finally:
            stream.close();path.chmod(0o444)
        gravity=_model_fn()(record['positions_rad'])
        print(json.dumps({'raw_file':str(path),'feedback_nm':record['effort_estimate_nm'],
            'model_gravity_nm':gravity,'stage_during_capture':record['stage'],
            'samples':record['static_window']['samples'],'client_exit':rc},ensure_ascii=False),flush=True)
        return 0
    finally:
        # The existing client owns all motor actions and its stop/confirmation.
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=12)
            except subprocess.TimeoutExpired:
                print('客户端尚未退出：请现场检查停止反馈，勿重新使能。',file=sys.stderr)
        recorder.close();rclpy.shutdown()


if __name__=='__main__':
    try:sys.exit(main())
    except (RuntimeError,subprocess.TimeoutExpired) as error:
        print('采集停止：'+str(error),file=sys.stderr);sys.exit(1)
