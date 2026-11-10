#!/usr/bin/env python3
"""ROS-only operator client; default preview. Never opens a serial device."""
import argparse
import hashlib
import json
import math
import re
from pathlib import Path
import signal
import time
import uuid
import xml.etree.ElementTree as ET
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from control_msgs.msg import DynamicJointState,JointTrajectoryControllerState,JointTolerance
from joint_waypoint_plan import compile_plan,assess_waypoints,REFERENCE_SPEED_RAD_S,joint_names
from drag_teaching import (load_playback,build_trajectory,tracking_error_deg,identity_fingerprint,
                           configure_axes)
from control_panel_model import parse_gravity_ff_state
import supported_capture as capture_flow
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import Trigger
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from trajectory_msgs.msg import JointTrajectoryPoint


def mark_unknown(result, observed_status):
    """Preserve UNKNOWN even when it first appears during the stop wait."""
    try:
        if any(v.get('control_state') == 9 for v in (observed_status or {}).values()):
            result['unknown_observed'] = True
            return True
    except (AttributeError, TypeError):
        return False
    return False


class PreArmError(RuntimeError):
    """A refused enable request; no arm or trajectory was requested."""

    def __init__(self, diagnostic, timeout=False):
        self.diagnostic = diagnostic
        self.timeout = bool(timeout)
        self.reasons = list(diagnostic.get("reasons") or [])
        self.observations = dict(diagnostic.get("observations") or {})
        super().__init__("pre-arm refused: " + "；".join(self.reasons))

def load_axis_bounds(joint_count, urdf_path, j5_limits_deg=None):
    """Canonical J1-J4 URDF bounds plus an explicit J5 interval; never inferred."""
    if joint_count not in (4,5):
        raise ValueError('只支持4轴或5轴，收到 %r' % (joint_count,))
    if joint_count==4 and j5_limits_deg is not None:
        raise ValueError('四轴不接受 --j5-limits-deg；J5限位只属于五轴')
    root=ET.parse(Path(urdf_path)).getroot()
    bounds=[]
    for name in joint_names(4):
        limit=root.find("joint[@name='%s']/limit" % name)
        if limit is None:raise ValueError('URDF 缺少 %s 的限位' % name)
        bounds.append(tuple(float(limit.get(key)) for key in ('lower','upper')))
    if joint_count==4:return bounds
    if j5_limits_deg is None:
        raise ValueError('五轴需要显式 --j5-limits-deg LOWER UPPER（J5有限限位来自已标定硬件映射）')
    lower,upper=(float(value) for value in j5_limits_deg)
    if not (math.isfinite(lower) and math.isfinite(upper)) or not lower<upper:
        raise ValueError('--j5-limits-deg 需要有限且 LOWER<UPPER 的 J5 限位（度）')
    return bounds+[(math.radians(lower),math.radians(upper))]


class _CaptureDone(Exception):
    """当前位置静态采样已完成（停止与落盘在分支内完成，跳过通用运动路径）。"""


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('execute','ack-supported','ack-clearance','ack-native-output','round-trip'):p.add_argument('--'+name,action='store_true')
    p.add_argument('--gravity-float',action='store_true',help='独立零刚度重力补偿，持续1～120秒')
    p.add_argument('--float-kd',type=float,default=5.,help='自由拖动阻尼，0～5；仅自由拖动模式生效')
    p.add_argument('--ack-gravity-float',action='store_true',help='确认托住机械臂等待补偿渐入，结束失能可能下落')
    p.add_argument('--joint-count',type=int,choices=(4,5),default=4,
                   help='轴数：4保持既有J1～J4路径（默认），5使用J1～J5')
    p.add_argument('--j5-limits-deg',type=float,nargs=2,metavar=('LOWER','UPPER'),
                   help='五轴必须显式给出的J5有限限位（度）；四轴不接受')
    p.add_argument('--plan',type=Path,help='显式JSON绝对角度/相对起点路点；轴数须与--joint-count一致；默认只读预览')
    p.add_argument('--point-to-point',action='store_true',help='官方JTC自适应时间缩放与终点稳定验收')
    p.add_argument('--calibrate', action='store_true', help='路点保持期间同步采集；不改alpha或质量档案')
    p.add_argument('--ack-free-static', action='store_true', help='显式确认采样时自由承重、无外部接触')
    p.add_argument('--load-id', default='no_end_load', help='标定档案 load_id；新负载必须显式传入')
    p.add_argument('--calibration-group', default=None, help='面板选中的标定组；仅审计')
    p.add_argument('--model-sha256', default=None, help='资格组要求的 URDF/model 指纹')
    p.add_argument('--mapping-sha256', default=None, help='资格组要求的硬件映射指纹')
    p.add_argument('--j5-geometry-sha256', default=None, help='资格组要求的 J5 几何指纹')
    p.add_argument('--delta-deg',type=float,default=1.)
    p.add_argument('--leg-seconds',type=float,default=4.)
    p.add_argument('--hold-seconds',type=float,default=5.)
    p.add_argument('--output-directory',type=Path,default=None)
    p.add_argument('--teaching-plan',type=Path,help='拖动示教回放冻结规格；使用正常位置控制')
    p.add_argument('--supported-static-capture',action='store_true',
                   help='当前位置普通保持：操作员撤手确认后采集一条静态样本')
    p.add_argument('--ack-file',type=Path,help='面板写入的一次性撤手确认文件')
    p.add_argument('--ack-timeout',type=float,default=capture_flow.ACK_TIMEOUT_S)
    p.add_argument('--hold-ready-timeout',type=float,default=capture_flow.HOLD_READY_TIMEOUT_S)
    p.add_argument('--capture-seconds',type=float,default=capture_flow.CAPTURE_SECONDS)
    p.add_argument('--capture-samples',type=int,default=capture_flow.CAPTURE_SAMPLES)
    p.add_argument('--capture-output',type=Path)
    p.add_argument('--panel-stop-epoch',type=int,default=-1)
    p.add_argument('--operation-id',type=str,default='')
    p.add_argument('--expected-identity-sha256',type=str,default=None,
                   help='预览票根冻结的运行身份；arm 前必须一致，否则不使能')
    p.add_argument('--support',choices=('free',),default='free',
                   help='操作员声明的支撑状态；网页静态采样只接受自由')
    p.add_argument('--contact',choices=('none',),default='none',
                   help='操作员声明的接触状态；网页静态采样只接受无接触')
    a=p.parse_args()
    if a.joint_count==4 and a.j5_limits_deg is not None:
        p.error('--j5-limits-deg 只用于五轴；四轴使用URDF J1～J4限位')
    if a.joint_count==5 and a.j5_limits_deg is None:
        p.error('五轴需要显式 --j5-limits-deg LOWER UPPER；J5有限限位不做推断')
    if a.expected_identity_sha256 is not None and not re.fullmatch(r'[0-9a-f]{64}', a.expected_identity_sha256):
        p.error('--expected-identity-sha256 必须是64位小写十六进制')
    if a.joint_count==5 and (a.calibrate or a.supported_static_capture):
        p.error('五轴静态采样/标定需要轴数自适应的标定模块（尚未接入）；'
                '五轴只开放计划、保持、往返、自由拖动与示教回放')
    if a.execute and not all((a.ack_supported,a.ack_clearance,a.ack_native_output)):
        p.error('执行需确认可靠支撑、完整路径净空/可切电、原生MIT估计门禁不是驱动硬限流')
    if a.plan and a.round_trip:p.error('--plan 与 --round-trip 互斥')
    if a.calibrate and (not a.plan or a.point_to_point or a.round_trip):
        p.error('重力采样需要独立的 --plan，不与其他模式混用')
    if a.calibrate and a.execute and not a.ack_free_static:
        p.error('重力采样需显式确认 --ack-free-static；防坠支撑不得承重')
    if a.gravity_float and (a.plan or a.round_trip or a.point_to_point or a.calibrate):
        p.error('自由拖动只允许独立的有限体验')
    if not math.isfinite(a.float_kd) or not 0<=a.float_kd<=5:
        p.error('自由拖动 Kd 需要在0～5之间')
    if a.gravity_float and a.execute and not a.ack_gravity_float:
        p.error('自由拖动需要 --ack-gravity-float')
    if a.teaching_plan and (a.plan or a.gravity_float or a.round_trip or a.point_to_point or a.calibrate):
        p.error('拖动示教回放不能与其他执行模式组合')
    if a.supported_static_capture and (a.plan or a.gravity_float or a.round_trip or a.point_to_point
                                       or a.calibrate or a.teaching_plan):
        p.error('当前位置静态采样不能与其他执行模式组合')
    if a.supported_static_capture and a.execute:
        if a.ack_file is None or a.capture_output is None or not a.operation_id:
            p.error('静态采样执行需要 --ack-file、--capture-output、--operation-id')
        if not (0<a.ack_timeout<=120) or not (0<a.hold_ready_timeout<=30):
            p.error('静态采样超时参数超出允许范围')
        if not (1<=a.capture_seconds<=capture_flow.CAPTURE_MAX_SECONDS) or \
                not (capture_flow.CAPTURE_MIN_SAMPLES<=a.capture_samples<=capture_flow.CAPTURE_MAX_SAMPLES):
            p.error('静态窗口参数超出允许范围')
        if a.panel_stop_epoch<0:
            p.error('静态采样需要有效的 panel-stop-epoch')
    configure_axes(a.joint_count)
    JOINT_COUNT=a.joint_count
    NAMES=list(joint_names(JOINT_COUNT))
    AXIS_LABEL='五轴' if JOINT_COUNT==5 else '四轴'
    teaching=load_playback(a.teaching_plan) if a.teaching_plan else None
    if teaching is not None and not a.execute:
        print('拖动示教回放规格有效，但未加 --execute；不请求使能。',flush=True)
        return
    document=json.loads(a.plan.read_text(encoding='utf-8-sig')) if a.plan else None
    if a.point_to_point and document is None:p.error('--point-to-point 需要 --plan')
    if a.point_to_point:
        from point_to_point import effective_document,EXTRA_SECONDS
        document=effective_document(document)
    if a.calibrate:
        from calibration_session import effective_document as calibration_document
        document=calibration_document(document)
    if a.round_trip and not a.execute:p.error('--round-trip 需要 --execute；无参数只预览')
    if not all(math.isfinite(v) for v in (a.delta_deg,a.leg_seconds,a.hold_seconds)) or not 0<a.hold_seconds<=(120 if a.gravity_float else 10) or a.leg_seconds<=0:
        p.error('需要有限、有效的试验参数')
    if a.gravity_float and a.hold_seconds<1:p.error('自由拖动持续时间至少1秒')
    if a.round_trip and (abs(a.delta_deg)<1e-9 or 1.875*abs(math.radians(a.delta_deg))/a.leg_seconds>REFERENCE_SPEED_RAD_S):
        p.error(f'参考峰值速度超过{REFERENCE_SPEED_RAD_S:g}rad/s，请增加每程时间')
    if a.hold_seconds+(2*a.leg_seconds if a.round_trip else 0)+8>178:p.error('试验及余量须落在180秒会话内')
    try:
        description=Path(get_package_share_directory('slaihand_description'))/'urdf/slaihand_four_axis.urdf'
        bounds=load_axis_bounds(JOINT_COUNT,description,a.j5_limits_deg)
    except (LookupError,OSError,ET.ParseError,TypeError,ValueError) as error:
        p.error('轴限位不可用：%s' % error)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,signal.default_int_handler)
    def terminate(signum,frame):raise KeyboardInterrupt('signal '+str(signum))
    signal.signal(signal.SIGTERM,terminate)
    node=rclpy.create_node('supported_trial_'+uuid.uuid4().hex[:8]);seen={};position_samples=[]
    capture=None;recorder=None
    if a.calibrate:
        from calibration_capture import Recorder, _local_identity
        recorder=Recorder(node=node);recorder.local_identity=_local_identity(a.load_id)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    def joint_message(m):
        seen['joints']=m
        row=dict(zip(m.name,m.position))
        if all(n in row and math.isfinite(row[n]) for n in NAMES):position_samples.append((time.monotonic(),[row[n] for n in NAMES]))
    node.create_subscription(JointState,'/slaihand/joint_states',joint_message,qos)
    node.create_subscription(DynamicJointState,'/slaihand/dynamic_joint_states',lambda m:seen.update(dynamic=m,at=time.monotonic()),qos)
    node.create_subscription(JointTrajectoryControllerState,'/slaihand/slaihand_joint_trajectory_controller/controller_state',lambda m:seen.update(controller=m,controller_at=time.monotonic()),qos)
    def identity_message(m):
        try: seen['identity']=json.loads(m.data)
        except (TypeError,ValueError): pass
    identity_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String,'/slaihand/runtime_identity',identity_message,identity_qos)
    arm=node.create_client(SetParametersAtomically,'/slaihand/arm_gravity_float_configured') if a.gravity_float else node.create_client(Trigger,'/slaihand/arm_supported_current_pose')
    node.create_subscription(Float64MultiArray,'/slaihand/gravity_ff_state',
        lambda m:seen.update(float_state=list(m.data),float_state_at=time.monotonic(),
                             float_state_layout=getattr(m,'layout',None)),qos)
    stop=node.create_client(Trigger,'/slaihand/stop_all')
    switch=node.create_client(SwitchController,'/slaihand/controller_manager/switch_controller')
    def status():
        m=seen.get('dynamic');return {} if m is None else {n:dict(zip(v.interface_names,v.values)) for n,v in zip(m.joint_names,m.interface_values)}
    def matches(key,value):return all(status().get(n,{}).get(key)==value for n in NAMES)
    def wait(predicate,seconds,monitor_control=False):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if capture is not None:
                capture.observe(time.monotonic())
            if predicate():return
            if monitor_control and any(v.get('control_state') in (8,9) for v in status().values()):
                raise RuntimeError('控制端已全局停止或UNKNOWN；检查失能反馈，状态未知时保持支撑并现场切电')
        raise RuntimeError('等待超时，当前状态：'+str(status()))
    def call(client,request,seconds=4):
        if not client.wait_for_service(timeout_sec=seconds):raise RuntimeError('ROS服务不可用')
        future=client.call_async(request);wait(future.done,seconds);r=future.result()
        if hasattr(r,'success') and not r.success:raise RuntimeError(r.message)
        if hasattr(r,'result') and not r.result.successful:raise RuntimeError(r.result.reason)
        return r
    def switch_controller(activate):
        r=SwitchController.Request();r.strictness=2;r.activate_asap=True;r.timeout.sec=3
        if activate:r.activate_controllers=['slaihand_joint_trajectory_controller']
        else:r.deactivate_controllers=['slaihand_joint_trajectory_controller']
        if not call(switch,r).ok:raise RuntimeError('标准轨迹控制器切换失败')
    attempted=False;controller=False;scaler=None
    result=dict(request_id=uuid.uuid4().hex,mode='preview',joint_count=JOINT_COUNT,
                physical_torque_off_verified=False,
                driver_current_clamp_verified=False,hardware_motion_accepted=False,
                stop_observations_confirmed=False,phase='pre_arm',arm_requested=False,
                trajectory_requested=False,trajectory_started=False,stop_requested=False,
                unknown_observed=False,prearm_timeout=False,prearm_reasons=[],prearm_observations={},
                load_id=getattr(a,'load_id','no_end_load'),
                calibration_group=getattr(a,'calibration_group',None),
                expected_identity={key:value for key,value in
                    (('load_id',a.load_id),('model_sha256',a.model_sha256),
                     ('mapping_sha256',a.mapping_sha256),
                     ('j5_geometry_sha256',a.j5_geometry_sha256)) if value})
    result['operator_acknowledgements']=dict(supported=a.ack_supported,clearance_and_local_power_cut=a.ack_clearance,native_output_not_hardware_clamp=a.ack_native_output)
    result['trial_parameters']=dict(hold_seconds=a.hold_seconds,round_trip=a.round_trip,delta_deg=a.delta_deg,leg_seconds=a.leg_seconds)
    if a.supported_static_capture:
        result['trial_parameters']=dict(ack_timeout_s=a.ack_timeout,capture_seconds=a.capture_seconds,
            capture_samples=a.capture_samples,operation_id=a.operation_id,
            panel_stop_epoch=a.panel_stop_epoch,support=a.support,contact=a.contact)
    try:
        wait(lambda:'joints' in seen and matches('feedback_valid',1),5)
        initial=dict(zip(seen['joints'].name,seen['joints'].position))
        result['initial_model_rad']=[initial[n] for n in NAMES]
        print('当前角度°：'+str([round(math.degrees(initial[n]),3) for n in NAMES]),flush=True)
        print(('完整URDF范围°：' if JOINT_COUNT==4 else '完整限位范围°（J1～J4 URDF + J5显式）：')
              +str([[round(math.degrees(q),3) for q in b] for b in bounds]),flush=True)
        if a.supported_static_capture:
            deadline=time.monotonic()+3
            while time.monotonic()<deadline and 'float_state' not in seen:
                rclpy.spin_once(node,timeout_sec=.05)
            observed_alpha=(seen.get('float_state') or [None])[0]
            baseline=capture_flow.fit_baseline(observed_alpha)
            print('当前位置静态采样预览：角度°=%s；α=%s（%s）；撤手等待上限 %.0f 秒；静态窗口 %.1f 秒/%d 帧'
                  %([round(math.degrees(initial[n]),3) for n in NAMES],baseline['alpha'],
                    '基线满足' if baseline['satisfied'] else '仅诊断',
                    a.ack_timeout,a.capture_seconds,a.capture_samples),flush=True)
            print('步骤：结束自由拖动→确认失能→软件复位→开始位置保持→撤手确认→静态采样。',flush=True)
            result['baseline']=baseline
        if teaching is not None:
            teaching_points,teaching_meta=build_trajectory(teaching,[initial[n] for n in NAMES],bounds)
            if a.hold_seconds+teaching_meta['retimed_duration_s']+8>178:
                raise ValueError('拖动示教回放、保持与余量须落在178秒会话内')
            print('拖动示教回放预览（执行时按实际起点重算接近段）：'+json.dumps(
                {k:teaching_meta[k] for k in ('speed_factor','approach_s','original_duration_s',
                 'retimed_duration_s','peak_speed_rad_s','ranges_deg','samples',
                 'interp_error_bound_deg')},ensure_ascii=False),flush=True)
            result['teaching_playback']=teaching_meta
            result['speed_factor']=teaching_meta['speed_factor']
        if document is not None:
            preview,windows=compile_plan(document,[initial[n] for n in NAMES],bounds)
            if a.hold_seconds+preview[-1][0]+8+(EXTRA_SECONDS if a.point_to_point else 0)>178:raise ValueError('计划、保持、自适应时间余量须落在178秒预算内')
            print('路点预览（软件绝对角度°）：'+json.dumps([dict(at_s=t,degrees=[round(math.degrees(q),3) for q in qs]) for t,qs in preview],ensure_ascii=False),flush=True)
            result['plan']=document
            result['plan_sha256']=hashlib.sha256(a.plan.read_bytes()).hexdigest()
        if not a.execute:print('仅预览，没有请求使能或轨迹。');return
        if recorder is not None:
            from calibration_capture import prearm_diagnostic
            result['phase']='pre_arm'
            result['arm_requested']=False
            result['trajectory_requested']=False
            result['trajectory_started']=False
            result['stop_requested']=False
            diagnostic={"reasons":["SAMPLE_UNAVAILABLE: 尚未收到预使能样本"],"observations":{}}
            deadline=time.monotonic()+5
            while True:
                rclpy.spin_once(node,timeout_sec=.05)
                diagnostic=prearm_diagnostic(
                    recorder._build(),
                    expected_identity={"load_id": a.load_id,
                                       "model_sha256": a.model_sha256,
                                       "mapping_sha256": a.mapping_sha256,
                                       "j5_geometry_sha256": a.j5_geometry_sha256})
                reasons=diagnostic['reasons']
                if not reasons:break
                # A known non-zero compensation baseline is not a transient wait.
                if any(reason.startswith('ALPHA_NONZERO:') for reason in reasons):
                    result['prearm_timeout']=False
                    raise PreArmError(diagnostic,timeout=False)
                if time.monotonic()>=deadline:
                    result['prearm_timeout']=True
                    raise PreArmError(diagnostic,timeout=True)
            result['prearm_diagnostics']=diagnostic
            print('预使能资格观察：'+json.dumps(diagnostic,ensure_ascii=False),flush=True)
        if not matches('control_state',0):raise RuntimeError('需要B驱动DISARMED；不能从停止/UNKNOWN状态直接使能')
        if a.round_trip and any(not bounds[i][0]<=initial[n]+math.radians(a.delta_deg)<=bounds[i][1] for i,n in enumerate(NAMES)):
            raise RuntimeError('预览目标超出完整限位范围，不裁剪')
        expected_identity_sha=a.expected_identity_sha256
        if teaching is not None:
            expected_identity_sha=expected_identity_sha or teaching['identity_sha256']
        if expected_identity_sha is not None:
            # Re-check the runtime identity before any arm/enable request: a
            # mapping/policy/URDF/boot change after preview must not be executed.
            deadline=time.monotonic()+3
            while time.monotonic()<deadline and 'identity' not in seen:
                rclpy.spin_once(node,timeout_sec=.05)
            observed=seen.get('identity')
            try: actual=identity_fingerprint(observed) if observed else None
            except Exception: actual=None
            if actual!=expected_identity_sha:
                raise RuntimeError('驱动运行身份与预览冻结不一致（%s != %s），拒绝使能；请重新预览'
                    % (actual,expected_identity_sha))
            result['runtime_identity_sha256']=actual
        if a.supported_static_capture:
            from calibration_capture import (Recorder, _local_identity, collect_static,
                                             provenance, _open_raw, _model_fn, _sha256)
            import calibration_core as calibration_core
            result['mode']='supported_static_capture'
            recorder=Recorder(node=node);recorder.local_identity=_local_identity(a.load_id)
            deadline=time.monotonic()+5
            ready=False
            while time.monotonic()<deadline:
                preview_sample=recorder.snapshot(.2)
                if preview_sample and preview_sample['runtime'].get('verified') \
                        and preview_sample['stage']==0 and preview_sample['motor_state']==[0]*JOINT_COUNT:
                    ready=True;break
            if not ready:raise RuntimeError('运行身份/未使能反馈未就绪，拒绝位置保持')
            attempted=True;result.update(phase='arm_requested',arm_requested=True)
            receipt=call(arm,Trigger.Request());result['backend_arm_receipt']=receipt.message
            print(receipt.message,flush=True)
            wait(lambda:matches('control_state',5) and matches('hold_ready',1),a.hold_ready_timeout,True)
            current=status()
            result['command_id']=int(current[NAMES[0]]['command_id'])
            result['hold_start_rad']=[float(current[n]['position']) for n in NAMES]
            predicted_identity=seen.get('identity') or {}
            try: held_identity_sha=identity_fingerprint(predicted_identity)
            except Exception: raise RuntimeError('缺少驱动运行身份，无法确认撤手归属')
            held_boot=predicted_identity.get('boot_id')
            result['runtime_identity_sha256']=held_identity_sha
            result['boot_id']=held_boot
            def ff_snapshot():
                row=seen.get('float_state') or []
                fresh=time.monotonic()-seen.get('float_state_at',0)<=1.5
                if not row or not fresh:return None
                return {'alpha':row[0],'ramp':row[1],'active':row[2]>=.5,'stage':int(row[3]),
                        'command_id':int(row[4]),
                        'gravity_float':(row[21]>=.5 if len(row)>21 else None)}
            result['position_mode_before']=ff_snapshot()
            result['hold_ready_at']=time.time()
            print('SUPPORTED_HOLD_READY command_id=%d boot=%s identity=%s'
                  %(result['command_id'],held_boot,held_identity_sha),flush=True)
            print('位置保持已就绪，托稳后缓慢撤手。',flush=True)
            ack=None;deadline=time.monotonic()+a.ack_timeout
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.02)
                if not matches('control_state',5):
                    raise RuntimeError('保持中止，未收到撤手确认；查看驱动审计')
                if a.ack_file.is_file():
                    payload=capture_flow.read_ack_file(a.ack_file)
                    ack=capture_flow.validate_ack(payload,{
                        'operation_id':a.operation_id,'command_id':result['command_id'],
                        'boot_id':held_boot,'identity_sha256':held_identity_sha,
                        'panel_stop_generation':int(a.panel_stop_epoch)})
                    break
            if ack is None:raise RuntimeError('等待撤手确认超时，停止采集')
            result['operator_ack']={key:ack[key] for key in
                ('schema','operation_id','command_id','boot_id','identity_sha256',
                 'panel_stop_generation','nonce','requested_at','operator_disengaged')}
            result['position_mode_after']=ff_snapshot()
            print('已收到撤手确认，开始静态窗口…',flush=True)
            aggregate=collect_static(recorder,a.capture_seconds,a.capture_samples,'auto',
                                     a.capture_seconds+capture_flow.CAPTURE_WINDOW_MARGIN_S,
                                     require_zero_ff=False)
            if aggregate is None:raise RuntimeError('静态窗口未通过，未生成合格记录')
            model=provenance(a.load_id)
            record={'schema':calibration_core.SCHEMA_SAMPLE,'recorded_at':time.time(),
                    'support':a.support,'contact':a.contact,'motion':'static',
                    'load_id':a.load_id,'model':model,
                    'note':'operator-supported current-pose hold; disengage ack; no trajectory',
                    'position_mode':result['position_mode_before'],
                    'operator_ack':result['operator_ack'],'requested_target_deg':None,**aggregate}
            ok,reasons=calibration_core.validate_sample(record,require_zero_ff=False)
            excluded=calibration_core.exclusion_reason(record) if ok else None
            baseline=capture_flow.fit_baseline(record['driver_ff']['alpha'])
            result['supported_capture']={'sample_valid':bool(ok),
                'invalid_reasons':[] if ok else reasons,
                'exclusion_reason':excluded,
                'fit_eligible':bool(ok and not excluded),
                'baseline':baseline,
                'alpha':record['driver_ff']['alpha'],
                'target_rad':result['hold_start_rad'],
                'positions_deg':[round(math.degrees(value),3) for value in record['positions_rad']],
                'static_window':aggregate['static_window'],
                'feedback_nm':record['effort_estimate_nm'],
                'operator_ack':result['operator_ack']}
            # Stop comes before any file I/O: the raw file is written only after the
            # full-axis stop and the new-disabled observation were requested.
            call(stop,Trigger.Request());stopped_at=time.monotonic()
            wait(lambda:seen.get('at',0)>stopped_at and matches('control_state',8)
                 and matches('disabled_observed',1),3)
            result['stop_observations_confirmed']=True
            path,stream=_open_raw(a.capture_output,'supported_hold')
            header={'schema':calibration_core.SCHEMA_CAPTURE,'tool':'commission_supported_trial.py',
                    'recorded_at':time.time(),'model':model,'load_id':a.load_id,
                    'runtime':record['runtime'],'requested_count':1,
                    'operation_id':a.operation_id,'operator_ack':result['operator_ack']}
            stream.write(json.dumps(header,ensure_ascii=False)+'\n')
            stream.write(json.dumps(record,ensure_ascii=False)+'\n')
            stream.flush();__import__('os').fsync(stream.fileno());stream.close();path.chmod(0o444)
            gravity=_model_fn()(record['positions_rad'])
            residual=[record['effort_estimate_nm'][i]-gravity[i] for i in range(JOINT_COUNT)]
            result['supported_capture'].update({'raw_file':str(path),
                'raw_sha256':_sha256(path),'model_gravity_nm':gravity,
                'residual_nm':residual})
            result['reason']='finite_trial_complete' if ok else 'CAPTURE_SAMPLE_INVALID'
            if not ok:result['reason']='CAPTURE_SAMPLE_INVALID'
            print('静态采样完成：%s；原始文件 %s' % ('合格' if ok else '不合格',path),flush=True)
            raise _CaptureDone()
        result['mode']='calibration' if a.calibrate else ('point_to_point' if a.point_to_point else ('waypoints' if document is not None else ('round_trip' if a.round_trip else 'current_pose_hold')))
        if a.gravity_float:result['mode']='gravity_float'
        if teaching is not None:result['mode']='drag_teaching'
        attempted=True;result.update(phase='arm_requested',arm_requested=True)
        if a.gravity_float:
            request=SetParametersAtomically.Request()
            request.parameters=[Parameter(name=name,value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=value))
                                for name,value in (('kd',float(a.float_kd)),('seconds',float(a.hold_seconds)))]
            receipt=call(arm,request);message=receipt.result.reason
        else:
            receipt=call(arm,Trigger.Request());message=receipt.message
        result['backend_arm_receipt']=message;print(message,flush=True)
        wait(lambda:matches('control_state',5) and matches('hold_ready',1),7,True)
        result['command_id']=int(status()[NAMES[0]]['command_id'])
        if a.gravity_float:
            def float_settings_confirmed():
                row=seen.get('float_state',[])
                if time.monotonic()-seen.get('float_state_at',0)>=1.5:
                    return False
                # Reuse the production FF parser; fields are not at a fixed
                # four-axis offset for five-axis messages.
                try:
                    from control_panel_model import parse_gravity_ff_state
                    state,error=parse_gravity_ff_state(
                        row,layout=seen.get('float_state_layout'))
                except Exception:
                    return False
                if state is None or error is not None:
                    result['float_settings_error']=error or {'code':'FF_PROTOCOL'}
                    return False
                expected_axes=result.get('joint_count')
                if (expected_axes is not None and state.get('axes')!=expected_axes) \
                        or not state.get('float_configurable'):
                    result['float_settings_error']={'code':'FF_AXIS_MISMATCH',
                        'axes':state.get('axes'),'expected':expected_axes}
                    return False
                if state.get('command_id')!=result['command_id'] or not state.get('gravity_float'):
                    return False
                if abs(state.get('float_kd',-1)-a.float_kd)>=1e-9 \
                        or abs(state.get('float_seconds',-1)-a.hold_seconds)>=1e-9:
                    return False
                result['float_settings_protocol']=state.get('protocol')
                return True
            wait(float_settings_confirmed,2,True)
            result['float_settings']={'kd':a.float_kd,'seconds':a.hold_seconds,'confirmed':True,
                                      'protocol':result.get('float_settings_protocol')}
        print(('自由拖动补偿渐入完成，Kd=%g，%g秒后失能，请保持防坠支撑。' % (a.float_kd,a.hold_seconds)) if a.gravity_float else (AXIS_LABEL+'使能、增益斜坡及稳定状态已观测；不是位置精度验收。'),flush=True)
        end=time.monotonic()+a.hold_seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if not matches('control_state',5):raise RuntimeError('保持中止，查看驱动审计原因')
        if a.round_trip or document is not None or teaching is not None:
            switch_controller(True);controller=True
            activated_at=time.monotonic()
            # A previous/inactive JTC reference can arrive around the switch
            # receipt. Do not freeze the new goal from that stale start: after
            # re-arm its anchor may differ from the previous goal by gravity sag.
            # Observe the controller publishing its newly active hold first.
            wait(lambda:seen.get('controller_at',0)>activated_at+.1
                 and len(seen['controller'].reference.positions)==JOINT_COUNT,3,True)
            # JTC keeps its scaling factor across goals. Reset explicitly for
            # either mode, so a previous paused session cannot stall a new goal.
            from point_to_point import scaling_publisher,publish_factor
            scaler=scaling_publisher(node)
            wait(lambda:scaler.get_subscription_count()>0,2)
            publish_factor(scaler,1.)
            scaling_at=time.monotonic()
            wait(lambda:seen.get('controller_at',0)>scaling_at and
                 abs(seen['controller'].speed_scaling_factor-1.)<1e-6,2)
            start=list(seen['controller'].reference.positions);delta=math.radians(a.delta_deg)
            result['trajectory_start_reference_rad']=list(start)
            movement_at=time.monotonic()
            if document is None and teaching is None and any(not lo<=q+delta<=hi for q,(lo,hi) in zip(start,bounds)):raise RuntimeError('接管后的轨迹目标超限，不裁剪')
            client=ActionClient(node,FollowJointTrajectory,'/slaihand/slaihand_joint_trajectory_controller/follow_joint_trajectory')
            if not client.wait_for_server(timeout_sec=3):raise RuntimeError('标准轨迹动作不可用')
            goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=NAMES;leg=a.leg_seconds
            if teaching is not None:
                teaching_points,teaching_meta=build_trajectory(teaching,start,bounds)
                planned=[(row['t'],row['q']) for row in teaching_points]
                goal.trajectory.header.stamp=(node.get_clock().now()+Duration(seconds=.3)).to_msg()
                movement_at=time.monotonic()+.3
                # Playback is a normal position-control trajectory; the 4-degree
                # bound is the unchanged driver following guard, not an accuracy claim.
                goal.goal_tolerance=[JointTolerance(name=n,position=math.radians(4.),
                                                    velocity=.03) for n in NAMES]
                result['teaching_playback']=teaching_meta
                print('拖动示教回放：接近 %.2f 秒 + 记录 %.2f 秒，总 %.2f 秒，峰值 %.3f rad/s'
                      %(teaching_meta['approach_s'],
                        teaching_meta['retimed_duration_s']-teaching_meta['approach_s'],
                        teaching_meta['retimed_duration_s'],teaching_meta['peak_speed_rad_s']),
                      flush=True)
            elif document is not None:
                planned,windows=compile_plan(document,start,bounds)
                goal.trajectory.header.stamp=(node.get_clock().now()+Duration(seconds=.3)).to_msg()
                movement_at=time.monotonic()+.3
                # Calibration needs stable torque at the MEASURED pose, not a
                # false claim of 0.5-degree accuracy. Keep that score separate.
                # Operational tolerance equals the unchanged 4-degree driver
                # following guard; its speed/position/stop checks remain active.
                goal.goal_tolerance=[JointTolerance(name=n,
                    position=math.radians(4. if a.calibrate else .5),
                    velocity=.03 if a.calibrate else 0.) for n in NAMES]
                if a.calibrate:
                    from calibration_session import CalibrationSession
                    capture=CalibrationSession(recorder,
                        (a.output_directory or (Path.home()/'.ros/slaihand/commissioning'))/'calibration',
                        result['command_id'],windows,movement_at,result['plan_sha256'],
                        load_id=a.load_id)
            else:
                planned=[(t,[q+s*delta for q in start]) for t,s in ((0.,0.),(leg,1.),(leg+.5,1.),(2*leg+.5,0.),(2*leg+1.,0.))]
            for t,qs in planned:
                point=JointTrajectoryPoint();point.positions=qs;point.velocities=[0.]*JOINT_COUNT;point.accelerations=[0.]*JOINT_COUNT
                point.time_from_start.sec=int(t);point.time_from_start.nanosec=round((t-int(t))*1e9);goal.trajectory.points.append(point)
            result['trajectory_requested']=True
            sent=client.send_goal_async(goal);wait(sent.done,3);handle=sent.result()
            if not handle.accepted:raise RuntimeError('轨迹被拒绝')
            result['trajectory_started']=True;result['phase']='trajectory'
            future=handle.get_result_async()
            control_error=None
            try:
                if a.point_to_point:
                    from point_to_point import run_retimed
                    run_retimed(node,seen,status,future,scaler,windows,planned[-1][0],movement_at,result)
                else:
                    wait(future.done,planned[-1][0]+6,True)
            except RuntimeError as error:
                control_error=error
                result['concurrent_control_error']=str(error)
                # Drain only ROS callbacks: an action abort can precede a backend
                # stop and its result must not be hidden by that later event.
                try:wait(future.done,.3)
                except RuntimeError:pass
            if teaching is not None:
                result['playback_tracking']=tracking_error_deg(
                    teaching_points,[(at,list(qs)) for at,qs in position_samples],movement_at)
                print('拖动示教回放跟踪误差°：'+json.dumps(result['playback_tracking'],ensure_ascii=False),flush=True)
            if document is not None:
                if not a.point_to_point:
                    result['waypoint_results']=assess_waypoints(windows,[(at-movement_at,qs) for at,qs in position_samples])
                result['waypoints_observed']=all(w['passed'] for w in result['waypoint_results'])
                print('各路点反馈验收：'+json.dumps(result['waypoint_results'],ensure_ascii=False),flush=True)
            if not future.done():
                raise control_error or RuntimeError('轨迹动作结果不可用')
            r=future.result().result
            result['trajectory_error_code']=r.error_code
            result['trajectory_error_string']=r.error_string
            if r.error_code:raise RuntimeError('轨迹未通过：'+r.error_string)
            if control_error is not None:raise control_error
            result['standard_jtc_action_succeeded']=True
            rows=[values for at,values in position_samples if at>=movement_at]
            if rows:
                direction=1 if delta>0 else -1
                if document is None and teaching is None:
                    result['actual_excursion_deg']=[math.degrees(max(direction*(q[i]-start[i]) for q in rows)) for i in range(JOINT_COUNT)]
                elif document is not None:
                    result['observed_delta_range_deg']=[[math.degrees(min(q[i]-start[i] for q in rows)),math.degrees(max(q[i]-start[i] for q in rows))] for i in range(JOINT_COUNT)]
                result['final_delta_from_start_deg']=[math.degrees(rows[-1][i]-start[i]) for i in range(JOINT_COUNT)]
                if document is None and teaching is None:result['return_error_deg']=result['final_delta_from_start_deg']
                if document is None and teaching is None:result['requested_excursion_observed']=all(d>=abs(a.delta_deg)*.8 for d in result['actual_excursion_deg'])
            elif teaching is None:result['requested_excursion_observed']=False
        result['reason']='finite_trial_complete'
        if document is not None and not result.get('waypoints_observed') and not a.calibrate:
            result['reason']='WAYPOINT_FEEDBACK_CRITERION_NOT_MET'
        if a.round_trip and not result.get('requested_excursion_observed'):
            result['reason']='JTC_ACTION_SUCCEEDED_BUT_EXCURSION_NOT_OBSERVED'
            print('动作服务成功，但反馈未证明所请求行程；不算往返通过。',flush=True)
    except _CaptureDone:
        pass
    except (Exception,KeyboardInterrupt) as error:
        if isinstance(error,PreArmError):
            result['phase']='pre_arm'
            result['prearm_timeout']=bool(error.timeout)
            result['prearm_reasons']=error.reasons
            result['prearm_observations']=error.observations
            result['reason']='PRE_ARM_REFUSED: '+'；'.join(error.reasons)
        else:
            result['reason']=str(error) or 'operator_interrupt'
            if not result.get('arm_requested'):
                result['phase']='pre_arm'
                result['prearm_reasons']=[result['reason']]
        print('试验结束/中止：'+result['reason'],flush=True)
        try:
            if mark_unknown(result, status()):
                print('UNKNOWN：立即保持机械支撑并现场切断电机电源，不重复使能。',flush=True)
        except Exception:
            pass
    finally:
        # Never collect during stop/disabled callbacks. Stop comes before disk
        # finalization: a full disk/fsync failure must not bypass the stop path.
        saved_capture=capture;capture=None
        if attempted:
            result['stop_requested']=True
            try:
                call(stop,Trigger.Request());at=time.monotonic()
                wait(lambda:seen.get('at',0)>at and matches('control_state',8) and matches('disabled_observed',1),2)
                result['stop_observations_confirmed']=True
                print('已观测'+AXIS_LABEL+'新反馈失能；不是物理切力矩证明。',flush=True)
            except (Exception,KeyboardInterrupt) as error:
                result['stop_error']=str(error)
                result['stop_observations_confirmed']=False
                print('失能观测未确认：保持支撑并现场切电，不重复使能。',flush=True)
            # UNKNOWN may first appear while waiting for the stop confirmation,
            # after the main exception handler already ran. Re-observe here.
            try:
                if mark_unknown(result, status()):
                    print('UNKNOWN：停止阶段观测到 UNKNOWN；保持支撑并现场切电，不重复使能。',flush=True)
            except Exception as error:
                result['unknown_observation_error']=str(error)[:200]
            if controller:
                try:switch_controller(False)
                except Exception as error:result['controller_deactivate_error']=str(error)
            if saved_capture is not None:
                try:
                    result['calibration']=saved_capture.finish(result.get('standard_jtc_action_succeeded') is True)
                    if not result['calibration']['campaign_complete'] and result.get('reason')=='finite_trial_complete':
                        result['reason']='CALIBRATION_INCOMPLETE'
                except (Exception,KeyboardInterrupt) as error:
                    result['calibration_error']=str(error)
                    result['reason']='CALIBRATION_FINALIZE_FAILED'
        # Result files are mandatory even when no enable was requested. A
        # pre-arm refusal is diagnostic evidence, not a missing client result.
        def clean(value):
            if isinstance(value,float) and not math.isfinite(value):return None
            if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
            if isinstance(value,(list,tuple)):return [clean(v) for v in value]
            return value
        try:
            result['last_observed_status']=clean(status())
        except Exception as error:
            result['last_observed_status_error']=str(error)[:200]
        if result.get('phase')=='pre_arm' and not result.get('arm_requested'):
            result['phase_result']='未请求使能'
        elif result.get('arm_requested') and not result.get('trajectory_requested'):
            result['phase_result']='已请求使能，未开始轨迹'
        elif result.get('trajectory_started'):
            result['phase_result']='已开始轨迹'
        if result.get('stop_requested') and not result.get('stop_observations_confirmed'):
            result['stop_result']='已请求停止，未确认新失能'
        elif result.get('stop_observations_confirmed'):
            result['stop_result']='已确认新失能'
        else:
            result['stop_result']='未请求停止'
        folder=a.output_directory or (Path.home()/'.ros/slaihand/commissioning');folder.mkdir(parents=True,exist_ok=True)
        path=folder/(time.strftime('%Y%m%d_%H%M%S')+'_'+result['request_id'][:8]+'.json')
        with path.open('x',encoding='utf-8') as stream:json.dump(clean(result),stream,ensure_ascii=False,indent=2,allow_nan=False)
        print('结果：'+str(path),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if result.get('reason')=='finite_trial_complete' and result['stop_observations_confirmed'] else 1


if __name__=='__main__':raise SystemExit(main())
