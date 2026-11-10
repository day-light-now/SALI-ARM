"""Actual ROS/JTC with an ideal generated-PTY drive, never a physical port.

This proves software routing/transitions, not coupled dynamics or safe torque.
"""
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'verification/hardware'))
sys.path.insert(0,str(ROOT))
from four_axis_monitor import load_mapping,usb_packet
from slai_console.openarm_mit import MITParam,pack_mit_control_data
from slai_console.commissioning import Profile
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.msg import DynamicJointState,JointTrajectoryControllerState
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


class Plant:
    def __init__(self):
        self.mapping=load_mapping();self.initial=[15.,25.,-10.,5.]
        self.raw=[self.mapping[i]['zero_offset_rad']+self.mapping[i]['hardware_sign']*math.radians(self.initial[i-1]) for i in range(1,5)]
        self.enabled=[False]*4;self.fc=[0]*4;self.fd=[0]*4;self.mit=0;self.reads=0;self.errors=[];self.first_targets={};self.fault=0
        self.master,slave=pty.openpty();self.path=os.ttyname(slave);os.close(slave)
        assert self.path.startswith('/dev/pts/')
        self.stop=threading.Event();self.thread=threading.Thread(target=self.serve,daemon=True);self.thread.start()

    def reply(self,i):
        code=round((self.raw[i-1]+12.5)*65535/25.)
        state=8 if self.fault==i else (1 if self.enabled[i-1] else 0)
        if hasattr(self,'max_follow_rate'):
            maximum=25. if i<3 else 45.
            velocity=getattr(self,'reported_velocity',[0.]*4)[i-1] if self.enabled[i-1] else 0.
            v=round((velocity+maximum)*4095/(2*maximum))
            return bytes([(state<<4)|i,code>>8,code&255,v>>4,((v&15)<<4)|7,0xFF,30,25])
        return bytes([(state<<4)|i,code>>8,code&255,0x7F,0xF7,0xFF,30,25])

    def serve(self):
        buf=bytearray()
        while not self.stop.is_set():
            try:
                if not select.select([self.master],[],[],.01)[0]:continue
                buf.extend(os.read(self.master,4096))
            except OSError:
                self.stop.wait(.002);continue
            while len(buf)>=30:
                packet=bytes(buf[:30]);del buf[:30]
                cid=int.from_bytes(packet[13:17],'little');b=packet[21:29]
                try:
                    assert packet[:13]==usb_packet(1)[:13] and packet[17:21]==usb_packet(1)[17:21]
                    if cid==0x7FF:
                        i=b[0];rid=None if b[2]==0xCC else b[3]
                        assert packet==usb_packet(i,rid);self.reads+=1
                        if rid is None:payload=self.reply(i)
                        else:
                            v={7:0,8:i,9:2000,10:1,21:12.5,22:25. if i<3 else 45.,23:200. if i<3 else 54.}[rid]
                            payload=bytes([i,0,0x33,rid])+struct.pack('<I' if rid<21 else '<f',v)
                    else:
                        assert cid in (1,2,3,4)
                        i=cid
                        if b[:7]==b'\xff'*7:
                            assert b[7] in (0xFC,0xFD)
                            if b[7]==0xFC:self.enabled[i-1]=True;self.fc[i-1]+=1
                            else:self.enabled[i-1]=False;self.fd[i-1]+=1
                        else:
                            self.mit+=1
                            q=int.from_bytes(b[:2],'big')*25./65535.-12.5
                            kp=(((b[3]&15)<<8)|b[4])*500./4095.
                            kd=((b[5]<<4)|(b[6]>>4))*5./4095.
                            vm=25. if i<3 else 45.;tm=200. if i<3 else 54.
                            dq=((b[2]<<4)|(b[3]>>4))*2*vm/4095.-vm
                            tau=(((b[6]&15)<<8)|b[7])*2*tm/4095.-tm
                            if not hasattr(self,'last_tau'):self.last_tau=[0.]*4
                            self.last_tau[i-1]=tau
                            assert kp<=160.0001 and kd<=5 and abs(dq)<=.02
                            self.first_targets.setdefault(i,self.mapping[i]['hardware_sign']*(q-self.mapping[i]['zero_offset_rad']))
                            # Independent golden path: compare C++ upstream bytes
                            # against the existing Python OpenArm encoder.
                            profile=Profile(i,1,0,2000,12.5,vm,tm,int(self.mapping[i]['hardware_sign']),self.mapping[i]['zero_offset_rad'])
                            golden=pack_mit_control_data(profile,MITParam(kp+.01,kd if kd==5 else kd+.00001,q+.00001,dq+.00001,tau+.00001))
                            assert golden==b,('official-codec parity',i,golden.hex(),b.hex())
                            if self.enabled[i-1] and kp>0:
                                desired=getattr(self,'tracking_bias',0.)
                                if not hasattr(self,'applied_bias'):self.applied_bias=[0.]*4
                                self.applied_bias[i-1]+=max(-.00002,min(.00002,desired-self.applied_bias[i-1]))
                                target=q+self.applied_bias[i-1]
                                if hasattr(self,'max_follow_rate'):
                                    step=self.max_follow_rate*.01
                                    change=max(-step,min(step,target-self.raw[i-1]))
                                    if not hasattr(self,'reported_velocity'):self.reported_velocity=[0.]*4
                                    self.reported_velocity[i-1]=change/.01
                                    self.raw[i-1]+=change
                                else:self.raw[i-1]=target
                        payload=self.reply(i)
                    os.write(self.master,b'\xaa\x11\x08'+b'\0'*4+payload+b'\x55')
                except Exception as e:
                    self.errors.append(str(e));self.stop.set();return

    def close(self):
        self.stop.set();self.thread.join(timeout=2);os.close(self.master)


def main():
    os.environ.update(ROS_DOMAIN_ID='174',ROS_LOCALHOST_ONLY='1')
    plant=Plant();node=None;process=None;result={};seen={};positions=[]
    (HERE/'ros_motion_result.json').write_text(json.dumps(dict(status='RUNNING',physical_port_opened=False)),encoding='utf-8')
    local_logs=Path(os.environ.get('TMPDIR','/tmp'))/('slai-motion-fake-audit-'+str(os.getpid()))
    with (HERE/'ros_motion.log').open('w',encoding='utf-8') as log:
        try:
            process=subprocess.Popen(['ros2','launch','slaihand_bringup','slaihand_canusb_motion.launch.py',
                'port:='+plant.path,'log_directory:='+str(local_logs)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            rclpy.init();node=rclpy.create_node('four_axis_motion_fake_verifier')
            node.create_subscription(JointState,'/slaihand/joint_states',lambda m:(seen.update(joints=m),positions.append(m)),10)
            node.create_subscription(DynamicJointState,'/slaihand/dynamic_joint_states',lambda m:seen.update(dynamic=m),10)
            node.create_subscription(JointTrajectoryControllerState,'/slaihand/slaihand_joint_trajectory_controller/controller_state',lambda m:seen.update(controller=m),10)
            def status():
                m=seen.get('dynamic')
                return {} if m is None else {n:dict(zip(v.interface_names,v.values)) for n,v in zip(m.joint_names,m.interface_values)}
            def wait(predicate,timeout=8):
                end=time.monotonic()+timeout
                while time.monotonic()<end and process.poll() is None and not plant.errors:
                    rclpy.spin_once(node,timeout_sec=.01)
                    if predicate():return
                raise AssertionError('ROS timeout/failure '+str(plant.errors)+' status='+str(status()))
            def all_field(key,value):
                s=status();return len(s)==4 and all(s.get('slaihand_joint'+str(i),{}).get(key)==value for i in range(1,5))
            def call(client,request,timeout=5):
                assert client.wait_for_service(timeout_sec=timeout),'Missing service'
                future=client.call_async(request);wait(future.done,timeout);return future.result()
            arm=node.create_client(Trigger,'/slaihand/arm_supported_current_pose')
            stop=node.create_client(Trigger,'/slaihand/stop_all')
            reset=node.create_client(Trigger,'/slaihand/reset_disabled')
            switch=node.create_client(SwitchController,'/slaihand/controller_manager/switch_controller')
            def switch_request(activate):
                r=SwitchController.Request();r.strictness=2;r.activate_asap=True;r.timeout.sec=3
                if activate:r.activate_controllers=['slaihand_joint_trajectory_controller']
                else:r.deactivate_controllers=['slaihand_joint_trajectory_controller']
                return r
            wait(lambda:all_field('control_state',0) and all_field('feedback_valid',1))
            wait(lambda:plant.reads>=40)
            assert sum(plant.fc)==0 and plant.mit==0,'Implicit enable or native control on launch'
            assert not call(switch,switch_request(True)).ok,'JTC activated before explicit takeover'
            assert call(arm,Trigger.Request()).success
            wait(lambda:all_field('control_state',5) and all_field('hold_ready',1),8)
            assert plant.fc==[1]*4
            assert all(abs(math.degrees(plant.first_targets[i])-plant.initial[i-1])<.05 for i in range(1,5)),'Takeover homed to zero'
            assert call(switch,switch_request(True)).ok,'Standard JTC failed activation after hold'
            wait(lambda:'controller' in seen and len(seen['controller'].reference.positions)==4)
            start=list(seen['controller'].reference.positions);positions.clear()
            client=ActionClient(node,FollowJointTrajectory,'/slaihand/slaihand_joint_trajectory_controller/follow_joint_trajectory')
            assert client.wait_for_server(timeout_sec=3)
            goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=['slaihand_joint'+str(i) for i in range(1,5)]
            delta=math.radians(1)
            for t,scale in ((0.,0.),(4.,1.),(4.5,1.),(8.5,0.),(9.,0.)):
                p=JointTrajectoryPoint();p.positions=[q+scale*delta for q in start];p.velocities=[0.]*4;p.accelerations=[0.]*4
                p.time_from_start.sec=int(t);p.time_from_start.nanosec=round((t-int(t))*1e9);goal.trajectory.points.append(p)
            sent=client.send_goal_async(goal);wait(sent.done,3);handle=sent.result();assert handle.accepted
            finished=handle.get_result_async();wait(finished.done,13)
            assert finished.result().result.error_code==0,finished.result().result.error_string
            excursions=[]
            for i,name in enumerate(goal.trajectory.joint_names):
                values=[dict(zip(m.name,m.position)).get(name,float('nan')) for m in positions]
                excursions.append(math.degrees(max(q for q in values if math.isfinite(q))-start[i]))
            assert all(.8<q<1.2 for q in excursions),excursions
            assert call(stop,Trigger.Request()).success
            wait(lambda:all_field('control_state',8) and all_field('disabled_observed',1),3)
            assert all(n>=1 for n in plant.fd)
            assert call(switch,switch_request(False)).ok
            assert call(reset,Trigger.Request()).success
            wait(lambda:all_field('control_state',0),3)
            assert plant.fc==[1]*4,'Reset implicitly enabled motors'
            # Exercise the installed operator client itself on this same PTY.
            with (HERE/'operator_client.log').open('w',encoding='utf-8') as client_log:
                runner=subprocess.Popen(['ros2','run','slaihand_bringup','commission_supported_trial.py','--execute',
                    '--ack-supported','--ack-clearance','--ack-native-output','--hold-seconds','1',
                    '--output-directory',str(local_logs/'client_results')],stdout=client_log,stderr=subprocess.STDOUT)
                try:wait(lambda:runner.poll() is not None,10)
                finally:
                    if runner.poll() is None:runner.terminate();runner.wait(timeout=3)
                assert runner.returncode==0,'Installed operator client failed'
            client_results=list((local_logs/'client_results').glob('*.json'));assert len(client_results)==1
            client_result=json.loads(client_results[0].read_text())
            assert client_result['reason']=='finite_trial_complete' and client_result['stop_observations_confirmed']
            assert client_result['command_id']==2,'Client receipt lost the backend command_id'
            assert all_field('control_state',8)
            assert call(reset,Trigger.Request()).success
            wait(lambda:all_field('control_state',0),3)
            # A second takeover, then one drive fault: every axis receives FD;
            # persistent fault feedback cannot be called confirmed disabled.
            assert call(arm,Trigger.Request()).success
            wait(lambda:all_field('hold_ready',1),8)
            before=list(plant.fd);plant.fault=3
            wait(lambda:all_field('control_state',9),3)
            assert all(a>b for a,b in zip(plant.fd,before)),'Single-axis fault did not stop all axes'
            assert not call(reset,Trigger.Request()).success and not call(arm,Trigger.Request()).success
            result=dict(status='PASS',physical_port_opened=False,software_only=True,
                no_auto_enable_or_home=True,official_codec_parity_checked=True,
                standard_jtc_round_trip=True,actual_fake_excursion_deg=excursions,
                fresh_disabled_observed=True,manual_reset_does_not_enable=True,
                single_axis_fault_stops_all=True,unknown_locked=True,j5_tx=0,parameter_writes=0)
            result['installed_operator_client_hold_passed']=True
        finally:
            if node is not None:node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM)
                    try:process.wait(timeout=2)
                    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=2)
            plant.close()
    import shutil
    destination=HERE/'fake_audit';destination.mkdir(exist_ok=True)
    for path in local_logs.glob('*.jsonl'):shutil.copy2(path,destination/path.name)
    for path in (local_logs/'client_results').glob('*.json'):shutil.copy2(path,HERE/('operator_'+path.name))
    (HERE/'ros_motion_result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
