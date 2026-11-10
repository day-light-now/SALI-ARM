"""Production HTTP+JTC, newly generated PTY only, isolated ROS domain178."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT/'verification/hardware/control_panel_20261104'))
os.environ['PANEL_SCRIPT']='/home/xuyu/slaihand_ptp_ws_20261104/install/slaihand_bringup/lib/slaihand_bringup/control_panel.py'
import verify_control_panel as fixture
os.environ.update(ROS_DOMAIN_ID='178',ROS_LOCALHOST_ONLY='1')
import rclpy

PLAN=dict(coordinates='relative_start_deg',waypoints=[
    dict(degrees=[10,5,0,0],move_s=2,dwell_s=.5),
    dict(degrees=[0,0,0,0],move_s=2,dwell_s=.5)])
results=[]
rclpy.init()
try:
    for name,rate,bias in [('ideal',None,0),('slow',.06,0),('biased',None,.6),('stalled',0,0)]:
        plant=fixture.Plant()
        assert plant.path.startswith('/dev/pts/')
        if rate is not None:plant.max_follow_rate=rate
        if bias:plant.tracking_bias=__import__('math').radians(bias)
        observer=fixture.Observer();panel=None;heartbeat=None;stack=None
        directory=Path(tempfile.mkdtemp(prefix='slai-ptp-'+name+'-'))
        log=(directory/'stack.log').open('w')
        try:
            stack=subprocess.Popen(['ros2','launch','slaihand_bringup','slaihand_canusb_motion.launch.py',
                'port:='+plant.path,'log_directory:='+str(directory/'audit')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            assert observer.wait(lambda o:o.stage()==0 and o.axis(0,'feedback_valid')==1,12),(directory/'stack.log').read_text()
            panel=fixture.Panel(directory/'archive',ttl=60,domain='178')
            heartbeat=fixture.Heartbeat(panel);heartbeat.start()
            panel.wait_status(lambda s:s['ros']['feedback_valid_all'])
            code,p=panel.call('POST','/api/preview',dict(document=PLAN,execution_mode='point_to_point'))
            assert code==200,p
            ticket=p['ticket']['id']
            wrong=panel.call('POST','/api/execute',dict(document=PLAN,ticket=ticket,
                 confirmations=fixture.confirmations(),execution_mode='trajectory'))
            assert wrong[0]==409 and wrong[1]['code']=='MODE_CHANGED',wrong
            assert plant.fc==[0]*4
            code,p=panel.call('POST','/api/execute',dict(document=PLAN,ticket=ticket,
                 confirmations=fixture.confirmations(),execution_mode='point_to_point'))
            assert code==202,p
            operation=panel.wait_operation(observer,timeout=27)
            print(name,json.dumps(operation,ensure_ascii=False),flush=True)
            result=operation.get('result') or {}
            assert not plant.errors,plant.errors
            assert result.get('stop_observations_confirmed') is True,result
            if name in ('ideal','slow'):
                assert operation['exit_code']==0 and result.get('accuracy_passed') is True,result
                if name=='slow':assert result['point_to_point']['minimum_scaling']<.9,result
            else:
                assert operation['exit_code']!=0 and result.get('overall_ok') is False,result
                assert 'PTP_' in result['reason'] or 'ENDPOINT_NOT_SETTLED' in result['reason'],result
            results.append(dict(name=name,result=result))
        finally:
            if heartbeat:heartbeat.stop()
            if panel:panel.terminate()
            observer.close()
            if stack and stack.poll() is None:
                stack.send_signal(signal.SIGINT)
                try:stack.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(stack.pid,signal.SIGKILL);stack.wait(timeout=3)
            log.close();plant.close()
finally:
    rclpy.shutdown()
    (HERE/'ptp_ros_result.json').write_text(json.dumps(dict(cases=results,software_only=True,physical_port_opened=False),indent=2))
print('PTP_ROS_PASS',flush=True)
