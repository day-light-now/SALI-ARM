"""Run the installed waypoint client against newly allocated PTY drives only."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from verify_ros_motion import Plant

HERE=Path(__file__).resolve().parent
os.environ.update(ROS_DOMAIN_ID='175',ROS_LOCALHOST_ONLY='1')
plant=Plant()
assert plant.path.startswith('/dev/pts/')
result=dict(software_only=True,physical_port_opened=False)
with tempfile.TemporaryDirectory(prefix='waypoint-client-') as directory:
    directory=Path(directory)
    plan=directory/'plan.json'
    plan.write_text(json.dumps(dict(coordinates='relative_start_deg',waypoints=[
        dict(degrees=[2,1,-1,2],move_s=4,dwell_s=1),
        dict(degrees=[0,0,0,0],move_s=4,dwell_s=1)])))
    log=(HERE/'waypoint_ros.log').open('w')
    proc=subprocess.Popen(['ros2','launch','slaihand_bringup','slaihand_canusb_motion.launch.py',
        'port:='+plant.path,'log_directory:='+str(directory/'audit')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        time.sleep(3)
        base=['ros2','run','slaihand_bringup','commission_supported_trial.py','--plan',str(plan),'--hold-seconds','1']
        preview=subprocess.run(base,capture_output=True,text=True,timeout=12)
        assert preview.returncode==0,preview.stdout+preview.stderr
        assert sum(plant.fc)==0,'Preview enabled hardware'
        execution=subprocess.run(base+['--execute','--ack-supported','--ack-clearance','--ack-native-output',
            '--output-directory',str(directory/'results')],capture_output=True,text=True,timeout=30)
        (HERE/'waypoint_client.log').write_text(execution.stdout+execution.stderr)
        assert execution.returncode==0,execution.stdout+execution.stderr
        actual=json.loads(next((directory/'results').glob('*.json')).read_text())
        assert actual['observed_delta_range_deg'][2][0]<-.8,'Negative movement lost in report'
        assert actual['waypoints_observed'] and actual['stop_observations_confirmed']
        assert len(actual['waypoint_results'])==2 and not plant.errors
        assert plant.fc==[1]*4 and all(v>=1 for v in plant.fd)
        result.update(status='PASS',preview_did_not_enable=True,independent_waypoints_observed=True,
                      fresh_stop_confirmed=True,client_result=actual)
        reset=subprocess.run(['ros2','service','call','/slaihand/reset_disabled','std_srvs/srv/Trigger','{}'],capture_output=True,text=True,timeout=8)
        assert 'success=True' in reset.stdout,reset.stdout+reset.stderr
        failure=subprocess.Popen(base+['--execute','--ack-supported','--ack-clearance','--ack-native-output',
            '--output-directory',str(directory/'failure_results')],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        time.sleep(5)
        plant.tracking_bias=math.radians(.6)
        failure_output,_=failure.communicate(timeout=25)
        (HERE/'waypoint_client_failure.log').write_text(failure_output)
        failed=json.loads(next((directory/'failure_results').glob('*.json')).read_text())
        assert failure.returncode!=0 and failed.get('trajectory_error_code')==-5,failed
        assert failed.get('trajectory_error_string') and 'waypoint_results' in failed
        assert not failed['waypoints_observed'] and failed['stop_observations_confirmed']
        result['goal_tolerance_failure_preserved']=True
        result['failure_result']=failed
    finally:
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)
        log.close();plant.close()
(HERE/'waypoint_client_result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
