"""Synthetic friction/gain comparison only; no port or hardware operations."""
import json, math
from slai_console.openarm_mit import model_command, decode_mit_control_data
from slai_console.commissioning import Profile
p=Profile(4,1,0,2000,12.5,45,54,-1,-.40951400015259054)
def simulate(kp,kd,inertia,kinetic):
    q=v=peak=0.
    for k in range(18001):
        t=k*.001;u=min(1,t/18)
        if k%10==0:
            ref=math.radians(10)*(10*u**3-15*u**4+6*u**5)
            vel=math.radians(10)*30*u*u*(1-u)**2/18
            d=decode_mit_control_data(p,model_command(p,q=ref,dq=vel,kp=kp,kd=kd))
            if abs(v)>.2:
                return dict(peak_speed=peak,stop='synthetic_0.2_speed_abort',angle_deg=math.degrees(q))
        tau=-(d.kp*(d.q-(p.zero-q))+d.kd*(d.dq+v)+d.tau)
        if abs(v)<1e-4 and abs(tau)<=1.2:
            v=0.
        else:
            direction=math.copysign(1,v if abs(v)>1e-8 else tau)
            next_v=v+(tau-.05*v-direction*kinetic)/inertia*.001
            if next_v*v<0 and abs(tau)<=1.2:next_v=0.
            v=next_v;q+=v*.001
        peak=max(peak,abs(v))
    return dict(peak_speed=peak,stop=None,angle_deg=math.degrees(q))
if __name__=='__main__':
    rows=[]
    for kp,kd in ((32,2),(160,5)):
        for inertia in (.02,.05,.15):
            for kinetic in (.2,.6,1.):
                rows.append(dict(kp=kp,kd=kd,inertia=inertia,static_friction=1.2,kinetic_friction=kinetic,
                                 **simulate(kp,kd,inertia,kinetic)))
    print(json.dumps(dict(note='synthetic parameters, not identified hardware; stricter 0.2 guard used for comparison',cases=rows),indent=2))
