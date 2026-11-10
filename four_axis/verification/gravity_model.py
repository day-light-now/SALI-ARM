"""Offline static gravity calculation. No ROS, CAN, serial or motor output.

Positive output is holding torque in URDF joint coordinates (dV/dq).
Mass data and base-frame gravity direction must be explicitly supplied.
This is not a dynamics simulator or a real-time controller.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


class GravityModel:
    def __init__(self, urdf, properties, gravity, *, fixed_joint_offsets=None):
        robot=ET.parse(urdf).getroot()
        self.joints=list(robot.findall('joint'))
        self.names=[j.get('name') for j in self.joints if j.get('type')=='revolute']
        self.fixed_joint_offsets=dict(fixed_joint_offsets or {})
        if any(n not in self.names or not np.isfinite(v)
               for n,v in self.fixed_joint_offsets.items()):
            raise ValueError('Invalid fixed joint placement offset')
        if any(j.get('type') not in ['fixed','revolute'] for j in self.joints):
            raise ValueError('Only fixed and revolute joints are supported')
        children={j.find('child').get('link') for j in self.joints}
        roots={l.get('name') for l in robot.findall('link')}-children
        if len(roots)!=1:
            raise ValueError('Expected one root')
        self.root=roots.pop()
        self.properties=properties
        self.gravity=np.asarray(gravity,dtype=float)
        if self.gravity.shape!=(3,) or not np.isfinite(self.gravity).all():
            raise ValueError('Invalid gravity vector')
        for link in children:
            p=properties[link]
            mass=float(p['mass_kg']); com=np.asarray(p['com_link_m'],dtype=float)
            if not np.isfinite(mass) or mass<=0 or com.shape!=(3,) or not np.isfinite(com).all():
                raise ValueError(f'Invalid mass or COM: {link}')

    def evaluate(self, positions):
        q=np.asarray(positions,dtype=float)
        if q.shape!=(len(self.names),) or not np.isfinite(q).all():
            raise ValueError('One finite radian value is required per movable joint')
        angles=dict(zip(self.names,q))
        frames={self.root:np.eye(4)}; chains={self.root:[]}
        pending=self.joints.copy(); torque=np.zeros(len(q)); energy=0.
        while pending:
            progressed=False
            for joint in pending.copy():
                parent=joint.find('parent').get('link'); child=joint.find('child').get('link')
                if parent not in frames:
                    continue
                origin=joint.find('origin'); t=np.eye(4)
                if origin is not None:
                    t[:3,3]=[float(x) for x in origin.get('xyz','0 0 0').split()]
                    t[:3,:3]=Rotation.from_euler('xyz',[float(x) for x in origin.get('rpy','0 0 0').split()]).as_matrix()
                world=frames[parent]@t
                chain=list(chains[parent])
                if joint.get('type')=='revolute':
                    axis=np.array([float(x) for x in joint.find('axis').get('xyz').split()])
                    axis/=np.linalg.norm(axis)
                    name=joint.get('name')
                    # Same fixed downstream-body placement used by MuJoCo.
                    # The variable coordinate and its URDF limits stay unchanged.
                    fixed=np.eye(4)
                    fixed[:3,:3]=Rotation.from_rotvec(
                        axis*self.fixed_joint_offsets.get(name,0.)).as_matrix()
                    world=world@fixed
                    limit=joint.find('limit')
                    if not float(limit.get('lower'))<=angles[name]<=float(limit.get('upper')):
                        raise ValueError(f'Position outside URDF limits: {name}')
                    chain.append((self.names.index(name),world[:3,3].copy(),world[:3,:3]@axis))
                    motion=np.eye(4); motion[:3,:3]=Rotation.from_rotvec(axis*angles[name]).as_matrix()
                    world=world@motion
                frames[child]=world; chains[child]=chain
                p=self.properties[child]
                center=world[:3,:3]@np.array(p['com_link_m'])+world[:3,3]
                force=float(p['mass_kg'])*self.gravity
                energy-=force@center
                for index,pivot,axis in chain:
                    torque[index]-=axis@np.cross(center-pivot,force)
                pending.remove(joint); progressed=True
            if not progressed:
                raise ValueError('Invalid or disconnected kinematic tree')
        return torque,float(energy)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',type=Path,default=Path(__file__).parent.parent/'src/slaihand_description/urdf/slaihand_four_axis.urdf')
    parser.add_argument('--mass-audit',type=Path,required=True)
    parser.add_argument('--degrees',type=float,nargs=4,required=True)
    parser.add_argument('--gravity-base',type=float,nargs=3,required=True,help='Measured/documented gravity vector in base_link, m/s^2')
    parser.add_argument('--allow-provisional',action='store_true',help='Offline review only; acknowledge uncalibrated CAD masses')
    args=parser.parse_args()
    if not args.allow_provisional:
        parser.error('CAD audit values are uncalibrated. Explicit --allow-provisional is required for offline review.')
    report=json.loads(args.mass_audit.read_text(encoding='utf-8'))
    for name,p in report['links'].items():
        if 'error' in p or p.get('read_count')!=p.get('part_count'):
            parser.error(f'Incomplete mass extraction: {name}')
    model=GravityModel(args.urdf,report['links'],args.gravity_base)
    torque,energy=model.evaluate(np.deg2rad(args.degrees))
    print(json.dumps({'status':'PROVISIONAL_OFFLINE_ONLY','holding_torque_Nm':dict(zip(model.names,torque.tolist())),
        'potential_energy_J':energy,'gravity_base_m_s2':args.gravity_base},indent=2))

if __name__=='__main__':
    main()
