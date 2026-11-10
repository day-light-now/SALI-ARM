"""Offline gravity/quantization/replay tests; no physical motor connection."""
from dataclasses import replace
import json
import math
from pathlib import Path
import struct
import tempfile
import sys
import unittest
from unittest.mock import patch

import numpy as np
from verification.gravity_model import GravityModel
from .commissioning import Profile
from .core import ROOT
from .gravity_feedforward import (GravityBounds, GravityCandidate, provisional_model,
                                  candidate_mit_packet)
from .gravity_replay import replay, read_mapping
from .openarm_mit import model_command, decode_mit_control_data


def synthetic_loaded_axis(compensate):
    """Known constant load, synthetic inertia; NOT identified SLAI dynamics."""
    profile=Profile(2,1,0,2000,12.5,25.,200.,1,-1.8)
    controller=GravityCandidate(ConstantGravity(1.6))
    q=0.;v=0.;target=0.;decoded=None
    for step in range(20001):
        now=step*.001
        if step%10==0:
            u=min(1.,max(0.,(now-1.)/18.))
            target=math.radians(10)*(10*u**3-15*u**4+6*u**5)
            velocity=math.radians(10)*30*u*u*(1-u)**2/18.
            packet=model_command(profile,q=target,dq=velocity,kp=160.,kd=5.)
            if compensate:
                row=controller.step([0.,q,0.,0.],now,[now]*4)
                packet=candidate_mit_packet(profile,packet,row['model_ff_nm'],2.)
            decoded=decode_mit_control_data(profile,packet)
        tau=(decoded.kp*(decoded.q-profile.zero-q)+
             decoded.kd*(decoded.dq-v)+decoded.tau)
        # Exact declared disturbance 1.6Nm; assumed J=.15 and damping=.2.
        v+=(tau-1.6-.2*v)/.15*.001;q+=v*.001
    return dict(final_error_deg=math.degrees(target-q),final_q_deg=math.degrees(q),
                synthetic_only=True,load_nm=1.6,inertia_kg_m2=.15,
                candidate_fraction=.5 if compensate else 0.)


class ConstantGravity:
    def __init__(self,value):self.value=value
    def evaluate(self,q):return [0.,self.value,0.,0.],0.


class Tests(unittest.TestCase):
    def test_compensation_reduces_known_load_offset_without_raising_kp(self):
        baseline=synthetic_loaded_axis(False);candidate=synthetic_loaded_axis(True)
        self.assertGreater(baseline['final_error_deg'],.5)
        self.assertLess(abs(candidate['final_error_deg']),.5)
        self.assertLess(abs(candidate['final_error_deg']),abs(baseline['final_error_deg']))

    def test_fixed_placement_analytic_pendulum(self):
        xml='''<robot name="analytic"><link name="base"/><link name="bob"/>
        <joint name="hinge" type="revolute"><parent link="base"/><child link="bob"/>
        <axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint></robot>'''
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'pendulum.urdf';path.write_text(xml,encoding='utf-8')
            properties={'bob':{'mass_kg':2.,'com_link_m':[.3,0,0]}}
            original=GravityModel(path,properties,[0,-9.8,0])
            flipped=GravityModel(path,properties,[0,-9.8,0],fixed_joint_offsets={'hinge':math.pi})
            for q in (-.5,0.,.5):
                self.assertAlmostEqual(original.evaluate([q])[0][0],2*.3*9.8*math.cos(q))
                self.assertAlmostEqual(flipped.evaluate([q])[0][0],-2*.3*9.8*math.cos(q))
            with self.assertRaises(ValueError):flipped.evaluate([1.1])

    def test_provisional_corrected_energy_gradient(self):
        model,evidence=provisional_model()
        self.assertFalse(evidence['hardware_authorized'])
        for q in (np.zeros(4),np.array([.2,.4,-.3,.2])):
            g=model.evaluate(q)[0];eps=1e-6
            grad=[(model.evaluate(q+np.eye(4)[i]*eps)[1]-
                   model.evaluate(q-np.eye(4)[i]*eps)[1])/(2*eps) for i in range(4)]
            np.testing.assert_allclose(g,grad,atol=1e-7)

    def test_default_gravity_model_preserves_old_results(self):
        model,_=provisional_model()
        original=GravityModel(ROOT/'src/slaihand_description/urdf/slaihand_four_axis.urdf',
                              model.properties,model.gravity)
        explicit=GravityModel(ROOT/'src/slaihand_description/urdf/slaihand_four_axis.urdf',
                              model.properties,model.gravity,fixed_joint_offsets={})
        np.testing.assert_allclose(original.evaluate([.1,.2,.3,.1])[0],
                                   explicit.evaluate([.1,.2,.3,.1])[0])

    def test_matches_current_mujoco_including_j3_placement(self):
        # Read existing MJCF and meshes without rebuilding or changing a model.
        import xml.etree.ElementTree as ET
        sys.path.append(str(ROOT.parent/'五轴仿真/_sim_deps'))
        import mujoco
        path=ROOT/'verification/impedance_demo/shoulder_dynamics.xml'
        xml=path.read_text(encoding='utf-8')
        assets={m.get('file'):(ROOT/'src/slaihand_description/meshes'/m.get('file')).read_bytes()
                for m in ET.fromstring(xml).findall('asset/mesh')}
        m=mujoco.MjModel.from_xml_string(xml,assets=assets);data=mujoco.MjData(m)
        reference,_=provisional_model()
        for degrees in ([0,0,0,0],[25,65,0,-25],[-30,100,20,-40]):
            data.qpos[:]=np.deg2rad(degrees);data.qvel[:]=0.
            mujoco.mj_forward(m,data)
            np.testing.assert_allclose(data.qfrc_bias,reference.evaluate(data.qpos)[0],atol=1e-8)

    def test_ramp_slew_and_cap(self):
        c=GravityCandidate(ConstantGravity(100.))
        first=c.step([0.]*4,0.,[0.]*4)
        self.assertEqual(first['model_ff_nm'],0.)
        previous=0.
        for i in range(1,301):
            now=i*.01;r=c.step([0.]*4,now,[now]*4)
            self.assertLessEqual(abs(r['model_ff_nm']-previous),.010000000001)
            self.assertLessEqual(abs(r['model_ff_nm']),2.)
            previous=r['model_ff_nm']
        self.assertAlmostEqual(previous,2.)
        self.assertTrue(r['clipped'])

    def test_sign_reversal_has_no_fast_reverse_growth(self):
        m=ConstantGravity(100.);c=GravityCandidate(m)
        for i in range(21):c.step([0.]*4,i*.01,[i*.01]*4)
        before=c.previous;m.value=-100.
        r=c.step([0.]*4,.21,[.21]*4)
        self.assertAlmostEqual(r['model_ff_nm'],before-.01)

    def test_stale_missing_nonfinite_and_clock_faults_latch(self):
        for q,now,at in (([0.]*4,1.,[.8]*4),([0.]*3,1.,[1.]*4),
                         ([0.,float('nan'),0.,0.],1.,[1.]*4),([0.]*4,1.,[1.1]*4)):
            c=GravityCandidate(ConstantGravity(1.))
            with self.assertRaises(ValueError):c.step(q,now,at)
            self.assertEqual(c.previous,0.)
            with self.assertRaises(ValueError):c.step([0.]*4,2.,[2.]*4)
        for t in (0.,-.1,.2):
            c=GravityCandidate(ConstantGravity(1.));c.step([0.]*4,0.,[0.]*4)
            with self.assertRaises(ValueError):c.step([0.]*4,t,[t]*4)

    def test_invalid_bounds_and_scope(self):
        for opts in ({'fraction':1.1},{'cap_nm':0.},{'slew_nm_s':-1.},
                     {'ramp_s':0.},{'max_gap_s':float('nan')}):
            with self.assertRaises(ValueError):GravityBounds(**opts)
        for joint in (1,3,4,5,True):
            with self.assertRaises(ValueError):GravityCandidate(ConstantGravity(1.),joint=joint)

    def test_candidate_field_sign_quantization_and_unchanged_pd(self):
        for sign in (-1,1):
            p=Profile(2,1,0,2000,12.5,25.,200.,sign,-1.8)
            original=model_command(p,q=.1,dq=.01,kp=160.,kd=5.)
            for ff in (-2.,-.2,0.,.2,2.):
                packet=candidate_mit_packet(p,original,ff,2.)
                d=decode_mit_control_data(p,packet)
                self.assertEqual(packet[:6],original[:6])
                self.assertEqual(packet[6]&0xf0,original[6]&0xf0)
                self.assertLessEqual(abs(d.tau),2.)
                self.assertAlmostEqual(sign*d.tau,ff,delta=2*p.tmax/4095)
                if ff==0.:self.assertEqual(packet,original)
            with self.assertRaises(ValueError):candidate_mit_packet(p,original,2.1,2.)
            with self.assertRaises(ValueError):candidate_mit_packet(p,original,0.,.001)
            with self.assertRaises(ValueError):candidate_mit_packet(replace(p,joint=4),original,1.,2.)

    def test_replay_uses_four_fresh_measured_axes_and_never_imports_serial(self):
        mapping=read_mapping(ROOT/'src/slaihand_bringup/config/hardware_mapping.yaml')
        with tempfile.TemporaryDirectory() as d:
            folder=Path(d);raw=[];audit=[]
            for i in range(1,5):
                for rid,value in ((21,12.5),(22,25. if i<3 else 45.),(23,200. if i<3 else 54.)):
                    payload=bytes([i,0,0x33,rid])+struct.pack('<f',value)
                    raw.append(dict(time=9.9,direction='rx',payload=payload.hex(' ')))
            for step in range(3):
                now=10.+step*.01
                for i in range(1,5):
                    code=round((mapping[i]['zero_offset_rad']+12.5)*65535/25)
                    payload=bytes([i,code>>8,code&255,0x7f,0xf7,0xff,25,25])
                    raw.append(dict(time=now,direction='rx',payload=payload.hex(' ')))
                p=Profile(2,1,0,2000,12.5,25.,200.,1,mapping[2]['zero_offset_rad'])
                packet=model_command(p,q=0.,dq=0.,kp=160.,kd=5.)
                audit.append(dict(unix_s=now,kind='SERIAL_WRITTEN',joint=2,frame_kind='mit',payload=packet.hex(' ')))
            (folder/'raw.jsonl').write_text('\n'.join(map(json.dumps,raw)),encoding='utf-8')
            (folder/'trial.jsonl').write_text('\n'.join(map(json.dumps,audit)),encoding='utf-8')
            path=folder/'trial.json';path.write_text(json.dumps(dict(joint=2,backend='drive_mit',
                readonly_log=str(folder/'raw.jsonl'),observed_state_estimate_guard_nm=10.)),encoding='utf-8')
            with patch.dict('sys.modules',{'serial':None}):summary=replay(path,folder/'output')
            self.assertEqual(summary['sample_count'],3)
            self.assertEqual(summary['actual_serial_writes'],0)
            self.assertFalse(summary['hardware_authorized'])


if __name__=='__main__':unittest.main()
