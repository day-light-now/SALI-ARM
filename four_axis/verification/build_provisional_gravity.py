"""Explicit offline assumptions; does not edit CAD or canonical URDF."""
from pathlib import Path
import json
from collections import defaultdict
import numpy as np

folder=Path(__file__).parent/'gravity_preparation'
audit=json.loads((folder/'cad_mass_audit.json').read_text(encoding='utf-8'))
vendor=json.loads((folder/'motor_mass_sources.json').read_text(encoding='utf-8'))
parts=audit['parts']; groups=defaultdict(list)
for p in parts:
    if 'DAMIAO' in p['component'] or 'DM-J4340' in p['component']:
        groups['/'.join(p['component'].split('/')[:2])].append(p)
corrections=[]
for name,rows in groups.items():
    model=next((n for n in vendor['motors'] if n.replace('DM-J','').replace('-2EC','') in name),None)
    if model is None: raise ValueError(f'Unknown motor: {name}')
    measured=sum(p['mass_kg'] for p in rows)
    factor=vendor['motors'][model]['mass_kg']/measured
    for p in rows:
        p['mass_kg']*=factor
        p['inertia_com_link_kg_m2']=(np.array(p['inertia_com_link_kg_m2'])*factor).tolist()
    corrections.append({'instance':name,'model':model,'scale':factor,'target_mass_kg':vendor['motors'][model]['mass_kg']})
links={}
for link in audit['links']:
    rows=[p for p in parts if p['link']==link and p.get('mass_kg',0)>0 and not p.get('error')]
    mass=sum(p['mass_kg'] for p in rows)
    center=sum(p['mass_kg']*np.array(p['com_link_m']) for p in rows)/mass
    tensor=np.zeros((3,3))
    for p in rows:
        delta=np.array(p['com_link_m'])-center
        tensor+=np.array(p['inertia_com_link_kg_m2'])+p['mass_kg']*((delta@delta)*np.eye(3)-np.outer(delta,delta))
    links[link]={'mass_kg':mass,'com_link_m':center.tolist(),'inertia_com_link_kg_m2':tensor.tolist()}
report={'status':'ESTIMATE_ONLY_NOT_FOR_HARDWARE','links':links,'motor_corrections':corrections,
    'gravity_base_m_s2':[0,9.80665,0], 'gravity_direction':'+y',
    'gravity_source':'User CAD shoulder-mount screenshot 2026-10-08: base +Y points down; installation not measured.',
    'assumptions':['Shoulder mounting: J1 horizontal; base +Y points down per user CAD diagram.',
    'Vendor assembled motor mass uniformly scales its CAD pieces; COM shape and rotor/stator split are estimates.',
    'Motor pieces remain in current CAD link grouping; rotor/stator dynamic allocation is not verified.',
    'Unassigned structural materials keep CAD default density; ABS is treated as solid.',
    'Zero-volume motor reference part omitted; its contribution is absorbed by total motor mass scaling.',
    'J4340 mass is included on link4, no J5 motion. No cable or extra payload included.',
    'Mass factor sensitivity is not a statistical confidence interval.']}
(folder/'provisional_gravity.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print('Offline provisional model generated; canonical robot unchanged.')
