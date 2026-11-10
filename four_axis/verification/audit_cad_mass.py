"""Read-only SolidWorks mass/material audit. Never enables torque control.

CAD values are provisional: imported motor solids are not calibrated masses.
SolidWorks GetMomentOfInertia(0) is about COM in document coordinate axes.
"""
from pathlib import Path
import json
import numpy as np
import pythoncom
import win32com.client

root = Path(__file__).parent
out = root/'gravity_preparation'
out.mkdir(exist_ok=True)
cad = json.loads((root/'cad_j4_corrected/cad_transforms.json').read_text(encoding='utf-8'))
doc = win32com.client.GetActiveObject('SldWorks.Application').ActiveDoc
if doc is None or str(doc.GetPathName).lower() != cad['source'].lower():
    raise RuntimeError('Open the verified SLAI assembly before extraction')

def transform(value):
    a=np.asarray(value.ArrayData)
    t=np.eye(4); t[:3,:3]=a[:9].reshape(3,3).T*a[12]; t[:3,3]=a[9:12]
    return t

records=[]
for component in doc.GetComponents(False):
    if component.GetChildren:
        continue
    name=str(component.Name2)
    group=name.split('/')[0].split('-')[0]
    group='base_link' if group=='base' else group
    if group not in cad['frames']:
        continue
    row={'component':name,'link':group,'confidence':'unverified_CAD_only'}
    try:
        model=component.GetModelDoc2
        if model is None:
            raise RuntimeError('Unresolved or suppressed component')
        row['document']=str(model.GetPathName)
        row['configuration']=str(component.ReferencedConfiguration)
        database=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_BSTR,'')
        try:
            row['material']=str(model.GetMaterialPropertyName2(row['configuration'],database) or '')
            row['material_database']=str(database.value)
        except Exception as exc:
            row['material']=''; row['material_query_error']=str(exc)
        mass=model.Extension.CreateMassProperty
        mass.UseSystemUnits=True
        row['mass_kg']=float(mass.Mass)
        row['volume_m3']=float(mass.Volume)
        row['effective_density_kg_m3']=row['mass_kg']/row['volume_m3'] if row['volume_m3']>0 else None
        c=np.asarray(mass.CenterOfMass,dtype=float)
        inertia=np.asarray(mass.GetMomentOfInertia(0),dtype=float).reshape(3,3)
        t=np.linalg.inv(cad['frames'][group])@transform(component.Transform2)
        row['com_link_m']=(t[:3,:3]@c+t[:3,3]).tolist()
        row['inertia_com_link_kg_m2']=(t[:3,:3]@inertia@t[:3,:3].T).tolist()
        row['configuration_matches_loaded']=row['configuration']==str(model.ConfigurationManager.ActiveConfiguration.Name)
        row['issues']=[]
        if not row['material']:
            row['issues'].append('No explicit part material; default/body/override mass must be checked')
        if not row['configuration_matches_loaded']:
            row['issues'].append('Loaded configuration differs from referenced configuration')
        if row['mass_kg']<=0 or row['volume_m3']<=0:
            row['issues'].append('Non-positive mass or volume')
        if any(x in name.lower() for x in ['damiao','dm-j','dm_','8009','10010','4340','pcb']):
            row['issues'].append('Motor/electronics CAD must be checked against vendor or measured mass')
    except Exception as exc:
        row['error']=str(exc)
    records.append(row)
    print('Read',ascii(name),row.get('mass_kg'),ascii(row.get('material','')),flush=True)

links={}
for group in cad['frames']:
    parts=[r for r in records if r['link']==group]
    valid=[r for r in parts if 'mass_kg' in r and 'com_link_m' in r and r['mass_kg']>0]
    total=sum(r['mass_kg'] for r in valid)
    if total<=0:
        links[group]={'error':'No positive mass'}; continue
    center=sum(r['mass_kg']*np.array(r['com_link_m']) for r in valid)/total
    tensor=np.zeros((3,3))
    for r in valid:
        offset=np.array(r['com_link_m'])-center
        tensor+=np.array(r['inertia_com_link_kg_m2'])+r['mass_kg']*((offset@offset)*np.eye(3)-np.outer(offset,offset))
    eig=np.linalg.eigvalsh(tensor)
    links[group]={'mass_kg':total,'com_link_m':center.tolist(),'inertia_com_link_kg_m2':tensor.tolist(),
        'part_count':len(parts),'read_count':len(valid),'missing_material_count':sum(not r.get('material') for r in parts),
        'inertia_eigenvalues':eig.tolist(),'tensor_sanity':bool(eig.min()>0 and eig[0]+eig[1]>=eig[2]-1e-10),
        'confidence':'provisional_not_for_hardware'}
report={'source':cad['source'],'units':'SI: kg, m, kg m^2','status':'audit_only_not_calibrated',
    'assumptions':['Leaf components only; one rigid grouping per top-level link',
    'Part-level materials queried; body-level materials and mass overrides not independently verified',
    'No payload, cable mass or vendor motor mass calibration added'], 'links':links,'parts':records}
(out/'cad_mass_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(links,indent=2),flush=True)
