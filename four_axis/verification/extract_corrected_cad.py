"""Read current SolidWorks tessellation; preserve its assembly transforms."""
from pathlib import Path
import json
import struct
import numpy as np
import win32com.client

ROOT = Path(__file__).parent
OUT = ROOT / 'cad_j3_corrected'
OUT.mkdir(exist_ok=True)
sw = win32com.client.GetActiveObject('SldWorks.Application')
doc = sw.ActiveDoc
if doc is None or not doc.GetPathName.lower().endswith('slaihand_four_axis.sldasm'):
    raise RuntimeError('The active document must be the SLAI four-axis assembly')

def transform(value):
    a = np.array(value.ArrayData)
    t = np.eye(4)
    t[:3, :3] = a[:9].reshape(3, 3).T * a[12]
    t[:3, 3] = a[9:12]
    return t

frames = {name: transform(doc.Extension.GetCoordinateSystemTransformByName(cs))
          for name, cs in [('base_link','base_link_cs'),('link1','base_link_cs'),
                           ('link2','J2_cs'),('link3','J3'),('link4','J4_cs')]}
groups = {name: [] for name in frames}
for component in doc.GetComponents(False):
    print('Reading', ascii(component.Name2), flush=True)
    if component.GetChildren:
        continue
    model = component.GetModelDoc2
    if model is None:
        continue
    parent = component.Name2.split('/')[0].split('-')[0]
    name = 'base_link' if parent == 'base' else parent
    if name not in groups:
        continue
    raw = model.GetTessTriangles(True)
    if not raw:
        continue
    triangles = np.asarray(raw).reshape(-1,3)
    t = np.linalg.inv(frames[name]) @ transform(component.Transform2)
    transformed = triangles @ t[:3,:3].T + t[:3,3]
    groups[name].append(transformed.reshape(-1,3,3))

report = {'source': doc.GetPathName, 'frames': {k:v.tolist() for k,v in frames.items()}, 'meshes': {}}
for name, parts in groups.items():
    tris = np.concatenate(parts)
    records = np.zeros(len(tris), dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')]))
    normals = np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0])
    lengths = np.linalg.norm(normals,axis=1)
    normals /= np.maximum(lengths[:,None],1e-30)
    records['normal'] = normals
    records['vertices'] = tris
    (OUT / (name+'.STL')).write_bytes(b'SLAI CAD tessellation'.ljust(80,b' ')+struct.pack('<I',len(tris))+records.tobytes())
    vertices = tris.reshape(-1,3)
    report['meshes'][name] = {'triangles':len(tris),'min':vertices.min(axis=0).tolist(),'max':vertices.max(axis=0).tolist()}
(OUT / 'cad_transforms.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report['meshes'],indent=2),flush=True)
