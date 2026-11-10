"""Convert CAD mesh and verify unchanged assembly-space geometry; no XML authoring."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

root = Path(__file__).parent
fresh = root / 'cad_j4_corrected'
package = root.parent / 'src/slaihand_description'
old = json.loads((root/'cad_j3_corrected/cad_transforms.json').read_text(encoding='utf-8'))['frames']
report = json.loads((fresh/'cad_transforms.json').read_text(encoding='utf-8'))
new = report['frames']
dtype = np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
for name in new:
    previous, revised = np.array(old[name]), np.array(new[name])
    if name != 'link4':
        assert np.allclose(previous, revised, atol=1e-10), f'Unexpected frame change: {name}'
    records = np.fromfile(fresh/(name+'.STL'), dtype=dtype, offset=84)
    vertices, inverse = np.unique(records['vertices'].reshape(-1,3),axis=0,return_inverse=True)
    old_vertices = np.array([list(map(float,line.split()[1:4])) for line in
        (package/'meshes'/(name+'.obj')).read_text(encoding='utf-8').splitlines() if line.startswith('v ')])
    old_world = old_vertices @ previous[:3,:3].T + previous[:3,3]
    new_world = vertices @ revised[:3,:3].T + revised[:3,3]
    error = max(cKDTree(old_world).query(new_world)[0].max(), cKDTree(new_world).query(old_world)[0].max())
    print(name, 'zero-pose geometry residual (m):', error, flush=True)
    if name != 'link4':
        assert error < 2e-6, f'Unexpected geometry change: {name}'
    else:
        print('Link4 changed in latest CAD; replacing with authoritative fresh tessellation.')
    if name == 'link4':
        with (fresh/'link4.obj').open('w',encoding='utf-8',newline='\n') as stream:
            for v in vertices:
                stream.write('v '+' '.join(f'{float(x):.12g}' for x in v)+'\n')
            for f in inverse.reshape(-1,3)+1:
                stream.write('f '+' '.join(map(str,f))+'\n')
relative = np.linalg.inv(new['link3']) @ np.array(new['link4'])
print('J4 xyz:', ' '.join(f'{x:.12g}' for x in relative[:3,3]))
print('J4 rpy:', ' '.join(f'{x:.12g}' for x in Rotation.from_matrix(relative[:3,:3]).as_euler('xyz')))
points = np.array(report['j4_axis_endpoints']).reshape(2,3)
direction = points[1]-points[0]; direction /= np.linalg.norm(direction)
frame = np.array(new['link4'])
alignment = abs(direction @ frame[:3,2])
error = np.linalg.norm(np.cross(frame[:3,3]-points[0],direction))
print('J4 CAD axis alignment:', alignment, 'axis-line residual (m):', error)
assert alignment > 1-1e-10 and error < 1e-8
old_positive = -np.array(old['link4'])[:3,2]
print('Old/new positive direction dot:', old_positive @ -frame[:3,2])
