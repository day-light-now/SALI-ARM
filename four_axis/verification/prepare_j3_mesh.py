"""Convert fresh CAD tessellation to OBJ and report frame/zero-pose checks.
Does not author URDF XML. STL geometry is in metres in its owning CAD frame.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

root = Path(__file__).parent
fresh = root / 'cad_j3_corrected'
current = root.parent / 'src/slaihand_description/meshes'
old = json.loads((root.parent.parent / 'slaihand_four_axis_sim/cad_verified/cad_transforms.json').read_text(encoding='utf-8'))
new = json.loads((fresh / 'cad_transforms.json').read_text(encoding='utf-8'))
dtype = np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
for name in new['frames']:
    previous = np.array(old['frames'][name])
    revised = np.array(new['frames'][name])
    if name != 'link3':
        assert np.allclose(previous, revised, atol=1e-10), f'Unexpected frame change: {name}'
    records = np.fromfile(fresh / (name+'.STL'), dtype=dtype, offset=84)
    vertices, inverse = np.unique(records['vertices'].reshape(-1,3),axis=0,return_inverse=True)
    old_vertices = np.array([list(map(float,line.split()[1:4])) for line in
        (current / (name+'.obj')).read_text(encoding='utf-8').splitlines() if line.startswith('v ')])
    old_world = old_vertices @ previous[:3,:3].T + previous[:3,3]
    new_world = vertices @ revised[:3,:3].T + revised[:3,3]
    # Compare geometry in the assembly/world frame, independent of reframing.
    from scipy.spatial import cKDTree
    distance, _ = cKDTree(old_world).query(new_world)
    print(name, 'zero-pose vertex mismatch max (m):', float(distance.max()), flush=True)
    assert distance.max() < 2e-6, f'CAD geometry changed for {name}; inspect before replacement'
    if name == 'link3':
        with (fresh / (name+'.obj')).open('w',encoding='utf-8',newline='\n') as stream:
            for v in vertices:
                stream.write('v '+' '.join(f'{float(x):.12g}' for x in v)+'\n')
            for f in inverse.reshape(-1,3)+1:
                stream.write('f '+' '.join(map(str,f))+'\n')
for parent, child in [('link2','link3'),('link3','link4')]:
    relative = np.linalg.inv(np.array(new['frames'][parent])) @ np.array(new['frames'][child])
    print(child, 'xyz', ' '.join(f'{x:.12g}' for x in relative[:3,3]),
          'rpy', ' '.join(f'{x:.12g}' for x in Rotation.from_matrix(relative[:3,:3]).as_euler('xyz')))
change = np.linalg.inv(np.array(old['frames']['link3'])) @ np.array(new['frames']['link3'])
print('J3 old-to-new (m):', change.tolist())
