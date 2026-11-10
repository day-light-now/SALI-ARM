"""Check canonical J4 against CAD and plot endpoint geometry, not collision proof."""
from pathlib import Path
import json
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

root = Path(__file__).parent
package = root.parent/'src/slaihand_description'
old = json.loads((root/'cad_j3_corrected/cad_transforms.json').read_text(encoding='utf-8'))['frames']
new = json.loads((root/'cad_j4_corrected/cad_transforms.json').read_text(encoding='utf-8'))['frames']
robot = ET.parse(package/'urdf/slaihand_four_axis.urdf').getroot()
before = ET.parse(root/'before_j4_correction/slaihand_four_axis.urdf').getroot()
for i in [1,2,3]:
    assert ET.tostring(robot.find(f"joint[@name='slaihand_joint{i}']")) == ET.tostring(before.find(f"joint[@name='slaihand_joint{i}']"))
origin = robot.find("joint[@name='slaihand_joint4']/origin")
t = np.eye(4)
t[:3,3] = list(map(float,origin.get('xyz').split()))
t[:3,:3] = Rotation.from_euler('xyz',list(map(float,origin.get('rpy').split()))).as_matrix()
assert np.allclose(t,np.linalg.inv(new['link3'])@new['link4'],atol=1e-10)

def obj(path):
    lines=path.read_text(encoding='utf-8').splitlines()
    v=np.array([list(map(float,x.split()[1:4])) for x in lines if x.startswith('v ')])
    f=np.array([[int(s.split('/')[0])-1 for s in x.split()[1:4]] for x in lines if x.startswith('f ')])
    return v[f][::max(1,len(f)//16000)]
a=obj(package/'meshes/link3.obj')
b_old=obj(root/'before_j4_correction/link4.obj')
b_new=obj(package/'meshes/link4.obj')
fig=plt.figure(figsize=(14,5))
for n,(title,frames,b,deg) in enumerate([
    ('Old pivot: J4 +50 deg',old,b_old,50),
    ('Corrected pivot: J4 +50 deg',new,b_new,50),
    ('Corrected pivot: J4 -90 deg',new,b_new,-90)],1):
    ax=fig.add_subplot(1,3,n,projection='3d')
    t=np.linalg.inv(frames['link3'])@np.array(frames['link4'])
    r=Rotation.from_rotvec(np.array([0,0,-np.deg2rad(deg)])).as_matrix()
    rotated=b@r.T@t[:3,:3].T+t[:3,3]
    for faces,color in [(a,'#25a34f'),(rotated,'#279dd1')]:
        ax.add_collection3d(Poly3DCollection(faces,facecolor=color,edgecolor='none',alpha=.85))
    ax.set(xlim=(-.18,.18),ylim=(-.18,.18),zlim=(-.05,.31),title=title,xlabel='x (m)',ylabel='y (m)',zlabel='z (m)')
    ax.set_box_aspect([1,1,1]); ax.view_init(elev=20,azim=-45)
fig.suptitle('Green: link3 / Blue: link4. CAD geometry relative to link3. Not a collision test.')
fig.tight_layout(); fig.savefig(root/'j4_pivot_comparison.png',dpi=140)
print('PASS: canonical J4 matches CAD; J1/J2/J3 preserved. Endpoint figure generated.')
