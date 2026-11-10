"""CAD-derived before/after geometry and coordinate regression check."""
import json
from pathlib import Path
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

root = Path(__file__).parent
package = root.parent / 'src/slaihand_description'
old = json.loads((root.parent.parent/'slaihand_four_axis_sim/cad_verified/cad_transforms.json').read_text(encoding='utf-8'))['frames']
new = json.loads((root/'cad_j3_corrected/cad_transforms.json').read_text(encoding='utf-8'))['frames']
robot = ET.parse(package/'urdf/slaihand_four_axis.urdf').getroot()
for name, parent, child in [('slaihand_joint3','link2','link3'),('slaihand_joint4','link3','link4')]:
    origin = robot.find(f"joint[@name='{name}']/origin")
    t = np.eye(4)
    t[:3,3] = list(map(float,origin.get('xyz').split()))
    t[:3,:3] = Rotation.from_euler('xyz',list(map(float,origin.get('rpy').split()))).as_matrix()
    assert np.allclose(t,np.linalg.inv(new[parent])@new[child],atol=1e-10)
# CAD reference axis is defined by its two assembly-space endpoints.
axis_points = np.array([[0.1350919560993038,-0.12918296344012073,0.30415895091689343],
    [0.1350137586705366,-0.1868449001298062,0.30415895091689366]])
direction = axis_points[1]-axis_points[0]; direction /= np.linalg.norm(direction)
frame = np.array(new['link3'])
assert abs(np.dot(direction,frame[:3,2])) > 1-1e-10
error = np.linalg.norm(np.cross(frame[:3,3]-axis_points[0],direction))
assert error < 1e-8, error
print('PASS: URDF J3/J4 match fresh CAD; J3 axis-line residual (m)',error)

def obj(path):
    lines=path.read_text(encoding='utf-8').splitlines()
    v=np.array([list(map(float,x.split()[1:4])) for x in lines if x.startswith('v ')])
    f=np.array([[int(s.split('/')[0])-1 for s in x.split()[1:4]] for x in lines if x.startswith('f ')])
    return v[f]
a=obj(package/'meshes/link2.obj')
b_old=obj(root/'before_j3_correction/link3.obj')
b_new=obj(package/'meshes/link3.obj')
fig=plt.figure(figsize=(14,5))
for n,(title,frames,b,deg) in enumerate([
    ('Old pivot: J3 +90 deg',old,b_old,90),
    ('Corrected pivot: J3 +90 deg',new,b_new,90),
    ('Corrected pivot: J3 -90 deg',new,b_new,-90)],1):
    ax=fig.add_subplot(1,3,n,projection='3d')
    t=np.linalg.inv(np.array(frames['link2']))@np.array(frames['link3'])
    r=Rotation.from_rotvec(np.array([0,0,-np.deg2rad(deg)])).as_matrix()
    rotated=b@r.T@t[:3,:3].T+t[:3,3]
    for faces,color in [(a,'#d9b32c'),(rotated,'#25a34f')]:
        ax.add_collection3d(Poly3DCollection(faces,facecolor=color,edgecolor='none',alpha=0.85))
    ax.set(xlim=(-.12,.3),ylim=(-.18,.24),zlim=(-.24,.18),title=title,xlabel='x (m)',ylabel='y (m)',zlabel='z (m)')
    ax.set_box_aspect([1,1,1]);ax.view_init(elev=20,azim=-45)
fig.suptitle('Yellow: link2 / Green: link3. CAD geometry, relative to link2. Not a collision test.')
fig.tight_layout();fig.savefig(root/'j3_pivot_comparison.png',dpi=140)
