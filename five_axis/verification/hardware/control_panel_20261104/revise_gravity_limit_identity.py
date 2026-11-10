"""Carry the fitted predictor across a proven limits-only URDF revision."""
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    source = Path('/home/xuyu/slaihand_control_panel_ws_float_20261105/install/slaihand_description/share/slaihand_description/urdf/slaihand_four_axis.urdf')
    target = ROOT / 'src/slaihand_description/urdf/slaihand_four_axis.urdf'
    profile_path = ROOT / 'src/slaihand_bringup/config/gravity_identified_candidate.json'
    profile = json.loads(profile_path.read_text())
    old_bytes, new_bytes = source.read_bytes(), target.read_bytes()
    assert sha(old_bytes) == profile['identity']['urdf_sha256']
    old, new = ET.fromstring(old_bytes), ET.fromstring(new_bytes)
    old_geometry, new_geometry = copy.deepcopy(old), copy.deepcopy(new)
    changes = []
    for before, after in zip(old_geometry.findall('joint'), new_geometry.findall('joint')):
        assert before.get('name') == after.get('name')
        old_limit, new_limit = before.find('limit'), after.find('limit')
        if old_limit is None:
            continue
        for key in ('lower', 'upper', 'velocity'):
            if old_limit.get(key) != new_limit.get(key):
                assert key == 'velocity' or before.get('name') == 'slaihand_joint4'
                changes.append({'joint': before.get('name'), 'attribute': key,
                                'before': old_limit.get(key), 'after': new_limit.get(key)})
                old_limit.set(key, new_limit.get(key))
    assert ET.tostring(old_geometry) == ET.tostring(new_geometry), 'geometry changed'
    assert len(changes) == 6, changes
    assert float(new.find("joint[@name='slaihand_joint4']/limit").get('lower')) == -0.8726646259971648
    assert float(new.find("joint[@name='slaihand_joint4']/limit").get('upper')) == 1.5707963267948966
    (HERE / 'evidence/gravity_identification_source.urdf').write_bytes(old_bytes)
    profile['compatibility_update'] = {
        'reason': 'User-corrected J4 endpoints and authorised 1rad/s velocity',
        'source_urdf_sha256': sha(old_bytes), 'target_urdf_sha256': sha(new_bytes),
        'kinematics_geometry_and_coefficients_unchanged': True, 'limit_changes': changes,
        'capture_identity_retained_in_original_evidence': True}
    profile['identity']['urdf_sha256'] = sha(new_bytes)
    profile_path.write_text(json.dumps(profile, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(profile['compatibility_update']))


if __name__ == '__main__':
    main()
