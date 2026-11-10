"""Check prerequisites for the legacy four-axis gravity-float drag mode."""
import hashlib
import json
from pathlib import Path

import yaml

ROOT = Path('/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain')
CONFIG = ROOT / 'src/slaihand_bringup/config'
MAPPING = CONFIG / 'hardware_mapping.yaml'
POLICY = CONFIG / 'canusb_motion_policy.yaml'
CANDIDATE = CONFIG / 'gravity_identified_candidate.json'
PROVISIONAL = CONFIG / 'gravity_provisional.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    policy = yaml.safe_load(POLICY.read_text(encoding='utf-8'))
    print('policy:', {k: policy.get(k) for k in
                      ('profile', 'load_id', 'kp', 'kd', 'gravity_ff_alpha', 'gravity_float_enabled')})
    print('mapping sha', sha(MAPPING))
    for name, path in (('candidate', CANDIDATE), ('provisional', PROVISIONAL)):
        if not path.is_file():
            print(name, 'MISSING', path)
            continue
        profile = json.loads(path.read_text(encoding='utf-8'))
        identity = profile.get('identity') or {}
        print(name, 'schema', profile.get('schema'))
        print('  identity mapping_sha256', identity.get('mapping_sha256'))
        print('  identity load_id', identity.get('load_id'))
        print('  training_gains', profile.get('training_gains'))
        print('  mapping match', identity.get('mapping_sha256') == sha(MAPPING))
        print('  load match', identity.get('load_id') == policy.get('load_id'))
        print('  gains match', all(profile.get('training_gains', {}).get(k) == policy.get(k)
                                   for k in ('kp', 'kd')))
        print('  file sha', sha(path))


if __name__ == '__main__':
    main()
