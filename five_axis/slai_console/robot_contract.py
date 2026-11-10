"""Offline coordinate contract. URDF owns limits; mapping YAML owns sign/zero.

No serial imports, fallback ranges, clipping or hardware authorization. All
angles returned by model_limits_deg are in model coordinates, not motor space.
"""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / 'src/slaihand_description/urdf/slaihand_four_axis.urdf'
MAPPING_PATH = ROOT / 'src/slaihand_bringup/config/hardware_mapping.yaml'
ACTIVE = (1, 2, 3, 4)


def model_limits_deg(path=MODEL_PATH):
    model = ET.parse(path).getroot()
    result = []
    for i in ACTIVE:
        joints = model.findall(f"joint[@name='slaihand_joint{i}']")
        if len(joints) != 1 or joints[0].get('type') != 'revolute':
            raise ValueError(f'J{i}: expected one canonical revolute joint')
        limits = joints[0].findall('limit')
        if len(limits) != 1:
            raise ValueError(f'J{i}: expected one canonical limit')
        low, high = (float(limits[0].attrib[k]) for k in ('lower', 'upper'))
        if not all(math.isfinite(v) for v in (low, high)) or not low < high:
            raise ValueError(f'J{i}: invalid canonical limits')
        result.append((math.degrees(low), math.degrees(high)))
    return tuple(result)


def operating_limits_deg(margin_deg=0.):
    if not math.isfinite(margin_deg) or margin_deg < 0:
        raise ValueError('Operating margin must be finite and nonnegative')
    result = tuple((low + margin_deg, high - margin_deg)
                   for low, high in model_limits_deg())
    if any(low >= high for low, high in result):
        raise ValueError('Operating margin leaves no travel')
    return result


def load_mapping(path=MAPPING_PATH):
    """Read the three numeric calibration fields in the known local YAML schema."""
    result = {}; active = None
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        if line.startswith('  slaihand_joint') and line.endswith(':'):
            active = int(line.split('slaihand_joint', 1)[1][:-1])
            if active in ACTIVE:
                if active in result:
                    raise ValueError(f'Duplicate mapping J{active}')
                result[active] = {}
        elif active in ACTIVE and line.startswith('    ') and ':' in line and not line.lstrip().startswith('#'):
            key, value = line.strip().split(':', 1)
            if key in ('can_id', 'hardware_sign', 'zero_offset_rad'):
                if key in result[active]:
                    raise ValueError(f'Duplicate mapping field J{active} {key}')
                result[active][key] = float(value.split('#', 1)[0].strip())
    for i in ACTIVE:
        item = result.get(i, {})
        if (item.get('can_id') != i or item.get('hardware_sign') not in (-1, 1)
                or not math.isfinite(item.get('zero_offset_rad', float('nan')))):
            raise ValueError(f'J{i} local software mapping missing or invalid')
    return result


def contract_hashes():
    return {name: hashlib.sha256(path.read_bytes()).hexdigest()
            for name, path in (('model', MODEL_PATH), ('mapping', MAPPING_PATH))}


def audit_contract():
    """Verify active consumers and return a reproducible, offline-only snapshot."""
    import json
    from .core import LIMITS
    from .five_axis_config import load_settings, template_settings, build_envelopes
    limits = model_limits_deg()
    if LIMITS != limits:
        raise ValueError('Model changed since process startup; restart before use')
    mapping = load_mapping()
    settings = load_settings()
    envelopes, active = build_envelopes(settings)
    if active != ACTIVE or settings.j5_enabled:
        raise ValueError('Current four-axis contract requires J5 disabled')
    template = template_settings()
    operating = operating_limits_deg()
    for i in ACTIVE:
        entry = template.axes[i]
        if (entry.travel_min_deg, entry.travel_max_deg) != operating[i-1]:
            raise ValueError(f'J{i}: template not derived from canonical limits')
    scan_path = ROOT/'verification/hardware/load_calibration/measured_limits_20261102.json'
    scan = json.loads(scan_path.read_text(encoding='utf-8'))
    hashes = contract_hashes()
    if scan['mapping_sha256'] != hashes['mapping']:
        raise ValueError('Acquisition coordinate confirmation is stale')
    for axis, entry in scan['joints'].items():
        i = int(axis)
        if i not in ACTIVE or any(entry[k] != mapping[i][k] for k in ('hardware_sign', 'zero_offset_rad')):
            raise ValueError(f'J{i}: acquisition coordinate mismatch')
        low, high = entry['scan_limits_deg']
        if entry.get('measured_limit_override') or not limits[i-1][0] <= low < high <= limits[i-1][1]:
            raise ValueError(f'J{i}: acquisition must narrow canonical limits without override')
    return dict(schema=1, status='OFFLINE_CONSISTENT', hardware_motion_authorized=False,
                hardware_parameters_written=False, j5_enabled=False,
                sources=dict(model=str(MODEL_PATH.relative_to(ROOT)),
                             mapping=str(MAPPING_PATH.relative_to(ROOT))),
                hashes=hashes, formula='q_model = hardware_sign * (q_motor - zero_offset_rad)',
                joints={str(i): dict(mapping[i], outer_limits_deg=limits[i-1],
                    default_operating_limits_deg=operating[i-1],
                    configured_travel_deg=(math.degrees(envelopes[i].lower), math.degrees(envelopes[i].upper)))
                    for i in ACTIVE}, acquisition=scan['joints'])


if __name__ == '__main__':
    import json
    print(json.dumps(audit_contract(), ensure_ascii=False, indent=2, allow_nan=False))
