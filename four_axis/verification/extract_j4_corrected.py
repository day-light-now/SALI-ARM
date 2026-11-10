"""Read-only CAD capture; reuse the established tessellation exporter."""
from pathlib import Path
import win32com.client

doc = win32com.client.GetActiveObject('SldWorks.Application').ActiveDoc
features = []
feature = doc.FirstFeature
while feature:
    if feature.GetTypeName2 == 'CoordSys':
        features.append(feature.Name)
    feature = feature.GetNextFeature
candidates = [n for n in features if n not in ['base_link_cs', 'J2_cs', 'J3', 'J5_cs']]
if len(candidates) != 1:
    raise RuntimeError(f'Cannot identify J4 coordinate system: {features!r}')
source = Path(__file__).with_name('extract_corrected_cad.py').read_text(encoding='utf-8')
source = source.replace("'cad_j3_corrected'", "'cad_j4_corrected'")
source = source.replace("('link4','J4_cs')", f"('link4',{candidates[0]!r})")
print('J4 coordinate:', ascii(candidates[0]), flush=True)
axis = doc.FeatureByName('J4_axis').GetSpecificFeature2.GetRefAxisParams
print('J4 axis:', axis, flush=True)
exec(compile(source, str(Path(__file__)), 'exec'))
import json
p = OUT / 'cad_transforms.json'
report = json.loads(p.read_text(encoding='utf-8'))
report['j4_coordinate_name'] = candidates[0]
report['j4_axis_endpoints'] = list(axis)
p.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
