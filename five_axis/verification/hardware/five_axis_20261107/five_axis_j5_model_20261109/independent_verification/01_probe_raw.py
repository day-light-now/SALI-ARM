"""Read-only probe of the 2026-11-09 J5 excitation recording. Offline, no hardware.

Prints record kinds, time spans, per-record field layouts so the independent
extraction can be written from the raw format rather than from the lead's code.
"""
import gzip
import json
import collections

P = ('/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain/verification/'
     'hardware/five_axis_20261107/acquisition_20261109/raw/j5_excitation_20261109.jsonl.gz')

kinds = collections.Counter()
first = {}
n = 0
tmin = {}
tmax = {}
with gzip.open(P, 'rt', encoding='utf-8') as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        n += 1
        r = json.loads(line)
        k = r.get('kind')
        kinds[k] += 1
        if k not in first:
            first[k] = r
        t = r.get('t')
        if isinstance(t, (int, float)):
            tmin[k] = min(tmin.get(k, t), t)
            tmax[k] = max(tmax.get(k, t), t)

print('total lines:', n)
print('kinds:', dict(kinds))
print('time spans (recorder t):')
for k in kinds:
    print(f'  {k}: {tmin.get(k)} .. {tmax.get(k)}')
print()
for k, r in first.items():
    print('=' * 30, k, 'first record keys:', list(r.keys()))
    print(json.dumps(r, indent=1)[:2500])
    print()

# controller_state payload key layout and joint_names order
cs = first.get('controller_state')
if cs:
    p = cs['payload']
    print('controller_state payload keys:', list(p.keys()))
    print('joint_names:', p.get('joint_names'))
    for fld in ('reference_positions', 'feedback_positions', 'reference_velocities'):
        if fld in p:
            print(fld, p[fld])

dyn = first.get('dynamic')
if dyn:
    p = dyn['payload']
    print('dynamic payload keys:', list(p.keys()))
    print('dynamic joint_names:', p.get('joint_names'))
    inter = p.get('interfaces')
    if isinstance(inter, dict):
        k0 = list(inter.keys())[0]
        print('interfaces is dict, first key:', k0)
        print(json.dumps(inter[k0], indent=1))
    elif isinstance(inter, list):
        print('interfaces is list len', len(inter))
        print(json.dumps(inter[0], indent=1)[:1500])
    else:
        print('interfaces type', type(inter))
