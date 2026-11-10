"""Read-only probe of the canusb motion audit file. Offline, no hardware access.

Streams /home/xuyu/.ros/slaihand/canusb_five_axis_candidate/canusb_motion_qsrQke.jsonl
(41 MB) and only keeps kind=SESSION and kind=CONTROL_CYCLE with command_id=1 in
memory while printing the schema of each kind.
"""
import json

P = '/home/xuyu/.ros/slaihand/canusb_five_axis_candidate/canusb_motion_qsrQke.jsonl'

kinds = {}
sessions = []
cc_first = None
cc_command_ids = {}
n = 0
with open(P, 'r', encoding='utf-8', errors='replace') as f:
    for line in f:
        n += 1
        line = line.strip()
        if not line:
            continue
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            continue
        k = r.get('kind')
        kinds[k] = kinds.get(k, 0) + 1
        if k == 'SESSION':
            sessions.append(r)
        elif k == 'CONTROL_CYCLE':
            cid = r.get('command_id', r.get('payload', {}).get('command_id'))
            cc_command_ids[cid] = cc_command_ids.get(cid, 0) + 1
            if cc_first is None:
                cc_first = r
print('total lines:', n)
print('kinds:', kinds)
print('CONTROL_CYCLE command_id counts:', cc_command_ids)
print('n SESSION records:', len(sessions))
print()
for s in sessions:
    print('SESSION:', json.dumps(s)[:2000])
print()
print('first CONTROL_CYCLE keys:', list(cc_first.keys()) if cc_first else None)
print(json.dumps(cc_first, indent=1)[:4000] if cc_first else '')
