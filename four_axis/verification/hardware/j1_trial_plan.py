"""Offline J1 trial proposal. No serial imports, writes, enable or motion.

Source: https://github.com/dmBots/DM-J10010L-2EC/manual
Mode 4 frame: little-endian float position, uint16 speed*100,
uint16 current fraction*10000. Manual current base: 99.74 A.
This prepares bytes only; it does not establish firmware safety behavior.
"""
import argparse
import json
import math
import struct


def plan(start_rad, forward_deg=2.0):
    if not math.isfinite(start_rad) or abs(start_rad) > 12.4:
        raise ValueError('Invalid starting encoder position')
    if not math.isfinite(forward_deg) or not 0 < forward_deg <= 2:
        raise ValueError('Forward trial must be within (0, 2] degrees')
    target = start_rad + math.radians(forward_deg)
    if abs(target) > 12.5:
        raise ValueError('Target exceeds verified feedback codec range')
    speed_units = 2  # 0.02 rad/s, approximately 1.15 degrees/s
    current_units = math.floor(0.5 / 99.74 * 10000)
    payload = struct.pack('<fHH', target, speed_units, current_units)
    return {
        'offline_only': True,
        'start_encoder_rad': start_rad,
        'target_encoder_rad': target,
        'human_forward_deg': forward_deg,
        'model_angle_change_deg': -forward_deg,
        'requested_speed_rad_s': speed_units / 100,
        'requested_current_limit_a': current_units / 10000 * 99.74,
        'proposed_timeout_count': 2000,
        'nominal_timeout_ms': 100,
        'control_mode_required': 4,
        'can_id': '0x301',
        'payload': payload.hex(' '),
        'live_prerequisites': [
            'Mechanically support arm; motor power cutoff within reach',
            'Fresh disabled feedback and exact RID80 position before mode change',
            'Write and read back timeout while disabled; no flash save',
            'Validate timeout actually disables output at stationary low current',
            'Read back mode4 and preload fresh hold-position command',
            'Use bounded runtime and stale-feedback shutdown; no automatic current escalation',
        ],
    }


def self_test():
    p = plan(1.421950)
    position, speed, current = struct.unpack('<fHH', bytes.fromhex(p['payload']))
    assert abs(position - p['target_encoder_rad']) < 1e-6
    assert speed == 2 and current == 50
    assert p['requested_current_limit_a'] <= .5
    assert p['model_angle_change_deg'] == -2
    for start, delta in [(float('nan'), 2), (1, 3), (1, -2), (12.49, 2)]:
        try:
            plan(start, delta)
        except ValueError:
            continue
        raise AssertionError('Invalid trial accepted')
    print('Offline trial bounds, direction, mode4 encoding and current cap: PASS')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start-rad', type=float)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
    elif args.start_rad is not None:
        print(json.dumps(plan(args.start_rad), ensure_ascii=False, indent=2))
    else:
        parser.error('Supply --start-rad or --self-test; this program never connects to hardware')
