"""Offline decoder for an eight-byte Damiao feedback payload. Never opens a port."""
import argparse
import json
import math


def decode_feedback(payload, pmax=12.5, vmax=25.0, tmax=200.0):
    if len(payload) != 8:
        raise ValueError('Feedback must contain exactly eight bytes')
    if not all(math.isfinite(x) and x > 0 for x in (pmax, vmax, tmax)):
        raise ValueError('Mapping limits must be finite and positive')
    position = ((payload[1] << 8) | payload[2]) * (2 * pmax) / 65535 - pmax
    velocity = ((payload[3] << 4) | (payload[4] >> 4)) * (2 * vmax) / 4095 - vmax
    torque = (((payload[4] & 15) << 8) | payload[5]) * (2 * tmax) / 4095 - tmax
    return dict(payload=payload.hex(' '), motor_id=payload[0] & 15,
                state_code=payload[0] >> 4, position_rad=position,
                position_deg=math.degrees(position), velocity_rad_s=velocity,
                torque_feedback_nm=torque, driver_temp_c=payload[6],
                motor_temp_c=payload[7], mapping_limits=dict(pmax=pmax,vmax=vmax,tmax=tmax))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('payload', help='Eight hexadecimal bytes, quoted and space separated')
    args = parser.parse_args()
    print(json.dumps(decode_feedback(bytes.fromhex(args.payload)), indent=2, ensure_ascii=False))
