"""Evaluate the provisional gravity candidate offline, with no hardware route."""
import argparse
import json
import numpy as np
from identify_payload import Geometry, HERE, read, sha


def evaluate(degrees):
    candidate=read(HERE/'offline_ff_candidate.json')
    if candidate['source_result_sha256']!=sha(HERE/'identification_result.json'):
        raise ValueError('candidate/result fingerprint mismatch')
    q=np.radians(np.array(degrees,dtype=float))
    if q.shape!=(4,) or not np.isfinite(q).all():
        raise ValueError('require four finite model angles')
    bounds=np.array(candidate['observed_q_bounds_rad'])
    if np.any(q<bounds[:,0]-1e-12) or np.any(q>bounds[:,1]+1e-12):
        raise ValueError('outside recorded angle bounds; no extrapolation')
    matrix=Geometry().regressor(q)
    theta=np.array(candidate['theta_total_kg_and_kg_m'])
    payload=np.zeros(16);payload[12]=candidate['payload_mass_assumed_kg']
    payload[13:16]=candidate['payload_first_moment_kg_m']
    torque=matrix@theta
    return {'offline_only':True,'deployable':False,'q_model_deg':list(degrees),
            'total_holding_torque_model_nm':torque.tolist(),
            'added_box_holding_torque_model_nm':(matrix@payload).tolist(),
            'total_holding_torque_motor_nm':(torque*candidate['motor_sign']).tolist(),
            'warning':'An angle bounding box is not validated pose coverage; this candidate failed the independent precision screen.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--degrees',type=float,nargs=4,required=True)
    args=parser.parse_args()
    print(json.dumps(evaluate(args.degrees),indent=2))
