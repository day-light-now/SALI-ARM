"""Read-only local reproduction of the two Codex counterexamples.

It reimplements the OLD shape_torque branch (before the rework) and compares it
with the current production function. No serial, no controller, no writes.

Run:
  python verification/hardware/commissioning_20261101/openarm_control_review/Codex返工反例复算.py
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from slai_console.actuation import PD_TIERS
from slai_console.j4_control import shape_torque


def old_shape(*, requested, previous, dt, cap, control, tracking,
              motion_confirmed, model_velocity, reference_velocity):
    """The exact pre-rework branch: startup cap, brake, then symmetric ramp."""
    def clamp(v, limit):
        return max(-limit, min(limit, v))
    torque = clamp(requested, cap)
    startup = tracking and not motion_confirmed
    if startup:
        torque = clamp(torque, control.startup_cap_nm)
    brake = 0.
    if tracking:
        excess = abs(model_velocity) - max(abs(reference_velocity),
                                           control.brake_deadband_rad_s)
        if excess > 0:
            brake = min(control.brake_cap_nm,
                        control.brake_gain_nm_per_rad_s * excess)
            torque = clamp(torque - math.copysign(brake, model_velocity), cap)
    increasing = torque * previous >= 0 and abs(torque) > abs(previous)
    rate = control.slew_up_nm_s if increasing else control.slew_down_nm_s
    delta = max(-rate * dt, min(rate * dt, torque - previous))
    return dict(torque=clamp(previous + delta, cap), pre_slew=torque,
                rate_nm_s=rate, release=not increasing, brake_nm=brake)


control = PD_TIERS['kp32_2nm_brake_candidate'].control

zero_old = old_shape(requested=-2., previous=.01, dt=.01, cap=2.,
                     control=control, tracking=True, motion_confirmed=True,
                     model_velocity=0., reference_velocity=0.)
zero_new = shape_torque(requested=-2., previous=.01, dt=.01, cap=2.,
                        control=control, tracking=True, motion_confirmed=True,
                        model_velocity=0., reference_velocity=0.)
print('zero-crossing old:', {k: round(v, 6) if isinstance(v, float) else v
                             for k, v in zero_old.items()})
print('zero-crossing new:', {k: round(v, 6) if isinstance(v, float) else v
                             for k, v in zero_new.items()})

startup_old = old_shape(requested=-1., previous=-1., dt=.04, cap=2.,
                        control=control, tracking=True, motion_confirmed=False,
                        model_velocity=.19, reference_velocity=0.)
startup_new = shape_torque(requested=-1., previous=-1., dt=.04, cap=2.,
                           control=control, tracking=True, motion_confirmed=False,
                           model_velocity=.19, reference_velocity=0.)
print('startup old:', {k: round(v, 6) if isinstance(v, float) else v
                       for k, v in startup_old.items()})
print('startup new:', {k: round(v, 6) if isinstance(v, float) else v
                       for k, v in startup_new.items()})

assert abs(zero_old['torque'] + .09) < 1e-12, zero_old
assert zero_old['rate_nm_s'] == 10. and zero_old['release'] is True
assert abs(startup_old['pre_slew'] + 1.28) < 1e-12, startup_old
assert abs(startup_old['torque'] + 1.04) < 1e-12, startup_old
assert abs(zero_new['torque'] + .009) < 1e-12, zero_new
assert zero_new['rate_nm_s'] == 1. and zero_new['reverse'] is True
assert abs(startup_new['pre_slew'] + 1.0) < 1e-12, startup_new
assert abs(startup_new['torque'] + 1.0) < 1e-12, startup_new
print('both old values match Codex; both new values are bounded as designed')
