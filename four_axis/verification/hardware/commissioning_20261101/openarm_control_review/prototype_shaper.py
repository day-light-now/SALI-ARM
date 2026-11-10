"""Scratch: compare torque-shaping candidates on a stick-slip + inertia plant.

Not product code and not imported by the console. It chooses bounded parameters
before the design node. Run with:
  python verification/hardware/commissioning_20261101/openarm_control_review/prototype_shaper.py
"""
import math


def minjerk(u):
    u = min(1., max(0., u))
    return 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5


def step_plant(q, v, tau, *, fs=1.2, fk=.5, J=.05, b=.05, dt=.01):
    if abs(v) < 1e-6:
        if abs(tau) <= fs:
            return q, 0.
        v = math.copysign(1e-5, tau)
    a = (tau - b * v - math.copysign(fk, v)) / J
    v += a * dt
    if tau * v < 0 and abs(v) < 2e-4:
        v = 0.
    q += v * dt
    return q, v


def run(profile, plant, *, duration=18., kp=32., kd=.5, cap=2.,
        abort_speed=.2, following_error_deg=4., dt=.01):
    q, v, target, out = 0., 0., 0., 0.
    t_motion = None
    peak_v = 0.
    history = []
    startup_stop = None
    for k in range(int(duration / dt)):
        t = k * dt
        u = min(1., t / duration)
        target = math.radians(10) * minjerk(u)
        v_ref = math.radians(10) * 30 * u * u * (1 - u) ** 2 / duration
        e = target - q
        demand = kp * e + kd * (v_ref - v)
        demand = max(-cap, min(cap, demand))
        if profile.get('brake_gain'):
            excess = abs(v) - max(abs(v_ref), profile.get('brake_deadband', .01))
            if excess > 0:
                demand -= math.copysign(min(profile['brake_cap'],
                                            profile['brake_gain'] * excess), v)
                demand = max(-cap, min(cap, demand))
        if (profile.get('startup_cap') is not None and t_motion is None):
            demand = max(-profile['startup_cap'], min(profile['startup_cap'], demand))
            if t >= profile.get('startup_window', 3.):
                startup_stop = t
                break
        if profile.get('release_guard') and t_motion is not None and v * v_ref >= 0 and abs(v) > profile['release_guard']:
            demand = 0.
        # velocity-aware down rate: fast only when actually overspeeding
        down = profile['down']
        if profile.get('smart_down') and not (abs(v) > max(abs(v_ref), .02) and v * v_ref >= 0):
            down = profile.get('down_slow', down)
        out += max(-down * dt, min(profile['up'] * dt, demand - out))
        out = max(-cap, min(cap, out))
        q, v = step_plant(q, v, out, **plant)
        if t_motion is None and abs(v) > .02:
            t_motion = t
        if t_motion is not None:
            peak_v = max(peak_v, abs(v))
        history.append((t, out, v, math.degrees(q), math.degrees(e)))
        if abs(v) > abort_speed:
            return dict(stop='speed', t=t, v=v, q=q, peak_v=peak_v, t_motion=t_motion,
                        out=out, history=history)
        if abs(math.degrees(e)) > following_error_deg:
            return dict(stop='error', t=t, v=v, q=q, peak_v=peak_v, t_motion=t_motion,
                        out=out, history=history)
    return dict(stop='startup_timeout' if startup_stop else 'done',
                t=startup_stop or duration, v=v, q=q, peak_v=peak_v,
                t_motion=t_motion, out=out, history=history)


def coast(plant, result, *, coast_s=.5, dt=.01):
    """FD: torque is zero from the first post-stop cycle; friction/inertia only."""
    q, v = result['q'], result['v']
    for _ in range(int(coast_s / dt)):
        q, v = step_plant(q, v, 0., **plant)
        if abs(v) < 1e-4:
            break
    return dict(coast_end_deg=math.degrees(q - result['q']), coast_end_v=v)


def show(name, profile, plant):
    r = run(profile, plant)
    c = coast(plant, r)
    print('%-10s stop=%-15s t=%5.2f v=%6.3f q=%8.4fdeg peak=%6.3f motion_t=%s out=%6.3f coast=%7.4fdeg' % (
        name, r['stop'], r['t'], r['v'], math.degrees(r['q']), r['peak_v'],
        None if r['t_motion'] is None else round(r['t_motion'], 2), r['out'],
        c['coast_end_deg']))
    return r, c


def main():
    variants = {
        'old': dict(up=1., down=1.),
        'fastdown': dict(up=1., down=10.),
        'smartdown': dict(up=1., down=10., smart_down=True, down_slow=1.),
        'smart+brake': dict(up=1., down=10., smart_down=True, down_slow=1.,
                            brake_gain=2., brake_cap=.4, brake_deadband=.02),
        'smart+cap': dict(up=1., down=10., smart_down=True, down_slow=1.,
                          startup_cap=1.0, startup_window=3.),
    }
    for fs in (.9, 1.2):
        print('\n== plant fs=%.2f fk=%.2f J=.05 ==' % (fs, max(.1, fs - .4)))
        for name, prof in variants.items():
            show(name, prof, dict(fs=fs, fk=max(.1, fs - .4), J=.05, b=.05))


if __name__ == '__main__':
    main()
