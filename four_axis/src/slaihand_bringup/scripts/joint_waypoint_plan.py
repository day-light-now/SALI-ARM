"""Finite four-axis plans; degrees at the boundary, radians for ROS."""
import math

REFERENCE_SPEED_RAD_S = 1.0


def compile_plan(document, start, bounds):
    if set(document) != {'coordinates', 'waypoints'}:
        raise ValueError('计划必须且仅包含 coordinates、waypoints')
    mode = document['coordinates']
    if mode not in ('absolute_deg', 'relative_start_deg'):
        raise ValueError('coordinates 必须是 absolute_deg 或 relative_start_deg')
    rows = document['waypoints']
    if not isinstance(rows, list) or not 1 <= len(rows) <= 12:
        raise ValueError('需要1～12个路点')
    previous = list(start)
    points = [(0., previous)]
    windows = []
    at = 0.
    for row in rows:
        if set(row) != {'degrees', 'move_s', 'dwell_s'}:
            raise ValueError('每点必须且仅包含 degrees、move_s、dwell_s')
        values = row['degrees']
        if not isinstance(values, list) or len(values) != 4:
            raise ValueError('每点需要J1～J4四个角度，不支持J5')
        numbers = values + [row['move_s'], row['dwell_s']]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in numbers):
            raise ValueError('角度与时间必须为有限数值')
        move, dwell = row['move_s'], row['dwell_s']
        if move <= 0 or dwell < .5:
            raise ValueError('move_s须大于0，dwell_s至少0.5秒')
        target = [math.radians(v) + (q if mode == 'relative_start_deg' else 0.) for v, q in zip(values, start)]
        if any(not lo <= q <= hi for q, (lo, hi) in zip(target, bounds)):
            raise ValueError('路点超出完整URDF限位；不自动裁剪')
        if any(1.875 * abs(q - p) / move > REFERENCE_SPEED_RAD_S + 1e-12 for p, q in zip(previous, target)):
            raise ValueError(f'五次插值参考峰值速度超过{REFERENCE_SPEED_RAD_S:g}rad/s；增加move_s')
        at += move
        points.append((at, target))
        windows.append({'arrival_s': at, 'end_s': at + dwell, 'target_rad': target})
        at += dwell
        points.append((at, target))
        previous = target
    return points, windows


def assess_waypoints(windows, samples, tolerance_deg=.5):
    results = []
    for window in windows:
        # Require observable settling at each waypoint, not just a successful action.
        rows = [(t, q) for t, q in samples if window['end_s'] - .3 <= t <= window['end_s']]
        span = rows[-1][0] - rows[0][0] if rows else 0.
        errors = [max(abs(math.degrees(q[i] - window['target_rad'][i])) for _, q in rows) for i in range(4)] if rows else None
        results.append(dict(window, samples=len(rows), sampled_span_s=span, max_error_deg=errors,
                            passed=span >= .2 and errors is not None and max(errors) <= tolerance_deg))
    return results
