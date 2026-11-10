"""Versioned finite joint plans; degrees at the boundary, radians for ROS.

Two explicit plan versions exist:

* ``slaihand_joint_plan/1`` - the historical four-axis document.  A document
  without a ``schema`` key is *also* read as this legacy version so old saved
  plans keep working, but that inference is recorded by ``normalize_document``
  and never silently promotes a four-axis plan to five axes.
* ``slaihand_joint_plan/2`` - the unified five-axis document.  It must declare
  the version and every waypoint must carry exactly five angles.  A five-axis
  plan can never be executed by a four-axis caller and a legacy plan can never
  be padded/truncated into five axes.
"""
import math

REFERENCE_SPEED_RAD_S = 1.0

SCHEMA = "slaihand_joint_plan/2"
LEGACY_SCHEMA = "slaihand_joint_plan/1"
SCHEMA_AXES = {LEGACY_SCHEMA: 4, SCHEMA: 5}
FOUR_AXES = 4
FIVE_AXES = 5
SUPPORTED_AXES = (FOUR_AXES, FIVE_AXES)
FOUR_AXIS_JOINT_NAMES = tuple("slaihand_joint%d" % index for index in range(1, 5))
FIVE_AXIS_JOINT_NAMES = tuple("slaihand_joint%d" % index for index in range(1, 6))
JOINT_NAMES = FIVE_AXIS_JOINT_NAMES
AXIS_LABELS = {4: "J1～J4", 5: "J1～J5"}


def joint_names(count):
    """Canonical joint order for a supported axis count; never guess."""
    if count == FOUR_AXES:
        return FOUR_AXIS_JOINT_NAMES
    if count == FIVE_AXES:
        return FIVE_AXIS_JOINT_NAMES
    raise ValueError("只支持4轴或5轴，收到 %r" % (count,))


def schema_for_axes(count):
    if count == FOUR_AXES:
        return LEGACY_SCHEMA
    if count == FIVE_AXES:
        return SCHEMA
    raise ValueError("只支持4轴或5轴，收到 %r" % (count,))


def axes_for_schema(schema):
    try:
        return SCHEMA_AXES[schema]
    except KeyError:
        raise ValueError("未知计划版本 %r；只接受 %s（四轴）或 %s（五轴）"
                         % (schema, LEGACY_SCHEMA, SCHEMA))


def _document_shape(document):
    if not isinstance(document, dict):
        raise ValueError("计划必须是 JSON 对象")
    schema = document.get("schema")
    if schema is None:
        # Explicit legacy inference.  The caller can see it in the normalised
        # document; a five-axis caller still rejects it by dimension.
        schema = LEGACY_SCHEMA
        allowed = {"coordinates", "waypoints"}
    elif schema in SCHEMA_AXES:
        allowed = {"schema", "coordinates", "waypoints"}
    else:
        raise ValueError("未知计划版本 %r；只接受 %s（四轴）或 %s（五轴）"
                         % (schema, LEGACY_SCHEMA, SCHEMA))
    if set(document) != allowed:
        raise ValueError("计划必须且仅包含 %s" % "、".join(sorted(allowed)))
    return schema, axes_for_schema(schema)


def document_shape(document):
    """Return ``(schema, axes)``; legacy documents are explicitly tagged v1."""
    return _document_shape(document)


def compile_plan(document, start, bounds):
    schema, axes = _document_shape(document)
    start = list(start)
    bounds = list(bounds)
    if len(start) != axes:
        raise ValueError("计划版本 %s 要求%d轴，但起点反馈有%d轴；不补0、不截断"
                         % (schema, axes, len(start)))
    if len(bounds) != axes:
        raise ValueError("计划版本 %s 要求%d轴限位，但限位源有%d轴；拒绝执行"
                         % (schema, axes, len(bounds)))
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
        if not isinstance(row, dict) or set(row) != {'degrees', 'move_s', 'dwell_s'}:
            raise ValueError('每点必须且仅包含 degrees、move_s、dwell_s')
        values = row['degrees']
        if not isinstance(values, list) or len(values) != axes:
            if axes == FOUR_AXES:
                raise ValueError('每点需要J1～J4四个角度；五轴计划必须显式声明 schema=%s'
                                 '并提供五个角度' % SCHEMA)
            raise ValueError('五轴计划每点需要J1～J5五个角度（schema=%s）' % SCHEMA)
        numbers = values + [row['move_s'], row['dwell_s']]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               for v in numbers):
            raise ValueError('角度与时间必须为有限数值')
        move, dwell = row['move_s'], row['dwell_s']
        if move <= 0 or dwell < .5:
            raise ValueError('move_s须大于0，dwell_s至少0.5秒')
        target = [math.radians(v) + (q if mode == 'relative_start_deg' else 0.)
                  for v, q in zip(values, start)]
        if any(not lo <= q <= hi for q, (lo, hi) in zip(target, bounds)):
            raise ValueError('路点超出完整URDF限位；不自动裁剪')
        if any(1.875 * abs(q - p) / move > REFERENCE_SPEED_RAD_S + 1e-12
               for p, q in zip(previous, target)):
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
        width = len(window['target_rad'])
        errors = [max(abs(math.degrees(q[i] - window['target_rad'][i])) for _, q in rows)
                  for i in range(width)] if rows else None
        results.append(dict(window, samples=len(rows), sampled_span_s=span, max_error_deg=errors,
                            passed=span >= .2 and errors is not None and max(errors) <= tolerance_deg))
    return results
