"""Analyze signed controller-state dwell data (local files only).

This revision addresses the review findings:

* a static window is selected using both reference and feedback velocity, not
  merely "adjacent reference positions are close";
* the whole-window reference span is reported and validated;
* the origin is the first validated dwell, not a moving sample;
* PD-request estimates use an explicitly supplied constant Kp or angle
  schedule, never a hidden fixed Kp=160 for schedule runs.

The values remain estimates from ``Kp*(reference-feedback)``; they are not
measured current or torque.
"""
from __future__ import annotations

import argparse
import gzip
import json
import math
from pathlib import Path

JOINTS = ("slaihand_joint1", "slaihand_joint2", "slaihand_joint3",
          "slaihand_joint4", "slaihand_joint5")
STATIC_REF_SPEED = 0.0001     # rad/s; a dwell must have near-zero reference velocity
STATIC_FEEDBACK_SPEED = 0.05  # rad/s; only for static-window selection
REF_SPAN_LIMIT = 0.0015       # rad (~0.086 deg) across the whole window


def _ordered(values, names):
    by_name = dict(zip(names, values))
    return [float(by_name[name]) for name in JOINTS]


def _read_text(path):
    path = Path(path)
    if path.suffix == ".gz":
        with gzip.open(path, "rt", encoding="utf-8") as stream:
            return stream.read()
    return path.read_text(encoding="utf-8")


def load_controller(path):
    rows = []
    for line in _read_text(path).splitlines():
        record = json.loads(line)
        if record.get("kind") != "controller_state":
            continue
        payload = record["payload"]
        names = payload["joint_names"]
        try:
            reference = _ordered(payload["reference_positions"], names)
            feedback = _ordered(payload["feedback_positions"], names)
        except KeyError:
            continue
        ref_velocity = payload.get("reference_velocities")
        fb_velocity = payload.get("feedback_velocities")
        if not isinstance(ref_velocity, list) or not isinstance(fb_velocity, list):
            raise ValueError("controller_state lacks velocity fields")
        rows.append({"t": float(record["t"]), "ref": reference, "fb": feedback,
                     "ref_v": _ordered(ref_velocity, names),
                     "fb_v": _ordered(fb_velocity, names)})
    rows.sort(key=lambda row: row["t"])
    if len(rows) < 10:
        raise ValueError("too few controller_state samples")
    return rows


def _max_abs(values):
    return max(abs(value) for value in values)


def _window_is_static(window):
    if not window:
        return False
    span = max(max(row["ref"][i] for row in window) -
               min(row["ref"][i] for row in window) for i in range(5))
    if span > REF_SPAN_LIMIT:
        return False
    return all(_max_abs(row["fb_v"]) <= STATIC_FEEDBACK_SPEED for row in window)


def stable_windows(rows, window_s):
    """Find plateau suffixes whose whole reference window is stationary.

    A window is valid only when all five reference positions stay within the
    span limit for the whole window and all feedback velocities are low. This
    avoids the earlier "adjacent samples look close" merge through slow motion.
    """
    windows, current = [], []
    for index, row in enumerate(rows):
        earliest = row["t"] - window_s
        window = [candidate for candidate in rows[max(0, index - 400):index + 1]
                  if candidate["t"] >= earliest]
        if len(window) >= 10 and _window_is_static(window):
            current.append((index, window))
        else:
            if current:
                windows.append(current[-1][1])
            current = []
    if current:
        windows.append(current[-1][1])
    return windows


def _stats(values):
    ordered = sorted(values)
    count = len(ordered)
    mean = sum(ordered) / count
    variance = sum((value - mean) ** 2 for value in ordered) / count
    median = (ordered[count // 2] if count % 2
              else (ordered[count // 2 - 1] + ordered[count // 2]) / 2)
    return {"mean": mean, "median": median, "std": math.sqrt(variance),
            "max_abs": max(abs(value) for value in ordered), "count": count}


def parse_kp_schedule(text):
    if not text:
        return None
    axes = []
    for axis_text in text.split(";"):
        points = []
        for element in axis_text.split(","):
            angle, kp = element.split(":")
            points.append((float(angle), float(kp)))
        if not points:
            raise ValueError("empty Kp schedule axis")
        axes.append(points)
    if len(axes) != 5:
        raise ValueError("Kp schedule must contain five axes")
    return axes


def parse_kp_axis(text):
    values = [float(value) for value in text.split(",")]
    if len(values) != 5:
        raise ValueError("--kp-axis must contain five values")
    return values


def interpolate(points, angle_deg):
    if angle_deg <= points[0][0]:
        return points[0][1]
    if angle_deg >= points[-1][0]:
        return points[-1][1]
    for (a_angle, a_kp), (b_angle, b_kp) in zip(points, points[1:]):
        if angle_deg <= b_angle:
            if not b_angle > a_angle:
                raise ValueError("Kp schedule angles must increase")
            ratio = (angle_deg - a_angle) / (b_angle - a_angle)
            return a_kp + (b_kp - a_kp) * ratio
    return points[-1][1]


def kp_for(axis_index, angle_rad, axis_kp, schedule):
    if schedule is None:
        return axis_kp
    return interpolate(schedule[axis_index], math.degrees(angle_rad))


def analyze(path, window_s=1.0, kp_axis=None, schedule=None):
    rows = load_controller(path)
    windows = stable_windows(rows, window_s)
    if not windows:
        raise ValueError("no stable reference plateau found")
    axis_kp = kp_axis or [160.0] * 5
    base = windows[0][-1]["ref"]
    result = {"schema": "slaihand_signed_static/3", "window_s": window_s,
              "static_feedback_speed_limit_rad_s": STATIC_FEEDBACK_SPEED,
              "window_ref_span_limit_rad": REF_SPAN_LIMIT,
              "kp_source": "schedule" if schedule is not None else "fixed",
              "kp_axis": axis_kp,
              "window_rule": "whole-window reference span + feedback speed; no adjacent-sample merge",
              "joints": list(JOINTS), "segments": []}
    for index, window in enumerate(windows):
        end = window[-1]["t"]
        target = window[-1]["ref"]
        ref_span = max(max(row["ref"][i] for row in window) -
                       min(row["ref"][i] for row in window) for i in range(5))
        rel = [math.degrees(target[i] - base[i]) for i in range(5)]
        segment = {"index": index, "start_t": window[0]["t"], "end_t": end,
                   "reference_deg": [math.degrees(value) for value in target],
                   "relative_deg": rel,
                   "window_reference_span_deg": math.degrees(ref_span),
                   "window_valid": True,
                   "axes": {}}
        for axis_index, axis in enumerate(JOINTS):
            signed = [row["fb"][axis_index] - row["ref"][axis_index]
                      for row in window]
            degrees = [math.degrees(value) for value in signed]
            kp_values = [kp_for(axis_index, row["fb"][axis_index], axis_kp[axis_index], schedule)
                         for row in window]
            pd_request = [-kp * value for kp, value in zip(kp_values, signed)]
            segment["axes"][axis] = {
                "signed_error_deg": _stats(degrees),
                "kp_used": _stats(kp_values),
                "pd_request_estimate_nm": _stats(pd_request),
            }
        result["segments"].append(segment)
    return result


def load_client_targets(path):
    data = json.loads(_read_text(path))
    initial = [float(value) for value in data["initial_model_rad"]]
    targets = []
    for row in data.get("waypoint_results", []):
        target = [float(value) for value in row["target_rad"]]
        if not targets or max(abs(a - b) for a, b in zip(target, targets[-1])) > 1e-9:
            targets.append(target)
    return initial, targets


def plan_windows(rows, initial, targets, window_s, tol=0.002):
    """Match velocity-confirmed plateaus to the client target sequence.

    Target-array matching alone can miss poses when the JTC reference settles
    slowly.  This selects stable suffixes first, then assigns each to the
    nearest planned target.  Repeated neutral targets keep only the last
    plateau; single-axis poses remain separate.
    """
    unique_targets = []
    for target in targets:
        if not unique_targets or max(abs(a - b) for a, b in zip(target, unique_targets[-1])) > 1e-9:
            unique_targets.append(target)
    by_target = {}
    for window in stable_windows(rows, window_s):
        reference = window[-1]["ref"]
        candidates = [(max(abs(reference[i] - target[i]) for i in range(5)), index)
                      for index, target in enumerate(unique_targets)]
        distance, index = min(candidates)
        if distance <= tol:
            by_target[index] = window
    return [(initial, by_target[index]) for index in sorted(by_target)]


def analyze_with_client(path, client_result, window_s=0.3, kp_axis=None, schedule=None,
                        target_tolerance=0.002):
    rows = load_controller(path)
    initial, targets = load_client_targets(client_result)
    windows = plan_windows(rows, initial, targets, window_s, tol=target_tolerance)
    if not windows:
        raise ValueError("no planner-target window found")
    axis_kp = kp_axis or [160.0] * 5
    result = {"schema": "slaihand_signed_static/4", "window_s": window_s,
              "window_rule": "client target list + whole-window reference span + feedback speed",
              "window_ref_span_limit_rad": REF_SPAN_LIMIT,
              "kp_source": "schedule" if schedule is not None else "fixed",
              "kp_axis": axis_kp,
              "joints": list(JOINTS), "segments": []}
    for index, (base, window) in enumerate(windows):
        target = window[-1]["ref"]
        ref_span = max(max(row["ref"][i] for row in window) -
                       min(row["ref"][i] for row in window) for i in range(5))
        rel = [math.degrees(target[i] - base[i]) for i in range(5)]
        segment = {"index": index, "start_t": window[0]["t"], "end_t": window[-1]["t"],
                   "reference_deg": [math.degrees(value) for value in target],
                   "relative_deg": rel,
                   "window_reference_span_deg": math.degrees(ref_span),
                   "window_valid": True,
                   "axes": {}}
        for axis_index, axis in enumerate(JOINTS):
            signed = [row["fb"][axis_index] - row["ref"][axis_index] for row in window]
            kp_values = [kp_for(axis_index, row["fb"][axis_index], axis_kp[axis_index], schedule)
                         for row in window]
            segment["axes"][axis] = {
                "signed_error_deg": _stats([math.degrees(value) for value in signed]),
                "kp_used": _stats(kp_values),
                "pd_request_estimate_nm": _stats([-kp * value for kp, value in zip(kp_values, signed)]),
            }
        result["segments"].append(segment)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input")
    parser.add_argument("--output", default="/tmp/signed_static_summary.json")
    parser.add_argument("--window", type=float, default=1.0)
    parser.add_argument("--kp-axis", default="160,160,160,160,160")
    parser.add_argument("--kp-schedule", default="")
    parser.add_argument("--client-result", default="")
    parser.add_argument("--target-tolerance", type=float, default=0.002)
    args = parser.parse_args()
    if args.client_result:
        summary = analyze_with_client(args.input, args.client_result, args.window,
                                      parse_kp_axis(args.kp_axis),
                                      parse_kp_schedule(args.kp_schedule),
                                      args.target_tolerance)
    else:
        summary = analyze(args.input, args.window,
                          parse_kp_axis(args.kp_axis),
                          parse_kp_schedule(args.kp_schedule))
    Path(args.output).write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
                                 encoding="utf-8")
    for segment in summary["segments"]:
        print("segment", segment["index"], "rel_deg",
              [round(value, 2) for value in segment["relative_deg"]],
              "ref_span_deg", round(segment["window_reference_span_deg"], 4),
              "valid", segment["window_valid"])
        for axis in JOINTS:
            stats = segment["axes"][axis]["signed_error_deg"]
            print("  %s mean=%+.4f median=%+.4f std=%.4f max_abs=%.4f" % (
                axis, stats["mean"], stats["median"], stats["std"], stats["max_abs"]))
    print("written", args.output)


if __name__ == "__main__":
    main()
