"""Join controller dwell windows with per-joint effort (torque estimate).

No ROS/hardware. The DAMIAO MIT feedback ``effort`` is a controller torque
estimate, not measured current. This tool only makes the existing per-joint
estimate usable for static load exploration; it must not be labelled current.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import analyze_signed


def _read_lines(path):
    if str(path).endswith(".gz"):
        import gzip
        with gzip.open(path, "rt", encoding="utf-8") as stream:
            return stream.read().splitlines()
    return Path(path).read_text(encoding="utf-8").splitlines()


def load_dynamic(path):
    rows = []
    for line in _read_lines(path):
        record = json.loads(line)
        if record.get("kind") != "dynamic":
            continue
        interfaces = record["payload"]["interfaces"]
        effort = {}
        for joint, values in interfaces.items():
            if "effort" in values:
                effort[joint] = float(values["effort"])
        if effort:
            rows.append({"t": float(record["t"]), "effort": effort})
    rows.sort(key=lambda row: row["t"])
    return rows


def nearest_effort(rows, t, tolerance=0.03):
    if not rows:
        return None
    row = min(rows, key=lambda item: abs(item["t"] - t))
    if abs(row["t"] - t) > tolerance:
        return None
    return row["effort"]


def _stats(values):
    ordered = sorted(values)
    count = len(ordered)
    mean = sum(ordered) / count
    variance = sum((value - mean) ** 2 for value in ordered) / count
    return {"mean": mean, "std": math.sqrt(variance),
            "max_abs": max(abs(value) for value in ordered), "count": count}


def analyze(state_path, client_result, window_s=0.3, target_tolerance=0.002):
    controller = analyze_signed.load_controller(state_path)
    initial, targets = analyze_signed.load_client_targets(client_result)
    windows = analyze_signed.plan_windows(controller, initial, targets, window_s, tol=target_tolerance)
    dynamic = load_dynamic(state_path)
    if not windows:
        raise ValueError("no plan-target static window")
    result = {"schema": "slaihand_static_torque_proxy/1",
              "source": "recorder dynamic effort",
              "measurement": "DAMIAO MIT torque estimate; NOT measured current",
              "window_s": window_s,
              "joints": list(analyze_signed.JOINTS), "segments": []}
    for index, (base, window) in enumerate(windows):
        target = window[-1]["ref"]
        rel = [math.degrees(target[i] - base[i]) for i in range(5)]
        segment = {"index": index, "relative_deg": rel, "axes": {}}
        for axis in analyze_signed.JOINTS:
            values = []
            errors = []
            for row in window:
                effort = nearest_effort(dynamic, row["t"])
                if effort is not None and axis in effort:
                    values.append(effort[axis])
                axis_index = analyze_signed.JOINTS.index(axis)
                errors.append(math.degrees(row["fb"][axis_index] - row["ref"][axis_index]))
            segment["axes"][axis] = {
                "effort_nm": _stats(values) if values else None,
                "signed_error_deg": _stats(errors),
            }
        result["segments"].append(segment)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("state")
    parser.add_argument("client_result")
    parser.add_argument("--output", default="/tmp/static_torque_proxy.json")
    args = parser.parse_args()
    report = analyze(args.state, args.client_result)
    Path(args.output).write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                 encoding="utf-8")
    for segment in report["segments"]:
        print("angle_rel_deg", [round(value, 2) for value in segment["relative_deg"]])
        for axis in ("slaihand_joint1", "slaihand_joint4"):
            effort = segment["axes"][axis]["effort_nm"]
            print(" ", axis, "torque_est_nm", None if effort is None else round(effort["mean"], 4))
    print("written", args.output)


if __name__ == "__main__":
    main()
