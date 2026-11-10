"""Build a one-shot bounded pose-trim plan from a signed summary.

The correction is applied to the commanded target only; it is not a driver FF.
Correction = -(measured mean feedback-reference), limited in magnitude, with a
small deadband. This is an experimental static trim, not a current/gravity model.
"""
from __future__ import annotations

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
PLAN = HERE / "MULTI_POSE_40_BOTH_GROUPS.json"
SUMMARY = HERE / "multi_pose_40_summary_20261109.json"
OUTPUT = HERE / "MULTI_POSE_40_TRIM1.json"
MAX_CORRECTION_DEG = 2.5
DEADBAND_DEG = 0.15


def correction(axis_stats):
    error = float(axis_stats["mean"])
    if abs(error) < DEADBAND_DEG:
        return 0.0
    return max(-MAX_CORRECTION_DEG, min(MAX_CORRECTION_DEG, -error))


def build(plan_path=PLAN, summary_path=SUMMARY):
    plan = json.loads(Path(plan_path).read_text(encoding="utf-8"))
    summary = json.loads(Path(summary_path).read_text(encoding="utf-8"))
    segments = summary["segments"]
    waypoints = plan["waypoints"]
    if len(segments) != len(waypoints):
        raise ValueError("plan and summary segment counts differ")
    axes = summary["joints"]
    corrections = []
    for index, (waypoint, segment) in enumerate(zip(waypoints, segments)):
        if index == 0:
            corrections.append([0.0] * 5)
            continue
        corr = [correction(segment["axes"][axis]["signed_error_deg"]) for axis in axes]
        corrections.append(corr)
        waypoint["degrees"] = [float(value) + delta
                               for value, delta in zip(waypoint["degrees"], corr)]
    plan["waypoints"] = waypoints
    return plan, corrections


def main():
    plan, corrections = build()
    OUTPUT.write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("corrections_deg", corrections)
    print("written", OUTPUT)


if __name__ == "__main__":
    main()
