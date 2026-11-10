"""Shadow-only comparison of the Codex FF candidate against GC baseline data.

No ROS, no driver output, no FF application. It reads the recorded baseline,
evaluates the candidate through Codex's offline calculator, and reports the
residual between predicted and recorded holding torque estimates.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

REPO = Path("/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain")
sys.path.insert(0, str(REPO / "verification/hardware/five_axis_20261107/signed_acquisition"))
sys.path.insert(0, str(REPO / "verification/hardware/five_axis_20261107/identification_codex_20261109"))
import analyze_signed
import analyze_static_torque
import evaluate_offline_ff

EXEC = REPO / "verification/hardware/five_axis_20261107/gravity_compensation_trajectories_20261109/execution_20261109"
CASES = {
    "GC01": ("raw/gc01_20261109.jsonl.gz", "clients/gc01f/20261109_133627_2d97c20e.json"),
    "GC02": ("raw/gc02_20261109.jsonl.gz", "clients/gc02/20261109_133915_2361e5fa.json"),
    "GC03": ("raw/gc03_20261109.jsonl.gz", "clients/gc03/20261109_134157_adcb6df9.json"),
    "GC04": ("raw/gc04_20261109.jsonl.gz", "clients/gc04/20261109_134444_3322dcce.json"),
}
JOINTS = ("slaihand_joint1", "slaihand_joint2", "slaihand_joint3", "slaihand_joint4")


def stats(values):
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / len(values)
    return {"mean": mean, "std": math.sqrt(variance),
            "rmse": math.sqrt(sum(value * value for value in values) / len(values)),
            "max_abs": max(abs(value) for value in values), "count": len(values)}


def main():
    result = {"schema": "slaihand_gc_shadow_ff_validation/1",
              "deployable": False, "ff_applied": False,
              "note": "candidate evaluated offline; recorded effort is a torque estimate, not current",
              "sessions": {}}
    all_residuals = {joint: [] for joint in JOINTS}
    for name, (state_rel, client_rel) in CASES.items():
        state = str(EXEC / state_rel)
        client = str(EXEC / client_rel)
        controller = analyze_signed.load_controller(state)
        initial, targets = analyze_signed.load_client_targets(client)
        windows = analyze_signed.plan_windows(controller, initial, targets, 0.3, 0.02)
        dynamic = analyze_static_torque.load_dynamic(state)
        session = {"windows": [], "skipped_outside_bounds": 0}
        for index, (_, window) in enumerate(windows):
            q_deg = [math.degrees(sum(row["ref"][axis] for row in window) / len(window))
                     for axis in range(4)]
            try:
                predicted = evaluate_offline_ff.evaluate(q_deg)
            except ValueError as error:
                session["skipped_outside_bounds"] += 1
                session["windows"].append({"index": index, "q_deg": q_deg,
                                           "skipped": str(error)})
                continue
            predicted_nm = predicted["total_holding_torque_model_nm"]
            actual = {joint: [] for joint in JOINTS}
            for row in window:
                effort = analyze_static_torque.nearest_effort(dynamic, row["t"])
                if effort is None:
                    continue
                for joint in JOINTS:
                    if joint in effort:
                        actual[joint].append(effort[joint])
            window_result = {"index": index, "q_deg": q_deg,
                             "predicted_model_nm": predicted_nm,
                             "actual_effort_estimate_nm": {
                                 joint: (sum(values) / len(values) if values else None)
                                 for joint, values in actual.items()}}
            for axis, joint in enumerate(JOINTS):
                values = actual[joint]
                if values:
                    residual = sum(values) / len(values) - predicted_nm[axis]
                    all_residuals[joint].append(residual)
                    window_result.setdefault("residual_nm", {})[joint] = residual
            session["windows"].append(window_result)
        result["sessions"][name] = session
    result["residual_stats_nm"] = {joint: stats(values) if values else None
                                   for joint, values in all_residuals.items()}
    output = EXEC / "GC_SHADOW_FF_VALIDATION_20261109.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result["residual_stats_nm"], ensure_ascii=False, indent=2))
    print("written", output)


if __name__ == "__main__":
    main()
