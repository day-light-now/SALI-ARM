"""只读残差与覆盖报告：候选模型 vs 现有 56 个静态样本，并预测大角度计划姿态。

不写参数、不改质量档案；输出 Markdown + JSON 证据。
"""
import hashlib
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
from control_panel_gravity import FourAxisGravityModel  # noqa: E402

URDF = ROOT / "src/slaihand_description/urdf/slaihand_four_axis.urdf"
CANDIDATE = ROOT / "src/slaihand_bringup/config/gravity_identified_candidate.json"
PLAN = HERE / "large_angle_capture_plan.json"
EVIDENCE = HERE / "evidence"
J4_LIMITS = [[-80.0, 200.0], [-10.0, 190.0], [-90.0, 90.0], [-50.0, 90.0]]


def sha256(path):
    with open(path, "rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def load_samples(candidate):
    rows = []
    for source in candidate["sources"]:
        path = Path(source["raw_file"])
        if not path.is_file():
            raise SystemExit("missing raw file: %s" % path)
        if sha256(path) != source["sha256"]:
            raise SystemExit("raw file changed (sha256 mismatch): %s" % path)
        content = path.read_text(encoding="utf-8").splitlines()
        for line in content[1:]:
            if not line.strip():
                continue
            sample = json.loads(line)
            if sample.get("schema") != "slaihand_calibration_sample/2":
                raise SystemExit("unexpected sample schema in %s" % path)
            rows.append({"group": source["group"], "sample": sample})
    return rows


def stats(residuals, model_values):
    count = len(residuals)
    mean = sum(residuals) / count
    std = math.sqrt(sum((value - mean) ** 2 for value in residuals) / count)
    rms = math.sqrt(sum(value * value for value in residuals) / count)
    signs = [1 if a * b >= 0 else 0 for a, b in zip(residuals, model_values) if abs(b) >= 0.05]
    return {"count": count, "residual_mean_nm": round(mean, 5),
            "residual_std_nm": round(std, 5), "residual_rms_nm": round(rms, 5),
            "max_abs_nm": round(max(abs(value) for value in residuals), 5),
            "sign_agreement": round(sum(signs) / len(signs), 4) if signs else None}


def main():
    candidate = json.loads(CANDIDATE.read_text(encoding="utf-8"))
    model = FourAxisGravityModel(
        URDF, CANDIDATE, gravity_base=(0.0, 9.80665, 0.0),
        fixed_joint_offsets={"slaihand_joint3": math.pi})
    planned = json.loads(PLAN.read_text(encoding="utf-8")) if PLAN.is_file() else {}

    rows = load_samples(candidate)
    pairs = {}
    for row in rows:
        sample = row["sample"]
        if sample.get("approach") in ("positive", "negative") and sample.get("pair_id"):
            pairs.setdefault(sample["pair_id"], {})[sample["approach"]] = (row["group"], sample)
    pair_values = {"training": [[], [], [], []], "holdout": [[], [], [], []]}
    pair_model = {"training": [[], [], [], []], "holdout": [[], [], [], []]}
    coverage = {"training": [[math.inf, -math.inf] for _ in range(4)],
                "holdout": [[math.inf, -math.inf] for _ in range(4)]}
    global_values = {"training": [[], [], [], []], "holdout": [[], [], [], []]}
    global_model = {"training": [[], [], [], []], "holdout": [[], [], [], []]}
    for row in rows:
        sample = row["sample"]
        role = "holdout" if sample.get("role") == "holdout" else "training"
        positions = sample["positions_rad"]
        measured = sample["effort_estimate_nm"]
        predicted = model.compute(positions)["tau_model_nm"]
        for axis in range(4):
            residual = measured[axis] - predicted[axis]
            global_values[role][axis].append(residual)
            global_model[role][axis].append(predicted[axis])
            coverage[role][axis][0] = min(coverage[role][axis][0], math.degrees(positions[axis]))
            coverage[role][axis][1] = max(coverage[role][axis][1], math.degrees(positions[axis]))
    for pair_id, entries in pairs.items():
        if set(entries) != {"positive", "negative"}:
            continue
        group, positive = entries["positive"]
        _, negative = entries["negative"]
        role = "holdout" if positive.get("role") == "holdout" else "training"
        midpoint = [(a + b) / 2 for a, b in zip(positive["positions_rad"], negative["positions_rad"])]
        measured = [(a + b) / 2 for a, b in zip(positive["effort_estimate_nm"],
                                               negative["effort_estimate_nm"])]
        predicted = model.compute(midpoint)["tau_model_nm"]
        for axis in range(4):
            pair_values[role][axis].append(measured[axis] - predicted[axis])
            pair_model[role][axis].append(predicted[axis])
    pair_axis = {}
    for axis in range(4):
        pair_axis["J%d" % (axis + 1)] = {
            role: stats(pair_values[role][axis], pair_model[role][axis])
            for role in ("training", "holdout")}
    pair_holdout_rms = math.sqrt(
        sum(value * value for axis in range(4) for value in pair_values["holdout"][axis])
        / max(1, sum(len(pair_values["holdout"][axis]) for axis in range(4))))

    per_axis = {}
    for axis in range(4):
        per_axis["J%d" % (axis + 1)] = {
            role: stats(global_values[role][axis], global_model[role][axis])
            for role in ("training", "holdout")}
    coverage_deg = {role: [[round(low, 2), round(high, 2)] for low, high in coverage[role]]
                    for role in ("training", "holdout")}
    holdout_rms = math.sqrt(sum(value * value
                                for axis in range(4)
                                for value in global_values["holdout"][axis])
                            / max(1, sum(len(global_values["holdout"][axis]) for axis in range(4))))

    predictions = []
    for pose in (planned.get("targets", {}).get("training_targets", [])
                 + planned.get("targets", {}).get("validation_targets", [])):
        degrees = pose["degrees"]
        radians = [math.radians(value) for value in degrees]
        tau = model.compute(radians)["tau_model_nm"]
        gaps = []
        inside = True
        for axis in range(4):
            low, high = coverage_deg["training"][axis]
            gaps.append(round(max(0.0, low - degrees[axis], degrees[axis] - high), 2))
            inside = inside and low <= degrees[axis] <= high
        predictions.append({"id": pose["id"], "group": pose["group"], "degrees": degrees,
                            "model_gravity_nm": [round(value, 4) for value in tau],
                            "outside_training_box_deg": gaps,
                            "inside_training_coverage": inside,
                            "urdf_inside": all(low <= value <= high for value, (low, high)
                                               in zip(degrees, J4_LIMITS))})
    report = {
        "schema": "slaihand_gravity_coverage_report/1",
        "generated_from": {"candidate": str(CANDIDATE), "candidate_sha256": sha256(CANDIDATE),
                           "urdf": str(URDF), "urdf_sha256": sha256(URDF),
                           "candidate_status": candidate.get("status"),
                           "identifiable_rank": candidate.get("identifiable_rank"),
                           "parameter_slots": len(candidate.get("parameter_slots") or []),
                           "samples": len(rows),
                           "sources": candidate.get("sources")},
        "coverage_deg": coverage_deg,
        "per_axis_residual": per_axis,
        "holdout_raw_sample_rmse_nm": round(holdout_rms, 5),
        "holdout_paired_midpoint_rmse_nm": round(pair_holdout_rms, 5),
        "paired_midpoint_axis": pair_axis,
        "pairs": len(pairs),
        "large_angle_predictions": predictions,
        "notes": [
            "残差 = 反馈力矩估计 − 候选模型预测；原始样本按候选档案列出的 sha256 校验后只读加载。",
            "训练覆盖仅为 J1 −17.40…16.79°、J2 −2.75…31.45° 的旧样本；大角度姿态属于模型外推，"
            "外推力矩只用于计划与风险提示，不能当作标定数据。",
            "可辨识秩 8/16：只能辨识 LINK 质量与前矩的 8 维有效基，单连杆参数不可辨识；"
            "新增样本若仍局限在同一基方向，不会提高秩。",
            "自由拖动含未知人手力矩，不能作为拟合样本；新参数必须来自合格的无接触静态保持样本。",
        ],
    }
    EVIDENCE.mkdir(exist_ok=True)
    (EVIDENCE / "large_angle_coverage_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    lines = ["# 大角度重力覆盖与只读残差报告", "",
             "候选：`%s`（%s，秩 %s/%s）" % (CANDIDATE, candidate.get("status"),
                                            candidate.get("identifiable_rank"),
                                            len(candidate.get("parameter_slots") or [])), "",
             "## 现有 56 个静态样本覆盖（训练 / 独立验证）", "",
             "| 轴 | 训练覆盖 ° | 验证覆盖 ° |", "|---|---|---|"]
    for axis in range(4):
        lines.append("| J%d | %s | %s |" % (
            axis + 1, coverage_deg["training"][axis], coverage_deg["holdout"][axis]))
    lines += ["", "## 逐轴残差（反馈力矩 − 候选模型）", "",
              "| 轴 | 训练均值 N·m | 训练标准差 | 训练 RMS | 验证均值 | 验证 RMS | 符号一致率(训练/验证) |",
              "|---|---|---|---|---|---|---|"]
    for axis in range(4):
        key = "J%d" % (axis + 1)
        training = per_axis[key]["training"]
        holdout = per_axis[key]["holdout"]
        lines.append("| %s | %.3f | %.3f | %.3f | %.3f | %.3f | %s / %s |" % (
            key, training["residual_mean_nm"], training["residual_std_nm"],
            training["residual_rms_nm"], holdout["residual_mean_nm"],
            holdout["residual_rms_nm"], training["sign_agreement"], holdout["sign_agreement"]))
    lines += ["", "原始逐样本留出 RMSE：**%.4f N·m**；正反来向配对中点留出 RMSE：**%.4f N·m**。"
              % (holdout_rms, pair_holdout_rms),
              "候选档案报告的 0.1576 N·m 来自配对中点口径；原始逐样本口径包含正反来向的摩擦/迟滞，"
              "因此更大是预期现象，两者不能混用。", "",
              "## 计划的大角度姿态（候选模型外推，只读预测）", "",
              "| 组 | ID | 角度 ° | 模型重力矩 N·m | 训练箱外距离 ° | 箱内? | URDF 内? |",
              "|---|---|---|---|---|---|---|"]
    for row in predictions:
        lines.append("| %s | %s | %s | %s | %s | %s | %s |" % (
            row["group"], row["id"], row["degrees"], row["model_gravity_nm"],
            row["outside_training_box_deg"], "是" if row["inside_training_coverage"] else "否",
            "是" if row["urdf_inside"] else "否"))
    lines += ["", "> 外推预测仅用于选择补采点与风险提示；大角度真实重力矩必须由无接触静态保持样本测得。"]
    (HERE / "大角度残差报告.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("REPORT_OK", len(rows), "samples, holdout rmse", round(holdout_rms, 4),
          "large-angle poses", len(predictions))


if __name__ == "__main__":
    main()
