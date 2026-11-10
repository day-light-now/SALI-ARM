import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import test_calibration_core as tests
import calibration_core as core

report = core.analyze(tests.diverse_samples(), tests.gravity)
print("plain status", report["status"])
print("plain reasons", report["reasons"])
print("counts", report["counts"])
print("fit", report["fit"])
print("holdout", report["holdout"])

rows = tests.diverse_samples()
for index, sample in enumerate(rows):
    if index % 2:
        sample["model"]["mapping_sha256"] = "e" * 64
report = core.analyze(rows, tests.gravity)
print("mixed status", report["status"])
print("mixed reasons", report["reasons"])
print("identity", report["identity"])
print("identities complete flags", [core.core_identity_complete(s) for s in rows[:4]])
