"""Cross-check the C++ gravity feedforward model against the Python production model.

Both sides consume the same model text that the launch generates from the
canonical URDF + mass profile, so a mismatch means a real port/regression bug.
Run inside the ROS/WSL environment (needs xacro/yaml to import the launch module).
"""
import importlib.util
import math
import random
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
import control_panel_gravity as gravity  # noqa: E402
import yaml  # noqa: E402

URDF = ROOT / "src/slaihand_description/urdf/slaihand_four_axis.urdf"
PROFILE = ROOT / "src/slaihand_bringup/config/gravity_provisional.json"
POLICY = ROOT / "src/slaihand_bringup/config/canusb_motion_policy.yaml"
INCLUDE = ROOT / "src/slaihand_hardware/include"
SOURCE = ROOT / "src/slaihand_hardware/src/gravity_ff.cpp"
DUMP = ROOT / "verification/hardware/control_panel_20261104/gravity_dump.cpp"
LIMITS = [(-1.3962634015954636, 3.4906585039886591), (-0.17453292519943295, 3.3161255787892263),
          (-1.5707963267948966, 1.5707963267948966), (-0.8726646259971648, 1.5707963267948966)]


def launch_module():
    spec = importlib.util.spec_from_file_location(
        "motion_launch", ROOT / "src/slaihand_bringup/launch/slaihand_canusb_motion.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_dump(binary):
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic",
                    "-I", str(INCLUDE), str(DUMP), str(SOURCE), "-o", str(binary)],
                   check=True)


def compare(label, gravity_vector, j3_offset, launch, binary, rng):
    with tempfile.TemporaryDirectory() as directory:
        text = launch.build_gravity_model_text(URDF, PROFILE, gravity_vector, j3_offset)
        text_path = Path(directory) / "model.txt"
        text_path.write_text(text, encoding="utf-8")
        poses = [[rng.uniform(low, high) for low, high in LIMITS] for _ in range(30)]
        payload = "\n".join(" ".join(repr(value) for value in pose) for pose in poses) + "\n"
        result = subprocess.run([str(binary), str(text_path)], input=payload, capture_output=True,
                                text=True, check=True)
        rows = [line.split() for line in result.stdout.strip().splitlines()]
        assert len(rows) == len(poses), (len(rows), len(poses))
        model = gravity.FourAxisGravityModel(
            URDF, PROFILE, gravity_base=tuple(gravity_vector),
            fixed_joint_offsets={"slaihand_joint3": j3_offset} if j3_offset else None)
        worst = 0.0
        for pose, row in zip(poses, rows):
            cpp = [float(value) for value in row]
            expected = model.compute(pose)
            for cpp_value, py_value in zip(cpp[:4], expected["tau_model_nm"]):
                worst = max(worst, abs(cpp_value - py_value))
            worst = max(worst, abs(cpp[4] - expected["potential_energy_j"]))
        if worst > 1e-9:
            raise SystemExit("FAIL %s: max difference %.3g" % (label, worst))
        print("PASS %-22s poses=%d max_diff=%.3g" % (label, len(poses), worst))


def main():
    rng = random.Random(20261105)
    launch = launch_module()
    with tempfile.TemporaryDirectory() as directory:
        binary = Path(directory) / "gravity_dump"
        build_dump(binary)
        policy = yaml.safe_load(POLICY.read_text(encoding="utf-8"))
        documented = [policy["gravity_base_x"], policy["gravity_base_y"], policy["gravity_base_z"]]
        j3 = float(policy["gravity_j3_offset_rad"])
        compare("documented +Y down", documented, j3, launch, binary, rng)
        compare("documented, no J3 offset", documented, 0.0, launch, binary, rng)
        compare("Z down variant", [0.0, 0.0, -9.80665], j3, launch, binary, rng)
        compare("Y down, no J3 offset", [0.0, 9.80665, 0.0], 0.0, launch, binary, rng)
    print("PASS gravity model cross-check")


if __name__ == "__main__":
    main()
