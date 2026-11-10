# SLAIHand four-axis ROS 2 workspace

Git workspace is `D:/CodexData/Projects/机械臂/slaihand_ros2_ws`.
The 2026-11-05 checkpoint is on `codex/control-panel-20261105`, tagged
`snapshot/2026-11-06-control-panel`. See [GIT使用说明.md](GIT使用说明.md)
for commits, branches and restoring the local bundle. No remote is configured.

This workspace contains the four-axis OpenArm adaptation, offline backends and
the explicitly selected CANUSB hardware backend with its web control panel.
The hardware launch starts disarmed and requires a separate operator action.
J5 is not active. Current model parameters are J4 −50° to +90° and peak
reference speed 1 rad/s. Gravity compensation uses a locally identified candidate.

## What it contains

- `slaihand_description`: validated CAD-derived four-axis URDF and meshes.
- `openarm_can`: unchanged upstream MIT encoder library.
- `slaihand_hardware`: configurable four/five-joint offline adapter.
- `slaihand_bringup`: `ros2_control` offline hardware, RViz launch, controller
  configuration, and a small safe trajectory example.

The joint order is `slaihand_joint1` through `slaihand_joint4`, corresponding
to J1 through J4. J5 is intentionally absent because it is not modeled yet.

## Windows desktop entry points

Desktop entry points are now organized under `SLAI机械臂控制`:
`01_四轴仿真/01_启动` for startup and `01_四轴仿真/02_控制` for controls.
Duplicate and disabled entries are under `99_历史与备用`. See the folder's `使用说明.md`.

Use `启动SLAI四轴OpenArm仿真.bat` on the Windows desktop to open RViz and the
fake controller. Then use `SLAI四轴小幅运动.bat` to send a single three-second
trajectory. The test exits when the controller confirms completion.
Do not launch twice. Old five-axis desktop entries are disabled.

For interactive targets and feedback, open `SLAI四轴控制面板.bat` after launching
the simulation, then visit http://127.0.0.1:8766 . The local Windows frontend
bridges to WSL over standard input/output, without LAN exposure. Targets use
degrees and require an explicit execute click; limit violations are rejected.
The cancel button is a simulation trajectory cancellation, not a hardware stop.
Do not run the old small-test script while a panel trajectory is active.

The configured distribution is `Ubuntu-24.04`, ROS distribution `jazzy`,
Linux user `xuyu`. The build workspace is `/home/xuyu/slaihand_ros2_ws`.
The Windows workspace holds the editable source; `build_workspace.sh` copies
it to Linux and rebuilds it. No user password has been assigned automatically.

## Linux commands

```bash
cd ~/slaihand_ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch slaihand_bringup slaihand_fake.launch.py
```

In a second terminal, after sourcing `install/setup.bash`:

```bash
ros2 run slaihand_bringup send_small_test.py
```

Use `backend:=generic` to select GenericSystem, or `gui:=false` for headless tests.
Do not substitute real hardware for the offline adapter until CAN
IDs, motor signs, and encoder zero offsets have been measured one axis at a
time with the arm safely supported.

## OpenArm migration status

This workspace uses `slaihand_hardware/OpenArmJointFake` by default, not OpenArm's real CAN
plugin. OpenArm's inspected `OpenArmHW` driver hardcodes seven arm joints,
OpenArm joint names, default motor IDs, and automatic return-to-zero during
activation. Those must be adapted to this four-axis robot and calibrated
before real control. J1/J2 use DM-J10010L-2EC and J3/J4 DM-J8009P-2EC.
The current CAD transforms, axes, and limits are unchanged from the verified
SLAI description. J5 and a gripper are not included.

2026-10-07 correction: the user identified J2 positive rotation as reversed.
J2 now uses local +Z; zero-pose origins and meshes are unchanged. Numeric limits
stay -10 to +190 degrees, so +190 now turns toward the old negative direction.
This changes the physical sector; full-range collision safety remains unverified.
Historical five-axis files and older standalone MuJoCo exports are not current.

J3 CAD correction on the same date: the new CAD coordinate system `J3` replaces
`J3_cs`. The link3 mesh and J3/J4 relative transforms were updated together;
zero-pose assembly geometry was preserved. See the adaptation note and
`verification/j3_pivot_comparison.png`. Historical exports remain unchanged.

MIT packets stay in memory; kp/kd/tau are zero. Feedback is simulated rate-limited
position following, not motor dynamics or impedance control. Motor IDs, signs
and zero offsets remain uncalibrated. DM8009 is an offline profile, not a verified
DM8009P-2EC specification. Activation holds position, without automatic homing.

The adapter reads joint count and parameters from configuration. Future J5 needs
verified CAD transforms, axis, geometry and limits, plus controller/trajectory
configuration updates. Historical five-axis wrappers are not runnable defaults.
See `OpenArm四轴适配说明.md` for verification and handoff details.

J4 CAD correction (2026-10-07): coordinate `坐标系1` replaces the previous
J4 frame. Parent-relative origin and link4 mesh are updated together from
`verification/cad_j4_corrected`. J1/J2/J3 are unchanged. Latest CAD link4
zero-pose geometry differs from the old model by up to 3.859 mm nearest-vertex
distance; fresh CAD geometry is authoritative. Axis-line residual is below
1e-8 m; local -Z direction and -90/+50 degree limits are preserved.
Backup: `verification/before_j4_correction`; review: `verification/j4_pivot_comparison.png`.
No collision safety or gravity-compensation validation is implied.

The old Ubuntu 26.04 distribution was exported before removal. Its recovery
archive is `../wsl_backups/Ubuntu-26.04-before-replacement.tar`.
