# SLAI shoulder dynamics demonstration

Run app.py with Python 3.12. NumPy and SciPy use the current installed environment; MuJoCo 3.13 is reused from the existing project-local 五轴仿真/_sim_deps directory. No install or machine configuration change was needed.

model.py converts the latest canonical four-axis URDF and provisional_gravity.json into a separate MJCF audit artifact, shoulder_dynamics.xml. Source meshes are loaded through MuJoCo's virtual file system to avoid native Windows Chinese-path issues. Load via model.build(), not by opening the generated XML alone. The formal URDF and CAD are unchanged; the external Rx(-90 degrees) shoulder installation preserves all internal joint frames and signs, following the URDF skill frame-semantics guidance.

app.py opens independent Tk/MuJoCo windows. No network, ROS, serial or CAN interfaces. This is gravity feedforward plus joint-space PD impedance, not a reproduction of OpenArm's full controller or a JEP paper. Modes: off (damping only), float (gravity+damping), spring (gravity+PD). Automatic comparison resets to HOME between control stages; it changes only this demonstration's state. HOME=[25,65,0,-25] degrees, not a new joint zero.

Verification 2026-10-08: six automated tests passed, covering independent analytical gravity agreement at three poses, compensation hold, uncompensated drift, force response and spring return, floating displacement, and invalid force/mode rejection. Native GUI automatic comparison also completed and returned to HOME. With an 8 N upward 0.4-second pulse, spring displacement at pulse end was approximately J2 +8.506 degrees and J4 -10.363 degrees; errors after another five seconds were below 1e-6 degrees in this ideal model. This is a numerical simulation result, not achievable real-world accuracy.

Limitations: provisional CAD inertials and motor mass scaling; rotor/stator split unverified; no measured friction, motor rotor inertia, cable/payload or collisions; demonstration gains and torque caps are not motor specifications. MuJoCo limits are soft constraints. No all-joint collision sweep or real hardware validation has been performed. Original RViz and the 8767 estimation page are separate and not driven by this demo.

Update 2026-10-08: Startup and reset now use ZERO=[0,0,0,0]. enter_demo() explicitly resets to HOME for the separate demo-pose button and automatic comparison stages. Automatic comparison finishes at the demonstration target, not ZERO. Seven tests pass including startup zero hold and reset clearing velocity and external force. Earlier HOME-based test results above remain comparison results, not startup values.

Desktop entry and Chinese instructions: C:/Users/ASUS/Desktop/SLAI机械臂控制/01_四轴仿真/05_动力学阻抗演示.
