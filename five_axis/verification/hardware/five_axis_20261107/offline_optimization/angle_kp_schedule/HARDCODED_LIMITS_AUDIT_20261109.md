# Hard-coded limits audit (2026-11-09)

Scope: `src/slaihand_hardware`, five-axis policy, launch/config. No hardware was
run for this audit.

## Changed now

| Location | Before | After |
|---|---|---|
| scalar Kp validation | literal `160` | explicit `MotionPolicy::kp_max` (default 160) |
| per-axis Kp validation | literal `280` | explicit `kp_max` |
| scheduled Kp point validation | literal `280` | explicit `kp_max` |
| policy plumbing | no cap field | `kp_max` through xacro / hardware parameter / identity JSON |

The experimental branch policy now sets `kp_max: 280.0`. A policy that does not
set it keeps the production default 160. `kp_max` must be in `(0,500]`; 500 is
the protocol encoding ceiling and is not a capability claim.

## Found but intentionally not parameterized

| Item | Where | Why |
|---|---|---|
| Kd cap `5` | scalar/per-axis/float/arm validation | protocol MIT Kd encoding bound; no evidence to raise it |
| Kp wire cap `500`, Kd wire cap `5` | `native_codec.hpp` | fixed MIT wire-format range, not a policy cap |
| following error `4 deg` | `MotionPolicy::validate` | adopted safety guard; must not be loosened through policy YAML |
| temperature `50 C` | same | adopted thermal guard |
| reference/feedback speed `1 rad/s` | same | adopted motion guard |
| gain ramp `1 s`, stagger `.03 s`, age `.06 s`, gap `.05 s`, runtime `180 s` | same | adopted timing/session guards |
| schedule points `1..32`, angles `[-720,720] deg`, slew `<=5000` | schedule validation | implementation sanity bounds, not operating capability |
| JTC path tolerance `10 deg` | controller YAML | deliberate deployment configuration, not C++ cap |
| planner waypoint precision `0.5 deg` | planner/client | test acceptance threshold |
| artifact/evidence freshness | admission path | separate admission policy; must stay at the default 1800 s |

None of the adopted safety guards above were raised by this change. The only
new configurable quantity is `kp_max`, and an experiment must opt in explicitly.

## Remaining policy asymmetry

Kp now has an explicit ceiling (`kp_max`) while Kd remains fixed at 5. This is
intentional: there is no proposed Kd schedule or Kd experiment, and the MIT wire
Kd range is 5. A future Kd experiment should add an explicit `kd_max` rather
than reusing this mechanism implicitly.
