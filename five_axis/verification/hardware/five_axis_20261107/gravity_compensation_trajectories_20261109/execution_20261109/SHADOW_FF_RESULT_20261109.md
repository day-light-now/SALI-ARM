# Shadow FF validation on GC baseline (2026-11-09)

Started the gravity-compensation work in **shadow mode only**: no torque added,
no driver FF enabled.

The Codex candidate was evaluated with its own offline calculator against the
completed GC01–GC04 baseline runs. Recorded `effort` remains a torque estimate,
not current.

## Residual: actual effort estimate - candidate prediction

| axis | RMSE | max abs | count |
|---|---:|---:|---:|
| J1 | 2.242 Nm | 5.096 Nm | 11 |
| J2 | 1.149 Nm | 2.566 Nm | 11 |
| J3 | 0.851 Nm | 1.895 Nm | 11 |
| J4 | 2.370 Nm | 4.674 Nm | 11 |

The candidate's earlier independent result was about 1.327 Nm total RMSE; on
this new baseline extraction it is worse, especially J1/J4. The extraction only
mapped 11 stable target windows from the four GC clients, so this is a screening
result, not a complete replay.

## Verdict

**Do not enable gravity compensation yet.** The shadow error is too large and
would add incorrect torque on J1/J4. J3 also does not improve.

Per the earlier plan:

1. weigh the actual 3.5 kg box;
2. constrain the link4/J4-origin-to-box-COM geometry, especially z;
3. obtain more paired/loaded-unloaded static data if needed;
4. repeat shadow validation;
5. only then implement bounded FF with limit/slew/STOP-zero and a small scale.

No FF was written to the driver. Driver is stopped; no active motion.
