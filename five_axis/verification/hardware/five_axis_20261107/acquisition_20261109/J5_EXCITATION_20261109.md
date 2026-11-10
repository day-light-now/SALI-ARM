# J5 excitation and readback-only admission (2026-11-09)

## Change requested

The operator wants to stop requiring a J5 power cycle / TIMEOUT0 before each
run. The J5 watchdog is **not** disabled and is **not** set to 0.

New mode:

```text
j5_timeout_setup.py --verify-existing
```

- reads the existing TIMEOUT and requires it to already be 2000;
- verifies all five axes are fresh and DISABLED;
- writes **no** parameter;
- writes evidence with `write_outcome=verified_existing`,
  `timeout_before=2000`, `timeout_after=2000`, `write_attempts=0`,
  `parameter_writes=0`;
- the artifact builder accepts this truthful readback-only evidence.

This removes the power-cycle requirement while keeping the 2000 ms watchdog.

## J5 excitation

Plan: `J5_EXCITATION_ZERO.json`.

- J1–J4 absolute zero;
- J5 `0 → +10 → −10 → +20 → −20 → 0`;
- 8 s moves, 6 s dwell, final 8 s;
- no FF;
- fresh artifact from readback-only evidence.

Result: client exit 0, all six windows sampled and passed:

```text
start   passed
J5 +10  passed
J5 −10  passed
J5 +20  passed
J5 −20  passed
J5 0    passed
```

This is the first dataset where the 3.5 kg box moves with the J5 output.

## Status

- driver stopped after the run;
- no FF applied;
- no collision;
- no current/thermal/1.5x claim;
- data is ready for Codex's five-axis model with J5 output load.
