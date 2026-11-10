# Serial/audit failure fix (2026-11-09)

## Finding

The stop reason was `SERIAL_OR_AUDIT_WRITE_FAILURE`, but `CanUsbSystem::send()`
covers both serial writes and audit queue/write failures. The audit writer had:

- a fixed 4096-record queue;
- one formatted record per `fwrite`;
- `fflush` on every drain;
- much larger records after the new Kp/audit fields.

If the audit writer falls behind, `push()` returns false and the same
`SERIAL_OR_AUDIT_WRITE_FAILURE` path is taken, even when the serial link is
healthy. The existing catch block also discarded the actual exception text.

## Code fix

- audit queue: 4096 -> 8192 records;
- writer batches JSON into a 256 KiB buffer and writes it in larger blocks;
- flushes once per drained batch rather than per record;
- serial/write exceptions are now logged with the actual `what()` text and
  whether the failing action was READ or MOTOR.

The serial write deadline was left at 5 ms; increasing it to 20 ms did not fix
the failure and would consume the control period.

## Tests

```text
C++ hardware/transport: 7/7 passed
build: 5 packages passed
```

No hardware run after the fix yet. The admission artifact/evidence expired while
diagnosing, so a fresh J5 TIMEOUT -> evidence -> artifact chain is required before
testing GC01 again.

The new error log will distinguish a real serial write failure from an audit
queue/IO failure on the next attempt.
