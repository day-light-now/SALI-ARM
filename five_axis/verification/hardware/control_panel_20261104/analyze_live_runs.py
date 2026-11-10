"""Read-only analysis of the control panel's own operation archive (live runs)."""
import json
import math
import os
from pathlib import Path

ARCHIVE = Path.home() / ".ros" / "slaihand" / "control_panel" / "operations"


def load(path, default=None):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except Exception:
        return default


def fmt(values, digits=3):
    return "[" + ", ".join("—" if v is None else ("%.*f" % (digits, v)) for v in values) + "]"


def main():
    operations = sorted(ARCHIVE.glob("*"), key=lambda p: p.stat().st_mtime)
    print("operations:", len(operations))
    for directory in operations:
        panel = load(directory / "panel.json", {}) or {}
        client = None
        client_dir = directory / "client"
        if client_dir.is_dir():
            files = sorted(client_dir.glob("*.json"))
            if files:
                client = load(files[-1])
        plan = load(directory / "plan.json")
        print("\n=== %s  %s" % (directory.name, panel.get("started_at")))
        print("kind=%s phase=%s exit=%s stop_reason=%s"
              % (panel.get("kind"), panel.get("phase"), panel.get("exit_code"), panel.get("stop_reason")))
        if plan:
            print("plan:", json.dumps(plan, ensure_ascii=False))
        if not client:
            print("client result: (none)")
            print("stdout tail:", (panel.get("stdout_tail") or [])[-6:])
            continue
        print("mode=%s reason=%s error_code=%s error_string=%s"
              % (client.get("mode"), client.get("reason"), client.get("trajectory_error_code"),
                 client.get("trajectory_error_string")))
        print("action_succeeded=%s waypoints_observed=%s stop_confirmed=%s"
              % (client.get("standard_jtc_action_succeeded"), client.get("waypoints_observed"),
                 client.get("stop_observations_confirmed")))
        start = [math.degrees(v) for v in (client.get("initial_model_rad") or [])]
        print("start_deg:", fmt(start))
        ranges = client.get("observed_delta_range_deg")
        if ranges:
            print("observed_delta_range_deg: J1%s J2%s J3%s J4%s"
                  % tuple(fmt([r[0], r[1]]) for r in ranges))
            print("observed_excursion_deg:   J1 %.3f  J2 %.3f  J3 %.3f  J4 %.3f"
                  % tuple(max(abs(r[0]), abs(r[1])) for r in ranges))
        final = client.get("final_delta_from_start_deg")
        if final:
            print("final_delta_from_start_deg:", fmt(final))
        for index, row in enumerate(client.get("waypoint_results") or [], 1):
            errors = row.get("max_error_deg") or []
            print("  point %d  arrival=%.2fs end=%.2fs samples=%s span=%.3fs errors=%s passed=%s"
                  % (index, row.get("arrival_s", -1), row.get("end_s", -1), row.get("samples"),
                     row.get("sampled_span_s") or 0.0, fmt(errors), row.get("passed")))
        print("command_id=%s plan_sha256=%s" % (client.get("command_id"),
                                               (client.get("plan_sha256") or "")[:16]))


if __name__ == "__main__":
    main()
