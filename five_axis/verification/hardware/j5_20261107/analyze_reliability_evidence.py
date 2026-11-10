"""Read-only analysis of the Codex J5 reliability evidence; never opens a device."""
import json
import statistics
import sys
from pathlib import Path

ROOT = Path("/mnt/d/CodexData/Projects/机械臂/slaihand_ros2_ws/tmp/j5_connect_20261107")


def load_json(name):
    return json.loads((ROOT / name).read_text(encoding="utf-8"))


def analyze_usbmon(path):
    pending = {}
    completions = []
    errors = []
    endpoints = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) < 5:
            continue
        key, stamp, event, endpoint, status = fields[:5]
        endpoints[endpoint] = endpoints.get(endpoint, 0) + 1
        if not endpoint.startswith("Bo:"):
            continue
        if event == "S":
            pending[key] = int(stamp)
        elif event == "C" and key in pending:
            start = pending.pop(key)
            latency_ms = (int(stamp) - start) / 1000.0
            completions.append(latency_ms)
            if status != "0":
                errors.append({"ms": latency_ms, "status": status})
    ordered = sorted(completions)

    def percentile(fraction):
        if not ordered:
            return None
        index = min(len(ordered) - 1, max(0, int(round(fraction * (len(ordered) - 1)))))
        return ordered[index]

    return {
        "completions": len(completions),
        "median_ms": statistics.median(ordered) if ordered else None,
        "p90_ms": percentile(.90),
        "p95_ms": percentile(.95),
        "p99_ms": percentile(.99),
        "max_ms": max(ordered) if ordered else None,
        "over50ms": sum(value > 50 for value in ordered),
        "over100ms": sum(value > 100 for value in ordered),
        "over500ms": sum(value > 500 for value in ordered),
        "errors": errors[-5:],
        "pending": len(pending),
        "endpoint_counts": endpoints,
    }


def main():
    windows = load_json("windows_direct_54ddb5c.json")
    wsl = load_json("wsl_comparison_54ddb5c.json")
    fault = load_json("j5_fault_54ddb5c.json")
    observation = load_json("j5_observation_54ddb5c.json")
    usbmon = analyze_usbmon(ROOT / "j5_usbmon_54ddb5c_target_only.txt")
    summary = {
        "evidence": "Codex 2026-11-07 J5 reliability comparison; offline only",
        "windows_native": {
            "transport": windows["transport"],
            "elapsed_s": windows["elapsed_s"],
            "requests": windows["requests"],
            "max_roundtrip_ms": windows["max_roundtrip_ms"],
            "over50ms": windows["over50ms"],
            "registers": windows["registers"],
            "state": windows["last_feedback"]["state"],
            "error": windows["error"],
        },
        "wsl_usbip": {
            "transport": wsl["transport"],
            "elapsed_s": wsl["elapsed_s"],
            "requests_before_error": wsl["requests"],
            "error": wsl["error"],
            "max_roundtrip_ms": wsl["max_roundtrip_ms"],
            "registers": wsl["registers"],
        },
        "deployed_54ddb5c_fault": {
            "tx": fault["j5"]["tx"],
            "rx": fault["j5"]["rx"],
            "transport": fault["j5"]["transport"]["state"],
            "fault": fault["j5"]["transport"]["fault"],
            "connected": fault["j5"]["connected"],
            "fresh": fault["j5"]["fresh"],
            "reconnect_available": fault["j5"]["reconnect_available"],
        },
        "after_one_reconnect": {
            "elapsed_s": observation["elapsed_s"],
            "tx": observation["last"]["tx"],
            "rx": observation["last"]["rx"],
            "connected": observation["last"]["connected"],
            "fresh": observation["last"]["fresh"],
            "last_write": observation["last"]["transport"]["last_write"],
        },
        "usbmon_target": usbmon,
        "conclusion": {
            "software_contract": "fault was correctly classified and port released; not the old silent-stale regression",
            "proven": [
                "Windows native COM9: 1207/1207 requests, max 7.2029ms, 0/1207 >50ms, state 0",
                "WSL USBIP comparison: 2 requests then SerialTimeoutException after reattach",
                "deployed 54ddb5c: 421/421 then 3 zero-byte EAGAIN -> WRITE_BACKPRESSURE",
                "after one explicit reconnect: 2364/2363, 120.16s, online/fresh",
                "USBmon target: real output completion tail with 50ms+ and 500ms+ outliers, no USB error codes",
            ],
            "not_proven": [
                "the exact WSL/USBIP forwarding sub-cause (vhci/usbip daemon/nat/scheduler/filter)",
                "absolute exclusion of the physical adapter or host USB stack",
                "that the deployed page is reliable for zero/limits capture",
            ],
            "next_diagnosis": "compare the same adapter locally attached to WSL versus the USBIP-forwarded path, and correlate USBmon tail with usbip/vhci host and guest logs; do not change writer threshold or protections",
        },
    }
    out = Path(__file__).resolve().parent / "evidence" / "j5_reliability_analysis.json"
    out.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
