"""Bounded J1-J4 read-only audit using the existing read-only transport."""
import argparse
import json
import time
from pathlib import Path
from four_axis_monitor import Reader, load_mapping, IDS, WORKSPACE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM9')
    args = parser.parse_args()
    mapping = load_mapping()
    reader = Reader(args.port, mapping)
    started = time.perf_counter()
    reason = 'bounded_capture_complete'
    try:
        while time.perf_counter() - started < 12:
            now = time.perf_counter()
            reader.tick(now)
            if any(f['state_code'] != 0 for f in reader.state.feedback.values()):
                reason = 'non_disabled_state_observed_stop_queries'
                break
            time.sleep(.002)
        now = time.perf_counter()
        rows = {}
        for i in IDS:
            rows[str(i)] = dict(regs=reader.state.regs[i],
                conflicts=sorted(reader.state.conflicts[i]),
                feedback=reader.state.feedback.get(i),
                age_s=now-reader.state.at[i] if i in reader.state.at else None,
                fresh=reader.state.fresh(i, now), mapping=mapping[i])
        report = dict(schema=1, captured_local=time.strftime('%Y-%m-%d %H:%M:%S'),
            port=args.port, reason=reason, rows=rows,
            raw_log=str(reader.path), request_tx=reader.tx, received_frames=reader.rx,
            actuation_tx=0, parameter_write_tx=0, queried_ids=list(IDS),
            limitations='No physical identity uniqueness, direction, zero alignment, mechanical limit or watchdog action acceptance.')
        destination = reader.path.with_name(reader.path.stem+'_stage1.json')
        destination.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps(dict(report_path=str(destination), **report), ensure_ascii=False, indent=2))
    finally:
        reader.close()


if __name__ == '__main__':
    main()
