"""Cross-language offline comparison against existing Python codec and real RX.

No Reader instances or serial modules are used. The original log is read only.
"""
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'verification/hardware'))
sys.path.insert(0, str(ROOT))
from four_axis_monitor import State, load_mapping, usb_packet


def prepare():
    source = ROOT/'tmp/four_axis_readonly/20261102_094926_978600.jsonl'
    mapping = load_mapping()
    state = State(mapping)
    rows = []
    for line in source.open(encoding='utf-8'):
        row = json.loads(line)
        if row.get('direction') != 'rx':
            continue
        data = bytes.fromhex(row['payload'])
        if len(data) != 8:
            continue
        rows.append((row['time'], row['can_id'], data.hex()))
        state.feed(row['can_id'], data, row['time'])
        if len(rows) == 200:
            break
    expected = {}
    for i in range(1, 5):
        assert state.ready(i) and i in state.feedback
        f, m = state.feedback[i], mapping[i]
        expected[str(i)] = [m['hardware_sign']*(f['position_rad']-m['zero_offset_rad']),
            m['hardware_sign']*f['velocity_rad_s'], m['hardware_sign']*f['torque_feedback_nm'],
            f['driver_temp_c'], f['motor_temp_c'], f['state_code'], state.at[i]]
    lines = [f"{i} {int(mapping[i]['hardware_sign'])} {mapping[i]['zero_offset_rad']:.17g}" for i in range(1, 5)]
    lines += [f'{at:.17g} {cid} {payload}' for at, cid, payload in rows]
    (HERE/'rx_fixture.txt').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    metadata = dict(source=str(source.relative_to(ROOT)), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        rx_frame_count=len(rows), calibration='current canonical mapping; not a historical zero/sign claim', expected=expected)
    (HERE/'python_expected.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def verify():
    packets = (HERE/'cpp_read_packets.txt').read_text().splitlines()
    assert len(packets) == 32
    for row in packets:
        axis, rid, raw = row.split()
        assert bytes.fromhex(raw) == usb_packet(int(axis), None if int(rid) == -1 else int(rid))
    expected = json.loads((HERE/'python_expected.json').read_text(encoding='utf-8'))
    decoded = (HERE/'cpp_decoded.txt').read_text().splitlines()
    assert len(decoded) == 4
    for row in decoded:
        axis, *actual = row.split()
        wanted = expected['expected'][axis]
        assert len(actual) == len(wanted)
        for a, b in zip(actual, wanted):
            assert math.isclose(float(a), b, rel_tol=1e-12, abs_tol=1e-10), (axis, a, b)
    audit_paths = list((HERE/'fake_audit').glob('*.jsonl'))
    assert audit_paths
    tx_count = 0
    for path in audit_paths:
        for line in path.read_text(encoding='utf-8').splitlines():
            event = json.loads(line)
            if event.get('direction') != 'tx':
                continue
            payload = bytes.fromhex(event['payload'])
            axis = payload[0]
            assert axis in (1, 2, 3, 4) and event['can_id'] == 0x7ff
            rid = None if payload[2] == 0xCC else payload[3]
            assert payload == usb_packet(axis, rid)[21:29]
            assert event['serial_bytes_written'] == 30 and event['drive_execution_ack'] is False
            tx_count += 1
    assert tx_count >= 32
    report = dict(status='PASS', read_packets_byte_identical=32,
        real_rx_frames_replayed=expected['rx_frame_count'], axes_decoder_parity=4,
        fake_bus_audit_tx_checked=tx_count, real_hardware_opened=False, source_sha256=expected['source_sha256'])
    (HERE/'parity_result.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print('PASS: all 32 read packets match Python byte-for-byte; 200 real RX frames replayed, four-axis decoded states match.')


if __name__ == '__main__':
    prepare() if '--prepare' in sys.argv else verify()
