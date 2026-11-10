"""Zero-stiffness float through the actual ROS, HTTP and MIT packet path."""
import json
import math
from pathlib import Path
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
import verify_control_panel as fixture
from control_panel_gravity import CoordinateMap, FourAxisGravityModel


def main():
    rclpy.init()
    observer = fixture.Observer()
    session = None
    work = Path(tempfile.mkdtemp(prefix='gravity-float-'))
    try:
        session = fixture.Session('float', work, observer)
        panel, plant = session.panel, session.plant
        status = panel.wait_status(lambda value: value.get('gravity_shadow', {}).get('driver', {}).get('float_available'), timeout=5)
        assert status['gravity_shadow']['driver']['float_available']
        code, _ = panel.call('POST', '/api/gravity-float', {'confirmations': fixture.confirmations()})
        assert code == 400 and plant.fc == [0] * 4
        confirmations = dict(fixture.confirmations(), gravity_float=True)
        code, body = panel.call('POST', '/api/gravity-float', {'confirmations': confirmations})
        assert code == 202, body
        moved = False
        active = False
        deadline = time.monotonic() + 22
        while time.monotonic() < deadline:
            observer.spin(0.03)
            status = panel.status()
            operation = status.get('operation') or {}
            driver = status['gravity_shadow']['driver']
            if driver.get('active') and driver.get('gravity_float'):
                active = True
                assert driver['alpha'] == 1 and driver['ramp'] == 3
                assert status['gravity_shadow']['metadata']['kp'] == 0
            if status['ros']['hold_ready'] and not moved:
                # Simulate an operator moving J1 by > the position-mode 4-degree gate.
                plant.raw[0] += math.radians(6)
                moved = True
            if operation and not operation.get('running'):
                break
        assert active and moved
        assert operation['exit_code'] == 0, operation
        assert operation['result']['mode'] == 'gravity_float'
        assert operation['result']['stop_observations_confirmed']
        packets = []
        for path in (session.dir / 'audit').glob('*.jsonl'):
            with path.open() as stream:
                for line in stream:
                    row = json.loads(line)
                    if row.get('kind') == 'TX_WRITTEN' and row.get('role') == 1:
                        payload = bytes.fromhex(row['payload'])
                        kp = (((payload[3] & 15) << 8) | payload[4]) * 500 / 4095
                        kd = ((payload[5] << 4) | (payload[6] >> 4)) * 5 / 4095
                        assert kp == 0 and 0 <= kd <= 5, row
                        packets.append(row)
        assert len(packets) > 500
        share = Path(get_package_share_directory('slaihand_bringup'))
        model = FourAxisGravityModel(
            Path(get_package_share_directory('slaihand_description')) / 'urdf/slaihand_four_axis.urdf',
            share / 'config/gravity_identified_candidate.json', gravity_base=(0., 9.80665, 0.),
            fixed_joint_offsets={'slaihand_joint3': math.pi})
        mapping = CoordinateMap(share / 'config/hardware_mapping.yaml')
        last = {row['can_id']: bytes.fromhex(row['payload']) for row in packets}
        raw = [int.from_bytes(last[axis][:2], 'big') * 25 / 65535 - 12.5 for axis in range(1, 5)]
        expected = model.compute(mapping.to_model_angle(raw))['tau_model_nm']
        decoded = []
        for index in range(4):
            payload = last[index + 1]
            maximum = 200 if index < 2 else 54
            torque = (((payload[6] & 15) << 8) | payload[7]) * 2 * maximum / 4095 - maximum
            assert abs(torque - mapping.signs[index] * expected[index]) <= maximum / 4095 + .003
            decoded.append(torque)
        assert not plant.errors and plant.fc == [1] * 4
        final = panel.status()['gravity_shadow']['driver']
        assert not final['active'] and final['applied'] == [0] * 4
        result = {'passed': True, 'mode': 'gravity_float', 'kp': 0, 'kd': 5,
                  'alpha': 1, 'ramp_s': 3, 'drag_deg': 6,
                  'zero_stiffness_written_packets': len(packets),
                  'decoded_full_ramp_tau_nm': decoded,
                  'stop_confirmed': True, 'physical_port_opened': False}
        (Path(__file__).parent / 'evidence/gravity_float_pty.json').write_text(json.dumps(result, indent=2))
        print('PASS', json.dumps(result))
    finally:
        if session:
            session.close()
        observer.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
