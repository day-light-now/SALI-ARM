import json,sys,time
from pathlib import Path
from unittest.mock import patch
root=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(root/'verification/hardware/gravity_calibration_ui_20261106'),str(root/'src/slaihand_bringup/scripts')]
from test_supported_capture_panel import CapturePanelCase,IDENTITY
import control_panel as panel
import supported_capture as flow
import drag_teaching as teaching
results={}
case=CapturePanelCase();case.setUp()
try:
    operation=case._ready_operation()
    write=flow.write_ack_file
    def stop_before_publish(path,payload):
        case.backend._request_stop('counterexample stop before ack publication')
        return write(path,payload)
    with patch.object(case.backend,'_terminate',return_value=None),patch.object(panel.capture_flow,'write_ack_file',side_effect=stop_before_publish):
        response=case.backend.ack_supported_capture(operation.id,{'operator_disengaged':True})
    ack=flow.read_ack_file(operation.capture_ack_path)
    results['stop_before_ack_publication']={'accepted':response['accepted'],'current_generation':case.backend._stop_generation,'ack_generation':ack['panel_stop_generation'],'cancel_requested':operation.cancel_requested,'ack_file_exists':operation.capture_ack_path.exists()}
finally:case.tearDown()
case=CapturePanelCase();case.setUp()
try:
    operation=case._ready_operation()
    expected=teaching.identity_fingerprint(IDENTITY)
    operation.capture_command_id=None
    operation.capture_identity_sha256=expected
    case.backend._store('identity',{'at':time.monotonic(),'payload':dict(IDENTITY,boot_id='boot-replacement',mapping_sha256='b'*64)})
    case.backend._check_supported_capture(time.monotonic())
    response=case.backend.ack_supported_capture(operation.id,{'operator_disengaged':True})
    results['identity_change_before_first_ready']={'accepted':response['accepted'],'preview_identity_retained':operation.capture_identity_sha256==expected,'captured_boot':operation.capture_boot_id}
finally:case.tearDown()
print(json.dumps(results,indent=2))
(root/'verification/hardware/gravity_calibration_ui_20261106/evidence/codex_handoff_counterexamples.json').write_text(json.dumps(results,indent=2))
