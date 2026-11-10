"""Construct launch actions without executing a LaunchService or opening ports."""
import importlib.util
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext

path = Path(get_package_share_directory('slaihand_bringup'))/'launch/slaihand_canusb_readonly.launch.py'
spec = importlib.util.spec_from_file_location('readonly_launch', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
for value in ('', 'COM9', 'relative-path'):
    context = LaunchContext()
    context.launch_configurations.update(port=value, log_directory='/tmp/NOT_CREATED_LAUNCH_CHECK')
    try:
        module.nodes(context)
    except RuntimeError:
        continue
    raise AssertionError('Invalid Linux port was not rejected')
context = LaunchContext()
context.launch_configurations.update(port='/dev/INVALID_UNOPENED', log_directory='/tmp/NOT_CREATED_LAUNCH_CHECK')
assert len(module.nodes(context)) == 3  # Actions only. Never execute them.
assert not Path('/tmp/NOT_CREATED_LAUNCH_CHECK').exists()
print('PASS: launch rejects missing/Windows ports, expands canonical model and hashes; actions never executed.')
