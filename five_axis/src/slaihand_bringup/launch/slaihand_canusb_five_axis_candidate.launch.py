"""Candidate five-axis admission launch (new; existing launch path unchanged).

This launch is the only candidate entry point that may start the five-axis
driver with J5 *motion-qualified*.  It:

1. uses ``config/load_3p5kg_provisional.yaml`` as the authoritative,
   position-only load identity source: the actual file SHA256 becomes
   ``mass_profile_sha256`` and the declaration's ``load_id`` is passed through
   (the policy-file load_id stays a fallback for the legacy launch only);
2. requires ``gravity_ff_alpha == 0`` and ``gravity_float_enabled == false`` and
   refuses any provisional-load attempt to enable gravity fitting/feedforward;
3. takes J5 sign/zero/finite limits from ``hardware_mapping.yaml`` only; manual
   ``j5_hardware_sign``/``j5_zero_offset_rad``/``j5_limit_*`` overrides and raw
   ``j5_calibration_sha256`` hex strings are rejected;
4. with ``j5_artifact:=<path>`` validates that artifact (canonical payload SHA,
   mapping/model/policy/load hashes, timeout evidence, device, age, J5
   sign/zero/limits) and only then passes ``j5_qualified:=true`` plus the
   artifact payload SHA256 to the existing driver launch;
5. without an artifact it starts unqualified/feedback-only, which is the
   production default (``j5_qualified=false``).

No real CANUSB port is opened or claimed by this file; it only wires the launch
description.  The actual driver still performs its own read-only preflight and
refuses motion unless every J5 gate passes.
"""
import json
import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration


def _module_search_paths():
    """Source-tree scripts first (dev), then the installed lib directory."""
    paths = []
    source = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          "scripts")
    paths.append(source)
    try:
        installed = os.path.join(get_package_prefix("slaihand_bringup"),
                                 "lib", "slaihand_bringup")
        paths.append(installed)
    except Exception:  # pragma: no cover - ament index always present under ros2 launch
        pass
    return paths


for _path in reversed(_module_search_paths()):
    if os.path.isdir(_path) and _path not in sys.path:
        sys.path.insert(0, _path)

import j5_motion_artifact as admission  # noqa: E402
import load_declaration  # noqa: E402

_LOGGER = get_logger(__name__)
# Sentinel used to prove a launch argument was not overridden by hand.
_FROM_MAPPING = "__hardware_mapping__"
# Sentinel default: omitting mass_profile_sha256 means "use the declaration's
# actual file SHA256"; an explicitly empty value is refused (except test-only).
_FROM_DECLARATION = "__load_declaration_sha256__"
_GUARD_ARGS = ("j5_hardware_sign", "j5_zero_offset_rad",
               "j5_limit_lower", "j5_limit_upper")


def _boolean(text, name):
    value = str(text).strip().lower()
    if value in ("true", "1"):
        return True
    if value in ("false", "0", ""):
        return False
    raise RuntimeError("%s 必须是 true/false，收到 %r" % (name, text))


def _paths():
    bringup = get_package_share_directory("slaihand_bringup")
    description = get_package_share_directory("slaihand_description")
    return {
        "bringup": bringup,
        "mapping": os.path.join(bringup, "config", "hardware_mapping.yaml"),
        "policy": os.path.join(bringup, "config", "canusb_five_axis_motion_policy.yaml"),
        "load": os.path.join(bringup, "config", load_declaration.CONFIG_FILENAME),
        "model": os.path.join(description, "urdf", "slaihand_five_axis.urdf"),
        "launch": os.path.join(bringup, "launch",
                               "slaihand_canusb_five_axis_motion.launch.py"),
    }


def _reject_raw_j5_knobs(context):
    """Refuse raw/arbitrary J5 qualification inputs before touching anything."""
    qualified = LaunchConfiguration("j5_qualified").perform(context).strip().lower()
    if qualified not in ("", "false", "0"):
        raise RuntimeError(
            "拒绝 j5_qualified=%s：J5 运动资格必须由 j5_artifact 校验后推导，"
            "不允许裸布尔覆盖" % qualified)
    raw_hash = LaunchConfiguration("j5_calibration_sha256").perform(context).strip()
    if raw_hash:
        raise RuntimeError(
            "拒绝任意 j5_calibration_sha256（%s...）：候选 launch 只接受 "
            "j5_artifact=<path>，校验后使用 artifact payload SHA256" % raw_hash[:12])
    for name in _GUARD_ARGS:
        value = LaunchConfiguration(name).perform(context).strip()
        if value != _FROM_MAPPING:
            raise RuntimeError(
                "拒绝 %s=%r：J5 sign/zero/limits 只能来自 hardware_mapping.yaml" % (name, value))


def _candidate(context):
    paths = _paths()
    port = LaunchConfiguration("port").perform(context).strip()
    if not port.startswith("/dev/"):
        raise RuntimeError("Explicit Linux/WSL CANUSB port required (/dev/...); "
                           "candidate launch 不会自动选择端口")
    log_directory = LaunchConfiguration("log_directory").perform(context)

    declared_load = (LaunchConfiguration("load_declaration").perform(context).strip()
                     or paths["load"])
    artifact = LaunchConfiguration("j5_artifact").perform(context).strip()
    evidence = LaunchConfiguration("timeout_evidence").perform(context).strip()
    max_age_raw = LaunchConfiguration("max_artifact_age_s").perform(context).strip()
    try:
        max_age = float(max_age_raw)
    except ValueError:
        raise RuntimeError("max_artifact_age_s 必须是数值，收到 %r" % max_age_raw)
    if not (max_age > 0.0):
        raise RuntimeError("max_artifact_age_s 必须为正数")
    test_only = _boolean(LaunchConfiguration("test_only_mass_profile").perform(context),
                         "test_only_mass_profile")

    raw_mass_hash = LaunchConfiguration("mass_profile_sha256").perform(context).strip()
    if raw_mass_hash == _FROM_DECLARATION:
        pass  # omitted: resolve_candidate_identity uses the actual file SHA256
    elif raw_mass_hash == "":
        if not test_only:
            raise RuntimeError(
                "拒绝空的显式 mass_profile_sha256：省略该参数即自动使用负载声明"
                "文件实际 SHA256；仅 test_only_mass_profile=true 的测试路径可显式传空")
    else:
        pass  # explicit non-empty values are checked against the actual hash
    mass_override = "" if raw_mass_hash == _FROM_DECLARATION else raw_mass_hash

    _reject_raw_j5_knobs(context)
    try:
        report = admission.resolve_candidate_identity(
            mapping=paths["mapping"],
            model=paths["model"],
            policy=paths["policy"],
            load_profile=declared_load,
            port=port,
            mass_profile_sha256=mass_override,
            test_only_mass_profile=test_only,
            j5_artifact=artifact,
            timeout_evidence=evidence,
            max_age_s=max_age)
    except (load_declaration.LoadDeclarationError, admission.AdmissionError) as error:
        raise RuntimeError("五轴候选准入被拒绝 [%s]: %s" % (error.code, error.message))

    arguments = dict(report["launch_arguments"])
    arguments["port"] = port
    arguments["log_directory"] = log_directory
    admission_report = report["admission"]
    _LOGGER.info(json.dumps({
        "kind": report["kind"],
        "load_id": report["load_id"],
        "mass_profile_sha256": report["mass_profile_sha256"],
        "test_only_mass_profile": report["test_only_mass_profile"],
        "j5_qualified": admission_report["j5_qualified"],
        "j5_calibration_sha256": admission_report["artifact_sha256"],
        "artifact": admission_report["artifact"],
        "device": admission_report["device"],
        "boot_id": admission_report["boot_id"],
        "reason": admission_report["reason"],
    }, ensure_ascii=False))
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(paths["launch"]),
            launch_arguments=arguments.items()),
    ]


def generate_launch_description():
    paths = _paths()
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value=""),
        DeclareLaunchArgument(
            "log_directory",
            default_value=os.path.expanduser("~/.ros/slaihand/canusb_five_axis_candidate")),
        # Authoritative position-only load identity source (actual file SHA256
        # becomes mass_profile_sha256; load_id comes from the declaration).
        DeclareLaunchArgument("load_declaration", default_value=paths["load"]),
        # Explicit hash check: omitted (sentinel) means "use the actual
        # declaration SHA256"; a non-empty explicit value must match it unless
        # test_only_mass_profile=true; an explicitly empty value is refused.
        DeclareLaunchArgument("mass_profile_sha256", default_value=_FROM_DECLARATION),
        DeclareLaunchArgument("test_only_mass_profile", default_value="false"),
        # Optional validated J5 admission artifact.  Empty = unqualified,
        # feedback-only (production default j5_qualified=false).
        DeclareLaunchArgument("j5_artifact", default_value=""),
        DeclareLaunchArgument(
            "timeout_evidence", default_value="",
            description="optional j5_timeout_setup_evidence.json to re-bind while validating"),
        DeclareLaunchArgument(
            "max_artifact_age_s", default_value="1800.0",
            description="reject artifacts/evidence older than this many seconds (default 30 min)"),
        # Refusal guards: these names exist only so raw manual values fail loudly.
        DeclareLaunchArgument(
            "j5_qualified", default_value="false",
            description="refused: derived from validated j5_artifact only"),
        DeclareLaunchArgument(
            "j5_calibration_sha256", default_value="",
            description="refused: artifact payload SHA256 is used instead"),
        DeclareLaunchArgument("j5_hardware_sign", default_value=_FROM_MAPPING,
                              description="refused: from hardware_mapping.yaml"),
        DeclareLaunchArgument("j5_zero_offset_rad", default_value=_FROM_MAPPING,
                              description="refused: from hardware_mapping.yaml"),
        DeclareLaunchArgument("j5_limit_lower", default_value=_FROM_MAPPING,
                              description="refused: from hardware_mapping.yaml"),
        DeclareLaunchArgument("j5_limit_upper", default_value=_FROM_MAPPING,
                              description="refused: from hardware_mapping.yaml"),
        OpaqueFunction(function=_candidate),
    ])
