"""Wide five-axis position trajectories exposed in the panel group picker.

These are *position-only* candidates, not published formal gravity groups.  The
panel must load them as ordinary ``trajectory`` plans and must not mark them as
qualified gravity-sampling groups.  The files live in the installed ``config``
directory so the dropdown works from the installed panel as well as the source
tree.
"""
from __future__ import annotations

import json
from pathlib import Path

NAMES = (
    "WIDE_J5_60",
    "WIDE_J5_80",
    "WIDE_J1_J2_J5",
    "WIDE_J3_J4_J5",
    "WIDE_MIXED_GRID",
    "WIDE_J5_COMBINED",
)


def _config_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("slaihand_bringup"))
        candidate = share / "config" / "five_axis_trajectories"
        if candidate.is_dir():
            return candidate
    except Exception:
        pass
    # Source tree: scripts/ is a sibling of config/.
    candidate = Path(__file__).resolve().parent.parent / "config" / "five_axis_trajectories"
    if candidate.is_dir():
        return candidate
    raise FileNotFoundError("five_axis_trajectories config directory not found")


def group_document(name):
    if name not in NAMES:
        raise KeyError(name)
    path = _config_dir() / (name + ".json")
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("schema") != "slaihand_joint_plan/2":
        raise ValueError("%s is not a five-axis plan" % name)
    if document.get("coordinates") != "relative_start_deg":
        raise ValueError("%s must use relative_start_deg" % name)
    return document


def groups():
    return {name: group_document(name) for name in NAMES}
