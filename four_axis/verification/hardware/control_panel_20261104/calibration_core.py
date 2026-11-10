"""Compatibility entry for the installed production calibration module."""
from pathlib import Path
_source = Path(__file__).resolve().parents[3] / "src/slaihand_bringup/scripts" / 'calibration_core.py'
__file__ = str(_source)
exec(compile(_source.read_text(encoding="utf-8"), str(_source), "exec"), globals())
