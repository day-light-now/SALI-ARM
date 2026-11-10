"""Operator-reviewed five-axis limits/gains; a template never authorizes motion.

`user_confirmed` must be true in the loaded JSON. The J1-J4 travel may narrow,
but never widen, `core.LIMITS`. J5 travel comes from the manual calibration
record; the config may only narrow it. A missing/incomplete J5 record means the
five-axis controller runs J1-J4 and J5 stays monitor-only.
"""
from dataclasses import dataclass
import json
import math
from pathlib import Path

from .core import LIMITS
from .robot_contract import operating_limits_deg
from .j5_calibration import load_j5_calibration
from .five_axis_mit import AxisEnvelope

DEFAULT_PATH = (Path(__file__).resolve().parents[1] / 'verification' / 'hardware' /
                'five_axis_20261102' / 'five_axis_config.json')
AXES5 = (1, 2, 3, 4, 5)
_TEMPLATE_TRAVEL = dict(enumerate(operating_limits_deg(), 1))


@dataclass(frozen=True)
class AxisSetting:
    travel_min_deg: float | None
    travel_max_deg: float | None
    estimate_guard_nm: float
    kp: float
    kd: float
    speed_rad_s: float = .02
    following_error_deg: float = 4.
    feedback_speed_rad_s: float = .2
    temperature_c: float = 50.
    arrival_error_deg: float = .5

    def check(self, axis):
        values = (self.estimate_guard_nm, self.kp, self.kd, self.speed_rad_s,
                  self.following_error_deg, self.feedback_speed_rad_s,
                  self.temperature_c, self.arrival_error_deg)
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            raise ValueError(f'J{axis} config contains a non-finite value')
        if self.estimate_guard_nm <= 0:
            raise ValueError(f'J{axis} estimate guard must be positive')
        if not 0 <= self.kp <= 500 or not 0 <= self.kd <= 5:
            raise ValueError(f'J{axis} gain outside MIT protocol')
        if not 0 < self.speed_rad_s <= .02+1e-12:
            raise ValueError(f'J{axis} speed cannot exceed 0.02rad/s')
        if not 0 < self.following_error_deg <= 4+1e-9:
            raise ValueError(f'J{axis} following error cannot exceed 4deg')
        if not 0 < self.feedback_speed_rad_s <= 1.0+1e-9:
            raise ValueError(f'J{axis} feedback abort cannot exceed 1.0rad/s')
        if not 0 < self.temperature_c <= 50:
            raise ValueError(f'J{axis} temperature cannot exceed 50C')
        if not 0 < self.arrival_error_deg <= 1:
            raise ValueError(f'J{axis} arrival error must be in (0,1deg]')
        if (self.travel_min_deg is None) != (self.travel_max_deg is None):
            raise ValueError(f'J{axis} travel needs both endpoints or neither')


@dataclass(frozen=True)
class FiveAxisSettings:
    user_confirmed: bool
    axes: dict
    source: str
    note: str = ''
    j5_enabled: bool = True

    def check(self):
        if not self.user_confirmed:
            raise ValueError('five-axis config is not operator-confirmed; refusing to build envelopes')
        if set(self.axes) != set(AXES5):
            raise ValueError('config must contain exactly J1..J5')
        for i in AXES5:
            if not isinstance(self.axes[i], AxisSetting):
                raise ValueError(f'J{i} config type invalid')
            self.axes[i].check(i)


def _default_axis(axis):
    if axis in _TEMPLATE_TRAVEL:
        low, high = _TEMPLATE_TRAVEL[axis]
    else:
        low, high = None, None
    return AxisSetting(low, high, 2.0 if axis != 5 else 1.0, 32., 2.)


def template_settings():
    return FiveAxisSettings(
        False, {i: _default_axis(i) for i in AXES5}, 'built-in template',
        'Template only: review every axis, especially J5 travel and estimate guards, '
        'then set user_confirmed=true. Estimate guards are observed-state abort '
        'thresholds, not continuous drive torque clamps.')


def ensure_template(path=DEFAULT_PATH):
    path = Path(path)
    if path.exists():
        return path
    settings = template_settings()
    data = dict(user_confirmed=False, j5_enabled=True, note=settings.note,
                axes={str(i): dict(travel_min_deg=settings.axes[i].travel_min_deg,
                                   travel_max_deg=settings.axes[i].travel_max_deg,
                                   estimate_guard_nm=settings.axes[i].estimate_guard_nm,
                                   kp=settings.axes[i].kp, kd=settings.axes[i].kd)
                      for i in AXES5})
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    return path


def load_settings(path=DEFAULT_PATH):
    path = Path(path)
    if not path.is_file():
        raise ValueError(f'five-axis config missing: {path}')
    data = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict) or not isinstance(data.get('axes'), dict):
        raise ValueError('five-axis config must contain an axes object')
    if set(data['axes']) != {str(i) for i in AXES5}:
        raise ValueError('five-axis config must contain J1..J5 exactly')
    axes = {}
    for i in AXES5:
        raw = data['axes'][str(i)]
        if not isinstance(raw, dict):
            raise ValueError(f'J{i} config must be an object')
        axes[i] = AxisSetting(
            travel_min_deg=raw.get('travel_min_deg'),
            travel_max_deg=raw.get('travel_max_deg'),
            estimate_guard_nm=raw.get('estimate_guard_nm'),
            kp=raw.get('kp'),
            kd=raw.get('kd'),
            speed_rad_s=raw.get('speed_rad_s', .02),
            following_error_deg=raw.get('following_error_deg', 4.),
            feedback_speed_rad_s=raw.get('feedback_speed_rad_s', .2),
            temperature_c=raw.get('temperature_c', 50.),
            arrival_error_deg=raw.get('arrival_error_deg', .5))
    settings = FiveAxisSettings(bool(data.get('user_confirmed') is True), axes,
                                str(path), str(data.get('note', '')),
                                bool(data.get('j5_enabled', True)))
    settings.check()
    return settings


def build_envelopes(settings, calibration=None):
    """Return (envelopes, active_axes); never widen model/J5-calibrated travel."""
    settings.check()
    envelopes = {}
    default_travel = operating_limits_deg()
    for i in (1, 2, 3, 4):
        setting = settings.axes[i]
        outer_low, outer_high = LIMITS[i-1]
        low = setting.travel_min_deg if setting.travel_min_deg is not None else default_travel[i-1][0]
        high = setting.travel_max_deg if setting.travel_max_deg is not None else default_travel[i-1][1]
        if not outer_low-1e-9 <= low < high <= outer_high+1e-9:
            raise ValueError(f'J{i} travel must stay inside model limits '
                             f'[{outer_low},{outer_high}]')
        envelopes[i] = _envelope(setting, low, high)
    j5_usable = calibration is not None and calibration.is_usable()
    if j5_usable and settings.j5_enabled:
        setting = settings.axes[5]
        low = math.degrees(calibration.travel_min_rad)
        high = math.degrees(calibration.travel_max_rad)
        if setting.travel_min_deg is not None:
            low = max(low, setting.travel_min_deg)
        if setting.travel_max_deg is not None:
            high = min(high, setting.travel_max_deg)
        if not low < high:
            raise ValueError('J5 config travel does not overlap the calibration travel')
        envelopes[5] = _envelope(setting, low, high)
        return envelopes, (1, 2, 3, 4, 5)
    return envelopes, (1, 2, 3, 4)


def _envelope(setting, low_deg, high_deg):
    return AxisEnvelope(math.radians(low_deg), math.radians(high_deg),
                        setting.speed_rad_s, setting.estimate_guard_nm,
                        math.radians(setting.following_error_deg), setting.kp,
                        setting.kd, setting.temperature_c,
                        setting.feedback_speed_rad_s,
                        math.radians(setting.arrival_error_deg))
