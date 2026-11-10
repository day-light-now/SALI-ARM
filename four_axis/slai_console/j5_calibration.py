"""Manual, read-only J5 calibration record.

J5 has no canonical sign/zero/travel yet. This module never guesses them and
never commands a motor. The operator manually places J5 at four poses while the
drive is disabled; a read-only monitor supplies the raw encoder readings; the
operator confirms each pose; only then is a usable record written.
"""
from dataclasses import dataclass, asdict
from datetime import datetime
import json
import math
from pathlib import Path

DEFAULT_PATH = (Path(__file__).resolve().parents[1] / 'verification' / 'hardware' /
                'five_axis_20261102' / 'j5_calibration.json')
MIN_DIRECTION_DELTA_RAD = .05
MIN_TRAVEL_SPAN_RAD = .10


@dataclass(frozen=True)
class J5Calibration:
    version: int
    joint: int
    sign: int
    zero_offset_rad: float
    travel_min_rad: float
    travel_max_rad: float
    zero_position_rad: float
    positive_position_rad: float
    travel_min_position_rad: float
    travel_max_position_rad: float
    zero_payload: str
    positive_payload: str
    travel_min_payload: str
    travel_max_payload: str
    user_zero_confirmed: bool
    user_direction_confirmed: bool
    user_limits_confirmed: bool
    created_at: str
    notes: str = ''

    def is_usable(self):
        try:
            if self.version != 1 or self.joint != 5:
                return False
            if self.sign not in (-1, 1):
                return False
            values = (self.zero_offset_rad, self.travel_min_rad, self.travel_max_rad,
                      self.zero_position_rad, self.positive_position_rad,
                      self.travel_min_position_rad, self.travel_max_position_rad)
            if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
                return False
            if not all(v is True for v in (self.user_zero_confirmed,
                                           self.user_direction_confirmed,
                                           self.user_limits_confirmed)):
                return False
            if not self.travel_min_rad < self.travel_max_rad:
                return False
            if not self.travel_max_rad-self.travel_min_rad >= MIN_TRAVEL_SPAN_RAD-1e-12:
                return False
            if not self.travel_min_rad <= 0. <= self.travel_max_rad:
                return False
            return True
        except Exception:
            return False

    def to_dict(self):
        data = asdict(self)
        data['validated'] = self.is_usable()
        return data

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise ValueError('J5 calibration must be an object')
        fields = {name: data[name] for name in cls.__dataclass_fields__ if name in data}
        if set(fields) != set(cls.__dataclass_fields__):
            missing = set(cls.__dataclass_fields__)-set(fields)
            raise ValueError(f'J5 calibration missing fields: {sorted(missing)}')
        record = cls(**fields)
        if not record.is_usable():
            raise ValueError('J5 calibration is incomplete or inconsistent')
        return record


def build_j5_calibration(*, zero_position_rad, positive_position_rad,
                         travel_min_position_rad, travel_max_position_rad,
                         zero_payload='', positive_payload='',
                         travel_min_payload='', travel_max_payload='',
                         user_zero_confirmed, user_direction_confirmed,
                         user_limits_confirmed, notes='', now=None):
    """Build a record from four manual poses; refuse ambiguity instead of guessing."""
    values = (zero_position_rad, positive_position_rad,
              travel_min_position_rad, travel_max_position_rad)
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        raise ValueError('all J5 calibration positions must be finite')
    if not all(v is True for v in (user_zero_confirmed, user_direction_confirmed,
                                   user_limits_confirmed)):
        raise ValueError('zero, direction and travel limits must each be confirmed')
    delta = positive_position_rad-zero_position_rad
    if abs(delta) < MIN_DIRECTION_DELTA_RAD-1e-12:
        raise ValueError('positive-direction move is too small to infer a sign')
    sign = 1 if delta > 0 else -1
    zero = float(zero_position_rad)
    q_min = sign*(travel_min_position_rad-zero)
    q_max = sign*(travel_max_position_rad-zero)
    if q_min > q_max:
        q_min, q_max = q_max, q_min
    if q_max-q_min < MIN_TRAVEL_SPAN_RAD-1e-12:
        raise ValueError('recorded travel span is too small')
    if not q_min-1e-12 <= 0. <= q_max+1e-12:
        raise ValueError('the recorded zero pose is outside the recorded travel')
    stamp = now or datetime.now().astimezone().isoformat(timespec='seconds')
    record = J5Calibration(
        version=1, joint=5, sign=sign, zero_offset_rad=zero,
        travel_min_rad=q_min, travel_max_rad=q_max,
        zero_position_rad=float(zero_position_rad),
        positive_position_rad=float(positive_position_rad),
        travel_min_position_rad=float(travel_min_position_rad),
        travel_max_position_rad=float(travel_max_position_rad),
        zero_payload=str(zero_payload), positive_payload=str(positive_payload),
        travel_min_payload=str(travel_min_payload),
        travel_max_payload=str(travel_max_payload),
        user_zero_confirmed=True, user_direction_confirmed=True,
        user_limits_confirmed=True, created_at=stamp, notes=str(notes))
    if not record.is_usable():
        raise ValueError('built J5 calibration is not usable')
    return record


def save_j5_calibration(record, path=DEFAULT_PATH):
    if not isinstance(record, J5Calibration) or not record.is_usable():
        raise ValueError('refusing to save an unusable J5 calibration')
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(record.to_dict(), ensure_ascii=False, indent=2)+'\n',
                         encoding='utf-8')
    temporary.replace(path)
    return path


def load_j5_calibration(path=DEFAULT_PATH, required=False):
    path = Path(path)
    if not path.is_file():
        if required:
            raise ValueError(f'J5 calibration missing: {path}')
        return None
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
    except Exception as exc:
        raise ValueError(f'J5 calibration unreadable: {exc}') from exc
    return J5Calibration.from_dict(data)
