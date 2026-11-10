"""Pure command policy and preview lifecycle, shared by all console pages.

Hardware motion intentionally has no adapter here. The existing CAN reader is
the only physical transport, and independently restricts every TX to IDs 1–4.
"""
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import time
import uuid
from .robot_contract import model_limits_deg

ROOT = Path(__file__).resolve().parents[1]
ACTIVE = (1, 2, 3, 4)
LIMITS = model_limits_deg()


def encoder_rad(raw16, pmax):
    if not isinstance(raw16, int) or not 0 <= raw16 <= 65535:
        raise ValueError('位置字段必须是 16 位无符号整数')
    if not math.isfinite(pmax) or pmax <= 0:
        raise ValueError('PMAX 必须来自有效的电机回复')
    return raw16 * 2 * pmax / 65535 - pmax


def model_rad(raw16, pmax, sign, zero):
    if sign not in (-1, 1) or not math.isfinite(zero):
        raise ValueError('软件映射无效')
    return sign * (encoder_rad(raw16, pmax) - zero)


class Rejected(ValueError):
    pass


@dataclass(frozen=True)
class Ticket:
    epoch: int
    targets: tuple
    nonce: str = field(default_factory=lambda: uuid.uuid4().hex[:10])
    plan: tuple = ()
    duration: float = 0.


class PreviewSlider:
    """Local draft only. No slider callback has access to a transport."""
    def __init__(self):
        self.epoch = 0
        self.values = {}
        self.state = 'IDLE'
        self.ticket = None

    def edit(self, joint, degree):
        if joint not in ACTIVE:
            raise Rejected('R1 J5 不查询／不使能／不控制')
        if not math.isfinite(degree):
            raise Rejected('R5 目标角度不是有限数')
        self.epoch += 1
        self.values[joint] = float(degree)
        self.ticket = Ticket(self.epoch, tuple(sorted(self.values.items())), duration=3.)
        self.state = 'PREVIEW'
        return self.ticket

    def clear(self):
        self.epoch += 1
        self.values.clear()
        self.ticket = None
        self.state = 'IDLE'

    def trajectory(self, points, duration):
        plan = tuple(validate_waypoints(points,duration))
        self.epoch += 1
        self.values = dict(enumerate(plan[0],1))
        self.ticket = Ticket(self.epoch,tuple(self.values.items()),plan=plan,duration=float(duration))
        self.state = 'PREVIEW'
        return self.ticket

    def matches(self, ticket):
        return (self.state == 'PREVIEW' and ticket is not None
                and ticket == self.ticket and ticket.epoch == self.epoch
                and ticket.targets == tuple(sorted(self.values.items())))


@dataclass
class Context:
    mode: str = 'sim'
    connected: bool = False
    enabled: bool = False
    fault: bool = False
    fresh: bool = False
    busy: bool = False


class CommandGateway:
    """Six ordered rejection rules; acceptance consumes a one-shot ticket."""
    def __init__(self, preview, audit=lambda *a, **k: None):
        self.preview = preview
        self.audit = audit

    def reason(self, ticket, ctx):
        if ticket is not None and any(i not in ACTIVE for i, _ in ticket.targets):
            return 'R1 J5 不查询／不使能／不控制'
        if ctx.mode != 'sim':
            return 'R2 实机接口仅只读；尚未验收运动下发'
        if not ctx.connected or not ctx.fresh:
            return 'R3 未连接或反馈过期'
        if not ctx.enabled or ctx.fault or ctx.busy:
            return 'R4 仿真未使能、有故障或正在执行'
        if ticket is not None:
            if not math.isfinite(ticket.duration) or not 1<=ticket.duration<=60:
                return 'R5 运动时长须为 1–60 秒'
            if not ticket.targets or len(dict(ticket.targets)) != len(ticket.targets):
                return 'R5 目标为空或含重复轴'
            for i, degree in ticket.targets:
                if not math.isfinite(degree) or not LIMITS[i-1][0] <= degree <= LIMITS[i-1][1]:
                    return f'R5 J{i} 超出模型限位或目标无效（不会自动裁剪）'
            if ticket.plan:
                try:
                    validate_waypoints(ticket.plan,ticket.duration)
                except ValueError as exc:
                    return 'R5 '+str(exc)
                if ticket.targets != tuple(enumerate(ticket.plan[0],1)):
                    return 'R5 轨迹与预览起点不一致'
        if not self.preview.matches(ticket):
            return 'R6 预览已失效；请重新设定并执行'
        return ''

    def commit(self, ticket, ctx):
        reason = self.reason(ticket, ctx)
        if reason:
            self.audit('SAFE', reason)
            raise Rejected(reason)
        result = dict(ticket.targets)
        self.audit('CMD', '接受仿真运动', command_id=ticket.nonce, targets=result,
                   waypoints=ticket.plan, duration=ticket.duration)
        self.preview.clear()
        self.preview.state = 'EXECUTING'
        return result


def minjerk(t):
    t = min(1., max(0., float(t)))
    return 10*t**3 - 15*t**4 + 6*t**5


def validate_waypoints(points, duration):
    if not math.isfinite(duration) or not 1 <= duration <= 60:
        raise Rejected('每段时间须为 1–60 秒')
    if not 1 <= len(points) <= 100:
        raise Rejected('需要 1–100 个路点')
    result = []
    for row in points:
        if len(row) != 4:
            raise Rejected('路点只接受 J1–J4，J5 不参与')
        row = tuple(float(v) for v in row)
        for i, v in enumerate(row):
            if not math.isfinite(v) or not LIMITS[i][0] <= v <= LIMITS[i][1]:
                raise Rejected(f'路点 J{i+1} 超出模型限位')
        result.append(row)
    return result


class Audit:
    """Append-and-flush audit with explicit normal/exceptional lifetime."""
    def __init__(self, directory=None):
        directory = Path(directory or ROOT/'tmp/desktop_console')
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / (time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]+'.jsonl')
        self.rows = []
        self.file = self.path.open('x', encoding='utf-8')

    def __call__(self, level, message, **fields):
        row = dict(time=time.time(), level=level, message=message, **fields)
        self.file.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
        self.file.flush()
        self.rows.append(row)
        if len(self.rows) > 2000:
            del self.rows[:500]
        return row

    def close(self):
        self.file.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        return False
