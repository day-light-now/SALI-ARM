"""Pure loader/validator for the PROVISIONAL five-axis load declaration.

The declaration file is a *user estimate*, not a measurement.  This module only
reads and checks it; it never computes mass/geometry/COM and never promotes a
PROVISIONAL load to a formal gravity model.  It imports no ROS and does not touch
hardware, so both the launch admission path and offline tests can use it.

Contract (file ``config/load_3p5kg_provisional.yaml``):

* ``schema``                : ``slaihand_load_declaration/1``
* ``load_id``               : non-empty candidate id, never ``no_end_load``
* ``mass_kg``               : finite positive user estimate
* ``mass_source``           : must say it is an estimate (no fake measurement)
* ``geometry_status`` /
  ``com_status`` /
  ``mount_status``          : ``unknown`` until actually measured
* ``gravity_status``        : ``PROVISIONAL``
* ``formal_gravity_allowed``: ``false``
* ``gravity_fit_allowed``   : ``false``
* ``gravity_ff_alpha``      : ``0.0``
* ``float_enabled``         : ``false``
* ``measurements``          : empty list (nothing was measured for this load)
"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path

import yaml

SCHEMA = "slaihand_load_declaration/1"
STALE_LOAD_ID = "no_end_load"
UNKNOWN = "unknown"
PROVISIONAL = "PROVISIONAL"

CONFIG_FILENAME = "load_3p5kg_provisional.yaml"


class LoadDeclarationError(ValueError):
    """A declaration was rejected; ``code`` is a stable machine-readable reason."""

    def __init__(self, code, message):
        super().__init__("%s: %s" % (code, message))
        self.code = code
        self.message = message


def sha256_file(path):
    """SHA256 of the actual file bytes on disk (what the launch identity binds)."""
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _require(condition, code, message):
    if not condition:
        raise LoadDeclarationError(code, message)


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _text(value):
    if not isinstance(value, str):
        return None
    value = value.strip()
    return value or None


def _false(document, key, code):
    _require(document.get(key) is False, code, "负载声明 %s 必须显式为 false" % key)


def source_default_path():
    """Default file in a source checkout; ``None`` when installed elsewhere."""
    candidate = Path(__file__).resolve().parent.parent / "config" / CONFIG_FILENAME
    return candidate if candidate.is_file() else None


def load_declaration(path):
    """Read, validate and hash one load declaration file.

    Returns a normalised mapping.  ``sha256`` is the hash of the actual file
    bytes, so callers can bind exactly what was on disk.  Raises
    :class:`LoadDeclarationError` with a stable ``code``.
    """
    path = Path(path)
    _require(path.is_file(), "LOAD_DECLARATION_MISSING",
             "负载声明文件不存在：%s" % path)
    raw = path.read_bytes()
    try:
        document = yaml.safe_load(raw)
    except yaml.YAMLError as error:
        raise LoadDeclarationError("LOAD_DECLARATION_PARSE",
                                   "负载声明 YAML 解析失败：%s" % error)
    _require(isinstance(document, dict), "LOAD_DECLARATION_SHAPE",
             "负载声明必须是映射对象")

    _require(document.get("schema") == SCHEMA, "LOAD_DECLARATION_SCHEMA",
             "schema 必须是 %s" % SCHEMA)

    load_id = _text(document.get("load_id"))
    _require(load_id is not None, "LOAD_ID_MISSING", "load_id 缺失或为空")
    _require(load_id != STALE_LOAD_ID, "LOAD_ID_STALE",
             "拒绝复用旧四轴 no_end_load 档案；新负载必须使用新 load_id")

    mass = _number(document.get("mass_kg"))
    _require(mass is not None and mass > 0.0, "LOAD_MASS_INVALID",
             "mass_kg 必须是有限正数（用户估计值）")
    mass_source = _text(document.get("mass_source"))
    _require(mass_source is not None, "LOAD_MASS_SOURCE",
             "mass_source 必须说明质量来源（用户估计，不是测量）")
    _require("estimate" in mass_source.lower() or "estimated" in mass_source.lower(),
             "LOAD_MASS_SOURCE_UNMEASURED",
             "mass_source 必须明确这是估计值，不能伪装成测量值")
    _require("measure" not in mass_source.lower()
             or "not_measured" in mass_source.lower()
             or "unmeasured" in mass_source.lower(),
             "LOAD_MASS_SOURCE_MEASURED_CLAIM",
             "mass_source 不得声称已测量质量")

    for key in ("geometry_status", "com_status", "mount_status"):
        value = _text(document.get(key))
        _require(value is not None and value.lower() == UNKNOWN,
                 "LOAD_%s_NOT_UNKNOWN" % key.upper().replace("_STATUS", ""),
                 "%s 必须是 %s：未测量前不得写入几何/质心/安装数值" % (key, UNKNOWN))

    gravity_status = _text(document.get("gravity_status"))
    _require(gravity_status == PROVISIONAL, "LOAD_GRAVITY_STATUS",
             "gravity_status 必须是 %s（该 schema 只描述暂定负载）" % PROVISIONAL)
    _require("provisional" in load_id.lower(), "LOAD_ID_NOT_PROVISIONAL",
             "PROVISIONAL 负载的 load_id 必须显式带 provisional 标记")

    _false(document, "formal_gravity_allowed", "LOAD_FORMAL_GRAVITY_ALLOWED")
    _false(document, "gravity_fit_allowed", "LOAD_GRAVITY_FIT_ALLOWED")
    _false(document, "float_enabled", "LOAD_FLOAT_ENABLED")
    alpha = _number(document.get("gravity_ff_alpha"))
    _require(alpha is not None and alpha == 0.0, "LOAD_FF_ALPHA_NONZERO",
             "暂定负载的 gravity_ff_alpha 必须为 0.0")

    if document.get("position_only") is not None:
        _require(document.get("position_only") is True, "LOAD_NOT_POSITION_ONLY",
                 "暂定负载只允许 position_only: true")
    if document.get("measurements") is not None:
        measurements = document.get("measurements")
        _require(isinstance(measurements, list) and not measurements,
                 "LOAD_FABRICATED_MEASUREMENTS",
                 "properties 未测量时 measurements 必须为空列表")

    _require(document.get("do_not_reuse_load_id", STALE_LOAD_ID) == STALE_LOAD_ID,
             "LOAD_REUSE_GUARD_INVALID",
             "do_not_reuse_load_id 必须是 %s" % STALE_LOAD_ID)

    return {
        "path": str(path.resolve()),
        "sha256": hashlib.sha256(raw).hexdigest(),
        "document": document,
        "schema": SCHEMA,
        "load_id": load_id,
        "joint": _text(document.get("joint")),
        "mass_kg": mass,
        "mass_source": mass_source,
        "geometry_status": UNKNOWN,
        "com_status": UNKNOWN,
        "mount_status": UNKNOWN,
        "gravity_status": PROVISIONAL,
        "formal_gravity_allowed": False,
        "gravity_fit_allowed": False,
        "gravity_ff_alpha": 0.0,
        "float_enabled": False,
        "position_only": True,
    }
