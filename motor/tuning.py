"""Validated drafts and atomic updates for the bench tuning fields."""
from copy import deepcopy
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import stat
from tempfile import NamedTemporaryFile

from . import motor_core as core
from . import advanced_tuning
from .control_m0 import DEFAULT_POSITION_GAIN, DEFAULT_POSITION_VEL_GAIN
from .torque_m0 import validate_trial


RAMPS = (
    ("velocity.vel_ramp_rate", "Normal ramp [turn/s²] · shared"),
    ("velocity.fast_vel_ramp_rate", "Fast ramp [turn/s²] · shared"),
)
FIELDS = {
    "velocity": (("velocity.vel_gain", "Velocity P"),
                 ("velocity.vel_integrator_gain", "Velocity I"), *RAMPS),
    "position": (("controller.pos_gain", "Position P [s⁻¹]"),
                 ("velocity.position_vel_gain", "Position-mode velocity P"), RAMPS[0]),
    "torque": (("torque.kp", "Torque speed P"), ("torque.ki", "Torque speed I"),
               ("torque.slew_rate_nm_per_s", "Torque slew [N·m/s]"), *RAMPS),
}
DEFAULTS = {
    "controller.pos_gain": DEFAULT_POSITION_GAIN,
    "velocity.position_vel_gain": DEFAULT_POSITION_VEL_GAIN,
    "torque.slew_rate_nm_per_s": 1.0,
}


def _document():
    blob = core.CONFIG_PATH.read_bytes()
    return json.loads(blob), sha256(blob).hexdigest()


def _validate(raw, axis, mode):
    if mode not in FIELDS or (mode != "velocity" and axis != 0):
        raise ValueError("This tuning mode is unavailable for the selected axis.")
    cfg = core.validate_config(deepcopy(raw), axis)
    if mode != "velocity":
        validate_trial(cfg["velocity"]["min_command_rpm"], cfg)
    return cfg


def snapshot(axis, mode):
    raw, revision = _document()
    return _snapshot(raw, revision, axis, mode)


def _snapshot(raw, revision, axis, mode):
    cfg = _validate(raw, axis, mode)
    fields = []
    for key, label in FIELDS[mode]:
        group, name = key.split(".")
        value = cfg[group].get(name)
        fields.append((key, label, DEFAULTS[key] if value is None else value))
    return {"mode": mode, "axis": axis, "revision": revision, "fields": fields}

def editor_snapshot(axis, mode):
    raw, revision = _document()
    warning = ""
    try:
        info = _snapshot(raw, revision, axis, mode)
    except (ValueError, KeyError) as exc:
        warning = str(exc)
        info = {"mode": mode, "axis": axis, "revision": revision, "fields": []}
    info["advanced"] = advanced_tuning.descriptors(raw, axis, mode)
    info["warning"] = warning
    return info


def _candidate(axis, mode, revision, edits):
    raw, current_revision = _document()
    if revision != current_revision:
        raise ValueError("motor_config.json has changed. Reload parameters and reapply your edits.")
    if mode not in FIELDS or (mode != "velocity" and axis != 0):
        raise ValueError("This tuning mode is unavailable for the selected axis.")
    allowed = dict(FIELDS[mode])
    if not isinstance(edits, dict) or edits.keys() - allowed.keys():
        raise ValueError("Only tuning fields for the selected mode can be changed.")
    selected = raw["axes"][f"m{axis}"] if "axes" in raw else raw
    for key, value in edits.items():
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f"{allowed[key]} must be a finite number.")
        group, name = key.split(".")
        selected.setdefault(group, {})[name] = value
    return raw, _validate(raw, axis, mode)


def draft_config(axis, mode, draft):
    _, cfg = _candidate(axis, mode, draft["revision"], draft["edits"])
    return cfg


def save(axis, mode, draft, *, advanced=False):
    with core.exclusive_motion():
        if advanced:
            raw, revision = _document()
            if revision != draft["revision"]:
                raise ValueError("motor_config.json has changed. Reload parameters and reapply your edits.")
            advanced_tuning.apply(raw, axis, mode, draft["edits"])
        else:
            raw, _ = _candidate(axis, mode, draft["revision"], draft["edits"])
        path = core.CONFIG_PATH
        scratch = None
        try:
            with NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent, delete=False) as file:
                scratch = file.name
                os.fchmod(file.fileno(), stat.S_IMODE(path.stat().st_mode))
                json.dump(raw, file, indent=2, ensure_ascii=False, allow_nan=False)
                file.write("\n")
                file.flush()
                os.fsync(file.fileno())
            if sha256(path.read_bytes()).hexdigest() != draft["revision"]:
                raise ValueError("motor_config.json has changed. Reload parameters before saving.")
            os.replace(scratch, path)
        finally:
            if scratch is not None:
                Path(scratch).unlink(missing_ok=True)
        return editor_snapshot(axis, mode) if advanced else snapshot(axis, mode)
