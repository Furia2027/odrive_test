"""Editable config fields for the docked bench editor."""
from copy import deepcopy
import math

from . import motor_core as core
from .torque_m0 import validate_trial


AXIS_GROUPS = ("controller", "velocity", "torque", "hardware", "safety", "calibration", "sensorless", "anticogging")
TORQUE = ("kp", "ki", "slew_rate_nm_per_s", "firmware_limiter_gain", "overspeed_ratio", "torque_limit_nm")
SHARED = ("board", "runtime")
UNITS = {
    "pos_gain": "s⁻¹", "vel_gain": "N·m/(turn/s)", "position_vel_gain": "N·m/(turn/s)",
    "vel_integrator_gain": "N·m/turn", "inertia": "N·m/(turn/s²)",
    "input_filter_bandwidth": "s⁻¹", "gain_scheduling_width": "turns",
    "vel_ramp_rate": "turn/s²", "fast_vel_ramp_rate": "turn/s²", "torque_ramp_rate": "N·m/s",
    "kp": "N·m/(turn/s)", "ki": "N·m/turn", "firmware_limiter_gain": "N·m/(turn/s)",
    "torque_limit_nm": "N·m", "torque_lim": "N·m", "torque_constant": "N·m/A",
    "max_regen_current": "A", "dc_max_negative_current": "A", "dc_max_positive_current": "A",
    "current_lim": "A", "current_lim_margin": "A", "calibration_current": "A",
    "resistance_calib_max_voltage": "V", "dc_bus_overvoltage_trip_level": "V",
    "brake_resistance": "Ω", "motor_kv": "RPM/V", "cpr": "counts/turn",
    "bandwidth": "s⁻¹", "current_control_bandwidth": "rad/s",
    "calib_pos_threshold": "counts", "calib_vel_threshold": "counts/s",
}
SUFFIX_UNITS = (("_nm_per_s", "N·m/s"), ("_nm_a", "N·m/A"), ("_rpm", "RPM"), ("_ohm", "Ω"),
                ("_hz", "Hz"), ("_s", "s"), ("_v", "V"), ("_a", "A"), ("_h", "H"))
OPTIONAL = {"hardware": ("calib_range", "torque_constant"), "velocity": ("position_vel_gain",),
            "torque": TORQUE, "sensorless": ("vel_gain", "vel_integrator_gain", "verified")}
NULLABLE = {"hardware.cpr", "hardware.bandwidth", "hardware.calib_range", "hardware.torque_constant",
            "sensorless.vel_gain", "sensorless.vel_integrator_gain", "board.dc_max_positive_current"}


def profile(raw, axis):
    return raw.get("axes", {}).get(f"m{axis}", {}) if "axes" in raw else raw if axis == 0 else {}


def descriptors(raw, axis, mode):
    selected = profile(raw, axis)
    verified = bool(selected) and selected.get("hardware_verified", axis == 0) is True
    fields = []

    def add(group, name, value, *, kind="number", nullable=False, locked=False):
        unit = UNITS.get(name, next((unit for suffix, unit in SUFFIX_UNITS if name.endswith(suffix)), ""))
        key = f"{group}.{name}"
        label = name.replace("_", " ").capitalize()
        if unit:
            label += f" [{unit}]"
        scope = "Shared by both axes" if group in SHARED else _applicability(group, name, mode)
        fields.append({"key": key, "group": group, "label": label, "value": value,
                       "kind": kind, "nullable": nullable, "editable": verified and not locked,
                       "scope": scope})

    add("axis", "odrive_serial_number", raw.get("odrive_serial_number"), kind="text", locked=True)
    add("axis", "hardware_verified", verified, kind="bool", locked=True)
    add("axis", "encoder_present", selected.get("encoder_present", axis == 0), kind="bool")
    for group in (*AXIS_GROUPS, *SHARED):
        if group == "torque" and axis != 0:
            continue
        optional = core.FIRMWARE_OPTIONS.get(group, {})
        keys = dict.fromkeys((*core.REQUIRED_FIELDS.get(group, ()), *optional, *OPTIONAL.get(group, ())))
        if group == "board":
            values = raw.get("board", {}) if "axes" in raw else raw.get("hardware", {})
        else:
            values = raw.get(group, {}) if group in SHARED else selected.get(group, {})
        for name in keys:
            kind, default = optional.get(name, (float, None))
            value = values.get(name, default)
            boolean = kind is bool or (group == "sensorless" and name == "verified")
            add(group, name, value, kind="bool" if boolean else "int" if name in ("pole_pairs", "cpr") else "number",
                nullable=(name in optional and default is None) or f"{group}.{name}" in NULLABLE,
                locked=name in core.PROTECTIONS or (group == "sensorless" and name == "verified"))
    return fields


def _applicability(group, name, mode):
    if group == "torque":
        return ("Speed PI / torque mode" if name in ("kp", "ki", "firmware_limiter_gain", "slew_rate_nm_per_s")
                else "M0 position and torque")
    if group == "sensorless":
        return "Sensorless CLI"
    if group == "calibration":
        return "Calibration and settling"
    if group == "anticogging":
        return "Encoder velocity" if name == "anticogging_enabled" else "Anticogging map calibration"
    if group == "controller":
        modes = {"pos_gain": "Position", "inertia": "Velocity ramp", "input_filter_bandwidth": "Firmware position filter",
                 "torque_ramp_rate": "Torque", "enable_gain_scheduling": "Firmware position loop",
                 "gain_scheduling_width": "Firmware position loop"}
        return modes.get(name, "All bench modes")
    if group == "velocity":
        if name == "position_vel_gain":
            return "Position"
        if name in ("vel_gain", "vel_integrator_gain"):
            return "Encoder velocity"
        if name in ("fast_vel_ramp_rate", "fast_ramp_above_rpm"):
            return "Encoder velocity / torque"
    return "Selected axis"


def apply(raw, axis, mode, edits):
    if not isinstance(edits, dict) or not edits:
        raise ValueError("Select at least one supported parameter to change.")
    fields = {row["key"]: row for row in descriptors(raw, axis, mode) if row["editable"]}
    if edits.keys() - fields.keys():
        raise ValueError("Only supported, unlocked settings can be changed.")
    selected = profile(raw, axis)
    shared = False
    for key, value in edits.items():
        field = fields[key]
        if value is None and field["nullable"]:
            pass
        elif field["kind"] == "bool":
            if type(value) is not bool:
                raise ValueError(f"{field['label']} must be a boolean.")
        elif type(value) not in (int, float) or not math.isfinite(value) or (
                field["kind"] == "int" and type(value) is not int):
            raise ValueError(f"{field['label']} needs a valid {field['kind']}.")
        group, name = key.split(".")
        if group == "axis":
            selected[name] = value
        elif group in SHARED:
            shared = True
            target = "hardware" if group == "board" and "axes" not in raw else group
            raw.setdefault(target, {})[name] = value
        else:
            selected.setdefault(group, {})[name] = value

    for index in ((0, 1) if shared else (axis,)):
        candidate = profile(raw, index)
        if candidate and candidate.get("hardware_verified", index == 0):
            cfg = core.validate_config(deepcopy(raw), index)
            if index == 0 and "torque" in cfg:
                validate_trial(cfg["velocity"]["min_command_rpm"], cfg)
    return raw
