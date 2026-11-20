#!/usr/bin/env python3
"""Shared ODrive 0.5.1 configuration and safety helpers.

The only user-editable settings are in motor_config.json. This helper is not
a motor test entrypoint. No function saves ODrive configuration to flash.
"""
from contextlib import contextmanager
import fcntl
import hashlib
import json
import math
import time

import odrive
from odrive.enums import (
    AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL,
    ENCODER_MODE_INCREMENTAL, ENCODER_MODE_SPI_ABS_AMS, MOTOR_TYPE_HIGH_CURRENT,
)
from odrive.utils import dump_errors

from .paths import CONFIG_PATH, MOTION_LOCK
AS5048A_CPR = 2**14
FLOAT32_MAX = 3.4028234663852886e38
# Only the observed board and its exact vendor 0.5.1 interface are authorized.
UNVERSIONED_SERIAL = "3874385D3335"
UNVERSIONED_SCHEMA_SHA256 = "6035e5f8f15addcbea8309597e590143e4a2b23af1f4255bd4652ea7ef1e7625"


BOARD_KEYS = (
    "brake_resistance", "dc_bus_overvoltage_trip_level",
    "dc_max_positive_current", "dc_max_negative_current", "max_regen_current",
)
PROTECTIONS = ("enable_vel_limit", "enable_overspeed_error", "enable_current_mode_vel_limit")
# (type, default). None keeps the live firmware value.
FIRMWARE_OPTIONS = {
    "hardware": {
        "current_control_bandwidth": (float, None),
        "torque_lim": (float, None),
        "enable_phase_interpolation": (bool, None),
    },
    "controller": {
        "pos_gain": (float, None),
        "inertia": (float, None),
        "input_filter_bandwidth": (float, None),
        "enable_gain_scheduling": (bool, None),
        "gain_scheduling_width": (float, None),
        **{key: (bool, True) for key in PROTECTIONS},
        "torque_ramp_rate": (float, None),
    },
    "anticogging": {
        "anticogging_enabled": (bool, False),
        "calib_pos_threshold": (float, None),
        "calib_vel_threshold": (float, None),
    },
}


REQUIRED_FIELDS = {
    'hardware': (
        'motor_kv pole_pairs cpr bandwidth current_lim '
        'calibration_current current_lim_margin resistance_calib_max_voltage '
    ).split(),
    'board': BOARD_KEYS,
    'velocity': (
        'vel_gain vel_integrator_gain max_command_rpm vel_ramp_rate fast_vel_ramp_rate fast_ramp_above_rpm '
        'min_command_rpm controller_min_limit_rpm controller_headroom_rpm vel_limit_tolerance '
        'software_min_limit_rpm software_max_command_factor software_target_factor '
        'software_headroom_rpm sample_seconds count_min_interval_s'
    ).split(),
    'safety': (
        'max_allowed_current_limit_a supply_min_v supply_max_v operating_min_v operating_max_v '
        'watchdog_timeout stationary_max_rpm brake_match_tolerance_ohm '
        'overvoltage_match_tolerance_v current_match_tolerance_a '
        'torque_constant_match_tolerance_nm_a bandwidth_match_tolerance'
    ).split(),
    'calibration': (
        'motor_timeout_s encoder_timeout_s poll_seconds max_phase_resistance_ohm '
        'max_phase_inductance_h settle_timeout_s settle_hold_s'
    ).split(),
    'runtime': (
        'discovery_timeout_s idle_timeout_s idle_poll_s closed_loop_timeout_s closed_loop_poll_s '
        'display_every_s'
    ).split(),
}


def axis_name(cfg):
    return f"M{cfg['axis_index']}"


def select_axis(dev, cfg):
    return dev.axis0 if cfg["axis_index"] == 0 else dev.axis1


@contextmanager
def exclusive_motion():
    MOTION_LOCK.parent.mkdir(exist_ok=True)
    with MOTION_LOCK.open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("Another motion command is running in this workspace") from exc
        yield


def load_config(axis_index=0, *, read_only=False):
    with CONFIG_PATH.open(encoding="utf-8") as file:
        raw = json.load(file)
    return validate_config(raw, axis_index, read_only=read_only)


def validate_config(raw, axis_index, *, read_only=False):
    if type(axis_index) is not int or axis_index not in (0, 1):
        raise ValueError("Axis must be 0 or 1")
    if not isinstance(raw, dict):
        raise ValueError("motor_config.json must contain an object")

    def get_group(obj, key):
        value = obj.get(key)
        if not isinstance(value, dict):
            raise ValueError(f"motor_config.json requires object '{key}'")
        return value

    if "axes" in raw:
        selected = get_group(get_group(raw, "axes"), f"m{axis_index}")
        board = get_group(raw, "board")
        if any(key in get_group(selected, "hardware") for key in BOARD_KEYS):
            raise ValueError("Shared DC-bus settings belong in board")
    else:
        if axis_index == 1 and not read_only:
            raise ValueError("Legacy configuration is M0 only. Configure axes.m1 first")
        selected = raw if axis_index == 0 else {"hardware": {}}
        board = {key: value for key, value in get_group(raw, "hardware").items() if key in BOARD_KEYS}

    cfg = {**selected, "axis_index": axis_index, "board": board,
           "runtime": get_group(raw, "runtime"), "odrive_serial_number": raw.get("odrive_serial_number")}
    cfg["hardware"] = {key: value for key, value in get_group(selected, "hardware").items()
                       if key not in BOARD_KEYS}
    cfg.setdefault("hardware_verified", axis_index == 0)
    cfg.setdefault("encoder_present", axis_index == 0)
    cfg.setdefault("sensorless", {"verified": axis_index == 0,
                                  "vel_gain": 0.01 if axis_index == 0 else None,
                                  "vel_integrator_gain": 0.05 if axis_index == 0 else None})
    serial = cfg["odrive_serial_number"]
    if not isinstance(serial, str) or len(serial) != 12 or any(c not in "0123456789ABCDEF" for c in serial):
        raise ValueError("odrive_serial_number must be 12 uppercase hexadecimal characters")
    for key in ("hardware_verified", "encoder_present"):
        if type(cfg[key]) is not bool:
            raise ValueError(f"{key} must be true or false")
    if read_only:
        timeout = cfg["runtime"].get("discovery_timeout_s")
        if type(timeout) not in (float, int) or not 0 < timeout < math.inf:
            raise ValueError("discovery_timeout_s must be a finite positive number in seconds")
        return cfg
    if not cfg["hardware_verified"]:
        raise ValueError(f"{axis_name(cfg)} hardware is unverified. Fill in its settings before enabling motion")
    for name in ("velocity", "safety", "calibration", "sensorless"):
        get_group(cfg, name)
    h, v, s = (cfg[k] for k in ("hardware", "velocity", "safety"))

    def number(obj, key, low=0, high=math.inf):
        if key not in obj:
            raise ValueError(f"motor_config.json requires setting {key!r}")
        x = obj[key]
        if isinstance(x, bool) or not isinstance(x, (int, float)):
            raise ValueError(f"{key} must be a number")
        if not low <= x <= high or abs(x) > FLOAT32_MAX or 0 < abs(x) < 2**-149:
            raise ValueError(f"{key}={x} outside configured range [{low}, {high}]")
        return float(x)

    # Required tuning values have no script-defined upper caps.
    for name, options in FIRMWARE_OPTIONS.items():
        values = cfg.setdefault(name, {})
        get_group(cfg, name)
        allowed = options.keys() | set(REQUIRED_FIELDS.get(name, ()))
        if name == "hardware":
            allowed.update(("calib_range", "torque_constant"))
        if values.keys() - allowed:
            raise ValueError(f"Unknown {name} settings: {sorted(values.keys() - allowed)}")
        for key, (kind, default) in options.items():
            value = values.setdefault(key, default)
            if key in PROTECTIONS and value is not True:
                raise ValueError(f"{name}.{key} must remain true")
            if value is None and key != "anticogging_enabled":
                continue
            if kind is bool:
                if type(value) is not bool:
                    raise ValueError(f"{name}.{key} must be true or false")
            else:
                number(values, key)
                if value == 0 and key in {"current_control_bandwidth", "gain_scheduling_width"}:
                    raise ValueError(f"{name}.{key} must be positive")
            if key == "enable_phase_interpolation" and not cfg["encoder_present"]:
                raise ValueError(f"{key} requires an installed encoder")

    if h.get("torque_constant") is not None:
        number(h, "torque_constant", low=2**-149)
    if not cfg["encoder_present"] and any(h.get(key) is not None for key in ("cpr", "bandwidth", "calib_range")):
        raise ValueError("cpr, bandwidth and calib_range must be null without an encoder")
    sensorless = cfg["sensorless"]
    if type(sensorless.get("verified")) is not bool:
        raise ValueError("sensorless.verified must be true or false")
    for key in ("vel_gain", "vel_integrator_gain"):
        if sensorless["verified"] or sensorless.get(key) is not None:
            number(sensorless, key)

    for group, names in REQUIRED_FIELDS.items():
        for key in names:
            if key in {"cpr", "bandwidth"} and not cfg["encoder_present"]:
                continue
            if key == "dc_max_positive_current" and cfg["board"].get(key, math.inf) is None:
                continue
            low = -math.inf if key == "dc_max_negative_current" else 0
            value = number(cfg[group], key, low)
            if value == 0 and key not in {'vel_integrator_gain', 'max_regen_current', 'dc_max_negative_current'}:
                raise ValueError(f"{group}.{key} must be positive")
    if "position_vel_gain" in v and number(v, "position_vel_gain") == 0:
        raise ValueError("velocity.position_vel_gain must be positive")
    if cfg["encoder_present"] and number(h, "calib_range", 0, 0.1) == 0:
        raise ValueError("hardware.calib_range must be positive")
    for key in ("pole_pairs", "cpr"):
        if key == "cpr" and not cfg["encoder_present"]:
            continue
        if not 1 <= h[key] <= 2**31 - 1 or int(h[key]) != h[key]:
            raise ValueError(f"{key} must be a positive 32-bit integer")
    if cfg["board"]["dc_max_negative_current"] > 0:
        raise ValueError("dc_max_negative_current must be nonpositive")
    if h["current_lim"] > s["max_allowed_current_limit_a"]:
        raise ValueError("Motor current exceeds safety.max_allowed_current_limit_a")
    if h["calibration_current"] > h["current_lim"]:
        raise ValueError("Calibration current exceeds motor current limit")
    if v["min_command_rpm"] > v["max_command_rpm"]:
        raise ValueError("min_command_rpm exceeds max_command_rpm")
    if v["vel_limit_tolerance"] < 1:
        raise ValueError("Overspeed tolerance must be at least 1")
    if min(v["software_max_command_factor"], v["software_target_factor"]) < 1:
        raise ValueError("Velocity software speed factors must be at least 1")
    if v["count_min_interval_s"] >= v["sample_seconds"]:
        raise ValueError("Velocity count interval must be shorter than sample interval")
    if s["operating_min_v"] > s["supply_min_v"] or s["supply_max_v"] > s["operating_max_v"]:
        raise ValueError("Supply window must be within operating voltage window")
    if not s["operating_min_v"] < s["operating_max_v"] < cfg["board"]["dc_bus_overvoltage_trip_level"]:
        raise ValueError("Operating voltage window must be below the overvoltage trip")
    if max(v["sample_seconds"], cfg["runtime"]["closed_loop_poll_s"]) >= s["watchdog_timeout"]:
        raise ValueError("Control sample/poll intervals must be shorter than watchdog timeout")
    r, cal = cfg["runtime"], cfg["calibration"]
    if r["idle_poll_s"] >= r["idle_timeout_s"] or r["closed_loop_poll_s"] >= r["closed_loop_timeout_s"]:
        raise ValueError("State poll intervals must be shorter than their timeouts")
    if cal["poll_seconds"] >= min(cal["motor_timeout_s"], cal["encoder_timeout_s"],
                                  cal["settle_timeout_s"]):
        raise ValueError("Calibration poll interval must be shorter than its timeouts")
    if cal["settle_hold_s"] >= cal["settle_timeout_s"]:
        raise ValueError("Calibration settle hold must be shorter than its timeout")
    if s["supply_min_v"] >= s["supply_max_v"]:
        raise ValueError("supply voltage range inverted")
    return cfg


def torque_constant(cfg):
    hardware = cfg["hardware"]
    override = hardware.get("torque_constant")
    return float(override) if override is not None else 8.27 / float(hardware["motor_kv"])


def confirm_motion(description):
    print(description)
    print("Secure the motor, guard the shaft, and keep the power cutoff accessible.")
    while input("Press Enter to proceed, or Ctrl+C to cancel: ") != "":
        print("Press Enter without typing anything to proceed.")


def fault_codes(axis, *, sensorless=False):
    return (axis.error, axis.motor.error, axis.encoder.error, axis.controller.error,
            axis.sensorless_estimator.error if sensorless else None)


def no_faults(axis):
    return not (axis.error or axis.motor.error or axis.encoder.error
                or axis.controller.error)


def find_odrive(cfg):
    serial = cfg["odrive_serial_number"]
    dev = odrive.find_any(serial_number=serial, timeout=cfg["runtime"]["discovery_timeout_s"])
    if dev is None:
        raise RuntimeError(f"ODrive {serial} not found")
    return dev


def verify_board(dev, cfg):
    expected = cfg["odrive_serial_number"]
    if f"{dev.serial_number:012X}" != expected:
        raise RuntimeError("ODrive serial number does not match motor_config.json")
    if (dev.hw_version_major, dev.hw_version_minor) != (3, 6):
        raise RuntimeError("Expected an ODrive v3.6 board")
    version = (dev.fw_version_major, dev.fw_version_minor, dev.fw_version_revision)
    if version != (0, 5, 1):
        if version == (0, 0, 0) and dev.fw_version_unreleased == 1 and expected == UNVERSIONED_SERIAL:
            schema = dev.__channel__.remote_endpoint_read_buffer(0)
            if hashlib.sha256(schema).hexdigest() == UNVERSIONED_SCHEMA_SHA256:
                print("WARNING: unversioned firmware; interface matches vendor 0.5.1, "
                      "but firmware and flash behavior are not verified.", flush=True)
                return
        actual = ".".join(str(part) for part in version)
        raise RuntimeError(f"ODrive reports firmware {actual}; this workflow requires verified "
                           "firmware 0.5.1 or the specifically approved unversioned board "
                           "and matching interface. The Python package version is not "
                           "the board firmware version.")


def ensure_idle(dev, cfg, *, check_faults=True):
    if dev.axis0.current_state != AXIS_STATE_IDLE or dev.axis1.current_state != AXIS_STATE_IDLE:
        raise RuntimeError("Both axes must be IDLE")
    if check_faults and not no_faults(select_axis(dev, cfg)):
        dump_errors(dev)
        raise RuntimeError(f"ODrive {axis_name(cfg)} fault already present")


def check_power(dev, cfg):
    h, s = cfg["board"], cfg["safety"]
    bus = float(dev.vbus_voltage)
    if not s["supply_min_v"] <= bus <= s["supply_max_v"]:
        raise RuntimeError(f"Unexpected PSU voltage: {bus:.2f} V")
    if not dev.brake_resistor_armed:
        raise RuntimeError("Brake resistor not armed; inspect connection")
    if not abs(dev.config.brake_resistance - h["brake_resistance"]) <= s["brake_match_tolerance_ohm"]:
        raise RuntimeError("Physical brake resistor setting does not match configuration")
    if not abs(dev.config.dc_bus_overvoltage_trip_level - h["dc_bus_overvoltage_trip_level"]) <= s["overvoltage_match_tolerance_v"]:
        raise RuntimeError("ODrive overvoltage trip differs from configured safe value")


def verify_configuration(dev, cfg):
    """Check board identity and the live encoder configuration."""
    if not cfg["encoder_present"]:
        raise RuntimeError(f"{axis_name(cfg)} requires a configured encoder for closed-loop velocity")
    verify_board(dev, cfg)
    ensure_idle(dev, cfg)
    check_power(dev, cfg)
    a = select_axis(dev, cfg)
    h = cfg["hardware"]
    mode = a.encoder.config.mode
    absolute = cfg["axis_index"] == 0 and mode == ENCODER_MODE_SPI_ABS_AMS
    repair = ("run .venv/bin/python -m motor.calibrate_m0_x with the wired --cs-gpio" if absolute
              else f"run .venv/bin/python -m motor.calibrate_m0 --axis {cfg['axis_index']}")
    if ((not absolute and mode != ENCODER_MODE_INCREMENTAL)
            or a.encoder.config.use_index
            or a.encoder.config.zero_count_on_find_idx
            or a.config.startup_encoder_index_search):
        raise RuntimeError(f"Encoder mode or index settings differ from the expected configuration; {repair}")
    if a.encoder.config.cpr != (AS5048A_CPR if absolute else h["cpr"]):
        raise RuntimeError(f"Encoder CPR differs from the expected value; {repair}")
    if a.motor.config.pole_pairs != h["pole_pairs"]:
        raise RuntimeError(f"Motor pole pairs differ from JSON; {repair}")
    if not abs(a.motor.config.current_lim - h["current_lim"]) <= cfg["safety"]["current_match_tolerance_a"]:
        raise RuntimeError(f"Motor current limit differs from JSON; {repair}")
    live_constant, expected_constant = a.motor.config.torque_constant, torque_constant(cfg)
    if h.get("torque_constant") is None:
        tolerance = cfg["safety"]["torque_constant_match_tolerance_nm_a"]
        constant_matches = abs(live_constant - expected_constant) <= tolerance
    else:
        # An explicit override must match within float32 rounding on the drive.
        constant_matches = math.isclose(live_constant, expected_constant, rel_tol=1e-6, abs_tol=2**-150)
    if not constant_matches:
        raise RuntimeError(f"Motor torque constant differs from JSON; {repair}")
    if not abs(a.encoder.config.bandwidth - h["bandwidth"]) <= cfg["safety"]["bandwidth_match_tolerance"]:
        raise RuntimeError(f"Encoder PLL bandwidth differs from JSON; {repair}")
    if a.controller.config.load_encoder_axis != cfg["axis_index"]:
        raise RuntimeError(f"Controller is not using the {axis_name(cfg)} encoder")
    return a


def valid_motor_calibration(axis, cfg):
    motor = axis.motor
    cal = cfg["calibration"]
    return (motor.is_calibrated
            and 0 < motor.config.phase_resistance < cal["max_phase_resistance_ohm"]
            and 0 < motor.config.phase_inductance < cal["max_phase_inductance_h"])


def valid_encoder_calibration(axis):
    return (axis.encoder.is_ready and math.isfinite(axis.encoder.config.offset_float)
            and axis.motor.config.direction in (-1, 1))


def preflight(dev, cfg):
    a = verify_configuration(dev, cfg)
    absolute = a.encoder.config.mode == ENCODER_MODE_SPI_ABS_AMS
    if (a.motor.config.pre_calibrated != absolute or a.encoder.config.pre_calibrated != absolute
            or not valid_motor_calibration(a, cfg) or not valid_encoder_calibration(a)):
        script = (".venv/bin/python -m motor.calibrate_m0_x with the wired --cs-gpio" if absolute
                  else f".venv/bin/python -m motor.calibrate_m0 --axis {cfg['axis_index']}")
        raise RuntimeError(f"{axis_name(cfg)} needs motor and encoder calibration. Run {script}.")
    if not math.isfinite(a.encoder.pos_estimate) or not math.isfinite(a.encoder.vel_estimate):
        raise RuntimeError("Encoder feedback is invalid")
    if abs(a.encoder.vel_estimate) * 60 > cfg["safety"]["stationary_max_rpm"]:
        raise RuntimeError("Wait until motor is stationary before test")
    # Controller gains are intentionally NOT checked here: each test reads its
    # mode-specific gains from JSON and overrides/restores them for its run.
    return a


def firmware_options(axis, cfg, *, report=False):
    """Inspect the selected axis before writing any optional settings."""
    controller = axis.controller.config
    owners = {
        "hardware": ("motor.config", axis.motor.config),
        "controller": ("controller.config", controller),
        "anticogging": ("controller.config.anticogging", getattr(controller, "anticogging", None)),
    }
    settings = []
    missing = []
    for group, options in FIRMWARE_OPTIONS.items():
        values = cfg.get(group, {})
        for key, (_, default) in options.items():
            path, obj = owners[group]
            if key == "enable_phase_interpolation":
                path, obj = "encoder.config", axis.encoder.config
            value = values.get(key, default)
            try:
                live = getattr(obj, key)
            except AttributeError:
                print(f"{axis_name(cfg)} {path}.{key}: unsupported", flush=True)
                if value is not None:
                    missing.append(f"{path}.{key}")
                continue
            if report:
                print(f"{axis_name(cfg)} {path}.{key}: available (live {live})")
            settings.append((path, obj, key, value, live))
    if missing and not report:
        raise RuntimeError("Configured firmware properties unavailable: " + ", ".join(missing))
    return settings


def check_anticogging(axis, *, enable):
    anticogging = getattr(axis.controller.config, "anticogging", None)
    if getattr(anticogging, "calib_anticogging", False):
        raise RuntimeError("Anticogging calibration already active. Clear it before testing")
    if enable and not getattr(axis.controller, "anticogging_valid", False):
        raise RuntimeError("Anticogging requested but firmware has no valid map")


def apply_options(settings, *, enable_anticogging):
    for path, obj, key, value, _ in settings:
        if key == "anticogging_enabled":
            value = enable_anticogging
        if value is not None:
            try:
                setattr(obj, key, value)
            except AttributeError as exc:
                raise RuntimeError(f"Cannot write firmware property {path}.{key}") from exc


def configure_for_calibration(dev, cfg, *, spi_cs_gpio=None):
    h, s = cfg["hardware"], cfg["safety"]
    a = select_axis(dev, cfg)
    m, e, c = a.motor.config, a.encoder.config, a.controller.config
    options = firmware_options(a, cfg)
    check_anticogging(a, enable=False)
    apply_options(options, enable_anticogging=False)
    a.config.startup_motor_calibration = False
    a.config.startup_encoder_index_search = False
    a.config.startup_encoder_offset_calibration = False
    a.config.startup_sensorless_control = False
    a.config.startup_closed_loop_control = False
    a.config.startup_homing = False
    a.config.enable_watchdog = False
    a.config.watchdog_timeout = s["watchdog_timeout"]

    # Verify existing brake/voltage configuration BEFORE calling this function.
    for key in BOARD_KEYS:
        value = cfg["board"][key]
        setattr(dev.config, key, math.inf if value is None else value)

    m.motor_type = MOTOR_TYPE_HIGH_CURRENT
    m.pole_pairs = int(h["pole_pairs"])
    m.direction = 1
    m.torque_constant = torque_constant(cfg)
    m.current_lim = h["current_lim"]
    m.current_lim_margin = h["current_lim_margin"]
    m.calibration_current = h["calibration_current"]
    m.resistance_calib_max_voltage = h["resistance_calib_max_voltage"]
    m.pre_calibrated = False

    if cfg["encoder_present"]:
        e.use_index = False
        e.zero_count_on_find_idx = False
        e.pre_calibrated = False
        e.bandwidth = h["bandwidth"]
        e.calib_range = h["calib_range"]
        if spi_cs_gpio is None:
            e.mode = ENCODER_MODE_INCREMENTAL
            e.cpr = int(h["cpr"])
        else:
            e.abs_spi_cs_gpio_pin = spi_cs_gpio
            e.mode = ENCODER_MODE_SPI_ABS_AMS
            e.cpr = AS5048A_CPR

    c.load_encoder_axis = cfg["axis_index"]
    a.config.enable_step_dir = False
    a.controller.input_vel = 0
    a.controller.input_torque = 0


def safe_idle(axis, cfg):
    try:
        axis.requested_state = AXIS_STATE_IDLE
        deadline = time.monotonic() + cfg["runtime"]["idle_timeout_s"]
        while time.monotonic() < deadline:
            if axis.current_state == AXIS_STATE_IDLE:
                axis.controller.input_vel = 0
                axis.controller.input_torque = 0
                axis.config.enable_watchdog = False
                return True
            time.sleep(cfg["runtime"]["idle_poll_s"])
    except Exception as exc:
        print(f"WARNING: could not confirm IDLE: {exc}", flush=True)
    print("DANGER: IDLE not confirmed; isolate motor power physically.", flush=True)
    return False


def verify_control_mode(axis, control_mode, input_mode):
    controller = axis.controller.config
    actual = (controller.control_mode, controller.input_mode)
    if actual != (control_mode, input_mode):
        raise RuntimeError(f"Axis control/input mode is {actual}; expected {(control_mode, input_mode)}")


def enter_closed_loop(axis, cfg, *, control_mode, input_mode, check_cancel=None):
    verify_control_mode(axis, control_mode, input_mode)
    axis.config.watchdog_timeout = cfg["safety"]["watchdog_timeout"]
    axis.watchdog_feed()
    axis.config.enable_watchdog = True
    if check_cancel is not None:
        check_cancel()
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    deadline = time.monotonic() + cfg["runtime"]["closed_loop_timeout_s"]
    while time.monotonic() < deadline:
        if check_cancel is not None:
            check_cancel()
        axis.watchdog_feed()
        if not no_faults(axis):
            raise RuntimeError("ODrive fault on closed-loop entry")
        if axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            verify_control_mode(axis, control_mode, input_mode)
            return
        time.sleep(cfg["runtime"]["closed_loop_poll_s"])
    raise TimeoutError(f"{axis_name(cfg)} failed to enter closed-loop control")


