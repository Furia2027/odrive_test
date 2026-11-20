#!/usr/bin/env python3
"""Clear faults and calibrate the selected ODrive axis with an incremental encoder."""
import argparse
import math
import time

from fibre import ChannelBrokenException
from odrive.enums import (
    AXIS_STATE_IDLE, AXIS_STATE_MOTOR_CALIBRATION,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION, ENCODER_MODE_INCREMENTAL,
    ENCODER_MODE_SPI_ABS_AMS,
)
from odrive.utils import dump_errors

from .motor_core import (
    load_config, find_odrive, verify_board, ensure_idle, check_power,
    configure_for_calibration, safe_idle, no_faults, fault_codes, confirm_motion,
    valid_motor_calibration, valid_encoder_calibration, axis_name, select_axis,
    firmware_options, exclusive_motion, verify_configuration, preflight,
)

STARTUP_MOTION = (
    "startup_motor_calibration", "startup_encoder_index_search",
    "startup_encoder_offset_calibration", "startup_sensorless_control",
    "startup_closed_loop_control", "startup_homing",
)


def calibrate_step(axis, state, label, deadline_s, success, poll_seconds, *, check_cancel=None):
    print(f"Running {label}; motor shaft may turn...", flush=True)
    entered = False
    if check_cancel is not None:
        check_cancel()
    axis.requested_state = state
    end = time.monotonic() + deadline_s
    while time.monotonic() < end:
        if check_cancel is not None:
            check_cancel()
        if not no_faults(axis):
            raise RuntimeError(f"ODrive fault during {label}")
        actual = axis.current_state
        if actual == state:
            entered = True
        if actual == AXIS_STATE_IDLE and entered:
            if not success():
                raise RuntimeError(f"{label} ended with an invalid result")
            print(f"{label}: PASS", flush=True)
            return
        time.sleep(poll_seconds)
    raise TimeoutError(f"{label} exceeded {deadline_s:g} s")


def wait_for_settling(axis, cfg, *, check_cancel=None):
    cal = cfg["calibration"]
    limit = cfg["safety"]["stationary_max_rpm"]
    deadline = time.monotonic() + cal["settle_timeout_s"]
    stable_since = None
    print(f"Waiting for {axis_name(cfg)} to settle below {limit:g} RPM...", flush=True)
    while time.monotonic() < deadline:
        if check_cancel is not None:
            check_cancel()
        if axis.current_state != AXIS_STATE_IDLE or not no_faults(axis):
            raise RuntimeError(f"{axis_name(cfg)} fault or left IDLE while settling")
        rpm = float(axis.encoder.vel_estimate) * 60
        if not math.isfinite(rpm):
            raise RuntimeError("Invalid encoder velocity while settling")
        now = time.monotonic()
        if abs(rpm) <= limit:
            if stable_since is None:
                stable_since = now
            if now - stable_since >= cal["settle_hold_s"]:
                return
        else:
            stable_since = None
        time.sleep(cal["poll_seconds"])
    raise TimeoutError(f"{axis_name(cfg)} did not settle below {limit:g} RPM within "
                       f"{cal['settle_timeout_s']:g} seconds")


def clear_axis_errors(dev, cfg):
    axis = select_axis(dev, cfg)
    if dev.axis0.current_state != AXIS_STATE_IDLE or dev.axis1.current_state != AXIS_STATE_IDLE:
        raise RuntimeError("Both axes must be IDLE before clearing faults")
    if any(fault_codes(axis, sensorless=True)):
        axis.clear_errors()
        if any(fault_codes(axis, sensorless=True)):
            dump_errors(dev)
            raise RuntimeError(f"ODrive {axis_name(cfg)} still reports a fault after clearing")
    ensure_idle(dev, cfg)


def save_and_reboot(dev, cfg, *, changing_encoder=False):
    ensure_idle(dev, cfg, check_faults=not changing_encoder)
    check_power(dev, cfg)
    for index, axis in enumerate((dev.axis0, dev.axis1)):
        if any(getattr(axis.config, name) for name in STARTUP_MOTION):
            raise RuntimeError(f"M{index} has a startup motion action enabled. Disable it before saving and rebooting.")
    print("Saving ODrive configuration to flash and rebooting...", flush=True)
    dev.save_configuration()
    try:
        dev.reboot()
    except ChannelBrokenException:
        pass  # A reboot can drop USB before the firmware acknowledges the request.
    time.sleep(0.5)
    dev = find_odrive(cfg)
    verify_board(dev, cfg)
    if not dev.user_config_loaded:
        raise RuntimeError("ODrive did not load a saved configuration after reboot")
    # USB comes up before the firmware finishes current-sense initialization.
    deadline = time.monotonic() + cfg["runtime"]["discovery_timeout_s"]
    while dev.axis0.current_state != AXIS_STATE_IDLE or dev.axis1.current_state != AXIS_STATE_IDLE:
        if time.monotonic() >= deadline:
            raise TimeoutError("Both axes must reach IDLE after reboot")
        time.sleep(cfg["runtime"]["idle_poll_s"])
    clear_axis_errors(dev, cfg)
    return dev


def calibration_values(axis):
    motor, encoder = axis.motor.config, axis.encoder.config
    return motor.phase_resistance, motor.phase_inductance, motor.direction, encoder.offset, encoder.offset_float


@exclusive_motion()
def calibrate(cfg, *, spi_cs_gpio=None, allow_encoder_switch=True, confirm=None, check_cancel=None):
    absolute = spi_cs_gpio is not None
    if absolute and (cfg["axis_index"] != 0 or not cfg["encoder_present"]):
        raise ValueError("AS5048A calibration requires an encoder configured on M0")
    dev = find_odrive(cfg)
    verify_board(dev, cfg)
    axis = select_axis(dev, cfg)
    switch_to_incremental = (cfg["axis_index"] == 0 and cfg["encoder_present"]
                             and axis.encoder.config.mode == ENCODER_MODE_SPI_ABS_AMS)
    reboot_for_mode = absolute or switch_to_incremental
    if not allow_encoder_switch and (
            reboot_for_mode or not cfg["encoder_present"]
            or axis.encoder.config.mode != ENCODER_MODE_INCREMENTAL):
        raise RuntimeError("RAM calibration requires an existing A/B encoder configuration. Use the CLI for encoder setup.")
    if reboot_for_mode or not allow_encoder_switch:
        # The previous encoder can fault continuously after the wiring changes.
        ensure_idle(dev, cfg, check_faults=False)
    else:
        clear_axis_errors(dev, cfg)
    check_power(dev, cfg)
    if absolute:
        subject = f"motor and AS5048A calibration (CS on GPIO{spi_cs_gpio})"
        if spi_cs_gpio in (1, 2) and dev.config.enable_uart:
            raise RuntimeError(f"GPIO{spi_cs_gpio} is in use by the ODrive UART")
    else:
        subject = "motor and A/B encoder calibration" if cfg["encoder_present"] else "motor calibration"
    prompt = f"{axis_name(cfg)} {subject}. The shaft may turn."
    if reboot_for_mode:
        if any(getattr(dev.axis1.config, name) for name in STARTUP_MOTION):
            raise RuntimeError("M1 has a startup motion action enabled. Disable it before changing M0's encoder mode.")
        prompt += " This will save the board's current configuration and reboot it."
    (confirm or confirm_motion)(prompt)
    if check_cancel is not None:
        check_cancel()
    if not allow_encoder_switch:
        clear_axis_errors(dev, cfg)
    ensure_idle(dev, cfg, check_faults=not reboot_for_mode)
    check_power(dev, cfg)

    cal = cfg["calibration"]
    if reboot_for_mode:
        configure_for_calibration(dev, cfg, spi_cs_gpio=spi_cs_gpio)
        dev = save_and_reboot(dev, cfg, changing_encoder=True)
        axis = verify_configuration(dev, cfg)
        expected_mode = ENCODER_MODE_SPI_ABS_AMS if absolute else ENCODER_MODE_INCREMENTAL
        if (axis.encoder.config.mode != expected_mode
                or (absolute and axis.encoder.config.abs_spi_cs_gpio_pin != spi_cs_gpio)
                or axis.motor.config.pre_calibrated or axis.encoder.config.pre_calibrated):
            raise RuntimeError("ODrive did not load the new encoder settings after reboot")

    complete = False
    try:
        cancellation = {} if check_cancel is None else {"check_cancel": check_cancel}
        if reboot_for_mode:
            wait_for_settling(axis, cfg, **cancellation)
        else:
            configure_for_calibration(dev, cfg)
        calibrate_step(axis, AXIS_STATE_MOTOR_CALIBRATION, "motor R/L calibration",
                       cal["motor_timeout_s"], lambda: valid_motor_calibration(axis, cfg),
                       cal["poll_seconds"], **cancellation)
        if cfg["encoder_present"]:
            calibrate_step(axis, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, "encoder phase offset",
                           cal["encoder_timeout_s"], lambda: valid_encoder_calibration(axis),
                           cal["poll_seconds"], **cancellation)
            wait_for_settling(axis, cfg, **cancellation)
        complete = True
    finally:
        if not safe_idle(axis, cfg):
            raise RuntimeError("IDLE not confirmed; further testing blocked")
        if not complete or not cfg["encoder_present"]:
            # Firmware may retain encoder readiness from a previous scan.
            axis.motor.config.direction = 0
        if not complete:
            try:
                dump_errors(dev)
            except Exception:
                pass

    if not cfg["encoder_present"]:
        print(f"{axis_name(cfg)} motor calibrated in RAM and IDLE. Encoder velocity unavailable.")
        return
    scan_response = axis.encoder.calib_scan_response
    if absolute:
        expected = calibration_values(axis)
        axis.motor.config.pre_calibrated = True
        axis.encoder.config.pre_calibrated = True
        if not (axis.motor.config.pre_calibrated and axis.encoder.config.pre_calibrated):
            raise RuntimeError("ODrive rejected the pre-calibrated flags")
        dev = save_and_reboot(dev, cfg)
        axis = verify_configuration(dev, cfg)
        if (axis.encoder.config.mode != ENCODER_MODE_SPI_ABS_AMS
                or axis.encoder.config.abs_spi_cs_gpio_pin != spi_cs_gpio
                or calibration_values(axis) != expected):
            raise RuntimeError("ODrive did not load the saved AS5048A calibration after reboot")
        wait_for_settling(axis, cfg)
        preflight(dev, cfg)
        print("M0 AS5048A and motor calibration saved and verified after reboot. M0 is IDLE.")
    else:
        print(f"{axis_name(cfg)} calibrated in RAM and IDLE. "
              f"Run .venv/bin/python -m motor.vel_m0 --axis {cfg['axis_index']} before the next ODrive boot.")
    e = axis.encoder
    expected = e.config.calib_scan_distance * e.config.cpr / (math.tau * axis.motor.config.pole_pairs)
    print(f"Encoder direction {axis.motor.config.direction:+d} | "
          f"scan {scan_response:.0f}/{expected:.1f} counts | "
          f"offset {e.config.offset + e.config.offset_float:.3f} counts")


@exclusive_motion()
def clear_errors(cfg):
    dev = find_odrive(cfg)
    verify_board(dev, cfg)
    clear_axis_errors(dev, cfg)
    print(f"{axis_name(cfg)} faults cleared. Both axes are IDLE.")


def main(*, absolute=False):
    description = ("Calibrate M0 with an AS5048A SPI encoder and save the calibration to flash."
                   if absolute else __doc__)
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--axis", type=int, choices=(0,) if absolute else (0, 1), default=0)
    if absolute:
        parser.add_argument("--cs-gpio", type=int, choices=range(1, 9),
                            help="GPIO wired to AS5048A CSn (required for calibration)")
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--check-config", action="store_true",
                        help="Read firmware property availability without changing settings or moving")
    action.add_argument("--clear", "--clear-errors", action="store_true",
                        help="Only clear faults on the selected axis while both axes are IDLE")
    action.add_argument("--calibrate", action="store_true",
                        help="Clear faults and calibrate the selected axis (default)")
    args = parser.parse_args()
    if absolute and not (args.check_config or args.clear) and args.cs_gpio is None:
        parser.error("calibration requires --cs-gpio with the GPIO wired to AS5048A CSn")
    cfg = load_config(args.axis, read_only=args.check_config or args.clear)
    if args.check_config:
        dev = find_odrive(cfg)
        verify_board(dev, cfg)
        axis = select_axis(dev, cfg)
        if absolute:
            e = axis.encoder.config
            print(f"M0 encoder mode {e.mode} | CPR {e.cpr} | CS GPIO{e.abs_spi_cs_gpio_pin}")
            print(f"Pre-calibrated motor: {axis.motor.config.pre_calibrated} | encoder: {e.pre_calibrated}")
            print(f"Motor calibrated: {axis.motor.is_calibrated} | encoder ready: {axis.encoder.is_ready}")
        firmware_options(axis, cfg, report=True)
    elif args.clear:
        clear_errors(cfg)
    elif absolute:
        calibrate(cfg, spi_cs_gpio=args.cs_gpio)
    else:
        calibrate(cfg)


def cli(*, absolute=False):
    try:
        main(absolute=absolute)
    except KeyboardInterrupt:
        print("\nODrive command interrupted.")
        raise SystemExit(130)
    except Exception as exc:
        print("ODRIVE COMMAND FAILED:", exc)
        raise SystemExit(1)


if __name__ == "__main__":
    cli()
