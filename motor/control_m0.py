#!/usr/bin/env python3
"""Run M0 in position mode below 200 RPM and torque mode at or above it."""

import argparse
from datetime import datetime, timezone
from functools import partial
import math
import time

from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH

from .m0_display import VELOCITY_COLUMNS
from .live import POSITION_BOUNDARY_RPM, TARGET_COLUMNS, effective_mode, slew_rpm
from .motor_core import (
    PROTECTIONS, check_anticogging, confirm_motion, enter_closed_loop, exclusive_motion,
    fault_codes, find_odrive, load_config, preflight,
)
from . import torque_m0 as torque
from .vel_m0 import DiagnosticStop, check_faults, overspeed, speed_limits

DEFAULT_POSITION_GAIN = 20.0
DEFAULT_POSITION_VEL_GAIN = 0.12
TORQUE_LIMIT_NM = 0.08
LEAD_TURNS = 0.05
FOLLOWING_LIMIT_TURNS = 0.075
NO_PROGRESS_S = 0.5
POSITION_FIELDS = (
    "commanded_position_turns", "measured_position_turns",
    "following_error_turns", "velocity_feedforward_rpm",
    "active_position_turns", "active_velocity_feedforward_rpm",
    "active_following_error_turns",
)
POSITION_INITIAL_FIELDS = (
    "initial_elapsed_s", "initial_encoder_count", "initial_position_turns",
    "initial_velocity_rps", "initial_active_position_turns",
)
POSITION_WRITE_FIELDS = (
    "position_write_before_s", "position_write_after_s",
    "feedforward_write_before_s", "feedforward_write_after_s",
)
POSITION_TIMING_FIELDS = (*POSITION_INITIAL_FIELDS, *POSITION_WRITE_FIELDS, "iq_measured_elapsed_s")
POSITION_LOG_FIELDS = (
    "read_span_s", "loop_interval_s", "id_setpoint_a", "encoder_shadow_count",
    "ibus_a", "brake_resistor_armed", "brake_resistor_saturated",
    "post_stop_faults_json", *POSITION_FIELDS, *POSITION_TIMING_FIELDS,
)
POSITION_COLUMNS = VELOCITY_COLUMNS + (
    ("id_measured_a", "Id Act A", "+.2f"),
    ("effective_current_lim_a", "I Limit A", ".2f"),
    ("active_torque_limit_nm", "Limit Nm", ".3f"),
    ("firmware_torque_setpoint_nm", "FW Nm", "+.3f"),
) + tuple((key, "", "") for key in POSITION_LOG_FIELDS)
POSITION_CAPTURE_COLUMNS = tuple(
    key for key in torque.CAPTURE_COLUMNS if key not in {
        "command_torque_nm", "previous_command_torque_nm", "proportional_torque_nm", "integral_torque_nm",
    }
) + ("encoder_count_rpm", *POSITION_FIELDS, *POSITION_TIMING_FIELDS)


@exclusive_motion()
def run_position(rpm, cfg, dev=None, *, capture=True, confirm=None, check_cancel=None,
                 live=None, notify=None, show_readings=None, check_setup=None):
    settings = torque.validate_trial(rpm, cfg)
    if abs(rpm) >= POSITION_BOUNDARY_RPM:
        raise ValueError(f"Position trials require a target below {POSITION_BOUNDARY_RPM:g} RPM")
    position_gain = cfg["controller"].get("pos_gain")
    if position_gain is None:
        position_gain = DEFAULT_POSITION_GAIN
    v = {**cfg["velocity"], "vel_limit_tolerance": settings["overspeed_ratio"]}
    velocity_gain = v.get("position_vel_gain", DEFAULT_POSITION_VEL_GAIN)
    h, s = cfg["hardware"], cfg["safety"]
    dev = dev if dev is not None else find_odrive(cfg)
    axis = preflight(dev, cfg)
    check_faults(fault_codes(dev.axis1, sensorless=True), "ODrive M1 reported a fault")
    reader = partial(torque.read_feedback, include_position=True)
    period = torque.admit_control_period(dev, axis, reader=reader)
    watchdog = min(s["watchdog_timeout"], 0.15)
    if watchdog <= 2 * period:
        raise RuntimeError("Position trial watchdog must exceed two control periods")
    requested_cap = min(TORQUE_LIMIT_NM, settings["torque_limit_nm"])
    cap = torque.motor_torque_cap(axis.motor.config, h, requested_cap)
    if check_setup is not None:
        check_setup(period, cap)
    ramp = v["vel_ramp_rate"]
    ceiling = live.ceiling if live is not None else abs(rpm)
    speed_limit, _, _ = speed_limits(v, ceiling, v["max_command_rpm"], sensorless=False)
    live_prompt = live.confirmation(ramp) if live is not None else ""
    initial_rpm = live.start_rpm if live is not None else rpm
    (confirm or confirm_motion)(
        f"POSITION M0 | target {initial_rpm:+g} RPM | position gain {position_gain:g} s⁻¹ | "
        f"velocity P {velocity_gain:g} | trial torque cap {cap:.3g} N·m | "
        f"firmware overspeed trip {speed_limit * v['vel_limit_tolerance']:g} RPM | {1 / period:g} Hz{live_prompt}")
    axis = preflight(dev, cfg)
    controller, motor, encoder = axis.controller, axis.motor, axis.encoder
    cpr = encoder.config.cpr
    c, m = controller.config, motor.config
    if torque.motor_torque_cap(m, h, requested_cap) != cap:
        raise RuntimeError("M0 torque limits changed during confirmation. Start the trial again.")
    torque_constant, motor_direction = m.torque_constant, m.direction
    check_anticogging(axis, enable=False)
    changes = (
        (m, "torque_lim", cap),
        (c.anticogging, "anticogging_enabled", False),
        (c, "control_mode", CONTROL_MODE_POSITION_CONTROL),
        (c, "input_mode", INPUT_MODE_PASSTHROUGH),
        (c, "circular_setpoints", False),
        (c, "pos_gain", position_gain),
        (c, "vel_gain", velocity_gain),
        (c, "vel_integrator_gain", 0.0),
        (c, "enable_gain_scheduling", False),
        (c, "vel_limit", speed_limit / 60),
        (c, "vel_limit_tolerance", v["vel_limit_tolerance"]),
        *((c, key, True) for key in PROTECTIONS),
    )
    metadata = {
        "input_mode": "PASSTHROUGH", "control_period_s": period, "watchdog_timeout_s": watchdog,
        "ramp_rpm_per_s": ramp * 60, "position_gain": position_gain, "vel_gain": velocity_gain,
        "vel_integrator_gain": 0.0, "torque_limit_nm": cap,
        "torque_constant_nm_per_a": torque_constant, "max_lead_turns": LEAD_TURNS,
        "following_limit_turns": FOLLOWING_LIMIT_TURNS, "no_progress_s": NO_PROGRESS_S,
        "firmware_limit_rpm": speed_limit, "firmware_overspeed_rpm": speed_limit * v["vel_limit_tolerance"],
    }
    if capture:
        metadata["capture_period_s"] = period
    live_columns = TARGET_COLUMNS if live is not None else ()
    if live is not None:
        metadata["live_ceiling_rpm"] = ceiling
    logger = torque.TorqueLog(POSITION_COLUMNS + live_columns, cfg=cfg, rpm=rpm, metadata=metadata,
                              capture=capture, mode="POSITION",
                              capture_columns=POSITION_CAPTURE_COLUMNS + tuple(c[0] for c in live_columns),
                              hidden=POSITION_FIELDS + POSITION_TIMING_FIELDS, notify=notify, show_readings=show_readings)
    entry_cfg = {**cfg, "safety": {**s, "watchdog_timeout": watchdog},
                 "runtime": {**cfg["runtime"],
                             "closed_loop_poll_s": min(cfg["runtime"]["closed_loop_poll_s"], watchdog / 4)}}
    direction = math.copysign(1, rpm)

    with torque.motion_session(dev, axis, cfg, logger, changes) as session:
        if check_cancel is not None:
            check_cancel()
        print("Runs until Ctrl+C or a fault. Speed correction can apply braking torque.", flush=True)
        reference = encoder.pos_estimate
        if not math.isfinite(reference):
            raise RuntimeError("Invalid encoder position before closed-loop entry")
        controller.input_pos = reference
        controller.input_vel = 0.0
        controller.input_torque = 0.0
        for obj, key, value in changes:
            setattr(obj, key, value)
        if not math.isclose(m.torque_lim, cap, rel_tol=1e-6):
            raise RuntimeError("Position torque cap did not take effect")
        enter_closed_loop(axis, entry_cfg, control_mode=CONTROL_MODE_POSITION_CONTROL,
                          input_mode=INPUT_MODE_PASSTHROUGH, check_cancel=check_cancel)
        active_position, active_feedforward = controller.input_pos, 0.0
        started = session.started = time.monotonic()
        if not math.isclose(axis.config.watchdog_timeout, watchdog, rel_tol=1e-6):
            raise RuntimeError("Position trial watchdog setting did not take effect")
        # Firmware re-anchors its position target at closed-loop entry.
        initial = reader(dev, axis, expected_state=AXIS_STATE_CLOSED_LOOP_CONTROL)
        initial_state = (initial.sampled_at - started, initial.count, initial.position,
                         initial.estimate, active_position)
        reference, last_count, previous = initial.position, initial.count, initial.sampled_at
        if notify:
            notify("running", None)
        tick, counts, setpoint = 1, 0, 0.0
        signed_rpm = 0.0
        progress_since = None
        progress_count = 0
        while True:
            release = started + tick * period
            delay = release - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            if check_cancel is not None:
                check_cancel()
            if time.monotonic() > release + period / 2:
                raise DiagnosticStop("timing_overrun", "Control tick started past its timing budget")
            deadline = started + (tick + 1) * period
            logger.check()
            feedback = reader(dev, axis, expected_state=AXIS_STATE_CLOSED_LOOP_CONTROL)
            sampled_utc = datetime.now(timezone.utc)
            now, count, position = feedback.sampled_at, feedback.count, feedback.position
            dt = now - previous
            if not 0 < dt <= 1.5 * period:
                raise DiagnosticStop("timing_overrun", f"Encoder feedback interval was {dt * 1000:.2f} ms")
            if not s["operating_min_v"] <= feedback.bus <= s["operating_max_v"]:
                raise RuntimeError(f"Bus voltage outside diagnostic range ({feedback.bus:.2f} V)")
            if feedback.available_current <= 0:
                raise RuntimeError("Invalid motor feedback: no effective current available")

            delta = (count - last_count + 2**31) % 2**32 - 2**31
            counts += delta
            count_rpm = delta * 60 / (cpr * dt)
            estimated_rpm = feedback.estimate * 60
            error = reference - position
            # During braking the shaft can travel past the previous tick's anchor.
            if abs(error) > FOLLOWING_LIMIT_TURNS and not (live is not None and live.braking):
                raise DiagnosticStop("following_error", f"Position following error reached {error:+.3f} turns")
            if live is None:
                setpoint = min(abs(rpm) / 60, setpoint + ramp * dt)
                signed_rpm = direction * setpoint * 60
            else:
                guard, _, _ = speed_limits(v, live.guard_rpm(signed_rpm), v["max_command_rpm"], sensorless=False)
                trip = guard * v["vel_limit_tolerance"]
                overspeed(now, None, estimated_rpm, count_rpm, trip, trip)
                was_braking = live.braking
                target = live.update(now, signed_rpm, estimated_rpm, count_rpm, ramp * 60)
                braking = was_braking or live.braking
                signed_rpm = 0.0 if braking else slew_rpm(signed_rpm, target, ramp * 60 * dt)
                setpoint = abs(signed_rpm) / 60
                direction = math.copysign(1, signed_rpm)
                if braking or live.reversed:
                    reference, error, progress_since = position, 0.0, None
            sample = (
                now - started, signed_rpm, estimated_rpm, count_rpm, signed_rpm - estimated_rpm,
                counts * 360 / cpr, feedback.iq_set, feedback.iq_measured, feedback.bus,
                feedback.id_measured, feedback.available_current, min(cap, feedback.available_current * torque_constant),
                feedback.iq_set * motor_direction * torque_constant,
                feedback.read_span, dt, feedback.id_set, count, feedback.ibus, feedback.brake_armed,
                feedback.brake_saturated, "",
            )
            active = (active_position, active_feedforward * 60, active_position - position)
            remaining_lead = max(0.0, LEAD_TURNS - direction * error)
            advance = min(setpoint * dt, remaining_lead)
            at_limit = advance >= remaining_lead and (live is None or setpoint > 0)
            progress = direction * counts
            if not at_limit:
                progress_since = None
            elif progress_since is None or progress > progress_count:
                progress_since, progress_count = now, progress
            elif now - progress_since >= NO_PROGRESS_S:
                raise DiagnosticStop("stall", "No forward encoder progress at the position lead limit")
            reference += direction * advance
            feedforward = direction * advance / dt
            logger.check()
            if live is not None and check_cancel is not None:
                check_cancel()
            pos_before = time.monotonic()
            if pos_before >= deadline:
                raise DiagnosticStop("timing_overrun", "Control tick missed the position write deadline")
            controller.input_pos = reference
            pos_after = time.monotonic()
            vel_before = time.monotonic()
            if vel_before >= deadline:
                raise DiagnosticStop("timing_overrun", "Control tick missed the velocity feed-forward deadline")
            controller.input_vel = feedforward
            vel_after = time.monotonic()
            if vel_after >= deadline:
                raise DiagnosticStop("timing_overrun", "Control tick missed the watchdog feed deadline")
            axis.watchdog_feed()
            if time.monotonic() >= deadline:
                raise DiagnosticStop("timing_overrun", "Watchdog feed exceeded the control tick deadline")
            logger.sample((
                *sample, reference, position, reference - position, feedforward * 60, *active,
                *(initial_state if tick == 1 else ("",) * len(POSITION_INITIAL_FIELDS)),
                pos_before - started, pos_after - started, vel_before - started, vel_after - started,
                feedback.iq_measured_at - started,
                *(live.values() if live is not None else ()),
            ), feedback.errors, sampled_utc)
            active_position, active_feedforward = reference, feedforward
            last_count, previous = count, now
            tick += 1


def run(rpm, cfg, dev=None, *, capture=True, confirm=None, check_cancel=None, live=None, notify=None,
        show_readings=None, check_setup=None):
    runner = run_position if effective_mode("auto", rpm) == "position" else torque.run
    return runner(rpm, cfg, dev, capture=capture, confirm=confirm, check_cancel=check_cancel,
                  live=live, notify=notify, show_readings=show_readings, check_setup=check_setup)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rpm", nargs="?", type=float, help="Signed target RPM (omit for repeat-test console)")
    parser.add_argument("--capture", action=argparse.BooleanOptionalAction, default=True,
                        help="Save control-tick snapshots to a companion CSV (default: enabled)")
    args = parser.parse_args()
    try:
        if args.rpm is not None:
            run(args.rpm, load_config(), capture=args.capture)
            return 0
        print("M0 CONTROL | Enter signed RPM for each trial. Ctrl+C stops motion. Enter 'q' to quit.")
        while True:
            try:
                raw = input("\nRPM > ").strip()
            except EOFError:
                return 0
            if raw.lower() in ("q", "quit", "exit"):
                return 0
            if not raw:
                continue
            try:
                rpm = float(raw)
                cfg = load_config()
                torque.validate_trial(rpm, cfg)
            except ValueError as exc:
                print(f"Invalid command or configuration: {exc}")
                continue
            run(rpm, cfg, capture=args.capture)
    except KeyboardInterrupt:
        print("\nM0 control interrupted.")
        return 130
    except Exception as exc:
        if not getattr(exc, "stop_reported", False):
            print(f"M0 control stopped: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
