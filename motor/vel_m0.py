#!/usr/bin/env python3
"""ODrive signed velocity test (defaults to M0).

Usage: python -m motor.vel_m0           # repeat-test console
       python -m motor.vel_m0 30        # run until Ctrl+C, then exit
       python -m motor.vel_m0 -300
       python -m motor.vel_m0 --sensorless 550

The repeat-test console requests RPM only while the axis is IDLE, then
asks for Enter before each test. Ctrl+C stops a test. Between trials the motor
returns to IDLE. Encoder-mode tuning is loaded from motor_config.json.
The sensorless test uses separate bench settings and logs estimated speed.
High RPM is not a verified continuous safe operating point.
Keep the motor securely mounted and guarded.
"""
import argparse
import math
import time

from odrive.enums import (
    CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP, INPUT_MODE_PASSTHROUGH,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_MOTOR_CALIBRATION, AXIS_STATE_SENSORLESS_CONTROL,
    LOCKIN_STATE_INACTIVE, CONTROLLER_ERROR_OVERSPEED, ENCODER_MODE_SPI_ABS_AMS,
)
from odrive.utils import dump_errors
from .calibrate_m0 import calibrate_step
from .motor_core import (
    load_config, find_odrive, preflight, enter_closed_loop, safe_idle,
    confirm_motion, verify_board, ensure_idle, check_power,
    configure_for_calibration, valid_motor_calibration,
    valid_encoder_calibration, axis_name, select_axis, exclusive_motion,
    firmware_options, check_anticogging, apply_options, fault_codes, PROTECTIONS,
)
from .m0_display import TelemetryTable, VELOCITY_COLUMNS
from .live import TARGET_COLUMNS


class DiagnosticStop(RuntimeError):
    def __init__(self, reason, message):
        super().__init__(message)
        self.reason = reason


def overspeed(now, since, estimated, count_rpm, soft_limit, hard_limit):
    if abs(estimated) > hard_limit or (count_rpm is not None and abs(count_rpm) > soft_limit):
        raise DiagnosticStop("overspeed", "Speed exceeds immediate diagnostic limit")
    if abs(estimated) <= soft_limit:
        return None
    if since is None:
        return now
    if now - since >= 0.15:
        raise DiagnosticStop("overspeed", "Estimated speed stayed above diagnostic limit")
    return since


def stall(now, since, setpoint, count_rpm, peak_rpm):
    if (count_rpm is None or abs(setpoint) < min(20.0, 0.9 * peak_rpm)
            or count_rpm * math.copysign(1, setpoint) >= 0.25 * abs(setpoint)):
        return None
    if since is None:
        return now
    if now - since >= 0.5:
        raise DiagnosticStop("stall", "Encoder-mode motor stalled or ran backward under command")
    return since


def check_faults(errors, message):
    if any(errors):
        reason = "overspeed" if errors[3] & CONTROLLER_ERROR_OVERSPEED else "fault"
        raise DiagnosticStop(reason, message)


def speed_limits(v, peak_rpm, max_rpm, *, sensorless):
    controller = min(max_rpm, max(v["controller_min_limit_rpm"], peak_rpm + v["controller_headroom_rpm"]))
    tracking = max(v["software_min_limit_rpm"], v["software_target_factor"] * peak_rpm + v["software_headroom_rpm"])
    software = min(max_rpm * v["software_max_command_factor"], tracking)
    hard = software if sensorless else min(1.25 * software, v["vel_limit_tolerance"] * controller)
    return controller, software, hard


@exclusive_motion()
def run(rpm, cfg, dev=None, *, sensorless=False, continuous=False,
        confirm=None, check_cancel=None, live=None, notify=None):
    confirm = confirm or confirm_motion
    v, s, h = cfg["velocity"], cfg["safety"], cfg["hardware"]
    if not math.isfinite(rpm) or not v["min_command_rpm"] <= abs(rpm) <= v["max_command_rpm"]:
        raise ValueError(f"RPM must be within +/-{v['min_command_rpm']} to +/-{v['max_command_rpm']}")
    if sensorless and not cfg["sensorless"]["verified"]:
        raise ValueError(f"{axis_name(cfg)} sensorless settings are unverified")
    if sensorless and not 550 <= abs(rpm) <= 800:
        raise ValueError("Sensorless bench test requires a signed target between 550 and 800 RPM")
    if live is not None and sensorless:
        raise ValueError("Live RPM requires an encoder-velocity bench test")
    dev = dev or find_odrive(cfg)
    initial_rpm = live.start_rpm if live is not None else rpm
    teleop = live is not None and initial_rpm == 0
    mode = "TELEOP" if teleop else "SENSORLESS VELOCITY" if sensorless else "VELOCITY"
    if sensorless:
        verify_board(dev, cfg)
        ensure_idle(dev, cfg)
        check_power(dev, cfg)
        a = select_axis(dev, cfg)
        if a.encoder.config.mode == ENCODER_MODE_SPI_ABS_AMS:
            raise RuntimeError("The sensorless bench test requires the incremental configuration. "
                               "Connect the A/B encoder and run .venv/bin/python -m motor.calibrate_m0 first.")
        confirm(f"{mode} {axis_name(cfg)} | target {rpm:+.1f} RPM | keep shaft unloaded and stationary")
        ensure_idle(dev, cfg)
        check_power(dev, cfg)
    else:
        preflight(dev, cfg)
        target = f"target {initial_rpm:+.1f} RPM"
        if live is not None:
            ramp = v["fast_vel_ramp_rate"] if abs(rpm) > v["fast_ramp_above_rpm"] else v["vel_ramp_rate"]
            limit, _, _ = speed_limits(v, live.ceiling, v["max_command_rpm"], sensorless=False)
            target += live.confirmation(ramp) + f" | firmware trip {limit * v['vel_limit_tolerance']:g} RPM"
        label = "TELEOP VELOCITY" if teleop else mode
        confirm(f"{label} {axis_name(cfg)} | {target}")
        a = preflight(dev, cfg)
    cpr = h["cpr"] if sensorless else a.encoder.config.cpr
    peak_rpm = abs(rpm)
    c = a.controller.config
    ramp = (v["fast_vel_ramp_rate"] if abs(rpm) > v["fast_ramp_above_rpm"]
            else v["vel_ramp_rate"])
    max_rpm = v["max_command_rpm"]
    active_rpm_limit, software_speed_limit, hard_speed_limit = speed_limits(
        v, peak_rpm, max_rpm, sensorless=sensorless)
    if live is not None:
        active_rpm_limit, _, _ = speed_limits(v, live.ceiling, max_rpm, sensorless=False)
    gains = cfg["sensorless"] if sensorless else v
    kp, ki = gains["vel_gain"], gains["vel_integrator_gain"]
    if teleop:
        print(f"{mode} {axis_name(cfg)} | ceiling {live.ceiling:g} RPM | ramp {ramp*60:g} RPM/s")
    else:
        print(f"{mode} {axis_name(cfg)} | target {rpm:+.1f} RPM | ramp {ramp*60:.1f} RPM/s")
        print("Runs for 5 seconds after startup or until Ctrl+C."
              if sensorless and not continuous else "Runs until Ctrl+C.")
        print(f"P={kp}, I={ki}, current limit {h['current_lim']} A.")
        print(f"Software overspeed threshold {software_speed_limit:.1f} RPM | "
              f"immediate estimator cutoff {hard_speed_limit:.1f} RPM.")
        print("Motor securely mounted, shaft unloaded and GUARDED; power cutoff accessible.")

    old = (c.control_mode, c.input_mode, c.vel_gain, c.vel_integrator_gain,
           c.vel_limit, c.vel_ramp_rate, c.vel_limit_tolerance, a.config.watchdog_timeout)
    options = firmware_options(a, cfg)
    enable_anticogging = not sensorless and cfg["anticogging"]["anticogging_enabled"]
    check_anticogging(a, enable=enable_anticogging)
    columns = VELOCITY_COLUMNS + (
        ("encoder_delta_counts", "Encoder counts", "d"),
        ("encoder_phase_deg", "Phase deg", "+.1f"),
        ("id_measured_a", "Id Act A", "+.2f"),
        ("effective_current_lim_a", "I Limit A", ".2f"),
    )
    if sensorless:
        columns += (("sensorless_turns", "Est turns", "+.2f"),
                    ("phase_error_deg", "Phase err deg", "+.1f"))
    if teleop:
        teleop_columns = (
            ("command_rpm", "Command RPM", "+.1f"),
            ("ceiling_rpm", "Ceiling RPM", ".1f"),
            ("ramp_rpm_per_s", "Ramp RPM/s", ".1f"),
        )
        columns += teleop_columns
    metadata = {
        "vel_gain": kp, "vel_integrator_gain": ki, "vel_ramp_rate": ramp,
        "controller_limit_rpm": active_rpm_limit, "software_limit_rpm": software_speed_limit,
        "hard_estimator_limit_rpm": hard_speed_limit,
        "firmware_settings": {
            f"{path}.{name}": (enable_anticogging if name == "anticogging_enabled"
                              else value if value is not None
                              else live if math.isfinite(live) else str(live))
            for path, _, name, value, live in options
        },
    }
    if live is not None:
        columns += TARGET_COLUMNS
        metadata.update(live_ceiling_rpm=live.ceiling,
                        firmware_overspeed_rpm=active_rpm_limit * v["vel_limit_tolerance"])
    table = TelemetryTable(mode, columns, cfg=cfg, target_rpm=rpm, metadata=metadata,
                           compact=teleop, hidden_columns=("effective_current_lim_a",), notify=notify)
    started = time.monotonic()
    reason, detail = "fault", ""
    sensorless_settings = []
    encoder_counts = 0
    estimated_turns = 0.0
    previous_speed, previous_speed_time = None, started
    tracking_bad_since = fast_since = None
    phase_errors = []
    diagnostic_window_complete = False
    errors = None
    t0 = started
    command_rpm = 0.0
    try:
        if check_cancel is not None:
            check_cancel()
        if sensorless:
            encoder = a.encoder
            if (cfg["encoder_present"] and valid_encoder_calibration(a) and not encoder.config.use_index
                    and encoder.config.cpr == h["cpr"] and a.motor.config.pole_pairs == h["pole_pairs"]):
                encoder_direction = a.motor.config.direction
                encoder_offset = encoder.config.offset + encoder.config.offset_float
            else:
                encoder_direction = None
                if cfg["encoder_present"]:
                    print("Phase check unavailable. Run A/B calibration before the sensorless test.")
            ramp_cfg = a.config.sensorless_ramp
            estimator = a.sensorless_estimator.config
            handoff = math.copysign(550 / 60, rpm)
            electrical_speed = handoff * math.tau * h["pole_pairs"]
            settings = (
                (ramp_cfg, "current", min(3.0, h["current_lim"] / 2)),
                (ramp_cfg, "ramp_time", 0.4),
                (ramp_cfg, "ramp_distance", math.copysign(math.pi, rpm)),
                (ramp_cfg, "accel", electrical_speed / 2),
                (ramp_cfg, "vel", electrical_speed),
                (ramp_cfg, "finish_on_vel", True),
                (ramp_cfg, "finish_on_distance", False),
                (ramp_cfg, "finish_on_enc_idx", False),
                (estimator, "pm_flux_linkage", 5.51328895422 / (h["pole_pairs"] * h["motor_kv"])),
            )
            configure_for_calibration(dev, cfg)
            for obj, name, value in settings:
                sensorless_settings.append((obj, name, getattr(obj, name)))
                setattr(obj, name, value)
            cal = cfg["calibration"]
            calibrate_step(a, AXIS_STATE_MOTOR_CALIBRATION, "motor R/L calibration",
                           cal["motor_timeout_s"], lambda: valid_motor_calibration(a, cfg),
                           cal["poll_seconds"])

        else:
            apply_options(options, enable_anticogging=enable_anticogging)
        c.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        c.input_mode = INPUT_MODE_VEL_RAMP
        c.vel_gain = kp
        c.vel_integrator_gain = ki
        c.vel_limit = active_rpm_limit / 60.0
        c.vel_limit_tolerance = v["vel_limit_tolerance"]
        c.vel_ramp_rate = ramp
        a.controller.input_vel = handoff if sensorless else 0
        a.controller.input_torque = 0
        state = AXIS_STATE_SENSORLESS_CONTROL if sensorless else AXIS_STATE_CLOSED_LOOP_CONTROL
        if sensorless:
            a.config.watchdog_timeout = s["watchdog_timeout"]
            a.watchdog_feed()
            a.config.enable_watchdog = True
            a.requested_state = state
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                a.watchdog_feed()
                errors = fault_codes(a, sensorless=True)
                check_faults(errors, "ODrive fault during sensorless startup")
                if not s["operating_min_v"] <= dev.vbus_voltage <= s["operating_max_v"]:
                    raise RuntimeError("Bus voltage outside diagnostic range")
                if (a.current_state == state and a.lockin_state == LOCKIN_STATE_INACTIVE
                        and math.isclose(a.controller.vel_setpoint, handoff, abs_tol=0.1)):
                    break
                time.sleep(cfg["runtime"]["closed_loop_poll_s"])
            else:
                raise TimeoutError("Sensorless startup did not complete")
        else:
            enter_closed_loop(a, cfg, control_mode=CONTROL_MODE_VELOCITY_CONTROL,
                              input_mode=INPUT_MODE_VEL_RAMP, check_cancel=check_cancel)
        last_count = a.encoder.shadow_count if cfg["encoder_present"] else None
        t0 = last_time = time.monotonic()
        if check_cancel is not None:
            check_cancel()
        command_rpm = initial_rpm
        a.controller.input_vel = command_rpm / 60.0
        if notify:
            notify("running", None)
        while True:
            if check_cancel is not None:
                check_cancel()
            a.watchdog_feed()
            errors = fault_codes(a, sensorless=sensorless)
            check_faults(errors, "ODrive reported a fault")
            if a.current_state != state:
                raise RuntimeError("ODrive left closed-loop control")
            now = time.monotonic()
            setpoint = a.controller.vel_setpoint * 60.0
            count_rpm = encoder_phase_deg = actual_deg = None
            dt = now - last_time
            if cfg["encoder_present"] and dt > v["count_min_interval_s"]:
                count = a.encoder.shadow_count
                # shadow_count is signed 32-bit and can roll over.
                delta = (count - last_count + 2**31) % 2**32 - 2**31
                encoder_counts += delta
                count_rpm = delta * 60.0 / (cpr * dt)
                actual_deg = encoder_counts * 360.0 / cpr
                last_count, last_time = count, now
            if sensorless:
                estimated = a.sensorless_estimator.vel_estimate * 60.0
                phase_error = None
                if encoder_direction is not None:
                    radians_per_count = math.tau * h["pole_pairs"] / h["cpr"]
                    phase = encoder_direction * (encoder.count_in_cpr + 0.5 - encoder_offset) * radians_per_count
                    encoder_phase_deg = math.degrees(math.remainder(phase, math.tau))
                if encoder_direction is not None and 1 <= now - t0 <= 5:
                    before = encoder.count_in_cpr
                    observer_phase = a.sensorless_estimator.phase
                    after = encoder.count_in_cpr
                    travel = (after - before + cpr // 2) % cpr - cpr // 2
                    radians_per_count = math.tau * h["pole_pairs"] / cpr
                    # Bound phase uncertainty from the sequential USB reads.
                    if math.isfinite(observer_phase) and abs(travel * radians_per_count) <= math.radians(30):
                        encoder_phase = encoder_direction * (before + travel / 2 + 0.5 - encoder_offset) * radians_per_count
                        phase_error = math.degrees(math.remainder(encoder_phase - observer_phase, math.tau))
                        phase_errors.append(math.radians(phase_error))
                if previous_speed is not None:
                    estimated_turns += (previous_speed + estimated) * (now - previous_speed_time) / 120
                previous_speed, previous_speed_time = estimated, now
            else:
                estimated = a.encoder.vel_estimate * 60.0
                phase = getattr(a.encoder, "phase", None)
                if phase is not None:
                    encoder_phase_deg = math.degrees(phase)
            current = a.motor.current_control
            bus = dev.vbus_voltage
            values = (now-t0, setpoint, estimated, count_rpm, setpoint-estimated, actual_deg,
                      current.Iq_setpoint, current.Iq_measured, bus,
                      encoder_counts if cfg["encoder_present"] else None, encoder_phase_deg,
                      current.Id_measured, a.motor.effective_current_lim)
            if not all(value is None or math.isfinite(value) for value in values):
                raise RuntimeError("Invalid motor feedback")
            if sensorless:
                values += (estimated_turns, phase_error)
            if teleop:
                values += (command_rpm, live.ceiling, ramp * 60.0)
            if live is not None:
                values += live.values()
                peak_rpm = live.guard_rpm(setpoint)
                _, software_speed_limit, hard_speed_limit = speed_limits(v, peak_rpm, max_rpm, sensorless=False)
            table.sample(now-t0, *values, errors=errors)
            fast_since = overspeed(now, fast_since, estimated, None if sensorless else count_rpm,
                                   software_speed_limit, hard_speed_limit)
            if not s["operating_min_v"] <= bus <= s["operating_max_v"]:
                raise RuntimeError("Bus voltage outside diagnostic range")
            if sensorless:
                if estimated * rpm < 0 or abs(estimated) < 275:
                    raise DiagnosticStop("stall", "Sensorless estimator lost speed or reversed")
                if now-t0 >= 5:
                    diagnostic_window_complete = True
                    if not continuous:
                        reason = "normal_completion"
                        break
            elif live is None or command_rpm != 0:
                tracking_bad_since = stall(now, tracking_bad_since, setpoint, count_rpm, peak_rpm)
            if live is not None:
                was_braking = live.braking
                desired = live.update(now, setpoint, estimated, count_rpm, ramp * 60)
                if desired != command_rpm or live.braking != was_braking:
                    if check_cancel is not None:
                        check_cancel()
                    a.controller.input_vel = desired / 60.0
                    if live.braking and not was_braking:
                        a.controller.vel_integrator_torque = 0.0
                        # Apply the zero target without the driving ramp or its feed-forward.
                        c.input_mode = INPUT_MODE_PASSTHROUGH
                    elif was_braking and not live.braking:
                        c.input_mode = INPUT_MODE_VEL_RAMP
                    command_rpm = desired
                    tracking_bad_since = None
            time.sleep(v["sample_seconds"])
    except KeyboardInterrupt as exc:
        reason, detail = "user_stop", str(exc)
    except BaseException as exc:
        detail = str(exc)
        if isinstance(exc, TimeoutError):
            reason = "timeout"
        elif isinstance(exc, DiagnosticStop):
            reason = exc.reason
        raise
    finally:
        ended = time.monotonic()
        is_idle = safe_idle(a, cfg)
        if is_idle:
            try:
                if enable_anticogging:
                    c.anticogging.anticogging_enabled = False
                if sensorless:
                    # Require a fresh encoder offset scan before returning to encoder mode.
                    a.motor.config.direction = 0
                    for obj, name, value in reversed(sensorless_settings):
                        setattr(obj, name, value)
                (c.control_mode, c.input_mode, c.vel_gain, c.vel_integrator_gain,
                 c.vel_limit, c.vel_ramp_rate, c.vel_limit_tolerance,
                 a.config.watchdog_timeout) = old
                if teleop:
                    for path, obj, key, value, live_value in options:
                        if (path.startswith("controller.config") and value is not None
                                and key not in PROTECTIONS and key != "anticogging_enabled"):
                            setattr(obj, key, live_value)
            except Exception as exc:
                detail = f"{detail} Controller settings restore failed: {exc}".strip()
                print("WARNING:", detail)
        else:
            detail = f"{detail} IDLE not confirmed. Isolate motor power.".strip()
        if not errors or not any(errors):
            try:
                errors = fault_codes(a, sensorless=sensorless)
            except Exception:
                pass
        if errors and errors[3] & CONTROLLER_ERROR_OVERSPEED and reason == "fault":
            reason = "overspeed"
        try:
            last_command = None
            if teleop:
                last_command = dict(zip((column[0] for column in teleop_columns),
                                        (command_rpm, live.ceiling, ramp * 60.0), strict=True))
            table.finish(reason, detail, ended-t0, is_idle, errors, extra=last_command)
        finally:
            table.close()
        print(f"{axis_name(cfg)} IDLE confirmed; shaft coasts." if is_idle else
              "DANGER: IDLE not confirmed. Isolate motor power!")
        if not is_idle:
            raise RuntimeError("IDLE not confirmed; further testing blocked")
        if reason in ("fault", "overspeed", "timeout", "stall"):
            try:
                dump_errors(dev)
            except Exception:
                pass

    if sensorless and diagnostic_window_complete and cfg["encoder_present"]:
        if encoder_direction is not None:
            if len(phase_errors) < 10:
                print(f"Phase check inconclusive: {len(phase_errors)} tightly timed USB samples; need 10.")
            else:
                x = sum(math.cos(error) for error in phase_errors)
                y = sum(math.sin(error) for error in phase_errors)
                phase_mean = math.degrees(math.atan2(y, x))
                coherence = math.hypot(x, y) / len(phase_errors)
                if coherence < 0.8:
                    print(f"Phase check inconclusive: no stable phase difference "
                          f"({len(phase_errors)} samples, agreement {coherence:.2f}/1).")
                else:
                    print(f"Encoder phase minus observer: {phase_mean:+.1f} electrical degrees "
                          f"({len(phase_errors)} samples, agreement {coherence:.2f}/1).")
        if abs(estimated_turns) < 5:
            print("Encoder CPR check inconclusive: fewer than 5 estimated turns sampled.")
            return
        expected = h["cpr"]
        observed = abs(encoder_counts / estimated_turns)
        error = 100 * (observed / expected - 1)
        # shortcut: this uses observer speed, use a one-turn reference for exact CPR.
        verdict = "within ±5%" if abs(error) <= 5 else "outside ±5%"
        print(f"Encoder CPR estimate: {observed:.1f} vs {expected} configured "
              f"({error:+.1f}%, {verdict}).")
        print(f"A/B count change: {encoder_counts:+d} over {estimated_turns:+.2f} estimated turns.")
        if encoder_counts == 0:
            print("No net A/B counts detected.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--axis", type=int, choices=(0, 1), default=0)
    parser.add_argument("rpm", nargs="?", type=float,
                        help="Optional signed target RPM (omit to enter repeat-test console)")
    parser.add_argument("--sensorless", action="store_true",
                        help="Run a short motor-calibrated sensorless test with a signed target RPM")
    parser.add_argument("--continuous", action="store_true",
                        help="Keep the sensorless test running until Ctrl+C or a fault")
    args = parser.parse_args()
    if args.continuous and not args.sensorless:
        parser.error("--continuous requires --sensorless")
    if args.sensorless and args.rpm is None:
        parser.error("--sensorless requires a target RPM, for example 550")
    if args.rpm is not None:
        run(args.rpm, load_config(args.axis), sensorless=args.sensorless, continuous=args.continuous)
        return

    print(f"M{args.axis} VELOCITY | Repeat-test console")
    print("Enter signed RPM for each test (e.g. 30 or -100).")
    print(f"Ctrl+C stops the current test and returns M{args.axis} to IDLE. Enter 'q' to quit.")
    print("Press Enter to confirm each test before motion.")
    while True:
        try:
            raw = input("\nRPM > ").strip()
        except EOFError:
            print("\nInput closed. Exiting.")
            return
        if raw.lower() in ("q", "quit", "exit"):
            return
        if not raw:
            continue
        try:
            rpm = float(raw)
            cfg = load_config(args.axis)  # reread edited gains before every test
            if not math.isfinite(rpm) or not cfg["velocity"]["min_command_rpm"] <= abs(rpm) <= cfg["velocity"]["max_command_rpm"]:
                raise ValueError(f"RPM must be within +/-{cfg['velocity']['min_command_rpm']} and +/-{cfg['velocity']['max_command_rpm']}")
        except ValueError as exc:
            print("Invalid command:", exc)
            continue
        # Runtime hardware/safety errors intentionally exit the entire session.
        run(rpm, cfg)
        print("Test finished; motor commanded IDLE. Enter another RPM or 'q'.")


if __name__=="__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nVelocity test interrupted.")
    except Exception as exc:
        print("VELOCITY TEST FAILED:",exc)
        raise SystemExit(1)
