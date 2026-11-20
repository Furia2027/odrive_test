#!/usr/bin/env python3
"""Regulate M0 speed with Python PI and ODrive 0.5.1 torque control."""

import argparse
from contextlib import ExitStack, contextmanager
import csv
from datetime import datetime, timezone
import json
import math
from queue import Empty, Full, Queue
import sys
from threading import Event, Thread
import time
from types import SimpleNamespace
from typing import NamedTuple

from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE, CONTROL_MODE_TORQUE_CONTROL,
    INPUT_MODE_PASSTHROUGH, INPUT_MODE_TORQUE_RAMP, MOTOR_TYPE_HIGH_CURRENT,
)
from .m0_display import ERROR_KEYS, TelemetryTable, VELOCITY_COLUMNS
from .live import TARGET_COLUMNS, slew_rpm
from .motor_core import (
    FLOAT32_MAX, PROTECTIONS, check_anticogging, check_power, confirm_motion, enter_closed_loop,
    ensure_idle, exclusive_motion, fault_codes, find_odrive, load_config, preflight, safe_idle,
)
from .vel_m0 import DiagnosticStop, check_faults, overspeed, speed_limits, stall

ADMISSION_SECONDS = 2.0
LOG_QUEUE_SIZE = 64
DIAGNOSTIC_COLUMNS = (
    "read_span_s", "loop_interval_s", "id_setpoint_a", "encoder_shadow_count",
    "proportional_torque_nm", "integral_torque_nm", "ibus_a",
    "brake_resistor_armed", "brake_resistor_saturated",
)
CAPTURE_COLUMNS = (
    "elapsed_s", "read_span_s", "id_measured_a", "iq_measured_a",
    "id_setpoint_a", "iq_setpoint_a", "estimated_rpm", "encoder_shadow_count",
    "vbus_v", "effective_current_lim_a", "setpoint_rpm", "command_torque_nm", "axis_error",
    "previous_command_torque_nm", "firmware_torque_setpoint_nm", "loop_interval_s",
    "proportional_torque_nm", "integral_torque_nm", "ibus_a",
    "brake_resistor_armed", "brake_resistor_saturated",
)


class TorqueLog:
    def __init__(self, columns, *, cfg, rpm, metadata, capture, mode="TORQUE",
                 capture_columns=CAPTURE_COLUMNS, hidden=(), notify=None, show_readings=None):
        self.queue = Queue(maxsize=LOG_QUEUE_SIZE)
        self.done = Event()
        self.failure = None
        self.stop_reported = False
        with ExitStack() as files:
            self.table = TelemetryTable(
                mode, columns, cfg=cfg, target_rpm=rpm, metadata=metadata, notify=notify, show_readings=show_readings,
                hidden_columns=("rotation_deg", "active_torque_limit_nm", "effective_current_lim_a",
                                *DIAGNOSTIC_COLUMNS, "post_stop_faults_json", *hidden),
            )
            files.callback(self.table.close)
            self.capture = None
            if capture:
                path = self.table.log_path
                path = path.with_name(path.name.replace(f"m0_{mode.lower()}_", "m0_capture_", 1))
                stream = files.enter_context(path.open("x", encoding="utf-8", newline=""))
                self.capture = csv.writer(stream)
                self.capture.writerow(capture_columns)
                stream.flush()
                self.capture_indices = tuple(self.table.keys.index(key) for key in capture_columns)
                print(f" Control capture CSV: {path}", flush=True)
            self.files = files.pop_all()
        self.worker = Thread(target=self.write, name="torque-log", daemon=True)
        try:
            self.worker.start()
        except BaseException:
            self.files.close()
            raise

    def check(self):
        if self.failure is not None:
            raise DiagnosticStop("logging_failure", f"Torque log write failed: {self.failure}") from self.failure
        if self.queue.full():
            raise DiagnosticStop("logging_failure", "Torque log queue filled")

    def sample(self, values, errors, utc_time):
        try:
            self.queue.put_nowait((values, errors, utc_time))
        except Full as exc:
            raise DiagnosticStop("logging_failure", "Torque log queue filled") from exc

    def write(self):
        try:
            with self.files:
                while not (self.done.is_set() and self.queue.empty()):
                    try:
                        values, errors, utc_time = self.queue.get(timeout=0.05)
                    except Empty:
                        continue
                    try:
                        self.table.sample(values[0], *values, errors=errors, utc_time=utc_time)
                        if self.capture is not None:
                            row = values + errors
                            self.capture.writerow(row[i] for i in self.capture_indices)
                    except Exception as exc:
                        self.failure = exc
                        self.done.wait()
                        break
                reason, detail, elapsed, idle, errors = self.result
                if self.failure is not None and reason == "user_stop":
                    reason, detail = "logging_failure", f"Torque log write failed: {self.failure}"
                self.table.finish(reason, detail, elapsed, idle, errors,
                                  extra=self.stop_fields, utc_time=self.stop_time)
                self.stop_reported = True
                if idle:
                    print("M0 IDLE confirmed. Shaft coasts.", flush=True)
        except Exception as exc:
            if self.failure is None:
                self.failure = exc

    def finish(self, reason, detail, elapsed, idle, errors, *, extra, utc_time):
        self.result = (reason, detail, elapsed, idle, errors)
        self.stop_fields = extra
        self.stop_time = utc_time
        self.done.set()
        self.worker.join(timeout=2.0)
        if self.worker.is_alive():
            raise DiagnosticStop("logging_failure", "Torque log writer did not finish after shutdown")
        if self.failure is not None:
            raise DiagnosticStop("logging_failure", f"Torque log write failed: {self.failure}") from self.failure


class Feedback(NamedTuple):
    errors: tuple
    estimate: float
    count: int
    sampled_at: float
    read_span: float
    bus: float
    available_current: float
    iq_set: float
    iq_measured: float
    id_measured: float
    id_set: float
    ibus: float
    brake_armed: bool
    brake_saturated: bool
    position: float | None = None
    iq_measured_at: float | None = None


def read_feedback(dev, axis, *, expected_state, include_position=False):
    read_started = time.monotonic()
    errors = fault_codes(axis)
    try:
        check_faults(errors, "ODrive reported a fault")
    except DiagnosticStop as exc:
        exc.fault_masks = errors
        raise
    if axis.current_state != expected_state:
        raise RuntimeError("ODrive left the expected axis state")
    if expected_state == AXIS_STATE_IDLE and dev.axis1.current_state != AXIS_STATE_IDLE:
        raise RuntimeError("Both axes must remain IDLE during USB timing admission")
    estimate = axis.encoder.vel_estimate
    count = axis.encoder.shadow_count
    position = axis.encoder.pos_estimate if include_position else None
    sampled_at = time.monotonic()
    current = axis.motor.current_control
    bus, available_current = dev.vbus_voltage, axis.motor.effective_current_lim
    iq_set = current.Iq_setpoint
    iq_started = time.monotonic() if include_position else None
    iq_measured = current.Iq_measured
    iq_at = (iq_started + time.monotonic()) / 2 if include_position else None
    id_measured, id_set = current.Id_measured, current.Id_setpoint
    ibus, brake_armed, brake_saturated = dev.ibus, dev.brake_resistor_armed, dev.brake_resistor_saturated
    if not all(math.isfinite(x) for x in (estimate, bus, available_current, iq_set, iq_measured,
                                         id_measured, id_set, ibus)) or (include_position and not math.isfinite(position)):
        raise RuntimeError("Invalid motor feedback")
    if not brake_armed:
        raise RuntimeError("Brake resistor no longer armed")
    return Feedback(errors, estimate, count, sampled_at, time.monotonic() - read_started,
                    bus, available_current, iq_set, iq_measured, id_measured, id_set,
                    ibus, brake_armed, brake_saturated, position, iq_at)


def admit_control_period(dev, axis, *, reader=None):
    reader = read_feedback if reader is None else reader
    for hz in (40, 30):
        period = 1.0 / hz
        start = time.monotonic()
        for tick in range(int(ADMISSION_SECONDS * hz)):
            release = start + tick * period
            delay = release - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            reader(dev, axis, expected_state=AXIS_STATE_IDLE)
            if time.monotonic() - release > period / 2:
                break
        else:
            delay = start + ADMISSION_SECONDS - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            return period
    raise RuntimeError("USB feedback did not meet the read budget at 40 Hz or 30 Hz")


def validate_trial(rpm, cfg):
    v = cfg["velocity"]
    if cfg["axis_index"] != 0:
        raise ValueError("This torque test requires M0")
    if not math.isfinite(rpm) or not v["min_command_rpm"] <= abs(rpm) <= v["max_command_rpm"]:
        raise ValueError(f"RPM magnitude must be between {v['min_command_rpm']} and {v['max_command_rpm']}")
    settings = cfg.get("torque")
    if not isinstance(settings, dict):
        raise ValueError("motor_config.json requires an axes.m0.torque object")
    settings = {"firmware_limiter_gain": 0.40, "overspeed_ratio": v["vel_limit_tolerance"],
                "slew_rate_nm_per_s": 1.0, **settings}
    for key in ("torque_limit_nm", "kp", "ki", "firmware_limiter_gain", "slew_rate_nm_per_s"):
        value = settings.get(key)
        if type(value) not in (int, float) or not 2**-149 <= value <= FLOAT32_MAX:
            raise ValueError(f"axes.m0.torque.{key} must be a finite positive number")
    ratio = settings["overspeed_ratio"]
    if type(ratio) not in (int, float) or not 1.0 <= ratio <= FLOAT32_MAX:
        raise ValueError("axes.m0.torque.overspeed_ratio must be a finite number of at least 1.0 "
                         "within the firmware range")
    torque_ramp_rate = cfg["controller"].get("torque_ramp_rate")
    if torque_ramp_rate is not None and (type(torque_ramp_rate) not in (int, float)
                                         or not 2**-149 <= torque_ramp_rate <= FLOAT32_MAX):
        raise ValueError("axes.m0.controller.torque_ramp_rate must be null or a finite positive number")
    return settings


def firmware_limits(rpm, velocity, soft_limit, cap, gain):
    target = abs(rpm)
    original_ceiling = max(velocity["controller_min_limit_rpm"],
                           target + velocity["controller_headroom_rpm"])
    trip = original_ceiling * velocity["vel_limit_tolerance"]
    max_ceiling = min(trip, max(original_ceiling, soft_limit))
    ceiling = min(max_ceiling, max(original_ceiling, target + (cap / gain) * 60))
    if not 2**-149 <= cap <= FLOAT32_MAX or trip / 60 > FLOAT32_MAX:
        raise ValueError("Firmware torque cap or overspeed trip is out of range")
    return ceiling, trip


def available_drive_torque(cap, ceiling_rpm, gain, speed_in_drive_direction_rpm):
    # The 0.5.1 limiter applies this bound before the motor's torque and current cap.
    return max(0.0, min(cap, gain * ((ceiling_rpm - speed_in_drive_direction_rpm) / 60)))


def motor_torque_cap(motor_config, hardware, requested_limit):
    if motor_config.motor_type != MOTOR_TYPE_HIGH_CURRENT:
        raise RuntimeError("M0 torque control requires a calibrated high-current motor")
    torque_constant, live_cap = motor_config.torque_constant, motor_config.torque_lim
    if not (0 < torque_constant < math.inf and live_cap > 0):
        raise RuntimeError("Invalid firmware torque settings")
    configured_cap = hardware["torque_lim"] if hardware["torque_lim"] is not None else math.inf
    return min(requested_limit, live_cap, configured_cap,
               min(motor_config.current_lim, hardware["current_lim"]) * torque_constant)


def pi_step(error, dt, integral, kp, ki, limit, *, hold_integral=False,
            previous_command=0.0, slew_rate=math.inf, speed_bounds=None):
    proportional = kp * error
    integral = max(-limit, min(limit, integral))
    step = slew_rate * dt
    low, high = speed_bounds if speed_bounds is not None else (-limit, limit)
    lo = max(low, min(high, previous_command - step))
    hi = max(low, min(high, previous_command + step))
    output = proportional + integral
    # Integrate up to the saturation boundary, or back toward it.
    if not hold_integral and error > 0 and output < hi:
        integral = min(integral + ki * error * dt, hi - proportional)
    elif not hold_integral and error < 0 and output > lo:
        integral = max(integral + ki * error * dt, lo - proportional)
    integral = max(-limit, min(limit, integral))
    return max(lo, min(hi, proportional + integral)), integral


@contextmanager
def motion_session(dev, axis, cfg, logger, changes):
    session = SimpleNamespace(started=time.monotonic())
    original = []
    reason, detail, stop_errors = "fault", "", None
    stop_fields = {}
    attempted_motion = False
    try:
        logger.check()
        ensure_idle(dev, cfg)
        check_power(dev, cfg)
        check_faults(fault_codes(dev.axis1, sensorless=True), "ODrive M1 reported a fault")
        original = [(obj, key, getattr(obj, key)) for obj, key, _ in changes]
        original.append((axis.config, "watchdog_timeout", axis.config.watchdog_timeout))
        attempted_motion = True
        yield session
    except KeyboardInterrupt as exc:
        reason, detail = "user_stop", str(exc)
    except Exception as exc:
        detail = str(exc)
        stop_errors = getattr(exc, "fault_masks", None)
        if isinstance(exc, DiagnosticStop):
            reason = exc.reason
        elif isinstance(exc, TimeoutError):
            reason = "timeout"
        raise
    finally:
        elapsed = time.monotonic() - session.started
        stopped_utc = datetime.now(timezone.utc)
        idle = False
        try:
            idle = safe_idle(axis, cfg) if attempted_motion else None
            if attempted_motion and not idle:
                raise RuntimeError("IDLE not confirmed. Isolate motor power.")
            restore_failures = []
            for obj, key, value in reversed(original if attempted_motion else ()):
                try:
                    setattr(obj, key, value)
                except Exception as exc:
                    restore_failures.append(f"{key} ({exc})")
            if attempted_motion:
                for name, active_axis in (("m0", dev.axis0), ("m1", dev.axis1)):
                    try:
                        masks = fault_codes(active_axis, sensorless=True)
                        stop_fields[name] = dict(zip(ERROR_KEYS, masks, strict=True))
                        if name == "m0" and stop_errors is None:
                            stop_errors = masks
                    except Exception as exc:
                        stop_fields[name] = {"read_error": str(exc)}
            if restore_failures:
                raise RuntimeError("M0 is IDLE. Could not restore " + ", ".join(restore_failures))
            if reason == "user_stop" and stop_errors is not None:
                check_faults(stop_errors, "ODrive reported a fault in the post-stop check")
        except Exception as exc:
            if reason == "user_stop":
                reason = exc.reason if isinstance(exc, DiagnosticStop) else "fault"
            detail = f"{detail} {exc}".strip()
            raise
        finally:
            active_error = sys.exception()
            extra = {"post_stop_faults_json": json.dumps(stop_fields)} if stop_fields else {}
            try:
                masks = stop_errors if stop_errors is not None else (None,) * len(ERROR_KEYS)
                logger.finish(reason, detail, elapsed, idle, masks, extra=extra, utc_time=stopped_utc)
            except DiagnosticStop as exc:
                if active_error is None:
                    exc.stop_reported = logger.stop_reported
                    raise
            finally:
                if active_error is not None and logger.stop_reported:
                    active_error.stop_reported = True


@exclusive_motion()
def run(rpm, cfg, dev=None, *, capture=False, confirm=None, check_cancel=None, live=None, notify=None,
        show_readings=None, check_setup=None):
    settings = validate_trial(rpm, cfg)
    requested_limit, kp, ki = (settings[key] for key in ("torque_limit_nm", "kp", "ki"))
    limiter_gain = settings["firmware_limiter_gain"]
    overspeed_ratio = settings["overspeed_ratio"]
    slew_rate = settings["slew_rate_nm_per_s"]
    torque_ramp_rate = cfg["controller"].get("torque_ramp_rate")
    input_mode = INPUT_MODE_PASSTHROUGH if torque_ramp_rate is None else INPUT_MODE_TORQUE_RAMP
    input_mode_name = "PASSTHROUGH" if torque_ramp_rate is None else "TORQUE_RAMP"
    v = {**cfg["velocity"], "vel_limit_tolerance": overspeed_ratio}
    s, h = cfg["safety"], cfg["hardware"]
    dev = dev if dev is not None else find_odrive(cfg)
    axis = preflight(dev, cfg)
    check_faults(fault_codes(dev.axis1, sensorless=True), "ODrive M1 reported a fault")
    period = admit_control_period(dev, axis)
    watchdog = min(s["watchdog_timeout"], 0.15)
    if watchdog <= 2 * period:
        raise RuntimeError("Torque trial watchdog must exceed two control periods")
    peak_rpm = abs(rpm)
    direction = math.copysign(1, rpm)
    ceiling = live.ceiling if live is not None else peak_rpm
    _, tracking_limit, _ = speed_limits(v, ceiling, v["max_command_rpm"], sensorless=False)
    torque_cap = motor_torque_cap(axis.motor.config, h, requested_limit)
    if check_setup is not None:
        check_setup(period, torque_cap)
    speed_ceiling_rpm, firmware_trip_rpm = firmware_limits(ceiling, v, tracking_limit, torque_cap, limiter_gain)
    torque_at_target = available_drive_torque(torque_cap, speed_ceiling_rpm, limiter_gain, peak_rpm)
    ramp = v["fast_vel_ramp_rate"] if peak_rpm > v["fast_ramp_above_rpm"] else v["vel_ramp_rate"]
    live_prompt = live.confirmation(ramp) if live is not None else ""
    initial_rpm = live.start_rpm if live is not None else rpm
    (confirm or confirm_motion)(
        f"TORQUE M0 | target {initial_rpm:+g} RPM | requested {requested_limit:g} N·m | "
        f"trial torque cap {torque_cap:.3g} N·m | estimated available at target {torque_at_target:.3g} N·m | "
        f"firmware overspeed trip {firmware_trip_rpm:g} RPM | {1 / period:g} Hz{live_prompt}")
    axis = preflight(dev, cfg)
    controller, motor, encoder = axis.controller, axis.motor, axis.encoder
    cpr = encoder.config.cpr
    current = motor.current_control
    c, m = controller.config, motor.config
    if motor_torque_cap(m, h, requested_limit) != torque_cap:
        raise RuntimeError("M0 torque limits changed during confirmation. Start the trial again.")
    torque_constant, motor_direction = m.torque_constant, m.direction
    check_anticogging(axis, enable=False)
    entry_cfg = {**cfg, "safety": {**s, "watchdog_timeout": watchdog},
                 "runtime": {**cfg["runtime"],
                             "closed_loop_poll_s": min(cfg["runtime"]["closed_loop_poll_s"], watchdog / 4)}}
    changes = (
        (m, "torque_lim", torque_cap),
        (c.anticogging, "anticogging_enabled", False),
        (c, "control_mode", CONTROL_MODE_TORQUE_CONTROL),
        (c, "input_mode", input_mode),
        (c, "vel_limit", speed_ceiling_rpm / 60),
        (c, "vel_gain", limiter_gain),
        (c, "vel_limit_tolerance", firmware_trip_rpm / speed_ceiling_rpm),
        *((c, key, True) for key in PROTECTIONS),
    )
    if torque_ramp_rate is not None:
        changes += ((c, "torque_ramp_rate", torque_ramp_rate),)
    columns = VELOCITY_COLUMNS + (
        ("id_measured_a", "Id Act A", "+.2f"),
        ("effective_current_lim_a", "I Limit A", ".2f"),
        ("command_torque_nm", "Torque Nm", "+.3f"),
        ("active_torque_limit_nm", "Limit Nm", ".3f"),
        ("previous_command_torque_nm", "Prev Nm", "+.3f"),
        ("firmware_torque_setpoint_nm", "FW Nm", "+.3f"),
        ("integration_held", "I held", ""),
    ) + tuple((key, "", "") for key in (*DIAGNOSTIC_COLUMNS, "post_stop_faults_json"))
    metadata = {
        "torque_limit_nm": requested_limit, "firmware_torque_limit_nm": torque_cap,
        "drive_torque_at_target_nm": torque_at_target, "torque_constant_nm_per_a": torque_constant,
        "input_mode": input_mode_name, "torque_ramp_rate_nm_per_s": torque_ramp_rate,
        "kp": kp, "ki": ki, "ramp_rpm_per_s": ramp * 60,
        "firmware_limit_rpm": speed_ceiling_rpm, "firmware_overspeed_rpm": firmware_trip_rpm,
        "firmware_limiter_gain": limiter_gain, "overspeed_ratio": overspeed_ratio,
        "control_period_s": period, "watchdog_timeout_s": watchdog,
        "slew_rate_nm_per_s": slew_rate, "current_lim_margin_a": m.current_lim_margin,
        "current_report_filter_k": current.I_measured_report_filter_k,
    }
    if capture:
        metadata["capture_period_s"] = period
    live_columns = TARGET_COLUMNS if live is not None else ()
    if live is not None:
        metadata["live_ceiling_rpm"] = ceiling
    logger = TorqueLog(columns + live_columns, cfg=cfg, rpm=rpm, metadata=metadata, capture=capture,
                       capture_columns=CAPTURE_COLUMNS + tuple(c[0] for c in live_columns), notify=notify,
                       show_readings=show_readings)
    with motion_session(dev, axis, cfg, logger, changes) as session:
        if check_cancel is not None:
            check_cancel()
        print("Runs until Ctrl+C or a fault. Speed correction can apply braking torque.", flush=True)
        controller.input_torque = 0.0
        controller.input_vel = 0.0
        for obj, key, value in changes:
            setattr(obj, key, value)
        enter_closed_loop(axis, entry_cfg, control_mode=CONTROL_MODE_TORQUE_CONTROL,
                          input_mode=input_mode, check_cancel=check_cancel)
        started = session.started = time.monotonic()
        if not math.isclose(axis.config.watchdog_timeout, watchdog, rel_tol=1e-6):
            raise RuntimeError("Torque-trial watchdog setting did not take effect")
        last_count = encoder.shadow_count
        previous = time.monotonic()
        if notify:
            notify("running", None)
        tick = 1
        counts = 0
        setpoint = integral = previous_command = 0.0
        stalled_since = None
        while True:
            release = started + tick * period
            delay = release - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            if check_cancel is not None:
                check_cancel()
            wake = time.monotonic()
            if wake > release + period / 2:
                raise DiagnosticStop("timing_overrun", "Control tick started past its timing budget")
            deadline = started + (tick + 1) * period
            logger.check()
            feedback = read_feedback(dev, axis, expected_state=AXIS_STATE_CLOSED_LOOP_CONTROL)
            sampled_utc = datetime.now(timezone.utc)
            estimate, count, now = feedback.estimate, feedback.count, feedback.sampled_at
            dt = now - previous
            if not 0 < dt <= 1.5 * period:
                raise DiagnosticStop("timing_overrun", f"Encoder feedback interval was {dt * 1000:.2f} ms")
            bus, available_current = feedback.bus, feedback.available_current
            iq_set, iq_measured, id_measured = feedback.iq_set, feedback.iq_measured, feedback.id_measured
            if not s["operating_min_v"] <= bus <= s["operating_max_v"]:
                raise RuntimeError(f"Bus voltage outside diagnostic range ({bus:.2f} V)")
            if available_current <= 0:
                raise RuntimeError("Invalid motor feedback: no effective current available")

            # shadow_count is signed 32-bit and can roll over.
            delta = (count - last_count + 2**31) % 2**32 - 2**31
            counts += delta
            count_rpm = delta * 60 / (cpr * dt)
            estimated_rpm = estimate * 60
            active_limit = min(torque_cap, available_current * torque_constant)
            speed_bounds = None
            taper_rpm = speed_ceiling_rpm
            if live is None:
                setpoint = math.copysign(min(peak_rpm / 60, abs(setpoint) + ramp * dt), rpm)
            else:
                magnitude = live.guard_rpm(setpoint * 60)
                _, tracking, _ = speed_limits(v, magnitude, v["max_command_rpm"], sensorless=False)
                _, trip = firmware_limits(magnitude, v, tracking, torque_cap, limiter_gain)
                overspeed(now, None, estimated_rpm, count_rpm, trip, trip)
                was_braking = live.braking
                target = live.update(now, setpoint * 60, estimated_rpm, count_rpm, ramp * 60)
                setpoint = 0.0 if live.braking else slew_rpm(setpoint, target / 60, ramp * dt)
                peak_rpm = live.guard_rpm(setpoint * 60)
                _, tracking, _ = speed_limits(v, peak_rpm, v["max_command_rpm"], sensorless=False)
                taper_rpm, _ = firmware_limits(peak_rpm, v, tracking, torque_cap, limiter_gain)
                # Apply the directional firmware taper for the active target inside the fixed envelope.
                speed_bounds = tuple(max(-active_limit, min(active_limit, (edge - estimate) * limiter_gain))
                                     for edge in (-taper_rpm / 60, taper_rpm / 60))
                direction = math.copysign(1, setpoint)
                if live.reversed or live.braking != was_braking:
                    integral, stalled_since = 0.0, None
            error = setpoint - estimate
            # Iq was read before this iteration's write, so compare it to the last request.
            firmware_torque = iq_set * motor_direction * torque_constant
            clipped_torque = previous_command - firmware_torque
            hold_integral = (abs(clipped_torque) > max(0.001, 0.01 * active_limit)
                             and clipped_torque * error > 0)
            command, next_integral = pi_step(error, dt, integral, kp, ki, active_limit,
                                             hold_integral=hold_integral,
                                             previous_command=previous_command, slew_rate=slew_rate,
                                             speed_bounds=speed_bounds)
            hold_integral = hold_integral or (error != 0 and next_integral == integral)
            integral = next_integral
            drive_limit = available_drive_torque(active_limit, taper_rpm, limiter_gain,
                                                 direction * estimated_rpm)
            driving_at_limit = drive_limit > 0 and direction * firmware_torque >= 0.99 * drive_limit
            stalled_since = stall(now, stalled_since, setpoint * 60,
                                  count_rpm if driving_at_limit and setpoint else None, peak_rpm)

            logger.check()
            if live is not None and check_cancel is not None:
                check_cancel()
            if time.monotonic() >= deadline:
                raise DiagnosticStop("timing_overrun", "Control tick missed the torque write deadline")
            controller.input_torque = command
            if time.monotonic() >= deadline:
                raise DiagnosticStop("timing_overrun", "Torque write missed the watchdog feed deadline")
            axis.watchdog_feed()
            if time.monotonic() >= deadline:
                raise DiagnosticStop("timing_overrun", "Watchdog feed exceeded the control tick deadline")
            previous, last_count = now, count
            elapsed = now - started
            logger.sample((elapsed, setpoint * 60, estimated_rpm, count_rpm, error * 60,
                           counts * 360 / cpr, iq_set, iq_measured, bus,
                           id_measured, available_current, command, active_limit, previous_command, firmware_torque,
                           hold_integral, feedback.read_span, dt, feedback.id_set, count, kp * error,
                           integral, feedback.ibus, feedback.brake_armed, feedback.brake_saturated, "",
                           *(live.values() if live is not None else ())),
                          feedback.errors, sampled_utc)
            previous_command = command
            tick += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rpm", nargs="?", type=float, help="Signed target RPM (omit for the repeat-test console)")
    parser.add_argument("--capture", action=argparse.BooleanOptionalAction, default=True,
                        help="Save control-tick snapshots to a companion CSV (default: enabled)")
    args = parser.parse_args()
    if args.rpm is not None:
        run(args.rpm, load_config(), capture=args.capture)
        return

    print("M0 TORQUE | Enter signed RPM for each trial. Ctrl+C stops motion. Enter 'q' to quit.")
    while True:
        try:
            raw = input("\nRPM > ").strip()
        except EOFError:
            return
        if raw.lower() in ("q", "quit", "exit"):
            return
        if not raw:
            continue
        try:
            rpm = float(raw)
            cfg = load_config()
            validate_trial(rpm, cfg)
        except ValueError as exc:
            print("Invalid command or configuration", exc)
            continue
        run(rpm, cfg, capture=args.capture)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nTorque test interrupted.")
    except Exception as exc:
        print("TORQUE TEST FAILED", exc)
        raise SystemExit(1)
