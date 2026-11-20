"""Offline M0 trial measurements and a log-fitted rotor model."""
import csv
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from hashlib import sha256
from io import StringIO
import json
import math
from itertools import repeat
from pathlib import Path
from typing import NamedTuple

import numpy as np
from odrive.enums import MOTOR_ERROR_CURRENT_LIMIT_VIOLATION

from motor import control_m0 as position
from motor import torque_m0 as torque
from motor.live import effective_mode
from motor.m0_display import ERROR_KEYS
from motor.vel_m0 import overspeed, speed_limits, stall

from .recorder import RecorderTrace

FIRMWARE_PERIOD = 1 / 8000  # ODrive v3, fw-v0.5.1 current-measurement period.


class Metrics(NamedTuple):
    ripple: float
    overshoot: float
    ramp_error: float
    plateau_error: float


def measurement_window(rpm, time, reference):
    at_target = np.flatnonzero(np.isclose(reference, rpm, atol=1e-6, rtol=0))
    if len(at_target):
        arrival = time[at_target[0]]
        settled, end = arrival + 2, arrival + 7
        if time[-1] >= end and np.allclose(reference[(time >= arrival) & (time <= end)],
                                             rpm, atol=1e-6, rtol=0):
            return arrival, settled, end
    return None


def measure(rpm, time, reference, speed):
    window = measurement_window(rpm, time, reference)
    if window is None:
        return None
    arrival, settled, end = window
    # Weight each report by its time in the fixed window, including partial boundary intervals.
    next_time = np.r_[time[1:], time[-1]]
    plateau = np.maximum(0.0, np.minimum(next_time, end) - np.maximum(time, settled))
    ramp = np.maximum(0.0, np.minimum(next_time, arrival) - time)
    mean = np.average(speed, weights=plateau)
    errors = speed - reference
    peak = (math.copysign(1, rpm) * speed[time <= end]).max() - abs(rpm)
    ramp_rmse = np.sqrt(np.average(errors**2, weights=ramp)) if ramp.any() else abs(errors[0])
    return Metrics(float(np.sqrt(np.average((speed - mean)**2, weights=plateau))),
                   max(0.0, float(peak)), float(ramp_rmse), abs(float(mean) - rpm))


def clamped(value, low, high):
    return min(high, max(low, value))


class Setup(NamedTuple):
    rpm: float
    period: float
    ramp: float
    cap: float
    ceiling: float
    trip: float
    cpr: int
    limiter: float
    slew: float
    torque_ramp: float | None
    software_guard: tuple[float, float] | None = None
    encoder_bandwidth: float = 0.0


def setup_for(cfg, rpm, period, cap):
    settings = torque.validate_trial(rpm, cfg)
    v = {**cfg["velocity"], "vel_limit_tolerance": settings["overspeed_ratio"]}
    ceiling, tracking, _ = speed_limits(v, abs(rpm), v["max_command_rpm"], sensorless=False)
    ramp = v["vel_ramp_rate"]
    if effective_mode("auto", rpm) == "position":
        trip = ceiling * settings["overspeed_ratio"]
    else:
        ceiling, trip = torque.firmware_limits(rpm, v, tracking, cap, settings["firmware_limiter_gain"])
        if abs(rpm) > v["fast_ramp_above_rpm"]:
            ramp = v["fast_vel_ramp_rate"]
    return Setup(rpm, period, ramp, cap, ceiling / 60, trip / 60, cfg["hardware"]["cpr"],
                 settings["firmware_limiter_gain"], settings["slew_rate_nm_per_s"],
                 cfg["controller"].get("torque_ramp_rate"), encoder_bandwidth=cfg["hardware"]["bandwidth"])


class Faults(NamedTuple):
    samples: tuple
    stop: dict
    post_stop: dict
    invalid: bool

    @property
    def current_limit(self):
        return any(masks.get("motor_error", 0) & MOTOR_ERROR_CURRENT_LIMIT_VIOLATION
                   for masks in (self.stop, self.post_stop.get("m0", {}), *(m for _, m in self.samples)))

    @property
    def any_fault(self):
        return any(any(masks.values()) for masks in (self.stop, *self.post_stop.values(),
                                                     *(m for _, m in self.samples)))


@dataclass
class Trial:
    path: Path
    setup: Setup
    gains: tuple
    time: np.ndarray
    reference: np.ndarray
    speed: np.ndarray
    counts: np.ndarray
    applied: np.ndarray
    reason: str
    clean: bool
    drive_fault: bool = False
    torque_set: np.ndarray | None = None
    estimated_rpm: np.ndarray | None = None
    started_at: datetime | None = None
    stopped_at: datetime | None = None
    stop_elapsed: float | None = None
    faults: Faults | None = None
    waveform_issue: str = ""
    digest: bytes = b""
    position_timing: tuple[float, np.ndarray] | None = None
    torque_calibration: tuple[float, float] | None = None
    board_serial: str | None = None
    recorder: RecorderTrace | None = None
    position_initial: tuple | None = None
    position_writes: np.ndarray | None = None
    position_current_time: np.ndarray | None = None
    position_feedback: np.ndarray | None = None

    def metrics(self):
        return measure(self.setup.rpm, self.time, self.reference, self.speed) if self.clean else None

    def initial_angle(self):
        if self.position_initial is not None:
            return self.position_initial[1] / self.setup.cpr
        count = self.counts[0]
        if self.position_timing is not None:
            interval, _ = self.position_timing
            count -= round(self.speed[0] * interval * self.setup.cpr / 60)
        return count / self.setup.cpr

    def current_sample_times(self, fraction=4 / 9):
        if self.position_current_time is not None:
            return self.position_current_time
        if self.position_timing is None:
            return self.time
        # Legacy logs approximate Iq timing as a fraction of the remaining USB reads.
        return self.time + fraction * self.position_timing[1]

    def position_inputs(self):
        return {"position_timing": self.position_timing, "position_initial": self.position_initial,
                "position_writes": self.position_writes}


def compatible_config(logged, current):
    return all(logged.get(k) == current[k] for k in ("odrive_serial_number", "board", "hardware", "safety"))


def read_faults(samples, stop):
    invalid = False

    def read_masks(row, *, json_masks=False):
        nonlocal invalid
        masks = {}
        for key in ERROR_KEYS:
            value = row.get(key)
            if value is None or value == "":
                continue
            try:
                if json_masks and type(value) is not int:
                    raise ValueError("invalid fault mask")
                mask = int(value)
                if mask < 0:
                    raise ValueError("negative fault mask")
                masks[key] = mask
            except (TypeError, ValueError):
                invalid = True
        return masks

    sample_masks = []
    for row in samples:
        masks = read_masks(row)
        if any(masks.values()):
            sample_masks.append((row.get("elapsed_s"), masks))
    stop_masks = read_masks(stop)
    post_stop = {}
    try:
        raw = json.loads(stop.get("post_stop_faults_json") or "{}")
        for axis in ("m0", "m1"):
            masks = raw.get(axis)
            if isinstance(masks, dict):
                post_stop[axis] = read_masks(masks, json_masks=True)
            elif masks is not None:
                invalid = True
    except (AttributeError, TypeError, ValueError):
        invalid = True
    return Faults(tuple(sample_masks), stop_masks, post_stop, invalid)


def read_waveform(samples, period, torque_constant):
    if len(samples) < 2:
        raise ValueError("too few waveform samples")
    keys = ("elapsed_s", "setpoint_rpm", "encoder_count_rpm", "encoder_shadow_count",
            "firmware_torque_setpoint_nm", "iq_setpoint_a", "iq_measured_a")
    time, reference, speed, counts, torque_set, iq_set, iq_measured = np.array(
        [[float(row[k]) for k in keys] for row in samples], dtype=float).T
    dt = np.diff(time)
    if (not all(np.isfinite(x).all() for x in (time, reference, speed, counts, torque_set, iq_set, iq_measured))
            or time[0] <= 0 or not ((0 < dt) & (dt <= 1.5 * period + 1e-9)).all()):
        raise ValueError("invalid sample data")
    delta = (np.diff(counts) + 2**31) % 2**32 - 2**31
    counts = np.r_[counts[0], counts[0] + np.cumsum(delta)]
    polarity = np.sign(np.dot(iq_set, torque_set))
    if not 0 < torque_constant < math.inf or not math.isfinite(polarity) or not polarity:
        raise ValueError("unknown applied torque polarity or units")
    estimated_rpm = None
    if "estimated_rpm" in samples[0]:
        estimated_rpm = np.array([float(row["estimated_rpm"]) for row in samples])
        if not np.isfinite(estimated_rpm).all():
            raise ValueError("invalid encoder estimate")
    scale = polarity * torque_constant
    calibration = float(scale), float(np.max(np.abs(torque_set - iq_set * scale)))
    return time, reference, speed, counts, iq_measured * scale, torque_set, estimated_rpm, calibration


def position_timing(samples, time, period):
    try:
        first = float(samples[0]["loop_interval_s"])
        spans = np.array([float(row["read_span_s"]) for row in samples])
    except (KeyError, TypeError, ValueError):
        return None
    if not 0 < first <= 1.5 * period or not ((spans > 0) & (spans < period)).all():
        return None
    # shortcut: use the end of feedback reads as the write boundary until USB writes have timestamps.
    since_release = time - period * np.arange(1, len(time) + 1)
    return first, np.maximum(0, spans - np.maximum(0, since_release))


def read_position_capture(samples, time, counts, period):
    initial = [float(samples[0][key]) for key in position.POSITION_INITIAL_FIELDS]
    brackets = np.array([[float(row[key]) for key in position.POSITION_WRITE_FIELDS] for row in samples])
    current_time = np.array([float(row["iq_measured_elapsed_s"]) for row in samples])
    ordered = np.column_stack((time, current_time, brackets))
    if (not np.isfinite(initial).all() or not np.isfinite(ordered).all()
            or not 0 <= initial[0] < time[0] or not 0 < time[0] - initial[0] <= 1.5 * period
            or initial[1] != int(initial[1]) or not -2**31 <= initial[1] < 2**31
            or np.any(np.diff(ordered, axis=1) < 0) or np.any(brackets[:-1, -1] >= time[1:])):
        raise ValueError("invalid position capture timestamps or initial state")
    initial[1] += round((counts[0] - initial[1]) / 2**32) * 2**32
    return tuple(initial), brackets.reshape(-1, 2, 2).mean(axis=2), current_time


def read_trial(path, mode, cfg):
    """Read compatible capture metadata and preserve faults even without a usable waveform."""
    content = path.read_bytes()
    rows = []
    parse_issue = ""
    try:
        rows.extend(csv.DictReader(StringIO(content.decode("utf-8"), newline=""), strict=True))
    except csv.Error as exc:
        parse_issue = f"malformed CSV: {exc}"
    if not rows or rows[0]["event"] != "start":
        raise ValueError(parse_issue or "incomplete trial")
    start = rows[0]
    stop = rows[-1] if rows[-1]["event"] == "stop" else {}
    rpm = float(start["target_rpm"])
    if start["axis"] != "0" or start["control_mode"].lower() != mode or effective_mode("auto", rpm) != mode:
        raise ValueError("different axis or mode")
    header = json.loads(start["config_json"])
    old, meta = header["config"], header["trial"]
    if not compatible_config(old, cfg) or "live_ceiling_rpm" in meta:
        raise ValueError("different hardware, protection settings, or live target")
    period = meta["control_period_s"]
    cap = meta.get("firmware_torque_limit_nm", meta.get("torque_limit_nm"))
    if not (math.isclose(period, 1 / 40) or math.isclose(period, 1 / 30)) or not 0 < cap < math.inf:
        raise ValueError("invalid control period or torque cap")
    setup = setup_for(cfg, rpm, period, cap)
    common = {"ramp_rpm_per_s": setup.ramp * 60, "firmware_limit_rpm": setup.ceiling * 60,
              "firmware_overspeed_rpm": setup.trip * 60}
    if mode == "position":
        common.update(vel_integrator_gain=0, max_lead_turns=position.LEAD_TURNS,
                      following_limit_turns=position.FOLLOWING_LIMIT_TURNS, no_progress_s=position.NO_PROGRESS_S)
        names = ("position_gain", "vel_gain")
        max_cap = min(position.TORQUE_LIMIT_NM, cfg["torque"]["torque_limit_nm"])
    else:
        common.update(firmware_limiter_gain=setup.limiter, slew_rate_nm_per_s=setup.slew)
        names = ("kp", "ki")
        max_cap = cfg["torque"]["torque_limit_nm"]
    input_mode = "TORQUE_RAMP" if mode == "torque" and setup.torque_ramp is not None else "PASSTHROUGH"
    if (cap > max_cap + 1e-6 or meta["input_mode"] != input_mode
            or (mode == "torque" and meta.get("torque_ramp_rate_nm_per_s") != setup.torque_ramp)
            or any(not math.isclose(meta[k], value, abs_tol=1e-6) for k, value in common.items())):
        raise ValueError("different effective trial settings")
    if "software_limit_rpm" in meta or "hard_estimator_limit_rpm" in meta:
        soft, hard = float(meta["software_limit_rpm"]), float(meta["hard_estimator_limit_rpm"])
        if not 0 < soft <= hard < math.inf:
            raise ValueError("invalid historical software guard")
        setup = setup._replace(software_guard=(soft, hard))
    gains = tuple(float(meta[name]) for name in names)
    if not all(math.isfinite(gain) and gain > 0 for gain in gains):
        raise ValueError("invalid recorded gains")
    started_at = datetime.fromisoformat(start["utc_time"])
    if started_at.tzinfo is None:
        raise ValueError("invalid capture start time")
    samples = [row for row in rows if row["event"] == "sample"]
    faults = read_faults(samples, stop)
    stopped_at = stop_elapsed = None
    issue = parse_issue
    try:
        if not stop:
            raise ValueError("missing stop record")
        stopped_at = datetime.fromisoformat(stop["utc_time"])
        if stopped_at.tzinfo is None or stopped_at < started_at:
            stopped_at = None
            raise ValueError("invalid capture stop time")
        stop_elapsed = float(stop["elapsed_s"])
        if not math.isfinite(stop_elapsed) or stop_elapsed < 0:
            raise ValueError("invalid stop elapsed time")
        if any(row["event"] != "sample" for row in rows[1:-1]) or any(
                row["axis"] != start["axis"] or row["control_mode"] != start["control_mode"]
                or float(row["target_rpm"]) != rpm for row in (*samples, stop)):
            raise ValueError("inconsistent capture rows")
        time, reference, speed, counts, applied, torque_set, estimated_rpm, calibration = read_waveform(
            samples, period, meta["torque_constant_nm_per_a"])
        if stop_elapsed < time[-1]:
            raise ValueError("stop elapsed time precedes the last sample")
    except (KeyError, TypeError, ValueError) as exc:
        issue = issue or str(exc)
        time = reference = speed = counts = applied = np.empty(0)
        torque_set = estimated_rpm = calibration = None
    reason = stop.get("stop_reason", "incomplete")
    post_stop_complete = all(set(faults.post_stop.get(axis, {})) == set(ERROR_KEYS) for axis in ("m0", "m1"))
    if faults.invalid or (stop and not post_stop_complete):
        issue = issue or "missing or invalid post-stop fault data"
    timing = position_timing(samples, time, period) if mode == "position" and len(time) else None
    initial = writes = current_time = position_feedback = None
    if mode == "position" and len(time):
        keys = ("active_position_turns", "active_velocity_feedforward_rpm",
                "measured_position_turns", "active_torque_limit_nm")
        try:
            if all(key in samples[0] for key in keys):
                position_feedback = np.array([[float(row[key]) for key in keys] for row in samples])
                if not np.isfinite(position_feedback).all() or np.any(position_feedback[:, 3] <= 0):
                    raise ValueError("invalid position feedback")
            if any(key in samples[0] for key in position.POSITION_TIMING_FIELDS):
                initial, writes, current_time = read_position_capture(samples, time, counts, period)
        except (KeyError, TypeError, ValueError) as exc:
            issue = issue or f"invalid position capture: {exc}"
    clean = (reason == "user_stop" and stop.get("idle_confirmed") == "True"
             and not issue and not faults.any_fault)
    return Trial(path, setup, gains, time, reference, speed, counts, applied, reason, clean,
                 faults.any_fault, torque_set, estimated_rpm, started_at, stopped_at,
                 stop_elapsed, faults, issue, sha256(content).digest(), timing, calibration,
                 board_serial=old["odrive_serial_number"], position_initial=initial, position_writes=writes,
                 position_current_time=current_time, position_feedback=position_feedback)


class Lag(NamedTuple):
    delay: float = 0.0
    response_time: float = 0.0


class Model(NamedTuple):
    rotor: tuple
    actuator: Lag = Lag()
    feedback: Lag = Lag()
    write_delay: float = 0.0
    feedforward_delay: float | None = None


def fit_lag(traces, period, *, continuous=False):
    """Estimate an effective delay and first-order response on the host sampling scale."""
    signals = [(time, np.diff(time), source, measured, max(np.std(measured), np.std(source), 1e-9))
               for time, source, measured in traces if np.ptp(source) > 0]
    if not signals:
        raise ValueError("the logs do not identify actuator or feedback dynamics")
    grid = tuple(period * fraction for fraction in (0, 0.125, 0.25, 0.5, 1))
    response_grid = (*grid, 1.5 * period, 2 * period) if continuous else grid
    best, best_error = Lag(), math.inf
    for delay in grid:
        for response_time in response_grid:
            error = 0.0
            for time, intervals, source, measured, scale in signals:
                delayed = np.interp(time - delay, time, source)
                if response_time and continuous:
                    # Solve the first-order response between the sparse command observations;
                    # applying the current endpoint across the whole interval biases the lag.
                    knots = np.union1d(time, (time + delay)[time + delay < time[-1]])
                    source_at_knots = np.interp(knots - delay, time, source)
                    widths = np.diff(knots)
                    weight = -np.expm1(-widths / response_time)
                    forcing = (source_at_knots[:-1] * weight
                               + np.diff(source_at_knots) * (1 - response_time * weight / widths))
                    filtered = np.empty(len(knots))
                    filtered[0] = measured[0]
                    for i, retention in enumerate(1 - weight, 1):
                        filtered[i] = filtered[i - 1] * retention + forcing[i - 1]
                    delayed = filtered[np.searchsorted(knots, time)]
                elif response_time:
                    decay = np.exp(-intervals / response_time)
                    filtered = delayed.copy()
                    for i, retain in enumerate(decay, 1):
                        filtered[i] += retain * (filtered[i - 1] - delayed[i])
                    delayed = filtered
                # Give each trial comparable weight and keep outlying current samples from dominating.
                residual = np.abs(delayed - measured) / scale
                error += np.mean(np.where(residual <= 2, residual**2, 4 * residual - 4))
            if error < best_error:
                best, best_error = Lag(delay, response_time), error
    return best


def recorder_interval_torque(edges, recorder):
    """Average the measured torque with the same triangular weight as the count acceleration."""
    time, torque_nm = recorder.time, recorder.applied
    impulses = []
    for start, end in zip(edges, edges[1:]):
        lo = np.searchsorted(time, start, side="right")
        hi = np.searchsorted(time, end, side="left")
        knots = np.r_[start, time[lo:hi], end]
        torque = np.interp(knots, time, torque_nm)
        rise = (knots - start) / (end - start)
        widths = np.diff(knots)

        def weighted_area(weight):
            product = torque * weight
            middle = (torque[:-1] + torque[1:]) * (weight[:-1] + weight[1:])
            return np.dot(widths, (product[:-1] + middle + product[1:]) / 6)

        impulses.append((weighted_area(rise), weighted_area(1 - rise)))
    areas = np.asarray(impulses)
    return (areas[:-1, 0] + areas[1:, 1]) / ((edges[2:] - edges[:-2]) / 2)


def fit(trials):
    """Fit rotor and signal responses using only the supplied training captures."""
    blocks, targets, actuator, feedback, write_delays, measured_writes = [], [], [], [], [], []
    position_mode = bool(trials) and effective_mode("auto", trials[0].setup.rpm) == "position"
    for trial in trials:
        time, angle, torque_nm = trial.time, trial.counts / trial.setup.cpr, trial.applied
        if trial.position_writes is not None:
            measured_writes.append(np.median(trial.position_writes - time[:, None], axis=0))
        elif trial.position_timing is not None:
            write_delays.append(float(np.median(trial.position_timing[1])))
        if trial.recorder is not None:
            samples = trial.recorder
            active = (samples.time >= 0) & (samples.time <= time[-1])
            actuator.append((samples.time[active], samples.commanded[active], samples.applied[active]))
        elif trial.torque_set is not None:
            actuator.append((trial.current_sample_times(), trial.torque_set, torque_nm))
        if not position_mode and trial.estimated_rpm is not None:
            # Count deltas are interval averages. Put them at the interval midpoints.
            midpoints = (time[1:] + time[:-1]) / 2
            count_velocity = np.diff(angle) / np.diff(time)
            feedback.append((time, np.interp(time, midpoints, count_velocity), trial.estimated_rpm / 60))
        # Two-tick differencing suppresses the very oscillations seen in the position captures.
        stride = 1 if position_mode else 2
        edges = time[::stride]
        before = np.diff(angle[::stride]) / np.diff(edges)
        if len(before) < 4:
            continue
        centers = (edges[1:] + edges[:-1]) / 2
        intervals = np.diff(centers)
        phase = 2 * np.pi * angle[stride:stride * len(before):stride]
        if trial.recorder is not None:
            center_torque = recorder_interval_torque(edges, trial.recorder)
        elif position_mode:
            if trial.position_timing is not None or trial.position_current_time is not None:
                torque_nm = np.interp(time, trial.current_sample_times(), torque_nm)
            # The second difference of angle observes triangularly weighted acceleration.
            widths = np.diff(edges)
            center_torque = (2 * torque_nm[1:-1] + (widths[:-1] * torque_nm[:-2]
                             + widths[1:] * torque_nm[2:]) / (2 * intervals)) / 3
        else:
            area = np.r_[0.0, np.cumsum((torque_nm[1:] + torque_nm[:-1]) * np.diff(time) / 2)]
            left = np.searchsorted(time, centers, side="right") - 1
            center_area = area[left] + (torque_nm[left] + np.interp(centers, time, torque_nm)) * (centers - time[left]) / 2
            center_torque = np.diff(center_area) / intervals
        mean_speed = (before[1:] + before[:-1]) / 2
        features = np.column_stack((center_torque, -mean_speed, -np.sign(mean_speed),
                                    np.cos(phase), np.sin(phase), np.cos(7 * phase), np.sin(7 * phase)))
        acceleration = np.diff(before) / intervals
        weight = len(features) ** -0.5
        blocks.append(features * weight)
        targets.append(acceleration * weight)
        ramping = centers[1:] <= abs(trial.setup.rpm) / (60 * trial.setup.ramp)
        if np.count_nonzero(ramping) >= 3:
            # Keep the measured ramp impulse identifiable even in long, nearly steady captures.
            blocks.append(np.average(features[ramping], axis=0, weights=intervals[ramping])[None, :])
            targets.append(np.atleast_1d(np.average(acceleration[ramping], weights=intervals[ramping])))
    if not blocks:
        raise ValueError("insufficient samples to identify the rotor model")
    x, y = np.vstack(blocks), np.concatenate(targets)
    # The seventh harmonic can alias into the host samples. Fit the once-per-turn disturbance only.
    active = list(range(5 if position_mode else x.shape[1]))
    coefficients = np.zeros(x.shape[1])
    while True:
        values, _, rank, _ = np.linalg.lstsq(x[:, active], y, rcond=None)
        if rank < len(active) or not np.isfinite(values).all() or values[0] <= 0:
            raise ValueError("the logs do not identify a positive torque response")
        negative_friction = [col for col, value in zip(active, values) if col in (1, 2) and value < 0]
        if not negative_friction:
            coefficients[active] = values
            period = min(trial.setup.period for trial in trials)
            pos_delay, feedforward_delay = (np.median(measured_writes, axis=0) if measured_writes
                                            else (float(np.median(write_delays)) if write_delays else 0.0, None))
            return Model(tuple(coefficients), fit_lag(actuator, period, continuous=position_mode),
                         Lag() if position_mode else fit_lag(feedback, period),
                         pos_delay, feedforward_delay)
        active = [col for col in active if col not in negative_friction]


class SignalResponse:
    def __init__(self, lag):
        self.lag = lag
        self.history = deque([(0.0, 0.0)])
        self.value = 0.0

    def step(self, time, dt, source, *, held=False):
        delay, response_time = self.lag
        if delay:
            history = self.history
            earlier = time - delay
            history.append((time, source))
            while len(history) > 1 and history[1][0] <= earlier:
                history.popleft()
            t0, source = history[0]
            if not held and earlier > t0 and len(history) > 1:
                t1, next_source = history[1]
                source += (next_source - source) * (earlier - t0) / (t1 - t0)
        self.value = (source + (self.value - source) * math.exp(-dt / response_time)
                      if response_time else source)
        return self.value


def encoder_tick(count, position_counts, circular_counts, velocity_counts, cpr, bandwidth):
    dt = FIRMWARE_PERIOD
    position_counts += dt * velocity_counts
    circular_counts += dt * velocity_counts
    phase = (count - math.floor(circular_counts) + cpr / 2) % cpr - cpr / 2
    position_counts += dt * 2 * bandwidth * (count - math.floor(position_counts))
    circular_counts = (circular_counts + dt * 2 * bandwidth * phase) % cpr
    integral = dt * bandwidth**2
    velocity_counts += integral * phase
    if abs(velocity_counts) < 0.5 * integral:
        velocity_counts = 0.0
    return position_counts, circular_counts, velocity_counts


def rotor_step(rotor, angle, velocity, dt, applied):
    response, damping, friction, c1, s1, c7, s7 = rotor
    phase = math.tau * (angle % 1)
    net = (response * applied - damping * velocity + c1 * math.cos(phase) + s1 * math.sin(phase)
           + c7 * math.cos(7 * phase) + s7 * math.sin(7 * phase))
    free = velocity + dt * net
    next_velocity = math.copysign(max(0.0, abs(free) - dt * friction), free)
    return angle + dt * (velocity + next_velocity) / 2, next_velocity


def reconstruct_torque(sample_times, torque_nm, query, *, hold=False):
    if hold:
        index = np.searchsorted(sample_times, query, side="right") - 1
        return torque_nm[np.maximum(0, index)]
    return np.interp(query, sample_times, torque_nm)


def advance_rotor_interval(rotor, angle, velocity, start, dt, samples, *, hold=False):
    steps = math.ceil(dt / FIRMWARE_PERIOD)
    step = dt / steps
    applied = reconstruct_torque(*samples, start + (np.arange(steps) + 0.5) * step, hold=hold)
    for value in applied:
        angle, velocity = rotor_step(rotor, angle, velocity, step, value)
    return angle, velocity


def one_step_rpm(trial, model, samples, count, *, hold=False, velocity_rpm=None):
    """Predict the next count interval from the current logged count and velocity only."""
    velocity_rpm = trial.estimated_rpm if velocity_rpm is None else velocity_rpm
    times = trial.time[:count]
    cpr = trial.setup.cpr
    predictions = []
    for start, dt, start_count, rpm in zip(times, np.diff(times), trial.counts, velocity_rpm):
        angle, _ = advance_rotor_interval(model.rotor, start_count / cpr, rpm / 60, start, dt, samples, hold=hold)
        predictions.append((round(angle * cpr) - start_count) * 60 / (cpr * dt))
    return np.asarray(predictions)


def interval_speed_changes(trial, model, samples, count):
    """Compare changes in adjacent interval-average RPM with one reset per pair."""
    time = trial.time[:count]
    widths = np.diff(time)
    counts = trial.counts[:count]
    cpr = trial.setup.cpr
    observed = np.diff(np.diff(counts) * 60 / (cpr * widths))
    predicted = []
    for i, (first, second) in enumerate(zip(widths, widths[1:])):
        angle, velocity = advance_rotor_interval(model.rotor, counts[i] / cpr, trial.estimated_rpm[i] / 60,
                                                 time[i], first, samples)
        middle_count = round(angle * cpr)
        angle, _ = advance_rotor_interval(model.rotor, angle, velocity, time[i + 1], second, samples)
        left_rpm = (middle_count - counts[i]) * 60 / (cpr * first)
        right_rpm = (round(angle * cpr) - middle_count) * 60 / (cpr * second)
        predicted.append(right_rpm - left_rpm)
    return observed, np.asarray(predicted)


def simulate(setup, model, gains, start_angle, *, duration=None, timestamps=None, references=None, trace=None,
             position_timing=None, telemetry=None, measured_torque=None, initial_velocity_rps=0.0,
             position_initial=None, position_writes=None):
    """Run the closed loop, or replay only the plant using recorded torque samples."""
    rpm, period, ramp, cap, ceiling, trip, cpr, limiter, slew, torque_ramp, guard, bandwidth = setup
    target, direction = abs(rpm) / 60, math.copysign(1, rpm)
    position_mode = effective_mode("auto", rpm) == "position"
    p_gain, second_gain = gains
    actuator, feedback = SignalResponse(model.actuator), SignalResponse(model.feedback)
    if position_initial is None:
        origin, count, reference, active_reference = 0.0, round(start_angle * cpr), start_angle, start_angle
        sensed_angle = count / cpr
    else:
        origin, count, sensed_angle, initial_velocity_rps, active_reference = position_initial
        reference = sensed_angle
    angle = start_angle
    encoder_velocity = initial_velocity_rps
    firmware_pll = position_mode and bandwidth > 0
    linear, pll_velocity = sensed_angle * cpr, initial_velocity_rps * cpr
    circular = linear % cpr
    firmware_tick = 0
    active_feedforward = 0.0
    position_pending, feedforward_pending = deque(), deque()
    first_interval, delays = position_timing if position_timing is not None else (None, ())
    time = origin
    integral = requested = ramped_torque = torque_set = feedforward = 0.0
    velocity = estimate = initial_velocity_rps
    applied = setpoint = 0.0
    stalled = above_soft_since = None
    last_progress = direction * count
    rows = [] if trace is None else trace
    horizon = target / ramp + 7 + 2 * period if duration is None else duration
    if timestamps is None:
        timestamps = (tick * period for tick in range(1, math.ceil(horizon / period) + 1))
    max_step = min(0.005, 0.4 / bandwidth) if bandwidth else 0.005
    for index, (next_time, rpm_reference) in enumerate(zip(
            timestamps, repeat(None) if references is None else references)):
        dt = next_time - time
        if dt <= 0:
            continue
        host_dt = first_interval if index == 0 and first_interval is not None else dt
        if firmware_pll:
            substeps = math.floor((next_time - origin) / FIRMWARE_PERIOD + 1e-9) - firmware_tick
            step = FIRMWARE_PERIOD
        else:
            substeps = max(1, math.ceil(dt / max_step - 1e-9))
            step = dt / substeps
        if measured_torque is not None:
            start = origin + firmware_tick * step if firmware_pll else time
            applied_steps = reconstruct_torque(*measured_torque, start + (np.arange(substeps) + 0.5) * step)
        for i in range(substeps):
            step_start = origin + firmware_tick * step if firmware_pll else time + i * step
            if measured_torque is not None:
                applied = applied_steps[i]
            elif position_mode:
                while position_pending and step_start + 1e-12 >= position_pending[0][0]:
                    _, active_reference = position_pending.popleft()
                while feedforward_pending and step_start + 1e-12 >= feedforward_pending[0][0]:
                    _, active_feedforward = feedforward_pending.popleft()
                desired = clamped(active_feedforward + p_gain * (active_reference - sensed_angle), -ceiling, ceiling)
                torque_set = clamped(second_gain * (desired - estimate), -cap, cap)
            else:
                change = requested - ramped_torque
                if torque_ramp is not None:
                    change = clamped(change, -torque_ramp * step, torque_ramp * step)
                ramped_torque += change
                torque_set = clamped(clamped(ramped_torque, (-ceiling - estimate) * limiter,
                                              (ceiling - estimate) * limiter), -cap, cap)
            if measured_torque is None:
                applied = actuator.step(step_start if position_mode else step_start + step,
                                        step, torque_set, held=position_mode)
            angle, velocity = rotor_step(model.rotor, angle, velocity, step, applied)
            if not math.isfinite(angle) or abs(velocity) > trip:
                return "overspeed", None
            quantized = round(angle * cpr)
            if firmware_pll:
                firmware_tick += 1
                linear, circular, pll_velocity = encoder_tick(quantized, linear, circular, pll_velocity, cpr, bandwidth)
                sensed_angle, encoder_velocity = linear / cpr, pll_velocity / cpr
            elif bandwidth:
                predicted_angle = sensed_angle + step * encoder_velocity
                phase_error = (quantized - math.floor(predicted_angle * cpr)) / cpr
                sensed_angle = predicted_angle + step * 2 * bandwidth * phase_error
                encoder_velocity += step * bandwidth**2 * phase_error
            else:
                sensed_angle, encoder_velocity = quantized / cpr, velocity
            estimate = encoder_velocity if position_mode else feedback.step(step_start + step, step, encoder_velocity)
            if not math.isfinite(estimate) or abs(estimate) > trip:
                return "overspeed", None
        time = next_time
        next_count = round(angle * cpr)
        count_rpm = (next_count - count) * 60 / (cpr * host_dt)
        count = next_count
        if rpm_reference is None:
            setpoint = direction * min(target, abs(setpoint) + ramp * host_dt)
        else:
            setpoint = rpm_reference / 60
        rows.append((time, setpoint * 60, count_rpm))
        if telemetry is not None:
            telemetry.append((torque_set, applied, active_reference, active_feedforward * 60))
        if measured_torque is not None:
            continue
        if guard is not None:
            try:
                above_soft_since = overspeed(time, above_soft_since, estimate * 60, count_rpm, *guard)
            except torque.DiagnosticStop:
                return "overspeed", None
        if position_mode:
            error = reference - sensed_angle
            if abs(error) > position.FOLLOWING_LIMIT_TURNS:
                return "following_error", None
            remaining = max(0.0, position.LEAD_TURNS - direction * error)
            advance = min(abs(setpoint) * host_dt, remaining)
            progress = direction * count
            if advance < remaining:
                stalled = None
            elif stalled is None or progress > last_progress:
                stalled, last_progress = time, progress
            elif time - stalled >= position.NO_PROGRESS_S:
                return "stall", None
            reference += direction * advance
            feedforward = direction * advance / host_dt
            if position_writes is not None and index < len(position_writes):
                position_at, feedforward_at = position_writes[index]
            else:
                delay = delays[index] if index < len(delays) else model.write_delay
                position_at = time + delay
                feedforward_at = (time + model.feedforward_delay
                                  if index >= len(delays) and model.feedforward_delay is not None
                                  else position_at + (FIRMWARE_PERIOD if firmware_pll else 0))
            position_pending.append((position_at, reference))
            feedforward_pending.append((feedforward_at, feedforward))
        else:
            error = setpoint - estimate
            clipped = requested - torque_set
            hold = abs(clipped) > max(0.001, 0.01 * cap) and clipped * error > 0
            available = torque.available_drive_torque(cap, ceiling * 60, limiter, direction * estimate * 60)
            driving = available > 0 and direction * torque_set >= 0.99 * available
            try:
                stalled = stall(time, stalled, setpoint * 60, count_rpm if driving else None, target * 60)
            except torque.DiagnosticStop:
                return "stall", None
            requested, integral = torque.pi_step(error, dt, integral, p_gain, second_gain, cap, hold_integral=hold,
                                                 previous_command=requested, slew_rate=slew)
    return "user_stop", measure(rpm, *np.asarray(rows).T) if rows else None
