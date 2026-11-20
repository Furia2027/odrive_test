"""Tune the two M0 Control modes from CSVs, with optional supervised bench validation."""
import argparse
from collections import Counter, defaultdict
from copy import deepcopy
import csv
from dataclasses import replace
from itertools import combinations
import json
import math
from pathlib import Path
from statistics import median
import sys
import time
from typing import NamedTuple

import numpy as np

from motor import calibrate_m0, control_m0, motor_core, tuning
from motor.live import effective_mode
from motor.paths import LOGS
from motor.vel_m0 import check_faults

from .pso_model import Metrics, Setup, fit, interval_speed_changes, measurement_window, one_step_rpm, read_trial, setup_for, simulate
from .recorder import read_recorder, recorder_paths


SPEEDS = {"position": (1, 10, 30, 100), "torque": (200, 500, 1000)}
HIGH_SPEEDS = (2000, 2200)
BOUNDS = {"position": ((6.0, 10.0), (0.12, 0.30)), "torque": ((0.005, 0.05), (0.0025, 0.03))}
GAIN_FIELDS = {mode: tuple(key for key, _ in tuning.FIELDS[mode][:2]) for mode in SPEEDS}
MODEL_STOPS = {"stall", "overspeed", "following_error"}


def swarm(score, bounds, baseline):
    """Bounded global-best PSO in normalized coordinates."""
    rng = np.random.default_rng(42)
    lower, upper = (np.array(edge) for edge in zip(*bounds))
    span = upper - lower
    particles = rng.random((16, len(span)))
    particles[0] = np.clip((baseline - lower) / span, 0, 1)
    velocity = np.zeros_like(particles)
    personal = particles.copy()
    scores = np.full(len(particles), np.inf)
    for _ in range(30):
        for index, point in enumerate(particles):
            value = score(tuple(lower + span * point))
            if value < scores[index]:
                scores[index], personal[index] = value, point
        best = personal[np.argmin(scores)]
        velocity = (0.6 * velocity + 1.5 * rng.random(particles.shape) * (personal - particles)
                    + 1.5 * rng.random(particles.shape) * (best - particles))
        particles = np.clip(particles + velocity, 0, 1)
    return tuple(lower + span * best) if np.isfinite(scores).any() else None


def recording_order(trial):
    return trial.started_at is not None, trial.started_at, str(trial.path)


def load_trials(mode, cfg, *, logs_dir=None, report=None, check_cancel=None):
    captures, skipped, issues = defaultdict(list), Counter(), []
    for path in sorted((LOGS if logs_dir is None else logs_dir).glob(f"m0_{mode}_*.csv")):
        if check_cancel is not None:
            check_cancel()
        try:
            trial = read_trial(path, mode, cfg)
        except (OSError, KeyError, TypeError, ValueError) as exc:
            reason = str(exc)
            skipped[reason] += 1
            issues.append(f"{path.name}: {reason}")
            continue
        copies = captures[trial.started_at]
        if any(copy.digest == trial.digest for copy in copies):
            skipped["duplicate capture"] += 1
            issues.append(f"{path.name}: duplicate capture")
            continue
        copies.append(trial)
    trials = []
    for copies in captures.values():
        if len(copies) == 1:
            trials.extend(copies)
        else:
            skipped["conflicting capture copies"] += len(copies)
            for copy in copies:
                issues.append(f"{copy.path.name}: conflicting capture copies")
                if copy.drive_fault:
                    trials.append(replace(copy, clean=False, waveform_issue="conflicting capture copies"))
    trials.sort(key=recording_order)
    for trial in trials:
        if trial.waveform_issue:
            issues.append(f"{trial.path.name}: {trial.waveform_issue}")
        elif trial.clean and trial.metrics() is None:
            issues.append(f"{trial.path.name}: incomplete plateau")
        elif not trial.clean and trial.reason not in MODEL_STOPS and not trial.drive_fault:
            issues.append(f"{trial.path.name}: {trial.reason} without a complete clean capture")
    faults = [fault_summary(trial) for trial in trials if trial.drive_fault]
    usable = sum(trial.metrics() is not None for trial in trials)
    print(f"{mode}: {len(trials)} retained records, {usable} usable plateaus, "
          f"{len(faults)} drive faults, {sum(skipped.values())} skipped copies/files")
    if report is not None:
        report({"compatible": len(trials), "usable": usable, "skipped": dict(skipped),
                "recording_issues": issues, "faults": faults})
    else:
        for line in (*issues, *faults):
            print(f"  {line}")
    return trials


def describe_setup(setup):
    return (f"{setup.rpm:+g} RPM, {setup.period * 1000:g} ms, cap {setup.cap:g} Nm, "
            f"ramp {setup.ramp * 60:g} RPM/s, ceiling/trip {setup.ceiling * 60:g}/{setup.trip * 60:g} RPM, "
            f"CPR {setup.cpr}, bandwidth {setup.encoder_bandwidth:g}, limiter {setup.limiter:g}, "
            f"slew {setup.slew:g}, torque ramp {setup.torque_ramp}, software guard {setup.software_guard}")


def fault_summary(trial):
    faults = trial.faults
    kind = "M0 current-limit fault" if faults.current_limit else "drive fault"
    sources = [f"sample {elapsed}: {masks}" for elapsed, masks in faults.samples]
    if any(faults.stop.values()):
        sources.append(f"stop: {faults.stop}")
    sources.extend(f"post-stop {axis}: {masks}" for axis, masks in faults.post_stop.items() if any(masks.values()))
    at = f" at {trial.stop_elapsed:.3g} s" if trial.stop_elapsed is not None else ""
    return f"{trial.path.name} · {trial.setup.rpm:+g} RPM · {kind}{at} · " + " · ".join(sources)


def split_trials(trials):
    groups = defaultdict(list)
    training, held_out = [], []
    for trial in sorted(trials, key=recording_order):
        if trial.metrics() is not None:
            groups[trial.setup, trial.gains].append(trial)
        elif trial.reason in MODEL_STOPS and not (trial.faults and trial.faults.current_limit):
            held_out.append(trial)
    for group in groups.values():
        middle = len(group) // 2
        training.extend(group[:middle])
        held_out.extend(group[middle:])
    return training, held_out


def recorded_trials(trials, recorder_dir, purpose, *, check_cancel=None):
    matched = []
    for trial in trials:
        if check_cancel is not None:
            check_cancel()
        try:
            recording = read_recorder(recorder_dir, trial)
        except (csv.Error, OSError, KeyError, TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"{trial.path.name}: invalid synchronized recording: {exc}") from exc
        if recording is not None:
            matched.append(replace(trial, recorder=recording))
    print(f"  Synchronized recordings: {len(matched)}/{len(trials)} matched {purpose} trials")
    return matched


def position_recorder_dir(mode, *, logs_dir=None):
    directory = (LOGS if logs_dir is None else logs_dir) / "position_recorder"
    return directory if mode == "position" and directory.is_dir() else None


def speed_diagnostics(time, measured, predicted, period, settled, end):
    uniform = time[0] + np.arange(1 + int((end - time[0]) / period)) * period
    residual = np.interp(uniform, time, predicted - measured)
    average = np.ones(round(0.25 / period))
    trend_rmse = np.sqrt(np.mean(np.convolve(residual, average / average.size, mode="valid")**2))
    plateau = settled + np.arange(round((end - settled) / period)) * period
    window = np.hanning(len(plateau))
    frequencies = np.fft.rfftfreq(len(plateau), period)
    first = np.searchsorted(frequencies, 1.0)
    peaks = []
    for speed in (measured, predicted):
        signal = np.interp(plateau, time, speed)
        amplitudes = 2 * np.abs(np.fft.rfft((signal - signal.mean()) * window)) / window.sum()
        peak = first + np.argmax(amplitudes[first:])
        peaks.append((frequencies[peak], amplitudes[peak]))
    return float(trend_rmse), peaks


def rotor_replay(trial, model, times, references, trace, *, recorder=None):
    if recorder is None:
        current_time = np.r_[0.0, trial.current_sample_times()]
        if not (np.diff(current_time) > 0).all():
            return "invalid current sample timing"
        inputs = current_time, np.r_[0.0, trial.applied]
    else:
        inputs = recorder.time, recorder.applied
    reason, _ = simulate(trial.setup, model, trial.gains, trial.initial_angle(), timestamps=times,
                         references=references, trace=trace, measured_torque=inputs, **trial.position_inputs())
    return reason


def firmware_torque_prediction(trial):
    if trial.position_feedback is None or trial.estimated_rpm is None or trial.torque_set is None:
        return None
    active_pos, active_rpm, position, cap = trial.position_feedback.T
    kp, velocity_p = trial.gains
    ceiling = trial.setup.ceiling
    desired = np.clip(active_rpm / 60 + kp * (active_pos - position), -ceiling, ceiling)
    return np.clip(velocity_p * (desired - trial.estimated_rpm / 60), -cap, cap)


def replay(trial, model, *, diagnostics_dir=None, rotor_recording=None):
    observed = trial.metrics()
    if trial.waveform_issue or len(trial.time) == 0:
        return False, "usable replay data unavailable"
    rpm = trial.setup.rpm
    trace, telemetry, rotor_trace = [], [], []
    if observed is not None:
        _, settled, end = measurement_window(rpm, trial.time, trial.reference)
        count = np.searchsorted(trial.time, end, side="right")
        times, references = trial.time[:count + 2], trial.reference[:count + 2]
    else:
        if trial.stop_elapsed is None:
            return False, "recorded stop timing unavailable"
        times, references = trial.time, trial.reference
        dt = trial.stop_elapsed - times[-1]
        if dt > 0:
            target = min(abs(rpm), abs(references[-1]) + trial.setup.ramp * 60 * dt)
            times = np.r_[times, trial.stop_elapsed]
            references = np.r_[references, math.copysign(target, rpm)]
    reason, predicted = simulate(trial.setup, model, trial.gains, trial.initial_angle(),
                                 timestamps=times, references=references, trace=trace,
                                 telemetry=telemetry, **trial.position_inputs())
    position = effective_mode("auto", rpm) == "position"
    detail, rotor_ok, firmware = "", True, None
    if position:
        timing = "recorded write timing" if trial.position_initial is not None else "legacy approximate timing"
        detail = f" · {timing}"
        if observed is not None:
            rotor_count = np.searchsorted(times, end, side="left") + 1
            rotor_reason = rotor_replay(trial, model, times[:rotor_count], references[:rotor_count], rotor_trace,
                                        recorder=rotor_recording)
            rotor_ok = rotor_reason == "user_stop" and len(rotor_trace) == rotor_count
            source = "synchronized" if rotor_recording is not None else "native USB"
            common = min(rotor_count, len(rotor_trace))
            detail += f" · measured-current rotor ({source}): {rotor_reason}, {common}/{rotor_count} samples"
            if common:
                error = rmse(np.asarray(rotor_trace[:common])[:, 2], trial.speed[:common])
                detail += f", {error:.3g} RPM RMSE"
            if not rotor_ok:
                detail += " FAIL"
            firmware = firmware_torque_prediction(trial)
            detail += (" · firmware torque from logged feedback: unavailable" if firmware is None else
                       f" · firmware torque from logged feedback: {rmse(firmware[:count], trial.torque_set[:count]):.3g} Nm RMSE")
    if diagnostics_dir is not None:
        filename = diagnostics_dir / f"replay-{trial.path.stem}.csv"
        with filename.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("elapsed_s", "reference_rpm", "measured_rpm", "predicted_rpm",
                             "recorded_torque_command_nm", "predicted_torque_command_nm",
                             "recorded_applied_torque_nm", "predicted_applied_torque_nm",
                             *(("measured_current_replay_rpm", "torque_from_logged_feedback_nm") if position else ())))
            for i, (timestamp, reference) in enumerate(zip(times[:len(trial.time)], references)):
                command, applied = telemetry[i][:2] if i < len(telemetry) else (None, None)
                extra = (rotor_trace[i][2] if i < len(rotor_trace) else None,
                         firmware[i] if firmware is not None else None) if position else ()
                writer.writerow((timestamp, reference, trial.speed[i], trace[i][2] if i < len(trace) else None,
                                 trial.torque_set[i] if trial.torque_set is not None else None,
                                 command, trial.applied[i], applied, *extra))
    if observed is None or reason != "user_stop":
        expected = trial.reason if observed is None else "user_stop"
        return reason == expected and rotor_ok, f"observed {expected}, model {reason}" + detail
    if predicted is None or not trace:
        return False, "model did not complete the measurement window" + detail
    simulated_time, _, speed = np.asarray(trace).T
    if not np.array_equal(simulated_time, times):
        return False, "model did not cover the recorded timestamps" + detail
    trace_rmse = float(np.sqrt(np.mean((speed[:count] - trial.speed[:count])**2)))
    trace_limit = max(1.0, 60 / (trial.setup.cpr * trial.setup.period), 0.20 * abs(rpm))
    checks = [("RPM RMSE", trace_rmse, trace_limit)]
    checks.extend((name, abs(actual - estimate), max(1.0, 0.20 * actual))
                  for name, actual, estimate in zip(Metrics._fields, observed, predicted))
    if position:
        trend, ((actual_hz, actual_amp), (predicted_hz, predicted_amp)) = speed_diagnostics(
            times, trial.speed[:len(times)], speed, trial.setup.period, settled, end)
        checks.extend((("trend RMSE", trend, trace_limit),
                       ("oscillation amplitude error (RPM)", abs(actual_amp - predicted_amp),
                        max(1.0, 0.20 * actual_amp))))
        if min(actual_amp, predicted_amp) >= 1.0:
            checks.append(("oscillation frequency error (Hz)", abs(actual_hz - predicted_hz),
                           max(1 / (end - settled), 0.20 * actual_hz)))
        detail += (f" · oscillation measured/model {actual_hz:.3g}/{predicted_hz:.3g} Hz, "
                   f"amplitude {actual_amp:.3g}/{predicted_amp:.3g} RPM")
        if trial.torque_set is not None and len(telemetry) == len(times):
            plateau = (times >= settled) & (times < end)
            predicted_torque, predicted_applied = np.asarray(telemetry)[plateau, :2].T
            actual_torque, actual_applied = trial.torque_set[:len(times)][plateau], trial.applied[:len(times)][plateau]
            saturation = [np.mean(np.abs(values) >= 0.99 * trial.setup.cap)
                          for values in (actual_torque, predicted_torque)]
            rms = [np.sqrt(np.mean(values**2)) for values in (actual_torque, predicted_torque,
                                                              actual_applied, predicted_applied)]
            detail += (f" · torque at cap {saturation[0]:.0%}/{saturation[1]:.0%}, "
                       f"command RMS {rms[0]:.3g}/{rms[1]:.3g} Nm, applied RMS {rms[2]:.3g}/{rms[3]:.3g} Nm")
    pairs = ", ".join(f"{name} {error:.3g}/{limit:.3g}{' FAIL' if error > limit else ''}"
                      for name, error, limit in checks)
    return rotor_ok and all(error <= limit for _, error, limit in checks), pairs + detail


def rmse(predicted, observed):
    return float(np.sqrt(np.mean((predicted - observed)**2)))


def phase_masks(time, start, arrival, settled, end):
    startup_end = start + 1.0
    return {
        "startup": (time >= start) & (time < startup_end) & (time <= end),
        "acceleration ramp": (time >= startup_end) & (time < arrival),
        "settling": (time >= max(startup_end, arrival)) & (time < settled),
        "steady": (time >= max(startup_end, settled)) & (time <= end),
    }


def reconstruction_spread(predictions, mask):
    return {(left, right): rmse(a[mask], b[mask])
            for (left, a), (right, b) in combinations(predictions.items(), 2)}


class PhaseAudit(NamedTuple):
    intervals: int
    step_rmse: float
    step_bias: float
    persistence_rmse: float
    changes: int
    change_rmse: float | None
    change_bias: float | None
    shared: int
    shared_rmse: tuple | None
    spread: dict


def report_phase(label, audits):
    changes = [audit for audit in audits if audit.changes]
    shared = [audit.shared_rmse for audit in audits if audit.shared]
    print(f"    {label} · {sum(audit.intervals for audit in audits)} intervals in {len(audits)} runs · "
          f"one-step RMSE {median(audit.step_rmse for audit in audits):.2f}, "
          f"bias {median(audit.step_bias for audit in audits):+.2f}, "
          f"persistence RMSE {median(audit.persistence_rmse for audit in audits):.2f} RPM")
    if changes:
        print(f"      adjacent speed change · {sum(audit.changes for audit in changes)} pairs · "
              f"RMSE {median(audit.change_rmse for audit in changes):.2f}, "
              f"bias {median(audit.change_bias for audit in changes):+.2f} RPM")
    if shared:
        free, step = map(median, zip(*shared))
        print(f"      matched free-running/one-step RMSE {free:.2f}/{step:.2f} RPM · "
              f"{sum(audit.shared for audit in audits)} intervals in {len(shared)} runs")
    else:
        print("      matched free-running comparison unavailable · replay stopped before this phase")
    comparisons = [audit.spread for audit in audits if audit.spread]
    if comparisons:
        pairs = {pair: [result[pair] for result in comparisons] for pair in comparisons[0]}
        pair, values = max(pairs.items(), key=lambda item: (sum(value > 6 for value in item[1]), median(item[1])))
        hits = sum(value > 6 for value in values)
        status = "incomplete evidence"
        if 2 * hits >= len(audits):
            status = "under-resolved"
        elif len(values) == len(audits):
            status = "below the under-resolution threshold"
        print(f"      reconstruction spread · {pair[0]} vs {pair[1]} · median {median(values):.2f} RPM · "
              f"{hits}/{len(audits)} runs above 6 RPM ({len(values)} evaluated) · {status}")
    else:
        print("      reconstruction spread unavailable · need all six reconstructions")


def report_rotor_fit(training, model):
    refits = []
    for i, omitted in enumerate(training):
        try:
            refits.append(fit(training[:i] + training[i + 1:]).rotor)
        except (ValueError, np.linalg.LinAlgError) as exc:
            print(f"  Training refit without {omitted.path.name} failed · {exc}")
    base = model.rotor
    print(f"  Rotor training fit · {len(refits)}/{len(training)} leave-one-out fits succeeded")
    quantities = (
        ("effective inertia (kg·m²)", lambda rotor: 1 / (math.tau * rotor[0])),
        ("Coulomb friction (Nm)", lambda rotor: rotor[2] / rotor[0]),
        ("torque response (turn/s² per Nm)", lambda rotor: rotor[0]),
        ("viscous damping (1/s)", lambda rotor: rotor[1]),
        ("once-per-turn cosine (turn/s²)", lambda rotor: rotor[3]),
        ("once-per-turn sine (turn/s²)", lambda rotor: rotor[4]),
    )
    for label, value in quantities:
        values = [value(rotor) for rotor in refits]
        detail = f" · leave-one-out range {min(values):.5g} to {max(values):.5g}" if values else ""
        print(f"    {label} {value(base):.5g}{detail}")
    if refits:
        zeros = (sum(rotor[i] == 0 for rotor in refits) for i in (1, 2))
        damping, friction = zeros
        print(f"    Zero in refits · damping {damping}/{len(refits)}, friction {friction}/{len(refits)}")

    onset = defaultdict(list)
    for trial in training:
        rpm = trial.setup.rpm
        if abs(rpm) != 30:
            continue
        direction = math.copysign(1, rpm)
        forward = direction * np.diff(trial.counts) > 0
        first = np.flatnonzero(forward[:-2] & forward[1:-1] & forward[2:])
        if len(first):
            time = trial.time[first[0] + 1]
            if trial.recorder is None:
                times, applied = trial.current_sample_times(), trial.applied
            else:
                times, applied = trial.recorder.time, trial.recorder.applied
            split = np.searchsorted(times, time, side="right")
            if 0 < split < len(applied):
                onset[rpm].append(direction * applied[split - 1:split + 1])
    for rpm in (30, -30):
        if onset[rpm]:
            before, after = map(median, zip(*onset[rpm]))
            print(f"    {rpm:+g} RPM motion onset · {len(onset[rpm])} training runs · target-direction torque "
                  f"before/after {before:+.5f}/{after:+.5f} Nm (first of three forward count intervals)")
    print("    Mechanical plausibility unverified · no independent inertia or friction measurement")


def one_step_comparison(trial, model, count, arrival, settled, end, free_trace, summary, phases):
    rpm = trial.setup.rpm
    time = trial.time[:count]
    intervals = np.diff(time)
    observed = trial.speed[1:count]
    reconstructions = {}
    if trial.recorder is not None:
        current = trial.recorder.applied
        sources = [("synchronized current", trial.recorder.time)]
    else:
        current = np.r_[0.0, trial.applied]
        if trial.position_current_time is not None:
            sources = [("recorded Iq read", np.r_[0.0, trial.current_sample_times()])]
        else:
            sources = [(label, np.r_[0.0, trial.current_sample_times(fraction)]) for label, fraction in
                       (("estimated Iq read", 4 / 9), ("row timestamp", 0), ("end of USB reads", 1))]
    nominal_source, nominal_time = sources[0]
    for label, times in sources:
        if not (np.diff(times) > 0).all():
            continue
        for hold in (False, True):
            predicted = one_step_rpm(trial, model, (times, current), count, hold=hold)
            name = f"{label}, {'hold' if hold else 'linear'}"
            reconstructions[name] = predicted
            summary[rpm, name].append(rmse(predicted, observed))
    nominal = reconstructions.get(f"{nominal_source}, linear")
    if nominal is None:
        return
    samples = nominal_time, current
    cpr = trial.setup.cpr
    counts = trial.counts[:count - 1]
    initial_rpm = trial.estimated_rpm[:count - 1]
    persisted_counts = np.rint(counts + initial_rpm * intervals * cpr / 60) - counts
    persistence = persisted_counts * 60 / (cpr * intervals)
    count_seed = one_step_rpm(trial, model, samples, count, velocity_rpm=trial.speed)
    summary[rpm, "persistence with firmware RPM"].append(rmse(persistence, observed))
    summary[rpm, "one-step with previous count RPM"].append(rmse(count_seed, observed))
    anchored = []
    result, _ = simulate(trial.setup, model, trial.gains, trial.counts[0] / cpr,
                         timestamps=time[1:] - time[0], references=trial.reference[1:count], trace=anchored,
                         measured_torque=(samples[0] - time[0], samples[1]), initial_velocity_rps=initial_rpm[0] / 60)
    common = min(len(free_trace) - 1, len(anchored), len(observed))
    if common > 0:
        for label, trace in (("free-running from trial start", np.asarray(free_trace[1:common + 1])[:, 2]),
                             ("free-running from first row", np.asarray(anchored[:common])[:, 2]),
                             ("one-step", nominal[:common]), ("persistence", persistence[:common])):
            summary[rpm, f"matched early {label}"].append(rmse(trace, observed[:common]))
    print(f"    Recorded first-row state · matched intervals {max(0, common)}/{len(observed)} · "
          f"free replay from that row {result} after {len(anchored)} intervals")
    observed_changes, predicted_changes = interval_speed_changes(trial, model, samples, count)
    direction = math.copysign(1, rpm)
    midpoint_phases = phase_masks(time[:-1] + intervals / 2, time[0], arrival, settled, end)
    change_phases = phase_masks(time[1:-1], time[0], arrival, settled, end)
    shared = min(len(anchored), len(observed))
    for phase, mask in midpoint_phases.items():
        if not mask.any():
            continue
        selected = mask[:shared]
        shared_rmse = None
        if selected.any():
            free = np.asarray(anchored[:shared])[:, 2]
            shared_rmse = tuple(rmse(prediction[:shared][selected], observed[:shared][selected])
                                for prediction in (free, nominal))
        changes = change_phases[phase]
        error = predicted_changes[changes] - observed_changes[changes]
        phases[rpm, phase].append(PhaseAudit(
            np.count_nonzero(mask), rmse(nominal[mask], observed[mask]),
            direction * float(np.mean(nominal[mask] - observed[mask])), rmse(persistence[mask], observed[mask]),
            len(error), rmse(predicted_changes[changes], observed_changes[changes]) if len(error) else None,
            direction * float(np.mean(error)) if len(error) else None, np.count_nonzero(selected), shared_rmse,
            reconstruction_spread(reconstructions, mask),
        ))


def measured_current_replay(trials, *, recorder_dir=None):
    if recorder_dir is not None:
        trials = [trial for trial in trials if trial.metrics() is not None
                  and any(path.exists() for path in recorder_paths(recorder_dir, trial))]
    training, held_out = split_trials(trials)
    targets = [trial for trial in held_out if abs(trial.setup.rpm) == 30 and trial.metrics() is not None]
    if len(training) < 2 or not targets:
        raise ValueError("need at least two training runs and a clean +/-30 RPM holdout")
    if recorder_dir is not None:
        training = recorded_trials(training, recorder_dir, "training")
        if len(training) < 2:
            raise ValueError("need at least two matched synchronized training recordings")
    model = fit(training)
    if recorder_dir is not None:
        targets = recorded_trials(targets, recorder_dir, "clean +/-30 RPM holdout")
        if not targets:
            raise ValueError("need a matched synchronized +/-30 RPM holdout for measured-current replay")
    summary = defaultdict(list)
    step_summary = defaultdict(list)
    phases = defaultdict(list)
    attempted = Counter(trial.setup.rpm for trial in targets)
    print(f"Measured-current diagnostic only · {len(training)} training runs · {len(targets)} clean holdouts")
    calibration = [trial.torque_calibration for trial in targets if trial.torque_calibration is not None]
    if calibration:
        scales = ", ".join(f"{scale:+.8g}" for scale in sorted({scale for scale, _ in calibration}))
        count_error = max(np.max(np.abs(np.diff(trial.counts) * 60 / (trial.setup.cpr * np.diff(trial.time))
                                       - trial.speed[1:])) for trial in targets)
        print(f"  CSV audit · signed torque scale {scales} Nm/A · max command residual "
              f"{max(error for _, error in calibration):.3g} Nm · max count RPM residual {count_error:.3g} RPM")
    offsets = [np.median(trial.current_sample_times() - trial.time) for trial in targets]
    initial = [trial.estimated_rpm[0] for trial in targets if trial.estimated_rpm is not None]
    if initial:
        print(f"  Initial firmware RPM {min(initial):+.3g} to {max(initial):+.3g} · "
              f"median native USB Iq timestamp offset {1000 * median(offsets):.3f} ms")
    report_rotor_fit(training, model)
    for trial in targets:
        rpm = trial.setup.rpm
        arrival, settled, end = measurement_window(rpm, trial.time, trial.reference)
        count = np.searchsorted(trial.time, end, side="right")
        times, references = trial.time[:count + 2], trial.reference[:count + 2]
        current_times = trial.current_sample_times()
        prefix = f"  {trial.path.name} · {rpm:+g} RPM"
        if trial.recorder is None and not (np.diff(current_times) > 0).all():
            print(f"{prefix} · current timestamps out of order")
            continue
        closed_trace, forced_trace = [], []
        closed, _ = simulate(trial.setup, model, trial.gains, trial.initial_angle(), timestamps=times,
                             references=references, trace=closed_trace, **trial.position_inputs())
        forced = rotor_replay(trial, model, times, references, forced_trace, recorder=trial.recorder)
        common = min(count, len(closed_trace), len(forced_trace))
        if common == 0:
            print(f"{prefix} · no shared samples · closed loop {closed}, measured current {forced}")
        else:
            errors = tuple(rmse(np.asarray(trace[:common])[:, 2], trial.speed[:common])
                           for trace in (closed_trace, forced_trace))
            comparison = f"raw RMSE {errors[0]:.2f} → {errors[1]:.2f} RPM (closed loop → measured current)"
            if common == count and closed == forced == "user_stop":
                summary[rpm].append(errors)
                print(f"{prefix} · {comparison}")
            else:
                print(f"{prefix} · {comparison} over {common}/{count} samples through {times[common - 1]:.3f} s · "
                      f"closed loop {closed}, measured current {forced}")
        if trial.estimated_rpm is not None:
            one_step_comparison(trial, model, count, arrival, settled, end, forced_trace, step_summary, phases)
        else:
            print("    One-step comparison unavailable · missing recorded firmware RPM")
    for rpm in (30, -30):
        comparisons = summary[rpm]
        print(f"  {rpm:+g} RPM · {len(comparisons)}/{attempted[rpm]} complete comparisons", end="")
        if comparisons:
            closed, forced = map(median, zip(*comparisons))
            print(f" · median raw RMSE {closed:.2f} → {forced:.2f} RPM")
        else:
            print()
        for (target, label), values in step_summary.items():
            if target == rpm:
                print(f"    {label} · median RMSE {median(values):.2f} RPM ({len(values)} runs)")
        for (target, label), audits in phases.items():
            if target == rpm:
                report_phase(label, audits)


def validate_model(trials, *, report=None, check_cancel=None, diagnostics_dir=None, recorder_dir=None):
    position = bool(trials) and effective_mode("auto", trials[0].setup.rpm) == "position"
    cohort, coverage = trials, []
    if position and recorder_dir is not None:
        cohort = [trial for trial in trials if trial.metrics() is None or (
            trial.position_initial is not None and any(path.exists() for path in recorder_paths(recorder_dir, trial))
        )]
        coverage.append(f"Position qualification uses timestamped synchronized captures in {recorder_dir} · "
                        f"{len(trials) - len(cohort)} other clean captures excluded")
    elif position:
        coverage.append("Position replay is diagnostic only. Qualification requires timestamped synchronized captures.")
    if coverage:
        if report is not None:
            report({"coverage": coverage})
        else:
            for line in coverage:
                print(f"  {line}")
    training, held_out = split_trials(cohort)
    if report is not None:
        report({"training": len(training), "held_out": len(held_out)})
    if not held_out or len(training) < 2:
        raise ValueError("need more independent fixed-gain runs for training and holdout")
    if len({trial.gains for trial in (*training, *held_out)}) < 2:
        raise ValueError("need reference runs at a second gain pair")
    if recorder_dir is not None:
        training = recorded_trials(training, recorder_dir, "training", check_cancel=check_cancel)
        if len(training) < 2:
            raise ValueError("need at least two matched synchronized training recordings")
        if position and len({trial.gains for trial in training}) < 2:
            raise ValueError("need synchronized training captures at two position gain pairs")
    model = fit(training)
    passing, failed_setups, failures, errors = defaultdict(list), Counter(), [], []
    rotor_holdouts = {}
    if position and recorder_dir is not None:
        rotor_holdouts = {trial.path: trial.recorder for trial in recorded_trials(
            [trial for trial in held_out if trial.metrics() is not None], recorder_dir, "rotor holdout",
            check_cancel=check_cancel)}
    for trial in held_out:
        if check_cancel is not None:
            check_cancel()
        ok, detail = replay(trial, model, diagnostics_dir=diagnostics_dir, rotor_recording=rotor_holdouts.get(trial.path))
        message = f"{trial.path.name} · {trial.setup.rpm:+g} RPM · {detail}"
        if ok:
            errors.append(message)
            if trial.metrics() is not None and (not position or trial.path in rotor_holdouts):
                passing[trial.setup].append(trial)
        else:
            failures.append(message)
            failed_setups[trial.setup] += 1
    evidence = {setup: repeats for setup, repeats in passing.items()
                if len(repeats) >= 2 and setup not in failed_setups and setup.software_guard is None}
    coverage = [f"{describe_setup(setup)} · {len(passing.get(setup, ()))} passing clean holdouts "
                f"(2 required) · {failed_setups[setup]} failed checks"
                for setup in dict.fromkeys(trial.setup for trial in held_out) if setup.software_guard is None]
    if report is not None:
        report({"failures": failures, "prediction_errors": errors, "coverage": coverage})
    else:
        print("  Holdouts (metric error / limit, RPM unless indicated):")
        for message in (*errors, *failures, *coverage):
            print(f"  {message}")
    print(f"  {len(training)} training runs, {len(held_out)} held out, {len(failures)} failed holdout checks")
    return model, evidence, failures


def current_limit_blocks(trials, baseline):
    groups = defaultdict(list)
    for trial in trials:
        groups[trial.setup].append(trial)
    blocked = {}
    for setup, group in groups.items():
        faults = [trial for trial in group if trial.faults and trial.faults.current_limit]
        if not faults:
            continue
        if any(trial.stopped_at is None for trial in faults):
            blocked[setup] = "current-limit fault with unverified stop time"
            continue
        latest = max(trial.stopped_at for trial in faults)
        repeats = {trial.started_at for trial in group
                   if trial.gains == baseline and trial.started_at is not None and trial.started_at > latest
                   and trial.metrics() is not None}
        if len(repeats) < 2:
            blocked[setup] = f"current-limit fault on {latest.isoformat()} · {len(repeats)}/2 later clean baseline captures"
    return blocked


def excessive_error(before, after):
    return after > before + max(1.0, 0.05 * before)


def signed(speeds):
    return [rpm for magnitude in speeds for rpm in (magnitude, -magnitude)]


def validation_speeds(mode):
    return signed(SPEEDS[mode] + (HIGH_SPEEDS if mode == "torque" else ()))


class Selection(NamedTuple):
    gains: tuple
    score: float
    setups: tuple[Setup, ...]
    full: bool


def optimize(mode, trials, baseline, *, report=None, check_cancel=None, diagnostics_dir=None, recorder_dir=None):
    targets = validation_speeds(mode)
    captured = {trial.setup.rpm for trial in trials
                if trial.metrics() is not None and trial.setup.software_guard is None}
    missing = [rpm for rpm in targets if rpm not in captured]
    blocks = current_limit_blocks(trials, baseline)
    if report is not None:
        report({"missing": missing, "blocked": [f"{describe_setup(s)} · {why}" for s, why in blocks.items()]})
    else:
        if missing:
            print("  Complete plateau recordings missing at " + ", ".join(f"{rpm:+g}" for rpm in missing) + " RPM.")
        for setup, why in blocks.items():
            print(f"  Blocked {describe_setup(setup)} · {why}")
    try:
        model, evidence, failures = validate_model(trials, report=report, check_cancel=check_cancel,
                                                   diagnostics_dir=diagnostics_dir, recorder_dir=recorder_dir)
    except ValueError:
        if report is not None:
            report({"unqualified": [rpm for rpm in targets if rpm in captured]})
        raise

    qualified, excluded = {}, []
    for setup, repeats in evidence.items():
        if setup.rpm not in targets or setup in blocks:
            continue
        comparable = [trial for trial in trials if trial.setup == setup and trial.gains == baseline
                      and trial.metrics() is not None]
        if comparable:
            qualified[setup] = repeats, max(comparable, key=recording_order).metrics()
        else:
            excluded.append(f"{describe_setup(setup)} · no complete comparable saved-gain baseline")
    scenarios = {}
    provisional = failures or {setup.rpm for setup in qualified} != set(targets)
    for setup, (repeats, measured_baseline) in list(qualified.items()):
        if mode == "torque" and provisional and abs(setup.rpm) not in SPEEDS[mode]:
            continue
        trial = max(repeats, key=recording_order)
        angle = trial.initial_angle() if mode == "position" else trial.counts[0] / setup.cpr
        for phase in (angle, angle + 0.5):
            reason, before = simulate(setup, model, baseline, phase)
            if reason != "user_stop" or before is None:
                if mode == "position":
                    excluded.append(f"{describe_setup(setup)} · saved-gain simulation failed at "
                                    f"{phase:g} turns ({reason if reason != 'user_stop' else 'incomplete window'})")
                    del qualified[setup]
                    break
                before = measured_baseline
            scenarios.setdefault(setup, []).append((phase, before))
    qualified_rpms = {setup.rpm for setup in qualified}
    unqualified = [rpm for rpm in targets if rpm in captured and rpm not in qualified_rpms]
    full = not failures and qualified_rpms == set(targets)
    setups = tuple(setup for rpm in targets for setup in qualified if setup.rpm == rpm
                   and (full or abs(rpm) in SPEEDS[mode]))
    if not full:
        excluded.extend(f"{describe_setup(setup)} · outside low and moderate provisional scope"
                        for setup in qualified if abs(setup.rpm) not in SPEEDS[mode])
    scope = [describe_setup(setup) for setup in setups]
    if report is not None:
        report({"unqualified": unqualified, "excluded": excluded, "scope": scope, "provisional": not full})
    else:
        if unqualified:
            print("  Recorded speeds that failed qualification: " + ", ".join(f"{rpm:+g}" for rpm in unqualified))
        for line in excluded:
            print(f"  Excluded {line}")
    if not setups:
        raise ValueError("the offline model did not qualify for any requested tuning setup")

    comparisons = [(setup, phase, before) for setup in setups for phase, before in scenarios[setup]]

    def evaluate(gains):
        if check_cancel is not None:
            check_cancel()
        total = 0.0
        results = []
        for setup, phase, baseline_metrics in comparisons:
            reason, predicted = simulate(setup, model, gains, phase)
            if reason != "user_stop" or predicted is None:
                return math.inf
            checked = zip(baseline_metrics, predicted) if mode == "position" else zip(
                baseline_metrics[2:], predicted[2:])
            if any(excessive_error(before, after) for before, after in checked):
                return math.inf
            results.append((setup.rpm, baseline_metrics, predicted))
            total += sum(predicted) / (len(predicted) * abs(setup.rpm))
        if mode == "position" and not compare(results):
            return math.inf
        return total / len(comparisons)

    scores = {}

    def score(gains):
        if gains not in scores:
            scores[gains] = evaluate(gains)
        return scores[gains]

    swarm(score, BOUNDS[mode], np.asarray(baseline))
    baseline_score = sum(sum(before) / (len(before) * abs(setup.rpm))
                         for setup, _, before in comparisons) / len(comparisons)
    ranked = sorted((value, tuple(map(float, gains))) for gains, value in scores.items() if value < baseline_score)[:3]
    if not ranked:
        raise ValueError("PSO found no qualifying improvement over the current gains")
    if report is not None:
        report({"ranked": [{"gains": gains, "score": value} for value, gains in ranked], "provisional": not full})
    else:
        print("  Full-range offline scope:" if full else "  Provisional offline-only scope:")
        for line in scope:
            print(f"  {line}")
        for index, (value, gains) in enumerate(ranked, 1):
            print(f"  Rank {index}: {gains} · predicted score {value:.5g}")
    best_score, candidate = ranked[0]
    return Selection(candidate, best_score, setups, full)


def compare(results):
    if not results or any(excessive_error(previous, final)
                          for _, before, after in results for previous, final in zip(before, after)):
        return False
    return all(median(after[i] for _, _, after in results)
               <= 0.9 * median(before[i] for _, before, _ in results) for i in (0, 1))


def make_draft(mode, revision, gains):
    return {"revision": revision, "edits": dict(zip(GAIN_FIELDS[mode], gains, strict=True))}


def reference_pair(mode, rpm, gains):
    if (type(rpm) not in (float, int) or not math.isfinite(rpm)
            or effective_mode("auto", rpm) != mode or not 0 < abs(rpm) <= max(SPEEDS[mode])):
        raise ValueError("Reference RPM must use the selected mode and stay within its initial validation speeds.")
    return bounded_pair(mode, gains)


def bounded_pair(mode, gains):
    if (not isinstance(gains, (tuple, list)) or len(gains) != 2
            or any(type(value) not in (float, int) or not lo <= value <= hi
                   for value, (lo, hi) in zip(gains, BOUNDS[mode]))):
        raise ValueError(f"Gain pair must be inside {BOUNDS[mode]}.")
    return tuple(gains)


def trial_duration(mode, rpm, cfg):
    velocity = cfg["velocity"]
    ramp = velocity["fast_vel_ramp_rate"] if (
        mode == "torque" and abs(rpm) > velocity["fast_ramp_above_rpm"]) else velocity["vel_ramp_rate"]
    return math.ceil(abs(rpm) / (60 * ramp) + 8)


def settled_drive(cfg, check_cancel):
    with motor_core.exclusive_motion():
        check_cancel()
        dev = motor_core.find_odrive(cfg)
        motor_core.verify_board(dev, cfg)
        motor_core.ensure_idle(dev, cfg)
        check_faults(motor_core.fault_codes(dev.axis1, sensorless=True), "ODrive M1 reported a fault")
        calibrate_m0.wait_for_settling(dev.axis0, cfg, check_cancel=check_cancel)
        motor_core.ensure_idle(dev, cfg)
        check_faults(motor_core.fault_codes(dev.axis1, sensorless=True), "ODrive M1 reported a fault")
        return dev


def run_trial(mode, rpm, draft, *, approve=None, check_cancel=None, emit=None, expected_setup=None):
    cfg = tuning.draft_config(0, mode, draft)
    events = {}
    duration = trial_duration(mode, rpm, cfg)
    deadline = None
    timed_stop = False
    dev = settled_drive(cfg, check_cancel) if approve is not None else None

    def notify(event, data):
        nonlocal deadline
        if event == "running":
            deadline = time.monotonic() + duration
        events[event] = data
        if emit is not None:
            emit(event, data)

    def check_capture():
        nonlocal timed_stop
        check_cancel()
        if deadline is not None and time.monotonic() >= deadline:
            timed_stop = True
            raise KeyboardInterrupt("Timed PSO capture complete")

    def confirm(description):
        gains = ", ".join(f"{key} {value:g}" for key, value in draft["edits"].items())
        if approve is None:
            print(f"Trial gains {gains}")
            print(f"Capture at least {duration} s, then stop with Ctrl+C.")
            motor_core.confirm_motion(description)
        else:
            approve(f"{description}\nGains {gains} · Timed capture {duration} s")
        # Recheck the revision at the last confirmation before the runner arms.
        tuning.draft_config(0, mode, draft)

    def check_setup(period, cap):
        if setup_for(cfg, rpm, period, cap) != expected_setup:
            raise RuntimeError(f"{rpm:+g} RPM admitted a different control period or torque cap than the qualified model")

    control_m0.run(rpm, cfg, dev=dev, capture=True, confirm=confirm, notify=notify,
                   check_cancel=check_capture if approve is not None else None, show_readings=emit is None,
                   check_setup=check_setup if expected_setup is not None else None)
    if check_cancel is not None:
        check_cancel()
    result = events.get("trial_result")
    if not result or result["reason"] != "user_stop" or result["idle"] is not True or (
            approve is not None and not timed_stop):
        raise RuntimeError(f"trial stopped without a clean IDLE result: {result}")
    if approve is not None:
        settled_drive(cfg, check_cancel)
    try:
        trial = read_trial(Path(events["csv"]), mode, cfg)
    except (OSError, KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"Incomplete or invalid PSO capture: {exc}") from exc
    if (trial.setup.rpm != rpm or (expected_setup is not None and trial.setup != expected_setup)
            or trial.gains != tuple(draft["edits"].values())
            or trial.metrics() is None):
        raise RuntimeError(f"trial lacks a clean full measurement window: {trial.path}")
    print(f"  IDLE confirmed. {trial.path.name}: {trial.metrics()}")
    return trial


def bench(mode, revision, baseline, selection, history, *, capture=None, guard_stage=None, report=None):
    capture = capture or run_trial
    results = []
    targets = validation_speeds(mode)
    if not selection.full or {setup.rpm for setup in selection.setups} != set(targets):
        raise RuntimeError("bench validation requires a qualified trial setup at every signed target")
    blocks = current_limit_blocks(history, baseline)
    baselines = {}
    for trial in sorted(history, key=recording_order):
        if (trial.gains == baseline and trial.setup in selection.setups and trial.setup not in blocks
                and trial.metrics() is not None):
            baselines[trial.setup.rpm] = trial
    for rpm in targets:
        if rpm not in baselines:
            raise RuntimeError(f"{rpm:+g} RPM lacks a complete saved-gain baseline for a qualified setup")

    for rpm in targets:
        baseline_trial = baselines[rpm]
        if mode == "torque" and abs(rpm) in HIGH_SPEEDS:
            if not compare(results):
                raise RuntimeError(f"lower stages or guarded speed confirmation did not qualify for {rpm} RPM")
            if guard_stage is not None:
                guard_stage(rpm)
            else:
                trip = baseline_trial.setup.trip * 60
                if input(f"Target {rpm:+g} RPM, firmware trip ±{trip:g} RPM. "
                         "Verify the guarded bench rating covers the trip, or arm an independent "
                         "speed cutoff within the bench rating. Enter the target to confirm: ").strip() != str(rpm):
                    raise RuntimeError(f"guarded speed confirmation did not qualify for {rpm} RPM")
        before = baseline_trial.metrics()
        for attempt in (1, 2):
            trial = capture(mode, rpm, make_draft(mode, revision, selection.gains),
                            expected_setup=baseline_trial.setup)
            after = trial.metrics()
            if trial.setup != baseline_trial.setup or after is None:
                raise RuntimeError(f"{rpm:+g} RPM finalist did not produce a clean capture under the qualified setup")
            if any(excessive_error(a, b) for a, b in zip(before, after)):
                raise RuntimeError(f"{rpm:+g} RPM exceeded the allowed regression: {before} -> {after}")
            results.append((rpm, before, after))
            if report is not None:
                report({"rpm": rpm, "attempt": attempt, "candidate": after, "csv": str(trial.path),
                        "baseline": before, "baseline_csv": str(baseline_trial.path)})
    if not compare(results):
        raise RuntimeError("the bench results did not meet both 10% improvement thresholds")
    return results


def apply_result(mode, revision, candidate):
    before = json.loads(motor_core.CONFIG_PATH.read_bytes())
    expected = deepcopy(before)
    m0 = expected["axes"]["m0"] if "axes" in expected else expected
    draft = make_draft(mode, revision, candidate)
    for field, value in draft["edits"].items():
        group, name = field.split(".")
        m0[group][name] = value
    snapshot = tuning.save(0, mode, draft)
    if json.loads(motor_core.CONFIG_PATH.read_bytes()) != expected:
        raise RuntimeError("config reload differs from the verified gain-only edit")
    return snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=tuple(SPEEDS))
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--bench", action="store_true", help="test the offline finalist and ask before saving")
    action.add_argument("--reference", type=float, metavar="RPM", help="confirm and record one fixed-target reference")
    action.add_argument("--diagnostics-dir", type=Path, metavar="DIR", help="export aligned offline holdout traces")
    action.add_argument("--measured-current-replay", action="store_true", help="diagnose +/-30 RPM position holdouts offline")
    parser.add_argument("--gains", type=float, nargs=2, metavar=("P", "SECOND_GAIN"),
                        help="gain pair for a reference run, in table order. Defaults to the current pair")
    parser.add_argument("--logs-dir", type=Path, help="directory of archived CSVs for offline analysis")
    parser.add_argument("--recorder-dir", type=Path,
                        help="synchronized position recorder exports (default: position_recorder inside the logs directory)")
    args = parser.parse_args()
    mode = args.mode
    if args.measured_current_replay and mode != "position":
        parser.error("--measured-current-replay requires position mode")
    if args.gains is not None and args.reference is None:
        parser.error("--gains requires --reference")
    if args.logs_dir is not None and (args.bench or args.reference is not None or not args.logs_dir.is_dir()):
        parser.error("--logs-dir requires an existing directory and offline analysis")
    if args.recorder_dir is not None and (mode != "position" or args.reference is not None
                                         or not args.recorder_dir.is_dir()):
        parser.error("--recorder-dir requires an existing directory and position analysis or bench validation")
    if (args.bench or args.reference is not None) and not sys.stdin.isatty():
        parser.error("supervised trials require an interactive operator terminal")
    try:
        snapshot = tuning.snapshot(0, mode)
        revision = snapshot["revision"]
        gains = dict((key, value) for key, _, value in snapshot["fields"])
        baseline = tuple(gains[key] for key in GAIN_FIELDS[mode])
        cfg = tuning.draft_config(0, mode, make_draft(mode, revision, baseline))
        print(f"Current {mode} gains: {make_draft(mode, revision, baseline)['edits']}")
        if args.reference is not None:
            rpm = args.reference
            reference = reference_pair(mode, rpm, args.gains if args.gains is not None else baseline)
            run_trial(mode, rpm, make_draft(mode, revision, reference))
        else:
            trials = load_trials(mode, cfg, logs_dir=args.logs_dir)
            if args.measured_current_replay:
                measured_current_replay(trials, recorder_dir=args.recorder_dir)
                return 0
            recorder_dir = args.recorder_dir or position_recorder_dir(mode, logs_dir=args.logs_dir)
            if args.diagnostics_dir is not None:
                args.diagnostics_dir.mkdir(parents=True, exist_ok=True)
            selection = optimize(mode, trials, baseline, diagnostics_dir=args.diagnostics_dir, recorder_dir=recorder_dir)
            candidate = selection.gains
            label = "Offline finalist" if selection.full else "Provisional offline-only pair"
            print(f"{label}: {make_draft(mode, revision, candidate)['edits']}, score {selection.score:.5f}")
            if args.bench:
                if not selection.full:
                    raise ValueError("Provisional results are offline only. Bench validation and saving require full qualification.")
                results = bench(mode, revision, baseline, selection, trials)
                for rpm, before, after in results:
                    print(f"  {rpm:+g} RPM  {before} -> {after}")
                if input(f"Validated {mode} gains {baseline} → {candidate}. Type save to apply: ").strip() == "save":
                    apply_result(mode, revision, candidate)
                    print(f"{mode} gains saved after supervised validation.")
                else:
                    print("Validated gains discarded.")
        return 0
    except KeyboardInterrupt:
        print("Tuning interrupted.", file=sys.stderr)
        return 130
    except (EOFError, OSError, RuntimeError, ValueError) as exc:
        print(f"Tuning stopped: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
