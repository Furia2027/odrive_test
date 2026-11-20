from contextlib import nullcontext, redirect_stdout
import csv
from dataclasses import replace
from datetime import datetime, timedelta, timezone
from hashlib import sha256
import io
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from motor import control_m0, m0_display, motor_core, tuning
from odrive.enums import MOTOR_ERROR_CURRENT_LIMIT_VIOLATION as CURRENT_LIMIT
from pso import pso_m0
from pso.pso_m0 import Selection, apply_result, bench, compare, swarm
from pso.pso_model import (FIRMWARE_PERIOD, Faults, Lag, Metrics, Model, Setup, Trial, encoder_tick,
                           fit_lag, interval_speed_changes, measure, one_step_rpm, position_timing, read_trial, read_waveform,
                           reconstruct_torque, recorder_interval_torque, setup_for, simulate)
from pso.recorder import CHANNELS, RecorderTrace, read_recorder
from ui import worker


def example(setup, gains, metrics=Metrics(10, 10, 3, 1), *, number=0, faults=None):
    started_at = datetime(2026, 11, 19, tzinfo=timezone.utc) + timedelta(seconds=10 * number)
    return SimpleNamespace(setup=setup, gains=gains, counts=[0], faults=faults,
                           started_at=started_at, stopped_at=started_at + timedelta(seconds=8),
                           path=Path(f"trial-{number}.csv"), metrics=lambda: metrics,
                           initial_angle=lambda: 0.0,
                           reason="user_stop" if metrics is not None else "fault")


class PsoTest(unittest.TestCase):
    def test_early_fault_retains_raw_masks_and_duplicate_capture_is_counted_once(self):
        cfg = motor_core.load_config()
        setup = setup_for(cfg, -500, 0.025, 1.0)
        metadata = {
            "control_period_s": setup.period, "torque_limit_nm": setup.cap, "ramp_rpm_per_s": setup.ramp * 60,
            "firmware_limit_rpm": setup.ceiling * 60, "firmware_overspeed_rpm": setup.trip * 60,
            "firmware_limiter_gain": setup.limiter, "slew_rate_nm_per_s": setup.slew,
            "input_mode": "PASSTHROUGH", "kp": 0.03, "ki": 0.03, "torque_constant_nm_per_a": 0.059,
        }
        start = {"event": "start", "utc_time": "2026-11-19T01:00:00+00:00", "axis": "0",
                 "target_rpm": -500, "control_mode": "TORQUE", "config_json": json.dumps(
                     {"config": cfg, "trial": metadata})}
        sample = {**start, "event": "sample", "elapsed_s": 0.025, "motor_error": CURRENT_LIMIT | 16}
        stop = {**start, "event": "stop", "utc_time": "2026-11-19T01:00:00.041+00:00", "elapsed_s": 0.041,
                "stop_reason": "fault", "motor_error": 64,
                "post_stop_faults_json": json.dumps({"m0": {"motor_error": CURRENT_LIMIT},
                                                     "m1": {"motor_error": 128}})}
        with TemporaryDirectory() as folder:
            directory = Path(folder)
            path = directory / "m0_torque_original.csv"
            with path.open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=list(stop))
                writer.writeheader()
                writer.writerows([start, sample, stop])
            trial = read_trial(path, "torque", cfg)
            self.assertEqual(trial.setup, setup)
            self.assertEqual(trial.stop_elapsed, 0.041)
            self.assertEqual(trial.faults.samples, (("0.025", {"motor_error": CURRENT_LIMIT | 16}),))
            self.assertEqual(trial.faults.stop["motor_error"], 64)
            self.assertEqual(trial.faults.post_stop["m1"]["motor_error"], 128)
            self.assertTrue(trial.faults.current_limit)
            self.assertFalse(trial.faults._replace(samples=(), post_stop={"m1": {"motor_error": CURRENT_LIMIT}}).current_limit)
            self.assertTrue(trial.faults._replace(samples=()).current_limit)
            self.assertEqual(trial.waveform_issue, "too few waveform samples")
            self.assertIsNone(trial.metrics())
            (directory / "m0_torque_copy.csv").write_bytes(path.read_bytes())
            report = Mock()
            with redirect_stdout(io.StringIO()):
                records = pso_m0.load_trials("torque", cfg, logs_dir=directory, report=report)
            self.assertEqual(len(records), 1)
            inventory = report.call_args.args[0]
            self.assertEqual(inventory["skipped"], {"duplicate capture": 1})
            self.assertIn("M0 current-limit fault", inventory["faults"][0])

            clean_copy = {**stop, "motor_error": 0, "post_stop_faults_json": "{}"}
            with path.open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=list(stop))
                writer.writeheader()
                writer.writerows([start, {**sample, "motor_error": 0}, clean_copy])
            with redirect_stdout(io.StringIO()):
                records = pso_m0.load_trials("torque", cfg, logs_dir=directory, report=report)
            self.assertEqual(len(records), 1)
            self.assertTrue(records[0].faults.current_limit)
            self.assertIsNone(records[0].metrics())

    def test_current_limit_recovery_uses_two_later_distinct_saved_pair_captures(self):
        cfg = motor_core.load_config()
        setup = setup_for(cfg, -500, 0.025, 1.0)
        baseline = (0.03, 0.03)
        fault = example(setup, (0.02, 0.02), None, number=3, faults=Faults((), {"motor_error": CURRENT_LIMIT}, {}, False))
        clean = example(setup, baseline, number=4)
        records = [fault, clean, clean, example(setup, baseline, number=2),
                   example(setup._replace(rpm=500), baseline, number=5),
                   example(setup._replace(period=1 / 30), baseline, number=5),
                   example(setup, (0.02, 0.02), number=5)]
        blocks = pso_m0.current_limit_blocks(records, baseline)
        self.assertEqual(list(blocks), [setup])
        self.assertIn("1/2 later clean baseline captures", blocks[setup])
        records.append(example(setup, baseline, number=6))
        self.assertEqual(pso_m0.current_limit_blocks(records, baseline), {})
        fault.stopped_at = None
        self.assertIn("unverified stop time", pso_m0.current_limit_blocks(records, baseline)[setup])

    def test_position_csv_accepts_legacy_and_uses_independent_native_timestamps(self):
        cfg = motor_core.load_config()
        setup = setup_for(cfg, 30, 0.025, 0.08)
        metadata = {
            "control_period_s": setup.period, "torque_limit_nm": setup.cap, "ramp_rpm_per_s": setup.ramp * 60,
            "firmware_limit_rpm": setup.ceiling * 60, "firmware_overspeed_rpm": setup.trip * 60,
            "vel_integrator_gain": 0, "max_lead_turns": control_m0.LEAD_TURNS,
            "following_limit_turns": control_m0.FOLLOWING_LIMIT_TURNS, "no_progress_s": control_m0.NO_PROGRESS_S,
            "input_mode": "PASSTHROUGH", "position_gain": 8, "vel_gain": 0.2, "torque_constant_nm_per_a": 0.06,
        }
        errors = dict.fromkeys(m0_display.ERROR_KEYS, 0)
        start = {**errors, "event": "start", "utc_time": "2026-11-19T01:00:00+00:00", "axis": "0",
                 "target_rpm": 30, "control_mode": "POSITION",
                 "config_json": json.dumps({"config": cfg, "trial": metadata})}
        stop = {**start, "event": "stop", "utc_time": "2026-11-19T01:00:00.065+00:00", "elapsed_s": 0.065,
                "stop_reason": "user_stop", "idle_confirmed": "True",
                "post_stop_faults_json": json.dumps({"m0": errors, "m1": errors})}
        rows = [{**start, "event": "sample", "elapsed_s": t, "setpoint_rpm": 30,
                 "encoder_count_rpm": 30, "encoder_shadow_count": count, "firmware_torque_setpoint_nm": 0.03,
                 "iq_setpoint_a": 0.5, "iq_measured_a": 0.4, "loop_interval_s": 0.025, "read_span_s": 0.004}
                for t, count in ((0.025, 25), (0.05, 50))]
        with TemporaryDirectory() as folder:
            path = Path(folder) / "m0_position.csv"
            for kind in ("legacy", "timestamped", "invalid"):
                if kind == "timestamped":
                    rows[0].update(zip(control_m0.POSITION_INITIAL_FIELDS, (0, 0, 0.001, 0.02, 0.002), strict=True))
                    for row in rows:
                        t = row["elapsed_s"]
                        row["iq_measured_elapsed_s"] = t + 0.001
                        row.update(zip(control_m0.POSITION_WRITE_FIELDS,
                                       (t + 0.002, t + 0.003, t + 0.003, t + 0.005), strict=True))
                elif kind == "invalid":
                    rows[0]["iq_measured_elapsed_s"] = 0.048
                with path.open("w", newline="") as file:
                    writer = csv.DictWriter(file, fieldnames=list(dict.fromkeys((*stop, *rows[0]))))
                    writer.writeheader()
                    writer.writerows([start, *rows, stop])
                trial = read_trial(path, "position", cfg)
                if kind == "legacy":
                    self.assertTrue(trial.clean, trial.waveform_issue)
                    self.assertIsNone(trial.position_initial)
                    self.assertIsNotNone(trial.position_timing)
                elif kind == "timestamped":
                    self.assertTrue(trial.clean, trial.waveform_issue)
                    self.assertEqual(trial.position_initial, (0, 0, 0.001, 0.02, 0.002))
                    np.testing.assert_allclose(trial.position_writes, [[0.0275, 0.029], [0.0525, 0.054]])
                    np.testing.assert_allclose(trial.current_sample_times(), [0.026, 0.051])
                else:
                    self.assertFalse(trial.clean)
                    self.assertIn("invalid position capture timestamps", trial.waveform_issue)

    def test_split_preserves_repeated_holdouts_across_exact_setups_and_gain_pairs(self):
        setup = setup_for(motor_core.load_config(), -500, 0.025, 1.0)
        pair = (0.03, 0.03)
        older, newer, newest = [example(setup, pair, number=i) for i in (1, 2, 3)]
        different_pair = example(setup, (0.02, 0.02), number=4)
        different_rate = example(setup._replace(period=1 / 30), pair, number=5)
        training, held_out = pso_m0.split_trials([newest, different_rate, older, different_pair, newer])
        self.assertEqual(training, [older])
        self.assertEqual(held_out, [newer, newest, different_pair, different_rate])

    def test_fault_replay_uses_recorded_stop_time_instead_of_an_extra_period(self):
        setup = setup_for(motor_core.load_config(), 30, 0.025, 0.08)
        times = np.array([0.025, 0.05])
        trial = Trial(Path("stall.csv"), setup, (8.0, 0.15), times, times * 6, np.zeros(2), np.zeros(2),
                      np.zeros(2), "stall", False, stop_elapsed=0.0523)
        with patch.object(pso_m0, "simulate", return_value=("stall", None)) as replay:
            self.assertTrue(pso_m0.replay(trial, None)[0])
        np.testing.assert_array_equal(replay.call_args.kwargs["timestamps"], [0.025, 0.05, 0.0523])

    def test_timed_trial_starts_at_running_and_stop_wins_at_deadline(self):
        cfg = motor_core.load_config()
        draft = pso_m0.make_draft("position", "revision", (8.0, 0.15))
        duration = pso_m0.trial_duration("position", -30, cfg)
        for stop_at in (None, "deadline", "after cleanup"):
            with self.subTest(stop_at=stop_at):
                clock, stopped = 0, False
                trial = SimpleNamespace(path=Path("trial.csv"), setup=SimpleNamespace(rpm=-30),
                                        gains=(8.0, 0.15), metrics=lambda: Metrics(8, 8, 3, 1))

                def check_stop():
                    if stopped:
                        raise KeyboardInterrupt("Stopped by operator")

                def run(_rpm, _cfg, *, confirm, notify, check_cancel, **_kwargs):
                    nonlocal clock, stopped
                    confirm("M0 -30 RPM · ramp · firmware trip")
                    clock = 100
                    check_cancel()
                    notify("csv", "trial.csv")
                    notify("running", None)
                    clock += duration - 0.1
                    check_cancel()
                    clock += 0.1
                    stopped = stop_at == "deadline"
                    with self.assertRaisesRegex(KeyboardInterrupt, "Stopped by operator" if stopped else "Timed PSO"):
                        check_cancel()
                    notify("trial_result", {"reason": "user_stop", "idle": True})
                    stopped = stopped or stop_at == "after cleanup"

                with (patch.object(tuning, "draft_config", return_value=cfg),
                      patch.object(pso_m0, "settled_drive") as settle,
                      patch.object(pso_m0.control_m0, "run", side_effect=run),
                      patch.object(pso_m0.time, "monotonic", side_effect=lambda: clock),
                      patch.object(pso_m0, "read_trial", return_value=trial) as read):
                    approve = Mock()
                    if stop_at is None:
                        self.assertIs(pso_m0.run_trial("position", -30, draft, approve=approve,
                                                       check_cancel=check_stop, emit=Mock()), trial)
                        self.assertEqual(settle.call_count, 2)
                    else:
                        with self.assertRaisesRegex(KeyboardInterrupt, "Stopped by operator"):
                            pso_m0.run_trial("position", -30, draft, approve=approve,
                                             check_cancel=check_stop, emit=Mock())
                        read.assert_not_called()
                    self.assertIn(f"Timed capture {duration} s", approve.call_args.args[0])

    def test_settling_failure_prevents_motion_prompt(self):
        cfg = motor_core.load_config()
        draft = pso_m0.make_draft("position", "revision", (8.0, 0.15))
        approve = Mock()
        with (patch.object(tuning, "draft_config", return_value=cfg),
              patch.object(motor_core, "exclusive_motion", return_value=nullcontext()),
              patch.object(motor_core, "find_odrive"),
              patch.object(motor_core, "verify_board"),
              patch.object(motor_core, "ensure_idle") as idle,
              patch.object(motor_core, "fault_codes", return_value=(0,) * 5),
              patch.object(pso_m0.calibrate_m0, "wait_for_settling", side_effect=TimeoutError("did not settle")),
              patch.object(pso_m0.control_m0, "run") as run):
            with self.assertRaisesRegex(TimeoutError, "did not settle"):
                pso_m0.run_trial("position", 30, draft, approve=approve, check_cancel=Mock())
        idle.assert_called_once()
        run.assert_not_called()
        approve.assert_not_called()

    def test_torque_guards_follow_two_clean_finalists_per_signed_target(self):
        baseline, candidate = (0.05, 0.03), (0.02, 0.01)
        speeds = pso_m0.signed(pso_m0.SPEEDS["torque"] + pso_m0.HIGH_SPEEDS)
        setups = tuple(setup_for(motor_core.load_config(), rpm, 0.025, 1.0) for rpm in speeds)
        history = [example(setup, baseline) for setup in setups]
        sequence, reports = [], []

        def capture(_mode, rpm, draft, *, expected_setup):
            self.assertEqual(tuple(draft["edits"].values()), candidate)
            self.assertEqual(rpm, expected_setup.rpm)
            sequence.append(rpm)
            return SimpleNamespace(setup=expected_setup, path=Path("final.csv"),
                                   metrics=lambda: Metrics(8, 8, 3, 1))

        bench("torque", "revision", baseline, Selection(candidate, 0.05, setups, True), history, capture=capture,
              guard_stage=lambda rpm: sequence.append(f"guard {rpm}"), report=reports.append)
        expected = []
        for rpm in speeds:
            if abs(rpm) in pso_m0.HIGH_SPEEDS:
                expected.append(f"guard {rpm}")
            expected.extend([rpm, rpm])
        self.assertEqual(sequence, expected)
        self.assertEqual(len(reports), 2 * len(speeds))
        self.assertTrue(all(row["baseline_csv"] == "trial-0.csv" for row in reports))

    def test_reverse_setup_matches_forward_firmware_envelope(self):
        cfg = motor_core.load_config()
        for rpm, cap in ((30, 0.08), (200, 1.0), (1000, 1.0), (2200, 1.0)):
            with self.subTest(rpm=rpm):
                forward = setup_for(cfg, rpm, 0.025, cap)
                self.assertEqual(setup_for(cfg, -rpm, 0.025, cap), forward._replace(rpm=-rpm))

    def test_saved_position_pair_is_inside_reference_bounds(self):
        self.assertEqual(pso_m0.reference_pair("position", -30, (8.0, 0.30)), (8.0, 0.30))

    def test_one_qualified_direction_produces_offline_only_result(self):
        baseline = (0.03, 0.03)
        setup = setup_for(motor_core.load_config(), -500, 0.025, 1.0)
        history = [example(setup, baseline, number=i) for i in range(2)]
        report = Mock()

        def simulated(_setup, _model, gains, _phase):
            return "user_stop", Metrics(10, 10, 3, 1) if gains == baseline else Metrics(8, 8, 2, 1)

        with (patch.object(pso_m0, "validate_model", return_value=(None, {setup: history}, [])),
              patch.object(pso_m0, "simulate", side_effect=simulated)):
            selection = pso_m0.optimize("torque", history, baseline, report=report)
        self.assertFalse(selection.full)
        self.assertEqual(selection.setups, (setup,))
        self.assertIn(500, report.call_args_list[0].args[0]["missing"])
        self.assertNotIn(-500, report.call_args_list[0].args[0]["missing"])
        capture = Mock()
        with self.assertRaisesRegex(RuntimeError, "every signed target"):
            bench("torque", "revision", baseline, selection, history, capture=capture)
        capture.assert_not_called()

    def test_position_pso_rejects_tracking_improvements_that_fail_bench_criteria(self):
        baseline = (8.0, 0.30)
        recorded = Metrics(2, 3, 4, 4)
        setup = setup_for(motor_core.load_config(), 30, 0.025, 0.08)
        trial = example(setup, baseline, recorded)
        with (patch.object(pso_m0, "validate_model", return_value=(None, {setup: [trial]}, [])),
              patch.object(pso_m0, "simulate", side_effect=lambda _, __, gains, ___:
                           ("user_stop", recorded if gains == baseline else Metrics(2, 3, 1, 1)))):
            with self.assertRaisesRegex(ValueError, "no qualifying improvement"):
                pso_m0.optimize("position", [trial], baseline)

    def test_pso_ranks_bench_eligible_pairs_by_tracking_metrics(self):
        baseline = (8.0, 0.30)
        recorded = Metrics(2, 3, 4, 4)
        trials = [example(Setup(rpm, 0.025, 0.1, 0.08, 0.83, 1.67, 2000, 0.4, 1.0, None), baseline, recorded)
                  for rpm in (30, -30)]
        evidence = {trial.setup: [trial] for trial in trials}
        report = Mock()

        def simulated(_setup, _model, gains, _phase):
            if gains == baseline:
                return "user_stop", recorded
            return "user_stop", Metrics(1.7, 2.6, 1 + (gains[0] - 7)**2,
                                         1 + 100 * (gains[1] - 0.16)**2)

        with (patch.dict(pso_m0.SPEEDS, position=(30,)), patch.object(pso_m0, "simulate", side_effect=simulated),
              patch.object(pso_m0, "validate_model", return_value=(None, evidence, []))):
            selection = pso_m0.optimize("position", trials, baseline, report=report)
        self.assertTrue(selection.full)
        gains, score = selection.gains, selection.score
        np.testing.assert_allclose(gains, (7, 0.16), atol=0.01)
        ranked = report.call_args.args[0]["ranked"]
        self.assertEqual(len(ranked), 3)
        self.assertEqual(ranked[0], {"gains": gains, "score": score})
        self.assertEqual([pair["score"] for pair in ranked], sorted(pair["score"] for pair in ranked))

    def test_bench_requires_matching_saved_gain_baseline_before_any_capture(self):
        baseline, candidate = (8.0, 0.15), (7.5, 0.16)
        cfg = motor_core.load_config()
        positive = setup_for(cfg, 30, 0.025, 0.08)
        negative = positive._replace(rpm=-30)
        selection = Selection(candidate, 0.05, (positive, negative), True)
        history = [example(positive, baseline), example(negative._replace(period=1 / 30), baseline),
                   example(negative, candidate)]
        capture = Mock()
        with patch.dict(pso_m0.SPEEDS, position=(30,)):
            with self.assertRaisesRegex(RuntimeError, "-30 RPM lacks a complete saved-gain baseline"):
                bench("position", "revision", baseline, selection, history, capture=capture)
        capture.assert_not_called()

    def test_offline_baseline_failure_cannot_use_another_gain_pairs_metrics(self):
        baseline = (8.0, 0.15)
        setup = setup_for(motor_core.load_config(), 30, 0.025, 0.08)
        heldout = example(setup, (7.5, 0.16), Metrics(100, 100, 10, 10))
        saved = example(setup, baseline, Metrics(2, 2, 1, 1))

        def simulated(_setup, _model, gains, _phase):
            return ("stall", None) if gains == baseline else ("user_stop", Metrics(3, 3, 1, 1))

        with (patch.object(pso_m0, "validate_model", return_value=(None, {setup: [heldout]}, [])),
              patch.object(pso_m0, "simulate", side_effect=simulated),
              patch.dict(pso_m0.SPEEDS, position=(30,)), redirect_stdout(io.StringIO()) as output):
            with self.assertRaisesRegex(ValueError, "did not qualify"):
                pso_m0.optimize("position", [heldout], baseline)
            with self.assertRaisesRegex(ValueError, "did not qualify"):
                pso_m0.optimize("position", [heldout, saved], baseline)
        self.assertIn("saved-gain simulation failed", output.getvalue())

    def test_fixed_window_measures_signed_speed_and_excludes_late_samples(self):
        t = np.arange(0.0, 10.01, 0.05)
        reference = -np.minimum(30.0, 30.0 * t)
        speed = reference.copy()
        speed[(t >= 3) & (t < 8)] += np.tile([-2.0, 2.0], 50)
        speed[t >= 8.5] = -70.0
        result = measure(-30, t, reference, speed)
        self.assertAlmostEqual(result.ripple, 2.0)
        self.assertAlmostEqual(result.overshoot, 2.0)
        self.assertAlmostEqual(result.ramp_error, 0.0)
        self.assertAlmostEqual(result.plateau_error, 0.0)
        self.assertIsNone(measure(-30, t[:150], reference[:150], speed[:150]))

    def test_swarm_is_seeded_bounded_and_skips_rejected_particles(self):
        def score(pair):
            p, i = pair
            return math.inf if p < 0.45 else (p - 0.7)**2 + (i - 0.2)**2

        limits = ((0.0, 1.0), (0.0, 1.0))
        found = swarm(score, limits, (0.5, 0.5))
        self.assertEqual(found, swarm(score, limits, (0.5, 0.5)))
        self.assertLess(score(found), 0.0001)
        self.assertIsNone(swarm(lambda _: math.inf, limits, (0.5, 0.5)))

    def test_bench_rejects_a_tracking_regression_despite_less_ripple(self):
        baseline = Metrics(10.0, 20.0, 3.0, 1.0)
        improved = Metrics(8.0, 16.0, 3.0, 1.0)
        lagging = Metrics(8.0, 16.0, 5.0, 1.0)
        self.assertTrue(compare([(100, baseline, improved)]))
        self.assertFalse(compare([(100, baseline, lagging)]))
        self.assertFalse(compare([]))

    def test_signed_model_and_overspeed_gate(self):
        model = Model((200.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0))
        positive = Setup(30.0, 0.025, 0.1, 0.08, 0.83, 1.67, 2000, 0.4, 1.0, None)
        negative = positive._replace(rpm=-30.0)
        self.assertEqual(simulate(positive, model, (8.0, 0.15), 0.5),
                         simulate(negative, model, (8.0, 0.15), -0.5))
        self.assertEqual(simulate(positive._replace(trip=0.1), model, (8.0, 0.15), 0.5),
                         ("overspeed", None))
        self.assertEqual(simulate(negative._replace(software_guard=(10, 15)), model, (8.0, 0.15), -0.5),
                         ("overspeed", None))

    def test_identifies_bounded_delay_and_response_from_irregular_samples(self):
        time = np.cumsum(np.tile([0.024, 0.026], 40))
        input_signal = np.sin(17 * time) + np.cos(45 * time)
        delay, response_time = 0.0125, 0.00625
        shifted = np.interp(time - delay, time, input_signal)
        measured = shifted.copy()
        for i in range(1, len(time)):
            decay = math.exp(-(time[i] - time[i - 1]) / response_time)
            measured[i] = decay * measured[i - 1] + (1 - decay) * shifted[i]
        self.assertEqual(fit_lag([(time, input_signal, measured)], 0.025), Lag(delay, response_time))

    def test_identifies_continuous_actuator_response_between_csv_samples(self):
        time = np.r_[0, np.cumsum(np.tile([0.024, 0.026], 20))]
        delay, response_time = 0.00625, 0.0125
        shifted = np.maximum(0, time - delay)
        measured = shifted + response_time * np.expm1(-shifted / response_time)
        self.assertEqual(fit_lag([(time, time, measured)], 0.025, continuous=True), Lag(delay, response_time))

    def test_encoder_pll_wraps_circular_phase_and_snaps_to_zero(self):
        linear, circular, speed = encoder_tick(2000, 1999.5, 1999.5, 0, 2000, 150)
        self.assertAlmostEqual(speed, FIRMWARE_PERIOD * 150**2)
        self.assertAlmostEqual(linear, circular)
        linear, circular, speed = encoder_tick(2000, 2000, 0, 0.2, 2000, 150)
        self.assertAlmostEqual(linear - circular, 2000)
        self.assertEqual(speed, 0)

    def test_position_host_writes_follow_feedback_and_hold_between_ticks(self):
        setup = Setup(100, 0.025, 1000, 0.08, 2, 10, 2000, 0.4, 1, None, encoder_bandwidth=150)
        samples = [{"loop_interval_s": "0.021", "read_span_s": "0.008"} for _ in range(3)]
        times = np.array([0.031, 0.055, 0.081])
        first, delays = position_timing(samples, times, setup.period)
        np.testing.assert_allclose(delays, [0.002, 0.003, 0.002], atol=1e-12)
        rotor = (0, 0, 0, 0, 0, 0, 0)
        trace, telemetry = [], []
        reason, _ = simulate(setup, Model(rotor, actuator=Lag(delay=0.025)), (10, 0.3), 0,
                             timestamps=times, references=[100] * 3, trace=trace,
                             position_timing=(first, delays), telemetry=telemetry)
        self.assertEqual(reason, "user_stop")
        self.assertEqual([row[0] for row in trace], list(times))
        self.assertEqual(telemetry[0], (0, 0, 0, 0))
        self.assertEqual(telemetry[1][:2], (setup.cap, 0))
        self.assertAlmostEqual(telemetry[1][2], 100 / 60 * first)
        self.assertEqual(telemetry[2][:2], (setup.cap, setup.cap))
        self.assertAlmostEqual(telemetry[2][2], control_m0.LEAD_TURNS)
        # A firmware tick between the two host writes sees the new position and old feedforward.
        telemetry.clear()
        simulate(setup, Model(rotor), (10, 0.3), 0, timestamps=[0.025, 0.025 + FIRMWARE_PERIOD],
                 references=[100, 100], telemetry=telemetry)
        self.assertGreater(telemetry[1][2], 0)
        self.assertEqual(telemetry[1][3], 0)

    def test_recorded_position_and_feedforward_writes_take_effect_independently(self):
        setup = setup_for(motor_core.load_config(), 30, 0.025, 0.08)
        telemetry = []
        initial = (0.001, 200, 0.101, 0.0, 0.103)
        for sample_time, feedforward in ((0.03, 0), (0.039, 30)):
            telemetry.clear()
            simulate(setup, Model((0,) * 7), (10, 0.2), 0.1, timestamps=[0.025, sample_time],
                     references=[30] * 2, telemetry=telemetry, position_initial=initial,
                     position_writes=np.array([[0.028, 0.035], [0.040, 0.041]]))
            self.assertAlmostEqual(telemetry[0][2], initial[-1])
            self.assertAlmostEqual(telemetry[1][2], initial[2] + 0.5 * 0.024)
            self.assertAlmostEqual(telemetry[1][3], feedforward)

    def test_position_write_between_firmware_ticks_survives_next_host_sample(self):
        setup = setup_for(motor_core.load_config(), 30, 0.025, 0.08)
        telemetry = []
        simulate(setup, Model((0,) * 7), (10, 0.2), 0, timestamps=[0.025, 0.05, 0.075],
                 references=[30] * 3, telemetry=telemetry, position_initial=(0.001, 0, 0, 0, 0),
                 position_writes=np.array([[0.04995, 0.04996], [0.07495, 0.07496], [0.079, 0.08]]))
        self.assertEqual(telemetry[1][2:], (0, 0))
        self.assertAlmostEqual(telemetry[2][2], 0.012)
        self.assertAlmostEqual(telemetry[2][3], 30)

    def test_firmware_position_simulation_tracks_each_signed_direction(self):
        setup = Setup(30, 0.025, 0.1, 0.08, 0.83, 1.67, 2000, 0.4, 1, None, encoder_bandwidth=150)
        model = Model((100, 0.1, 0.2, 0, 0, 0, 0), write_delay=0.002)
        for direction in (1, -1):
            trace = []
            reason, metrics = simulate(setup._replace(rpm=direction * 30), model, (8, 0.15), 0, trace=trace)
            self.assertEqual(reason, "user_stop")
            self.assertLess(metrics.plateau_error, 1.2)
            self.assertGreater(direction * np.mean(np.asarray(trace)[-40:, 2]), 25)

    def test_recorded_torque_drives_rotor_without_controller_or_actuator_lag(self):
        setup = Setup(-30, 0.025, 0.1, 0.08, 0.83, 1.67, 2000, 0.4, 1, None, encoder_bandwidth=150)
        model = Model((100, 0, 0, 0, 0, 0, 0), actuator=Lag(delay=1))
        times = np.array([0.025, 0.05, 0.075, 0.1])
        trace = []
        reason, _ = simulate(setup, model, (0, 0), 0, timestamps=times, trace=trace,
                             measured_torque=([0, 0.025, 0.1], [0, 0, -0.06]))
        self.assertEqual(reason, "user_stop")
        self.assertEqual([row[0] for row in trace], list(times))
        self.assertEqual([row[2] for row in trace[:2]], [0, 0])
        self.assertAlmostEqual(trace[-1][2], -9.5, delta=1.2)

    def test_waveform_applies_logged_torque_constant_with_encoder_polarity(self):
        samples = [dict(elapsed_s=str(i * 0.025), setpoint_rpm="30", encoder_count_rpm="1.2",
                        encoder_shadow_count=str(i), firmware_torque_setpoint_nm=str(i * -0.06),
                        iq_setpoint_a=str(i), iq_measured_a=str(i * 0.5)) for i in (1, 2)]
        waveform = read_waveform(samples, 0.025, 0.06)
        np.testing.assert_allclose(waveform[4], [-0.03, -0.06])
        self.assertEqual(waveform[-1], (-0.06, 0.0))

    def test_recorder_matches_native_capture_aligns_ticks_and_rejects_bad_exports(self):
        setup = Setup(30, 0.025, 0.1, 0.08, 0.83, 1.67, 2000, 0.4, 1, None)
        time = np.arange(1, 301) / 40
        speed = np.full(len(time), 30.0)
        trial = Trial(Path("native.csv"), setup, (8, 0.15), time, speed, speed, time * 1000, np.zeros(len(time)),
                      "user_stop", True, stop_elapsed=7.6, digest=sha256(b"native").digest(),
                      board_serial="SERIAL", torque_calibration=(-0.06, 0))
        digest = trial.digest.hex()
        meta = {"axis": 0, "native_sha256": digest, "board_serial": "SERIAL", "encoder_cpr": 2000,
                "overflow": False, "tick_hz": 8000, "tick_stride": 40,
                "clock_anchors": [{"tick": 100, "elapsed_s": -0.05}, {"tick": 61300, "elapsed_s": 7.6}]}
        rows = [(i * 40 + 500, i * 5, 0.5, -0.25) for i in range(1521)]

        with TemporaryDirectory() as folder:
            root = Path(folder)
            self.assertIsNone(read_recorder(root, trial))
            manifest = root / f"{digest}.json"
            manifest.write_text(json.dumps(meta))
            with self.assertRaises(FileNotFoundError):
                read_recorder(root, trial)

            def write_channels(headers, data):
                with (root / f"{digest}.csv").open("w", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(headers)
                    writer.writerows(data)

            for headers, data, reason in ((CHANNELS[:-1], rows, "four synchronized channels"),
                                          (CHANNELS, rows[:-2], "complete native trial"),
                                          (CHANNELS, rows[:3] + rows[4:], "ticks are missing"),
                                          (CHANNELS, [(tick, count + 15, q, measured) for tick, count, q, measured in rows],
                                           "alignment disagrees")):
                with self.subTest(reason=reason):
                    write_channels(headers, data)
                    with self.assertRaisesRegex(ValueError, reason):
                        read_recorder(root, trial)

            write_channels(CHANNELS, rows)
            for edits, reason in (({"native_sha256": "other"}, "identity"), ({"board_serial": "other"}, "identity"),
                                  ({"encoder_cpr": 4000}, "identity"), ({"overflow": True}, "overflow"),
                                  ({"tick_hz": 4000}, "cadence"),
                                  ({"clock_anchors": [{"tick": 100, "elapsed_s": -0.05},
                                                      {"tick": 61300, "elapsed_s": 8.6}]}, "anchors disagree")):
                with self.subTest(edits=edits):
                    manifest.write_text(json.dumps({**meta, **edits}))
                    with self.assertRaisesRegex(ValueError, reason):
                        read_recorder(root, trial)

            manifest.write_text(json.dumps(meta))
            recorded = read_recorder(root, trial)
        np.testing.assert_allclose(recorded.time, np.arange(1521) / 200, atol=1e-12)
        np.testing.assert_allclose(recorded.commanded, -0.03)
        np.testing.assert_allclose(recorded.applied, 0.015)
        self.assertIsNone(trial.recorder)

    def test_recorder_torque_integrates_pulses_between_native_samples(self):
        time = np.array([0, 0.01, 0.02, 0.035, 0.05])
        applied = np.array([0, 0.10, 0, 0.12, 0])
        samples = RecorderTrace(time, applied, applied)
        np.testing.assert_allclose(recorder_interval_torque(np.array([0, 0.02, 0.05]), samples), [0.056])

    def test_synchronized_holdout_current_is_used_only_for_rotor_diagnostics(self):
        setup = Setup(30, 0.025, 0.1, 0.08, 0.83, 1.67, 2000, 0.4, 1, None)
        time = np.arange(1, 301) / 40
        rpm = np.full(len(time), 30.0)
        trial = Trial(Path("native.csv"), setup, (8, 0.15), time, rpm, rpm, time * 1000, np.zeros(len(time)),
                      "user_stop", True)
        trials = [replace(trial, path=Path(name), gains=gains, digest=name.encode(),
                          position_initial=(0, 0, 0, 0, 0)) for name, gains in (
            ("a-train", (8, 0.15)), ("b-holdout", (8, 0.15)), ("c-train", (9, 0.15)), ("d-holdout", (9, 0.15)),
        )]
        recording = RecorderTrace(time, np.zeros(len(time)), np.zeros(len(time)))
        with (TemporaryDirectory() as folder, patch.object(pso_m0, "read_recorder", return_value=recording) as load,
              patch.object(pso_m0, "fit") as fit,
              patch.object(pso_m0, "replay", return_value=(True, "ok")) as replay,
              redirect_stdout(io.StringIO())):
            directory = Path(folder)
            for capture in trials:
                (directory / f"{capture.digest.hex()}.csv").touch()
            _, evidence, failures = pso_m0.validate_model([trial, *trials], recorder_dir=directory)
        self.assertEqual([call.args[1].path.name for call in load.call_args_list],
                         ["a-train", "c-train", "b-holdout", "d-holdout"])
        self.assertEqual([item.path.name for item in fit.call_args.args[0]], ["a-train", "c-train"])
        self.assertTrue(all(item.recorder is recording for item in fit.call_args.args[0]))
        self.assertEqual([call.args[0].path.name for call in replay.call_args_list], ["b-holdout", "d-holdout"])
        self.assertTrue(all(call.args[0].recorder is None for call in replay.call_args_list))
        self.assertTrue(all(call.kwargs["rotor_recording"] is recording for call in replay.call_args_list))
        self.assertEqual(failures, [])
        self.assertEqual(len(evidence[setup]), 2)

    def test_one_step_resets_recorded_state_and_predicts_count_delta(self):
        setup = Setup(30, 0.025, 0.1, 0.08, 0.83, 1.67, 12000, 0.4, 1, None)
        model = Model((100, 0, 0, 0, 0, 0, 0))
        time = np.array([0.025, 0.050, 0.080])
        trial = Trial(Path("step.csv"), setup, (8, 0.15), time, np.full(3, 30), np.full(3, 500),
                      np.array([101, 900, 1200]), np.full(3, 0.04), "user_stop", True,
                      estimated_rpm=np.array([12, -6, 500]))
        samples = np.r_[0, time], np.full(4, 0.04)
        np.testing.assert_allclose(one_step_rpm(trial, model, samples, 3), [15, -14 / 6], atol=1e-10)
        trace = []
        simulate(setup, Model((0, 0, 0, 0, 0, 0, 0)), trial.gains, trial.counts[0] / setup.cpr,
                 timestamps=[0.025], trace=trace, initial_velocity_rps=-0.2, measured_torque=samples)
        self.assertAlmostEqual(trace[0][2], -12)

    def test_reconstruction_keeps_hold_causal_at_shifted_sample_times(self):
        time = np.array([0.0, 0.025, 0.051])
        torque = np.array([0.0, 0.06, -0.06])
        query = np.array([0.013, 0.025, 0.038, 0.051])
        np.testing.assert_allclose(reconstruct_torque(time, torque, query, hold=True), [0, 0.06, 0.06, -0.06])
        np.testing.assert_allclose(reconstruct_torque(time, torque, query), [0.0312, 0.06, 0, -0.06], atol=1e-12)

    def test_speed_change_uses_adjacent_count_intervals_on_irregular_timestamps(self):
        setup = Setup(30, 0.025, 0.1, 0.08, 0.83, 1.67, 10000, 0.4, 1, None)
        time = np.array([0.03, 0.05, 0.08, 0.12])
        trial = Trial(Path("speed-change.csv"), setup, (8, 0.15), time, np.full(4, 30), np.zeros(4),
                      np.array([100, 148, 250, 442]), np.full(4, 0.04), "user_stop", True,
                      estimated_rpm=np.array([12, 16.8, 999, 999]))
        model = Model((100, 0, 0, 0, 0, 0, 0))
        observed, predicted = interval_speed_changes(trial, model, (time, trial.applied), len(time))
        np.testing.assert_allclose(observed, [6, 8.4], atol=1e-10)
        np.testing.assert_allclose(predicted, observed, atol=1e-10)

    def test_replay_phase_boundaries_assign_each_interval_once(self):
        time = np.array([0.2, 0.25, 1.24, 1.25, 5.24, 5.25, 7.24, 7.25, 12.25, 12.26])
        masks = pso_m0.phase_masks(time, 0.25, 5.25, 7.25, 12.25)
        self.assertEqual({name: np.flatnonzero(mask).tolist() for name, mask in masks.items()},
                         {"startup": [1, 2], "acceleration ramp": [3, 4], "settling": [5, 6], "steady": [7, 8]})

    def test_reconstruction_spread_compares_traces_and_requires_a_consistent_pair(self):
        predictions = {"linear": np.array([50, 4, -4]), "hold": np.array([0, -4, 4])}
        spread = pso_m0.reconstruction_spread(predictions, np.array([False, True, True]))
        self.assertEqual(spread, {("linear", "hold"): 8.0})
        pair, other = ("linear", "hold"), ("linear", "late")
        audits = [pso_m0.PhaseAudit(2, 4, 0, 4, 0, None, None, 0, None, {pair: x, other: y})
                  for x, y in [(8, 1), (7, 1), (6, 8), (6, 6)]]
        for count, expected in ((3, "1/4 runs above 6 RPM"), (4, "2/4 runs above 6 RPM")):
            cases = audits if count == 4 else [*audits[:1], audits[3], *audits[2:]]
            with self.subTest(expected=expected), redirect_stdout(io.StringIO()) as output:
                pso_m0.report_phase("startup", cases)
            self.assertIn(expected, output.getvalue())
            self.assertEqual("· under-resolved" in output.getvalue(), count == 4)

    def test_spectral_mismatch_rejects_a_holdout_with_passing_rpm_rmse(self):
        setup = Setup(30, 0.025, 0.1, 0.08, 0.83, 1.67, 2000, 0.4, 1, None)
        time = np.arange(1, 330) * setup.period
        ref = np.full(len(time), setup.rpm)
        actual, modeled = [ref + 3 * np.sin(2 * np.pi * frequency * (time - time[0])) for frequency in (12, 8)]
        trial = Trial(Path("wave.csv"), setup, (8, 0.15), time, ref, actual, np.zeros(len(time)),
                      np.zeros(len(time)), "user_stop", True)

        def simulated(*args, timestamps, references, trace, telemetry=None, **kwargs):
            speed = modeled[:len(timestamps)]
            trace.extend(zip(timestamps, references, speed))
            if telemetry is not None:
                telemetry.extend([(0, 0, 0, 0)] * len(timestamps))
            return "user_stop", measure(setup.rpm, timestamps, references, speed)

        with TemporaryDirectory() as folder, patch.object(pso_m0, "simulate", side_effect=simulated):
            ok, detail = pso_m0.replay(trial, None, diagnostics_dir=Path(folder))
            with (Path(folder) / "replay-wave.csv").open() as stream:
                rows = list(csv.DictReader(stream))
        self.assertFalse(ok)
        raw, _, _ = detail.partition(",")
        self.assertLess(float(raw.removeprefix("RPM RMSE ").split("/")[0]), 6)
        self.assertIn("oscillation frequency error (Hz) 4/2.4 FAIL", detail)
        self.assertIn("oscillation measured/model 12/8 Hz", detail)
        self.assertAlmostEqual(float(rows[3]["measured_rpm"]), actual[3])
        self.assertAlmostEqual(float(rows[3]["predicted_rpm"]), modeled[3])

    def test_position_holdout_fails_if_measured_current_rotor_stops_early(self):
        setup = setup_for(motor_core.load_config(), 30, 0.025, 0.08)
        time = np.arange(1, 301) * setup.period
        rpm = np.full(len(time), 30.0)
        torque = np.full(len(time), 0.03)
        # The logged torque law predicts 0.03 Nm for 0.01 turn of position error.
        feedback = np.tile((0.01, 30, 0.0, setup.cap), (len(time), 1))
        trial = Trial(Path("holdout.csv"), setup, (10, 0.3), time, rpm, rpm, np.zeros(len(time)),
                      torque, "user_stop", True, torque_set=torque, estimated_rpm=rpm,
                      position_feedback=feedback)

        def simulate_closed(*args, timestamps, references, trace, **kwargs):
            trace.extend(zip(timestamps, references, references))
            return "user_stop", trial.metrics()

        with (patch.object(pso_m0, "simulate", side_effect=simulate_closed),
              patch.object(pso_m0, "rotor_replay", return_value="overspeed")):
            ok, detail = pso_m0.replay(trial, None, rotor_recording=RecorderTrace(time, torque, torque))
        self.assertFalse(ok)
        self.assertIn("measured-current rotor (synchronized): overspeed", detail)
        self.assertIn("firmware torque from logged feedback", detail)
        np.testing.assert_allclose(pso_m0.firmware_torque_prediction(trial), torque)

    def test_simulator_quantizes_physical_count_interval_and_delays_applied_torque(self):
        setup = Setup(500, 0.025, 1e4, 0.1, 10, 20, 10000, 0.4, 100.0, None)
        rotor = (4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        times = np.array([0.02, 0.049, 0.072, 0.10])
        immediate, delayed = [], []
        simulate(setup, Model(rotor), (1, 0.01), 0, timestamps=times, trace=immediate)
        simulate(setup, Model(rotor, actuator=Lag(0.025)), (1, 0.01), 0, timestamps=times, trace=delayed)
        self.assertEqual([row[0] for row in immediate], list(times))
        dt = np.diff(times, prepend=0)
        counts = np.array([row[2] for row in immediate]) * setup.cpr * dt / 60
        np.testing.assert_allclose(counts, np.rint(counts), atol=1e-10)
        # From rest, constant 0.1 Nm after the first host write moves 12.8 counts by 0.10 s.
        self.assertAlmostEqual(counts.sum(), 12.8, delta=1.1)
        self.assertLess(sum(row[2] * interval for row, interval in zip(delayed, dt)),
                        sum(row[2] * interval for row, interval in zip(immediate, dt)))

    def test_model_gate_checks_ramp_rmse_and_unshifted_speed_trace(self):
        setup = Setup(30, 0.025, 0.1, 0.08, 0.83, 1.67, 2000, 0.4, 1.0, None)
        model = Model((200.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0), feedback=Lag(0.00625, 0.0125))
        trials = []
        for i, gains in enumerate(((7.0, 0.16), (8.0, 0.15)) * 2):
            rows = []
            reason, metrics = simulate(setup, model, gains, 0, trace=rows)
            self.assertEqual(reason, "user_stop")
            time, reference, speed = np.asarray(rows).T
            counts = np.cumsum(speed * np.diff(time, prepend=0) * setup.cpr / 60)
            trials.append(Trial(Path(f"run-{i}.csv"), setup, gains, time, reference, speed, counts,
                                np.zeros(len(time)), reason, True))
        with patch.object(pso_m0, "fit", return_value=model) as fit, redirect_stdout(io.StringIO()):
            _, evidence, failures = pso_m0.validate_model(trials)
            self.assertEqual([trial.path for trial in fit.call_args.args[0]], [trial.path for trial in trials[:2]])
            self.assertEqual(failures, [])
            self.assertFalse(evidence)
            for channel in ("ramp_error", "trace"):
                def mismatched(*args, **kwargs):
                    reason, predicted = simulate(*args, **kwargs)
                    if channel == "trace":
                        trace = kwargs["trace"]
                        trace[:] = [(t, rpm, measured + 10) for t, rpm, measured in trace]
                    else:
                        predicted = predicted._replace(ramp_error=predicted.ramp_error + 5)
                    return reason, predicted

                with self.subTest(channel=channel), patch.object(pso_m0, "simulate", side_effect=mismatched):
                    _, evidence, failures = pso_m0.validate_model(trials)
                    self.assertFalse(evidence)
                    self.assertEqual(len(failures), 2)

    def test_cli_requires_explicit_save_after_bench(self):
        for answer in ("", "save"):
            with (self.subTest(answer=answer), patch("sys.argv", ["pso_m0", "position", "--bench"]),
                  patch("sys.stdin.isatty", return_value=True), patch("builtins.input", return_value=answer),
                  patch.object(pso_m0, "load_trials", return_value=[]),
                  patch.object(pso_m0, "optimize", return_value=Selection((7.0, 0.17), 0.04, (), True)),
                  patch.object(pso_m0, "bench", return_value=[]),
                  patch.object(pso_m0, "apply_result") as save,
                  redirect_stdout(io.StringIO())):
                self.assertEqual(pso_m0.main(), 0)
                self.assertEqual(save.call_count, int(answer == "save"))

    def test_cli_rejects_provisional_bench_without_prompting_or_saving(self):
        with (patch("sys.argv", ["pso_m0", "position", "--bench"]),
              patch("sys.stdin.isatty", return_value=True), patch("builtins.input") as prompt,
              patch.object(pso_m0, "load_trials", return_value=[]),
              patch.object(pso_m0, "optimize", return_value=Selection((7.0, 0.17), 0.04, (), False)),
              patch.object(pso_m0, "bench") as run, patch.object(pso_m0, "apply_result") as save,
              redirect_stdout(io.StringIO()), patch("sys.stderr", new_callable=io.StringIO)):
            self.assertEqual(pso_m0.main(), 1)
        prompt.assert_not_called()
        run.assert_not_called()
        save.assert_not_called()

    def test_cli_diagnostics_mode_excludes_hardware_actions(self):
        for option in (["--diagnostics-dir", "replays"], ["--measured-current-replay"]):
            for action in (["--bench"], ["--reference", "30"]):
                argv = ["pso_m0", "position", *option, *action]
                with (self.subTest(argv=argv), patch("sys.argv", argv), patch("sys.stderr", new_callable=io.StringIO),
                      patch.object(tuning, "snapshot") as snapshot, self.assertRaises(SystemExit) as stop):
                    pso_m0.main()
                self.assertEqual(stop.exception.code, 2)
                snapshot.assert_not_called()

    def test_measured_current_cli_only_replays_position_holdouts(self):
        with (TemporaryDirectory() as folder, patch.object(pso_m0, "LOGS", Path(folder)),
              patch("sys.argv", ["pso_m0", "position", "--measured-current-replay"]),
              patch.object(pso_m0, "load_trials", return_value=["records"]),
              patch.object(pso_m0, "measured_current_replay") as diagnose,
              patch.object(pso_m0, "optimize") as optimize, patch.object(pso_m0, "run_trial") as run,
              patch.object(pso_m0, "apply_result") as save, redirect_stdout(io.StringIO())):
            (Path(folder) / "position_recorder").mkdir()
            self.assertEqual(pso_m0.main(), 0)
        diagnose.assert_called_once_with(["records"], recorder_dir=None)
        optimize.assert_not_called()
        run.assert_not_called()
        save.assert_not_called()

    def test_synchronized_diagnostic_splits_only_matched_native_captures(self):
        setup = setup_for(motor_core.load_config(), 30, 0.025, 0.08)
        time = np.arange(1, 301) * setup.period
        rpm = np.full(len(time), 30.0)
        trial = Trial(Path("native.csv"), setup, (8, 0.15), time, rpm, rpm, time * 1000,
                      np.zeros(len(time)), "user_stop", True)
        trials = [replace(trial, path=Path(str(i)), digest=bytes([i]), gains=(g, 0.15))
                  for i, g in enumerate((8, 9, 8, 9, 8, 9, 8, 9))]
        recording = RecorderTrace(time, np.zeros(len(time)), np.zeros(len(time)))
        with (TemporaryDirectory() as folder, patch.object(pso_m0, "read_recorder", return_value=recording) as load,
              patch.object(pso_m0, "fit", side_effect=RuntimeError("end test")), redirect_stdout(io.StringIO())):
            directory = Path(folder)
            for capture in trials[4:]:
                (directory / f"{capture.digest.hex()}.csv").touch()
            with self.assertRaisesRegex(RuntimeError, "end test"):
                pso_m0.measured_current_replay(trials, recorder_dir=directory)
        self.assertEqual([call.args[1].path.name for call in load.call_args_list], ["4", "5"])

    def test_recorder_cli_is_position_analysis_and_bench_only(self):
        with TemporaryDirectory() as folder:
            for mode, action in (("position", ["--reference", "30"]), ("torque", [])):
                argv = ["pso_m0", mode, "--recorder-dir", folder, *action]
                with (self.subTest(argv=argv), patch("sys.argv", argv), patch("sys.stderr", new_callable=io.StringIO),
                      patch.object(tuning, "snapshot") as snapshot, self.assertRaises(SystemExit) as stop):
                    pso_m0.main()
                self.assertEqual(stop.exception.code, 2)
                snapshot.assert_not_called()
            with (patch("sys.argv", ["pso_m0", "position", "--bench", "--recorder-dir", folder]),
                  patch("sys.stdin.isatty", return_value=True), patch("builtins.input", return_value=""),
                  patch.object(pso_m0, "load_trials", return_value=[]),
                  patch.object(pso_m0, "optimize", return_value=Selection((7.0, 0.17), 0.04, (), True)) as optimize,
                  patch.object(pso_m0, "bench", return_value=[]), redirect_stdout(io.StringIO())):
                self.assertEqual(pso_m0.main(), 0)
                self.assertEqual(optimize.call_args.kwargs["recorder_dir"], Path(folder))

    def test_cli_and_bench_revalidation_use_same_default_recorder_directory(self):
        selection = Selection((7.0, 0.17), 0.04, (), True)
        records = []
        with TemporaryDirectory() as folder:
            logs = Path(folder)
            exports = logs / "position_recorder"
            exports.mkdir()
            revision = tuning.snapshot(0, "position")["revision"]
            operator = Mock()
            operator.ask_pso.return_value = "discard"
            with (patch.object(pso_m0, "LOGS", logs), patch("sys.argv", ["pso_m0", "position"]),
                  patch.object(pso_m0, "load_trials", return_value=records),
                  patch.object(pso_m0, "optimize", return_value=selection) as optimize,
                  patch.object(pso_m0, "bench"), redirect_stdout(io.StringIO())):
                self.assertEqual(pso_m0.main(), 0)
                worker.perform_pso("pso_analyze", None, operator, {"mode": "position"})
                with self.assertRaisesRegex(KeyboardInterrupt, "Validated gains discarded"):
                    worker.perform_pso("pso_validate", None, operator,
                                       {"mode": "position", "revision": revision, "candidate": selection.gains})
        self.assertEqual([call.kwargs["recorder_dir"] for call in optimize.call_args_list], [exports] * 3)

    def test_measured_current_replay_compares_only_shared_samples_after_an_early_stop(self):
        setup = Setup(30, 0.025, 0.1, 0.08, 0.83, 1.67, 2000, 0.4, 1, None)
        time = np.arange(1, 300) * setup.period
        rpm = np.full(len(time), 30.0)
        trial = Trial(Path("holdout.csv"), setup, (8, 0.15), time, rpm, rpm, np.zeros(len(time)),
                      np.full(len(time), 0.02), "user_stop", True)
        training = ["first training capture", "second training capture"]

        def simulated(*args, timestamps, trace, measured_torque=None, **kwargs):
            if measured_torque is None:
                trace.extend((t, 30, 30 if i < 2 else 100) for i, t in enumerate(timestamps))
                return "user_stop", None
            np.testing.assert_array_equal(measured_torque[0][1:], time)
            np.testing.assert_array_equal(measured_torque[1][1:], trial.applied)
            trace.extend((t, 30, 34) for t in timestamps[:2])
            return "overspeed", None

        with (patch.object(pso_m0, "split_trials", return_value=(training, [trial])),
              patch.object(pso_m0, "report_rotor_fit") as audit,
              patch.object(pso_m0, "fit") as fit, patch.object(pso_m0, "simulate", side_effect=simulated),
              redirect_stdout(io.StringIO()) as output):
            pso_m0.measured_current_replay([trial])
        fit.assert_called_once_with(training)
        audit.assert_called_once_with(training, fit.return_value)
        self.assertIn("raw RMSE 0.00 → 4.00 RPM", output.getvalue())
        self.assertIn("measured current overspeed", output.getvalue())
        self.assertIn("+30 RPM · 0/1 complete comparisons", output.getvalue())

    def test_bench_stops_after_a_failed_trial(self):
        baseline = (0.05, 0.03)
        cfg = motor_core.load_config()
        setups = tuple(setup_for(cfg, rpm, 0.025, 1.0) for rpm in pso_m0.validation_speeds("torque"))
        history = [example(setup, baseline) for setup in setups]
        with patch.object(pso_m0, "run_trial", side_effect=RuntimeError("trial fault")) as run:
            with self.assertRaisesRegex(RuntimeError, "trial fault"):
                bench("torque", "revision", baseline, Selection((0.01, 0.015), 0.04, setups, True), history)
        self.assertEqual(run.call_count, 1)
        self.assertEqual(run.call_args.args[1], 200)

    def test_bench_checks_admitted_torque_cap_before_motion_confirmation(self):
        cfg = motor_core.load_config()
        expected = setup_for(cfg, -30, 0.025, 0.06)
        draft = pso_m0.make_draft("position", "revision", (8.0, 0.15))
        approve = Mock()

        def run(_rpm, _cfg, *, check_setup, confirm, **kwargs):
            check_setup(0.025, 0.08)
            confirm("would arm")

        with (patch.object(tuning, "draft_config", return_value=cfg),
              patch.object(pso_m0, "settled_drive"),
              patch.object(pso_m0.control_m0, "run", side_effect=run)):
            with self.assertRaisesRegex(RuntimeError, "different control period or torque cap"):
                pso_m0.run_trial("position", -30, draft, approve=approve, check_cancel=Mock(),
                                 expected_setup=expected)
        approve.assert_not_called()

    def test_supervised_telemetry_keeps_live_readings_with_csv_notification(self):
        cfg = motor_core.load_config()
        notify = Mock()
        output = io.StringIO()
        with TemporaryDirectory() as root, patch.object(m0_display, "LOGS", Path(root)):
            with redirect_stdout(output):
                table = m0_display.TelemetryTable(
                    "torque", (("speed", "Speed RPM", ".2f"),), cfg=cfg, target_rpm=500,
                    metadata={}, notify=notify, show_readings=True)
                table.sample(0.2, 497.25, errors=(0, 0, 0, 0, 0))
                table.close()
        self.assertIn("497.25", output.getvalue())
        notify.assert_called_once_with("csv", str(table.log_path))

    def test_apply_checks_revision_and_edits_only_selected_pair(self):
        original = motor_core.CONFIG_PATH.read_bytes()
        with TemporaryDirectory() as root:
            path = Path(root) / "motor_config.json"
            path.write_bytes(original)
            with (patch.object(motor_core, "CONFIG_PATH", path),
                  patch.object(motor_core, "MOTION_LOCK", path.with_suffix(".lock"))):
                revision = tuning.snapshot(0, "torque")["revision"]
                apply_result("torque", revision, (0.02, 0.015))
                expected = json.loads(original)
                expected["axes"]["m0"]["torque"].update(kp=0.02, ki=0.015)
                self.assertEqual(json.loads(path.read_bytes()), expected)
                with self.assertRaisesRegex(ValueError, "has changed"):
                    apply_result("torque", revision, (0.01, 0.015))
