import copy
import csv
import io
import json
import math
import os
import sys
import tempfile
import termios
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from threading import Event
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from motor import calibrate_m0
from motor import control_m0 as control
from motor import encoder_m0
from motor import m0_display
from motor import motor_core as core
from motor import teleop_motor as teleop
from motor import torque_m0 as torque
from motor import vel_m0 as vel
from motor.live import LiveRPM, slew_rpm
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    AXIS_STATE_IDLE, AXIS_STATE_MOTOR_CALIBRATION, AXIS_STATE_SENSORLESS_CONTROL,
    AXIS_ERROR_BRAKE_RESISTOR_DISARMED, AXIS_ERROR_MOTOR_FAILED,
    CONTROLLER_ERROR_OVERSPEED, LOCKIN_STATE_INACTIVE,
    MOTOR_ERROR_CURRENT_LIMIT_VIOLATION, MOTOR_ERROR_DC_BUS_OVER_REGEN_CURRENT,
    INPUT_MODE_PASSTHROUGH, INPUT_MODE_VEL_RAMP,
    CONTROL_MODE_POSITION_CONTROL, CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_TORQUE_RAMP,
    ENCODER_MODE_INCREMENTAL, ENCODER_MODE_SPI_ABS_AMS,
)


class Axis:
    def __init__(self):
        self.current_state = AXIS_STATE_IDLE
        self.error = 0
        self.requests = []
        self.watchdog_feed = Mock()
        self.config = NS(watchdog_timeout=1.5, enable_watchdog=False, sensorless_ramp=NS(
            current=4.0, ramp_time=0.5, ramp_distance=1.0, accel=5.0, vel=10.0,
            finish_on_vel=False, finish_on_distance=True, finish_on_enc_idx=False))
        for name in calibrate_m0.STARTUP_MOTION:
            setattr(self.config, name, False)
        self.motor = NS(error=0, is_calibrated=True, effective_current_lim=8.0,
                        config=NS(current_control_bandwidth=1000.0, torque_lim=math.inf,
                                  phase_resistance=0.5, phase_inductance=0.01),
                        current_control=NS(Iq_setpoint=0.5, Iq_measured=0.4, Id_measured=0.0,
                                           Id_setpoint=0.0, I_measured_report_filter_k=1.0))
        self.encoder = NS(error=0, is_ready=True, config=NS(enable_phase_interpolation=True,
                          offset=0, offset_float=0.0, calib_scan_distance=16 * math.pi,
                          mode=ENCODER_MODE_INCREMENTAL), vel_estimate=0.0, pos_estimate=0.0,
                          shadow_count=0, phase=0.1, calib_scan_response=4096)
        self.controller = NS(error=0, vel_setpoint=0.0, anticogging_valid=False,
                             config=NS(control_mode=3, input_mode=1, vel_gain=0.1,
                                       pos_gain=20.0, vel_integrator_gain=0.2, vel_limit=2.0,
                                       vel_ramp_rate=1.0, vel_limit_tolerance=1.2, torque_ramp_rate=0.01,
                                       anticogging=NS(anticogging_enabled=False,
                                                      calib_anticogging=False)))
        for key in core.PROTECTIONS:
            setattr(self.controller.config, key, False)
        self.sensorless_estimator = NS(config=NS(pm_flux_linkage=0.1),
                                       vel_estimate=-550 / 60, error=0)
        self.lockin_state = LOCKIN_STATE_INACTIVE

    @property
    def requested_state(self):
        return self.current_state

    @requested_state.setter
    def requested_state(self, state):
        self.requests.append(state)
        self.current_state = state
        if state == AXIS_STATE_SENSORLESS_CONTROL:
            self.controller.vel_setpoint = self.controller.input_vel


class ODriveTest(unittest.TestCase):
    def setUp(self):
        self.raw = json.loads(core.CONFIG_PATH.read_text())
        self.raw["axes"]["m0"]["torque"] = {"torque_limit_nm": 0.10, "kp": 0.02, "ki": 0.05}
        temp = self.enterContext(tempfile.TemporaryDirectory())
        self.root = Path(temp)
        self.config_path = self.root / "motor_config.json"
        self.enterContext(patch.object(core, "CONFIG_PATH", self.config_path))
        self.enterContext(patch.object(core, "MOTION_LOCK", self.root / "logs" / ".motion.lock"))
        self.enterContext(patch.object(m0_display, "LOGS", self.root / "logs"))
        self.enterContext(patch.object(core.odrive, "find_any",
                                     side_effect=AssertionError("Unexpected USB discovery")))
        self.enterContext(redirect_stdout(io.StringIO()))
        self.save()

    def save(self):
        self.config_path.write_text(json.dumps(self.raw))

    def test_control_routes_signed_targets_before_arming(self):
        cfg = core.load_config()
        with patch.object(control, "run_position") as position, patch.object(torque, "run") as high:
            for rpm in (30, -30, 199, -199, 200, -200, 201, -201):
                control.run(rpm, cfg, capture=False)
        self.assertEqual([c.args[0] for c in position.call_args_list], [30, -30, 199, -199])
        self.assertEqual([c.args[0] for c in high.call_args_list], [200, -200, 201, -201])
        self.assertTrue(all(c.kwargs["capture"] is False for c in [*position.call_args_list, *high.call_args_list]))

    def test_encoder_monitor_counts_and_stops_when_axis_leaves_idle(self):
        cfg = core.load_config(read_only=True)
        dev = self.board(cfg)
        axis = dev.axis0
        axis.encoder.shadow_count = 2**31 - 1
        notify = Mock()

        def advance(_seconds):
            if notify.call_count == 1:
                axis.encoder.shadow_count = -2**31
            else:
                axis.current_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        with patch.object(encoder_m0, "find_odrive", return_value=dev), \
                patch.object(encoder_m0.time, "sleep", side_effect=advance):
            with self.assertRaisesRegex(RuntimeError, "must be IDLE"):
                encoder_m0.run(cfg, notify=notify)
        self.assertEqual([c.args[1]["delta"] for c in notify.call_args_list], [0, 1])
        self.assertEqual(axis.requests, [])

    def test_live_runs_log_targets_ramp_hold_reverse_and_stop(self):
        for action, start, high, lower, pattern in (
            ("velocity", 30, 50, 20, "velocity"),
            ("auto", 30, 50, 20, "position"),
            ("auto", 220, 240, 205, "torque"),
            ("torque", 30, 50, 20, "torque"),
        ):
            with self.subTest(action=action, start=start):
                cfg = core.load_config()
                cfg["velocity"].update(vel_ramp_rate=10, fast_vel_ramp_rate=10, sample_seconds=0.025)
                cfg["calibration"]["settle_hold_s"] = 0.12
                cfg["torque"].update(kp=0.002, ki=0.0001)
                dev = self.control_board(cfg)
                axis = dev.axis0
                events, pending, reports = [], [], []
                scheduled = [(0.16, 1, high), (0.31, 2, lower), (0.42, 3, 0), (0.65, 4, -start)]
                clock, turns = 0.0, axis.encoder.pos_estimate
                live = LiveRPM(start, high, cfg, action=action,
                               take_request=lambda: pending.pop(0) if pending else None,
                               emit=lambda event, data: events.append((event, data)))

                def advance(dt):
                    nonlocal clock, turns
                    clock += dt
                    if pattern == "velocity":
                        vel_set = slew_rpm(axis.controller.vel_setpoint, axis.controller.input_vel, dt * 10)
                        axis.controller.vel_setpoint = axis.encoder.vel_estimate = vel_set
                        turns += vel_set * dt
                    elif pattern == "position":
                        turns = axis.controller.input_pos
                        axis.encoder.vel_estimate = axis.controller.input_vel
                    axis.encoder.pos_estimate = turns
                    axis.encoder.shadow_count = round(turns * cfg["hardware"]["cpr"])
                    if scheduled and clock >= scheduled[0][0]:
                        _, seq, rpm = scheduled.pop(0)
                        pending.append((seq, rpm))

                def cancel_at_end():
                    if clock >= 1.25:
                        pending.append((5, high))
                        raise KeyboardInterrupt

                runner = {"velocity": vel.run, "auto": control.run, "torque": torque.run}[action]
                with patch.object(torque, "admit_control_period", return_value=0.025), \
                        patch.object(torque.time, "monotonic", side_effect=lambda: clock), \
                        patch.object(torque.time, "sleep", side_effect=advance):
                    runner(start, cfg, dev, confirm=lambda _: None, live=live, check_cancel=cancel_at_end,
                           notify=lambda event, data: reports.append((event, data)))
                self.assertEqual([e[1]["rpm"] for e in events], [high, lower, 0, -start])
                self.assertEqual([r[0] for r in reports], ["csv", "running", "trial_result"])
                self.assertTrue(reports[-1][1]["idle"])
                self.assertEqual(axis.requests, [AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE])
                self.assertFalse(axis.config.enable_watchdog)
                path = sorted((self.root / "logs").glob(f"m0_{pattern}_*_{start:+g}rpm.csv"))[-1]
                with path.open() as file:
                    initial, *rows, stop = list(csv.DictReader(file))
                self.assertEqual(json.loads(initial["config_json"])["trial"]["live_ceiling_rpm"], high)
                self.assertEqual((stop["stop_reason"], stop["idle_confirmed"]), ("user_stop", "True"))
                self.assertTrue(any(row["target_sequence"] == "2" and float(row["setpoint_rpm"]) < high
                                    for row in rows))
                reverse = [row for row in rows if float(row["setpoint_rpm"]) < 0]
                self.assertTrue(reverse)
                self.assertGreaterEqual(float(reverse[0]["elapsed_s"]), 0.65 + cfg["calibration"]["settle_hold_s"])
                self.assertTrue(any(row["target_sequence"] == "4" and float(row["active_target_rpm"]) == 0
                                    for row in rows))
                self.assertTrue(all(abs(float(a["setpoint_rpm"]) - float(b["setpoint_rpm"])) <= 15.00001
                                    for a, b in zip(rows, rows[1:])))

    def test_live_overspeed_uses_target_guard_inside_firmware_envelope(self):
        for action, runner in (("velocity", vel.run), ("auto", control.run), ("torque", torque.run)):
            with self.subTest(action=action):
                cfg = core.load_config()
                dev = self.control_board(cfg)
                clock = 0.0
                live = LiveRPM(10, 100, cfg, action=action, take_request=lambda: None, emit=Mock())

                def overspeed_after_start(dt):
                    nonlocal clock
                    clock += dt
                    dev.axis0.encoder.vel_estimate = 90 / 60

                with patch.object(torque, "admit_control_period", return_value=0.025), \
                        patch.object(torque.time, "monotonic", side_effect=lambda: clock), \
                        patch.object(torque.time, "sleep", side_effect=overspeed_after_start):
                    with self.assertRaises(vel.DiagnosticStop) as result:
                        runner(10, cfg, dev, confirm=lambda _: None, live=live)
                self.assertEqual(result.exception.reason, "overspeed")
                self.assertEqual(dev.axis0.requests, [AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE])

    def control_board(self, cfg):
        dev = self.board(cfg)
        core.configure_for_calibration(dev, cfg)
        c = dev.axis0.controller.config
        c.control_mode = CONTROL_MODE_TORQUE_CONTROL
        c.input_mode = INPUT_MODE_TORQUE_RAMP
        c.circular_setpoints = True
        c.pos_gain = 23.0
        c.enable_gain_scheduling = False
        dev.axis0.encoder.pos_estimate = 2.25
        dev.axis0.encoder.shadow_count = 4500
        return dev

    def test_control_position_seeds_caps_and_stalls_once_in_both_directions(self):
        for rpm, live_cap, expected_cap, configured_gain, expected_gain in (
            (30, 0.06, 0.06, 10.0, 10.0), (-30, 0.12, 0.08, None, 20.0),
        ):
            with self.subTest(rpm=rpm):
                self.raw["axes"]["m0"]["controller"]["pos_gain"] = configured_gain
                self.raw["axes"]["m0"]["velocity"]["position_vel_gain"] = 0.10
                self.save()
                cfg = core.load_config()
                dev = self.control_board(cfg)
                axis = dev.axis0
                axis.motor.config.torque_lim = live_cap
                cfg_before = vars(axis.controller.config).copy()
                clock = 0.0
                terminal_start = len(sys.stdout.getvalue())

                def admit(board, selected, *, reader):
                    self.assertEqual(reader(board, selected, expected_state=AXIS_STATE_IDLE).position, 2.25)
                    return 0.025

                def enter(selected, config, **modes):
                    self.assertEqual(selected.controller.config.pos_gain, expected_gain)
                    self.assertEqual(selected.controller.config.vel_gain, 0.10)
                    self.assertEqual(selected.controller.input_pos, 2.25)
                    self.assertEqual(selected.controller.input_vel, 0)
                    self.assertEqual(selected.controller.input_torque, 0)
                    core.enter_closed_loop(selected, config, **modes)
                    selected.encoder.pos_estimate = 2.3
                    selected.encoder.shadow_count = 4600
                    selected.controller.input_pos = 2.3

                def sleep(delay):
                    nonlocal clock
                    clock += delay
                    self.assertLess(clock, 2.0)
                    self.assertEqual(axis.controller.config.control_mode, CONTROL_MODE_POSITION_CONTROL)
                    self.assertFalse(axis.controller.config.circular_setpoints)
                    self.assertEqual(axis.controller.config.vel_integrator_gain, 0)
                    self.assertEqual(axis.motor.config.torque_lim, expected_cap)

                with patch.object(control, "find_odrive", return_value=dev), \
                        patch.object(control, "confirm_motion") as confirm, \
                        patch.object(control, "enter_closed_loop", side_effect=enter), \
                        patch.object(torque, "admit_control_period", side_effect=admit), \
                        patch.object(torque, "LOG_QUEUE_SIZE", 128), \
                        patch.object(control.time, "monotonic", side_effect=lambda: clock), \
                        patch.object(control.time, "sleep", side_effect=sleep), \
                        patch.object(sys, "argv", ["control_m0.py", str(rpm)]):
                    self.assertEqual(control.main(), 1)
                self.assertIn("POSITION M0", confirm.call_args.args[0])
                terminal = sys.stdout.getvalue()[terminal_start:]
                self.assertEqual(terminal.count("RESULT:"), 1)
                self.assertNotIn("M0 control stopped:", terminal)
                self.assertEqual(axis.requests, [AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE])
                self.assertEqual(axis.motor.config.torque_lim, live_cap)
                self.assertEqual(vars(axis.controller.config), cfg_before)
                path, = (self.root / "logs").glob(f"m0_position_*_{rpm:+g}rpm.csv")
                with path.open() as file:
                    start, *samples, stop = list(csv.DictReader(file))
                self.assertEqual(start["control_mode"], "POSITION")
                self.assertEqual(json.loads(start["config_json"])["trial"]["position_gain"], expected_gain)
                self.assertEqual(json.loads(start["config_json"])["trial"]["vel_gain"], 0.10)
                self.assertEqual(float(samples[0]["active_position_turns"]), 2.3)
                self.assertEqual(float(samples[0]["active_velocity_feedforward_rpm"]), 0.0)
                self.assertEqual(float(samples[0]["initial_position_turns"]), 2.3)
                self.assertEqual(float(samples[0]["initial_active_position_turns"]), 2.3)
                self.assertEqual(samples[1]["initial_elapsed_s"], "")
                for sample in samples:
                    stamp_keys = ("elapsed_s", "iq_measured_elapsed_s", *control.POSITION_WRITE_FIELDS)
                    stamps = [float(sample[key]) for key in stamp_keys]
                    self.assertEqual(stamps, sorted(stamps))
                self.assertEqual(samples[1]["active_position_turns"], samples[0]["commanded_position_turns"])
                self.assertEqual(samples[1]["active_velocity_feedforward_rpm"], samples[0]["velocity_feedforward_rpm"])
                self.assertEqual(stop["stop_reason"], "stall")
                self.assertEqual(stop["idle_confirmed"], "True")
                self.assertEqual(float(samples[0]["measured_position_turns"]), 2.3)
                self.assertGreater((float(samples[0]["commanded_position_turns"]) - 2.3) * rpm, 0)
                lead = [float(row["following_error_turns"]) for row in samples]
                ff = [float(row["velocity_feedforward_rpm"]) for row in samples]
                self.assertTrue(all(0 <= math.copysign(1, rpm) * value <= 0.05 + 1e-9 for value in lead))
                self.assertGreater(math.copysign(1, rpm) * ff[0], 0)
                self.assertAlmostEqual(ff[-1], 0)
                self.assertAlmostEqual(abs(lead[-1]), 0.05)
                capture_path = path.with_name(path.name.replace("m0_position_", "m0_capture_", 1))
                with capture_path.open() as file:
                    self.assertEqual(list(csv.DictReader(file)), [
                        {key: sample[key] for key in control.POSITION_CAPTURE_COLUMNS} for sample in samples
                    ])

    def test_control_position_firmware_overspeed_following_error_and_ctrl_c(self):
        for case in ("overspeed", "following_error", "user_stop"):
            with self.subTest(case=case):
                cfg = core.load_config()
                dev = self.control_board(cfg)
                axis = dev.axis0
                clock = 0.0

                def sleep(delay):
                    nonlocal clock
                    clock += delay
                    if clock < 0.05:
                        return
                    if case == "overspeed":
                        self.assertTrue(axis.controller.config.enable_overspeed_error)
                        if axis.encoder.vel_estimate == 0.0:
                            axis.encoder.vel_estimate = 0.85
                            axis.encoder.shadow_count += 42
                            axis.encoder.pos_estimate += 42 / axis.encoder.config.cpr
                        else:
                            axis.controller.error = CONTROLLER_ERROR_OVERSPEED
                    elif case == "following_error":
                        axis.encoder.pos_estimate -= 0.1
                    else:
                        raise KeyboardInterrupt

                with patch.object(control, "confirm_motion"), \
                        patch.object(torque, "admit_control_period", return_value=0.025), \
                        patch.object(control.time, "monotonic", side_effect=lambda: clock), \
                        patch.object(control.time, "sleep", side_effect=sleep):
                    if case == "user_stop":
                        control.run_position(30, cfg, dev, capture=False)
                    else:
                        with self.assertRaises(vel.DiagnosticStop) as caught:
                            control.run_position(30, cfg, dev, capture=False)
                        self.assertEqual(caught.exception.reason, case)
                        self.assertTrue(caught.exception.stop_reported)
                path = sorted((self.root / "logs").glob("m0_position_*.csv"))[-1]
                with path.open() as file:
                    *rows, stop = list(csv.DictReader(file))
                self.assertEqual(stop["stop_reason"], case)
                if case == "overspeed":
                    self.assertEqual(stop["controller_error"], str(CONTROLLER_ERROR_OVERSPEED))
                    self.assertAlmostEqual(float(rows[-1]["estimated_rpm"]), 51.0)
                    self.assertGreater(float(rows[-1]["encoder_count_rpm"]), 46.0)
                    self.assertTrue(rows[-1]["commanded_position_turns"])
                    self.assertEqual(rows[-1]["active_position_turns"], rows[-2]["commanded_position_turns"])
                self.assertEqual(stop["idle_confirmed"], "True")
                self.assertEqual(axis.requests, [AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE])
                self.assertEqual(axis.controller.input_vel, 0)
        self.assertFalse(list((self.root / "logs").glob("m0_capture_*.csv")))

    def test_control_position_late_write_skips_feedforward_and_watchdog(self):
        cfg = core.load_config()
        dev = self.control_board(cfg)
        axis = dev.axis0
        clock = 0.0
        active_velocity_writes = []

        class SlowController(NS):
            def __setattr__(self, name, value):
                nonlocal clock
                if axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                    if name == "input_pos":
                        clock += 0.026
                    elif name == "input_vel":
                        active_velocity_writes.append(value)
                super().__setattr__(name, value)

        axis.controller = SlowController(**vars(axis.controller))

        def sleep(delay):
            nonlocal clock
            clock += delay

        with patch.object(control, "confirm_motion"), \
                patch.object(torque, "admit_control_period", return_value=0.025), \
                patch.object(control.time, "monotonic", side_effect=lambda: clock), \
                patch.object(control.time, "sleep", side_effect=sleep):
            with self.assertRaisesRegex(vel.DiagnosticStop, "feed-forward deadline"):
                control.run_position(30, cfg, dev)
        self.assertEqual(active_velocity_writes, [])
        self.assertEqual(axis.watchdog_feed.call_count, 2)
        self.assertEqual(axis.requests, [AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE])

    def configure_m1(self, *, encoder_present=True):
        # Synthetic settings exercise distinct M1 hardware and tuning.
        axis = copy.deepcopy(self.raw["axes"]["m0"])
        axis["hardware"].update(motor_kv=190, pole_pairs=5, cpr=4096, current_lim=8,
                                calibration_current=6)
        axis["velocity"]["vel_gain"] = 0.077
        axis["encoder_present"] = encoder_present
        if not encoder_present:
            axis["hardware"].update(cpr=None, bandwidth=None, calib_range=None,
                                    enable_phase_interpolation=None)
        self.raw["axes"]["m1"] = axis
        self.save()

    def board(self, cfg):
        return NS(axis0=Axis(), axis1=Axis(), vbus_voltage=24.0, ibus=0.15,
                  brake_resistor_armed=True, brake_resistor_saturated=False,
                  config=NS(**cfg["board"]), serial_number=int(cfg["odrive_serial_number"], 16),
                  hw_version_major=3, hw_version_minor=6, fw_version_major=0,
                  fw_version_minor=5, fw_version_revision=1)

    def test_as5048a_calibration_is_saved_only_after_success_and_reusable_after_reboot(self):
        cfg = core.load_config()
        for fail in (True, False):
            with self.subTest(fail=fail):
                dev = self.board(cfg)
                m1 = dev.axis1
                saves = []

                def save():
                    saves.append(copy.deepcopy(dev.axis0))

                def reboot():
                    dev.axis0 = copy.deepcopy(saves[-1])
                    dev.axis0.motor.is_calibrated = dev.axis0.motor.config.pre_calibrated
                    dev.axis0.encoder.is_ready = dev.axis0.encoder.config.pre_calibrated
                    dev.user_config_loaded = True
                    raise calibrate_m0.ChannelBrokenException()

                def step(axis, state, *_args):
                    self.assertIs(axis, dev.axis0)
                    self.assertTrue(dev.user_config_loaded)
                    self.assertEqual(axis.encoder.config.mode, ENCODER_MODE_SPI_ABS_AMS)
                    if state == AXIS_STATE_MOTOR_CALIBRATION:
                        axis.motor.is_calibrated = True
                    elif fail:
                        raise RuntimeError("SPI calibration failed")
                    else:
                        axis.encoder.is_ready = True
                        axis.encoder.config.offset = 7531
                        axis.encoder.config.offset_float = 0.25
                        axis.motor.config.direction = -1

                dev.save_configuration, dev.reboot = save, reboot
                with patch.object(calibrate_m0, "find_odrive", return_value=dev), \
                        patch.object(calibrate_m0, "confirm_motion"), \
                        patch.object(calibrate_m0, "wait_for_settling"), \
                        patch.object(calibrate_m0, "dump_errors"), \
                        patch.object(calibrate_m0.time, "sleep"), \
                        patch.object(calibrate_m0, "calibrate_step", side_effect=step):
                    if fail:
                        with self.assertRaisesRegex(RuntimeError, "SPI calibration failed"):
                            calibrate_m0.calibrate(cfg, spi_cs_gpio=4)
                    else:
                        calibrate_m0.calibrate(cfg, spi_cs_gpio=4)

                self.assertEqual(len(saves), 1 if fail else 2)
                self.assertFalse(saves[0].motor.config.pre_calibrated)
                setup = saves[0].encoder.config
                self.assertFalse(setup.pre_calibrated)
                self.assertEqual((setup.mode, setup.cpr, setup.abs_spi_cs_gpio_pin),
                                 (ENCODER_MODE_SPI_ABS_AMS, 16384, 4))
                self.assertEqual(dev.axis0.current_state, AXIS_STATE_IDLE)
                self.assertIs(dev.axis1, m1)
                if not fail:
                    self.assertIs(core.preflight(dev, cfg), dev.axis0)
                    self.assertEqual(dev.axis0.motor.config.direction, -1)
                    self.assertEqual(dev.axis0.encoder.config.offset, 7531)
                    self.assertEqual(dev.axis0.encoder.config.offset_float, 0.25)
                else:
                    self.assertEqual(dev.axis0.motor.config.direction, 0)

    def test_as5048a_clear_cli_delegates_without_requiring_cs(self):
        with patch.object(sys, "argv", ["calibrate_m0_x.py", "--clear-errors"]), \
                patch.object(calibrate_m0, "clear_errors") as clear, \
                patch.object(calibrate_m0, "calibrate") as calibrate:
            calibrate_m0.main(absolute=True)
        self.assertEqual(clear.call_args.args[0]["axis_index"], 0)
        calibrate.assert_not_called()

    def test_incremental_calibration_can_switch_from_faulted_spi_encoder(self):
        cfg = core.load_config()
        dev = self.board(cfg)
        core.configure_for_calibration(dev, cfg, spi_cs_gpio=4)
        dev.axis0.encoder.error = 1
        dev.save_configuration = Mock()

        def reboot():
            dev.axis0.encoder.error = 0
            dev.user_config_loaded = True

        dev.reboot = Mock(side_effect=reboot)
        with patch.object(calibrate_m0, "find_odrive", return_value=dev), \
                patch.object(calibrate_m0, "confirm_motion"), \
                patch.object(calibrate_m0, "wait_for_settling"), \
                patch.object(calibrate_m0.time, "sleep"), \
                patch.object(calibrate_m0, "calibrate_step") as step:
            calibrate_m0.calibrate(cfg)
        dev.save_configuration.assert_called_once()
        dev.reboot.assert_called_once()
        self.assertEqual(step.call_count, 2)
        self.assertEqual(dev.axis0.encoder.config.mode, ENCODER_MODE_INCREMENTAL)
        self.assertEqual(dev.axis0.encoder.config.cpr, cfg["hardware"]["cpr"])
        self.assertFalse(dev.axis0.encoder.config.pre_calibrated)
        self.assertFalse(dev.axis0.motor.config.pre_calibrated)

    def test_legacy_load_keeps_m0_values(self):
        new = core.load_config()
        old = self.raw["axes"]["m0"]
        for group, options in core.FIRMWARE_OPTIONS.items():
            for key, (_, default) in options.items():
                del old[group][key]
                new[group][key] = default
        for key in ("controller", "anticogging", "sensorless", "hardware_verified", "encoder_present"):
            del old[key]
        old["hardware"].update(self.raw["board"])
        self.raw = {**old, "runtime": self.raw["runtime"],
                    "odrive_serial_number": self.raw["odrive_serial_number"]}
        self.save()
        self.assertEqual(core.load_config(), new)
        with self.assertRaisesRegex(ValueError, "Legacy.*M0"):
            core.load_config(1)

    def test_unverified_m1_is_read_only(self):
        with self.assertRaisesRegex(ValueError, "M1 hardware is unverified"):
            core.load_config(1)
        self.assertEqual(core.load_config(1, read_only=True)["axis_index"], 1)

    def test_axis_selection_calibration_and_isolation(self):
        self.configure_m1()
        cfg = core.load_config(1)
        dev = self.board(cfg)
        original_m0 = copy.deepcopy(dev.axis0.motor.config)
        with patch.object(calibrate_m0, "find_odrive", return_value=dev), \
                patch.object(calibrate_m0, "confirm_motion"), \
                patch.object(calibrate_m0, "wait_for_settling"), \
                patch.object(calibrate_m0, "calibrate_step") as step:
            calibrate_m0.calibrate(cfg)
        self.assertEqual(dev.axis0.motor.config, original_m0)
        self.assertEqual(dev.axis0.requests, [])
        self.assertEqual(dev.axis1.encoder.config.cpr, 4096)
        self.assertEqual(dev.axis1.motor.config.pole_pairs, 5)
        self.assertEqual(dev.axis1.controller.config.load_encoder_axis, 1)
        self.assertTrue(all(call.args[0] is dev.axis1 for call in step.call_args_list))
        self.assertEqual([call.args[1] for call in step.call_args_list],
                         [AXIS_STATE_MOTOR_CALIBRATION, AXIS_STATE_ENCODER_OFFSET_CALIBRATION])
        self.assertEqual(dev.axis1.current_state, AXIS_STATE_IDLE)
        self.assertIs(core.preflight(dev, cfg), dev.axis1)
        dev.axis1.encoder.is_ready = False
        with self.assertRaisesRegex(RuntimeError, "M1 needs.*calibration"):
            core.preflight(dev, cfg)

    def test_validation(self):
        for group, key, value in (
            ("hardware", "cpr", 500.5), ("hardware", "pole_pairs", True),
            ("hardware", "calibration_current", self.raw["axes"]["m0"]["hardware"]["current_lim"] + 1),
            ("hardware", "torque_lim", -1),
            ("hardware", "current_control_bandwidth", 0), ("hardware", "enable_phase_interpolation", 1),
            ("controller", "enable_overspeed_error", False), ("controller", "inertia", math.nan),
            ("velocity", "sample_seconds", 0.01), ("velocity", "vel_limit_tolerance", 0.9),
            ("safety", "operating_max_v", 28), ("safety", "watchdog_timeout", 0.05),
            ("anticogging", "calib_vel_threshold", -1),
        ):
            with self.subTest(key=key):
                settings = self.raw["axes"]["m0"][group]
                old = settings[key]
                settings[key] = value
                self.save()
                with self.assertRaises(ValueError):
                    core.load_config()
                settings[key] = old

    def test_optional_capabilities_and_anticogging(self):
        cfg = core.load_config()
        axis = Axis()
        del axis.motor.config.torque_lim
        settings = core.firmware_options(axis, cfg)
        self.assertFalse(any(option[2] == "torque_lim" for option in settings))
        self.assertIn("motor.config.torque_lim: unsupported", sys.stdout.getvalue())
        cfg["hardware"]["torque_lim"] = 0.1
        with self.assertRaisesRegex(RuntimeError, "motor.config.torque_lim"):
            core.firmware_options(axis, cfg)
        self.assertFalse(hasattr(axis.motor.config, "torque_lim"))
        with self.assertRaisesRegex(RuntimeError, "no valid map"):
            core.check_anticogging(axis, enable=True)
        axis.controller.config.anticogging.calib_anticogging = True
        with self.assertRaisesRegex(RuntimeError, "calibration already active"):
            core.check_anticogging(axis, enable=False)

    def test_overspeed_elapsed_time_and_immediate_cutoffs(self):
        start = vel.overspeed(0, None, 105, 90, 100, 120)
        self.assertEqual(vel.overspeed(0.08, start, 105, 90, 100, 120), 0)
        self.assertIsNone(vel.overspeed(0.10, start, 95, 90, 100, 120))
        for args in ((0.16, start, 105, 90), (0, None, -121, 90), (0, None, 95, -101)):
            with self.assertRaises(vel.DiagnosticStop) as error:
                vel.overspeed(*args, 100, 120)
            self.assertEqual(error.exception.reason, "overspeed")

    def test_stall_elapsed_time_signed_and_resets(self):
        start = vel.stall(0, None, -25, 1, 30)
        self.assertEqual(vel.stall(0.20, start, -25, -1, 30), 0)
        self.assertIsNone(vel.stall(0.49, start, -25, -20, 30))
        self.assertIsNone(vel.stall(0.49, start, -10, 0, 30))
        with self.assertRaises(vel.DiagnosticStop) as error:
            vel.stall(0.51, start, -25, -1, 30)
        self.assertEqual(error.exception.reason, "stall")

    def test_csv_rpm_conversion_fault_and_cleanup(self):
        self.configure_m1()
        for index in (0, 1):
            cfg = core.load_config(index)
            dev = self.board(cfg)
            axis = core.select_axis(dev, cfg)
            core.configure_for_calibration(dev, cfg)

            def on_sleep(_dt, axis=axis, cpr=cfg["hardware"]["cpr"]):
                self.assertEqual(axis.controller.input_vel, -0.5)
                self.assertAlmostEqual(axis.controller.config.vel_limit, 50 / 60)
                self.assertTrue(axis.config.enable_watchdog)
                axis.encoder.shadow_count -= cpr // 20
                if axis.encoder.shadow_count < -(cpr // 20):
                    axis.controller.error = CONTROLLER_ERROR_OVERSPEED

            with patch.object(vel, "confirm_motion"), patch.object(vel, "dump_errors"), \
                    patch.object(vel.time, "sleep", side_effect=on_sleep), \
                    patch.object(vel.time, "monotonic", side_effect=iter(i / 10 for i in range(30))):
                with self.assertRaises(vel.DiagnosticStop):
                    vel.run(-30, cfg, dev)
            self.assertEqual(axis.requests, [AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE])
            self.assertFalse(axis.config.enable_watchdog)
            self.assertEqual(axis.controller.input_vel, 0)
            self.assertEqual(dev.axis1.requests if index == 0 else dev.axis0.requests, [])
            path, = (self.root / "logs").glob(f"m{index}_velocity_*-30rpm.csv")
            with path.open() as file:
                start, *samples, stop = list(csv.DictReader(file))
            self.assertEqual(start["axis"], str(index))
            self.assertEqual(start["target_rpm"], "-30")
            self.assertTrue(json.loads(start["config_json"])["config"]["hardware_verified"])
            self.assertTrue(samples)
            self.assertAlmostEqual(float(samples[1]["encoder_count_rpm"]), -30, delta=0.2)
            self.assertEqual(float(samples[1]["effective_current_lim_a"]), 8.0)
            self.assertNotIn("I Limit A", sys.stdout.getvalue())
            self.assertEqual(stop["stop_reason"], "overspeed")
            self.assertEqual(stop["controller_error"], str(CONTROLLER_ERROR_OVERSPEED))
            self.assertEqual(stop["idle_confirmed"], "True")

    def test_sensorless_without_encoder(self):
        self.configure_m1(encoder_present=False)
        cfg = core.load_config(1)
        dev = self.board(cfg)
        axis = dev.axis1
        del axis.encoder.shadow_count
        with patch.object(vel, "confirm_motion"), patch.object(vel, "calibrate_step") as step, \
                patch.object(vel.time, "sleep"), \
                patch.object(vel.time, "monotonic", side_effect=iter(i / 10 for i in range(100))):
            vel.run(-550, cfg, dev, sensorless=True)
        self.assertEqual(step.call_args.args[:2], (axis, AXIS_STATE_MOTOR_CALIBRATION))
        self.assertEqual(axis.requests, [AXIS_STATE_SENSORLESS_CONTROL, AXIS_STATE_IDLE])
        self.assertEqual(axis.motor.config.direction, 0)
        path, = (self.root / "logs").glob("m1_sensorless_velocity_*-550rpm.csv")
        with path.open() as file:
            start, sample, *_, stop = list(csv.DictReader(file))
        self.assertEqual(start["target_rpm"], "-550")
        self.assertEqual(float(sample["setpoint_rpm"]), -550)
        self.assertEqual(sample["encoder_count_rpm"], "")
        self.assertEqual(sample["encoder_delta_counts"], "")
        self.assertEqual(sample["effective_current_lim_a"], "8.0")
        self.assertEqual(stop["stop_reason"], "normal_completion")

    def test_cleanup_when_closed_loop_entry_times_out(self):
        cfg = core.load_config()
        dev = self.board(cfg)
        core.configure_for_calibration(dev, cfg)
        with patch.object(vel, "confirm_motion"), patch.object(vel, "dump_errors"), \
                patch.object(vel, "enter_closed_loop", side_effect=TimeoutError("entry failed")):
            with self.assertRaises(TimeoutError):
                vel.run(30, cfg, dev)
        self.assertEqual(dev.axis0.requests, [AXIS_STATE_IDLE])
        path, = (self.root / "logs").glob("m0_velocity_*.csv")
        with path.open() as file:
            start, stop = list(csv.DictReader(file))
        self.assertEqual(start["event"], "start")
        self.assertEqual(stop["stop_reason"], "timeout")

    def test_user_stop_and_unexpected_exception(self):
        cfg = core.load_config()
        for exc, reason in ((KeyboardInterrupt(), "user_stop"), (OSError("USB failure"), "fault")):
            dev = self.board(cfg)
            core.configure_for_calibration(dev, cfg)
            with patch.object(vel, "confirm_motion"), patch.object(vel, "dump_errors"), \
                    patch.object(vel.time, "sleep", side_effect=exc):
                if reason == "user_stop":
                    vel.run(30, cfg, dev)
                else:
                    with self.assertRaisesRegex(OSError, "USB failure"):
                        vel.run(30, cfg, dev)
            self.assertEqual(dev.axis0.requests, [AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE])
            self.assertFalse(dev.axis0.config.enable_watchdog)
            path = sorted((self.root / "logs").glob("m0_velocity_*.csv"))[-1]
            with path.open() as file:
                stop = list(csv.DictReader(file))[-1]
            self.assertEqual(stop["stop_reason"], reason)
            self.assertEqual(stop["idle_confirmed"], "True")

    def test_gui_velocity_cancellation_requests_idle_and_logs_stop(self):
        cfg = core.load_config()
        dev = self.board(cfg)
        core.configure_for_calibration(dev, cfg)
        cancelled = Event()

        def check_cancel():
            if cancelled.is_set():
                raise KeyboardInterrupt

        with patch.object(vel.time, "sleep", side_effect=lambda _delay: cancelled.set()):
            vel.run(-30, cfg, dev, confirm=lambda _: None, check_cancel=check_cancel)
        self.assertEqual(dev.axis0.requests, [AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE])
        self.assertFalse(dev.axis0.config.enable_watchdog)
        path, = (self.root / "logs").glob("m0_velocity_*.csv")
        with path.open() as file:
            stop = list(csv.DictReader(file))[-1]
        self.assertEqual((stop["stop_reason"], stop["idle_confirmed"]), ("user_stop", "True"))

    def test_gui_ram_calibration_refuses_spi_and_cancellation_requests_idle(self):
        cfg = core.load_config()
        dev = self.board(cfg)
        axis = dev.axis0
        confirm = Mock()
        with patch.object(calibrate_m0, "find_odrive", return_value=dev), \
                patch.object(calibrate_m0, "save_and_reboot") as flash:
            axis.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
            with self.assertRaisesRegex(RuntimeError, "requires an existing A/B"):
                calibrate_m0.calibrate(cfg, allow_encoder_switch=False, confirm=confirm)
            confirm.assert_not_called()
            self.assertEqual(axis.requests, [])
            axis.encoder.config.mode = ENCODER_MODE_INCREMENTAL

            def check_cancel():
                if axis.current_state == AXIS_STATE_MOTOR_CALIBRATION:
                    raise KeyboardInterrupt

            with self.assertRaises(KeyboardInterrupt):
                calibrate_m0.calibrate(cfg, allow_encoder_switch=False,
                                       confirm=confirm, check_cancel=check_cancel)
            flash.assert_not_called()
        self.assertEqual(axis.requests, [AXIS_STATE_MOTOR_CALIBRATION, AXIS_STATE_IDLE])
        self.assertEqual(axis.motor.config.direction, 0)

    def test_read_only_capability_command(self):
        dev = self.board(core.load_config())
        del dev.axis1.controller.config.pos_gain
        with patch.object(sys, "argv", ["calibrate_m0.py", "--axis", "1", "--check-config"]), \
                patch.object(calibrate_m0, "find_odrive", return_value=dev):
            calibrate_m0.main()
        self.assertIn("M1 motor.config.torque_lim: available", sys.stdout.getvalue())
        self.assertIn("M1 controller.config.pos_gain: unsupported", sys.stdout.getvalue())
        self.assertEqual(dev.axis1.requests, [])
        self.assertFalse(dev.axis1.controller.config.enable_vel_limit)

    def test_terminal_arrow_sequences_and_double_tap(self):
        keyboard = teleop.TerminalKeyboard(-1)
        self.assertEqual(keyboard.decode(b"\x1b[", 0), ())
        self.assertEqual(keyboard.decode(b"A\x1bOB\x1b[1;2Ablah", 0.05), ("forward", "reverse"))
        self.assertEqual(keyboard.decode(b" ", 0.1), ())
        for now in (0.60, 0.63, 0.66, 0.69):
            self.assertEqual(keyboard.decode(b" ", now), ())
        self.assertEqual(keyboard.decode(b" ", 1.5), ())
        self.assertEqual(keyboard.decode(b"\x1b[A \x1b[B", 1.75), ("brake",))
        self.assertEqual(keyboard.decode(b"  ", 1.85), ())
        self.assertEqual(keyboard.decode(b"  ", 2.65), ("brake",))
        with self.assertRaises(KeyboardInterrupt):
            keyboard.decode(b"\x03\x1b[A", 3.0)

    def test_teleop_repeats_accelerate_and_latch_then_reverse_after_settling(self):
        cfg = core.load_config()
        events = ()
        reports = Mock()
        live = teleop.TeleopRPM(12, cfg, action="velocity", read_events=lambda _: events, notify=reports)

        def update(now, *keys, ramped=0, estimate=0, count=0):
            nonlocal events
            events = keys
            return live.update(now, ramped, estimate, count, 60)

        self.assertEqual(update(0), 0)
        initial = update(0.1, "forward")
        self.assertEqual(initial, cfg["velocity"]["min_command_rpm"])
        early = update(0.6, "forward")
        late = update(1.1, "forward")
        self.assertGreater(late - early, early - initial)
        self.assertEqual(update(1.13, *("forward",) * 10), 12)
        self.assertEqual(update(1.9), 12)

        self.assertEqual(update(2.0, "reverse", ramped=12, estimate=12, count=12), 0)
        self.assertEqual(live.requested, -initial)
        self.assertEqual(update(2.13, "reverse", estimate=0, count=12), 0)
        self.assertEqual(update(2.25, estimate=12, count=0), 0)
        self.assertEqual(update(2.5), 0)
        self.assertEqual(update(2.75), 0)
        self.assertLess(update(3.01), -initial)
        self.assertTrue(live.reversed)
        self.assertEqual([call.args[1] for call in reports.call_args_list], [
            {"state": "holding", "direction": ""},
            {"state": "driving", "direction": "forward"},
            {"state": "reversing", "direction": "reverse"},
            {"state": "driving", "direction": "reverse"},
        ])

    def test_teleop_brake_keeps_guard_until_both_speed_readings_settle(self):
        cfg = core.load_config()
        events = iter([("forward",) * 20, ("brake", "reverse")])
        live = teleop.TeleopRPM(50, cfg, action="velocity", read_events=lambda _: next(events, ()))
        prior = live.update(0, 0, 0, 0, 60)
        self.assertEqual(live.update(0.1, prior, prior, prior, 60), 0)
        for now, estimate, count in ((0.2, 0, 0), (0.3, 0, 4), (0.4, 0, 0),
                                     (0.5, 4, 0), (0.6, 0, None), (0.7, 0, 0), (0.84, 0, 0)):
            self.assertEqual(live.update(now, 0, estimate, count, 60), 0)
            self.assertTrue(live.braking)
            self.assertEqual(live.guard_rpm(0), prior)
        self.assertEqual(live.update(0.86, 0, 0, 0, 60), 0)
        self.assertFalse(live.braking)
        self.assertEqual(live.guard_rpm(0), 0)

    def test_noninteractive_cli_stops_before_discovery(self):
        stdin = Mock()
        stdin.isatty.return_value = False
        with patch.object(sys, "stdin", stdin), patch.object(sys, "argv", ["teleop_motor.py"]), \
                patch("sys.stderr", new=io.StringIO()), patch.object(teleop, "run_teleop") as run:
            with self.assertRaises(SystemExit):
                teleop.main()
        run.assert_not_called()

    def test_terminal_restores_pty_after_fragmented_input_and_eof(self):
        master, slave = os.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        keyboard = teleop.TerminalKeyboard(slave)
        original = termios.tcgetattr(slave)
        with self.assertRaisesRegex(EOFError, "disconnected"):
            with keyboard.terminal():
                self.assertFalse(termios.tcgetattr(slave)[3] & termios.ICANON)
                os.write(master, b"\x1b[")
                self.assertEqual(keyboard.read_events(0), ())
                os.write(master, b"A ")
                self.assertEqual(keyboard.read_events(0.05), ("forward",))
                os.write(master, b" ")
                self.assertEqual(keyboard.read_events(0.15), ("brake",))
                with patch.object(teleop.select, "select", return_value=([slave], [], [])), \
                        patch.object(teleop.os, "read", return_value=b""):
                    keyboard.read_events(0.2)
        self.assertEqual(termios.tcgetattr(slave), original)

    def test_teleop_cli_uses_confirm_then_restores_terminal(self):
        master, slave = os.openpty()
        self.addCleanup(os.close, master)
        with os.fdopen(slave, "r") as stdin:
            original = termios.tcgetattr(slave)
            argv = ["teleop_motor.py", "--mode", "control", "--max-rpm", "80"]

            def test_run(cfg, *, confirm, read_events, mode, ceiling_rpm):
                self.assertEqual((cfg["axis_index"], mode, ceiling_rpm), (0, "control", 80))
                self.assertEqual(termios.tcgetattr(slave), original)
                confirm("POSITION M0 at zero")
                self.assertFalse(termios.tcgetattr(slave)[3] & termios.ICANON)
                os.write(master, b"\x1b[A")
                self.assertEqual(read_events(0), ())
                os.write(master, b"\x1b[B")
                self.assertEqual(read_events(0.1), ("reverse",))
                raise KeyboardInterrupt

            with patch.object(sys, "stdin", stdin), patch.object(sys, "argv", argv), \
                    patch.object(teleop, "confirm_motion") as confirm, \
                    patch.object(teleop, "run_teleop", side_effect=test_run):
                with self.assertRaises(KeyboardInterrupt):
                    teleop.main()
            confirm.assert_called_once_with("POSITION M0 at zero")
            self.assertEqual(termios.tcgetattr(slave), original)

    def test_teleop_validates_mode_axis_and_ceiling_before_discovery(self):
        cfg = core.load_config()
        read = Mock(return_value=())
        with self.assertRaisesRegex(ValueError, "finite starting RPM"):
            teleop.run_teleop(cfg, read_events=read, ceiling_rpm=float("nan"))
        with self.assertRaisesRegex(ValueError, "ceiling"):
            teleop.run_teleop(cfg, read_events=read, mode="control",
                              ceiling_rpm=cfg["velocity"]["max_command_rpm"] + 1)
        self.configure_m1()
        cfg1 = core.load_config(1)
        with self.assertRaisesRegex(ValueError, "require M0"):
            teleop.run_teleop(cfg1, read_events=read, mode="torque")
        cfg1["velocity"]["max_command_rpm"] = 40
        with patch.object(vel, "run") as run:
            teleop.run_teleop(cfg1, read_events=read)
        self.assertEqual(run.call_args.args[:2], (40, cfg1))
        self.assertEqual(run.call_args.kwargs["live"].start_rpm, 0)
        read.assert_not_called()

    def test_control_teleop_selects_one_runner_for_the_approved_ceiling(self):
        cfg = core.load_config()
        cfg["velocity"]["max_command_rpm"] = 2200
        cases = ((199, "position"), (200, "torque"), (2200, "torque"))
        for ceiling, mode in cases:
            with self.subTest(ceiling=ceiling), patch.object(control, "run_position") as position, \
                    patch.object(torque, "run") as torque_run:
                teleop.run_teleop(cfg, read_events=lambda _: ("forward",), mode="control", ceiling_rpm=ceiling)
                runner, unused = (position, torque_run) if mode == "position" else (torque_run, position)
                runner.assert_called_once()
                unused.assert_not_called()
                self.assertEqual(runner.call_args.args[0], ceiling)
                live = runner.call_args.kwargs["live"]
                self.assertEqual((live.mode, live.start_rpm, live.ceiling), (mode, 0, ceiling))
                self.assertLess(live.update(0, 0, 0, 0, 600), 200)
                for tick in range(1, 180):
                    target = live.update(tick * 0.03, 0, 0, 0, 600)
                self.assertEqual(target, ceiling)
                self.assertEqual(live.mode, mode)

    def test_all_teleop_modes_brake_hold_resume_and_idle_on_failure(self):
        for mode, pattern in (("velocity", "teleop"), ("control", "position"), ("torque", "torque")):
            for reason in ("user_stop", "fault", "timeout", "operator_link_lost"):
                with self.subTest(mode=mode, reason=reason):
                    self.check_teleop_motion(mode, pattern, reason)

    def check_teleop_motion(self, mode, pattern, reason):
        cfg = core.load_config()
        cfg["velocity"].update(vel_ramp_rate=3, sample_seconds=0.025)
        dev = self.control_board(cfg)
        axis = dev.axis0
        controller = axis.controller
        old_input_mode = controller.config.input_mode
        clock = 0.0
        braking = False
        resume = False
        counts_per_turn = cfg["hardware"]["cpr"]
        moving_until = math.inf if reason == "timeout" else 0.62
        scheduled = iter([(0.05, ("forward",) * 50),
                          (0.30, ("brake", "reverse")),
                          (0.45, ("reverse",)),
                          (1.00, ("reverse",)),
                          (1.80, ("reverse",) * 50)])
        when, events = next(scheduled)
        confirm, notify = Mock(), Mock()

        def read_events(now):
            nonlocal when, events, braking, resume
            if reason == "operator_link_lost" and braking:
                raise EOFError("Terminal input disconnected")
            if now + 1e-9 < when:
                return ()
            upcoming = events
            when, events = next(scheduled, (math.inf, ()))
            if "brake" in upcoming:
                self.assertGreater(controller.input_torque if mode == "torque" else controller.input_vel, 0)
                braking = True
            if reason == "user_stop" and now >= 1.79:
                resume = True
            return upcoming

        def advance(dt):
            nonlocal clock
            if axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                if clock < 0.05 - 1e-9:
                    self.assertEqual(controller.input_vel, 0)
                    self.assertEqual(controller.input_torque, 0)
                if braking and not resume:
                    if mode == "velocity":
                        self.assertEqual(controller.input_vel, 0.0)
                        self.assertEqual(controller.vel_integrator_torque, 0.0)
                        if clock < moving_until:
                            self.assertEqual(controller.config.input_mode, INPUT_MODE_PASSTHROUGH)
                        controller.vel_setpoint = 0.0
                    if mode == "control":
                        self.assertEqual(controller.input_vel, 0.0)
                        self.assertAlmostEqual(controller.input_pos, axis.encoder.pos_estimate)
                    if reason == "fault":
                        axis.error = AXIS_ERROR_MOTOR_FAILED
                        axis.motor.error = MOTOR_ERROR_CURRENT_LIMIT_VIOLATION
                    axis.encoder.vel_estimate = 18 / 60 if clock < moving_until else 0
                    axis.encoder.pos_estimate += axis.encoder.vel_estimate * dt
                elif mode == "control":
                    axis.encoder.pos_estimate = controller.input_pos
                    axis.encoder.vel_estimate = controller.input_vel
                elif mode == "velocity":
                    controller.vel_setpoint = slew_rpm(controller.vel_setpoint, controller.input_vel, 3 * dt)
                    axis.encoder.vel_estimate = controller.vel_setpoint
                    axis.encoder.pos_estimate += axis.encoder.vel_estimate * dt
                else:
                    axis.encoder.vel_estimate = (-18 if resume else 18 if clock >= 0.1 else 0) / 60
                    axis.encoder.pos_estimate += axis.encoder.vel_estimate * dt
                axis.encoder.shadow_count = round(axis.encoder.pos_estimate * counts_per_turn)
            clock += dt

        def cancel():
            self.assertLess(clock, 4, "Teleop failed to terminate in the expected time")
            if reason == "user_stop" and clock >= 2.15:
                raise KeyboardInterrupt

        def run():
            teleop.run_teleop(cfg, mode=mode, dev=dev, read_events=read_events,
                              confirm=confirm, check_cancel=cancel, notify=notify)

        with patch.object(torque, "admit_control_period", return_value=0.025), \
                patch.object(torque, "LOG_QUEUE_SIZE", 256), \
                patch.object(vel, "dump_errors"), \
                patch.object(torque.time, "monotonic", side_effect=lambda: clock), \
                patch.object(torque.time, "sleep", side_effect=advance):
            if reason == "user_stop":
                run()
            else:
                failure = TimeoutError if reason == "timeout" else vel.DiagnosticStop
                with self.assertRaises(failure):
                    run()

        self.assertIn("target +0", confirm.call_args.args[0])
        self.assertIn("live ceiling 50 RPM", confirm.call_args.args[0])
        self.assertEqual(axis.requests, [AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE])
        self.assertFalse(axis.config.enable_watchdog)
        self.assertEqual(controller.config.input_mode, old_input_mode)
        self.assertEqual(dev.axis1.requests, [])
        path = sorted((self.root / "logs").glob(f"m0_{pattern}_*.csv"))[-1]
        with path.open() as stream:
            start, *samples, stop = list(csv.DictReader(stream))
        self.assertEqual(json.loads(start["config_json"])["trial"]["live_ceiling_rpm"], 50)
        self.assertTrue(any(float(row["setpoint_rpm"]) > 0 for row in samples))
        at_brake = [row for row in samples if row["target_sequence"] == "2"]
        if reason in ("user_stop", "timeout"):
            self.assertTrue(at_brake)
            self.assertLessEqual(float(at_brake[0]["elapsed_s"]), 0.33)
            self.assertEqual(float(at_brake[0]["setpoint_rpm"]), 0)
            self.assertTrue(any(float(row["estimated_rpm"]) > 15 for row in at_brake))
            if mode == "torque":
                self.assertTrue(any(float(row["command_torque_nm"]) < 0 for row in at_brake))
        if reason == "user_stop":
            negative = [row for row in samples if float(row["setpoint_rpm"]) < 0]
            self.assertTrue(negative)
            self.assertGreaterEqual(float(negative[0]["elapsed_s"]), 1.79)
        self.assertEqual((stop["stop_reason"], stop["idle_confirmed"]), (reason, "True"))
        lifecycle = [call.args[0] for call in notify.call_args_list if call.args[0] != "teleop_state"]
        self.assertEqual(lifecycle, ["csv", "running", "trial_result"])
        states = [call.args[1]["state"] for call in notify.call_args_list if call.args[0] == "teleop_state"]
        self.assertEqual(states, ["holding", "driving", "braking"] +
                         (["holding", "driving"] if reason == "user_stop" else []))

    def test_torque_tracks_signed_speed_and_recovers_after_load_step(self):
        for direction in (1, -1):
            with self.subTest(direction=direction):
                cfg = core.load_config()
                cfg["velocity"]["vel_ramp_rate"] = 0.5
                dev = self.board(cfg)
                core.configure_for_calibration(dev, cfg)
                axis = dev.axis0
                c = axis.controller.config
                axis.motor.config.torque_lim = 0.08
                c.enable_overspeed_error = False
                c.anticogging.anticogging_enabled = True
                original = vars(c).copy()
                watchdog_timeout = axis.config.watchdog_timeout = 2.0
                clock = turns = speed = 0.0
                samples = []

                def step(dt):
                    nonlocal clock, turns, speed
                    command = axis.controller.input_torque
                    self.assertEqual(c.control_mode, CONTROL_MODE_TORQUE_CONTROL)
                    self.assertEqual(c.input_mode, INPUT_MODE_PASSTHROUGH)
                    self.assertEqual(axis.controller.input_vel, 0)
                    self.assertTrue(axis.config.enable_watchdog)
                    self.assertEqual(axis.config.watchdog_timeout, 0.15)
                    self.assertTrue(all(getattr(c, key) for key in core.PROTECTIONS))
                    self.assertAlmostEqual(c.vel_limit * 60, 50)
                    self.assertFalse(c.anticogging.anticogging_enabled)
                    self.assertEqual(axis.motor.config.torque_lim, 0.08)
                    self.assertLessEqual(abs(command), min(0.08, axis.motor.effective_current_lim
                                                           * axis.motor.config.torque_constant))
                    samples.append((clock, speed, command))
                    load = 0.0
                    if clock >= 2.5:
                        load = direction * 0.025 * max(0.0, min(1.0, (7.4 - clock) / 2.4))
                    gain = c.vel_gain
                    applied = max((-c.vel_limit - speed) * gain,
                                  min((c.vel_limit - speed) * gain, command))
                    active_cap = min(axis.motor.config.torque_lim,
                                     axis.motor.effective_current_lim * axis.motor.config.torque_constant)
                    applied = max(-active_cap, min(active_cap, applied))
                    speed += dt * (applied - 0.02 * speed - load) / 0.01
                    clock += dt
                    if 8.4 <= clock < 9.2:
                        speed = direction * 0.72
                    axis.motor.effective_current_lim = 0.2 if 7.6 <= clock < 8.1 else 8.0
                    axis.motor.current_control.Iq_setpoint = applied / axis.motor.config.torque_constant
                    turns += speed * dt
                    axis.encoder.vel_estimate = speed
                    axis.encoder.shadow_count = round(turns * cfg["hardware"]["cpr"])
                    if clock >= 12.0:
                        raise KeyboardInterrupt

                with patch.object(torque, "confirm_motion"), \
                        patch.object(torque, "LOG_QUEUE_SIZE", 512), \
                        patch.object(torque, "admit_control_period", return_value=0.025), \
                        patch.object(torque.time, "monotonic", side_effect=lambda: clock), \
                        patch.object(torque.time, "sleep", side_effect=step):
                    torque.run(direction * 30, cfg, dev)
                self.assertEqual(samples[0], (0, 0, 0))
                for check_time in (2.4, 4.9, 11.8):
                    _, reached_speed, _ = min(samples, key=lambda row: abs(row[0] - check_time))
                    self.assertAlmostEqual(reached_speed * 60, direction * 30, delta=2.0, msg=f"at {check_time} s")
                self.assertTrue(any(direction * command > 0.03 for t, _, command in samples if 3 < t < 4.9))
                self.assertTrue(any(direction * command < 0 for t, _, command in samples if 8.4 <= t < 9.2))
                self.assertTrue(any(abs(command) <= 0.012 for t, _, command in samples if 7.6 <= t < 8.1))
                self.assertEqual(axis.requests, [AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE])
                self.assertEqual(dev.axis1.requests, [])
                self.assertEqual(axis.controller.input_torque, 0)
                self.assertEqual(axis.config.watchdog_timeout, watchdog_timeout)
                self.assertFalse(axis.config.enable_watchdog)
                self.assertTrue(c.anticogging.anticogging_enabled)
                self.assertEqual(vars(c), original)
                path, = (self.root / "logs").glob(f"m0_torque_*_{direction * 30:+g}rpm.csv")
                with path.open() as file:
                    start, *rows, stop = list(csv.DictReader(file))
                trial = json.loads(start["config_json"])["trial"]
                self.assertEqual(trial["torque_limit_nm"], 0.10)
                self.assertEqual(trial["input_mode"], "PASSTHROUGH")
                self.assertIsNone(trial["torque_ramp_rate_nm_per_s"])
                self.assertEqual(stop["stop_reason"], "user_stop")
                self.assertAlmostEqual(float(stop["elapsed_s"]), 12.0)
                self.assertEqual(len(rows), 479)
                self.assertTrue(all(abs(float(r["loop_interval_s"]) - 0.025) < 1e-9 for r in rows))
                self.assertEqual(stop["idle_confirmed"], "True")
                self.assertTrue(any(float(row["active_torque_limit_nm"]) < 0.012 for row in rows))
                self.assertAlmostEqual(float(rows[-1]["setpoint_rpm"]), direction * 30)

    def test_torque_pi_uses_measured_dt_and_recovers_from_saturation(self):
        command, integral = torque.pi_step(0.1, 0.1, 0.0, 0.2, 0.5, 0.10)
        self.assertAlmostEqual(command, 0.025)
        self.assertAlmostEqual(integral, 0.005)
        for _ in range(100):
            command, integral = torque.pi_step(2, 0.07, integral, 0.2, 0.5, 0.10)
            self.assertEqual(command, 0.10)
        self.assertAlmostEqual(integral, 0.005)
        command, integral = torque.pi_step(-0.1, 0.1, integral, 0.2, 0.5, 0.04)
        self.assertAlmostEqual(command, -0.02)
        self.assertAlmostEqual(integral, 0)

    def test_torque_pi_limits_slew_and_prioritizes_live_cap(self):
        for direction in (1, -1):
            with self.subTest(direction=direction):
                command, integral = torque.pi_step(
                    direction, 0.025, 0, 0.2, 0.5, 1.0, previous_command=0, slew_rate=1.0,
                )
                self.assertAlmostEqual(command, direction * 0.025)
                self.assertEqual(integral, 0)
                command, _ = torque.pi_step(
                    -direction, 0.025, 0, 0.2, 0.5, 0.04, previous_command=direction * 0.4, slew_rate=1.0,
                )
                self.assertAlmostEqual(command, direction * 0.04)

    def test_torque_rate_admission_times_reads_while_idle(self):
        for latency, expected in ((0.005, 1 / 40), (0.014, 1 / 30), (0.018, None)):
            with self.subTest(latency=latency):
                cfg = core.load_config()
                dev = self.board(cfg)
                clock = 0.0
                reads = []
                core.configure_for_calibration(dev, cfg)

                def advance(dt):
                    nonlocal clock
                    clock += dt

                def read(_dev, axis, *, expected_state):
                    self.assertEqual(axis.current_state, expected_state)
                    self.assertEqual(dev.axis1.current_state, AXIS_STATE_IDLE)
                    reads.append(clock)
                    advance(latency)

                with patch.object(torque.time, "monotonic", side_effect=lambda: clock), \
                        patch.object(torque.time, "sleep", side_effect=advance), \
                        patch.object(torque, "read_feedback", side_effect=read):
                    if expected is None:
                        with self.assertRaisesRegex(RuntimeError, "USB.*30 Hz"):
                            torque.run(30, cfg, dev)
                    else:
                        self.assertEqual(torque.admit_control_period(dev, dev.axis0), expected)
                        self.assertAlmostEqual(reads[-1] - reads[-2], expected)
                        self.assertGreaterEqual(clock, 2)
                self.assertEqual(dev.axis0.requests, [])

    def test_torque_refuses_watchdog_shorter_than_two_periods(self):
        cfg = core.load_config()
        cfg["safety"]["watchdog_timeout"] = 0.06
        dev = self.board(cfg)
        core.configure_for_calibration(dev, cfg)
        with patch.object(torque, "admit_control_period", return_value=1 / 30), \
                patch.object(torque, "confirm_motion") as confirm:
            with self.assertRaisesRegex(RuntimeError, "watchdog must exceed two control periods"):
                torque.run(30, cfg, dev)
        confirm.assert_not_called()
        self.assertEqual(dev.axis0.requests, [])

    def test_torque_limiter_keeps_the_trip_and_tapers_without_reducing_the_trial_cap(self):
        v = {"controller_min_limit_rpm": 20, "controller_headroom_rpm": 20,
             "vel_limit_tolerance": 1.2}
        cap = 1.0
        for rpm in (1000, -1000):
            ceiling, trip = torque.firmware_limits(rpm, v, 1210, cap, 0.40)
            self.assertAlmostEqual(ceiling, 1150)
            self.assertAlmostEqual(trip, 1224)
            self.assertAlmostEqual((ceiling - abs(rpm)) * 0.40 / 60, cap)
            self.assertAlmostEqual((ceiling - 1169) * 0.40 / 60, -0.1266666667)
        ceiling, trip = torque.firmware_limits(30, v, 46, cap, 0.40)
        self.assertAlmostEqual(ceiling, 50)
        self.assertAlmostEqual(trip, 60)
        self.assertAlmostEqual(torque.available_drive_torque(cap, ceiling, 0.40, 0), 1 / 3)
        self.assertAlmostEqual(torque.available_drive_torque(cap, ceiling, 0.40, 30), 2 / 15)

    def test_torque_overspeed_ratio_programs_and_logs_trip_in_both_directions(self):
        for rpm, ratio, expected_trip, expected_cap in (
                (1000, None, 1224, 1.0),
                (1000, 1.20, 1224, 1.0),
                (1000, 1.10, 1122, 1.0),
                (-1000, 1.10, 1122, 1.0)):
            with self.subTest(rpm=rpm, ratio=ratio):
                cfg = core.load_config()
                cfg["velocity"]["vel_limit_tolerance"] = 1.30 if ratio is not None else 1.20
                cfg["torque"]["torque_limit_nm"] = 1.0
                if ratio is not None:
                    cfg["torque"]["overspeed_ratio"] = ratio
                dev = self.board(cfg)
                core.configure_for_calibration(dev, cfg)
                axis = dev.axis0
                c = axis.controller.config
                original = (c.vel_limit, c.vel_limit_tolerance)
                clock = 0.0

                def step(dt):
                    nonlocal clock
                    clock += dt
                    self.assertTrue(c.enable_overspeed_error)
                    self.assertAlmostEqual(c.vel_limit * c.vel_limit_tolerance * 60, expected_trip)
                    self.assertAlmostEqual(axis.motor.config.torque_lim, expected_cap)
                    axis.controller.error = CONTROLLER_ERROR_OVERSPEED

                with patch.object(torque, "confirm_motion") as confirm, patch.object(torque, "admit_control_period", return_value=0.025), \
                        patch.object(torque.time, "monotonic", side_effect=lambda: clock), \
                        patch.object(torque.time, "sleep", side_effect=step):
                    with self.assertRaisesRegex(torque.DiagnosticStop, "ODrive reported a fault"):
                        torque.run(rpm, cfg, dev)
                self.assertIn(f"firmware overspeed trip {expected_trip} RPM", confirm.call_args.args[0])
                self.assertEqual((c.vel_limit, c.vel_limit_tolerance), original)
                self.assertEqual(axis.requests, [AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE])
                self.assertEqual(dev.axis1.requests, [])
                path = sorted((self.root / "logs").glob(f"m0_torque_*_{rpm:+g}rpm.csv"))[-1]
                with path.open() as file:
                    start, stop = list(csv.DictReader(file))
                trial = json.loads(start["config_json"])["trial"]
                self.assertEqual(trial["overspeed_ratio"], ratio if ratio is not None else 1.20)
                self.assertAlmostEqual(trial["firmware_overspeed_rpm"], expected_trip)
                self.assertEqual(stop["stop_reason"], "overspeed")
                self.assertEqual(stop["controller_error"], str(CONTROLLER_ERROR_OVERSPEED))
                self.assertEqual(stop["idle_confirmed"], "True")

    def test_torque_overspeed_ratio_defaults_to_the_shared_tolerance(self):
        cfg = core.load_config()
        cfg["velocity"]["vel_limit_tolerance"] = 1.35
        cfg["torque"].pop("overspeed_ratio", None)
        self.assertEqual(torque.validate_trial(1000, cfg)["overspeed_ratio"], 1.35)
        cfg["torque"]["overspeed_ratio"] = 1.0
        self.assertEqual(torque.validate_trial(1000, cfg)["overspeed_ratio"], 1.0)

    def test_torque_confirmation_shows_taper_and_rechecks_live_limit(self):
        cfg = core.load_config()
        cfg["torque"]["torque_limit_nm"] = 1.0
        dev = self.board(cfg)
        core.configure_for_calibration(dev, cfg)
        axis = dev.axis0

        def confirm(description):
            self.assertIn("trial torque cap 1 N·m", description)
            self.assertIn("available at target 0.133 N·m", description)
            self.assertEqual(axis.requests, [])
            axis.motor.config.torque_lim = 0.02

        with patch.object(torque, "confirm_motion", side_effect=confirm), \
                patch.object(torque, "admit_control_period", return_value=0.025):
            with self.assertRaisesRegex(RuntimeError, "changed during confirmation"):
                torque.run(30, cfg, dev)
        self.assertEqual(axis.requests, [])

    def test_torque_constant_override_applies_during_calibration_and_matches_live(self):
        hardware = self.raw["axes"]["m0"]["hardware"]
        for value in (0, -0.06, True, math.inf, "0.06"):
            hardware["torque_constant"] = value
            self.save()
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "torque_constant"):
                core.load_config()
        for value in (None, 0.060):
            hardware["torque_constant"] = value
            self.save()
            cfg = core.load_config()
            dev = self.board(cfg)
            core.configure_for_calibration(dev, cfg)
            expected = 8.27 / hardware["motor_kv"] if value is None else value
            self.assertAlmostEqual(dev.axis0.motor.config.torque_constant, expected)
            self.assertIs(core.preflight(dev, cfg), dev.axis0)
        dev.axis0.motor.config.torque_constant = 8.27 / hardware["motor_kv"]
        with self.assertRaisesRegex(RuntimeError, "Motor torque constant differs"):
            core.preflight(dev, cfg)
        hardware.pop("torque_constant")
        self.save()
        self.assertAlmostEqual(core.torque_constant(core.load_config()), 8.27 / hardware["motor_kv"])

    def test_torque_ramp_holds_integral_and_waits_for_applied_saturation_before_stall(self):
        for direction in (1, -1):
            with self.subTest(direction=direction):
                cfg = core.load_config()
                cfg["hardware"]["torque_constant"] = 0.060
                cfg["controller"]["torque_ramp_rate"] = 0.5
                cfg["torque"]["kp"] = 0.90
                cfg["torque"]["torque_limit_nm"] = 1.0
                dev = self.board(cfg)
                core.configure_for_calibration(dev, cfg)
                axis = dev.axis0
                c = axis.controller.config
                c.torque_ramp_rate = 0.25
                initial_mode = c.input_mode
                axis.motor.config.direction = -1
                clock = applied = 0.0
                sample_count = 0
                reached_cap = None

                def step(dt):
                    nonlocal clock, applied, sample_count, reached_cap
                    self.assertEqual(axis.controller.config.input_mode, INPUT_MODE_TORQUE_RAMP)
                    self.assertEqual(c.torque_ramp_rate, 0.5)
                    requested = axis.controller.input_torque * direction
                    if sample_count == 0:
                        self.assertEqual(requested, 0)
                    sample_count += 1
                    clock += dt
                    self.assertLess(clock, 4)
                    applied += max(-c.torque_ramp_rate * dt,
                                   min(c.torque_ramp_rate * dt, requested - applied))
                    # At 30 RPM the limiter's zero-drive point is 50 RPM.
                    limited = min(applied, 0.40 * 50 / 60)
                    if reached_cap is None and limited >= 0.40 * 50 / 60:
                        reached_cap = clock
                    axis.motor.current_control.Iq_setpoint = -direction * limited / axis.motor.config.torque_constant

                with patch.object(torque, "confirm_motion"), patch.object(torque, "admit_control_period", return_value=0.025), \
                        patch.object(torque, "LOG_QUEUE_SIZE", 512), \
                        patch.object(torque.time, "monotonic", side_effect=lambda: clock), \
                        patch.object(torque.time, "sleep", side_effect=step):
                    with self.assertRaisesRegex(torque.DiagnosticStop, "stalled"):
                        torque.run(direction * 30, cfg, dev)
                self.assertIsNotNone(reached_cap)
                self.assertGreaterEqual(clock - reached_cap, 0.45)
                self.assertEqual(c.input_mode, initial_mode)
                self.assertEqual(c.torque_ramp_rate, 0.25)
                self.assertEqual(axis.requests, [AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE])
                path, = (self.root / "logs").glob(f"m0_torque_*_{direction * 30:+g}rpm.csv")
                with path.open() as file:
                    start, *rows, stop = list(csv.DictReader(file))
                trial = json.loads(start["config_json"])["trial"]
                self.assertEqual(trial["input_mode"], "TORQUE_RAMP")
                self.assertEqual(trial["torque_ramp_rate_nm_per_s"], 0.5)
                self.assertAlmostEqual(trial["torque_constant_nm_per_a"], core.torque_constant(cfg))
                self.assertTrue(any(r["integration_held"] == "True" for r in rows))
                self.assertEqual(stop["idle_confirmed"], "True")

    def test_torque_holds_integral_only_toward_downstream_clipping(self):
        for direction in (1, -1):
            with self.subTest(direction=direction):
                cfg = core.load_config()
                cfg["velocity"]["vel_ramp_rate"] = 10.0
                dev = self.board(cfg)
                core.configure_for_calibration(dev, cfg)
                axis = dev.axis0
                axis.motor.config.direction = -1
                clock = turns = 0.0
                sample = 0

                def step(dt):
                    nonlocal clock, turns, sample
                    sample += 1
                    clock += dt
                    if sample == 5:
                        raise KeyboardInterrupt
                    speed = direction * (40 if sample == 3 else 20) / 60
                    turns += speed * dt
                    axis.encoder.vel_estimate = speed
                    axis.encoder.shadow_count = round(turns * cfg["hardware"]["cpr"])
                    applied_torque = (-direction * 0.02 if sample in (2, 3)
                                      else axis.controller.input_torque)
                    axis.motor.current_control.Iq_setpoint = -applied_torque / axis.motor.config.torque_constant

                with patch.object(torque, "confirm_motion"), \
                        patch.object(torque, "admit_control_period", return_value=0.025), \
                        patch.object(torque.time, "monotonic", side_effect=lambda: clock), \
                        patch.object(torque.time, "sleep", side_effect=step):
                    torque.run(direction * 30, cfg, dev)
                path, = (self.root / "logs").glob(f"m0_torque_*_{direction * 30:+g}rpm.csv")
                with path.open() as file:
                    _, *samples, _ = list(csv.DictReader(file))
                self.assertEqual([r["integration_held"] for r in samples],
                                 ["False", "True", "False", "False"])
                self.assertAlmostEqual(float(samples[1]["firmware_torque_setpoint_nm"]), -direction * 0.02)
                for old, new in zip(samples, samples[1:]):
                    self.assertEqual(old["command_torque_nm"], new["previous_command_torque_nm"])
                integral = [direction * (float(r["command_torque_nm"])
                                         - cfg["torque"]["kp"] * float(r["speed_error_rpm"]) / 60)
                            for r in samples]
                self.assertAlmostEqual(integral[0], integral[1])
                self.assertLess(integral[2], integral[1])
                self.assertGreater(integral[3], integral[2])

    def test_torque_invalid_config_stops_before_discovery(self):
        cfg = core.load_config()
        for settings in (None, {}, {"torque_limit_nm": 0, "kp": 1, "ki": 1},
                         {"torque_limit_nm": 1, "kp": True, "ki": 1},
                         {"torque_limit_nm": 1, "kp": 1, "ki": 0},
                         {"torque_limit_nm": 1, "kp": 1, "ki": 1, "firmware_limiter_gain": 0},
                         {"torque_limit_nm": 1, "kp": 1, "ki": 1, "firmware_limiter_gain": None},
                         {"torque_limit_nm": 1, "kp": 1, "ki": math.nan}):
            with self.subTest(settings=settings), self.assertRaisesRegex(ValueError, "torque"):
                torque.run(30, {**cfg, "torque": settings})
        for ratio in (0.99, None, True, "1.2", math.nan, math.inf):
            invalid = {**cfg, "torque": {**cfg["torque"], "overspeed_ratio": ratio}}
            with self.subTest(ratio=ratio), self.assertRaisesRegex(ValueError, "overspeed_ratio"):
                torque.run(30, invalid)
        cfg["controller"]["torque_ramp_rate"] = 0
        with self.assertRaisesRegex(ValueError, "torque_ramp_rate"):
            torque.run(30, cfg)

    def test_torque_stops_on_faults_and_does_not_restore_without_idle(self):
        for case, reason, message in (
                ("bus", "fault", "Bus voltage"), ("feedback", "fault", "Invalid motor feedback"),
                ("overspeed", "overspeed", "ODrive reported a fault"), ("time", "timing_overrun", "timing budget"),
                ("stall", "stall", "stalled"), ("drive", "fault", "ODrive reported a fault"),
                ("idle", "fault", "IDLE not confirmed")):
            with self.subTest(case=case):
                cfg = core.load_config()
                cfg["velocity"]["vel_ramp_rate"] = 0.5
                dev = self.board(cfg)
                core.configure_for_calibration(dev, cfg)
                axis = dev.axis0
                original_mode = axis.controller.config.control_mode
                clock = 0.0

                def step(dt):
                    nonlocal clock
                    clock += dt
                    axis.motor.current_control.Iq_setpoint = (axis.controller.input_torque
                                                             / axis.motor.config.torque_constant)
                    if case == "bus":
                        dev.vbus_voltage = 28
                    elif case == "feedback":
                        axis.encoder.vel_estimate = math.nan
                    elif case == "overspeed":
                        if axis.encoder.vel_estimate == 0.0:
                            axis.encoder.vel_estimate = 160 / 60
                            axis.encoder.shadow_count += 135
                        else:
                            axis.controller.error = CONTROLLER_ERROR_OVERSPEED
                    elif case == "time":
                        clock += cfg["safety"]["watchdog_timeout"]
                    elif case == "drive":
                        axis.motor.error = MOTOR_ERROR_CURRENT_LIMIT_VIOLATION
                    elif case == "idle":
                        raise KeyboardInterrupt
                    if clock > 4:
                        self.fail("Torque test failed to stop")

                real_safe_idle = torque.safe_idle
                with patch.object(torque, "confirm_motion"), patch.object(torque, "admit_control_period", return_value=0.025), \
                        patch.object(torque, "LOG_QUEUE_SIZE", 512), \
                        patch.object(torque.time, "monotonic", side_effect=lambda: clock), \
                        patch.object(torque.time, "sleep", side_effect=step), \
                        patch.object(torque, "safe_idle", side_effect=real_safe_idle if case != "idle"
                                     else lambda *_: False):
                    with self.assertRaisesRegex((RuntimeError, TimeoutError), message):
                        torque.run(120, cfg, dev)
                if case == "idle":
                    self.assertTrue(axis.config.enable_watchdog)
                    self.assertEqual(axis.controller.config.control_mode, CONTROL_MODE_TORQUE_CONTROL)
                    self.assertEqual(axis.motor.config.torque_lim, 0.10)
                    self.assertEqual(axis.requests, [AXIS_STATE_CLOSED_LOOP_CONTROL])
                else:
                    self.assertFalse(axis.config.enable_watchdog)
                    self.assertEqual(axis.controller.config.control_mode, original_mode)
                    self.assertEqual(axis.motor.config.torque_lim, math.inf)
                    self.assertEqual(axis.requests, [AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE])
                path = sorted((self.root / "logs").glob("m0_torque_*.csv"))[-1]
                with path.open() as file:
                    rows = list(csv.DictReader(file))
                stop = rows[-1]
                if case == "overspeed":
                    self.assertGreater(float(rows[-2]["encoder_count_rpm"]), 154.0)
                    self.assertEqual(stop["controller_error"], str(CONTROLLER_ERROR_OVERSPEED))
                self.assertEqual(stop["stop_reason"], reason)
                self.assertEqual(stop["idle_confirmed"], str(case != "idle"))

    def test_torque_attempts_other_restores_after_one_write_fails(self):
        cfg = core.load_config()
        dev = self.board(cfg)
        core.configure_for_calibration(dev, cfg)
        axis = dev.axis0
        original_mode = axis.controller.config.control_mode

        class ControllerConfig(NS):
            def __setattr__(self, key, value):
                if key == "vel_limit_tolerance" and value == 1.1:
                    raise OSError("Restore write failed")
                super().__setattr__(key, value)

        original = vars(axis.controller.config).copy()
        original["vel_limit_tolerance"] = 1.1
        axis.controller.config = ControllerConfig(**original)
        with patch.object(torque, "confirm_motion"), patch.object(torque, "admit_control_period", return_value=0.025), \
                patch.object(torque.time, "monotonic", return_value=0.0), \
                patch.object(torque.time, "sleep", side_effect=KeyboardInterrupt):
            with self.assertRaisesRegex(RuntimeError, "Restore write failed"):
                torque.run(30, cfg, dev)
        self.assertEqual(axis.current_state, AXIS_STATE_IDLE)
        self.assertEqual(axis.controller.config.control_mode, original_mode)
        self.assertEqual(axis.motor.config.torque_lim, math.inf)

    def test_torque_capture_reuses_control_samples_and_preserves_board_fault(self):
        for capture in (False, True):
            with self.subTest(capture=capture):
                cfg = core.load_config()
                dev = self.board(cfg)
                core.configure_for_calibration(dev, cfg)
                axis = dev.axis0
                current = axis.motor.current_control
                current.Id_setpoint = 0.25
                clock = 0.0

                def step(dt):
                    nonlocal clock
                    clock += dt
                    current.Id_measured = -10 * clock
                    current.Iq_measured = 20 * clock
                    current.Iq_setpoint = axis.controller.input_torque / axis.motor.config.torque_constant
                    if clock >= 0.105:
                        axis.error = AXIS_ERROR_MOTOR_FAILED
                        axis.motor.error = MOTOR_ERROR_CURRENT_LIMIT_VIOLATION
                        dev.axis1.motor.error = MOTOR_ERROR_CURRENT_LIMIT_VIOLATION

                with patch.object(torque, "confirm_motion"), \
                        patch.object(torque, "admit_control_period", return_value=0.025), \
                        patch.object(torque.time, "monotonic", side_effect=lambda: clock), \
                        patch.object(torque.time, "sleep", side_effect=step), \
                        patch.object(torque, "read_feedback", wraps=torque.read_feedback) as read:
                    with self.assertRaisesRegex(RuntimeError, "ODrive reported a fault"):
                        torque.run(30, cfg, dev, capture=capture)
                    self.assertEqual(read.call_count, 5)

                path = sorted((self.root / "logs").glob("m0_torque_*.csv"))[-1]
                capture_path = path.with_name(path.name.replace("m0_torque_", "m0_capture_", 1))
                with path.open() as file:
                    start, *samples, stop = list(csv.DictReader(file))
                self.assertEqual(capture_path.exists(), capture)
                if capture:
                    with capture_path.open() as file:
                        readings = list(csv.DictReader(file))
                    self.assertEqual(readings, [
                        {key: sample[key] for key in torque.CAPTURE_COLUMNS} for sample in samples
                    ])
                self.assertEqual(len(samples), 4)
                first = samples[0]
                self.assertAlmostEqual(float(first["elapsed_s"]), 0.025)
                self.assertAlmostEqual(float(first["id_measured_a"]), -0.25)
                self.assertEqual(float(first["id_setpoint_a"]), 0.25)
                self.assertEqual(float(first["effective_current_lim_a"]), 8.0)
                self.assertEqual(float(first["rotation_deg"]), 0.0)
                self.assertEqual(float(first["active_torque_limit_nm"]), 0.10)
                self.assertEqual(float(first["ibus_a"]), 0.15)
                self.assertEqual(first["brake_resistor_saturated"], "False")
                self.assertNotIn("Angle deg", sys.stdout.getvalue())
                self.assertNotIn("Limit Nm", sys.stdout.getvalue())
                self.assertNotIn("I Limit A", sys.stdout.getvalue())
                self.assertIn("Torque Nm", sys.stdout.getvalue())
                self.assertEqual(json.loads(start["config_json"])["trial"]["current_lim_margin_a"],
                                 axis.motor.config.current_lim_margin)
                for prior, sample in zip(samples, samples[1:]):
                    self.assertAlmostEqual(float(sample["proportional_torque_nm"]),
                                           cfg["torque"]["kp"] * float(sample["speed_error_rpm"]) / 60)
                    self.assertAlmostEqual(float(sample["firmware_torque_setpoint_nm"]),
                                           float(prior["command_torque_nm"]))
                    self.assertAlmostEqual(float(sample["command_torque_nm"]),
                                           float(sample["proportional_torque_nm"])
                                           + float(sample["integral_torque_nm"]))
                self.assertEqual(stop["motor_error"], str(MOTOR_ERROR_CURRENT_LIMIT_VIOLATION))
                self.assertEqual(stop["idle_confirmed"], "True")
                self.assertAlmostEqual(float(stop["elapsed_s"]), 0.125)
                self.assertEqual(json.loads(stop["post_stop_faults_json"])["m1"]["motor_error"],
                                 MOTOR_ERROR_CURRENT_LIMIT_VIOLATION)

    def test_torque_stalls_writer_and_disarms_before_waiting_for_it(self):
        cfg = core.load_config()
        dev = self.board(cfg)
        core.configure_for_calibration(dev, cfg)
        clock = 0.0
        writer_started = Event()
        idle_requested = Event()
        sample = m0_display.TelemetryTable.sample

        def advance(dt):
            nonlocal clock
            clock += dt
            if clock >= 0.05:
                self.assertTrue(writer_started.wait(1.0))

        def slow_sample(table, *args, **kwargs):
            writer_started.set()
            if not idle_requested.wait(1.0):
                raise RuntimeError("Timed out waiting for IDLE from the control loop")
            sample(table, *args, **kwargs)

        def idle(axis, config):
            result = core.safe_idle(axis, config)
            idle_requested.set()
            return result

        with patch.object(torque, "confirm_motion"), \
                patch.object(torque, "admit_control_period", return_value=0.025), \
                patch.object(torque, "LOG_QUEUE_SIZE", 2), \
                patch.object(torque.time, "monotonic", side_effect=lambda: clock), \
                patch.object(torque.time, "sleep", side_effect=advance), \
                patch.object(torque, "safe_idle", side_effect=idle), \
                patch.object(m0_display.TelemetryTable, "sample", slow_sample):
            with self.assertRaisesRegex(torque.DiagnosticStop, "queue filled"):
                torque.run(30, cfg, dev, capture=True)
        path, = (self.root / "logs").glob("m0_torque_*.csv")
        with path.open() as file:
            stop = list(csv.DictReader(file))[-1]
        self.assertEqual(stop["stop_reason"], "logging_failure")
        self.assertEqual(stop["idle_confirmed"], "True")
        self.assertEqual(dev.axis0.requests, [AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE])

    def test_torque_uses_post_stop_masks_if_brake_disarms_mid_read(self):
        cfg = core.load_config()

        class Board(NS):
            @property
            def brake_resistor_armed(self):
                if self.axis0.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                    self.axis0.error = AXIS_ERROR_BRAKE_RESISTOR_DISARMED
                    self.axis0.motor.error = MOTOR_ERROR_DC_BUS_OVER_REGEN_CURRENT
                    self.axis1.motor.error = MOTOR_ERROR_DC_BUS_OVER_REGEN_CURRENT
                    return False
                return True

        dev = Board(**vars(self.board(cfg)))
        core.configure_for_calibration(dev, cfg)
        clock = 0.0

        def advance(dt):
            nonlocal clock
            clock += dt

        with patch.object(torque, "confirm_motion"), \
                patch.object(torque, "admit_control_period", return_value=0.025), \
                patch.object(torque.time, "monotonic", side_effect=lambda: clock), \
                patch.object(torque.time, "sleep", side_effect=advance):
            with self.assertRaisesRegex(RuntimeError, "Brake resistor no longer armed"):
                torque.run(30, cfg, dev)
        path, = (self.root / "logs").glob("m0_torque_*.csv")
        with path.open() as file:
            _, stop = list(csv.DictReader(file))
        self.assertEqual(stop["stop_reason"], "fault")
        self.assertEqual(stop["motor_error"], str(MOTOR_ERROR_DC_BUS_OVER_REGEN_CURRENT))
        self.assertEqual(json.loads(stop["post_stop_faults_json"])["m1"]["motor_error"],
                         MOTOR_ERROR_DC_BUS_OVER_REGEN_CURRENT)

    def test_torque_absolute_ticks_continue_past_ten_seconds_or_stop_before_late_write(self):
        for late_read in (False, True):
            with self.subTest(late_read=late_read):
                cfg = core.load_config()
                dev = self.board(cfg)
                core.configure_for_calibration(dev, cfg)
                axis = dev.axis0
                clock = 0.0
                read = torque.read_feedback

                def advance(dt):
                    nonlocal clock
                    clock += dt
                    if clock >= 10.5:
                        raise KeyboardInterrupt
                    axis.encoder.vel_estimate = min(0.5, clock)
                    axis.encoder.shadow_count += round(axis.encoder.vel_estimate * dt * cfg["hardware"]["cpr"])
                    axis.motor.current_control.Iq_setpoint = (axis.controller.input_torque
                                                              / axis.motor.config.torque_constant)

                def feedback(*args, **kwargs):
                    nonlocal clock
                    snapshot = read(*args, **kwargs)
                    clock += 0.03 if late_read else 0.004
                    return snapshot

                with patch.object(torque, "confirm_motion"), \
                        patch.object(torque, "admit_control_period", return_value=0.025), \
                        patch.object(torque, "LOG_QUEUE_SIZE", 512), \
                        patch.object(torque.time, "monotonic", side_effect=lambda: clock), \
                        patch.object(torque.time, "sleep", side_effect=advance), \
                        patch.object(torque, "read_feedback", side_effect=feedback):
                    if late_read:
                        with self.assertRaisesRegex(torque.DiagnosticStop, "torque write deadline"):
                            torque.run(30, cfg, dev)
                    else:
                        torque.run(30, cfg, dev)
                path = sorted((self.root / "logs").glob("m0_torque_*.csv"))[-1]
                with path.open() as file:
                    _, *samples, stop = list(csv.DictReader(file))
                self.assertEqual(stop["idle_confirmed"], "True")
                if late_read:
                    self.assertEqual(stop["stop_reason"], "timing_overrun")
                    self.assertEqual(axis.watchdog_feed.call_count, 2)
                    self.assertFalse(samples)
                else:
                    self.assertEqual(stop["stop_reason"], "user_stop")
                    self.assertAlmostEqual(float(stop["elapsed_s"]), 10.5)
                    self.assertEqual(len(samples), 419)
                    self.assertGreater(float(samples[-1]["elapsed_s"]), 10.0)
                    self.assertTrue(all(abs(float(r["loop_interval_s"]) - 0.025) < 1e-9 for r in samples))

    def test_torque_console_reloads_settings_and_accepts_signed_rpm(self):
        def prompt(_):
            return next(replies)

        def trial(rpm, cfg, *, capture):
            self.assertTrue(capture)
            self.assertEqual(cfg["axis_index"], 0)
            self.assertEqual(cfg["torque"]["torque_limit_nm"], 0.1 if rpm > 0 else 0.2)
            self.raw["axes"]["m0"]["torque"]["torque_limit_nm"] = 0.2
            self.save()

        replies = iter(["30", "-30", "q"])
        with patch.object(sys, "argv", ["torque_m0.py"]), patch("builtins.input", side_effect=prompt), \
                patch.object(torque, "run", side_effect=trial) as run:
            torque.main()
        self.assertEqual([call.args[0] for call in run.call_args_list], [30, -30])
        for flags, capture in (([], True), (["--capture"], True), (["--no-capture"], False)):
            with patch.object(sys, "argv", ["torque_m0.py", *flags, "-30"]), patch.object(torque, "run") as run:
                torque.main()
            self.assertEqual(run.call_args.args[0], -30)
            self.assertEqual(run.call_args.kwargs["capture"], capture)

    def test_motion_lock(self):
        with core.exclusive_motion():
            with self.assertRaisesRegex(RuntimeError, "Another motion command"):
                with core.exclusive_motion():
                    self.fail("Multiple motion sessions were allowed")

    def test_clear_errors_cli_clears_only_selected_axis_and_preserves_calibration(self):
        for axis_index, flags in ((0, ["--clear"]), (1, ["--axis", "1", "--clear"]),
                                  (0, ["--clear-errors"])):
            with self.subTest(axis_index=axis_index, flags=flags):
                dev = self.board(core.load_config())
                axis = (dev.axis0, dev.axis1)[axis_index]
                other = (dev.axis1, dev.axis0)[axis_index]
                targets = (axis, axis.motor, axis.encoder, axis.controller, axis.sensorless_estimator)
                for target in targets:
                    target.error = 1
                other.error = 1

                def firmware_clear():
                    for target in targets:
                        target.error = 0

                axis.clear_errors = Mock(side_effect=firmware_clear)
                with patch.object(sys, "argv", ["calibrate_m0.py", *flags]), \
                        patch.object(calibrate_m0, "find_odrive", return_value=dev):
                    calibrate_m0.main()
                axis.clear_errors.assert_called_once_with()
                self.assertEqual(core.fault_codes(axis, sensorless=True), (0, 0, 0, 0, 0))
                self.assertEqual(other.error, 1)
                self.assertTrue(axis.motor.is_calibrated)
                self.assertTrue(axis.encoder.is_ready)
                self.assertEqual(axis.requests, [])
                self.assertEqual(other.requests, [])

    def test_clear_errors_rejects_active_axes_and_reports_persistent_faults(self):
        cfg = core.load_config()
        dev = self.board(cfg)
        dev.axis0.clear_errors = Mock()
        with patch.object(calibrate_m0, "find_odrive", return_value=dev), \
                patch.object(calibrate_m0, "dump_errors") as report:
            for moving in (dev.axis0, dev.axis1):
                moving.current_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                with self.assertRaisesRegex(RuntimeError, "Both axes must be IDLE"):
                    calibrate_m0.clear_errors(cfg)
                moving.current_state = AXIS_STATE_IDLE
            dev.axis0.clear_errors.assert_not_called()
            dev.axis0.sensorless_estimator.error = 1
            with self.assertRaisesRegex(RuntimeError, "still reports a fault"):
                calibrate_m0.clear_errors(cfg)
            dev.axis0.clear_errors.assert_called_once_with()
            report.assert_called_once_with(dev)
        self.assertEqual(dev.axis0.requests, [])
        self.assertEqual(dev.axis1.requests, [])

    def test_calibrate_cli_default_clears_faults_before_calibration(self):
        dev = self.board(core.load_config())
        axis = dev.axis0
        axis.error = AXIS_ERROR_MOTOR_FAILED
        axis.clear_errors = Mock(side_effect=lambda: setattr(axis, "error", 0))

        def confirm(_description):
            axis.clear_errors.assert_called_once_with()
            self.assertEqual(axis.error, 0)
            self.assertEqual(axis.requests, [])

        with patch.object(sys, "argv", ["calibrate_m0.py"]), \
                patch.object(calibrate_m0, "find_odrive", return_value=dev) as discover, \
                patch.object(calibrate_m0, "confirm_motion", side_effect=confirm) as prompt, \
                patch.object(calibrate_m0, "calibrate_step") as step, \
                patch.object(calibrate_m0, "wait_for_settling"):
            calibrate_m0.main()
        discover.assert_called_once()
        prompt.assert_called_once()
        self.assertEqual([call.args[1] for call in step.call_args_list],
                         [AXIS_STATE_MOTOR_CALIBRATION, AXIS_STATE_ENCODER_OFFSET_CALIBRATION])
        self.assertEqual(axis.current_state, AXIS_STATE_IDLE)
        self.assertEqual(dev.axis1.requests, [])

    def test_m0_command_line_defaults_and_signed_rpm(self):
        with patch.object(sys, "argv", ["calibrate_m0.py"]), \
                patch.object(calibrate_m0, "calibrate") as calibrate:
            calibrate_m0.main()
        self.assertEqual(calibrate.call_args.args[0]["axis_index"], 0)
        for args, rpm in ((["-30"], -30), (["--sensorless", "550", "--continuous"], 550), ([], 30)):
            with patch.object(sys, "argv", ["vel_m0.py", *args]), \
                    patch("builtins.input", side_effect=["30", "q"]), patch.object(vel, "run") as run:
                vel.main()
            self.assertEqual(run.call_args.args[0], rpm)
            self.assertEqual(run.call_args.args[1]["axis_index"], 0)
        with patch.object(sys, "argv", ["encoder_m0.py"]), \
                patch.object(encoder_m0, "find_odrive") as find:
            find.side_effect = KeyboardInterrupt
            with self.assertRaises(KeyboardInterrupt):
                encoder_m0.main()
        self.assertEqual(find.call_args.args[0]["axis_index"], 0)
