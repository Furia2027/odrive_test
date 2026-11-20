import csv
import io
import json
import math
from pathlib import Path
from queue import Queue
import select
import subprocess
import sys
from tempfile import TemporaryDirectory
from threading import Event, Thread
import time
import tkinter as tk
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from fibre import ChannelBrokenException
from matplotlib.colors import same_color

from motor import motor_core, tuning
from pso import pso_m0
from ui import app as gui
from ui import worker


HEADER = ("event", "elapsed_s", *gui.FIELDS, "stop_reason", "idle_confirmed", "detail")


class BenchWorkerTest(unittest.TestCase):
    def test_teleop_dispatch_uses_shared_runner(self):
        operator = Mock()
        with patch.object(worker.teleop_motor, "run_teleop") as command, patch.object(worker.core, "load_config") as load:
            worker.perform("teleop", 1, None, operator, {"mode": "velocity", "ceiling": 35})
            load.assert_called_once_with(1, read_only=False)
            command.assert_called_once_with(load.return_value, mode="velocity", ceiling_rpm=35,
                                            read_events=operator.take_teleop, notify=operator.notify,
                                            confirm=operator.confirm, check_cancel=operator.check_cancel)

    def test_teleop_queue_prioritizes_braking_and_drops_old_arrows(self):
        operator = object.__new__(worker.Operator)
        operator.check_cancel = Mock()
        operator.teleop_events = Queue(maxsize=32)
        for event in ((0, "forward"), (0.8, "reverse")):
            operator.teleop_events.put_nowait(event)
        self.assertEqual(operator.take_teleop(1), ["reverse"])
        for event in ((1, "forward"), (1.01, "brake"), (1.02, "reverse")):
            operator.teleop_events.put_nowait(event)
        self.assertEqual(operator.take_teleop(1.03), ("brake",))
        self.assertTrue(operator.teleop_events.empty())

    def test_teleop_input_stops_on_operator_link_loss(self):
        script = """
import time
from ui import worker

def perform(action, axis, rpm, operator, options):
    operator.confirm("Teleop mock")
    operator.notify("running", None)
    try:
        while True:
            if events := operator.take_teleop(time.monotonic()):
                operator.emit("input", events)
            time.sleep(0.01)
    finally:
        operator.emit("cleanup", None)

worker.perform = perform
raise SystemExit(worker.main())
"""
        for stop in ("explicit stop", "closed input"):
            with self.subTest(stop=stop), subprocess.Popen(
                    [sys.executable, "-B", "-u", "-c", script, "teleop", "0"],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) as child:
                self.addCleanup(child.kill)
                self.assertTrue(select.select([child.stdout], [], [], 5)[0])
                self.assertEqual(json.loads(child.stdout.readline()), ["confirm", "Teleop mock"])
                child.stdin.write('{"teleop":"reverse"}\nconfirm\n')
                child.stdin.flush()
                self.assertEqual(json.loads(child.stdout.readline()), ["approved", None])
                self.assertEqual(json.loads(child.stdout.readline()), ["running", None])
                child.stdin.write('{"teleop":"forward"}\n')
                child.stdin.flush()
                self.assertTrue(select.select([child.stdout], [], [], 5)[0])
                self.assertEqual(json.loads(child.stdout.readline()), ["input", ["forward"]])
                if stop == "explicit stop":
                    child.stdin.write('stop\n{"teleop":"reverse"}\n')
                    child.stdin.flush()
                else:
                    child.stdin.close()
                self.assertEqual(child.wait(timeout=5), 130 if stop == "explicit stop" else 1)
                self.assertEqual(json.loads(child.stdout.readline()), ["cleanup", None])

    def test_pso_worker_consumes_only_the_current_prompt_id(self):
        script = """
from ui import worker

def perform(action, axis, rpm, operator, options):
    for kind in ("guard", "motion"):
        operator.emit("answer", operator.ask_pso(kind))

worker.perform = perform
raise SystemExit(worker.main())
"""
        with subprocess.Popen([sys.executable, "-B", "-u", "-c", script, "pso_validate", "0"],
                              stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              text=True) as child:
            self.addCleanup(child.kill)
            self.assertTrue(select.select([child.stdout], [], [], 5)[0])
            self.assertEqual(json.loads(child.stdout.readline()), ["pso_prompt", {"id": 1, "kind": "guard"}])
            child.stdin.write('{"pso_id": 1, "value": "2000"}\n')
            child.stdin.flush()
            self.assertEqual(json.loads(child.stdout.readline()), ["answer", "2000"])
            self.assertEqual(json.loads(child.stdout.readline()), ["pso_prompt", {"id": 2, "kind": "motion"}])
            child.stdin.write('{"pso_id": 1, "value": "confirm"}\n')
            child.stdin.flush()
            self.assertFalse(select.select([child.stdout], [], [], 0.1)[0])
            child.stdin.write('{"pso_id": 2, "value": "confirm"}\n')
            child.stdin.flush()
            self.assertEqual(json.loads(child.stdout.readline()), ["answer", "confirm"])
            self.assertEqual(child.wait(timeout=5), 0, child.stderr.read())

    def test_pso_save_requires_confirmation_and_rechecks_revision(self):
        original = motor_core.CONFIG_PATH.read_bytes()
        for answer in ("discard", "stale", "save"):
            with self.subTest(answer=answer), TemporaryDirectory() as folder:
                path = Path(folder) / "motor_config.json"
                path.write_bytes(original)
                candidate = (0.02, 0.018)

                def respond(kind, **_details):
                    self.assertEqual(kind, "save")
                    if answer == "stale":
                        path.write_bytes(original + b"\n")
                    return "discard" if answer == "discard" else "save"

                operator = Mock()
                operator.ask_pso.side_effect = respond
                with (patch.object(motor_core, "CONFIG_PATH", path),
                      patch.object(motor_core, "MOTION_LOCK", path.with_suffix(".lock")),
                      patch.object(pso_m0, "load_trials", return_value=[]),
                      patch.object(pso_m0, "optimize", return_value=pso_m0.Selection(candidate, 0.04, (), True)),
                      patch.object(pso_m0, "bench") as bench):
                    revision = tuning.snapshot(0, "torque")["revision"]
                    options = {"mode": "torque", "revision": revision, "candidate": candidate}
                    if answer == "save":
                        worker.perform("pso_validate", 0, None, operator, options)
                        saved = json.loads(original)
                        saved["axes"]["m0"]["torque"].update(kp=candidate[0], ki=candidate[1])
                        self.assertEqual(json.loads(path.read_bytes()), saved)
                        self.assertEqual(operator.emit.call_args.args[0], "pso_saved")
                    elif answer == "stale":
                        with self.assertRaisesRegex(ValueError, "has changed"):
                            worker.perform("pso_validate", 0, None, operator, options)
                        self.assertEqual(path.read_bytes(), original + b"\n")
                    else:
                        with self.assertRaisesRegex(KeyboardInterrupt, "discarded"):
                            worker.perform("pso_validate", 0, None, operator, options)
                        self.assertEqual(path.read_bytes(), original)
                    bench.assert_called_once()

    def test_pso_worker_blocks_provisional_validation_and_save(self):
        operator = Mock()
        revision = tuning.snapshot(0, "torque")["revision"]
        candidate = (0.02, 0.018)
        options = {"mode": "torque", "revision": revision, "candidate": candidate}
        with (patch.object(pso_m0, "load_trials", return_value=[]),
              patch.object(pso_m0, "optimize", return_value=pso_m0.Selection(candidate, 0.04, (), False)),
              patch.object(pso_m0, "bench") as bench, patch.object(pso_m0, "apply_result") as save):
            worker.perform("pso_analyze", 0, None, operator, options)
            self.assertFalse(any(call.args[0] == "pso_candidate" for call in operator.emit.call_args_list))
            with self.assertRaisesRegex(ValueError, "Provisional gains are offline only"):
                worker.perform("pso_validate", 0, None, operator, options)
        operator.ask_pso.assert_not_called()
        bench.assert_not_called()
        save.assert_not_called()

    def test_worker_cancels_before_confirmation_during_motion_and_on_eof(self):
        script = """
import time
from ui import worker

def perform(action, axis, rpm, operator, options):
    operator.confirm(f"{action} M{axis} {rpm:+g} RPM")
    try:
        operator.emit("active", None)
        while True:
            time.sleep(0.01)
            operator.check_cancel()
    finally:
        operator.emit("cleanup", None)

worker.perform = perform
raise SystemExit(worker.main())
"""
        for kind in ("before confirm", "active stop", "parent closed", "heartbeat timeout"):
            with self.subTest(kind=kind), subprocess.Popen(
                    [sys.executable, "-B", "-u", "-c", script, "auto", "0", "--rpm=-100"],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    text=True) as child:
                self.addCleanup(child.kill)
                ready, _, _ = select.select([child.stdout], [], [], 5)
                self.assertTrue(ready)
                self.assertEqual(json.loads(child.stdout.readline()), ["confirm", "auto M0 -100 RPM"])
                if kind != "before confirm":
                    child.stdin.write("confirm\n")
                    child.stdin.flush()
                    self.assertEqual(json.loads(child.stdout.readline()), ["approved", None])
                    self.assertEqual(json.loads(child.stdout.readline()), ["active", None])
                if kind == "parent closed":
                    child.stdin.close()
                elif kind != "heartbeat timeout":
                    child.stdin.write("stop\n")
                    child.stdin.flush()
                lost = kind in ("heartbeat timeout", "parent closed")
                self.assertEqual(child.wait(timeout=5), 1 if lost else 130, child.stderr.read())
                messages = [json.loads(line) for line in child.stdout]
                expected = ["cancelled"] if kind == "before confirm" else ["cleanup", "error" if lost else "cancelled"]
                self.assertEqual([message[0] for message in messages], expected)
                if kind == "heartbeat timeout":
                    self.assertIn("heartbeat timed out", messages[-1][1])

    def test_signed_m1_velocity_dispatch_and_m0_only_modes(self):
        command = Mock()
        operator = Mock()
        with patch.dict(worker.ACTIONS, velocity=command), patch.object(worker.core, "load_config") as load:
            worker.perform("velocity", 1, -30, operator)
            load.assert_called_once_with(1, read_only=False)
            command.assert_called_once_with(-30, load.return_value, confirm=operator.confirm,
                                            check_cancel=operator.check_cancel, notify=operator.notify)
            for action in ("torque", "auto"):
                with self.assertRaisesRegex(ValueError, "require M0"):
                    worker.perform(action, 1, -30, operator)

    def test_encoder_monitor_dispatch_without_rpm_or_motion_confirmation(self):
        operator = Mock()
        with patch.object(worker.encoder_m0, "run") as command, patch.object(worker.core, "load_config") as load:
            worker.perform("encoder", 1, None, operator)
            load.assert_called_once_with(1, read_only=True)
            command.assert_called_once_with(load.return_value, check_cancel=operator.check_cancel, notify=operator.emit)
            operator.confirm.assert_not_called()

    def test_status_propagates_usb_link_loss(self):
        operator = Mock()
        with patch.object(worker.core, "find_odrive", side_effect=ChannelBrokenException()):
            with self.assertRaises(ChannelBrokenException):
                worker.perform("status", 0, None, operator)
        operator.emit.assert_not_called()


class TrialLogTest(unittest.TestCase):
    def test_csv_tails_partial_rows_and_keeps_missing_values_and_final_fault(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "trial.csv"
            with path.open("w", newline="") as stream:
                csv.writer(stream).writerows([HEADER, ("sample", "0.025", "10", "9", "", 1, 1, 0, 24)])
                stream.write("sample,0.05,10,")
                stream.flush()
                trial = gui.TrialLog(path)
                self.addCleanup(trial.close)
                trial.read(0)
                self.assertEqual(len(trial.points), 1)
                self.assertTrue(math.isnan(trial.points[0][3]))
                self.assertTrue(trial.stale(1))
                stream.write("11,12,1,1,0,24\n")
                csv.writer(stream).writerow(("stop", "0.06", *("",) * 7,
                                             "fault", "False", "USB disconnected\nNo IDLE response"))
                stream.flush()
                trial.read(1.1)
                self.assertEqual(len(trial.points), 2)
                self.assertFalse(trial.stale(1.1))
                self.assertEqual(trial.stop["stop_reason"], "fault")
                self.assertEqual(trial.stop["idle_confirmed"], "False")
                self.assertEqual(trial.stop["detail"], "USB disconnected\nNo IDLE response")

    def test_index_reads_old_windows_outside_live_buffer(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "long.csv"
            with path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(HEADER)
                for index in range(4801):
                    writer.writerow(("sample", index / 40, index, index, index, 1, 2, 0, 24))
            trial = gui.TrialLog(path)
            self.addCleanup(trial.close)
            for _ in range(5):
                trial.read(0)
            self.assertEqual(trial.latest, 120)
            self.assertGreaterEqual(trial.points[0][0], 90)
            self.assertLessEqual(len(trial.points), 1201)
            self.assertLess(len(trial.index), 30)
            window = trial.history(trial.offset_for(3), 3, 33, Event())
            self.assertEqual((window[0][0], window[-1][0]), (3, 33))
            self.assertEqual(len(window), 1201)


class BenchDesktopTest(unittest.TestCase):
    def setUp(self):
        try:
            root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(str(exc))
        self.app = gui.BenchApp(root)
        self.addCleanup(self.app.destroy)
        self.root = root
        self.root.update()

    def pump(self, duration):
        end = time.monotonic() + duration
        longest = 0
        while time.monotonic() < end:
            start = time.monotonic()
            self.root.update()
            longest = max(longest, time.monotonic() - start)
            time.sleep(0.005)
        return longest

    def test_axis_gating_and_idle_unconfirmed_latch(self):
        app = self.app
        m0 = dict(idle=True, state=1, encoder_ready=True, faults={}, calibration_issue="",
                  motion_issue="", rpm_range=(1, 200))
        m1 = {**m0, "motion_issue": "M1 hardware is unverified",
              "calibration_issue": "M1 hardware is unverified", "rpm_range": None}
        app.status = {"serial": "MOCK", "both_idle": True, "axes": [m0, m1]}
        app.sync()
        self.assertEqual(app.mode.get(), "Control")
        self.assertEqual(app.drive_badge["style"], "DriveIdle.TLabel")
        self.assertIn("Enter a nonzero signed RPM", app.detail.get())
        app.rpm.set("-30")
        self.assertFalse(app.run_button.instate(("disabled",)))
        self.assertIs(app.run_button.master, app.stop_button.master)
        with patch.object(app, "start") as start:
            app.run_button.invoke()
            start.assert_called_once_with("auto")
        app.rpm.set("-201")
        self.assertTrue(app.run_button.instate(("disabled",)))
        app.axis.set("M1")
        self.assertFalse(app.clear_button.instate(("disabled",)))
        self.assertTrue(app.cal_button.instate(("disabled",)))
        self.assertTrue(app.run_button.instate(("disabled",)))
        self.assertEqual(app.mode_picker["values"], ("Control", "Velocity", "Encoder", "Torque"))
        self.assertIn("require M0", app.detail.get())
        self.assertEqual(app.drive_badge["style"], "DriveCaution.TLabel")
        app.mode.set("Velocity")
        self.assertIn("hardware is unverified", app.detail.get())
        app.mode.set("Encoder")
        self.assertTrue(app.rpm_input.instate(("disabled",)))
        self.assertEqual(app.run_button["text"], "Monitor encoder")
        with patch.object(app, "start") as start:
            app.run_button.invoke()
            start.assert_called_once_with("encoder")
        app.process = SimpleNamespace(stdin=io.StringIO())
        app.action = "status"
        app.status = None
        app.reviewing_stop = True
        app.finish_worker(1)
        self.assertIn("Isolate motor power", app.activity.get())
        self.assertEqual(app.drive_badge["style"], "DriveFault.TLabel")
        self.assertTrue(app.clear_button.instate(("disabled",)))
        app.process = SimpleNamespace(stdin=io.StringIO())
        app.action = "status"
        app.status = {"serial": "MOCK", "both_idle": True, "axes": [m0, m1]}
        app.finish_worker(0)
        self.assertFalse(app.idle_unconfirmed)
        self.assertFalse(app.clear_button.instate(("disabled",)))
        app.process = SimpleNamespace(stdin=io.StringIO())
        app.action = "velocity"
        with patch.object(app, "send", return_value=True) as send:
            app.stop()
            app.stop()
            send.assert_called_once_with("stop")
        app.stop_alert_at = time.monotonic() - 1
        self.pump(0.15)
        self.assertIn("Isolate motor power", app.activity.get())

    def test_teleop_uses_selected_axis_mode_and_separate_ceiling(self):
        app = self.app
        idle = dict(idle=True, state=1, encoder_ready=True, faults={}, calibration_issue="",
                    motion_issue="", rpm_range=(1, 2200))
        app.status = {"serial": "MOCK", "both_idle": True, "axes": [idle, {**idle, "rpm_range": (1, 35)}]}
        app.rpm.set("-30")
        app.teleop_ceiling.set("2201")
        self.assertTrue(app.teleop_button.instate(("disabled",)))
        self.assertIn("2200", app.teleop_text.get())
        app.teleop_ceiling.set("2200")
        self.assertFalse(app.teleop_button.instate(("disabled",)))
        self.assertIn("Torque", app.teleop_text.get())
        with patch.object(app, "launch", return_value=True) as launch:
            app.teleop_button.invoke()
        launch.assert_called_once_with("teleop", options={"mode": "control", "ceiling": 2200.0})
        app.mode.set("Torque")
        self.assertFalse(app.teleop_button.instate(("disabled",)))
        app.teleop_ceiling.set("50")
        app.axis.set("M1")
        self.assertTrue(app.teleop_button.instate(("disabled",)))
        app.mode.set("Encoder")
        self.assertTrue(app.teleop_button.instate(("disabled",)))
        app.mode.set("Velocity")
        self.assertEqual(app.teleop_ceiling.get(), "35")
        self.assertFalse(app.teleop_button.instate(("disabled",)))
        self.assertEqual(app.rpm.get(), "-30")
        with patch.object(app, "launch", return_value=True) as launch:
            app.teleop_button.invoke()
        launch.assert_called_once_with("teleop", options={"mode": "velocity", "ceiling": 35.0})

    def test_teleop_indicators_and_callbacks_follow_worker_state(self):
        app = self.app
        app.process = SimpleNamespace(stdin=io.StringIO())
        app.action = "teleop"
        app.handle("confirm", "Start at zero")
        self.assertIn("Confirm motion", app.teleop_text.get())
        with patch.object(app, "send", return_value=True) as send:
            keys = app.teleop_keys
            keys.press(SimpleNamespace(keysym="Up", time=1, state=0))
            send.assert_not_called()
            app.confirm()
            send.assert_called_once_with("confirm")
            send.reset_mock()
            app.handle("running", None)
            keys.press(SimpleNamespace(keysym="Up", time=2, state=0))
            send.assert_not_called()  # A held preflight key cannot begin motion.
            keys.release(SimpleNamespace(keysym="Up", time=3))
            keys.press(SimpleNamespace(keysym="Up", time=4, state=0))
            self.assertEqual(json.loads(send.call_args.args[0]), {"teleop": "forward"})
            for kind, direction, active in (("driving", "forward", "forward"),
                                            ("reversing", "reverse", ""),
                                            ("driving", "reverse", "reverse"),
                                            ("braking", "", ""), ("holding", "", "")):
                app.handle("teleop_state", {"state": kind, "direction": direction})
                for key, indicator in app.teleop_arrows.items():
                    self.assertEqual(indicator["style"], "DriveActive.TLabel" if key == active else "Muted.TLabel")
            self.assertIn("Holding zero", app.teleop_text.get())
            app.stop()
            self.assertEqual(send.call_args.args[0], "stop")
            self.assertIn("Stopping", app.teleop_text.get())
            keys.press(SimpleNamespace(keysym="Up", time=5, state=0))
            self.assertEqual(send.call_count, 2)

    def test_teleop_requires_distinct_space_presses(self):
        app = self.app
        app.action, app.is_running = "teleop", True
        keys = app.teleop_keys
        with patch.object(keys, "send") as send:
            for timestamp in (100, 130, 160):
                if timestamp > 100:
                    keys.release(SimpleNamespace(keysym="space", time=timestamp))
                keys.press(SimpleNamespace(keysym="space", time=timestamp, state=0))
            send.assert_not_called()
            keys.release(SimpleNamespace(keysym="space", time=180))
            keys.press(SimpleNamespace(keysym="space", time=250, state=0))
            send.assert_called_once_with("brake")
            keys.release(SimpleNamespace(keysym="space", time=300))
            keys.press(SimpleNamespace(keysym="space", time=500, state=0))
            keys.release(SimpleNamespace(keysym="space", time=515))
            keys.press(SimpleNamespace(keysym="space", time=900, state=0))
            send.assert_called_once_with("brake")

    def test_teleop_stops_on_focus_loss_during_confirmation_and_motion(self):
        app = self.app
        for running in (False, True):
            with self.subTest(running=running), patch.object(app, "send", return_value=True) as send:
                app.process, app.action = SimpleNamespace(stdin=io.StringIO()), "teleop"
                app.stop_requested = False
                app.awaiting_confirmation, app.is_running = not running, running
                with patch.object(self.root, "focus_displayof", return_value=app.events):
                    app.teleop_keys.check_focus()
                send.assert_not_called()
                with patch.object(self.root, "focus_displayof", return_value=None):
                    app.teleop_keys.check_focus()
                    app.teleop_keys.check_focus()
                send.assert_called_once_with("stop")
                self.assertFalse(app.awaiting_confirmation)

    def test_live_40_hz_plot_freezes_without_interrupting_csv(self):
        app = self.app
        with TemporaryDirectory() as folder:
            path = Path(folder) / "fixture.csv"
            with path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow((*HEADER, "requested_rpm", "active_target_rpm"))
                stream.flush()

                def record():
                    start = time.monotonic()
                    for index in range(80):
                        time.sleep(max(0, start + index / 40 - time.monotonic()))
                        writer.writerow(("sample", index / 40, -30, -29 + index / 100, -31,
                                         0.2, 0.3, 0, 23.2, "", "", "", -32, -32))
                        stream.flush()

                app.trial = gui.TrialLog(path)
                app.recording = True
                recording = Thread(target=record)
                recording.start()
                try:
                    draw_times = []
                    app.charts.canvas.mpl_connect("draw_event", lambda _event: draw_times.append(time.monotonic()))
                    longest = self.pump(0.8)
                    self.assertGreaterEqual(len(draw_times), 2)
                    self.assertLess(longest, 0.5)
                    target = app.charts.lines["requested_rpm"][1]
                    self.assertTrue(target.get_visible())
                    self.assertEqual(target.get_ydata()[-1], -32)
                    app.freeze()
                    self.pump(0.2)  # Finish any draw already queued at the moment of freezing.
                    trace = app.charts.lines["setpoint_rpm"][1]
                    plotted = len(trace.get_xdata())
                    ingested = len(app.trial.points)
                    self.pump(0.5)
                    self.assertEqual(len(trace.get_xdata()), plotted)
                    self.assertGreater(len(app.trial.points), ingested)
                    self.assertIn("recording continues", app.telemetry.get())
                    app.freeze()
                    self.pump(0.6)
                    self.assertGreater(len(trace.get_xdata()), plotted)
                finally:
                    recording.join()
                    app.read_log(time.monotonic())
                    self.assertEqual(len(app.trial.points), 80)

    def test_graph_and_trace_visibility_and_theme(self):
        app = self.app
        charts = app.charts
        self.assertEqual(len(charts.axes), 3)
        self.assertTrue(charts.graph_controls["Torque"].instate(("disabled",)))
        self.assertEqual(charts.viewport.itemcget(charts.rows["Torque"], "state"), "normal")
        for ax in charts.axes:
            self.assertAlmostEqual((ax.get_position().x0 + ax.get_position().x1) / 2, 0.5)
        self.assertIs(charts.trace_controls["iq_measured_a"].master, charts.graph_controls["Current"].master)
        charts.trace_controls["iq_measured_a"].invoke()
        charts.trace_controls["id_measured_a"].invoke()
        self.assertIn("iq_setpoint_a", charts.lines)
        self.assertNotIn("iq_measured_a", charts.lines)
        self.assertNotIn("id_measured_a", charts.lines)
        self.assertEqual(len(charts.axes), 3)
        charts.graph_controls["Speed"].invoke()
        self.assertEqual(len(charts.axes), 2)
        self.assertNotIn("estimated_rpm", charts.lines)
        self.assertEqual(charts.graph_controls["Speed"].winfo_manager(), "place")
        self.assertEqual(charts.trace_controls["estimated_rpm"].winfo_manager(), "")
        charts.graph_controls["Speed"].invoke()
        self.assertEqual(len(charts.axes), 3)
        self.assertNotIn("iq_measured_a", charts.lines)
        self.assertTrue(app.dark.get())
        app.handle("notice", "WARNING: test warning")
        app.handle("error", "test error")
        for tag in ("warning", "error"):
            self.assertIn(f"test {tag}", app.events.get(*app.events.tag_ranges(tag)))
        app.dark.set(False)
        app.apply_theme()
        self.assertTrue(same_color(charts.axes[0].get_facecolor(), app.palette.chart))
        self.assertEqual(app.events["foreground"], app.palette.ink)
        self.assertEqual(app.events.tag_cget("warning", "foreground"), app.palette.warning)
        self.assertEqual(app.events.tag_cget("error", "foreground"), app.palette.danger)

    def test_diagnostic_graphs_are_opt_in_and_use_recorded_values(self):
        app = self.app
        charts = app.charts
        signals = {
            "command_torque_nm": 0.08, "speed_error_rpm": -3, "rotation_deg": 90,
            "ibus_a": -0.4, "read_span_s": 0.003, "loop_interval_s": 0.025,
        }
        with TemporaryDirectory() as folder:
            path = Path(folder) / "diagnostics.csv"
            with path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow((*HEADER, *signals))
                writer.writerow(("sample", 0, 30, 33, 32, 1, 2, 0, 24, "", "", "", *signals.values()))
            app.trial = gui.TrialLog(path)
            app.read_log(time.monotonic())
            self.assertEqual(set(charts.graph_axes), {"Speed", "Current", "DC bus"})
            for graph in ("Torque", "Speed error", "Shaft rotation", "Bus current", "Loop timing"):
                self.assertFalse(charts.enabled[graph].get())
                self.assertFalse(charts.graph_controls[graph].instate(("disabled",)))
                charts.graph_controls[graph].invoke()
                self.assertIn(graph, charts.graph_axes)
            for key, expected in {**signals, "read_span_s": 3, "loop_interval_s": 25}.items():
                self.assertAlmostEqual(charts.lines[key][1].get_ydata()[0], expected)
            self.assertTrue(charts.graph_controls["Following error"].instate(("disabled",)))
            charts.graph_controls["Torque"].invoke()
            self.assertNotIn("command_torque_nm", charts.lines)

    def test_drag_history_stays_detached_as_new_samples_arrive(self):
        app = self.app
        charts = app.charts
        with TemporaryDirectory() as folder:
            path = Path(folder) / "long.csv"
            with path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow((*HEADER, "command_torque_nm"))
                writer.writerows(("sample", i, i, i, i, 1, 2, 0, 24, "", "", "", i / 10)
                                 for i in range(81))
                stream.flush()
                app.trial = gui.TrialLog(path)
                app.recording = True
                app.read_log(time.monotonic())
                self.assertEqual(charts.axes[-1].get_xlim(), (50, 80))
                self.assertFalse(charts.graph_controls["Torque"].instate(("disabled",)))
                charts.graph_controls["Torque"].invoke()
                axis = charts.axes[0]
                charts.press(SimpleNamespace(button=1, inaxes=axis, x=0, y=0, dblclick=False))
                charts.pan(SimpleNamespace(x=axis.bbox.width, y=10))
                charts.release(None)
                self.pump(0.35)
                self.assertEqual(charts.axes[-1].get_xlim(), (20, 50))
                self.assertEqual(charts.lines["estimated_rpm"][1].get_xdata()[0], 20)
                writer.writerow(("sample", 85, 85, 85, 85, 1, 2, 0, 24, "", "", "", 0))
                stream.flush()
                app.read_log(time.monotonic())
                self.assertTrue(app.browsing)
                self.assertEqual(charts.axes[-1].get_xlim(), (20, 50))
                app.freeze()
                self.assertEqual(charts.axes[-1].get_xlim(), (55, 85))

    def test_scrolling_telemetry_keeps_the_rendered_image(self):
        charts = self.app.charts
        self.root.geometry("1080x680")
        self.pump(0.25)
        widget = charts.canvas.get_tk_widget()
        rendered = widget.find_all()
        drawn = Mock()
        charts.canvas.mpl_connect("draw_event", drawn)
        initial = charts.viewport.yview()
        widget.event_generate("<Button-5>", x=100, y=140)
        self.root.update()
        self.assertGreater(charts.viewport.yview()[0], initial[0])
        widget.event_generate("<Button-4>", x=100, y=140)
        self.root.update()
        self.assertEqual(charts.viewport.yview(), initial)
        self.assertEqual(widget.find_all(), rendered)
        drawn.assert_not_called()
        self.root.geometry("1240x680")
        self.pump(0.25)
        drawn.assert_called()

    def test_vertical_drag_pans_only_one_graph_and_keeps_time_live(self):
        charts = self.app.charts
        speed, current, _ = charts.axes
        x, y = speed.transAxes.transform((0.5, 0.5))
        widget = charts.canvas.get_tk_widget()
        coords = {"x": int(x), "y": charts.canvas.get_width_height(physical=True)[1] - int(y)}
        speed_limits = speed.get_ylim()
        current_limits = current.get_xlim(), current.get_ylim()
        widget.event_generate("<Button-1>", **coords)
        widget.event_generate("<Motion>", x=coords["x"] + 2, y=coords["y"] + 20, state=0x0100)
        widget.event_generate("<ButtonRelease-1>", **coords)
        panned = speed.get_ylim()
        self.assertGreater(panned[0], speed_limits[0])
        self.assertAlmostEqual(panned[1] - panned[0], speed_limits[1] - speed_limits[0])
        self.assertEqual(speed.get_xlim(), current_limits[0])
        self.assertEqual((current.get_xlim(), current.get_ylim()), current_limits)
        self.assertFalse(charts.detached)
        charts.draw()
        self.assertEqual(speed.get_ylim(), panned)
        charts.press(SimpleNamespace(button=1, inaxes=speed, dblclick=True))
        self.assertEqual(speed.get_ylim(), speed_limits)

    def test_ctrl_scroll_zooms_only_the_graph_under_the_cursor(self):
        charts = self.app.charts
        speed, current, _ = charts.axes
        limits = current.get_xlim(), current.get_ylim()
        self.pump(0.2)
        widget = charts.canvas.get_tk_widget()
        x, y = speed.transAxes.transform((0.5, 0.5))
        coords = {"x": int(x), "y": charts.canvas.get_width_height(physical=True)[1] - int(y)}
        widget.event_generate("<Button-4>", **coords)
        self.assertEqual(speed.get_xlim(), (0, 30))
        widget.event_generate("<Button-4>", state=4, **coords)
        self.assertAlmostEqual(speed.get_xlim()[1] - speed.get_xlim()[0], 24)
        self.assertEqual((current.get_xlim(), current.get_ylim()), limits)
        zoomed = speed.get_xlim(), speed.get_ylim()
        charts.left = 55
        charts.draw()
        self.assertEqual(speed.get_xlim(), tuple(55 + x for x in zoomed[0]))
        self.assertEqual(speed.get_ylim(), zoomed[1])
        self.assertEqual(current.get_xlim(), (55, 85))
        charts.press(SimpleNamespace(button=1, inaxes=speed, dblclick=True))
        self.assertEqual(speed.get_xlim(), (55, 85))
        self.assertEqual(current.get_ylim(), limits[1])

    def test_docked_review_requires_explicit_save_and_recheck(self):
        app = self.app
        self.assertEqual(len(app.panes.panes()), 3)
        app.mode.set("Velocity")
        with patch.object(app, "launch", return_value=True) as launch:
            app.open_tuning()
            self.assertEqual(str(app.panes.panes()[1]), str(app.editor.window))
            app.load_editor(tuning.editor_snapshot(0, "velocity"))
            app.editor.advanced.set(True)
            app.editor.render()
            self.assertNotIn("controller.enable_overspeed_error", app.editor.entries)
            app.editor.entries["velocity.fast_ramp_above_rpm"][1].set("150")
            launch.reset_mock()
            app.save_tuning()
            launch.assert_not_called()
            self.assertEqual(app.editor.save_button["text"], "Save reviewed changes")
            app.status = {"both_idle": True}
            app.save_tuning()
            self.assertEqual(launch.call_args.args[0], "advanced_save")
            self.assertIsNone(app.status)
            self.assertTrue(app.run_button.instate(("disabled",)))

    def test_pso_dock_gates_motion_and_save_on_worker_evidence(self):
        app = self.app
        idle = dict(idle=True, state=1, encoder_ready=True, faults={}, calibration_issue="",
                    motion_issue="", rpm_range=(1, 2200))
        app.status = {"serial": "MOCK", "both_idle": True, "axes": [idle, idle]}
        app.sync()
        self.assertEqual(app.pso_button.grid_info()["row"], app.tune_button.grid_info()["row"] + 1)
        with patch.object(app, "launch", return_value=True) as launch:
            app.pso_button.invoke()
            launch.assert_called_once_with("pso_info", None, {"mode": "position"})
        panel = app.pso
        self.assertEqual(str(app.panes.panes()[1]), str(panel.window))
        app.process = SimpleNamespace(stdin=io.StringIO())
        app.action = "pso_analyze"
        info = {"mode": "position", "revision": "a", "bounds": pso_m0.BOUNDS["position"],
                "fields": [("controller.pos_gain", "Position P", 8),
                           ("velocity.position_vel_gain", "Position velocity P", 0.15)]}
        app.handle("pso_info", info)
        app.handle("pso_analysis", {"failures": ["holdout failed"], "missing": [-30, 30]})
        app.handle("pso_analysis", {"scope": ["-30 RPM, 25 ms"], "provisional": True,
                                    "ranked": [{"gains": [7, 0.16], "score": 0.03}]})
        app.finish_worker(1)
        self.assertIn("holdout failed", panel.results.get("1.0", "end"))
        self.assertIn("Provisional offline-only scope", panel.results.get("1.0", "end"))
        self.assertIn("-30 RPM", panel.results.get("1.0", "end"))
        self.assertTrue(panel.buttons["pso_validate"].instate(("disabled",)))
        self.assertTrue(panel.buttons["pso_save"].instate(("disabled",)))
        app.process = SimpleNamespace(stdin=io.StringIO())
        app.action = "pso_analyze"
        app.handle("pso_candidate", {"gains": [7, 0.16], "score": 0.03, "revision": "a"})
        app.finish_worker(0)
        self.assertFalse(panel.buttons["pso_validate"].instate(("disabled",)))
        self.assertTrue(panel.buttons["pso_save"].instate(("disabled",)))
        app.process = SimpleNamespace(stdin=io.StringIO())
        app.action = "pso_validate"
        with patch.object(app, "send", return_value=True) as send:
            app.handle("pso_prompt", {"id": 1, "kind": "guard", "description": "Firmware trip 2700 RPM", "rpm": -2000})
            self.assertIn("-2000 RPM stage", app.prompt.get())
            app.guard_value.set("2000")
            app.confirm()
            send.assert_not_called()
            app.guard_value.set("-2000")
            app.confirm()
            self.assertEqual(json.loads(send.call_args.args[0]), {"pso_id": 1, "value": "-2000"})
            app.handle("pso_prompt", {"id": 2, "kind": "motion", "description": "Finalist -2000 RPM"})
            app.confirm()
            self.assertEqual(json.loads(send.call_args.args[0]), {"pso_id": 2, "value": "confirm"})
            app.handle("pso_prompt", {"id": 3, "kind": "save", "baseline": [8, 0.15], "gains": [7, 0.16]})
            panel.buttons["pso_save"].invoke()
            self.assertEqual(json.loads(send.call_args.args[0]), {"pso_id": 3, "value": "save"})
            self.assertTrue(panel.buttons["pso_save"].instate(("disabled",)))
        app.handle("pso_saved", {**info, "revision": "b"})
        self.assertIsNone(panel.candidate)
        self.assertIn("saved", panel.status.get())

    def test_live_entry_uses_worker_acknowledgement_and_disabled_after_stop(self):
        app = self.app
        app.allow_live.set(True)
        app.process = SimpleNamespace(stdin=io.StringIO())
        app.action = "auto"
        app.live_run = True
        app.live_mode = "position"
        app.handle("approved", None)
        self.assertFalse(app.apply_rpm_button.winfo_manager())
        app.handle("running", None)
        app.next_rpm.set("-25")
        with patch.object(app, "send", return_value=True) as send:
            app.apply_rpm()
            request = json.loads(send.call_args.args[0])
            self.assertEqual(request["rpm"], -25)
            self.assertIn("Requested -25", app.live_text.get())
            self.assertTrue(app.apply_rpm_button.instate(("disabled",)))
            app.handle("rpm_accepted", {"sequence": request["sequence"], "rpm": -25})
            self.assertIn("Accepted -25", app.live_text.get())
            app.stop()
            self.assertFalse(app.apply_rpm_button.winfo_manager())
