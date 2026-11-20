from contextlib import nullcontext, redirect_stdout, redirect_stderr
import csv
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE

from pso import usb_recorder


class UsbRecorderTest(unittest.TestCase):
    def test_read_timestamps_and_skipped_schedule_slots(self):
        now = 0

        def sleep(seconds):
            nonlocal now
            now += round(seconds * usb_recorder.NANOSECONDS)

        class Encoder:
            @property
            def shadow_count(self):
                sleep(0.012 if now == 0 else 0.001)
                return 21

        class Current:
            @property
            def Iq_setpoint(self):
                sleep(0.001)
                return -0.5

            @property
            def Iq_measured(self):
                sleep(0.001)
                return 0.25

        axis = SimpleNamespace(encoder=Encoder(), motor=SimpleNamespace(current_control=Current()))
        check_idle = Mock()
        with (patch.object(usb_recorder.time, "monotonic_ns", side_effect=lambda: now),
              patch.object(usb_recorder.time, "sleep", side_effect=sleep)):
            frames = usb_recorder.capture_frames(axis, check_idle, period_ns=5_000_000, duration_ns=20_000_000)
        self.assertEqual([frame.slot for frame in frames], [0, 2, 3])
        self.assertEqual(frames[0].encoder, (0, 21, 12_000_000))
        self.assertEqual(frames[0].commanded, (12_000_000, -0.5, 13_000_000))
        self.assertEqual(frames[0].measured, (13_000_000, 0.25, 14_000_000))
        self.assertEqual(check_idle.call_count, 2)

    def test_refuses_motion_and_preserves_existing_files(self):
        drive = SimpleNamespace(axis0=SimpleNamespace(current_state=AXIS_STATE_CLOSED_LOOP_CONTROL),
                                axis1=SimpleNamespace(current_state=AXIS_STATE_IDLE))
        with TemporaryDirectory() as folder:
            output = Path(folder) / "recording.csv"
            with (patch("sys.argv", ["usb_recorder", "--output", str(output)]),
                  patch.object(usb_recorder.motor_core, "load_config", return_value={}),
                  patch.object(usb_recorder.motor_core, "exclusive_motion", return_value=nullcontext()),
                  patch.object(usb_recorder.motor_core, "find_odrive", return_value=drive) as find,
                  patch.object(usb_recorder.motor_core, "verify_board"), redirect_stdout(StringIO()),
                  redirect_stderr(StringIO()) as errors):
                self.assertEqual(usb_recorder.main(), 1)
                self.assertIn("Both axes must be IDLE", errors.getvalue())
                self.assertFalse(output.exists())

                output.write_text("existing data", encoding="utf-8")
                find.reset_mock()
                self.assertEqual(usb_recorder.main(), 1)
                self.assertEqual(output.read_text(encoding="utf-8"), "existing data")
                find.assert_not_called()

    def test_writes_host_time_brackets_and_values_to_csv(self):
        frame = usb_recorder.Frame(0, 0, usb_recorder.Reading(10, 345, 20),
                                   usb_recorder.Reading(20, -0.8, 35), usb_recorder.Reading(35, 0.4, 42))
        with TemporaryDirectory() as folder:
            output = Path(folder) / "test.csv"
            with (patch("sys.argv", ["usb_recorder", "--output", str(output)]),
                  patch.object(usb_recorder.motor_core, "load_config", return_value={}),
                  patch.object(usb_recorder.motor_core, "exclusive_motion", return_value=nullcontext()),
                  patch.object(usb_recorder.motor_core, "find_odrive"), patch.object(usb_recorder.motor_core, "verify_board"),
                  patch.object(usb_recorder, "capture_frames", return_value=[frame]) as capture, redirect_stdout(StringIO())):
                self.assertEqual(usb_recorder.main(), 0)
            with output.open(newline="", encoding="utf-8") as stream:
                row, = csv.DictReader(stream)
        self.assertEqual(capture.call_args.args[2:], (5_000_000, 15_000_000_000))
        self.assertEqual(row["encoder_shadow_count"], "345")
        self.assertEqual(row["iq_setpoint_a"], "-0.8")
        self.assertEqual(row["iq_measured_a"], "0.4")
        self.assertEqual(row["iq_measured_host_before_ns"], "35")
        self.assertEqual(row["iq_measured_host_after_ns"], "42")
