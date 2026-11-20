import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch

from motor import motor_core as core
from motor import tuning
from motor.live import LiveRPM


class LiveTargetTest(unittest.TestCase):
    def test_auto_mode_boundary_invalid_targets_and_unsettled_reversal(self):
        cfg = core.load_config()
        with self.assertRaisesRegex(ValueError, "below 200"):
            LiveRPM(199, 200, cfg, action="auto", take_request=Mock(), emit=Mock())
        for start, cap, invalid in ((30, 199, 200), (220, 250, 199)):
            with self.subTest(start=start):
                emit = Mock()
                commands = [(1, invalid), (2, float("nan")), (3, -start)]
                live = LiveRPM(start, cap, cfg, action="auto",
                               take_request=lambda: commands.pop(0) if commands else None, emit=emit)
                self.assertEqual(live.update(0, start, 0, 0, 60), start)
                self.assertEqual(live.update(0.1, start, 0, 0, 60), start)
                self.assertEqual(live.update(0.2, start, 0, 0, 60), 0)
                self.assertEqual([call.args[0] for call in emit.call_args_list],
                                 ["rpm_rejected", "rpm_rejected", "rpm_accepted"])
                self.assertEqual(live.update(0.3, 0, 0, None, 60), 0)
                self.assertEqual(live.update(0.4, 0, 0, 50, 60), 0)
                with self.assertRaisesRegex(TimeoutError, "near-zero"):
                    live.update(10, 0, 0, 50, 60)


class TuningSaveTest(unittest.TestCase):
    def setUp(self):
        folder = Path(self.enterContext(TemporaryDirectory()))
        self.path = folder / "motor_config.json"
        self.path.write_bytes(core.CONFIG_PATH.read_bytes())
        self.enterContext(patch.object(core, "CONFIG_PATH", self.path))
        self.enterContext(patch.object(core, "MOTION_LOCK", folder / "logs" / ".motion.lock"))
        self.original = self.path.read_bytes()
        self.draft = {"revision": tuning.snapshot(0, "velocity")["revision"],
                      "edits": {"velocity.vel_gain": 0.12}}

    def test_draft_and_save_change_only_edited_field(self):
        self.assertEqual(tuning.draft_config(0, "velocity", self.draft)["velocity"]["vel_gain"], 0.12)
        self.assertEqual(self.path.read_bytes(), self.original)
        saved = tuning.save(0, "velocity", self.draft)
        expected = json.loads(self.original)
        expected["axes"]["m0"]["velocity"]["vel_gain"] = 0.12
        self.assertEqual(json.loads(self.path.read_bytes()), expected)
        self.assertNotEqual(saved["revision"], self.draft["revision"])

    def test_stale_invalid_interrupted_and_busy_saves_keep_file(self):
        for edits in ({"velocity.vel_ramp_rate": -2}, {"hardware.current_lim": 99}):
            with self.assertRaises(ValueError):
                tuning.save(0, "velocity", {**self.draft, "edits": edits})
        with core.exclusive_motion():
            with self.assertRaisesRegex(RuntimeError, "Another motion command"):
                tuning.save(0, "velocity", self.draft)
        with patch.object(tuning.os, "replace", side_effect=OSError("interrupted")):
            with self.assertRaises(OSError):
                tuning.save(0, "velocity", self.draft)
        self.assertEqual(self.path.read_bytes(), self.original)
        self.assertEqual(list(self.path.parent.glob("tmp*")), [])
        external = self.original + b"\n"
        self.path.write_bytes(external)
        with self.assertRaisesRegex(ValueError, "Reload parameters"):
            tuning.save(0, "velocity", self.draft)
        self.assertEqual(self.path.read_bytes(), external)

    def test_legacy_config_save_keeps_legacy_shape(self):
        raw = json.loads(self.original)
        m0 = raw["axes"]["m0"]
        legacy = {**m0, "hardware": {**m0["hardware"], **raw["board"]},
                  "odrive_serial_number": raw["odrive_serial_number"], "runtime": raw["runtime"]}
        self.path.write_text(json.dumps(legacy))
        draft = {"revision": tuning.snapshot(0, "velocity")["revision"],
                 "edits": {"velocity.vel_gain": 0.13}}
        tuning.save(0, "velocity", draft)
        legacy["velocity"]["vel_gain"] = 0.13
        self.assertEqual(json.loads(self.path.read_bytes()), legacy)

    def test_advanced_fields_are_typed_and_validate_before_saving(self):
        snapshot = tuning.editor_snapshot(0, "velocity")
        entries = {field["key"]: field for field in snapshot["advanced"]}
        self.assertEqual(entries["hardware.cpr"]["kind"], "int")
        self.assertTrue(entries["controller.inertia"]["nullable"])
        self.assertFalse(entries["controller.enable_overspeed_error"]["editable"])
        for edits in ({"controller.enable_overspeed_error": False}, {"axis.hardware_verified": False},
                      {"hardware.cpr": 4.5}, {"hardware.calibration_current": 200},
                      {"torque.overspeed_ratio": 0.5}, {"board.dc_max_negative_current": True}):
            with self.subTest(edits=edits), self.assertRaises(ValueError):
                tuning.save(0, "velocity", {**self.draft, "edits": edits}, advanced=True)
            self.assertEqual(self.path.read_bytes(), self.original)
        edits = {"controller.inertia": 0.02, "board.dc_max_positive_current": 2.0}
        tuning.save(0, "velocity", {**self.draft, "edits": edits}, advanced=True)
        cfg = json.loads(self.path.read_bytes())
        self.assertEqual(cfg["axes"]["m0"]["controller"]["inertia"], 0.02)
        self.assertEqual(cfg["board"]["dc_max_positive_current"], 2.0)

    def test_advanced_shared_save_validates_another_axis_and_inspects_unverified(self):
        self.assertTrue(all(not field["editable"] for field in tuning.editor_snapshot(1, "velocity")["advanced"]))
        raw = json.loads(self.original)
        raw["axes"]["m1"] = json.loads(json.dumps(raw["axes"]["m0"]))
        raw["axes"]["m1"]["safety"]["operating_max_v"] = 27.7
        self.path.write_text(json.dumps(raw))
        previous = self.path.read_bytes()
        draft = {"revision": tuning.snapshot(0, "velocity")["revision"],
                 "edits": {"board.dc_bus_overvoltage_trip_level": 27.6}}
        with self.assertRaisesRegex(ValueError, "overvoltage trip"):
            tuning.save(0, "velocity", draft, advanced=True)
        self.assertEqual(self.path.read_bytes(), previous)

    def test_advanced_stale_and_interrupted_changes_are_atomic(self):
        draft = {**self.draft, "edits": {"controller.inertia": 0.02}}
        with patch.object(tuning.os, "replace", side_effect=OSError("interrupted")):
            with self.assertRaises(OSError):
                tuning.save(0, "velocity", draft, advanced=True)
        self.assertEqual(self.path.read_bytes(), self.original)
        external = self.original + b"\n"
        with patch.object(tuning.os, "fsync", side_effect=lambda _: self.path.write_bytes(external)):
            with self.assertRaisesRegex(ValueError, "Reload parameters"):
                tuning.save(0, "velocity", draft, advanced=True)
        self.assertEqual(self.path.read_bytes(), external)
        self.assertEqual(list(self.path.parent.glob("tmp*")), [])
