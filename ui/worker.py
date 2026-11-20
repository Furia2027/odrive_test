"""Isolate USB access and the existing motor runners from desktop rendering."""
import argparse
from contextlib import redirect_stdout
import json
import os
from queue import Empty, Full, Queue
import sys
from threading import Event, Thread
import time

from fibre import ChannelBrokenException
from odrive.enums import AXIS_STATE_IDLE, ENCODER_MODE_INCREMENTAL

from motor import calibrate_m0
from motor import control_m0
from motor import encoder_m0
from motor import motor_core as core
from motor import torque_m0
from motor import teleop_motor
from motor import vel_m0
from motor import tuning
from motor.live import LiveRPM, effective_mode
from motor.m0_display import ERROR_KEYS


ACTIONS = {
    "velocity": vel_m0.run,
    "auto": control_m0.run,
    "torque": torque_m0.run,
}
HEARTBEAT_TIMEOUT = 2.0
TELEOP_ARROW_MAX_AGE = 0.25
PSO_ACTIONS = ("pso_info", "pso_analyze", "pso_reference", "pso_validate")


def drive_status():
    with core.exclusive_motion():
        cfg = core.load_config(read_only=True)
        dev = core.find_odrive(cfg)
        core.verify_board(dev, cfg)
        axes = (dev.axis0, dev.axis1)
        states = [axis.current_state for axis in axes]
        idle = all(state == AXIS_STATE_IDLE for state in states)
        status = []
        for index, axis in enumerate(axes):
            faults = dict(zip(ERROR_KEYS, core.fault_codes(axis, sensorless=True), strict=True))
            info = {
                "state": states[index],
                "idle": states[index] == AXIS_STATE_IDLE,
                "encoder_ready": axis.encoder.is_ready,
                "faults": faults,
                "calibration_issue": "",
                "motion_issue": "",
                "rpm_range": None,
            }
            try:
                settings = core.load_config(index)
            except (ValueError, KeyError, TypeError) as exc:
                info.update(calibration_issue=str(exc), motion_issue=str(exc))
                status.append(info)
                continue
            velocity = settings["velocity"]
            info["rpm_range"] = (velocity["min_command_rpm"], velocity["max_command_rpm"])
            try:
                if not idle:
                    raise RuntimeError("Both axes must be IDLE.")
                if not settings["encoder_present"] or axis.encoder.config.mode != ENCODER_MODE_INCREMENTAL:
                    raise RuntimeError("A/B calibration requires an existing incremental encoder configuration.")
                core.check_power(dev, settings)
            except RuntimeError as exc:
                info["calibration_issue"] = str(exc)
            try:
                core.preflight(dev, settings)
            except RuntimeError as exc:
                info["motion_issue"] = str(exc)
            status.append(info)
        return {"serial": cfg["odrive_serial_number"], "both_idle": idle, "axes": status}


class Operator:
    def __init__(self, stream, emit, output_failed, *, live_enabled=False, teleop_enabled=False):
        self.emit = emit
        self.approval = Queue(maxsize=1)
        self.requests = Queue(maxsize=1)
        self.cancelled = Event()
        self.output_failed = output_failed
        self.live_enabled = live_enabled
        self.teleop_enabled = teleop_enabled
        self.teleop_events = Queue(maxsize=32)
        self.teleop_overflow = False
        self.active = False
        self.running = False
        self.last_heartbeat = None
        self.user_stop = False
        self.pso_sequence = 0
        self.pending_pso = None
        self.pso_answer = Queue(maxsize=1)
        Thread(target=self.listen, args=(stream,), daemon=True).start()

    def listen(self, stream):
        approved = False
        last_sequence = 0
        try:
            for message in stream:
                if message == b"stop\n":
                    self.user_stop = True
                    break
                if message == b"confirm\n" and not approved:
                    approved = True
                    self.approval.put(True)
                elif message == b"heartbeat\n":
                    self.last_heartbeat = time.monotonic()
                elif message.startswith(b"{"):
                    try:
                        request = json.loads(message)
                        if not isinstance(request, dict):
                            continue
                        if "teleop" in request:
                            event = request["teleop"]
                            if self.teleop_enabled and self.running and event in ("forward", "reverse", "brake"):
                                try:
                                    self.teleop_events.put_nowait((time.monotonic(), event))
                                except Full:
                                    self.teleop_overflow = True
                            continue
                        if "pso_id" in request:
                            prompt = request["pso_id"]
                            if type(prompt) is int and prompt == self.pending_pso:
                                self.pending_pso = None
                                self.pso_answer.put_nowait(request.get("value"))
                            continue
                        seq, rpm = request["sequence"], request["rpm"]
                        if type(seq) is not int or seq <= last_sequence:
                            continue
                    except (ValueError, TypeError, KeyError):
                        continue
                    last_sequence = seq
                    if self.running and self.live_enabled:
                        try:
                            self.requests.put_nowait((seq, rpm))
                            continue
                        except Full:
                            detail = "An RPM request is already pending."
                    else:
                        detail = "Live RPM is unavailable in this operation."
                    self.emit("rpm_rejected", {"sequence": seq, "detail": detail})
        finally:
            self.cancelled.set()
            if not approved:
                self.approval.put(False)

    def check_cancel(self):
        if self.user_stop:
            raise KeyboardInterrupt("Stopped by operator")
        if self.cancelled.is_set() or self.output_failed.is_set():
            raise vel_m0.DiagnosticStop("operator_link_lost", "Bench window connection closed")
        if self.active and time.monotonic() - self.last_heartbeat >= HEARTBEAT_TIMEOUT:
            raise vel_m0.DiagnosticStop("operator_link_lost", "Bench window heartbeat timed out")
        if self.teleop_overflow:
            raise vel_m0.DiagnosticStop("operator_link_lost", "Teleop input queue overflowed")

    def take_teleop(self, now):
        self.check_cancel()
        events = []
        for _ in range(self.teleop_events.maxsize):
            try:
                stamped, event = self.teleop_events.get_nowait()
            except Empty:
                break
            if event == "brake" or now - stamped <= TELEOP_ARROW_MAX_AGE:
                events.append(event)
        self.check_cancel()
        return ("brake",) if "brake" in events else events

    def take_request(self):
        try:
            return self.requests.get_nowait()
        except Empty:
            return None

    def notify(self, event, data):
        if event == "running":
            self.running = True
        elif event == "trial_result":
            self.running = False
        self.emit(event, data)

    def confirm(self, description):
        self.check_cancel()
        self.emit("confirm", description)
        if not self.approval.get():
            raise KeyboardInterrupt("Cancelled before motion")
        self.mark_approved()

    def mark_approved(self):
        self.check_cancel()
        self.last_heartbeat = time.monotonic()
        self.active = True
        self.emit("approved", None)
        self.check_cancel()

    def ask_pso(self, kind, **details):
        self.check_cancel()
        self.pso_sequence += 1
        self.pending_pso = self.pso_sequence
        self.emit("pso_prompt", {"kind": kind, "id": self.pso_sequence, **details})
        while True:
            self.check_cancel()
            try:
                answer = self.pso_answer.get(timeout=0.05)
            except Empty:
                continue
            self.check_cancel()
            return answer


class RunnerOutput:
    """Turn runner notices into events and leave readings in the CSV."""
    def __init__(self, emit):
        self.emit = emit
        self.pending = ""

    def write(self, part):
        self.pending += part
        while "\n" in self.pending:
            line, self.pending = self.pending.split("\n", 1)
            line = line.strip()
            if line and not line.startswith(("|", "+-", "==", "ODRIVE M", "Each row", "Samples collected")):
                self.emit("notice", line)
        return len(part)

    def flush(self):
        pass


def perform_pso(action, rpm, operator, options):
    from pso import pso_m0 as pso

    mode = options.get("mode")
    if mode not in pso.SPEEDS:
        raise ValueError("Choose Position or Torque for PSO tuning.")
    snapshot = tuning.snapshot(0, mode)
    revision = snapshot["revision"]

    def display_snapshot(data):
        return {"mode": mode, "fields": [field for field in data["fields"] if field[0] in pso.GAIN_FIELDS[mode]],
                "bounds": pso.BOUNDS[mode], "revision": data["revision"]}

    info = display_snapshot(snapshot)
    baseline = tuple(value for _, _, value in info["fields"])
    operator.emit("pso_info", info)
    if action == "pso_info":
        return

    cfg = tuning.draft_config(0, mode, pso.make_draft(mode, revision, baseline))
    if action in ("pso_reference", "pso_validate") and options.get("revision") != revision:
        raise ValueError("Configuration changed. Analyze logs again before proceeding.")

    guarded_speed = None
    capture_label = "Reference"

    def approve(description, rpm):
        nonlocal guarded_speed
        description = f"{capture_label}\n{description}"
        if guarded_speed is not None:
            response = operator.ask_pso("guard", description=description, rpm=guarded_speed)
            if response != str(guarded_speed):
                raise KeyboardInterrupt("Guarded speed not confirmed")
            guarded_speed = None
        if operator.ask_pso("motion", description=description, rpm=rpm) != "confirm":
            raise KeyboardInterrupt("Motion not confirmed")
        operator.mark_approved()

    def notify(event, data):
        if event == "trial_result":
            operator.active = False
        operator.notify(event, data)

    def capture(mode, rpm, draft, *, expected_setup=None):
        trial = pso.run_trial(mode, rpm, draft, approve=lambda description: approve(description, rpm),
                              check_cancel=operator.check_cancel, emit=notify, expected_setup=expected_setup)
        operator.emit("pso_capture", {"rpm": rpm, "csv": str(trial.path), "metrics": trial.metrics()})
        return trial

    if action == "pso_reference":
        reference = pso.reference_pair(mode, rpm, options.get("gains", baseline))
        capture(mode, rpm, pso.make_draft(mode, revision, reference))
        return

    def report(progress):
        operator.emit("pso_analysis", progress)

    operator.check_cancel()
    trials = pso.load_trials(mode, cfg, report=report, check_cancel=operator.check_cancel)
    selection = pso.optimize(mode, trials, baseline, report=report, check_cancel=operator.check_cancel,
                             recorder_dir=pso.position_recorder_dir(mode))
    operator.check_cancel()
    if action == "pso_analyze":
        if selection.full:
            tuning.draft_config(0, mode, pso.make_draft(mode, revision, selection.gains))
            operator.emit("pso_candidate", {"gains": selection.gains, "score": selection.score, "revision": revision})
        return
    if not selection.full:
        raise ValueError("Provisional gains are offline only. Bench validation and saving require full qualification.")
    candidate = selection.gains
    if candidate != pso.bounded_pair(mode, options.get("candidate")):
        raise ValueError("Offline finalist changed. Analyze logs again to review the new candidate.")

    def validate_trial(mode, rpm, draft, *, expected_setup):
        nonlocal capture_label
        pair = tuple(draft["edits"].values())
        capture_label = "Baseline validation" if pair == baseline else "Finalist validation"
        return capture(mode, rpm, draft, expected_setup=expected_setup)

    def guard_stage(rpm):
        nonlocal guarded_speed
        guarded_speed = rpm

    pso.bench(mode, revision, baseline, selection, trials, capture=validate_trial, guard_stage=guard_stage,
              report=lambda row: operator.emit("pso_comparison", row))
    tuning.draft_config(0, mode, pso.make_draft(mode, revision, candidate))
    if operator.ask_pso("save", baseline=baseline, gains=candidate) != "save":
        raise KeyboardInterrupt("Validated gains discarded")
    operator.check_cancel()
    operator.emit("pso_saved", display_snapshot(pso.apply_result(mode, revision, candidate)))


def perform(action, axis, rpm, operator, options=None):
    options = options or {}
    operator.check_cancel()
    if action in PSO_ACTIONS:
        if axis != 0:
            raise ValueError("PSO tuning requires M0.")
        perform_pso(action, rpm, operator, options)
        return
    if action == "status":
        operator.emit("status", drive_status())
        return
    if action in ("tuning", "save", "advanced_save"):
        mode = options["mode"]
        if action == "advanced_save":
            info = tuning.save(axis, mode, options["draft"], advanced=True)
        else:
            if action == "save":
                tuning.save(axis, mode, options["draft"])
            info = tuning.editor_snapshot(axis, mode)
        operator.emit("tuning", info)
        return
    if action in ("auto", "torque") and axis != 0:
        raise ValueError("Control and Torque tests require M0.")
    draft = options.get("draft")
    cfg = (tuning.draft_config(axis, effective_mode(action, rpm), draft) if draft and action in ACTIONS
           else core.load_config(axis, read_only=action in ("clear", "encoder")))
    operator.check_cancel()
    if action == "clear":
        calibrate_m0.clear_errors(cfg)
        return
    if action == "encoder":
        encoder_m0.run(cfg, check_cancel=operator.check_cancel, notify=operator.emit)
        return
    interaction = {"confirm": operator.confirm, "check_cancel": operator.check_cancel}
    if action == "teleop":
        teleop_motor.run_teleop(cfg, mode=options["mode"], ceiling_rpm=options["ceiling"],
                                read_events=operator.take_teleop, notify=operator.notify, **interaction)
        return
    if action == "calibrate":
        calibrate_m0.calibrate(cfg, allow_encoder_switch=False, **interaction)
        return
    interaction["notify"] = operator.notify
    if "ceiling" in options:
        interaction["live"] = LiveRPM(rpm, options["ceiling"], cfg, action=action,
                                      take_request=operator.take_request, emit=operator.emit)
    if action == "torque":
        interaction["capture"] = True
    ACTIONS[action](rpm, cfg, **interaction)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("status", "clear", "calibrate", "encoder", "teleop", "tuning", "save",
                                          "advanced_save", *PSO_ACTIONS, *ACTIONS))
    parser.add_argument("axis", type=int, choices=(0, 1))
    parser.add_argument("--rpm", type=float)
    parser.add_argument("--options", type=json.loads, default={})
    args = parser.parse_args()
    if args.action in ACTIONS and args.rpm is None:
        parser.error("--rpm is required for a test")
    output = sys.stdout.fileno()
    output_failed = Event()
    events = Queue(maxsize=128)

    def emit(event, data):
        try:
            events.put_nowait((event, data))
        except Full:
            output_failed.set()

    def publish():
        try:
            while (item := events.get()) is not None:
                payload = memoryview((json.dumps(item, allow_nan=False) + "\n").encode("utf-8"))
                while payload:
                    payload = payload[os.write(output, payload):]
        except (OSError, ValueError):
            output_failed.set()

    writer = Thread(target=publish, daemon=True)
    writer.start()
    operator = Operator(os.fdopen(sys.stdin.fileno(), "rb", buffering=0, closefd=False),
                        emit, output_failed, live_enabled=args.action in ACTIONS and "ceiling" in args.options,
                        teleop_enabled=args.action == "teleop")
    try:
        with redirect_stdout(RunnerOutput(emit)):
            perform(args.action, args.axis, args.rpm, operator, args.options)
    except KeyboardInterrupt as exc:
        emit("cancelled", str(exc) or "Operation cancelled.")
        return 130
    except Exception as exc:
        detail = str(exc) or type(exc).__name__
        if args.action in PSO_ACTIONS and isinstance(exc, ChannelBrokenException):
            detail = f"Drive communication lost. {detail}"
        emit("error", detail)
        return 1
    finally:
        try:
            events.put_nowait(None)
        except Full:
            pass
        writer.join(timeout=0.5)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
