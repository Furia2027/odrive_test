#!/usr/bin/env python3
"""Drive a calibrated ODrive axis with the terminal's Up, Down and Space keys."""
import argparse
import errno
import math
import os
import select
import signal
import sys
import termios
import tty
from contextlib import ExitStack, contextmanager

from . import control_m0, torque_m0, vel_m0
from .live import LiveRPM, TELEOP_DOUBLE_TAP_S
from .motor_core import confirm_motion, load_config

KEY_BURST_GAP_S = 0.75
REPEAT_INTERVAL_S = 0.03
BRAKE_TIMEOUT_S = 3.0
BRAKE_SETTLE_S = 0.15

ARROWS = {
    b"\x1b[A": "forward", b"\x1bOA": "forward",
    b"\x1b[B": "reverse", b"\x1bOB": "reverse",
}


class TerminalKeyboard:
    def __init__(self, fd):
        self.fd = fd
        self.escape = b""
        self.last_space = -math.inf
        self.first_space = None

    @contextmanager
    def terminal(self):
        original = termios.tcgetattr(self.fd)
        try:
            tty.setcbreak(self.fd, termios.TCSAFLUSH)
            yield
        finally:
            try:
                termios.tcsetattr(self.fd, termios.TCSAFLUSH, original)
            except termios.error as exc:
                if exc.args[0] != errno.EIO:
                    raise

    def read_events(self, now):
        if not select.select([self.fd], [], [], 0)[0]:
            return ()
        try:
            data = os.read(self.fd, 4096)
        except OSError as exc:
            if exc.errno != errno.EIO:
                raise
            raise EOFError("Terminal input disconnected") from exc
        if not data:
            raise EOFError("Terminal input disconnected")
        return self.decode(data, now)

    def decode(self, data, now):
        events = []
        if b"\x03" in data:
            raise KeyboardInterrupt
        for key in data:
            if key == 27:
                self.escape = b"\x1b"
            elif self.escape:
                self.escape += bytes((key,))
                if len(self.escape) == 2 and key in b"[O":
                    continue
                if len(self.escape) > 2 and len(self.escape) < 32 and 32 <= key <= 63:
                    continue
                event = ARROWS.get(self.escape)
                self.escape = b""
                if event is not None:
                    events.append(event)
            elif key == ord(" "):
                # shortcut: repeat and taps share bytes, use release events if repeat timing changes.
                if now - self.last_space >= KEY_BURST_GAP_S:
                    self.first_space = now
                elif self.first_space is not None:
                    if now - self.first_space <= TELEOP_DOUBLE_TAP_S:
                        events.append("brake")
                    self.first_space = None
                self.last_space = now
        return ("brake",) if "brake" in events else tuple(events)


class TeleopRPM(LiveRPM):
    def __init__(self, ceiling, cfg, *, action, read_events, notify=None):
        super().__init__(ceiling, ceiling, cfg, action=action, take_request=None, emit=None)
        self.start_rpm = self.requested = self.target = 0.0
        self.direction = 0
        self.read_events = read_events
        self.last_arrow = -math.inf
        self.arrow = None
        self.held_since = None
        self.rearm = False
        self.brake_deadline = None
        self.brake_guard = 0.0
        self.notify = notify
        self.last_status = None

    def guard_rpm(self, ramped):
        return max(self.brake_guard, super().guard_rpm(ramped))

    def update(self, now, ramped, estimate, counts, ramp_per_second):
        target = self._update(now, ramped, estimate, counts, ramp_per_second)
        if self.notify is not None:
            if self.braking:
                status = ("braking", "")
            elif self.deadline is not None:
                status = ("reversing", "forward" if self.requested > 0 else "reverse")
            elif target:
                status = ("driving", "forward" if target > 0 else "reverse")
            else:
                status = ("holding", "")
            if status != self.last_status:
                self.last_status = status
                self.notify("teleop_state", dict(zip(("state", "direction"), status, strict=True)))
        return target

    def _update(self, now, ramped, estimate, counts, ramp_per_second):
        self.reversed = False
        try:
            events = tuple(self.read_events(now))
        except EOFError as exc:
            raise vel_m0.DiagnosticStop("operator_link_lost", str(exc)) from exc
        if "brake" in events and not self.braking:
            self.brake_guard = self.guard_rpm(ramped)
            self.requested = self.target = 0.0
            self.sequence += 1
            self.braking = self.rearm = True
            self.brake_deadline = now + BRAKE_TIMEOUT_S
            self.still_since = None
            self.last_arrow = now
        arrows = [event for event in events if event in ("forward", "reverse")]
        if self.braking:
            if arrows:
                self.last_arrow = now
            if now >= self.brake_deadline:
                raise TimeoutError("Active braking could not confirm near-zero speed")
            if counts is None or max(abs(estimate), abs(counts)) > self.threshold:
                self.still_since = None
            elif self.still_since is None:
                self.still_since = now
            elif now - self.still_since >= BRAKE_SETTLE_S:
                self.braking = False
                self.brake_guard = 0.0
                self.direction = 0
                self.deadline = self.still_since = None
            return 0.0

        if arrows:
            fresh = now - self.last_arrow >= KEY_BURST_GAP_S
            self.last_arrow = now
            if self.rearm and not fresh:
                return 0.0
            self.rearm = False
            previous = self.requested
            for arrow in arrows:
                new_direction = arrow != self.arrow
                sign = 1 if arrow == "forward" else -1
                if fresh or new_direction:
                    self.held_since = now
                    step = self.minimum
                else:
                    progress = min(1.0, now - self.held_since)
                    step = (0.25 + 0.75 * progress) * ramp_per_second * REPEAT_INTERVAL_S
                speed = 0.0 if new_direction else abs(self.requested)
                self.requested = sign * min(self.ceiling, speed + step)
                self.arrow = arrow
                fresh = False
            if self.requested != previous:
                self.sequence += 1
        return self._track_target(now, ramped, estimate, counts, ramp_per_second)


def run_teleop(cfg, *, read_events, mode="velocity", ceiling_rpm=None, dev=None,
               confirm=None, check_cancel=None, notify=None):
    """Run one Bench mode. read_events(now) returns forward, reverse or brake events."""
    runners = {"velocity": vel_m0.run, "control": control_m0.run, "torque": torque_m0.run}
    if mode not in runners:
        raise ValueError(f"Unknown teleop mode: {mode}")
    if mode != "velocity" and cfg["axis_index"] != 0:
        raise ValueError("Control and Torque teleop require M0")
    limits = cfg["velocity"]
    minimum = limits["min_command_rpm"]
    maximum = limits["max_command_rpm"]
    if ceiling_rpm is None:
        ceiling_rpm = min(maximum, max(minimum, 50.0))
    # Control selects a runner from the ceiling because every session starts at zero.
    live = TeleopRPM(ceiling_rpm, cfg, action="auto" if mode == "control" else mode,
                     read_events=read_events, notify=notify)
    options = {"capture": True} if mode == "torque" else {}
    return runners[mode](ceiling_rpm, cfg, dev, live=live, confirm=confirm, check_cancel=check_cancel,
                         notify=notify, **options)


def interrupted(signum, _frame):
    raise KeyboardInterrupt(f"Received {signal.Signals(signum).name}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--axis", type=int, choices=(0, 1), default=0)
    parser.add_argument("--mode", choices=("velocity", "control", "torque"), default="velocity")
    parser.add_argument("--max-rpm", type=float, help="approved speed ceiling (default 50 RPM)")
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error("Teleop requires interactive terminal input")
    if args.axis != 0 and args.mode != "velocity":
        parser.error("Control and Torque teleop require M0")
    cfg = load_config(args.axis)
    keyboard = TerminalKeyboard(sys.stdin.fileno())
    starting = True

    with ExitStack() as stack:
        for sig in (signal.SIGHUP, signal.SIGTERM, signal.SIGTSTP):
            previous = signal.signal(sig, interrupted)
            stack.callback(signal.signal, sig, previous)

        def confirm(description):
            print("Up/Down changes speed | double Space brakes | Ctrl+C exits", flush=True)
            confirm_motion(description)
            stack.enter_context(keyboard.terminal())

        def read_events(now):
            nonlocal starting
            if starting:
                # Discard keys entered before closed-loop control was ready.
                termios.tcflush(keyboard.fd, termios.TCIFLUSH)
                starting = False
            return keyboard.read_events(now)

        run_teleop(cfg, mode=args.mode, ceiling_rpm=args.max_rpm, read_events=read_events,
                   confirm=confirm)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nTeleop stopped.")
    except Exception as exc:
        raise SystemExit(f"TELEOP FAILED: {exc}") from exc
