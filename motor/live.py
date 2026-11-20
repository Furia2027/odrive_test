"""Signed targets shared by the live versions of the bench runners."""
import math

POSITION_BOUNDARY_RPM = 200.0
TELEOP_DOUBLE_TAP_S = 0.30
TARGET_COLUMNS = (
    ("requested_rpm", "Requested RPM", "+g"),
    ("active_target_rpm", "Target RPM", "+g"),
    ("target_sequence", "Target #", "d"),
)


def effective_mode(action, rpm):
    if action == "auto":
        return "position" if abs(rpm) < POSITION_BOUNDARY_RPM else "torque"
    return action


def slew_rpm(current, target, step):
    delta = target - current
    return target if abs(delta) <= step else current + math.copysign(step, delta)


class LiveRPM:
    braking = False

    def __init__(self, rpm, ceiling, cfg, *, action, take_request, emit):
        limits = cfg["velocity"]
        self.minimum = limits["min_command_rpm"]
        self.action = action
        if type(rpm) not in (int, float) or not math.isfinite(rpm):
            raise ValueError("A finite starting RPM is required.")
        self.mode = effective_mode(action, rpm)
        maximum = limits["max_command_rpm"]
        if (not self.minimum <= abs(rpm) <= maximum
                or type(ceiling) not in (int, float) or not abs(rpm) <= ceiling <= maximum):
            raise ValueError(f"Live RPM ceiling must cover the starting target and be at most {maximum:g} RPM.")
        if self.mode == "position" and ceiling >= POSITION_BOUNDARY_RPM:
            raise ValueError(f"Position live ceiling must be below {POSITION_BOUNDARY_RPM:g} RPM.")
        self.ceiling = ceiling
        self.threshold = cfg["safety"]["stationary_max_rpm"]
        self.hold = cfg["calibration"]["settle_hold_s"]
        self.timeout = cfg["calibration"]["settle_timeout_s"]
        self.start_rpm = self.requested = self.target = rpm
        self.direction = math.copysign(1, rpm)
        self.sequence = 0
        self.deadline = self.still_since = None
        self.reversed = False
        self.take_request = take_request
        self.emit = emit

    def confirmation(self, ramp):
        return f" | live ceiling {self.ceiling:g} RPM | ramp {ramp * 60:g} RPM/s"

    def values(self):
        return self.requested, self.target, self.sequence

    def guard_rpm(self, ramped):
        return max(abs(self.target), abs(ramped))

    def update(self, now, ramped, estimate, counts, ramp_per_second):
        request = self.take_request()
        if request is not None:
            seq, rpm = request
            valid = type(rpm) in (float, int) and (
                rpm == 0 or self.minimum <= abs(rpm) <= self.ceiling)
            if not valid:
                self.emit("rpm_rejected", {"sequence": seq, "detail": "Target is outside the approved live RPM range."})
            elif rpm and effective_mode(self.action, rpm) != self.mode:
                self.emit("rpm_rejected", {"sequence": seq,
                                          "detail": f"Start a new Control run to cross {POSITION_BOUNDARY_RPM:g} RPM."})
            else:
                self.requested, self.sequence = rpm, seq
                self.emit("rpm_accepted", {"sequence": seq, "rpm": rpm})
        return self._track_target(now, ramped, estimate, counts, ramp_per_second)

    def _track_target(self, now, ramped, estimate, counts, ramp_per_second):
        self.reversed = False
        desired = self.requested
        # A zero hold retains the last driven direction for future reversals.
        if desired == 0:
            self.deadline = self.still_since = None
            self.target = 0.0
        elif self.deadline is not None or self.direction * desired < 0:
            if self.deadline is None:
                self.deadline = now + abs(ramped) / ramp_per_second + self.timeout
                self.still_since = None
            self.target = 0.0
            if now >= self.deadline:
                raise TimeoutError("Live reversal could not confirm near-zero speed")
            if ramped == 0 and counts is not None and max(abs(estimate), abs(counts)) <= self.threshold:
                if self.still_since is None:
                    self.still_since = now
                elif now - self.still_since >= self.hold:
                    self.reversed = self.direction * desired < 0
                    self.direction = math.copysign(1, desired)
                    self.target = desired
                    self.deadline = self.still_since = None
            else:
                self.still_since = None
        else:
            self.target = desired
            if self.direction == 0:
                self.direction = math.copysign(1, desired)
        return self.target
