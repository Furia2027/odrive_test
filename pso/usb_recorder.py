"""Record timestamped M0 USB feedback while both axes are IDLE."""
import argparse
import csv
from datetime import datetime, timezone
import math
from pathlib import Path
from statistics import median
import sys
import time
from typing import NamedTuple

from motor import motor_core
from motor.paths import LOGS


NANOSECONDS = 1_000_000_000
HEADER = (
    "slot", "scheduled_host_elapsed_ns",
    "encoder_host_before_ns", "encoder_shadow_count", "encoder_host_after_ns",
    "iq_setpoint_host_before_ns", "iq_setpoint_a", "iq_setpoint_host_after_ns",
    "iq_measured_host_before_ns", "iq_measured_a", "iq_measured_host_after_ns",
)


class Reading(NamedTuple):
    before_ns: int
    value: int | float
    after_ns: int


class Frame(NamedTuple):
    slot: int
    scheduled_ns: int
    encoder: Reading
    commanded: Reading
    measured: Reading


def read_signal(owner, name, origin):
    before = time.monotonic_ns()
    value = getattr(owner, name)
    after = time.monotonic_ns()
    if not math.isfinite(value):
        raise RuntimeError(f"Invalid M0 reading for {name}")
    return Reading(before - origin, value, after - origin)


def capture_frames(axis, check_idle, period_ns, duration_ns):
    check_idle()
    encoder, current = axis.encoder, axis.motor.current_control
    origin = time.monotonic_ns()
    stop = origin + duration_ns
    next_idle_check = origin + NANOSECONDS
    slot = 0
    frames = []
    while (release := origin + slot * period_ns) < stop:
        delay = release - time.monotonic_ns()
        if delay > 0:
            time.sleep(delay / NANOSECONDS)
        now = time.monotonic_ns()
        if now >= next_idle_check:
            check_idle()
            now = time.monotonic_ns()
            next_idle_check = now + NANOSECONDS
        if now >= stop:
            break
        # Keep the original schedule and leave a slot gap when a frame runs late.
        slot = max(slot, (now - origin) // period_ns)
        frames.append(Frame(slot, slot * period_ns,
                            read_signal(encoder, "shadow_count", origin),
                            read_signal(current, "Iq_setpoint", origin),
                            read_signal(current, "Iq_measured", origin)))
        slot += 1
    check_idle()
    return frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=int, default=200, help="requested frames per second (default: 200)")
    parser.add_argument("--duration", type=int, default=15, help="recording duration in seconds (default: 15)")
    parser.add_argument("--output", type=Path, help="new CSV path (default: a timestamped file in logs/)")
    args = parser.parse_args()
    if not 0 < args.rate <= NANOSECONDS or args.duration <= 0:
        parser.error("rate and duration must be positive, at most one frame per nanosecond")
    period_ns = round(NANOSECONDS / args.rate)
    duration_ns = args.duration * NANOSECONDS
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    output = args.output or LOGS / f"m0_usb_idle_{timestamp}.csv"

    try:
        if output.exists():
            raise FileExistsError(f"Output already exists: {output}")
        cfg = motor_core.load_config(0, read_only=True)
        with motor_core.exclusive_motion():
            dev = motor_core.find_odrive(cfg)
            motor_core.verify_board(dev, cfg)
            print(f"Recording M0 IDLE feedback for {args.duration} s at {args.rate} Hz.", flush=True)
            frames = capture_frames(dev.axis0, lambda: motor_core.ensure_idle(dev, cfg, check_faults=False),
                                    period_ns, duration_ns)
        if not frames:
            raise RuntimeError("No feedback frames captured")
        with output.open("x", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(HEADER)
            writer.writerows((f.slot, f.scheduled_ns, *f.encoder, *f.commanded, *f.measured) for f in frames)
    except KeyboardInterrupt:
        print("USB recording interrupted.", file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"USB recording stopped: {exc}", file=sys.stderr)
        return 1

    scheduled_frames = (duration_ns + period_ns - 1) // period_ns
    print(f"Saved {len(frames)}/{scheduled_frames} frames to {output.resolve()} · both axes IDLE.")
    if len(frames) > 1:
        intervals = [b.encoder.before_ns - a.encoder.before_ns for a, b in zip(frames, frames[1:])]
        rate = len(intervals) * NANOSECONDS / sum(intervals)
        print(f"Actual rate {rate:.2f} Hz · max frame interval {max(intervals) / 1e6:.3f} ms")
    skew = [(f.measured.before_ns + f.measured.after_ns - f.encoder.before_ns - f.encoder.after_ns) / 2e6
            for f in frames]
    print(f"Encoder-to-measured-current USB midpoint separation · median {median(skew):.3f} ms, max {max(skew):.3f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
