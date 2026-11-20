#!/usr/bin/env python3
"""Display the selected axis's live encoder count while it is idle."""
import argparse
import time

from odrive.enums import AXIS_STATE_IDLE

from .motor_core import axis_name, exclusive_motion, find_odrive, load_config, select_axis


@exclusive_motion()
def run(cfg, *, check_cancel=None, notify=None):
    axis = select_axis(find_odrive(cfg), cfg)
    name = axis_name(cfg)
    start = None
    while True:
        if check_cancel is not None:
            check_cancel()
        if axis.current_state != AXIS_STATE_IDLE:
            raise RuntimeError(f"{name} must be IDLE before turning the shaft by hand")
        count = axis.encoder.shadow_count
        if start is None:
            start = count
            if notify is None:
                print(f"{name} IDLE. Turn the shaft by hand. Press Ctrl+C to stop.")
        if notify is None:
            print(f"\rshadow_count {count:12d}  change from start {count - start:+12d}",
                  end="", flush=True)
        else:
            delta = (count - start + 2**31) % 2**32 - 2**31
            notify("encoder_sample", {"count": count, "delta": delta})
        time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--axis", type=int, choices=(0, 1), default=0)
    run(load_config(parser.parse_args().axis, read_only=True))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
    except RuntimeError as exc:
        raise SystemExit(f"\nEncoder monitor stopped. {exc}") from exc
