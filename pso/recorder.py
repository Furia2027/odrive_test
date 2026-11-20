"""Read synchronized M0 recorder exports without altering native trial data."""
import csv
import json
import math
from typing import NamedTuple

import numpy as np


CHANNELS = ("tick", "encoder_shadow_count", "iq_setpoint_a", "iq_measured_a")
MIN_RATE = 200


class RecorderTrace(NamedTuple):
    time: np.ndarray
    commanded: np.ndarray
    applied: np.ndarray


def recorder_paths(directory, trial):
    digest = trial.digest.hex()
    return directory / f"{digest}.csv", directory / f"{digest}.json"


def read_recorder(directory, trial):
    """Return the verified export for this native CSV, or None if absent."""
    digest = trial.digest.hex()
    csv_path, metadata_path = recorder_paths(directory, trial)
    if not csv_path.exists() and not metadata_path.exists():
        return None
    if trial.metrics() is None or trial.torque_calibration is None or trial.stop_elapsed is None:
        raise ValueError("a synchronized recording requires a complete native trial")
    with metadata_path.open(encoding="utf-8") as stream:
        meta = json.load(stream)
    if (type(meta["axis"]) is not int or meta["axis"] != 0 or meta["native_sha256"] != digest
            or not trial.board_serial or meta["board_serial"] != trial.board_serial
            or type(meta["encoder_cpr"]) is not int or meta["encoder_cpr"] != trial.setup.cpr):
        raise ValueError("recorder identity disagrees with the native trial")
    if meta["overflow"] is not False:
        raise ValueError("recorder reported overflow or provided no valid overflow status")
    tick_hz, stride = meta["tick_hz"], meta["tick_stride"]
    if (type(tick_hz) is not int or type(stride) is not int or stride <= 0 or tick_hz < MIN_RATE * stride):
        raise ValueError(f"invalid recorder cadence (at least {MIN_RATE} Hz required)")
    first, last = meta["clock_anchors"]
    first_tick, last_tick = first["tick"], last["tick"]
    start, end = first["elapsed_s"], last["elapsed_s"]
    if (type(first_tick) is not int or type(last_tick) is not int or not 0 <= first_tick < last_tick < 2**63
            or type(start) not in (int, float) or type(end) not in (int, float)
            or not (math.isfinite(start) and math.isfinite(end) and start < end)):
        raise ValueError("invalid clock anchors")
    seconds_per_tick = (end - start) / (last_tick - first_tick)
    if not math.isclose(seconds_per_tick * tick_hz, 1.0, rel_tol=0.01):
        raise ValueError("clock anchors disagree with the declared device tick rate")

    with csv_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream, strict=True)
        if reader.fieldnames != list(CHANNELS):
            raise ValueError("recorder CSV must contain the four synchronized channels in order")
        frames = []
        for row in reader:
            if None in row:
                raise ValueError("unexpected recorder CSV fields")
            frames.append((int(row["tick"]), int(row["encoder_shadow_count"]),
                           float(row["iq_setpoint_a"]), float(row["iq_measured_a"])))
    if len(frames) < 2:
        raise ValueError("too few recorder frames")
    ticks, raw_counts, commanded, measured = map(np.asarray, zip(*frames))
    if not (((first_tick <= ticks) & (ticks <= last_tick)).all() and (np.diff(ticks) == stride).all()):
        raise ValueError("recorder ticks are missing, duplicated, out of order, or outside the clock anchors")
    if (not ((-2**31 <= raw_counts) & (raw_counts < 2**31)).all()
            or not (np.isfinite(commanded).all() and np.isfinite(measured).all())):
        raise ValueError("invalid recorder count or current")
    time = start + (ticks - first_tick) * seconds_per_tick
    if time[0] > 1e-9 or time[-1] < trial.stop_elapsed - 1e-9:
        raise ValueError("recorder does not cover the complete native trial")

    # Match the native encoder's rollover convention before checking clock alignment.
    increments = (np.diff(raw_counts) + 2**31) % 2**32 - 2**31
    counts = np.r_[raw_counts[0], raw_counts[0] + np.cumsum(increments)]
    right = np.clip(np.searchsorted(time, trial.time), 1, len(time) - 1)
    counts += round((trial.counts[0] - counts[right[0]]) / 2**32) * 2**32
    low, high = np.minimum(counts[right - 1], counts[right]), np.maximum(counts[right - 1], counts[right])
    agreement = (low - 1 <= trial.counts) & (trial.counts <= high + 1)
    if np.mean(agreement) < 0.95:
        raise ValueError("recorder clock alignment disagrees with native encoder counts")
    scale, _ = trial.torque_calibration
    commanded, measured = commanded * scale, measured * scale
    if not (np.isfinite(commanded).all() and np.isfinite(measured).all()):
        raise ValueError("invalid recorder torque after unit conversion")
    return RecorderTrace(time, commanded, measured)
