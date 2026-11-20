#!/usr/bin/env python3
"""Human-readable axis telemetry with full-resolution CSV logging.

No dependencies beyond the Python standard library. Printing is intentionally
rate-limited. Every sample is recorded to CSV.
"""
import csv
from datetime import datetime, timezone
import json
from .paths import LOGS


class TelemetryTable:
    """columns: iterable of (CSV key, human label, format spec)."""

    def __init__(self, mode, columns, *, cfg, target_rpm, metadata, compact=False,
                 hidden_columns=(), notify=None, show_readings=None):
        self.mode = str(mode).upper()
        self.compact = compact
        self.columns = tuple(columns)
        self.keys = tuple(c[0] for c in self.columns) + ERROR_KEYS
        self.display_every_s = cfg["runtime"]["display_every_s"]
        self.identity = (cfg["axis_index"], self.mode, target_rpm)
        self.next_print_s = -1.0
        self.displayed = 0
        self.samples = 0
        self.last_values = None
        self.csv_file = None
        self.notify = notify
        self.show_readings = not notify if show_readings is None else show_readings

        LOGS.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        name = f"m{self.identity[0]}_{self.mode.lower().replace(' ', '_')}_{stamp}_{target_rpm:+g}rpm.csv"
        self.log_path = LOGS / name
        self.csv_file = self.log_path.open("x", encoding="utf-8", newline="")
        try:
            self.writer = csv.writer(self.csv_file)
            self.writer.writerow(("utc_time", "axis", "control_mode", "target_rpm", "event",
                                  *self.keys, "stop_reason", "detail", "idle_confirmed", "config_json"))
            snapshot = json.dumps({"config": cfg, "trial": metadata}, sort_keys=True, allow_nan=False)
            self._row("start", [""] * len(self.keys), config_json=snapshot)
            self.csv_file.flush()
            if notify:
                notify("csv", str(self.log_path))
            if self.show_readings:
                if self.compact:
                    print(f"CSV: {self.log_path}", flush=True)
                else:
                    self._start_display(hidden_columns)
        except BaseException:
            self.csv_file.close()
            raise

    def _row(self, event, values, *, reason="", detail="", idle="", config_json="", utc_time=None):
        utc_time = utc_time if utc_time is not None else datetime.now(timezone.utc)
        self.writer.writerow((utc_time.isoformat(), *self.identity, event,
                              *values, reason, detail, idle, config_json))

    def _start_display(self, hidden_columns):
        self.display_columns = tuple(
            (index, label, spec) for index, (key, label, spec) in enumerate(self.columns)
            if key not in hidden_columns
        )
        self.widths = tuple(max(len(label), 9) for _, label, _ in self.display_columns)
        self.rule = "+-" + "-+-".join("-" * w for w in self.widths) + "-+"
        self.header = "| " + " | ".join(
            label.center(w) for (_, label, _), w in zip(self.display_columns, self.widths, strict=True)
        ) + " |"

        print("\n" + "=" * min(len(self.rule), 92), flush=True)
        print(f" ODRIVE M{self.identity[0]}  |  {self.mode} CONTROL  |  LIVE READINGS", flush=True)
        print("=" * min(len(self.rule), 92), flush=True)
        print(" Each row is a snapshot. Full-rate samples are saved as CSV.", flush=True)
        print(f" CSV: {self.log_path}", flush=True)
        self._header()

    def _header(self):
        print(self.rule, flush=True)
        print(self.header, flush=True)
        print(self.rule, flush=True)

    def sample(self, elapsed_s, *values, errors, utc_time=None):
        """Record every sample; display approximately every display_every_s."""
        if len(values) != len(self.columns):
            raise ValueError(f"{self.mode}: expected {len(self.columns)} values, got {len(values)}")
        self.samples += 1
        self.last_values = (*values, *errors)
        self._row("sample", self.last_values, utc_time=utc_time)

        if not self.show_readings or elapsed_s + 1e-9 < self.next_print_s:
            return
        if self.compact:
            readings = dict(zip(self.keys, self.last_values, strict=True))
            status = (f"Speed {readings['estimated_rpm']:+.0f} RPM | "
                      f"Command {readings['command_rpm']:+.0f} RPM | "
                      f"Ceiling {readings['ceiling_rpm']:.0f} RPM | Ramp {readings['ramp_rpm_per_s']:.4g} RPM/s")
            print(f"\r{status}\x1b[K", end="", flush=True)
        else:
            if self.displayed and self.displayed % 15 == 0:
                self._header()
            cells = []
            for (index, _, spec), width in zip(self.display_columns, self.widths, strict=True):
                value = values[index]
                try:
                    txt = format(value, spec)
                except (TypeError, ValueError):
                    txt = str(value)
                cells.append(txt.rjust(width))
            print("| " + " | ".join(cells) + " |", flush=True)
        self.displayed += 1
        self.next_print_s = elapsed_s + self.display_every_s

        # Keep the CSV reasonably up to date without flushing on every sample.
        self.csv_file.flush()

    def finish(self, reason, message, elapsed_s, idle, errors, *, extra=None, utc_time=None):
        values = dict(zip(self.keys, self.last_values, strict=True)) if self.last_values is not None else {}
        values["elapsed_s"] = elapsed_s
        if extra:
            values.update(extra)
        if errors is not None:
            values.update(zip(ERROR_KEYS, errors, strict=True))
        self._row("stop", (values.get(key, "") for key in self.keys),
                  reason=reason, detail=message, idle=idle, utc_time=utc_time)
        self.csv_file.flush()
        if self.notify:
            self.notify("trial_result", {"reason": reason, "detail": message, "idle": idle,
                                         "faults": dict(zip(ERROR_KEYS, errors, strict=True)) if errors else {}})
            return
        status = reason.upper().replace("_", " ")
        if self.compact:
            print("\n" if self.displayed else "", end="")
        else:
            print(self.rule, flush=True)
        print(f" RESULT: {status}" + (f"  |  {message}" if message else ""), flush=True)
        if self.compact:
            return
        if self.last_values is not None:
            print(f" Last speed: {values['estimated_rpm']:+.1f} RPM / "
                  f"setpoint {values['setpoint_rpm']:+.1f} RPM "
                  f"(error {values['speed_error_rpm']:+.1f} RPM)", flush=True)
        print(f" Samples collected: {self.samples}  |  CSV: {self.log_path}", flush=True)

    def close(self):
        if self.csv_file is not None and not self.csv_file.closed:
            self.csv_file.close()


VELOCITY_COLUMNS = (
    ("elapsed_s", "Time s", ".2f"),
    ("setpoint_rpm", "Set RPM", "+.1f"),
    ("estimated_rpm", "Est RPM", "+.1f"),
    ("encoder_count_rpm", "Enc RPM", "+.1f"),
    ("speed_error_rpm", "Err RPM", "+.1f"),
    ("rotation_deg", "Angle deg", "+.1f"),
    ("iq_setpoint_a", "Iq Set A", "+.2f"),
    ("iq_measured_a", "Iq Act A", "+.2f"),
    ("vbus_v", "Bus V", ".2f"),
)

ERROR_KEYS = ("axis_error", "motor_error", "encoder_error", "controller_error", "sensorless_error")
