#!/usr/bin/env python3
"""Desktop test console for the workspace's ODrive v3.6 bench."""
from contextlib import suppress
from datetime import datetime
import json
import math
import os
from pathlib import Path
from queue import Empty, SimpleQueue
import subprocess
import sys
from threading import Thread
import time
import tkinter as tk
from tkinter import ttk

from PIL import Image, ImageTk


from motor.paths import LOGS, WORKSPACE
from motor.live import POSITION_BOUNDARY_RPM, effective_mode
from .pso_panel import PsoPanel
from .telemetry import FIELDS, TrialLog, TelemetryCharts, number
from .teleop_keys import TeleopKeys
from .theme import LIGHT, DARK, configure as configure_theme
from .tuning_panel import TuningEditor

ICON = Path(__file__).with_name("odrive-bench.png")
STOP_ALERT_SECONDS = 5
HEARTBEAT_INTERVAL = 0.25
MODES = {"Control": "auto", "Velocity": "velocity", "Encoder": "encoder", "Torque": "torque"}
SPEED_ACTIONS = ("auto", "velocity", "torque")
PSO_MOTION = ("pso_reference", "pso_validate")
PSO_OFFLINE = ("pso_info", "pso_analyze")
MOTOR_ACTIONS = ("calibrate", "teleop", *SPEED_ACTIONS, *PSO_MOTION)
STOPPABLE_ACTIONS = (*MOTOR_ACTIONS, "encoder", "pso_analyze")
TUNING_ACTIONS = ("tuning", "save", "advanced_save")


def fault_text(faults):
    return "  ".join(f"{name.removesuffix('_error')} {int(mask):#x}"
                     for name, mask in faults.items() if mask not in (None, ""))


class BenchApp:
    def __init__(self, root):
        self.root = root
        root.title("ODrive Bench")
        with Image.open(ICON) as icon:
            root.iconphoto(True, ImageTk.PhotoImage(icon, master=root))
            mark = icon.resize((36, 36), Image.Resampling.LANCZOS)
            self.brand_mark = ImageTk.PhotoImage(mark, master=root)
        root.geometry("1360x820")
        root.minsize(1080, 680)
        self.palette = DARK
        self.dark = tk.BooleanVar(master=root, value=True)
        self.messages = SimpleQueue()
        self.process = self.action = None
        self.status = None
        self.reviewing_stop = self.idle_unconfirmed = self.closing = False
        self.awaiting_confirmation = self.approved = self.stop_requested = False
        self.stop_alert_at = None
        self.trial = None
        self.run_logged = self.recording = False
        self.reported_stop = False
        self.frozen = False
        self.browsing = False
        self.pane_sizing = None
        self.editor_reload = None
        self.plotted_at = 0
        self.draw_due = 0
        self.live_run = False
        self.live_mode = ""
        self.is_running = False
        self.used_draft = None
        self.rpm_sequence = 0
        self.rpm_pending = None
        self.last_request = self.accepted_rpm = math.nan
        self.next_heartbeat = 0
        self.drafts = {}
        self.editor = None
        self.pso = self.pso_prompt = None
        self.teleop_state = ("holding", "")
        self.teleop_default = 50.0

        self.axis = tk.StringVar(value="M0")
        self.mode = tk.StringVar(value="Control")
        self.rpm = tk.StringVar()
        self.allow_live = tk.BooleanVar()
        self.ceiling = tk.StringVar()
        self.teleop_ceiling = tk.StringVar(value="50")
        self.teleop_text = tk.StringVar()
        self.next_rpm = tk.StringVar()
        self.live_text = tk.StringVar()
        self.drive = tk.StringVar()
        self.drive_state = tk.StringVar()
        self.encoder_text = tk.StringVar()
        self.activity = tk.StringVar(value="Check the drive to get started")
        self.detail = tk.StringVar()
        self.masks = tk.StringVar(value="Fault masks unavailable")
        self.telemetry = tk.StringVar(value="No motor trial yet")
        self.params = tk.StringVar(value="Settings appear when a test starts.")
        self.result = tk.StringVar(value="No operation recorded.")
        self.log_name = tk.StringVar(value="")
        self.prompt = tk.StringVar()

        self.build()
        self.teleop_keys = TeleopKeys(root, lambda: self.action == "teleop",
                                     lambda: self.is_running and not self.stop_requested and not self.idle_unconfirmed,
                                     self.send_teleop, self.stop_on_focus_loss)
        self.apply_theme()
        self.axis.trace_add("write", self.sync)
        self.mode.trace_add("write", self.sync)
        self.rpm.trace_add("write", self.sync)
        self.allow_live.trace_add("write", self.sync)
        self.ceiling.trace_add("write", self.sync)
        self.teleop_ceiling.trace_add("write", self.sync)
        self.sync()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<Escape>", lambda _event: self.stop())
        root.bind("<F11>", lambda _event: root.attributes("-fullscreen", not root.attributes("-fullscreen")))
        self.timer = root.after(60, self.poll)

    def build(self):
        self.style = ttk.Style(self.root)
        self.style.theme_use("clam")
        configure_theme(self.style, self.palette)

        header = ttk.Frame(self.root, padding=(16, 12))
        header.pack(fill="x")
        ttk.Label(header, image=self.brand_mark).grid(row=0, column=0, rowspan=2, padx=(0, 10))
        ttk.Label(header, text="ODrive Bench", style="Title.TLabel").grid(row=0, column=1, sticky="w")
        drive_line = ttk.Frame(header)
        drive_line.grid(row=1, column=1, sticky="w", pady=(5, 0))
        self.drive_badge = ttk.Label(drive_line, textvariable=self.drive_state, style="DriveCaution.TLabel")
        self.drive_badge.pack(side="left")
        ttk.Label(drive_line, textvariable=self.drive, style="DriveMeta.TLabel", wraplength=295).pack(
            side="left", padx=(8, 0))
        header.columnconfigure(1, weight=1)
        status = ttk.Label(header, textvariable=self.activity, anchor="e", wraplength=220, justify="right")
        status.grid(row=0, column=2, rowspan=2, sticky="e", padx=(8, 10))
        self.activity_label = status
        ttk.Checkbutton(header, text="Dark mode", variable=self.dark, command=self.apply_theme).grid(
            row=0, column=3, rowspan=2, padx=(0, 8))
        self.run_button = ttk.Button(header, text="Run test", style="Run.TButton", command=self.prepare_test)
        self.run_button.grid(row=0, column=4, rowspan=2, padx=(0, 7))
        self.stop_button = ttk.Button(header, text="Stop test  ·  Esc", style="Stop.TButton", command=self.stop)
        self.stop_button.grid(row=0, column=5, rowspan=2)

        body = self.body = ttk.Frame(self.root, padding=(16, 0, 16, 12))
        body.pack(fill="both", expand=True)
        self.panes = tk.PanedWindow(body, orient="horizontal", sashwidth=8, sashrelief="ridge",
                                  opaqueresize=False, borderwidth=0)
        self.panes.pack(fill="both", expand=True)
        operations = self.operations = ttk.Frame(self.panes, padding=12, style="Panel.TFrame")
        operations.columnconfigure(0, weight=1)
        ttk.Label(operations, text="Drive operations", style="Section.TLabel").grid(sticky="w", pady=(0, 11))
        ttk.Label(operations, text="Selected axis", style="Panel.TLabel").grid(sticky="w")
        self.axis_picker = ttk.Combobox(operations, textvariable=self.axis, values=("M0", "M1"),
                                        state="readonly", width=14)
        self.axis_picker.grid(sticky="ew", pady=(3, 8))
        self.check_button = ttk.Button(operations, text="Check drive", command=self.refresh)
        self.check_button.grid(sticky="ew", pady=(0, 5))
        self.clear_button = ttk.Button(operations, text="Clear faults", command=lambda: self.start("clear"))
        self.clear_button.grid(sticky="ew", pady=(0, 5))
        self.cal_button = ttk.Button(operations, text="Calibrate A/B", command=lambda: self.start("calibrate"))
        self.cal_button.grid(sticky="ew")
        ttk.Label(operations, text="Speed test", style="Section.TLabel").grid(sticky="w", pady=(21, 10))
        ttk.Label(operations, text="Control mode", style="Panel.TLabel").grid(sticky="w")
        self.mode_picker = ttk.Combobox(operations, textvariable=self.mode, values=tuple(MODES),
                                        state="readonly", width=14)
        self.mode_picker.grid(sticky="ew", pady=(3, 8))
        ttk.Label(operations, text="Signed target [RPM]", style="Panel.TLabel").grid(sticky="w")
        self.rpm_input = ttk.Entry(operations, textvariable=self.rpm, width=14)
        self.rpm_input.grid(sticky="ew", pady=(3, 7))
        self.live_toggle = ttk.Checkbutton(operations, text="Allow live RPM", variable=self.allow_live)
        self.live_toggle.grid(sticky="w", pady=(0, 4))
        self.live_controls = ttk.Frame(operations, style="Panel.TFrame")
        self.live_controls.grid(sticky="ew", pady=(0, 6))
        self.live_label = ttk.Label(self.live_controls, text="Live RPM ceiling", style="Panel.TLabel")
        self.live_label.pack(anchor="w")
        self.live_input = ttk.Entry(self.live_controls, textvariable=self.ceiling, width=14)
        self.live_input.pack(fill="x", pady=(3, 3))
        self.apply_rpm_button = ttk.Button(self.live_controls, text="Apply RPM", command=self.apply_rpm)
        self.tune_button = ttk.Button(operations, text="Tune parameters", command=self.open_tuning)
        self.tune_button.grid(sticky="ew", pady=(0, 5))
        self.pso_button = ttk.Button(operations, text="PSO tuning", command=self.open_pso)
        self.pso_button.grid(sticky="ew", pady=(0, 5))
        note = ttk.Label(operations, textvariable=self.detail, wraplength=195, style="Muted.TLabel")
        note.grid(sticky="w", pady=(10, 0))
        operations.bind("<Configure>", lambda e: note.configure(wraplength=max(120, e.width - 24)))

        plot = self.plot = ttk.Frame(self.panes, padding=(12, 10), style="Panel.TFrame")
        toolbar = ttk.Frame(plot, style="Panel.TFrame")
        toolbar.pack(fill="x")
        plot_title = ttk.Label(toolbar, text="Live telemetry", style="Section.TLabel")
        self.freeze_button = ttk.Button(toolbar, text="Freeze view", command=self.freeze, state="disabled")
        ttk.Label(plot, textvariable=self.telemetry, style="Muted.TLabel", wraplength=470).pack(anchor="w")
        self.target_label = ttk.Label(plot, textvariable=self.live_text, style="Panel.TLabel", wraplength=470)
        self.encoder_label = ttk.Label(plot, textvariable=self.encoder_text, style="Section.TLabel", wraplength=420)
        ttk.Label(plot, text="Drag up/down to pan one graph · Ctrl+scroll to zoom · Double-click to reset",
                  style="Muted.TLabel", wraplength=470).pack(anchor="w")
        self.charts = TelemetryCharts(plot, self.detach_timeline, self.palette)
        plot_title.pack(side="left")
        self.freeze_button.pack(side="right")

        diagnostics = self.diagnostics = ttk.Frame(self.panes, padding=12, style="Panel.TFrame")
        self.panes.add(operations, width=225, minsize=190, stretch="never")
        self.panes.add(plot, minsize=330, stretch="always")
        self.panes.add(diagnostics, width=250, minsize=200, stretch="never")
        self.panes.bind("<Map>", self.size_panes, add="+")
        diagnostics.columnconfigure(0, weight=1)
        diagnostics.rowconfigure(2, weight=1)
        ttk.Label(diagnostics, text="Run events", style="Section.TLabel").grid(sticky="w")
        self.result_label = ttk.Label(diagnostics, textvariable=self.result, wraplength=245, style="Panel.TLabel")
        self.result_label.grid(sticky="w", pady=(5, 7))
        events_frame = ttk.Frame(diagnostics)
        events_frame.grid(sticky="nsew")
        self.events = tk.Text(events_frame, width=29, height=5, wrap="word", padx=6, pady=6,
                              relief="flat",
                              font=("DejaVu Sans", 9), state="disabled")
        scrollbar = ttk.Scrollbar(events_frame, command=self.events.yview)
        self.events.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.events.pack(fill="both", expand=True)
        self.teleop_panel = teleop = ttk.Frame(diagnostics, style="Panel.TFrame")
        teleop.grid(sticky="ew", pady=(10, 0))
        ttk.Label(teleop, text="Teleop", style="Section.TLabel").pack(anchor="w")
        keys = ttk.Frame(teleop, style="Panel.TFrame")
        keys.pack(fill="x", pady=(4, 3))
        self.teleop_arrows = {}
        for direction, label in (("forward", "↑ Forward"), ("reverse", "↓ Reverse")):
            indicator = ttk.Label(keys, text=label, style="Muted.TLabel", anchor="center")
            indicator.pack(side="left", fill="x", expand=True)
            self.teleop_arrows[direction] = indicator
        options = ttk.Frame(teleop, style="Panel.TFrame")
        options.pack(fill="x")
        ttk.Label(options, text="RPM ceiling", style="Panel.TLabel").pack(side="left")
        self.teleop_input = ttk.Entry(options, textvariable=self.teleop_ceiling, width=7)
        self.teleop_input.pack(side="right")
        self.teleop_button = ttk.Button(teleop, text="Activate Teleop", command=self.start_teleop)
        self.teleop_button.pack(fill="x", pady=(4, 3))
        teleop_status = ttk.Label(teleop, textvariable=self.teleop_text, style="Muted.TLabel", wraplength=245)
        teleop_status.pack(fill="x")
        ttk.Label(diagnostics, text="Fault masks", style="Section.TLabel").grid(sticky="w", pady=(11, 4))
        masks = ttk.Label(diagnostics, textvariable=self.masks, style="Muted.TLabel", wraplength=245)
        masks.grid(sticky="w")
        ttk.Label(diagnostics, text="Run settings", style="Section.TLabel").grid(sticky="w", pady=(11, 4))
        params = ttk.Label(diagnostics, textvariable=self.params, style="Muted.TLabel", wraplength=245)
        params.grid(sticky="w")
        log_name = ttk.Label(diagnostics, textvariable=self.log_name, style="Muted.TLabel", wraplength=245)
        log_name.grid(sticky="w", pady=(7, 4))
        ttk.Button(diagnostics, text="Open log folder", command=self.open_logs).grid(sticky="ew")

        def wrap_diagnostics(event):
            for label in (self.result_label, teleop_status, masks, params, log_name):
                label.configure(wraplength=max(120, event.width - 24))

        diagnostics.bind("<Configure>", wrap_diagnostics)

        self.confirmation = ttk.Frame(self.root, padding=(16, 6, 16, 10))
        self.confirmation.columnconfigure(0, weight=1)
        ttk.Label(self.confirmation, textvariable=self.prompt, wraplength=820).grid(sticky="w")
        self.confirm_button = ttk.Button(self.confirmation, text="Confirm motion",
                                         style="Run.TButton", command=self.confirm)
        self.confirm_button.grid(row=0, column=1, padx=(10, 0), sticky="e")
        ttk.Button(self.confirmation, text="Cancel", command=self.stop).grid(row=0, column=2, padx=(7, 0))
        self.guard_value = tk.StringVar()
        self.guard_input = ttk.Entry(self.confirmation, textvariable=self.guard_value, width=12)

    def size_panes(self, _event):
        def layout():
            self.pane_sizing = None
            if len(self.panes.panes()) == 3:
                width = self.panes.winfo_width()
                self.panes.paneconfigure(self.operations, width=min(260, int(width * 0.2)))
                self.panes.paneconfigure(self.diagnostics, width=int(width * 0.24))

        if self.pane_sizing is None:
            self.pane_sizing = self.root.after(150, layout)

    def apply_theme(self):
        p = self.palette = DARK if self.dark.get() else LIGHT
        configure_theme(self.style, p)
        self.root.configure(background=p.bg)
        self.panes.configure(background=p.border)
        self.events.configure(background=p.field, foreground=p.ink, selectbackground=p.selection,
                              selectforeground=p.ink, insertbackground=p.ink)
        self.events.tag_configure("error", foreground=p.danger)
        self.events.tag_configure("warning", foreground=p.warning)
        for option, color in (("background", p.field), ("foreground", p.ink),
                              ("selectBackground", p.selection), ("selectForeground", p.ink)):
            self.root.option_add(f"*TCombobox*Listbox.{option}", color)
        self.charts.style(p)
        if self.editor:
            self.editor.style(p)
        if self.pso:
            self.pso.style(p)

    def prepare_test(self):
        if not self.run_button.instate(("disabled",)):
            self.start(MODES[self.mode.get()])

    def append(self, line, severity=None):
        prefix = line.lstrip().partition(":")[0].upper()
        printed = {"DANGER": "error", "ERROR": "error", "WARNING": "warning"}.get(prefix)
        severity = severity or printed
        if severity and not printed:
            line = f"{severity.capitalize()} · {line}"
        follow = self.events.yview()[1] >= 0.99
        self.events.configure(state="normal")
        self.events.insert("end", f"{datetime.now():%H:%M:%S}  {line}\n", (severity,) if severity else ())
        if int(self.events.index("end-1c").split(".")[0]) > 180:
            self.events.delete("1.0", "2.0")
        if follow:
            self.events.see("end")
        self.events.configure(state="disabled")

    @property
    def selected(self):
        return int(self.axis.get()[1])

    def sync(self, *_args):
        busy = self.process is not None
        status = self.status
        idle = bool(status and status["both_idle"] and not self.idle_unconfirmed)
        axis = status["axes"][self.selected] if status else None
        mode = self.mode.get()
        action = MODES[mode]
        monitor = action == "encoder"
        motion_issue = axis["motion_issue"] if axis else ""
        if action in ("auto", "torque"):
            if self.selected != 0:
                motion_issue = "Control and Torque modes require M0."
            elif status and any(status["axes"][1]["faults"].values()):
                motion_issue = motion_issue or "Clear the M1 fault before running this M0 test."
        cal_possible = idle and not axis["calibration_issue"]
        rpm = number(self.rpm.get())
        limits = axis["rpm_range"] if axis else None
        valid_rpm = bool(limits and limits[0] <= abs(rpm) <= limits[1])
        live_issue = ""
        if valid_rpm and self.allow_live.get():
            ceiling = self.live_ceiling(rpm)
            valid_rpm = abs(rpm) <= ceiling <= limits[1]
            if not valid_rpm:
                live_issue = f"Live ceiling must be between {abs(rpm):g} and {limits[1]:g} RPM."
            elif effective_mode(action, rpm) == "position" and ceiling >= POSITION_BOUNDARY_RPM:
                valid_rpm = False
                live_issue = f"Position live ceiling must be below {POSITION_BOUNDARY_RPM:g} RPM."
        self.check_button.configure(state="disabled" if busy else "normal")
        self.clear_button.configure(state="normal" if idle and not busy else "disabled")
        self.cal_button.configure(state="normal" if cal_possible and not busy else "disabled")
        can_run = idle and not busy and (monitor or (not motion_issue and valid_rpm))
        self.run_button.configure(state="normal" if can_run else "disabled",
                                  text="Monitor encoder" if monitor else "Run test")
        self.axis_picker.configure(state="disabled" if busy else "readonly")
        self.mode_picker.configure(state="disabled" if busy else "readonly")
        self.rpm_input.configure(state="disabled" if busy or monitor else "normal")
        self.tune_button.configure(state="normal" if not busy and not monitor and
                                   (action != "auto" or math.isfinite(rpm)) else "disabled",
                                   text="Tune parameters · draft" if (self.selected, effective_mode(action, rpm))
                                   in self.drafts else "Tune parameters")
        self.pso_button.configure(state="normal" if not busy and self.selected == 0 else "disabled")
        if self.pso:
            if self.selected != 0 and not busy:
                self.close_pso()
            else:
                ready = idle and not axis["motion_issue"] and not any(
                    any(item["faults"].values()) for item in status["axes"])
                can_save = (self.pso_prompt is not None and self.pso_prompt["kind"] == "save"
                            and not self.stop_requested and not self.idle_unconfirmed)
                self.pso.controls(busy, ready, can_save)
        self.live_toggle.configure(state="disabled" if busy or monitor else "normal")
        if self.allow_live.get() and not monitor:
            self.live_controls.grid()
        else:
            self.live_controls.grid_remove()
        live_input = busy and self.live_run and self.is_running and not self.stop_requested
        self.live_label.configure(text="New target [RPM]" if live_input else "Live ceiling [RPM]")
        self.live_input.configure(textvariable=self.next_rpm if live_input else self.ceiling,
                                  state="normal" if live_input or not busy else "disabled")
        if live_input:
            self.apply_rpm_button.pack(fill="x")
            self.apply_rpm_button.configure(state="normal" if self.rpm_pending is None else "disabled")
        else:
            self.apply_rpm_button.pack_forget()
        if busy and self.action in MOTOR_ACTIONS:
            self.tune_button.grid_remove()
        else:
            self.tune_button.grid()
        can_stop = busy and self.action in STOPPABLE_ACTIONS and not self.stop_requested
        self.stop_button.configure(state="normal" if can_stop else "disabled",
                                   text="Stop monitor  ·  Esc" if self.action == "encoder" else "Stop test  ·  Esc")
        self.sync_teleop()
        self.freeze_button.configure(state="normal" if self.trial and self.trial.points else "disabled")
        if axis:
            readiness = "encoder ready" if axis["encoder_ready"] else "encoder unready"
            self.drive.set(f"{self.axis.get()}  ·  {status['serial']}  ·  {readiness}")
            self.masks.set("\n".join(f"M{index}  {fault_text(item['faults'])}"
                                    for index, item in enumerate(status["axes"])))
        else:
            self.drive.set(self.axis.get())
            self.masks.set("Current fault masks unavailable")
        self.show_drive_state(axis, "" if monitor else motion_issue)
        rpm_help = f"Allowed magnitude {limits[0]:g}–{limits[1]:g} RPM" if limits else ""
        if self.allow_live.get() and not self.ceiling.get().strip():
            rpm_help = "Blank ceiling uses starting RPM. " + rpm_help
        issue = motion_issue or (axis["calibration_issue"] if axis else "")
        if busy:
            detail = "Review the confirmation to start motion." if self.awaiting_confirmation else ""
        elif self.idle_unconfirmed:
            detail = "Check drive to verify IDLE before testing."
        elif not status:
            detail = "Check drive before running a test."
        elif not idle:
            detail = "Both axes must be IDLE."
        elif monitor:
            detail = "Read encoder counts while IDLE. Turn the shaft by hand."
        elif motion_issue:
            detail = motion_issue
        elif not math.isfinite(rpm) or rpm == 0:
            detail = "Enter a nonzero signed RPM to run a test."
        elif not valid_rpm:
            detail = live_issue or rpm_help or "The target is outside the allowed range."
        else:
            detail = issue or (f"{effective_mode(action, rpm).capitalize()} · {rpm_help}" if action == "auto" else rpm_help)
        self.detail.set(detail)
        if self.editor:
            mismatch = not self.editor_matches_selection()
            self.editor.busy(busy, mismatch)
            if mismatch and not busy and self.editor.snapshot:
                if monitor:
                    self.editor.status.set("Select a motor test mode to edit tuning.")
                elif self.editor.dirty():
                    self.editor.status.set("Axis or mode changed. Reload parameters for the selected profile.")
                elif self.editor_reload is None:
                    self.editor_reload = self.root.after(240, self.refresh_editor_selection)

    def show_drive_state(self, axis, motion_issue):
        faults = [f"M{i}" for i, item in enumerate(self.status["axes"]) if any(item["faults"].values())] if axis else []
        if self.idle_unconfirmed:
            text, tone = "IDLE unconfirmed", "Fault"
        elif self.stop_requested or self.reviewing_stop:
            text, tone = "Stopping" if self.stop_requested else "Verifying IDLE", "Caution"
        elif self.awaiting_confirmation:
            guard = self.pso_prompt and self.pso_prompt["kind"] == "guard"
            text, tone = ("Confirm guarded speed" if guard else "Confirm motion"), "Caution"
        elif self.pso_prompt and self.pso_prompt["kind"] == "save":
            text, tone = "Validation passed", "Idle"
        elif self.action == "encoder":
            text, tone = "Encoder monitor" if self.is_running else "Checking encoder", "Active"
        elif self.action == "teleop":
            text, tone = ("Teleop active" if self.is_running else "Teleop preflight"), "Active"
        elif self.action in MOTOR_ACTIONS:
            text = "Test running" if self.is_running else "Calibrating" if self.approved and self.action == "calibrate" else "Preflight"
            tone = "Active"
        elif self.action == "status":
            text, tone = "Checking drive", "Caution"
        elif axis is None:
            text, tone = ("Unavailable" if self.activity.get() == "Drive unavailable" else "Unchecked"), "Caution"
        elif faults:
            text, tone = f"{' / '.join(faults)} fault", "Fault"
        elif not self.status["both_idle"]:
            text, tone = "Drive busy", "Caution"
        elif motion_issue:
            text, tone = "IDLE · blocked", "Caution"
        else:
            text, tone = "IDLE · ready", "Idle"
        self.drive_state.set(text)
        self.drive_badge.configure(style=f"Drive{tone}.TLabel")

    def launch(self, action, rpm=None, options=None):
        if self.process is not None:
            return False
        argv = [sys.executable, "-u", "-m", "ui.worker", action, str(self.selected)]
        if rpm is not None:
            argv.append(f"--rpm={rpm}")
        if options:
            argv.append(f"--options={json.dumps(options, allow_nan=False)}")
        try:
            process = subprocess.Popen(argv, cwd=WORKSPACE, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                                       errors="replace")
        except OSError as exc:
            self.append(str(exc), "error")
            return False
        os.set_blocking(process.stdin.fileno(), False)
        self.process, self.action = process, action
        Thread(target=self.read_worker, args=(process,), daemon=True).start()
        self.sync()
        return True

    def read_worker(self, process):
        with process.stdout:
            for line in process.stdout:
                try:
                    event, data = json.loads(line)
                except (ValueError, TypeError):
                    event, data = "notice", line.strip()
                self.messages.put((event, data))
        self.messages.put(("exit", process.wait()))

    def refresh(self):
        if self.launch("status") and not self.reviewing_stop:
            if not self.idle_unconfirmed:
                self.activity.set("Checking drive…")
            self.status = None
            self.sync()

    def start(self, action):
        rpm = number(self.rpm.get()) if action in SPEED_ACTIONS else None
        options = {}
        key = None
        if rpm is not None:
            if not math.isfinite(rpm):
                return
            key = (self.selected, effective_mode(action, rpm))
            if key in self.drafts:
                options["draft"] = self.drafts[key]
            if self.allow_live.get():
                ceiling = self.live_ceiling(rpm)
                if not math.isfinite(ceiling):
                    return
                options["ceiling"] = ceiling
        if self.launch(action, rpm, options):
            self.run_logged = False
            self.reported_stop = False
            self.live_run = "ceiling" in options
            self.live_mode = effective_mode(action, rpm) if rpm is not None else ""
            self.used_draft = key if "draft" in options else None
            self.rpm_pending = None
            self.last_request = rpm if self.live_run else math.nan
            self.accepted_rpm = math.nan
            if self.live_run:
                self.next_rpm.set(f"{rpm:+g}")
                self.target_label.pack(anchor="w", before=self.charts.frame)
            else:
                self.target_label.pack_forget()
            self.update_live_text()
            if action == "encoder":
                self.activity.set("Checking encoder…")
                self.encoder_text.set("Waiting for encoder counts…")
                self.encoder_label.pack(anchor="w", before=self.charts.frame)
            else:
                self.activity.set("Checking test conditions…" if action in MOTOR_ACTIONS else "Clearing faults…")
            self.activity_label.configure(style="TLabel")
            self.result.set("Waiting for operation result.")
            self.result_label.configure(style="Panel.TLabel")
            label = {"calibrate": "A/B calibration", "clear": "Clear faults", "encoder": "Encoder monitor"}
            self.append(f"{self.axis.get()} · {self.mode.get()} · {rpm:+g} RPM" if rpm is not None else
                        f"{self.axis.get()} · {label[action]}")

    def live_ceiling(self, rpm):
        return number(self.ceiling.get()) if self.ceiling.get().strip() else abs(rpm)

    def teleop_issue(self):
        mode = self.mode.get()
        if mode == "Encoder":
            return "Select Velocity, Control, or Torque."
        if self.selected != 0 and mode != "Velocity":
            return "Control and Torque require M0."
        if self.idle_unconfirmed:
            return "IDLE unconfirmed. Check drive."
        if not self.status:
            return "Check drive to enable Teleop."
        if not self.status["both_idle"]:
            return "Both axes must be IDLE."
        axis = self.status["axes"][self.selected]
        if axis["motion_issue"]:
            return axis["motion_issue"]
        if any(axis["faults"].values()) or (mode != "Velocity" and any(self.status["axes"][1]["faults"].values())):
            return "Clear drive faults first."
        if not axis["rpm_range"]:
            return "RPM limits unavailable for this axis."
        minimum, maximum = axis["rpm_range"]
        ceiling = number(self.teleop_ceiling.get())
        if not minimum <= ceiling <= maximum:
            return f"Enter a ceiling from {minimum:g} to {maximum:g} RPM."
        return ""

    def sync_teleop(self):
        if self.process is None and self.status:
            limits = self.status["axes"][self.selected]["rpm_range"]
            if limits:
                default = min(limits[1], max(limits[0], 50.0))
                if default != self.teleop_default:
                    previous, self.teleop_default = self.teleop_default, default
                    if number(self.teleop_ceiling.get()) == previous:
                        self.teleop_ceiling.set(f"{default:g}")
        busy = self.process is not None
        issue = self.teleop_issue() if not busy else ""
        self.teleop_input.configure(state="disabled" if busy else "normal")
        self.teleop_button.configure(state="disabled" if busy or issue else "normal")
        state, direction = self.teleop_state
        active = ""
        if self.action == "teleop":
            if self.idle_unconfirmed:
                text = "IDLE unconfirmed. Isolate motor power."
            elif self.stop_requested or state == "stopping":
                text = "Stopping…"
            elif self.awaiting_confirmation:
                text = "Confirm motion to activate."
            elif not self.is_running:
                text = "Preparing Teleop…"
            elif state == "driving":
                active = direction
                text = f"{direction.capitalize()} target · Space twice brakes"
            elif state == "reversing":
                text = f"Reversing to {direction} · Waiting for zero"
            elif state == "braking":
                text = "Braking to zero…"
            else:
                text = "Holding zero · ↑/↓ to drive"
        elif self.reviewing_stop and state == "stopping":
            text = "Verifying IDLE…"
        elif not busy and not issue and self.mode.get() == "Control":
            mode = effective_mode("auto", number(self.teleop_ceiling.get())).capitalize()
            text = f"{mode} · ↑/↓ to drive · Space twice brakes"
        else:
            text = "Another operation is active." if busy else issue or "↑/↓ changes speed · Space twice brakes"
        self.teleop_text.set(text)
        for key, indicator in self.teleop_arrows.items():
            indicator.configure(style="DriveActive.TLabel" if key == active else "Muted.TLabel")

    def start_teleop(self):
        if self.process is not None or self.teleop_issue():
            return
        ceiling = number(self.teleop_ceiling.get())
        self.teleop_state = ("holding", "")
        if not self.launch("teleop", options={"mode": self.mode.get().lower(), "ceiling": ceiling}):
            return
        self.teleop_keys.prepare()
        self.run_logged = self.reported_stop = self.live_run = False
        self.used_draft = None
        self.rpm_pending = None
        self.target_label.pack_forget()
        self.activity.set("Preparing Teleop…")
        self.activity_label.configure(style="TLabel")
        self.result.set("Waiting for Teleop confirmation.")
        self.result_label.configure(style="Panel.TLabel")
        self.append(f"Teleop {self.axis.get()} · {self.mode.get()} · ±{ceiling:g} RPM ceiling")

    def send_teleop(self, event):
        if not self.send(json.dumps({"teleop": event})):
            self.stop()

    def stop_on_focus_loss(self):
        if self.action == "teleop" and not self.stop_requested:
            self.append("Teleop ended after the window lost focus.")
            self.stop()

    def open_tuning(self):
        if self.editor:
            self.close_editor()
            return
        if self.process is not None or self.mode.get() == "Encoder":
            return
        self.close_pso()
        self.editor = TuningEditor(self.panes, self.stage_tuning, self.save_tuning, self.reload_tuning,
                                   self.close_editor)
        self.panes.add(self.editor.window, before=self.plot, width=255, minsize=218, stretch="never")
        self.editor.style(self.palette)
        if not self.launch("tuning", options={"mode": self.selected_tuning_mode()}):
            self.close_editor()

    def open_pso(self):
        if self.process is not None or self.selected != 0:
            return
        if self.pso:
            self.close_pso()
            return
        self.close_editor()
        self.pso = PsoPanel(self.panes, self.start_pso, self.close_pso)
        self.panes.add(self.pso.window, before=self.plot, width=304, minsize=275, stretch="never")
        self.pso.style(self.palette)
        self.start_pso("pso_info")

    def close_pso(self):
        if self.pso and self.process is None:
            self.panes.forget(self.pso.window)
            self.pso.window.destroy()
            self.pso = None

    def start_pso(self, action):
        panel = self.pso
        if panel is None or self.selected != 0:
            return
        if action == "pso_save":
            if not panel.buttons[action].instate(("disabled",)):
                if self.answer_pso("save"):
                    panel.status.set("Saving validated gains…")
                self.sync()
            return
        if self.process is not None or (action in panel.buttons and panel.buttons[action].instate(("disabled",))):
            return
        options = {"mode": panel.mode.get().lower()}
        rpm = None
        if action in PSO_MOTION:
            options["revision"] = panel.snapshot["revision"]
        if action == "pso_reference":
            rpm = number(panel.rpm.get())
            gains = [number(value.get()) for value in panel.gains]
            if not all(map(math.isfinite, (rpm, *gains))):
                panel.status.set("Enter a finite signed RPM and both reference gains.")
                return
            options["gains"] = gains
        elif action == "pso_validate":
            options["candidate"] = panel.candidate["gains"]
            options["revision"] = panel.candidate["revision"]
        if action != "pso_validate":
            panel.candidate = None
        if not self.launch(action, rpm, options):
            panel.status.set("Could not start the PSO worker.")
            return
        panel.add("", clear=True)
        panel.status.set({"pso_info": "Loading gains…", "pso_analyze": "Analyzing saved CSVs…",
                          "pso_reference": "Preparing reference…",
                          "pso_validate": "Rechecking offline evidence…"}[action])
        if action in PSO_MOTION:
            self.run_logged = self.reported_stop = self.live_run = False
            self.used_draft = None
            self.target_label.pack_forget()
            self.activity.set("Preparing PSO trial…")
            self.activity_label.configure(style="TLabel")

    def answer_pso(self, value):
        if self.pso_prompt is None or self.stop_requested:
            return False
        prompt, self.pso_prompt = self.pso_prompt, None
        if not self.send(json.dumps({"pso_id": prompt["id"], "value": value})):
            self.stop()
            return False
        return True

    def close_editor(self):
        if self.editor is not None and self.action not in TUNING_ACTIONS:
            self.panes.forget(self.editor.window)
            self.editor.window.destroy()
            self.editor = None

    def selected_tuning_mode(self):
        return effective_mode(MODES[self.mode.get()], number(self.rpm.get()))

    def editor_matches_selection(self):
        snapshot = self.editor.snapshot if self.editor else None
        return bool(snapshot and snapshot["axis"] == self.selected and snapshot["mode"] == self.selected_tuning_mode())

    def refresh_editor_selection(self):
        self.editor_reload = None
        if (self.editor and self.editor.snapshot and self.process is None and self.mode.get() != "Encoder"
                and not self.editor_matches_selection() and not self.editor.dirty()):
            self.launch("tuning", options={"mode": self.selected_tuning_mode()})

    def load_editor(self, snapshot, *, saved=False):
        if not self.editor:
            return
        key = (snapshot["axis"], snapshot["mode"])
        if saved:
            self.drafts.pop(key, None)
        self.editor.load(snapshot, self.drafts.get(key))
        if saved:
            self.editor.status.set("Saved to motor_config.json.")

    def stage_tuning(self):
        if self.process is not None or not self.editor_matches_selection() or self.editor.advanced.get():
            return
        draft = self.editor.draft()
        if draft is not None:
            key = (self.selected, self.selected_tuning_mode())
            if draft["edits"]:
                self.drafts[key] = draft
                self.append(f"{self.axis.get()} {self.tuning_mode}: unsaved tuning staged for the next run.")
            else:
                self.drafts.pop(key, None)
            self.editor.status.set("Draft staged for one confirmed run." if draft["edits"] else "No draft changes.")
            self.sync()

    def save_tuning(self):
        if self.process is not None or not self.editor_matches_selection():
            return
        advanced = self.editor.advanced.get()
        draft = self.editor.reviewed or self.editor.draft()
        if draft is not None:
            if not draft["edits"]:
                self.editor.status.set("No parameter changes to save.")
            elif advanced and self.editor.reviewed is None:
                self.editor.review(draft)
            elif self.launch("advanced_save" if advanced else "save",
                             options={"mode": self.selected_tuning_mode(), "draft": draft}):
                if advanced:
                    self.status = None
                    self.sync()
                self.editor.status.set("Saving parameters…")

    def reload_tuning(self):
        if self.editor and self.mode.get() != "Encoder" and self.launch("tuning", options={"mode": self.selected_tuning_mode()}):
            self.drafts.pop((self.selected, self.selected_tuning_mode()), None)
            self.editor.status.set("Loading parameters…")

    def send(self, message):
        try:
            os.write(self.process.stdin.fileno(), (message + "\n").encode("utf-8"))
        except OSError:
            return False
        return True

    def apply_rpm(self):
        if not (self.live_run and self.is_running and not self.stop_requested and self.rpm_pending is None):
            return
        rpm = number(self.next_rpm.get())
        if not math.isfinite(rpm):
            self.append("Enter a finite signed RPM.", "warning")
            return
        self.rpm_sequence += 1
        if self.send(json.dumps({"sequence": self.rpm_sequence, "rpm": rpm})):
            self.last_request = rpm
            self.rpm_pending = (self.rpm_sequence, time.monotonic())
            self.update_live_text()
            self.sync()
        else:
            self.append("Could not send RPM request to the runner.", "error")

    def update_live_text(self):
        def label(value):
            return f"{value:+g}" if math.isfinite(value) else "—"

        ramped = self.trial.ramped_rpm if self.run_logged and self.trial else math.nan
        reversing = (self.is_running and self.trial and self.trial.points and self.run_logged
                     and self.accepted_rpm != 0 and self.trial.points[-1][len(FIELDS) + 1] == self.accepted_rpm
                     and self.trial.active_target_rpm == 0)
        self.live_text.set(f"{self.live_mode.capitalize()} RPM   ·   Requested {label(self.last_request)}"
                           f"   ·   Accepted {label(self.accepted_rpm)}   ·   Ramped {label(ramped)}"
                           + ("   ·   Waiting for zero" if reversing else ""))

    def confirm(self):
        if self.awaiting_confirmation and not self.stop_requested:
            response = "confirm"
            if self.pso_prompt and self.pso_prompt["kind"] == "guard":
                response = self.guard_value.get().strip()
                if response != str(self.pso_prompt["rpm"]):
                    self.append(f"Enter {self.pso_prompt['rpm']} to confirm the guarded speed stage.", "warning")
                    return
            self.awaiting_confirmation = False
            self.confirmation.pack_forget()
            self.activity.set("Starting…")
            if self.pso_prompt:
                self.answer_pso(response)
            elif not self.send("confirm"):
                self.stop()

    def stop(self):
        if self.process is not None and self.action in STOPPABLE_ACTIONS and not self.stop_requested:
            self.stop_requested = True
            if self.action in MOTOR_ACTIONS and (self.action not in PSO_MOTION or self.approved):
                self.stop_alert_at = time.monotonic() + STOP_ALERT_SECONDS
            self.awaiting_confirmation = False
            self.pso_prompt = None
            self.confirmation.pack_forget()
            self.activity.set("Stopping…")
            self.append("Stop requested.")
            if not self.send("stop"):
                with suppress(OSError):
                    self.process.stdin.close()
            self.sync()

    def freeze(self):
        self.frozen = not self.frozen
        self.browsing = False
        if self.frozen:
            self.charts.detached = True
        else:
            self.charts.follow()
        self.freeze_button.configure(text="Follow live" if self.frozen else "Freeze view")
        if not self.frozen:
            self.plotted_at = 0

    def detach_timeline(self):
        self.frozen = self.browsing = True
        self.freeze_button.configure(text="Follow live")

    def open_logs(self):
        try:
            subprocess.Popen(["xdg-open", str(LOGS)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as exc:
            self.append(str(exc), "error")

    def accept_log(self, path):
        path = Path(path)
        if path.parent != LOGS:
            return
        try:
            trial = TrialLog(path)
        except OSError as exc:
            self.append(f"Cannot display trial log. {exc}", "error")
            return
        if self.trial:
            self.trial.close()
        self.trial = trial
        self.run_logged = self.recording = True
        self.reported_stop = self.frozen = False
        self.browsing = False
        self.plotted_at = 0
        self.freeze_button.configure(text="Freeze view")
        self.log_name.set(str(path.relative_to(WORKSPACE)))
        self.params.set("Loading run settings…")
        self.draw_plot()
        self.append(f"Recording {path.name}")

    def show_parameters(self):
        settings = self.trial.metadata["trial"]
        display_fields = (
            ("vel_gain", "Velocity P", ""), ("vel_integrator_gain", "Velocity I", ""),
            ("position_gain", "Position P", " s⁻¹"), ("kp", "Torque speed P", ""),
            ("ki", "Torque speed I", ""), ("firmware_limiter_gain", "Limiter gain", ""),
            ("ramp_rpm_per_s", "Ramp", " RPM/s"), ("vel_ramp_rate", "Ramp", " turn/s²"),
            ("firmware_overspeed_rpm", "Firmware trip", " RPM"),
            ("live_ceiling_rpm", "Live ceiling", " RPM"),
            ("software_limit_rpm", "Software speed check", " RPM"),
            ("firmware_torque_limit_nm", "Firmware torque cap", " N·m"),
            ("torque_limit_nm", "Trial torque cap", " N·m"),
        )
        items = [f"{label} {settings[key]:g}{unit}" for key, label, unit in display_fields
                 if settings.get(key) is not None]
        current = self.trial.metadata["config"]["hardware"].get("current_lim")
        if current is not None:
            items.append(f"Motor current limit {current:g} A")
        self.params.set("\n".join("  ·  ".join(items[i:i + 2]) for i in range(0, len(items), 2)))

    def draw_plot(self):
        if self.charts.trial is not self.trial:
            self.charts.attach(self.trial)
        else:
            self.charts.update()

    def read_log(self, now):
        trial = self.trial
        if trial is None:
            return
        has_metadata = bool(trial.metadata)
        if trial.stop is None:
            try:
                trial.read(now)
                if not has_metadata and trial.metadata:
                    self.show_parameters()
            except (OSError, ValueError, KeyError, TypeError) as exc:
                self.telemetry.set("Telemetry unavailable.")
                self.append(f"Cannot read trial CSV. {exc}", "error")
                trial.close()
                self.trial = None
                self.charts.attach(None)
                return
        if trial.stop and self.run_logged and not self.reported_stop:
            row = trial.stop
            self.show_result(row.get("stop_reason", "unknown"), row.get("detail", ""),
                             {"True": True, "False": False}.get(row.get("idle_confirmed")),
                             {key: value for key, value in row.items() if key.endswith("_error")})
        if not trial.points:
            self.telemetry.set("Waiting for samples…" if self.recording and not trial.stop else "No samples recorded")
            return
        if self.frozen:
            state = "recording ended"
            if self.recording and not trial.stop:
                state = "telemetry stale" if trial.stale(now) else "recording continues"
            mode = "Browsing history" if self.browsing else "View frozen"
            self.telemetry.set(f"{mode} · {state}")
            self.charts.update_scrollbar()
        elif trial.stop or not self.recording:
            self.telemetry.set("Recorded · " + self.sample_label(trial.points[-1]))
        elif trial.stale(now):
            self.telemetry.set("Telemetry stale · last sample " + self.sample_label(trial.points[-1]))
        else:
            self.telemetry.set(self.sample_label(trial.points[-1]))
        if not self.frozen and trial.last_sample_at != self.plotted_at and now >= self.draw_due:
            self.plotted_at = trial.last_sample_at
            self.draw_due = now + 0.25
            self.draw_plot()
        if self.live_run:
            self.update_live_text()
        if self.freeze_button.instate(("disabled",)):
            self.freeze_button.configure(state="normal")

    @staticmethod
    def sample_label(point):
        fields = ("Command", "Estimated", "Count speed", "Iq command", "Iq measured", "Id", "Bus")
        missing = [label for label, value in zip(fields, point[1:len(FIELDS) + 1], strict=True)
                   if not math.isfinite(value)]
        rpm = f"{point[2]:+.1f} RPM" if math.isfinite(point[2]) else "— RPM"
        bus = f"{point[len(FIELDS)]:.2f} V" if math.isfinite(point[len(FIELDS)]) else "— V"
        label = f"{point[0]:.2f} s  ·  {rpm} estimated  ·  {bus} bus"
        return label + ("  |  Unavailable: " + ", ".join(missing) if missing else "")

    def show_result(self, reason, detail, idle, faults):
        if self.reported_stop:
            return
        self.reported_stop = True
        label = reason.replace("_", " ").capitalize()
        state = {True: "IDLE confirmed", False: "IDLE unconfirmed"}.get(idle, "IDLE not recorded")
        self.result.set(f"{label} · {state}")
        style = "Panel.TLabel" if reason in ("user_stop", "normal_completion") else "Warning.Panel.TLabel"
        self.result_label.configure(style="Danger.Panel.TLabel" if idle is False else style)
        failed = idle is False or reason not in ("user_stop", "normal_completion")
        self.append(f"{label}. {detail}".strip(), "error" if failed else None)
        if masks := fault_text(faults):
            self.append(f"Stop masks · {masks}")
        if idle is False:
            self.flag_unconfirmed()

    def flag_unconfirmed(self):
        self.idle_unconfirmed = True
        self.activity.set("IDLE unconfirmed. Isolate motor power.")
        self.activity_label.configure(style="Danger.TLabel")

    def finish_worker(self, code):
        action = self.action
        if action == "teleop":
            self.teleop_keys.reset()
            self.teleop_state = ("stopping", "")
        with suppress(OSError):
            self.process.stdin.close()
        self.process = self.action = None
        self.confirmation.pack_forget()
        self.awaiting_confirmation = False
        self.pso_prompt = None
        if action in PSO_OFFLINE:
            if self.stop_requested and not self.idle_unconfirmed:
                self.activity.set("Both axes IDLE" if self.status and self.status["both_idle"] else "Check drive")
            self.stop_requested = False
            if self.pso and action == "pso_info" and code == 0:
                self.pso.status.set("Analyze logs or check the drive to capture a reference.")
            self.sync()
            return
        if action == "encoder":
            self.encoder_label.pack_forget()
            self.stop_requested = self.is_running = False
            if self.closing:
                self.destroy()
                return
            self.result.set("Encoder monitor stopped" if code in (0, 130) else "Encoder monitor failed")
            self.result_label.configure(style="Panel.TLabel" if code in (0, 130) else "Danger.Panel.TLabel")
            self.status = None
            self.refresh()
            self.sync()
            return
        if action in TUNING_ACTIONS:
            if action == "advanced_save" and not code:
                if not self.idle_unconfirmed:
                    self.activity.set("Configuration saved. Check drive.")
                    self.activity_label.configure(style="TLabel")
                self.append("Advanced parameters saved. Check drive before the next run.")
            self.sync()
            return
        if action == "status":
            self.stop_alert_at = None
            if code or not self.status:
                self.status = None
                if self.reviewing_stop:
                    self.flag_unconfirmed()
                elif not self.idle_unconfirmed:
                    self.activity.set("Drive unavailable")
            elif not self.status["both_idle"] and self.reviewing_stop:
                self.flag_unconfirmed()
            elif self.status["both_idle"] and not self.reviewing_stop:
                self.idle_unconfirmed = False
            if self.status and not code and not self.idle_unconfirmed:
                self.activity.set("Both axes IDLE" if self.status["both_idle"] else "Drive busy")
                self.activity_label.configure(style="TLabel")
            if self.closing and self.status and self.status["both_idle"] and not self.idle_unconfirmed:
                self.destroy()
                return
            self.reviewing_stop = self.closing = False
        else:
            self.recording = False
            if action not in ("teleop", *SPEED_ACTIONS, *PSO_MOTION) or not self.reported_stop:
                operation = "Operation finished" if code == 0 else "Operation cancelled" if code == 130 else "Operation failed"
                if self.approved and action in ("teleop", *SPEED_ACTIONS):
                    operation = "Test ended · final CSV result unavailable"
                self.result.set(operation)
                self.result_label.configure(style="Warning.Panel.TLabel" if code == 1 else "Panel.TLabel")
            if not self.idle_unconfirmed:
                self.activity.set("Stopping…" if self.stop_requested else "Verifying drive state…")
            self.status = None
            self.reviewing_stop = True
            self.refresh()
            if self.process is None:
                self.flag_unconfirmed()
                self.reviewing_stop = self.closing = False
        self.approved = self.stop_requested = False
        self.is_running = False
        self.rpm_pending = None
        self.sync()

    def handle_pso(self, event, data):
        if (self.pso is None or (self.stop_requested and event != "pso_saved")
                or self.action not in (*PSO_OFFLINE, *PSO_MOTION)):
            return
        if event == "pso_prompt":
            self.pso_prompt = data
            if data["kind"] == "save":
                self.pso.add("Validation passed. Review the per-speed results above.\n"
                             f"Old gains {data['baseline']} → New gains {data['gains']}")
                self.pso.status.set("Save validated gains, or Stop test to discard.")
                self.activity.set("Review validated gains")
                self.sync()
                return
            guard = data["kind"] == "guard"
            description = data["description"]
            target = f" · {data['rpm']:+g} RPM" if "rpm" in data else ""
            self.pso.status.set(description.splitlines()[0] + target)
            if guard:
                rpm = data["rpm"]
                description += (f"\n{rpm:+g} RPM stage. Verify the bench rating covers the firmware trip "
                                f"or arm an independent speed cutoff within the bench rating. Enter {rpm} to confirm.")
            self.show_confirmation(description, guard=guard)
        elif event == "pso_saved":
            self.pso.load(data)
            self.drafts.pop((0, data["mode"]), None)
            self.pso.status.set("Validated gains saved.")
            self.append("Validated PSO gains saved.")
        else:
            self.pso.report(event, data)
            if event == "pso_capture" and self.action == "pso_reference":
                self.pso.status.set("Reference captured. Analyze logs to reassess the model.")
        self.sync()

    def show_confirmation(self, description, *, guard=False):
        self.awaiting_confirmation = True
        self.activity.set("Awaiting guarded speed confirmation" if guard else "Awaiting confirmation")
        self.prompt.set(f"{description}\nSecure the motor, guard the shaft, and keep the power cutoff accessible.")
        self.confirm_button.configure(text="Confirm guarded speed" if guard else "Confirm motion")
        if guard:
            self.guard_value.set("")
            self.guard_input.grid(row=1, column=0, sticky="w", pady=(4, 0))
        else:
            self.guard_input.grid_remove()
        self.confirmation.pack(fill="x", before=self.body)
        self.sync()

    def handle(self, event, data):
        if event.startswith("pso_"):
            self.handle_pso(event, data)
        elif event == "confirm" and not self.stop_requested:
            self.show_confirmation(data)
        elif event == "approved":
            self.approved = True
            self.next_heartbeat = 0
            if self.live_run:
                self.accepted_rpm = self.last_request
                self.update_live_text()
            if not self.stop_requested:
                self.activity.set("Calibrating…" if self.action == "calibrate" else "Starting test…")
            self.sync()
        elif event == "running":
            self.is_running = True
            if self.used_draft is not None:
                self.drafts.pop(self.used_draft, None)
                self.used_draft = None
                self.append("Unsaved tuning applied to this run.")
            if not self.stop_requested and not self.idle_unconfirmed:
                self.activity.set("Teleop active" if self.action == "teleop" else "Test running")
            self.sync()
        elif event == "teleop_state" and self.action == "teleop" and self.is_running and not self.stop_requested:
            self.teleop_state = (data["state"], data["direction"])
            self.sync_teleop()
        elif event == "status":
            self.status = data
        elif event == "csv":
            self.accept_log(data)
        elif event == "encoder_sample" and self.action == "encoder" and not self.stop_requested:
            if not self.is_running:
                self.is_running = True
                self.activity.set("Monitoring encoder while IDLE")
                self.sync()
            self.encoder_text.set(f"Encoder count {data['count']:,}  ·  Change {data['delta']:+,}")
        elif event == "tuning":
            self.load_editor(data, saved=self.action in ("save", "advanced_save"))
        elif event == "trial_result":
            self.is_running = False
            if self.action == "teleop":
                self.teleop_state = ("stopping", "")
            self.show_result(data["reason"], data["detail"], data["idle"], data["faults"])
            if self.action in PSO_MOTION:
                self.approved = self.recording = False
                if not self.stop_requested and not self.idle_unconfirmed:
                    self.activity.set("Checking IDLE and standstill…" if data["idle"] else "Checking drive…")
            self.sync()
        elif event in ("rpm_accepted", "rpm_rejected"):
            if self.rpm_pending is not None and data["sequence"] == self.rpm_pending[0]:
                self.rpm_pending = None
                if event == "rpm_accepted":
                    self.accepted_rpm = data["rpm"]
                    self.append(f"Runner accepted {self.accepted_rpm:+g} RPM.")
                else:
                    self.append(data["detail"], "warning")
                self.update_live_text()
                self.sync()
        elif event == "exit":
            self.read_log(time.monotonic())
            self.finish_worker(data)
        elif event in ("notice", "error", "warning", "cancelled") and data and not data.startswith("RESULT:"):
            self.append(data, event if event in ("error", "warning") else None)
            if event == "error" and self.action in TUNING_ACTIONS and self.editor:
                self.editor.status.set(data)
            if event in ("error", "cancelled") and self.action in (*PSO_OFFLINE, *PSO_MOTION) and self.pso:
                self.pso.status.set("PSO stopped. See details below.")
                self.pso.add(data)
                if self.action == "pso_validate":
                    self.pso.candidate = None
            if data.startswith("DANGER:"):
                self.flag_unconfirmed()

    def poll(self):
        try:
            for _ in range(100):
                event, data = self.messages.get_nowait()
                self.handle(event, data)
                if self.timer is None:
                    return
        except Empty:
            pass
        now = time.monotonic()
        if self.process is not None and self.approved and not self.stop_requested and now >= self.next_heartbeat:
            self.send("heartbeat")
            self.next_heartbeat = now + HEARTBEAT_INTERVAL
        if self.rpm_pending is not None and now - self.rpm_pending[1] >= 2 and not self.stop_requested:
            self.append("Runner did not acknowledge the RPM request.", "error")
            self.stop()
        if self.stop_alert_at is not None and now >= self.stop_alert_at and not self.idle_unconfirmed:
            self.flag_unconfirmed()
            self.sync()
        self.read_log(now)
        self.charts.poll()
        self.timer = self.root.after(60, self.poll)

    def close(self):
        if self.process is None:
            self.destroy()
        elif self.action in ("status", *TUNING_ACTIONS, *PSO_OFFLINE) and not self.reviewing_stop:
            self.process.stdin.close()
            self.destroy()
        else:
            self.closing = True
            self.stop()

    def destroy(self):
        self.teleop_keys.reset()
        if self.timer is not None:
            self.root.after_cancel(self.timer)
            self.timer = None
        if self.trial:
            self.trial.close()
        if self.pane_sizing is not None:
            self.root.after_cancel(self.pane_sizing)
        if self.editor_reload is not None:
            self.root.after_cancel(self.editor_reload)
        self.charts.close()
        self.root.destroy()


def main():
    root = tk.Tk(className="ODriveBench")
    app = BenchApp(root)

    def maximize(event):
        if event.widget is root:
            root.unbind("<Map>", startup_map)
            root.after_idle(root.attributes, "-zoomed", True)

    startup_map = root.bind("<Map>", maximize, add="+")
    root.after(100, app.refresh)
    root.mainloop()


if __name__ == "__main__":
    main()
