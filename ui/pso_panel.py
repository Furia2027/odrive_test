"""Controls and results for supervised PSO tuning."""
from pathlib import Path
import tkinter as tk
from tkinter import ttk


def metrics(values):
    if values is None:
        return "No comparable clean baseline"
    ripple, peak, ramp, plateau = values
    return (f"Ripple {ripple:.3g} · Overshoot {peak:.3g} · "
            f"Ramp RMSE {ramp:.3g} · Plateau error {plateau:.3g} RPM")


class PsoPanel:
    def __init__(self, parent, start, close):
        self.window = ttk.Frame(parent, padding=10, style="Panel.TFrame")
        body = self.window
        self.snapshot = self.candidate = None
        self.mode = tk.StringVar(master=body, value="Position")
        self.rpm = tk.StringVar(master=body)
        self.gains = [tk.StringVar(master=body) for _ in range(2)]
        self.current = tk.StringVar(master=body)
        self.status = tk.StringVar(master=body, value="Loading gains…")
        header = ttk.Frame(body, style="Panel.TFrame")
        header.pack(fill="x")
        ttk.Label(header, text="M0 · PSO tuning", style="Section.TLabel").pack(side="left")
        self.close_button = ttk.Button(header, text="Close", command=close)
        self.close_button.pack(side="right")
        self.mode_picker = ttk.Combobox(body, textvariable=self.mode, values=("Position", "Torque"),
                                        state="readonly", width=12)
        self.mode_picker.pack(fill="x", pady=(5, 4))
        self.mode_picker.bind("<<ComboboxSelected>>", lambda _event: start("pso_info"))
        summary = ttk.Label(body, textvariable=self.current, style="Muted.TLabel", wraplength=245)
        summary.pack(fill="x")

        form = ttk.Frame(body, style="Panel.TFrame")
        form.pack(fill="x", pady=4)
        form.columnconfigure(0, weight=1)
        ttk.Label(form, text="Reference [signed RPM]", style="Panel.TLabel").grid(row=0, column=0, sticky="w")
        self.field_labels = [ttk.Label(form, style="Panel.TLabel") for _ in self.gains]
        self.inputs = []
        for row, variable in enumerate((self.rpm, *self.gains)):
            if row:
                self.field_labels[row - 1].grid(row=row, column=0, sticky="w")
            entry = ttk.Entry(form, textvariable=variable, width=9)
            entry.grid(row=row, column=1, sticky="e", pady=1)
            self.inputs.append(entry)

        self.buttons = {}
        for action, label in (
                ("pso_analyze", "Analyze logs"), ("pso_reference", "Capture reference"),
                ("pso_validate", "Validate finalist"), ("pso_save", "Save validated gains")):
            button = ttk.Button(body, text=label, command=lambda action=action: start(action))
            button.pack(fill="x", pady=(0, 3))
            self.buttons[action] = button
        status = ttk.Label(body, textvariable=self.status, wraplength=245, style="Muted.TLabel")
        status.pack(fill="x", pady=(3, 4))
        output = ttk.Frame(body, style="Panel.TFrame")
        output.pack(fill="both", expand=True)
        self.results = tk.Text(output, width=22, height=4, wrap="word", padx=5, pady=5, state="disabled")
        bar = ttk.Scrollbar(output, command=self.results.yview)
        self.results.configure(yscrollcommand=bar.set)
        bar.pack(side="right", fill="y")
        self.results.pack(side="left", fill="both", expand=True)
        body.bind("<Configure>", lambda event: (
            summary.configure(wraplength=max(160, event.width - 20)),
            status.configure(wraplength=max(160, event.width - 20))))

    def style(self, palette):
        self.results.configure(background=palette.field, foreground=palette.ink,
                               selectbackground=palette.selection, selectforeground=palette.ink)

    def load(self, data):
        changed = not self.snapshot or any(self.snapshot[key] != data[key] for key in ("mode", "revision"))
        if changed:
            self.candidate = None
            for label, variable, (_, field, value) in zip(self.field_labels, self.gains, data["fields"]):
                label.configure(text=field.replace("Position-mode velocity", "Position velocity"))
                variable.set(str(value))
        self.snapshot = data
        pairs = [f"{value:g} [{lo:g}–{hi:g}]" for (_, _, value), (lo, hi) in zip(data["fields"], data["bounds"])]
        self.current.set(f"Current gains [search bounds]\n{'  ·  '.join(pairs)}")

    def add(self, line, *, clear=False):
        self.results.configure(state="normal")
        if clear:
            self.results.delete("1.0", "end")
        if line:
            self.results.insert("end", line + "\n")
            self.results.see("end")
        self.results.configure(state="disabled")

    def controls(self, busy, drive_ready, can_save):
        self.close_button.configure(state="disabled" if busy else "normal")
        self.mode_picker.configure(state="disabled" if busy else "readonly")
        for entry in self.inputs:
            entry.configure(state="disabled" if busy else "normal")
        for action, enabled in (
                ("pso_analyze", not busy), ("pso_reference", not busy and drive_ready and self.snapshot),
                ("pso_validate", not busy and drive_ready and self.candidate), ("pso_save", can_save)):
            self.buttons[action].configure(state="normal" if enabled else "disabled")

    def report(self, event, data):
        if event == "pso_info":
            self.load(data)
        elif event == "pso_analysis":
            if "compatible" in data:
                skipped = data["skipped"]
                self.add(f"{data['compatible']} compatible CSVs · {sum(skipped.values())} skipped")
                if "usable" in data:
                    self.add(f"{data['usable']} usable plateaus · {len(data.get('faults', ()))} drive faults")
                for reason, count in skipped.items():
                    self.add(f"{count} {reason}")
            if "held_out" in data:
                self.add(f"{data['training']} training runs · {data['held_out']} held out")
            if "failures" in data:
                self.add(f"{len(data['failures'])} failed holdout checks")
                if data.get("prediction_errors"):
                    self.add("Holdout metrics in RPM (error / limit)")
                for failure in data["failures"]:
                    self.add(failure)
            for prediction in data.get("prediction_errors", ()):
                if prediction not in data.get("failures", ()):
                    self.add(prediction)
            if data.get("missing"):
                self.add("Complete plateau recordings missing at "
                         + ", ".join(f"{rpm:+g}" for rpm in data["missing"]) + " RPM")
            if data.get("unqualified"):
                self.add("Recorded speeds that failed model qualification "
                         + ", ".join(f"{rpm:+g}" for rpm in data["unqualified"]) + " RPM")
            for key in ("recording_issues", "faults", "blocked", "excluded", "coverage"):
                for line in data.get(key, ()):
                    self.add(line)
            if data.get("scope"):
                self.add("Provisional offline-only scope" if data["provisional"] else "Full-range offline scope")
                for line in data["scope"]:
                    self.add(line)
            if data.get("ranked") and data.get("provisional"):
                self.candidate = None
                self.status.set("Provisional ranking. Offline review only.")
            for index, pair in enumerate(data.get("ranked", ()), 1):
                gains = " / ".join(f"{gain:g}" for gain in pair["gains"])
                self.add(f"Rank {index} · {gains} · Predicted score {pair['score']:.5g}")
        elif event == "pso_candidate":
            self.candidate = data
            self.status.set("Finalist ready for bench validation.")
            self.add(f"Offline finalist {' / '.join(f'{g:g}' for g in data['gains'])} · Predicted score {data['score']:.5g}")
        elif event == "pso_capture":
            self.add(f"{data['rpm']:+g} RPM · {Path(data['csv']).name}\n{metrics(data['metrics'])}")
        elif event == "pso_comparison":
            baseline = f"Baseline {Path(data['baseline_csv']).name}\n{metrics(data['baseline'])}"
            self.add(f"{data['rpm']:+g} RPM · Finalist {data['attempt']}/2 passed\n"
                     f"{baseline}\nFinalist {metrics(data['candidate'])}\n")
