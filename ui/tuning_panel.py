"""Docked basic tuning and reviewed advanced config edits."""
import math
import tkinter as tk
from tkinter import ttk


def _text(value):
    if value is None:
        return "null"
    if type(value) is bool:
        return "true" if value else "false"
    return str(value)


class TuningEditor:
    def __init__(self, parent, apply, save, reload, close):
        self.window = ttk.Frame(parent, padding=(10, 10), style="Panel.TFrame")
        body = self.window
        self.title = tk.StringVar(master=body, value="Tune parameters")
        self.status = tk.StringVar(master=body, value="Loading parameters…")
        self.advanced = tk.BooleanVar(master=body)
        self.snapshot = None
        self.reviewed = None
        self.locked = False
        self.mismatch = False
        self.entries = {}
        heading = ttk.Frame(body, style="Panel.TFrame")
        heading.pack(fill="x")
        ttk.Label(heading, textvariable=self.title, style="Section.TLabel").pack(side="left")
        ttk.Button(heading, text="Close", command=close).pack(side="right")
        self.toggle = ttk.Checkbutton(body, text="Advanced", variable=self.advanced, command=self.render)
        self.toggle.pack(anchor="w", pady=(4, 5))
        tools = ttk.Frame(body, style="Panel.TFrame")
        tools.pack(side="bottom", fill="x")
        self.reload_button = ttk.Button(tools, text="Reload parameters", command=reload)
        self.reload_button.pack(fill="x", pady=(4, 3))
        self.save_button = ttk.Button(tools, text="Save parameters", command=save)
        self.save_button.pack(fill="x", pady=(0, 3))
        self.apply_button = ttk.Button(tools, text="Use for next run", command=apply)
        self.apply_button.pack(fill="x")
        self.cancel_review = ttk.Button(tools, text="Back to parameters", command=self.back)
        self.status_label = ttk.Label(body, textvariable=self.status, style="Muted.TLabel", wraplength=220)
        self.status_label.pack(side="bottom", fill="x", pady=(4, 2))
        scroll = ttk.Frame(body, style="Panel.TFrame")
        scroll.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(scroll, width=200, highlightthickness=0)
        bar = ttk.Scrollbar(scroll, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=bar.set)
        bar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.form = ttk.Frame(self.canvas, style="Panel.TFrame")
        self.window_item = self.canvas.create_window(0, 0, anchor="nw", window=self.form)
        self.canvas.bind("<Configure>", self.resize)
        self.form.bind("<Configure>", lambda _e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.saved_edits = {}
        self.form.bind("<Button-4>", self.scroll)
        self.form.bind("<Button-5>", self.scroll)
        self.canvas.bind("<Button-4>", self.scroll)
        self.canvas.bind("<Button-5>", self.scroll)
        self.busy(True)

    def resize(self, event):
        self.canvas.itemconfigure(self.window_item, width=event.width)
        self.status_label.configure(wraplength=max(100, event.width))

    def scroll(self, event):
        self.canvas.yview_scroll(-1 if event.num == 4 else 1, "units")
        return "break"

    def style(self, palette):
        self.canvas.configure(background=palette.panel)

    def load(self, snapshot, draft=None):
        self.snapshot = snapshot
        self.reviewed = None
        self.entries.clear()
        self.saved_edits = {False: {}, True: {}}
        self.title.set(f"M{snapshot['axis']} · {snapshot['mode'].capitalize()}")
        self.status.set(snapshot["warning"])
        if draft:
            if draft["revision"] == snapshot["revision"]:
                self.saved_edits[False] = {k: _text(v) for k, v in draft["edits"].items()}
                self.status.set("Unsaved draft loaded.")
            else:
                self.status.set("The saved config changed. Review the current values and re-enter your draft.")
        self.render()

    def remember_inputs(self):
        if self.entries:
            self.saved_edits[self.show_advanced] = {key: var.get() for key, (_, var, _) in self.entries.items()}

    def dirty(self):
        if self.reviewed:
            return True
        self.remember_inputs()
        basic = {key: _text(value) for key, _, value in self.snapshot["fields"]}
        advanced = {field["key"]: _text(field["value"]) for field in self.snapshot["advanced"]}
        return any(value != defaults.get(key) for mode, defaults in ((False, basic), (True, advanced))
                   for key, value in self.saved_edits[mode].items())

    def bind_wheel(self):
        for widget in self.form.winfo_children():
            widget.bind("<Button-4>", self.scroll)
            widget.bind("<Button-5>", self.scroll)

    def reset_form(self):
        for widget in self.form.winfo_children():
            widget.destroy()
        self.canvas.yview_moveto(0)

    def render(self):
        if not self.snapshot:
            return
        self.remember_inputs()
        self.entries.clear()
        self.reviewed = None
        self.reset_form()
        self.show_advanced = advanced = self.advanced.get()
        self.cancel_review.pack_forget()
        if advanced:
            self.apply_button.pack_forget()
            self.save_button.configure(text="Review changes")
            rows = self.snapshot["advanced"]
        else:
            self.apply_button.pack(fill="x")
            self.save_button.configure(text="Save parameters")
            rows = [
                {"key": key, "label": label, "value": value, "kind": "number",
                 "nullable": False, "editable": True} for key, label, value in self.snapshot["fields"]
            ]
        previous = None
        for field in rows:
            group = field.get("group")
            if group and group != previous:
                ttk.Label(self.form, text=group.capitalize(), style="Section.TLabel").pack(anchor="w", pady=(9, 3))
                previous = group
            label = ttk.Label(self.form, text=field["label"], style="Panel.TLabel", wraplength=184)
            label.pack(anchor="w")
            value = field["value"]
            if not field["editable"]:
                ttk.Label(self.form, text=f"{_text(value)} · read-only", style="Muted.TLabel").pack(anchor="w", pady=(0, 4))
                continue
            key = field["key"]
            var = tk.StringVar(master=self.window, value=self.saved_edits[advanced].get(key, _text(value)))
            if field["kind"] == "bool":
                choices = ("true", "false", "null") if field["nullable"] else ("true", "false")
                widget = ttk.Combobox(self.form, textvariable=var, values=choices, state="readonly", width=14)
            else:
                widget = ttk.Entry(self.form, textvariable=var, width=14)
            widget.pack(fill="x", pady=(2, 1))
            if advanced:
                suffix = " · accepts null" if field["nullable"] else ""
                ttk.Label(self.form, text=field["scope"] + suffix, style="Muted.TLabel",
                          wraplength=184).pack(anchor="w", pady=(0, 6))
            else:
                widget.pack_configure(pady=(2, 6))
            self.entries[key] = (field, var, widget)
        if not rows:
            ttk.Label(self.form, text="No basic tuning for this profile.", style="Muted.TLabel",
                      wraplength=180).pack(anchor="w")
        self.bind_wheel()
        self.busy(self.locked, self.mismatch)

    def draft(self):
        edits = {}
        for key, (field, var, _) in self.entries.items():
            text = var.get().strip()
            if text == _text(field["value"]):
                continue
            try:
                if field["nullable"] and text == "null":
                    value = None
                elif field["kind"] == "bool":
                    value = {"true": True, "false": False}[text]
                elif field["kind"] == "int":
                    value = int(text)
                else:
                    value = float(text)
                    if not math.isfinite(value):
                        raise ValueError
            except (ValueError, KeyError):
                self.status.set(f"{field['label']} needs a valid {field['kind']}.")
                return None
            if value != field["value"]:
                edits[key] = value
        return {"revision": self.snapshot["revision"], "edits": edits}

    def review(self, draft):
        self.reviewed = draft
        fields = {field["key"]: field for field in self.snapshot["advanced"]}
        self.remember_inputs()
        self.entries.clear()
        self.reset_form()
        ttk.Label(self.form, text="Review configuration", style="Section.TLabel").pack(anchor="w", pady=(3, 6))
        for key, value in draft["edits"].items():
            field = fields[key]
            ttk.Label(self.form, text=field["label"], style="Panel.TLabel",
                      wraplength=180).pack(anchor="w")
            ttk.Label(self.form, text=f"{_text(field['value'])} → {_text(value)}", style="Muted.TLabel",
                      wraplength=180).pack(anchor="w", pady=(0, 6))
        self.status.set("Writes motor_config.json. Check drive again after saving.")
        self.cancel_review.pack(fill="x")
        self.save_button.configure(text="Save reviewed changes")
        self.bind_wheel()
        self.busy(self.locked, self.mismatch)

    def back(self):
        self.status.set("")
        self.render()

    def busy(self, busy, mismatch=False):
        self.locked = busy
        self.mismatch = mismatch
        allowed = not (busy or mismatch)
        self.reload_button.configure(state="disabled" if busy else "normal")
        self.toggle.configure(state="disabled" if busy or not self.snapshot or self.reviewed else "normal")
        self.apply_button.configure(state="normal" if allowed and self.entries else "disabled")
        self.save_button.configure(state="normal" if allowed and (self.entries or self.reviewed) else "disabled")
        self.cancel_review.configure(state="disabled" if busy else "normal")
        for field, _, widget in self.entries.values():
            widget.configure(state=("readonly" if field["kind"] == "bool" else "normal") if allowed else "disabled")
