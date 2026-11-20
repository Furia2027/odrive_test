"""CSV telemetry and a shared, scrollable time window."""
from bisect import bisect_right
from collections import deque
import csv
import json
import math
from queue import Empty, SimpleQueue
from threading import Event, Thread
import tkinter as tk
from tkinter import ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


WINDOW_SECONDS = 30
STALE_SECONDS = 1
SPEED = ("setpoint_rpm", "estimated_rpm", "encoder_count_rpm")
CURRENT = ("iq_setpoint_a", "iq_measured_a", "id_measured_a")
BUS = ("vbus_v",)
FIELDS = SPEED + CURRENT + BUS
TARGET = "requested_rpm"
TIMING = ("read_span_s", "loop_interval_s")
DEFAULT_GRAPHS = ("Speed", "Current", "DC bus")
GRAPHS = {
    "Speed": ("RPM", (*zip(SPEED, ("Command", "Estimate", "Count")), (TARGET, "Target"))),
    "Current": ("A", tuple(zip(CURRENT, ("Iq command", "Iq measured", "Id measured")))),
    "DC bus": ("V", (("vbus_v", "Voltage"),)),
    "Torque": ("N·m", (("command_torque_nm", "Command"), ("firmware_torque_setpoint_nm", "Firmware setpoint"),
                       ("active_torque_limit_nm", "Limit"))),
    "Speed error": ("RPM", (("speed_error_rpm", "Logged error"),)),
    "Shaft rotation": ("°", (("rotation_deg", "Accumulated rotation"),)),
    "Position": ("turns", (("commanded_position_turns", "Command"), ("measured_position_turns", "Measured"))),
    "Following error": ("turns", (("active_following_error_turns", "Active"), ("following_error_turns", "Next"))),
    "Bus current": ("A", (("ibus_a", "DC bus current"),)),
    "Loop timing": ("ms", tuple(zip(TIMING, ("USB feedback read", "Loop interval")))),
}
POINT_FIELDS = FIELDS + (TARGET,) + tuple(key for name, (_, series) in GRAPHS.items()
                                        if name not in DEFAULT_GRAPHS for key, _ in series)
LINE_STYLES = ("--", "-", ":", "-.")
SAMPLE_BUFFER = 4096


def number(raw):
    try:
        value = float(raw)
    except (ValueError, TypeError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def _complete_lines(stream):
    while line := stream.readline():
        if not line.endswith("\n"):
            return
        yield line


def _record(stream):
    try:
        return next(csv.reader(_complete_lines(stream), strict=True))
    except (StopIteration, csv.Error):
        return None


def _point(row, elapsed):
    if row.get("active_velocity_feedforward_rpm"):
        row["setpoint_rpm"] = row["active_velocity_feedforward_rpm"]
    if row.get("active_position_turns"):
        row["commanded_position_turns"] = row["active_position_turns"]
    return (elapsed, *(number(row.get(key)) * (1000 if key in TIMING else 1) for key in POINT_FIELDS))


class TrialLog:
    def __init__(self, path):
        self.path = path
        self.stream = path.open(encoding="utf-8", newline="")
        self.header = None
        self.metadata = {}
        self.points = deque(maxlen=SAMPLE_BUFFER)
        self.index = []
        self.stop = None
        self.last_sample_at = None
        self.ramped_rpm = math.nan
        self.active_target_rpm = math.nan

    @property
    def latest(self):
        return self.points[-1][0] if self.points else 0

    def read(self, now, limit=1024):
        for _ in range(limit):
            position = self.stream.tell()
            record = _record(self.stream)
            if record is None:
                self.stream.seek(position)
                break
            if self.header is None:
                self.header = record
                continue
            row = dict(zip(self.header, record))
            event = row.get("event")
            if event == "start":
                self.metadata = json.loads(row["config_json"])
            elif event == "stop":
                self.stop = row
            elif event == "sample":
                elapsed = number(row.get("elapsed_s"))
                if not math.isfinite(elapsed):
                    continue
                if not self.index or elapsed >= self.index[-1][0] + 5:
                    self.index.append((elapsed, position))
                self.ramped_rpm = number(row.get("setpoint_rpm"))
                self.active_target_rpm = number(row.get("active_target_rpm"))
                self.points.append(_point(row, elapsed))
                self.last_sample_at = now
                while self.points[0][0] < elapsed - WINDOW_SECONDS:
                    self.points.popleft()

    def offset_for(self, start):
        slot = bisect_right(self.index, start, key=lambda entry: entry[0]) - 1
        return self.index[max(0, slot)][1]

    def history(self, offset, start, end, cancel):
        points = deque(maxlen=SAMPLE_BUFFER)
        with self.path.open(encoding="utf-8", newline="") as stream:
            stream.seek(offset)
            while not cancel.is_set() and (record := _record(stream)) is not None:
                row = dict(zip(self.header, record))
                if row.get("event") != "sample":
                    continue
                elapsed = number(row.get("elapsed_s"))
                if elapsed > end:
                    break
                if elapsed >= start:
                    points.append(_point(row, elapsed))
        return points

    def stale(self, now):
        return self.last_sample_at is not None and now - self.last_sample_at >= STALE_SECONDS

    def close(self):
        self.stream.close()


class PlotCanvas(FigureCanvasTkAgg):
    def resize(self, event):
        # Scrolling moves the Tk widget without changing the rendered image size.
        if (event.width, event.height) != tuple(round(size) for size in self.figure.bbox.size):
            super().resize(event)


class TelemetryCharts:
    def __init__(self, parent, change_view, palette):
        self.change_view = change_view
        self.palette = palette
        self.figure = Figure(figsize=(5, 5), dpi=100)
        self.timeline = ttk.Scrollbar(parent, orient="horizontal", command=self.scroll)
        self.timeline.pack(side="bottom", fill="x", pady=(3, 0))
        self.timeline.set(0, 1)
        self.frame = ttk.Frame(parent, style="Panel.TFrame")
        self.frame.pack(fill="both", expand=True, pady=(4, 0))
        self.viewport = tk.Canvas(self.frame, highlightthickness=0, width=300, height=220)
        self.vertical = ttk.Scrollbar(self.frame, orient="vertical", command=self.viewport.yview)
        self.viewport.configure(yscrollcommand=self.vertical.set)
        self.viewport.pack(side="left", fill="both", expand=True)
        self.canvas = PlotCanvas(self.figure, master=self.viewport)
        self.plot_window = self.viewport.create_window(0, 0, window=self.canvas.get_tk_widget(), anchor="nw")
        self.viewport.bind("<Configure>", self.resize_plot)
        self.bind_scroll(self.canvas.get_tk_widget())
        self.styles = ttk.Style(parent)
        self.optional_label = ttk.Label(self.viewport, text="Optional graphs", style="Muted.TLabel")
        self.optional_heading = self.viewport.create_window(0, 0, anchor="nw", window=self.optional_label)
        self.bind_scroll(self.optional_label)
        self.enabled, self.traces_enabled = {}, {}
        self.rows, self.graph_controls, self.trace_controls = {}, {}, {}
        for name, (_, traces) in GRAPHS.items():
            row = ttk.Frame(self.viewport, style="Panel.TFrame")
            self.rows[name] = self.viewport.create_window(0, 0, anchor="nw", window=row)
            visible = self.enabled[name] = tk.BooleanVar(master=parent, value=name in DEFAULT_GRAPHS)
            self.graph_controls[name] = ttk.Checkbutton(row, text=name, variable=visible,
                                                        style="Graph.TCheckbutton", command=self.rebuild)
            self.bind_scroll(row)
            self.bind_scroll(self.graph_controls[name])
            for slot, (key, label) in enumerate(traces):
                var = self.traces_enabled[key] = tk.BooleanVar(master=parent, value=True)
                trace = ttk.Checkbutton(row, text=label, variable=var, style=f"Trace{slot}.TCheckbutton",
                                        command=self.rebuild)
                self.trace_controls[key] = trace
                self.bind_scroll(trace)
        self.trial = None
        self.left = 0.0
        self.detached = False
        self.points = ()
        self.available = set(FIELDS)
        self.wanted = self.reading = None
        self.ready = SimpleQueue()
        self.cancel = Event()
        self.drag = None
        self.drag_direction = None
        self.zoom_limits = {}
        self.vertical_limits = {}
        self.canvas.mpl_connect("button_press_event", self.press)
        self.canvas.mpl_connect("motion_notify_event", self.pan)
        self.canvas.mpl_connect("button_release_event", self.release)
        self.canvas.mpl_connect("scroll_event", self.zoom)
        self.rebuild()

    @property
    def extent(self):
        return max(WINDOW_SECONDS, self.trial.latest if self.trial else 0)

    def bind_scroll(self, widget):
        for button, step in (("<Button-4>", -1), ("<Button-5>", 1)):
            widget.bind(button, lambda event, step=step: self.scroll_panel(event, step), add="+")

    def scroll_panel(self, event, step):
        if not event.state & 0x0004:
            self.viewport.yview_scroll(step, "units")

    def style(self, palette):
        self.palette = palette
        self.viewport.configure(background=palette.panel)
        self.figure.set_facecolor(palette.panel)
        self.styles.configure("Graph.TCheckbutton", background=palette.panel,
                              foreground=palette.ink, font=("DejaVu Sans", 10, "bold"))
        for slot, color in enumerate(palette.lines):
            name = f"Trace{slot}.TCheckbutton"
            self.styles.configure(name, background=palette.panel, foreground=color,
                                  font=("DejaVu Sans", 9))
            self.styles.map(name, foreground=[("disabled", palette.muted), ("!disabled", color)])
        for ax in self.axes:
            ax.set_facecolor(palette.chart)
            ax.tick_params(colors=palette.muted, labelsize=8)
            ax.xaxis.label.set_color(palette.ink)
            ax.yaxis.label.set_color(palette.ink)
            ax.grid(axis="y", color=palette.border, linewidth=0.6)
            for spine in ax.spines.values():
                spine.set_color(palette.border)
            for text in ax.texts:
                text.set_color(palette.muted)
        for slot, line in self.lines.values():
            line.set_color(palette.lines[slot])
        if self.empty:
            self.empty.set_color(palette.muted)
        self.canvas.draw_idle()

    def rebuild(self):
        self.drag = None
        self.figure.clear()
        self.axes, self.lines = [], {}
        self.graph_axes = {}
        self.empty = None
        for name, (units, traces) in GRAPHS.items():
            keys = [key for key, _ in traces if key in self.available]
            self.viewport.itemconfigure(self.rows[name], state="normal" if keys or name not in DEFAULT_GRAPHS else "hidden")
            self.graph_controls[name].configure(state="normal" if keys else "disabled")
            if not keys or not self.enabled[name].get():
                continue
            ax = self.figure.add_axes((0.2, 0.2, 0.6, 0.5))
            self.axes.append(ax)
            self.graph_axes[name] = ax
            ax.set_ylabel(units, fontsize=8)
            for slot, (key, _) in enumerate(traces):
                if key in keys and self.traces_enabled[key].get():
                    line, = ax.plot([], [], linestyle=LINE_STYLES[slot], linewidth=1.2)
                    self.lines[key] = (slot, line)
            if not ax.lines:
                ax.text(0.5, 0.5, "No traces selected", ha="center", va="center", transform=ax.transAxes)
        if self.axes:
            self.axes[-1].set_xlabel("Elapsed time [s]", fontsize=9)
        else:
            self.empty = self.figure.text(0.5, 0.4, "Select a graph to display", ha="center", va="center")
        self.style(self.palette)
        self.resize_plot()
        self.draw()

    def layout_controls(self, name, width):
        title = self.graph_controls[name]
        widgets = [title]
        for key, _ in GRAPHS[name][1]:
            trace = self.trace_controls[key]
            if name in self.graph_axes and key in self.available:
                widgets.append(trace)
            else:
                trace.place_forget()
        x = y = row_height = 0
        for widget in widgets:
            w, h = widget.winfo_reqwidth(), widget.winfo_reqheight()
            if x and x + w > width:
                x = 0
                y += row_height
                row_height = 0
            widget.place(x=x, y=y)
            x += w + 8
            row_height = max(row_height, h)
        return y + row_height

    def resize_plot(self, _event=None):
        width, visible = self.viewport.winfo_width(), self.viewport.winfo_height()
        if width <= 1 or visible <= 1:
            return
        inset, gap, chart_top, bottom = 10, 10, 6, 56
        controls_width = max(1, width - 2 * inset)
        headers = [(name, self.layout_controls(name, controls_width)) for name in GRAPHS
                   if name not in DEFAULT_GRAPHS or not self.graph_controls[name].instate(("disabled",))]
        heading_space = self.optional_label.winfo_reqheight() + gap
        zoom_spacing = {name for name, ax in self.graph_axes.items()
                        if name in self.zoom_limits and ax is not self.axes[-1]}
        fixed = (sum(h + gap for _, h in headers) + inset + bottom
                 + chart_top * len(self.axes) + heading_space + 18 * len(zoom_spacing))
        trailing = []
        for name, row_height in reversed(headers):
            if name in self.graph_axes:
                break
            trailing.append(row_height + gap)
        # Leave room for the first collapsed graph when the viewport is tall enough.
        plot_height = max(100, (visible - fixed + sum(trailing[:-1])) // len(self.axes)) if self.axes else 0
        height = max(visible, fixed + plot_height * len(self.axes))
        if height > visible:
            self.vertical.pack(side="right", fill="y")
        else:
            self.vertical.pack_forget()
            self.viewport.yview_moveto(0)
        self.viewport.itemconfigure(self.plot_window, width=width, height=height)
        self.viewport.configure(scrollregion=(0, 0, width, height))
        margin = min(72 / width, 0.24)
        y = inset
        heading_pending = True
        for name, row_height in headers:
            if heading_pending and name not in DEFAULT_GRAPHS:
                self.viewport.coords(self.optional_heading, inset, y)
                y += heading_space
                heading_pending = False
            self.viewport.coords(self.rows[name], inset, y)
            self.viewport.itemconfigure(self.rows[name], width=controls_width, height=row_height)
            y += row_height
            if ax := self.graph_axes.get(name):
                y += chart_top + plot_height
                ax.set_position((margin, 1 - y / height, 1 - 2 * margin, plot_height / height))
                ax.tick_params(labelbottom=name in self.zoom_limits or ax is self.axes[-1])
            y += gap
            if self.axes and self.graph_axes.get(name) is self.axes[-1]:
                y += bottom
            elif name in zoom_spacing:
                y += 18
        if self.empty:
            self.empty.set_position((0.5, 1 - (y + bottom / 2) / height))
        self.canvas.draw_idle()

    def attach(self, trial):
        self.trial = trial
        self.left = 0.0
        self.detached = False
        self.points = ()
        self.wanted = None
        self.drag = None
        self.vertical_limits.clear()
        if self.zoom_limits:
            self.zoom_limits.clear()
            self.rebuild()
        self.update()

    def update(self):
        available = set(self.trial.header or FIELDS) if self.trial else set(FIELDS)
        if "active_position_turns" in available:
            available.add("commanded_position_turns")
        if not self.detached:
            self.left = max(0, self.extent - WINDOW_SECONDS)
            self.points = tuple(self.trial.points) if self.trial else ()
        if available != self.available:
            self.available = available
            self.rebuild()
        else:
            self.draw()

    def draw(self):
        columns = tuple(zip(*self.points)) if self.points else ((),) * (len(POINT_FIELDS) + 1)
        for key, (_, line) in self.lines.items():
            line.set_data(columns[0], columns[POINT_FIELDS.index(key) + 1])
        self.set_time_limits()
        for name, ax in self.graph_axes.items():
            ax.relim(visible_only=True)
            if name in self.vertical_limits:
                ax.set_ylim(self.vertical_limits[name])
            elif not all(math.isfinite(limit) for limit in ax.dataLim.intervaly):
                ax.set_ylim(-1, 1, auto=True)
            else:
                ax.set_autoscaley_on(True)
                ax.autoscale_view(scalex=False)
        self.update_scrollbar()
        self.canvas.draw_idle()

    def set_time_limits(self):
        for name, ax in self.graph_axes.items():
            start, end = self.zoom_limits.get(name, (0, WINDOW_SECONDS))
            ax.set_xlim(self.left + start, self.left + end)

    def update_scrollbar(self):
        total = self.extent
        self.timeline.set(self.left / total, min(1, (self.left + WINDOW_SECONDS) / total))

    def view_at(self, left):
        if not self.trial or not self.trial.points or self.extent <= WINDOW_SECONDS:
            return
        left = min(max(0, left), self.extent - WINDOW_SECONDS)
        if self.detached and left == self.left:
            return
        self.detached = True
        self.change_view()
        self.left = left
        end = self.left + WINDOW_SECONDS
        self.wanted = (self.trial, self.left, end)
        self.set_time_limits()
        self.update_scrollbar()
        self.canvas.draw_idle()
        self.load_pending()

    def load_pending(self):
        if self.wanted is None or self.reading is not None:
            return
        self.reading = request = self.wanted
        trial, start, end = request
        offset = trial.offset_for(start)

        def load():
            try:
                result = trial.history(offset, start, end, self.cancel)
            except (OSError, ValueError):
                result = ()
            self.ready.put((request, result))

        Thread(target=load, daemon=True).start()

    def poll(self):
        try:
            request, points = self.ready.get_nowait()
        except Empty:
            return
        self.reading = None
        if self.wanted == request:
            self.wanted = None
            if self.detached:
                self.points = points
                self.draw()
        self.load_pending()

    def follow(self):
        self.detached = False
        self.wanted = None
        self.update()

    def scroll(self, *args):
        if not self.trial or not self.trial.points:
            return
        if args[0] == "moveto":
            left = float(args[1]) * self.extent
        else:
            left = self.left + int(args[1]) * (WINDOW_SECONDS if args[2] == "pages" else 3)
        self.view_at(left)

    def press(self, event):
        if event.button == 1 and event.inaxes in self.axes:
            ax = event.inaxes
            name = next(name for name, graph in self.graph_axes.items() if graph is ax)
            if event.dblclick:
                self.drag = None
                vertical = self.vertical_limits.pop(name, None)
                zoomed = self.zoom_limits.pop(name, None)
                if zoomed:
                    self.resize_plot()
                if zoomed or vertical:
                    self.draw()
                return
            start, end = ax.get_xlim()
            self.drag = (name, event, self.left, (end - start) / ax.bbox.width, ax.get_ylim(), ax.bbox.height)
            self.drag_direction = None

    def pan(self, event):
        if self.drag is None or event.x is None or event.y is None:
            return
        name, origin, left, seconds_per_pixel, (low, high), height = self.drag
        dx, dy = event.x - origin.x, event.y - origin.y
        if self.drag_direction is None:
            if max(abs(dx), abs(dy)) < 5:
                return
            self.drag_direction = "horizontal" if abs(dx) >= abs(dy) else "vertical"
        if self.drag_direction == "horizontal":
            self.view_at(left - dx * seconds_per_pixel)
        else:
            shift = -dy * (high - low) / height
            self.vertical_limits[name] = low + shift, high + shift
            origin.inaxes.set_ylim(self.vertical_limits[name])
            self.canvas.draw_idle()

    def release(self, _event):
        self.drag = None

    def zoom(self, event):
        ax = event.inaxes
        if ("ctrl" not in event.modifiers or ax not in self.axes
                or event.xdata is None or event.ydata is None or not event.step):
            return
        name = next(name for name, graph in self.graph_axes.items() if graph is ax)
        factor = 0.8 if event.step > 0 else 1.25
        start, end = ax.get_xlim()
        width = max(0.02, min(WINDOW_SECONDS, (end - start) * factor))
        fraction = (event.xdata - start) / (end - start)
        start = max(0, min(WINDOW_SECONDS - width, event.xdata - self.left - width * fraction))
        low, high = ax.get_ylim()
        low = event.ydata + (low - event.ydata) * factor
        high = event.ydata + (high - event.ydata) * factor
        if math.isfinite(high - low) and high - low >= 1e-9:
            first_zoom = name not in self.zoom_limits
            self.zoom_limits[name] = (start, start + width)
            self.vertical_limits[name] = low, high
            if first_zoom:
                self.resize_plot()
            self.draw()

    def close(self):
        self.cancel.set()
        self.wanted = None
        if draw := self.canvas._idle_draw_id:
            self.canvas.get_tk_widget().after_cancel(draw)
            self.canvas._idle_draw_id = None
