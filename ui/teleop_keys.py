"""Window-local keyboard input for Bench Teleop."""

from motor.live import TELEOP_DOUBLE_TAP_S


class TeleopKeys:
    tag = "BenchTeleop"
    arrows = {"Up": "forward", "Down": "reverse"}

    def __init__(self, root, active, ready, send, stop):
        self.root = root
        self.active = active
        self.ready = ready
        self.send = send
        self.stop = stop
        self.pending_focus = None
        self.reset()
        for key in (*self.arrows, "space"):
            root.bind_class(self.tag, f"<KeyPress-{key}>", self.press)
            root.bind_class(self.tag, f"<KeyRelease-{key}>", self.release)
        root.bind("<FocusOut>", self.focus_out, add="+")
        root.bind("<Unmap>", self.unmap, add="+")

    def prepare(self):
        self.reset()
        # Handle motion keys before a focused widget's default action, such as button activation.
        widgets = [self.root]
        while widgets:
            widget = widgets.pop()
            tags = widget.bindtags()
            if self.tag not in tags:
                widget.bindtags((self.tag, *tags))
            widgets.extend(widget.winfo_children())

    def reset(self):
        self.keys = {}
        self.blocked = set()
        self.first_space = None
        if self.pending_focus is not None:
            self.root.after_cancel(self.pending_focus)
            self.pending_focus = None

    def press(self, event):
        if not self.active():
            return
        key = event.keysym
        # X11 can report a release/press pair with the same timestamp for an auto-repeat.
        repeat = key in self.keys and self.keys[key] in (None, event.time)
        self.keys[key] = None
        if not self.ready():
            self.blocked.add(key)
        else:
            if not repeat:
                self.blocked.discard(key)
            if key not in self.blocked:
                if key in self.arrows:
                    if event.state & 0x000d == 0:  # Leave modified arrows out of the motion channel.
                        self.send(self.arrows[key])
                elif not repeat and event.state & 0x000d == 0:
                    if (self.first_space is not None
                            and ((event.time - self.first_space) & 0xffffffff) <= 1000 * TELEOP_DOUBLE_TAP_S):
                        self.first_space = None
                        self.send("brake")
                    else:
                        self.first_space = event.time
        return "break"

    def release(self, event):
        if self.active():
            self.keys[event.keysym] = event.time
            return "break"

    def focus_out(self, _event):
        if self.active() and self.pending_focus is None:
            self.pending_focus = self.root.after_idle(self.check_focus)

    def check_focus(self):
        self.pending_focus = None
        focus = self.root.focus_displayof()
        if self.active() and (focus is None or focus.winfo_toplevel() != self.root):
            self.stop()

    def unmap(self, event):
        if event.widget == self.root and self.active():
            self.stop()
