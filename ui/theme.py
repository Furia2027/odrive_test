"""Light and dark colors shared by the Tk controls and telemetry charts."""
from types import SimpleNamespace


LIGHT = SimpleNamespace(
    bg="#eef2f5", panel="#f8fafc", ink="#19283a", muted="#4c5e72", chart="#ffffff", border="#d1dce6",
    field="#ffffff", button="#e5ecf2", selection="#c9dbec", run="#006b68", run_active="#005855",
    stop="#a0212d", stop_active="#861926", danger="#a0212d", warning="#775400",
    lines=("#526177", "#007b79", "#9f5300", "#73469d"),
    states={"Idle": ("#d5f1e6", "#124e3b"), "Active": ("#d8eafb", "#153f64"),
            "Caution": ("#faebbd", "#684800"), "Fault": ("#fadddf", "#8b1d27")},
)
DARK = SimpleNamespace(
    bg="#19212a", panel="#232d38", ink="#eff4fa", muted="#b7c8da", chart="#18222d", border="#475b70",
    field="#1b2530", button="#344457", selection="#425c75", run="#006e69", run_active="#005f5b",
    stop="#a8303e", stop_active="#8e2632", danger="#ff9da8", warning="#ffda75",
    lines=("#b3c3d6", "#62e0ca", "#ffb36a", "#d3a7fa"),
    states={"Idle": ("#173f35", "#9eefd5"), "Active": ("#203e58", "#b6dfff"),
            "Caution": ("#49381b", "#ffda75"), "Fault": ("#512631", "#ffb4bd")},
)


def configure(style, p):
    style.configure(".", background=p.bg, foreground=p.ink, font=("DejaVu Sans", 10))
    style.configure("Panel.TFrame", background=p.panel)
    style.configure("Panel.TLabel", background=p.panel)
    style.configure("Muted.TLabel", background=p.panel, foreground=p.muted)
    style.configure("Title.TLabel", font=("DejaVu Sans", 17, "bold"))
    style.configure("Section.TLabel", background=p.panel, font=("DejaVu Sans", 12, "bold"))
    style.configure("DriveMeta.TLabel", foreground=p.muted)
    for name, (background, ink) in p.states.items():
        style.configure(f"Drive{name}.TLabel", background=background, foreground=ink,
                        font=("DejaVu Sans", 10, "bold"), padding=(7, 3))
    style.configure("Danger.TLabel", foreground=p.danger)
    style.configure("Danger.Panel.TLabel", background=p.panel, foreground=p.danger)
    style.configure("Warning.Panel.TLabel", background=p.panel, foreground=p.warning)
    style.configure("TButton", padding=(9, 7), background=p.button, bordercolor=p.border)
    style.map("TButton", background=[("active", p.selection)], foreground=[("disabled", p.muted)])
    for name, background, active in (("Run", p.run, p.run_active), ("Stop", p.stop, p.stop_active)):
        style.configure(f"{name}.TButton", foreground="white", background=background, bordercolor=background)
        style.map(f"{name}.TButton", background=[("disabled", p.button), ("active", active)],
                  bordercolor=[("disabled", p.border)], foreground=[("disabled", p.muted)])
    for widget in ("TEntry", "TCombobox"):
        style.configure(widget, fieldbackground=p.field, foreground=p.ink, insertcolor=p.ink,
                        selectbackground=p.selection, selectforeground=p.ink,
                        background=p.button, bordercolor=p.border, lightcolor=p.border,
                        darkcolor=p.border, arrowcolor=p.ink)
        style.map(widget, fieldbackground=[("disabled", p.button), ("readonly", p.field)],
                  foreground=[("disabled", p.muted), ("readonly", p.ink)])
    style.configure("TCheckbutton", indicatorbackground=p.field, indicatorforeground=p.ink)
    style.map("TCheckbutton", background=[("active", p.selection)], foreground=[("disabled", p.muted)])
    style.configure("TScrollbar", troughcolor=p.field, background=p.button, bordercolor=p.border,
                    arrowcolor=p.ink, lightcolor=p.border, darkcolor=p.border)
    style.map("TScrollbar", background=[("active", p.selection)])
    style.configure("TMenubutton", background=p.button, bordercolor=p.border, arrowcolor=p.ink)
    style.map("TMenubutton", background=[("active", p.selection)])
