#!/usr/bin/env python3
"""Plot software-in-the-loop runs written by `haul-sim --out DIR`.

    python tools/plot_run.py run results/haul_cycle.csv -o haul_cycle.png
    python tools/plot_run.py readme --results results --out docs/img     # the README figures

The readme command expects the ablation runs too:
    haul-sim --out results scenarios/*.json
    haul-sim --out results/no_lead --cal drive_lag_s=0 scenarios/13_speed_steps_empty.json
    haul-sim --out results/no_grade_ff --cal grade_feedforward=0 scenarios/02_ramp_climb_loaded.json
"""

from __future__ import annotations

import argparse
import csv
import textwrap
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

# Palette: validated categorical slots (fixed order) + neutral chart chrome. Text always uses
# the ink colors, never a series color; a series is identified by legend and direct label.
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]  # blue, orange, aqua
STATUS = {"good": "#0ca30c", "warning": "#fab219", "serious": "#ec835a", "critical": "#d03b3b"}
STATE_FILL = {
    "INIT": ("#e1e0d9", INK),
    "READY": ("#c3c2b7", INK),
    "DRIVING": (STATUS["good"], "#ffffff"),
    "SAFE_STOP": (STATUS["serious"], INK),
    "EMERGENCY_STOP": (STATUS["critical"], "#ffffff"),
    "FAULT": (STATUS["critical"], "#ffffff"),
}
LINE = 1.6

plt.rcParams.update(
    {
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE,
        "axes.edgecolor": AXIS,
        "axes.labelcolor": INK_2,
        "axes.titlecolor": INK,
        "axes.titlesize": 11,
        "axes.titleweight": "bold",
        "axes.titlelocation": "left",
        "axes.labelsize": 9,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": True,
        "axes.axisbelow": True,
        "grid.color": GRID,
        "grid.linewidth": 0.6,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "xtick.labelcolor": INK_2,
        "ytick.labelcolor": INK_2,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "legend.frameon": False,
        "legend.fontsize": 8,
        "legend.labelcolor": INK_2,
        "font.size": 9,
        "font.family": "DejaVu Sans",
    }
)


@dataclass
class Run:
    rows: list[dict[str, str]]

    @classmethod
    def load(cls, path: Path) -> "Run":
        with path.open() as f:
            return cls(list(csv.DictReader(f)))

    def col(self, name: str) -> list[float]:
        return [float(r[name]) for r in self.rows]

    def text(self, name: str) -> list[str]:
        return [r[name] for r in self.rows]

    def window(self, t0: float, t1: float) -> "Run":
        return Run([r for r in self.rows if t0 <= float(r["t"]) <= t1])


def spans(t: list[float], flags: list[bool]) -> list[tuple[float, float]]:
    out, start = [], None
    for ti, f in zip(t, flags):
        if f and start is None:
            start = ti
        elif not f and start is not None:
            out.append((start, ti))
            start = None
    if start is not None:
        out.append((start, t[-1]))
    return out


def state_strip(ax: plt.Axes, run: Run) -> None:
    t, states = run.col("t"), run.text("state")
    t_end = t[-1]
    segments, start = [], 0
    for i in range(1, len(states) + 1):
        if i == len(states) or states[i] != states[start]:
            segments.append((t[start], t[i] if i < len(t) else t_end, states[start]))
            start = i
    for a, b, s in segments:
        fill, ink = STATE_FILL.get(s, (GRID, INK))
        ax.axvspan(a, b, color=fill, lw=0)
        ax.axvline(a, color=SURFACE, lw=1.5)  # 2px surface gap between segments
        if (b - a) / max(t_end, 1e-9) > 0.07:
            ax.text((a + b) / 2, 0.5, s.replace("_", " "), ha="center", va="center", fontsize=7.5, color=ink)
    ax.set_xlim(t[0], t_end)
    ax.set_ylim(0, 1)
    ax.set_yticks([])
    ax.grid(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_visible(False)
    ax.tick_params(axis="x", length=0, labelbottom=False)
    ax.set_ylabel("state", rotation=0, ha="right", va="center")


def end_label(ax: plt.Axes, x: list[float], y: list[float], text: str, dy: float = 0.0) -> None:
    ax.annotate(text, (x[-1], y[-1]), xytext=(4, dy), textcoords="offset points", va="center", fontsize=8, color=INK_2,
                annotation_clip=False)


def shade_fault(ax: plt.Axes, run: Run, fault: str, label: str) -> None:
    t = run.col("t")
    flags = [fault in f.split("|") for f in run.text("faults")]
    for i, (a, b) in enumerate(spans(t, flags)):
        ax.axvspan(a, b, color=STATUS["warning"], alpha=0.22, lw=0)
        if i == 0:
            ax.text(a, 0.03, f" {label}", transform=ax.get_xaxis_transform(), va="bottom", fontsize=7.5, color=INK_2)


def speed_panel(ax: plt.Axes, run: Run, estimate: bool = False) -> None:
    t = run.col("t")
    ax.step(t, run.col("target_kph"), where="post", color=MUTED, lw=1.0, label="target")
    ax.plot(t, run.col("setpoint_kph"), color=INK_2, lw=1.0, ls=(0, (4, 3)), label="setpoint")
    ax.plot(t, run.col("speed_kph"), color=SERIES[0], lw=LINE, label="speed (truth)")
    if estimate:
        ax.plot(t, run.col("estimate_kph"), color=SERIES[1], lw=LINE * 0.8, label="VCU estimate")
    ax.set_ylabel("km/h")
    ax.set_ylim(bottom=0)


def force_panel(ax: plt.Axes, run: Run) -> None:
    t = run.col("t")
    drive = run.col("drive_actual_kn")
    brake = [-b for b in run.col("brake_force_kn")]
    ax.axhline(0, color=AXIS, lw=0.8)
    ax.plot(t, drive, color=SERIES[0], lw=LINE, label="drive force")
    ax.plot(t, brake, color=SERIES[1], lw=LINE, label="service brake")
    ax.set_ylabel("force kN\n(drive +, brake −)")


def grade_panel(ax: plt.Axes, run: Run) -> None:
    t, g = run.col("t"), run.col("grade_pct")
    ax.fill_between(t, g, 0, color=MUTED, alpha=0.35, lw=0)
    ax.plot(t, g, color=MUTED, lw=1.0)
    ax.set_ylabel("grade %")


def layout(n_panels: int, heights: list[float], width: float = 10.0) -> tuple[plt.Figure, list[plt.Axes]]:
    fig, axes = plt.subplots(n_panels, 1, sharex=True, figsize=(width, sum(heights)),
                             gridspec_kw={"height_ratios": heights, "hspace": 0.18})
    return fig, list(axes)


def finish(fig: plt.Figure, axes: list[plt.Axes], title: str, subtitle: str, out: Path) -> None:
    axes[-1].set_xlabel("time (s)")
    # Fixed header and footer heights in inches, so short and tall figures look alike.
    height = fig.get_figheight()
    lines = textwrap.fill(subtitle, 128)
    header = 0.62 + 0.19 * lines.count("\n")
    fig.text(0.085, 1 - 0.12 / height, title, ha="left", va="top", fontsize=12.5, fontweight="bold", color=INK)
    fig.text(0.085, 1 - 0.40 / height, lines, ha="left", va="top", fontsize=9, color=INK_2, linespacing=1.4)
    fig.subplots_adjust(left=0.085, right=0.87, top=1 - (header + 0.12) / height, bottom=0.55 / height)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"wrote {out}")


def plot_run(csv_path: Path, out: Path) -> None:
    run = Run.load(csv_path)
    fig, (s, v, f, g) = layout(4, [0.35, 2.2, 1.7, 0.9])
    state_strip(s, run)
    speed_panel(v, run, estimate=True)
    force_panel(f, run)
    grade_panel(g, run)
    for ax in (v, f):
        ax.legend(loc="upper left", bbox_to_anchor=(1.0, 1.0))
    finish(fig, [s, v, f, g], csv_path.stem.replace("_", " "), "software-in-the-loop run", out)


# ------------------------------------------------------------------------------- README figures

def fig_haul_cycle(results: Path, out: Path) -> None:
    run = Run.load(results / "haul_cycle.csv")
    fig, (s, v, f, g) = layout(4, [0.35, 2.3, 1.8, 0.9])
    state_strip(s, run)
    speed_panel(v, run)
    force_panel(f, run)
    grade_panel(g, run)
    v.legend(loc="upper left", bbox_to_anchor=(1.0, 1.0))
    f.legend(loc="upper left", bbox_to_anchor=(1.0, 1.0))
    finish(fig, [s, v, f, g], "A loaded leg out of the pit, 392 t",
           "Speed zones from fleet management by position: 25 km/h on the pit floor, 15 km/h up an 8 % ramp, "
           "40 km/h on the haul road, stop at the dump. Slowing down uses electric retarding; the service brake "
           "only holds the truck at standstill.", out)


def fig_wheel_slip(results: Path, out: Path) -> None:
    run = Run.load(results / "wheel_slip_wet_road.csv").window(0, 60)
    fig, (s, v, f, sl) = layout(4, [0.35, 2.0, 1.7, 1.0])
    state_strip(s, run)
    speed_panel(v, run, estimate=True)
    shade_fault(v, run, "WheelSlip", "WheelSlip detected")
    force_panel(f, run)
    f.plot(run.col("t"), run.col("drive_request_kn"), color=SERIES[2], lw=1.0, ls=(0, (2, 2)),
           label="drive request")
    shade_fault(f, run, "WheelSlip", "")
    t = run.col("t")
    sl.plot(t, [100 * x for x in run.col("slip")], color=SERIES[0], lw=LINE, label="rear-wheel slip")
    sl.set_ylabel("slip %")
    shade_fault(sl, run, "WheelSlip", "")
    for ax in (v, f, sl):
        ax.legend(loc="upper left", bbox_to_anchor=(1.0, 1.0))
    finish(fig, [s, v, f, sl], "Traction control on a wet patch (grip 0.08), 392 t",
           "The VCU sees the driven rear axle spinning faster than the undriven front axle, cuts drive force until "
           "the wheels grip, then gives it back gradually. The speed estimate (from the front axle, IMU and GNSS) "
           "stays on top of the true speed throughout.", out)


def fig_safe_stop(results: Path, out: Path) -> None:
    run = Run.load(results / "fms_link_loss_safe_stop.csv").window(40, 110)
    fig, (s, v, f) = layout(3, [0.35, 2.0, 1.7])
    state_strip(s, run)
    speed_panel(v, run)
    shade_fault(v, run, "MissionTimeout", "no messages from fleet management")
    force_panel(f, run)
    shade_fault(f, run, "MissionTimeout", "")
    v.legend(loc="upper left", bbox_to_anchor=(1.0, 1.0))
    f.legend(loc="upper left", bbox_to_anchor=(1.0, 1.0))
    finish(fig, [s, v, f], "Network loss: controlled stop, latched fault, operator reset",
           "MissionTimeout after 0.21 s, SAFE_STOP at a jerk-limited 1 m/s², FAULT once stopped, READY again "
           "1 s after the link returns with STOP held.", out)


def fig_ablations(results: Path, out: Path) -> None:
    fig, (a, b) = plt.subplots(1, 2, figsize=(10, 3.6), gridspec_kw={"wspace": 0.28})
    # (a) lag-lead feedforward: the 20 -> 45 km/h step.
    for name, path, color in (("with drive-lag lead (default)", results / "speed_steps_empty.csv", SERIES[0]),
                              ("drive_lag_s = 0", results / "no_lead" / "speed_steps_empty.csv", SERIES[1])):
        r = Run.load(path).window(50, 75)
        t, v = r.col("t"), r.col("speed_kph")
        a.plot(t, v, color=color, lw=LINE, label=f"{name}: peak {max(v) - 45:+.2f} km/h")
    a.axhline(45, color=MUTED, lw=1.0, ls=(0, (4, 3)))
    a.set_ylim(42.0, 45.8)
    a.set_xlabel("time (s)")
    a.set_ylabel("km/h")
    a.set_title("Overshoot after a 20 → 45 km/h step")
    a.legend(loc="lower right")
    # (b) grade feedforward: entering the 10 % ramp at 15 km/h.
    for name, path, color in (("with grade feedforward (default)", results / "ramp_climb_loaded.csv", SERIES[0]),
                              ("grade_feedforward = 0", results / "no_grade_ff" / "ramp_climb_loaded.csv",
                               SERIES[1])):
        r = Run.load(path)
        rows = [(x, v) for x, v in zip(r.col("position_m"), r.col("speed_kph")) if 150 <= x <= 600]
        xs, vs = [x for x, _ in rows], [v for _, v in rows]
        low = min(v for x, v in rows if x >= 250)
        b.plot(xs, vs, color=color, lw=LINE, label=f"{name}: min {low:.1f} km/h")
    b.axhline(15, color=MUTED, lw=1.0, ls=(0, (4, 3)))
    b.axvspan(250, 600, color=MUTED, alpha=0.12, lw=0)
    b.text(595, 0.96, "ramp starts at 250 m (0 → 10 % over 40 m)", transform=b.get_xaxis_transform(), ha="right",
           va="top", fontsize=7.5, color=INK_2)
    b.set_xlabel("position (m)")
    b.set_ylabel("km/h")
    b.set_title("Speed dip entering a 10 % ramp, loaded")
    b.legend(loc="lower left", bbox_to_anchor=(0.0, 0.08))
    fig.subplots_adjust(left=0.07, right=0.98, top=0.88, bottom=0.15)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"wrote {out}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    one = sub.add_parser("run", help="plot one run CSV")
    one.add_argument("csv", type=Path)
    one.add_argument("-o", "--out", type=Path)
    rd = sub.add_parser("readme", help="regenerate the figures in docs/img")
    rd.add_argument("--results", type=Path, default=Path("results"))
    rd.add_argument("--out", type=Path, default=Path("docs/img"))
    args = ap.parse_args()

    if args.cmd == "run":
        plot_run(args.csv, args.out or args.csv.with_suffix(".png"))
    else:
        fig_haul_cycle(args.results, args.out / "haul_cycle.png")
        fig_wheel_slip(args.results, args.out / "wheel_slip.png")
        fig_safe_stop(args.results, args.out / "safe_stop.png")
        fig_ablations(args.results, args.out / "ablations.png")


if __name__ == "__main__":
    main()
