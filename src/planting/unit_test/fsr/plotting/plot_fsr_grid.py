#!/usr/bin/env python3
"""
plot_fsr_grid.py — grid of FSR voltage-vs-time traces, one subplot per trial,
with the planting_fsr_test.py state machine shaded behind each trace and the
top few percent of each trial's voltage scale called out as a horizontal band.

Reads the CSVs planting_fsr_test.py writes
(elapsed_s,state,voltage_V,resistance_ohm) and orders them by the trial number
in the filename (planting_fsr_<n>_<label>.csv), so the grid reads 1..N.

The band spans [(1 - top_pct/100) * Vmax, Vmax] — by default the top 2% of the
voltage scale, not the top 2% of the samples. Alongside it we report dwell time
(how long the trace sits in the band) and the number of separate times it
re-enters, which is where the auger's loading behaviour actually shows up.

Deliberately NOT smoothed. fsr.ino's readFSRVoltage() already averages 8
analogRead calls per logged sample, so every CSV value is an 8x mean and the
idle-state standard deviation measures ~0.000-0.005 V. The visible ripple in
the loaded region is a correlated mechanical oscillation (sign-reversal rate
0.18-0.39 vs ~0.67 for white noise), i.e. auger/soil chatter worth keeping — a
moving average here would only erode it.

Usage:
  python plot_fsr_grid.py ../data_InitialFSRFieldTest
  python plot_fsr_grid.py ../data_InitialFSRFieldTest --out fsr_1_to_12_voltage.png
  python plot_fsr_grid.py ../data_InitialFSRFieldTest --cols 4 --first 1 --last 12
  python plot_fsr_grid.py ../data_InitialFSRFieldTest --top-pct 5
  python plot_fsr_grid.py ../data_InitialFSRFieldTest --no-band
"""

import argparse
import csv
import re
import statistics
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# States planting_fsr_test.py logs, in the order it runs through them, each with
# the shade drawn behind the trace while that state is active.
STATE_COLORS = {
    "INIT": "#AECDE3",
    "LINAK_HOME": "#F5C08A",
    "DRILLING_DOWN": "#A9D5A2",
    "DRILLING_DWELL": "#F0A6A6",
    "AUGER_RETRACT": "#C7BEE0",
    "COMPLETE": "#C4AEA3",
}
SPAN_ALPHA = 0.35

# The top-of-scale band is drawn over the state shading, so it is a saturated
# colour at low alpha rather than another pastel — it has to read as an overlay.
BAND_COLOR = "#B5601F"
BAND_ALPHA = 0.16

FNAME_RE = re.compile(r"planting_fsr_(\d+)_(.+)\.csv$", re.IGNORECASE)


def top_band(times, voltages, top_pct):
    """Stats for the top `top_pct`% of the voltage scale, i.e. [Vmax*(1-p), Vmax].

    Returns a dict with the band limits plus how long the trace spends inside it
    and how many separate times it re-enters (the ripple crosses in and out).
    """
    v_peak = max(voltages)
    v_lo = v_peak * (1.0 - top_pct / 100.0)
    idx = [i for i, v in enumerate(voltages) if v >= v_lo]

    # Sample interval varies slightly between runs, so measure it rather than assume.
    steps = [b - a for a, b in zip(times, times[1:])]
    dt = statistics.median(steps) if steps else 0.0

    # Count contiguous runs of in-band samples.
    n_spans = 1 + sum(1 for a, b in zip(idx, idx[1:]) if b - a > 1) if idx else 0

    return {
        "v_peak": v_peak,
        "v_lo": v_lo,
        "t_peak": times[voltages.index(v_peak)],
        "n_in": len(idx),
        "dwell_s": len(idx) * dt,
        "t_first": times[idx[0]] if idx else float("nan"),
        "t_last": times[idx[-1]] if idx else float("nan"),
        "n_spans": n_spans,
    }


def load(path):
    """Return (times, voltages, [(state, t_start, t_end), ...]) for one trial CSV."""
    times, voltages, states = [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["elapsed_s"])
                v = float(row["voltage_V"])
            except (TypeError, ValueError, KeyError):
                continue
            times.append(t)
            voltages.append(v)
            states.append((row.get("state") or "").strip())

    # Collapse the per-sample state column into contiguous spans to shade.
    spans = []
    for t, state in zip(times, states):
        if spans and spans[-1][0] == state:
            spans[-1][2] = t
        else:
            spans.append([state, t, t])

    return times, voltages, [tuple(s) for s in spans]


def discover(data_dir, first, last):
    """Trial CSVs in the directory, sorted by trial number, filtered to [first, last]."""
    trials = []
    for path in Path(data_dir).glob("planting_fsr_*.csv"):
        m = FNAME_RE.search(path.name)
        if not m:
            continue
        n = int(m.group(1))
        if first is not None and n < first:
            continue
        if last is not None and n > last:
            continue
        trials.append((n, f"{n}_{m.group(2)}", path))
    return sorted(trials, key=lambda t: t[0])


def main():
    parser = argparse.ArgumentParser(description="Grid-plot FSR voltage vs time for a set of planting trials.")
    parser.add_argument("data_dir", help="Directory holding the planting_fsr_<n>_<label>.csv files")
    parser.add_argument("--out", help="Save the figure here instead of showing it interactively")
    parser.add_argument("--cols", type=int, default=4, help="Subplots per row (default: 4)")
    parser.add_argument("--first", type=int, help="Lowest trial number to include")
    parser.add_argument("--last", type=int, help="Highest trial number to include")
    parser.add_argument(
        "--top-pct",
        type=float,
        default=2.0,
        help="Width of the top-of-scale band as %% of each trial's Vmax (default: 2.0)",
    )
    parser.add_argument(
        "--no-band",
        action="store_true",
        help="Annotate the peak only, without the top-of-scale band",
    )
    args = parser.parse_args()

    trials = discover(args.data_dir, args.first, args.last)
    if not trials:
        print(f"No planting_fsr_*.csv files found in {args.data_dir}", file=sys.stderr)
        sys.exit(1)

    cols = max(1, args.cols)
    rows = -(-len(trials) // cols)
    fig, axes = plt.subplots(rows, cols, figsize=(4.8 * cols, 3.6 * rows), squeeze=False)
    flat = [ax for row in axes for ax in row]

    # Shared y limit so peak heights are comparable across trials at a glance.
    y_max = 0.0
    seen_states = []
    peaks = []

    for ax, (_, label, path) in zip(flat, trials):
        times, voltages, spans = load(path)
        if not times:
            ax.set_title(f"{label} (no data)")
            continue

        for state, t0, t1 in spans:
            if state not in STATE_COLORS:
                continue
            if state not in seen_states:
                seen_states.append(state)
            ax.axvspan(t0, t1, color=STATE_COLORS[state], alpha=SPAN_ALPHA, linewidth=0)

        b = top_band(times, voltages, args.top_pct)
        y_max = max(y_max, b["v_peak"])
        b["label"] = label
        peaks.append(b)

        if not args.no_band:
            ax.axhspan(b["v_lo"], b["v_peak"], color=BAND_COLOR, alpha=BAND_ALPHA, linewidth=0, zorder=2)
            ax.axhline(b["v_lo"], color=BAND_COLOR, linewidth=0.8, linestyle="--", alpha=0.8, zorder=3)

        ax.plot(times, voltages, color="#1A1A1A", linewidth=0.9, zorder=4)
        ax.plot([b["t_peak"]], [b["v_peak"]], marker="v", markersize=5, color=BAND_COLOR, zorder=5)

        if args.no_band:
            note = f"max {b['v_peak']:.2f} V\n@ {b['t_peak']:.1f} s"
        else:
            note = (
                f"top {args.top_pct:g}%: {b['v_lo']:.2f}-{b['v_peak']:.2f} V\n"
                f"dwell {b['dwell_s']:.1f} s  ({b['n_spans']}x)"
            )
        ax.annotate(
            note,
            xy=(b["t_peak"], b["v_peak"]),
            xytext=(4, 6),
            textcoords="offset points",
            fontsize=8,
            color=BAND_COLOR,
            zorder=6,
        )

        ax.set_title(label)
        ax.set_xlabel("elapsed_s")
        ax.set_ylabel("voltage_V")
        ax.grid(True, alpha=0.25)
        ax.margins(x=0)

    for ax in flat:
        ax.set_ylim(0, y_max * 1.18 if y_max else 1)
    for ax in flat[len(trials):]:
        ax.set_visible(False)

    lo, hi = trials[0][0], trials[-1][0]
    fig.suptitle(f"FSR voltage vs time  (planting_fsr_{lo} .. {hi})", fontsize=14)
    ordered = [s for s in STATE_COLORS if s in seen_states]
    fig.legend(
        handles=[Patch(facecolor=STATE_COLORS[s], alpha=SPAN_ALPHA, label=s) for s in ordered],
        loc="upper center",
        bbox_to_anchor=(0.5, 0.965),
        ncol=len(ordered),
        frameon=True,
        fontsize=9,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.93))

    print(f"Top {args.top_pct:g}% of voltage scale per trial:")
    print(
        f"  {'trial':<20}{'Vmax':>7}{'band_lo':>9}{'n_in':>6}"
        f"{'dwell_s':>9}{'t_first':>9}{'t_last':>8}{'spans':>7}"
    )
    for b in peaks:
        print(
            f"  {b['label']:<20}{b['v_peak']:7.3f}{b['v_lo']:9.3f}{b['n_in']:6d}"
            f"{b['dwell_s']:9.2f}{b['t_first']:9.2f}{b['t_last']:8.2f}{b['n_spans']:7d}"
        )

    if args.out:
        fig.savefig(args.out, dpi=150)
        print(f"Saved to {args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
