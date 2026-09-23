#!/usr/bin/env python3
"""
plot_fsr.py — plots fsr.ino / log_fsr.py CSV output: voltage and resistance
vs time, side by side.

Handles both the 3-column format (time_ms,voltage_V,resistance_ohm) and the
2-column format (voltage_V,resistance_ohm) that a log can end up with if
fsr.ino's millis() print was ever commented out — in the 2-column case,
time is reconstructed as row_index * SAMPLE_INTERVAL_MS (must match
fsr.ino's LOG_INTERVAL_MS). Resistance is -1 in the log when the sensor
reads OPEN (no measurable force); those points are skipped in the
resistance plot rather than shown as a fake negative resistance.

Usage:
  python plot_fsr.py my_run.csv
  python plot_fsr.py my_run.csv --out my_run.png
"""

import argparse
import csv
import sys

import matplotlib.pyplot as plt

SAMPLE_INTERVAL_MS = 100  # fsr.ino's LOG_INTERVAL_MS, used only if the log has no time column


def load(path):
    with open(path, newline="") as f:
        rows = [r for r in csv.reader(f) if r]

    data_rows = rows[1:]  # first row is always the header fsr.ino prints in setup()

    times, voltages, resistances = [], [], []
    for i, row in enumerate(data_rows):
        try:
            if len(row) >= 3:
                t_ms, v, r = float(row[0]), float(row[1]), float(row[2])
            elif len(row) == 2:
                v, r = float(row[0]), float(row[1])
                t_ms = i * SAMPLE_INTERVAL_MS
            else:
                continue
        except ValueError:
            continue
        times.append(t_ms / 1000.0)
        voltages.append(v)
        resistances.append(r if r > 0 else float("nan"))  # -1 = OPEN, hide from plot

    return times, voltages, resistances


def main():
    parser = argparse.ArgumentParser(description="Plot fsr.ino CSV log: voltage & resistance vs time.")
    parser.add_argument("csv_path", help="Path to the logged CSV (e.g. my_run.csv)")
    parser.add_argument("--out", help="Save the figure to this path instead of showing it interactively")
    args = parser.parse_args()

    times, voltages, resistances = load(args.csv_path)
    if not times:
        print("No data rows found.", file=sys.stderr)
        sys.exit(1)

    fig, (ax_v, ax_r) = plt.subplots(1, 2, figsize=(11, 4.5))

    ax_v.plot(times, voltages, color="#2F6F5E", linewidth=1.2)
    ax_v.set_title("Voltage vs time")
    ax_v.set_xlabel("Time (s)")
    ax_v.set_ylabel("Voltage (V)")
    ax_v.grid(True, alpha=0.3)

    ax_r.plot(times, resistances, color="#B5601F", linewidth=1.2)
    ax_r.set_title("Resistance vs time")
    ax_r.set_xlabel("Time (s)")
    ax_r.set_ylabel("Resistance (ohm)")
    ax_r.grid(True, alpha=0.3)

    fig.suptitle(args.csv_path)
    fig.tight_layout()

    if args.out:
        fig.savefig(args.out, dpi=150)
        print(f"Saved to {args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
