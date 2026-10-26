#!/usr/bin/env python3
"""
fsr_calibration.py — builds Vout→resistance and Vout→force curves from the
calibration_<grams>g.csv runs in ../calibration_csv/.

Each calibration run is a log_fsr.py capture: the sensor sits unloaded, a known
mass is placed on it, and the log runs for ~5 minutes while the reading creeps
upward (FSRs creep under sustained load — that is physics, not noise). This
script therefore does NOT average a whole file. Per run it:

  1. finds the loaded segment (first sample above V_LOAD_THRESHOLD),
  2. takes the median of the last SETTLE_WINDOW_S as the steady-state value,
  3. records the creep from the first 5 s to that steady value.

It then fits mass against conductance, since an FSR's conductance (1/R) is
roughly linear in applied force — that linearisation is what makes a straight-
line fit meaningful at all. With the divider in fsr.ino
(5V --[FSR]--+--[RM]-- GND, tap at A0) conductance is proportional to
V/(VCC−V), so the fitted model collapses to a closed form in Vout alone:

    grams ≈ A * ( Vout / (VCC − Vout) ) + B

Three panels are produced:
  • Vout → resistance — the divider's analytic curve with the measured runs on it
  • Vout → grams      — the calibration points and the fitted curve
  • creep             — each run's voltage vs time-under-load

Usage:
  python fsr_calibration.py
  python fsr_calibration.py --out fsr_calibration.png
  python fsr_calibration.py --exclude 647            # drop a suspect run
  python fsr_calibration.py --csv fit_points.csv     # write the summary table
"""

import argparse
import csv
import glob
import os
import re
import statistics
import sys

import matplotlib.pyplot as plt

# Windows consoles default to cp1252 and choke on the arrows/units below.
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

# ── Divider constants — must match fsr.ino / planting_arduino.ino ────────────
RM   = 5000.0   # measuring resistor, ohms
VCC  = 5.0

V_LOAD_THRESHOLD = 0.05   # volts — above this, a mass is considered present
SETTLE_WINDOW_S  = 30.0   # take the steady-state median over the last 30 s
ONSET_WINDOW_S   = 5.0    # "just-applied" value, for the creep figure
SAMPLE_INTERVAL_S = 0.1   # fsr.ino LOG_INTERVAL_MS, used only for 2-column logs

# ROYGBIV, assigned in ascending order of mass — one colour per calibration run.
RAMP = ['#ff0000', '#ffa500', '#ffff00', '#008000', '#0000ff', '#4b0082', '#ee82ee']
INK, MUTED, ACCENT = '#1F2421', '#6B7772', '#B5601F'


def load_run(path):
    """
    Returns (times_s, voltages) for one calibration log.

    Handles both the 3-column format (time_ms,voltage_V,resistance_ohm) and the
    2-column format (voltage_V,resistance_ohm) that plot_fsr.py also tolerates —
    calibration_510g.csv is the 2-column case, despite its 3-column header.
    """
    with open(path, newline='') as f:
        rows = [r for r in csv.reader(f) if r]

    times, volts = [], []
    for i, row in enumerate(rows[1:]):        # row 0 is fsr.ino's header
        try:
            if len(row) >= 3:
                t, v = float(row[0]) / 1000.0, float(row[1])
            elif len(row) == 2:
                v, t = float(row[0]), i * SAMPLE_INTERVAL_S
            else:
                continue
        except ValueError:
            continue                           # header fragment or partial line
        times.append(t)
        volts.append(v)
    return times, volts


def summarise(path, grams):
    """Reduce one run to a single calibration point plus its creep figures."""
    times, volts = load_run(path)
    loaded = [(t, v) for t, v in zip(times, volts) if v > V_LOAD_THRESHOLD]
    if not loaded:
        return None

    t_on, t_end = loaded[0][0], loaded[-1][0]
    onset = statistics.median(
        [v for t, v in loaded if t <= t_on + ONSET_WINDOW_S]) or 0.0
    steady = statistics.median(
        [v for t, v in loaded if t >= t_end - SETTLE_WINDOW_S])

    return {
        'path':   os.path.basename(path),
        'grams':  grams,
        'v':      steady,
        'r':      RM * (VCC / steady - 1.0),
        'onset':  onset,
        'creep':  100.0 * (steady - onset) / onset if onset else float('nan'),
        'hold_s': t_end - t_on,
        # creep trace, re-zeroed to the moment the load landed
        'trace':  [(t - t_on, v) for t, v in loaded],
    }


def fit_linear(xs, ys):
    """Least-squares y = a*x + b, returning (a, b, r_squared)."""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    a = sxy / sxx
    b = my - a * mx
    ss_res = sum((y - (a * x + b)) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum((y - my) ** 2 for y in ys)
    return a, b, (1 - ss_res / ss_tot if ss_tot else float('nan'))


def conductance_x(v):
    """V/(VCC−V) — proportional to FSR conductance through this divider."""
    return v / (VCC - v)


def fit_saturation(runs):
    """
    Fit the datasheet's force-vs-Vout form to the calibration points.

    Interlink's "VOUT vs force, one curve per RM" figure is the divider driven
    by an FSR whose conductance is linear in force (1/R = k*F):

        V(F) = VCC * RM*k*F / (1 + RM*k*F)  =  VCC * F / (F + F0)

    so the whole family collapses to a single parameter F0 = 1/(RM*k), the
    force at which Vout reaches half of VCC. Rearranged, F = F0 * V/(VCC−V),
    which is a straight line through the origin — fit F0 by least squares on
    that form, then report R² against the measured voltages.
    """
    fs = [float(r['grams']) for r in runs]

    # Fit F0 in VOLTAGE space — that is where the residual we care about lives.
    # Solving the linear form F = F0*V/(VCC−V) instead minimises force error and
    # can return a curve that fits the voltages worse than a flat line does.
    # It is one bounded parameter, so a ternary search is exact enough and keeps
    # this script dependency-free.
    def sse(f0):
        return sum((r['v'] - VCC * f / (f + f0)) ** 2 for r, f in zip(runs, fs))

    lo, hi = 1.0, 20000.0
    for _ in range(200):
        a, b = lo + (hi - lo) / 3, hi - (hi - lo) / 3
        if sse(a) < sse(b):
            hi = b
        else:
            lo = a
    f0 = (lo + hi) / 2

    pred = [VCC * f / (f + f0) for f in fs]
    mv = sum(r['v'] for r in runs) / len(runs)
    ss_res = sum((r['v'] - p) ** 2 for r, p in zip(runs, pred))
    ss_tot = sum((r['v'] - mv) ** 2 for r in runs)
    return f0, (1 - ss_res / ss_tot if ss_tot else float('nan'))


# Conductance per gram of the "typical" sensor in Interlink's figure, read off
# its published curves: every RM trace there satisfies V/(VCC−V) = RM*k*F with
# k ≈ 2.22e-7 S/g (checked against the 3k and 10k curves at 1000 g). Used only
# to draw a reference curve for our RM, never for our own fit.
K_TYPICAL = 2.22e-7


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    # The logs live one level up in calibration_csv/; this script sits in plotting/.
    default_dir = os.path.normpath(os.path.join(here, '..', 'calibration_csv'))
    p = argparse.ArgumentParser(
        description='Build Vout→resistance and Vout→force curves from calibration runs.')
    p.add_argument('--dir', default=default_dir,
                   help='Folder holding calibration_<grams>g.csv '
                        '(default ../calibration_csv)')
    p.add_argument('--exclude', type=int, nargs='*', default=[],
                   help='Mass values (grams) to drop from the fit, e.g. --exclude 647')
    p.add_argument('--out', help='Save the figure here instead of showing it')
    p.add_argument('--csv', help='Write the per-run summary table to this CSV')
    args = p.parse_args()

    paths = sorted(glob.glob(os.path.join(args.dir, 'calibration_*g.csv')),
                   key=lambda s: int(re.search(r'_(\d+)g', s).group(1)))
    if not paths:
        print(f'No calibration_*g.csv found in {args.dir}', file=sys.stderr)
        sys.exit(1)

    runs = []
    for path in paths:
        grams = int(re.search(r'_(\d+)g', path).group(1))
        rec = summarise(path, grams)
        if rec is None:
            print(f'  skipping {os.path.basename(path)} — no loaded samples')
            continue
        runs.append(rec)

    print(f'{"mass":>6} {"V_steady":>9} {"R_ohm":>8} {"creep":>7} {"hold":>6}  file')
    for r in runs:
        print(f'{r["grams"]:5d}g {r["v"]:9.3f} {r["r"]:8.0f} '
              f'{r["creep"]:+6.1f}% {r["hold_s"]:5.0f}s  {r["path"]}')

    fit_runs = [r for r in runs if r['grams'] not in args.exclude]
    if len(fit_runs) < 3:
        print('Need at least 3 runs to fit.', file=sys.stderr)
        sys.exit(1)

    xs = [conductance_x(r['v']) for r in fit_runs]
    ys = [float(r['grams']) for r in fit_runs]
    A, B, r2 = fit_linear(xs, ys)

    print(f'\nFit over {len(fit_runs)} runs'
          + (f' (excluded {args.exclude})' if args.exclude else ''))
    print(f'  grams = {A:.1f} * (V/({VCC:.0f}-V)) + {B:.1f}')
    print(f'  R^2   = {r2:.3f}')

    # Datasheet form: V = VCC*F/(F+F0). One parameter, and it is the curve the
    # Interlink "VOUT vs force per RM" figure plots.
    F0, r2_sat = fit_saturation(fit_runs)
    print(f'  V     = {VCC:.0f} * F/(F + {F0:.0f})   (F in g)   R^2 = {r2_sat:.3f}')
    print(f'          half-scale at {F0:.0f} g; implied k = {1.0/(RM*F0):.3e} S/g '
          f'(datasheet typical {K_TYPICAL:.2e})')

    resid = [(r['grams'], A * conductance_x(r['v']) + B - r['grams']) for r in fit_runs]
    print('  residuals (fit - actual):')
    for g, d in resid:
        print(f'    {g:5d}g  {d:+7.0f} g')

    if r2 < 0.9:
        print('\n  WARNING: R^2 below 0.9 — this fit is not trustworthy as a '
              'force calibration. See the Vout-to-grams panel: the points are not '
              'monotonic in mass, which an FSR only does when the contact area '
              'or placement changed between runs.')

    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['grams', 'v_steady', 'r_ohm', 'creep_pct', 'hold_s', 'file'])
            for r in runs:
                w.writerow([r['grams'], f'{r["v"]:.3f}', f'{r["r"]:.0f}',
                            f'{r["creep"]:.1f}', f'{r["hold_s"]:.0f}', r['path']])
        print(f'\nSummary table → {args.csv}')

    # ── Figure ───────────────────────────────────────────────────────────────
    fig, (ax_r, ax_f, ax_c) = plt.subplots(1, 3, figsize=(15.5, 4.8))
    colour = {r['grams']: RAMP[min(i, len(RAMP) - 1)]
              for i, r in enumerate(sorted(runs, key=lambda r: r['grams']))}

    # Panel 1 — Vout → resistance. Analytic, not fitted: the divider fixes it.
    vv = [0.05 + i * 0.01 for i in range(int((VCC - 0.15) / 0.01))]
    ax_r.plot(vv, [RM * (VCC / v - 1.0) for v in vv],
              color=MUTED, linewidth=2, zorder=1,
              label=f'R = {RM:.0f}·({VCC:.0f}/V − 1)')
    # The loaded runs all land within ~0.4 V of each other, so fan the labels
    # out vertically instead of stacking them on top of one another.
    for i, r in enumerate(sorted(runs, key=lambda r: r['v'])):
        ax_r.plot(r['v'], r['r'], 'o', markersize=9, color=colour[r['grams']],
                  markeredgecolor='white', markeredgewidth=2, zorder=3)
        ax_r.annotate(f'{r["grams"]}g', (r['v'], r['r']), textcoords='offset points',
                      xytext=(40, 30 - 13 * i), fontsize=8, color=MUTED,
                      arrowprops=dict(arrowstyle='-', color=MUTED,
                                      linewidth=0.6, shrinkA=0, shrinkB=4))
    ax_r.set_yscale('log')
    ax_r.set_title(f'V$_{{out}}$ → resistance  (RM={RM/1000:.0f}k, exact)',
                   color=INK, fontsize=11)
    ax_r.set_xlabel('V$_{out}$ (V)')
    ax_r.set_ylabel('R$_{FSR}$ (ohm, log)')
    ax_r.legend(frameon=False, fontsize=8.5, loc='upper right')

    # Panel 2 — force → Vout, on the same axes as Interlink's datasheet figure
    # (force in grams 0–1000, Vout 0–VCC) so the two can be compared directly.
    curve_g = [i * 1000.0 / 400 for i in range(401)]
    f0_typ = 1.0 / (RM * K_TYPICAL)   # the datasheet curve in the same F0 form
    ax_f.plot(curve_g, [VCC * g / (g + F0) for g in curve_g],
              color=ACCENT, linewidth=2, zorder=2,
              label=f'fit:  V = {VCC:.0f}·F/(F + {F0:.0f})   R² = {r2_sat:.2f}')
    ax_f.plot(curve_g, [VCC * g / (g + f0_typ) for g in curve_g],
              color=MUTED, linewidth=1.6, linestyle='--', zorder=1,
              label=f'datasheet typical:  V = {VCC:.0f}·F/(F + {f0_typ:.0f})')
    for r in runs:
        excluded = r['grams'] in args.exclude
        ax_f.plot(r['grams'], r['v'], 'X' if excluded else 'o', markersize=9,
                  color=colour[r['grams']], markeredgecolor='white',
                  markeredgewidth=2, zorder=3)
    ax_f.set_xlim(0, 1000)
    ax_f.set_ylim(0, VCC)
    ax_f.set_title('Force → V$_{out}$  (datasheet axes)', color=INK, fontsize=11)
    ax_f.set_xlabel('Force (g)')
    ax_f.set_ylabel('V$_{out}$ (V)')
    ax_f.legend(frameon=False, fontsize=8.5, loc='lower right')

    # Panel 3 — creep. This is why a single Vout does not imply a single force.
    for r in sorted(runs, key=lambda r: r['grams']):
        t = [pt[0] for pt in r['trace']]
        v = [pt[1] for pt in r['trace']]
        ax_c.plot(t, v, color=colour[r['grams']], linewidth=1.6,
                  label=f'{r["grams"]}g')
    ax_c.set_title('Creep — V$_{out}$ vs time under load', color=INK, fontsize=11)
    ax_c.set_xlabel('Time since load applied (s)')
    ax_c.set_ylabel('V$_{out}$ (V)')
    ax_c.legend(frameon=False, fontsize=8, ncol=2, loc='lower right')

    for ax in (ax_r, ax_f, ax_c):
        ax.grid(True, alpha=0.25, linewidth=0.7)
        ax.set_axisbelow(True)
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)
        for side in ('left', 'bottom'):
            ax.spines[side].set_color(MUTED)
        ax.tick_params(colors=MUTED, labelsize=9)

    fig.tight_layout()
    if args.out:
        fig.savefig(args.out, dpi=150)
        print(f'Figure → {args.out}')
    else:
        plt.show()


if __name__ == '__main__':
    main()
