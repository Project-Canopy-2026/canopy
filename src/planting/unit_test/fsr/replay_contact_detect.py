#!/usr/bin/env python3
"""
replay_contact_detect.py — run the ground-contact detector over logged FSR runs.

No hardware, no ROS. Replays the DRILLING_DOWN samples of each
planting_fsr_*.csv through the same trip rule planting_fsm.py uses
(`_check_ground_contact`), so the contact threshold can be tuned at a desk
instead of in a field.

What it answers
───────────────
  1. When did the auger reach soil, in each run?
  2. How much of the stroke was spent in the air getting there? That is the
     term that changes with slope — the planting mechanism hangs off the front
     of the robot, so facing downhill it starts closer to the ground.
  3. With the mechanism lowered by --lower cm, is there enough stroke left to
     dig --depth cm below the surface in EVERY orientation?

(3) is the one that decides whether the fix works. Depth measured from the soil
surface is slope-independent by construction, but it is only reachable if the
actuator still has travel left when it gets there.

Usage
─────
  python3 replay_contact_detect.py                       # defaults
  python3 replay_contact_detect.py --sweep               # threshold sensitivity
  python3 replay_contact_detect.py --lower 5 --depth 10

The trip rule is duplicated here rather than imported, because importing it
would drag in rclpy and a live node. It is five lines; if you change
`fsr_contact_voltage` / `fsr_contact_samples` handling in planting_fsm.py,
change it here too.
"""

import argparse
import csv
import glob
import os
import sys

# Matches linak_can_node.py: SLOW = 0x64 = 1.09 cm/s, and the usable stroke.
SLOW_CM_S = 1.09
STROKE_CM = 30.0

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "data_InitialFSRFieldTest")


def orientation(name: str) -> str:
    """Field runs are tagged in the filename: _up / _dn. 5,6,7 up; 8,9,10 down."""
    if "_up" in name:
        return "UPHILL"
    if "_dn" in name:
        return "DOWNHILL"
    return "flat/?"


def load_descent(path: str):
    """The (elapsed_s, voltage) samples of the DRILLING_DOWN phase only."""
    out = []
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            if row.get("state") != "DRILLING_DOWN":
                continue
            try:
                out.append((float(row["elapsed_s"]), float(row["voltage_V"])))
            except (ValueError, KeyError, TypeError):
                continue
    return out


def detect_contact(samples, threshold: float, needed: int):
    """
    Seconds into the descent at which contact trips, or None.

    Same rule as planting_fsm._check_ground_contact: N consecutive samples at
    or above the threshold, timestamped at the FIRST of them. Consecutive
    matters — run 10 has a stray 0.05 V blip about 13 s before real contact,
    and a single-sample rule fires on it.
    """
    if not samples:
        return None
    t0 = samples[0][0]
    run = 0
    first = None
    for t, v in samples:
        if v > threshold:
            run += 1
            if run == 1:
                first = t
            if run >= needed:
                return first - t0
        else:
            run = 0
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default=DATA_DIR, help="directory of planting_fsr_*.csv")
    ap.add_argument("--voltage", type=float, default=0.15, help="contact threshold (V)")
    ap.add_argument("--samples", type=int, default=3, help="consecutive samples to trip")
    ap.add_argument("--depth", type=float, default=10.0, help="wanted hole depth (cm)")
    ap.add_argument("--lower", type=float, default=5.0,
                    help="how far the mechanism is lowered on its mount (cm)")
    ap.add_argument("--speed", type=float, default=SLOW_CM_S, help="descent speed (cm/s)")
    ap.add_argument("--sweep", action="store_true",
                    help="show detected contact time across a range of thresholds")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.data, "planting_fsr_*.csv")))
    if not files:
        print(f"No planting_fsr_*.csv under {args.data}", file=sys.stderr)
        return 1

    if args.sweep:
        grid = [(0.05, 3), (0.10, 3), (0.15, 3), (0.20, 5), (0.30, 5), (0.50, 5)]
        print("Contact time (s into descent) vs detector tuning\n")
        print(f"{'run':<26}{'orient':<10}" + "".join(f"{v}/{n:<4}" for v, n in grid))
        for path in files:
            name = os.path.basename(path)
            samples = load_descent(path)
            row = f"{name[13:-4]:<26}{orientation(name):<10}"
            for v, n in grid:
                t = detect_contact(samples, v, n)
                row += f"{'--':<9}" if t is None else f"{t:<9.1f}"
            print(row)
        print("\nA column that reads the same as its neighbours means the trip point")
        print("is not delicate. '--' means that run never registered contact.")
        return 0

    print(f"Contact detector: >{args.voltage} V for {args.samples} samples, "
          f"descent {args.speed} cm/s")
    print(f"Mechanism lowered {args.lower:.1f} cm, wanting a {args.depth:.1f} cm hole, "
          f"{STROKE_CM:.0f} cm stroke\n")
    print(f"{'run':<26}{'orient':<10}{'contact':>9}{'air':>8}{'air-':>8}"
          f"{'usable':>8}  verdict")
    print(f"{'':<26}{'':<10}{'(s)':>9}{'(cm)':>8}{'lower':>8}{'(cm)':>8}")

    failures = 0
    checked = 0
    for path in files:
        name = os.path.basename(path)
        samples = load_descent(path)
        t = detect_contact(samples, args.voltage, args.samples)
        label = name[13:-4]
        if t is None:
            print(f"{label:<26}{orientation(name):<10}{'--':>9}{'':>8}{'':>8}{'':>8}"
                  f"  no contact seen in descent")
            continue

        air = t * args.speed            # stroke spent reaching the ground
        air_lowered = air - args.lower  # ... once the mount comes down
        usable = STROKE_CM - air_lowered
        checked += 1
        if usable >= args.depth:
            verdict = f"OK ({usable - args.depth:.1f} cm spare)"
        else:
            verdict = f"SHORT by {args.depth - usable:.1f} cm"
            failures += 1
        print(f"{label:<26}{orientation(name):<10}{t:>9.1f}{air:>8.1f}"
              f"{air_lowered:>8.1f}{usable:>8.1f}  {verdict}")

    print()
    # Bare soil only. Runs 11 and 12 put a pot and a plastic sheet on the
    # ground, so what the auger met first was not the soil surface — including
    # them in a slope comparison hides the very effect being measured.
    def bare(path: str) -> bool:
        n = os.path.basename(path)
        return "_pot" not in n and "_plastic" not in n

    def contacts(want: str):
        out = []
        for f in files:
            if not bare(f) or orientation(os.path.basename(f)) != want:
                continue
            t = detect_contact(load_descent(f), args.voltage, args.samples)
            if t is not None:
                out.append(t)
        return out

    ups, dns = contacts("UPHILL"), contacts("DOWNHILL")
    if ups and dns:
        mu, md = sum(ups) / len(ups), sum(dns) / len(dns)
        print(f"Uphill contact   {mu:.1f} s mean  (n={len(ups)})")
        print(f"Downhill contact {md:.1f} s mean  (n={len(dns)})")
        print(f"Downhill reaches ground {mu - md:.1f} s sooner "
              f"= {(mu - md) * args.speed:.1f} cm less air.")
        print("That gap is the whole bug: at a FIXED extension it becomes "
              "extra depth.\nMeasuring from contact removes it.")

    print(f"\n{checked - failures}/{checked} runs can reach "
          f"{args.depth:.1f} cm with the mount lowered {args.lower:.1f} cm.")
    if failures:
        print("Lower the mechanism further, or ask for a shallower hole.")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
