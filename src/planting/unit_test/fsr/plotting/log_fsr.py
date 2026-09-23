#!/usr/bin/env python3
"""
log_fsr.py — captures fsr.ino's CSV serial output straight to a .csv file.

fsr.ino streams a CSV header line ("time_ms,voltage_V,resistance_ohm")
followed by one data row every LOG_INTERVAL_MS over USB serial at 9600
baud. This script relays those lines verbatim into a timestamped .csv
file (and echoes them to the console) for plotting later in Excel,
Python, etc. If you just want a live graph instead, use the Arduino
IDE's Serial Plotter (Tools > Serial Plotter) directly — no need for
this script in that case.

Usage:
  python log_fsr.py --list                # show available serial ports
  python log_fsr.py --port COM5
  python log_fsr.py --port COM5 --out my_run.csv

Stop with Ctrl+C — the file is flushed and closed cleanly.
"""

import argparse
import datetime
import sys

import serial
import serial.tools.list_ports


def list_ports() -> None:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for p in ports:
        print(f"  {p.device}  —  {p.description}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Log fsr.ino's CSV serial output to a file.")
    parser.add_argument("--port", help="Serial port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate (default 9600, matches fsr.ino)")
    parser.add_argument("--out", help="Output CSV path (default: fsr_log_<timestamp>.csv)")
    parser.add_argument("--list", action="store_true", help="List available serial ports and exit")
    args = parser.parse_args()

    if args.list:
        list_ports()
        return

    if not args.port:
        print("Error: --port is required (use --list to see available ports).", file=sys.stderr)
        sys.exit(1)

    out_path = args.out or f"fsr_log_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"

    print(f"Opening {args.port} @ {args.baud} baud ...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as exc:
        print(f"Error: cannot open {args.port}: {exc}", file=sys.stderr)
        sys.exit(1)

    print(f"Logging to {out_path} — press Ctrl+C to stop.")
    with open(out_path, "w", newline="") as f:
        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue  # timeout with no data
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                print(line)
                f.write(line + "\n")
                f.flush()
        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            ser.close()

    print(f"Saved to {out_path}")


if __name__ == "__main__":
    main()
