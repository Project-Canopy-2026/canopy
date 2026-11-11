#!/usr/bin/env python3
"""
Standalone stepper (iCL57-23 ball screw) test over USB serial. No ROS needed.

Works with either Arduino sketch:
  --fw prod        planting_arduino.ino   115200 baud  "stepper,left,<mm>"  -> DONE:STEPPER
  --fw standalone  unit_test/stepper/stepper.ino  9600 baud  "L <mm>"       -> Done.

Usage:
  python3 stepper_test.py left 50            # one move, waits for it to finish
  python3 stepper_test.py right 50 --port /dev/ttyACM0
  python3 stepper_test.py                    # interactive: type "l 50", "r 50", "stop", "q"
  python3 stepper_test.py --fw standalone right 20

Ctrl+C during a move sends a stop.
Stop the ROS serial_bridge first, only one program can hold the port.
"""
import argparse
import sys
import time

import serial

STEPS_PER_MM = 160.0                      # 800 steps/rev / 5 mm lead
STEPPER_RPM = 270
MM_PER_S = STEPPER_RPM * 800 / 60 / STEPS_PER_MM   # ~22.5 mm/s

FW = {
    'prod': {
        'baud': 115200,
        'move': lambda d, mm: f'stepper,{d},{mm}',
        'stop': 'stepper,stop',
        'done': 'DONE:STEPPER',
        'ready': 'READY',
    },
    'standalone': {
        'baud': 9600,
        'move': lambda d, mm: f'{d[0].upper()} {mm}',
        'stop': None,                     # sketch has no stop command
        'done': 'Done.',
        'ready': 'Enter:',
    },
}


def send(ser, line):
    print(f'>> {line}')
    ser.write((line + '\n').encode())


def read_until(ser, token, timeout):
    """Print every line from the Arduino until one contains token. True if found."""
    end = time.time() + timeout
    while time.time() < end:
        line = ser.readline().decode(errors='replace').strip()
        if line:
            print(f'<< {line}')
            if token in line:
                return True
            if line.startswith('ERR:'):
                return False
    return False


def move(ser, fw, direction, mm):
    send(ser, fw['move'](direction, mm))
    timeout = mm / MM_PER_S + 5.0
    try:
        if read_until(ser, fw['done'], timeout):
            print(f'Move done ({direction} {mm} mm).')
        else:
            print(f'No "{fw["done"]}" within {timeout:.1f} s.')
    except KeyboardInterrupt:
        stop(ser, fw)
        raise


def stop(ser, fw):
    if fw['stop']:
        send(ser, fw['stop'])
    else:
        print('Standalone sketch has no stop command; cut power or reset the Arduino.')


def parse_dir(s):
    s = s.lower()
    if s in ('l', 'left'):
        return 'left'
    if s in ('r', 'right'):
        return 'right'
    raise ValueError(f'bad direction "{s}" (use left/right)')


def interactive(ser, fw):
    print('Commands: l <mm> | r <mm> | stop | q')
    while True:
        try:
            parts = input('stepper> ').split()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not parts:
            continue
        cmd = parts[0].lower()
        if cmd in ('q', 'quit', 'exit'):
            return
        if cmd == 'stop':
            stop(ser, fw)
            continue
        try:
            direction = parse_dir(cmd)
            mm = float(parts[1])
            if mm <= 0:
                raise ValueError('distance must be > 0')
        except (ValueError, IndexError) as e:
            print(f'?? {e}' if str(e) else '?? usage: l <mm> | r <mm>')
            continue
        try:
            move(ser, fw, direction, mm)
        except KeyboardInterrupt:
            print('\nInterrupted, stop sent.')


def main():
    ap = argparse.ArgumentParser(description='Move the planting stepper over serial.')
    ap.add_argument('direction', nargs='?', help='left / right (omit for interactive)')
    ap.add_argument('mm', nargs='?', type=float, help='distance in mm')
    ap.add_argument('--port', default='/dev/arduino')
    ap.add_argument('--fw', choices=FW.keys(), default='prod',
                    help='which sketch is flashed (default: prod)')
    ap.add_argument('--baud', type=int, help='override baud rate')
    args = ap.parse_args()

    fw = FW[args.fw]
    baud = args.baud or fw['baud']

    try:
        ser = serial.Serial(args.port, baud, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f'Could not open {args.port}: {e}\n'
                 f'Check `ls /dev/ttyUSB* /dev/ttyACM*` and that serial_bridge is not running.')

    with ser:
        # Opening the port resets the Arduino; wait for its banner.
        print(f'Opened {args.port} @ {baud}, waiting for Arduino...')
        if not read_until(ser, fw['ready'], 4.0):
            print('(no ready banner, continuing anyway)')
        ser.reset_input_buffer()

        if args.direction is None:
            interactive(ser, fw)
            return
        if args.mm is None or args.mm <= 0:
            sys.exit('Give a distance > 0, e.g. `stepper_test.py left 50`')
        try:
            move(ser, fw, parse_dir(args.direction), args.mm)
        except KeyboardInterrupt:
            print('\nInterrupted, stop sent.')
        except ValueError as e:
            sys.exit(str(e))


if __name__ == '__main__':
    main()
