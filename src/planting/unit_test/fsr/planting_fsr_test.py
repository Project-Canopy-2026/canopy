#!/usr/bin/env python3
"""
planting_fsr_test.py — standalone (no ROS) unit test of the drilling half of
planting_fsm.py, logging FSR ground-reaction force for the whole run.

Sequence (mirrors State.LINAK_HOME → AUGER_RETRACT of planting_fsm.py):

  LINAK_HOME      LINAK 1 retracts to IN_MAX                 (skip with --no-home)
  AUGER_SPIN_UP   bldc,in,<rpm>                              (immediate)
  DRILLING_DOWN   LINAK 1 extends <drill_cm> cm at SLOW      (distance-based, TPDO)
  DRILLING_DWELL  auger keeps spinning in the soil           (<dwell> s)
  AUGER_RETRACT   LINAK 1 retracts <retract_cm> cm at FAST   (auger still spinning)
  DONE            bldc,stop + LINAK STOP

Throughout, a background thread polls the Arduino with "fsr,read" and writes
every reply to a CSV together with the state that was active at that moment:

  elapsed_s,state,voltage_V,resistance_ohm

Hardware assumed (same as the real stack):
  • Arduino running planting_arduino.ino on /dev/ttyACM0 @ 115200
    (planting_arduino.ino, NOT fsr.ino — this test needs the bldc + fsr,read
    command protocol, not the free-running CSV stream)
  • LINAK 1 (auger) on CANopen node 0x20, can1 @ 125 kbps

  sudo ip link set can1 up type can bitrate 125000

Usage:
  python3 planting_fsr_test.py --port /dev/ttyACM0
  python3 planting_fsr_test.py --port /dev/ttyACM0 --drill-cm 12 --dwell 5 --rpm 75
  python3 planting_fsr_test.py --port /dev/ttyACM0 --no-home --out bench_run.csv

Ctrl+C at any point stops the auger and the actuator before exiting.
"""

import argparse
import csv
import datetime
import os
import sys
import threading
import time

import canopen
import serial

# ═══════════════════════════════════════════════════════════════════════════
#  TEST SETTINGS — edit these, or override any of them on the command line.
#  Every one is the default for the matching --flag (see main()).
# ═══════════════════════════════════════════════════════════════════════════

# ── Hardware / connection ──────────────────────────────────────────────────
PORT        = '/dev/ttyACM0'  # Arduino running planting_arduino.ino  (--port)
BAUD        = 115200          # must match its Serial.begin()         (--baud)
CAN_CHANNEL = 'can1'          # SocketCAN iface; Kvaser Leaf often enumerates
                              # as can0 — check `ip link show`  (--can-channel)
CAN_BITRATE = 125000          # LINAK bus bitrate                  (--bitrate)
NODE_ID     = 0x20            # LINAK 1 (auger) CANopen node id    (--node-id)
HB_PERIOD_MS = 100            # master heartbeat period              (--hb-ms)

# ── Motion ─────────────────────────────────────────────────────────────────
AUGER_RPM   = 75      # BLDC rated speed                              (--rpm)
DRILL_CM    = 12.0    # extend distance, driven at SLOW (0x64)   (--drill-cm)
RETRACT_CM  = 12.0    # retract distance, driven at FAST (0xCD) (--retract-cm)
DWELL_S     = 5.0     # auger spins in the soil at depth             (--dwell)
SPIN_UP_S   = 1.0     # settle time after starting the auger       (--spin-up)
TAIL_S      = 2.0     # keep logging after the sequence ends          (--tail)

# ── Logging ────────────────────────────────────────────────────────────────
FSR_HZ      = 10.0    # how often to poll the Arduino for force     (--fsr-hz)

# ── LINAK PDO codes / geometry (see linak_can_node.py, LINAK manual p.11) ─────
CMD_OUT   = 64257   # run out  (extend / down)
CMD_IN    = 64258   # run in   (retract / up)
CMD_STOP  = 64259
CMD_CLEAR = 64256

POS_OUT_MAX = 64255
POS_IN_MAX  = 150
STROKE_CM     = 30.0
COUNTS_PER_CM = (POS_OUT_MAX - POS_IN_MAX) / STROKE_CM   # ≈ 2136.83

SPEED_FULL = 0xCD   # ~2.18 cm/s  (FAST)
SPEED_HALF = 0x64   # ~1.09 cm/s  (SLOW)

_POS_EPSILON      = 5      # counts — "not moving"
_TARGET_TOLERANCE = 3000   # counts ≈ 1.4 cm — overshoot allowance on STOP
_STABLE_COUNT     = 4      # ~1 s of no movement → homing done
_STALL_S          = 1.0    # seconds of no movement before a move is called
                           # stalled (hard stop / jammed auger)
_MOVE_START_S     = 3.0    # grace period for motion to begin before the
                           # command is re-sent — stall detection stays
                           # disarmed until the actuator has actually moved

HB_COB = 0x701  # master heartbeat COB-ID the actuator's 0x1016 entry expects


def _rpdo(code: int, speed: int) -> list:
    """Build an 8-byte RPDO1 payload (LINAK manual p.11)."""
    return [
        code & 0xFF, (code >> 8) & 0xFF,
        0xFB,          # current — default
        speed & 0xFF,  # speed
        0xFB, 0xFB,    # ramp up / ramp down — default
        0x00, 0x00,
    ]


# ============================================================
#  Run log — shared clock + current state, written by the FSR thread
# ============================================================

class RunLog:
    """Timestamped CSV of FSR samples, tagged with the active FSM state."""

    def __init__(self, path: str):
        self.path  = path
        self._t0   = time.time()
        self._lock = threading.Lock()
        self._state = 'INIT'
        self._f = open(path, 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow(['elapsed_s', 'state', 'voltage_V', 'resistance_ohm'])
        self._f.flush()

    def elapsed(self) -> float:
        return time.time() - self._t0

    def set_state(self, state: str) -> None:
        with self._lock:
            self._state = state
        print(f'[{self.elapsed():6.2f}s] STATE → {state}')

    def sample(self, voltage: float, resistance: float) -> None:
        with self._lock:
            state = self._state
            self._w.writerow([f'{self.elapsed():.3f}', state,
                              f'{voltage:.3f}', f'{resistance:.0f}'])
            self._f.flush()

    def close(self) -> None:
        with self._lock:
            self._f.close()


# ============================================================
#  Arduino link — bldc commands out, FSR samples in
# ============================================================

class ArduinoLink:
    """
    Serial link to planting_arduino.ino.

    One reader thread consumes every line: "FSR:<v>,<ohm>" replies go to the
    CSV, everything else (ACK/DONE/ERR/READY) is printed. One poller thread
    sends "fsr,read" at --fsr-hz so the log keeps ticking through every state.
    """

    def __init__(self, port: str, baud: int, log: RunLog, fsr_hz: float):
        print(f'Opening {port} @ {baud} baud ...')
        self.ser = serial.Serial(port, baud, timeout=1.0)
        self._log = log
        self._period = 1.0 / fsr_hz
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()

        threading.Thread(target=self._read_loop, daemon=True,
                         name='arduino_read').start()

    def wait_ready(self, timeout: float = 10.0) -> None:
        """The UNO resets when the port opens — wait for its READY banner."""
        print('Waiting for Arduino READY ...')
        if not self._ready.wait(timeout):
            # Not fatal: the board may have booted before we opened the port.
            print(f'WARNING: no READY within {timeout} s — continuing anyway.')
        # Flush any boot noise so the first fsr,read reply is clean.
        self.ser.reset_input_buffer()

    def send(self, cmd: str) -> None:
        with self._write_lock:
            self.ser.write((cmd.strip() + '\n').encode('utf-8'))
        print(f'[{self._log.elapsed():6.2f}s] → arduino: {cmd}')

    def start_fsr_logging(self) -> None:
        threading.Thread(target=self._fsr_loop, daemon=True,
                         name='fsr_poll').start()

    def stop(self) -> None:
        self._stop.set()
        time.sleep(self._period + 0.1)  # let the poller finish its last read
        try:
            self.ser.close()
        except Exception:
            pass

    # ── Threads ───────────────────────────────────────────────────────────

    def _fsr_loop(self) -> None:
        while not self._stop.is_set():
            try:
                with self._write_lock:
                    self.ser.write(b'fsr,read\n')
            except serial.SerialException as exc:
                print(f'FSR poll write error: {exc}')
                return
            self._stop.wait(self._period)

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                raw = self.ser.readline()
            except serial.SerialException as exc:
                print(f'Serial read error: {exc}')
                return
            if not raw:
                continue
            line = raw.decode('utf-8', errors='replace').strip()
            if not line:
                continue

            if line.startswith('FSR:'):
                try:
                    v_str, r_str = line[4:].split(',')
                    self._log.sample(float(v_str), float(r_str))
                except ValueError:
                    print(f'Malformed FSR reply: {line}')
            elif line == 'READY':
                self._ready.set()
                print('Arduino READY.')
            else:
                print(f'← arduino: {line}')


# ============================================================
#  LINAK channel — trimmed copy of linak_can_node.ActuatorChannel
# ============================================================

class LinakChannel:
    """One CANopen LINAK actuator, driven synchronously (blocking moves)."""

    def __init__(self, network: canopen.Network, node_id: int,
                 eds_file: str, hb_ms: int):
        self.cob_rpdo1 = 0x200 + node_id
        self.cob_tpdo1 = 0x180 + node_id
        self._net   = network
        self._hb_ms = hb_ms
        self.node   = canopen.RemoteNode(node_id, eds_file)
        network.add_node(self.node)

        self._pos = None
        self._pos_lock = threading.Lock()

    # ── Startup (same order as can_tests.py / linak_can_node.py) ──────────

    def initialize(self) -> None:
        # Consumer-heartbeat watchdog at 3× period, armed while the master
        # heartbeat is already flowing (otherwise the node EMCYs and drops
        # to PRE-OPERATIONAL, silently ignoring every RPDO afterwards).
        self.node.sdo[0x1016][1].raw = (0x01 << 16) + self._hb_ms * 3
        time.sleep(0.1)

        rpdo = self.node.rpdo[1]
        rpdo.clear()
        rpdo.add_variable('Actuator Command.Position')
        rpdo.enabled = True
        rpdo.save()
        time.sleep(0.1)

        self._net.subscribe(self.cob_tpdo1, self._on_tpdo)

        print('LINAK → OPERATIONAL ...')
        self.node.nmt.state = 'OPERATIONAL'
        time.sleep(0.2)
        self.send(CMD_STOP)
        time.sleep(1.0)
        self.send(CMD_CLEAR)   # clear faults latched by a previous run
        time.sleep(1.0)
        print('LINAK ready.')

    def _on_tpdo(self, can_id: int, data: bytes, timestamp: float) -> None:
        if len(data) >= 2:
            with self._pos_lock:
                self._pos = data[0] | (data[1] << 8)

    def position(self):
        with self._pos_lock:
            return self._pos

    def _settled_position(self, fallback: int) -> int:
        """Latest TPDO position, falling back to the caller's last good read."""
        pos = self.position()
        return fallback if pos is None else pos

    def send(self, code: int, speed: int = SPEED_HALF) -> None:
        self._net.send_message(self.cob_rpdo1, _rpdo(code, speed))

    def stop(self) -> None:
        try:
            self.send(CMD_STOP)
        except Exception:
            pass

    # ── Blocking moves ────────────────────────────────────────────────────

    def home_in_max(self, timeout_s: float = 60.0) -> None:
        """Retract to IN_MAX; done when TPDO position stops changing."""
        self.send(CMD_IN, SPEED_FULL)
        poll, elapsed = 0.25, 0.0
        stable, last, start = 0, None, None
        moving = False

        while elapsed < timeout_s:
            time.sleep(poll)
            elapsed += poll
            pos = self.position()
            if pos is None:
                continue

            if not moving:
                if start is None:
                    start = pos
                elif abs(pos - start) > _POS_EPSILON:
                    moving = True
                elif elapsed >= 3.0:
                    # Post-CLEAR settling can swallow the first RPDO — resend.
                    print('  no motion yet — re-sending IN')
                    self.send(CMD_IN, SPEED_FULL)
                    start, elapsed = pos, 0.0
                last = pos
                continue

            if last is not None and abs(pos - last) <= _POS_EPSILON:
                stable += 1
            else:
                stable = 0
            last = pos
            if stable >= _STABLE_COUNT:
                self.send(CMD_STOP)
                print(f'  homed at pos={pos}')
                return

        self.send(CMD_STOP)
        raise TimeoutError(f'IN_MAX homing timed out after {timeout_s} s')

    def move_distance(self, direction: int, distance_cm: float,
                      speed: int, timeout_s: float = 60.0,
                      floor_pos: int = None) -> tuple:
        """
        Extend/retract distance_cm from the live TPDO position, stopping at
        the computed target — or earlier if the actuator stalls on a hard stop.

        floor_pos (retract only) clamps the target so the move can never travel
        further in than that position. Pass the position the drill started from
        and a retract can never overshoot back past it, however short the
        drill-down actually got.

        Returns (start_pos, end_pos) so the caller can see how far it really
        travelled — a stalled drill travels less than it was asked to.
        """
        poll = 0.1
        waited, start = 0.0, self.position()
        while start is None and waited < 3.0:
            time.sleep(poll)
            waited += poll
            start = self.position()
        if start is None:
            raise RuntimeError('no TPDO position received — cannot start move')

        delta = int(round(distance_cm * COUNTS_PER_CM))
        if direction == CMD_OUT:
            target  = min(start + delta, POS_OUT_MAX)
            reached = lambda p: p >= target - _TARGET_TOLERANCE
        else:
            target  = max(start - delta, POS_IN_MAX)
            if floor_pos is not None:
                # Never travel further in than the caller's floor, whatever
                # distance was asked for.
                target = max(target, floor_pos)
            reached = lambda p: p <= target + _TARGET_TOLERANCE

        print(f'  move {start} → {target} ({distance_cm:.2f} cm, '
              f'speed=0x{speed:02X})')
        self.send(direction, speed)

        elapsed, last_moving, stalls = 0.0, start, 0
        moving = False   # stall detection stays disarmed until it has moved
        resent = False
        while elapsed < timeout_s:
            time.sleep(poll)
            elapsed += poll
            pos = self.position()
            if pos is None:
                continue

            # Reaching the target always wins, and is checked before anything
            # else: the actuator is stopped here and the move returns
            # immediately, so sitting still afterwards can never be counted as
            # a stall no matter how long it rests.
            if reached(pos):
                self.send(CMD_STOP)
                print(f'  target reached at pos={pos}')
                return start, self._settled_position(pos)

            if abs(pos - last_moving) > _POS_EPSILON:
                last_moving, stalls = pos, 0
                moving = True
            elif moving:
                stalls += 1
            elif elapsed >= _MOVE_START_S:
                # Still hasn't budged. A post-CLEAR/STOP settle can swallow the
                # first RPDO, so re-send once before giving up — same recovery
                # home_in_max() uses. Not a stall: it never started.
                if not resent:
                    print('  no motion yet — re-sending command')
                    self.send(direction, speed)
                    resent, elapsed = True, 0.0
                else:
                    self.send(CMD_STOP)
                    print(f'  never started moving from pos={pos} — aborting')
                    return start, pos

            if stalls >= int(_STALL_S / poll):   # stopped mid-travel → hard stop
                self.send(CMD_STOP)
                print(f'  stalled at pos={pos} (target {target}) — treating as done')
                return start, self._settled_position(pos)

        self.send(CMD_STOP)
        raise TimeoutError(f'distance move timed out after {timeout_s} s')


# ============================================================
#  Test sequence
# ============================================================

def run_sequence(args, log: RunLog, ard: ArduinoLink, linak: LinakChannel) -> None:
    if not args.no_home:
        log.set_state('LINAK_HOME')
        linak.home_in_max()

    log.set_state('AUGER_SPIN_UP')
    ard.send(f'bldc,in,{args.rpm}')
    time.sleep(args.spin_up)

    log.set_state('DRILLING_DOWN')
    drill_start, drill_end = linak.move_distance(CMD_OUT, args.drill_cm, SPEED_HALF)
    travelled_cm = abs(drill_end - drill_start) / COUNTS_PER_CM
    print(f'  drilled {travelled_cm:.2f} cm of {args.drill_cm:.2f} cm commanded')

    log.set_state('DRILLING_DWELL')
    print(f'  dwelling {args.dwell} s with auger spinning ...')
    time.sleep(args.dwell)

    log.set_state('AUGER_RETRACT')
    ard.send(f'bldc,in,{args.rpm}')          # FSM re-asserts the spin command
    # Retract only as far as the drill actually went. If the auger stalled on a
    # rock at 5 cm, retracting the commanded 12 cm would drive the carriage past
    # where it started and into whatever is mounted above the LINAK. floor_pos
    # enforces the same limit on position, in case travel and counts disagree.
    retract_cm = min(args.retract_cm, travelled_cm)
    if retract_cm < args.retract_cm:
        print(f'  LIMITING retract to {retract_cm:.2f} cm '
              f'(drill only travelled that far; {args.retract_cm:.2f} cm requested)')
    linak.move_distance(CMD_IN, retract_cm, SPEED_FULL, floor_pos=drill_start)

    log.set_state('COMPLETE')
    ard.send('bldc,stop')
    linak.stop()
    time.sleep(args.tail)                    # keep logging as force settles


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    default_eds = os.path.join(here, '..', 'can_tests', 'LINAK-actuator-v3-1.eds')

    p = argparse.ArgumentParser(
        description='Auger + LINAK drilling unit test with FSR logging.')
    p.add_argument('--port', default=PORT,
                   help=f'Arduino serial port (default {PORT})')
    p.add_argument('--baud', type=int, default=BAUD,
                   help=f'Arduino baud rate (default {BAUD}, matches planting_arduino.ino)')
    p.add_argument('--can-channel', default=CAN_CHANNEL,
                   help=f'SocketCAN interface (default {CAN_CHANNEL})')
    p.add_argument('--bitrate', type=int, default=CAN_BITRATE,
                   help=f'CAN bitrate (default {CAN_BITRATE})')
    p.add_argument('--node-id', type=lambda s: int(s, 0), default=NODE_ID,
                   help=f'LINAK 1 (auger) CANopen node id (default 0x{NODE_ID:02X})')
    p.add_argument('--eds', default=os.path.normpath(default_eds), help='Path to the LINAK EDS file')
    p.add_argument('--hb-ms', type=int, default=HB_PERIOD_MS,
                   help=f'Master heartbeat period ms (default {HB_PERIOD_MS})')

    p.add_argument('--rpm', type=int, default=AUGER_RPM,
                   help=f'Auger RPM (default {AUGER_RPM})')
    p.add_argument('--drill-cm', type=float, default=DRILL_CM,
                   help=f'Drill-down distance in cm (default {DRILL_CM:g})')
    p.add_argument('--retract-cm', type=float, default=RETRACT_CM,
                   help=f'Retract distance in cm (default {RETRACT_CM:g})')
    p.add_argument('--dwell', type=float, default=DWELL_S,
                   help=f'Dwell in soil, s (default {DWELL_S:g})')
    p.add_argument('--spin-up', type=float, default=SPIN_UP_S,
                   help=f'Settle time after starting the auger, s (default {SPIN_UP_S:g})')
    p.add_argument('--tail', type=float, default=TAIL_S,
                   help=f'Extra logging time after the sequence, s (default {TAIL_S:g})')

    p.add_argument('--fsr-hz', type=float, default=FSR_HZ,
                   help=f'FSR sample rate (default {FSR_HZ:g} Hz)')
    p.add_argument('--out', help='Output CSV (default planting_fsr_<timestamp>.csv)')
    p.add_argument('--no-home', action='store_true',
                   help='Skip the LINAK_HOME retract-to-IN_MAX step')
    args = p.parse_args()

    if not os.path.isfile(args.eds):
        print(f'Error: EDS not found at {args.eds} (override with --eds)', file=sys.stderr)
        sys.exit(1)

    out_path = args.out or os.path.join(
        here, f'planting_fsr_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv')

    log = RunLog(out_path)
    print(f'Logging FSR to {out_path}')

    ard = ArduinoLink(args.port, args.baud, log, args.fsr_hz)
    ard.wait_ready()
    ard.send('stop')          # known-safe starting point: all motors off
    ard.start_fsr_logging()

    print(f"Connecting to '{args.can_channel}' @ {args.bitrate} bps ...")
    network = canopen.Network()
    network.connect(channel=args.can_channel, bustype='socketcan', bitrate=args.bitrate)

    # Heartbeat must already be flowing before the 0x1016 watchdog is armed.
    stop_hb = threading.Event()

    def heartbeat_loop():
        while not stop_hb.is_set():
            try:
                network.send_message(HB_COB, [0x05])   # 0x05 = OPERATIONAL
            except Exception as exc:
                print(f'Heartbeat send error: {exc}')
            time.sleep(args.hb_ms / 1000.0)

    network.send_message(HB_COB, [0x05])
    threading.Thread(target=heartbeat_loop, daemon=True, name='linak_hb').start()
    time.sleep(args.hb_ms / 1000.0 * 3 + 0.05)   # ≥3 heartbeats on the bus

    linak = LinakChannel(network, args.node_id, args.eds, args.hb_ms)

    try:
        linak.initialize()
        run_sequence(args, log, ard, linak)
        print('✅ Sequence complete.')
    except KeyboardInterrupt:
        print('\nInterrupted — stopping motors.')
        log.set_state('ABORTED')
    except Exception as exc:
        print(f'❌ Error: {exc}')
        log.set_state('FAULT')
    finally:
        # Always leave the hardware stopped, whichever way we got here.
        try:
            ard.send('stop')
        except Exception:
            pass
        linak.stop()
        time.sleep(0.3)
        ard.stop()
        stop_hb.set()
        time.sleep(0.15)
        try:
            network.disconnect()
        except Exception:
            pass
        log.close()
        print(f'Saved {out_path}')


if __name__ == '__main__':
    main()
