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

Obstacle abort (mirrors State.DIGGING_OBSTACLE of planting_fsm.py)
───────────────────────────────────────────────────────────────────
During DRILLING_DOWN and DRILLING_DWELL — and only those two — two signals
abort the drill:

  • FSR holds at or above --fsr-abort-v for --fsr-abort-s seconds
  • the actuator stops making progress for --stall-s seconds while commanded
    to move (DRILLING_DOWN only; during the dwell it is meant to sit still)

Either one sends the run down the recovery path: stop the actuator, note the
position it actually reached, drive it back to exactly where the drill
started — never further, which would grind it into its own end stop — with
the auger still spinning so it cannot seize in the hole, stop the auger once
clear, and finish in ABORTED_OBSTACLE instead of COMPLETE. Nothing is planted.

Pass --no-abort for a baseline run with both detectors off (the pre-October
2026 behaviour), e.g. when deliberately recording what a hard strike looks
like without the script cutting the run short.

Throughout, a background thread polls the Arduino with "fsr,read" and writes
every reply to a CSV together with the state that was active at that moment:

  elapsed_s,state,voltage_V,resistance_ohm

so an aborted run is distinguishable in the log by its DIGGING_OBSTACLE and
ABORTED_OBSTACLE rows.

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
  python3 planting_fsr_test.py --port /dev/ttyACM0 --fsr-abort-v 3.5   # trip sooner
  python3 planting_fsr_test.py --port /dev/ttyACM0 --no-abort          # detectors off

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
PORT        = '/dev/ttyACM1'  # Arduino running planting_arduino.ino  (--port)
BAUD        = 115200          # must match its Serial.begin()         (--baud)
CAN_CHANNEL = 'can1'          # SocketCAN iface the LINAKs are on (--can-channel)
CAN_BITRATE = 125000          # LINAK bus bitrate                  (--bitrate)
NODE_ID     = 0x20            # LINAK 1 (auger) CANopen node id    (--node-id)
HB_PERIOD_MS = 100            # master heartbeat period              (--hb-ms)

# ── Motion ─────────────────────────────────────────────────────────────────
AUGER_RPM   = 75      # BLDC rated speed                              (--rpm)
DRILL_CM    = 28.0    # extend distance, driven at SLOW (0x64)   (--drill-cm)
RETRACT_CM  = 3.0    # retract distance, driven at FAST (0xCD) (--retract-cm)
DWELL_S     = 5.0     # auger spins in the soil at depth             (--dwell)
TAIL_S      = 2.0     # keep logging after the sequence ends          (--tail)

# ── Logging ────────────────────────────────────────────────────────────────
FSR_HZ      = 10.0    # how often to poll the Arduino for force     (--fsr-hz)

# ── Obstacle abort (same numbers as planting_fsm.py) ───────────────────────
# UNCALIBRATED: the 12 initial field trials peaked at 3.559 V on workable
# soil, so 3.7 V clears every drill ever recorded, and the 2 s hold covers the
# rest of the margin. No rock or wood strike has been logged yet — retune once
# one has. See data_InitialFSRFieldTest/ and ../../planting_controller/WORKFLOW.md.
FSR_ABORT_V   = 3.7   # sustained FSR voltage = obstacle       (--fsr-abort-v)
FSR_ABORT_S   = 2.0   # how long it must stay there            (--fsr-abort-s)
STALL_S       = 1.0   # no position change this long = obstacle    (--stall-s)
STALL_GRACE_S = 1.5   # ignore stalls this soon after a move starts; the
                      # actuator needs a moment to break away  (--stall-grace)
MOVE_TIMEOUT_S = 60.0 # give up on a move that never arrives (--move-timeout)

# ── LINAK PDO codes / geometry (see linak_can_node.py, LINAK manual p.11) ─────
CMD_OUT   = 64257   # run out  (extend / down)
CMD_IN    = 64258   # run in   (retract / up)
CMD_STOP  = 64259
CMD_CLEAR = 64256

POS_OUT_MAX = 64255   # highest value that is not a command code (64256+)
POS_IN_MAX  = 10      # fully retracted, as the actuator reports it

# The actuator reports and accepts position in 0.1 mm units, so 100 counts = 1 cm.
# This is the scale confirmed on the hardware. Note it does NOT mean the usable
# position range is 0…64255: a 30 cm stroke only ever reads up to ~3000.
COUNTS_PER_CM = 100

SPEED_FULL = 0xCD   # ~2.18 cm/s  (FAST)
SPEED_HALF = 0x64   # ~1.09 cm/s  (SLOW)

_TARGET_TOLERANCE = 15   # counts — close-enough window for position
_POS_EPSILON      = 5    # counts — movement smaller than this is "not moving"

HB_COB = 0x701  # master heartbeat COB-ID the actuator's 0x1016 entry expects


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
#  Obstacle watch — the FSR half of State.DIGGING_OBSTACLE
# ============================================================

class ObstacleWatch:
    """
    Trips when the FSR holds at or above `threshold_v` for `seconds`.

    Armed only while the auger is actually in the ground, so a settling spike
    in any other state cannot abort anything — the FSR reader thread keeps
    logging either way, it just stops being consulted.

    Over-threshold *samples* are counted rather than wall-clock time: if a
    reply is dropped we under-count and simply take longer to trip, which is
    the safe direction. A clock would keep running through the silence and
    could fire on two samples two seconds apart.
    """

    def __init__(self, threshold_v: float, seconds: float, fsr_hz: float,
                 enabled: bool = True):
        self.threshold = threshold_v
        self.seconds   = seconds
        self.needed    = max(1, int(round(seconds * fsr_hz)))
        self.enabled   = enabled
        self.reason    = None

        self.tripped = threading.Event()
        self._armed  = threading.Event()
        self._over   = 0
        self._lock   = threading.Lock()

    # ── Arming ────────────────────────────────────────────────────────────

    def arm(self) -> None:
        """Start consulting samples. Clears any previous trip and count."""
        with self._lock:
            self._over = 0
        self.reason = None
        self.tripped.clear()
        self._armed.set()

    def disarm(self) -> None:
        self._armed.clear()
        with self._lock:
            self._over = 0

    @property
    def armed(self) -> bool:
        return self._armed.is_set()

    # ── Sample intake (called from the Arduino reader thread) ─────────────

    def on_sample(self, voltage: float) -> None:
        if not self.enabled or not self._armed.is_set():
            return

        with self._lock:
            if voltage < self.threshold:
                self._over = 0
                return
            self._over += 1
            over = self._over
            if over < self.needed:
                print(f'  FSR {voltage:.3f} V ≥ {self.threshold:.2f} V '
                      f'({over}/{self.needed})')
                return
            self._over = 0

        self.trip(f'FSR {voltage:.3f} V ≥ {self.threshold:.2f} V '
                  f'for {self.seconds:.1f} s')

    def trip(self, reason: str) -> None:
        """Declare an obstacle. Also used by the stall detector."""
        if self.tripped.is_set():
            return
        self.reason = reason
        self.tripped.set()
        print(f'  ⚠  OBSTACLE — {reason}')


# ============================================================
#  Arduino link — bldc commands out, FSR samples in
# ============================================================

class ArduinoLink:
    """
    Serial link to planting_arduino.ino.

    One reader thread consumes every line: "FSR:<v>,<ohm>" replies go to the
    CSV — and to the obstacle watch, when one is armed — while everything else
    (ACK/DONE/ERR/READY) is printed. One poller thread sends "fsr,read" at
    --fsr-hz so the log keeps ticking through every state.
    """

    def __init__(self, port: str, baud: int, log: RunLog, fsr_hz: float,
                 watch: 'ObstacleWatch' = None):
        print(f'Opening {port} @ {baud} baud ...')
        self.ser = serial.Serial(port, baud, timeout=1.0)
        self._log = log
        self._watch = watch
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
            except (serial.SerialException, TypeError, OSError):
                return
            if not raw:
                continue
            line = raw.decode('utf-8', errors='replace').strip()
            if not line:
                continue

            if line.startswith('FSR:'):
                try:
                    v_str, r_str = line[4:].split(',')
                    voltage = float(v_str)
                    self._log.sample(voltage, float(r_str))
                except ValueError:
                    print(f'Malformed FSR reply: {line}')
                else:
                    # Logged first, then judged — a trip must never cost a row.
                    if self._watch is not None:
                        self._watch.on_sample(voltage)
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
        self.node.sdo[0x1016][1].raw = (0x01 << 16) + 500
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
        self.send_command(CMD_STOP)
        time.sleep(1.0)
        self.send_command(CMD_CLEAR)
        time.sleep(1.0)
        print('LINAK ready.')

    def _on_tpdo(self, can_id: int, data: bytes, timestamp: float) -> None:
        if len(data) < 7:
            return
        pos = data[0] | (data[1] << 8)
        cur = data[2]
        status = data[3]
        err = data[4]
        speed = data[5] | (data[6] << 8)
        with self._pos_lock:
            self._pos = pos
            self._tpdo = (timestamp, pos, cur, status, err, speed)

    def position(self):
        with self._pos_lock:
            return self._pos

    def send_command(self, position_code: int, speed: int = SPEED_FULL) -> None:
        """Send RPDO — position code + speed byte."""
        msg = [
            position_code & 0xFF, (position_code >> 8) & 0xFF,
            0xFB, speed & 0xFF, 0xFB, 0xFB,
            0x00, 0x00,
        ]
        self._net.send_message(self.cob_rpdo1, msg)
        self._cmd_time = time.time()
        print(f'  RPDO → pos_code={position_code}  raw={msg}')

    def stop(self) -> None:
        try:
            self.send_command(CMD_STOP)
        except Exception:
            pass

    def monitor(self, duration_s: float, poll_s: float = 0.1) -> None:
        """Print TPDO feedback for duration_s, same format as can_tests.py."""
        last_ts = None
        end = time.time() + duration_s
        while time.time() < end:
            with self._pos_lock:
                sample = self._tpdo if hasattr(self, '_tpdo') else None
            if sample is not None and sample[0] != last_ts:
                last_ts = sample[0]
                ts, pos, cur, status, err, speed = sample
                t = time.time() - getattr(self, '_cmd_time', time.time())
                print(f'    t={t:6.2f}s  pos={pos:5d}  cur={cur:3d}  '
                      f'speed={speed:5d}  status=0x{status:02X}  err=0x{err:02X}')
            time.sleep(poll_s)

    def send_and_wait(self, position_code: int, target: int,
                      timeout_s: float = 60.0) -> int:
        """
        Send a position code via RPDO (like can_tests.py send_actuator_command)
        and poll TPDO until pos reaches target (within _TARGET_TOLERANCE).
        Prints position on every TPDO update. Returns final position.
        """
        extending = (target > (self.position() or 0))
        self.send_command(position_code)

        poll, elapsed = 0.1, 0.0
        last_ts = None
        while elapsed < timeout_s:
            time.sleep(poll)
            elapsed += poll
            with self._pos_lock:
                sample = self._tpdo if hasattr(self, '_tpdo') else None
                pos = self._pos
            # Print every new TPDO
            if sample is not None and sample[0] != last_ts:
                last_ts = sample[0]
                ts, p, cur, status, err, speed = sample
                t = time.time() - self._cmd_time
                print(f'    t={t:6.2f}s  pos={p:5d}  cur={cur:3d}  '
                      f'speed={speed:5d}  status=0x{status:02X}  err=0x{err:02X}')
            if pos is None:
                continue
            if extending and pos >= target - _TARGET_TOLERANCE:
                self.send_command(CMD_STOP)
                print(f'  reached pos={pos}')
                return pos
            if not extending and pos <= target + _TARGET_TOLERANCE:
                self.send_command(CMD_STOP)
                print(f'  reached pos={pos}')
                return pos

        self.send_command(CMD_STOP)
        final = self.position()
        raise TimeoutError(
            f'move timed out after {timeout_s} s (at pos={final})')

    def run_to(self, target: int, speed: int = SPEED_FULL,
               watch: 'ObstacleWatch' = None,
               stall_s: float = None,
               stall_grace_s: float = STALL_GRACE_S,
               stall_is_done: bool = False,
               timeout_s: float = MOVE_TIMEOUT_S,
               poll_s: float = 0.1) -> tuple:
        """
        Abortable version of send_and_wait — the primitive every move in the
        sequence now goes through. Sends `target` as an absolute position
        setpoint and watches TPDO until one of:

          ('DONE',     pos)  target reached within _TARGET_TOLERANCE
          ('STALL',    pos)  position stopped changing short of the target
                             (reported as DONE when stall_is_done, i.e. a
                             retraction whose intended end IS the end stop)
          ('OBSTACLE', pos)  `watch` tripped — the FSR saw a sustained load
          ('TIMEOUT',  pos)  none of the above within timeout_s

        The actuator is always left stopped, whichever way we leave. Pass
        stall_s=None to skip stall detection, which is what the dwell wants:
        there, standing still is the whole point.

        Checking "arrived" before "stalled" matters — once the actuator is
        parked on target its position stops changing, which is indistinguishable
        from being stuck against a rock if you look in the other order.
        """
        # Direction is decided from where we are now, so a position is not
        # optional here — guessing it would get the comparison backwards and
        # the move would "arrive" immediately without going anywhere.
        waited, start = 0.0, self.position()
        while start is None and waited < 3.0:
            time.sleep(poll_s)
            waited += poll_s
            start = self.position()
        if start is None:
            raise RuntimeError(
                'no TPDO position received within 3 s — cannot start move')

        extending = target > start
        self.send_command(target, speed)

        elapsed = 0.0
        last_ts = None
        last_moving = start
        stall_elapsed = 0.0

        while elapsed < timeout_s:
            time.sleep(poll_s)
            elapsed += poll_s

            with self._pos_lock:
                sample = self._tpdo if hasattr(self, '_tpdo') else None
                pos = self._pos

            if sample is not None and sample[0] != last_ts:
                last_ts = sample[0]
                ts, p, cur, status, err, spd = sample
                t = time.time() - self._cmd_time
                print(f'    t={t:6.2f}s  pos={p:5d}  cur={cur:3d}  '
                      f'speed={spd:5d}  status=0x{status:02X}  err=0x{err:02X}')

            # 1. Obstacle (FSR) — checked first so a trip is honoured even if
            #    this poll would also have satisfied the target.
            if watch is not None and watch.tripped.is_set():
                self.send_command(CMD_STOP)
                return 'OBSTACLE', (pos if pos is not None else start)

            if pos is None:
                continue

            # 2. Arrived.
            if (extending and pos >= target - _TARGET_TOLERANCE) or \
               (not extending and pos <= target + _TARGET_TOLERANCE):
                self.send_command(CMD_STOP)
                print(f'  reached pos={pos}')
                return 'DONE', pos

            # 3. Not moving. Only after the grace period — the actuator needs a
            #    moment to break away, and that pause is not a stall.
            if stall_s is not None and elapsed >= stall_grace_s:
                if last_moving is None or abs(pos - last_moving) > _POS_EPSILON:
                    last_moving = pos
                    stall_elapsed = 0.0
                else:
                    stall_elapsed += poll_s
                    if stall_elapsed >= stall_s:
                        self.send_command(CMD_STOP)
                        if stall_is_done:
                            print(f'  end stop reached at pos={pos}')
                            return 'DONE', pos
                        print(f'  stalled at pos={pos} short of target {target}')
                        return 'STALL', pos
            else:
                last_moving = pos

        self.send_command(CMD_STOP)
        final = self.position()
        print(f'  TIMEOUT after {timeout_s} s (at pos={final})')
        return 'TIMEOUT', (final if final is not None else target)

    def wait_still(self, seconds: float, watch: 'ObstacleWatch' = None,
                   poll_s: float = 0.1) -> str:
        """
        Hold for `seconds` (the dwell), returning early as 'OBSTACLE' if the
        watch trips. No stall detection here — the actuator is parked on
        purpose. Returns 'DONE' if the full time elapsed.
        """
        waited = 0.0
        while waited < seconds:
            if watch is not None and watch.tripped.is_set():
                return 'OBSTACLE'
            time.sleep(poll_s)
            waited += poll_s
        return 'DONE'

    # ── Blocking moves (position control like can_tests.py) ──────────

    def home_in_max(self, timeout_s: float = 60.0) -> None:
        """Retract to IN_MAX by sending POS_IN_MAX as position target."""
        self.send_and_wait(POS_IN_MAX, POS_IN_MAX, timeout_s)

    def move_distance(self, direction: int, distance_cm: float,
                      speed: int = 0xCD, timeout_s: float = 60.0,
                      floor_pos: int = None) -> tuple:
        """
        Extend/retract distance_cm from the current TPDO position.
        Sends the computed target position directly (like can_tests.py).
        Returns (start_pos, end_pos).
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
            target = min(start + delta, POS_OUT_MAX)
        else:
            target = max(start - delta, POS_IN_MAX)
            if floor_pos is not None:
                target = max(target, floor_pos)

        print(f'  move {start} → {target} ({distance_cm:.2f} cm)')
        end = self.send_and_wait(target, target, timeout_s)
        return start, end


# ============================================================
#  Test sequence
# ============================================================

def recover_from_obstacle(args, log: RunLog, ard: ArduinoLink,
                          linak: LinakChannel, drill_start: int,
                          stopped_at: int, reason: str) -> None:
    """
    State.DIGGING_OBSTACLE, standalone edition.

    Retract to `drill_start` — exactly where the drill began, no further. The
    ROS FSM expresses this as "retract the distance actually travelled"; here we
    have the start position in raw counts, so we can just drive back to it,
    which is the same thing and cannot overshoot into the end stop however
    short the drill was.

    The auger keeps spinning the whole way out so it does not seize in a hole
    it is still wedged in, and only stops once the actuator is clear.
    """
    log.set_state('DIGGING_OBSTACLE')
    travelled = abs(stopped_at - drill_start)
    print(f'  drill aborted after {travelled} counts '
          f'({travelled / COUNTS_PER_CM:.2f} cm) — {reason}')
    print(f'  retracting to the start position ({drill_start}) at FULL speed, '
          f'auger still spinning')

    ard.send(f'bldc,in,{args.rpm}')      # re-assert, same as AUGER_RETRACT

    # No watch on the way out: the load that tripped it is still there, and
    # retracting is the cure, not something to abort. A stall IS the end of
    # this move — we are heading back to where we started, which after homing
    # is the end stop itself.
    outcome, pos = linak.run_to(
        drill_start,
        speed         = SPEED_FULL,
        stall_s       = args.stall_s if not args.no_abort else None,
        stall_is_done = True,
        timeout_s     = args.move_timeout,
    )
    if outcome != 'DONE':
        print(f'  ⚠  retract ended as {outcome} at pos={pos} — check the arm '
              f'is clear of the hole before the next run')

    ard.send('bldc,stop')                # clear of the hole: safe to stop now
    linak.stop()
    log.set_state('ABORTED_OBSTACLE')
    print('  obstacle abort complete — nothing planted at this site.')
    time.sleep(args.tail)


def run_sequence(args, log: RunLog, ard: ArduinoLink, linak: LinakChannel,
                 watch: ObstacleWatch) -> None:
    if not args.no_home:
        log.set_state('LINAK_HOME')
        linak.home_in_max()

    log.set_state('AUGER_SPIN_UP')
    ard.send(f'bldc,in,{args.rpm}')
    # planting_fsm.py transitions AUGER_SPIN_UP → DRILLING_DOWN immediately,
    # with no settle time; this test does the same.

    # ── DRILLING_DOWN ──────────────────────────────────────────────────────
    log.set_state('DRILLING_DOWN')
    target_counts = int(round(args.drill_cm * COUNTS_PER_CM))
    drill_start = linak.position()
    if drill_start is None:
        raise RuntimeError('no TPDO position received — cannot start the drill')
    print(f'  drilling to {target_counts} counts ({args.drill_cm:.1f} cm) '
          f'at HALF speed from pos={drill_start}')

    # Watch armed for the drill and the dwell, and only those two.
    watch.arm()
    outcome, pos = linak.run_to(
        target_counts,
        speed     = SPEED_HALF,
        watch     = watch,
        stall_s   = None if args.no_abort else args.stall_s,
        timeout_s = args.move_timeout,
    )

    if outcome in ('OBSTACLE', 'STALL'):
        watch.disarm()
        reason = watch.reason if outcome == 'OBSTACLE' else \
            f'actuator stopped making progress for {args.stall_s:.1f} s'
        recover_from_obstacle(args, log, ard, linak, drill_start, pos, reason)
        return
    if outcome == 'TIMEOUT':
        watch.disarm()
        raise TimeoutError(
            f'drill never reached {target_counts} (stopped at {pos})')

    # ── DRILLING_DWELL ─────────────────────────────────────────────────────
    log.set_state('DRILLING_DWELL')
    print(f'  dwelling {args.dwell} s with auger spinning ...')
    if linak.wait_still(args.dwell, watch) == 'OBSTACLE':
        watch.disarm()
        dwell_pos = linak.position()
        recover_from_obstacle(args, log, ard, linak, drill_start,
                              pos if dwell_pos is None else dwell_pos,
                              watch.reason)
        return

    # Out of the ground from here on: the FSR is logged but no longer judged.
    watch.disarm()

    # ── AUGER_RETRACT ──────────────────────────────────────────────────────
    log.set_state('AUGER_RETRACT')
    ard.send(f'bldc,in,{args.rpm}')          # FSM re-asserts the spin command
    print(f'  retracting to {POS_IN_MAX} counts at FULL speed')
    outcome, pos = linak.run_to(
        POS_IN_MAX,
        speed         = SPEED_FULL,
        stall_s       = None if args.no_abort else args.stall_s,
        stall_is_done = True,              # the end stop is where we are going
        timeout_s     = args.move_timeout,
    )
    if outcome != 'DONE':
        print(f'  ⚠  retract ended as {outcome} at pos={pos}')

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
    p.add_argument('--tail', type=float, default=TAIL_S,
                   help=f'Extra logging time after the sequence, s (default {TAIL_S:g})')

    p.add_argument('--fsr-hz', type=float, default=FSR_HZ,
                   help=f'FSR sample rate (default {FSR_HZ:g} Hz)')
    p.add_argument('--out', help='Output CSV (default planting_fsr_<timestamp>.csv)')
    p.add_argument('--no-home', action='store_true',
                   help='Skip the LINAK_HOME retract-to-IN_MAX step')

    p.add_argument('--fsr-abort-v', type=float, default=FSR_ABORT_V,
                   help=f'FSR voltage that counts as an obstacle '
                        f'(default {FSR_ABORT_V:g} V, UNCALIBRATED)')
    p.add_argument('--fsr-abort-s', type=float, default=FSR_ABORT_S,
                   help=f'How long the FSR must hold there before aborting '
                        f'(default {FSR_ABORT_S:g} s)')
    p.add_argument('--stall-s', type=float, default=STALL_S,
                   help=f'No position change for this long while drilling = '
                        f'obstacle (default {STALL_S:g} s)')
    p.add_argument('--stall-grace', type=float, default=STALL_GRACE_S,
                   help=f'Ignore stalls for this long after a move starts '
                        f'(default {STALL_GRACE_S:g} s)')
    p.add_argument('--move-timeout', type=float, default=MOVE_TIMEOUT_S,
                   help=f'Give up on a move that never arrives '
                        f'(default {MOVE_TIMEOUT_S:g} s)')
    p.add_argument('--no-abort', action='store_true',
                   help='Disable both obstacle detectors — drill to the '
                        'commanded depth whatever the force reads. Use for a '
                        'deliberate baseline or hard-strike recording.')
    args = p.parse_args()

    if not os.path.isfile(args.eds):
        print(f'Error: EDS not found at {args.eds} (override with --eds)', file=sys.stderr)
        sys.exit(1)

    out_path = args.out or os.path.join(
        here, f'planting_fsr_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv')

    log = RunLog(out_path)
    print(f'Logging FSR to {out_path}')

    watch = ObstacleWatch(args.fsr_abort_v, args.fsr_abort_s, args.fsr_hz,
                          enabled=not args.no_abort)
    if args.no_abort:
        print('Obstacle abort DISABLED (--no-abort) — FSR is logged, not acted on.')
    else:
        print(f'Obstacle abort: FSR ≥ {args.fsr_abort_v:g} V for '
              f'{args.fsr_abort_s:g} s ({watch.needed} samples), or no '
              f'movement for {args.stall_s:g} s while drilling.')

    ard = ArduinoLink(args.port, args.baud, log, args.fsr_hz, watch)
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
        run_sequence(args, log, ard, linak, watch)
        if watch.tripped.is_set():
            print(f'⚠  Sequence aborted on an obstacle: {watch.reason}')
        else:
            print('✅ Sequence complete.')
    except KeyboardInterrupt:
        print('\nInterrupted — stopping motors.')
        log.set_state('ABORTED')
    except Exception as exc:
        print(f'❌ Error: {exc}')
        log.set_state('FAULT')
    finally:
        # Always leave the hardware stopped, whichever way we got here.
        watch.disarm()          # no late trip from a reply still in flight
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
