#!/usr/bin/env python3
# ============================================================
#  LINAK CAN Node — planting_controller
#
#  Manages both LINAK LA36 actuators over two separate CAN
#  interfaces, because the two actuators run different protocols
#  at different bitrates and CANNOT share a bus.
#
#  LINAK_1  (Auger)  — CANopen  — can0 @ 125 kbps — Node ID 0x20
#  LINAK_2  (Chute)  — J1939    — can1 @ 250 kbps — Addr  0xC8
#
#  Direction mapping (confirmed):
#    DOWN = RUN_OUT  for both actuators
#    UP   = RUN_IN   for both actuators
#
#  Subscribes:  /linak_cmd    (std_msgs/String)
#  Publishes:   /linak_status (std_msgs/String)
#
#  Command format (from FSM):
#    "LINAK,1,DOWN,<speed_pct>,<duration_s>"
#    "LINAK,1,UP,<speed_pct>,<duration_s>"
#    "LINAK,1,STOP"
#    "LINAK,2,DOWN,<speed_pct>,<duration_s>"
#    "LINAK,2,UP,<speed_pct>,<duration_s>"
#    "LINAK,2,STOP"
#
#  Status replies (to FSM):
#    "ACK:LINAK1"      command received
#    "ACK:LINAK2"
#    "DONE:LINAK1"     timed move completed
#    "DONE:LINAK2"
#    "ERR:LINAK1:<reason>"
#    "ERR:LINAK2:<reason>"
#    "READY:LINAK"     both actuators initialised
#
#  Pre-requisites (run before launching this node):
#    sudo ip link set can0 up type can bitrate 125000
#    sudo ip link set can1 up type can bitrate 250000
#
#  Python deps:  python-can, canopen
#    pip install python-can canopen
# ============================================================

import threading
import time

import can
import canopen

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ============================================================
#  LINAK_1  —  CANopen constants  (can0, 125 kbps)
# ============================================================

L1_CHANNEL  = 'can0'
L1_NODE_ID  = 0x20          # 32 decimal
L1_EDS_FILE = 'LINAK-actuator-v3-1.eds'

L1_COB_RPDO1        = 0x200 + L1_NODE_ID   # 0x220  command PDO
L1_HEARTBEAT_COB_ID = 0x701                 # master heartbeat
L1_HEARTBEAT_MS     = 100

# CANopen position codes (shared with J1939 numeric values)
L1_CMD_STOP        = 64259   # 0xFB03
L1_CMD_CLEAR_ERROR = 64256   # 0xFB00
L1_CMD_RUN_OUT     = 64257   # 0xFB01  → DOWN (into soil)
L1_CMD_RUN_IN      = 64258   # 0xFB02  → UP   (retract)


# ============================================================
#  LINAK_2  —  J1939 constants  (can1, 250 kbps)
# ============================================================

L2_CHANNEL        = 'can1'
L2_ACTUATOR_ADDR  = 0xC8     # 200 decimal (default LINAK address)
L2_MASTER_ADDR    = 0x01
L2_PRIORITY       = 6
L2_PF_PROP_A      = 0xEF     # Proprietary A PDU format

# J1939 position codes
L2_CMD_STOP        = 0xFB03  # stop
L2_CMD_CLEAR_ERROR = 0xFB00  # clear error
L2_CMD_RUN_OUT     = 0xFB01  # → DOWN (chute into ground)
L2_CMD_RUN_IN      = 0xFB02  # → UP   (retract)

DEFAULT_BYTE      = 0xFB     # "use driver default" for speed/ramp bytes
L2_RESEND_INTERVAL = 0.1     # seconds — must be < 250 ms (actuator keepalive)


# ============================================================
#  Helper — build J1939 29-bit CAN ID (PDU1 peer-to-peer)
# ============================================================

def _j1939_can_id(priority: int, pf: int, dest: int, src: int) -> int:
    return (priority << 26) | (pf << 16) | (dest << 8) | src


# ============================================================
#  LINAK_1 handler  (CANopen)
# ============================================================

class Linak1Handler:
    """
    Wraps the CANopen network for LINAK_1 (auger actuator).
    Runs a background heartbeat thread to keep the actuator alive.
    All public methods are called from the ROS2 spin thread and
    are protected by an internal lock.
    """

    def __init__(self, logger):
        self._log          = logger
        self._lock         = threading.Lock()
        self._network      = None
        self._node         = None
        self._hb_stop      = threading.Event()
        self._hb_thread    = None

        # Timed move state
        self._move_stop    = threading.Event()
        self._move_thread  = None
        self._done_cb      = None   # called with "DONE:LINAK1" when move ends

    # ── Startup ──────────────────────────────────────────────────────

    def init(self, done_callback):
        """
        Connect to can0, initialise the actuator, start heartbeat.
        done_callback(status_str) is called when a timed move completes.
        """
        self._done_cb = done_callback

        self._log.info('LINAK_1: connecting to can0 @ 125 kbps...')
        self._network = canopen.Network()
        self._network.connect(channel=L1_CHANNEL, bustype='socketcan')

        self._node = canopen.RemoteNode(L1_NODE_ID, L1_EDS_FILE)
        self._network.add_node(self._node)

        # Send one heartbeat before NMT transitions
        self._network.send_message(L1_HEARTBEAT_COB_ID, [0x05])
        time.sleep(0.05)

        # Set actuator to OPERATIONAL
        self._log.info('LINAK_1: setting OPERATIONAL...')
        self._node.nmt.state = 'OPERATIONAL'
        time.sleep(0.1)

        # Mandatory init sequence: STOP → CLEAR_ERROR → STOP
        self._send_raw(L1_CMD_STOP)
        time.sleep(0.5)
        self._send_raw(L1_CMD_CLEAR_ERROR)
        time.sleep(0.5)
        self._send_raw(L1_CMD_STOP)
        time.sleep(0.3)

        # Start background heartbeat thread
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True, name='l1_heartbeat'
        )
        self._hb_thread.start()

        self._log.info('LINAK_1: ready.')

    def shutdown(self):
        self._hb_stop.set()
        self._cancel_move()
        if self._network:
            try:
                self._send_raw(L1_CMD_STOP)
                time.sleep(0.1)
                self._network.disconnect()
            except Exception:
                pass

    # ── Commands ─────────────────────────────────────────────────────

    def move(self, direction: str, duration_s: float):
        """
        Start a timed move. direction = 'DOWN' or 'UP'.
        Calls done_callback("DONE:LINAK1") when duration elapses.
        """
        self._cancel_move()                     # abort any prior move
        code = L1_CMD_RUN_OUT if direction == 'DOWN' else L1_CMD_RUN_IN
        with self._lock:
            self._send_raw(code)
        self._log.info(f'LINAK_1: moving {direction} for {duration_s} s...')

        self._move_stop.clear()
        self._move_thread = threading.Thread(
            target=self._timed_move,
            args=(duration_s,),
            daemon=True,
            name='l1_move'
        )
        self._move_thread.start()

    def stop(self):
        """Immediate stop — cancels any running timed move."""
        self._cancel_move()
        with self._lock:
            self._send_raw(L1_CMD_STOP)
        self._log.info('LINAK_1: stopped.')

    # ── Internal ─────────────────────────────────────────────────────

    def _send_raw(self, position_code: int):
        """Send an 8-byte RPDO command. Call with _lock held (or during init)."""
        msg = [
            position_code & 0xFF,
            (position_code >> 8) & 0xFF,
            0xFB, 0xFB, 0xFB, 0xFB,   # default current, speed, ramps
            0x00, 0x00                  # padding
        ]
        self._network.send_message(L1_COB_RPDO1, msg)

    def _heartbeat_loop(self):
        """Send CANopen master heartbeat every L1_HEARTBEAT_MS ms."""
        while not self._hb_stop.is_set():
            try:
                self._network.send_message(L1_HEARTBEAT_COB_ID, [0x05])
            except Exception as exc:
                self._log.warn(f'LINAK_1 heartbeat error: {exc}')
            self._hb_stop.wait(timeout=L1_HEARTBEAT_MS / 1000.0)

    def _timed_move(self, duration_s: float):
        """Sleep for duration_s then stop and fire done callback."""
        finished = not self._move_stop.wait(timeout=duration_s)
        if finished:                            # normal completion
            with self._lock:
                self._send_raw(L1_CMD_STOP)
            self._log.info('LINAK_1: timed move complete.')
            if self._done_cb:
                self._done_cb('DONE:LINAK1')
        # else: move was cancelled — caller already sent STOP

    def _cancel_move(self):
        """Signal any running timed move thread to exit without firing done_cb."""
        self._move_stop.set()
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=0.5)
        self._move_stop.clear()


# ============================================================
#  LINAK_2 handler  (J1939)
# ============================================================

class Linak2Handler:
    """
    Wraps a python-can Bus for LINAK_2 (chute actuator) using J1939
    Proprietary A framing.

    The actuator requires Proprietary A to be resent at least every
    250 ms ('signal alive').  A background command_loop thread handles
    this automatically.
    """

    def __init__(self, logger):
        self._log          = logger
        self._lock         = threading.Lock()
        self._bus          = None
        self._can_id       = _j1939_can_id(
            L2_PRIORITY, L2_PF_PROP_A, L2_ACTUATOR_ADDR, L2_MASTER_ADDR
        )

        # Current command kept alive by command_loop
        self._current_code = None
        self._cmd_stop_evt = threading.Event()
        self._cmd_thread   = None

        # Timed move
        self._move_stop    = threading.Event()
        self._move_thread  = None
        self._done_cb      = None

    # ── Startup ──────────────────────────────────────────────────────

    def init(self, done_callback):
        self._done_cb = done_callback

        self._log.info('LINAK_2: connecting to can1 @ 250 kbps...')
        self._bus = can.Bus(
            channel=L2_CHANNEL, bustype='socketcan', bitrate=250000
        )

        # Start background command keepalive loop
        self._cmd_thread = threading.Thread(
            target=self._command_loop, daemon=True, name='l2_cmd_loop'
        )
        self._cmd_thread.start()

        # Mandatory init: STOP → CLEAR_ERROR → STOP
        self._set_command(L2_CMD_STOP)
        time.sleep(0.5)
        self._set_command(L2_CMD_CLEAR_ERROR)
        time.sleep(0.5)
        self._set_command(L2_CMD_STOP)
        time.sleep(0.3)

        self._log.info('LINAK_2: ready.')

    def shutdown(self):
        self._cancel_move()
        self._set_command(L2_CMD_STOP)
        time.sleep(0.2)
        self._cmd_stop_evt.set()
        if self._bus:
            try:
                self._bus.shutdown()
            except Exception:
                pass

    # ── Commands ─────────────────────────────────────────────────────

    def move(self, direction: str, duration_s: float):
        self._cancel_move()
        code = L2_CMD_RUN_OUT if direction == 'DOWN' else L2_CMD_RUN_IN
        self._set_command(code)
        self._log.info(f'LINAK_2: moving {direction} for {duration_s} s...')

        self._move_stop.clear()
        self._move_thread = threading.Thread(
            target=self._timed_move,
            args=(duration_s,),
            daemon=True,
            name='l2_move'
        )
        self._move_thread.start()

    def stop(self):
        self._cancel_move()
        self._set_command(L2_CMD_STOP)
        self._log.info('LINAK_2: stopped.')

    # ── Internal ─────────────────────────────────────────────────────

    def _build_payload(self, position_code: int) -> list:
        return [
            position_code & 0xFF,
            (position_code >> 8) & 0xFF,
            DEFAULT_BYTE,   # current limit — use driver default
            DEFAULT_BYTE,   # speed         — use driver default
            DEFAULT_BYTE,   # ramp up       — use driver default
            DEFAULT_BYTE,   # ramp down     — use driver default
            0xFF,           # reserved
            0xFF,           # reserved
        ]

    def _set_command(self, code: int):
        """Update the command the keepalive loop will resend."""
        with self._lock:
            self._current_code = code

    def _command_loop(self):
        """
        Resend current command every L2_RESEND_INTERVAL seconds.
        Must fire at least every 250 ms — the actuator will stop
        if it stops receiving Proprietary A messages.
        """
        while not self._cmd_stop_evt.is_set():
            with self._lock:
                code = self._current_code
            if code is not None:
                try:
                    msg = can.Message(
                        arbitration_id=self._can_id,
                        data=self._build_payload(code),
                        is_extended_id=True
                    )
                    self._bus.send(msg)
                except can.CanError as exc:
                    self._log.warn(f'LINAK_2 CAN send error: {exc}')
            self._cmd_stop_evt.wait(timeout=L2_RESEND_INTERVAL)

    def _timed_move(self, duration_s: float):
        finished = not self._move_stop.wait(timeout=duration_s)
        if finished:
            self._set_command(L2_CMD_STOP)
            self._log.info('LINAK_2: timed move complete.')
            if self._done_cb:
                self._done_cb('DONE:LINAK2')

    def _cancel_move(self):
        self._move_stop.set()
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=0.5)
        self._move_stop.clear()


# ============================================================
#  ROS2 Node
# ============================================================

class LinakCanNode(Node):

    # ── Lifecycle ─────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('linak_can_node')

        # ── Publishers / Subscribers ──────────────────────────────────
        self.status_pub = self.create_publisher(String, '/linak_status', 10)
        self.create_subscription(String, '/linak_cmd', self._on_cmd, 10)

        # ── Instantiate handlers ──────────────────────────────────────
        self._l1 = Linak1Handler(self.get_logger())
        self._l2 = Linak2Handler(self.get_logger())

        # Initialise both actuators in a background thread so we don't
        # block the ROS2 executor during the init sleep sequences.
        init_thread = threading.Thread(
            target=self._init_actuators, daemon=True, name='linak_init'
        )
        init_thread.start()

    def destroy_node(self):
        self._l1.shutdown()
        self._l2.shutdown()
        super().destroy_node()

    # ── Actuator init (runs off spin thread) ──────────────────────────

    def _init_actuators(self):
        try:
            self._l1.init(done_callback=self._publish_status)
            self._l2.init(done_callback=self._publish_status)
            self._publish_status('READY:LINAK')
            self.get_logger().info('Both LINAK actuators initialised.')
        except Exception as exc:
            self.get_logger().fatal(f'LINAK init failed: {exc}')
            self._publish_status(f'ERR:LINAK_INIT:{exc}')

    # ── Command subscriber ────────────────────────────────────────────

    def _on_cmd(self, msg: String):
        """
        Parse /linak_cmd and dispatch to the correct actuator handler.

        Expected formats:
          "LINAK,1,DOWN,<speed_pct>,<duration_s>"
          "LINAK,1,UP,<speed_pct>,<duration_s>"
          "LINAK,1,STOP"
          "LINAK,2,DOWN,<speed_pct>,<duration_s>"
          "LINAK,2,UP,<speed_pct>,<duration_s>"
          "LINAK,2,STOP"

        Note: speed_pct is accepted and parsed but not yet used —
        both actuators currently run at their driver-default speed
        (0xFB).  Wire it into build_proprietary_a / _send_raw if
        you need variable speed later.
        """
        raw = msg.data.strip()
        parts = raw.split(',')

        if len(parts) < 3 or parts[0] != 'LINAK':
            self._publish_status(f'ERR:LINAK_PARSE:{raw}')
            return

        actuator_id = parts[1]   # "1" or "2"
        action      = parts[2]   # "DOWN", "UP", "STOP"

        # Select handler
        if actuator_id == '1':
            handler = self._l1
            label   = 'LINAK1'
        elif actuator_id == '2':
            handler = self._l2
            label   = 'LINAK2'
        else:
            self._publish_status(f'ERR:UNKNOWN_ACTUATOR:{actuator_id}')
            return

        # Dispatch
        try:
            if action in ('DOWN', 'UP'):
                if len(parts) < 5:
                    self._publish_status(f'ERR:{label}:MISSING_PARAMS')
                    return
                duration_s = float(parts[4])
                # speed_pct = float(parts[3])  # available when needed
                handler.move(action, duration_s)
                self._publish_status(f'ACK:{label}')

            elif action == 'STOP':
                handler.stop()
                self._publish_status(f'ACK:{label}')

            else:
                self._publish_status(f'ERR:{label}:UNKNOWN_ACTION:{action}')

        except Exception as exc:
            self.get_logger().error(f'{label} command error: {exc}')
            self._publish_status(f'ERR:{label}:{exc}')

    # ── Status publisher ──────────────────────────────────────────────

    def _publish_status(self, status: str):
        """Thread-safe publish to /linak_status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'→ /linak_status: {status}')


# ============================================================
#  Entry point
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = LinakCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()