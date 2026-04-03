#!/usr/bin/env python3
# ============================================================
#  LINAK CAN Node — planting_controller
#
#  Manages both LINAK LA36 actuators over two separate CAN
#  interfaces (different protocols, different bitrates — cannot
#  share a bus).
#
#  LINAK_1  (Auger)  — CANopen — can0 @ 125 kbps — Node 0x20
#  LINAK_2  (Chute)  — J1939  — can1 @ 250 kbps — Addr 0xC8
#
#  Direction mapping (confirmed):
#    DOWN = RUN_OUT  for both actuators
#    UP   = RUN_IN   for both actuators
#
#  Subscribes:  /linak_cmd    (std_msgs/String)
#  Publishes:   /linak_status (std_msgs/String)
#
#  Command format:
#    "LINAK,<1|2>,<DOWN|UP>,<speed_pct>,<duration_s>"
#    "LINAK,<1|2>,STOP"
#
#  Status replies:
#    "READY:LINAK"       both actuators initialised
#    "ACK:LINAK<1|2>"    command received
#    "DONE:LINAK<1|2>"   timed move completed
#    "ERR:LINAK<1|2>:?"  fault
#
#  Pre-requisites:
#    sudo ip link set can0 up type can bitrate 125000
#    sudo ip link set can1 up type can bitrate 250000
#
#  Python deps:  python-can  canopen
# ============================================================

import threading
import time

import can
import canopen

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ============================================================
#  Shared position codes  (identical across CANopen and J1939)
# ============================================================

CMD_STOP        = 64259   # 0xFB03
CMD_CLEAR_ERROR = 64256   # 0xFB00
CMD_RUN_OUT     = 64257   # 0xFB01  DOWN (into soil / ground)
CMD_RUN_IN      = 64258   # 0xFB02  UP   (retract)


# ============================================================
#  Base handler  shared move / stop / timing logic
# ============================================================

class LinakHandler:
    """
    Protocol-agnostic base for both actuator handlers.
    Subclasses implement init(), _send_raw(), shutdown().
    move(), stop(), timed move, cancellation all live here.
    """

    label: str = ''   # set by subclass e.g. 'LINAK1'

    def __init__(self, logger):
        self._log       = logger
        self._done_cb   = None
        self._move_stop = threading.Event()
        self._move_thread = None

    def init(self, done_callback):   raise NotImplementedError
    def shutdown(self):              raise NotImplementedError
    def _send_raw(self, code: int): raise NotImplementedError

    def move(self, direction: str, duration_s: float):
        """Start a non-blocking timed move. direction = 'DOWN' or 'UP'."""
        self._cancel_move()
        self._send_raw(CMD_RUN_OUT if direction == 'DOWN' else CMD_RUN_IN)
        self._log.info(f'{self.label}: moving {direction} for {duration_s} s')
        self._move_stop.clear()
        self._move_thread = threading.Thread(
            target=self._timed_move, args=(duration_s,),
            daemon=True, name=f'{self.label}_move'
        )
        self._move_thread.start()

    def stop(self):
        self._cancel_move()
        self._send_raw(CMD_STOP)
        self._log.info(f'{self.label}: stopped.')

    def _timed_move(self, duration_s: float):
        if not self._move_stop.wait(timeout=duration_s):  # False = completed normally
            self._send_raw(CMD_STOP)
            self._log.info(f'{self.label}: timed move complete.')
            if self._done_cb:
                self._done_cb(f'DONE:{self.label}')

    def _cancel_move(self):
        self._move_stop.set()
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=0.5)
        self._move_stop.clear()

    def _init_sequence(self):
        """Mandatory STOP -> CLEAR_ERROR -> STOP before any run command."""
        for code, delay in [(CMD_STOP, 0.5), (CMD_CLEAR_ERROR, 0.5), (CMD_STOP, 0.3)]:
            self._send_raw(code)
            time.sleep(delay)


# ============================================================
#  LINAK_1  CANopen  (can0, 125 kbps, Node ID 0x20)
# ============================================================

class Linak1Handler(LinakHandler):

    label        = 'LINAK1'
    _CHANNEL     = 'can0'
    _NODE_ID     = 0x20
    _EDS_FILE    = 'LINAK-actuator-v3-1.eds'
    _COB_RPDO1   = 0x200 + _NODE_ID   # 0x220
    _HB_COB_ID   = 0x701              # master heartbeat
    _HB_INTERVAL = 0.1                # seconds

    def init(self, done_callback):
        self._done_cb = done_callback
        self._hb_stop = threading.Event()

        self._log.info('LINAK_1: connecting to can0 @ 125 kbps...')
        self._network = canopen.Network()
        self._network.connect(channel=self._CHANNEL, bustype='socketcan')
        self._node = canopen.RemoteNode(self._NODE_ID, self._EDS_FILE)
        self._network.add_node(self._node)

        self._network.send_message(self._HB_COB_ID, [0x05])
        time.sleep(0.05)
        self._node.nmt.state = 'OPERATIONAL'
        time.sleep(0.1)
        self._init_sequence()

        threading.Thread(
            target=self._heartbeat_loop, daemon=True, name='l1_heartbeat'
        ).start()
        self._log.info('LINAK_1: ready.')

    def shutdown(self):
        self._cancel_move()
        self._hb_stop.set()
        if hasattr(self, '_network'):
            try:
                self._send_raw(CMD_STOP)
                time.sleep(0.1)
                self._network.disconnect()
            except Exception:
                pass

    def _send_raw(self, code: int):
        self._network.send_message(self._COB_RPDO1, [
            code & 0xFF, (code >> 8) & 0xFF,
            0xFB, 0xFB, 0xFB, 0xFB, 0x00, 0x00
        ])

    def _heartbeat_loop(self):
        while not self._hb_stop.is_set():
            try:
                self._network.send_message(self._HB_COB_ID, [0x05])
            except Exception as exc:
                self._log.warn(f'LINAK_1 heartbeat error: {exc}')
            self._hb_stop.wait(timeout=self._HB_INTERVAL)


# ============================================================
#  LINAK_2  J1939  (can1, 250 kbps, Address 0xC8)
# ============================================================

class Linak2Handler(LinakHandler):

    label            = 'LINAK2'
    _CHANNEL         = 'can1'
    _ACTUATOR_ADDR   = 0xC8
    _MASTER_ADDR     = 0x01
    _RESEND_INTERVAL = 0.1   # seconds — must stay under 250 ms (keepalive)

    def init(self, done_callback):
        self._done_cb      = done_callback
        self._cmd_stop_evt = threading.Event()
        self._current_code = None
        self._cmd_lock     = threading.Lock()
        # J1939 29-bit CAN ID: Priority=6, PF=0xEF (Proprietary A), Dest, Src
        self._can_id = (6 << 26) | (0xEF << 16) | (self._ACTUATOR_ADDR << 8) | self._MASTER_ADDR

        self._log.info('LINAK_2: connecting to can1 @ 250 kbps...')
        self._bus = can.Bus(channel=self._CHANNEL, bustype='socketcan', bitrate=250000)
        threading.Thread(
            target=self._command_loop, daemon=True, name='l2_cmd_loop'
        ).start()
        self._init_sequence()
        self._log.info('LINAK_2: ready.')

    def shutdown(self):
        self._cancel_move()
        self._set_current(CMD_STOP)
        time.sleep(0.2)
        self._cmd_stop_evt.set()
        if hasattr(self, '_bus'):
            try:
                self._bus.shutdown()
            except Exception:
                pass

    def _send_raw(self, code: int):
        """For J1939, sending = updating the keepalive loop's current code."""
        self._set_current(code)

    def _set_current(self, code: int):
        with self._cmd_lock:
            self._current_code = code

    def _command_loop(self):
        """Resend current command every _RESEND_INTERVAL s (J1939 keepalive)."""
        while not self._cmd_stop_evt.is_set():
            with self._cmd_lock:
                code = self._current_code
            if code is not None:
                try:
                    self._bus.send(can.Message(
                        arbitration_id=self._can_id,
                        data=[code & 0xFF, (code >> 8) & 0xFF,
                              0xFB, 0xFB, 0xFB, 0xFB, 0xFF, 0xFF],
                        is_extended_id=True
                    ))
                except can.CanError as exc:
                    self._log.warn(f'LINAK_2 CAN send error: {exc}')
            self._cmd_stop_evt.wait(timeout=self._RESEND_INTERVAL)


# ============================================================
#  ROS2 Node
# ============================================================

class LinakCanNode(Node):

    def __init__(self):
        super().__init__('linak_can_node')
        self.status_pub = self.create_publisher(String, '/linak_status', 10)
        self.create_subscription(String, '/linak_cmd', self._on_cmd, 10)
        self._handlers = {
            '1': Linak1Handler(self.get_logger()),
            '2': Linak2Handler(self.get_logger()),
        }
        threading.Thread(target=self._init_all, daemon=True, name='linak_init').start()

    def destroy_node(self):
        for h in self._handlers.values():
            h.shutdown()
        super().destroy_node()

    def _init_all(self):
        try:
            for h in self._handlers.values():
                h.init(done_callback=self._publish)
            self._publish('READY:LINAK')
            self.get_logger().info('Both LINAK actuators ready.')
        except Exception as exc:
            self.get_logger().fatal(f'LINAK init failed: {exc}')
            self._publish(f'ERR:LINAK_INIT:{exc}')

    def _on_cmd(self, msg: String):
        parts = msg.data.strip().split(',')
        if len(parts) < 3 or parts[0] != 'LINAK' or parts[1] not in self._handlers:
            self._publish(f'ERR:LINAK_PARSE:{msg.data}')
            return

        handler = self._handlers[parts[1]]
        action  = parts[2]
        try:
            if action in ('DOWN', 'UP'):
                if len(parts) < 5:
                    self._publish(f'ERR:{handler.label}:MISSING_PARAMS')
                    return
                handler.move(action, duration_s=float(parts[4]))
                self._publish(f'ACK:{handler.label}')
            elif action == 'STOP':
                handler.stop()
                self._publish(f'ACK:{handler.label}')
            else:
                self._publish(f'ERR:{handler.label}:UNKNOWN_ACTION:{action}')
        except Exception as exc:
            self.get_logger().error(f'{handler.label} error: {exc}')
            self._publish(f'ERR:{handler.label}:{exc}')

    def _publish(self, status: str):
        self.status_pub.publish(String(data=status))
        self.get_logger().info(f'-> /linak_status: {status}')


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