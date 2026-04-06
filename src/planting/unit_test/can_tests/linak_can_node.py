#!/usr/bin/env python3
# ============================================================
#  LINAK CAN Node — planting_controller
#
#  Both LINAK LA36 actuators on a single CANopen bus:
#    can0 @ 125 kbps
#    LINAK_1 (Auger) — Node ID 0x20 (32)
#    LINAK_2 (Chute) — Node ID 0x21 (33)
#
#  Direction mapping (confirmed):
#    DOWN = RUN_OUT  for both actuators
#    UP   = RUN_IN   for both actuators
#
#  Subscribes:  /linak_cmd    (std_msgs/String)
#  Publishes:   /linak_status (std_msgs/String)
#
#  Command format:
#    "LINAK,<1|2>,<DOWN|UP|OUT_MAX|IN_MAX|STOP>[,<speed_pct>,<duration_s>]"
#    OUT_MAX / IN_MAX require no duration — actuator self-stops at the travel limit.
#
#  Status replies:
#    "ACK:LINAK<n>"     command received
#    "DONE:LINAK<n>"    timed move completed
#    "ERR:LINAK<n>:…"   fault
#    "READY:LINAK"      both actuators initialised
#
#  Pre-requisite:
#    sudo ip link set can0 up type can bitrate 125000
#
#  Python deps:  canopen
#    pip install canopen
# ============================================================

import threading
import time

import canopen

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ============================================================
#  Shared position codes
# ============================================================

CMD_STOP        = 64259   # 0xFB03
CMD_CLEAR_ERROR = 64256   # 0xFB00
CMD_RUN_OUT     = 64257   # 0xFB01 → DOWN (timed)
CMD_RUN_IN      = 64258   # 0xFB02 → UP  (timed)
CMD_RUN_OUT_MAX = 64255   # 0xFAFF → DOWN to full extension (actuator self-stops at limit)
CMD_RUN_IN_MAX  = 150     # 0x0064 → UP to full retraction (actuator self-stops at limit)
NMT_COB_ID      = 0x000   # NMT broadcast
HB_COB_ID       = 0x701   # master heartbeat (node ID 1)
HB_MS           = 50      # heartbeat interval (ms)


# ============================================================
#  Base handler  —  shared move / stop / timing logic
# ============================================================

class LinakHandler:
    """
    Common interface for a single CANopen LINAK actuator.
    Subclasses set node_id and label; the shared network is
    injected by LinakCanNode after it is created.
    """

    label:   str = ''
    node_id: int = 0

    def __init__(self, logger):
        self._log           = logger
        self._network       = None   # injected after init
        self._cob_rpdo      = 0x200 + self.node_id
        self._tpdo_cob      = 0x180 + self.node_id
        self._done_cb       = None
        self._move_stop     = threading.Event()
        self._move_thread   = None
        self._last_position = None
        self._position_lock = threading.Lock()

    # ── Public API ────────────────────────────────────────────────────

    def move(self, direction: str, duration_s: float):
        self._cancel_move()
        code = CMD_RUN_OUT if direction == 'DOWN' else CMD_RUN_IN
        self._send_raw(code)
        self._log.info(f'{self.label}: moving {direction} for {duration_s} s')

        self._move_stop.clear()
        self._move_thread = threading.Thread(
            target=self._timed_move, args=(duration_s,),
            daemon=True, name=f'{self.label}_move'
        )
        self._move_thread.start()

    def run_out_max(self):
        """Extend to full travel (CMD_RUN_OUT_MAX). Fires _done_cb when position stabilises."""
        self._cancel_move()
        self._send_raw(CMD_RUN_OUT_MAX)
        self._log.info(f'{self.label}: running out to full extension ({CMD_RUN_OUT_MAX}).')
        self._move_stop.clear()
        self._move_thread = threading.Thread(
            target=self._monitor_until_stopped, daemon=True, name=f'{self.label}_move'
        )
        self._move_thread.start()

    def run_in_max(self):
        """Retract to full travel (CMD_RUN_IN_MAX). Fires _done_cb when position stabilises."""
        self._cancel_move()
        self._send_raw(CMD_RUN_IN_MAX)
        self._log.info(f'{self.label}: running in to full retraction ({CMD_RUN_IN_MAX}).')
        self._move_stop.clear()
        self._move_thread = threading.Thread(
            target=self._monitor_until_stopped, daemon=True, name=f'{self.label}_move'
        )
        self._move_thread.start()

    def stop(self):
        self._cancel_move()
        self._send_raw(CMD_STOP)
        self._log.info(f'{self.label}: stopped.')

    def init_sequence(self):
        """STOP → CLEAR_ERROR → STOP before any run command."""
        for code, delay in [(CMD_STOP, 0.5), (CMD_CLEAR_ERROR, 0.5), (CMD_STOP, 0.3)]:
            self._send_raw(code)
            time.sleep(delay)

    # ── Internal ─────────────────────────────────────────────────────

    def _send_raw(self, code: int):
        msg = [code & 0xFF, (code >> 8) & 0xFF,
               0xFB, 0xCD, 0xFB, 0xFB, 0x00, 0x00]
        self._network.send_message(self._cob_rpdo, msg)

    def _timed_move(self, duration_s: float):
        finished = not self._move_stop.wait(timeout=duration_s)
        if finished:
            self._send_raw(CMD_STOP)
            self._log.info(f'{self.label}: timed move complete.')
            if self._done_cb:
                self._done_cb(f'DONE:{self.label}')

    def _on_tpdo(self, cob_id: int, data: bytes, timestamp: float):
        """Called by the canopen network on every TPDO1 from this actuator."""
        pos = data[0] | (data[1] << 8)
        with self._position_lock:
            self._last_position = pos

    def _read_position(self):
        with self._position_lock:
            return self._last_position

    def _monitor_until_stopped(self, timeout: float = 60.0, stable_secs: float = 1.0,
                                poll: float = 0.3):
        """Poll TPDO position; fire _done_cb once stable for stable_secs, or ERR on timeout."""
        deadline = time.time() + timeout
        last_pos = None
        stable_since = None

        while not self._move_stop.is_set():
            if time.time() >= deadline:
                self._log.warning(f'{self.label}: limit-move timed out after {timeout} s.')
                if self._done_cb:
                    self._done_cb(f'ERR:{self.label}:TIMEOUT')
                return

            pos = self._read_position()
            if pos is not None:
                if pos == last_pos:
                    if stable_since is None:
                        stable_since = time.time()
                    elif time.time() - stable_since >= stable_secs:
                        self._log.info(f'{self.label}: position stabilised at {pos} — done.')
                        if self._done_cb:
                            self._done_cb(f'DONE:{self.label}')
                        return
                else:
                    last_pos = pos
                    stable_since = None

            self._move_stop.wait(timeout=poll)

    def _cancel_move(self):
        self._move_stop.set()
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=0.5)
        self._move_stop.clear()


# ============================================================
#  Concrete handlers  —  only differ by label and node_id
# ============================================================

class Linak1Handler(LinakHandler):
    label   = 'LINAK1'
    node_id = 0x20


class Linak2Handler(LinakHandler):
    label   = 'LINAK2'
    node_id = 0x21


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
            h.stop()
        if hasattr(self, '_hb_stop'):
            self._hb_stop.set()
        if hasattr(self, '_network'):
            try:
                self._network.disconnect()
            except Exception:
                pass
        super().destroy_node()

    # ── Startup ───────────────────────────────────────────────────────

    def _init_all(self):
        try:
            self.get_logger().info('Connecting to can0 @ 125 kbps...')

            self._network = canopen.Network()
            self._network.connect(channel='can0', bustype='socketcan')

            for h in self._handlers.values():
                h._network = self._network
                h._done_cb = self._publish
                self._network.subscribe(h._tpdo_cob, h._on_tpdo)

            # Start heartbeat before NMT so actuators don't time out
            self._hb_stop = threading.Event()
            threading.Thread(
                target=self._heartbeat_loop, daemon=True, name='can_heartbeat'
            ).start()
            time.sleep(0.2)

            # NMT: start all nodes (raw broadcast, no RemoteNode/library monitoring)
            self._network.send_message(NMT_COB_ID, [0x01, 0x00])
            time.sleep(0.5)

            # Mandatory init sequence per actuator
            for h in self._handlers.values():
                h.init_sequence()

            self._publish('READY:LINAK')
            self.get_logger().info('Both LINAK actuators initialised on can0.')

        except Exception as exc:
            self.get_logger().fatal(f'LINAK init failed: {exc}')
            self._publish(f'ERR:LINAK_INIT:{exc}')

    def _heartbeat_loop(self):
        """One heartbeat on the shared bus keeps both actuators alive."""
        while not self._hb_stop.is_set():
            try:
                self._network.send_message(HB_COB_ID, [0x05])
            except Exception as exc:
                self.get_logger().warn(f'Heartbeat error: {exc}')
            self._hb_stop.wait(timeout=HB_MS / 1000.0)

    # ── Command subscriber ────────────────────────────────────────────

    def _on_cmd(self, msg: String):
        parts = msg.data.strip().split(',')

        if len(parts) < 3 or parts[0] != 'LINAK':
            self._publish(f'ERR:LINAK_PARSE:{msg.data}')
            return

        actuator_id, action = parts[1], parts[2]
        handler = self._handlers.get(actuator_id)

        if not handler:
            self._publish(f'ERR:UNKNOWN_ACTUATOR:{actuator_id}')
            return

        label = handler.label
        try:
            if action in ('DOWN', 'UP'):
                if len(parts) == 4:
                    duration = float(parts[3])
                elif len(parts) >= 5:
                    duration = float(parts[4])
                else:
                    self._publish(f'ERR:{label}:MISSING_PARAMS')
                    return
                handler.move(action, duration)
                self._publish(f'ACK:{label}')

            elif action == 'OUT_MAX':
                handler.run_out_max()
                self._publish(f'ACK:{label}')

            elif action == 'IN_MAX':
                handler.run_in_max()
                self._publish(f'ACK:{label}')

            elif action == 'STOP':
                handler.stop()
                self._publish(f'ACK:{label}')

            else:
                self._publish(f'ERR:{label}:UNKNOWN_ACTION:{action}')

        except Exception as exc:
            self.get_logger().error(f'{label} error: {exc}')
            self._publish(f'ERR:{label}:{exc}')

    def _publish(self, status: str):
        self.status_pub.publish(String(data=status))
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