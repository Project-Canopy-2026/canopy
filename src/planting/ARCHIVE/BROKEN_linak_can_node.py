#!/usr/bin/env python3
"""
linak_driver.py — ROS 2 CANopen driver for two LINAK actuators.

Subscribes to  /linak_cmd     (std_msgs/String)
Publishes  to  /linak_status  (std_msgs/String)

Command protocol  (matches planting_fsm.py)
───────────────────────────────────────────
  LINAK,<id>,DOWN,<duration_s>  — extend actuator <id> for <duration_s> s, then STOP
  LINAK,<id>,UP,<duration_s>    — retract actuator <id> for <duration_s> s, then STOP
  LINAK,<id>,STOP               — stop immediately (no status reply)
  LINAK,<id>,CLEAR              — clear faults     (no status reply)

Status replies on /linak_status
────────────────────────────────
  DONE:LINAK<id>          — timed move completed normally
  ERR:LINAK<id>:<detail>  — exception during move

ROS parameters
──────────────
  can_channel          str   "can0"
  can_bitrate          int   125000
  node_id_1            int   0x20    (LINAK 1 — auger)
  node_id_2            int   0x21    (LINAK 2 — chute)
  heartbeat_period_ms  int   100
  eds_file_1           str   "LINAK-actuator-v3-1.eds"
  eds_file_2           str   "LINAK-actuator-v3-1.eds"
"""

import threading
import time
from typing import Callable, Optional

import canopen
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── LINAK position codes ──────────────────────────────────────────────────────
CMD_OUT   = 64257   # run out (extend)
CMD_IN    = 64258   # run in  (retract)
CMD_STOP  = 64259   # stop
CMD_CLEAR = 64256   # clear errors


def _rpdo(code: int) -> list:
    """Build an 8-byte RPDO1 payload (LINAK manual p.11)."""
    return [
        code & 0xFF, (code >> 8) & 0xFF,
        0xFB,        # current  — default
        0xCD,        # speed    — MAX
        0xFB,        # ramp up  — default
        0xFB,        # ramp dn  — default
        0x00, 0x00,
    ]


# ── Per-actuator channel ──────────────────────────────────────────────────────

class ActuatorChannel:
    """Owns one CANopen RemoteNode and a background timed-move thread."""

    def __init__(
        self,
        linak_id:    int,
        node_id:     int,
        eds_file:    str,
        network:     canopen.Network,
        hb_period_ms: int,
        logger,
    ) -> None:
        self.linak_id  = linak_id
        self.cob_rpdo1 = 0x200 + node_id
        self._network  = network
        self._hb_ms    = hb_period_ms
        self._log      = logger

        self.can_node = canopen.RemoteNode(node_id, eds_file)
        network.add_node(self.can_node)

        self._move_thread: Optional[threading.Thread] = None
        self._cancel      = threading.Event()

    # ── Startup ───────────────────────────────────────────────────────────

    def initialize(self) -> None:
        lid = self.linak_id
        nid = self.can_node.id
        self._log.info(f"[LINAK{lid}] Initialising node 0x{nid:02X} ...")

        # Consumer heartbeat: expect master (node 1) every N ms
        self.can_node.sdo[0x1016][1].raw = (0x01 << 16) + self._hb_ms
        time.sleep(0.1)

        # RPDO1 mapping
        rpdo = self.can_node.rpdo[1]
        rpdo.clear()
        rpdo.add_variable("Actuator Command.Position")
        rpdo.enabled = True
        rpdo.save()
        time.sleep(0.1)

        # NMT -> OPERATIONAL
        self.can_node.nmt.state = "OPERATIONAL"
        time.sleep(0.1)

        # Safe initial state
        self._send(CMD_STOP)
        time.sleep(1.0)
        self._send(CMD_CLEAR)
        time.sleep(1.0)

        self._log.info(f"[LINAK{lid}] Ready.")

    # ── Commands ──────────────────────────────────────────────────────────

    def start_timed_move(
        self,
        direction: int,
        duration_s: float,
        on_done: Callable,
        on_err:  Callable,
    ) -> None:
        """Run in <direction> for <duration_s> seconds, then STOP.
        Cancels any move already in progress."""
        self._abort()
        self._cancel.clear()
        self._move_thread = threading.Thread(
            target=self._worker,
            args=(direction, duration_s, on_done, on_err),
            daemon=True,
            name=f"linak{self.linak_id}_move",
        )
        self._move_thread.start()

    def stop(self) -> None:
        """Abort any running move and send STOP."""
        self._abort()
        try:
            self._send(CMD_STOP)
        except Exception as exc:
            self._log.warn(f"[LINAK{self.linak_id}] STOP failed: {exc}")

    def clear(self) -> None:
        self._send(CMD_CLEAR)

    # ── Internals ─────────────────────────────────────────────────────────

    def _send(self, code: int) -> None:
        self._network.send_message(self.cob_rpdo1, _rpdo(code))

    def _abort(self) -> None:
        """Signal the worker to exit and wait for it."""
        self._cancel.set()
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=2.0)
        self._move_thread = None

    def _worker(
        self,
        direction: int,
        duration_s: float,
        on_done: Callable,
        on_err:  Callable,
    ) -> None:
        try:
            self._send(direction)
            cancelled = self._cancel.wait(timeout=duration_s)
            self._send(CMD_STOP)
            if not cancelled:
                on_done()
        except Exception as exc:
            try:
                self._send(CMD_STOP)
            except Exception:
                pass
            on_err(str(exc))


# ── Driver node ───────────────────────────────────────────────────────────────

class LinakDriver(Node):

    def __init__(self) -> None:
        super().__init__("linak_can_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("can_channel",         "can0")
        self.declare_parameter("can_bitrate",         125000)
        self.declare_parameter("node_id_1",           0x20)
        self.declare_parameter("node_id_2",           0x21)
        self.declare_parameter("heartbeat_period_ms", 100)
        self.declare_parameter("eds_file_1",          "LINAK-actuator-v3-1.eds")
        self.declare_parameter("eds_file_2",          "LINAK-actuator-v3-1.eds")

        channel   = self.get_parameter("can_channel").value
        bitrate   = self.get_parameter("can_bitrate").value
        node_id_1 = self.get_parameter("node_id_1").value
        node_id_2 = self.get_parameter("node_id_2").value
        hb_ms     = self.get_parameter("heartbeat_period_ms").value
        eds_1     = self.get_parameter("eds_file_1").value
        eds_2     = self.get_parameter("eds_file_2").value

        HB_COB = 0x701  # master heartbeat COB-ID (node 0x01)

        # ── CAN network ───────────────────────────────────────────────────
        self.get_logger().info(f"Connecting to '{channel}' at {bitrate} bit/s ...")
        self._network = canopen.Network()
        self._network.connect(channel=channel, bustype="socketcan", bitrate=bitrate)
        self._network.send_message(HB_COB, [0x05])  # first heartbeat before SDO

        # ── Actuator channels ─────────────────────────────────────────────
        self._ch: dict[int, ActuatorChannel] = {
            1: ActuatorChannel(1, node_id_1, eds_1, self._network, hb_ms,
                               self.get_logger()),
            2: ActuatorChannel(2, node_id_2, eds_2, self._network, hb_ms,
                               self.get_logger()),
        }
        for ch in self._ch.values():
            ch.initialize()

        # ── Master heartbeat thread ───────────────────────────────────────
        self._hb_cob  = HB_COB
        self._hb_ms   = hb_ms
        self._stop_hb = threading.Event()
        threading.Thread(
            target=self._heartbeat_loop, daemon=True, name="linak_heartbeat"
        ).start()

        # ── ROS I/O ───────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, "/linak_status", 10)
        self.create_subscription(String, "/linak_cmd", self._on_cmd, 10)
        self.get_logger().info("linak_driver ready.")

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        self.get_logger().info("Shutting down ...")
        for ch in self._ch.values():
            ch.stop()
        self._stop_hb.set()
        try:
            self._network.disconnect()
        except Exception:
            pass

    # ── Heartbeat ─────────────────────────────────────────────────────────────

    def _heartbeat_loop(self) -> None:
        interval = self._hb_ms / 1000.0
        self.get_logger().info(f"Heartbeat started ({self._hb_ms} ms)")
        while not self._stop_hb.is_set():
            try:
                self._network.send_message(self._hb_cob, [0x05])
            except Exception as exc:
                self.get_logger().warn(f"Heartbeat error: {exc}")
            time.sleep(interval)

    # ── Status helpers ────────────────────────────────────────────────────────

    def _done(self, linak_id: int) -> None:
        token = f"DONE:LINAK{linak_id}"
        self.get_logger().info(f"-> /linak_status: {token}")
        self._status_pub.publish(String(data=token))

    def _err(self, linak_id: int, detail: str) -> None:
        token = f"ERR:LINAK{linak_id}:{detail}"
        self.get_logger().error(f"-> /linak_status: {token}")
        self._status_pub.publish(String(data=token))

    # ── Command callback ──────────────────────────────────────────────────────

    def _on_cmd(self, msg: String) -> None:
        """
        Expected formats:
          LINAK,<id>,DOWN,<seconds>
          LINAK,<id>,UP,<seconds>
          LINAK,<id>,STOP
          LINAK,<id>,CLEAR
        """
        raw = msg.data.strip()
        self.get_logger().info(f"<- /linak_cmd: '{raw}'")

        parts = [p.strip() for p in raw.split(",")]

        if len(parts) < 3 or parts[0].upper() != "LINAK":
            self.get_logger().error(
                f"Malformed command '{raw}'. "
                "Expected: LINAK,<id>,<DOWN|UP|STOP|CLEAR>[,<seconds>]"
            )
            return

        try:
            lid = int(parts[1])
        except ValueError:
            self.get_logger().error(f"Invalid actuator id '{parts[1]}'")
            return

        if lid not in self._ch:
            self.get_logger().error(
                f"Unknown actuator id {lid}. Known ids: {list(self._ch.keys())}"
            )
            return

        ch     = self._ch[lid]
        action = parts[2].upper()

        if action in ("DOWN", "UP"):
            if len(parts) < 4:
                self.get_logger().error(
                    f"'{action}' requires a duration. "
                    f"Expected: LINAK,{lid},{action},<seconds>"
                )
                return
            try:
                duration = float(parts[3])
            except ValueError:
                self.get_logger().error(f"Invalid duration '{parts[3]}'")
                return

            direction = CMD_OUT if action == "DOWN" else CMD_IN
            label     = "Extending" if action == "DOWN" else "Retracting"
            self.get_logger().info(f"[LINAK{lid}] {label} for {duration} s")
            ch.start_timed_move(
                direction  = direction,
                duration_s = duration,
                on_done    = lambda: self._done(lid),
                on_err     = lambda detail: self._err(lid, detail),
            )

        elif action == "STOP":
            self.get_logger().info(f"[LINAK{lid}] STOP")
            ch.stop()

        elif action == "CLEAR":
            self.get_logger().info(f"[LINAK{lid}] CLEAR")
            ch.clear()

        else:
            self.get_logger().error(
                f"Unknown action '{action}'. Valid: DOWN, UP, STOP, CLEAR"
            )


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    driver = LinakDriver()
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.shutdown()
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()