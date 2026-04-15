#!/usr/bin/env python3
"""
linak_can_node.py — ROS 2 CANopen driver for two LINAK LA36 actuators.

Subscribes to  /linak_cmd     (std_msgs/String)
Publishes  to  /linak_status  (std_msgs/String)

Command protocol  (matches planting_fsm_outmax.py)
───────────────────────────────────────────────────
  LINAK,<id>,DOWN,<duration_s>  — extend for <duration_s> s, then STOP
  LINAK,<id>,UP,<duration_s>    — retract for <duration_s> s, then STOP
  LINAK,<id>,OUT_MAX            — drive to full extension; completion via TPDO
  LINAK,<id>,IN_MAX             — drive to full retraction; completion via TPDO
  LINAK,<id>,STOP               — stop immediately
  LINAK,<id>,CLEAR              — clear faults

Status replies on /linak_status
────────────────────────────────
  READY:LINAK              — both actuators initialised (published once on startup)
  ACK:LINAK<id>            — command accepted
  DONE:LINAK<id>           — timed move or position-based move completed
  ERR:LINAK<id>:<detail>   — exception during move

ROS parameters
──────────────
  can_channel          str    "can1"
  can_bitrate          int    125000
  node_id_1            int    0x20    (LINAK 1 — auger)
  node_id_2            int    0x21    (LINAK 2 — chute)
  heartbeat_period_ms  int    100
  eds_file             str    ""      (empty = use bundled EDS from share/planting_controller/eds/)

BUGS FIXED vs. previous versions
──────────────────────────────────
  1. OUT_MAX / IN_MAX commands were silently ignored — now handled.
  2. TPDO position listener added for OUT_MAX/IN_MAX completion detection.
  3. EDS path resolved relative to this file's install location, not CWD.
  4. lambda closure capture bug in _on_cmd fixed (lid=lid default arg).
  5. READY:LINAK and ACK:LINAK<n> status messages now published.
  6. Both actuators share one canopen.Network (correct); HB_COB = 0x701
     matches the SDO consumer-heartbeat entry (0x01 << 16) already in use.
"""

import os
import threading
import time
from typing import Callable, Optional

import canopen
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from ament_index_python.packages import get_package_share_directory as _get_share
    _AMENT_AVAILABLE = True
except ImportError:
    _AMENT_AVAILABLE = False

# ── LINAK PDO position codes (LINAK manual, p.11) ────────────────────────────
CMD_OUT   = 64257   # run out  (extend)
CMD_IN    = 64258   # run in   (retract)
CMD_STOP  = 64259   # stop
CMD_CLEAR = 64256   # clear errors

# Position targets for OUT_MAX / IN_MAX
# can also achieve this with full speed (CD) for 14 seconds
POS_OUT_MAX = 64255  # full extension
POS_IN_MAX  = 150    # full retraction

# How many consecutive stable TPDO samples (at ~250 ms each) before declaring done
_STABLE_COUNT = 4    # ~1 second of no movement
_POS_EPSILON  = 5    # position counts tolerance for "not moving"


def _rpdo(code: int) -> list:
    """Build an 8-byte RPDO1 payload (LINAK manual p.11)."""
    return [
        code & 0xFF, (code >> 8) & 0xFF,
        0xFB,   # current  — default
        0x64,   # speed    — MAX
        0xFB,   # ramp up  — default
        0xFB,   # ramp dn  — default
        0x00, 0x00,
    ]


def _resolve_eds(eds_param: str) -> str:
    """
    Return an absolute path to LINAK-actuator-v3-1.eds.

    Search order:
      1. 'eds_file' ROS parameter — used as-is if it points to a real file.
      2. share/planting_controller/eds/ via ament_index (correct colcon install).
      3. Same directory as this source file (dev / unit-test run-from-source layout).
    """
    eds_name = "LINAK-actuator-v3-1.eds"

    # 1 — explicit parameter override
    if eds_param:
        if os.path.isfile(eds_param):
            return os.path.abspath(eds_param)
        raise FileNotFoundError(
            f"eds_file parameter points to non-existent path: '{eds_param}'"
        )

    # 2 — colcon install share directory (ament_index is the authoritative way)
    if _AMENT_AVAILABLE:
        try:
            share = _get_share("planting_controller")
            candidate = os.path.join(share, "eds", eds_name)
            if os.path.isfile(candidate):
                return candidate
        except Exception:
            pass  # package not found in ament index — fall through

    # 3 — dev layout: EDS lives next to this .py file (unit_test/can_tests/)
    local = os.path.join(os.path.dirname(os.path.abspath(__file__)), eds_name)
    if os.path.isfile(local):
        return local

    raise FileNotFoundError(
        f"Cannot find '{eds_name}'. "
        "Either set the 'eds_file' ROS parameter to the full path, "
        "or ensure the file is installed under share/planting_controller/eds/ "
        "(rebuild with 'colcon build --packages-select planting_controller')."
    )


# ── Per-actuator channel ──────────────────────────────────────────────────────

class ActuatorChannel:
    """
    Owns one CANopen RemoteNode and a background move thread.

    Handles three move modes:
      • timed  (DOWN / UP)  — sends direction command, waits N seconds, sends STOP.
      • to_max (OUT_MAX / IN_MAX) — drives to a position limit; completion is
        detected by watching TPDO position stabilisation.
    """

    def __init__(
        self,
        linak_id:     int,
        node_id:      int,
        eds_file:     str,
        network:      canopen.Network,
        hb_period_ms: int,
        logger,
    ) -> None:
        self.linak_id  = linak_id
        self.cob_rpdo1 = 0x200 + node_id
        self.cob_tpdo1 = 0x180 + node_id
        self._network  = network
        self._hb_ms    = hb_period_ms
        self._log      = logger

        self.can_node = canopen.RemoteNode(node_id, eds_file)
        network.add_node(self.can_node)

        self._move_thread: Optional[threading.Thread] = None
        self._cancel = threading.Event()

        # Latest TPDO position (updated by network message callback)
        self._tpdo_pos: Optional[int] = None
        self._tpdo_lock = threading.Lock()

    # ── Startup ───────────────────────────────────────────────────────────

    def configure(self) -> None:
        """
        Phase 1 of init — runs while node stays in PRE-OPERATIONAL.

        Mirrors the exact sequence from the working can_tests.py unit test:
          1. Arm consumer-HB watchdog via SDO  (heartbeat already flowing)
          2. RPDO1 mapping
          3. Subscribe to TPDO1
        NO NMT Reset Communication — that wipes NVM config and creates a
        window where the node fires EMCY 0x8130 before the new SDO lands,
        latching error code 0x05 and causing the node to ignore all RPDOs.
        Faults from previous runs are cleared by the CLEAR command in
        go_operational(), exactly as can_tests.py does.
        """
        lid = self.linak_id
        nid = self.can_node.id
        self._log.info(f"[LINAK{lid}] Configure node 0x{nid:02X} ...")

        # ── Consumer heartbeat watchdog ───────────────────────────────────
        # Watchdog = 3× heartbeat period for OS scheduling jitter margin.
        # The heartbeat thread is already running and warm before this call.
        watchdog_ms = self._hb_ms * 3
        self._log.info(
            f"[LINAK{lid}] HB watchdog: period={self._hb_ms} ms, "
            f"timeout={watchdog_ms} ms"
        )
        self.can_node.sdo[0x1016][1].raw = (0x01 << 16) + watchdog_ms
        time.sleep(0.1)

        # ── RPDO1 mapping ─────────────────────────────────────────────────
        rpdo = self.can_node.rpdo[1]
        rpdo.clear()
        rpdo.add_variable("Actuator Command.Position")
        rpdo.enabled = True
        rpdo.save()
        time.sleep(0.1)

        # ── TPDO1 subscription for position feedback ──────────────────────
        self._network.subscribe(self.cob_tpdo1, self._on_tpdo)

        self._log.info(f"[LINAK{lid}] Configuration done.")

    def go_operational(self) -> None:
        """
        Phase 2 of init — called after ALL actuators are configured.

        Sends NMT OPERATIONAL then the initial STOP + CLEAR.
        Both actuators are brought up together so neither waits idle long
        enough to trip its heartbeat watchdog while the other is being set up.
        """
        lid = self.linak_id
        self._log.info(f"[LINAK{lid}] NMT → OPERATIONAL ...")
        self.can_node.nmt.state = "OPERATIONAL"
        time.sleep(0.2)   # let the node settle

        self._send(CMD_STOP)
        time.sleep(1.0)
        self._send(CMD_CLEAR)
        time.sleep(1.0)

        self._log.info(f"[LINAK{lid}] Ready.")

    def initialize(self) -> None:
        """Convenience wrapper — configure then go_operational (single-node use)."""
        self.configure()
        self.go_operational()

    # ── TPDO callback ─────────────────────────────────────────────────────

    def _on_tpdo(self, can_id: int, data: bytes, timestamp: float) -> None:
        """
        TPDO1 layout (LINAK-actuator-v3-1.eds, object 0x2001):
          bytes 0-1  Position (uint16, little-endian)
          byte  2    Current  (uint8)
          byte  3    Status Flags (uint8)
          byte  4    Error Code   (uint8)
          bytes 5-6  Speed        (uint16, little-endian)
          byte  7    Input State  (uint8)
        """
        if len(data) >= 2:
            pos = data[0] | (data[1] << 8)
            with self._tpdo_lock:
                self._tpdo_pos = pos

    def get_position(self) -> Optional[int]:
        with self._tpdo_lock:
            return self._tpdo_pos

    # ── Public commands ───────────────────────────────────────────────────

    def start_timed_move(
        self,
        direction:  int,
        duration_s: float,
        on_done:    Callable,
        on_err:     Callable,
    ) -> None:
        """Extend/retract for <duration_s> seconds, then STOP."""
        self._abort()
        self._cancel.clear()
        self._move_thread = threading.Thread(
            target=self._worker_timed,
            args=(direction, duration_s, on_done, on_err),
            daemon=True,
            name=f"linak{self.linak_id}_timed",
        )
        self._move_thread.start()

    def start_position_move(
        self,
        direction:    int,
        target_pos:   int,
        on_done:      Callable,
        on_err:       Callable,
    ) -> None:
        """
        Drive in <direction> until position stabilises near <target_pos>.
        Completion fires on_done(); the move thread monitors TPDO feedback.
        """
        self._abort()
        self._cancel.clear()
        self._move_thread = threading.Thread(
            target=self._worker_position,
            args=(direction, target_pos, on_done, on_err),
            daemon=True,
            name=f"linak{self.linak_id}_pos",
        )
        self._move_thread.start()

    def stop(self) -> None:
        """Abort any running move and send STOP."""
        self._abort()
        try:
            self._send(CMD_STOP)
        except Exception as exc:
            self._log.warn(f"[LINAK{self.linak_id}] STOP send failed: {exc}")

    def clear(self) -> None:
        try:
            self._send(CMD_CLEAR)
        except Exception as exc:
            self._log.warn(f"[LINAK{self.linak_id}] CLEAR send failed: {exc}")

    # ── Internals ─────────────────────────────────────────────────────────

    def _send(self, code: int) -> None:
        self._network.send_message(self.cob_rpdo1, _rpdo(code))

    def _abort(self) -> None:
        """Signal the worker to exit and join it (max 3 s)."""
        self._cancel.set()
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=3.0)
        self._move_thread = None

    def _worker_timed(
        self,
        direction:  int,
        duration_s: float,
        on_done:    Callable,
        on_err:     Callable,
    ) -> None:
        try:
            self._send(direction)
            cancelled = self._cancel.wait(timeout=duration_s)
            self._send(CMD_STOP)
            if not cancelled:
                on_done()
        except Exception as exc:
            self._safe_stop()
            on_err(str(exc))

    def _worker_position(
        self,
        direction:  int,
        target_pos: int,
        on_done:    Callable,
        on_err:     Callable,
    ) -> None:
        """
        Drives the actuator to a position limit and detects completion via TPDO.

        Phase 1 — Movement detection (up to MOVE_START_TIMEOUT_S):
          Wait until the position actually starts changing by more than
          _POS_EPSILON.  This guards against the RPDO arriving before the
          actuator has processed the previous CLEAR/STOP and started moving.

        Phase 2 — Stabilisation detection (up to TIMEOUT_S total):
          Once movement is confirmed, count consecutive stable readings.
          _STABLE_COUNT readings within _POS_EPSILON of each other → done.

        Hard timeout of TIMEOUT_S covers mechanical stalls and fault conditions.
        """
        POLL_INTERVAL         = 0.25   # seconds between position checks
        TIMEOUT_S             = 60.0   # total hard timeout
        MOVE_START_TIMEOUT_S  = 3.0    # how long to wait for motion to begin

        try:
            self._send(direction)

            stable_count  = 0
            last_pos      = None
            start_pos     = None
            elapsed       = 0.0
            moving        = False   # True once we've seen actual displacement

            while not self._cancel.is_set():
                time.sleep(POLL_INTERVAL)
                elapsed += POLL_INTERVAL

                if elapsed >= TIMEOUT_S:
                    raise TimeoutError(
                        f"LINAK{self.linak_id} position move timed out after {TIMEOUT_S} s"
                    )

                pos = self.get_position()
                if pos is None:
                    # No TPDO received yet — keep waiting
                    continue

                # ── Phase 1: wait for motion to start ─────────────────────
                if not moving:
                    if start_pos is None:
                        start_pos = pos
                    elif abs(pos - start_pos) > _POS_EPSILON:
                        moving = True
                        self._log.info(
                            f"[LINAK{self.linak_id}] Motion detected: "
                            f"{start_pos} → {pos}"
                        )
                    elif elapsed >= MOVE_START_TIMEOUT_S:
                        # Actuator hasn't moved — re-send the command once and
                        # extend the window (could be post-CLEAR settling time)
                        self._log.warn(
                            f"[LINAK{self.linak_id}] No motion after "
                            f"{MOVE_START_TIMEOUT_S} s — re-sending command"
                        )
                        self._send(direction)
                        start_pos = pos          # reset baseline
                        elapsed   = 0.0          # reset clock for this phase
                    last_pos = pos
                    continue

                # ── Phase 2: stabilisation detection ──────────────────────
                if last_pos is not None and abs(pos - last_pos) <= _POS_EPSILON:
                    stable_count += 1
                else:
                    stable_count = 0

                last_pos = pos

                if stable_count >= _STABLE_COUNT:
                    self._send(CMD_STOP)
                    self._log.info(
                        f"[LINAK{self.linak_id}] Position stable at {pos} "
                        f"(target ~{target_pos})"
                    )
                    on_done()
                    return

            # Cancelled externally
            self._safe_stop()

        except Exception as exc:
            self._safe_stop()
            on_err(str(exc))

    def _safe_stop(self) -> None:
        try:
            self._send(CMD_STOP)
        except Exception:
            pass


# ── Driver node ───────────────────────────────────────────────────────────────

class LinakDriver(Node):

    def __init__(self) -> None:
        super().__init__("linak_can_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("can_channel",         "can1")
        self.declare_parameter("can_bitrate",         125000)
        self.declare_parameter("node_id_1",           0x20)
        self.declare_parameter("node_id_2",           0x21)
        self.declare_parameter("heartbeat_period_ms", 100)
        self.declare_parameter("eds_file",            "")

        channel   = self.get_parameter("can_channel").value
        bitrate   = self.get_parameter("can_bitrate").value
        node_id_1 = self.get_parameter("node_id_1").value
        node_id_2 = self.get_parameter("node_id_2").value
        hb_ms     = self.get_parameter("heartbeat_period_ms").value
        eds_param = self.get_parameter("eds_file").value

        eds_file = _resolve_eds(eds_param)
        self.get_logger().info(f"Using EDS: {eds_file}")

        # HB_COB = 0x701 (node 0x01 heartbeat).
        # The actuator's consumer-heartbeat SDO (0x1016 sub1) is set to
        # (0x01 << 16) + period_ms, meaning "I expect heartbeats from node 1
        # on COB-ID 0x701."  This matches what the working unit test uses.
        HB_COB = 0x701

        # ── CAN network ───────────────────────────────────────────────────
        self.get_logger().info(f"Connecting to '{channel}' @ {bitrate} bps ...")
        self._network = canopen.Network()
        self._network.connect(channel=channel, bustype="socketcan", bitrate=bitrate)
        # Prime the heartbeat before any SDO traffic so the actuator doesn't
        # report a consumer-heartbeat timeout during initialisation.
        self._network.send_message(HB_COB, [0x05])

        # ── Master heartbeat thread — START BEFORE any SDO traffic ──────────
        # The actuator's consumer-heartbeat watchdog (0x1016) is armed during
        # initialize() via SDO. If the heartbeat thread isn't already running
        # when that SDO completes, the watchdog fires immediately, the actuator
        # broadcasts EMCY 0x8130 (heartbeat consumer timeout), drops to
        # PRE-OPERATIONAL, and silently discards all subsequent RPDOs.
        # Starting the thread first — and letting it run for at least 3 full
        # periods before arming the watchdog — eliminates that race entirely.
        self._hb_cob  = HB_COB
        self._hb_ms   = hb_ms
        self._stop_hb = threading.Event()
        threading.Thread(
            target=self._heartbeat_loop, daemon=True, name="linak_heartbeat"
        ).start()
        # Warm-up: let ≥3 heartbeats reach the bus before SDO setup begins
        time.sleep((hb_ms / 1000.0) * 3 + 0.05)
        self.get_logger().info("Heartbeat warm-up complete — starting actuator init ...")

        # ── Actuator channels ─────────────────────────────────────────────
        self._ch: dict[int, ActuatorChannel] = {
            1: ActuatorChannel(1, node_id_1, eds_file, self._network, hb_ms,
                               self.get_logger()),
            2: ActuatorChannel(2, node_id_2, eds_file, self._network, hb_ms,
                               self.get_logger()),
        }
        # ── Two-phase init ────────────────────────────────────────────────
        # Phase 1: configure ALL nodes (SDO, RPDO mapping) while they stay in
        # PRE-OPERATIONAL.  Neither node's heartbeat watchdog can fire here
        # because they are not yet armed with an NMT OPERATIONAL state.
        for ch in self._ch.values():
            ch.configure()

        # Phase 2: bring ALL nodes OPERATIONAL together.
        # Doing this sequentially (node1 then node2) is fine now because the
        # watchdog timeout is 3× the HB period — 300 ms of margin easily
        # covers the ~1.2 s STOP+CLEAR sequence for the other node.
        for ch in self._ch.values():
            ch.go_operational()

        # ── ROS I/O ───────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, "/linak_status", 10)
        self.create_subscription(String, "/linak_cmd", self._on_cmd, 10)

        # Signal that both actuators are ready
        self._publish_status("READY:LINAK")
        self.get_logger().info("linak_can_node ready — both actuators initialised.")

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        self.get_logger().info("Shutting down linak_can_node ...")
        for ch in self._ch.values():
            ch.stop()
        self._stop_hb.set()
        time.sleep(0.15)
        try:
            self._network.disconnect()
        except Exception:
            pass

    # ── Heartbeat ─────────────────────────────────────────────────────────────

    def _heartbeat_loop(self) -> None:
        interval = self._hb_ms / 1000.0
        self.get_logger().info(f"Master heartbeat started ({self._hb_ms} ms, COB 0x{self._hb_cob:03X})")
        while not self._stop_hb.is_set():
            try:
                self._network.send_message(self._hb_cob, [0x05])
            except Exception as exc:
                self.get_logger().warn(f"Heartbeat send error: {exc}")
            time.sleep(interval)

    # ── Status helpers ────────────────────────────────────────────────────────

    def _publish_status(self, token: str) -> None:
        self.get_logger().info(f"→ /linak_status: {token}")
        self._status_pub.publish(String(data=token))

    def _done(self, linak_id: int) -> None:
        self._publish_status(f"DONE:LINAK{linak_id}")

    def _err(self, linak_id: int, detail: str) -> None:
        self._publish_status(f"ERR:LINAK{linak_id}:{detail}")

    # ── Command callback ──────────────────────────────────────────────────────

    def _on_cmd(self, msg: String) -> None:
        """
        Accepted formats:
          LINAK,<id>,DOWN,<seconds>
          LINAK,<id>,UP,<seconds>
          LINAK,<id>,OUT_MAX
          LINAK,<id>,IN_MAX
          LINAK,<id>,STOP
          LINAK,<id>,CLEAR
        """
        raw = msg.data.strip()
        self.get_logger().info(f"← /linak_cmd: '{raw}'")

        parts = [p.strip() for p in raw.split(",")]

        if len(parts) < 3 or parts[0].upper() != "LINAK":
            self.get_logger().error(
                f"Malformed command '{raw}'. "
                "Expected: LINAK,<id>,<DOWN|UP|OUT_MAX|IN_MAX|STOP|CLEAR>[,<seconds>]"
            )
            return

        try:
            lid = int(parts[1])
        except ValueError:
            self.get_logger().error(f"Invalid actuator id '{parts[1]}' — must be integer")
            return

        if lid not in self._ch:
            self.get_logger().error(
                f"Unknown actuator id {lid}. Known ids: {list(self._ch.keys())}"
            )
            return

        ch     = self._ch[lid]
        action = parts[2].upper()

        # ── Timed moves ───────────────────────────────────────────────────
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
            self._publish_status(f"ACK:LINAK{lid}")

            # FIX: capture lid by value in the lambda (was a closure bug)
            ch.start_timed_move(
                direction  = direction,
                duration_s = duration,
                on_done    = lambda lid=lid: self._done(lid),
                on_err     = lambda detail, lid=lid: self._err(lid, detail),
            )

        # ── Position-based moves ──────────────────────────────────────────
        elif action == "OUT_MAX":
            self.get_logger().info(f"[LINAK{lid}] OUT_MAX → driving to full extension")
            self._publish_status(f"ACK:LINAK{lid}")
            ch.start_position_move(
                direction  = CMD_OUT,
                target_pos = POS_OUT_MAX,
                on_done    = lambda lid=lid: self._done(lid),
                on_err     = lambda detail, lid=lid: self._err(lid, detail),
            )

        elif action == "IN_MAX":
            self.get_logger().info(f"[LINAK{lid}] IN_MAX → driving to full retraction")
            self._publish_status(f"ACK:LINAK{lid}")
            ch.start_position_move(
                direction  = CMD_IN,
                target_pos = POS_IN_MAX,
                on_done    = lambda lid=lid: self._done(lid),
                on_err     = lambda detail, lid=lid: self._err(lid, detail),
            )

        # ── Immediate commands ────────────────────────────────────────────
        elif action == "STOP":
            self.get_logger().info(f"[LINAK{lid}] STOP")
            ch.stop()

        elif action == "CLEAR":
            self.get_logger().info(f"[LINAK{lid}] CLEAR")
            ch.clear()

        else:
            self.get_logger().error(
                f"Unknown action '{action}'. "
                "Valid: DOWN, UP, OUT_MAX, IN_MAX, STOP, CLEAR"
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