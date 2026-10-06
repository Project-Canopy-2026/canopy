#!/usr/bin/env python3
"""
linak_can_node.py — ROS 2 CANopen driver for two LINAK LA36 actuators.

Subscribes to  /linak_cmd     (std_msgs/String)
Publishes  to  /linak_status  (std_msgs/String)

Moves are ABSOLUTE positions, in cm measured from full retraction
─────────────────────────────────────────────────────────────────
Position 0 cm is home (the hard stop at POS_HOME); 30 cm is the end of the
stroke. The setpoint is written into the actuator's mapped 'Actuator Command.
Position' and the hardware's own servo drives there — this node watches TPDO,
confirms arrival and reports. Every target is clamped into [0, STROKE_CM], so no
command can aim past the hard stop and sit grinding against it.

  LINAK,<id>,GOTO,<position_cm>[,FAST|SLOW]  — end up AT position_cm
      Independent of where the actuator is now: GOTO 30 from 10 cm is a 20 cm
      move, not a 30 cm one. This is what planting_fsm.py uses.
  LINAK,<id>,HOME               — retract onto the hard stop (= GOTO 0, FAST,
                                  where a stall IS success)
  LINAK,<id>,DOWN,<distance_cm>[,FAST|SLOW]  — relative jog, extend BY
  LINAK,<id>,UP,<distance_cm>[,FAST|SLOW]    — relative jog, retract BY
      Kept for manual testing (unit_test/can_tests/linak_cmd_test_node.py).
      The FSM does not use them: a relative command is only as good as your
      belief about where the arm currently is, which is exactly the assumption
      that turned a short home into a false obstacle abort.
  LINAK,<id>,STOP               — stop immediately

Speed tag on GOTO/DOWN/UP: FAST = 2.18 cm/s (0xCD), SLOW = 1.09 cm/s (0x64).
Omitting it uses SLOW (preserves the legacy slow-drilling default).
IN_MAX is accepted as a synonym for HOME.

Status replies on /linak_status
────────────────────────────────
  READY:LINAK               — both actuators initialised (published once on startup)
  ACK:LINAK<id>             — command accepted
  DONE:LINAK<id>,<cm>       — reached the commanded target
  STALL:LINAK<id>,<cm>      — stopped short against something solid
  STOPPED:LINAK<id>,<cm>    — aborted by an external STOP
  ERR:LINAK<id>:<detail>    — exception during move

Every terminal reply carries the distance ACTUALLY travelled. With absolute
targets a caller no longer needs it to undo a move — it just commands HOME — but
it is what distinguishes a real obstacle (stalled well short) from the hard stop
(stalled at the commanded depth), and it is worth having in the log either way.

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
  1. Stall no longer masquerades as success — it reports STALL, not DONE, so a
     caller can tell "reached depth" from "stopped short against a rock".
  2. Terminal replies carry actual travel; retracts no longer over-drive into
     the hard stop after a short move.
  3. EDS path resolved relative to this file's install location, not CWD.
  4. lambda closure capture bug in _on_cmd fixed (lid=lid default arg).
  5. READY:LINAK and ACK:LINAK<n> status messages now published.
  6. Both actuators share one canopen.Network (correct); HB_COB = 0x701
     matches the SDO consumer-heartbeat entry (0x01 << 16) already in use.
  7. Position scale corrected to 100 counts/cm (0.1 mm per count, as the
     actuator actually reports) from a derived ≈2136.83. The old value made
     every commanded distance ~21x too large, so a drill aimed at a target it
     could never reach and ended on stall detection — which planting_fsm.py
     reads as an obstacle. Every plant aborted as a false DIGGING_OBSTACLE.
     _TARGET_TOLERANCE and the home position are in the same units and moved
     with it.
  8. Moves are absolute positions driven by the actuator's own servo, matching
     unit_test/fsr/planting_fsr_test.py — the only version of this that has
     ever run in the field. Previously every move was relative (start ± delta)
     and driven by CMD_OUT/CMD_IN "run continuously" with this node's poll loop
     as the only thing that stopped it. Relative targets compound: a homing
     that finished short left the arm part-extended, the next full-depth DOWN
     aimed past the hard stop, and the resulting stall read as a rock. Absolute
     targets plus a real [POS_HOME, POS_MAX] clamp remove the whole class.
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

# ── LINAK PDO command codes (LINAK manual, p.11) ─────────────────────────────
# Values 64256+ are commands; anything below is an absolute position setpoint,
# which is how moves are driven here.
CMD_STOP  = 64259   # stop
CMD_CLEAR = 64256   # clear errors
# "run out" / "run in" continuously. Not used: moves are position setpoints now.
# Kept because they are the other half of the manual's table and knowing which
# values are reserved is what makes the 64255 ceiling make sense.
CMD_OUT   = 64257
CMD_IN    = 64258

# Stroke geometry. The actuator reports and accepts position in 0.1 mm units, so
# 100 counts = 1 cm — confirmed on the hardware, and the same scale
# unit_test/fsr/planting_fsr_test.py uses.
#
# This was previously derived as (64255 - 150) / STROKE_CM ≈ 2136.83, on the
# assumption that the full 0…64255 range spans the stroke. It does not: 64255 is
# merely the highest value that is not a command code (64256+). That made every
# distance ~21x too large, so a drill clamped at a target it could never reach,
# ended on stall detection instead — and a stall during DRILLING_DOWN is exactly
# what planting_fsm.py reads as an obstacle. Every plant aborted as a false
# DIGGING_OBSTACLE.
STROKE_CM     = 30.0
COUNTS_PER_CM = 100.0

# Position reported at full retraction, and the origin all depths are measured
# from: position_cm 0 == POS_HOME. 10 matches planting_fsr_test.py, which homes
# to it in practice.
POS_HOME = 10
# The real end of travel. THIS is the clamp that matters — every target, absolute
# or relative, is clipped into [POS_HOME, POS_MAX], so no command can aim past
# the hard stop and sit there grinding until stall detection calls it a rock.
POS_MAX  = POS_HOME + int(STROKE_CM * COUNTS_PER_CM)   # 3010

# Speed bytes for RPDO1 byte 3 (LINAK manual p.11)
SPEED_FULL = 0xCD   # ~2.18 cm/s
SPEED_HALF = 0x64   # ~1.09 cm/s
SPEED_DEFAULT = SPEED_HALF   # used when no speed specified (preserves slow drilling)

_POS_EPSILON  = 5    # position counts tolerance for "not moving" — 0.5 mm. A
                     # move in progress advances ~11 counts per 0.1 s poll even
                     # at SLOW, so this has ample margin against a false stall.

# Tolerance (in counts) when comparing live TPDO position against a target:
# 50 counts = 5 mm. Coupled to COUNTS_PER_CM — it has to be read in the same
# units, and the old 3000 was sized for the old 2136.83 scale. Left at 3000 here
# it would be 30 cm, i.e. the whole stroke, and every move would report DONE
# before it had moved at all.
#
# 5 mm is comfortably more than the ~2.2 mm the actuator covers between polls at
# FAST, so a move ends at most one poll early.
_TARGET_TOLERANCE = 50


def _rpdo(code: int, speed: int = SPEED_DEFAULT) -> list:
    """Build an 8-byte RPDO1 payload (LINAK manual p.11)."""
    return [
        code & 0xFF, (code >> 8) & 0xFF,
        0xFB,            # current  — default
        speed & 0xFF,    # speed byte (0xCD = MAX / 2.18 cm/s, 0x64 = HALF / 1.09 cm/s)
        0xFB,            # ramp up  — default
        0xFB,            # ramp dn  — default
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

    One move primitive: write an absolute position setpoint and watch TPDO until
    the reported position reaches it, stalls short of it, or an external STOP
    cancels the move. Every outcome carries the distance actually travelled.
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

        # Where the current/last move started, and where it has got to. Kept
        # separate from _tpdo_pos so travelled_cm() still answers correctly
        # after a move is cancelled midway.
        self._start_pos: Optional[int] = None
        self._live_pos:  Optional[int] = None
        self._travel_lock = threading.Lock()

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
        self._log.debug(f"[LINAK{lid}] Configure node 0x{nid:02X} ...")

        # ── Consumer heartbeat watchdog ───────────────────────────────────
        # Watchdog = 3× heartbeat period for OS scheduling jitter margin.
        # The heartbeat thread is already running and warm before this call.
        watchdog_ms = self._hb_ms * 3
        self._log.debug(
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

        self._log.debug(f"[LINAK{lid}] Configuration done.")

    def go_operational(self) -> None:
        """
        Phase 2 of init — called after ALL actuators are configured.

        Sends NMT OPERATIONAL then the initial STOP + CLEAR.
        Both actuators are brought up together so neither waits idle long
        enough to trip its heartbeat watchdog while the other is being set up.
        """
        lid = self.linak_id
        self._log.debug(f"[LINAK{lid}] NMT → OPERATIONAL ...")
        self.can_node.nmt.state = "OPERATIONAL"
        time.sleep(0.2)   # let the node settle

        self._send(CMD_STOP)
        time.sleep(1.0)
        self._send(CMD_CLEAR)
        time.sleep(1.0)

        self._log.debug(f"[LINAK{lid}] Ready.")

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

    def travelled_cm(self) -> float:
        """How far the current/last move actually moved, in cm."""
        with self._travel_lock:
            if self._start_pos is None or self._live_pos is None:
                return 0.0
            return abs(self._live_pos - self._start_pos) / COUNTS_PER_CM

    # ── Public commands ───────────────────────────────────────────────────

    def start_move(
        self,
        target_pos:    int,
        on_result:     Callable,
        speed:         int  = SPEED_DEFAULT,
        stall_is_done: bool = False,
    ) -> None:
        """
        Drive to absolute position <target_pos> (already clamped by the caller).

        The actuator servos itself there — we write the setpoint into the mapped
        'Actuator Command.Position' RPDO and the hardware does the rest. The poll
        loop only watches, confirms arrival and reports.

        Completion is reported through on_result(outcome, travelled_cm, detail)
        with outcome one of DONE / STALL / ERR. Set stall_is_done for moves whose
        intended end IS a hard stop (a HOME).
        """
        self._abort()
        self._cancel.clear()
        self._move_thread = threading.Thread(
            target=self._worker_move,
            args=(target_pos, on_result, speed, stall_is_done),
            daemon=True,
            name=f"linak{self.linak_id}_move",
        )
        self._move_thread.start()

    def stop(self) -> float:
        """
        Abort any running move and send STOP.

        Returns how far the aborted move travelled (cm) so the caller can
        retract exactly that much rather than guessing the commanded distance.
        """
        self._abort()
        try:
            self._send(CMD_STOP)
        except Exception as exc:
            self._log.warn(f"[LINAK{self.linak_id}] STOP send failed: {exc}")
        # Refresh from the newest TPDO sample — the actuator coasts a little
        # after CMD_STOP, and we want the position it actually ended at.
        pos = self.get_position()
        if pos is not None:
            with self._travel_lock:
                self._live_pos = pos
        return self.travelled_cm()

    # ── Internals ─────────────────────────────────────────────────────────

    def _send(self, code: int, speed: int = SPEED_DEFAULT) -> None:
        self._network.send_message(self.cob_rpdo1, _rpdo(code, speed))

    def _abort(self) -> None:
        """Signal the worker to exit and join it (max 3 s)."""
        self._cancel.set()
        if self._move_thread and self._move_thread.is_alive():
            self._move_thread.join(timeout=3.0)
        self._move_thread = None

    def _worker_move(
        self,
        target_pos:    int,
        on_result:     Callable,
        speed:         int,
        stall_is_done: bool,
    ) -> None:
        """
        Drive to absolute <target_pos>, comparing live TPDO position against it
        each poll.

        The setpoint goes to the actuator's own position servo, which is what
        actually stops the move — this loop watches and reports. (It still sends
        CMD_STOP on arrival to make the stop explicit rather than implied.)

        Three ways to finish, all reporting actual travel:
          • target reached            → DONE
          • stalled short of target   → STALL  (or DONE if stall_is_done,
                                        i.e. a HOME run onto the hard stop)
          • cancelled by stop()       → no report; stop() publishes STOPPED
        """
        POLL_INTERVAL        = 0.1
        TIMEOUT_S            = 60.0
        INITIAL_POS_TIMEOUT  = 3.0   # how long to wait for a first TPDO sample
        STALL_SAMPLES        = 10    # ~1 s with no motion → hit something

        try:
            # ── Wait for a first TPDO reading so we know where we started ───
            waited = 0.0
            start_pos = self.get_position()
            while start_pos is None and waited < INITIAL_POS_TIMEOUT:
                if self._cancel.is_set():
                    return
                time.sleep(POLL_INTERVAL)
                waited += POLL_INTERVAL
                start_pos = self.get_position()
            if start_pos is None:
                raise RuntimeError(
                    f"LINAK{self.linak_id}: no TPDO position received within "
                    f"{INITIAL_POS_TIMEOUT} s — cannot start move"
                )

            # Publish the origin so travelled_cm() is meaningful even if this
            # move is cancelled partway by stop().
            with self._travel_lock:
                self._start_pos = start_pos
                self._live_pos  = start_pos

            # Direction is implied by the setpoint, not commanded: whether this
            # is an extend or a retract is just which side of the target we
            # happen to be on.
            target = target_pos
            if target >= start_pos:
                reached = lambda p: p >= target - _TARGET_TOLERANCE
            else:
                reached = lambda p: p <= target + _TARGET_TOLERANCE

            self._log.debug(
                f"[LINAK{self.linak_id}] move: {start_pos} → {target} "
                f"({abs(target - start_pos) / COUNTS_PER_CM:.2f} cm, "
                f"speed=0x{speed:02X})"
            )
            self._send(target, speed)

            elapsed       = 0.0
            last_moving   = start_pos
            stall_samples = 0

            while not self._cancel.is_set():
                time.sleep(POLL_INTERVAL)
                elapsed += POLL_INTERVAL
                if elapsed >= TIMEOUT_S:
                    raise TimeoutError(
                        f"LINAK{self.linak_id} move timed out after {TIMEOUT_S} s"
                    )

                pos = self.get_position()
                if pos is None:
                    continue
                with self._travel_lock:
                    self._live_pos = pos

                if reached(pos):
                    self._send(CMD_STOP)
                    travelled = self.travelled_cm()
                    self._log.debug(
                        f"[LINAK{self.linak_id}] target reached at {pos} "
                        f"(target {target}, travelled {travelled:.2f} cm)"
                    )
                    on_result("DONE", travelled)
                    return

                # Stall detection — hard stop or mechanical obstruction
                if abs(pos - last_moving) > _POS_EPSILON:
                    last_moving   = pos
                    stall_samples = 0
                else:
                    stall_samples += 1
                if stall_samples >= STALL_SAMPLES:
                    self._send(CMD_STOP)
                    travelled = self.travelled_cm()
                    if stall_is_done:
                        self._log.debug(
                            f"[LINAK{self.linak_id}] reached hard stop at {pos} "
                            f"after {travelled:.2f} cm"
                        )
                        on_result("DONE", travelled)
                    else:
                        self._log.warn(
                            f"[LINAK{self.linak_id}] stalled at {pos} before "
                            f"reaching target {target} — travelled "
                            f"{travelled:.2f} of "
                            f"{abs(target - start_pos) / COUNTS_PER_CM:.2f} cm"
                        )
                        on_result("STALL", travelled)
                    return

            # Cancelled externally — stop() owns the STOPPED report.
            self._safe_stop()

        except Exception as exc:
            self._safe_stop()
            on_result("ERR", self.travelled_cm(), str(exc))


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
        self.get_logger().debug(f"Using EDS: {eds_file}")

        # HB_COB = 0x701 (node 0x01 heartbeat).
        # The actuator's consumer-heartbeat SDO (0x1016 sub1) is set to
        # (0x01 << 16) + period_ms, meaning "I expect heartbeats from node 1
        # on COB-ID 0x701."  This matches what the working unit test uses.
        HB_COB = 0x701

        # ── CAN network ───────────────────────────────────────────────────
        self.get_logger().debug(f"Connecting to '{channel}' @ {bitrate} bps ...")
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
        self.get_logger().debug("Heartbeat warm-up complete — starting actuator init ...")

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
        self.get_logger().debug("Shutting down linak_can_node ...")
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
        self.get_logger().debug(f"Master heartbeat started ({self._hb_ms} ms, COB 0x{self._hb_cob:03X})")
        while not self._stop_hb.is_set():
            try:
                self._network.send_message(self._hb_cob, [0x05])
            except Exception as exc:
                self.get_logger().warn(f"Heartbeat send error: {exc}")
            time.sleep(interval)

    # ── Status helpers ────────────────────────────────────────────────────────

    def _publish_status(self, token: str) -> None:
        self.get_logger().debug(f"→ /linak_status: {token}")
        self._status_pub.publish(String(data=token))

    def _result(self, linak_id: int, outcome: str, travelled_cm: float,
                detail: str = "") -> None:
        """
        Publish a move outcome. Every terminal token carries how far the
        actuator actually travelled, so the caller can undo exactly that much
        instead of assuming the commanded distance was achieved.

          DONE:LINAK<id>,<cm>     reached the commanded target
          STALL:LINAK<id>,<cm>    stopped short against something solid
          STOPPED:LINAK<id>,<cm>  aborted by an external STOP
          ERR:LINAK<id>:<detail>  move failed
        """
        if outcome == "ERR":
            self._publish_status(f"ERR:LINAK{linak_id}:{detail}")
        else:
            self._publish_status(f"{outcome}:LINAK{linak_id},{travelled_cm:.2f}")


    def _on_cmd(self, msg: String) -> None:
        """
        Accepted formats:
          LINAK,<id>,GOTO,<position_cm>[,FAST|SLOW]
          LINAK,<id>,HOME
          LINAK,<id>,DOWN,<distance_cm>[,FAST|SLOW]   (relative jog)
          LINAK,<id>,UP,<distance_cm>[,FAST|SLOW]     (relative jog)
          LINAK,<id>,STOP
        """
        raw = msg.data.strip()
        self.get_logger().debug(f"← /linak_cmd: '{raw}'")

        parts = [p.strip() for p in raw.split(",")]

        if len(parts) < 3 or parts[0].upper() != "LINAK":
            self.get_logger().error(
                f"Malformed command '{raw}'. "
                "Expected: LINAK,<id>,<GOTO|HOME|DOWN|UP|STOP>[,<cm>[,FAST|SLOW]]"
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

        # ── Moves ─────────────────────────────────────────────────────────
        if action in ("GOTO", "HOME", "IN_MAX", "DOWN", "UP"):
            if action in ("HOME", "IN_MAX"):
                # Full retraction onto the hard stop. Reaching the stop is the
                # point, so a stall here IS success.
                target_pos    = POS_HOME
                speed         = SPEED_FULL
                stall_is_done = True
            else:
                if len(parts) < 4:
                    unit = "position" if action == "GOTO" else "distance"
                    self.get_logger().error(
                        f"'{action}' requires a {unit} in cm. "
                        f"Expected: LINAK,{lid},{action},<{unit}_cm>[,FAST|SLOW]"
                    )
                    return
                try:
                    value_cm = float(parts[3])
                except ValueError:
                    self.get_logger().error(f"Invalid {action} value '{parts[3]}'")
                    return
                if value_cm < 0:
                    self.get_logger().error(
                        f"Value must be non-negative, got {value_cm}"
                    )
                    return

                # Optional 5th field: speed (FAST=0xCD, SLOW=0x64). Default SLOW,
                # preserving the pre-existing slow-drilling behaviour.
                speed = SPEED_DEFAULT
                if len(parts) >= 5 and parts[4]:
                    tag = parts[4].upper()
                    if tag == "FAST":
                        speed = SPEED_FULL
                    elif tag == "SLOW":
                        speed = SPEED_HALF
                    else:
                        self.get_logger().error(
                            f"Invalid speed tag '{parts[4]}' — expected FAST or SLOW"
                        )
                        return
                stall_is_done = False

                counts = int(round(value_cm * COUNTS_PER_CM))
                if action == "GOTO":
                    # Absolute depth from home. Where the actuator happens to be
                    # is irrelevant: GOTO 30 means "end up at 30 cm", whether
                    # that is a 30 cm move or a 2 cm one.
                    target_pos = POS_HOME + counts
                else:
                    # DOWN / UP are relative jogs, kept for manual testing.
                    here = ch.get_position()
                    if here is None:
                        self.get_logger().error(
                            f"[LINAK{lid}] no position feedback yet — cannot "
                            f"compute a relative {action}. Use GOTO or HOME."
                        )
                        return
                    target_pos = here + counts if action == "DOWN" else here - counts

            # One clamp for every path. This is what stops a too-deep command
            # from parking against the hard stop until stall detection reports
            # it as an obstacle.
            clamped = max(POS_HOME, min(target_pos, POS_MAX))
            if clamped != target_pos:
                self.get_logger().warn(
                    f"[LINAK{lid}] target {target_pos} outside "
                    f"[{POS_HOME}, {POS_MAX}] — clamped to {clamped} "
                    f"({(clamped - POS_HOME) / COUNTS_PER_CM:.2f} cm)"
                )
            target_pos = clamped

            self.get_logger().debug(
                f"[LINAK{lid}] {action} → pos {target_pos} "
                f"({(target_pos - POS_HOME) / COUNTS_PER_CM:.2f} cm from home, "
                f"speed=0x{speed:02X})"
            )
            self._publish_status(f"ACK:LINAK{lid}")
            ch.start_move(
                target_pos    = target_pos,
                on_result     = lambda outcome, cm, detail="", lid=lid: (
                    self._result(lid, outcome, cm, detail)
                ),
                speed         = speed,
                stall_is_done = stall_is_done,
            )

        # ── Immediate stop — reports how far the aborted move actually got ──
        elif action == "STOP":
            travelled = ch.stop()
            self.get_logger().debug(
                f"[LINAK{lid}] STOP after {travelled:.2f} cm"
            )
            self._publish_status(f"STOPPED:LINAK{lid},{travelled:.2f}")

        else:
            self.get_logger().error(
                f"Unknown action '{action}'. "
                f"Valid: GOTO, HOME, DOWN, UP, STOP"
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