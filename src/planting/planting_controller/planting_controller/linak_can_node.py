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
  READY:LINAK               — EVERY actuator initialised (published once on
                              startup; absent if any of them is unavailable)
  ACK:LINAK<id>             — command accepted
  DONE:LINAK<id>,<cm>       — reached the commanded target
  STALL:LINAK<id>,<cm>      — stopped short against something solid
  STOPPED:LINAK<id>,<cm>    — aborted by an external STOP
  ERR:LINAK<id>:<detail>    — exception during move, or that actuator failed to
                              initialise / was commanded while unavailable

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
  9. Bring-up is per-actuator fault-isolated, and says so out loud. On the 10/6
     field test one actuator never answered its first SDO (0x1016 sub1); the
     SdoCommunicationError propagated out of __init__, the process exited 1,
     serial_bridge was left running against a live Arduino, and planting_fsm
     never started because the launch file's "both actuators initialised" gate
     never printed. Worse, every bring-up log line was DEBUG while the launch
     runs at INFO, so the log held no node output at all — there was no way to
     tell whether the dead node had been 0x20 or 0x21. Now: startup logging is
     INFO and names the node id at each step, each startup SDO is retried
     (_SDO_ATTEMPTS) before being believed, and a node that still does not
     answer disables only its own channel. READY:LINAK and the launch gate
     string are published only when every actuator came up, so a degraded node
     stays alive for diagnosis without letting the FSM run a sequence it
     cannot complete.
 10. "No SDO response received" is no longer a dead end. That one line was
     where six unrelated faults converged, and the 10/6 log could not tell them
     apart. Now, before the first SDO is sent:
       • the interface is described from sysfs and `ip` — driver, link state,
         CAN state, real bitrate, restart-ms — plus the OTHER CAN interfaces
         present, because the Kvaser commonly enumerates as can0 while
         everything here defaults to can1, and an interface that is UP and
         accepts writes says nothing about what is on it. A bus-off interface
         and a wrong bitrate are each called out by name (python-can IGNORES
         the bitrate passed to socketcan — it comes from `ip link`).
       • each actuator must be HEARD before it is asked anything. A powered
         LINAK emits TPDO1 / heartbeat / boot-up unprompted, so a node that is
         silent for _PRESENCE_WINDOW is absent from the bus, which is a
         different fault from one that is slow to answer SDO — and the log now
         says which, naming the node id.
       • EMCY frames are decoded and logged, so 0x8130 (consumer-heartbeat
         timeout) is visible as the cause it is.
     And the watchdog this node arms is DISARMED on shutdown, while the
     heartbeat is still flowing. Leaving it armed is how a Ctrl+C'd run
     poisons the next one: the node keeps waiting for a 0x701 that will never
     come, EMCYs, and refuses the next launch's first SDO. The same fix is in
     unit_test/can_tests/can_tests.py and unit_test/fsr/planting_fsr_test.py,
     which both armed it and exited.

     unit_test/can_tests/can_bus_check.py does all of the above from a bare
     shell, with no ROS and without moving an actuator.
"""

import json
import os
import subprocess
import threading
import time
from typing import Callable, Optional

import canopen
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# The two SDO failure modes mean opposite things at startup and are handled
# differently, so they are imported by name:
#   SdoCommunicationError — nothing answered on 0x600+node_id. The node is
#                           absent, unpowered, or still booting. Worth retrying.
#   SdoAbortedError       — the node answered and rejected the request. Retrying
#                           the identical write gets the identical rejection.
# The module path is the one from the 10/6 field-test traceback; older canopen
# releases expose both at top level.
try:
    from canopen.sdo.exceptions import SdoAbortedError, SdoCommunicationError
except ImportError:                                   # older canopen layout
    from canopen import SdoAbortedError, SdoCommunicationError

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

# How long to listen for an actuator to speak for itself before deciding it is
# not on the bus. TPDO1 arrives roughly every 250 ms by default, so 1.5 s is six
# chances — and it is spent only once, as a shared window across all actuators.
_PRESENCE_WINDOW = 1.5

# The EMCY codes worth naming in the log. 0x8130 is the one that explains an
# actuator which worked minutes ago and does not now.
_EMCY_TEXT = {
    0x0000: "error cleared",
    0x8110: "CAN controller overrun",
    0x8120: "CAN in error-passive",
    0x8130: "consumer-heartbeat timeout — this node was armed (0x1016) by an "
            "earlier master that then stopped sending 0x701, so it dropped to "
            "PRE-OPERATIONAL and latched an error. Power-cycle the actuator",
    0x8140: "recovered from bus-off",
    0x8150: "CAN-ID collision",
}


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


# ── SocketCAN interface diagnostics ──────────────────────────────────────────
#
# Three of the ways this node can fail to reach an actuator are invisible in its
# own log, and all three look exactly like "No SDO response received":
#
#   wrong interface  — an interface that is UP and accepts writes says nothing
#                      about what is on it. A frame written to a bus with no
#                      responder is queued and retried by the controller; the
#                      write() still succeeds. The Kvaser commonly enumerates as
#                      can0 while everything in this repo defaults to can1.
#   wrong bitrate    — python-can's socketcan backend does NOT apply the
#                      bitrate= it is handed. Bitrate belongs to `ip link`; the
#                      kwarg is accepted and ignored. So "Connecting to 'can1' @
#                      125000 bps" gets printed whatever the wire is running at,
#                      and a bus brought up with a bare
#                      `ip link set can1 up type can` logs identically to a
#                      correct one.
#   BUS-OFF          — an interface driven bus-off by repeated unacknowledged
#                      transmission still reports UP, and still accepts writes
#                      that never reach the wire. Without `restart-ms` it stays
#                      that way until someone cycles the link by hand.
#
# So the real state is read out of sysfs and `ip` and logged at INFO before any
# of it can be mistaken for a dead actuator.

_ARPHRD_CAN = 280          # /sys/class/net/<if>/type for a CAN interface


def _list_can_ifaces() -> list:
    """Every CAN interface present on this machine, by name."""
    found = []
    try:
        for name in sorted(os.listdir("/sys/class/net")):
            try:
                with open(f"/sys/class/net/{name}/type") as fh:
                    if int(fh.read().strip()) == _ARPHRD_CAN:
                        found.append(name)
            except (OSError, ValueError):
                continue
    except OSError:
        pass                                   # not Linux, or no sysfs
    return found


def _iface_driver(name: str) -> str:
    """Kernel driver behind an interface — 'kvaser_usb' for the Leaf Light."""
    link = f"/sys/class/net/{name}/device/driver"
    if not os.path.exists(link):
        return "none (virtual interface — vcan?)"
    return os.path.basename(os.path.realpath(link))


def _can_link_details(name: str) -> dict:
    """
    CAN link state from `ip -details link show`, or {} if it cannot be read.

    Everything here is best-effort diagnostics: a missing `ip`, an iproute2 too
    old for -json, or an unexpected schema must never stop the node from coming
    up, so every failure returns what was parsed so far.
    """
    info: dict = {}
    try:
        proc = subprocess.run(
            ["ip", "-details", "-json", "link", "show", name],
            capture_output=True, text=True, timeout=2.0, check=False,
        )
        if proc.returncode != 0 or not proc.stdout.strip():
            return info
        entry = json.loads(proc.stdout)[0]
        info["link"] = entry.get("operstate")
        data = (entry.get("linkinfo") or {}).get("info_data") or {}
        info["state"]      = data.get("state")
        info["restart_ms"] = data.get("restart_ms")
        info["bitrate"]    = (data.get("bittiming") or {}).get("bitrate")
        info["berr"]       = data.get("berr_counter") or {}
    except Exception:
        pass
    return info


def _report_can_iface(channel: str, want_bitrate: int, log) -> None:
    """
    Log what `channel` actually is, and warn about each way it can lie.

    Called before connect(), so that a log which ends in "No SDO response"
    already contains the evidence for whether the bus was ever usable.
    """
    ifaces = _list_can_ifaces()
    if not ifaces:
        log.warn(
            "No CAN interfaces found in /sys/class/net — if this is the Jetson, "
            "the adapter is not plugged in or its driver did not load "
            "(`dmesg | grep -i kvaser`)."
        )
        return

    if channel not in ifaces:
        log.error(
            f"CAN interface '{channel}' does not exist. Present: "
            f"{', '.join(ifaces)}. Relaunch with can_channel:=<one of those>."
        )
        return

    d = _can_link_details(channel)
    log.info(
        f"CAN iface '{channel}': driver={_iface_driver(channel)}, "
        f"link={d.get('link', '?')}, state={d.get('state', '?')}, "
        f"bitrate={d.get('bitrate', '?')}, restart-ms={d.get('restart_ms', '?')}"
    )

    bitrate = d.get("bitrate")
    if bitrate is not None and bitrate != want_bitrate:
        log.error(
            f"'{channel}' is running at {bitrate} bps but the LINAKs need "
            f"{want_bitrate}. python-can IGNORES the bitrate passed here — it "
            f"comes from `ip link`. Fix it: `sudo ip link set {channel} down && "
            f"sudo ip link set {channel} up type can bitrate {want_bitrate} "
            f"restart-ms 100`."
        )

    state = (d.get("state") or "").upper()
    if state in ("BUS-OFF", "STOPPED"):
        log.error(
            f"'{channel}' is {state} — it is UP and will accept every frame "
            f"this node writes, and none of them will reach the wire. "
            f"Cycle it: `sudo ip link set {channel} down && sudo ip link set "
            f"{channel} up type can bitrate {want_bitrate} restart-ms 100`."
        )
    elif state == "ERROR-PASSIVE":
        log.warn(
            f"'{channel}' is ERROR-PASSIVE — it is seeing transmit errors, "
            f"usually nothing acknowledging on the bus (unpowered actuators) or "
            f"missing 120 Ω termination. It is one step from BUS-OFF."
        )

    berr = d.get("berr") or {}
    if berr.get("tx") or berr.get("rx"):
        log.warn(
            f"'{channel}' bus error counters: tx={berr.get('tx')}, "
            f"rx={berr.get('rx')} — the bus is not clean."
        )

    if d.get("restart_ms") == 0:
        log.info(
            f"'{channel}' has restart-ms 0, so a bus-off will NOT self-recover. "
            f"Bringing it up with `restart-ms 100` makes that failure "
            f"transient instead of permanent."
        )

    others = [i for i in ifaces if i != channel]
    if others:
        log.info(
            "Other CAN interfaces on this machine: "
            + ", ".join(f"{i} (driver={_iface_driver(i)})" for i in others)
            + f" — if nothing answers on '{channel}', the actuators are "
              "probably on one of these. `candump <iface>` settles it."
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
        # Everything this node emits unprompted, used to tell "absent from the
        # bus" from "slow to answer an SDO" before the first SDO is sent.
        self.cob_emcy  = 0x080 + node_id   # emergency
        self.cob_hb    = 0x700 + node_id   # heartbeat / boot-up
        self._network  = network
        self._hb_ms    = hb_period_ms
        self._log      = logger

        # Presence: set by _on_any as soon as anything arrives from this node.
        self._seen      = threading.Event()
        self._seen_cobs: set = set()

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
    #
    # Startup logs are INFO, not DEBUG. The launch default verbosity is INFO,
    # so with these at DEBUG the 10/6 field-test log contained no node output
    # at all before the traceback — there was no way to tell from it whether
    # the SDO that timed out had been addressed to 0x20 or to 0x21. Every line
    # below names the actuator and its node id for exactly that reason.
    # Per-move and per-command logging stays at DEBUG; that is runtime chatter,
    # not bring-up state.

    # How hard to try each startup SDO before declaring the actuator absent.
    # canopen's SdoClient gives up after RESPONSE_TIMEOUT (0.3 s) and one
    # retry, which is not much patience for a node that is still booting.
    _SDO_ATTEMPTS    = 3
    _SDO_RETRY_DELAY = 0.25

    # ── Presence, before anything is asked of the node ────────────────────

    def start_listening(self) -> None:
        """
        Subscribe to every COB-ID this node emits without being asked.

        A powered LINAK on the right bus at the right bitrate talks on its own:
        TPDO1 roughly every 250 ms, plus a heartbeat and a boot-up frame. None
        of that needs an SDO, a mapping, or OPERATIONAL — which makes it the
        cheapest possible proof that the actuator is reachable at all.
        """
        for cob in (self.cob_tpdo1, self.cob_hb, self.cob_emcy):
            self._network.subscribe(cob, self._on_any)
        self._network.subscribe(self.cob_emcy, self._on_emcy)

    def _on_any(self, can_id: int, data: bytes, timestamp: float) -> None:
        self._seen_cobs.add(can_id)
        self._seen.set()

    def _on_emcy(self, can_id: int, data: bytes, timestamp: float) -> None:
        """
        Log any EMCY this node broadcasts, decoded.

        0x8130 is the one worth watching for: it means this actuator's
        consumer-heartbeat watchdog fired, which it does whenever a previous
        master (a finished unit-test run, a Ctrl+C'd launch) armed 0x1016 and
        then stopped sending 0x701. The node drops to PRE-OPERATIONAL and
        latches an error, and the next process to come along finds it
        unresponsive for reasons that have nothing to do with wiring.
        """
        if len(data) < 2:
            return
        code = data[0] | (data[1] << 8)
        reg  = data[2] if len(data) > 2 else 0
        text = _EMCY_TEXT.get(code, "see the LINAK CANopen manual")
        line = (f"[LINAK{self.linak_id}] EMCY from node "
                f"0x{self.can_node.id:02X}: 0x{code:04X} ({text}), "
                f"error register 0x{reg:02X}")
        if code == 0:
            self._log.info(line)          # 0x0000 = error cleared
        else:
            self._log.warn(line)

    def wait_for_presence(self, timeout: float) -> bool:
        """
        True if this node said anything at all within `timeout`.

        This is the check the 10/6 field test did not have. One actuator never
        answered its first SDO, the SdoCommunicationError killed the process,
        and because every bring-up line was DEBUG the log could not say whether
        the silent node had been 0x20 or 0x21 — or whether the bus had ever
        worked. Listening first separates the two: a node heard here but failing
        SDO is a different problem from a node that was never on the bus.
        """
        lid = self.linak_id
        nid = self.can_node.id
        if self._seen.wait(timeout):
            cobs = ", ".join(f"0x{c:03X}" for c in sorted(self._seen_cobs))
            self._log.info(
                f"[LINAK{lid}] node 0x{nid:02X} is alive on the bus "
                f"(heard {cobs})."
            )
            return True
        # Not fatal: on 10/25 both LINAKs sat silent until asked (heartbeat
        # producer off, still PRE-OPERATIONAL) yet answered SDO at once. The
        # SDO retries in configure() are what decide absence.
        self._log.warn(
            f"[LINAK{lid}] node 0x{nid:02X} sent nothing unprompted on "
            f"0x{self.cob_tpdo1:03X} (TPDO1), 0x{self.cob_hb:03X} (heartbeat) "
            f"or 0x{self.cob_emcy:03X} (EMCY) in {_PRESENCE_WINDOW:.1f} s. "
            f"Trying SDO anyway; if that fails too, check its power and CAN "
            f"wiring and confirm its node id is really 0x{nid:02X}."
        )
        return False

    def _sdo_with_retry(self, what: str, write: Callable[[], None]) -> None:
        """
        Run one startup SDO write, retrying a no-response timeout.

        Only the last attempt's exception propagates, and LinakDriver turns
        that into a disabled channel rather than a dead process. `write` must
        be idempotent — every retry re-runs it from the start.
        """
        lid = self.linak_id
        nid = self.can_node.id
        for attempt in range(1, self._SDO_ATTEMPTS + 1):
            try:
                write()
                return
            except SdoCommunicationError as exc:
                # Nothing answered. canopen logs its own abort (0x05040000,
                # "SDO protocol timed out") just above this — note that abort
                # is sent BY US, it is not a rejection from the actuator.
                self._log.warn(
                    f"[LINAK{lid}] {what}: no SDO response from node "
                    f"0x{nid:02X} (attempt {attempt}/{self._SDO_ATTEMPTS}) — "
                    f"{exc}"
                )
                if attempt == self._SDO_ATTEMPTS:
                    raise
                time.sleep(self._SDO_RETRY_DELAY)
            except SdoAbortedError as exc:
                self._log.error(
                    f"[LINAK{lid}] {what}: node 0x{nid:02X} rejected it — {exc}"
                )
                raise

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

        Raises on an unreachable node — the caller decides whether that is
        fatal. It is not: see LinakDriver._disable().
        """
        lid = self.linak_id
        nid = self.can_node.id
        self._log.info(f"[LINAK{lid}] configuring node 0x{nid:02X} ...")

        # ── Consumer heartbeat watchdog ───────────────────────────────────
        # Watchdog = 3× heartbeat period for OS scheduling jitter margin.
        # The heartbeat thread is already running and warm before this call.
        watchdog_ms = self._hb_ms * 3
        self._log.info(
            f"[LINAK{lid}] HB watchdog: period={self._hb_ms} ms, "
            f"timeout={watchdog_ms} ms"
        )

        def _arm_watchdog() -> None:
            self.can_node.sdo[0x1016][1].raw = (0x01 << 16) + watchdog_ms

        self._sdo_with_retry("arm consumer-heartbeat watchdog 0x1016 sub1",
                             _arm_watchdog)
        time.sleep(0.1)

        # ── RPDO1 mapping ─────────────────────────────────────────────────
        def _map_rpdo1() -> None:
            rpdo = self.can_node.rpdo[1]
            rpdo.clear()
            rpdo.add_variable("Actuator Command.Position")
            rpdo.enabled = True
            rpdo.save()

        self._sdo_with_retry("map RPDO1", _map_rpdo1)
        time.sleep(0.1)

        # ── TPDO1 subscription for position feedback ──────────────────────
        self._network.subscribe(self.cob_tpdo1, self._on_tpdo)

        self._log.info(f"[LINAK{lid}] node 0x{nid:02X} configured.")

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

        self._log.info(f"[LINAK{lid}] ready.")

    def initialize(self) -> None:
        """Convenience wrapper — configure then go_operational (single-node use)."""
        self.configure()
        self.go_operational()

    def disarm_watchdog(self) -> None:
        """
        Tell this actuator to stop expecting a master heartbeat.

        configure() arms its consumer-heartbeat watchdog (0x1016 sub1) at 3x the
        heartbeat period. If this process exits with that still armed, the
        heartbeat stops while the node is still waiting for it: the node fires
        EMCY 0x8130, drops to PRE-OPERATIONAL and latches an error, and the NEXT
        launch gets 'No SDO response received' from an actuator that is powered,
        wired and on the right bus. Ctrl+C on a launch is routine, so without
        this every restart is a coin toss.

        Must run while the heartbeat thread is still alive — see
        LinakDriver.shutdown().
        """
        try:
            self.can_node.sdo[0x1016][1].raw = 0     # 0 = expect no heartbeat
            time.sleep(0.05)
            self._log.info(
                f"[LINAK{self.linak_id}] heartbeat watchdog disarmed."
            )
        except Exception as exc:
            self._log.warn(
                f"[LINAK{self.linak_id}] could not disarm the heartbeat "
                f"watchdog ({exc}). If the next launch cannot reach this "
                f"actuator, power-cycle it."
            )

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

        self._channel = channel          # kept for the diagnostics in
                                         # _report_startup()

        eds_file = _resolve_eds(eds_param)
        self.get_logger().info(f"Using EDS: {eds_file}")

        # HB_COB = 0x701 (node 0x01 heartbeat).
        # The actuator's consumer-heartbeat SDO (0x1016 sub1) is set to
        # (0x01 << 16) + period_ms, meaning "I expect heartbeats from node 1
        # on COB-ID 0x701."  This matches what the working unit test uses.
        HB_COB = 0x701

        # ── CAN network ───────────────────────────────────────────────────
        # No bus means nothing to degrade to, so this one stays fatal — but it
        # says why before it dies, rather than leaving a bare socket traceback
        # in the launch log.
        # What the interface actually is, before anything can be blamed on the
        # actuators. See _report_can_iface: wrong iface, wrong bitrate and
        # bus-off are all indistinguishable from a dead node further down.
        _report_can_iface(channel, bitrate, self.get_logger())

        self.get_logger().info(f"Connecting to '{channel}' @ {bitrate} bps ...")
        self._network = canopen.Network()
        try:
            self._network.connect(channel=channel, bustype="socketcan",
                                  bitrate=bitrate)
            # Prime the heartbeat before any SDO traffic so the actuator
            # doesn't report a consumer-heartbeat timeout during init.
            self._network.send_message(HB_COB, [0x05])
        except Exception as exc:
            self.get_logger().error(
                f"Cannot use SocketCAN interface '{channel}': {exc}. "
                f"Check it exists and is up: "
                f"`ip -details link show {channel}`, and if it is down "
                f"`sudo ip link set {channel} up type can bitrate {bitrate}`."
            )
            raise

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

        # ── Status publisher ──────────────────────────────────────────────
        # Created BEFORE actuator init so a startup failure can be published as
        # ERR:LINAK<n> instead of only appearing in the log. /linak_cmd is
        # subscribed after init, so no command can arrive before the channels
        # it would address exist.
        self._status_pub = self.create_publisher(String, "/linak_status", 10)

        # ── Actuator channels ─────────────────────────────────────────────
        # Construction only parses the EDS and registers the node — no bus
        # traffic, so a failure here is a bad install affecting both actuators
        # equally, and stays fatal.
        self._node_ids: dict[int, int] = {1: node_id_1, 2: node_id_2}
        self._ch: dict[int, ActuatorChannel] = {
            lid: ActuatorChannel(lid, nid, eds_file, self._network, hb_ms,
                                 self.get_logger())
            for lid, nid in self._node_ids.items()
        }
        # Channels that did not come up, keyed by id → why. _disable() moves a
        # channel from _ch to here; _ch only ever holds usable actuators.
        self._failed: dict[int, str] = {}

        # ── Phase 0: is anybody out there? ────────────────────────────────
        # Listen before asking. This is diagnostic only: a LINAK that is still
        # PRE-OPERATIONAL with its heartbeat producer off says nothing until
        # it is addressed, so silence here is logged, not acted on. The SDO
        # retries in Phase 1 decide whether an actuator is really absent.
        for ch in self._ch.values():
            ch.start_listening()

        self.get_logger().info(
            f"Listening {_PRESENCE_WINDOW:.1f} s for the actuators to "
            f"announce themselves ..."
        )
        deadline = time.monotonic() + _PRESENCE_WINDOW
        heard_any = False
        for ch in self._ch.values():
            remaining = max(0.0, deadline - time.monotonic())
            heard_any |= ch.wait_for_presence(remaining)

        if not heard_any:
            self.get_logger().warn(
                f"Nothing unprompted on '{channel}' — continuing to SDO. If "
                f"both actuators then fail, rule out the bus: `candump "
                f"{channel}`, `ip -details -statistics link show {channel}` "
                f"(BUS-OFF? bitrate {bitrate}?), or run "
                f"unit_test/can_tests/can_bus_check.py."
            )

        # ── Two-phase init ────────────────────────────────────────────────
        # Each phase is per-channel fault-isolated: on 10/6 a single
        # unanswered SDO propagated out of __init__ and killed the process,
        # which left serial_bridge running against a live Arduino and stalled
        # the whole launch sequence. One absent actuator now disables that
        # actuator, not the node.
        #
        # Phase 1: configure ALL nodes (SDO, RPDO mapping) while they stay in
        # PRE-OPERATIONAL.  Neither node's heartbeat watchdog can fire here
        # because they are not yet armed with an NMT OPERATIONAL state.
        for lid, ch in list(self._ch.items()):
            try:
                ch.configure()
            except Exception as exc:
                self._disable(lid, f"configure failed: {exc}")

        # Phase 2: bring the surviving nodes OPERATIONAL together.
        # Doing this sequentially (node1 then node2) is fine now because the
        # watchdog timeout is 3× the HB period — 300 ms of margin easily
        # covers the ~1.2 s STOP+CLEAR sequence for the other node.
        for lid, ch in list(self._ch.items()):
            try:
                ch.go_operational()
            except Exception as exc:
                self._disable(lid, f"go_operational failed: {exc}")

        # ── ROS I/O ───────────────────────────────────────────────────────
        self.create_subscription(String, "/linak_cmd", self._on_cmd, 10)

        self._report_startup()

    # ── Startup outcome ───────────────────────────────────────────────────────

    def _disable(self, lid: int, reason: str) -> None:
        """
        Drop one actuator from service without taking the node down.

        The channel object is left alive but unreachable from _ch, so nothing
        can command it. If it already subscribed to its TPDO the callback keeps
        updating a position nobody reads, which is harmless and cheaper than
        unsubscribing through a canopen API that has moved between releases.
        """
        self._ch.pop(lid, None)
        self._failed[lid] = reason
        self.get_logger().error(
            f"[LINAK{lid}] DISABLED (node 0x{self._node_ids[lid]:02X}) — {reason}"
        )
        self._publish_status(f"ERR:LINAK{lid}:{reason}")

    def _report_startup(self) -> None:
        """
        Announce the startup outcome — and gate the rest of the launch on it.

        planting_bringup.launch.py starts planting_fsm when it sees "both
        actuators initialised" on this node's output, so that string is printed
        only when every actuator really did come up. A degraded node stays
        alive and diagnosable (its working actuator can still be driven by hand
        over /linak_cmd) but will not let the FSM start a sequence whose
        CHUTE_DOWN step has nothing to drive.
        """
        if not self._failed:
            self._publish_status("READY:LINAK")
            self.get_logger().info(
                "linak_can_node ready — both actuators initialised."
            )
            return

        dead = ", ".join(
            f"LINAK{lid} (node 0x{self._node_ids[lid]:02X})"
            for lid in sorted(self._failed)
        )
        live = ", ".join(
            f"LINAK{lid} (node 0x{self._node_ids[lid]:02X})"
            for lid in sorted(self._ch)
        ) or "none"
        self.get_logger().error(
            f"linak_can_node DEGRADED — unavailable: {dead}; usable: {live}. "
            "The planting sequence will NOT be started. Diagnose it in one "
            "pass, without ROS and without moving anything: "
            "`python3 unit_test/can_tests/can_bus_check.py --channel "
            f"{self._channel}` — it separates an absent actuator from a wrong "
            "interface, a bus-off, a wrong bitrate, a latched EMCY and a "
            "second master still on the bus."
        )
        if not self._ch:
            self.get_logger().error(
                "No actuator is usable — this node is up for diagnostics only."
            )

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        self.get_logger().info("Shutting down linak_can_node ...")
        for ch in self._ch.values():
            ch.stop()
        # Leave every actuator in a state the next launch can talk to: disarm
        # the heartbeat watchdogs while the heartbeat is still flowing, and only
        # then stop sending it. Reversing these two lines re-creates exactly the
        # 'worked ten minutes ago, dead now' failure this node hit on 10/6.
        for ch in self._ch.values():
            ch.disarm_watchdog()
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

        if lid in self._failed:
            # Distinguish "there is no such actuator" from "that actuator
            # never came up" — otherwise a disabled channel looks like a typo
            # in the command.
            self.get_logger().error(
                f"[LINAK{lid}] unavailable — it failed to initialise at "
                f"startup ({self._failed[lid]}). Restart the node once the "
                f"actuator answers on the bus."
            )
            self._publish_status(f"ERR:LINAK{lid}:unavailable")
            return

        if lid not in self._ch:
            self.get_logger().error(
                f"Unknown actuator id {lid}. Known ids: {sorted(self._ch)}"
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