#!/usr/bin/env python3
"""
can_bus_check.py — read-only CAN bus diagnostic for the two LINAK actuators.

NOTHING HERE MOVES AN ACTUATOR. It reads the interface state out of the kernel,
listens to the bus, and (optionally) reads one read-only CANopen object per node
id. No RPDO is sent, no position setpoint, no NMT state change, and the only SDO
traffic is an upload (a read) of 0x1000 Device Type. Safe to run with the drill
in the ground and people standing next to it.

Why this exists
───────────────
On the 10/6 field test, `ros2 launch planting_controller planting_bringup.launch.py`
died with:

    canopen.sdo.exceptions.SdoCommunicationError: No SDO response received

on the very first SDO of bring-up (0x1016 sub1). That one line is where at least
six different faults converge, and the log could not tell them apart:

  1. one actuator absent from the bus — unpowered, unplugged, or not the node id
     we think (the chute, 0x21, has never been exercised by any passing test;
     can_tests.py is hard-coded to 0x20 and planting_fsr_test.py defaults to it)
  2. wrong interface — the Kvaser Leaf Light commonly enumerates as can0 while
     everything in this repo defaults to can1. An interface that is UP and
     accepts writes says nothing about what is on it
  3. the actuator left in a latched error by the previous run: arm its
     consumer-heartbeat watchdog (0x1016), stop sending 0x701, and it EMCYs
     0x8130, drops to PRE-OPERATIONAL and stops cooperating
  4. the interface in BUS-OFF — still reports UP, still accepts every write,
     and none of them reach the wire
  5. wrong bitrate — python-can IGNORES the bitrate it is handed for socketcan;
     the real one comes from `ip link`, so the log reads correct either way
  6. a second CANopen master still running (a leftover unit test or launch),
     fighting over NMT state and heartbeats

This script answers all six in one pass, and prints which one it found.

Usage
─────
  python3 can_bus_check.py                        # can1, nodes 0x20 and 0x21
  python3 can_bus_check.py --channel can0         # check the other interface
  python3 can_bus_check.py --listen 10            # longer passive window
  python3 can_bus_check.py --scan                 # probe node ids 1..127
  python3 can_bus_check.py --no-probe             # passive only, send nothing

Exit status is 0 only if every requested node id was found.
"""

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from collections import Counter

# ── Defaults (match linak_can_node.py / the launch file) ─────────────────────
CHANNEL  = 'can1'
BITRATE  = 125000
NODE_IDS = [0x20, 0x21]          # 0x20 auger (LINAK 1), 0x21 chute (LINAK 2)
HB_COB   = 0x701                 # master heartbeat the actuators' 0x1016 expects
LISTEN_S = 3.0

_EMCY_TEXT = {
    0x0000: 'error cleared',
    0x8110: 'CAN controller overrun',
    0x8120: 'CAN in error-passive',
    0x8130: 'CONSUMER-HEARTBEAT TIMEOUT — armed by an earlier master that then '
            'stopped sending 0x701',
    0x8140: 'recovered from bus-off',
    0x8150: 'CAN-ID collision',
}

_NMT_STATE = {
    0x00: 'BOOT-UP',
    0x04: 'STOPPED',
    0x05: 'OPERATIONAL',
    0x7F: 'PRE-OPERATIONAL',
}


# ── Interface state ───────────────────────────────────────────────────────────
#
# Deliberately self-contained — the whole point of this script is that it runs
# on a bare Jetson shell with no ROS workspace sourced. linak_can_node.py has
# the same three helpers for its own startup log.

_ARPHRD_CAN = 280                # /sys/class/net/<if>/type for a CAN interface


def list_can_ifaces():
    found = []
    try:
        for name in sorted(os.listdir('/sys/class/net')):
            try:
                with open(f'/sys/class/net/{name}/type') as fh:
                    if int(fh.read().strip()) == _ARPHRD_CAN:
                        found.append(name)
            except (OSError, ValueError):
                continue
    except OSError:
        pass
    return found


def iface_driver(name):
    link = f'/sys/class/net/{name}/device/driver'
    if not os.path.exists(link):
        return 'none (virtual — vcan?)'
    return os.path.basename(os.path.realpath(link))


def can_link_details(name):
    info = {}
    try:
        proc = subprocess.run(
            ['ip', '-details', '-json', 'link', 'show', name],
            capture_output=True, text=True, timeout=2.0, check=False)
        if proc.returncode != 0 or not proc.stdout.strip():
            return info
        entry = json.loads(proc.stdout)[0]
        info['link'] = entry.get('operstate')
        data = (entry.get('linkinfo') or {}).get('info_data') or {}
        info['state']      = data.get('state')
        info['restart_ms'] = data.get('restart_ms')
        info['bitrate']    = (data.get('bittiming') or {}).get('bitrate')
        info['berr']       = data.get('berr_counter') or {}
    except Exception:
        pass
    return info


def step_interfaces(channel, bitrate):
    """Print every CAN interface and verdict on the chosen one. → ok?"""
    print('── 1. Interfaces ' + '─' * 58)
    ifaces = list_can_ifaces()
    if not ifaces:
        print('  ✗ No CAN interfaces exist at all (/sys/class/net).')
        print('    The adapter is unplugged or its driver did not load:')
        print('      dmesg | grep -i kvaser')
        return False

    for name in ifaces:
        d = can_link_details(name)
        mark = '→' if name == channel else ' '
        print(f'  {mark} {name:6s} driver={iface_driver(name):24s} '
              f'link={str(d.get("link")):6s} state={str(d.get("state")):14s} '
              f'bitrate={d.get("bitrate")} restart-ms={d.get("restart_ms")}')

    if channel not in ifaces:
        print(f'\n  ✗ CAUSE 2: \'{channel}\' does not exist. '
              f'Present: {", ".join(ifaces)}.')
        print(f'    Re-run against one of those, and launch with '
              f'can_channel:=<name>.')
        return False

    ok = True
    d = can_link_details(channel)

    if d.get('bitrate') is not None and d['bitrate'] != bitrate:
        print(f'\n  ✗ CAUSE 5: \'{channel}\' is at {d["bitrate"]} bps, the '
              f'LINAKs need {bitrate}.')
        print(f'    python-can ignores the bitrate= passed by the node — it '
              f'comes from `ip link`:')
        print(f'      sudo ip link set {channel} down')
        print(f'      sudo ip link set {channel} up type can bitrate {bitrate} '
              f'restart-ms 100')
        ok = False

    state = (d.get('state') or '').upper()
    if state in ('BUS-OFF', 'STOPPED'):
        print(f'\n  ✗ CAUSE 4: \'{channel}\' is {state}. It still reports UP '
              f'and still accepts every')
        print(f'    frame written to it — none of them reach the wire. '
              f'Cycle the link:')
        print(f'      sudo ip link set {channel} down')
        print(f'      sudo ip link set {channel} up type can bitrate {bitrate} '
              f'restart-ms 100')
        ok = False
    elif state == 'ERROR-PASSIVE':
        print(f'\n  ⚠ \'{channel}\' is ERROR-PASSIVE — transmit errors, '
              f'usually nothing on the bus')
        print(f'    acknowledging (unpowered actuators) or missing 120 Ω '
              f'termination at BOTH ends.')

    berr = d.get('berr') or {}
    if berr.get('tx') or berr.get('rx'):
        print(f'\n  ⚠ bus error counters tx={berr.get("tx")} '
              f'rx={berr.get("rx")} — the bus is not clean.')

    if d.get('restart_ms') == 0:
        print(f'\n  ⚠ restart-ms is 0, so a bus-off will never self-recover. '
              f'Prefer `restart-ms 100`.')

    return ok


# ── Passive listen ────────────────────────────────────────────────────────────

def step_listen(channel, seconds, node_ids):
    """
    Watch the bus without transmitting. → (frames_by_cob, nodes_heard)

    A powered LINAK on the right bus at the right bitrate talks on its own:
    TPDO1 roughly every 250 ms, plus heartbeat and boot-up frames. None of that
    needs an SDO, a mapping or OPERATIONAL, which makes it the cheapest
    possible proof that an actuator is reachable at all.
    """
    print('\n── 2. Passive listen ' + '─' * 54)
    print(f'  Watching {channel} for {seconds:.1f} s, transmitting nothing ...')

    try:
        import can
    except ImportError:
        print('  ✗ python-can is not installed — cannot listen. '
              '(pip3 install python-can)')
        return Counter(), {}, []

    counts = Counter()
    extra = {}            # cob → a note worth printing once
    nodes = {}            # node id → set of roles heard

    try:
        bus = can.interface.Bus(channel=channel, bustype='socketcan')
    except Exception as exc:
        print(f'  ✗ Cannot open {channel}: {exc}')
        return counts, nodes, []

    end = time.time() + seconds
    try:
        while time.time() < end:
            msg = bus.recv(timeout=0.2)
            if msg is None or msg.is_error_frame:
                if msg is not None:
                    counts['ERROR-FRAME'] += 1
                continue
            cob = msg.arbitration_id
            counts[cob] += 1
            base, nid = cob & 0xF80, cob & 0x7F
            role = None
            if base == 0x180:
                role = 'TPDO1'
            elif base == 0x700:
                role = 'heartbeat'
                if msg.data:
                    extra[cob] = _NMT_STATE.get(msg.data[0],
                                                f'state 0x{msg.data[0]:02X}')
            elif base == 0x080 and cob != 0x080:
                role = 'EMCY'
                if len(msg.data) >= 2:
                    code = msg.data[0] | (msg.data[1] << 8)
                    extra[cob] = (f'0x{code:04X} — '
                                  f'{_EMCY_TEXT.get(code, "see the manual")}')
            elif base == 0x580:
                role = 'SDO response'
            elif base == 0x600:
                role = 'SDO request (from another master!)'
            elif base == 0x200:
                role = 'RPDO1 (commanded by another master!)'
            if role:
                nodes.setdefault(nid, set()).add(role)
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass

    if not counts:
        print('  ✗ Total silence — not one frame.')
        return counts, nodes, []

    for cob in sorted(k for k in counts if isinstance(k, int)):
        note = f'  ({extra[cob]})' if cob in extra else ''
        print(f'    0x{cob:03X}  ×{counts[cob]:<5d}{note}')
    if counts.get('ERROR-FRAME'):
        print(f'    ERROR FRAMES ×{counts["ERROR-FRAME"]} — bitrate mismatch, '
              f'termination, or wiring')

    # A second master is a fault in its own right, and a confusing one.
    foreign = sorted(c for c in counts if isinstance(c, int)
                     and ((c & 0xF80) in (0x600, 0x200) or c == HB_COB))
    if foreign:
        print(f'\n  ✗ CAUSE 6: another CANopen master is live on this bus '
              f'({", ".join(f"0x{c:03X}" for c in sorted(foreign))}).')
        print(f'    Something is still running — a unit test, an old launch. '
              f'Find it and stop it:')
        print(f'      pgrep -af "planting_fsr_test|can_tests|linak|ros2 launch"')

    for nid in sorted(nodes):
        print(f'\n  node 0x{nid:02X}: ' + ', '.join(sorted(nodes[nid])))

    for nid in node_ids:
        if nid not in nodes:
            print(f'\n  ✗ node 0x{nid:02X} said NOTHING in {seconds:.1f} s.')

    return counts, nodes, foreign


# ── Active read-only SDO probe ────────────────────────────────────────────────

def step_probe(channel, node_ids, eds, hb_ms):
    """
    Read 0x1000 (Device Type, read-only) from each node id. → set of answers.

    This is the same transaction class that failed on 10/6 — an SDO on a freshly
    connected bus — but an upload rather than a download, so nothing is
    configured and nothing moves. The master heartbeat runs during the probe so
    that a node whose 0x1016 watchdog is already armed stays happy while we talk
    to it.
    """
    print('\n── 3. SDO probe (read-only) ' + '─' * 47)
    try:
        import canopen
    except ImportError:
        print('  ✗ canopen is not installed. (pip3 install canopen)')
        return set()

    if not os.path.isfile(eds):
        print(f'  ✗ EDS not found: {eds}')
        return set()

    network = canopen.Network()
    try:
        network.connect(channel=channel, bustype='socketcan')
    except Exception as exc:
        print(f'  ✗ Cannot open {channel}: {exc}')
        return set()

    stop_hb = threading.Event()

    def heartbeat_loop():
        while not stop_hb.is_set():
            try:
                network.send_message(HB_COB, [0x05])     # 0x05 = OPERATIONAL
            except Exception:
                pass
            time.sleep(hb_ms / 1000.0)

    network.send_message(HB_COB, [0x05])
    threading.Thread(target=heartbeat_loop, daemon=True,
                     name='hb').start()
    time.sleep(hb_ms / 1000.0 * 3 + 0.05)

    answered = set()
    try:
        for nid in node_ids:
            node = canopen.RemoteNode(nid, eds)
            network.add_node(node)
            try:
                device_type = node.sdo[0x1000].raw
                print(f'  ✓ node 0x{nid:02X} answered — '
                      f'Device Type 0x{device_type:08X}')
                answered.add(nid)
            except Exception as exc:
                print(f'  ✗ node 0x{nid:02X}: {type(exc).__name__}: {exc}')
    finally:
        stop_hb.set()
        time.sleep(0.15)
        try:
            network.disconnect()
        except Exception:
            pass
    return answered


# ── Verdict ───────────────────────────────────────────────────────────────────

def verdict(node_ids, nodes_heard, answered, iface_ok, probed, foreign=()):
    print('\n── 4. Verdict ' + '─' * 61)
    missing = [n for n in node_ids
               if n not in nodes_heard and (not probed or n not in answered)]
    # Heard on the bus but will not answer an SDO. This is the worst one to get
    # wrong: it is the signature of an actuator left latched by the previous run,
    # it is the most likely reading of the 10/6 log, and it stays invisible if
    # the verdict short-circuits on "nothing is missing".
    heard_not_answering = [n for n in node_ids
                           if n in nodes_heard and probed and n not in answered]

    if not missing and not heard_not_answering and not foreign and iface_ok:
        print('  ✓ Every requested actuator is on the bus and answering SDO.')
        print('    The 10/6 failure does NOT reproduce from the bus side — '
              'bring up the')
        print('    ROS node next and compare its per-actuator log:')
        print('      ros2 launch planting_controller planting_bringup.launch.py')
        return 0

    for nid in missing:
        print(f'  ✗ node 0x{nid:02X} is absent from this bus.')
    if missing and len(missing) < len(node_ids):
        print('\n  CAUSE 1: one actuator is missing while the other works, so '
              'the bus itself')
        print('  is fine. In order: its power (bike batteries, E-stop '
              'released), its CAN')
        print('  connector, then its node id — a LINAK ships configurable and '
              'no passing')
        print('  test in this repo has ever addressed 0x21.')
    elif len(missing) == len(node_ids):
        print('\n  CAUSE 2/4/5: nothing at all is on this bus, so suspect the '
              'bus before the')
        print('  actuators — wrong interface, bus-off, or wrong bitrate. '
              'Re-run this script')
        print('  against every interface listed in step 1 before touching the '
              'hardware.')

    for nid in heard_not_answering:
        print(f'\n  ✗ node 0x{nid:02X} is BROADCASTING but will not answer an '
              f'SDO.')
        print('    CAUSE 3: it is almost certainly in a latched error from an '
              'earlier run')
        print('    (EMCY 0x8130 above?). Power-cycle that actuator and '
              're-run.')
        print('    This is the most likely reading of the 10/6 log: the FSR '
              'unit test drove')
        print('    this actuator perfectly eight minutes before the launch '
              'failed on its')
        print('    first SDO, and nothing was power-cycled in between.')

    if foreign:
        print()
        print('  ✗ CAUSE 6: a second CANopen master is on this bus (see '
              'step 2). Nothing')
        print('    this script says about the actuators is reliable until it '
              'is stopped.')

    if not iface_ok:
        print()
        print('  ✗ The interface itself is misconfigured (see step 1). Fix '
              'that first —')
        print('    every other result here is downstream of it.')
    return 1


def main():
    # The box-drawing and tick/cross characters below are the same ones the rest
    # of these tests use, and the Jetson's UTF-8 console renders them fine. A
    # diagnostic that dies on its own first print() is worse than useless, so on
    # a console that cannot encode them, degrade instead of raising.
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

    here = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser(
        description='Read-only CAN/CANopen bus diagnostic for the LINAK '
                    'actuators. Moves nothing.')
    p.add_argument('--channel', default=CHANNEL,
                   help=f'SocketCAN interface (default {CHANNEL})')
    p.add_argument('--bitrate', type=int, default=BITRATE,
                   help=f'Bitrate the LINAKs need (default {BITRATE})')
    p.add_argument('--node-ids', default=','.join(f'0x{n:02X}' for n in NODE_IDS),
                   help='Comma-separated node ids to look for '
                        '(default 0x20,0x21)')
    p.add_argument('--scan', action='store_true',
                   help='Probe node ids 1..127 instead of --node-ids. Slow '
                        '(~0.3 s each) but finds a relabelled actuator')
    p.add_argument('--listen', type=float, default=LISTEN_S,
                   help=f'Passive listen window, s (default {LISTEN_S:g})')
    p.add_argument('--no-probe', action='store_true',
                   help='Passive only — transmit absolutely nothing')
    p.add_argument('--eds', default=os.path.join(here,
                                                 'LINAK-actuator-v3-1.eds'),
                   help='Path to the LINAK EDS')
    p.add_argument('--hb-ms', type=int, default=100,
                   help='Master heartbeat period during the probe (default 100)')
    args = p.parse_args()

    node_ids = [int(s, 0) for s in args.node_ids.split(',') if s.strip()]

    print(f'can_bus_check — {args.channel} @ {args.bitrate} bps, looking for '
          f'{", ".join(f"0x{n:02X}" for n in node_ids)}')
    print('Read-only: no actuator will move.\n')

    iface_ok = step_interfaces(args.channel, args.bitrate)
    _, nodes_heard, foreign = step_listen(args.channel, args.listen,
                                          node_ids)

    answered = set()
    probed = not args.no_probe
    if probed:
        probe_ids = list(range(1, 128)) if args.scan else node_ids
        answered = step_probe(args.channel, probe_ids, args.eds, args.hb_ms)
        if args.scan:
            extra = sorted(answered - set(node_ids))
            if extra:
                print('\n  ! Node ids answering that we did not expect: '
                      + ', '.join(f'0x{n:02X}' for n in extra)
                      + ' — an actuator is configured to the wrong id.')

    return verdict(node_ids, nodes_heard, answered, iface_ok, probed,
                   foreign)


if __name__ == '__main__':
    sys.exit(main())
