# LINAK CANOpen reference: https://cdn.linak.com/-/media/files/ic-and-bus-actuators/techline-canopen-user-manual-eng-legacy.pdf?_gl=1*1os9jmo*_gcl_au*NzcwNjk5ODIyLjE3NDQyOTk4MjQ
# On page 11, it details the PDO message structure mapping
import canopen
import time
import threading

# === Configuration ===
CHANNEL = 'can0'
NODE_ID = 0x20 # 33 for chute
COB_ID_RPDO1 = 0x200 + NODE_ID
COB_ID_TPDO1 = 0x180 + NODE_ID  # position feedback (object 0x2001), every 250 ms by default
BOOTUP_COB_ID = 0x700 + NODE_ID
HEARTBEAT_PRODUCER_ID = 0x701
HEARTBEAT_TIME_MS = 100

bootup_received = threading.Event()
stop_heartbeat = threading.Event()

def send_actuator_command(position_code):
    """Send 8-byte RPDO message."""
    msg = [
        position_code & 0xFF, (position_code >> 8) & 0xFF,  # Position field (little endian, right to left)
        0xFB, 0xCD, 0xFB, 0xFB,  # Default current, MAX speed, defualt ramps
        0x00, 0x00  # Padding
    ]
    global move_start
    print(f"RPDO Command → {msg}")
    network.send_message(COB_ID_RPDO1, msg)
    move_start = time.time()

# Latest TPDO1 feedback, printed by monitor()
tpdo_lock = threading.Lock()
tpdo_latest = None  # (timestamp, position, current, status, error, speed)
move_start = time.time()

def on_tpdo(can_id, data, timestamp):
    """TPDO1 layout (LINAK-actuator-v3-1.eds, object 0x2001):
      bytes 0-1 Position (uint16 LE), byte 2 Current, byte 3 Status Flags,
      byte 4 Error Code, bytes 5-6 Speed (uint16 LE), byte 7 Input State
    """
    global tpdo_latest
    if len(data) < 7:
        return
    pos = data[0] | (data[1] << 8)
    speed = data[5] | (data[6] << 8)
    with tpdo_lock:
        tpdo_latest = (timestamp, pos, data[2], data[3], data[4], speed)

def monitor(duration_s, period_s=0.1):
    """Print TPDO position feedback for duration_s, relative to the last command."""
    last_ts = None
    end = time.time() + duration_s
    while time.time() < end:
        with tpdo_lock:
            sample = tpdo_latest
        if sample is not None and sample[0] != last_ts:
            last_ts = sample[0]
            ts, pos, cur, status, err, speed = sample
            print(f"  t={time.time() - move_start:6.2f}s  pos={pos:5d}  cur={cur:3d}  "
                  f"speed={speed:5d}  status=0x{status:02X}  err=0x{err:02X}")
        time.sleep(period_s)

def heartbeat_loop():
    """Send master heartbeat every 100 ms."""
    print("Starting master heartbeat at 100 ms...")
    while not stop_heartbeat.is_set():
        try:
            network.send_message(HEARTBEAT_PRODUCER_ID, [0x05])  # 0x05 = OPERATIONAL
        except Exception as e:
            print(f"Heartbeat send error: {e}")
        time.sleep(HEARTBEAT_TIME_MS / 1000.0)

# === Connect to CAN ===
network = canopen.Network()
network.connect(channel=CHANNEL, bustype='socketcan')

# === Add node ===
import os
eds_path = os.path.join(os.path.dirname(__file__), 'LINAK-actuator-v3-1.eds')
node = canopen.RemoteNode(NODE_ID, eds_path)
network.add_node(node)
network.subscribe(COB_ID_TPDO1, on_tpdo)

network.send_message(HEARTBEAT_PRODUCER_ID, [0x05])

# === Set heartbeat expectation (consumer heartbeat time) ===
print(f"Setting actuator consumer heartbeat time to {HEARTBEAT_TIME_MS} ms...")
node.sdo[0x1016][1].raw = (0x01 << 16) + 100  # 0x00010064
time.sleep(0.1)

rpdo = node.rpdo[1]
rpdo.clear()
rpdo.add_variable('Actuator Command.Position')
rpdo.enabled = True
rpdo.save()
time.sleep(0.1)

# === Set actuator to OPERATIONAL ===
print("Setting actuator to OPERATIONAL...")
node.nmt.state = 'OPERATIONAL'
time.sleep(0.1)

# === Initialize with STOP ===
print("Initial STOP...")
send_actuator_command(64259)
time.sleep(1)

# === Clear errors, if any ===
send_actuator_command(64256)
time.sleep(1)

# === GO DOWN (extend) to position 500, printing position feedback ===
print("⬇️  DOWN to 500...")
send_actuator_command(500)
monitor(10)
# # === RUN OUT ===
# print("⬆️  RUN OUT...")
# send_actuator_command(64257)
# time.sleep(5)

# # === STOP ===
# print("STOP...")
# send_actuator_command(64259)
# time.sleep(1)

# === RUN IN ===
print("⬇️  RUN IN...")
send_actuator_command(150)
monitor(10)

# === Final STOP ===
print("Final STOP...")
send_actuator_command(64259)
monitor(0.5)

# === Shutdown ===
print("Disconnecting...")
# stop_heartbeat.set()
# time.sleep(0.1)  # Let heartbeat thread exit cleanly
network.disconnect()
print("✅ Done.")
