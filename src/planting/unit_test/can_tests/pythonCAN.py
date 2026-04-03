# LINAK CANOpen reference: https://cdn.linak.com/-/media/files/ic-and-bus-actuators/techline-canopen-user-manual-eng-legacy.pdf?_gl=1*1os9jmo*_gcl_au*NzcwNjk5ODIyLjE3NDQyOTk4MjQ
# On page 11, it details the PDO message structure mapping
import canopen
import time
import threading

# === Configuration ===
CHANNEL = 'can0'

NODE_ID_AUGER = 0x20 #32 decimal
NODE_ID_CHUTE = 0x21 #33 decimal

COB_ID_RPDO1 = 0x200 + NODE_ID_AUGER
BOOTUP_COB_ID = 0x700 + NODE_ID_AUGER
COB_ID_RPDO2 = 0x200 + NODE_ID_CHUTE
BOOTUP_COB_ID_CHUTE = 0x700 + NODE_ID_CHUTE

HEARTBEAT_PRODUCER_ID = 0x701  # Master heartbeat ID
HEARTBEAT_TIME_MS = 100

bootup_received = threading.Event()
stop_heartbeat = threading.Event()

def send_actuator_command(cob_id, position_code):
    """Send 8-byte RPDO message."""
    msg = [
        position_code & 0xFF, (position_code >> 8) & 0xFF,  # Position field (little endian, right to left)
        0xFB, 0xFB, 0xFB, 0xFB,  # Default current, speed, ramps
        0x00, 0x00  # Padding
    ]
    print(f"RPDO Command → {msg}")
    network.send_message(cob_id, msg)

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
node_auger = canopen.RemoteNode(NODE_ID_AUGER, 'LINAK-actuator-v3-1.eds')
node_chute = canopen.RemoteNode(NODE_ID_CHUTE, 'LINAK-actuator-v3-1.eds')
network.add_node(node_auger)
network.add_node(node_chute)

# === Start sending heartbeat ===
# heartbeat_thread = threading.Thread(target=heartbeat_loop)
# heartbeat_thread.daemon = True
# heartbeat_thread.start()
network.send_message(HEARTBEAT_PRODUCER_ID, [0x05])

# === Set heartbeat expectation (consumer heartbeat time) ===
print(f"Setting actuator consumer heartbeat time to {HEARTBEAT_TIME_MS} ms...")
for node in [node_auger, node_chute]:
    node.sdo[0x1016][1].raw = (0x01 << 16) + 100  # 0x00010064
    time.sleep(0.1)

    rpdo = node.rpdo[1]
    rpdo.clear()
    rpdo.add_variable('Actuator Command.Position')
    rpdo.enabled = True
    rpdo.save()
    time.sleep(0.1)

# === Set actuators to OPERATIONAL ===
print("Setting actuators to OPERATIONAL...")
for node in [node_auger, node_chute]:
    node.nmt.state = 'OPERATIONAL'
time.sleep(0.1)

# === Initialize with STOP ===
print("Initial STOP...")
send_actuator_command(COB_ID_RPDO1, 64259)
send_actuator_command(COB_ID_RPDO2, 64259)
time.sleep(1)

# === Clear errors, if any ===
send_actuator_command(COB_ID_RPDO1, 64256)
send_actuator_command(COB_ID_RPDO2, 64256)
time.sleep(1)

# === RUN OUT ===
print("⬆️  RUN OUT...")
send_actuator_command(COB_ID_RPDO1, 64257)
send_actuator_command(COB_ID_RPDO2, 64257)
time.sleep(5)

# # === STOP ===
print("STOP...")
send_actuator_command(COB_ID_RPDO1, 64259)
send_actuator_command(COB_ID_RPDO2, 64259)
time.sleep(1)

# # === RUN IN ===
print("⬇️  RUN IN...")
send_actuator_command(COB_ID_RPDO1, 64258)
send_actuator_command(COB_ID_RPDO2, 64258)
time.sleep(5)

# === Final STOP ===
print("Final STOP...")
send_actuator_command(COB_ID_RPDO1, 64259)
send_actuator_command(COB_ID_RPDO2, 64259)
time.sleep(0.5)

# === Shutdown ===
print("Disconnecting...")
# stop_heartbeat.set()
# time.sleep(0.1)  # Let heartbeat thread exit cleanly
network.disconnect()
print("✅ Done.")
