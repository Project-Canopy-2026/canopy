import canopen
import time
import threading

# === Configuration ===
CHANNEL = 'can0'
NODE_ID = 0x20 #32 decimal
COB_ID_RPDO1 = 0x200 + NODE_ID
BOOTUP_COB_ID = 0x700 + NODE_ID
HEARTBEAT_PRODUCER_ID = 0x701  # Master heartbeat ID
HEARTBEAT_TIME_MS = 100

bootup_received = threading.Event()
stop_heartbeat = threading.Event()

# def monitor_bootup():
#     """Listen for 0x720 boot-up message from actuator."""
#     def on_message(msg):
#         if msg.arbitration_id == BOOTUP_COB_ID and msg.data == b'\x00':
#             print("Actuator boot-up message received.")
#             bootup_received.set()
#     network.subscribe(on_message)

def send_actuator_command(position_code):
    """Send 8-byte RPDO message."""
    msg = [
        position_code & 0xFF, (position_code >> 8) & 0xFF,  # Position field (little endian)
        0xFB, 0xFB, 0xFB, 0xFB,  # Default current, speed, ramps
        0x00, 0x00  # Padding
    ]
    print(f"📤 RPDO Command → {msg}")
    network.send_message(COB_ID_RPDO1, msg)

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


# === Wait for boot-up ===
# monitor_thread = threading.Thread(target=monitor_bootup)
# monitor_thread.start()

# print("Waiting for actuator boot-up...")
# if not bootup_received.wait(timeout=5):
#     print(" Boot-up not received. Check power and CAN wiring.")
#     network.disconnect()
#     exit(1)

# === Add node ===
node = canopen.RemoteNode(NODE_ID, 'LINAK-actuator-v3-1.eds')
network.add_node(node)

# === Start sending heartbeat ===
# heartbeat_thread = threading.Thread(target=heartbeat_loop)
# heartbeat_thread.daemon = True
# heartbeat_thread.start()
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

send_actuator_command(500)
# === RUN OUT ===
print("⬆️  RUN OUT...")
send_actuator_command(64257)
time.sleep(5)

# # === STOP ===
# print("STOP...")
# send_actuator_command(64259)
# time.sleep(1)

# # === RUN IN ===
# print("⬇️  RUN IN...")
# send_actuator_command(64258)
# time.sleep(5)

# === Final STOP ===
print("Final STOP...")
send_actuator_command(64259)
time.sleep(0.5)

# === Shutdown ===
print("Disconnecting...")
# stop_heartbeat.set()
# time.sleep(0.1)  # Let heartbeat thread exit cleanly
network.disconnect()
print("✅ Done.")






# import canopen
# import time
# import threading

# # === Configuration ===
# CHANNEL = 'can0'
# NODE_ID_AUGER = 0x20 #32 decimal
# # NODE_ID_CHUTE = 0x21 #33 decimal
# COB_ID_RPDO1_AUGER = 0x200 + NODE_ID_AUGER
# # COB_ID_RPDO1_CHUTE = 0x200 + NODE_ID_CHUTE
# # BOOTUP_COB_ID_AUGER = 0x700 + NODE_ID_AUGER

# HEARTBEAT_PRODUCER_ID = 0x701  # Master heartbeat ID
# HEARTBEAT_TIME_MS = 100

# bootup_received = threading.Event()
# stop_heartbeat = threading.Event()

# # def monitor_bootup():
# #     """Listen for 0x720 boot-up message from actuator."""
# #     def on_message(msg):
# #         if msg.arbitration_id == BOOTUP_COB_ID_AUGER and msg.data == b'\x00':
# #             print("Actuator boot-up message received.")
# #             bootup_received.set()
# #     network.subscribe(on_message)

# def send_actuator_command(position_code, cob_id):
#     """Send 8-byte RPDO message to the given COB-ID."""
#     msg = [
#         position_code & 0xFF, (position_code >> 8) & 0xFF,  # Position field (little endian)
#         0xFB, 0xFB, 0xFB, 0xFB,  # Default current, speed, ramps
#         0x00, 0x00  # Padding
#     ]
#     print(f"📤 RPDO Command (0x{cob_id:X}) → {msg}")
#     network.send_message(cob_id, msg)

# def heartbeat_loop():
#     """Send master heartbeat every 100 ms."""
#     print("Starting master heartbeat at 100 ms...")
#     while not stop_heartbeat.is_set():
#         try:
#             network.send_message(HEARTBEAT_PRODUCER_ID, [0x05])  # 0x05 = OPERATIONAL
#         except Exception as e:
#             print(f"Heartbeat send error: {e}")
#         time.sleep(HEARTBEAT_TIME_MS / 1000.0)

# # === Connect to CAN ===
# network = canopen.Network()
# network.connect(channel=CHANNEL, bustype='socketcan')

# # === Add nodes ===
# node_auger = canopen.RemoteNode(NODE_ID_AUGER, 'LINAK-actuator-v3-1.eds')
# # node_chute = canopen.RemoteNode(NODE_ID_CHUTE, 'LINAK-actuator-v3-1.eds')
# network.add_node(node_auger)
# # network.add_node(node_chute)

# # === Reset and wait for boot-up ===
# print("Resetting actuator...")
# node_auger.nmt.state = 'RESET'
# # node_chute.nmt.state = 'RESET'

# print("Waiting for actuator boot-up...")
# try:
#     node_auger.nmt.wait_for_bootup(timeout=10)
#     print("Actuator booted.")
# except canopen.nmt.NmtError:
#     print("No boot-up message received, assuming actuator already running...")
# time.sleep(0.5)  # Give actuator time to finish initialization

# # === Start sending heartbeat ===
# heartbeat_thread = threading.Thread(target=heartbeat_loop)
# heartbeat_thread.daemon = True
# heartbeat_thread.start()

# time.sleep(0.1)

# time.sleep(0.1)

# # === Set actuator to OPERATIONAL ===
# print("Setting actuator to OPERATIONAL...")
# node_auger.nmt.state = 'OPERATIONAL'
# # node_chute.nmt.state = 'OPERATIONAL'
# time.sleep(0.1)

# # === Initialize with STOP ===
# print("Initial STOP...")
# send_actuator_command(64259, COB_ID_RPDO1_AUGER)
# # send_actuator_command(64259, COB_ID_RPDO1_CHUTE)
# time.sleep(1)

# # === Clear errors, if any ===
# send_actuator_command(64256, COB_ID_RPDO1_AUGER)
# # send_actuator_command(64256, COB_ID_RPDO1_CHUTE)
# time.sleep(1)

# # === INITIALIZE to 5cm ===
# send_actuator_command(500, COB_ID_RPDO1_AUGER)

# # === RUN OUT ===
# print("⬆️  RUN IN...")
# send_actuator_command(64258, COB_ID_RPDO1_AUGER)
# # send_actuator_command(64257, COB_ID_RPDO1_CHUTE)
# time.sleep(5)

# # # === STOP ===
# # print("STOP...")
# # send_actuator_command(64259, COB_ID_RPDO1_AUGER)
# # send_actuator_command(64259, COB_ID_RPDO1_CHUTE)
# # time.sleep(1)

# # # === RUN IN ===
# # print("⬇️  RUN IN...")
# # send_actuator_command(64258, COB_ID_RPDO1_AUGER)
# # send_actuator_command(64258, COB_ID_RPDO1_CHUTE)
# # time.sleep(5)

# # === Final STOP ===
# print("Final STOP...")
# send_actuator_command(64259, COB_ID_RPDO1_AUGER)
# # send_actuator_command(64259, COB_ID_RPDO1_CHUTE)
# time.sleep(0.5)

# # === Shutdown ===
# print("Disconnecting...")
# stop_heartbeat.set()
# time.sleep(0.15)
# network.disconnect()
# print("Done.")
