import canopen # for auger CANOpen
import can     # for chute CAN J1939
import time
import threading

# === CANOPEN Configuration ===
CHANNEL_CANOPEN = 'can0'
BITRATE_CANOPEN = 125000
CANOPEN_ID = 0x20                   #32 decimal
COB_ID_RPDO1 = 0x200 + CANOPEN_ID
BOOTUP_COB_ID = 0x700 + CANOPEN_ID
HEARTBEAT_PRODUCER_ID = 0x701       # Master heartbeat ID
HEARTBEAT_TIME_MS = 100

bootup_received = threading.Event()
stop_heartbeat = threading.Event()

# === CAN Configuration ===
CHANNEL_CAN = 'can1'
BUSTYPE_CAN = 'socketcan'
BITRATE_CAN = 250000

CAN_ID = 0xC8                       # 200 decimal
MASTER_ADDRESS = 0x01
PRIORITY = 6
PGN_PROP_A_PF = 0xEF

# === COMMAND CODEs ===
POS_CLEAR_ERROR = 0xFB00   # 64256
POS_RUN_OUT     = 0xFB01   # 64257
POS_RUN_IN      = 0xFB02   #  64258
POS_STOP        = 0xFB03   # 64259