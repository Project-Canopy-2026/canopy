# This is working for new actuator on chute
# Use BITRATE = 250000 and address 0xC8 which is the default address
import can
import time
import threading

# === Configuration ===
CHANNEL = 'can0'
BUSTYPE = 'socketcan'
BITRATE = 250000

# J1939 Addresses
ACTUATOR_ADDRESS = 0xC8       # 200 decimal (default LINAK address)
MASTER_ADDRESS = 0x01         # Our ECU address

# J1939 PGN for Proprietary A = 0xEF00 (PDU1, peer-to-peer)
# 29-bit CAN ID for J1939:
# [Priority(3)] [Reserved(1)] [DataPage(1)] [PDU Format(8)] [PDU Specific = Dest Addr(8)] [Source Addr(8)]
# Proprietary A: PF = 0xEF, PDU1 (destination-specific)
# Priority = 6 (default), R=0, DP=0
# CAN ID = (priority << 26) | (PF << 16) | (dest_addr << 8) | src_addr
 
PRIORITY = 6
PGN_PROP_A_PF = 0xEF  # PDU Format for Proprietary A

# Position command codes (from manual)
POS_CLEAR_ERROR = 0xFB00   # 64256
POS_RUN_OUT     = 0xFB01   # 64257
POS_RUN_IN      = 0xFB02   #  64258
POS_STOP        = 0xFB03   # 64259

# Target position for run-to-position (mm)
TARGET_POSITION_MM = 1200

# Byte value for "use default" setting
DEFAULT_VALUE = 0xFB


def mm_to_pos_code(mm: float) -> int:
    """Convert millimetres to a J1939 position code (0.1 mm/bit). Valid range: 0–6399.9 mm."""
    code = int(mm * 10)
    if not (0x0000 <= code <= 0xFAFF):
        raise ValueError(f"Position {mm} mm out of range (0–6399.9 mm)")
    return code

# How often to resend Proprietary A (must be <= 250ms)
COMMAND_INTERVAL_S = 0.1

stop_event = threading.Event()
current_command = None
command_lock = threading.Lock()


def build_j1939_id(priority: int, pf: int, dest_addr: int, src_addr: int) -> int:
    """
    Build a 29-bit J1939 CAN ID for PDU1 (peer-to-peer) messages.
    PDU1: PF < 0xF0, PDU Specific field = destination address
    """
    r = 0   # Reserved bit
    dp = 0  # Data page
    can_id = (priority << 26) | (r << 25) | (dp << 24) | (pf << 16) | (dest_addr << 8) | src_addr
    return can_id


def build_proprietary_a(position_code: int,
                         current_limit: int = DEFAULT_VALUE,
                         speed: int = DEFAULT_VALUE,
                         ramp_up: int = DEFAULT_VALUE,
                         ramp_down: int = DEFAULT_VALUE) -> list:
    """
    Build 8-byte Proprietary A (Command) message payload.

    Byte layout (from manual):
      Byte 0-1: Position (UINT16, little-endian)
      Byte 2:   Current Limit
      Byte 3:   Speed
      Byte 4:   Ramp Up
      Byte 5:   Ramp Down
      Byte 6-7: Reserved (must be 0xFF)

    Position codes:
      0x0000–0xFAFF : Run to position (0.1 mm/bit)
      0xFB00        : Clear error
      0xFB01        : Run out
      0xFB02        : Run in
      0xFB03        : Stop
      0xFB04        : Recovery run out
      0xFB05        : Recovery run in
    """
    pos_lo = position_code & 0xFF
    pos_hi = (position_code >> 8) & 0xFF
    return [
        pos_lo,       # Byte 0: Position LSB
        pos_hi,       # Byte 1: Position MSB
        current_limit,# Byte 2: Current limit (0xFB = default)
        speed,        # Byte 3: Speed (0xFB = default)
        ramp_up,      # Byte 4: Ramp up (0xFB = default)
        ramp_down,    # Byte 5: Ramp down (0xFB = default)
        0xFF,         # Byte 6: Reserved
        0xFF,         # Byte 7: Reserved
    ]


def send_command(bus: can.Bus, position_code: int, label: str = ""):
    """Send a single Proprietary A J1939 message."""
    can_id = build_j1939_id(PRIORITY, PGN_PROP_A_PF, ACTUATOR_ADDRESS, MASTER_ADDRESS)
    data = build_proprietary_a(position_code)
    msg = can.Message(
        arbitration_id=can_id,
        data=data,
        is_extended_id=True  # J1939 always uses 29-bit IDs
    )
    bus.send(msg)
    if label:
        print(f"  [{label}] CAN ID=0x{can_id:08X} Data={[f'0x{b:02X}' for b in data]}")


def command_loop(bus: can.Bus):
    """
    Continuously resend the current command every COMMAND_INTERVAL_S.
    The manual requires Proprietary A to be sent at least every 250 ms
    to keep the 'signal alive' — otherwise the actuator may stop.
    """
    while not stop_event.is_set():
        with command_lock:
            code = current_command
        if code is not None:
            can_id = build_j1939_id(PRIORITY, PGN_PROP_A_PF, ACTUATOR_ADDRESS, MASTER_ADDRESS)
            data = build_proprietary_a(code)
            msg = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=True
            )
            try:
                bus.send(msg)
            except can.CanError as e:
                print(f"CAN send error: {e}")
        stop_event.wait(timeout=COMMAND_INTERVAL_S)


def set_command(position_code: int):
    """Update the command that the loop will keep resending."""
    global current_command
    with command_lock:
        current_command = position_code


def print_feedback(bus: can.Bus, timeout: float = 0.5):
    """
    Listen briefly for Proprietary B feedback from the actuator.
    Proprietary B uses PDU2 (broadcast), PF >= 0xF0.
    The actuator broadcasts at 100ms intervals.

    Byte layout:
      Byte 0-1: Position (mm * 10)
      Byte 2:   Current draw (A * 4)
      Byte 3:   Status flags
      Byte 4:   Error code
      Byte 5-6: Speed
      Byte 7:   AUX input
    """
    # Proprietary B source address will be ACTUATOR_ADDRESS
    # PF for Proprietary B PDU2 = 0xFF, so CAN ID source byte = ACTUATOR_ADDRESS
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = bus.recv(timeout=0.05)
        if msg and msg.is_extended_id:
            src = msg.arbitration_id & 0xFF
            pf = (msg.arbitration_id >> 16) & 0xFF
            # Proprietary B: PF=0xFF (PDU2 broadcast), src=actuator address
            if src == ACTUATOR_ADDRESS and pf == 0xFF and len(msg.data) == 8:
                pos_raw = msg.data[0] | (msg.data[1] << 8)
                position_mm = pos_raw / 10.0 if pos_raw <= 0xFAFF else None
                current_a = msg.data[2] * 0.25 if msg.data[2] <= 250 else None
                status = msg.data[3]
                error = msg.data[4]
                speed_raw = msg.data[5] | (msg.data[6] << 8)
                speed_mms = speed_raw * 0.1 if speed_raw <= 0x0FAF else None

                print(f"  [Feedback] "
                      f"Pos={f'{position_mm:.1f} mm' if position_mm is not None else 'N/A'} | "
                      f"Current={f'{current_a:.2f} A' if current_a is not None else 'N/A'} | "
                      f"Status=0x{status:02X} | "
                      f"Error=0x{error:02X} | "
                      f"Speed={f'{speed_mms:.1f} mm/s' if speed_mms is not None else 'N/A'}")
                return


# ==================== Main ====================
def main():
    print(f"Connecting to CAN bus ({CHANNEL}, {BITRATE} bps)...")
    bus = can.Bus(channel=CHANNEL, bustype=BUSTYPE, bitrate=BITRATE)

    # Start background command loop (keeps Proprietary A alive every 100ms)
    loop_thread = threading.Thread(target=command_loop, args=(bus,), daemon=True)
    loop_thread.start()

    try:
        # --- Step 1: Send STOP first (mandatory before any run command) ---
        print("\n[1/6] Sending STOP (required before run)...")
        set_command(POS_STOP)
        time.sleep(0.5)
        print_feedback(bus)

        # --- Step 2: Clear any existing errors ---
        print("\n[2/6] Clearing errors...")
        set_command(POS_CLEAR_ERROR)
        time.sleep(0.5)
        print_feedback(bus)

        # --- Step 3: Send STOP again, confirm idle ---
        print("\n[3/6] Re-sending STOP, confirming idle state...")
        set_command(POS_STOP)
        time.sleep(0.5)
        print_feedback(bus)

        # --- Step 4: Run to position ---
        print(f"\n[4/6] Running to position {TARGET_POSITION_MM} mm...")
        set_command(mm_to_pos_code(TARGET_POSITION_MM))
        for i in range(20):
            time.sleep(0.5)
            print(f"  t={i * 0.5 + 0.5:.1f}s ", end="")
            print_feedback(bus, timeout=0.1)

        # # --- Step 5: Run OUT ---
        # print("\n[5/6] Running OUT for 5 seconds...")
        # set_command(POS_RUN_OUT)
        # for i in range(10):
        #     time.sleep(0.5)
        #     print(f"  t={i * 0.5 + 0.5:.1f}s ", end="")
        #     print_feedback(bus, timeout=0.1)

        # --- Step 6: Final STOP ---
        print("\n[6/6] Sending final STOP...")
        set_command(POS_STOP)
        time.sleep(0.5)
        print_feedback(bus)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("\nStopping command loop and disconnecting...")
        set_command(POS_STOP)
        time.sleep(0.3)
        stop_event.set()
        loop_thread.join(timeout=1.0)
        bus.shutdown()
        print("Done.")


if __name__ == "__main__":
    main()