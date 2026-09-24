# Auger Drilling Force Test (FSR)
Cteamj@ubuntu:~/dev/ros2_ws/src/canopy/src/planting/unit_test/can_tests$ python can_tests.py 
Setting actuator consumer heartbeat time to 100 ms...
Transfer aborted by client with code 0x05040000
Traceback (most recent call last):
  File "/home/teamj/.local/lib/python3.10/site-packages/canopen/sdo/client.py", line 76, in read_response
    response = self.responses.get(
  File "/usr/lib/python3.10/queue.py", line 179, in get
    raise Empty
_queue.Empty

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "/home/teamj/dev/ros2_ws/src/canopy/src/planting/unit_test/can_tests/can_tests.py", line 88, in <module>
    node.sdo[0x1016][1].raw = (0x01 << 16) + 100  # 0x00010064
  File "/home/teamj/.local/lib/python3.10/site-packages/canopen/variable.py", line 90, in raw
    self.data = self.od.encode_raw(value)
  File "/home/teamj/.local/lib/python3.10/site-packages/canopen/variable.py", line 46, in data
    self.set_data(data)
  File "/home/teamj/.local/lib/python3.10/site-packages/canopen/sdo/base.py", line 155, in set_data
    self.sdo_node.download(self.od.index, self.od.subindex, data, force_segment)
  File "/home/teamj/.local/lib/python3.10/site-packages/canopen/sdo/client.py", line 167, in download
    with self.open(index, subindex, "wb", buffering=7, size=len(data),
  File "/home/teamj/.local/lib/python3.10/site-packages/canopen/sdo/client.py", line 400, in write
    response = self.sdo_client.request_response(request)
  File "/home/teamj/.local/lib/python3.10/site-packages/canopen/sdo/client.py", line 95, in request_response
    return self.read_response()
  File "/home/teamj/.local/lib/python3.10/site-packages/canopen/sdo/client.py", line 79, in read_response
    raise SdoCommunicationError("No SDO response received")
canopen.sdo.exceptions.SdoCommunicationError: No SDO response received
teamj@ubuntu:~/dev/ros2_ws/src/canopy/src/planting/unit_test/can_tests$ 




teamj@ubuntu:~/dev/ros2_ws/src/canopy/src/planting/unit_test/fsr$ candump can0
^Cteamj@ubuntu:~/dev/ros2_ws/src/canopy/src/planting/unit_test/fsr$ python3 planting_fsr_test.py --can-channel can0 --drill-cm 3 --dwell 2 --rpm 3
Logging FSR to /home/teamj/dev/ros2_ws/src/canopy/src/planting/unit_test/fsr/planting_fsr_20260922_192104.csv
Opening /dev/ttyACM1 @ 115200 baud ...
Waiting for Arduino READY ...
Arduino READY.
[  1.57s] → arduino: stop
Connecting to 'can0' @ 125000 bps ...
← arduino: ACK:stop
Transfer aborted by client with code 0x05040000
❌ Error: No SDO response received
[  2.28s] STATE → FAULT
[  2.28s] → arduino: stop
← arduino: ACK:stop
Exception in thread arduino_read:
Traceback (most recent call last):
  File "/usr/lib/python3.10/threading.py", line 1016, in _bootstrap_inner
    self.run()
  File "/usr/lib/python3.10/threading.py", line 953, in run
    self._target(*self._args, **self._kwargs)
  File "/home/teamj/dev/ros2_ws/src/canopy/src/planting/unit_test/fsr/planting_fsr_test.py", line 211, in _read_loop
    raw = self.ser.readline()
  File "/usr/lib/python3/dist-packages/serial/serialposix.py", line 575, in read
    buf = os.read(self.fd, size - len(read))
TypeError: 'NoneType' object cannot be interpreted as an integer
Saved /home/teamj/dev/ros2_ws/src/canopy/src/planting/unit_test/fsr/planting_fsr_20260922_192104.csv

Unit test that replays the drilling half of `planting_fsm.py` — auger spin-up,
drill down, dwell, retract — while logging the FSR ground-reaction force to a
CSV for the whole run. No ROS2 required; the script talks to the Arduino and
the LINAK directly.

**Sequence** (each state is tagged in the log so force can be sliced by phase):

| State | What happens |
|---|---|
| `LINAK_HOME` | LINAK 1 retracts to IN_MAX (skip with `--no-home`) |
| `AUGER_SPIN_UP` | `bldc,in,<rpm>` — auger starts spinning |
| `DRILLING_DOWN` | LINAK 1 extends `--drill-cm` at SLOW (1.09 cm/s) |
| `DRILLING_DWELL` | auger keeps spinning at depth for `--dwell` seconds |
| `AUGER_RETRACT` | LINAK 1 retracts `--retract-cm` at FAST (2.18 cm/s) |
| `COMPLETE` | auger stops, actuator stops, logging continues for `--tail` s |

---

## Hardware checklist

- Arduino UNO on USB (`/dev/ttyACM0`), running `planting_arduino.ino`
- LINAK 1 (auger actuator), CANopen node `0x20`, powered
- CAN adapter (e.g. Kvaser Leaf Light v2) on the LINAK bus @ 125 kbps,
  **120 Ω termination at both ends of the bus**
- BLDC driver powered; auger clear of obstructions
- FSR wired as a voltage divider, tap at **A0**:
  `5V --[FSR]--+--[5 kΩ]-- GND`
- BLDC pins: `SV = 3`, `FR = 7`, `EN = 8`

---

## Step 1 — Flash the Arduino

Open in the Arduino IDE and upload at **115200 baud**:

```
src/planting/planting_controller/planting_controller/planting_arduino/planting_arduino.ino
```

**Not `fsr.ino`.** That sketch streams CSV at 9600 baud with no command parser —
it cannot drive the BLDC and will ignore `fsr,read`. This test needs the
combined controller's command protocol.

Close the Arduino IDE Serial Monitor afterwards — it holds the port and the
test will fail to open it.

## Step 2 — Bring up the CAN interface

```bash
sudo ip link set can1 down
sudo ip link set can1 up type can bitrate 125000
ip link show can1            # should report UP
```

Check the interface name first with `ip link show` — a single-channel Kvaser
usually enumerates as `can0`, while the repo's other scripts default to `can1`.
No driver download is needed on Linux; the in-kernel `kvaser_usb` module
presents the adapter as a normal SocketCAN interface.

Confirm the actuator is talking before going further:

```bash
candump can1                 # expect 0x1A0 frames roughly every 250 ms
```

## Step 3 — Check the serial port

List the candidate ports. The UNO normally appears as `/dev/ttyACM0`; a board
with an FTDI/CH340 USB chip appears as `/dev/ttyUSB0` instead:

```bash
ls /dev/ttyACM*             # UNO R3 and most genuine Arduinos
ls /dev/ttyUSB*             # FTDI / CH340 clones, USB-serial adapters
ls -l /dev/serial/by-id/    # shows which physical device is which port
```

Confirm the board is actually enumerating on USB:

```bash
lsusb                       # look for "Arduino" or the clone's USB chip
dmesg | tail -20            # shows the port name assigned on plug-in
```

If the port exists but cannot be opened, check permissions:

```bash
groups                      # you need to be in 'dialout'
sudo usermod -aG dialout $USER   # then log out and back in
sudo lsof /dev/ttyACM0      # shows any process already holding the port
```

Pass whatever you found to the script with `--port` if it isn't
`/dev/ttyACM0`.

## Step 4 — Dry run, off the ground

Auger clear of soil, actuator free to move. Small distance, low RPM — this
confirms CAN init, the Arduino `READY` handshake, direction sense, and that
`FSR:` replies are arriving.

```bash
cd src/planting/unit_test/fsr
python3 planting_fsr_test.py --can-channel can1 --drill-cm 3 --dwell 2 --rpm 30
```

## Step 5 — Real run

```bash
python3 planting_fsr_test.py --can-channel can1 --drill-cm 12 --retract-cm 12 --dwell 5 --rpm 75
```

**Ctrl+C aborts safely at any point** — the auger and actuator are stopped, the
run is marked `ABORTED`, and the CSV is closed cleanly.

---

## Settings

Defaults live in one block at the top of `planting_fsr_test.py` (look for
`TEST SETTINGS`). Edit them there for a lasting change, or override any of them
on the command line for a one-off. `--help` lists them all.

| Setting | Flag | Default |
|---|---|---|
| Drill-down distance (cm) | `--drill-cm` | 12.0 |
| Retract distance (cm) | `--retract-cm` | 12.0 |
| Auger speed (RPM) | `--rpm` | 75 |
| Dwell at depth (s) | `--dwell` | 5.0 |
| Serial port | `--port` | `/dev/ttyACM0` |
| CAN interface | `--can-channel` | `can1` |
| LINAK node id | `--node-id` | `0x20` |
| FSR sample rate (Hz) | `--fsr-hz` | 10.0 |
| Skip homing | `--no-home` | off |

---

## Output

A CSV lands next to the script as `planting_fsr_<timestamp>.csv` (override with
`--out`):

```
elapsed_s,state,voltage_V,resistance_ohm
```

`resistance_ohm` is `-1` when the sensor reads OPEN (no measurable force).

**The force column is raw voltage and resistance, not newtons.** The FSR is not
yet calibrated — see `plotting/fsr_calibration.py` and the calibration runs in
`calibration_csv/`. Use the numbers for comparing runs, not as absolute force.

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `cannot open /dev/ttyACM0` | Serial Monitor still open, or wrong port — check `ls /dev/ttyACM*`; permissions: `sudo usermod -aG dialout $USER`, then re-login |
| `no READY within 10 s` | Wrong sketch flashed, or wrong baud. Should be `planting_arduino.ino` @ 115200 |
| `no TPDO position received` | Actuator unpowered, wrong CAN interface, wrong bitrate, or missing bus termination. Verify with `candump` |
| Actuator never moves | Faults latched from a previous run — the script sends STOP+CLEAR on startup; power-cycle the actuator if it persists |
| Missing `ACK:bldc` in console | Serial contention with FSR polling — lower `--fsr-hz` to 5 |
| Move stops short of the commanded distance | Expected: ±1.4 cm tolerance (`_TARGET_TOLERANCE`), same as the ROS driver. Don't read depth off the flag |

---

## Notes

- **Homing runs first by default** and is a full-stroke move at FAST speed. If
  the rig cannot take that, pass `--no-home` to start from the current position.
- **This script has not yet been validated against hardware.** Its CAN startup
  sequence mirrors the known-good `can_tests.py`, but keep a hand on the e-stop
  for the first run.
- `RM = 5000.0` is hardcoded in both `planting_arduino.ino` and
  `plotting/fsr_calibration.py`. Change the measuring resistor and you must
  update both, or the logged resistance will be wrong.
