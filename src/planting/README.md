## Planting Motion Sequence

0. Robot is in place and receives `do_planting` on a ROS2 topic.
1. BLDC spins the auger at 75 RPM while LINAK_1 drives the auger+BLDC assembly 15 cm into the soil. Hold for 10 seconds.
2. BLDC reverses and LINAK_1 retracts the assembly back to its initial position.
3. Stepper motor drives the assembly horizontally at 100 RPM for 5 seconds.
4. Wait for `seedling_dropped` topic signal.
5. LINAK_2 drives the chute down into the ground.
6. LINAK_2 retracts the chute.
7. Stepper motor returns the assembly to its initial position.


## Repo Structure

```
src/planting/
├── bldc/
│   └── bldc.ino                        ← standalone BLDC test sketch
├── stepper/
│   └── stepper.ino                     ← standalone stepper test sketch
├── planting_arduino/
│   └── planting_arduino.ino            ← combined BLDC + stepper sketch (used in production)
├── planting_controller/                ← ROS2 Python package
│   ├── package.xml
│   ├── setup.py
│   ├── planting_controller/
│   │   ├── __init__.py
│   │   ├── serial_bridge.py            ← bridges /arduino_cmd ↔ serial ↔ /arduino_status
│   │   ├── planting_fsm.py             ← (TODO) state machine node
│   │   └── manual_fsm_tester.py        ← interactive CLI for serial_bridge + FSM
│   └── launch/
│       ├── planting_bringup.launch.py  ← serial bridge + FSM
│       └── linak_test.launch.py        ← linak_can_node + linak_cmd_test_node
├── unit_test/
│   └── can_tests/
│       ├── linak_can_node.py           ← CANopen node for both LINAK actuators (can0 @ 125 kbps)
│       ├── linak_cmd_test_node.py      ← interactive CLI tester for linak_can_node
│       ├── can_tests.py
│       └── LINAK-actuator-v3-1.eds
└── README.md
```


## Communication: ROS2 ↔ Arduino

`serial_bridge` forwards `/arduino_cmd` string messages to the Arduino over serial and publishes replies to `/arduino_status`. A background thread handles `readline()` so it doesn't block the ROS2 spin loop.

### Command Protocol

Commands are newline-terminated, comma-delimited strings:

| Command | Effect |
|---|---|
| `BLDC,FWD,<rpm>` | Spin BLDC forward at RPM |
| `BLDC,REV,<rpm>` | Spin BLDC reverse at RPM |
| `BLDC,STOP` | Stop BLDC |
| `STEPPER,CW,<rpm>,<ms>` | Run stepper CW for duration |
| `STEPPER,CCW,<rpm>,<ms>` | Run stepper CCW for duration |
| `STEPPER,STOP` | Stop stepper immediately |

Arduino replies:
- `ACK:<cmd>` — command received
- `DONE:STEPPER` — timed move completed
- `ERR:<reason>` — unknown or malformed command


## Running the Manual Test for the Serial Bridge + manual_fsm tester before LINAK is connected

### Step 1 — Find and fix the serial port

Arduino Uno R3 boards typically show up as `/dev/ttyACM0`, not `/dev/ttyUSB0`. Check what's available:

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

If the port is not `/dev/ttyUSB0` (e.g. it's `/dev/ttyACM0`), update the port in `planting_bringup.launch.py` before building.

If you get a **Permission denied** error on the port, add your user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
# then log out and back in, or run:
newgrp dialout
```

### Step 2 — Flash the Arduino

Open `src/planting/planting_arduino/planting_arduino.ino` in the Arduino IDE and upload it to the board (baud rate: 115200).

### Step 3 — Build and run

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build (from workspace root)
cd ~/Documents/canopy
colcon build --packages-select planting_controller

# Source the workspace overlay
source install/setup.bash
```

Run each node in a **separate terminal** (source the overlay in each one first):

**Terminal 1 — serial bridge:**
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run planting_controller serial_bridge
```

**Terminal 2 — manual FSM tester:**
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run planting_controller manual_fsm_tester
```

The tester must run in its own terminal so the interactive `>` prompt can register commands like `step`, `help`, etc.

### Step 4 — Use the CLI

Once launched, you'll get a `>` prompt:

| Input | Effect |
|---|---|
| `step` | Advance FSM one state |
| `reset` | Return to IDLE |
| `state` | Print current FSM state |
| `BLDC,FWD,50` | Spin auger CW at 50 RPM |
| `BLDC,STOP` | Stop auger |
| `STEPPER,CW,100,3000` | Move stepper CW at 100 RPM for 3 seconds |
| `help` | Show all commands |
| `quit` | Shutdown |

Arduino replies print as `[ARDUINO] ...`.


## Running the Manual Test for linak_can_node and linak_cmd_test_node to test the CAN side

### Step 1 — Bring up the CAN interface

```bash
sudo ip link set can0 up type can bitrate 125000
```

Verify it's up:

```bash
ip link show can0
```

### Step 2 — Build and source

```bash
source /opt/ros/humble/setup.bash

cd ~/Documents/canopy
colcon build --packages-select planting_controller
source install/setup.bash
```

### Step 3 — Launch

```bash
ros2 launch planting_controller linak_test.launch.py
```

This starts two nodes:
- `linak_can_node` — connects to `can0`, initialises both LINAK actuators, and listens on `/linak_cmd`
- `linak_test` — opens in a separate `xterm` with an interactive `>` prompt that publishes to `/linak_cmd` and prints replies from `/linak_status`

> **Note:** `linak_can_node` looks for `LINAK-actuator-v3-1.eds` in its working directory. If you see an EDS-not-found error, run the launch from `src/planting/unit_test/can_tests/`, or copy the EDS file to the directory you launch from.

### Step 4 — Use the CLI (in the xterm)

| Input | Effect |
|---|---|
| `1 down <secs>` | Drive LINAK_1 (auger) down for N seconds |
| `1 up <secs>` | Drive LINAK_1 up for N seconds |
| `2 down <secs>` | Drive LINAK_2 (chute) down for N seconds |
| `2 up <secs>` | Drive LINAK_2 up for N seconds |
| `1 stop` | Stop LINAK_1 immediately |
| `2 stop` | Stop LINAK_2 immediately |
| `q` | Quit |

Status replies print automatically as `← <status>`:

| Status | Meaning |
|---|---|
| `READY:LINAK` | Both actuators initialised |
| `ACK:LINAK<n>` | Command accepted |
| `DONE:LINAK<n>` | Timed move finished |
| `ERR:LINAK<n>:…` | Fault — check log for details |

### Tear down

```bash
sudo ip link set can0 down
```
