# Planting Stack


## Planting Motion Sequence

0. Robot is in place and receives `/behavior/do_plant` (Empty) from the planning FSM.
1. BLDC spins the auger at 75 RPM while LINAK_1 drives the auger+BLDC assembly into the soil. Hold for 10 seconds.
2. BLDC reverses and LINAK_1 retracts the assembly back to its initial position.
3. Stepper motor drives the assembly horizontally at 100 RPM for 5 seconds.
4. Wait for `seedling_dropped` topic signal.
5. LINAK_2 drives the chute down into the ground.
6. LINAK_2 retracts the chute.
7. Stepper motor returns the assembly to its initial position.


## Repo Structure

```
src/planting/
├── planting_controller/                ← ROS2 Python package
│   ├── package.xml
│   ├── setup.py
│   ├── planting_controller/
│   │   ├── __init__.py
│   │   ├── planting_arduino/
│   │   │   └── planting_arduino.ino    ← combined BLDC + stepper sketch (used in production)
│   │   ├── serial_bridge.py            ← bridges /arduino_cmd ↔ serial ↔ /arduino_status
│   │   ├── planting_fsm.py             ← FSM using timed DOWN/UP commands for LINAK_1
│   │   ├── planting_fsm_test_node.py   ← interactive tester for the planting FSM
│   │   ├── linak_can_node.py           ← CANopen node for both LINAK actuators (can0 @ 125 kbps)
│   │   └── LINAK-actuator-v3-1.eds
│   └── launch/
│       ├── planting_bringup.launch.py  ← serial bridge + FSM
│       └── linak_test.launch.py        ← linak_can_node + linak_cmd_test_node
├── unit_test/
│   ├── bldc/
│   │   └── bldc.ino                    ← standalone BLDC test sketch
│   ├── stepper/
│   │   └── stepper.ino                 ← standalone stepper test sketch
│   └── can_tests/
│       ├── linak_cmd_test_node.py      ← interactive CLI tester for linak_can_node
│       └── can_tests.py
└── README.md
```
## Full FSM with Launch File
### Terminal 1    
```bash
  source /opt/ros/humble/setup.bash
  colcon build --packages-select planting_controller --symlink-install
  source install/setup.bash
  ros2 launch planting_controller planting_bringup.launch.py
```
### Terminal 2 (after you see "Both LINAK actuators initialised")
```bash
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 run planting_controller planting_fsm_test
```
## Full FSM Test (planting_fsm)

Tests the full planting sequence end-to-end with all hardware: Arduino (BLDC + stepper via serial) and both LINAK actuators (via CAN).

### Step 1 — Flash the Arduino

Open `src/planting/planting_controller/planting_controller/planting_arduino/planting_arduino.ino` in the Arduino IDE and upload to the board (baud rate: 115200).

### Step 2 — Serial port permissions (once)

```bash
sudo usermod -aG dialout $USER
# then log out and back in, or:
newgrp dialout
```

Verify the Arduino port:
```bash
ls /dev/ttyACM*
```

### Step 3 — CAN interface (once per boot)

```bash
sudo ip link set can0 up type can bitrate 125000
ip link show can0   # verify
```

### Step 4 — Build

```bash
source /opt/ros/humble/setup.bash
cd ~/Documents/canopy
colcon build --packages-select planting_controller
source install/setup.bash
```

### Step 5 — Launch nodes (start in this order, one terminal each)

Source the workspace in every terminal first:
```bash
source /opt/ros/humble/setup.bash && source ~/Documents/canopy/install/setup.bash
```

**Terminal 1 — serial bridge** (wait for `Arduino is READY` before continuing):
```bash
ros2 run planting_controller serial_bridge
```

**Terminal 2 — LINAK CAN node** (wait for `Both LINAK actuators initialised`):
```bash
ros2 run planting_controller linak_can_node
```

**Terminal 3 — planting FSM:**
```bash
ros2 run planting_controller planting_fsm
```

**Terminal 4 — test node:**
```bash
ros2 run planting_controller planting_fsm_test
```

### Step 6 — Run the sequence

In the test node terminal:

| Input | Effect |
|---|---|
| `p` | Publish `/behavior/do_plant` — starts the planting sequence |
| `s` | Publish `seedling_dropped` — advances FSM out of WAIT_SEEDLING |
| `q` | Quit |

FSM state changes, LINAK status, and Arduino status all print automatically.

Watch for these in 4Parallel windows
- ros2 topic echo /arduino_cmd
- ros2 topic echo /linak_cmd
- ros2 topic echo planting_state
- ros2 topic echp /linak_status
### Tear down

```bash
sudo ip link set can0 down
```


---


## Manual Test — Serial Bridge + Arduino only (no LINAK)

Use this to test the Arduino (BLDC + stepper) independently before CAN is connected.

### Step 1 — Flash and set up serial port

Same as steps 1–2 above.

### Step 2 — Build and source

```bash
source /opt/ros/humble/setup.bash
cd ~/Documents/canopy
colcon build --packages-select planting_controller
source install/setup.bash
```

### Step 3 — Run

**Terminal 1 — serial bridge:**
```bash
ros2 run planting_controller serial_bridge
```

**Terminal 2 — manual FSM tester:**
```bash
ros2 run planting_controller manual_fsm_tester
```

### Step 4 — CLI commands

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


---


## Manual Test — LINAK CAN only (no Arduino)

Use this to test both LINAK actuators independently.

### Step 1 — CAN interface

```bash
sudo ip link set can0 up type can bitrate 125000
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

Starts `linak_can_node` and `linak_test` (interactive CLI in an xterm).

> **Note:** `linak_can_node` finds `LINAK-actuator-v3-1.eds` automatically — it checks the ament share directory (after `colcon build`) and falls back to the directory alongside the source file.

### Step 4 — CLI commands

| Input | Effect |
|---|---|
| `1 down <secs>` | Drive LINAK_1 (auger) down for N seconds |
| `1 up <secs>` | Drive LINAK_1 up for N seconds |
| `1 out_max` | Drive LINAK_1 to full extension (self-stops at 64255) |
| `1 in_max` | Retract LINAK_1 to full retraction (self-stops at 150) |
| `2 down <secs>` | Drive LINAK_2 (chute) down for N seconds |
| `2 up <secs>` | Drive LINAK_2 up for N seconds |
| `1 stop` / `2 stop` | Stop actuator immediately |
| `q` | Quit |

Status replies print as `← <status>`:

| Status | Meaning |
|---|---|
| `READY:LINAK` | Both actuators initialised |
| `ACK:LINAK<n>` | Command accepted |
| `DONE:LINAK<n>` | Move finished |
| `ERR:LINAK<n>:…` | Fault — check log for details |

### Tear down

```bash
sudo ip link set can0 down
```


---


## Communication: ROS2 ↔ Arduino

`serial_bridge` forwards `/arduino_cmd` string messages to the Arduino over serial (`/dev/ttyACM0` @ 115200) and publishes replies to `/arduino_status`. A background thread handles `readline()` so it doesn't block the ROS2 spin loop.

### Command Protocol

Commands are newline-terminated, comma-delimited strings:

| Command | Effect |
|---|---|
| `bldc,in,<rpm>` | Spin BLDC in drilling direction |
| `bldc,out,<rpm>` | Spin BLDC in retract direction |
| `bldc,stop` | Stop BLDC |
| `stepper,left,<rpm>,<secs>` | Run stepper left for N seconds |
| `stepper,right,<rpm>,<secs>` | Run stepper right for N seconds |

Arduino replies:
- `ACK:<cmd>` — command received
- `DONE:STEPPER` — timed move completed
- `ERR:<reason>` — unknown or malformed command
- `READY` — Arduino finished `setup()`, safe to send commands
