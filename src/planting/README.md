## Planting Motion Sequence

0. The robot is in place and receives a ROS2 topic `do_planting`. Then start the planting sequence below.
1. The BLDC spins the auger at 75 RPM, and LINAK_1 starts driving the auger+BLDC assembly 15 cm down into the soil at a fixed speed. Then stay and let the BLDC + auger spin for 10 seconds to complete the drilling.
2. Spin the BLDC the other way and start raising LINAK_1 to pull the auger back up. Once the assembly returns to its initial position, go to the next step.
3. Stepper motor drives the assembly horizontally at 100 RPM for 5 seconds (or similar).
4. LINAK_2 runs for a fixed duration at a fixed speed to drive the chute down into the ground.
5. Wait for a ROS2 topic signal `seedling_dropped`.
6. LINAK_2 retracts and moves the chute back up to original position.
7. The stepper motor moves in the other direction to return the entire assembly to its initial position.


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
│   │   └── linak_can_node.py           ← (TODO) LINAK CAN bus node
│   └── launch/
│       └── planting_bringup.launch.py
└── README.md
```


## Communication Between ROS2 and Arduino
# if there is issue with dev/ttyUSB , run this bash command
```bash 
ls /dev/ttyACM* /dev/ttyUSB*
```

The `serial_bridge` node forwards ROS2 string messages to the Arduino over serial and publishes replies back to ROS2.

Arduino serial setup:
```cpp
void setup() {
    Serial.begin(115200);   // must match Python side
}
```

### Command Protocol

Commands are newline-terminated comma-delimited strings:

| Command | Effect |
|---|---|
| `BLDC,FWD,<rpm>` | Spin BLDC forward at RPM |
| `BLDC,REV,<rpm>` | Spin BLDC reverse at RPM |
| `BLDC,STOP` | Stop BLDC |
| `STEPPER,CW,<rpm>,<ms>` | Run stepper CW for duration |
| `STEPPER,CCW,<rpm>,<ms>` | Run stepper CCW for duration |
| `STEPPER,STOP` | Stop stepper immediately |

Arduino replies:
- `ACK:<cmd>` — command received and dispatched
- `DONE:STEPPER` — timed stepper move completed
- `ERR:<reason>` — unknown or malformed command

### Data Flow

```
FSM Node                    Serial Bridge Node            Arduino UNO R3
   │                               │                            │
   │  pub /arduino_cmd             │                            │
   │  "BLDC,FWD,75"  ────────────► │  ser.write("BLDC,FWD,75\n")──────────► │
   │                               │                            │  (executes)
   │                               │  ◄── readline() ─────────  │
   │  ◄─────────────────────────── │  pub /arduino_status       │ "ACK:BLDC\n"
   │  sub /arduino_status          │  "ACK:BLDC"                │
   │  → FSM transitions state      │                            │
```

### Why a Background Thread for Reading?

The ROS2 spin loop and `serial.readline()` are both **blocking** — they'd deadlock on the same thread. The fix:

```
Main thread:   rclpy.spin()   ← handles ROS2 callbacks (incoming /arduino_cmd)
Read thread:   readline()     ← waits for bytes from Arduino
```



  Step 1 — Flash the Arduino

  Open src/planting/planting_arduino/planting_arduino.ino in the Arduino IDE and upload it to the board. It expects:
  - USB connection on /dev/ttyUSB0
  - Baud rate: 115200

  Verify the port with:
  ls /dev/ttyUSB*

  If it's a different port (e.g. /dev/ttyUSB1), update planting_bringup.launch.py accordingly before building.

  ---
  Step 2 — Source ROS2

  source /opt/ros/humble/setup.bash

  ---
  Step 3 — Build the package

  From the workspace root (/home/alina/Documents/canopy):
  colcon build --packages-select planting_controller

  ---
  Step 4 — Source the workspace overlay

  source install/setup.bash

  ---
  Step 5 — Launch

  ros2 launch planting_controller planting_bringup.launch.py

  This starts two nodes:
  - serial_bridge — opens /dev/ttyUSB0 at 115200 baud, bridges /arduino_cmd → serial and serial → /arduino_status
  - manual_fsm_tester — interactive CLI that publishes to /arduino_cmd and prints /arduino_status responses

  ---
  Step 6 — Use the CLI

  Once launched, you'll get a > prompt. Commands:

  ┌──────────────────┬──────────────────────────────────────────────────┐
  │      Input       │                      Effect                      │
  ├──────────────────┼──────────────────────────────────────────────────┤
  │ step             │ Advance FSM one state, sends default Arduino cmd │
  ├──────────────────┼──────────────────────────────────────────────────┤
  │ reset            │ Return to IDLE                                   │
  ├──────────────────┼──────────────────────────────────────────────────┤
  │ state            │ Print current FSM state                          │
  ├──────────────────┼──────────────────────────────────────────────────┤
  │ BLDC,IN,50       │ Raw command — spin auger CW at 50 RPM            │
  ├──────────────────┼──────────────────────────────────────────────────┤
  │ BLDC,STOP        │ Stop auger                                       │
  ├──────────────────┼──────────────────────────────────────────────────┤
  │ STEPPER,CW,100,3 │ Move stepper CW at 100 RPM for 3 seconds         │
  ├──────────────────┼──────────────────────────────────────────────────┤
  │ help             │ Show all commands                                │
  ├──────────────────┼──────────────────────────────────────────────────┤
  │ quit             │ Shutdown                                         │
  └──────────────────┴──────────────────────────────────────────────────┘

  Arduino replies (ACK:, DONE:STEPPER, ERR:) will print as [ARDUINO] ....                                                                                   
  