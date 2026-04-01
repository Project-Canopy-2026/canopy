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
