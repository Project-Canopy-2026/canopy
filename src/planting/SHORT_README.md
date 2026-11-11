# Planting — Field Quick Start

The short version of [`README.md`](README.md), for a run where nothing has
changed since the last one. Section numbers match that file.

## 0. Hardware
### Power on
1. Power on all e-bike batteries
2. Release e-stop that is zip-tied to the 8020 (check the horizontal stepper motor's LED, it should turn green)
3. Arduino and LINAK CAN USB wire should be plugged into Jetson

## 1. CAN — the two LINAK actuators (once per boot)

```bash
sudo ip link set can1 down               # harmless if already down
sudo ip link set can1 up type can bitrate 125000
ip -details link show can1               # expect: UP, bitrate 125000
```

## 2. Build

```bash
source /opt/ros/humble/setup.bash
cd ~/dev/ros2_ws        # or wherever the workspace root is
colcon build --packages-select planting_controller --symlink-install
source install/setup.bash
```

## 3. Launch

Source the workspace in **every** terminal first:

```bash
source /opt/ros/humble/setup.bash && source ~/dev/ros2_ws/install/setup.bash
```

### Terminal 1 — the stack

```bash
ros2 launch planting_controller planting_bringup.launch.py
```

**Expected Output**
```
  [linak_can_node-2] [LINAK1] configuring node 0x20 ...
  [linak_can_node-2] [LINAK1] HB watchdog: period=100 ms, timeout=300 ms
  [linak_can_node-2] [LINAK1] node 0x20 configured.
  [linak_can_node-2] [LINAK2] configuring node 0x21 ...
  [linak_can_node-2] [LINAK2] node 0x21 configured.
  [linak_can_node-2] [LINAK1] NMT → OPERATIONAL ...
  [linak_can_node-2] [LINAK1] ready.
  [linak_can_node-2] [LINAK2] NMT → OPERATIONAL ...
  [linak_can_node-2] [LINAK2] ready.
  [linak_can_node-2] linak_can_node ready — both actuators initialised.
  [INFO] [planting_fsm-3]: process started with pid [12347]
  [planting_fsm-3]   planting_fsm ready — waiting for do_planting.  ← the last line
```

### Terminal 2 — the trigger

The tester is deliberately *not* in the launch file: a launched node gets no
real stdin, so it has to be its own terminal.

```bash
ros2 run planting_controller planting_fsm_test
```
**Expected Output**
```
Planting FSM tester ready.
  p  — do_planting (start sequence)
  s  — seedling_dropped
  q  — quit
>
```