# Planting Stack

Drill a hole, wait for a seedling, tamp it in. Three ROS 2 nodes
(`serial_bridge` → `linak_can_node` → `planting_fsm`) driving two LINAK
actuators over CAN, plus an auger / stepper / force sensor over an Arduino.

Follow sections 1–4 in order for a field test. Everything after that is
reference and troubleshooting.

Paths assume the workspace at `~/dev/ros2_ws` and this repo at
`~/dev/ros2_ws/src/canopy`.

---

## 1. CAN bus — once per boot

The LINAKs are on **`can1`** at 125 kbps, which is the driver default. Cycle the
link, then bring it up:

```bash
sudo ip link set can1 down
sudo ip link set can1 up type can bitrate 125000 restart-ms 100
ip link show can1                # expect: UP, and not BUS-OFF
candump can1                     # optional: traffic sanity check, Ctrl+C to exit
```

`restart-ms 100` matters: without it a bus-off leaves the interface reporting UP
while nothing reaches the wire.

Nothing to do for the Arduino — `planting_arduino.ino` is already flashed, and a
udev rule gives the board the stable symlink `/dev/arduino`, so the port no
longer depends on whether it enumerated as ACM0 or ACM1.

## 2. Build

```bash
source /opt/ros/humble/setup.bash
cd ~/dev/ros2_ws
colcon build --packages-select planting_controller --symlink-install
source install/setup.bash
```

With `--symlink-install`, editing a Python node takes effect on the next launch —
no rebuild. If a change does not seem to take, rebuild and re-source before
looking for anything subtler.

## 3. Launch

Source the workspace in **every** terminal first:

```bash
source /opt/ros/humble/setup.bash && source ~/dev/ros2_ws/install/setup.bash
```

**Terminal 1 — the stack:**

```bash
ros2 launch planting_controller planting_bringup.launch.py
```

Each node is gated on the previous one reporting ready — `serial_bridge` drops
commands sent before the Arduino says `READY`, and the actuators ignore RPDOs
until they are `OPERATIONAL`. Wait for all three banners:

```
[serial_bridge]   Arduino is READY — bridge active.
[linak_can_node]  linak_can_node ready — both actuators initialised.
[planting_fsm]    planting_fsm ready — waiting for do_planting.
```

Whichever banner is missing tells you which layer failed — see
[Troubleshooting](#troubleshooting).

## 4. Trigger a plant

**Terminal 2 — the tester** (deliberately not in the launch file: a launched
node gets no real stdin, so it needs its own terminal):

```bash
ros2 run planting_controller planting_fsm_test
```

| Key | Does |
|---|---|
| `p` | Publish `/behavior/do_planting` — **starts the sequence** |
| `s` | Publish `/behavior/seedling_dropped` — stands in for the arm |
| `q` | Quit |

It echoes `/planting_state`, `/linak_status`, `/arduino_status` and
`/chute_in_position`, so this one terminal is usually enough to follow a run.

**Optional monitors, terminal 3+:**

```bash
ros2 topic echo /planting_state      # state transitions only — the clearest view
ros2 topic echo /linak_status        # ACK / DONE / STALL / STOPPED / ERR + travel
ros2 topic echo /arduino_status      # noisy: an FSR reply per poll while drilling
```

**Teardown:**

```bash
sudo ip link set can1 down
```

---

## What a good run looks like

Press `p`:

| # | State | What moves | Ends on | ≈ Time |
|---|---|---|---|---|
| 1 | `LINAK_HOME` | both actuators retract to the end stop | 5 s timer | 5 s |
| 2 | `AUGER_SPIN_UP` | BLDC starts, 75 RPM | immediate | — |
| 3 | `DRILLING_DOWN` | LINAK 1 to 30 cm, SLOW | `DONE:LINAK1` | ~28 s |
| 4 | `DRILLING_DWELL` | auger keeps spinning at depth | 6 s timer | 6 s |
| 5 | `AUGER_RETRACT` | LINAK 1 home, FAST, still spinning | `DONE:LINAK1` | ~14 s |
| 6 | `SHIFT_TO_CHUTE` | auger stops, stepper 23 cm left | `DONE:STEPPER` | ~10 s |
| 7 | `WAIT_SEEDLING` | nothing — `/chute_in_position` published | `seedling_dropped` | you / the arm |
| 8 | `TAMPING_DOWN_1` | LINAK 2 (tamper) to 12 cm, FAST | `DONE:LINAK2` or `STALL` | ~9 s |
| 9 | `TAMPING_LIFT` | LINAK 2 up half that, from where it stopped | `DONE:LINAK2` | ~5 s |
| 10 | `TAMPING_DOWN_2` | LINAK 2 to 12 cm again | `DONE:LINAK2` or `STALL` | ~5 s |
| 11 | `TAMPING_RETRACT` | LINAK 2 home | `DONE:LINAK2` | ~9 s |
| 12 | `SHIFT_TO_AUGER` | stepper 23 cm right | `DONE:STEPPER` | ~10 s |
| 13 | `COMPLETE` → `IDLE` | all stop, ready for the next `p` | — | — |

**≈110 s end to end**, excluding however long step 7 waits.

Two things that look wrong but are not:

- **`STALL:LINAK2` in step 8 or 10.** The tamper is *supposed* to stop when it
  presses on the soil; a stall there is how the move finishes.
- **`STALL:LINAK1` in step 1 or 5.** A full retraction ends on the end stop,
  which is exactly where it was going.

A `STALL:LINAK1` during **step 3** is not normal — that is an obstacle.

---

## Obstacle abort

While drilling — `DRILLING_DOWN` and `DRILLING_DWELL`, nowhere else — two
signals abort the hole. Either is enough on its own:

- **Force.** The FSM polls `fsr,read` at 10 Hz; 10 consecutive replies at or
  above **3.5 V** (1 s) counts as hitting something.
- **No progress.** The actuator is being driven but its position has not moved
  for ~1 s. This catches anything too hard to register as force.

When either trips, the FSM enters `DIGGING_OBSTACLE` and:

1. Stops LINAK 1 — the driver answers `STOPPED:LINAK1,<cm actually drilled>`.
2. Sends it `HOME`. That is an absolute target, so how far down the drill got
   does not matter; the reported `<cm>` still goes in the log, because it is
   what distinguishes a real obstacle from the end stop.
3. Keeps the auger spinning the whole way out so it cannot seize in the hole.
4. Returns to `IDLE`, **skipping the chute entirely** — no seedling is spent on
   an unfinished hole.

`behavior_fsm` treats `DIGGING_OBSTACLE` the same as `COMPLETE`: it releases the
navigation hold and the rover moves on.

**The 3.5 V threshold is uncalibrated and may be too low.** Of the twelve trials
in `unit_test/fsr/data_InitialFSRFieldTest/`, three ordinary-soil drills peaked
at 3.511, 3.529 and 3.559 V — above the current threshold. Expect false aborts
in hard soil, and raise `fsr_abort_voltage` in `planting_fsm.py` when you see
them. In those same trials the `pot` and `plastic` strikes peaked at 3.381 and
3.533 V, at or *below* plain soil, so force may not separate rock from soil at
all; the no-progress detector is what has to catch those.

**A deliberate strike into something known-hard, logged, is the most valuable
thing to get out of a field test.** Use the
[standalone script](#standalone-drill--fsr-log) for it — it writes a CSV you can
plot.

Telling a real obstacle from the end stop: both arrive as `STALL`, and the
**distance** is how you tell them apart. `STALL:LINAK1,<cm>` at or near the
commanded depth is the end stop; well short of it is a real obstruction.

---

## Tuning

**FSM tuning is set in one place only:** the `INDOOR PARAMS` / `OUTDOOR PARAMS`
block in `planting_controller/planting_fsm.py` `__init__`. Comment one block in
and the other out. The launch file does not pass these, so
`drilling_distance_cm:=12.0` on the command line does nothing.

| Parameter | Outdoor | Notes |
|---|---|---|
| `drilling_distance_cm` | `30.0` | drill depth from home; clamped to the 30 cm stroke |
| `drilling_dwell` | `6.0` | seconds spinning at the bottom |
| `tamping_distance_cm` | `12.0` | tamp depth (LINAK 2), pressed twice, lifting half of it in between |
| `shift_distance_cm` | `23.0` | measured auger ↔ chute spacing — don't guess |
| `auger_rpm` | `75` | BLDC rated speed |
| `fsr_abort_voltage` | `3.5` | **uncalibrated** — see above |
| `fsr_abort_seconds` | `1.0` | |
| `fsr_poll_hz` | `10.0` | |

Write every distance and time as a float — `23.0`, not `23` (`auger_rpm` is the
one int). A bare int declares the parameter as an integer type, which the FSM
used to read as a double and get `0.0` from: a 0 cm stepper move with no target
to stop at. `_cast_params()` now casts every value to the type the node needs at
startup and logs the cast, so a missing decimal point is a warning in the launch
output rather than a wrong command:

```
[WARN] shift_distance_cm was declared as int 23 — cast to float 23.0. Fix the literal in planting_fsm.py.
```

Only a value that cannot be cast at all (`'high'` for a voltage) stops the node,
and it stops before anything is commanded. Parameters are read once at startup,
so `ros2 param set` mid-run changes nothing.

The INDOOR block is a short, shallow run (5 cm drill, 5 cm tamp) — the cheap way
to shake out wiring and direction errors before committing to a 30 cm hole.

Hardware stays launch arguments on `planting_bringup.launch.py`:

| Argument | Default | Notes |
|---|---|---|
| `arduino_port` | `/dev/arduino` | udev symlink; pass `/dev/ttyACMx` to override |
| `arduino_baud` | `115200` | must match the sketch |
| `can_channel` | `can1` | where the LINAKs are; leave it alone |
| `can_bitrate` | `125000` | |

Hard-coded, listed so you know where to look:

| Thing | Value | Where |
|---|---|---|
| Home wait | 5.0 s | `planting_fsm.py`, the `LINAK_HOME` branch of `_on_enter` |
| No-progress abort | ~1 s (10 samples) | `linak_can_node.py` `STALL_SAMPLES` |
| Per-move timeout | 60 s | `linak_can_node.py` `TIMEOUT_S` |
| LINAK speeds | 1.09 / 2.18 cm/s | `linak_can_node.py` `SPEED_HALF` / `SPEED_FULL` |
| Stepper speed | 270 RPM | `planting_arduino.ino` `STEPPER_RPM` |
| FSR threshold resistor | 5 kΩ | `planting_arduino.ino` `FSR_RM` |

---

## Running with the arm or nav instead

With the full stack, `behavior_fsm` publishes `/behavior/do_planting` itself when
the rover reaches a seedling, and the arm answers `/chute_in_position` with
`/behavior/seedling_dropped`. Skip step 4 and launch one of these instead of
`planting_bringup`:

```bash
ros2 launch ~/dev/ros2_ws/src/canopy/launch/arm_planting.launch.py     # arm + planting
ros2 launch ~/dev/ros2_ws/src/canopy/launch/nav_planting.launch.py     # nav + planting
ros2 launch ~/dev/ros2_ws/src/canopy/launch/canopy_launch.launch.py    # everything
```

All three include `planting_bringup.launch.py`, so its launch arguments work
there too.

---

## Troubleshooting

### The launch chain stalls

| Missing banner | Means | Do |
|---|---|---|
| `Arduino is READY` | serial opened, no `READY` line | Re-seat USB. `ls -l /dev/arduino` — if the symlink is gone, `ls /dev/ttyACM*` and relaunch with `arduino_port:=/dev/ttyACMx` |
| `Cannot open /dev/arduino` | wrong port or permissions | `groups \| grep dialout`; if missing, `sudo usermod -aG dialout $USER` then log out and back in |
| `both actuators initialised` | CAN init failed | See below |
| `planting_fsm ready` | the gate above never fired | The triggers match on banner text — if you edited those log lines, the launch file's trigger strings need the same edit |

If the Arduino is ever reflashed it must be `planting_arduino.ino`, **not**
`fsr.ino` — the FSM needs the `bldc` / `stepper` / `fsr,read` protocol, and
`fsr.ino` only streams CSV.

### CAN init fails

**Run the diagnostic first.** It is read-only — nothing moves — and it separates
six faults that all surface as the same `No SDO response received`:

```bash
cd ~/dev/ros2_ws/src/canopy/src/planting/unit_test/can_tests
python3 can_bus_check.py
python3 can_bus_check.py --channel can0    # if can1 comes back silent
python3 can_bus_check.py --scan            # if an actuator may be relabelled
```

It prints every CAN interface with its driver, link state, real bitrate and
restart-ms; listens for each actuator to announce itself; reads one read-only
object per node id; and names the cause it found. Exit status is 0 only if every
actuator answered.

| Symptom | Cause | Fix |
|---|---|---|
| `node 0x21 sent nothing unprompted`, then `[LINAK2] DISABLED ... configure failed` | that actuator is absent from the bus (the silence warning alone is harmless — these LINAKs stay quiet until addressed) | Its power (bike batteries, E-stop released), its CAN connector, then its node id |
| `NOTHING was heard on 'can1'` | wrong interface, bus-off, or wrong bitrate | Re-run the diagnostic against every interface it lists. A single-channel Kvaser commonly enumerates as `can0` while everything here defaults to `can1` |
| `EMCY 0x8130` in the log | the actuator was left armed by a previous run and latched an error | Power-cycle that actuator. Current code disarms the watchdog on shutdown, so this should only bite after a `kill -9` |
| iface reports `BUS-OFF`, or bitrate ≠ 125000 | python-can **ignores** the bitrate the node passes — it comes from `ip link` | Redo [section 1](#1-can-bus--once-per-boot) |
| `SDO request`/`RPDO1` from another master | a leftover unit test or launch still owns the bus | `pgrep -af "planting_fsr_test\|can_tests\|linak\|ros2 launch"` |
| `Cannot find 'LINAK-actuator-v3-1.eds'` | not installed | Rebuild; the EDS goes to `share/planting_controller/eds/` |

An actuator that does not answer its first SDO is **absent from the bus** —
unpowered, unwired, or not at the node id being addressed. The driver retries
three times, then disables just that channel, logs `[LINAK<n>] DISABLED` plus a
`DEGRADED` summary, and stays up so the other actuator can still be driven by
hand over `/linak_cmd`. It deliberately does **not** print `both actuators
initialised` in that state, so the FSM never starts on a rig that cannot finish
the sequence.

### The FSM reaches FAULT

Any `ERR:` on `/arduino_status` or `/linak_status` sends the FSM to `FAULT`, and
**`FAULT` is terminal** — it issues an Arduino-wide `stop` and stays there.
Ctrl+C the launch and relaunch.

> ⚠ `FAULT` stops the Arduino motors but **does not command the LINAKs**. If an
> Arduino error arrives mid-drill, the actuator keeps driving to its target.
> Hit the E-stop, or from another terminal:
>
> ```bash
> ros2 topic pub --once /linak_cmd std_msgs/msg/String "{data: 'LINAK,1,STOP'}"
> ```

### Nothing happens when I press `p`

- `do_planting received in state X — ignoring`: the FSM is not in `IDLE`. A
  previous run did not finish, or it is in `FAULT`. Restart it.
- No log line at all: `ros2 topic info /behavior/do_planting` should show one
  publisher and one subscriber.

### It hangs in WAIT_SEEDLING

Expected — it is waiting for someone to answer `/chute_in_position`. Press `s`
in the tester, or:

```bash
ros2 topic pub --once /behavior/seedling_dropped std_msgs/msg/Bool "{data: true}"
```

### The auger does not spin

If the log says `Arduino not ready yet — dropping command`, the launch ordering
broke — relaunch and wait for the banners. Otherwise test the BLDC on its own:

```bash
ros2 topic pub --once /arduino_cmd std_msgs/msg/String "{data: 'bldc,in,75'}"
ros2 topic pub --once /arduino_cmd std_msgs/msg/String "{data: 'bldc,stop'}"
```

### `Unusable planting parameters — not starting`

A parameter in the `INDOOR` / `OUTDOOR` block holds something that cannot be
read as a number at all. The log names each one; fix the block and relaunch,
nothing was commanded. A merely mistyped value (`23` for `23.0`) does not stop
the node — it is cast, with a warning.

Both exist because an int `shift_distance_cm` read back as `0.0`, and a
zero-step stepper move used to pulse forever without ever reporting `DONE`. The
firmware now rejects a zero-step move too (`ERR:BAD_STEPPER_MM`) — that half
needs a reflash of `planting_arduino.ino` to take effect.

### Homing is cut off after 5 s

`LINAK_HOME` advances on a flat 5 s timer, but a full 30 cm retraction at FAST
takes ~14 s, so the drill command that follows cancels the rest of the
retraction. Because moves are absolute, the only consequence is a **shallower
hole** — a drill that starts 8 cm out and is told `GOTO 30` still ends at 30 cm.
Nothing stalls and nothing is misreported.

It is fine between consecutive plants, since a normal run leaves both arms home.
For a full-depth first run of the day, retract both arms yourself with the
[LINAK manual tester](#linak-manual-tester-no-arduino) (`1 home`, `2 home`), or
raise the 5 s timer in the `LINAK_HOME` branch of `planting_fsm.py` `_on_enter`.

---

## Fallbacks — testing one layer at a time

### LINAK manual tester (no Arduino)

For jogging the actuators by hand: pre-retracting before a run, or checking
direction and travel.

```bash
ros2 launch planting_controller linak_test.launch.py    # driver + CLI in an xterm
```

Or, without `xterm`, two terminals:

```bash
ros2 run planting_controller linak_can_node
ros2 run planting_controller linak_test
```

| Input | Effect |
|---|---|
| `1 goto 12` | LINAK 1 (auger) to **12 cm from home** |
| `1 goto 12 fast` | same, at 2.18 cm/s |
| `1 home` | LINAK 1 full retraction to the end stop |
| `2 goto 20` | LINAK 2 (chute) to 20 cm |
| `1 down 2` / `1 up 2` | relative jog, 2 cm from wherever it is |
| `1 stop` / `2 stop` | Stop immediately |
| `q` | Quit |

**Everything is centimetres, never seconds.** `goto` is absolute from home;
`down`/`up` are relative nudges. Omitting `fast`/`slow` gets SLOW (1.09 cm/s).
Targets are clamped to the 30 cm stroke.

### Arduino only (no CAN)

```bash
ros2 run planting_controller serial_bridge
```

then from another terminal:

```bash
ros2 topic echo /arduino_status &
ros2 topic pub --once /arduino_cmd std_msgs/msg/String "{data: 'bldc,in,75'}"
ros2 topic pub --once /arduino_cmd std_msgs/msg/String "{data: 'bldc,stop'}"
ros2 topic pub --once /arduino_cmd std_msgs/msg/String "{data: 'stepper,left,230'}"
ros2 topic pub --once /arduino_cmd std_msgs/msg/String "{data: 'fsr,read'}"
ros2 topic pub --once /arduino_cmd std_msgs/msg/String "{data: 'stop'}"
```

Or skip ROS entirely and send the same strings from the Arduino IDE's Serial
Monitor at 115200.

### Standalone drill + FSR log

**No ROS at all** — one script that drives the auger half of the sequence
directly over serial + CAN and writes a timestamped CSV of every FSR sample
tagged with the state it was taken in. The fallback if the ROS nodes misbehave,
and the right tool for a deliberate obstacle test.

```bash
cd ~/dev/ros2_ws/src/canopy/src/planting/unit_test/fsr
sudo ip link set can1 up type can bitrate 125000 restart-ms 100

python3 planting_fsr_test.py
python3 planting_fsr_test.py --drill-cm 12 --dwell 5 --rpm 75
python3 planting_fsr_test.py --no-home --drill-cm 5
python3 planting_fsr_test.py --no-abort          # both detectors OFF
```

It mirrors `LINAK_HOME → AUGER_SPIN_UP → DRILLING_DOWN → DRILLING_DWELL →
AUGER_RETRACT`, including the obstacle abort and the same recovery, finishing in
`ABORTED_OBSTACLE`. Its thresholds are its own flags, not the FSM's parameters:

| Flag | Default | |
|---|---|---|
| `--port` | `/dev/ttyACM0` | pass `/dev/arduino` to use the udev symlink |
| `--drill-cm` | `28.0` | same centimetre as `drilling_distance_cm` |
| `--fsr-abort-v` | `2.5` | trip voltage |
| `--fsr-abort-s` | `2.0` | how long it must hold |
| `--stall-s` | `1.0` | no-progress window |
| `--stall-grace` | `1.5` | ignore stalls this soon after a move starts |
| `--move-timeout` | `60` | give up on a move that never arrives |
| `--no-abort` | off | **both detectors off** — drill regardless |

`--no-abort` is how you record what a hard strike actually looks like: the run
will not cut itself short, so the CSV captures the whole force curve. **Keep a
hand on the E-stop.**

Output is `planting_fsr_<timestamp>.csv` with
`elapsed_s,state,voltage_V,resistance_ohm`; an aborted run is identifiable by
its `DIGGING_OBSTACLE` and `ABORTED_OBSTACLE` rows. Plot with
`plotting/plot_fsr.py` or `plotting/plot_fsr_grid.py`. Ctrl+C stops the auger
and the actuator before exiting.

---

# Reference

## Topics

| Topic | Type | From → To |
|---|---|---|
| `/behavior/do_planting` | `Empty` | `behavior_fsm` / tester → `planting_fsm` |
| `/planting_state` | `String` | `planting_fsm` → `behavior_fsm`, monitors |
| `/chute_in_position` | `Bool` | `planting_fsm` → arm (drop the seedling now) |
| `/behavior/seedling_dropped` | `Bool` | arm / tester → `planting_fsm` |
| `/arduino_cmd` | `String` | `planting_fsm` → `serial_bridge` |
| `/arduino_status` | `String` | `serial_bridge` → `planting_fsm` |
| `/linak_cmd` | `String` | `planting_fsm` → `linak_can_node` |
| `/linak_status` | `String` | `linak_can_node` → `planting_fsm` |

## Arduino protocol — `/arduino_cmd` ↔ `/arduino_status`

Newline-terminated, comma-delimited, 115200 baud.

| Command | Effect |
|---|---|
| `bldc,in,<rpm>` | Spin auger in the drilling direction (CW) |
| `bldc,out,<rpm>` | Spin auger the other way |
| `bldc,stop` | Stop the auger |
| `stepper,left,<mm>` | Step-counted move toward the chute, 270 RPM |
| `stepper,right,<mm>` | Step-counted move back to the auger |
| `stepper,stop` | Stop the stepper |
| `fsr,read` | Read the force sensor |
| `stop` | Stop everything |

Note `<mm>` — the FSM converts `shift_distance_cm` for this.

| Reply | Meaning |
|---|---|
| `READY` | Finished `setup()`, safe to send commands |
| `ACK:<cmd>` | Command received and dispatched |
| `DONE:STEPPER` | Step-counted move finished |
| `FSR:<volts>,<ohms>` | Reply to `fsr,read`; raw, uncalibrated |
| `ERR:BAD_STEPPER_MM:<mm>` | Stepper move rejected because it came to zero steps — nothing moved. **Sends the FSM to FAULT** |
| `ERR:<reason>` | Unknown or malformed — **sends the FSM to FAULT** |

## LINAK protocol — `/linak_cmd` ↔ `/linak_status`

CANopen, 125 kbps. Node `0x20` = LINAK 1 (auger), `0x21` = LINAK 2 (chute).
30 cm stroke. Position is reported and accepted in 0.1 mm units —
**100 counts = 1 cm** — so `--drill-cm` and `drilling_distance_cm` mean the same
thing.

| Command | Effect |
|---|---|
| `LINAK,<id>,GOTO,<cm>[,FAST\|SLOW]` | Drive to **`<cm>` from home** — what the FSM uses |
| `LINAK,<id>,HOME` | Full retraction onto the end stop (FAST; a stall here is success) |
| `LINAK,<id>,DOWN,<cm>[,FAST\|SLOW]` | Relative jog: extend **by** `<cm>` |
| `LINAK,<id>,UP,<cm>[,FAST\|SLOW]` | Relative jog: retract **by** `<cm>` |
| `LINAK,<id>,STOP` | Stop immediately |

Moves are **absolute positions in cm measured from full retraction**: 0 cm is
home, 30 cm is the end of the stroke. Every target is clamped into `[0, 30]` cm,
so no command can aim past the hard stop. `DOWN`/`UP` remain for manual jogging;
the FSM does not use them, because a relative command is only as good as your
belief about where the arm is now — the assumption that turned a short home into
a false obstacle abort. `IN_MAX` is a synonym for `HOME`. No speed tag means
SLOW; `FAST` = 2.18 cm/s (0xCD), `SLOW` = 1.09 cm/s (0x64).

| Reply | Meaning |
|---|---|
| `READY:LINAK` | Both actuators initialised, published once at startup. **Not published if either actuator failed to come up** |
| `ACK:LINAK<n>` | Command accepted |
| `DONE:LINAK<n>,<cm>` | Reached the commanded target |
| `STALL:LINAK<n>,<cm>` | Stopped short against something solid |
| `STOPPED:LINAK<n>,<cm>` | Aborted by an explicit `STOP` |
| `ERR:LINAK<n>:<detail>` | Fault — **sends the FSM to FAULT**. Also published when that actuator fails to initialise, or is commanded while unavailable |

**Every terminal reply carries the distance actually travelled**, which is what
makes the obstacle retract safe.

If every plant ends in `DIGGING_OBSTACLE` with a tiny travelled figure before
the auger has gone anywhere, the unit constants in `linak_can_node.py` have
drifted apart — `COUNTS_PER_CM` (`100.0`), `_TARGET_TOLERANCE` (`50`, i.e. 5 mm)
and `POS_HOME` (`10`) all have to be read in the same units.

## Repo structure

```
src/planting/
├── planting_controller/                    ← ROS 2 Python package
│   ├── package.xml  setup.py
│   ├── launch/
│   │   ├── planting_bringup.launch.py      ← the field-test launch file
│   │   └── linak_test.launch.py            ← linak_can_node + manual CLI
│   └── planting_controller/
│       ├── planting_arduino/
│       │   └── planting_arduino.ino        ← BLDC + stepper + FSR (production)
│       ├── serial_bridge.py                ← /arduino_cmd ↔ serial ↔ /arduino_status
│       ├── planting_fsm.py                 ← the sequence + obstacle abort
│       ├── planting_fsm_test_node.py       ← interactive trigger (p / s / q)
│       ├── linak_can_node.py               ← CANopen driver, both actuators
│       └── LINAK-actuator-v3-1.eds
├── unit_test/
│   ├── bldc/bldc.ino                       ← standalone BLDC sketch
│   ├── stepper/                            ← standalone stepper sketch + test
│   ├── fsr/
│   │   ├── fsr.ino                         ← FSR-only CSV streamer
│   │   ├── planting_fsr_test.py            ← no-ROS drill + FSR log + abort
│   │   ├── data_InitialFSRFieldTest/       ← the 12 trials behind the FSR threshold
│   │   ├── plotting/                       ← plot_fsr, plot_fsr_grid, log_fsr
│   │   └── README.md                       ← worked example of a raw CAN failure
│   └── can_tests/
│       ├── can_bus_check.py                ← read-only CAN diagnostic (start here)
│       ├── can_tests.py                    ← bare CANopen bring-up probe
│       └── linak_cmd_test_node.py          ← manual LINAK CLI
├── ARCHIVE/                                ← superseded; not built
└── README.md                               ← this file
```

## Node entry points

```bash
ros2 run planting_controller serial_bridge       # Arduino bridge
ros2 run planting_controller linak_can_node      # CANopen driver
ros2 run planting_controller planting_fsm        # the sequence
ros2 run planting_controller planting_fsm_test   # interactive trigger
ros2 run planting_controller linak_test          # manual LINAK CLI
```
