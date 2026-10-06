# Planting Stack — Field Test Guide

Everything needed to run a planting field test from the ROS 2 nodes, in order.
For *why* the machine behaves the way it does — especially the obstacle abort —
read [`planting_controller/WORKFLOW.md`](planting_controller/WORKFLOW.md) first;
this file is the operating procedure.

> **Status — read this first.** Two things are new and unproven on the robot:
>
> 1. **The obstacle abort** has never run on hardware; it was tested with the CAN
>    and Arduino layers faked out. Expect to spend the first few runs watching for
>    false aborts, and read [Obstacle abort](#obstacle-abort) first so you can
>    tell one from a real strike.
> 2. **The actuators are now driven by absolute position** — `GOTO <cm from home>`
>    into the LINAK's own position servo — instead of "run continuously" with this
>    node stopping the move. That is the scheme `planting_fsr_test.py` has run in
>    the field 12 times, so the *mechanism* is proven; its use from the ROS driver
>    is not. See [Position scale](#position-scale) for what was wrong before.
>
> First run of the day: use `drilling_distance_cm:=5.0` and watch
> `/linak_status`. Full depth once you have seen one clean cycle.

---

## 0. Before you touch a terminal

| ✔ | Check |
|---|---|
| | E-stop released |
| | 3 Amiga bike batteries in, power on |
| | Auger and chute both **retracted** — see the [homing caveat](#homing-is-cut-off-after-5-s) |
| | Nothing under the auger that you mind drilling into |
| | Someone within reach of the E-stop for the whole run |

The FSM has no soft stop once a drill is underway. The E-stop is the stop.

---

## 1. Interfaces (once per boot)

### CAN — the two LINAK actuators

The LINAKs are on **`can1`**, which is the driver default — nothing to pass at
launch.

```bash
sudo ip link set can1 down       # harmless if already down
sudo ip link set can1 up type can bitrate 125000
ip link show can1                # expect: UP, bitrate 125000
candump can1                     # optional: traffic sanity check, Ctrl+C to exit
```

### Serial — the Arduino

```bash
ls /dev/ttyACM*                  # ACM0 or ACM1 — note which
```

A udev rule for a stable `/dev/arduino` symlink already exists
(`/etc/udev/rules.d/99-arduino.rules`). If it is in place, use
`arduino_port:=/dev/arduino` and stop caring about the number.

Serial permissions, once per machine:

```bash
sudo usermod -aG dialout $USER   # then log out / back in, or: newgrp dialout
```

### Arduino firmware

`planting_arduino.ino` must be flashed — **not** `fsr.ino`. The FSM needs the
`bldc` / `stepper` / `fsr,read` command protocol; `fsr.ino` just streams CSV.

```
planting_controller/planting_controller/planting_arduino/planting_arduino.ino
```

Open in the Arduino IDE, upload, 115200 baud.

---

## 2. Build

```bash
source /opt/ros/humble/setup.bash
cd ~/dev/ros2_ws        # or wherever the workspace root is
colcon build --packages-select planting_controller --symlink-install
source install/setup.bash
```

`--symlink-install` is worth it here: a Python node edit then takes effect on
the next node restart, without a rebuild. If a change does not seem to take,
rebuild and re-source before you go looking for anything subtler.

---

## 3. Launch

Source the workspace in **every** terminal first:

```bash
source /opt/ros/humble/setup.bash && source ~/dev/ros2_ws/install/setup.bash
```

### Terminal 1 — the stack

```bash
ros2 launch planting_controller planting_bringup.launch.py
```

This starts three nodes in a chain, each gated on the previous one reporting
ready — that ordering is not cosmetic. `serial_bridge` drops any command that
arrives before the Arduino says `READY`, and the actuators ignore RPDOs until
they are `OPERATIONAL`.

```
serial_bridge   ──"Arduino is READY"──────────────▶ linak_can_node
linak_can_node  ──"both actuators initialised"───▶ planting_fsm
```

Wait for all three banners:

```
[serial_bridge]   Arduino is READY — bridge active.
[linak_can_node]  linak_can_node ready — both actuators initialised.
[planting_fsm]    planting_fsm ready — waiting for do_planting.
```

If the chain stalls, whichever banner is missing tells you which layer failed —
see [Troubleshooting](#troubleshooting).

**With non-default hardware or tuning:**

```bash
ros2 launch planting_controller planting_bringup.launch.py \
    arduino_port:=/dev/ttyACM1 \
    drilling_distance_cm:=20.0 \
    fsr_abort_voltage:=3.9
```

Every tunable is a launch argument — no rebuild to change a depth or a
threshold. Full list: `ros2 launch planting_controller planting_bringup.launch.py -s`,
or [Tuning knobs](#tuning-knobs) below.

### Terminal 2 — the trigger

The tester is deliberately *not* in the launch file: a launched node gets no
real stdin, so it has to be its own terminal.

```bash
ros2 run planting_controller planting_fsm_test
```

| Key | Does |
|---|---|
| `p` | Publish `/behavior/do_planting` — **starts the sequence** |
| `s` | Publish `/behavior/seedling_dropped` — stands in for the arm |
| `q` | Quit |

It also echoes `planting_state`, `/linak_status`, `/arduino_status` and
`/chute_in_position`, so this one terminal is usually enough to follow a run.

### Terminal 3+ — optional monitors

```bash
ros2 topic echo /planting_state      # state transitions only — the clearest view
ros2 topic echo /linak_status        # ACK / DONE / STALL / STOPPED / ERR + travel
ros2 topic echo /arduino_cmd         # noisy: fsr,read at 10 Hz while drilling
ros2 topic echo /arduino_status      # noisier: an FSR reply per poll
```

### Running with the arm instead

With the full stack, `behavior_fsm` publishes `/behavior/do_planting` itself
when the rover reaches a seedling, and the arm answers `/chute_in_position`
with `/behavior/seedling_dropped`. Launch from the repo root instead, and skip
terminal 2:

```bash
ros2 launch /home/teamj/dev/ros2_ws/src/canopy/launch/arm_planting.launch.py   # arm + planting
ros2 launch /home/teamj/dev/ros2_ws/src/canopy/launch/nav_planting.launch.py   # nav + planting
ros2 launch /home/teamj/dev/ros2_ws/src/canopy/launch/canopy_launch.launch.py  # everything
```

All three include `planting_bringup.launch.py`, so its launch arguments are
available there too.

---

## 4. What a good run looks like

Press `p`. You should see this, in this order:

| # | State | What moves | Ends on | ≈ Time |
|---|---|---|---|---|
| 1 | `LINAK_HOME` | both actuators retract to the end stop | 5 s timer | 5 s |
| 2 | `AUGER_SPIN_UP` | BLDC starts, 75 RPM | immediate | — |
| 3 | `DRILLING_DOWN` | LINAK 1 to 30 cm, SLOW | `DONE:LINAK1` | ~28 s |
| 4 | `DRILLING_DWELL` | auger keeps spinning at depth | 5 s timer | 5 s |
| 5 | `AUGER_RETRACT` | LINAK 1 home, FAST, still spinning | `DONE:LINAK1` | ~14 s |
| 6 | `SHIFT_TO_CHUTE` | auger stops, stepper 23 cm left | `DONE:STEPPER` | ~10 s |
| 7 | `WAIT_SEEDLING` | nothing — `/chute_in_position` published | `seedling_dropped` | you / the arm |
| 8 | `CHUTE_DOWN` | LINAK 2 to 20 cm, FAST | `DONE:LINAK2` or `STALL` | ~9 s |
| 9 | `CHUTE_RETRACT` | LINAK 2 home | `DONE:LINAK2` | ~9 s |
| 10 | `SHIFT_TO_AUGER` | stepper 23 cm right | `DONE:STEPPER` | ~10 s |
| 11 | `COMPLETE` → `IDLE` | all stop, ready for the next `p` | — | — |

**≈90 s end to end**, excluding however long step 7 waits.

Two things that look wrong but are not:

- **`STALL:LINAK2` in step 8 is normal.** The chute arm is *supposed* to stop
  when the tube touches the ground; a stall there is how the move finishes.
- **`STALL:LINAK1` in step 1 (or 5) is normal.** A full retraction ends at the
  end stop, which is exactly where it was going.

A `STALL:LINAK1` during **step 3** is not normal — that is an obstacle.

---

## Obstacle abort

While drilling — `DRILLING_DOWN` and `DRILLING_DWELL`, nowhere else — two
signals abort the hole. Either is enough on its own:

- **Force.** The FSM polls the Arduino with `fsr,read` at 10 Hz. 20 consecutive
  replies at or above **3.7 V** (2 s) counts as hitting something.
- **No progress.** The actuator is being driven but its position has not changed
  for ~1 s. This catches anything too hard to register as force before the
  actuator gives up.

When either trips, the FSM enters `DIGGING_OBSTACLE` and:

1. Stops LINAK 1 — the driver answers `STOPPED:LINAK1,<cm actually drilled>`.
2. Sends it **`HOME`**. Because that is an absolute target, it does not matter
   how far down the drill got; there is no distance to compute and nothing to get
   wrong. The reported `<cm>` still goes in the log, because it is what
   distinguishes a real obstacle from the end stop.
3. Keeps the auger spinning the whole way out so it cannot seize in the hole,
   and stops it only once clear.
4. Returns to `IDLE`, **skipping the chute entirely**. No seedling is spent on an
   unfinished hole.

`behavior_fsm` treats `DIGGING_OBSTACLE` the same as `COMPLETE` — it releases
the navigation hold and the rover moves on.

### The threshold is a guess — watch it

3.7 V was picked because the twelve trials in
`unit_test/fsr/data_InitialFSRFieldTest/` peaked at **3.559 V in workable soil**,
so it clears every drill ever recorded. No rock or wood strike has ever been
logged. Two consequences for the field:

- **False abort in ordinary soil?** Raise it: `fsr_abort_voltage:=3.9`. Hard soil
  reading high is the likely failure mode, and the log will show it.
- **Drilled straight into a rock without aborting?** Force may simply not be the
  right signal — in those same trials the `pot` and `plastic` runs peaked at
  3.381 V and 3.533 V, at or *below* plain soil. Then the no-progress detector is
  what has to catch it, and the FSR threshold is a red herring.

Either way, **a deliberate strike into something known-hard, with the run logged,
is the single most valuable thing to get out of this field test.** Use the
standalone script for that — see [Standalone drill + FSR log](#standalone-drill--fsr-log),
which writes a CSV you can plot afterwards.

---

## Tuning knobs

All are launch arguments on `planting_bringup.launch.py`:

```bash
ros2 launch planting_controller planting_bringup.launch.py drilling_distance_cm:=12.0
```

| Argument | Default | Notes |
|---|---|---|
| `arduino_port` | `/dev/ttyACM0` | or `/dev/arduino` with the udev rule |
| `arduino_baud` | `115200` | must match the sketch |
| `can_channel` | `can1` | where the LINAKs are; leave it alone |
| `can_bitrate` | `125000` | |
| `drilling_distance_cm` | `30.0` | drill depth from home; clamped to the 30 cm stroke |
| `drilling_dwell` | `5.0` | seconds spinning at the bottom |
| `chute_distance_cm` | `20.0` | |
| `shift_distance_cm` | `23.0` | measured auger ↔ chute spacing — don't guess |
| `auger_rpm` | `75` | BLDC rated speed |
| `fsr_abort_voltage` | `3.7` | **uncalibrated** |
| `fsr_abort_seconds` | `2.0` | |
| `fsr_poll_hz` | `10.0` | |

A **short, shallow first run** is the cheap way to shake out wiring and
direction errors before committing to a 30 cm hole:

```bash
ros2 launch planting_controller planting_bringup.launch.py \
    drilling_distance_cm:=5.0 drilling_dwell:=2.0
```

Anything not in that table (the 5 s home wait, LINAK speeds, stepper RPM) is
hard-coded — see [Not adjustable at launch](#not-adjustable-at-launch).

---

## Troubleshooting

### The launch chain stalls

| Missing banner | Means | Do |
|---|---|---|
| `Arduino is READY` | serial opened, no `READY` line | Check the sketch is `planting_arduino.ino`, not `fsr.ino`. Re-seat USB. `ls /dev/ttyACM*` and relaunch with the right `arduino_port:=` |
| `Cannot open /dev/ttyACM0` | wrong port or permissions | `ls /dev/ttyACM*`; `groups \| grep dialout` |
| `both actuators initialised` | CAN init failed | See below |
| `planting_fsm ready` | the gate above never fired | The trigger is matched on the banner text — if you edited those log lines, the launch file's trigger strings need the same edit |

### CAN init fails

```bash
ip link show can1                 # UP? right bitrate?
candump can1                      # any traffic at all?
```

- `SdoCommunicationError: No SDO response received` — the actuators are not
  answering. Power (bike batteries, E-stop released), termination, and wiring,
  in that order. There is a worked example of this failure in
  [`unit_test/fsr/README.md`](unit_test/fsr/README.md).
- `Cannot find 'LINAK-actuator-v3-1.eds'` — rebuild; the EDS is installed into
  `share/planting_controller/eds/`.

### The FSM reaches FAULT

Any `ERR:` on `/arduino_status` or `/linak_status` sends the FSM to `FAULT`, and
**`FAULT` is terminal** — it issues an Arduino-wide `stop` and stays there. The
node must be restarted (Ctrl+C the launch and relaunch).

> ⚠ `FAULT` stops the Arduino motors but **does not command the LINAKs**. If an
> Arduino error arrives mid-drill, the actuator keeps driving to its target.
> Hit the E-stop, or from another terminal:
>
> ```bash
> ros2 topic pub --once /linak_cmd std_msgs/msg/String "{data: 'LINAK,1,STOP'}"
> ```

### Nothing happens when I press `p`

- `do_planting received in state X — ignoring` in the FSM log: it is not in
  `IDLE`. A previous run did not finish, or it is in `FAULT`. Restart the FSM.
- No log line at all: the tester and the FSM are not seeing each other.
  `ros2 topic info /behavior/do_planting` should show one publisher and one
  subscriber.

### It hangs in WAIT_SEEDLING

Expected — it is waiting for someone to answer `/chute_in_position`. Press `s`
in the tester, or if the arm should be answering:

```bash
ros2 topic pub --once /behavior/seedling_dropped std_msgs/msg/Bool "{data: true}"
```

### The auger does not spin

`serial_bridge` drops commands until the Arduino's `READY`. If it logged
`Arduino not ready yet — dropping command`, the ordering broke — relaunch and
wait for the banners. Otherwise test the BLDC on its own:

```bash
ros2 topic pub --once /arduino_cmd std_msgs/msg/String "{data: 'bldc,in,75'}"
ros2 topic pub --once /arduino_cmd std_msgs/msg/String "{data: 'bldc,stop'}"
```

### Homing is cut off after 5 s

`LINAK_HOME` sends `HOME` to both actuators and then advances on a **flat 5 s
timer**, but a full 30 cm retraction at FAST takes ~14 s. The drill command that
follows cancels the retraction wherever it has got to.

Since moves are absolute, the only consequence is a **shallower hole**: a drill
that starts 8 cm out and is told `GOTO 30` still ends at 30 cm, it just travels
22 cm to get there. Nothing stalls and nothing is misreported. (This used to be
much worse — with relative moves, a short home made the next full-depth drill
aim past the end stop, and the stall got reported as a rock.)

In practice it is fine between consecutive plants, because a normal run leaves
both arms already home. If you want the first run of the day to drill full depth
anyway, retract both arms yourself first with the
[LINAK manual tester](#linak-manual-tester-no-arduino) (`1 home`, `2 home`), or
raise the wait at `planting_controller/planting_fsm.py:360`.

### Position scale

The actuator reports and accepts position in 0.1 mm units: **100 counts = 1 cm**.
Both the ROS driver and the standalone script use that scale, so `--drill-cm` and
`drilling_distance_cm` mean the same thing.

`linak_can_node.py` previously *derived* ≈2136.83 counts/cm, assuming the full
0…64255 range spanned the 30 cm stroke. It doesn't — 64255 is just the highest
value that isn't a command code (64256+), and a 30 cm stroke only ever reads up
to ~3000. Every commanded distance came out ~21× too large, so a drill clamped at
`POS_OUT_MAX`, never arrived, and ended on stall detection — and a stall during
`DRILLING_DOWN` is exactly what the FSM reads as an obstacle.

**If you ever see this again:** every plant ends in `DIGGING_OBSTACLE` with a
tiny travelled figure, before the auger has gone anywhere. Three constants in
`linak_can_node.py` have to stay in the same units:

| Constant | Now | Was |
|---|---|---|
| `COUNTS_PER_CM` | `100.0` | `≈2136.83`, derived |
| `_TARGET_TOLERANCE` | `50` (5 mm) | `3000` — would be the whole stroke |
| `POS_HOME` | `10` | `150` (as `POS_IN_MAX`) — 1.4 cm short of home |

### Telling a real obstacle from the end stop

Both arrive as `STALL`, and the **distance** is how you tell them apart:

- `STALL:LINAK1,<cm>` where `<cm>` is at or near the commanded depth — that is
  the end stop, not a rock.
- `STALL:LINAK1,<cm>` *well short* of the commanded depth — a real obstruction.

A full-stroke `drilling_distance_cm:=30.0` used to make the first kind common,
because a relative 30 cm command from a part-homed start aimed past the hard
stop. Absolute targets plus the driver's `[0, 30]` cm clamp mean a too-deep
command now gives a shallower hole instead, so you should not see end-stop
stalls during `DRILLING_DOWN` at all. If you do, that is worth knowing about —
it means the stroke is shorter than 30 cm and `STROKE_CM` needs correcting.

### Not adjustable at launch

Hard-coded, listed here so you know where to look if a run needs them:

| Thing | Value | Where |
|---|---|---|
| Home wait | 5.0 s | `planting_fsm.py:360` |
| No-progress abort | ~1 s (10 samples) | `linak_can_node.py` `STALL_SAMPLES` |
| Per-move timeout | 60 s | `linak_can_node.py` `TIMEOUT_S` |
| LINAK speeds | 1.09 / 2.18 cm/s | `linak_can_node.py` `SPEED_HALF` / `SPEED_FULL` |
| Stepper speed | 270 RPM | `planting_arduino.ino` `STEPPER_RPM` |
| FSR threshold resistor | 5 kΩ | `planting_arduino.ino` `FSR_RM` |

---

## Fallbacks — testing one layer at a time

### LINAK manual tester (no Arduino)

For jogging the actuators by hand: pre-retracting before a run, or checking
direction and travel.

```bash
ros2 launch planting_controller linak_test.launch.py    # driver + CLI in an xterm
```

Or, if `xterm` is not installed, two terminals:

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

**Everything is centimetres, never seconds.** `goto` is absolute from home, so
`1 goto 12` ends at 12 cm wherever it started; `down`/`up` are relative nudges.
Omitting `fast`/`slow` gets SLOW (1.09 cm/s), the driver's default. Targets are
clamped to the 30 cm stroke.

### Arduino only (no CAN)

```bash
ros2 run planting_controller serial_bridge
```

then, from another terminal:

```bash
ros2 topic echo /arduino_status &
ros2 topic pub --once /arduino_cmd std_msgs/msg/String "{data: 'bldc,in,75'}"
ros2 topic pub --once /arduino_cmd std_msgs/msg/String "{data: 'bldc,stop'}"
ros2 topic pub --once /arduino_cmd std_msgs/msg/String "{data: 'stepper,left,230'}"
ros2 topic pub --once /arduino_cmd std_msgs/msg/String "{data: 'fsr,read'}"
ros2 topic pub --once /arduino_cmd std_msgs/msg/String "{data: 'stop'}"
```

Or skip ROS entirely and use the Arduino IDE's Serial Monitor at 115200 with the
same strings.

### Standalone drill + FSR log

**No ROS at all** — one script that drives the auger half of the sequence
directly over serial + CAN and writes a timestamped CSV of every FSR sample
tagged with the state it was taken in. This is the fallback if the ROS nodes
misbehave, and it is also the right tool for a deliberate obstacle test.

```bash
cd src/planting/unit_test/fsr
sudo ip link set can1 up type can bitrate 125000

python3 planting_fsr_test.py --port /dev/ttyACM0
python3 planting_fsr_test.py --port /dev/ttyACM0 --drill-cm 12 --dwell 5 --rpm 75
python3 planting_fsr_test.py --port /dev/ttyACM0 --no-home --drill-cm 5
```

It mirrors `LINAK_HOME → AUGER_SPIN_UP → DRILLING_DOWN → DRILLING_DWELL →
AUGER_RETRACT`, **including the obstacle abort**: same 3.7 V / 2 s force trip and
same no-progress trip, and the same recovery — stop, retract to exactly where the
drill started, auger spinning until clear, finish in `ABORTED_OBSTACLE`.

| Flag | Default | |
|---|---|---|
| `--fsr-abort-v` | `3.7` | trip voltage |
| `--fsr-abort-s` | `2.0` | how long it must hold |
| `--stall-s` | `1.0` | no-progress window |
| `--stall-grace` | `1.5` | ignore stalls this soon after a move starts |
| `--move-timeout` | `60` | give up on a move that never arrives |
| `--no-abort` | off | **both detectors off** — drill regardless |

`--no-abort` is how you record what a hard strike actually looks like: the run
will not cut itself short, so the CSV captures the whole force curve. Keep a
hand on the E-stop when you use it.

Output is `planting_fsr_<timestamp>.csv` with
`elapsed_s,state,voltage_V,resistance_ohm`; an aborted run is identifiable by its
`DIGGING_OBSTACLE` and `ABORTED_OBSTACLE` rows. Plot with
`plotting/plot_fsr.py` or `plotting/plot_fsr_grid.py`.

Ctrl+C stops the auger and the actuator before exiting.

`--drill-cm` here and `drilling_distance_cm` in the ROS FSM now mean the same
centimetre — see [Position scale](#position-scale).

### Teardown

```bash
sudo ip link set can1 down
```

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
| `ERR:<reason>` | Unknown or malformed — **sends the FSM to FAULT** |

## LINAK protocol — `/linak_cmd` ↔ `/linak_status`

CANopen, 125 kbps. Node `0x20` = LINAK 1 (auger), `0x21` = LINAK 2 (chute).
30 cm stroke, positions 150 (retracted) … 64255 (extended).

| Command | Effect |
|---|---|
| `LINAK,<id>,GOTO,<cm>[,FAST\|SLOW]` | Drive to **`<cm>` from home** — what the FSM uses |
| `LINAK,<id>,HOME` | Full retraction onto the end stop (FAST; a stall here is success) |
| `LINAK,<id>,DOWN,<cm>[,FAST\|SLOW]` | Relative jog: extend **by** `<cm>` |
| `LINAK,<id>,UP,<cm>[,FAST\|SLOW]` | Relative jog: retract **by** `<cm>` |
| `LINAK,<id>,STOP` | Stop immediately |

Moves are **absolute positions in cm measured from full retraction**: 0 cm is
home, 30 cm is the end of the stroke. The setpoint goes to the actuator's own
position servo and the driver watches TPDO to confirm arrival. Every target is
clamped into `[0, 30]` cm, so no command can aim past the hard stop.

`DOWN`/`UP` remain for manual jogging. The FSM does not use them — a relative
command is only as good as your belief about where the arm is now, and that
assumption is what turned a short home into a false obstacle abort.
`IN_MAX` is accepted as a synonym for `HOME`. No speed tag means SLOW;
`FAST` = 2.18 cm/s (0xCD), `SLOW` = 1.09 cm/s (0x64).

| Reply | Meaning |
|---|---|
| `READY:LINAK` | Both actuators initialised, published once at startup |
| `ACK:LINAK<n>` | Command accepted |
| `DONE:LINAK<n>,<cm>` | Reached the commanded target |
| `STALL:LINAK<n>,<cm>` | Stopped short against something solid |
| `STOPPED:LINAK<n>,<cm>` | Aborted by an explicit `STOP` |
| `ERR:LINAK<n>:<detail>` | Fault — **sends the FSM to FAULT** |

**Every terminal reply carries the distance actually travelled**, which is what
makes the obstacle retract safe: a drill that stalled 15 cm into a commanded
30 cm is retracted 15 cm, not 30.

## Repo structure

```
src/planting/
├── planting_controller/                    ← ROS 2 Python package
│   ├── WORKFLOW.md                         ← why it behaves this way (read first)
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
│   ├── stepper/stepper.ino                 ← standalone stepper sketch
│   ├── fsr/
│   │   ├── fsr.ino                         ← FSR-only CSV streamer
│   │   ├── planting_fsr_test.py            ← no-ROS drill + FSR log + abort
│   │   ├── data_InitialFSRFieldTest/       ← the 12 trials behind the 3.7 V guess
│   │   ├── calibration_csv/  archive/
│   │   ├── plotting/                       ← plot_fsr, plot_fsr_grid, log_fsr
│   │   └── README.md
│   └── can_tests/
│       ├── linak_cmd_test_node.py          ← manual LINAK CLI
│       └── can_tests.py                    ← bare CANopen bring-up probe
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
