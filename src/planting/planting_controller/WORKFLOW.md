# How planting works

Plain-English walkthrough of what the machine does at a seedling site, including
what happens when it hits a rock. Written after the force-sensor changes of
October 2026.

## The parts involved

**The auger arm** pushes a spinning drill bit into the ground to make a hole.
It is the arm called LINAK 1.

**The chute arm** lowers a tube that a seedling drops through. It is LINAK 2.

**The slider** moves sideways so that either the auger or the chute sits over
the same spot. Both tools share one hole.

**The force sensor** (an FSR) sits under the auger and reads how hard the
machine is pushing back against the ground. It reads as a voltage: more force,
higher voltage. It is on the auger arm only. The chute has none.

Both arms report their own position back over the CAN bus, so the software
always knows where each one actually is.

Each arm is given **a destination, not a distance** — "be at 20 cm", not "move
20 cm". Measured from fully retracted, so 0 cm is home and 30 cm is as far as
the arm goes. A destination beyond that is quietly trimmed back to 30. This is
why a move that starts from the wrong place gives a shallower hole rather than
an arm straining against its own end stop.

## A normal run

The rover drives to a seedling, stops, and says "plant here." Then:

1. **Home** — both arms pull all the way back so we start from a known place.
2. **Spin up** — the auger starts turning.
3. **Drill down** — the auger arm pushes down 30 cm, slowly. This is the long
   part, around 27 seconds.
4. **Dwell** — it sits at the bottom for 5 seconds with the auger still
   turning, to clear the hole out.
5. **Pull out** — the auger arm comes back up, quickly, still spinning.
6. **Slide to chute** — the auger stops and the slider moves the chute over
   the hole.
7. **Wait for seedling** — the arm drops a seedling down the tube.
8. **Chute down, chute up** — the tube lowers to the ground, the seedling goes
   in, the tube comes back up.
9. **Slide back** — the slider returns to the auger position.
10. **Done** — back to idle, ready for the next site.

## Watching for rocks

During steps 3 and 4 — and only those two — the software asks the force sensor
for a reading ten times a second.

If the reading sits **at or above 3.7 volts for two seconds straight**, that
counts as hitting something. Two seconds is deliberate: the reading wobbles
normally as the auger chews through soil, so a brief spike is ignored. The
count resets to zero the moment a reading drops back below 3.7.

There is a second way a rock gets caught. If the auger arm simply stops making
progress — it is pushing but the position is not changing for about a second —
that also counts as hitting something, even if the force reading never got
high. Either signal is enough on its own.

Outside those two drilling steps the force sensor is ignored entirely.

## What happens when it hits a rock

The machine does **not** plant at that spot. It:

1. Stops the auger arm immediately.
2. Sends it all the way home. The arm is told a destination, not a distance, so
   it does not matter how far down it got — "go home" is right whether it
   managed 2 cm or 25.
3. Keeps the auger spinning the whole way out, so it does not seize up in the
   hole. The auger only stops once the arm is clear.
4. Reports **DIGGING_OBSTACLE** instead of the usual "complete."
5. Goes straight back to idle, skipping the chute entirely. No seedling is
   wasted on a hole that was never finished.

It still asks how far down it actually got, and writes that in the log — not
because the retract needs it, but because it is how you later tell a real rock
(stopped well short of the depth asked for) from the arm simply reaching the end
of its own travel (stopped at about the depth asked for).

The rover then drives on to the next seedling.

## How the arms report back

Each arm sends one of four messages when a move ends, and every one of them
carries the distance actually travelled:

| Message | Meaning |
|---|---|
| `DONE` | Arrived where it was sent |
| `STALL` | Stopped early — something was in the way |
| `STOPPED` | Cancelled partway by an explicit stop |
| `ERR` | Something went wrong |

The same physical event — arm pushing, nothing moving — means different things
depending on which arm it is:

- **Auger arm stalls while drilling** → there is a rock, abort.
- **Chute arm stalls** → the tube has touched the ground. That is the normal
  way it finishes, so carry on.
- **Either arm stalls on a full pull-back** → it reached its end stop, which
  is exactly where it was going. Counts as finished.

## Telling the rover it can move on

When the planting sequence reports either "complete" or "DIGGING_OBSTACLE,"
the navigation side releases its hold and the rover drives to the next
seedling.

Note a pre-existing quirk that has **not** been changed: navigation also
releases its hold on a flat 10-second timer, and a normal plant takes about 45
seconds. So the rover already un-latches early on every plant, obstacle or not.
That is a separate bug, left alone on purpose.

## Settings you can change at launch

| Setting | Default | What it does |
|---|---|---|
| `fsr_abort_voltage` | 3.7 | Force reading that counts as hitting something |
| `fsr_abort_seconds` | 2.0 | How long it must stay there before aborting |
| `fsr_poll_hz` | 10.0 | How often the sensor is read |
| `drilling_distance_cm` | 30.0 | How deep to drill |
| `drilling_dwell` | 5.0 | Seconds to sit at the bottom |
| `chute_distance_cm` | 20.0 | How far the chute lowers |
| `auger_rpm` | 75 | Auger speed |

## What is not proven yet

**The 3.7 V threshold is a guess.** No rock or piece of wood has ever been
drilled into with the sensor recording. The number comes from the twelve field
trials in `unit_test/fsr/data_InitialFSRFieldTest`, where normal soil peaked at
3.559 V — so 3.7 V clears everything ever recorded, and the two-second hold
covers the rest of the margin. It needs a deliberate rock test to confirm.

**Force may not even be the right signal.** In those twelve trials the runs
labelled `pot` and `plastic` peaked at 3.381 V and 3.533 V — at or *below*
plain soil. If a real obstacle turns out not to read high, the stall detection
is what will actually catch it, not the force reading.

**None of this has run on the robot.** The logic was tested off-hardware with
the CAN and Arduino layers faked out. The timing and behaviour of the rewritten
actuator code on a real bus is still unverified.

## Testing it before the field

Drive the sequence by hand with the hardware faked, and watch what comes out:

```bash
ros2 topic echo /planting_state &
ros2 topic pub --once /behavior/do_planting std_msgs/Empty {}
ros2 topic pub --once /linak_status std_msgs/String "{data: 'DONE:LINAK1,30.00'}"
# 20 of these in a row triggers the abort
ros2 topic pub --once /arduino_status std_msgs/String "{data: 'FSR:3.9,1800'}"
```

Then check on the bench that the sensor is only polled during the two drilling
steps, squeeze the sensor by hand past 3.7 V to watch a real abort, and finally
re-run a normal soil drill to confirm it does **not** false-trigger.
