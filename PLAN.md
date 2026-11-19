# Field test plan: stop/go jitter after planting (2026-11-19)

## Problem

After the arm finishes planting and the robot resumes navigation, it stops and starts several
times before it cruises. This only happens on the departure after a planting, when the lidar
obstacle pipeline (velodyne_driver, velodyne_convert, Patchwork++) is restarted by
`obstacle_pipeline_manager`. The first leg of a mission does not show it.

What we already know:

- The relaunch uses the **same parameters** as the first launch (same `obstacle_pipeline.yaml`,
  static TF is never restarted). It is not a config mismatch.
- On Nov 16 the first fresh obstacle grid reached the planner **3.1-8.8 s** after
  "Planting finished" (median ~6.4 s), although the processes were up in < 2 s.
- None of the Nov 16 bags contain a real relaunch, so the post-planting jitter has never been
  recorded. **Tomorrow's main goal is to capture it and tell the causes apart.**
- The lidar calibration was wrong on Nov 16 (hires beam table, roll off by ~11 deg). The 11-18
  calibration (roll -12.58 deg, pitch 23.37 deg, height 1.321 m) removes the ground phantoms
  in replay. The lidar mount may shift while driving.

How the code decides the lidar is "back" today:

1. `obstacle_pipeline_manager` starts the 3 processes; it publishes no ready/failed status.
2. `occupancy_grid_node` publishes a grid for each `/nonground` cloud, stamped with its own
   `now()` (not the scan time).
3. `planner.py` holds still until **one** grid stamped after planting ended arrives
   (`awaiting_fresh_occ`), then drives. After that, it only holds if the latest grid is
   > 1.5 s old. A single clear scan also resets the obstacle-stop timer (`planner.py:957`).

---

## Assumptions (hypotheses) and possible fixes

Ordered by how likely we think they are.

### A1. The grid stream is bursty right after the relaunch (stale-data hold flapping)

- **Why we think so:** 3 new processes have to be discovered over DDS on a Jetson that just
  finished MoveIt work. The first grid already takes 3-9 s. The planner releases on one grid,
  then holds again whenever the next one is > 1.5 s late.
- **Signature in the bag:** `/diagnostics` from `trajectory_planner` shows
  "Obstacle data X s old. Holding." within ~20 s after "Fresh obstacle data ... resuming".
  Gaps > 1.5 s between `/cost/occupancy` messages in that window.
- **Fixes:**
  - **F1 Steady-stream gate (planner):** clear `awaiting_fresh_occ` only after N consecutive
    grids (e.g. 5), each < 0.5 s after the previous one.
  - **F4 Keep the lidar running during planting:** removes the relaunch entirely. Cost: more
    CPU load during arm motion (the reason the manager exists). Used tomorrow as a diagnostic.
  - **F5 Pause instead of kill:** keep the processes (and their DDS discovery) alive and
    stop only the work, e.g. stop only `velodyne_driver`, or make Patchwork++ skip clouds
    while `/behavior/is_planting` is true.

### A2. People near the robot after planting make the stop box flicker

- **Why we think so:** the crew stands near the robot during planting. The stop box is
  1.0-3.0 m ahead, |y| < 1.0 m. One clear scan sends the robot moving, the next blocked scan
  stops it, so it never waits long enough to plan a detour. Nov 16 logs show 0.1-0.17 m hops
  on post-planting departures.
- **Signature:** obstacle-stop status in `/diagnostics`, occupied cells in the stop box on
  `/cost/occupancy` that move (person), video shows people in front.
- **Fixes:**
  - **F2 Clear-time hysteresis (planner):** after an obstacle stop (and after planting),
    resume only when the stop box has been clear for ~1 s.
  - **Procedure:** crew stays > 3 m away and behind the robot when it departs.

### A3. Ground phantoms from calibration error or mount drift

- **Why we think so:** pitch error shows up ahead of the robot: 1 deg is ~5 cm at 3 m, 2 deg
  makes flat ground at 2.5-3 m look like a 0.1 m obstacle. The mount is not rigid, and the
  11-18 parked roll (-12.58) differs from the Nov 16 field fit (-10.5) by ~2 deg.
- **Signature:** flat returns 0.1-0.35 m high at x ~2.5-2.9 m, y within the box, moving
  with the robot. Ground fit at end of day differs from the start-of-day fit by > 1 deg.
- **Fixes:**
  - **F6 Recalibrate** (two opposite headings, average), stiffen or brace the mount, add
    match marks to see movement.
  - Temporary: raise `min_obstacle_height` from 0.1 to 0.15 m.
  - Longer term: auto-calibrate the ground plane whenever the robot is stopped.

### A4. Grids are built from old scans (processing lag)

- **Why we think so:** the grid is stamped when the Python occupancy node publishes, not when
  the lidar scanned. If the node is behind after the restart, it publishes "fresh" grids
  from scans that are seconds old, and the robot reacts to obstacles where they were.
- **Signature:** delay between a `/velodyne_packets` message and the matching
  `/cost/occupancy` grid > 0.5 s; occupancy node CPU near 100 %.
- **Fixes:**
  - **F3 Scan-time stamps:** fix Patchwork++ to keep the input cloud's full stamp (it drops the
    sub-second part today) and pass it through in `occupancy_grid_node`, so the planner's
    1.5 s age check measures real sensor age.

### A5. Pose/heading inputs are stale after planting (lower likelihood)

- **Why we think so:** on Nov 10-11, `/tf` was starved after arm planting and node TF
  listeners lagged up to ~9 s. A stale bearing makes the planner hold
  ("Seedling bearing is X s old. Holding.") or zig-zag.
- **Signature:** bearing-age holds in `/diagnostics`; "GPS track disagrees with gyro
  heading; re-initialising" in the gnss_interface log; `/gnss/heading_valid` going false.
- **Fixes:** out of scope for tomorrow; if seen, follow up on TF load (see the Nov 10 notes).

### A6. Patchwork++ warm-up after restart (low likelihood)

- **Why:** its self-tuned ground thresholds reset on every restart. The Nov 13 bag with real
  restarts showed no burst of false obstacles, so we rank it low.
- **Signature:** more nonground points in the box for the first few seconds after restart
  than later on the same ground.
- **Fix:** covered by F5 (process stays alive, keeps its state).

---

## Fix variants to prepare tonight

**Not implemented yet.** Each one should be a launch argument or parameter whose default is
today's behaviour, so we can switch between variants in the field without rebuilding.

| ID | Change | Where | Switch | Effort |
|---|---|---|---|---|
| F1 | Require N steady grids before resuming after planting | `planner.py` | `fresh_grids_required` (1 = today), `fresh_grid_max_gap` 0.5 s | small |
| F2 | Stop box must be clear for T s before resuming | `planner.py` | `obstacle_clear_time` (0 = today) | small |
| F4 | Don't stop the lidar pipeline while planting | `obstacle_pipeline_manager.py` | `stop_during_planting` (true = today) | small |
| F5 | Pause instead of kill (stop the driver only, or Patchwork skips clouds) | manager / Patchwork++ | `pause_mode` | medium |
| F3 | Grids carry the scan time | Patchwork++ + `occupancy_grid_node.py` | none (bug fix) | medium; stretch goal |
| F6 | Recalibrate lidar | `obstacle_pipeline.yaml` + launch TF | n/a | field procedure |

Minimum for tomorrow: **F1, F2, F4**. F5 and F3 only if time allows.

---

## Test procedure

### Before leaving

- [ ] Build with the fix flags; confirm defaults reproduce today's behaviour.
- [ ] Check free disk space for bags (`/velodyne_packets` is the big topic).
- [ ] Print the run log sheet (template below). Phone or camera for video with a visible clock.

### On site (~2.5-3 h total)

**0. Calibration check, start of day (15 min)**

1. Park on short grass with an even slope, all 4 wheels on the same plane.
2. Run the ground calibration facing one way, turn 180 deg on the same spot, run it again:
   ```bash
   python3 src/perception/scripts/lidar_ground_calib.py \
     --nominal-pitch-deg 23 --max-tilt-deg 25 --max-range 6 --scans 20
   ```
3. Record both results. If the two rolls differ by > 1 deg, move to flatter ground.
4. If the average differs from the 11-18 values (roll -12.58, pitch 23.37, h 1.321) by
   > 1 deg or > 3 cm, update both `obstacle_pipeline.yaml` and the static TF together.
5. Mark the spot and heading; we come back here at the end of the day.

**Bag command (every run):**

```bash
ros2 bag record -s mcap \
  -o ~/dev/bags/nov19-<run>-$(date +%H%M) \
  /diagnostics /cost/occupancy /nonground /velodyne_packets \
  /behavior/is_planting /w200_0120/cmd_vel /tf_static \
  /gnss/heading_valid /planning/detour_path
```

Do not use `-a` (it collapsed `/tf` on Nov 13). Start the bag before the mission starts and
stop it after the last departure.

**Runs.** Each run = a mission with at least 3 seedlings, i.e. at least 2 post-planting
departures. Same field, same seedling layout for all runs.

| Run | Code variant | Crew position at departure | Question it answers |
|---|---|---|---|
| A | Baseline (all defaults) | > 3 m away, behind robot | Does the jitter happen with no people nearby? Which hold reasons appear (A1/A3/A4/A5)? |
| B | Baseline | One person standing ~2 m ahead-left, walks away when planting finishes | How much is people (A2)? |
| C | F4: lidar keeps running during planting | > 3 m away | Does the jitter disappear without a relaunch? If yes, A1/A4/A6 are confirmed as the cause class. |
| D | F1 (N = 5) | > 3 m away | Does a steady-stream gate remove the stop/go? |
| E | F1 + F2 (T = 1.0 s) | Same as run B | Does the combined fix handle people too? |
| F *(optional)* | F5 pause mode | > 3 m away | Can we keep the CPU saving and lose the jitter? |

Run order: A, B, C first. If time is short, look at A and C (from `/diagnostics`
and video) before choosing between D and E.

**Safety:** e-stop holder next to the robot for every run. Run B and E: the person in front
stays outside the robot's path once it moves.

**7. Calibration check, end of day (10 min)**

Repeat step 0 at the marked spot and heading. A change of > 1 deg means the mount moved
during the day (A3).

### Run log sheet

| Run | Seedling # | Clock time "planting finished" | Jitter? (Y/N, # of stops) | People within 3 m? | Notes |
|---|---|---|---|---|---|
| | | | | | |

---

## What to measure (from the bags, after the test)

For each post-planting departure:

| Metric | Source |
|---|---|
| Time from "Planting finished" to first fresh grid | launch log + `/cost/occupancy` |
| Largest gap between grids in the first 20 s | `/cost/occupancy` |
| Time from "Planting finished" to first motion and to steady cruise (> 0.4 m/s) | `/w200_0120/cmd_vel` |
| Number of stop -> go transitions in the first 10 m | `/w200_0120/cmd_vel` |
| Hold reasons and their counts | `/diagnostics` (name `trajectory_planner`) |
| Occupied cells in the stop box: moving (person) or fixed at ~2.7 m (phantom) | `/cost/occupancy` |
| Packet -> grid delay | `/velodyne_packets` vs `/cost/occupancy` receive times |

**Success criteria for a fix:** with nobody within 3 m, the robot reaches cruise within 10 s
of the first fresh grid, with no more than 1 stop in the first 10 m, on every departure.

## How to read the results

| Observation | Likely cause | Next step |
|---|---|---|
| "Obstacle data X s old" holds after resuming; run C has no jitter | A1 | Keep F1; consider F5 to also avoid the startup delay |
| Obstacle stops with moving cells; only in run B; E fixes it | A2 | Keep F2; brief the crew |
| Obstacle stops with flat cells at ~2.7 m; end-of-day calibration moved | A3 | Fix the mount; recalibrate; F6 |
| Packet -> grid delay > 0.5 s; jitter persists with F1 | A4 | Implement F3; look at occupancy node CPU |
| "Seedling bearing is X s old" holds or heading re-initialised | A5 | TF / localization follow-up |
| Run C still jitters | Not the relaunch | Look at A2/A3/A5 first |
