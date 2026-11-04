# Seedling Planner (Foxglove panel)

Third tab of `config/foxglove/canopy_monitor_diag.json`. With it you can:

- pick seedling locations on a satellite map of Flagstaff Hill,
- paste seedling coordinates you already know,
- start or stop the robot stack (`launch/canopy_launch.launch.py`), and
- send the plan to the robot.

## Install (once per computer that runs Foxglove)

```bash
python3 config/foxglove/seedling_planner/build_foxe.py --install
```

Restart Foxglove, then import `config/foxglove/canopy_monitor_diag.json`.
To install on a laptop that doesn't have the repo, copy the `.foxe` file the script writes and drag it onto the Foxglove window.

## Run on the robot

Use this launch file instead of `canopy_launch.launch.py`:

```bash
ros2 launch ~/dev/ros2_ws/src/canopy/launch/foxglove_operator.launch.py
```

It starts `foxglove_bridge` and `scripts/canopy_launcher.py` only. The robot stack starts when you press **Start planting**.
The stack runs with `foxglove:=false`, because a second bridge would conflict with the one already running.
Stack output goes to `~/.ros/canopy_launcher/canopy_<time>.log`.
Export the same environment as for a normal run first (for example `FASTRTPS_DEFAULT_PROFILES_FILE`), because the stack inherits it.

## Use

1. Connect Foxglove to `ws://<jetson-ip>:8765` and open the third tab.
2. **Start planting**, then click again to confirm. Wait until **Stack** shows *running* and **Mode** shows a value, which means the FSM is up.
3. Add seedlings:
   - Click the map to add one.
   - Drag a marker to move it.
   - Right-click a marker, or select it and press Delete, to remove it.
   - Ctrl+Z undoes.
   - To enter known coordinates, type one `lat, lon` per line, or paste a whole `ros2 topic pub ... PlantingPlan "{...}"` command, then press **Add to list**.
   - You can also edit any row in the list directly.
4. **Send plan**, then click again to confirm. This publishes the same two messages as:

   ```bash
   ros2 topic pub --once /planning/requested_mode canopy_msgs/msg/Mode "{level: [2]}"
   ros2 topic pub --once /planning/complete_plan canopy_msgs/msg/PlantingPlan "{bounds_geojson: '', seedlings: [...]}"
   ```

   Each seedling's `species_id` is `seedling_1`, `seedling_2`, and so on, unless the pasted text gave one. The exact commands are shown under *Equivalent ros2 CLI commands*.
5. Orange rings are `/planning/remaining_plan`, the plan the robot actually has. The blue dot is the robot (`/vis/robot_fix`).
6. **Stop** sends Ctrl-C to the whole stack. If the stack hangs, it escalates to SIGTERM and then SIGKILL.

Topics, mode level, and the tile source can be changed in the panel settings.

## Notes

- Satellite tiles come from Esri World Imagery, so the Foxglove computer needs internet.
  Esri imagery stops at zoom 19, so beyond that the map shows enlarged zoom-19 tiles.
  The imagery can be a few meters off from GNSS. Compare the robot dot with where the robot really is before trusting a click.
- Points more than 2 km from Flagstaff Hill are shown in yellow and logged. This usually means latitude and longitude were swapped.
- The plan is not latched. If `plan_manager` isn't running when you press Send, the plan is lost; send it again.
- `extension.js` is the source. There is no build step: edit it, then rerun `build_foxe.py --install`.
  If you rename `displayName` in `package.json` or the panel name, update the panel id
  `Canopy Planner.Seedling Planner!...` in the layout JSON too.
