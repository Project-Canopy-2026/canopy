# Planning

ROS2 planning stack for the Canopy tree-planting robot. Handles route planning, trajectory generation, costmaps, and behavior management.

## Packages

### `behavior`
High-level state machine and mission management.
- **`fsm`** — finite state machine managing robot modes (STOPPED, TELEOP, ASSISTED, AUTO)
- **`plan_manager`** — loads and tracks the planting plan, publishes remaining seedlings via `/planning/remaining_plan`

### `costmaps`
Builds and maintains cost maps used by the trajectory planner.
- **`occupancy_grid_node`** — builds an occupancy grid from perception data
- **`cost_map_node`** — aggregates costs and publishes the total cost map to `/cost/total`

### `route_planning`
High-level waypoint following.
- **`demo_waypoint_follower`** — subscribes to a goal pose on `/planning/goal_pose_geo` (geographic_msgs/GeoPoint) and drives toward it using proportional control

### `trajectory_planning`
Low-level trajectory generation and execution.
- **`planner`** — selects the best trajectory arc from pre-generated candidates based on the cost map. Subscribes to `/odometry/global` (nav_msgs/Odometry) for ego pose and velocity.

## Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odometry/global` | `nav_msgs/Odometry` | EKF-fused pose + velocity from `robot_localization` |
| `/cost/total` | `nav_msgs/OccupancyGrid` | Aggregated cost map |
| `/planning/goal_pose_geo` | `geographic_msgs/GeoPoint` | Goal waypoint in GPS coordinates |
| `/planning/closest_seedling_bl` | `geometry_msgs/PointStamped` | Closest seedling in base_link frame |
| `/planning/current_mode` | `canopy_msgs/Mode` | Current robot mode |
| `/planning/remaining_plan` | `canopy_msgs/PlantingPlan` | Remaining seedlings to plant |
| `/cmd_vel` | `geometry_msgs/Twist` | Output velocity commands |
| `/behavior/is_planting` | `std_msgs/Bool` | Whether robot is currently planting |
| `/behavior/is_turning_downhill` | `std_msgs/Bool` | Whether robot is aligning downhill |
| `/behavior/facing_downhill` | `std_msgs/Empty` | Published when downhill alignment is complete |

## Building

> **Note:** If you have conda active, its Python will shadow the system Python and break the
> `canopy_msgs` CMake build (`ModuleNotFoundError: No module named 'catkin_pkg'` or `'em'`).
> Fix: prefix `PATH=/usr/bin:$PATH` so the ROS Python is found first, and clear the stale
> build cache before the first clean build.

From the workspace root (`/home/ronald/Documents/canopy`):

```bash
source /opt/ros/humble/setup.bash

# 1. Build canopy_msgs first (CMake/C++ package — must come before the Python packages)
#    PATH prefix ensures /usr/bin/python3 is used instead of miniconda's Python
rm -rf build/canopy_msgs   # only needed if a previous failed build left a stale cache
PATH=/usr/bin:$PATH colcon build --packages-select canopy_msgs
source install/setup.bash

# 2. Build all Python planning packages
PATH=/usr/bin:$PATH colcon build \
  --packages-select behavior costmaps route_planning trajectory_planning forest_planning
source install/setup.bash
```

To rebuild a single package after changes:
```bash
source /opt/ros/humble/setup.bash
PATH=/usr/bin:$PATH colcon build --packages-select trajectory_planning
source install/setup.bash
```

## Running

### Full planning stack + visualization
```bash
source /opt/ros/humble/setup.bash
source /home/ronald/Documents/canopy/install/setup.bash

ros2 launch /home/ronald/Documents/canopy/launch/trajectory_planner_test.launch.py
```

This launches all planning nodes plus RViz2 (pre-configured via `config/canopy.rviz`) and rqt.

RViz2 displays:
- **CostMap (total)** — `/cost/total` OccupancyGrid, costmap color scheme
- **OccupancyGrid (obstacles)** — `/cost/occupancy` from LiDAR (blank without hardware)
- **PlannedPath** — `/cmd_vel/path` green trajectory arc selected by the planner
- **Odometry** — `/odometry/global` robot pose arrow
- **TF** — coordinate frames (`map`, `base_link`)

Fixed frame is set to `base_link` (all costmap topics publish with `frame_id = "base_link"`).

### Publishing test inputs (no hardware)

In a second terminal (with ROS and workspace sourced):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Static TF so cost_map_node tf lookup succeeds
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link &

# Fake odometry at 10 Hz (required by trajectory_planner and cost_map_node)
ros2 topic pub /odometry/global nav_msgs/msg/Odometry \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --rate 10 &

# Set mode to AUTO (level=2) to activate the trajectory planner
# Note: ros2 topic pub doesn't handle byte fields well — use Python instead
/usr/bin/python3 -c "import rclpy, time; from rclpy.node import Node; from canopy_msgs.msg import Mode; rclpy.init(); n = Node('mode_publisher'); p = n.create_publisher(Mode, '/planning/requested_mode', 1); time.sleep(0.5); m = Mode(); m.level = bytes([2]); p.publish(m); time.sleep(0.1); rclpy.shutdown()"

# Publish a planting plan — triggers the costmap gradient toward seedling locations
ros2 topic pub /planning/complete_plan canopy_msgs/msg/PlantingPlan \
  "{seedlings: [{latitude: 40.4432, longitude: -79.9402}, {latitude: 40.4433, longitude: -79.9401}]}" --once
```

After publishing these, the costmap gradient and green trajectory arc should appear in RViz2.
